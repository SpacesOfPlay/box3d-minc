// Box3D debug-draw adapter. Box3D calls createDebugShape once per
// distinct shape and caches the record we return, then hands it back
// each frame through b3World_Draw with the body transform.
//
// Handles sphere, capsule, hull and compound. Box3D has no box shape
// type; b3MakeBoxHull produces a b3_hullShape. Mesh and height field
// return null, which Box3D reads as "skip this shape".

import box3d;
import sokol_all;
import math;
import linear;
import renderer;
import camera;

// B3_DEFAULT_MASK_BITS is a macro, so it does not reach the module.
const u64 DEBUG_DRAW_ALL_BITS = 18446744073709551615;

// upstream's View menu drives these onto b3DebugDraw.
bool g_dbg_shapes = true;
bool g_dbg_joints = true;
bool g_dbg_joint_extras;
bool g_dbg_bounds;
bool g_dbg_contacts;
bool g_dbg_contact_normals;
bool g_dbg_contact_forces;
bool g_dbg_mass;
bool g_dbg_sleep;
bool g_dbg_islands;
bool g_dbg_graph_colors;
f32 g_dbg_axis_scale = 1.0f;

// upstream View menu inputs (sample.cpp:1659). A joint marker is about
// 0.1 units across at jointScale 1.
f32 g_dbg_joint_scale = 1.0f;
f32 g_dbg_force_scale = 1.0f;   // b3DefaultDebugDraw's default

// --- mesh cache ------------------------------------------------------
//
// Keyed on the hash Box3D computes per geometry. Refcounted:
// destroyDebugShape releases, the last release frees the buffer.

const i32 MESH_CACHE_MAX = 96;

// upstream SetMeshKind: the edge overlay colours hull edges and mesh
// edges differently, so the geometry has to remember which it is.
const i32 MESH_KIND_HULL = 0;
const i32 MESH_KIND_MESH = 1;
const i32 MESH_KIND_HEIGHTFIELD = 2;

struct CachedMesh {
    u32 hash;
    i32 refs;
    sg_buffer vbuf;
    i32 nverts;
    sg_buffer ebuf;      // one endpoint pair per edge, drawn instanced
    i32 nedges;
    f32 bound;           // bounding radius about the shape origin
    i32 kind;
    bool live;
}

CachedMesh[MESH_CACHE_MAX] g_meshes;

// Scratch for one geometry under construction, grown on demand and
// freed once the buffers are uploaded — upstream debug_shapes.c
// BufferReserveVertices/BufferFree, same 256 seed and doubling. A
// triangle mesh has no useful bound to cap at: the largest procedural
// grid in the samples is 80 x 80, or 38,400 flat vertices.
f32* g_mesh_scratch;
i32 g_scratch_verts;
i32 g_scratch_cap;

i32 g_mesh_overflow_count;   // geometries dropped for an allocation failure

// Edge list for the outline pass. Hulls emit one segment per half-edge
// twin pair; meshes emit each triangle edge and dedup.
f32* g_edge_scratch;
i32 g_edge_count;
i32 g_edge_scratch_cap;

// Grow to hold `extra` more vertices of `stride` floats each.
bool buffer_reserve_n(f32** buf, i32* cap, i32 count, i32 extra, i32 stride) {
    i32 need = count + extra;
    if need <= *cap { return true; }
    i32 newCap = *cap == 0 ? 256 : *cap;
    while newCap < need { newCap *= 2; }
    f32* grown = cast(f32*, alloc(cast(i64, newCap * stride * 4)));
    if grown == null { return false; }
    if *buf != null {
        memcpy(cast(void*, grown), cast(void*, *buf), cast(i64, count * stride * 4));
        free(cast(void*, *buf));
    }
    *buf = grown;
    *cap = newCap;
    return true;
}

// Surface vertices are 6 floats: position, normal.
bool buffer_reserve(f32** buf, i32* cap, i32 count, i32 extra) {
    return buffer_reserve_n(buf, cap, count, extra, 6);
}

void scratch_free() {
    if g_mesh_scratch != null { free(cast(void*, g_mesh_scratch)); g_mesh_scratch = null; }
    if g_edge_scratch != null { free(cast(void*, g_edge_scratch)); g_edge_scratch = null; }
    g_scratch_cap = 0;
    g_edge_scratch_cap = 0;
    g_scratch_verts = 0;
    g_edge_count = 0;
}

f32 absf(f32 v) { return v < 0.0f ? 0.0f - v : v; }
f32 vec_len(b3Vec3 v) { return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z); }

f32 hull_bound(b3HullData* hull) {
    b3AABB ab = hull.aabb;
    f32 mx = absf(ab.lowerBound.x) > absf(ab.upperBound.x) ? absf(ab.lowerBound.x) : absf(ab.upperBound.x);
    f32 my = absf(ab.lowerBound.y) > absf(ab.upperBound.y) ? absf(ab.lowerBound.y) : absf(ab.upperBound.y);
    f32 mz = absf(ab.lowerBound.z) > absf(ab.upperBound.z) ? absf(ab.lowerBound.z) : absf(ab.upperBound.z);
    return sqrtf(mx * mx + my * my + mz * mz);
}

f32 g_scratch_bound;

void scratch_reset() { g_scratch_verts = 0; g_edge_count = 0; g_scratch_bound = 0.0f; }

// One record per edge, drawn instanced: the six quad corners come from a
// shared per-vertex buffer, so an edge costs its two endpoints and
// nothing else. Upstream's storage buffer holds two vec4 per edge, 32
// bytes; this is 24.
const i32 EDGE_FLOATS_PER_EDGE = 6;   // A.xyz, B.xyz

bool edge_scratch_line(b3Vec3 a, b3Vec3 b) {
    if !buffer_reserve_n(&g_edge_scratch, &g_edge_scratch_cap,
                         g_edge_count, 1, EDGE_FLOATS_PER_EDGE) {
        return false;
    }
    i32 o = g_edge_count * EDGE_FLOATS_PER_EDGE;
    g_edge_scratch[o + 0] = a.x;
    g_edge_scratch[o + 1] = a.y;
    g_edge_scratch[o + 2] = a.z;
    g_edge_scratch[o + 3] = b.x;
    g_edge_scratch[o + 4] = b.y;
    g_edge_scratch[o + 5] = b.z;
    g_edge_count += 1;
    return true;
}

// One vertex per triangle corner carrying the face normal, for flat
// shading. upstream debug_shapes.c EmitFlatTriangle.
bool scratch_tri(b3Vec3 a, b3Vec3 b, b3Vec3 c, b3Vec3 n) {
    if !buffer_reserve(&g_mesh_scratch, &g_scratch_cap, g_scratch_verts, 3) {
        return false;
    }
    i32 v = g_scratch_verts * 6;
    g_mesh_scratch[v + 0] = a.x; g_mesh_scratch[v + 1] = a.y; g_mesh_scratch[v + 2] = a.z;
    g_mesh_scratch[v + 3] = n.x; g_mesh_scratch[v + 4] = n.y; g_mesh_scratch[v + 5] = n.z;
    g_mesh_scratch[v + 6] = b.x; g_mesh_scratch[v + 7] = b.y; g_mesh_scratch[v + 8] = b.z;
    g_mesh_scratch[v + 9] = n.x; g_mesh_scratch[v + 10] = n.y; g_mesh_scratch[v + 11] = n.z;
    g_mesh_scratch[v + 12] = c.x; g_mesh_scratch[v + 13] = c.y; g_mesh_scratch[v + 14] = c.z;
    g_mesh_scratch[v + 15] = n.x; g_mesh_scratch[v + 16] = n.y; g_mesh_scratch[v + 17] = n.z;
    g_scratch_verts += 3;
    return true;
}

i32 mesh_find(u32 hash) {
    for i32 i = 0; i < MESH_CACHE_MAX; i++ {
        if g_meshes[i].live && g_meshes[i].hash == hash { return i; }
    }
    return -1;
}

i32 mesh_alloc_slot() {
    for i32 i = 0; i < MESH_CACHE_MAX; i++ {
        if !g_meshes[i].live { return i; }
    }
    return -1;
}

// Upload the scratch buffer as a new cache entry, then release it —
// upstream RegisterMesh followed by BufferFree.
i32 mesh_commit(u32 hash, i32 kind) {
    i32 slot = mesh_alloc_slot();
    if slot < 0 || g_scratch_verts == 0 { scratch_free(); return -1; }
    g_meshes[slot].vbuf = sg_make_buffer(&sg_buffer_desc{
        .data.ptr = cast(void*, g_mesh_scratch),
        .data.size = cast(i64, g_scratch_verts * 6 * 4) });
    if g_edge_count > 0 {
        g_meshes[slot].ebuf = sg_make_buffer(&sg_buffer_desc{
            .data.ptr = cast(void*, g_edge_scratch),
            .data.size = cast(i64, g_edge_count * EDGE_FLOATS_PER_EDGE * 4) });
    }
    g_meshes[slot].nedges = g_edge_count;
    g_meshes[slot].bound = g_scratch_bound;
    g_meshes[slot].hash = hash;
    g_meshes[slot].nverts = g_scratch_verts;
    g_meshes[slot].kind = kind;
    g_meshes[slot].refs = 1;
    g_meshes[slot].live = true;
    scratch_free();
    return slot;
}

void mesh_addref(i32 slot) {
    if slot >= 0 && g_meshes[slot].live { g_meshes[slot].refs++; }
}

void mesh_release(i32 slot) {
    if slot < 0 || !g_meshes[slot].live { return; }
    g_meshes[slot].refs--;
    if g_meshes[slot].refs <= 0 {
        sg_destroy_buffer(g_meshes[slot].vbuf);
        if g_meshes[slot].nedges > 0 { sg_destroy_buffer(g_meshes[slot].ebuf); }
        g_meshes[slot].live = false;
        g_meshes[slot].nverts = 0;
        g_meshes[slot].nedges = 0;
        g_meshes[slot].hash = 0;
    }
}

// Per-face fan, following upstream debug_shapes.c BuildHull. The
// outward direction comes from the face plane, not a cross product.
i32 build_hull_mesh(b3HullData* hull) {
    i32 existing = mesh_find(hull.hash);
    if existing >= 0 {
        mesh_addref(existing);
        return existing;
    }

    b3Vec3* points = b3GetHullPoints(hull);
    b3HullHalfEdge* edges = b3GetHullEdges(hull);
    b3HullFace* faces = b3GetHullFaces(hull);
    b3Plane* planes = b3GetHullPlanes(hull);
    if points == null || edges == null || faces == null || planes == null {
        return -1;
    }

    scratch_reset();
    g_scratch_bound = hull_bound(hull);
    for i32 f = 0; f < hull.faceCount; f++ {
        b3Vec3 n = planes[f].normal;
        u8 start = faces[f].edge;

        // half-edge indices are u8, so the walk is bounded
        u8[256] loop;
        i32 loopLen = 0;
        u8 e = start;
        while true {
            if loopLen >= 256 { scratch_free(); return -1; }
            loop[loopLen] = edges[e].origin;
            loopLen++;
            e = edges[e].next;
            if e == start { break; }
        }
        if loopLen < 3 { continue; }

        b3Vec3 p0 = points[loop[0]];
        for i32 i = 1; i < loopLen - 1; i++ {
            if !scratch_tri(p0, points[loop[i]], points[loop[i + 1]], n) {
                g_mesh_overflow_count++;
                scratch_free();
                return -1;
            }
        }
    }
    // one segment per unique half-edge pair
    for i32 e = 0; e < hull.edgeCount; e++ {
        i32 twin = cast(i32, edges[e].twin);
        if e >= twin { continue; }
        if !edge_scratch_line(points[edges[e].origin], points[edges[twin].origin]) {
            break;
        }
    }
    return mesh_commit(hull.hash, MESH_KIND_HULL);
}

// upstream debug_shapes.c TriangleNormal. A degenerate triangle gets
// the up axis rather than a NaN.
b3Vec3 triangle_normal(b3Vec3 a, b3Vec3 b, b3Vec3 c) {
    f32 ex = b.x - a.x; f32 ey = b.y - a.y; f32 ez = b.z - a.z;
    f32 fx = c.x - a.x; f32 fy = c.y - a.y; f32 fz = c.z - a.z;
    b3Vec3 n;
    n.x = ey * fz - ez * fy;
    n.y = ez * fx - ex * fz;
    n.z = ex * fy - ey * fx;
    f32 lenSq = n.x * n.x + n.y * n.y + n.z * n.z;
    if lenSq > 0.00000000000000000001f {
        f32 inv = 1.0f / sqrtf(lenSq);
        n.x *= inv; n.y *= inv; n.z *= inv;
    } else {
        n.x = 0.0f; n.y = 1.0f; n.z = 0.0f;
    }
    return n;
}

f32 aabb_bound(b3AABB ab) {
    f32 mx = absf(ab.lowerBound.x) > absf(ab.upperBound.x) ? absf(ab.lowerBound.x) : absf(ab.upperBound.x);
    f32 my = absf(ab.lowerBound.y) > absf(ab.upperBound.y) ? absf(ab.lowerBound.y) : absf(ab.upperBound.y);
    f32 mz = absf(ab.lowerBound.z) > absf(ab.upperBound.z) ? absf(ab.lowerBound.z) : absf(ab.upperBound.z);
    return sqrtf(mx * mx + my * my + mz * mz);
}

// Hoare partition, recursing on the smaller side and looping on the
// larger so the depth stays logarithmic. Upstream uses a QSORT macro
// for the same job.
void u64_sort(u64* a, i32 lo, i32 hi) {
    while lo < hi {
        u64 pivot = a[lo + (hi - lo) / 2];
        i32 i = lo;
        i32 j = hi;
        while i <= j {
            while a[i] < pivot { i++; }
            while a[j] > pivot { j--; }
            if i <= j {
                u64 t = a[i]; a[i] = a[j]; a[j] = t;
                i++; j--;
            }
        }
        if j - lo < hi - i {
            u64_sort(a, lo, j);
            lo = i;
        } else {
            u64_sort(a, i, hi);
            hi = j;
        }
    }
}

// Every triangle edge, canonicalised so the two triangles sharing one
// produce the same key, then sorted and uniqued — upstream's
// EmitEdge/QSORT/unique pass without the convexity flags.
void build_mesh_edges(b3MeshData* meshData, b3Vec3* verts, b3MeshTriangle* tris) {
    i32 cap = meshData.triangleCount * 3;
    u64* keys = cast(u64*, alloc(cast(i64, cap * 8)));
    if keys == null { return; }

    i32 n = 0;
    for i32 t = 0; t < meshData.triangleCount; t++ {
        b3MeshTriangle tri = tris[t];
        i32[3] a; i32[3] b;
        a[0] = tri.index1; b[0] = tri.index2;
        a[1] = tri.index2; b[1] = tri.index3;
        a[2] = tri.index3; b[2] = tri.index1;
        for i32 k = 0; k < 3; k++ {
            i32 v0 = a[k] < b[k] ? a[k] : b[k];
            i32 v1 = a[k] < b[k] ? b[k] : a[k];
            keys[n] = (cast(u64, v0) << 32) | cast(u64, v1);
            n++;
        }
    }

    if n > 1 { u64_sort(keys, 0, n - 1); }

    u64 prev = 0;
    bool havePrev = false;
    for i32 i = 0; i < n; i++ {
        if havePrev && keys[i] == prev { continue; }
        prev = keys[i];
        havePrev = true;
        i32 v0 = cast(i32, keys[i] >> 32);
        i32 v1 = cast(i32, keys[i] & cast(u64, 0xFFFFFFFF));
        if !edge_scratch_line(verts[v0], verts[v1]) { break; }
    }

    free(cast(void*, keys));
}

// The cell diagonal, plus every row and column line. A line is skipped
// when its only adjacent cell is a hole — upstream sources an edge flag
// from that same adjacent triangle, and finding none is what tells it to
// skip. Deduped like a mesh's, since neighbouring cells share lines.
void build_height_field_edges(b3HeightFieldData* hf, b3Vec3* grid, u8* materials,
                              i32 rows, i32 cols) {
    ignore hf;
    i32 cap = (rows - 1) * (cols - 1) + rows * (cols - 1) + cols * (rows - 1);
    u64* keys = cast(u64*, alloc(cast(i64, cap * 8)));
    if keys == null { return; }
    i32 n = 0;

    // diagonals, one per solid cell
    for i32 row = 0; row < rows - 1; row++ {
        for i32 col = 0; col < cols - 1; col++ {
            i32 cellIndex = row * (cols - 1) + col;
            if materials != null && materials[cellIndex] == 255 { continue; }
            i32 i10 = row * cols + col + 1;
            i32 i01 = (row + 1) * cols + col;
            i32 v0 = i10 < i01 ? i10 : i01;
            i32 v1 = i10 < i01 ? i01 : i10;
            keys[n] = (cast(u64, v0) << 32) | cast(u64, v1);
            n++;
        }
    }

    // row lines, including the outer two
    for i32 row = 0; row < rows; row++ {
        for i32 col = 0; col < cols - 1; col++ {
            bool solid = false;
            if row < rows - 1 {
                i32 cellIndex = row * (cols - 1) + col;
                if materials == null || materials[cellIndex] != 255 { solid = true; }
            }
            if !solid && row > 0 {
                i32 cellIndex = (row - 1) * (cols - 1) + col;
                if materials == null || materials[cellIndex] != 255 { solid = true; }
            }
            if !solid { continue; }
            i32 i0 = row * cols + col;
            i32 i1 = row * cols + col + 1;
            keys[n] = (cast(u64, i0) << 32) | cast(u64, i1);
            n++;
        }
    }

    // column lines, including the outer two
    for i32 col = 0; col < cols; col++ {
        for i32 row = 0; row < rows - 1; row++ {
            bool solid = false;
            if col < cols - 1 {
                i32 cellIndex = row * (cols - 1) + col;
                if materials == null || materials[cellIndex] != 255 { solid = true; }
            }
            if !solid && col > 0 {
                i32 cellIndex = row * (cols - 1) + col - 1;
                if materials == null || materials[cellIndex] != 255 { solid = true; }
            }
            if !solid { continue; }
            i32 i0 = row * cols + col;
            i32 i1 = (row + 1) * cols + col;
            keys[n] = (cast(u64, i0) << 32) | cast(u64, i1);
            n++;
        }
    }

    if n > 1 { u64_sort(keys, 0, n - 1); }

    u64 prev = 0;
    bool havePrev = false;
    for i32 i = 0; i < n; i++ {
        if havePrev && keys[i] == prev { continue; }
        prev = keys[i];
        havePrev = true;
        i32 v0 = cast(i32, keys[i] >> 32);
        i32 v1 = cast(i32, keys[i] & cast(u64, 0xFFFFFFFF));
        if !edge_scratch_line(grid[v0], grid[v1]) { break; }
    }

    free(cast(void*, keys));
}

// upstream debug_shapes.c HeightFieldSample: heights are compressed to
// u16 and decoded against the field's own minimum and scale.
b3Vec3 height_field_sample(b3HeightFieldData* hf, i32 row, i32 col) {
    i32 index = row * hf.columnCount + col;
    u16* heights = b3GetHeightFieldCompressedHeights(hf);
    f32 decoded = hf.minHeight + hf.heightScale * cast(f32, heights[index]);
    b3Vec3 scale = hf.scale;
    return b3Vec3{scale.x * cast(f32, col), scale.y * decoded, scale.z * cast(f32, row)};
}

// upstream debug_shapes.c BuildHeightField. Two triangles per cell over
// a regular grid, holes skipped, winding flipped by the field's own
// clockwise flag. Edges are the cell diagonal plus every row and column
// line, deduped like a mesh's; upstream classifies them by convexity,
// which we do not use (see build_mesh_data).
i32 build_height_field(b3HeightFieldData* hf) {
    i32 existing = mesh_find(hf.hash);
    if existing >= 0 {
        mesh_addref(existing);
        return existing;
    }
    if hf.columnCount < 2 || hf.rowCount < 2 { return -1; }

    i32 cols = hf.columnCount;
    i32 rows = hf.rowCount;
    u8* materials = b3GetHeightFieldMaterialIndices(hf);   // null if all solid
    bool clockwise = hf.clockwise;

    // The edge builder refers to grid indices, so the grid is built up
    // front even though triangles duplicate their corners for flat
    // shading.
    b3Vec3* grid = cast(b3Vec3*, alloc(cast(i64, rows * cols * cast(i32, sizeof(b3Vec3)))));
    if grid == null { return -1; }
    for i32 row = 0; row < rows; row++ {
        for i32 col = 0; col < cols; col++ {
            grid[row * cols + col] = height_field_sample(hf, row, col);
        }
    }

    scratch_reset();
    g_scratch_bound = aabb_bound(hf.aabb);

    // Triangulation matches Box3D's collision triangulation:
    //   tri0: (col, row), (col, row+1), (col+1, row)
    //   tri1: (col+1, row+1), (col+1, row), (col, row+1)
    for i32 row = 0; row < rows - 1; row++ {
        for i32 col = 0; col < cols - 1; col++ {
            i32 cellIndex = row * (cols - 1) + col;
            if materials != null && materials[cellIndex] == 255 { continue; }

            b3Vec3 p00 = grid[row * cols + col];
            b3Vec3 p10 = grid[row * cols + col + 1];
            b3Vec3 p01 = grid[(row + 1) * cols + col];
            b3Vec3 p11 = grid[(row + 1) * cols + col + 1];

            b3Vec3 a0; b3Vec3 b0; b3Vec3 c0;
            b3Vec3 a1; b3Vec3 b1; b3Vec3 c1;
            if clockwise {
                a0 = p00; b0 = p10; c0 = p01;
                a1 = p11; b1 = p01; c1 = p10;
            } else {
                a0 = p00; b0 = p01; c0 = p10;
                a1 = p11; b1 = p10; c1 = p01;
            }

            if !scratch_tri(a0, b0, c0, triangle_normal(a0, b0, c0))
               || !scratch_tri(a1, b1, c1, triangle_normal(a1, b1, c1)) {
                g_mesh_overflow_count++;
                free(cast(void*, grid));
                scratch_free();
                return -1;
            }
        }
    }

    if g_scratch_verts == 0 {
        free(cast(void*, grid));
        scratch_free();
        return -1;
    }

    build_height_field_edges(hf, grid, materials, rows, cols);

    free(cast(void*, grid));
    return mesh_commit(hf.hash, MESH_KIND_HEIGHTFIELD);
}

// Triangle meshes arrive already wound, so unlike a hull there is no
// face loop to fan — one flat-shaded triangle per index triple.
// upstream debug_shapes.c BuildMeshData.
//
// Upstream also classifies each edge by the convexity bits in
// b3GetMeshFlags, which its edge shader uses to colour convex green and
// concave red. That is behind showEdgeConvexity, false by default, and
// with it off every class collapses to the one flat colour — which is
// what we draw. The classification is not ported; the toggle would need
// it. The edges are still deduped, since a shared edge would otherwise
// be drawn twice.
i32 build_mesh_data(b3MeshData* meshData) {
    i32 existing = mesh_find(meshData.hash);
    if existing >= 0 {
        mesh_addref(existing);
        return existing;
    }

    b3Vec3* verts = b3GetMeshVertices(meshData);
    b3MeshTriangle* tris = b3GetMeshTriangles(meshData);
    if verts == null || tris == null || meshData.triangleCount <= 0 {
        return -1;
    }

    scratch_reset();
    g_scratch_bound = aabb_bound(meshData.bounds);

    for i32 t = 0; t < meshData.triangleCount; t++ {
        b3MeshTriangle tri = tris[t];
        b3Vec3 a = verts[tri.index1];
        b3Vec3 b = verts[tri.index2];
        b3Vec3 c = verts[tri.index3];
        if !scratch_tri(a, b, c, triangle_normal(a, b, c)) {
            g_mesh_overflow_count++;
            scratch_free();
            return -1;
        }
    }

    build_mesh_edges(meshData, verts, tris);
    return mesh_commit(meshData.hash, MESH_KIND_MESH);
}

// --- shape pool ------------------------------------------------------
//
// One record per live shape. The returned pointer is a slot address, so
// a free list keeps slots stable.

const i32 ASHAPE_MESH = 0;
const i32 ASHAPE_SPHERE = 1;
const i32 ASHAPE_CAPSULE = 2;
const i32 ASHAPE_COMPOUND = 3;

// A compound draws as its children, each with its own local transform.
// Village is one compound of 52500: 40000 hulls on a 200x200 grid,
// 5000 capsules and 5000 spheres over the odd cells, and a 50x50 grid
// of building meshes. Measured, not estimated.
const i32 COMPOUND_CHILD_MAX = 57344;

const i32 ADAPTER_POOL_MAX = 12288;

struct AdapterChild {
    i32 kind;
    i32 mesh;
    b3Vec3 c1;
    b3Vec3 c2;
    b3Vec3 scale;        // compound meshes scale per child
    f32 radius;
    b3Transform xf;      // child transform in the compound
}

struct AdapterShape {
    i32 kind;
    i32 mesh;            // mesh cache slot for ASHAPE_MESH
    b3Vec3 c1;           // sphere centre, or capsule end 1
    b3Vec3 c2;           // capsule end 2
    b3Vec3 scale;        // b3Mesh scales per shape while sharing geometry
    f32 radius;
    f32 bound;           // bounding radius about the body origin
    b3ShapeId shapeId;
    i32 childStart;      // ASHAPE_COMPOUND: slice of g_achildren
    i32 childCount;
    bool live;
    i32 nextFree;
}

const i32 ACHILD_MAX = 57344;
AdapterChild[ACHILD_MAX] g_achildren;
i32 g_achild_count;

// upstream gfx/draw.c GetLastCompoundDrawStats: children that survived
// the lit pass's frustum cull, out of those recorded. `drawn` is the
// LAST completed frame's — the sample's step runs before the passes.
i32 g_compound_children_drawn;
i32 g_compound_children_total;
i32 g_ccd_frame;

i32 compound_draw_stats(i32* total) {
    *total = g_compound_children_total;
    return g_compound_children_drawn;
}

AdapterShape[ADAPTER_POOL_MAX] g_ashapes;
i32 g_ashape_high;       // high-water mark of slots ever used
i32 g_ashape_free = -1;  // head of the free list, -1 when empty
i32 g_ashape_live;

void adapter_pool_init() {
    g_ashape_high = 0;
    g_ashape_free = -1;
    g_ashape_live = 0;
}

i32 ashape_alloc() {
    if g_ashape_free >= 0 {
        i32 slot = g_ashape_free;
        g_ashape_free = g_ashapes[slot].nextFree;
        g_ashape_live++;
        return slot;
    }
    if g_ashape_high >= ADAPTER_POOL_MAX { return -1; }
    i32 slot = g_ashape_high;
    g_ashape_high++;
    g_ashape_live++;
    return slot;
}

void ashape_free(i32 slot) {
    g_ashapes[slot].live = false;
    g_ashapes[slot].nextFree = g_ashape_free;
    g_ashape_free = slot;
    g_ashape_live--;
}

// Release every mesh reference and reset the pool. Called before the
// world is destroyed.
void adapter_reset() {
    for i32 i = 0; i < g_ashape_high; i++ {
        if g_ashapes[i].live && g_ashapes[i].kind == ASHAPE_MESH {
            mesh_release(g_ashapes[i].mesh);
        }
        if g_ashapes[i].live && g_ashapes[i].kind == ASHAPE_COMPOUND {
            for i32 c = 0; c < g_ashapes[i].childCount; c++ {
                AdapterChild* ch = &g_achildren[g_ashapes[i].childStart + c];
                if ch.kind == ASHAPE_MESH { mesh_release(ch.mesh); }
            }
        }
        g_ashapes[i].live = false;
    }
    adapter_pool_init();
    g_achild_count = 0;
    g_mesh_overflow_count = 0;
}

void* adapter_create_shape(b3DebugShape* ds, void* ctx) {
    ignore ctx;
    i32 kind = 0;
    i32 mesh = -1;
    b3Vec3 c1 = b3Vec3{0.0f, 0.0f, 0.0f};
    b3Vec3 c2 = b3Vec3{0.0f, 0.0f, 0.0f};
    b3Vec3 scale = b3Vec3{1.0f, 1.0f, 1.0f};
    f32 radius = 0.0f;

    f32 bound = 0.0f;

    if ds.type == b3_sphereShape {
        kind = ASHAPE_SPHERE;
        c1 = ds.sphere.center;
        radius = ds.sphere.radius;
        bound = vec_len(c1) + radius;
    } else if ds.type == b3_capsuleShape {
        kind = ASHAPE_CAPSULE;
        c1 = ds.capsule.center1;
        c2 = ds.capsule.center2;
        radius = ds.capsule.radius;
        f32 b1 = vec_len(c1);
        f32 b2 = vec_len(c2);
        bound = (b1 > b2 ? b1 : b2) + radius;
    } else if ds.type == b3_hullShape {
        mesh = build_hull_mesh(ds.hull);
        if mesh < 0 { return null; }
        kind = ASHAPE_MESH;
        // the hull AABB is in shape space; its farthest corner bounds it
        bound = hull_bound(ds.hull);
    } else if ds.type == b3_meshShape {
        mesh = build_mesh_data(ds.mesh.data);
        if mesh < 0 { return null; }
        kind = ASHAPE_MESH;
        // geometry is shared and unscaled; the shape's own scale grows
        // both the drawn size and the bound
        scale = ds.mesh.scale;
        f32 sMax = absf(scale.x);
        if absf(scale.y) > sMax { sMax = absf(scale.y); }
        if absf(scale.z) > sMax { sMax = absf(scale.z); }
        bound = aabb_bound(ds.mesh.data.bounds) * sMax;
    } else if ds.type == b3_heightShape {
        mesh = build_height_field(ds.heightField);
        if mesh < 0 { return null; }
        kind = ASHAPE_MESH;
        // the field's scale is already baked into its vertices
        bound = aabb_bound(ds.heightField.aabb);
    } else if ds.type == b3_compoundShape {
        kind = ASHAPE_COMPOUND;
    } else {
        return null;
    }

    i32 slot = ashape_alloc();
    if slot < 0 {
        if mesh >= 0 { mesh_release(mesh); }
        return null;
    }
    g_ashapes[slot].kind = kind;
    g_ashapes[slot].mesh = mesh;
    g_ashapes[slot].c1 = c1;
    g_ashapes[slot].c2 = c2;
    g_ashapes[slot].scale = scale;
    g_ashapes[slot].radius = radius;
    g_ashapes[slot].bound = bound;
    g_ashapes[slot].shapeId = ds.shapeId;
    g_ashapes[slot].childStart = 0;
    g_ashapes[slot].childCount = 0;

    if kind == ASHAPE_COMPOUND {
        b3CompoundData* cd = ds.compound;
        i32 start = g_achild_count;
        i32 n = 0;
        f32 far = 0.0f;
        for i32 i = 0; i < cd.hullCount; i++ {
            if g_achild_count >= ACHILD_MAX || n >= COMPOUND_CHILD_MAX { break; }
            b3CompoundHull h = b3GetCompoundHull(cd, i);
            i32 m = build_hull_mesh(h.hull);
            if m < 0 { continue; }
            AdapterChild* ch = &g_achildren[g_achild_count];
            ch.kind = ASHAPE_MESH;
            ch.mesh = m;
            ch.xf = h.transform;
            ch.scale = b3Vec3_one;
            f32 d = vec_len(h.transform.p) + hull_bound(h.hull);
            if d > far { far = d; }
            g_achild_count++;
            n++;
        }
        // upstream debug_adapter.c takes a mesh handle for a compound
        // child of either hull or mesh type
        for i32 i = 0; i < cd.meshCount; i++ {
            if g_achild_count >= ACHILD_MAX || n >= COMPOUND_CHILD_MAX { break; }
            b3CompoundMesh cm = b3GetCompoundMesh(cd, i);
            i32 m = build_mesh_data(cm.meshData);
            if m < 0 { continue; }
            AdapterChild* ch = &g_achildren[g_achild_count];
            ch.kind = ASHAPE_MESH;
            ch.mesh = m;
            ch.xf = cm.transform;
            ch.scale = cm.scale;
            f32 sMax = absf(cm.scale.x);
            if absf(cm.scale.y) > sMax { sMax = absf(cm.scale.y); }
            if absf(cm.scale.z) > sMax { sMax = absf(cm.scale.z); }
            f32 d = vec_len(cm.transform.p) + aabb_bound(cm.meshData.bounds) * sMax;
            if d > far { far = d; }
            g_achild_count++;
            n++;
        }
        for i32 i = 0; i < cd.sphereCount; i++ {
            if g_achild_count >= ACHILD_MAX || n >= COMPOUND_CHILD_MAX { break; }
            b3CompoundSphere sp = b3GetCompoundSphere(cd, i);
            AdapterChild* ch = &g_achildren[g_achild_count];
            ch.kind = ASHAPE_SPHERE;
            ch.mesh = -1;
            ch.c1 = sp.sphere.center;
            ch.radius = sp.sphere.radius;
            ch.xf = b3Transform_identity;
            ch.scale = b3Vec3_one;
            f32 d = vec_len(sp.sphere.center) + sp.sphere.radius;
            if d > far { far = d; }
            g_achild_count++;
            n++;
        }
        for i32 i = 0; i < cd.capsuleCount; i++ {
            if g_achild_count >= ACHILD_MAX || n >= COMPOUND_CHILD_MAX { break; }
            b3CompoundCapsule cp = b3GetCompoundCapsule(cd, i);
            AdapterChild* ch = &g_achildren[g_achild_count];
            ch.kind = ASHAPE_CAPSULE;
            ch.mesh = -1;
            ch.c1 = cp.capsule.center1;
            ch.c2 = cp.capsule.center2;
            ch.radius = cp.capsule.radius;
            ch.xf = b3Transform_identity;
            ch.scale = b3Vec3_one;
            f32 b1 = vec_len(cp.capsule.center1);
            f32 b2 = vec_len(cp.capsule.center2);
            f32 d = (b1 > b2 ? b1 : b2) + cp.capsule.radius;
            if d > far { far = d; }
            g_achild_count++;
            n++;
        }
        g_ashapes[slot].childStart = start;
        g_ashapes[slot].childCount = n;
        g_ashapes[slot].bound = far;
        if n == 0 {
            ashape_free(slot);
            return null;
        }
    }

    g_ashapes[slot].live = true;
    return cast(void*, &g_ashapes[slot]);
}

void adapter_destroy_shape(void* userShape, void* ctx) {
    ignore ctx;
    if userShape == null { return; }
    AdapterShape* s = cast(AdapterShape*, userShape);
    if !s.live { return; }
    if s.kind == ASHAPE_MESH { mesh_release(s.mesh); }
    if s.kind == ASHAPE_COMPOUND {
        for i32 c = 0; c < s.childCount; c++ {
            AdapterChild* ch = &g_achildren[s.childStart + c];
            if ch.kind == ASHAPE_MESH { mesh_release(ch.mesh); }
        }
    }
    // slot index from the pointer offset into the pool
    i64 base = cast(i64, &g_ashapes[0]);
    i64 here = cast(i64, s);
    i32 slot = cast(i32, (here - base) / cast(i64, sizeof(AdapterShape)));
    if slot >= 0 && slot < ADAPTER_POOL_MAX { ashape_free(slot); }
}

// --- per-frame draw list ---------------------------------------------
//
// b3World_Draw walks the world once; the renderer makes one depth pass
// per shadow cascade plus the lit pass. The callback records commands,
// the passes replay them.

// upstream gfx/draw.c s_drawOrigin. Set once per frame before anything
// draws. Every world position handed to the GPU is shifted against it, so
// the view matrix carries no translation and the shader never sums a
// large coordinate against a large negated eye — at 1e6 an f32 ulp is
// 0.0625, and that cancellation costs about that much in view space, on
// top of what the stored position already lost.
b3Pos g_draw_origin;

void set_draw_origin(b3Pos origin) { g_draw_origin = origin; }
b3Pos get_draw_origin() { return g_draw_origin; }

// upstream b3ToRelativeTransform( t, base ) (math_functions.h:742):
// rotation rides through unchanged, translation narrows against the base.
b3Transform to_relative_frame(b3WorldTransform t) {
    b3Transform r;
    r.q = t.q;
    r.p = b3Vec3{t.p.x - g_draw_origin.x,
                 t.p.y - g_draw_origin.y,
                 t.p.z - g_draw_origin.z};
    return r;
}

struct AdapterDraw {
    i32 kind;
    i32 mesh;
    b3Transform xf;
    b3Vec3 c1;
    b3Vec3 c2;
    b3Vec3 scale;
    f32 radius;
    f32 bound;
    i32 childStart;
    i32 childCount;
    float4 tint;
    f32 gridCell;
}

const i32 ADAPTER_DRAW_MAX = 12288;
AdapterDraw[ADAPTER_DRAW_MAX] g_adraws;
i32 g_adraw_count;

// b3MakeDebugColor packs a shading preset into the top byte:
// (rgb & 0x00FFFFFF) | material << 24 (types.h:2938). Masked off here.
float4 hex_to_rgba(b3HexColor c) {
    i32 v = cast(i32, c);
    f32 r = cast(f32, (v >> 16) & 255) / 255.0f;
    f32 g = cast(f32, (v >> 8) & 255) / 255.0f;
    f32 b = cast(f32, v & 255) / 255.0f;
    return float4{r, g, b, 1.0f};
}

// Box3D picks the colour from body state (physics_world.c:1248):
// yellow speed-capped, orange fast, lime time-of-impact, wheat sensor,
// slate gray disabled.
void adapter_draw_shape(void* userShape, b3WorldTransform xf, b3HexColor color, void* ctx) {
    ignore ctx;
    if userShape == null { return; }
    if g_adraw_count >= ADAPTER_DRAW_MAX { return; }
    AdapterShape* s = cast(AdapterShape*, userShape);
    if !s.live { return; }

    AdapterDraw* d = &g_adraws[g_adraw_count];
    d.kind = s.kind;
    d.mesh = s.mesh;
    d.xf = to_relative_frame(xf);
    d.c1 = s.c1;
    d.c2 = s.c2;
    d.scale = s.scale;
    d.radius = s.radius;
    d.bound = s.bound;
    d.childStart = s.childStart;
    d.childCount = s.childCount;
    g_compound_children_total += s.childCount;
    d.tint = hex_to_rgba(color);
    // selection brightens toward white
    if g_selected_valid && b3Shape_IsValid(s.shapeId)
       && b3Shape_GetBody(s.shapeId).index1 == g_selected_body.index1
       && b3Shape_GetBody(s.shapeId).world0 == g_selected_body.world0 {
        d.tint = float4{d.tint.x + (1.0f - d.tint.x) * 0.45f,
                        d.tint.y + (1.0f - d.tint.y) * 0.45f,
                        d.tint.z + (1.0f - d.tint.z) * 0.45f, 1.0f};
    }
    // the ground tag comes from the sample side
    d.gridCell = 0.0f;
    if g_ground_shape_valid && s.shapeId.index1 == g_ground_shape.index1
       && s.shapeId.world0 == g_ground_shape.world0 {
        d.gridCell = GROUND_GRID_CELL;
    }
    g_adraw_count++;
}

// upstream gfx/draw.c DrawSolidSphere / DrawSolidCapsule: the same
// impostors the world's own shapes draw with, at an arbitrary transform
// and colour. Pushed straight onto the draw list, so they pick up
// lighting and shadows like any shape and last exactly one frame.
void dbg_solid_sphere(b3WorldTransform xf, b3Sphere sphere, float4 color) {
    if g_adraw_count >= ADAPTER_DRAW_MAX { return; }
    AdapterDraw* d = &g_adraws[g_adraw_count];
    d.kind = ASHAPE_SPHERE;
    d.mesh = -1;
    d.xf = to_relative_frame(xf);
    d.c1 = sphere.center;
    d.c2 = b3Vec3{0.0f, 0.0f, 0.0f};
    d.scale = b3Vec3{1.0f, 1.0f, 1.0f};
    d.radius = sphere.radius;
    d.bound = vec_len(sphere.center) + sphere.radius;
    d.childStart = 0;
    d.childCount = 0;
    d.tint = color;
    d.gridCell = 0.0f;
    g_adraw_count++;
}

void dbg_solid_capsule(b3WorldTransform xf, b3Capsule capsule, float4 color) {
    if g_adraw_count >= ADAPTER_DRAW_MAX { return; }
    AdapterDraw* d = &g_adraws[g_adraw_count];
    d.kind = ASHAPE_CAPSULE;
    d.mesh = -1;
    d.xf = to_relative_frame(xf);
    d.c1 = capsule.center1;
    d.c2 = capsule.center2;
    d.scale = b3Vec3{1.0f, 1.0f, 1.0f};
    d.radius = capsule.radius;
    f32 b1 = vec_len(capsule.center1);
    f32 b2 = vec_len(capsule.center2);
    d.bound = (b1 > b2 ? b1 : b2) + capsule.radius;
    d.childStart = 0;
    d.childCount = 0;
    d.tint = color;
    d.gridCell = 0.0f;
    g_adraw_count++;
}

// upstream gfx/draw.c DrawTriangle: the three edges, in the overlay.
void dbg_triangle(b3WorldTransform xf, b3Vec3 a, b3Vec3 b, b3Vec3 c, b3HexColor color) {
    b3Pos ra = b3TransformPoint(xf, a);
    b3Pos rb = b3TransformPoint(xf, b);
    b3Pos rc = b3TransformPoint(xf, c);
    dbg_line(ra, rb, color);
    dbg_line(rb, rc, color);
    dbg_line(rc, ra, color);
}

// upstream gfx/text.c: world-space labels accumulate for the frame and
// the GUI shell drains them after the scene, projecting each to screen
// pixels and emitting through ImGui's background draw list. The
// renderer never rasterizes text itself.
const i32 LABEL_MAX = 512;
const i32 LABEL_TEXT_MAX = 64;   // upstream's 63 chars + NUL

struct Label {
    b3Pos worldPos;
    float4 color;
    u8[LABEL_TEXT_MAX] text;
}

Label[LABEL_MAX] g_labels;
i32 g_label_count;
i32 g_label_dropped;

void label_reset() {
    g_label_count = 0;
    g_label_dropped = 0;
}

// upstream DrawString3D, minus the varargs: callers snprintf first, the
// way every other formatted line in this port does.
void dbg_string_3d(b3Pos point, float4 color, u8* text) {
    if g_label_count >= LABEL_MAX { g_label_dropped++; return; }
    Label* l = &g_labels[g_label_count];
    // upstream DrawString( b3SubPos( p, GetDrawOrigin() ), ... ): stored
    // relative, because label_flush projects with the relative viewproj.
    l.worldPos = b3Pos{point.x - g_draw_origin.x,
                       point.y - g_draw_origin.y,
                       point.z - g_draw_origin.z};
    l.color = color;
    i32 i = 0;
    while i < LABEL_TEXT_MAX - 1 && text[i] != 0 {
        l.text[i] = text[i];
        i += 1;
    }
    l.text[i] = 0;
    g_label_count++;
}

// upstream ProjectWorldToScreen: clip space, then the viewport. Returns
// false behind the camera or outside the frustum, and the caller drops
// the label rather than smearing it along an edge.
bool project_world_to_screen(float4x4* viewproj, b3Pos p, f32 vw, f32 vh,
                             f32* outX, f32* outY) {
    float4 clip = mul(*viewproj, float4{p.x, p.y, p.z, 1.0f});
    if clip.w <= 0.0f { return false; }
    f32 ndcX = clip.x / clip.w;
    f32 ndcY = clip.y / clip.w;
    f32 ndcZ = clip.z / clip.w;
    if ndcX < -1.0f || ndcX > 1.0f || ndcY < -1.0f || ndcY > 1.0f { return false; }
    if ndcZ < 0.0f || ndcZ > 1.0f { return false; }
    *outX = (ndcX * 0.5f + 0.5f) * vw;
    *outY = (1.0f - (ndcY * 0.5f + 0.5f)) * vh;
    return true;
}

// Drain into ImGui's background draw list. Called after the scene, from
// inside the ImGui frame.
void label_flush(float4x4* viewproj) {
    if g_label_count == 0 { return; }
    ImDrawList* dl = ImGui_GetBackgroundDrawList(null);
    f32 vw = sapp_widthf();
    f32 vh = sapp_heightf();
    for i32 i = 0; i < g_label_count; i += 1 {
        Label* l = &g_labels[i];
        f32 sx = 0.0f;
        f32 sy = 0.0f;
        if !project_world_to_screen(viewproj, l.worldPos, vw, vh, &sx, &sy) { continue; }
        u32 col = imgui_col32(l.color);
        ImDrawList_AddText(dl, ImVec2{sx, sy}, col, cast(u8*, &l.text), null);
    }
}

// ImGui packs its colours ABGR.
u32 imgui_col32(float4 c) {
    f32 a = c.w <= 0.0f ? 1.0f : c.w;
    u32 r8 = cast(u32, cast(i32, c.x * 255.0f + 0.5f));
    u32 g8 = cast(u32, cast(i32, c.y * 255.0f + 0.5f));
    u32 b8 = cast(u32, cast(i32, c.z * 255.0f + 0.5f));
    u32 a8 = cast(u32, cast(i32, a * 255.0f + 0.5f));
    return r8 | (g8 << 8) | (b8 << 16) | (a8 << 24);
}

// upstream MakeColor / MakeColorAlpha.
float4 make_color(b3HexColor c) {
    return hex_to_rgba(c);
}

float4 make_color_alpha(b3HexColor c, f32 alpha) {
    float4 v = hex_to_rgba(c);
    return float4{v.x, v.y, v.z, alpha};
}

// --- debug channel ---------------------------------------------------
//
// Joints, contacts, bounds and transforms arrive as line work during
// b3World_Draw. Collected into one stream-updated buffer, drawn in a
// single call.

const i32 DBG_VERT_MAX = 32768;          // 16384 segments
const i32 DBG_FLOATS_PER_VERT = 7;       // position, rgba
f32[DBG_VERT_MAX * DBG_FLOATS_PER_VERT] g_dbg_verts;
i32 g_dbg_vert_count;
i32 g_dbg_dropped;                        // segments past the buffer

void dbg_reset() {
    g_dbg_vert_count = 0;
    g_dbg_dropped = 0;
    // The solid draw list clears here rather than in adapter_collect, so
    // that dbg_solid_sphere / dbg_solid_capsule from a sample's step
    // survive to the passes. Collect appends the world's shapes after.
    g_adraw_count = 0;
    g_compound_children_total = 0;
    // upstream ResetTextArena, called from ResetFrameArena.
    label_reset();
}

// The one funnel for the overlay channel — lines, points, planes, arrows,
// grids and axes all reach the vertex buffer here, so this is where the
// draw-origin shift happens for all of them.
void dbg_line_rgba(f32 ax, f32 ay, f32 az, f32 bx, f32 by, f32 bz,
                   f32 r, f32 g, f32 b, f32 a) {
    if g_dbg_vert_count + 2 > DBG_VERT_MAX { g_dbg_dropped++; return; }
    f32 ox = g_draw_origin.x; f32 oy = g_draw_origin.y; f32 oz = g_draw_origin.z;
    i32 v = g_dbg_vert_count * DBG_FLOATS_PER_VERT;
    g_dbg_verts[v..] = { ax - ox, ay - oy, az - oz, r, g, b, a,
                         bx - ox, by - oy, bz - oz, r, g, b, a };
    g_dbg_vert_count += 2;
}

void dbg_line_rgb(f32 ax, f32 ay, f32 az, f32 bx, f32 by, f32 bz,
                  f32 r, f32 g, f32 b) {
    dbg_line_rgba(ax, ay, az, bx, by, bz, r, g, b, 1.0f);
}

void dbg_line(b3Pos a, b3Pos b, b3HexColor c) {
    float4 col = hex_to_rgba(c);
    dbg_line_rgb(a.x, a.y, a.z, b.x, b.y, b.z, col.x, col.y, col.z);
}

// upstream gfx/draw.c DrawPlane: a unit quad about `point` plus a
// half-length normal and a centre point.
void dbg_plane(b3Vec3 normal, b3Pos point, b3HexColor c) {
    float4 col = hex_to_rgba(c);
    b3Vec3 perp1 = b3Perp(normal);
    b3Vec3 perp2 = b3Cross(perp1, normal);
    b3Vec3 cc = b3Vec3{point.x, point.y, point.z};
    b3Vec3 p1 = b3Add(cc, b3Add(perp1, perp2));
    b3Vec3 p2 = b3Add(cc, b3Sub(perp2, perp1));
    b3Vec3 p3 = b3Sub(cc, b3Add(perp1, perp2));
    b3Vec3 p4 = b3Add(cc, b3Sub(perp1, perp2));
    dbg_line_rgb(p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, col.x, col.y, col.z);
    dbg_line_rgb(p2.x, p2.y, p2.z, p3.x, p3.y, p3.z, col.x, col.y, col.z);
    dbg_line_rgb(p3.x, p3.y, p3.z, p4.x, p4.y, p4.z, col.x, col.y, col.z);
    dbg_line_rgb(p4.x, p4.y, p4.z, p1.x, p1.y, p1.z, col.x, col.y, col.z);
    b3Vec3 tip = b3Add(cc, b3MulSV(0.5f, normal));
    dbg_line_rgb(cc.x, cc.y, cc.z, tip.x, tip.y, tip.z, col.x, col.y, col.z);
    adapter_point(point, 10.0f, c, null);
}

// upstream gfx/draw.c DrawHull. Half-edges come in twin pairs, so draw
// each undirected edge once.
void dbg_hull(b3WorldTransform transform, b3HullData* hull, b3HexColor c) {
    float4 col = hex_to_rgba(c);
    b3Vec3* points = b3GetHullPoints(hull);
    b3HullHalfEdge* edges = b3GetHullEdges(hull);
    b3Transform rel = b3Transform{b3Vec3{transform.p.x, transform.p.y, transform.p.z},
                                  transform.q};
    for i32 i = 0; i < hull.edgeCount; i += 1 {
        i32 twin = cast(i32, edges[i].twin);
        if i >= twin { continue; }
        b3Vec3 p1 = b3TransformPoint(rel, points[edges[i].origin]);
        b3Vec3 p2 = b3TransformPoint(rel, points[edges[twin].origin]);
        dbg_line_rgb(p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, col.x, col.y, col.z);
    }
}

// upstream gfx/draw.c DrawDisc: a ring about `axis`.
void dbg_disc(b3Pos center, b3Vec3 axis, f32 radius, i32 segments, b3HexColor c) {
    float4 col = hex_to_rgba(c);
    b3Vec3 t1 = b3Perp(axis);
    b3Vec3 t2 = b3Cross(axis, t1);
    f32 prevx = center.x + radius * t1.x;
    f32 prevy = center.y + radius * t1.y;
    f32 prevz = center.z + radius * t1.z;
    for i32 i = 1; i <= segments; i++ {
        f32 a = 2.0f * PI_F * cast(f32, i) / cast(f32, segments);
        b3CosSin cs = b3ComputeCosSin(a);
        f32 x = center.x + radius * (cs.cosine * t1.x + cs.sine * t2.x);
        f32 y = center.y + radius * (cs.cosine * t1.y + cs.sine * t2.y);
        f32 z = center.z + radius * (cs.cosine * t1.z + cs.sine * t2.z);
        dbg_line_rgb(prevx, prevy, prevz, x, y, z, col.x, col.y, col.z);
        prevx = x; prevy = y; prevz = z;
    }
}

// upstream gfx/draw.c DrawWireSphere: three discs on the frame axes.
void dbg_wire_sphere(b3Transform xf, b3Sphere* sphere, i32 segments, b3HexColor c) {
    b3Vec3 wo = b3RotateVector(xf.q, sphere.center);
    b3Pos center = b3Pos{xf.p.x + wo.x, xf.p.y + wo.y, xf.p.z + wo.z};
    dbg_disc(center, b3RotateVector(xf.q, b3Vec3{1.0f, 0.0f, 0.0f}), sphere.radius, segments, c);
    dbg_disc(center, b3RotateVector(xf.q, b3Vec3{0.0f, 1.0f, 0.0f}), sphere.radius, segments, c);
    dbg_disc(center, b3RotateVector(xf.q, b3Vec3{0.0f, 0.0f, 1.0f}), sphere.radius, segments, c);
}

void adapter_segment(b3Pos p1, b3Pos p2, b3HexColor c, void* ctx) {
    ignore ctx;
    dbg_line(p1, p2, c);
}

// Three axes, red/green/blue.
void dbg_line_alpha(b3Pos a, b3Pos b, b3HexColor c, f32 alpha) {
    float4 col = hex_to_rgba(c);
    dbg_line_rgba(a.x, a.y, a.z, b.x, b.y, b.z, col.x, col.y, col.z, alpha);
}

void dbg_point_alpha(b3Pos p, f32 size, b3HexColor c, f32 alpha) {
    float4 col = hex_to_rgba(c);
    f32 k = 0.002f * size * cam_radius;
    dbg_line_rgba(p.x - k, p.y, p.z, p.x + k, p.y, p.z, col.x, col.y, col.z, alpha);
    dbg_line_rgba(p.x, p.y - k, p.z, p.x, p.y + k, p.z, col.x, col.y, col.z, alpha);
    dbg_line_rgba(p.x, p.y, p.z - k, p.x, p.y, p.z + k, col.x, col.y, col.z, alpha);
}

// upstream gfx/draw.c DrawCross: three axis-aligned strokes of `size`.
void dbg_cross(b3Pos center, f32 size, b3HexColor c) {
    float4 col = hex_to_rgba(c);
    f32 h = size * 0.5f;
    dbg_line_rgb(center.x - h, center.y, center.z, center.x + h, center.y, center.z,
                 col.x, col.y, col.z);
    dbg_line_rgb(center.x, center.y - h, center.z, center.x, center.y + h, center.z,
                 col.x, col.y, col.z);
    dbg_line_rgb(center.x, center.y, center.z - h, center.x, center.y, center.z + h,
                 col.x, col.y, col.z);
}

// upstream gfx/draw.c DrawGrid: a lattice in the plane of `normal`.
void dbg_grid(b3Pos center, b3Vec3 normal, f32 halfExtent, i32 divisions, f32 r, f32 g, f32 b) {
    if divisions < 1 || halfExtent <= 0.0f { return; }
    b3Vec3 c = b3Vec3{center.x, center.y, center.z};

    // Orthonormal in-plane axes from the normal.
    b3Vec3 n = b3Normalize(normal);
    b3Vec3 u = b3Normalize(b3Perp(n));
    b3Vec3 v = b3Cross(n, u);

    const f32 step = (2.0f * halfExtent) / cast(f32, divisions);
    for i32 i = 0; i <= divisions; i += 1 {
        const f32 o = -halfExtent + cast(f32, i) * step;
        // Line spanning u at this offset along v.
        b3Vec3 ua = b3Add(c, b3Add(b3MulSV(-halfExtent, u), b3MulSV(o, v)));
        b3Vec3 ub = b3Add(c, b3Add(b3MulSV(halfExtent, u), b3MulSV(o, v)));
        dbg_line_rgb(ua.x, ua.y, ua.z, ub.x, ub.y, ub.z, r, g, b);
        // Line spanning v at this offset along u.
        b3Vec3 va = b3Add(c, b3Add(b3MulSV(o, u), b3MulSV(-halfExtent, v)));
        b3Vec3 vb = b3Add(c, b3Add(b3MulSV(o, u), b3MulSV(halfExtent, v)));
        dbg_line_rgb(va.x, va.y, va.z, vb.x, vb.y, vb.z, r, g, b);
    }
}

void dbg_ground_grid(i32 size) {
    dbg_grid(b3Pos_zero, b3Vec3_axisY, cast(f32, size), size, 0.3f, 0.3f, 0.3f);
}

// upstream gfx/draw.c DrawArrow: a shaft plus two head strokes at
// DEFAULT_ARROW_HEAD_FRAC of its length.
void dbg_arrow(b3Pos a, b3Pos b, b3HexColor c) {
    float4 col = hex_to_rgba(c);
    b3Vec3 tail = b3Vec3{a.x, a.y, a.z};
    b3Vec3 tip = b3Vec3{b.x, b.y, b.z};
    dbg_line_rgb(tail.x, tail.y, tail.z, tip.x, tip.y, tip.z, col.x, col.y, col.z);

    b3Vec3 shaft = b3Sub(tip, tail);
    f32 shaftLen = b3Length(shaft);
    if shaftLen < 0.000001f { return; }
    b3Vec3 dir = b3Vec3{shaft.x / shaftLen, shaft.y / shaftLen, shaft.z / shaftLen};
    b3Vec3 perp = b3Perp(dir);
    f32 headLen = shaftLen * 0.15f;

    b3Vec3 backFromTip = b3MulSV(-headLen, dir);
    b3Vec3 sideStep = b3MulSV(headLen * 0.5f, perp);
    b3Vec3 tip1 = b3Add(tip, b3Add(backFromTip, sideStep));
    b3Vec3 tip2 = b3Add(tip, b3Sub(backFromTip, sideStep));
    dbg_line_rgb(tip.x, tip.y, tip.z, tip1.x, tip1.y, tip1.z, col.x, col.y, col.z);
    dbg_line_rgb(tip.x, tip.y, tip.z, tip2.x, tip2.y, tip2.z, col.x, col.y, col.z);
}

// upstream gfx/draw.c DrawAxes: the frame's basis, `size` long.
void dbg_axes(b3WorldTransform xf, f32 size) {
    b3Vec3 ax = b3RotateVector(xf.q, b3Vec3{size, 0.0f, 0.0f});
    b3Vec3 ay = b3RotateVector(xf.q, b3Vec3{0.0f, size, 0.0f});
    b3Vec3 az = b3RotateVector(xf.q, b3Vec3{0.0f, 0.0f, size});
    dbg_line_rgb(xf.p.x, xf.p.y, xf.p.z, xf.p.x + ax.x, xf.p.y + ax.y, xf.p.z + ax.z,
                 1.0f, 0.0f, 0.0f);
    dbg_line_rgb(xf.p.x, xf.p.y, xf.p.z, xf.p.x + ay.x, xf.p.y + ay.y, xf.p.z + ay.z,
                 0.0f, 1.0f, 0.0f);
    dbg_line_rgb(xf.p.x, xf.p.y, xf.p.z, xf.p.x + az.x, xf.p.y + az.y, xf.p.z + az.z,
                 0.0f, 0.0f, 1.0f);
}

void adapter_transform(b3WorldTransform xf, void* ctx) {
    ignore ctx;
    dbg_axes(xf, 0.5f * g_dbg_axis_scale);
}

// A line pipeline has no point primitive, so a point is a small cross.
// Box3D gives the size in pixels; scaled to world by the orbit distance.
void adapter_point(b3Pos p, f32 size, b3HexColor c, void* ctx) {
    ignore ctx;
    float4 col = hex_to_rgba(c);
    f32 k = 0.002f * size * cam_radius;
    dbg_line_rgb(p.x - k, p.y, p.z, p.x + k, p.y, p.z, col.x, col.y, col.z);
    dbg_line_rgb(p.x, p.y - k, p.z, p.x, p.y + k, p.z, col.x, col.y, col.z);
    dbg_line_rgb(p.x, p.y, p.z - k, p.x, p.y, p.z + k, col.x, col.y, col.z);
}
void adapter_sphere_prim(b3Pos p, f32 r, b3HexColor c, f32 a, void* ctx) {
    ignore p; ignore r; ignore c; ignore a; ignore ctx;
}
void adapter_capsule_prim(b3Pos p1, b3Pos p2, f32 r, b3HexColor c, f32 a, void* ctx) {
    ignore p1; ignore p2; ignore r; ignore c; ignore a; ignore ctx;
}
void adapter_bounds(b3AABB b, b3HexColor c, void* ctx) {
    ignore ctx;
    float4 col = hex_to_rgba(c);
    f32 x0 = b.lowerBound.x; f32 y0 = b.lowerBound.y; f32 z0 = b.lowerBound.z;
    f32 x1 = b.upperBound.x; f32 y1 = b.upperBound.y; f32 z1 = b.upperBound.z;
    f32 r = col.x; f32 g = col.y; f32 bl = col.z;
    dbg_line_rgb(x0, y0, z0, x1, y0, z0, r, g, bl);
    dbg_line_rgb(x0, y1, z0, x1, y1, z0, r, g, bl);
    dbg_line_rgb(x0, y0, z1, x1, y0, z1, r, g, bl);
    dbg_line_rgb(x0, y1, z1, x1, y1, z1, r, g, bl);
    dbg_line_rgb(x0, y0, z0, x0, y1, z0, r, g, bl);
    dbg_line_rgb(x1, y0, z0, x1, y1, z0, r, g, bl);
    dbg_line_rgb(x0, y0, z1, x0, y1, z1, r, g, bl);
    dbg_line_rgb(x1, y0, z1, x1, y1, z1, r, g, bl);
    dbg_line_rgb(x0, y0, z0, x0, y0, z1, r, g, bl);
    dbg_line_rgb(x1, y0, z0, x1, y0, z1, r, g, bl);
    dbg_line_rgb(x0, y1, z0, x0, y1, z1, r, g, bl);
    dbg_line_rgb(x1, y1, z0, x1, y1, z1, r, g, bl);
}
void adapter_box_prim(b3Vec3 e, b3WorldTransform xf, b3HexColor c, void* ctx) {
    ignore e; ignore xf; ignore c; ignore ctx;
}
void adapter_string(b3Pos p, u8* s, b3HexColor c, void* ctx) {
    ignore p; ignore s; ignore c; ignore ctx;
}

void adapter_attach_world_def(b3WorldDef* def) {
    def.createDebugShape = adapter_create_shape;
    def.destroyDebugShape = adapter_destroy_shape;
    def.userDebugShapeContext = null;
}

b3DebugDraw adapter_make_debug_draw() {
    b3DebugDraw d = b3DefaultDebugDraw();
    d.DrawShapeFcn = adapter_draw_shape;
    d.DrawSegmentFcn = adapter_segment;
    d.DrawTransformFcn = adapter_transform;
    d.DrawPointFcn = adapter_point;
    d.DrawSphereFcn = adapter_sphere_prim;
    d.DrawCapsuleFcn = adapter_capsule_prim;
    d.DrawBoundsFcn = adapter_bounds;
    d.DrawBoxFcn = adapter_box_prim;
    d.DrawStringFcn = adapter_string;
    d.drawShapes = g_dbg_shapes;
    d.drawJoints = g_dbg_joints;
    d.drawJointExtras = g_dbg_joint_extras;
    d.drawBounds = g_dbg_bounds;
    d.drawContacts = g_dbg_contacts;
    d.drawContactNormals = g_dbg_contact_normals;
    d.drawContactForces = g_dbg_contact_forces;
    d.drawMass = g_dbg_mass;
    d.drawSleep = g_dbg_sleep;
    d.drawIslands = g_dbg_islands;
    d.drawGraphColors = g_dbg_graph_colors;
    d.jointScale = g_dbg_joint_scale;
    d.forceScale = g_dbg_force_scale;
    // upstream Camera::DrawBounds (sample.cpp:490): a cube of the draw
    // distance about the eye. Box3D uses it to pick the draw set.
    f32 h = cam_draw_distance;
    d.drawingBounds = b3AABB{
        b3Pos{cam_eye.x - h, cam_eye.y - h, cam_eye.z - h},
        b3Pos{cam_eye.x + h, cam_eye.y + h, cam_eye.z + h}};
    d.context = null;
    return d;
}

// Rebuild the draw list from the world.
void adapter_collect() {
    // Neither list clears here: the sample's step runs before this and
    // has already drawn into both. dbg_reset owns the frame boundary.
    b3DebugDraw d = adapter_make_debug_draw();
    b3World_Draw(g_world, &d, DEBUG_DRAW_ALL_BITS);
}

// --- drawing ---------------------------------------------------------

// Camera-frustum cull, four side planes only (near/far clip convention
// is backend-specific). viewproj is column-major, so row i is
// (m[i], m[4+i], m[8+i], m[12+i]). Checked by test/box3d_probes.
bool adraw_view_skips(float4x4* viewproj, AdapterDraw* d) {
    f32* m = cast(f32*, viewproj);
    f32 x = d.xf.p.x;
    f32 y = d.xf.p.y;
    f32 z = d.xf.p.z;
    f32 r = d.bound;
    // Sphere and capsule children keep an identity child transform and
    // carry their geometry in c1/c2, so xf.p is the body origin.
    if d.kind == ASHAPE_SPHERE || d.kind == ASHAPE_CAPSULE {
        b3Vec3 mid = d.c1;
        f32 half = 0.0f;
        if d.kind == ASHAPE_CAPSULE {
            mid = b3Vec3{0.5f * (d.c1.x + d.c2.x),
                         0.5f * (d.c1.y + d.c2.y),
                         0.5f * (d.c1.z + d.c2.z)};
            f32 ex = d.c2.x - d.c1.x;
            f32 ey = d.c2.y - d.c1.y;
            f32 ez = d.c2.z - d.c1.z;
            half = 0.5f * sqrtf(ex * ex + ey * ey + ez * ez);
        }
        b3Vec3 w = b3RotateVector(d.xf.q, mid);
        x += w.x;
        y += w.y;
        z += w.z;
        r = half + d.radius;
    }
    for i32 p = 0; p < 4; p++ {
        i32 i = p / 2;
        f32 s = (p & 1) == 0 ? 1.0f : 0.0f - 1.0f;
        f32 a = m[3] + s * m[i];
        f32 b = m[7] + s * m[4 + i];
        f32 c = m[11] + s * m[8 + i];
        f32 w = m[15] + s * m[12 + i];
        f32 len = sqrtf(a * a + b * b + c * c);
        if len <= 0.0f { continue; }
        if (a * x + b * y + c * z + w) / len < 0.0f - r { return true; }
    }
    return false;
}

// A negative-determinant scale reverses winding.
bool adraw_mirrored(AdapterDraw* d) {
    return d.scale.x * d.scale.y * d.scale.z < 0.0f;
}

f32 hull_child_bound(AdapterChild* ch) {
    f32 b = ch.radius;
    if ch.mesh >= 0 { b = g_meshes[ch.mesh].bound; }
    // Compound mesh children scale per child; the cached bound doesn't.
    f32 s = absf(ch.scale.x);
    if absf(ch.scale.y) > s { s = absf(ch.scale.y); }
    if absf(ch.scale.z) > s { s = absf(ch.scale.z); }
    return b * s;
}

// Compose the child transform onto the body transform.
AdapterDraw achild_draw(AdapterDraw* d, AdapterChild* ch) {
    AdapterDraw o;
    o.kind = ch.kind;
    o.mesh = ch.mesh;
    b3Vec3 wo = b3RotateVector(d.xf.q, ch.xf.p);
    o.xf.p = b3Vec3{d.xf.p.x + wo.x, d.xf.p.y + wo.y, d.xf.p.z + wo.z};
    o.xf.q = b3MulQuat(d.xf.q, ch.xf.q);
    o.c1 = ch.c1;
    o.c2 = ch.c2;
    o.scale = ch.scale;
    o.radius = ch.radius;
    o.bound = d.bound;
    o.childStart = 0;
    o.childCount = 0;
    o.tint = d.tint;
    o.gridCell = d.gridCell;
    return o;
}

// Hull and mesh vertices are already in body space, so they need no
// local placement — only a triangle mesh's per-shape scale. Spheres
// have a centre, capsules an axis.
float4x4 adraw_model(AdapterDraw* d) {
    if d.kind == ASHAPE_SPHERE {
        b3Vec3 wo = b3RotateVector(d.xf.q, d.c1);
        return make_model(d.xf.q,
                          d.xf.p.x + wo.x, d.xf.p.y + wo.y, d.xf.p.z + wo.z,
                          d.radius, d.radius, d.radius);
    }
    if d.kind == ASHAPE_CAPSULE {
        f32 mx = 0.5f * (d.c1.x + d.c2.x);
        f32 my = 0.5f * (d.c1.y + d.c2.y);
        f32 mz = 0.5f * (d.c1.z + d.c2.z);
        b3Vec3 wo = b3RotateVector(d.xf.q, b3Vec3{mx, my, mz});
        f32 dx = d.c2.x - d.c1.x;
        f32 dy = d.c2.y - d.c1.y;
        f32 dz = d.c2.z - d.c1.z;
        f32 len = sqrtf(dx * dx + dy * dy + dz * dz);
        b3Quat lq = b3Quat_identity;
        if len > 0.000001f {
            lq = b3ComputeQuatBetweenUnitVectors(b3Vec3_axisX,
                     b3Vec3{dx / len, dy / len, dz / len});
        }
        return make_model(b3MulQuat(d.xf.q, lq),
                          d.xf.p.x + wo.x, d.xf.p.y + wo.y, d.xf.p.z + wo.z,
                          1.0f, 1.0f, 1.0f);
    }
    return make_model(d.xf.q, d.xf.p.x, d.xf.p.y, d.xf.p.z,
                      d.scale.x, d.scale.y, d.scale.z);
}

// The capsule mesh sizes itself in the shader from params.yz.
f32 inv_scale_sq(f32 s) {
    if s == 0.0f { return 1.0f; }
    return 1.0f / (s * s);
}

float4 adraw_params(AdapterDraw* d) {
    if d.kind == ASHAPE_CAPSULE {
        f32 dx = d.c2.x - d.c1.x;
        f32 dy = d.c2.y - d.c1.y;
        f32 dz = d.c2.z - d.c1.z;
        f32 halfLen = 0.5f * sqrtf(dx * dx + dy * dy + dz * dz);
        return float4{0.0f, halfLen, d.radius, 0.0f};
    }
    return float4{d.gridCell, inv_scale_sq(d.scale.x),
                  inv_scale_sq(d.scale.y), inv_scale_sq(d.scale.z)};
}

// --- passes ----------------------------------------------------------
//
// One walk of the collected list per frame fills a stream buffer with an
// InstRec per shape, grouped into buckets that share a pipeline, a mesh
// and a cull mode. Each pass then draws one call per bucket.

const i32 ICLS_MESH = 0;
const i32 ICLS_SPHERE = 1;
const i32 ICLS_CAPSULE = 2;

// Instances that survived the camera cull sit at the front of the
// bucket, so the lit pass draws the first `vis` and the shadow passes,
// which the camera frustum does not bound, draw all `count`.
struct IBucket {
    i32 cls;
    i32 mesh;       // mesh cache slot for ICLS_MESH, -1 otherwise
    bool mirror;    // negative-determinant scale, drawn front-culled
    bool casts;     // not the ground, so the shadow passes draw it
    i32 start;
    i32 count;
    i32 vis;
    i32 fill;       // cursors while grouping: visible up, culled down
    i32 fillBack;
}

// Geometry slots: one per cached mesh, plus the sphere and capsule
// impostors. Every bucket is one of these crossed with the two flags,
// so a bucket is found by direct index rather than a scan.
const i32 IGEOM_SPHERE = MESH_CACHE_MAX;
const i32 IGEOM_CAPSULE = MESH_CACHE_MAX + 1;
const i32 IGEOM_MAX = MESH_CACHE_MAX + 2;
const i32 IBUCKET_MAX = IGEOM_MAX * 4;
// every record plus every compound child it can expand to
const i32 INST_MAX = ADAPTER_DRAW_MAX + ACHILD_MAX;
const i32 INST_STRIDE = 80;      // sizeof(InstRec)

IBucket[IBUCKET_MAX] g_ibuckets;
i32 g_ibucket_count;
i32[IBUCKET_MAX] g_ibucket_of;   // key -> bucket index, -1 when unused
InstRec[INST_MAX] g_inst_walk;   // collection order
i32[INST_MAX] g_inst_bucket;
InstRec[INST_MAX] g_inst;        // grouped by bucket, the upload image
i32 g_inst_count;
i32 g_inst_dropped;
i32 g_inst_mirror;               // instances in mirrored buckets
i32 g_inst_caps;                 // instances in capsule buckets
sg_buffer g_inst_vbuf;

i32 ibucket_find(i32 cls, i32 mesh, bool mirror, bool casts) {
    i32 geom = mesh;
    if cls == ICLS_SPHERE { geom = IGEOM_SPHERE; }
    if cls == ICLS_CAPSULE { geom = IGEOM_CAPSULE; }
    if geom < 0 || geom >= IGEOM_MAX { return -1; }
    i32 key = geom * 4;
    if mirror { key += 1; }
    if casts { key += 2; }
    if g_ibucket_of[key] >= 0 { return g_ibucket_of[key]; }

    i32 n = g_ibucket_count;
    IBucket* b = &g_ibuckets[n];
    b.cls = cls;
    b.mesh = mesh;
    b.mirror = mirror;
    b.casts = casts;
    b.start = 0;
    b.count = 0;
    b.vis = 0;
    b.fill = 0;
    b.fillBack = 0;
    g_ibucket_of[key] = n;
    g_ibucket_count++;
    return n;
}

void inst_emit(AdapterDraw* d, i32 cls, bool visible) {
    if g_inst_count >= INST_MAX { g_inst_dropped++; return; }
    i32 mesh = cls == ICLS_MESH ? d.mesh : -1;
    bool mirror = cls == ICLS_MESH && adraw_mirrored(d);
    // the ground receives but never casts
    bool casts = d.gridCell == 0.0f;
    i32 bi = ibucket_find(cls, mesh, mirror, casts);
    if bi < 0 { g_inst_dropped++; return; }

    float4x4 m = adraw_model(d);
    f32* e = cast(f32*, &m);
    InstRec* r = &g_inst_walk[g_inst_count];
    r.row0 = float4{e[0], e[4], e[8], e[12]};
    r.row1 = float4{e[1], e[5], e[9], e[13]};
    r.row2 = float4{e[2], e[6], e[10], e[14]};
    r.tint = d.tint;
    r.params = adraw_params(d);
    // low bit carries visibility into the grouping pass
    g_inst_bucket[g_inst_count] = bi * 2 + (visible ? 1 : 0);
    g_ibuckets[bi].count++;
    if visible { g_ibuckets[bi].vis++; }
    g_inst_count++;
    if mirror { g_inst_mirror++; }
    if cls == ICLS_CAPSULE { g_inst_caps++; }
}

i32 inst_class(i32 kind) {
    if kind == ASHAPE_SPHERE { return ICLS_SPHERE; }
    if kind == ASHAPE_CAPSULE { return ICLS_CAPSULE; }
    if kind == ASHAPE_MESH { return ICLS_MESH; }
    return -1;
}

// Compound children are culled against the camera here, as the lit pass
// used to; they still reach the shadow passes, which the camera frustum
// does not bound.
void adapter_build_instances(float4x4* viewproj) {
    for i32 k = 0; k < IBUCKET_MAX; k++ { g_ibucket_of[k] = -1; }
    g_ibucket_count = 0;
    g_inst_count = 0;
    g_inst_dropped = 0;
    g_inst_mirror = 0;
    g_inst_caps = 0;
    g_ccd_frame = 0;

    for i32 i = 0; i < g_adraw_count; i++ {
        AdapterDraw* d = &g_adraws[i];
        if d.kind == ASHAPE_COMPOUND {
            for i32 c = 0; c < d.childCount; c++ {
                AdapterChild* ch = &g_achildren[d.childStart + c];
                i32 cls = inst_class(ch.kind);
                if cls < 0 { continue; }
                AdapterDraw cd = achild_draw(d, ch);
                cd.bound = hull_child_bound(ch);
                bool visible = !adraw_view_skips(viewproj, &cd);
                if visible { g_ccd_frame++; }
                inst_emit(&cd, cls, visible);
            }
            continue;
        }
        i32 cls = inst_class(d.kind);
        if cls < 0 { continue; }
        inst_emit(d, cls, true);
    }
    g_compound_children_drawn = g_ccd_frame;

    if g_inst_count == 0 { return; }
    i32 off = 0;
    for i32 b = 0; b < g_ibucket_count; b++ {
        g_ibuckets[b].start = off;
        g_ibuckets[b].fill = off;
        off += g_ibuckets[b].count;
        g_ibuckets[b].fillBack = off - 1;
    }
    for i32 i = 0; i < g_inst_count; i++ {
        IBucket* b = &g_ibuckets[g_inst_bucket[i] / 2];
        if (g_inst_bucket[i] & 1) != 0 {
            g_inst[b.fill] = g_inst_walk[i];
            b.fill++;
        } else {
            g_inst[b.fillBack] = g_inst_walk[i];
            b.fillBack--;
        }
    }
    // sokol allows one update per stream buffer per frame, so every pass
    // draws out of this one upload
    sg_update_buffer(g_inst_vbuf, &sg_range{
        .ptr = &g_inst, .size = cast(i64, g_inst_count * INST_STRIDE) });
}

sg_buffer ibucket_vbuf(IBucket* b) {
    if b.cls == ICLS_SPHERE { return g_sph_vbuf; }
    if b.cls == ICLS_CAPSULE { return g_cap_vbuf; }
    return g_meshes[b.mesh].vbuf;
}

i32 ibucket_nverts(IBucket* b) {
    if b.cls == ICLS_SPHERE { return g_sph_nverts; }
    if b.cls == ICLS_CAPSULE { return g_cap_nverts; }
    return g_meshes[b.mesh].nverts;
}

void ibucket_draw(IBucket* b, bool lit) {
    i32 n = lit ? b.vis : b.count;
    if n == 0 { return; }
    if lit {
        sg_apply_bindings(&sg_bindings{
            .vertex_buffers[0] = ibucket_vbuf(b),
            .vertex_buffers[1] = g_inst_vbuf,
            .vertex_buffer_offsets[1] = b.start * INST_STRIDE,
            .views[0] = g_shadow_tex,
            .samplers[0] = g_shadow_smp });
    } else {
        sg_apply_bindings(&sg_bindings{
            .vertex_buffers[0] = ibucket_vbuf(b),
            .vertex_buffers[1] = g_inst_vbuf,
            .vertex_buffer_offsets[1] = b.start * INST_STRIDE });
    }
    sg_draw(0, ibucket_nverts(b), n);
}

void adapter_draw_depth(i32 cascade) {
    if g_inst_count == 0 { return; }
    InstPass ip;
    ip.viewproj = g_light_clip[cascade];

    sg_apply_pipeline(g_pip_inst_depth);
    sg_apply_uniforms(0, &sg_range{ .ptr = &ip, .size = sizeof(ip) });
    for i32 i = 0; i < g_ibucket_count; i++ {
        IBucket* b = &g_ibuckets[i];
        if !b.casts || b.cls == ICLS_CAPSULE { continue; }
        ibucket_draw(b, false);
    }

    if g_inst_caps == 0 { return; }
    sg_apply_pipeline(g_pip_inst_depth_cap);
    sg_apply_uniforms(0, &sg_range{ .ptr = &ip, .size = sizeof(ip) });
    for i32 i = 0; i < g_ibucket_count; i++ {
        IBucket* b = &g_ibuckets[i];
        if !b.casts || b.cls != ICLS_CAPSULE { continue; }
        ibucket_draw(b, false);
    }
}

void adapter_draw_lit(float4x4* viewproj, ShadowUni* shu) {
    if g_inst_count == 0 { return; }
    InstPass ip;
    ip.viewproj = *viewproj;

    // back-culled, then the mirrored ones front-culled
    for i32 pass = 0; pass < 2; pass++ {
        bool mirror = pass == 1;
        if mirror && g_inst_mirror == 0 { continue; }
        sg_apply_pipeline(mirror ? g_pip_inst_mirror : g_pip_inst);
        sg_apply_uniforms(0, &sg_range{ .ptr = &ip, .size = sizeof(ip) });
        sg_apply_uniforms(1, &sg_range{ .ptr = shu, .size = sizeof(*shu) });
        for i32 i = 0; i < g_ibucket_count; i++ {
            IBucket* b = &g_ibuckets[i];
            if b.mirror != mirror || b.cls == ICLS_CAPSULE { continue; }
            ibucket_draw(b, true);
        }
    }

    if g_inst_caps == 0 { return; }
    sg_apply_pipeline(g_pip_inst_cap);
    sg_apply_uniforms(0, &sg_range{ .ptr = &ip, .size = sizeof(ip) });
    sg_apply_uniforms(1, &sg_range{ .ptr = shu, .size = sizeof(*shu) });
    for i32 i = 0; i < g_ibucket_count; i++ {
        IBucket* b = &g_ibuckets[i];
        if b.cls != ICLS_CAPSULE { continue; }
        ibucket_draw(b, true);
    }
}

// Upload the line work and draw it in one call.
void adapter_draw_debug_lines(float4x4* viewproj) {
    if g_dbg_vert_count == 0 { return; }
    sg_update_buffer(g_dbg_vbuf, &sg_range{
        .ptr = &g_dbg_verts,
        .size = cast(i64, g_dbg_vert_count * DBG_FLOATS_PER_VERT * 4) });
    sg_apply_pipeline(g_pip_dbg);
    sg_apply_bindings(&sg_bindings{ .vertex_buffers[0] = g_dbg_vbuf });
    DbgUni u;
    u.viewproj = *viewproj;
    sg_apply_uniforms(0, &sg_range{ .ptr = &u, .size = sizeof(u) });
    sg_draw(0, g_dbg_vert_count, 1);
}

// upstream gfx/edges.c: hull edges draw in a fixed grey, mesh edges in
// the flat-class colour. showEdgeConvexity is false by default, so the
// convex/concave colours never appear without the toggle.
// The edge shader expands in pixels, so it needs the viewport; w is
// upstream's zBias.
float4 edge_params() {
    return float4{sapp_widthf(), sapp_heightf(), 0.0f, EDGE_Z_BIAS};
}

float4 edge_tint(i32 meshSlot) {
    // upstream colours heightfield edges with the mesh colours; only
    // hulls take the separate one
    if g_meshes[meshSlot].kind != MESH_KIND_HULL {
        return float4{0.6f, 0.6f, 0.6f, 0.5f};
    }
    return float4{0.5f, 0.5f, 0.5f, 0.5f};
}

// Shape outlines from the edge list built with the geometry. The depth
// tie is broken in the shader, not by moving the geometry.
void adapter_draw_outlines(float4x4* viewproj) {
    sg_apply_pipeline(g_pip_lines_ns);
    i32 curMesh = -1;
    for i32 i = 0; i < g_adraw_count; i++ {
        AdapterDraw* d = &g_adraws[i];
        if d.kind == ASHAPE_COMPOUND {
            for i32 c = 0; c < d.childCount; c++ {
                AdapterChild* ch = &g_achildren[d.childStart + c];
                if ch.kind != ASHAPE_MESH { continue; }
                if g_meshes[ch.mesh].nedges == 0 { continue; }
                AdapterDraw cd = achild_draw(d, ch);
                cd.bound = hull_child_bound(ch);
                if adraw_view_skips(viewproj, &cd) { continue; }
                sg_apply_bindings(&sg_bindings{ .vertex_buffers[0] = g_edge_corner_vbuf,
                                                .vertex_buffers[1] = g_meshes[cd.mesh].ebuf });
                curMesh = cd.mesh;
                PerDraw pd;
                // no geometric nudge: the shader biases in clip space
                pd.model = adraw_model(&cd);
                pd.mvp = mul(*viewproj, pd.model);
                pd.tint = edge_tint(cd.mesh);
                pd.params = edge_params();
                sg_apply_uniforms(0, &sg_range{ .ptr = &pd, .size = sizeof(pd) });
                sg_draw(0, 6, g_meshes[cd.mesh].nedges);
            }
            continue;
        }
        if d.kind != ASHAPE_MESH { continue; }
        if g_meshes[d.mesh].nedges == 0 { continue; }
        if d.mesh != curMesh {
            sg_apply_bindings(&sg_bindings{ .vertex_buffers[0] = g_edge_corner_vbuf,
                                            .vertex_buffers[1] = g_meshes[d.mesh].ebuf });
            curMesh = d.mesh;
        }
        PerDraw pd;
        pd.model = adraw_model(d);
        pd.mvp = mul(*viewproj, pd.model);
        pd.tint = edge_tint(d.mesh);
        pd.params = edge_params();
        sg_apply_uniforms(0, &sg_range{ .ptr = &pd, .size = sizeof(pd) });
        sg_draw(0, 6, g_meshes[d.mesh].nedges);
    }
}
