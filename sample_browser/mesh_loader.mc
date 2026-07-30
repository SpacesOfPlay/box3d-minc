// Wavefront .obj loading. Port of samples/mesh_loader.cpp.
//
// Upstream calls tinyobjloader with triangulate = true and reads back
// only attrib.vertices and idx.vertex_index — normals, texcoords,
// materials and groups are parsed and discarded. This does the same
// directly: `v` and `f` lines, positive indices, n-gons triangulated by
// projected ear clipping the way tinyobjloader's earcut path does.
//
// Not supported, because no shipped mesh uses it: negative (relative)
// indices, line continuations, `vp`/curve elements.

import box3d;
import math;
import str;
import file;

// upstream TempMesh: the three parallel arrays, owned.
struct TempMesh {
    b3Vec3* vertices;
    i32* indices;
    u8* materialIndices;
    i32 vertexCount;
    i32 triangleCount;
}

bool obj_is_space(u8 c) {
    return c == 32 || c == 9 || c == 13;
}

bool obj_is_digit(u8 c) {
    return c >= 48 && c <= 57;
}

i32 obj_skip_space(u8* s, i32 end, i32 i) {
    while i < end && obj_is_space(s[i]) { i += 1; }
    return i;
}

// The meshes carry up to nine decimals against magnitudes near 3200,
// past f32's seven significant digits. Accumulate every digit into an
// integer mantissa and divide once, so the result is the correctly
// rounded f32 rather than a drifted one.
f32 obj_parse_f32(u8* s, i32 end, i32* io) {
    i32 i = obj_skip_space(s, end, *io);
    bool neg = false;
    if i < end && (s[i] == 43 || s[i] == 45) {
        neg = s[i] == 45;
        i += 1;
    }

    i64 mantissa = cast(i64, 0);
    while i < end && obj_is_digit(s[i]) {
        mantissa = mantissa * cast(i64, 10) + cast(i64, s[i] - 48);
        i += 1;
    }

    i32 fracDigits = 0;
    if i < end && s[i] == 46 {
        i += 1;
        while i < end && obj_is_digit(s[i]) {
            mantissa = mantissa * cast(i64, 10) + cast(i64, s[i] - 48);
            fracDigits += 1;
            i += 1;
        }
    }

    // Exact: powers of ten up to 1e22 are representable in f64.
    f64 divisor = 1.0;
    for i32 k = 0; k < fracDigits; k += 1 { divisor *= 10.0; }

    f64 value = cast(f64, mantissa) / divisor;
    *io = i;
    return cast(f32, neg ? -value : value);
}

// One face token: the vertex index, then everything up to the next space.
// `1`, `1/2`, `1//3` and `1/2/3` all yield 1.
i32 obj_parse_index(u8* s, i32 end, i32* io) {
    i32 i = obj_skip_space(s, end, *io);
    i32 value = 0;
    while i < end && obj_is_digit(s[i]) {
        value = value * 10 + cast(i32, s[i] - 48);
        i += 1;
    }
    while i < end && obj_is_space(s[i]) == false { i += 1; }
    *io = i;
    return value;
}

// Vertex count, triangle count after triangulation, and the widest face,
// so the emit pass can size everything exactly.
void obj_count(u8* s, i32 len, i32* outVertices, i32* outTriangles, i32* outMaxArity) {
    i32 vertices = 0;
    i32 triangles = 0;
    i32 maxArity = 0;

    i32 i = 0;
    while i < len {
        i32 lineEnd = i;
        while lineEnd < len && s[lineEnd] != 10 { lineEnd += 1; }

        i32 p = obj_skip_space(s, lineEnd, i);
        if p < lineEnd && s[p] == 118 && p + 1 < lineEnd && obj_is_space(s[p + 1]) {
            vertices += 1;
        } else if p < lineEnd && s[p] == 102 && p + 1 < lineEnd && obj_is_space(s[p + 1]) {
            i32 arity = 0;
            i32 q = p + 1;
            while true {
                q = obj_skip_space(s, lineEnd, q);
                if q >= lineEnd { break; }
                while q < lineEnd && obj_is_space(s[q]) == false { q += 1; }
                arity += 1;
            }
            if arity > maxArity { maxArity = arity; }
            if arity >= 3 { triangles += arity - 2; }
        }

        i = lineEnd < len ? lineEnd + 1 : len;
    }

    *outVertices = vertices;
    *outTriangles = triangles;
    *outMaxArity = maxArity;
}

f32 obj_cross2(f32 ax, f32 ay, f32 bx, f32 by, f32 cx, f32 cy) {
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

bool obj_in_triangle(f32 ax, f32 ay, f32 bx, f32 by, f32 cx, f32 cy, f32 px, f32 py) {
    f32 d1 = obj_cross2(ax, ay, bx, by, px, py);
    f32 d2 = obj_cross2(bx, by, cx, cy, px, py);
    f32 d3 = obj_cross2(cx, cy, ax, ay, px, py);
    bool hasNeg = d1 < 0.0f || d2 < 0.0f || d3 < 0.0f;
    bool hasPos = d1 > 0.0f || d2 > 0.0f || d3 > 0.0f;
    return !(hasNeg && hasPos);
}

// Ear clipping over the projected ring. Emits positions in the ring, not
// vertex indices, so the caller keeps the original winding. Mirrored
// projections are harmless: the signed area decides the ear test's sign,
// and the emitted order still follows the ring.
i32 obj_ear_clip(f32* px, f32* py, i32 n, i32* prev, i32* next, i32* out) {
    for i32 i = 0; i < n; i += 1 {
        next[i] = (i + 1) % n;
        prev[i] = (i + n - 1) % n;
    }

    f32 area = 0.0f;
    for i32 i = 0; i < n; i += 1 {
        i32 j = (i + 1) % n;
        area += px[i] * py[j] - px[j] * py[i];
    }
    f32 sign = area >= 0.0f ? 1.0f : -1.0f;

    i32 count = 0;
    i32 remaining = n;
    i32 v = 0;
    i32 misses = 0;
    while remaining > 2 && misses <= remaining {
        i32 a = prev[v];
        i32 c = next[v];
        bool ear = sign * obj_cross2(px[a], py[a], px[v], py[v], px[c], py[c]) > 0.0f;
        i32 k = next[c];
        while ear && k != a {
            if obj_in_triangle(px[a], py[a], px[v], py[v], px[c], py[c], px[k], py[k]) {
                ear = false;
            }
            k = next[k];
        }
        if ear {
            out[count + 0] = a;
            out[count + 1] = v;
            out[count + 2] = c;
            count += 3;
            next[a] = c;
            prev[c] = a;
            remaining -= 1;
            misses = 0;
            v = a;
        } else {
            misses += 1;
            v = c;
        }
    }

    // A degenerate ring leaves ears unfound. Fan the remainder rather
    // than drop the face.
    if count < 3 * (n - 2) {
        count = 0;
        for i32 i = 1; i + 1 < n; i += 1 {
            out[count + 0] = 0;
            out[count + 1] = i;
            out[count + 2] = i + 1;
            count += 3;
        }
    }
    return count;
}

// Parse into caller-owned buffers sized by obj_count.
void obj_emit(u8* s, i32 len, TempMesh* mesh, f32 scale, bool zUp, i32 maxArity) {
    i32 vertexCount = 0;
    i32 triangleCount = 0;
    i32 materialIndex = 0;

    // Face scratch: the ring's vertex indices, its projection, and the
    // ear clipper's links.
    i32* ring = cast(i32*, alloc(cast(i64, maxArity * 4)));
    f32* px = cast(f32*, alloc(cast(i64, maxArity * 4)));
    f32* py = cast(f32*, alloc(cast(i64, maxArity * 4)));
    i32* prev = cast(i32*, alloc(cast(i64, maxArity * 4)));
    i32* next = cast(i32*, alloc(cast(i64, maxArity * 4)));
    i32* fan = cast(i32*, alloc(cast(i64, 3 * maxArity * 4)));

    i32 i = 0;
    while i < len {
        i32 lineEnd = i;
        while lineEnd < len && s[lineEnd] != 10 { lineEnd += 1; }

        i32 p = obj_skip_space(s, lineEnd, i);
        if p < lineEnd && s[p] == 118 && p + 1 < lineEnd && obj_is_space(s[p + 1]) {
            i32 q = p + 1;
            f32 x = scale * obj_parse_f32(s, lineEnd, &q);
            f32 y = scale * obj_parse_f32(s, lineEnd, &q);
            f32 z = scale * obj_parse_f32(s, lineEnd, &q);
            mesh.vertices[vertexCount] = zUp ? b3Vec3{y, z, x} : b3Vec3{x, y, z};
            vertexCount += 1;
        } else if p < lineEnd && s[p] == 102 && p + 1 < lineEnd && obj_is_space(s[p + 1]) {
            i32 arity = 0;
            i32 q = p + 1;
            while true {
                q = obj_skip_space(s, lineEnd, q);
                if q >= lineEnd { break; }
                i32 index = obj_parse_index(s, lineEnd, &q);
                if arity < maxArity {
                    ring[arity] = index - 1; // .obj indices are 1-based
                    arity += 1;
                }
            }
            if arity >= 3 {
                i32 triangles = arity - 2;
                if arity == 3 {
                    fan[0] = 0;
                    fan[1] = 1;
                    fan[2] = 2;
                } else {
                    // Newell normal, then drop the dominant axis to get a
                    // 2D ring. This is what tinyobjloader does before
                    // handing the face to earcut.
                    f32 nx = 0.0f;
                    f32 ny = 0.0f;
                    f32 nz = 0.0f;
                    for i32 k = 0; k < arity; k += 1 {
                        b3Vec3 a = mesh.vertices[ring[k]];
                        b3Vec3 b = mesh.vertices[ring[(k + 1) % arity]];
                        nx += (a.y - b.y) * (a.z + b.z);
                        ny += (a.z - b.z) * (a.x + b.x);
                        nz += (a.x - b.x) * (a.y + b.y);
                    }
                    f32 ax = b3AbsFloat(nx);
                    f32 ay = b3AbsFloat(ny);
                    f32 az = b3AbsFloat(nz);
                    for i32 k = 0; k < arity; k += 1 {
                        b3Vec3 vtx = mesh.vertices[ring[k]];
                        if ax >= ay && ax >= az {
                            px[k] = vtx.y;
                            py[k] = vtx.z;
                        } else if ay >= az {
                            px[k] = vtx.z;
                            py[k] = vtx.x;
                        } else {
                            px[k] = vtx.x;
                            py[k] = vtx.y;
                        }
                    }
                    ignore obj_ear_clip(px, py, arity, prev, next, fan);
                }

                for i32 t = 0; t < triangles; t += 1 {
                    mesh.indices[3 * triangleCount + 0] = ring[fan[3 * t + 0]];
                    mesh.indices[3 * triangleCount + 1] = ring[fan[3 * t + 1]];
                    mesh.indices[3 * triangleCount + 2] = ring[fan[3 * t + 2]];
                    mesh.materialIndices[triangleCount] = cast(u8, materialIndex);
                    materialIndex = (materialIndex + 1) % 3;
                    triangleCount += 1;
                }
            }
        }

        i = lineEnd < len ? lineEnd + 1 : len;
    }

    free(cast(void*, ring));
    free(cast(void*, px));
    free(cast(void*, py));
    free(cast(void*, prev));
    free(cast(void*, next));
    free(cast(void*, fan));

    mesh.vertexCount = vertexCount;
    mesh.triangleCount = triangleCount;
}

void destroy_temp_mesh(TempMesh* mesh) {
    free(cast(void*, mesh.vertices));
    free(cast(void*, mesh.indices));
    free(cast(void*, mesh.materialIndices));
    mesh.vertices = null;
    mesh.indices = null;
    mesh.materialIndices = null;
    mesh.vertexCount = 0;
    mesh.triangleCount = 0;
}

// upstream LoadTempMesh. Leaves the mesh empty if the file is missing —
// upstream calls exit(1), which would take a browser tab down with it.
void load_temp_mesh(str path, TempMesh* mesh, f32 scale, bool zUp) {
    mesh.vertices = null;
    mesh.indices = null;
    mesh.materialIndices = null;
    mesh.vertexCount = 0;
    mesh.triangleCount = 0;

    FileData fd = file_read(path);
    if fd.data == null || fd.len == 0 {
        eprint("mesh_loader: cannot read {}\n", path);
        return;
    }

    i32 vertexCount = 0;
    i32 triangleCount = 0;
    i32 maxArity = 0;
    obj_count(fd.data, fd.len, &vertexCount, &triangleCount, &maxArity);

    if vertexCount == 0 || triangleCount == 0 {
        eprint("mesh_loader: {} has no geometry\n", path);
        free(cast(void*, fd.data));
        return;
    }

    mesh.vertices = cast(b3Vec3*, alloc(cast(i64, vertexCount * 12)));
    mesh.indices = cast(i32*, alloc(cast(i64, triangleCount * 3 * 4)));
    mesh.materialIndices = cast(u8*, alloc(cast(i64, triangleCount)));

    obj_emit(fd.data, fd.len, mesh, scale, zUp, maxArity);
    free(cast(void*, fd.data));
}

// upstream CreateMeshData. Returns null if the file is missing.
b3MeshData* create_mesh_data(str path, f32 scale, bool zUp, bool useMedianSplit,
                             bool identifyConvexEdges, bool weldVertices) {
    TempMesh mesh;
    load_temp_mesh(path, &mesh, scale, zUp);
    if mesh.vertexCount == 0 || mesh.triangleCount == 0 {
        return null;
    }

    b3MeshDef def = b3MeshDef{};
    def.vertices = mesh.vertices;
    def.vertexCount = mesh.vertexCount;
    def.indices = mesh.indices;
    def.triangleCount = mesh.triangleCount;
    def.materialIndices = mesh.materialIndices;
    def.useMedianSplit = useMedianSplit;
    def.identifyEdges = identifyConvexEdges;
    def.weldVertices = weldVertices;
    def.weldTolerance = 0.002f;

    b3MeshData* meshData = b3CreateMesh(&def, null, 0);
    destroy_temp_mesh(&mesh);
    return meshData;
}
