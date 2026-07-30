// Dynamic tree benchmark. Port of samples/sample_tree.cpp.
//
// Upstream keeps the records in `std::unordered_map<uint64_t, Proxy>`.
// The key starts at 0 and increments once per record, and every lookup
// is either a loop index or `node->userData`, which is set to that same
// index at CreateProxy time — a growable array wearing a map's
// interface, so a plain array replaces it.

import box3d;
import sokol_all;
import sokol_imgui;
import math;
import str;
import file;
import camera;
import gui;
import debug_adapter;
import sample;
import mesh_loader;

struct TreeProxy {
    b3AABB aabb;
    i32 proxyId;
    i32 queryTimeStamp;
    i32 rayTimeStamp;
}

struct TreeRay {
    b3Vec3 origin;
    b3Vec3 translation;
}

const i32 TB_FILE_COUNT = 3;
const i32 TB_TEST_COUNT = 1024;
const i32 TB_DRAW_COLOR_COUNT = 20;

b3DynamicTree g_tb_tree;
TreeProxy* g_tb_proxies;
i32 g_tb_proxy_count;
u16* g_tb_depths;
TreeRay[TB_TEST_COUNT] g_tb_rays;
b3AABB[TB_TEST_COUNT] g_tb_overlap_queries;
b3Sphere[TB_TEST_COUNT] g_tb_closest_queries;
b3Pos g_tb_closest_point;
i32 g_tb_file_index;
i32 g_tb_test_index;
i32 g_tb_time_stamp;
i32 g_tb_draw_level;
i32 g_tb_height;
f32 g_tb_area_ratio;
f32 g_tb_ray_time;
f32 g_tb_overlap_time;
f32 g_tb_closest_time;
f32 g_tb_build_time;
f32 g_tb_draw_distance;
f32 g_tb_load_scale;
bool g_tb_have_closest;
bool g_tb_do_overlap;
bool g_tb_do_closest;
bool g_tb_do_ray;

u8*[TB_FILE_COUNT] g_tb_file_names;
f32[TB_FILE_COUNT] g_tb_scales;
bool[TB_FILE_COUNT] g_tb_zups;

const str TB_SAVE_FILE_NAME = "tree.dt";

b3HexColor[TB_DRAW_COLOR_COUNT] g_tb_level_colors;

void tb_fill_level_colors() {
    g_tb_level_colors[0] = b3_colorAliceBlue;
    g_tb_level_colors[1] = b3_colorAntiqueWhite;
    g_tb_level_colors[2] = b3_colorAqua;
    g_tb_level_colors[3] = b3_colorAquamarine;
    g_tb_level_colors[4] = b3_colorAzure;
    g_tb_level_colors[5] = b3_colorBeige;
    g_tb_level_colors[6] = b3_colorBisque;
    g_tb_level_colors[7] = b3_colorBlanchedAlmond;
    g_tb_level_colors[8] = b3_colorBlue;
    g_tb_level_colors[9] = b3_colorBlueViolet;
    g_tb_level_colors[10] = b3_colorBrown;
    g_tb_level_colors[11] = b3_colorBurlywood;
    g_tb_level_colors[12] = b3_colorCadetBlue;
    g_tb_level_colors[13] = b3_colorChartreuse;
    g_tb_level_colors[14] = b3_colorChocolate;
    g_tb_level_colors[15] = b3_colorCoral;
    g_tb_level_colors[16] = b3_colorCornflowerBlue;
    g_tb_level_colors[17] = b3_colorCornsilk;
    g_tb_level_colors[18] = b3_colorCrimson;
    g_tb_level_colors[19] = b3_colorCyan;
}

// Upstream builds the path with snprintf. The three are spelled out
// here so the name stays a str and needs no conversion.
str tb_file_path(i32 index) {
    if index == 0 { return "data/trees/bounds01.txt"; }
    if index == 1 { return "data/trees/bounds02.txt"; }
    return "data/trees/bounds03.txt";
}

// Breath-first search to compute node depth values start with 0 at the root.
i32 tb_compute_depths() {
    i32 capacity = g_tb_tree.nodeCapacity;
    if g_tb_depths != null { free(cast(void*, g_tb_depths)); }
    g_tb_depths = cast(u16*, alloc(cast(i64, capacity * 2)));
    for i32 i = 0; i < capacity; i += 1 { g_tb_depths[i] = cast(u16, 0); }

    i32* queue = cast(i32*, alloc(cast(i64, g_tb_tree.nodeCount * 4)));
    i32 front = 0;
    i32 back = 0;

    b3TreeNode* nodes = g_tb_tree.nodes;
    queue[0] = g_tb_tree.root;
    back += 1;
    i32 depth = 0;

    while back > front {
        i32 index = queue[front];
        front += 1;

        b3TreeNode* node = nodes + index;
        if node.parent == -1 {
            g_tb_depths[index] = cast(u16, 0);
        } else {
            g_tb_depths[index] = cast(u16, cast(i32, g_tb_depths[node.parent]) + 1);
            depth = b3MaxInt(depth, cast(i32, g_tb_depths[index]));
        }

        if (cast(i32, node.flags) & cast(i32, b3_leafNode)) == 0 {
            queue[back] = node.children.child1;
            back += 1;
            queue[back] = node.children.child2;
            back += 1;
        }
    }

    free(cast(void*, queue));

    return depth;
}

// Six whitespace-separated floats. Returns false the way upstream's
// `parsed != 6` does, and the line is skipped.
bool tb_parse_bounds(u8* s, i32 lineEnd, i32 start, f32* out) {
    i32 i = start;
    for i32 k = 0; k < 6; k += 1 {
        i32 before = obj_skip_space(s, lineEnd, i);
        if before >= lineEnd { return false; }
        i32 p = before;
        out[k] = obj_parse_f32(s, lineEnd, &p);
        if p == before { return false; }
        i = p;
    }
    return true;
}

void tb_create_tree() {
    if g_tb_tree.nodeCapacity > 0 {
        b3DynamicTree_Destroy(&g_tb_tree);
        g_tb_tree = b3DynamicTree{};
    }

    g_tb_proxy_count = 0;
    if g_tb_proxies != null {
        free(cast(void*, g_tb_proxies));
        g_tb_proxies = null;
    }

    g_tb_tree = b3DynamicTree_Create(512);

    f32 scale = g_tb_scales[g_tb_file_index];
    bool zUp = g_tb_zups[g_tb_file_index];

    FileData fd = file_read(tb_file_path(g_tb_file_index));
    if fd.data == null || fd.len == 0 {
        eprint("sample_tree: cannot read {}\n", tb_file_path(g_tb_file_index));
        return;
    }

    i32 lineCount = 1;
    for i32 i = 0; i < fd.len; i += 1 {
        if fd.data[i] == 10 { lineCount += 1; }
    }
    g_tb_proxies = cast(TreeProxy*, alloc(cast(i64, lineCount * 40)));

    f32 maxArea = 0.0f;
    i32 maxAreaIndex = -1;

    i32 pos = 0;
    while pos < fd.len {
        i32 lineEnd = pos;
        while lineEnd < fd.len && fd.data[lineEnd] != 10 { lineEnd += 1; }

        i32 trim = pos;
        while trim < lineEnd && (fd.data[trim] == 32 || fd.data[trim] == 9) { trim += 1; }

        if trim < lineEnd && fd.data[trim] != 35 {
            f32[6] b;
            if tb_parse_bounds(fd.data, lineEnd, trim, cast(f32*, &b)) {
                b3AABB aabb;
                if zUp {
                    aabb.lowerBound = b3Vec3{scale * b[0], scale * b[2], scale * b[1]};
                    aabb.upperBound = b3Vec3{scale * b[3], scale * b[5], scale * b[4]};
                } else {
                    aabb.lowerBound = b3Vec3{scale * b[0], scale * b[1], scale * b[2]};
                    aabb.upperBound = b3Vec3{scale * b[3], scale * b[4], scale * b[5]};
                }

                f32 area = b3AABB_Area(aabb);
                if area > maxArea {
                    maxArea = area;
                    maxAreaIndex = g_tb_proxy_count;
                }

                g_tb_proxies[g_tb_proxy_count] = TreeProxy{aabb, 0, 0, 0};
                g_tb_proxy_count += 1;
            }
        }

        pos = lineEnd + 1;
    }

    free(cast(void*, fd.data));
    ignore maxAreaIndex;

    u64 ticks = b3GetTicks();
    for i32 i = 0; i < g_tb_proxy_count; i += 1 {
        // Arbitrary category bits to ensure correctness
        g_tb_proxies[i].proxyId = b3DynamicTree_CreateProxy(&g_tb_tree, g_tb_proxies[i].aabb,
                                                            cast(u64, 1), cast(u64, i));
        g_tb_proxies[i].queryTimeStamp = 0;
        g_tb_proxies[i].rayTimeStamp = 0;
    }

    g_tb_build_time = b3GetMilliseconds(ticks);
    b3DynamicTree_Validate(&g_tb_tree);
    g_tb_area_ratio = b3DynamicTree_GetAreaRatio(&g_tb_tree);

    g_tb_height = tb_compute_depths();
    g_tb_draw_level = -1;

    tb_generate();
}

void tb_load_tree() {
    if g_tb_tree.nodeCapacity > 0 {
        b3DynamicTree_Destroy(&g_tb_tree);
        g_tb_tree = b3DynamicTree{};
    }

    g_tb_proxy_count = 0;
    if g_tb_proxies != null {
        free(cast(void*, g_tb_proxies));
        g_tb_proxies = null;
    }

    g_tb_tree = b3DynamicTree_Load(TB_SAVE_FILE_NAME.data, g_tb_load_scale);

    // Load returns an empty tree when the file is missing. Upstream
    // walks it anyway; the depth BFS reads node 0 through a null
    // pointer, so stop here instead.
    if g_tb_tree.nodeCapacity == 0 || g_tb_tree.nodes == null {
        return;
    }

    i32 capacity = g_tb_tree.nodeCapacity;
    g_tb_proxies = cast(TreeProxy*, alloc(cast(i64, capacity * 40)));
    for i32 i = 0; i < capacity; i += 1 {
        g_tb_proxies[i] = TreeProxy{b3AABB{b3Vec3_zero, b3Vec3_zero}, 0, 0, 0};
    }
    g_tb_proxy_count = capacity;

    for i32 i = 0; i < capacity; i += 1 {
        b3TreeNode* node = g_tb_tree.nodes + i;

        if (cast(i32, node.flags) & cast(i32, b3_leafNode)) == 0 {
            continue;
        }

        i32 slot = cast(i32, node.userData);
        if slot < 0 || slot >= capacity { continue; }

        g_tb_proxies[slot].aabb = node.aabb;
        g_tb_proxies[slot].proxyId = i;
    }

    g_tb_build_time = 0.0f;
    b3DynamicTree_Validate(&g_tb_tree);
    g_tb_area_ratio = b3DynamicTree_GetAreaRatio(&g_tb_tree);

    g_tb_height = tb_compute_depths();
    g_tb_draw_level = -1;

    tb_generate();
}

f32 tb_ray_callback(b3RayCastInput* input, i32 proxyId, u64 userData, void* context) {
    ignore input;
    ignore proxyId;
    ignore context;
    g_tb_proxies[cast(i32, userData)].rayTimeStamp = g_tb_time_stamp;
    return 1.0f;
}

bool tb_query_callback(i32 proxyId, u64 userData, void* context) {
    ignore proxyId;
    ignore context;
    g_tb_proxies[cast(i32, userData)].queryTimeStamp = g_tb_time_stamp;
    return true;
}

f32 tb_closest_point_callback(f32 minDistanceSquared, i32 proxyId, u64 userData, void* context) {
    ignore proxyId;
    ignore context;

    i32 slot = cast(i32, userData);
    g_tb_proxies[slot].queryTimeStamp = g_tb_time_stamp;
    b3Vec3 queryPoint = g_tb_closest_queries[g_tb_test_index].center;
    b3Vec3 closestPoint = b3ClosestPointToAABB(queryPoint, g_tb_proxies[slot].aabb);
    f32 distanceSquared = b3DistanceSquared(queryPoint, closestPoint);
    if distanceSquared < minDistanceSquared {
        g_tb_closest_point = b3ToPos(closestPoint);
        g_tb_have_closest = true;
    }

    return distanceSquared;
}

void tb_generate() {
    g_randomSeed = 42;

    b3AABB bounds = g_tb_tree.nodes[g_tb_tree.root].aabb;
    b3Vec3 extents = b3AABB_Extents(bounds);
    f32 radius = (extents.x + extents.y + extents.z) / 3.0f;

    for i32 i = 0; i < TB_TEST_COUNT; i += 1 {
        g_tb_rays[i].origin = random_vec3(bounds.lowerBound, bounds.upperBound);

        b3Vec3 end = random_vec3(bounds.lowerBound, bounds.upperBound);
        g_tb_rays[i].translation = b3Sub(end, g_tb_rays[i].origin);

        f32 s = random_float_range(0.01f, 0.2f);
        b3Vec3 c = random_vec3(bounds.lowerBound, bounds.upperBound);
        b3Vec3 p1 = b3Sub(c, b3MulSV(s, extents));
        b3Vec3 p2 = b3Add(c, b3MulSV(s, extents));

        g_tb_overlap_queries[i].lowerBound = b3Min(p1, p2);
        g_tb_overlap_queries[i].upperBound = b3Max(p1, p2);

        g_tb_closest_queries[i] = b3Sphere{c, s * radius};
    }
}

void tb_profile() {
    u64 ticks = b3GetTicks();
    for i32 i = 0; i < TB_TEST_COUNT; i += 1 {
        TreeRay ray = g_tb_rays[i];
        b3RayCastInput input = b3RayCastInput{ray.origin, ray.translation, 1.0f};

        ignore b3DynamicTree_RayCast(&g_tb_tree, &input, ~cast(u64, 0), false,
                                     cast(b3TreeRayCastCallbackFcn, tb_ray_callback), null);
    }
    g_tb_ray_time = b3GetMillisecondsAndReset(&ticks);

    for i32 i = 0; i < TB_TEST_COUNT; i += 1 {
        ignore b3DynamicTree_Query(&g_tb_tree, g_tb_overlap_queries[i], ~cast(u64, 0), false,
                                   cast(b3TreeQueryCallbackFcn, tb_query_callback), null);
    }
    g_tb_overlap_time = b3GetMilliseconds(ticks);

    for i32 i = 0; i < TB_TEST_COUNT; i += 1 {
        b3Vec3 point = g_tb_closest_queries[i].center;
        f32 radius = g_tb_closest_queries[i].radius;
        f32 distanceSquared = radius * radius;
        g_tb_closest_point = b3ToPos(point);
        g_tb_have_closest = false;
        ignore b3DynamicTree_QueryClosest(&g_tb_tree, point, ~cast(u64, 0), false,
                                          cast(b3TreeQueryClosestCallbackFcn, tb_closest_point_callback),
                                          null, &distanceSquared);
    }

    g_tb_closest_time = b3GetMilliseconds(ticks);
}

void build_tree_benchmark() {
    tb_fill_level_colors();

    g_tb_file_names[0] = "bounds01";
    g_tb_file_names[1] = "bounds02";
    g_tb_file_names[2] = "bounds03";
    g_tb_scales[0] = 1.0f;
    g_tb_scales[1] = 1.0f;
    g_tb_scales[2] = 0.01f;
    g_tb_zups[0] = false;
    g_tb_zups[1] = false;
    g_tb_zups[2] = false;
    g_tb_file_index = 0;

    g_tb_closest_point = b3Pos_zero;
    g_tb_time_stamp = 1;
    g_tb_do_overlap = false;
    g_tb_do_closest = false;
    g_tb_do_ray = false;
    g_tb_test_index = 0;
    g_tb_ray_time = 0.0f;
    g_tb_overlap_time = 0.0f;
    g_tb_closest_time = 0.0f;
    g_tb_have_closest = false;
    g_tb_build_time = 0.0f;
    g_tb_area_ratio = 0.0f;
    g_tb_draw_distance = 1.0f;
    g_tb_load_scale = 1.0f;
    g_tb_height = 0;
    g_tb_tree = b3DynamicTree{};

    tb_create_tree();
}

void destroy_tree_benchmark() {
    b3DynamicTree_Destroy(&g_tb_tree);
    g_tb_tree = b3DynamicTree{};
    if g_tb_proxies != null {
        free(cast(void*, g_tb_proxies));
        g_tb_proxies = null;
    }
    if g_tb_depths != null {
        free(cast(void*, g_tb_depths));
        g_tb_depths = null;
    }
    g_tb_proxy_count = 0;
}

bool tree_benchmark_controls() {
    u8[192] buf;
    ignore snprintf(cast(u8*, &buf), 192, "leaves = %d, height = %d, area = %g",
                    g_tb_tree.proxyCount, g_tb_height, g_tb_area_ratio);
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 192, "build time = %g ms", g_tb_build_time);
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 192,
                    "total: ray = %g ms, overlap = %g ms, closest = %g ms",
                    g_tb_ray_time, g_tb_overlap_time, g_tb_closest_time);
    draw_text_line(cast(u8*, &buf));

    f32 s = 1000.0f / cast(f32, TB_TEST_COUNT);
    ignore snprintf(cast(u8*, &buf), 192,
                    "ave: ray = %g us, overlap = %g us, closest = %g us",
                    s * g_tb_ray_time, s * g_tb_overlap_time, s * g_tb_closest_time);
    draw_text_line(cast(u8*, &buf));

    if ImGui_Combo("File", &g_tb_file_index, cast(u8**, &g_tb_file_names), TB_FILE_COUNT, -1) {
        tb_create_tree();
    }

    if ImGui_Button("Top Down", ImVec2{0.0f, 0.0f}) {
        u64 ticks = b3GetTicks();
        b3DynamicTree_Rebuild(&g_tb_tree, true);
        g_tb_build_time = b3GetMilliseconds(ticks);
        g_tb_area_ratio = b3DynamicTree_GetAreaRatio(&g_tb_tree);
        g_tb_height = tb_compute_depths();
    }

    ImGui_Separator();

    ignore ImGui_Checkbox("Ray Cast", &g_tb_do_ray);
    ignore ImGui_Checkbox("Overlap", &g_tb_do_overlap);
    ignore ImGui_Checkbox("Closet Point", &g_tb_do_closest);

    if ImGui_Button("Profile", ImVec2{0.0f, 0.0f}) {
        tb_profile();
    }

    ignore ImGui_SliderInt("Test", &g_tb_test_index, 0, TB_TEST_COUNT - 1, null, 0);
    ignore ImGui_SliderInt("Level", &g_tb_draw_level, -1, g_tb_height, null, 0);
    ignore ImGui_SliderFloat("Kilometers", &g_tb_draw_distance, 0.5f, 20.0f, "%.1f", 0);

    if ImGui_Button("Save", ImVec2{0.0f, 0.0f}) {
        b3DynamicTree_Save(&g_tb_tree, TB_SAVE_FILE_NAME.data);
    }

    if ImGui_Button("Load", ImVec2{0.0f, 0.0f}) {
        tb_load_tree();
    }

    ignore ImGui_SliderFloat("Load Scale", &g_tb_load_scale, 0.01f, 1.0f, "%.3f", 0);

    return true;
}

void tb_render() {
    b3WorldTransform axes;
    axes.p = b3Pos_zero;
    axes.q = b3Quat_identity;
    dbg_axes(axes, 2.0f);

    b3Pos cp = b3Pos_zero;
    b3TreeNode* nodes = g_tb_tree.nodes;
    f32 distSquared = g_tb_draw_distance * g_tb_draw_distance * 1000.0f * 1000.0f;

    if g_tb_draw_level >= 0 {
        i32 capacity = g_tb_tree.nodeCapacity;
        for i32 i = 0; i < capacity; i += 1 {
            b3TreeNode* node = nodes + i;
            if cast(i32, g_tb_depths[i]) != g_tb_draw_level
               || (cast(i32, node.flags) & cast(i32, b3_allocatedNode)) == 0 {
                // skip internal nodes
                continue;
            }

            b3Vec3 c = b3AABB_Center(node.aabb);
            if g_tb_draw_level < 10
               || b3LengthSquared(b3SubPos(cp, b3ToPos(c))) < distSquared {
                adapter_bounds(node.aabb,
                               g_tb_level_colors[g_tb_draw_level % TB_DRAW_COLOR_COUNT], null);
            }
        }
    } else {
        i32 requiredFlags = cast(i32, b3_allocatedNode) | cast(i32, b3_leafNode);
        for i32 i = 0; i < g_tb_tree.nodeCapacity; i += 1 {
            b3TreeNode* node = nodes + i;

            if cast(i32, node.flags) != requiredFlags {
                continue;
            }

            b3Vec3 c = b3AABB_Center(node.aabb);
            if b3LengthSquared(b3SubPos(cp, b3ToPos(c))) > distSquared {
                continue;
            }

            i32 slot = cast(i32, node.userData);
            if g_tb_proxies[slot].queryTimeStamp == g_tb_time_stamp
               || g_tb_proxies[slot].rayTimeStamp == g_tb_time_stamp {
                adapter_bounds(node.aabb, b3_colorLightGray, null);
            } else {
                adapter_bounds(node.aabb, b3_colorLightBlue, null);
            }
        }
    }

    if g_tb_do_ray {
        TreeRay ray = g_tb_rays[g_tb_test_index];
        dbg_line(b3ToPos(ray.origin), b3ToPos(b3Add(ray.origin, ray.translation)), b3_colorRed);
    }

    if g_tb_do_overlap {
        adapter_bounds(g_tb_overlap_queries[g_tb_test_index], b3_colorRed, null);
    }

    if g_tb_do_closest {
        b3WorldTransform identity;
        identity.p = b3Pos_zero;
        identity.q = b3Quat_identity;
        dbg_solid_sphere(identity, g_tb_closest_queries[g_tb_test_index],
                         make_color(b3_colorCyan));
        if g_tb_have_closest {
            adapter_point(g_tb_closest_point, 15.0f, b3_colorOrange, null);
        }
    }
}

void step_tree_benchmark(f32 timeStep) {
    ignore timeStep;

    g_tb_time_stamp += 1;

    if g_tb_do_ray {
        TreeRay ray = g_tb_rays[g_tb_test_index];
        b3RayCastInput input = b3RayCastInput{ray.origin, ray.translation, 1.0f};

        ignore b3DynamicTree_RayCast(&g_tb_tree, &input, ~cast(u64, 0), false,
                                     cast(b3TreeRayCastCallbackFcn, tb_ray_callback), null);
    }

    if g_tb_do_overlap {
        ignore b3DynamicTree_Query(&g_tb_tree, g_tb_overlap_queries[g_tb_test_index],
                                   ~cast(u64, 0), false,
                                   cast(b3TreeQueryCallbackFcn, tb_query_callback), null);
    }

    if g_tb_do_closest {
        b3Vec3 point = g_tb_closest_queries[g_tb_test_index].center;
        f32 radius = g_tb_closest_queries[g_tb_test_index].radius;
        f32 distanceSquared = radius * radius;
        g_tb_closest_point = b3ToPos(point);
        g_tb_have_closest = false;
        ignore b3DynamicTree_QueryClosest(&g_tb_tree, point, ~cast(u64, 0), false,
                                          cast(b3TreeQueryClosestCallbackFcn, tb_closest_point_callback),
                                          null, &distanceSquared);
    }

    tb_render();
}
