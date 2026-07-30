// World scenes. Ports of samples/sample_world.cpp.

import box3d;
import box3d_human;
import sokol_all;
import sokol_imgui;
import math;
import linear;
import str;
import camera;
import gui;
import renderer;
import debug_adapter;
import sample;
import sample_benchmark;
import sample_bodies;
import sample_continuous;
import sample_robustness;
import sample_stacking;
import sample_joint;
import sample_compound;
import sample_shapes;

// samples/sample_world.cpp FarStack
const f32 FS_MAX_OFFSET = 10000.0f;
f32 g_fs_offset_km;
i32 g_fs_column_count = 6;
b3BodyId g_fs_top_body;
b3Pos g_fs_base;

// Place the ground and stack at the current offset and aim the camera at them. The draw origin
// rides the camera eye, so the float renderer works in a small relative frame near the content
// no matter how large the offset.
void fs_build_scene() {
    b3Pos base = b3Pos{1000.0f * g_fs_offset_km, 0.0f, 0.0f};
    g_fs_base = base;
    cam_pivot = float3{base.x, base.y + 2.0f, base.z};
    cam_rebuild_basis();

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.position = b3OffsetPos(base, b3Vec3{0.0f, -1.0f, 0.0f});
    b3BodyId groundId = b3CreateBody(g_world, &bodyDef);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    b3BoxHull groundHull = b3MakeBoxHull(12.0f, 1.0f, 12.0f);
    b3ShapeId groundShapeId = b3CreateHullShape(groundId, &shapeDef, &groundHull.base);
    set_ground_shape(groundShapeId);

    b3BoxHull box = b3MakeBoxHull(0.5f, 0.5f, 0.5f);
    b3BodyDef boxDef = b3DefaultBodyDef();
    boxDef.type = b3_dynamicBody;
    b3ShapeDef boxShape = b3DefaultShapeDef();
    for i32 i = 0; i < g_fs_column_count; i++ {
        // A small alternating skew so a float build visibly drifts rather than balancing by luck.
        f32 skew = 0.02f * ((i & 1) != 0 ? 1.0f : -1.0f);
        boxDef.position = b3OffsetPos(base, b3Vec3{skew, 0.5f + 1.0f * cast(f32, i), 0.0f});
        b3BodyId body = b3CreateBody(g_world, &boxDef);
        ignore b3CreateHullShape(body, &boxShape, &box.base);
        g_fs_top_body = body;
    }
}

void build_far_stack() {
    // Double precision opens at the dramatic offset, float opens at the origin so it is usable
    // out of the box. Either way the slider sweeps the full range.
    if !g_fs_keep {
        g_fs_offset_km = b3IsDoublePrecision() ? FS_MAX_OFFSET : 0.0f;
    }
    g_fs_keep = false;
    g_fs_column_count = 6;
    fs_build_scene();
}

void step_far_stack(f32 timeStep) {
    ignore timeStep;
    // Height of the top box above the ground, measured in the offset's own frame. This holds
    // steady at any offset under double precision and drifts once float runs out of resolution.
    b3Vec3 top = b3SubPos(b3Body_GetWorldCenter(g_fs_top_body), g_fs_base);
    u8[160] buf;
    ignore snprintf(cast(u8*, &buf), 160, "double precision: %s",
                    b3IsDoublePrecision() ? cast(u8*, "ON") : cast(u8*, "OFF"));
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 160, "world offset: %.1f km", g_fs_offset_km);
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 160, "top box height above ground: %.4f m", top.y);
    draw_text_line(cast(u8*, &buf));
}

bool far_stack_controls() {
    f32[5] presets = { 0.0f, 10.0f, 100.0f, 1000.0f, 10000.0f };
    if ImGui_Button("origin", ImVec2{0.0f, 0.0f}) { g_fs_offset_km = presets[0]; g_reset_pending = true; g_fs_keep = true; }
    if ImGui_Button("10km", ImVec2{0.0f, 0.0f}) { g_fs_offset_km = presets[1]; g_reset_pending = true; g_fs_keep = true; }
    if ImGui_Button("100km", ImVec2{0.0f, 0.0f}) { g_fs_offset_km = presets[2]; g_reset_pending = true; g_fs_keep = true; }
    if ImGui_Button("1000km", ImVec2{0.0f, 0.0f}) { g_fs_offset_km = presets[3]; g_reset_pending = true; g_fs_keep = true; }
    if ImGui_Button("10000km", ImVec2{0.0f, 0.0f}) { g_fs_offset_km = presets[4]; g_reset_pending = true; g_fs_keep = true; }
    return true;
}

// samples/sample_world.cpp FarPyramid
const f32 FP_OFFSET_KM = 10000.0f;

void build_far_pyramid() {
    b3Pos base = b3Pos{1000.0f * FP_OFFSET_KM, 0.0f, 0.0f};
    i32 baseCount = 40;
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.position = b3OffsetPos(base, b3Vec3{0.0f, -1.0f, 0.0f});
    b3BodyId groundId = b3CreateBody(g_world, &bodyDef);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    b3BoxHull groundHull = b3MakeBoxHull(400.0f, 1.0f, 400.0f);
    b3ShapeId groundShapeId = b3CreateHullShape(groundId, &shapeDef, &groundHull.base);
    set_ground_shape(groundShapeId);

    f32 h = 0.5f;
    f32 shift = h;
    b3BoxHull box = b3MakeBoxHull(h, h, h);
    shapeDef.density = 100.0f;
    bodyDef.type = b3_dynamicBody;
    for i32 i = 0; i < baseCount; i++ {
        f32 y = (2.0f * cast(f32, i) + 1.0f) * shift;
        for i32 j = i; j < baseCount; j++ {
            f32 x = (cast(f32, i) + 1.0f) * shift + 2.0f * (cast(f32, j) - cast(f32, i)) * shift
                  - h * cast(f32, baseCount);
            bodyDef.position = b3OffsetPos(base, b3Vec3{x, y, 0.0f});
            b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
            ignore b3CreateHullShape(bodyId, &shapeDef, &box.base);
        }
    }
}

// samples/sample_world.cpp FarRagdolls
const i32 FR_COUNT = 20;
const f32 FR_OFFSET_KILOMETERS = 1000.0f;
b3MeshData* g_fr_ground_mesh;
Human[FR_COUNT] g_fr_humans;

void build_far_ragdolls() {
    b3Pos base = b3Pos{1000.0f * FR_OFFSET_KILOMETERS, 0.0f, 0.0f};

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.position = b3OffsetPos(base, b3Vec3{0.0f, -1.0f, 0.0f});
    b3BodyId groundId = b3CreateBody(g_world, &bodyDef);

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    g_fr_ground_mesh = b3CreateGridMesh(20, 20, 1.0f, 1, true);
    ignore b3CreateMeshShape(groundId, &shapeDef, g_fr_ground_mesh, b3Vec3_one);

    for i32 i = 0; i < FR_COUNT; i += 1 {
        g_fr_humans[i] = Human{};
        b3Vec3 offset = b3Vec3{
            0.15f * (cast(f32, i) - 0.5f * cast(f32, FR_COUNT)),
            2.0f + 0.25f * cast(f32, i),
            0.15f * (0.5f * cast(f32, FR_COUNT) - cast(f32, i))};
        b3Pos position = b3OffsetPos(base, offset);
        f32 torque = 10.0f;
        f32 hertz = 0.5f;
        f32 damping = 0.7f;
        CreateHuman(&g_fr_humans[i], g_world, position, torque, hertz, damping, i, null, false);
    }
}

void destroy_far_ragdolls() {
    b3DestroyMesh(g_fr_ground_mesh);
}

void step_far_ragdolls(f32 timeStep) {
    ignore timeStep;
    u8[128] buf;
    ignore snprintf(cast(u8*, &buf), 128, "double precision: %s",
                    b3IsDoublePrecision() ? "ON" : "OFF");
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 128,
                    "%d ragdolls piled %.0f km from the world origin", FR_COUNT,
                    cast(f64, FR_OFFSET_KILOMETERS));
    draw_text_line(cast(u8*, &buf));
}
