// Robustness scenes: small shapes, high mass ratios, deep overlap.

import box3d;
import sokol_all;
import sokol_imgui;
import math;
import linear;
import str;
import camera;
import gui;
import renderer;
import sample;
import sample_benchmark;
import sample_bodies;
import sample_continuous;
import sample_stacking;
import sample_joint;
import sample_compound;
import sample_world;

void build_tiny_pyramid() {
    ignore add_ground_box(20.0f);
    f32 extent = 0.025f;
    i32 baseCount = 30;
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    b3BoxHull box = b3MakeBoxHull(extent, extent, extent);
    for i32 i = 0; i < baseCount; i++ {
        f32 y = (2.0f * cast(f32, i) + 1.0f) * extent;
        for i32 j = i; j < baseCount; j++ {
            f32 x = (cast(f32, i) + 1.0f) * extent
                + 2.0f * cast(f32, j - i) * extent - cast(f32, baseCount) * extent;
            bodyDef.position = b3Pos{x, y, 0.0f};
            b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
            ignore b3CreateHullShape(bodyId, &shapeDef, &box.base);
        }
    }
}

// samples/sample_robustness.cpp HighMassRatio1
void build_high_mass_ratio() {
    ignore add_ground_box(50.0f);
    f32 extent = 1.0f;
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    b3BoxHull box = b3MakeBoxHull(extent, extent, extent);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    for i32 j = 0; j < 3; j++ {
        i32 count = 10;
        f32 offset = -20.0f * extent + 2.0f * (cast(f32, count) + 1.0f) * extent * cast(f32, j);
        f32 y = extent;
        while count > 0 {
            for i32 i = 0; i < count; i++ {
                f32 coeff = cast(f32, i) - 0.5f * cast(f32, count);
                f32 yy = count == 1 ? y + 2.0f : y;
                bodyDef.position = b3Pos{2.0f * coeff * extent + offset, yy, 0.0f};
                b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
                shapeDef.density = count == 1 ? (cast(f32, j) + 1.0f) * 100.0f : 1.0f;
                ignore b3CreateHullShape(bodyId, &shapeDef, &box.base);
            }
            count--;
            y += 2.0f * extent;
        }
    }
}

// samples/sample_robustness.cpp OverlapRecovery (upstream defaults)
void build_overlap_recovery() {
    if !g_or_keep {
        g_or_base_count = 4;
        g_or_overlap = 0.25f;
        g_or_extent = 0.5f;
        g_or_speed = 3.0f;
        g_or_hertz = 30.0f;
        g_or_damping = 10.0f;
    }
    g_or_keep = false;
    ignore add_ground_box(20.0f);
    b3World_SetContactTuning(g_world, g_or_hertz, g_or_damping, g_or_speed);
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    b3BoxHull box = b3MakeBoxHull(g_or_extent, g_or_extent, g_or_extent);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.density = 1.0f;
    f32 fraction = 1.0f - g_or_overlap;
    f32 y = g_or_extent;
    for i32 i = 0; i < g_or_base_count; i++ {
        f32 x = fraction * g_or_extent * cast(f32, i - g_or_base_count);
        for i32 j = i; j < g_or_base_count; j++ {
            bodyDef.position = b3Pos{x, y, 0.0f};
            b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
            ignore b3CreateHullShape(bodyId, &shapeDef, &box.base);
            x += 2.0f * fraction * g_or_extent;
        }
        y += 2.0f * fraction * g_or_extent;
    }
}

// samples/sample_bodies.cpp Kinematic (body + per-step target drive)

void step_tiny_pyramid(f32 timeStep) {
    // upstream: "%.1fcm boxes", 200 * extent with extent = 0.025
    draw_text_line("5.0cm boxes");
}

bool or_controls() {
    ImGui_PushItemWidth(6.0f * ImGui_GetFontSize());
    bool changed = false;
    changed = ImGui_SliderFloat("Extent", &g_or_extent, 0.1f, 1.0f, "%.1f", 0) || changed;
    changed = ImGui_SliderInt("Base Count", &g_or_base_count, 1, 10, null, 0) || changed;
    changed = ImGui_SliderFloat("Overlap", &g_or_overlap, 0.0f, 1.0f, "%.2f", 0) || changed;
    changed = ImGui_SliderFloat("Speed", &g_or_speed, 0.0f, 10.0f, "%.1f", 0) || changed;
    changed = ImGui_SliderFloat("Hertz", &g_or_hertz, 0.0f, 240.0f, "%.f", 0) || changed;
    changed = ImGui_SliderFloat("Damping Ratio", &g_or_damping, 0.0f, 20.0f, "%.1f", 0) || changed;
    changed = ImGui_Button("Reset Scene", ImVec2{0.0f, 0.0f}) || changed;
    if changed {
        g_or_keep = true;
        g_reset_pending = true;
    }
    ImGui_PopItemWidth();
    return true;
}

// samples/sample_robustness.cpp OverflowColorPile, scene from
// shared/overflow_color.c CreateOverflowColorPile.
const i32 OVERFLOW_PILE_RING_COUNT = 5;
const i32 OVERFLOW_PILE_PER_RING = 5;

void build_overflow_color_pile() {
    // upstream's helper builds its own ground
    b3BodyDef groundDef = b3DefaultBodyDef();
    groundDef.position = b3Pos{0.0f, -1.0f, 0.0f};
    b3BodyId groundId = b3CreateBody(g_world, &groundDef);
    b3BoxHull groundBox = b3MakeBoxHull(20.0f, 1.0f, 20.0f);
    b3ShapeDef groundShape = b3DefaultShapeDef();
    b3ShapeId groundShapeId = b3CreateHullShape(groundId, &groundShape, &groundBox.base);
    set_ground_shape(groundShapeId);

    // tall so neighbours ring it in layers, heavy so it holds position
    f32 hubHalfX = 0.5f;
    f32 hubHalfY = 2.5f;
    f32 hubHalfZ = 0.5f;
    b3BodyDef hubDef = b3DefaultBodyDef();
    hubDef.type = b3_dynamicBody;
    hubDef.position = b3Pos{0.0f, hubHalfY, 0.0f};
    b3BodyId hubId = b3CreateBody(g_world, &hubDef);
    b3BoxHull hubBox = b3MakeBoxHull(hubHalfX, hubHalfY, hubHalfZ);
    b3ShapeDef hubShape = b3DefaultShapeDef();
    hubShape.density = 50.0f;
    ignore b3CreateHullShape(hubId, &hubShape, &hubBox.base);

    f32 neighborHalf = 0.2f;
    f32 ringRadius = hubHalfX + neighborHalf - 0.03f;
    b3BoxHull neighborBox = b3MakeBoxHull(neighborHalf, neighborHalf, neighborHalf);
    b3ShapeDef neighborShape = b3DefaultShapeDef();
    f32 ringSpacing = 0.5f;
    f32 baseY = neighborHalf + 0.05f;
    for i32 ring = 0; ring < OVERFLOW_PILE_RING_COUNT; ring++ {
        f32 y = baseY + ringSpacing * cast(f32, ring);
        // alternate rings sit half a slot round
        f32 thetaOffset = 0.0f;
        if (ring & 1) != 0 { thetaOffset = PI_F / cast(f32, OVERFLOW_PILE_PER_RING); }
        for i32 slot = 0; slot < OVERFLOW_PILE_PER_RING; slot++ {
            f32 theta = thetaOffset
                      + (2.0f * PI_F * cast(f32, slot)) / cast(f32, OVERFLOW_PILE_PER_RING);
            b3BodyDef bodyDef = b3DefaultBodyDef();
            bodyDef.type = b3_dynamicBody;
            bodyDef.position = b3Pos{ringRadius * cosf(theta), y, ringRadius * sinf(theta)};
            b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
            ignore b3CreateHullShape(bodyId, &neighborShape, &neighborBox.base);
        }
    }
}
