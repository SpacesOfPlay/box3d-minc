// Stacking scenes.

// --- samples (setups verbatim from upstream) ------------------------

// samples/sample_stacking.cpp SingleBox

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
import sample_robustness;

void build_single_box() {
    ignore add_ground_box(20.0f);
    b3BoxHull cube = b3MakeCubeHull(0.5f);
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = b3Pos{0.0f, 0.5f, 0.0f};
    b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
    g_single_box_body = bodyId;
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    ignore b3CreateHullShape(bodyId, &shapeDef, &cube.base);
}

// samples/sample_stacking.cpp BoxStack
void build_box_stack() {
    ignore add_ground_box(40.0f);
    f32 a = 0.5f;
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    b3BoxHull cube = b3MakeBoxHull(a, a, a);
    for i32 i = 0; i < 40; i++ {
        bodyDef.position = b3Pos{0.0f, 1.5f * a + 2.5f * a * cast(f32, i), 0.0f};
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        shapeDef.baseMaterial.rollingResistance = 0.1f;
        ignore b3CreateHullShape(bodyId, &shapeDef, &cube.base);
    }
}

// samples/sample_stacking.cpp SphereStack
void build_sphere_stack() {
    ignore add_ground_box(15.0f);
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    f32 r = 0.5f;
    b3Sphere sphere;
    sphere.center = b3Vec3{0.0f, 0.0f, 0.0f};
    sphere.radius = r;
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.baseMaterial.rollingResistance = 0.1f;
    f32 y = 1.5f * r;
    for i32 i = 0; i < 30; i++ {
        bodyDef.position = b3Pos{0.0f, y, 0.0f};
        bodyDef.angularVelocity = b3Vec3{0.0f, 0.0f, 0.0f};
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateSphereShape(bodyId, &shapeDef, &sphere);
        y += 3.0f * r;
    }
}

// samples/sample_stacking.cpp JengaStack (hull arm)
void build_jenga_stack() {
    // the radio keeps its choice across the rebuild; a fresh load
    // starts on hulls
    if !g_jenga_keep { g_jenga_capsule = false; }
    g_jenga_keep = false;
    ignore add_ground_box(20.0f);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.baseMaterial.rollingResistance = g_jenga_capsule ? 0.1f : 0.05f;
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    f32 h = 1.0f;
    f32 r = 0.1f;
    b3BoxHull box = b3MakeBoxHull(h, r, r);
    b3Capsule capsule;
    capsule.center1 = b3Vec3{0.0f - h, 0.0f, 0.0f};
    capsule.center2 = b3Vec3{h, 0.0f, 0.0f};
    capsule.radius = r;
    i32 count = 30;
    for i32 i = 0; i < count; i++ {
        f32 alpha = (i & 1) == 1 ? 0.0f : 0.5f * PI_F;
        f32 x = (i & 1) == 0 ? h - 2.0f * r : 0.0f;
        f32 z = (i & 1) == 0 ? 0.0f : h - 2.0f * r;
        f32 y = (2.1f * cast(f32, i) + 0.5f) * r;
        bodyDef.position = b3Pos{x, y, z};
        bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisY, alpha);
        b3BodyId body1 = b3CreateBody(g_world, &bodyDef);
        bodyDef.position = b3Pos{-x, y, -z};
        bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisY, alpha);
        b3BodyId body2 = b3CreateBody(g_world, &bodyDef);
        if g_jenga_capsule {
            ignore b3CreateCapsuleShape(body1, &shapeDef, &capsule);
            ignore b3CreateCapsuleShape(body2, &shapeDef, &capsule);
        } else {
            ignore b3CreateHullShape(body1, &shapeDef, &box.base);
            ignore b3CreateHullShape(body2, &shapeDef, &box.base);
        }
    }
}

// samples/sample_stacking.cpp CardHouse
void build_card_house() {
    ignore add_ground_box(10.0f);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.baseMaterial.rollingResistance = 0.05f;
    shapeDef.baseMaterial.friction = 0.7f;
    f32 cardHeight = 0.2f;
    f32 cardThickness = 0.001f;
    f32 cardDepth = 0.1f;
    f32 angle0 = 25.0f * PI_F / 180.0f;
    f32 angle1 = -25.0f * PI_F / 180.0f;
    f32 angle2 = 0.5f * PI_F;
    b3BoxHull cardBox = b3MakeBoxHull(cardThickness, cardHeight, cardDepth);
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    i32 Nb = 5;
    f32 z0 = 0.0f;
    f32 y = cardHeight - 0.02f;
    while Nb != 0 {
        f32 z = z0;
        for i32 i = 0; i < Nb; i++ {
            if i != Nb - 1 {
                bodyDef.position = b3Pos{z + 0.25f, y + cardHeight - 0.015f, 0.0f};
                bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, angle2);
                b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
                ignore b3CreateHullShape(bodyId, &shapeDef, &cardBox.base);
            }
            bodyDef.position = b3Pos{z, y, 0.0f};
            bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, angle1);
            b3BodyId body1 = b3CreateBody(g_world, &bodyDef);
            ignore b3CreateHullShape(body1, &shapeDef, &cardBox.base);
            z += 0.175f;
            bodyDef.position = b3Pos{z, y, 0.0f};
            bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, angle0);
            b3BodyId body2 = b3CreateBody(g_world, &bodyDef);
            ignore b3CreateHullShape(body2, &shapeDef, &cardBox.base);
            z += 0.175f;
        }
        y += cardHeight * 2.0f - 0.03f;
        z0 += 0.175f;
        Nb--;
    }
}

// samples/sample_stacking.cpp Dominoes
void dominoes_ring(f32 radius, b3BoxHull* box) {
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    f32 alpha = 0.0f;
    while alpha <= 360.0f {
        b3CosSin cs = b3ComputeCosSin(PI_F / 180.0f * alpha);
        b3Pos position = b3Pos{radius * cs.cosine - alpha / 630.0f * cs.cosine,
                               0.8f,
                               radius * cs.sine - alpha / 630.0f * cs.sine};
        b3Quat orientation = b3MakeQuatFromAxisAngle(b3Vec3_axisY, -(PI_F / 180.0f) * alpha);
        bodyDef.position = position;
        bodyDef.rotation = orientation;
        b3BodyId body = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateHullShape(body, &shapeDef, &box.base);
        if alpha == 0.0f {
            b3Body_ApplyLinearImpulse(body, b3Vec3{0.0f, 0.0f, 25.0f},
                b3Pos{position.x, position.y + 0.8f, position.z}, true);
        }
        alpha += 2.0f;
    }
}

void build_dominoes() {
    ignore add_ground_box(80.0f);
    b3BoxHull box = b3MakeBoxHull(0.2f, 0.8f, 0.05f);
    for i32 ring = 0; ring < 30; ring++ {
        f32 radius = 7.0f + 1.1f * cast(f32, ring);
        dominoes_ring(radius, &box);
    }
}

// samples/sample_stacking.cpp DoubleDomino
void build_double_domino() {
    ignore add_ground_box(20.0f);
    b3BoxHull box = b3MakeBoxHull(0.125f, 0.5f, 0.25f);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.baseMaterial.friction = 0.6f;
    shapeDef.density = 4.0f;
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    i32 count = 15;
    f32 x = -0.5f * cast(f32, count);
    for i32 i = 0; i < count; i++ {
        bodyDef.position = b3Pos{x, 0.5f, 0.0f};
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateHullShape(bodyId, &shapeDef, &box.base);
        if i == 0 {
            b3Body_ApplyLinearImpulse(bodyId, b3Vec3{0.2f, 0.0f, 0.0f},
                b3Pos{x, 1.0f, 0.0f}, true);
        }
        x += 1.01f;
    }
}

// samples/sample_stacking.cpp Pyramid2D
void build_pyramid_2d() {
    ignore add_ground_box(40.0f);
    f32 a = 1.0f;
    b3BoxHull box = b3MakeBoxHull(a, a, a);
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.motionLocks.linearZ = true;
    bodyDef.motionLocks.angularX = true;
    bodyDef.motionLocks.angularY = true;
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    i32 size = 12;
    for i32 row = 0; row < size; row++ {
        for i32 column = 0; column < size - row; column++ {
            bodyDef.position = b3Pos{
                (-10.0f + 2.0f * cast(f32, column) + cast(f32, row)) * a,
                (1.5f + 2.5f * cast(f32, row)) * a, 0.0f};
            b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
            ignore b3CreateHullShape(bodyId, &shapeDef, &box.base);
        }
    }
}

// shared/benchmarks.c CreateLargePyramid (upstream BenchmarkLargePyramid)

// upstream SingleBox::Step / TinyPyramid::Step text readouts
// (upstream formats with %g; minc's formatter has no %g, so %f)
void step_single_box(f32 timeStep) {
    b3Pos position = b3Body_GetPosition(g_single_box_body);
    u8[128] buf;
    ignore snprintf(cast(u8*, &buf), 128, "(x, y, z) = (%.2f, %.2f, %.2f)",
                    position.x, position.y, position.z);
    draw_text_line(cast(u8*, &buf));
}

// per-sample DrawControls (upstream returns true when the sample drew
// widgets so the panel adds a separator)

// upstream JengaStack::DrawControls: the shape-type radio pair, each
// rebuilding the stack.
bool jenga_controls() {
    if ImGui_RadioButton("Capsule", g_jenga_capsule) {
        g_jenga_capsule = true;
        g_jenga_keep = true;
        g_reset_pending = true;
    }
    if ImGui_RadioButton("Hull", !g_jenga_capsule) {
        g_jenga_capsule = false;
        g_jenga_keep = true;
        g_reset_pending = true;
    }
    return true;
}

// sample_stacking.cpp CapsuleStack: 20 capsules stacked flat, with
// out-of-plane motion locked.
void build_capsule_stack() {
    ignore add_ground_box(40.0f);
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.motionLocks.linearZ = true;
    bodyDef.motionLocks.angularX = true;
    bodyDef.motionLocks.angularY = true;
    bodyDef.motionLocks.angularZ = true;
    f32 r = 0.5f;
    b3Capsule capsule;
    capsule.center1 = b3Vec3{-1.0f, 0.0f, 0.0f};
    capsule.center2 = b3Vec3{1.0f, 0.0f, 0.0f};
    capsule.radius = r;
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    f32 y = 1.5f * r;
    for i32 i = 0; i < 20; i++ {
        bodyDef.position = b3Pos{0.0f, y, 0.0f};
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateCapsuleShape(bodyId, &shapeDef, &capsule);
        y += 2.0f * r;
    }
}

// samples/sample_stacking.cpp EdgeCrossing
void build_edge_crossing() {
    ignore add_ground_box(40.0f);

    b3Vec3 h = b3Vec3{0.2f, 0.02f, 0.04f};
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    b3ShapeDef shapeDef = b3DefaultShapeDef();

    // same extents, one with y and z swapped
    b3BoxHull box1 = b3MakeBoxHull(h.x, h.y, h.z);
    b3BoxHull box2 = b3MakeBoxHull(h.x, h.z, h.y);

    b3Vec3 axis = b3Normalize(b3Vec3{0.1f, 0.9f, 0.0f});

    for i32 row = 0; row < 3; row++ {
        // Upstream builds a box hull per row from per-row half-extents;
        // both rows only ever ask for one of two shapes, so the hulls
        // are hoisted above the loop and selected here. `y` is the
        // lower box's half-height — its resting height on the ground.
        b3BoxHull* lower = row == 1 ? &box2 : &box1;
        b3BoxHull* upper = row == 0 ? &box1 : &box2;
        f32 y = row == 1 ? h.z : h.y;

        bodyDef.position.x = -10.0f;
        bodyDef.position.z = -2.0f + 2.0f * cast(f32, row);

        f32 angle = 0.0f - PI_F;
        while angle < PI_F + 0.001f {
            bodyDef.position.y = y;
            bodyDef.rotation = b3Quat_identity;
            b3BodyId bodyId1 = b3CreateBody(g_world, &bodyDef);
            ignore b3CreateHullShape(bodyId1, &shapeDef, &lower.base);

            bodyDef.position.y = 20.0f * y;
            bodyDef.rotation = b3MakeQuatFromAxisAngle(axis, angle);
            b3BodyId bodyId2 = b3CreateBody(g_world, &bodyDef);
            ignore b3CreateHullShape(bodyId2, &shapeDef, &upper.base);

            bodyDef.position.x += 1.0f;
            angle += 0.1f * PI_F;
        }
    }
}

// samples/sample_stacking.cpp Cylinder
void build_cylinder() {
    ignore add_ground_box(10.0f);

    b3HullData* hull = b3CreateCylinder(1.0f, 0.25f, 0.0f, 12);
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = b3Pos{0.0f, 2.0f, 0.0f};
    bodyDef.linearVelocity = b3Vec3{0.0f, 0.0f, 0.0f};
    // upstream leaves an angular velocity commented out here
    b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.baseMaterial.rollingResistance = 0.05f;
    ignore b3CreateHullShape(bodyId, &shapeDef, hull);
    b3DestroyHull(hull);
}

// samples/sample_stacking.cpp CylinderStack
void build_cylinder_stack() {
    ignore add_ground_box(10.0f);

    b3HullData* hull = b3CreateCylinder(1.0f, 0.5f, 0.0f, 15);
    b3Vec3[4] scales;
    scales[0] = b3Vec3{1.0f, 1.0f, 1.0f};
    scales[1] = b3Vec3{-0.75f, 1.0f, 1.0f};
    scales[2] = b3Vec3{1.2f, 1.0f, -0.9f};
    scales[3] = b3Vec3{0.9f, 0.9f, 0.9f};

    for i32 i = 0; i < 10; i++ {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;
        bodyDef.position = b3Pos{0.0f, 0.0f + 1.1f * cast(f32, i), 0.0f};
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        // upstream leaves rolling resistance commented out here
        ignore b3CreateTransformedHullShape(bodyId, &shapeDef, hull,
                                            b3Transform_identity, scales[i % 4]);
    }
    b3DestroyHull(hull);
}

// samples/sample_stacking.cpp Wedge
void build_wedge() {
    ignore add_ground_box(20.0f);

    b3Vec3[6] vertices;
    vertices[0] = b3Vec3{-1.0f, 1.0f, -0.1f};
    vertices[1] = b3Vec3{1.0f, 1.0f, -0.1f};
    vertices[2] = b3Vec3{-1.0f, 1.0f, 0.1f};
    vertices[3] = b3Vec3{1.0f, 1.0f, 0.1f};
    vertices[4] = b3Vec3{-0.5f, 0.5f, 0.0f};
    vertices[5] = b3Vec3{0.5f, 0.5f, 0.0f};

    b3HullData* wedgeHull = b3CreateHull(&vertices[0], 6, 6);
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = b3Pos{0.0f, 1.0f, 0.0f};
    b3BodyId wedgeBody = b3CreateBody(g_world, &bodyDef);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    ignore b3CreateHullShape(wedgeBody, &shapeDef, wedgeHull);
    b3DestroyHull(wedgeHull);
}

// samples/sample_stacking.cpp Arch
//
// Arc coordinates are upstream's, at full precision.
const f32 ARCH_SCALE = 0.25f;
const f32 ARCH_HALF_DEPTH = 0.5f;

f32[9] g_arch_x1;
f32[9] g_arch_y1;
f32[9] g_arch_x2;
f32[9] g_arch_y2;

void arch_init_points() {
    g_arch_x1[0..] = { 16.0f, 14.93803712795643f, 13.79871746027416f,
                       12.56252963284711f, 11.20040987372525f, 9.66521217819836f,
                       7.87179930638133f, 5.635199558196225f, 2.405937953536585f };
    g_arch_y1[0..] = { 0.0f, 5.133601056842984f, 10.24928069555078f,
                       15.34107019122473f, 20.39856541571217f, 25.40369899225096f,
                       30.3179337000085f, 35.03820717801641f, 39.09554102558315f };
    g_arch_x2[0..] = { 24.0f, 22.33619528222415f, 20.54936888969905f,
                       18.60854610798073f, 16.46769273811807f, 14.05325025774858f,
                       11.23551045834022f, 7.752568160730571f, 3.016931552701656f };
    g_arch_y2[0..] = { 0.0f, 6.02299846205841f, 12.00964361211476f,
                       17.9470321677465f, 23.81367936585418f, 29.57079353071012f,
                       35.13775818285372f, 40.30450679009583f, 44.28891593799322f };
    for i32 i = 0; i < 9; i++ {
        g_arch_x1[i] *= ARCH_SCALE;
        g_arch_y1[i] *= ARCH_SCALE;
        g_arch_x2[i] *= ARCH_SCALE;
        g_arch_y2[i] *= ARCH_SCALE;
    }
}

// One voussoir: the quad between arc samples i and i+1, swept in z.
// side +1 is the right half, -1 the mirrored left half, which also
// swaps the inner and outer arcs.
void arch_segment(i32 i, f32 side, b3ShapeDef* shapeDef, b3BodyDef* bodyDef) {
    f32 h = ARCH_HALF_DEPTH;
    b3Vec3[8] ps;
    if side > 0.0f {
        ps[0] = b3Vec3{g_arch_x1[i], g_arch_y1[i], 0.0f - h};
        ps[1] = b3Vec3{g_arch_x2[i], g_arch_y2[i], 0.0f - h};
        ps[2] = b3Vec3{g_arch_x2[i + 1], g_arch_y2[i + 1], 0.0f - h};
        ps[3] = b3Vec3{g_arch_x1[i + 1], g_arch_y1[i + 1], 0.0f - h};
        ps[4] = b3Vec3{g_arch_x1[i], g_arch_y1[i], h};
        ps[5] = b3Vec3{g_arch_x2[i], g_arch_y2[i], h};
        ps[6] = b3Vec3{g_arch_x2[i + 1], g_arch_y2[i + 1], h};
        ps[7] = b3Vec3{g_arch_x1[i + 1], g_arch_y1[i + 1], h};
    } else {
        ps[0] = b3Vec3{0.0f - g_arch_x2[i], g_arch_y2[i], 0.0f - h};
        ps[1] = b3Vec3{0.0f - g_arch_x1[i], g_arch_y1[i], 0.0f - h};
        ps[2] = b3Vec3{0.0f - g_arch_x1[i + 1], g_arch_y1[i + 1], 0.0f - h};
        ps[3] = b3Vec3{0.0f - g_arch_x2[i + 1], g_arch_y2[i + 1], 0.0f - h};
        ps[4] = b3Vec3{0.0f - g_arch_x2[i], g_arch_y2[i], h};
        ps[5] = b3Vec3{0.0f - g_arch_x1[i], g_arch_y1[i], h};
        ps[6] = b3Vec3{0.0f - g_arch_x1[i + 1], g_arch_y1[i + 1], h};
        ps[7] = b3Vec3{0.0f - g_arch_x2[i + 1], g_arch_y2[i + 1], h};
    }
    b3BodyId bodyId = b3CreateBody(g_world, bodyDef);
    b3HullData* hull = b3CreateHull(&ps[0], 8, 8);
    ignore b3CreateHullShape(bodyId, shapeDef, hull);
    b3DestroyHull(hull);
}

void build_arch() {
    ignore add_ground_box(40.0f);
    arch_init_points();

    f32 h = ARCH_HALF_DEPTH;
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.density = 200.0f;
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;

    for i32 i = 0; i < 8; i++ { arch_segment(i, 1.0f, &shapeDef, &bodyDef); }
    for i32 i = 0; i < 8; i++ { arch_segment(i, -1.0f, &shapeDef, &bodyDef); }

    // keystone
    b3Vec3[8] ks;
    ks[0] = b3Vec3{g_arch_x1[8], g_arch_y1[8], 0.0f - h};
    ks[1] = b3Vec3{g_arch_x2[8], g_arch_y2[8], 0.0f - h};
    ks[2] = b3Vec3{0.0f - g_arch_x2[8], g_arch_y2[8], 0.0f - h};
    ks[3] = b3Vec3{0.0f - g_arch_x1[8], g_arch_y1[8], 0.0f - h};
    ks[4] = b3Vec3{g_arch_x1[8], g_arch_y1[8], h};
    ks[5] = b3Vec3{g_arch_x2[8], g_arch_y2[8], h};
    ks[6] = b3Vec3{0.0f - g_arch_x2[8], g_arch_y2[8], h};
    ks[7] = b3Vec3{0.0f - g_arch_x1[8], g_arch_y1[8], h};
    b3BodyId keyId = b3CreateBody(g_world, &bodyDef);
    b3HullData* keyHull = b3CreateHull(&ks[0], 8, 8);
    ignore b3CreateHullShape(keyId, &shapeDef, keyHull);
    b3DestroyHull(keyHull);

    // load on top
    for i32 i = 0; i < 4; i++ {
        b3BoxHull box = b3MakeBoxHull(2.0f, 0.5f, h);
        bodyDef.position = b3Pos{0.0f, 0.5f + g_arch_y2[8] + 1.0f * cast(f32, i), 0.0f};
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateHullShape(bodyId, &shapeDef, &box.base);
    }
}
