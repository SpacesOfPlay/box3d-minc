// Shape-material scenes: restitution, friction.

import box3d;
import sokol_all;
import sokol_imgui;
import math;
import linear;
import str;
import camera;
import gui;
import mesh_loader;
import renderer;
import sample;
import debug_adapter;
import sample_issues;

// sample_shapes.cpp Restitution: 40 spheres drop from y=40, restitution
// ramping 0 -> 1 left to right.
void build_restitution() {
    ignore add_ground_box(50.0f);
    i32 count = 40;
    b3Sphere sphere;
    sphere.center = b3Vec3{0.0f, 0.0f, 0.0f};
    sphere.radius = 0.5f;
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    f32 dr = 1.0f / cast(f32, count - 1);
    f32 x = -1.0f * cast(f32, count - 1);
    for i32 i = 0; i < count; i++ {
        bodyDef.position = b3Pos{x, 40.0f, 0.0f};
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateSphereShape(bodyId, &shapeDef, &sphere);
        shapeDef.baseMaterial.restitution += dr;
        x += 2.0f;
    }
}

// sample_shapes.cpp InclinedPlane: a 40-degree ramp, five unit cubes
// with increasing friction.
void build_inclined_plane() {
    ignore add_ground_box(50.0f);
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.position = b3Pos{0.0f, 7.5f, -5.0f};
    bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisX, 40.0f * PI_F / 180.0f);
    b3BodyId planeBody = b3CreateBody(g_world, &bodyDef);
    b3BoxHull planeBox = b3MakeBoxHull(16.0f, 0.5f, 10.0f);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.baseMaterial.friction = 1.0f;
    ignore b3CreateHullShape(planeBody, &shapeDef, &planeBox.base);

    b3BoxHull box = b3MakeBoxHull(1.0f, 1.0f, 1.0f);
    bodyDef.rotation = b3Quat_identity;
    bodyDef.type = b3_dynamicBody;
    for i32 index = 0; index < 5; index++ {
        bodyDef.position = b3Pos{-10.0f + 5.0f * cast(f32, index), 14.25f, -10.6f};
        b3BodyId boxBody = b3CreateBody(g_world, &bodyDef);
        shapeDef.baseMaterial.friction = cast(f32, (index + 1) * (index + 1)) * 0.04f;
        ignore b3CreateHullShape(boxBody, &shapeDef, &box.base);
    }
}

// sample_shapes.cpp IsotropicFriction: 32 unit cubes on a circle of
// radius 15, each launched radially at 25 m/s.
void build_isotropic_friction() {
    ignore add_ground_box(100.0f);
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    b3BoxHull box = b3MakeBoxHull(1.0f, 1.0f, 1.0f);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.baseMaterial.friction = 0.6f;
    for i32 index = 0; index < 32; index++ {
        f32 alpha = PI_F / 16.0f * cast(f32, index);
        f32 c = cosf(alpha);
        f32 s = sinf(alpha);
        bodyDef.position = b3Pos{15.0f * c, 1.0f, 15.0f * s};
        bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisY, 0.0f - alpha);
        bodyDef.linearVelocity = b3Vec3{25.0f * c, 0.0f, 25.0f * s};
        b3BodyId boxBody = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateHullShape(boxBody, &shapeDef, &box.base);
    }
}

// sample_shapes.cpp RollingResistance: a shallow ramp with a row of
// spheres and a row of capsules, rolling resistance ramping across each.
void build_rolling_resistance() {
    ignore add_ground_box(80.0f);
    b3BodyDef bodyDef = b3DefaultBodyDef();
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    bodyDef.position = b3Pos{0.0f, 2.0f, -20.0f};
    bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisX, 10.0f * PI_F / 180.0f);
    b3BodyId planeBody = b3CreateBody(g_world, &bodyDef);
    b3BoxHull plane = b3MakeBoxHull(32.0f, 0.5f, 15.0f);
    ignore b3CreateHullShape(planeBody, &shapeDef, &plane.base);

    b3Sphere sphere;
    sphere.center = b3Vec3{0.0f, 0.0f, 0.0f};
    sphere.radius = 1.0f;
    bodyDef.type = b3_dynamicBody;
    for i32 index = 0; index < 5; index++ {
        bodyDef.position = b3Pos{-25.0f + 5.0f * cast(f32, index), 8.0f, -24.0f};
        b3BodyId body = b3CreateBody(g_world, &bodyDef);
        shapeDef.baseMaterial.rollingResistance = 0.05f * cast(f32, index);
        ignore b3CreateSphereShape(body, &shapeDef, &sphere);
    }

    b3Capsule capsule;
    capsule.center1 = b3Vec3{-1.0f, 0.0f, 0.0f};
    capsule.center2 = b3Vec3{1.0f, 0.0f, 0.0f};
    capsule.radius = 0.5f;
    for i32 index = 0; index < 5; index++ {
        bodyDef.position = b3Pos{2.0f + 5.0f * cast(f32, index), 8.0f, -24.0f};
        b3BodyId body = b3CreateBody(g_world, &bodyDef);
        shapeDef.baseMaterial.rollingResistance = 0.05f * cast(f32, index);
        ignore b3CreateCapsuleShape(body, &shapeDef, &capsule);
    }
}

// sample_shapes.cpp HighResistance: ten upright capsules tipped 30
// degrees, each with more rolling resistance than the last.
void build_high_resistance() {
    ignore add_ground_box(50.0f);
    b3BodyDef bodyDef = b3DefaultBodyDef();
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    b3Capsule capsule;
    capsule.center1 = b3Vec3{0.0f, -1.0f, 0.0f};
    capsule.center2 = b3Vec3{0.0f, 1.0f, 0.0f};
    capsule.radius = 0.5f;
    bodyDef.type = b3_dynamicBody;
    bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, 30.0f * PI_F / 180.0f);
    for i32 index = 0; index < 10; index++ {
        bodyDef.position = b3Pos{-22.0f + 5.0f * cast(f32, index), 1.5f, 0.0f};
        b3BodyId body = b3CreateBody(g_world, &bodyDef);
        shapeDef.baseMaterial.rollingResistance = 0.2f * cast(f32, index);
        ignore b3CreateCapsuleShape(body, &shapeDef, &capsule);
    }
}

// samples/sample_shapes.cpp SlideTwist
//
// Upstream sets baseMaterial.friction three times; only the value before
// each b3CreateHullShape applies.
void build_slide_twist() {
    ignore add_ground_box(50.0f);

    b3Quat orientation = b3MakeQuatFromAxisAngle(b3Vec3_axisX, 20.0f * B3_DEG_TO_RAD);
    b3BodyDef bodyDef = b3DefaultBodyDef();
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.baseMaterial.friction = 1.0f;

    bodyDef.position = b3Pos{0.0f, 4.0f, 0.0f};
    bodyDef.rotation = orientation;
    b3BodyId planeBody = b3CreateBody(g_world, &bodyDef);
    b3BoxHull plane = b3MakeBoxHull(10.0f, 0.5f, 10.0f);
    shapeDef.baseMaterial.friction = 0.6f;
    ignore b3CreateHullShape(planeBody, &shapeDef, &plane.base);

    bodyDef.type = b3_dynamicBody;
    bodyDef.position = b3Pos{0.0f, 5.0f, 0.0f};
    bodyDef.rotation = orientation;
    // spin about the plane's up axis
    b3Vec3 up = b3RotateVector(orientation, b3Vec3_axisY);
    bodyDef.angularVelocity = b3Vec3{25.0f * up.x, 25.0f * up.y, 25.0f * up.z};
    b3BodyId boxBody = b3CreateBody(g_world, &bodyDef);
    b3BoxHull mBox = b3MakeBoxHull(1.0f, 0.5f, 1.0f);
    shapeDef.baseMaterial.friction = 0.3f;
    ignore b3CreateHullShape(boxBody, &shapeDef, &mBox.base);
}

// samples/sample_shapes.cpp StaticInvoke
//
// invokeContactCreation decides whether the new static shape wakes the
// body already touching it.
b3BodyId g_si_body;
bool g_si_body_valid;
bool g_si_invoke;

void build_static_invoke() {
    ignore add_ground_box(20.0f);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = b3Pos{0.25f, 1.0f, 0.0f};
    b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.baseMaterial.rollingResistance = 0.2f;
    b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, 0.5f};
    ignore b3CreateSphereShape(bodyId, &shapeDef, &sphere);

    g_si_invoke = false;
    g_si_body_valid = false;
}

void si_create_static() {
    if g_si_body_valid {
        b3DestroyBody(g_si_body);
        g_si_body_valid = false;
    }
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.position = b3Pos{0.0f, 0.5f, 0.0f};
    g_si_body = b3CreateBody(g_world, &bodyDef);
    g_si_body_valid = true;
    b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, 0.5f};
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.invokeContactCreation = g_si_invoke;
    ignore b3CreateSphereShape(g_si_body, &shapeDef, &sphere);
}

void step_static_invoke(f32 dt) {
    ignore dt;
    if g_step_count == 20 { si_create_static(); }
}

bool si_controls() {
    if ImGui_RadioButton("Invoke", g_si_invoke) { g_si_invoke = true; }
    if ImGui_RadioButton("Passive", !g_si_invoke) { g_si_invoke = false; }
    if !g_si_body_valid {
        if ImGui_Button("Create", ImVec2{0.0f, 0.0f}) { si_create_static(); }
    } else {
        if ImGui_Button("Destroy", ImVec2{0.0f, 0.0f}) {
            b3DestroyBody(g_si_body);
            g_si_body_valid = false;
        }
    }
    return true;
}

// samples/sample_shapes.cpp ConveyorBelt
void build_conveyor_belt() {
    ignore add_ground_box(20.0f);

    // Platform
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{-5.0f, 5.0f, 0.0f};
        bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisY, 0.2f);
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);

        b3BoxHull box = b3MakeBoxHull(10.0f, 0.25f, 2.0f);

        b3ShapeDef shapeDef = b3DefaultShapeDef();
        shapeDef.baseMaterial.friction = 0.8f;
        shapeDef.baseMaterial.tangentVelocity = b3Vec3{2.0f, 0.0f, 0.0f};
        ignore b3CreateHullShape(bodyId, &shapeDef, &box.base);
    }

    // Boxes
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    b3BoxHull cube = b3MakeBoxHull(0.5f, 0.5f, 0.5f);
    for i32 i = 0; i < 5; i += 1 {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;
        bodyDef.position = b3Pos{-10.0f + 2.0f * cast(f32, i), 7.0f, 0.0f};
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);

        ignore b3CreateHullShape(bodyId, &shapeDef, &cube.base);
    }
}

// samples/sample_shapes.cpp Wind
const i32 WIND_SPHERE_SHAPE = 0;
const i32 WIND_CAPSULE_SHAPE = 1;
const i32 WIND_BOX_SHAPE = 2;
const i32 WIND_MAX_COUNT = 60;

i32 g_wind_shape_type;
b3Vec3 g_wind_wind;
f32 g_wind_drag;
f32 g_wind_lift;
b3Vec3 g_wind_noise;
b3BodyId g_wind_ground;
b3BodyId[WIND_MAX_COUNT] g_wind_bodies;
i32 g_wind_count;
i32 g_wind_created;

void wind_create_scene() {
    for i32 i = 0; i < g_wind_created; i += 1 {
        b3DestroyBody(g_wind_bodies[i]);
    }
    g_wind_created = 0;

    f32 radius = 0.1f;
    f32 verticalOffset = 2.0f;

    b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, radius};
    b3Capsule capsule = b3Capsule{b3Pos{-radius, 0.0f, 0.0f}, b3Pos{radius, 0.0f, 0.0f}, 0.5f * radius};
    b3BoxHull box = b3MakeBoxHull(1.25f * radius, 0.75f * radius, 0.125f * radius);

    b3SphericalJointDef jointDef = b3DefaultSphericalJointDef();
    jointDef.base.bodyIdA = g_wind_ground;
    jointDef.base.localFrameA.p = b3Vec3{0.0f, verticalOffset, 0.0f};
    jointDef.base.drawScale = 0.1f;

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.density = 20.0f;

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.gravityScale = 0.5f;
    bodyDef.enableSleep = false;

    for i32 i = 0; i < g_wind_count; i += 1 {
        bodyDef.position = b3Pos{(2.0f * cast(f32, i) + 1.0f) * radius, verticalOffset, 0.0f};
        g_wind_bodies[i] = b3CreateBody(g_world, &bodyDef);

        if g_wind_shape_type == WIND_SPHERE_SHAPE {
            ignore b3CreateSphereShape(g_wind_bodies[i], &shapeDef, &sphere);
        } else if g_wind_shape_type == WIND_CAPSULE_SHAPE {
            ignore b3CreateCapsuleShape(g_wind_bodies[i], &shapeDef, &capsule);
        } else {
            ignore b3CreateHullShape(g_wind_bodies[i], &shapeDef, &box.base);
        }

        jointDef.base.bodyIdB = g_wind_bodies[i];
        jointDef.base.localFrameB.p = b3Vec3{-radius, 0.0f, 0.0f};
        ignore b3CreateSphericalJoint(g_world, &jointDef);

        jointDef.base.bodyIdA = g_wind_bodies[i];
        jointDef.base.localFrameA.p = b3Vec3{radius, 0.0f, 0.0f};
        g_wind_created += 1;
    }
}

void build_wind() {
    ignore add_ground_box(20.0f);

    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        g_wind_ground = b3CreateBody(g_world, &bodyDef);
    }

    g_wind_shape_type = WIND_BOX_SHAPE;
    g_wind_wind = b3Vec3{6.0f, 0.0f, 0.0f};
    g_wind_drag = 1.0f;
    g_wind_lift = 0.75f;
    g_wind_count = 10;
    g_wind_noise = b3Vec3{0.0f, 0.0f, 0.0f};
    g_wind_created = 0;

    wind_create_scene();
}

bool wind_controls() {
    u8*[3] shapeTypes;
    shapeTypes[0] = "Circle";
    shapeTypes[1] = "Capsule";
    shapeTypes[2] = "Box";
    i32 shapeType = g_wind_shape_type;
    if ImGui_Combo("Shape", &shapeType, cast(u8**, &shapeTypes), 3, -1) {
        g_wind_shape_type = shapeType;
        wind_create_scene();
    }

    ignore ImGui_SliderFloat("Wind", &g_wind_wind.x, -50.0f, 50.0f, "%.1f", 0);
    ignore ImGui_SliderFloat("Drag", &g_wind_drag, 0.0f, 1.0f, "%.2f", 0);
    ignore ImGui_SliderFloat("Lift", &g_wind_lift, 0.0f, 4.0f, "%.2f", 0);
    if ImGui_SliderInt("Count", &g_wind_count, 1, WIND_MAX_COUNT, "%d", 0) {
        wind_create_scene();
    }

    return true;
}

void step_wind(f32 timeStep) {
    // upstream captures shouldStep before Sample::Step decrements
    // singleStep; timeStep carries the same answer.
    if timeStep <= 0.0f { return; }
    f32 speed;
    b3Vec3 direction = b3GetLengthAndNormalize(&speed, g_wind_wind);
    b3Vec3 wind = b3MulSV(speed, b3Add(direction, g_wind_noise));

    for i32 i = 0; i < g_wind_count; i += 1 {
        b3ShapeId[1] shapeIds;
        i32 count = b3Body_GetShapes(g_wind_bodies[i], cast(b3ShapeId*, &shapeIds), 1);
        for i32 j = 0; j < count; j += 1 {
            b3Shape_ApplyWind(shapeIds[j], wind, g_wind_drag, g_wind_lift, 10.0f, true);
        }
    }

    b3Vec3 rand = random_vec3(b3Vec3{-0.3f, -0.3f, -0.3f}, b3Vec3{0.3f, 0.3f, 0.3f});
    g_wind_noise = b3Lerp(g_wind_noise, rand, 0.05f);

    b3Pos p1 = b3Pos{0.0f, 0.5f, 0.0f};
    b3Vec3 head = b3MulSV(0.2f, wind);
    b3Pos p2 = b3Pos{p1.x + head.x, p1.y + head.y, p1.z + head.z};
    dbg_arrow(p1, p2, b3_colorFuchsia);
}

// samples/sample_shapes.cpp WindDrop
f32 g_wd_drag;
f32 g_wd_lift;
b3ShapeId g_wd_shape;

void build_wind_drop() {
    ignore add_ground_box(15.0f);

    g_wd_drag = 1.0f;
    g_wd_lift = 4.0f;

    f32 radius = 0.1f;
    // b3BoxHull box = b3MakeBoxHull( 0.25f * radius, 1.25f * radius, 0.25f * radius );
    b3BoxHull box = b3MakeBoxHull(4.0f * radius, 0.1f * radius, 4.0f * radius);

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.density = 2.0f;

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.linearVelocity = b3Vec3{0.0f, 0.0f, 0.0f};
    bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisX, 0.25f);
    bodyDef.gravityScale = 0.5f;

    bodyDef.position = b3Pos{0.0f, 10.0f, 0.0f};
    b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);

    g_wd_shape = b3CreateHullShape(bodyId, &shapeDef, &box.base);
}

void step_wind_drop(f32 timeStep) {
    // upstream captures shouldStep before Sample::Step decrements
    // singleStep; timeStep carries the same answer.
    if timeStep <= 0.0f { return; }
    b3Shape_ApplyWind(g_wd_shape, b3Vec3_zero, g_wd_drag, g_wd_lift, 10.0f, true);
}

// samples/sample_shapes.cpp WindFlap
f32 g_wf_time;
f32 g_wf_drag;
f32 g_wf_lift;
b3ShapeId g_wf_shape1;
b3ShapeId g_wf_shape2;
b3ShapeId g_wf_torso_shape;
b3JointId g_wf_joint1;
b3JointId g_wf_joint2;

void build_wind_flap() {
    ignore add_ground_box(50.0f);

    g_wf_drag = 1.0f;
    g_wf_lift = 2.0f;

    f32 a = 0.4f;
    // b3BoxHull box = b3MakeBoxHull( 0.25f * radius, 1.25f * radius, 0.25f * radius );
    b3Capsule capsule = b3Capsule{b3Pos{0.0f, 0.0f, -a}, b3Pos{0.0f, 0.0f, a}, 0.25f * a};
    // b3BoxHull box = b3MakeBoxHull( 2.0f * a, 0.01f, a );
    b3Transform wingTransform1 = b3Transform{b3Vec3_zero, b3MakeQuatFromAxisAngle(b3Vec3_axisX, 0.1f)};
    b3BoxHull box1 = b3MakeTransformedBoxHull(2.0f * a, 0.01f, a, wingTransform1);
    b3Transform wingTransform2 = b3Transform{b3Vec3_zero, b3MakeQuatFromAxisAngle(b3Vec3_axisX, 0.1f)};
    b3BoxHull box2 = b3MakeTransformedBoxHull(2.0f * a, 0.01f, a, wingTransform2);

    f32 y = 20.0f;

    b3ShapeDef shapeDef = b3DefaultShapeDef();

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    // bodyDef.gravityScale = 0.5f;

    shapeDef.density = 5.0f;
    bodyDef.position = b3Pos{-2.0f * a, y, 0.0f};
    b3BodyId wingBodyId1 = b3CreateBody(g_world, &bodyDef);
    g_wf_shape1 = b3CreateHullShape(wingBodyId1, &shapeDef, &box1.base);

    bodyDef.position = b3Pos{2.0f * a, y, 0.0f};
    b3BodyId wingBodyId2 = b3CreateBody(g_world, &bodyDef);
    g_wf_shape2 = b3CreateHullShape(wingBodyId2, &shapeDef, &box2.base);

    bodyDef.position = b3Pos{0.0f, y, 0.0f};
    // bodyDef.type = b3_staticBody;
    b3BodyId torsoBodyId = b3CreateBody(g_world, &bodyDef);

    shapeDef.density = 10.0f;
    g_wf_torso_shape = b3CreateCapsuleShape(torsoBodyId, &shapeDef, &capsule);

    b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
    jointDef.base.drawScale = 0.1f;
    jointDef.base.bodyIdA = torsoBodyId;
    jointDef.base.localFrameA.p = b3Vec3{0.0f, 0.0f, 0.0f};
    jointDef.base.bodyIdB = wingBodyId1;
    jointDef.base.localFrameB.p = b3Vec3{2.0f * a, 0.0f, 0.0f};
    jointDef.enableSpring = true;
    jointDef.hertz = 6.0f;
    jointDef.dampingRatio = 0.5f;
    jointDef.enableLimit = true;
    jointDef.lowerAngle = -30.0f * PI_F / 180.0f;
    jointDef.upperAngle = 30.0f * PI_F / 180.0f;
    g_wf_joint1 = b3CreateRevoluteJoint(g_world, &jointDef);

    jointDef.base.bodyIdB = wingBodyId2;
    jointDef.base.localFrameB.p = b3Vec3{-2.0f * a, 0.0f, 0.0f};
    g_wf_joint2 = b3CreateRevoluteJoint(g_world, &jointDef);

    b3FilterJointDef filterDef = b3DefaultFilterJointDef();
    filterDef.base.bodyIdA = wingBodyId1;
    filterDef.base.bodyIdB = wingBodyId2;
    ignore b3CreateFilterJoint(g_world, &filterDef);

    g_wf_time = 0.0f;
}

void step_wind_flap(f32 timeStep) {
    // upstream captures shouldStep before Sample::Step decrements
    // singleStep; timeStep carries the same answer.
    if timeStep <= 0.0f { return; }
    f32 maxSpeed = 10.0f;
    bool wake = false;
    b3Shape_ApplyWind(g_wf_shape1, b3Vec3_zero, g_wf_drag, g_wf_lift, maxSpeed, wake);
    b3Shape_ApplyWind(g_wf_shape2, b3Vec3_zero, g_wf_drag, g_wf_lift, maxSpeed, wake);
    // b3Shape_ApplyWind( m_torsoShapeId, b3Vec3_zero, m_drag, m_lift, maxSpeed, wake );

    f32 angle = b3Sin(10.0f * g_wf_time);
    b3RevoluteJoint_SetTargetAngle(g_wf_joint1, angle);
    b3RevoluteJoint_SetTargetAngle(g_wf_joint2, -angle);

    g_wf_time += g_hertz > 0.0f ? 1.0f / g_hertz : 0.0f;
}

// samples/sample_shapes.cpp ConveyorMesh
b3WorldTransform g_cvm_mesh_transform;
b3MeshData* g_cvm_mesh_data;
b3HullData* g_cvm_cylinder_hull;
b3Vec3[7] g_cvm_velocities;

void build_conveyor_mesh() {
    ignore add_ground_box(20.0f);

    // Mesh
    {
        g_cvm_mesh_data = create_mesh_data("data/meshes/conveyor.obj", 1.0f, false, true,
                                           true, true);
        if g_cvm_mesh_data == null { return; }

        i32 triangleCount = g_cvm_mesh_data.triangleCount;
        u8* materialIndices = cast(u8*, cast(i64, g_cvm_mesh_data)
                                        + cast(i64, g_cvm_mesh_data.materialOffset));
        memset(cast(void*, materialIndices), 0, cast(i64, triangleCount));

        g_cvm_velocities[0] = b3Vec3{0.0f, 0.0f, 0.0f};

        // +x -z
        materialIndices[0] = cast(u8, 1);
        materialIndices[4] = cast(u8, 1);
        g_cvm_velocities[1] = b3Vec3{0.7f, 0.0f, -0.2f};

        // +x +z
        materialIndices[9] = cast(u8, 2);
        materialIndices[12] = cast(u8, 2);
        g_cvm_velocities[2] = b3Vec3{0.6f, 0.0f, 0.4f};

        // +z
        materialIndices[21] = cast(u8, 3);
        materialIndices[38] = cast(u8, 3);
        g_cvm_velocities[3] = b3Vec3{0.0f, 0.0f, 1.3f};

        // -x +z
        materialIndices[43] = cast(u8, 4);
        materialIndices[46] = cast(u8, 4);
        g_cvm_velocities[4] = b3Vec3{-0.6f, 0.0f, 0.4f};

        // -x -z
        materialIndices[30] = cast(u8, 5);
        materialIndices[33] = cast(u8, 5);
        g_cvm_velocities[5] = b3Vec3{-0.75f, 0.0f, -0.4f};

        // -z
        materialIndices[18] = cast(u8, 6);
        materialIndices[24] = cast(u8, 6);
        g_cvm_velocities[6] = b3Vec3{0.0f, 0.0f, -1.3f};

        b3HexColor[7] colors = {
            b3_colorGreen,     b3_colorGreenYellow, b3_colorHoneyDew, b3_colorHotPink,
            b3_colorIndianRed, b3_colorIndigo,      b3_colorIvory,
        };

        b3SurfaceMaterial[7] materials;
        for i32 i = 0; i < 7; i += 1 {
            materials[i] = b3DefaultSurfaceMaterial();
            materials[i].friction = 0.8f;
            materials[i].tangentVelocity = b3MulSV(2.0f, g_cvm_velocities[i]);
            materials[i].customColor = colors[i];
        }

        g_cvm_mesh_transform.p = b3Pos{0.0f, 0.5f, 6.0f};
        g_cvm_mesh_transform.q = b3MakeQuatFromAxisAngle(b3Vec3_axisY, 0.5f * PI_F);
        // m_meshTransform.q = b3Quat_identity;

        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = g_cvm_mesh_transform.p;
        bodyDef.rotation = g_cvm_mesh_transform.q;
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);

        b3ShapeDef shapeDef = b3DefaultShapeDef();
        shapeDef.materials = cast(b3SurfaceMaterial*, &materials);
        shapeDef.materialCount = 7;
        ignore b3CreateMeshShape(bodyId, &shapeDef, g_cvm_mesh_data, b3Vec3_one);
    }

    // High number of sides to stress the collision code
    // Normally the number of sides should be 16 or less.
    g_cvm_cylinder_hull = b3CreateCylinder(0.3f, 0.15f, 0.0f, 32);

    b3ShapeDef shapeDef = b3DefaultShapeDef();

    // Cylinders
    for i32 i = 0; i < 20; i += 1 {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;
        bodyDef.position = b3Pos{-8.5f + 0.9f * cast(f32, i), 1.5f, -5.5f};
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateHullShape(bodyId, &shapeDef, g_cvm_cylinder_hull);
    }
}

void destroy_conveyor_mesh() {
    if g_cvm_mesh_data != null {
        b3DestroyMesh(g_cvm_mesh_data);
        g_cvm_mesh_data = null;
    }
    b3DestroyHull(g_cvm_cylinder_hull);
}

void step_conveyor_mesh(f32 timeStep) {
    ignore timeStep;
    if g_cvm_mesh_data == null {
        draw_text_line("data/meshes/conveyor.obj could not be read");
        return;
    }

    i32 triangleCount = g_cvm_mesh_data.triangleCount;
    b3MeshTriangle* triangles = b3GetMeshTriangles(g_cvm_mesh_data);
    b3Vec3* vertices = b3GetMeshVertices(g_cvm_mesh_data);
    u8* materialIndices = b3GetMeshMaterialIndices(g_cvm_mesh_data);

    u8[64] buf;
    for i32 i = 0; i < triangleCount; i += 1 {
        b3MeshTriangle* t = triangles + i;
        b3Vec3 v1 = vertices[t.index1];
        b3Vec3 v2 = vertices[t.index2];
        b3Vec3 v3 = vertices[t.index3];
        b3Vec3 n = b3Cross(b3Sub(v2, v1), b3Sub(v3, v1));
        n = b3Normalize(n);
        if n.y < 0.9f {
            continue;
        }

        b3Pos p = b3TransformWorldPoint(g_cvm_mesh_transform,
                                        b3MulSV(1.0f / 3.0f, b3Add(b3Add(v1, v2), v3)));
        ignore snprintf(cast(u8*, &buf), 64, "%d", i);
        dbg_string_3d(p, make_color(b3_colorAqua), cast(u8*, &buf));

        i32 materialIndex = cast(i32, materialIndices[i]);
        b3Vec3 v = b3RotateVector(g_cvm_mesh_transform.q, g_cvm_velocities[materialIndex]);
        dbg_line(p, b3OffsetPos(p, v), b3_colorBlueViolet);
    }

    dbg_axes(b3WorldTransform_identity, 0.5f);
}
