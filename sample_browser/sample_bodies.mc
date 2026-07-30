// Body-behaviour scenes.

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
import sample_geometry;
import sample_events;
import sample_issues;
import sample_shapes;
import sample_benchmark;
import sample_continuous;
import sample_robustness;
import sample_stacking;
import sample_joint;
import sample_compound;
import sample_world;

void build_kinematic() {
    ignore add_ground_box(20.0f);
    f32 amplitude = 2.0f;
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_kinematicBody;
    bodyDef.position = b3Pos{2.0f * amplitude, amplitude + 1.0f, 0.0f};
    g_kinematic_body = b3CreateBody(g_world, &bodyDef);
    b3BoxHull box = b3MakeBoxHull(0.1f, 1.0f, 0.2f);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    ignore b3CreateHullShape(g_kinematic_body, &shapeDef, &box.base);
}

void step_kinematic(f32 timeStep) {
    f32 amplitude = 2.0f;
    f32 delay = 2.0f;
    if g_sample_time > delay {
        f32 t = g_sample_time - delay;
        b3Pos point;
        point.x = 2.0f * amplitude * cosf(t);
        point.y = amplitude * (sinf(2.0f * t) + 1.0f) + 1.0f;
        point.z = 0.0f;
        b3Quat rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, 2.0f * t);
        b3WorldTransform target;
        target.p = point;
        target.q = rotation;
        b3Body_SetTargetTransform(g_kinematic_body, target, timeStep, true);
    }
}

// samples/sample_bodies.cpp SpinningBooks
void build_spinning_books() {
    ignore add_ground_box(10.0f);
    b3BoxHull box = b3MakeBoxHull(0.35f, 0.08f, 0.5f);
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.gravityScale = 0.0f;
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    bodyDef.position = b3Pos{-2.0f, 2.0f, 0.0f};
    bodyDef.angularVelocity = b3Vec3{5.0f, 0.01f, 0.01f};
    b3BodyId body1 = b3CreateBody(g_world, &bodyDef);
    ignore b3CreateHullShape(body1, &shapeDef, &box.base);
    bodyDef.position = b3Pos{0.0f, 2.0f, 0.0f};
    bodyDef.angularVelocity = b3Vec3{0.01f, 5.0f, 0.01f};
    b3BodyId body2 = b3CreateBody(g_world, &bodyDef);
    ignore b3CreateHullShape(body2, &shapeDef, &box.base);
    bodyDef.position = b3Pos{2.0f, 2.0f, 0.0f};
    bodyDef.angularVelocity = b3Vec3{0.01f, 0.01f, -5.0f};
    b3BodyId body3 = b3CreateBody(g_world, &bodyDef);
    ignore b3CreateHullShape(body3, &shapeDef, &box.base);
}

// samples/sample_continuous.cpp BulletVersusStack

// samples/sample_bodies.cpp LockMixing
void build_lock_mixing() {
    ignore add_ground_box(20.0f);

    b3BoxHull cube = b3MakeBoxHull(1.0f, 1.0f, 1.0f);
    b3ShapeDef shapeDef = b3DefaultShapeDef();

    // "free"
    b3BodyDef free = b3DefaultBodyDef();
    free.type = b3_dynamicBody;
    free.position = b3Pos{0.0f, 2.0f, 0.0f};
    b3BodyId freeId = b3CreateBody(g_world, &free);
    ignore b3CreateHullShape(freeId, &shapeDef, &cube.base);

    // upstream leaves angularY commented out
    b3BodyDef angXZ = b3DefaultBodyDef();
    angXZ.type = b3_dynamicBody;
    angXZ.position = b3Pos{2.0f, 2.0f, 0.0f};
    angXZ.motionLocks.angularX = true;
    angXZ.motionLocks.angularZ = true;
    b3BodyId angXZId = b3CreateBody(g_world, &angXZ);
    ignore b3CreateHullShape(angXZId, &shapeDef, &cube.base);

    // "linear xyz"
    b3BodyDef linXYZ = b3DefaultBodyDef();
    linXYZ.type = b3_dynamicBody;
    linXYZ.position = b3Pos{-2.0f, 2.0f, 0.0f};
    linXYZ.motionLocks.linearX = true;
    linXYZ.motionLocks.linearY = true;
    linXYZ.motionLocks.linearZ = true;
    b3BodyId linXYZId = b3CreateBody(g_world, &linXYZ);
    ignore b3CreateHullShape(linXYZId, &shapeDef, &cube.base);

    // "full"
    b3BodyDef full = b3DefaultBodyDef();
    full.type = b3_dynamicBody;
    full.position = b3Pos{0.0f, 1.0f, 2.0f};
    full.motionLocks.linearX = true;
    full.motionLocks.linearY = true;
    full.motionLocks.linearZ = true;
    full.motionLocks.angularX = true;
    full.motionLocks.angularY = true;
    full.motionLocks.angularZ = true;
    b3BodyId fullId = b3CreateBody(g_world, &full);
    ignore b3CreateHullShape(fullId, &shapeDef, &cube.base);

    // "static"
    b3BodyDef stat = b3DefaultBodyDef();
    stat.position = b3Pos{0.0f, 1.0f, -3.0f};
    b3BodyId statId = b3CreateBody(g_world, &stat);
    ignore b3CreateHullShape(statId, &shapeDef, &cube.base);
}

// samples/sample_bodies.cpp FixedRotation
void build_fixed_rotation() {
    ignore add_ground_box(20.0f);

    b3Capsule capsule = b3Capsule{b3Pos{0.0f, 0.0f, 0.0f}, b3Pos{0.0f, 1.0f, 0.0f}, 0.3f};
    b3ShapeDef shapeDef = b3DefaultShapeDef();

    b3BodyDef staticDef = b3DefaultBodyDef();
    staticDef.position = b3Pos{0.0f, 0.5f, 0.0f};
    b3BodyId staticId = b3CreateBody(g_world, &staticDef);
    ignore b3CreateCapsuleShape(staticId, &shapeDef, &capsule);

    b3BodyDef lockedDef = b3DefaultBodyDef();
    lockedDef.position = b3Pos{0.3f, 0.5f, 0.0f};
    lockedDef.type = b3_dynamicBody;
    lockedDef.gravityScale = 0.0f;
    lockedDef.enableSleep = false;
    lockedDef.motionLocks.angularX = true;
    lockedDef.motionLocks.angularY = true;
    lockedDef.motionLocks.angularZ = true;

    // upstream mutates the shared capsule before the second body
    capsule.radius = 0.2f;
    b3BodyId lockedId = b3CreateBody(g_world, &lockedDef);
    ignore b3CreateCapsuleShape(lockedId, &shapeDef, &capsule);
}

// samples/sample_bodies.cpp DisableBody
const i32 DISABLE_COUNT = 4;
b3BodyId[DISABLE_COUNT] g_disable_bodies;
b3BodyId g_disable_ball;

void build_disable() {
    ignore add_ground_box(20.0f);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    b3ShapeDef shapeDef = b3DefaultShapeDef();

    f32 linkRadius = 0.1f;
    f32 linkLength = 5.0f * linkRadius;
    b3Capsule capsule = b3Capsule{b3Pos{0.0f, 0.0f, 0.0f},
                                  b3Pos{0.0f, 0.0f - linkLength, 0.0f}, linkRadius};

    bool haveParent = false;
    b3BodyId parentId;
    for i32 link = 0; link < DISABLE_COUNT; link++ {
        bodyDef.position = b3Pos{0.0f,
            (cast(f32, DISABLE_COUNT) - cast(f32, link)) * linkLength + 1.0f, 0.0f};
        bodyDef.type = haveParent ? b3_dynamicBody : b3_kinematicBody;
        b3BodyId childId = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateCapsuleShape(childId, &shapeDef, &capsule);
        g_disable_bodies[link] = childId;

        if haveParent {
            b3WeldJointDef jointDef = b3DefaultWeldJointDef();
            jointDef.base.bodyIdA = parentId;
            jointDef.base.bodyIdB = childId;
            jointDef.base.localFrameA.p = b3Pos{0.0f, 0.0f - linkLength, 0.0f};
            jointDef.angularHertz = 10.0f;
            jointDef.angularDampingRatio = 1.0f;
            ignore b3CreateWeldJoint(g_world, &jointDef);
        }

        parentId = childId;
        haveParent = true;
    }

    bodyDef.type = b3_dynamicBody;
    bodyDef.position = b3Pos{3.0f, 3.0f, 0.0f};
    g_disable_ball = b3CreateBody(g_world, &bodyDef);
    b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, 0.5f};
    ignore b3CreateSphereShape(g_disable_ball, &shapeDef, &sphere);
}

// upstream Step
void step_disable(f32 dt) {
    ignore dt;
    b3Body_ApplyLinearImpulseToCenter(g_disable_bodies[2], b3Vec3{0.0f, 0.1f, 0.0f}, true);
}

bool disable_controls() {
    bool link = b3Body_IsEnabled(g_disable_bodies[2]);
    if ImGui_Checkbox("Enable Link", &link) {
        if link { b3Body_Enable(g_disable_bodies[2]); }
        else { b3Body_Disable(g_disable_bodies[2]); }
    }
    bool ball = b3Body_IsEnabled(g_disable_ball);
    if ImGui_Checkbox("Enable Ball", &ball) {
        if ball { b3Body_Enable(g_disable_ball); }
        else { b3Body_Disable(g_disable_ball); }
    }
    return true;
}

// samples/sample_bodies.cpp BodyType
b3BodyId g_bt_attachment;
b3BodyId g_bt_second_attachment;
b3BodyId g_bt_platform;
b3BodyId g_bt_second_payload;
b3BodyId g_bt_touching;
b3BodyId g_bt_floating;
i32 g_bt_type = b3_dynamicBody;
f32 g_bt_speed = 3.0f;
bool g_bt_enabled = true;

void build_body_type() {
    g_bt_type = b3_dynamicBody;
    g_bt_enabled = true;
    b3BodyId groundId = add_ground_box(20.0f);

    // "attach1"
    b3BodyDef a1 = b3DefaultBodyDef();
    a1.type = b3_dynamicBody;
    a1.position = b3Pos{-2.0f, 3.0f, 0.0f};
    g_bt_attachment = b3CreateBody(g_world, &a1);
    b3BoxHull attachBox = b3MakeBoxHull(0.5f, 2.0f, 0.5f);
    b3ShapeDef attachShape = b3DefaultShapeDef();
    attachShape.density = 1.0f;
    ignore b3CreateHullShape(g_bt_attachment, &attachShape, &attachBox.base);

    // "attach2"
    b3BodyDef a2 = b3DefaultBodyDef();
    a2.type = g_bt_type;
    a2.isEnabled = g_bt_enabled;
    a2.position = b3Pos{3.0f, 3.0f, 0.0f};
    g_bt_second_attachment = b3CreateBody(g_world, &a2);
    ignore b3CreateHullShape(g_bt_second_attachment, &attachShape, &attachBox.base);

    // platform: a box turned on its side by a local transform, 4 m
    // along local +x from the body origin
    b3BodyDef pd = b3DefaultBodyDef();
    pd.type = g_bt_type;
    pd.isEnabled = g_bt_enabled;
    pd.position = b3Pos{-4.0f, 5.0f, 0.0f};
    g_bt_platform = b3CreateBody(g_world, &pd);
    b3Quat platformQ = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, 0.5f * PI_F);
    b3BoxHull platformBox = b3MakeTransformedBoxHull(0.5f, 4.0f, 0.5f,
        b3Transform{b3Vec3{4.0f, 0.0f, 0.0f}, platformQ});
    b3ShapeDef platformShape = b3DefaultShapeDef();
    platformShape.density = 2.0f;
    ignore b3CreateHullShape(g_bt_platform, &platformShape, &platformBox.base);

    // revolute to each attachment, anchored in world space
    b3RevoluteJointDef rev = b3DefaultRevoluteJointDef();
    b3Pos pivot = b3Pos{-2.0f, 5.0f, 0.0f};
    rev.base.bodyIdA = g_bt_attachment;
    rev.base.bodyIdB = g_bt_platform;
    rev.base.localFrameA.p = b3Body_GetLocalPoint(g_bt_attachment, pivot);
    rev.base.localFrameB.p = b3Body_GetLocalPoint(g_bt_platform, pivot);
    rev.maxMotorTorque = 50.0f;
    rev.enableMotor = true;
    ignore b3CreateRevoluteJoint(g_world, &rev);

    pivot = b3Pos{3.0f, 5.0f, 0.0f};
    rev.base.bodyIdA = g_bt_second_attachment;
    rev.base.bodyIdB = g_bt_platform;
    rev.base.localFrameA.p = b3Body_GetLocalPoint(g_bt_second_attachment, pivot);
    rev.base.localFrameB.p = b3Body_GetLocalPoint(g_bt_platform, pivot);
    rev.maxMotorTorque = 50.0f;
    rev.enableMotor = true;
    ignore b3CreateRevoluteJoint(g_world, &rev);

    // prismatic track
    b3PrismaticJointDef pris = b3DefaultPrismaticJointDef();
    b3Pos anchor = b3Pos{0.0f, 5.0f, 0.0f};
    pris.base.bodyIdA = groundId;
    pris.base.bodyIdB = g_bt_platform;
    pris.base.localFrameA.p = b3Body_GetLocalPoint(groundId, anchor);
    pris.base.localFrameB.p = b3Body_GetLocalPoint(g_bt_platform, anchor);
    pris.maxMotorForce = 1000.0f;
    pris.motorSpeed = 0.0f;
    pris.enableMotor = true;
    pris.lowerTranslation = -10.0f;
    pris.upperTranslation = 10.0f;
    pris.enableLimit = true;
    ignore b3CreatePrismaticJoint(g_world, &pris);
    g_bt_speed = 3.0f;

    // "crate1"
    b3BodyDef c1 = b3DefaultBodyDef();
    c1.type = b3_dynamicBody;
    c1.position = b3Pos{-3.0f, 8.0f, 0.0f};
    b3BodyId crate1 = b3CreateBody(g_world, &c1);
    b3BoxHull crateBox = b3MakeBoxHull(0.75f, 0.75f, 0.75f);
    b3ShapeDef crateShape = b3DefaultShapeDef();
    crateShape.density = 2.0f;
    ignore b3CreateHullShape(crate1, &crateShape, &crateBox.base);

    // "crate2"
    b3BodyDef c2 = b3DefaultBodyDef();
    c2.type = g_bt_type;
    c2.isEnabled = g_bt_enabled;
    c2.position = b3Pos{2.0f, 8.0f, 0.0f};
    g_bt_second_payload = b3CreateBody(g_world, &c2);
    ignore b3CreateHullShape(g_bt_second_payload, &crateShape, &crateBox.base);

    // "debris"
    b3BodyDef dd = b3DefaultBodyDef();
    dd.type = g_bt_type;
    dd.isEnabled = g_bt_enabled;
    dd.position = b3Pos{8.0f, 0.2f, 0.0f};
    g_bt_touching = b3CreateBody(g_world, &dd);
    b3Capsule debris = b3Capsule{b3Pos{0.0f, 0.0f, 0.0f}, b3Pos{1.0f, 0.0f, 0.0f}, 0.25f};
    b3ShapeDef debrisShape = b3DefaultShapeDef();
    debrisShape.density = 2.0f;
    ignore b3CreateCapsuleShape(g_bt_touching, &debrisShape, &debris);

    // "floater": gravity off, sphere half a metre up
    b3BodyDef fd = b3DefaultBodyDef();
    fd.type = g_bt_type;
    fd.isEnabled = g_bt_enabled;
    fd.position = b3Pos{-8.0f, 12.0f, 0.0f};
    fd.gravityScale = 0.0f;
    g_bt_floating = b3CreateBody(g_world, &fd);
    b3Sphere floater = b3Sphere{b3Pos{0.0f, 0.5f, 0.0f}, 0.25f};
    b3ShapeDef floaterShape = b3DefaultShapeDef();
    floaterShape.density = 2.0f;
    ignore b3CreateSphereShape(g_bt_floating, &floaterShape, &floater);
}

// upstream Step: bounce the kinematic platform between x = -14 and 6
void step_body_type(f32 dt) {
    ignore dt;
    if g_bt_type != b3_kinematicBody { return; }
    b3Pos p = b3Body_GetPosition(g_bt_platform);
    b3Vec3 v = b3Body_GetLinearVelocity(g_bt_platform);
    if (p.x < -14.0f && v.x < 0.0f) || (p.x > 6.0f && v.x > 0.0f) {
        v.x = 0.0f - v.x;
        b3Body_SetLinearVelocity(g_bt_platform, v);
    }
}

void bt_set_type(i32 type) {
    g_bt_type = type;
    b3Body_SetType(g_bt_platform, type);
    if type == b3_kinematicBody {
        b3Body_SetLinearVelocity(g_bt_platform, b3Vec3{0.0f - g_bt_speed, 0.0f, 0.0f});
        b3Body_SetAngularVelocity(g_bt_platform, b3Vec3_zero);
    }
    b3Body_SetType(g_bt_second_attachment, type);
    if type == b3_kinematicBody {
        b3Body_SetLinearVelocity(g_bt_second_attachment, b3Vec3_zero);
        b3Body_SetAngularVelocity(g_bt_second_attachment, b3Vec3_zero);
    }
    b3Body_SetType(g_bt_second_payload, type);
    b3Body_SetType(g_bt_touching, type);
    b3Body_SetType(g_bt_floating, type);
}

bool bt_controls() {
    if ImGui_RadioButton("Static", g_bt_type == b3_staticBody) {
        bt_set_type(b3_staticBody);
    }
    if ImGui_RadioButton("Kinematic", g_bt_type == b3_kinematicBody) {
        bt_set_type(b3_kinematicBody);
    }
    if ImGui_RadioButton("Dynamic", g_bt_type == b3_dynamicBody) {
        bt_set_type(b3_dynamicBody);
    }
    // upstream toggles attach1, crate2 and the floater, not attach2
    if ImGui_Checkbox("Enable", &g_bt_enabled) {
        if g_bt_enabled {
            b3Body_Enable(g_bt_attachment);
            b3Body_Enable(g_bt_second_payload);
            b3Body_Enable(g_bt_floating);
        } else {
            b3Body_Disable(g_bt_attachment);
            b3Body_Disable(g_bt_second_payload);
            b3Body_Disable(g_bt_floating);
        }
    }
    return true;
}

// samples/sample_bodies.cpp GyroscopicTorque
b3BodyId g_gt_body;

void build_gyroscopic_torque() {
    ignore add_ground_box(20.0f);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = b3Pos{0.0f, 2.0f, 0.0f};
    bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisX, -0.5f * PI_F);
    bodyDef.gravityScale = 0.0f;
    g_gt_body = b3CreateBody(g_world, &bodyDef);

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.updateBodyMass = false;
    b3HullData* cylinder = b3CreateCylinder(0.6f, 0.15f, 0.0f, 32);
    b3BoxHull box = b3MakeBoxHull(1.0f, 0.05f, 0.1f);
    ignore b3CreateHullShape(g_gt_body, &shapeDef, cylinder);
    ignore b3CreateHullShape(g_gt_body, &shapeDef, &box.base);
    b3Body_ApplyMassFromShapes(g_gt_body);

    // Set the angular velocity after creating the shapes and the local center of mass is fixed.
    b3Body_SetAngularVelocity(g_gt_body, b3Vec3{0.01f, 0.01f, 10.0f});

    b3DestroyHull(cylinder);
}

void step_gyroscopic_torque(f32 timeStep) {
    ignore timeStep;
    b3Pos c = b3Body_GetWorldCenter(g_gt_body);
    u8[128] buf;
    ignore snprintf(cast(u8*, &buf), 128, "center %.3f %.3f %.3f", c.x, c.y, c.z);
    draw_text_line(cast(u8*, &buf));
}

// samples/sample_bodies.cpp Weeble
b3BodyId g_weeble_body;
b3Pos g_weeble_explosion_position;
f32 g_weeble_explosion_radius = 8.0f;
f32 g_weeble_explosion_magnitude = 20000.0f;

void build_weeble() {
    ignore add_ground_box(30.0f);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = b3Pos{0.0f, 3.0f, 0.0f};
    g_weeble_body = b3CreateBody(g_world, &bodyDef);
    b3Capsule capsule = b3Capsule{b3Pos{0.0f, -1.0f, 0.0f}, b3Pos{0.0f, 1.0f, 0.0f}, 1.0f};
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.baseMaterial.rollingResistance = 0.1f;
    ignore b3CreateCapsuleShape(g_weeble_body, &shapeDef, &capsule);

    f32 mass = b3Body_GetMass(g_weeble_body);
    b3Matrix3 inertiaTensor = b3Body_GetLocalRotationalInertia(g_weeble_body);
    b3Vec3 offset = b3Vec3{0.0f, -1.5f, 0.0f};

    // See: https://en.wikipedia.org/wiki/Parallel_axis_theorem
    inertiaTensor = b3AddMM(inertiaTensor, b3Steiner(mass, offset));

    b3MassData massData;
    massData.mass = mass;
    massData.center = offset;
    massData.inertia = inertiaTensor;
    b3Body_SetMassData(g_weeble_body, massData);

    g_weeble_explosion_position = b3Pos{0.0f, -0.1f, 0.0f};
    g_weeble_explosion_radius = 8.0f;
    g_weeble_explosion_magnitude = 20000.0f;
}

void step_weeble(f32 timeStep) {
    ignore timeStep;
    b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, g_weeble_explosion_radius};
    dbg_wire_sphere(b3Transform{b3Vec3{g_weeble_explosion_position.x,
                                       g_weeble_explosion_position.y,
                                       g_weeble_explosion_position.z}, b3Quat_identity},
                    &sphere, 64, b3_colorAzure);

    // This shows how to get the velocity of a point on a body
    b3Vec3 localPoint = b3Vec3{0.0f, 2.0f, 0.0f};
    b3Pos worldPoint = b3Body_GetWorldPoint(g_weeble_body, localPoint);
    b3Vec3 v1 = b3Body_GetLocalPointVelocity(g_weeble_body, localPoint);
    b3Vec3 v2 = b3Body_GetWorldPointVelocity(g_weeble_body, worldPoint);
    b3Vec3 offset = b3Vec3{0.05f, 0.0f, 0.0f};
    adapter_segment(worldPoint, b3Pos{worldPoint.x + v1.x, worldPoint.y + v1.y,
                                      worldPoint.z + v1.z}, b3_colorRed, null);
    adapter_segment(b3Pos{worldPoint.x + offset.x, worldPoint.y + offset.y, worldPoint.z + offset.z},
                    b3Pos{worldPoint.x + v2.x + offset.x, worldPoint.y + v2.y + offset.y,
                          worldPoint.z + v2.z + offset.z}, b3_colorGreen, null);
}

bool weeble_controls() {
    if ImGui_Button("Teleport", ImVec2{0.0f, 0.0f}) {
        b3Body_SetTransform(g_weeble_body, b3Pos{0.0f, 5.0f, 0.0f},
                            b3MakeQuatFromAxisAngle(b3Vec3_axisZ, 0.95f * PI_F));
        b3Body_SetAwake(g_weeble_body, true);
    }
    if ImGui_Button("Explode", ImVec2{0.0f, 0.0f}) {
        b3ExplosionDef def = b3DefaultExplosionDef();
        def.position = g_weeble_explosion_position;
        def.radius = g_weeble_explosion_radius;
        def.falloff = 0.1f;
        def.impulsePerArea = g_weeble_explosion_magnitude;
        b3World_Explode(g_world, &def);
    }
    ImGui_PushItemWidth(6.0f * ImGui_GetFontSize());
    ignore ImGui_SliderFloat("Magnitude", &g_weeble_explosion_magnitude,
                             -100000.0f, 100000.0f, "%.0f", 0);
    ImGui_PopItemWidth();
    return true;
}

// samples/sample_bodies.cpp GyroscopicPrecession
const f32 FLT_EPSILON = 1.192092896e-07f;

b3BodyId g_gp_top;
f32 g_gp_mass;
f32 g_gp_gravity;
f32 g_gp_pivot_distance;
f32 g_gp_spin_inertia;
f32 g_gp_transverse_inertia;
f32 g_gp_azimuth;
f32 g_gp_actual_angle;
f32 g_gp_expected_angle;
f32 g_gp_elapsed;
f32 g_gp_ground_time;
f32 g_gp_rate;
bool g_gp_measuring;

void build_gyroscopic_precession() {
    ignore add_ground_box(40.0f);

    // Top shape: a wide n-gon rim up top and a point at the origin, so it balances on its tip.
    const i32 numSegs = 7;
    const f32 r = 2.0f;
    const f32 h = 2.0f;
    b3Vec3[numSegs + 1] hullPoints;
    const f32 dphi = 2.0f * PI_F / cast(f32, numSegs);
    for i32 i = 0; i < numSegs; i += 1 {
        hullPoints[i] = b3Vec3{r * cosf(cast(f32, i) * dphi), h, r * sinf(cast(f32, i) * dphi)};
    }
    hullPoints[numSegs] = b3Vec3_zero;
    b3HullData* hull = b3CreateHull(cast(b3Vec3*, &hullPoints), numSegs + 1, numSegs + 1);

    b3ShapeDef shapeDef = b3DefaultShapeDef();

    // Tilt the top, then spin it about its own symmetry axis. Gravity does the rest.
    b3Quat rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, 15.0f * PI_F / 180.0f);
    b3Vec3 angularVelocity = b3RotateVector(rotation, b3Vec3{0.0f, 75.0f, 0.0f});

    const i32 count = 8;
    const f32 separation = 6.0f;
    for i32 x = 0; x < count; x += 1 {
        for i32 z = 0; z < count; z += 1 {
            b3BodyDef bodyDef = b3DefaultBodyDef();
            bodyDef.type = b3_dynamicBody;
            bodyDef.position = b3Pos{cast(f32, x - count / 2) * separation, h,
                                     cast(f32, z - count / 2) * separation};
            bodyDef.rotation = rotation;

            // The spin rate exceeds the default cap, so bypass it as the test intends.
            bodyDef.allowFastRotation = true;

            b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
            ignore b3CreateHullShape(bodyId, &shapeDef, hull);

            b3Body_SetAngularVelocity(bodyId, angularVelocity);

            if x == 0 && z == 0 {
                g_gp_top = bodyId;
            }
        }
    }

    b3DestroyHull(hull);

    // Mass properties of the measured top. The tip sits at the body origin, so the pivot distance is
    // just the height of the center of mass, and the symmetry axis is the local up axis.
    b3MassData massData = b3Body_GetMassData(g_gp_top);
    g_gp_mass = massData.mass;
    g_gp_pivot_distance = massData.center.y;
    g_gp_spin_inertia = massData.inertia.cy.y;

    // Transverse inertia belongs about the pivot, not the center of mass
    f32 transverse = 0.5f * (massData.inertia.cx.x + massData.inertia.cz.z);
    g_gp_transverse_inertia = transverse + g_gp_mass * g_gp_pivot_distance * g_gp_pivot_distance;

    g_gp_gravity = b3Length(b3World_GetGravity(g_world));

    g_gp_azimuth = 0.0f;
    g_gp_actual_angle = 0.0f;
    g_gp_expected_angle = 0.0f;
    g_gp_elapsed = 0.0f;
    g_gp_ground_time = 0.0f;
    g_gp_rate = 0.0f;
    g_gp_measuring = false;
}

// Steady precession rate of a heavy symmetric top about the vertical. The slow root of
// I1 * W^2 * cos(tilt) - I3 * spin * W + M * g * d = 0. Goldstein 5.7.
// Collapses to torque over spin momentum for a fast top.
f32 gyroscopic_expected_rate(f32 spin, f32 cosTilt) {
    f32 momentum = g_gp_spin_inertia * spin;
    f32 torque = g_gp_mass * g_gp_gravity * g_gp_pivot_distance;
    if momentum <= FLT_EPSILON {
        return 0.0f;
    }

    f32 a = g_gp_transverse_inertia * cosTilt;
    f32 discriminant = momentum * momentum - 4.0f * a * torque;
    if a <= FLT_EPSILON || discriminant < 0.0f {
        // Axis at or past horizontal, or spinning too slowly to precess steadily
        return torque / momentum;
    }

    return (momentum - sqrtf(discriminant)) / (2.0f * a);
}

void step_gyroscopic_precession(f32 timeStep) {
    bool isAwake = b3Body_IsAwake(g_gp_top);
    if isAwake == false {
        draw_text_line("top is sleeping");
        return;
    }

    b3Pos tip = b3Body_GetPosition(g_gp_top);
    b3Quat quat = b3Body_GetRotation(g_gp_top);
    b3Vec3 axis = b3RotateVector(quat, b3Vec3_axisY);
    b3Vec3 omega = b3Body_GetAngularVelocity(g_gp_top);
    f32 spin = b3Dot(omega, axis);
    f32 cosTilt = b3ClampFloat(axis.y, -1.0f, 1.0f);
    f32 expected = gyroscopic_expected_rate(spin, cosTilt);

    if timeStep > 0.0f {
        // Gravity exerts no torque about the center of mass while the top is airborne, so the pivot
        // solution only applies once the tip lands. Give the landing impulse time to wash out too,
        // otherwise it biases the average for the rest of the run.
        if tip.y < 0.05f {
            g_gp_ground_time += timeStep;
        }

        if g_gp_measuring == false {
            if g_gp_ground_time > 0.5f {
                g_gp_measuring = true;
                g_gp_azimuth = b3Atan2(-axis.z, axis.x);
            }
        } else {
            // Right handed angle of the symmetry axis about the world up axis
            f32 azimuth = b3Atan2(-axis.z, axis.x);
            f32 delta = b3UnwindAngle(azimuth - g_gp_azimuth);
            g_gp_azimuth = azimuth;

            g_gp_actual_angle += delta;
            g_gp_expected_angle += expected * timeStep;
            g_gp_elapsed += timeStep;

            // Nutation swings the instantaneous rate between zero and twice the mean, so filter it
            // with a one second time constant.
            f32 alpha = timeStep / (timeStep + 1.0f);
            g_gp_rate += alpha * (delta / timeStep - g_gp_rate);
        }
    }

    dbg_line(tip, b3OffsetPos(tip, b3MulSV(5.0f, axis)), b3_colorYellow);

    u8[192] buf;
    ignore snprintf(cast(u8*, &buf), 192, "spin %.1f rad/s, tilt %.1f deg",
                    spin, (180.0f / PI_F) * acosf(cosTilt));
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 192,
                    "precession: expected %.4f rad/s, actual %.4f rad/s", expected, g_gp_rate);
    draw_text_line(cast(u8*, &buf));

    if g_gp_elapsed > 0.0f {
        // Average both sides over the same window. The spin decays, so the expected rate drifts with it.
        f32 expectedAverage = g_gp_expected_angle / g_gp_elapsed;
        f32 actualAverage = g_gp_actual_angle / g_gp_elapsed;
        f32 error = 0.0f;
        if expectedAverage != 0.0f {
            error = 100.0f * (actualAverage - expectedAverage) / expectedAverage;
        }
        ignore snprintf(cast(u8*, &buf), 192,
                        "%.1f s average: expected %.4f, actual %.4f, error %+.1f%%",
                        g_gp_elapsed, expectedAverage, actualAverage, error);
        draw_text_line(cast(u8*, &buf));
    } else {
        draw_text_line("waiting for the tip to land");
    }
}

// samples/sample_bodies.cpp BodyCast
b3HullData* g_bc_cylinder;
b3BodyId g_bc_body;
b3WorldTransform g_bc_transform;
b3Pos g_bc_base_translation;
b3Pos g_bc_origin;
i32 g_bc_base_x;
i32 g_bc_base_y;
bool g_bc_tracking;

void build_body_cast() {
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_kinematicBody;
    bodyDef.position = b3Pos{5.0f, 5.0f, 0.0f};
    bodyDef.angularVelocity = b3Vec3{0.1f, -0.1f, 0.1f};
    g_bc_body = b3CreateBody(g_world, &bodyDef);

    g_bc_cylinder = b3CreateCylinder(2.0f, 0.5f, 0.0f, 16);

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    ignore b3CreateHullShape(g_bc_body, &shapeDef, g_bc_cylinder);

    g_bc_transform.p = b3Pos{-10.0f, 2.0f, 0.0f};
    g_bc_transform.q = b3MakeQuatFromAxisAngle(b3Normalize(b3Vec3{1.0f, -2.0f, 3.0f}),
                                               0.75f * PI_F);

    g_bc_base_translation = b3Pos_zero;
    g_bc_base_x = 0;
    g_bc_base_y = 0;
    g_bc_origin = b3Pos_zero;
    g_bc_tracking = false;
}

void destroy_body_cast() {
    b3DestroyHull(g_bc_cylinder);
}

bool body_cast_mouse_down(f32 px, f32 py, i32 button, i32 modifiers) {
    if button == 0 && modifiers == 1 {
        PickRay pickRay = build_pick_ray(px, py);
        b3Vec3 dir = b3Normalize(b3Vec3{pickRay.translation.x, pickRay.translation.y,
                                        pickRay.translation.z});
        g_bc_origin = b3OffsetPos(b3Pos{pickRay.origin.x, pickRay.origin.y, pickRay.origin.z},
                                  b3MulSV(10.0f, dir));
        g_bc_base_translation = g_bc_transform.p;
        g_bc_tracking = true;
        return true;
    } else {
        g_bc_tracking = false;
    }
    return false;
}

void body_cast_mouse_up(f32 px, f32 py, i32 button) {
    ignore px; ignore py;
    if button == 0 {
        g_bc_tracking = false;
    }
}

void body_cast_mouse_move(f32 px, f32 py) {
    if g_bc_tracking {
        PickRay pickRay = build_pick_ray(px, py);
        b3Vec3 dir = b3Normalize(b3Vec3{pickRay.translation.x, pickRay.translation.y,
                                        pickRay.translation.z});
        b3Pos origin = b3OffsetPos(b3Pos{pickRay.origin.x, pickRay.origin.y, pickRay.origin.z},
                                   b3MulSV(10.0f, dir));
        g_bc_transform.p = b3OffsetPos(g_bc_base_translation, b3SubPos(origin, g_bc_origin));
    }
}

void step_body_cast(f32 timeStep) {
    ignore timeStep;

    // Cast ray
    {
        b3Pos origin = b3Pos{-9.75f, 3.0f, -4.0f};
        b3Vec3 translation = b3Vec3{0.0f, 0.0f, 8.0f};
        b3QueryFilter filter = b3DefaultQueryFilter();
        f32 maxFraction = 1.0f;
        b3BodyCastResult result = b3Body_CastRay(g_bc_body, origin, translation, filter,
                                                 maxFraction, g_bc_transform);
        dbg_line(origin, b3OffsetPos(origin, b3MulSV(maxFraction, translation)), b3_colorCyan);
        if result.hit {
            b3Pos hitPoint = result.point;
            dbg_line(hitPoint, b3OffsetPos(hitPoint, b3MulSV(0.2f, result.normal)),
                     b3_colorYellow);
            adapter_point(hitPoint, 10.0f, b3_colorYellow, null);
        }
        adapter_point(origin, 10.0f, b3_colorGreen, null);
        adapter_point(b3OffsetPos(origin, translation), 10.0f, b3_colorRed, null);
    }

    // Cast sphere
    {
        b3Pos origin = b3Pos{-14.5f, 2.5f, 0.5f};
        b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, 0.2f};
        b3ShapeProxy proxy = b3ShapeProxy{&sphere.center, 1, sphere.radius};
        b3Vec3 translation = b3Vec3{8.0f, 0.0f, 0.0f};
        b3QueryFilter filter = b3DefaultQueryFilter();
        f32 maxFraction = 1.0f;
        bool canEncroach = true;
        b3BodyCastResult result = b3Body_CastShape(g_bc_body, origin, &proxy, translation,
                                                   filter, maxFraction, canEncroach,
                                                   g_bc_transform);
        b3Pos sphereCenter = b3OffsetPos(origin, sphere.center);
        if result.hit {
            b3WorldTransform t;
            t.p = b3OffsetPos(origin, b3MulSV(result.fraction, translation));
            t.q = b3Quat_identity;
            dbg_solid_sphere(t, sphere, make_color(b3_colorGreen));
            b3Pos hitPoint = result.point;
            dbg_line(hitPoint, b3OffsetPos(hitPoint, b3MulSV(0.2f, result.normal)),
                     b3_colorYellow);
        } else {
            b3WorldTransform t;
            t.p = b3OffsetPos(origin, b3MulSV(maxFraction, translation));
            t.q = b3Quat_identity;
            dbg_solid_sphere(t, sphere, make_color(b3_colorWhite));
        }
        dbg_line(sphereCenter, b3OffsetPos(sphereCenter, b3MulSV(maxFraction, translation)),
                 b3_colorWhite);
        adapter_point(sphereCenter, 10.0f, b3_colorGreen, null);
        adapter_point(b3OffsetPos(sphereCenter, b3MulSV(maxFraction, translation)), 10.0f,
                      b3_colorRed, null);
    }

    // Overlap capsule
    {
        b3Pos origin = b3Pos{-10.0f, 1.0f, 0.5f};
        b3Capsule capsule = b3Capsule{b3Pos{-0.5f, 1.0f, 0.0f}, b3Pos{0.5f, 0.0f, 0.0f}, 0.5f};
        b3ShapeProxy proxy = b3ShapeProxy{&capsule.center1, 2, capsule.radius};
        bool overlaps = b3Body_OverlapShape(g_bc_body, origin, &proxy, b3DefaultQueryFilter(),
                                            g_bc_transform);
        b3WorldTransform t;
        t.p = origin;
        t.q = b3Quat_identity;
        if overlaps {
            dbg_solid_capsule(t, capsule, make_color(b3_colorGreen));
        } else {
            dbg_solid_capsule(t, capsule, make_color(b3_colorGray));
        }
    }

    // Collide capsule
    {
        b3Pos origin = b3Pos{-10.0f, 2.0f, -0.75f};
        b3Capsule capsule = b3Capsule{b3Pos{-0.25f, 0.0f, 0.0f}, b3Pos{0.25f, 1.0f, 0.0f},
                                      0.3f};
        b3BodyPlaneResult[4] bodyPlanes;
        i32 count = b3Body_CollideMover(g_bc_body, cast(b3BodyPlaneResult*, &bodyPlanes), 4,
                                        origin, &capsule, b3DefaultQueryFilter(),
                                        g_bc_transform);
        b3WorldTransform t;
        t.p = origin;
        t.q = b3Quat_identity;
        dbg_solid_capsule(t, capsule, make_color(b3_colorPurple));
        for i32 i = 0; i < count; i += 1 {
            b3PlaneResult result = bodyPlanes[i].result;
            dbg_plane(result.plane.normal, b3ToPos(result.point), b3_colorOrange);
        }
    }

    dbg_ground_grid(10);
    b3WorldTransform t;
    t.p = b3Pos{0.0f, 0.1f, 0.0f};
    t.q = b3Quat_identity;
    dbg_axes(t, 4.0f);
    dbg_hull(g_bc_transform, g_bc_cylinder, b3_colorBlue);
}
