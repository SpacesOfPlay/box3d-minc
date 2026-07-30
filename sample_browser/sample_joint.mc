// Joint scenes. Ports of samples/sample_joint.cpp.

import box3d;
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
import sample_shapes;

// samples/sample_joint.cpp FilterJoint
void build_filter_joint() {
    ignore add_ground_box(20.0f);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = b3Pos{2.0f, 4.0f, 0.0f};
    b3BodyId bodyId1 = b3CreateBody(g_world, &bodyDef);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    b3BoxHull box = b3MakeBoxHull(0.5f, 0.5f, 0.5f);
    ignore b3CreateHullShape(bodyId1, &shapeDef, &box.base);

    bodyDef.position = b3Pos{-2.0f, 4.0f, 0.0f};
    b3BodyId bodyId2 = b3CreateBody(g_world, &bodyDef);
    ignore b3CreateHullShape(bodyId2, &shapeDef, &box.base);

    b3FilterJointDef jointDef = b3DefaultFilterJointDef();
    jointDef.base.bodyIdA = bodyId1;
    jointDef.base.bodyIdB = bodyId2;
    ignore b3CreateFilterJoint(g_world, &jointDef);
}

// samples/sample_joint.cpp BallAndChain
void build_ball_and_chain() {
    b3BodyDef bodyDef = b3DefaultBodyDef();
    b3BodyId groundBody = b3CreateBody(g_world, &bodyDef);

    f32 linkRadius = 0.125f;
    f32 linkExtent = 0.5f;
    b3Capsule capsule = b3Capsule{b3Pos{0.0f - linkExtent, 0.0f, 0.0f},
                                  b3Pos{linkExtent, 0.0f, 0.0f}, linkRadius};
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    i32 linkCount = 32;
    bodyDef.type = b3_dynamicBody;
    b3BodyId parent = groundBody;

    b3SphericalJointDef jointDef = b3DefaultSphericalJointDef();
    jointDef.base.localFrameA = b3Transform_identity;
    jointDef.base.localFrameB = b3Transform{b3Vec3{0.0f - linkExtent, 0.0f, 0.0f}, b3Quat_identity};
    jointDef.enableMotor = true;
    jointDef.maxMotorTorque = 10.0f;

    for i32 i = 0; i < linkCount; i++ {
        bodyDef.position = b3Pos{(1.0f + 2.0f * cast(f32, i)) * linkExtent, 0.0f, 0.0f};
        b3BodyId childId = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateCapsuleShape(childId, &shapeDef, &capsule);
        jointDef.base.bodyIdA = parent;
        jointDef.base.bodyIdB = childId;
        ignore b3CreateSphericalJoint(g_world, &jointDef);
        jointDef.base.localFrameA = b3Transform{b3Vec3{linkExtent, 0.0f, 0.0f}, b3Quat_identity};
        parent = childId;
    }

    f32 sphereRadius = 2.0f;
    b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, sphereRadius};
    bodyDef.position = b3Pos{(1.0f + 2.0f * cast(f32, linkCount)) * linkExtent
                             + sphereRadius - linkExtent, 0.0f, 0.0f};
    b3BodyId childId = b3CreateBody(g_world, &bodyDef);
    ignore b3CreateSphereShape(childId, &shapeDef, &sphere);
    jointDef.base.bodyIdA = parent;
    jointDef.base.bodyIdB = childId;
    jointDef.base.localFrameB = b3Transform{b3Vec3{0.0f - sphereRadius, 0.0f, 0.0f}, b3Quat_identity};
    ignore b3CreateSphericalJoint(g_world, &jointDef);
}

// samples/sample_joint.cpp Bridge
const i32 BRIDGE_COUNT = 150;
b3BodyId[BRIDGE_COUNT] g_bridge_bodies;
f32 g_bridge_gravity_scale = 1.0f;

void bridge_pair(b3SphericalJointDef* jointDef, b3BodyId prev, b3BodyId cur,
                 f32 x, f32 z) {
    b3Pos pivot = b3Pos{x, 20.0f, z};
    jointDef.base.bodyIdA = prev;
    jointDef.base.bodyIdB = cur;
    jointDef.base.localFrameA.p = b3Body_GetLocalPoint(prev, pivot);
    jointDef.base.localFrameB.p = b3Body_GetLocalPoint(cur, pivot);
    ignore b3CreateSphericalJoint(g_world, jointDef);
}

void build_bridge() {
    ignore add_ground_box(60.0f);

    b3BodyDef groundDef = b3DefaultBodyDef();
    b3BodyId groundId = b3CreateBody(g_world, &groundDef);

    f32 a = 0.125f;
    b3BoxHull box = b3MakeBoxHull(a, 0.125f, 0.5f);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.density = 20.0f;

    b3SphericalJointDef jointDef = b3DefaultSphericalJointDef();
    jointDef.base.constraintHertz = 1000.0f;
    jointDef.enableSpring = true;
    jointDef.hertz = 2.0f;
    jointDef.dampingRatio = 1.0f;
    g_bridge_gravity_scale = 1.0f;

    f32 xbase = -160.0f * a;
    b3BodyId prevBodyId = groundId;
    for i32 i = 0; i < BRIDGE_COUNT; i++ {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;
        bodyDef.position = b3Pos{xbase + a * (1.0f + 2.0f * cast(f32, i)), 20.0f, 0.0f};
        bodyDef.linearDamping = 0.1f;
        bodyDef.angularDamping = 0.1f;
        g_bridge_bodies[i] = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateHullShape(g_bridge_bodies[i], &shapeDef, &box.base);

        f32 x = xbase + 2.0f * a * cast(f32, i);
        bridge_pair(&jointDef, prevBodyId, g_bridge_bodies[i], x, -0.5f);
        bridge_pair(&jointDef, prevBodyId, g_bridge_bodies[i], x, 0.5f);
        prevBodyId = g_bridge_bodies[i];
    }

    f32 xend = xbase + 2.0f * a * cast(f32, BRIDGE_COUNT);
    bridge_pair(&jointDef, prevBodyId, groundId, xend, -0.5f);
    bridge_pair(&jointDef, prevBodyId, groundId, xend, 0.5f);
}

bool bridge_controls() {
    ImGui_PushItemWidth(6.0f * ImGui_GetFontSize());
    if ImGui_SliderFloat("Gravity scale", &g_bridge_gravity_scale, -1.0f, 1.0f, "%.1f", 0) {
        for i32 i = 0; i < BRIDGE_COUNT; i++ {
            b3Body_SetGravityScale(g_bridge_bodies[i], g_bridge_gravity_scale);
        }
    }
    ImGui_PopItemWidth();
    return true;
}

// samples/sample_joint.cpp WeldJoint
b3BodyId g_weld_body;
b3JointId g_weld_joint;
f32 g_weld_linear_hertz;
f32 g_weld_linear_damping;
f32 g_weld_angular_hertz = 2.0f;
f32 g_weld_angular_damping = 0.7f;

void build_weld_joint() {
    ignore add_ground_box(20.0f);

    b3BodyDef groundDef = b3DefaultBodyDef();
    groundDef.position = b3Pos{0.0f, -1.0f, 0.0f};
    b3BodyId groundId = b3CreateBody(g_world, &groundDef);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = b3Pos{0.0f, 4.0f, 0.0f};
    bodyDef.gravityScale = 0.0f;
    g_weld_body = b3CreateBody(g_world, &bodyDef);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    b3BoxHull box = b3MakeBoxHull(0.5f, 1.5f, 0.25f);
    ignore b3CreateHullShape(g_weld_body, &shapeDef, &box.base);

    b3WeldJointDef jointDef = b3DefaultWeldJointDef();
    jointDef.base.bodyIdA = groundId;
    jointDef.base.bodyIdB = g_weld_body;
    jointDef.base.localFrameA.p = b3Pos{0.0f, 6.5f, 0.0f};
    jointDef.base.localFrameB.p = b3Pos{0.0f, 1.5f, 0.0f};
    jointDef.base.constraintHertz = 240.0f;
    jointDef.linearHertz = g_weld_linear_hertz;
    jointDef.linearDampingRatio = g_weld_linear_damping;
    jointDef.angularHertz = g_weld_angular_hertz;
    jointDef.angularDampingRatio = g_weld_angular_damping;
    jointDef.base.drawScale = 2.0f;
    g_weld_joint = b3CreateWeldJoint(g_world, &jointDef);
}

bool weld_controls() {
    ImGui_PushItemWidth(6.0f * ImGui_GetFontSize());
    if ImGui_SliderFloat("Linear Hertz", &g_weld_linear_hertz, 0.0f, 10.0f, "%.1f", 0) {
        b3WeldJoint_SetLinearHertz(g_weld_joint, g_weld_linear_hertz);
        b3Joint_WakeBodies(g_weld_joint);
    }
    if ImGui_SliderFloat("Linear Damping", &g_weld_linear_damping, 0.0f, 2.0f, "%.1f", 0) {
        b3WeldJoint_SetLinearDampingRatio(g_weld_joint, g_weld_linear_damping);
        b3Joint_WakeBodies(g_weld_joint);
    }
    if ImGui_SliderFloat("Angular Hertz", &g_weld_angular_hertz, 0.0f, 10.0f, "%.1f", 0) {
        b3WeldJoint_SetAngularHertz(g_weld_joint, g_weld_angular_hertz);
        b3Joint_WakeBodies(g_weld_joint);
    }
    if ImGui_SliderFloat("Angular Damping", &g_weld_angular_damping, 0.0f, 2.0f, "%.1f", 0) {
        b3WeldJoint_SetAngularDampingRatio(g_weld_joint, g_weld_angular_damping);
        b3Joint_WakeBodies(g_weld_joint);
    }
    ImGui_PopItemWidth();
    return true;
}

// samples/sample_joint.cpp DistanceJoint
const i32 DJ_MAX_COUNT = 20;
b3BodyId g_dj_ground;
b3BodyId[DJ_MAX_COUNT] g_dj_bodies;
b3JointId[DJ_MAX_COUNT] g_dj_joints;
i32 g_dj_count;
f32 g_dj_hertz = 5.0f;
f32 g_dj_damping = 0.5f;
f32 g_dj_length = 1.0f;
f32 g_dj_min_length = 1.0f;
f32 g_dj_max_length = 1.0f;
f32 g_dj_tension = 2000.0f;
f32 g_dj_compression = 100.0f;
bool g_dj_spring;
bool g_dj_limit;

void dj_create_scene(i32 newCount) {
    for i32 i = 0; i < g_dj_count; i++ { b3DestroyJoint(g_dj_joints[i], false); }
    for i32 i = 0; i < g_dj_count; i++ { b3DestroyBody(g_dj_bodies[i]); }
    g_dj_count = newCount;

    f32 radius = 0.25f;
    b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, radius};
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.density = 20.0f;
    f32 yOffset = 20.0f;

    b3DistanceJointDef jointDef = b3DefaultDistanceJointDef();
    jointDef.hertz = g_dj_hertz;
    jointDef.dampingRatio = g_dj_damping;
    jointDef.length = g_dj_length;
    jointDef.lowerSpringForce = 0.0f - g_dj_tension;
    jointDef.upperSpringForce = g_dj_compression;
    jointDef.minLength = g_dj_min_length;
    jointDef.maxLength = g_dj_max_length;
    jointDef.enableSpring = g_dj_spring;
    jointDef.enableLimit = g_dj_limit;

    b3BodyId prevBodyId = g_dj_ground;
    for i32 i = 0; i < g_dj_count; i++ {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;
        bodyDef.angularDamping = 1.0f;
        bodyDef.position = b3Pos{g_dj_length * (cast(f32, i) + 1.0f), yOffset, 0.0f};
        g_dj_bodies[i] = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateSphereShape(g_dj_bodies[i], &shapeDef, &sphere);

        b3Pos pivotA = b3Pos{g_dj_length * cast(f32, i), yOffset, 0.0f};
        b3Pos pivotB = b3Pos{g_dj_length * (cast(f32, i) + 1.0f), yOffset, 0.0f};
        jointDef.base.bodyIdA = prevBodyId;
        jointDef.base.bodyIdB = g_dj_bodies[i];
        jointDef.base.localFrameA.p = b3Body_GetLocalPoint(prevBodyId, pivotA);
        jointDef.base.localFrameB.p = b3Body_GetLocalPoint(g_dj_bodies[i], pivotB);
        g_dj_joints[i] = b3CreateDistanceJoint(g_world, &jointDef);
        prevBodyId = g_dj_bodies[i];
    }
}

void build_distance_joint() {
    ignore add_ground_box(20.0f);
    b3BodyDef bodyDef = b3DefaultBodyDef();
    g_dj_ground = b3CreateBody(g_world, &bodyDef);

    g_dj_count = 0;
    g_dj_hertz = 5.0f;
    g_dj_damping = 0.5f;
    g_dj_length = 1.0f;
    g_dj_min_length = g_dj_length;
    g_dj_max_length = g_dj_length;
    g_dj_tension = 2000.0f;
    g_dj_compression = 100.0f;
    g_dj_spring = false;
    g_dj_limit = false;
    dj_create_scene(1);
}

bool dj_controls() {
    ImGui_PushItemWidth(6.0f * ImGui_GetFontSize());
    if ImGui_SliderFloat("Length", &g_dj_length, 0.1f, 4.0f, "%3.1f", 0) {
        for i32 i = 0; i < g_dj_count; i++ {
            b3DistanceJoint_SetLength(g_dj_joints[i], g_dj_length);
            b3Joint_WakeBodies(g_dj_joints[i]);
        }
    }
    if ImGui_Checkbox("Spring", &g_dj_spring) {
        for i32 i = 0; i < g_dj_count; i++ {
            b3DistanceJoint_EnableSpring(g_dj_joints[i], g_dj_spring);
            b3Joint_WakeBodies(g_dj_joints[i]);
        }
    }
    if g_dj_spring {
        if ImGui_SliderFloat("Tension##Spring", &g_dj_tension, 0.0f, 4000.0f, "%.0f", 0) {
            for i32 i = 0; i < g_dj_count; i++ {
                b3DistanceJoint_SetSpringForceRange(g_dj_joints[i],
                    0.0f - g_dj_tension, g_dj_compression);
            }
        }
        if ImGui_SliderFloat("Compression##Spring", &g_dj_compression, 0.0f, 200.0f, "%.0f", 0) {
            for i32 i = 0; i < g_dj_count; i++ {
                b3DistanceJoint_SetSpringForceRange(g_dj_joints[i],
                    0.0f - g_dj_tension, g_dj_compression);
            }
        }
        if ImGui_SliderFloat("Hertz##Spring", &g_dj_hertz, 0.0f, 15.0f, "%3.1f", 0) {
            for i32 i = 0; i < g_dj_count; i++ {
                b3DistanceJoint_SetSpringHertz(g_dj_joints[i], g_dj_hertz);
            }
        }
        if ImGui_SliderFloat("Damping##Spring", &g_dj_damping, 0.0f, 4.0f, "%3.1f", 0) {
            for i32 i = 0; i < g_dj_count; i++ {
                b3DistanceJoint_SetSpringDampingRatio(g_dj_joints[i], g_dj_damping);
            }
        }
    }
    if ImGui_Checkbox("Limit", &g_dj_limit) {
        for i32 i = 0; i < g_dj_count; i++ {
            b3DistanceJoint_EnableLimit(g_dj_joints[i], g_dj_limit);
            b3Joint_WakeBodies(g_dj_joints[i]);
        }
    }
    if g_dj_limit {
        if ImGui_SliderFloat("Min##Limit", &g_dj_min_length, 0.1f, 4.0f, "%3.1f", 0) {
            for i32 i = 0; i < g_dj_count; i++ {
                b3DistanceJoint_SetLengthRange(g_dj_joints[i], g_dj_min_length, g_dj_max_length);
            }
        }
        if ImGui_SliderFloat("Max##Limit", &g_dj_max_length, 0.1f, 4.0f, "%3.1f", 0) {
            for i32 i = 0; i < g_dj_count; i++ {
                b3DistanceJoint_SetLengthRange(g_dj_joints[i], g_dj_min_length, g_dj_max_length);
            }
        }
    }
    i32 count = g_dj_count;
    if ImGui_SliderInt("Count", &count, 1, DJ_MAX_COUNT, null, 0) {
        dj_create_scene(count);
    }
    ImGui_PopItemWidth();
    return true;
}

// samples/sample_joint.cpp RevoluteJoint
b3BodyId g_rev_body;
b3JointId g_rev_joint;
f32 g_rev_target_angle;
f32 g_rev_motor_speed;
f32 g_rev_motor_torque = 5000.0f;
f32 g_rev_hertz = 2.0f;
f32 g_rev_damping = 0.7f;
f32 g_rev_lower_deg = -35.0f;
f32 g_rev_upper_deg = 35.0f;
bool g_rev_spring;
bool g_rev_motor;
bool g_rev_limit;

void build_revolute_joint() {
    g_rev_target_angle = 0.0f;
    g_rev_motor_speed = 0.0f;
    g_rev_motor_torque = 5000.0f;
    g_rev_hertz = 2.0f;
    g_rev_damping = 0.7f;
    g_rev_lower_deg = -35.0f;
    g_rev_upper_deg = 35.0f;
    g_rev_spring = false;
    g_rev_motor = false;
    g_rev_limit = false;

    ignore add_ground_box(20.0f);
    b3BodyDef groundDef = b3DefaultBodyDef();
    groundDef.position = b3Pos{0.0f, -1.0f, 0.0f};
    b3BodyId groundId = b3CreateBody(g_world, &groundDef);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = b3Pos{0.0f, 4.0f, 0.0f};
    g_rev_body = b3CreateBody(g_world, &bodyDef);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    b3BoxHull box = b3MakeBoxHull(0.5f, 1.5f, 0.25f);
    ignore b3CreateHullShape(g_rev_body, &shapeDef, &box.base);

    b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
    jointDef.base.bodyIdA = groundId;
    jointDef.base.bodyIdB = g_rev_body;
    jointDef.base.localFrameA.p = b3Pos{0.0f, 6.5f, 0.0f};
    jointDef.base.localFrameB.p = b3Pos{0.0f, 1.5f, 0.0f};
    jointDef.base.drawScale = 2.0f;
    jointDef.enableLimit = g_rev_limit;
    jointDef.lowerAngle = B3_DEG_TO_RAD * g_rev_lower_deg;
    jointDef.upperAngle = B3_DEG_TO_RAD * g_rev_upper_deg;
    jointDef.enableSpring = g_rev_spring;
    jointDef.hertz = g_rev_hertz;
    jointDef.dampingRatio = g_rev_damping;
    jointDef.enableMotor = g_rev_motor;
    jointDef.maxMotorTorque = g_rev_motor_torque;
    jointDef.motorSpeed = g_rev_motor_speed;
    g_rev_joint = b3CreateRevoluteJoint(g_world, &jointDef);
}

bool revolute_controls() {
    if ImGui_Checkbox("Limit", &g_rev_limit) {
        b3RevoluteJoint_EnableLimit(g_rev_joint, g_rev_limit);
        b3Joint_WakeBodies(g_rev_joint);
    }
    if g_rev_limit {
        if ImGui_SliderFloat("Lower Angle", &g_rev_lower_deg, -180.0f, 180.0f, "%.0f", 0) {
            b3RevoluteJoint_SetLimits(g_rev_joint, B3_DEG_TO_RAD * g_rev_lower_deg,
                                      B3_DEG_TO_RAD * g_rev_upper_deg);
            b3Joint_WakeBodies(g_rev_joint);
        }
        if ImGui_SliderFloat("Upper Angle", &g_rev_upper_deg, -180.0f, 180.0f, "%.0f", 0) {
            b3RevoluteJoint_SetLimits(g_rev_joint, B3_DEG_TO_RAD * g_rev_lower_deg,
                                      B3_DEG_TO_RAD * g_rev_upper_deg);
            b3Joint_WakeBodies(g_rev_joint);
        }
    }
    if ImGui_Checkbox("Motor", &g_rev_motor) {
        b3RevoluteJoint_EnableMotor(g_rev_joint, g_rev_motor);
        b3Joint_WakeBodies(g_rev_joint);
    }
    if g_rev_motor {
        if ImGui_SliderFloat("Max Torque", &g_rev_motor_torque, 0.0f, 50000.0f, "%.0f", 0) {
            b3RevoluteJoint_SetMaxMotorTorque(g_rev_joint, g_rev_motor_torque);
            b3Joint_WakeBodies(g_rev_joint);
        }
        if ImGui_SliderFloat("Speed", &g_rev_motor_speed, -10.0f, 10.0f, "%.0f", 0) {
            b3RevoluteJoint_SetMotorSpeed(g_rev_joint, g_rev_motor_speed);
            b3Joint_WakeBodies(g_rev_joint);
        }
    }
    if ImGui_Checkbox("Spring", &g_rev_spring) {
        b3RevoluteJoint_EnableSpring(g_rev_joint, g_rev_spring);
        b3Joint_WakeBodies(g_rev_joint);
    }
    if g_rev_spring {
        if ImGui_SliderFloat("Hertz", &g_rev_hertz, 0.0f, 10.0f, "%.1f", 0) {
            b3RevoluteJoint_SetSpringHertz(g_rev_joint, g_rev_hertz);
            b3Joint_WakeBodies(g_rev_joint);
        }
        if ImGui_SliderFloat("Damping", &g_rev_damping, 0.0f, 2.0f, "%.1f", 0) {
            b3RevoluteJoint_SetSpringDampingRatio(g_rev_joint, g_rev_damping);
            b3Joint_WakeBodies(g_rev_joint);
        }
        if ImGui_SliderFloat("Rotation", &g_rev_target_angle, -180.0f, 180.0f, "%.0f", 0) {
            b3RevoluteJoint_SetTargetAngle(g_rev_joint, B3_DEG_TO_RAD * g_rev_target_angle);
            b3Joint_WakeBodies(g_rev_joint);
        }
    }
    return true;
}

// upstream formats with %g; minc's formatter has no %g, so %f
void step_revolute_joint(f32 timeStep) {
    ignore timeStep;
    b3MassData massData = b3Body_GetMassData(g_rev_body);
    b3Vec3 w = b3Body_GetAngularVelocity(g_rev_body);
    b3Vec3 v = b3Body_GetLinearVelocity(g_rev_body);
    f32 ke = 0.5f * b3Dot(w, b3MulMV(massData.inertia, w));
    ke += 0.5f * massData.mass * b3Dot(v, v);
    b3Pos center = b3Body_GetWorldCenter(g_rev_body);
    b3Vec3 gravity = b3World_GetGravity(g_world);
    f32 pe = 0.0f - massData.mass * center.y * gravity.y;

    u8[128] buf;
    ignore snprintf(cast(u8*, &buf), 128, "kinetic energy = %.4f", ke);
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 128, "potential energy = %.4f", pe);
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 128, "total energy = %.4f", ke + pe);
    draw_text_line(cast(u8*, &buf));
}

// samples/sample_joint.cpp MotorJoint
b3BodyId g_motor_target;
b3BodyId g_motor_body;
b3JointId g_motor_joint;
b3Transform g_motor_transform;
f32 g_motor_max_force = 400000.0f;
f32 g_motor_max_torque = 500000.0f;
f32 g_motor_speed;
f32 g_motor_time;

void build_motor_joint() {
    ignore add_ground_box(20.0f);
    b3BodyDef groundDef = b3DefaultBodyDef();
    groundDef.position.y = -1.0f;
    b3BodyId groundId = b3CreateBody(g_world, &groundDef);

    g_motor_transform = b3Transform{b3Vec3{0.0f, 10.0f, 0.0f}, b3Quat_identity};

    b3BodyDef targetDef = b3DefaultBodyDef();
    targetDef.type = b3_kinematicBody;
    targetDef.position = b3Pos{g_motor_transform.p.x, g_motor_transform.p.y, g_motor_transform.p.z};
    g_motor_target = b3CreateBody(g_world, &targetDef);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = b3Pos{g_motor_transform.p.x, g_motor_transform.p.y, g_motor_transform.p.z};
    g_motor_body = b3CreateBody(g_world, &bodyDef);
    b3BoxHull box = b3MakeBoxHull(1.0f, 0.25f, 0.25f);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    ignore b3CreateHullShape(g_motor_body, &shapeDef, &box.base);

    g_motor_max_force = 400000.0f;
    g_motor_max_torque = 500000.0f;
    b3MotorJointDef jointDef = b3DefaultMotorJointDef();
    jointDef.base.bodyIdA = g_motor_target;
    jointDef.base.bodyIdB = g_motor_body;
    jointDef.linearHertz = 4.0f;
    jointDef.linearDampingRatio = 0.7f;
    jointDef.angularHertz = 4.0f;
    jointDef.angularDampingRatio = 0.7f;
    jointDef.maxSpringForce = g_motor_max_force;
    jointDef.maxSpringTorque = g_motor_max_torque;
    g_motor_joint = b3CreateMotorJoint(g_world, &jointDef);

    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;
        bodyDef.position = b3Pos{-2.0f, 2.0f, 0.0f};
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);

        b3BoxHull box2 = b3MakeBoxHull(0.5f, 0.5f, 0.5f);
        b3ShapeDef shapeDef2 = b3DefaultShapeDef();
        ignore b3CreateHullShape(bodyId, &shapeDef2, &box2.base);

        b3MotorJointDef jointDef2 = b3DefaultMotorJointDef();
        jointDef2.base.bodyIdA = groundId;
        jointDef2.base.bodyIdB = bodyId;
        jointDef2.base.localFrameA.p = b3Vec3{-1.75f, 3.25f, 0.0f};
        jointDef2.base.localFrameB.p = b3Vec3{0.25f, 0.25f, 0.0f};
        jointDef2.base.collideConnected = true;
        jointDef2.linearHertz = 7.5f;
        jointDef2.linearDampingRatio = 0.7f;
        jointDef2.angularHertz = 7.5f;
        jointDef2.angularDampingRatio = 0.7f;
        jointDef2.maxSpringForce = 200000.0f;
        jointDef2.maxSpringTorque = 10000.0f;

        ignore b3CreateMotorJoint(g_world, &jointDef2);
    }

    g_motor_speed = 0.0f;
    g_motor_time = 0.0f;
}

void step_motor_joint(f32 dt) {
    ignore dt;
    f32 timeStep = g_hertz > 0.0f ? 1.0f / g_hertz : 0.0f;
    if g_pause && g_single_step == 0 { timeStep = 0.0f; }
    if timeStep > 0.0f {
        g_motor_time += g_motor_speed * timeStep;
        b3Pos linearOffset = b3Pos{6.0f * sinf(2.0f * g_motor_time),
                                   10.0f + 4.0f * sinf(1.0f * g_motor_time),
                                   0.0f};
        f32 angularOffset = 2.0f * g_motor_time;
        g_motor_transform = b3Transform{
            b3Vec3{linearOffset.x, linearOffset.y, linearOffset.z},
            b3MakeQuatFromAxisAngle(b3Vec3_axisZ, angularOffset)};
        b3Body_SetTargetTransform(g_motor_target, g_motor_transform, timeStep, true);
    }

    dbg_axes(b3MakeWorldTransform(g_motor_transform), 1.0f);

    b3Vec3 force = b3Joint_GetConstraintForce(g_motor_joint);
    b3Vec3 torque = b3Joint_GetConstraintTorque(g_motor_joint);
    u8[128] buf;
    ignore snprintf(cast(u8*, &buf), 128, "force = %3.f, torque = %3.f",
                    b3Length(force), b3Length(torque));
    draw_text_line(cast(u8*, &buf));
}

bool motor_controls() {
    ImGui_PushItemWidth(6.0f * ImGui_GetFontSize());
    ignore ImGui_SliderFloat("Speed", &g_motor_speed, -5.0f, 5.0f, "%.0f", 0);
    if ImGui_SliderFloat("Max Force", &g_motor_max_force, 0.0f, 1000000.0f, "%.0f", 0) {
        b3MotorJoint_SetMaxSpringForce(g_motor_joint, g_motor_max_force);
    }
    if ImGui_SliderFloat("Max Torque", &g_motor_max_torque, 0.0f, 1000000.0f, "%.0f", 0) {
        b3MotorJoint_SetMaxSpringTorque(g_motor_joint, g_motor_max_torque);
    }
    ImGui_PopItemWidth();
    if ImGui_Button("Apply Impulse", ImVec2{0.0f, 0.0f}) {
        b3Body_ApplyLinearImpulseToCenter(g_motor_body, b3Vec3{100000.0f, 0.0f, 0.0f}, true);
    }
    return true;
}

// samples/sample_joint.cpp PrismaticJoint
b3BodyId g_pris_body;
b3JointId g_pris_joint;
f32 g_pris_target_translation;
f32 g_pris_motor_speed;
f32 g_pris_motor_force = 20.0f;
f32 g_pris_hertz = 2.0f;
f32 g_pris_damping = 0.7f;
f32 g_pris_lower = -1.0f;
f32 g_pris_upper = 1.0f;
bool g_pris_spring = true;
bool g_pris_motor;
bool g_pris_limit;

void build_prismatic_joint() {
    g_pris_target_translation = 0.0f;
    g_pris_motor_speed = 0.0f;
    g_pris_motor_force = 20.0f;
    g_pris_hertz = 2.0f;
    g_pris_damping = 0.7f;
    g_pris_lower = -1.0f;
    g_pris_upper = 1.0f;
    g_pris_spring = true;
    g_pris_motor = false;
    g_pris_limit = false;

    ignore add_ground_box(20.0f);
    b3BodyDef groundDef = b3DefaultBodyDef();
    groundDef.position = b3Pos{0.0f, -1.0f, 0.0f};
    b3BodyId groundId = b3CreateBody(g_world, &groundDef);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = b3Pos{0.0f, 4.0f, 0.0f};
    bodyDef.gravityScale = 0.0f;
    g_pris_body = b3CreateBody(g_world, &bodyDef);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    b3BoxHull box = b3MakeBoxHull(0.5f, 1.5f, 0.25f);
    ignore b3CreateHullShape(g_pris_body, &shapeDef, &box.base);

    b3PrismaticJointDef jointDef = b3DefaultPrismaticJointDef();
    jointDef.base.bodyIdA = groundId;
    jointDef.base.bodyIdB = g_pris_body;
    jointDef.base.localFrameA.p = b3Pos{0.0f, 6.5f, 0.0f};
    jointDef.base.localFrameB.p = b3Pos{0.0f, 1.5f, 0.0f};
    jointDef.base.constraintHertz = 120.0f;
    jointDef.enableLimit = g_pris_limit;
    jointDef.lowerTranslation = g_pris_lower;
    jointDef.upperTranslation = g_pris_upper;
    jointDef.enableSpring = g_pris_spring;
    jointDef.hertz = g_pris_hertz;
    jointDef.dampingRatio = g_pris_damping;
    jointDef.targetTranslation = g_pris_target_translation;
    jointDef.enableMotor = g_pris_motor;
    jointDef.maxMotorForce = g_pris_motor_force;
    jointDef.motorSpeed = g_pris_motor_speed;
    g_pris_joint = b3CreatePrismaticJoint(g_world, &jointDef);
}

bool prismatic_controls() {
    if ImGui_Checkbox("Limit", &g_pris_limit) {
        b3PrismaticJoint_EnableLimit(g_pris_joint, g_pris_limit);
        b3Joint_WakeBodies(g_pris_joint);
    }
    if g_pris_limit {
        if ImGui_SliderFloat("Lower Translation", &g_pris_lower, -10.0f, 10.0f, "%.1f", 0) {
            b3PrismaticJoint_SetLimits(g_pris_joint, g_pris_lower, g_pris_upper);
            b3Joint_WakeBodies(g_pris_joint);
        }
        if ImGui_SliderFloat("Upper Translation", &g_pris_upper, -10.0f, 10.0f, "%.1f", 0) {
            b3PrismaticJoint_SetLimits(g_pris_joint, g_pris_lower, g_pris_upper);
            b3Joint_WakeBodies(g_pris_joint);
        }
    }
    if ImGui_Checkbox("Motor", &g_pris_motor) {
        b3PrismaticJoint_EnableMotor(g_pris_joint, g_pris_motor);
        b3Joint_WakeBodies(g_pris_joint);
    }
    if g_pris_motor {
        if ImGui_SliderFloat("Max Force", &g_pris_motor_force, 0.0f, 100000.0f, "%.0f", 0) {
            b3PrismaticJoint_SetMaxMotorForce(g_pris_joint, g_pris_motor_force);
            b3Joint_WakeBodies(g_pris_joint);
        }
        if ImGui_SliderFloat("Speed", &g_pris_motor_speed, -10.0f, 10.0f, "%.0f", 0) {
            b3PrismaticJoint_SetMotorSpeed(g_pris_joint, g_pris_motor_speed);
            b3Joint_WakeBodies(g_pris_joint);
        }
    }
    if ImGui_Checkbox("Spring", &g_pris_spring) {
        b3PrismaticJoint_EnableSpring(g_pris_joint, g_pris_spring);
        b3Joint_WakeBodies(g_pris_joint);
    }
    if g_pris_spring {
        if ImGui_SliderFloat("Hertz", &g_pris_hertz, 0.0f, 10.0f, "%.1f", 0) {
            b3PrismaticJoint_SetSpringHertz(g_pris_joint, g_pris_hertz);
            b3Joint_WakeBodies(g_pris_joint);
        }
        if ImGui_SliderFloat("Damping", &g_pris_damping, 0.0f, 2.0f, "%.1f", 0) {
            b3PrismaticJoint_SetSpringDampingRatio(g_pris_joint, g_pris_damping);
            b3Joint_WakeBodies(g_pris_joint);
        }
        if ImGui_SliderFloat("Translation", &g_pris_target_translation, -20.0f, 20.0f, "%.1f", 0) {
            b3PrismaticJoint_SetTargetTranslation(g_pris_joint, g_pris_target_translation);
            b3Joint_WakeBodies(g_pris_joint);
        }
    }
    return true;
}

// samples/sample_joint.cpp SphericalJoint
b3BodyId g_sph_body;
b3JointId g_sph_joint;
float3 g_sph_target_rotation;
float3 g_sph_motor_velocity;
f32 g_sph_motor_torque = 20.0f;
f32 g_sph_hertz = 2.0f;
f32 g_sph_damping = 0.7f;
f32 g_sph_cone_deg = 30.0f;
f32 g_sph_lower_twist_deg = -35.0f;
f32 g_sph_upper_twist_deg = 35.0f;
bool g_sph_spring = true;
bool g_sph_motor;
bool g_sph_cone_limit;
bool g_sph_twist_limit;

void build_spherical_joint() {
    ignore add_ground_box(20.0f);
    b3BodyDef groundDef = b3DefaultBodyDef();
    groundDef.position = b3Pos{0.0f, -1.0f, 0.0f};
    b3BodyId groundId = b3CreateBody(g_world, &groundDef);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = b3Pos{0.0f, 4.0f, 0.0f};
    bodyDef.gravityScale = 0.0f;
    g_sph_body = b3CreateBody(g_world, &bodyDef);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.density = 100.0f;
    b3BoxHull box = b3MakeBoxHull(0.5f, 1.5f, 0.25f);
    ignore b3CreateHullShape(g_sph_body, &shapeDef, &box.base);

    b3SphericalJointDef jointDef = b3DefaultSphericalJointDef();
    jointDef.base.bodyIdA = groundId;
    jointDef.base.bodyIdB = g_sph_body;
    jointDef.base.drawScale = 2.0f;
    jointDef.base.localFrameA.p = b3Pos{0.0f, 6.5f, 0.0f};
    jointDef.base.localFrameB.p = b3Pos{0.0f, 1.5f, 0.0f};
    jointDef.enableConeLimit = g_sph_cone_limit;
    jointDef.coneAngle = B3_DEG_TO_RAD * g_sph_cone_deg;
    jointDef.enableTwistLimit = g_sph_twist_limit;
    jointDef.lowerTwistAngle = B3_DEG_TO_RAD * g_sph_lower_twist_deg;
    jointDef.upperTwistAngle = B3_DEG_TO_RAD * g_sph_upper_twist_deg;
    jointDef.enableSpring = g_sph_spring;
    jointDef.hertz = g_sph_hertz;
    jointDef.dampingRatio = g_sph_damping;
    jointDef.enableMotor = g_sph_motor;
    jointDef.maxMotorTorque = g_sph_motor_torque;
    jointDef.motorVelocity = b3Vec3{g_sph_motor_velocity.x, g_sph_motor_velocity.y,
                                    g_sph_motor_velocity.z};
    g_sph_joint = b3CreateSphericalJoint(g_world, &jointDef);
}

bool spherical_controls() {
    if ImGui_Checkbox("Cone Limit", &g_sph_cone_limit) {
        b3SphericalJoint_EnableConeLimit(g_sph_joint, g_sph_cone_limit);
        b3Joint_WakeBodies(g_sph_joint);
    }
    if g_sph_cone_limit {
        if ImGui_SliderFloat("Cone Angle", &g_sph_cone_deg, 0.0f, 90.0f, "%.0f", 0) {
            b3SphericalJoint_SetConeLimit(g_sph_joint, B3_DEG_TO_RAD * g_sph_cone_deg);
            b3Joint_WakeBodies(g_sph_joint);
        }
    }
    ImGui_Separator();
    if ImGui_Checkbox("Twist Limit", &g_sph_twist_limit) {
        b3SphericalJoint_EnableTwistLimit(g_sph_joint, g_sph_twist_limit);
        b3Joint_WakeBodies(g_sph_joint);
    }
    if g_sph_twist_limit {
        if ImGui_SliderFloat("Lower Twist", &g_sph_lower_twist_deg, -180.0f, 180.0f, "%.0f", 0) {
            b3SphericalJoint_SetTwistLimits(g_sph_joint,
                B3_DEG_TO_RAD * g_sph_lower_twist_deg, B3_DEG_TO_RAD * g_sph_upper_twist_deg);
            b3Joint_WakeBodies(g_sph_joint);
        }
        if ImGui_SliderFloat("Upper Twist", &g_sph_upper_twist_deg, -180.0f, 180.0f, "%.0f", 0) {
            b3SphericalJoint_SetTwistLimits(g_sph_joint,
                B3_DEG_TO_RAD * g_sph_lower_twist_deg, B3_DEG_TO_RAD * g_sph_upper_twist_deg);
            b3Joint_WakeBodies(g_sph_joint);
        }
    }
    if ImGui_Checkbox("Motor", &g_sph_motor) {
        b3SphericalJoint_EnableMotor(g_sph_joint, g_sph_motor);
        b3Joint_WakeBodies(g_sph_joint);
    }
    if g_sph_motor {
        if ImGui_SliderFloat("Max Torque", &g_sph_motor_torque, 0.0f, 10000.0f, "%.0f", 0) {
            b3SphericalJoint_SetMaxMotorTorque(g_sph_joint, g_sph_motor_torque);
            b3Joint_WakeBodies(g_sph_joint);
        }
        if ImGui_SliderFloat3("Velocity", &g_sph_motor_velocity.x, -10.0f, 10.0f, "%.0f", 0) {
            b3SphericalJoint_SetMotorVelocity(g_sph_joint,
                b3Vec3{g_sph_motor_velocity.x, g_sph_motor_velocity.y, g_sph_motor_velocity.z});
            b3Joint_WakeBodies(g_sph_joint);
        }
    }
    if ImGui_Checkbox("Spring", &g_sph_spring) {
        b3SphericalJoint_EnableSpring(g_sph_joint, g_sph_spring);
        b3Joint_WakeBodies(g_sph_joint);
    }
    if g_sph_spring {
        if ImGui_SliderFloat("Hertz", &g_sph_hertz, 0.0f, 10.0f, "%.1f", 0) {
            b3SphericalJoint_SetSpringHertz(g_sph_joint, g_sph_hertz);
            b3Joint_WakeBodies(g_sph_joint);
        }
        if ImGui_SliderFloat("Damping", &g_sph_damping, 0.0f, 2.0f, "%.1f", 0) {
            b3SphericalJoint_SetSpringDampingRatio(g_sph_joint, g_sph_damping);
            b3Joint_WakeBodies(g_sph_joint);
        }
        if ImGui_SliderFloat3("Rotation", &g_sph_target_rotation.x, -180.0f, 180.0f, "%.0f", 0) {
            b3Quat qx = b3MakeQuatFromAxisAngle(b3Vec3_axisX, B3_DEG_TO_RAD * g_sph_target_rotation.x);
            b3Quat qy = b3MakeQuatFromAxisAngle(b3Vec3_axisY, B3_DEG_TO_RAD * g_sph_target_rotation.y);
            b3Quat qz = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, B3_DEG_TO_RAD * g_sph_target_rotation.z);
            b3Quat q = b3MulQuat(qz, b3MulQuat(qy, qx));
            b3SphericalJoint_SetTargetRotation(g_sph_joint, q);
            b3Joint_WakeBodies(g_sph_joint);
        }
    }
    return true;
}

// samples/sample_joint.cpp ParallelJoint
b3BodyId g_par_body;
b3JointId g_par_joint;
f32 g_par_max_torque = 5000.0f;
f32 g_par_hertz = 10.0f;
f32 g_par_damping = 0.7f;

void par_wall(b3BodyId groundId, f32 hx, f32 hy, f32 hz, b3Vec3 p) {
    b3Transform transform;
    transform.p = p;
    transform.q = b3Quat_identity;
    b3BoxHull wallBox = b3MakeTransformedBoxHull(hx, hy, hz, transform);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    ignore b3CreateHullShape(groundId, &shapeDef, &wallBox.base);
}

void build_parallel_joint() {
    g_par_hertz = 10.0f;
    g_par_damping = 0.7f;
    g_par_max_torque = 5000.0f;

    ignore add_ground_box(20.0f);
    b3BodyDef groundDef = b3DefaultBodyDef();
    groundDef.position = b3Pos{0.0f, -1.0f, 0.0f};
    b3BodyId groundId = b3CreateBody(g_world, &groundDef);

    par_wall(groundId, 20.0f, 5.0f, 0.1f, b3Vec3{0.0f, 5.0f, -20.0f});
    par_wall(groundId, 20.0f, 5.0f, 0.1f, b3Vec3{0.0f, 5.0f, 20.0f});
    par_wall(groundId, 0.1f, 5.0f, 20.0f, b3Vec3{-20.0f, 5.0f, 0.0f});
    par_wall(groundId, 0.1f, 5.0f, 20.0f, b3Vec3{20.0f, 5.0f, 0.0f});

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = b3Pos{0.0f, 4.0f, 0.0f};
    bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisX, 0.25f * PI_F);
    g_par_body = b3CreateBody(g_world, &bodyDef);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    b3BoxHull box = b3MakeBoxHull(0.5f, 1.5f, 0.25f);
    ignore b3CreateHullShape(g_par_body, &shapeDef, &box.base);

    b3ParallelJointDef jointDef = b3DefaultParallelJointDef();
    jointDef.base.bodyIdA = groundId;
    jointDef.base.bodyIdB = g_par_body;
    jointDef.base.localFrameA.q = b3ComputeQuatBetweenUnitVectors(b3Vec3_axisZ, b3Vec3_axisY);
    jointDef.base.localFrameB.q = b3InvMulQuat(bodyDef.rotation, jointDef.base.localFrameA.q);
    jointDef.base.drawScale = 2.0f;
    jointDef.base.collideConnected = true;
    jointDef.hertz = g_par_hertz;
    jointDef.dampingRatio = g_par_damping;
    g_par_joint = b3CreateParallelJoint(g_world, &jointDef);
}

bool parallel_controls() {
    if ImGui_SliderFloat("Hertz", &g_par_hertz, 0.0f, 5.0f, "%.1f", 0) {
        b3ParallelJoint_SetSpringHertz(g_par_joint, g_par_hertz);
        b3Joint_WakeBodies(g_par_joint);
    }
    if ImGui_SliderFloat("Damping", &g_par_damping, 0.0f, 2.0f, "%.1f", 0) {
        b3ParallelJoint_SetSpringDampingRatio(g_par_joint, g_par_damping);
        b3Joint_WakeBodies(g_par_joint);
    }
    return true;
}

// samples/sample_joint.cpp TopDownFriction
void build_top_down_friction() {
    b3BodyDef groundDef = b3DefaultBodyDef();
    b3BodyId groundId = b3CreateBody(g_world, &groundDef);
    b3ShapeDef groundShape = b3DefaultShapeDef();
    b3BoxHull box = b3MakeTransformedBoxHull(10.0f, 0.5f, 4.0f,
        b3Transform{b3Vec3{0.0f, 0.0f, 0.0f}, b3Quat_identity});
    ignore b3CreateHullShape(groundId, &groundShape, &box.base);
    box = b3MakeTransformedBoxHull(0.5f, 10.0f, 4.0f,
        b3Transform{b3Vec3{-10.0f, 10.0f, 0.0f}, b3Quat_identity});
    ignore b3CreateHullShape(groundId, &groundShape, &box.base);
    box = b3MakeTransformedBoxHull(0.5f, 10.0f, 4.0f,
        b3Transform{b3Vec3{10.0f, 10.0f, 0.0f}, b3Quat_identity});
    ignore b3CreateHullShape(groundId, &groundShape, &box.base);
    box = b3MakeTransformedBoxHull(10.0f, 0.5f, 4.0f,
        b3Transform{b3Vec3{0.0f, 20.0f, 0.0f}, b3Quat_identity});
    ignore b3CreateHullShape(groundId, &groundShape, &box.base);

    b3MotorJointDef jointDef = b3DefaultMotorJointDef();
    jointDef.base.bodyIdA = groundId;
    jointDef.base.collideConnected = true;
    jointDef.maxVelocityForce = 1000.0f;
    jointDef.maxVelocityTorque = 1000.0f;

    b3Capsule capsule = b3Capsule{b3Pos{-0.25f, 0.0f, 0.0f}, b3Pos{0.25f, 0.0f, 0.0f}, 0.25f};
    b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, 0.35f};
    b3BoxHull cube = b3MakeBoxHull(0.35f, 0.35f, 0.35f);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.gravityScale = 0.0f;
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.baseMaterial.restitution = 0.8f;

    i32 n = 10;
    f32 x = -5.0f;
    f32 y = 15.0f;
    for i32 i = 0; i < n; i++ {
        for i32 j = 0; j < n; j++ {
            bodyDef.position = b3Pos{x, y, 0.0f};
            b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
            i32 remainder = (n * i + j) % 4;
            if remainder == 0 {
                ignore b3CreateCapsuleShape(bodyId, &shapeDef, &capsule);
            } else if remainder == 1 {
                ignore b3CreateSphereShape(bodyId, &shapeDef, &sphere);
            } else {
                ignore b3CreateHullShape(bodyId, &shapeDef, &cube.base);
            }
            jointDef.base.bodyIdB = bodyId;
            ignore b3CreateMotorJoint(g_world, &jointDef);
            x += 1.0f;
        }
        x = -5.0f;
        y -= 1.0f;
    }
}

bool top_down_friction_controls() {
    if ImGui_Button("Explode", ImVec2{0.0f, 0.0f}) {
        b3Sphere sphere = b3Sphere{b3Pos{0.0f, 10.0f, 0.0f}, 10.0f};
        b3ExplosionDef def = b3DefaultExplosionDef();
        def.position = b3Pos{0.0f, 10.0f, 0.0f};
        def.radius = sphere.radius;
        def.falloff = 5.0f;
        def.impulsePerArea = 10000.0f;
        b3World_Explode(g_world, &def);

        dbg_solid_sphere(b3WorldTransform_identity, sphere, make_color(b3_colorWhite));
    }
    return true;
}

// samples/sample_joint.cpp WheelJoint
b3BodyId g_wj_body;
b3JointId g_wj_joint;
f32 g_wj_spin_speed;
f32 g_wj_max_spin_torque = 20.0f;
f32 g_wj_suspension_hertz = 2.0f;
f32 g_wj_suspension_damping = 0.7f;
f32 g_wj_lower_translation = -1.0f;
f32 g_wj_upper_translation = 1.0f;
bool g_wj_suspension;
bool g_wj_spin_motor;
bool g_wj_suspension_limit;
bool g_wj_steering;
f32 g_wj_steering_hertz = 1.0f;
f32 g_wj_steering_damping = 0.7f;
bool g_wj_steering_limit;
f32 g_wj_lower_steering_deg = -45.0f;
f32 g_wj_upper_steering_deg = 45.0f;
f32 g_wj_max_steering_torque = 20.0f;
f32 g_wj_target_steering_deg;

void build_wheel_joint() {
    g_wj_spin_speed = 0.0f;
    g_wj_max_spin_torque = 20.0f;
    g_wj_suspension_hertz = 2.0f;
    g_wj_suspension_damping = 0.7f;
    g_wj_lower_translation = -1.0f;
    g_wj_upper_translation = 1.0f;
    g_wj_suspension = false;
    g_wj_spin_motor = false;
    g_wj_suspension_limit = false;
    g_wj_steering = false;
    g_wj_steering_hertz = 1.0f;
    g_wj_steering_damping = 0.7f;
    g_wj_steering_limit = false;
    g_wj_lower_steering_deg = -45.0f;
    g_wj_upper_steering_deg = 45.0f;
    g_wj_max_steering_torque = 20.0f;
    g_wj_target_steering_deg = 0.0f;

    ignore add_ground_box(20.0f);
    b3BodyDef groundDef = b3DefaultBodyDef();
    groundDef.position = b3Pos{0.0f, -1.0f, 0.0f};
    b3BodyId groundId = b3CreateBody(g_world, &groundDef);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = b3Pos{0.0f, 2.0f, 0.0f};
    bodyDef.rotation = b3ComputeQuatBetweenUnitVectors(b3Vec3_axisY, b3Vec3_axisZ);
    // bodyDef.gravityScale = 0.0f;
    g_wj_body = b3CreateBody(g_world, &bodyDef);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    b3HullData* hull = b3CreateCylinder(0.25f, 0.4f, 0.0f, 12);
    // b3BoxHull box = b3MakeBoxHull( 0.5f, 1.5f, 0.25f );
    ignore b3CreateHullShape(g_wj_body, &shapeDef, hull);
    b3DestroyHull(hull);

    b3WheelJointDef jointDef = b3DefaultWheelJointDef();
    jointDef.base.bodyIdA = groundId;
    jointDef.base.bodyIdB = g_wj_body;
    jointDef.base.localFrameA.p = b3Pos{0.0f, 3.0f, 0.0f};
    jointDef.base.localFrameA.q = b3ComputeQuatBetweenUnitVectors(b3Vec3_axisX, b3Vec3_axisY);
    jointDef.base.localFrameB.p = b3Pos{0.0f, 0.0f, 0.0f};
    jointDef.base.localFrameB.q = b3ComputeQuatBetweenUnitVectors(b3Vec3_axisZ, b3Vec3_axisY);
    jointDef.base.collideConnected = true;
    jointDef.enableSuspensionLimit = g_wj_suspension_limit;
    jointDef.lowerSuspensionLimit = g_wj_lower_translation;
    jointDef.upperSuspensionLimit = g_wj_upper_translation;
    jointDef.enableSuspensionSpring = g_wj_suspension;
    jointDef.suspensionHertz = g_wj_suspension_hertz;
    jointDef.suspensionDampingRatio = g_wj_suspension_damping;
    jointDef.enableSpinMotor = g_wj_spin_motor;
    jointDef.maxSpinTorque = g_wj_max_spin_torque;
    jointDef.spinSpeed = g_wj_spin_speed;
    jointDef.enableSteering = g_wj_steering;
    jointDef.steeringHertz = g_wj_steering_hertz;
    jointDef.steeringDampingRatio = g_wj_steering_damping;
    jointDef.targetSteeringAngle = PI_F / 180.0f * g_wj_target_steering_deg;
    jointDef.maxSteeringTorque = g_wj_max_steering_torque;
    jointDef.enableSteeringLimit = g_wj_steering_limit;
    jointDef.lowerSteeringLimit = PI_F / 180.0f * g_wj_lower_steering_deg;
    jointDef.upperSteeringLimit = PI_F / 180.0f * g_wj_upper_steering_deg;
    g_wj_joint = b3CreateWheelJoint(g_world, &jointDef);
}

bool wheel_controls() {
    if ImGui_Checkbox("Suspension Limit", &g_wj_suspension_limit) {
        b3WheelJoint_EnableSuspensionLimit(g_wj_joint, g_wj_suspension_limit);
        b3Joint_WakeBodies(g_wj_joint);
    }
    if g_wj_suspension_limit {
        if ImGui_SliderFloat("Min##Limit", &g_wj_lower_translation, -10.0f, 10.0f, "%.1f", 0) {
            b3WheelJoint_SetSuspensionLimits(g_wj_joint, g_wj_lower_translation,
                                             g_wj_upper_translation);
            b3Joint_WakeBodies(g_wj_joint);
        }
        if ImGui_SliderFloat("Max##Limit", &g_wj_upper_translation, -10.0f, 10.0f, "%.1f", 0) {
            b3WheelJoint_SetSuspensionLimits(g_wj_joint, g_wj_lower_translation,
                                             g_wj_upper_translation);
            b3Joint_WakeBodies(g_wj_joint);
        }
    }
    ImGui_Separator();
    if ImGui_Checkbox("Motor", &g_wj_spin_motor) {
        b3WheelJoint_EnableSpinMotor(g_wj_joint, g_wj_spin_motor);
        b3Joint_WakeBodies(g_wj_joint);
    }
    if g_wj_spin_motor {
        if ImGui_SliderFloat("Max Torque", &g_wj_max_spin_torque, 0.0f, 100.0f, "%.0f", 0) {
            b3WheelJoint_SetMaxSpinTorque(g_wj_joint, g_wj_max_spin_torque);
            b3Joint_WakeBodies(g_wj_joint);
        }
        if ImGui_SliderFloat("Speed", &g_wj_spin_speed, -10.0f, 10.0f, "%.0f", 0) {
            b3WheelJoint_SetSpinMotorSpeed(g_wj_joint, g_wj_spin_speed);
            b3Joint_WakeBodies(g_wj_joint);
        }
    }
    ImGui_Separator();
    if ImGui_Checkbox("Suspension Spring", &g_wj_suspension) {
        b3WheelJoint_EnableSuspension(g_wj_joint, g_wj_suspension);
        b3Joint_WakeBodies(g_wj_joint);
    }
    if g_wj_suspension {
        if ImGui_SliderFloat("Hertz##Suspension", &g_wj_suspension_hertz, 0.0f, 10.0f, "%.1f", 0) {
            b3WheelJoint_SetSuspensionHertz(g_wj_joint, g_wj_suspension_hertz);
            b3Joint_WakeBodies(g_wj_joint);
        }
        if ImGui_SliderFloat("Damping##Suspension", &g_wj_suspension_damping, 0.0f, 2.0f, "%.1f", 0) {
            b3WheelJoint_SetSuspensionDampingRatio(g_wj_joint, g_wj_suspension_damping);
            b3Joint_WakeBodies(g_wj_joint);
        }
    }
    ImGui_Separator();
    if ImGui_Checkbox("Steering", &g_wj_steering) {
        b3WheelJoint_EnableSteering(g_wj_joint, g_wj_steering);
        b3Joint_WakeBodies(g_wj_joint);
    }
    if g_wj_steering {
        if ImGui_SliderFloat("Hertz##Steering", &g_wj_steering_hertz, 0.0f, 10.0f, "%.1f", 0) {
            b3WheelJoint_SetSteeringHertz(g_wj_joint, g_wj_steering_hertz);
            b3Joint_WakeBodies(g_wj_joint);
        }
        // upstream sets the suspension damping ratio here
        if ImGui_SliderFloat("Damping##Steering", &g_wj_steering_damping, 0.0f, 2.0f, "%.1f", 0) {
            b3WheelJoint_SetSuspensionDampingRatio(g_wj_joint, g_wj_suspension_damping);
            b3Joint_WakeBodies(g_wj_joint);
        }
        if ImGui_SliderFloat("Degrees##Steering", &g_wj_target_steering_deg, -90.0f, 90.0f, "%.0f", 0) {
            b3WheelJoint_SetTargetSteeringAngle(g_wj_joint,
                g_wj_target_steering_deg * PI_F / 180.0f);
            b3Joint_WakeBodies(g_wj_joint);
        }
        ImGui_Separator();
        if ImGui_Checkbox("Steering Limit", &g_wj_steering_limit) {
            b3WheelJoint_EnableSteeringLimit(g_wj_joint, g_wj_steering_limit);
            b3Joint_WakeBodies(g_wj_joint);
        }
        if g_wj_steering_limit {
            if ImGui_SliderFloat("Min Degrees", &g_wj_lower_steering_deg, -90.0f, 0.0f, "%.0f", 0) {
                b3WheelJoint_SetSteeringLimits(g_wj_joint,
                    PI_F / 180.0f * g_wj_lower_steering_deg,
                    PI_F / 180.0f * g_wj_upper_steering_deg);
                b3Joint_WakeBodies(g_wj_joint);
            }
            if ImGui_SliderFloat("Max Degrees", &g_wj_upper_steering_deg, 0.0f, 90.0f, "%.0f", 0) {
                b3WheelJoint_SetSteeringLimits(g_wj_joint,
                    PI_F / 180.0f * g_wj_lower_steering_deg,
                    PI_F / 180.0f * g_wj_upper_steering_deg);
                b3Joint_WakeBodies(g_wj_joint);
            }
        }
    }
    return true;
}

// samples/sample_joint.cpp MotionLocks
const i32 ML_CAPACITY = 6;
b3BodyId[ML_CAPACITY] g_ml_bodies;
i32 g_ml_count;
b3MotionLocks g_ml_locks;

void ml_apply_locks() {
    for i32 i = 0; i < g_ml_count; i++ {
        b3Body_SetMotionLocks(g_ml_bodies[i], g_ml_locks);
        b3Body_SetAwake(g_ml_bodies[i], true);
    }
}

void build_motion_locks() {
    ignore add_ground_box(20.0f);
    b3BodyDef groundDef = b3DefaultBodyDef();
    b3BodyId groundId = b3CreateBody(g_world, &groundDef);

    g_ml_locks = b3MotionLocks{};

    b3Pos position = b3Pos{-12.5f, 10.0f, 0.0f};
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.enableSleep = false;
    b3BoxHull box = b3MakeBoxHull(1.0f, 1.0f, 0.5f);
    g_ml_count = 0;
    f32 forceThreshold = 20000.0f;
    f32 torqueThreshold = 10000.0f;
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.density = 1.0f;

    // distance joint
    {
        bodyDef.position = position;
        g_ml_bodies[g_ml_count] = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateHullShape(g_ml_bodies[g_ml_count], &shapeDef, &box.base);
        f32 length = 2.0f;
        b3Pos pivot1 = b3Pos{position.x, position.y + 1.0f + length, 0.0f};
        b3Pos pivot2 = b3Pos{position.x, position.y + 1.0f, 0.0f};
        b3DistanceJointDef jointDef = b3DefaultDistanceJointDef();
        jointDef.base.bodyIdA = groundId;
        jointDef.base.bodyIdB = g_ml_bodies[g_ml_count];
        jointDef.base.localFrameA.p = b3Body_GetLocalPoint(jointDef.base.bodyIdA, pivot1);
        jointDef.base.localFrameB.p = b3Body_GetLocalPoint(jointDef.base.bodyIdB, pivot2);
        jointDef.length = length;
        jointDef.base.forceThreshold = forceThreshold;
        jointDef.base.torqueThreshold = torqueThreshold;
        jointDef.base.collideConnected = true;
        ignore b3CreateDistanceJoint(g_world, &jointDef);
    }
    position.x += 5.0f;
    g_ml_count++;

    // motor joint
    // (upstream #if 0)

    // prismatic joint
    {
        bodyDef.position = position;
        g_ml_bodies[g_ml_count] = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateHullShape(g_ml_bodies[g_ml_count], &shapeDef, &box.base);
        b3Pos pivot = b3Pos{position.x - 1.0f, position.y, 0.0f};
        b3PrismaticJointDef jointDef = b3DefaultPrismaticJointDef();
        jointDef.base.bodyIdA = groundId;
        jointDef.base.bodyIdB = g_ml_bodies[g_ml_count];
        jointDef.base.localFrameA.p = b3Body_GetLocalPoint(jointDef.base.bodyIdA, pivot);
        jointDef.base.localFrameB.p = b3Body_GetLocalPoint(jointDef.base.bodyIdB, pivot);
        jointDef.base.forceThreshold = forceThreshold;
        jointDef.base.torqueThreshold = torqueThreshold;
        jointDef.base.collideConnected = true;
        ignore b3CreatePrismaticJoint(g_world, &jointDef);
    }
    position.x += 5.0f;
    g_ml_count++;

    // revolute joint
    {
        bodyDef.position = position;
        g_ml_bodies[g_ml_count] = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateHullShape(g_ml_bodies[g_ml_count], &shapeDef, &box.base);
        b3Pos pivot = b3Pos{position.x - 1.0f, position.y, 0.0f};
        b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
        jointDef.base.bodyIdA = groundId;
        jointDef.base.bodyIdB = g_ml_bodies[g_ml_count];
        jointDef.base.localFrameA.p = b3Body_GetLocalPoint(jointDef.base.bodyIdA, pivot);
        jointDef.base.localFrameB.p = b3Body_GetLocalPoint(jointDef.base.bodyIdB, pivot);
        jointDef.base.forceThreshold = forceThreshold;
        jointDef.base.torqueThreshold = torqueThreshold;
        jointDef.base.collideConnected = true;
        ignore b3CreateRevoluteJoint(g_world, &jointDef);
    }
    position.x += 5.0f;
    g_ml_count++;

    // weld joint
    {
        bodyDef.position = position;
        g_ml_bodies[g_ml_count] = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateHullShape(g_ml_bodies[g_ml_count], &shapeDef, &box.base);
        b3Pos pivot = b3Pos{position.x - 1.0f, position.y, 0.0f};
        b3WeldJointDef jointDef = b3DefaultWeldJointDef();
        jointDef.base.bodyIdA = groundId;
        jointDef.base.bodyIdB = g_ml_bodies[g_ml_count];
        jointDef.base.localFrameA.p = b3Body_GetLocalPoint(jointDef.base.bodyIdA, pivot);
        jointDef.base.localFrameB.p = b3Body_GetLocalPoint(jointDef.base.bodyIdB, pivot);
        jointDef.angularHertz = 2.0f;
        jointDef.angularDampingRatio = 0.5f;
        jointDef.base.forceThreshold = forceThreshold;
        jointDef.base.torqueThreshold = torqueThreshold;
        jointDef.base.collideConnected = true;
        ignore b3CreateWeldJoint(g_world, &jointDef);
    }
    position.x += 5.0f;
    g_ml_count++;

    // wheel joint
    // (upstream #if 0)
}

bool motion_locks_controls() {
    if ImGui_Checkbox("Lock Linear X", &g_ml_locks.linearX) { ml_apply_locks(); }
    if ImGui_Checkbox("Lock Linear Y", &g_ml_locks.linearY) { ml_apply_locks(); }
    if ImGui_Checkbox("Lock Linear Z", &g_ml_locks.linearZ) { ml_apply_locks(); }
    if ImGui_Checkbox("Lock Angular X", &g_ml_locks.angularX) { ml_apply_locks(); }
    if ImGui_Checkbox("Lock Angular Y", &g_ml_locks.angularY) { ml_apply_locks(); }
    if ImGui_Checkbox("Lock Angular Z", &g_ml_locks.angularZ) { ml_apply_locks(); }
    if ImGui_IsKeyDown(ImGuiKey_L) {
        b3Body_ApplyLinearImpulseToCenter(g_ml_bodies[0], b3Vec3{100.0f, 0.0f, 0.0f}, true);
    }
    return true;
}

// samples/sample_joint.cpp Door
b3BodyId g_door_ground;
b3BodyId g_door_body;
b3JointId g_door_joint1;
b3JointId g_door_joint2;
bool g_door_joint1_valid;
bool g_door_joint2_valid;
f32 g_door_magnitude = 50000.0f;
f32 g_door_error1;
f32 g_door_error2;
f32 g_door_constraint_hertz = 120.0f;
f32 g_door_constraint_damping;
bool g_door_enable_limit = true;
bool g_door_two_joints = true;

void door_create_joints() {
    if g_door_joint1_valid {
        b3DestroyJoint(g_door_joint1, false);
        g_door_joint1_valid = false;
    }
    if g_door_joint2_valid {
        b3DestroyJoint(g_door_joint2, false);
        g_door_joint2_valid = false;
    }
    b3Quat axisQuat = b3ComputeQuatBetweenUnitVectors(b3Vec3_axisZ, b3Vec3_axisY);
    {
        b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
        jointDef.base.bodyIdA = g_door_ground;
        jointDef.base.bodyIdB = g_door_body;
        jointDef.base.localFrameA.p = b3Pos{-0.75f, 1.0f, 0.0f};
        jointDef.base.localFrameA.q = axisQuat;
        jointDef.base.localFrameB.p = b3Pos{-0.75f, -1.5f, 0.0f};
        jointDef.base.localFrameB.q = axisQuat;
        jointDef.base.constraintHertz = g_door_constraint_hertz;
        jointDef.base.constraintDampingRatio = g_door_constraint_damping;
        jointDef.enableLimit = true;
        jointDef.lowerAngle = B3_DEG_TO_RAD * -90.0f;
        jointDef.upperAngle = B3_DEG_TO_RAD * 90.0f;
        jointDef.enableSpring = true;
        jointDef.hertz = 1.0f;
        jointDef.dampingRatio = 0.5f;
        jointDef.enableMotor = false;
        jointDef.maxMotorTorque = 100.0f;
        jointDef.motorSpeed = 0.0f;
        jointDef.base.drawScale = 2.0f;
        g_door_joint1 = b3CreateRevoluteJoint(g_world, &jointDef);
        g_door_joint1_valid = true;
    }
    if g_door_two_joints {
        b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
        jointDef.base.bodyIdA = g_door_ground;
        jointDef.base.bodyIdB = g_door_body;
        jointDef.base.localFrameA.p = b3Pos{-0.75f, 4.0f, 0.0f};
        jointDef.base.localFrameA.q = axisQuat;
        jointDef.base.localFrameB.p = b3Pos{-0.75f, 1.5f, 0.0f};
        jointDef.base.localFrameB.q = axisQuat;
        jointDef.base.constraintHertz = g_door_constraint_hertz;
        jointDef.base.constraintDampingRatio = g_door_constraint_damping;
        jointDef.enableLimit = true;
        jointDef.lowerAngle = B3_DEG_TO_RAD * -90.0f;
        jointDef.upperAngle = B3_DEG_TO_RAD * 90.0f;
        jointDef.enableSpring = true;
        jointDef.hertz = 1.0f;
        jointDef.dampingRatio = 0.5f;
        jointDef.enableMotor = false;
        jointDef.maxMotorTorque = 100.0f;
        jointDef.motorSpeed = 0.0f;
        jointDef.base.drawScale = 2.0f;
        g_door_joint2 = b3CreateRevoluteJoint(g_world, &jointDef);
        g_door_joint2_valid = true;
    }
}

void build_door() {
    g_door_ground = add_ground_box(20.0f);
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;
        bodyDef.position = b3Pos{0.0f, 1.5f, 0.0f};
        bodyDef.gravityScale = 2.0f;
        g_door_body = b3CreateBody(g_world, &bodyDef);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        shapeDef.density = 1000.0f;
        b3BoxHull box = b3MakeBoxHull(0.75f, 1.5f, 0.1f);
        ignore b3CreateHullShape(g_door_body, &shapeDef, &box.base);
    }
    //{
    //	b3BoxHull cube = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
    //	b3BodyDef bodyDef = b3DefaultBodyDef();
    //	bodyDef.name = "cube";
    //	bodyDef.type = b3_dynamicBody;
    //	bodyDef.position = { 0.0f, 2.0f, 0.0f };
    //	bodyDef.gravityScale = 2.0f;
    //	b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
    //	b3ShapeDef shapeDef = b3DefaultShapeDef();
    //	b3CreateHullShape( bodyId, &shapeDef, &cube.base );
    //}
    g_door_magnitude = 50000.0f;
    g_door_two_joints = true;
    g_door_constraint_hertz = 120.0f;
    g_door_constraint_damping = 0.0f;
    g_door_enable_limit = true;
    g_door_error1 = 0.0f;
    g_door_error2 = 0.0f;
    g_door_joint1_valid = false;
    g_door_joint2_valid = false;
    door_create_joints();
}

bool door_mouse_down(f32 px, f32 py, i32 button, i32 modifiers) {
    if button == 0 && modifiers == 2 {
        PickRay pickRay = build_pick_ray(px, py);
        b3RayResult result = b3World_CastRayClosest(g_world,
            b3Pos{pickRay.origin.x, pickRay.origin.y, pickRay.origin.z},
            b3Vec3{pickRay.translation.x, pickRay.translation.y, pickRay.translation.z},
            b3DefaultQueryFilter());
        if result.hit {
            b3BodyId bodyId = b3Shape_GetBody(result.shapeId);
            b3Vec3 dir = b3Normalize(b3Vec3{pickRay.translation.x, pickRay.translation.y,
                                            pickRay.translation.z});
            b3Vec3 impulse = b3Vec3{g_door_magnitude * dir.x, g_door_magnitude * dir.y,
                                    g_door_magnitude * dir.z};
            b3Body_ApplyLinearImpulse(bodyId, impulse, result.point, true);
        }
        return true;
    }
    return false;
}

void step_door(f32 timeStep) {
    ignore timeStep;
    b3Pos p = b3Body_GetWorldPoint(g_door_body, b3Vec3{0.75f, 0.0f, 0.0f});
    adapter_point(p, 10.0f, b3_colorDarkKhaki, null);
    f32 translationError1 = b3Joint_GetLinearSeparation(g_door_joint1);
    if translationError1 > g_door_error1 { g_door_error1 = translationError1; }
    u8[128] buf;
    ignore snprintf(cast(u8*, &buf), 128, "translation error 1 = %.6f", g_door_error1);
    draw_text_line(cast(u8*, &buf));
    if g_door_joint2_valid {
        f32 translationError2 = b3Joint_GetLinearSeparation(g_door_joint2);
        if translationError2 > g_door_error2 { g_door_error2 = translationError2; }
        ignore snprintf(cast(u8*, &buf), 128, "translation error 2 = %.6f", g_door_error2);
        draw_text_line(cast(u8*, &buf));
    }
}

bool door_controls() {
    ImGui_PushItemWidth(6.0f * ImGui_GetFontSize());
    if ImGui_Button("Impulse##Door", ImVec2{0.0f, 0.0f}) {
        b3Pos p = b3Body_GetWorldPoint(g_door_body, b3Vec3{0.75f, 0.0f, 0.0f});
        b3Body_ApplyLinearImpulse(g_door_body,
            b3Vec3{0.0f, 0.0f, 0.0f - g_door_magnitude}, p, true);
        g_door_error1 = 0.0f;
        g_door_error2 = 0.0f;
    }
    ignore ImGui_SliderFloat("Magnitude##Door", &g_door_magnitude, 1000.0f, 100000.0f, "%.0f", 0);
    if ImGui_Checkbox("Limit##Door", &g_door_enable_limit) {
        b3RevoluteJoint_EnableLimit(g_door_joint1, g_door_enable_limit);
        if g_door_joint2_valid {
            b3RevoluteJoint_EnableLimit(g_door_joint2, g_door_enable_limit);
        }
    }
    if ImGui_Checkbox("Two joints##Door", &g_door_two_joints) {
        door_create_joints();
    }
    if ImGui_SliderFloat("Hertz##Door", &g_door_constraint_hertz, 15.0f, 240.0f, "%.0f", 0) {
        b3Joint_SetConstraintTuning(g_door_joint1, g_door_constraint_hertz,
                                    g_door_constraint_damping);
        if g_door_joint2_valid {
            b3Joint_SetConstraintTuning(g_door_joint2, g_door_constraint_hertz,
                                        g_door_constraint_damping);
        }
    }
    if ImGui_SliderFloat("Damping##Door", &g_door_constraint_damping, 0.0f, 10.0f, "%.1f", 0) {
        b3Joint_SetConstraintTuning(g_door_joint1, g_door_constraint_hertz,
                                    g_door_constraint_damping);
        if g_door_joint2_valid {
            b3Joint_SetConstraintTuning(g_door_joint2, g_door_constraint_hertz,
                                        g_door_constraint_damping);
        }
    }
    ImGui_PopItemWidth();
    return true;
}

// samples/sample_joint.cpp GearLift
const f32 GL_GEAR_RADIUS = 1.0f;
const f32 GL_GEAR_HALF_DEPTH = 0.125f; // 25 cm gear depth
const f32 GL_GEAR_Z = 1.5f;            // near and far gear planes
const f32 GL_AXLE_RADIUS = 0.2f;
const f32 GL_TOOTH_HALF_WIDTH = 0.11f;
const f32 GL_TOOTH_HALF_HEIGHT = 0.09f;
const f32 GL_TOOTH_RADIUS = 0.03f;
const f32 GL_LINK_HALF_LENGTH = 0.07f;
const f32 GL_LINK_RADIUS = 0.05f;
const i32 GL_LINK_COUNT = 40;
const f32 GL_DOOR_HALF_HEIGHT = 1.5f;
const f32 GL_DOOR_HALF_DEPTH = 1.95f;
const i32 GL_GEAR_SIDES = 24;
const i32 GL_AXLE_SIDES = 12;
const f32 GL_ROCK_RADIUS = 0.3f;

const i32 GL_POINT_COUNT = 32;
const i32 GL_CAP_INDEX_COUNT = 3 * (GL_POINT_COUNT - 2);
const i32 GL_INDEX_CAPACITY = 6 * GL_POINT_COUNT + 2 * GL_CAP_INDEX_COUNT;

b3MeshData* g_gl_mesh;
b3JointId g_gl_driver;
f32 g_gl_motor_torque;
f32 g_gl_motor_speed;
bool g_gl_enable_motor;

f32 gl_cross2(b3Vec2 a, b3Vec2 b, b3Vec2 c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool gl_in_triangle(b3Vec2 a, b3Vec2 b, b3Vec2 c, b3Vec2 p) {
    f32 d1 = gl_cross2(a, b, p);
    f32 d2 = gl_cross2(b, c, p);
    f32 d3 = gl_cross2(c, a, p);
    bool hasNeg = d1 < 0.0f || d2 < 0.0f || d3 < 0.0f;
    bool hasPos = d1 > 0.0f || d2 > 0.0f || d3 > 0.0f;
    return !(hasNeg && hasPos);
}

// Ear clipping in place of mapbox::earcut, which is C++ only. Any valid
// triangulation of the silhouette gives the same solid.
i32 gl_triangulate(b3Vec2* poly, i32 n, i32* out) {
    i32[GL_POINT_COUNT] next;
    i32[GL_POINT_COUNT] prev;
    for i32 i = 0; i < n; i += 1 {
        next[i] = (i + 1) % n;
        prev[i] = (i + n - 1) % n;
    }

    f32 area = 0.0f;
    for i32 i = 0; i < n; i += 1 {
        i32 j = (i + 1) % n;
        area += poly[i].x * poly[j].y - poly[j].x * poly[i].y;
    }
    f32 sign = area >= 0.0f ? 1.0f : -1.0f;

    i32 count = 0;
    i32 remaining = n;
    i32 v = 0;
    i32 misses = 0;
    while remaining > 2 && misses <= remaining {
        i32 a = prev[v];
        i32 c = next[v];
        bool ear = sign * gl_cross2(poly[a], poly[v], poly[c]) > 0.0f;
        i32 k = next[c];
        while ear && k != a {
            if gl_in_triangle(poly[a], poly[v], poly[c], poly[k]) { ear = false; }
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
    return count;
}

// Push one cap triangle, flipping the winding so its z-normal has the wanted sign.
i32 gl_push_cap(i32* indices, i32 count, b3Vec2* poly, i32 r0, i32 r1, i32 r2,
                i32 vOffset, bool wantPositiveZ) {
    f32 cross = (poly[r1].x - poly[r0].x) * (poly[r2].y - poly[r0].y)
              - (poly[r1].y - poly[r0].y) * (poly[r2].x - poly[r0].x);
    bool positive = cross > 0.0f;

    i32 v0 = 2 * r0 + vOffset;
    i32 v1 = 2 * r1 + vOffset;
    i32 v2 = 2 * r2 + vOffset;

    indices[count + 0] = v0;
    if positive == wantPositiveZ {
        indices[count + 1] = v1;
        indices[count + 2] = v2;
    } else {
        indices[count + 1] = v2;
        indices[count + 2] = v1;
    }
    return count + 3;
}

void gl_create_mesh(b3BodyId groundId) {
    // Silhouette of the Box2D stairwell traced as a closed loop. It is extruded
    // four meters along z into a single triangle mesh, the 3D analogue of a
    // b2Chain loop. The solid is inside the loop, so the windings face outward
    // to put the collision normals on the open basin side where the debris sits.
    b3Vec2[GL_POINT_COUNT] points = {
        b3Vec2{-11.3000f, -0.2167f}, b3Vec2{9.3375f, -0.2167f},  b3Vec2{9.3375f, 7.1917f},  b3Vec2{8.8083f, 7.1917f},  b3Vec2{8.8083f, 0.3125f},
        b3Vec2{0.3417f, 0.3125f},    b3Vec2{0.3417f, 0.8417f},   b3Vec2{-0.1875f, 0.8417f}, b3Vec2{-0.1875f, 1.3708f}, b3Vec2{-0.7167f, 1.3708f},
        b3Vec2{-0.7167f, 1.9000f},   b3Vec2{-1.2458f, 1.9000f},  b3Vec2{-1.2458f, 2.4292f}, b3Vec2{-1.7750f, 2.4292f}, b3Vec2{-1.7750f, 2.9583f},
        b3Vec2{-2.3042f, 2.9583f},   b3Vec2{-2.3042f, 3.4875f},  b3Vec2{-2.8333f, 3.4875f}, b3Vec2{-2.8333f, 4.0167f}, b3Vec2{-3.3625f, 4.0167f},
        b3Vec2{-3.3625f, 4.5458f},   b3Vec2{-3.8917f, 4.5458f},  b3Vec2{-3.8917f, 5.0750f}, b3Vec2{-4.4208f, 5.0750f}, b3Vec2{-4.4208f, 5.6042f},
        b3Vec2{-4.9500f, 5.6042f},   b3Vec2{-4.9500f, 6.1333f},  b3Vec2{-5.4792f, 6.1333f}, b3Vec2{-5.4792f, 6.6625f}, b3Vec2{-6.0083f, 6.6625f},
        b3Vec2{-6.0083f, 7.1917f},   b3Vec2{-11.3000f, 7.1917f},
    };

    f32 zMin = -2.0f; // four meters across z
    f32 zMax = 2.0f;

    // Two vertices per silhouette point, one at each depth.
    b3Vec3[2 * GL_POINT_COUNT] vertices;
    for i32 i = 0; i < GL_POINT_COUNT; i += 1 {
        vertices[2 * i + 0] = b3Vec3{points[i].x, points[i].y, zMin};
        vertices[2 * i + 1] = b3Vec3{points[i].x, points[i].y, zMax};
    }

    i32[GL_INDEX_CAPACITY] indices;
    i32 indexCount = 0;

    // Side walls: two triangles per silhouette edge, wound so the normal faces
    // the open basin. The lo ring sits at zMin, the hi ring at zMax.
    for i32 i = 0; i < GL_POINT_COUNT; i += 1 {
        i32 j = (i + 1) % GL_POINT_COUNT;
        i32 aLo = 2 * i;
        i32 aHi = 2 * i + 1;
        i32 bLo = 2 * j;
        i32 bHi = 2 * j + 1;

        indices[indexCount + 0] = aLo;
        indices[indexCount + 1] = bLo;
        indices[indexCount + 2] = bHi;
        indexCount += 3;

        indices[indexCount + 0] = aLo;
        indices[indexCount + 1] = bHi;
        indices[indexCount + 2] = aHi;
        indexCount += 3;
    }

    // End caps close the prism into a watertight manifold with no boundary edges.
    // The non convex silhouette is triangulated with earcut, then each triangle is
    // wound so the cap normal points out of the solid: the zMin cap faces -z and
    // the zMax cap faces +z. The two ring edges each gain a second owner this way.
    i32[GL_CAP_INDEX_COUNT] cap;
    i32 capCount = gl_triangulate(cast(b3Vec2*, &points), GL_POINT_COUNT, cast(i32*, &cap));

    for i32 k = 0; k + 3 <= capCount; k += 3 {
        i32 r0 = cap[k];
        i32 r1 = cap[k + 1];
        i32 r2 = cap[k + 2];
        indexCount = gl_push_cap(cast(i32*, &indices), indexCount, cast(b3Vec2*, &points),
                                 r0, r1, r2, 1, true);  // zMax cap, +z out
        indexCount = gl_push_cap(cast(i32*, &indices), indexCount, cast(b3Vec2*, &points),
                                 r0, r1, r2, 0, false); // zMin cap, -z out
    }

    b3MeshDef def = b3MeshDef{};
    def.vertices = cast(b3Vec3*, &vertices);
    def.vertexCount = 2 * GL_POINT_COUNT;
    def.indices = cast(i32*, &indices);
    def.triangleCount = indexCount / 3;
    def.identifyEdges = true;

    g_gl_mesh = b3CreateMesh(&def, null, 0);

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.baseMaterial.customColor = b3_colorDarkSeaGreen;
    b3SurfaceMaterial material = shapeDef.baseMaterial;
    shapeDef.materials = &material;
    shapeDef.materialCount = 1;
    ignore b3CreateMeshShape(groundId, &shapeDef, g_gl_mesh, b3Vec3_one);

    // Back wall: a 0.1 m thick box closing the far side over the full mesh extent.
    b3Vec2 lower = points[0];
    b3Vec2 upper = points[0];
    for i32 i = 1; i < GL_POINT_COUNT; i += 1 {
        lower.x = b3MinFloat(lower.x, points[i].x);
        lower.y = b3MinFloat(lower.y, points[i].y);
        upper.x = b3MaxFloat(upper.x, points[i].x);
        upper.y = b3MaxFloat(upper.y, points[i].y);
    }

    f32 wallHalfThick = 0.05f;
    b3Vec3 wallCenter = b3Vec3{0.5f * (lower.x + upper.x), 0.5f * (lower.y + upper.y),
                               -zMax - wallHalfThick};
    b3BoxHull wall = b3MakeOffsetBoxHull(0.5f * (upper.x - lower.x),
                                         0.5f * (upper.y - lower.y), wallHalfThick, wallCenter);
    ignore b3CreateHullShape(groundId, &shapeDef, &wall.base);
}

// A cylinder built directly along z (depth) so it needs no reorientation.
b3HullData* gl_make_z_cylinder(f32 radius, f32 zMin, f32 zMax, i32 sides) {
    b3Vec3[64] points;
    for i32 i = 0; i < sides; i += 1 {
        f32 angle = (2.0f * PI_F * cast(f32, i)) / cast(f32, sides);
        f32 c = cosf(angle);
        f32 s = sinf(angle);
        points[2 * i + 0] = b3Vec3{radius * c, radius * s, zMin};
        points[2 * i + 1] = b3Vec3{radius * c, radius * s, zMax};
    }

    return b3CreateHull(cast(b3Vec3*, &points), 2 * sides, 2 * sides);
}

// Ring of 16 teeth around the gear rim at depth zCenter, in the body frame. Box3D
// has no rounded hulls, so each tooth tapers tangentially toward the tip to clear
// the meshing teeth the way Box2D's rounded teeth do.
void gl_add_teeth(b3BodyId bodyId, f32 centerRadius, f32 zCenter) {
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.baseMaterial.friction = 0.1f;
    shapeDef.baseMaterial.customColor = b3_colorGray;

    i32 count = 16;
    f32 deltaAngle = 2.0f * PI_F / cast(f32, count);

    f32 hx = GL_TOOTH_HALF_WIDTH;                         // radial half extent
    f32 hz = GL_GEAR_HALF_DEPTH;                          // depth half extent
    f32 baseHalf = GL_TOOTH_HALF_HEIGHT;                  // tangential half width at the base
    f32 tipHalf = GL_TOOTH_HALF_HEIGHT - GL_TOOTH_RADIUS; // narrower at the tip

    for i32 i = 0; i < count; i += 1 {
        b3Quat q = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, cast(f32, i) * deltaAngle);
        b3Vec3 center = b3RotateVector(q, b3Vec3{centerRadius, 0.0f, 0.0f});
        center.z = zCenter;

        // Base at the inner radius, narrower tip at the outer radius.
        b3Vec3[8] local = {
            b3Vec3{-hx, -baseHalf, -hz}, b3Vec3{-hx, baseHalf, -hz}, b3Vec3{-hx, baseHalf, hz}, b3Vec3{-hx, -baseHalf, hz},
            b3Vec3{hx, -tipHalf, -hz},   b3Vec3{hx, tipHalf, -hz},   b3Vec3{hx, tipHalf, hz},   b3Vec3{hx, -tipHalf, hz},
        };

        b3Vec3[8] points;
        for i32 k = 0; k < 8; k += 1 {
            points[k] = b3Add(center, b3RotateVector(q, local[k]));
        }

        b3HullData* tooth = b3CreateHull(cast(b3Vec3*, &points), 8, 8);
        ignore b3CreateHullShape(bodyId, &shapeDef, tooth);
        b3DestroyHull(tooth);
    }
}

// One gear shaft: a disk and tooth ring at each depth joined by an axle, all on
// one rigid body so the two depth gears share an angle and read as a clean gear.
b3BodyId gl_build_gear_body(b3Pos position, f32 toothCenterRadius, b3HullData* diskNear,
                            b3HullData* diskFar, b3HullData* axle) {
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = position;
    b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.baseMaterial.friction = 0.1f;
    shapeDef.baseMaterial.customColor = b3_colorSaddleBrown;
    ignore b3CreateHullShape(bodyId, &shapeDef, diskNear);
    ignore b3CreateHullShape(bodyId, &shapeDef, diskFar);

    shapeDef.baseMaterial.customColor = b3_colorSlateGray;
    ignore b3CreateHullShape(bodyId, &shapeDef, axle);

    gl_add_teeth(bodyId, toothCenterRadius, -GL_GEAR_Z);
    gl_add_teeth(bodyId, toothCenterRadius, GL_GEAR_Z);

    return bodyId;
}

// Chain of capsule links hanging from a body down to the gate. Returns the last link.
b3BodyId gl_create_chain(b3BodyId topBodyId, b3Pos attach) {
    b3Capsule capsule = b3Capsule{b3Pos{0.0f, -GL_LINK_HALF_LENGTH, 0.0f},
                                  b3Pos{0.0f, GL_LINK_HALF_LENGTH, 0.0f}, GL_LINK_RADIUS};

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.baseMaterial.customColor = b3_colorLightSteelBlue;

    b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
    jointDef.maxMotorTorque = 0.05f;
    jointDef.enableMotor = true;
    jointDef.base.drawScale = 0.2f;

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    b3Pos position = b3Pos{attach.x, attach.y - GL_LINK_HALF_LENGTH, attach.z};

    b3BodyId prevBodyId = topBodyId;
    for i32 i = 0; i < GL_LINK_COUNT; i += 1 {
        bodyDef.position = position;
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateCapsuleShape(bodyId, &shapeDef, &capsule);

        b3Pos pivot = b3Pos{position.x, position.y + GL_LINK_HALF_LENGTH, attach.z};
        jointDef.base.bodyIdA = prevBodyId;
        jointDef.base.bodyIdB = bodyId;
        jointDef.base.localFrameA.p = b3Body_GetLocalPoint(prevBodyId, pivot);
        jointDef.base.localFrameB.p = b3Body_GetLocalPoint(bodyId, pivot);
        ignore b3CreateRevoluteJoint(g_world, &jointDef);

        position.y -= 2.0f * GL_LINK_HALF_LENGTH;
        prevBodyId = bodyId;
    }

    return prevBodyId;
}

// A single gate box raised by both chains and held upright sliding along y.
void gl_create_door(b3BodyId groundId, b3Pos doorPosition, b3BodyId nearLink, b3BodyId farLink) {
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = doorPosition;
    b3BodyId doorId = b3CreateBody(g_world, &bodyDef);

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.density *= 0.5f;
    shapeDef.baseMaterial.friction = 0.1f;
    shapeDef.baseMaterial.customColor = cast(b3HexColor,
        b3MakeDebugColor(b3_colorDarkCyan, b3_debugMaterialMetallic));
    b3BoxHull box = b3MakeBoxHull(0.05f, GL_DOOR_HALF_HEIGHT, GL_DOOR_HALF_DEPTH);
    ignore b3CreateHullShape(doorId, &shapeDef, &box.base);

    // Hinge the gate to each chain at its depth.
    b3BodyId[2] links;
    links[0] = nearLink;
    links[1] = farLink;
    f32[2] depths = { -GL_GEAR_Z, GL_GEAR_Z };
    for i32 i = 0; i < 2; i += 1 {
        b3Pos pivot = b3Pos{doorPosition.x, doorPosition.y + GL_DOOR_HALF_HEIGHT, depths[i]};
        b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
        jointDef.base.bodyIdA = links[i];
        jointDef.base.bodyIdB = doorId;
        jointDef.base.localFrameA.p = b3Body_GetLocalPoint(links[i], pivot);
        jointDef.base.localFrameB.p = b3Pos{0.0f, GL_DOOR_HALF_HEIGHT, depths[i]};
        jointDef.enableMotor = true;
        jointDef.maxMotorTorque = 50.0f;
        ignore b3CreateRevoluteJoint(g_world, &jointDef);
    }

    // Prismatic axis is frame A local x, so rotate it onto world y.
    b3Quat slideAxis = b3ComputeQuatBetweenUnitVectors(b3Vec3_axisX, b3Vec3_axisY);
    b3PrismaticJointDef jointDef = b3DefaultPrismaticJointDef();
    jointDef.base.bodyIdA = groundId;
    jointDef.base.bodyIdB = doorId;
    jointDef.base.localFrameA.p = b3Body_GetLocalPoint(groundId, doorPosition);
    jointDef.base.localFrameA.q = slideAxis;
    jointDef.base.localFrameB.p = b3Pos{0.0f, 0.0f, 0.0f};
    jointDef.base.localFrameB.q = slideAxis;
    jointDef.maxMotorForce = 200.0f;
    jointDef.enableMotor = true;
    jointDef.base.collideConnected = true;
    ignore b3CreatePrismaticJoint(g_world, &jointDef);
}

void gl_create_debris() {
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.baseMaterial.rollingResistance = 0.3f;

    b3HexColor[5] colors = {
        b3_colorGray, b3_colorGainsboro, b3_colorLightGray, b3_colorLightSlateGray, b3_colorDarkGray,
    };

    b3HullData* rockHull = b3CreateRock(GL_ROCK_RADIUS);

    f32 x = -5.0f;
    i32 xCount = 12;
    i32 yCount = 10;
    for i32 i = 0; i < xCount; i += 1 {
        f32 y = 6.5f - 0.25f * cast(f32, i);
        for i32 j = 0; j < yCount; j += 1 {
            // Spread the debris across the depth of the stairwell.
            bodyDef.position = b3Pos{x, y, random_float_range(-1.65f, 0.35f)};
            bodyDef.rotation = random_quat();
            b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);

            shapeDef.baseMaterial.customColor = colors[random_int_range(0, 4)];
            ignore b3CreateHullShape(bodyId, &shapeDef, rockHull);

            y += 0.2f;
        }
        x += 0.3f;
    }

    b3DestroyHull(rockHull);
}

void build_gear_lift() {
    ignore add_ground_box(20.0f);

    b3BodyId groundId;
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        groundId = b3CreateBody(g_world, &bodyDef);
    }

    gl_create_mesh(groundId);

    // Gear cores shared by both shafts: a disk at each depth with an axle bridging
    // them, all cylinders along z so the gears spin about z.
    b3HullData* diskNear = gl_make_z_cylinder(GL_GEAR_RADIUS, -GL_GEAR_Z - GL_GEAR_HALF_DEPTH,
                                              -GL_GEAR_Z + GL_GEAR_HALF_DEPTH, GL_GEAR_SIDES);
    b3HullData* diskFar = gl_make_z_cylinder(GL_GEAR_RADIUS, GL_GEAR_Z - GL_GEAR_HALF_DEPTH,
                                             GL_GEAR_Z + GL_GEAR_HALF_DEPTH, GL_GEAR_SIDES);
    b3HullData* axle = gl_make_z_cylinder(GL_AXLE_RADIUS, -GL_GEAR_Z, GL_GEAR_Z, GL_AXLE_SIDES);

    g_gl_motor_torque = 30000.0f;
    g_gl_motor_speed = -0.3f;
    g_gl_enable_motor = true;

    b3Pos gearPosition1 = b3Pos{-4.25f, 9.75f, 0.0f};
    b3Pos gearPosition2 = b3Pos{-2.25f, 10.75f, 0.0f};

    b3BodyId driverId = gl_build_gear_body(gearPosition1, GL_GEAR_RADIUS + GL_TOOTH_HALF_HEIGHT,
                                           diskNear, diskFar, axle);
    b3BodyId followerId = gl_build_gear_body(gearPosition2, GL_GEAR_RADIUS + GL_TOOTH_HALF_WIDTH,
                                             diskNear, diskFar, axle);

    b3DestroyHull(diskNear);
    b3DestroyHull(diskFar);
    b3DestroyHull(axle);

    // Driver shaft, motorized
    {
        b3RevoluteJointDef revoluteDef = b3DefaultRevoluteJointDef();
        revoluteDef.base.bodyIdA = groundId;
        revoluteDef.base.bodyIdB = driverId;
        revoluteDef.base.localFrameA.p = b3Body_GetLocalPoint(groundId, gearPosition1);
        revoluteDef.base.localFrameB.p = b3Pos{0.0f, 0.0f, 0.0f};
        revoluteDef.enableMotor = g_gl_enable_motor;
        revoluteDef.maxMotorTorque = g_gl_motor_torque;
        revoluteDef.motorSpeed = g_gl_motor_speed;
        g_gl_driver = b3CreateRevoluteJoint(g_world, &revoluteDef);
    }

    // Follower shaft, swept between angle limits
    {
        b3RevoluteJointDef revoluteDef = b3DefaultRevoluteJointDef();
        revoluteDef.base.bodyIdA = groundId;
        revoluteDef.base.bodyIdB = followerId;
        revoluteDef.base.localFrameA.p = b3Body_GetLocalPoint(groundId, gearPosition2);
        revoluteDef.base.localFrameA.q = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, 0.25f * PI_F);
        revoluteDef.base.localFrameB.p = b3Pos{0.0f, 0.0f, 0.0f};
        revoluteDef.enableMotor = true;
        revoluteDef.maxMotorTorque = 0.5f;
        revoluteDef.lowerAngle = -0.3f * PI_F;
        revoluteDef.upperAngle = 0.8f * PI_F;
        revoluteDef.enableLimit = true;
        ignore b3CreateRevoluteJoint(g_world, &revoluteDef);
    }

    // One chain hangs from the follower rim at each depth and both lift the gate.
    b3Pos linkAttach = b3Pos{gearPosition2.x + GL_GEAR_RADIUS + 2.0f * GL_TOOTH_HALF_WIDTH
                                 + GL_TOOTH_RADIUS, gearPosition2.y, 0.0f};
    b3Pos doorPosition = b3Pos{linkAttach.x,
        linkAttach.y - (2.0f * cast(f32, GL_LINK_COUNT) * GL_LINK_HALF_LENGTH
                        + GL_DOOR_HALF_HEIGHT), 0.0f};

    b3BodyId nearLink = gl_create_chain(followerId, b3Pos{linkAttach.x, linkAttach.y, -GL_GEAR_Z});
    b3BodyId farLink = gl_create_chain(followerId, b3Pos{linkAttach.x, linkAttach.y, GL_GEAR_Z});

    gl_create_door(groundId, doorPosition, nearLink, farLink);

    gl_create_debris();
}

void destroy_gear_lift() {
    b3DestroyMesh(g_gl_mesh);
}

void gl_set_motor_speed(f32 speed) {
    b3RevoluteJoint_SetMotorSpeed(g_gl_driver, speed);
    b3Joint_WakeBodies(g_gl_driver);
}

bool gear_lift_controls() {
    if ImGui_Checkbox("Motor##GearLift", &g_gl_enable_motor) {
        b3RevoluteJoint_EnableMotor(g_gl_driver, g_gl_enable_motor);
        b3Joint_WakeBodies(g_gl_driver);
    }

    ImGui_PushItemWidth(6.0f * ImGui_GetFontSize());

    if ImGui_SliderFloat("Max Torque##GearLift", &g_gl_motor_torque, 0.0f, 100000.0f, "%.0f", 0) {
        b3RevoluteJoint_SetMaxMotorTorque(g_gl_driver, g_gl_motor_torque);
        b3Joint_WakeBodies(g_gl_driver);
    }

    if ImGui_SliderFloat("Speed##GearLift", &g_gl_motor_speed, -0.3f, 0.3f, "%.2f", 0) {
        gl_set_motor_speed(g_gl_motor_speed);
    }

    ImGui_PopItemWidth();

    return true;
}

// samples/sample_joint.cpp Driving
b3HeightFieldData* g_dr_height_field;
b3BodyId g_dr_chassis;
b3JointId g_dr_front_left;
b3JointId g_dr_front_right;
b3JointId g_dr_rear_left;
b3JointId g_dr_rear_right;
f32 g_dr_spin_speed;
f32 g_dr_max_spin_torque;
f32 g_dr_suspension_hertz;
f32 g_dr_suspension_damping_ratio;
f32 g_dr_lower_translation;
f32 g_dr_upper_translation;
f32 g_dr_steering_hertz;
f32 g_dr_steering_damping_ratio;
f32 g_dr_lower_steering_degrees;
f32 g_dr_upper_steering_degrees;
f32 g_dr_max_steering_torque;
f32 g_dr_target_steering_degrees;

void build_driving() {
    g_dr_spin_speed = 30.0f;
    g_dr_max_spin_torque = 5.0f;
    g_dr_suspension_hertz = 4.0f;
    g_dr_suspension_damping_ratio = 0.7f;
    g_dr_lower_translation = -0.2f;
    g_dr_upper_translation = 0.2f;
    g_dr_steering_hertz = 10.0f;
    g_dr_steering_damping_ratio = 0.7f;
    g_dr_lower_steering_degrees = -45.0f;
    g_dr_upper_steering_degrees = 45.0f;
    g_dr_max_steering_torque = 5.0f;
    g_dr_target_steering_degrees = 0.0f;

    b3BodyId groundId;
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        // bodyDef.position = { 0.0f, -1.0f, 0.0f };
        bodyDef.position = b3Pos{-20.0f, 0.0f, -20.0f};
        groundId = b3CreateBody(g_world, &bodyDef);

        b3ShapeDef shapeDef = b3DefaultShapeDef();
        // b3BoxHull groundBox = b3MakeBoxHull( 200.0f, 1.0f, 200.0f );
        // b3CreateHullShape( groundId, &shapeDef, &groundBox.base );
        g_dr_height_field = b3CreateWave(50, 50, b3Vec3{4.0f, 2.0f, 4.0f}, 0.02f, 0.04f,
                                         false);
        ignore b3CreateHeightFieldShape(groundId, &shapeDef, g_dr_height_field);
    }

    b3BodyDef bodyDef = b3DefaultBodyDef();
    b3ShapeDef shapeDef = b3DefaultShapeDef();

    {
        bodyDef.position = b3Pos{0.0f, 2.5f, 0.0f};
        bodyDef.type = b3_dynamicBody;
        g_dr_chassis = b3CreateBody(g_world, &bodyDef);
        shapeDef.density = 0.5f;
        b3BoxHull box = b3MakeBoxHull(2.0f, 0.5f, 1.0f);
        ignore b3CreateHullShape(g_dr_chassis, &shapeDef, &box.base);
    }

    // Keep vehicle upright
    {
        b3ParallelJointDef parallelJointDef = b3DefaultParallelJointDef();
        parallelJointDef.base.bodyIdA = groundId;
        parallelJointDef.base.bodyIdB = g_dr_chassis;
        parallelJointDef.base.localFrameA.q = b3ComputeQuatBetweenUnitVectors(b3Vec3_axisZ,
                                                                             b3Vec3_axisY);
        parallelJointDef.base.localFrameB.q = b3ComputeQuatBetweenUnitVectors(b3Vec3_axisZ,
                                                                             b3Vec3_axisY);
        parallelJointDef.base.drawScale = 2.0f;
        parallelJointDef.base.collideConnected = true;
        parallelJointDef.hertz = 0.5f;
        parallelJointDef.dampingRatio = 1.0f;
        ignore b3CreateParallelJoint(g_world, &parallelJointDef);
    }

    shapeDef.density = 2.0f;
    shapeDef.baseMaterial.friction = 3.0f;
    bodyDef.type = b3_dynamicBody;
    bodyDef.allowFastRotation = true;
    bodyDef.rotation = b3ComputeQuatBetweenUnitVectors(b3Vec3_axisY, b3Vec3_axisZ);

    b3WheelJointDef jointDef = b3DefaultWheelJointDef();
    jointDef.base.bodyIdA = g_dr_chassis;
    jointDef.base.localFrameA.q = b3ComputeQuatBetweenUnitVectors(b3Vec3_axisX, b3Vec3_axisY);
    jointDef.base.localFrameB.q = b3ComputeQuatBetweenUnitVectors(b3Vec3_axisZ, b3Vec3_axisY);
    jointDef.enableSuspensionLimit = true;
    jointDef.lowerSuspensionLimit = g_dr_lower_translation;
    jointDef.upperSuspensionLimit = g_dr_upper_translation;
    jointDef.enableSuspensionSpring = true;
    jointDef.suspensionHertz = g_dr_suspension_hertz;
    jointDef.suspensionDampingRatio = g_dr_suspension_damping_ratio;
    jointDef.enableSpinMotor = true;
    jointDef.maxSpinTorque = g_dr_max_spin_torque;
    jointDef.enableSteering = true;
    jointDef.steeringHertz = g_dr_steering_hertz;
    jointDef.steeringDampingRatio = g_dr_steering_damping_ratio;
    jointDef.targetSteeringAngle = 0.0f;
    jointDef.maxSteeringTorque = g_dr_max_steering_torque;
    jointDef.enableSteeringLimit = true;
    jointDef.lowerSteeringLimit = PI_F / 180.0f * g_dr_lower_steering_degrees;
    jointDef.upperSteeringLimit = PI_F / 180.0f * g_dr_upper_steering_degrees;

    b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, 0.4f};

    {
        bodyDef.position = b3Pos{1.5f, 2.0f, 0.8f};
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateSphereShape(bodyId, &shapeDef, &sphere);
        jointDef.base.bodyIdB = bodyId;
        jointDef.base.localFrameA.p = b3Pos{1.5f, -0.5f, 0.8f};
        jointDef.enableSteering = true;
        jointDef.enableSpinMotor = false;
        g_dr_front_left = b3CreateWheelJoint(g_world, &jointDef);
    }
    {
        bodyDef.position = b3Pos{1.5f, 2.0f, -0.8f};
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateSphereShape(bodyId, &shapeDef, &sphere);
        jointDef.base.bodyIdB = bodyId;
        jointDef.base.localFrameA.p = b3Pos{1.5f, -0.5f, -0.8f};
        jointDef.enableSteering = true;
        jointDef.enableSpinMotor = false;
        g_dr_front_right = b3CreateWheelJoint(g_world, &jointDef);
    }
    {
        bodyDef.position = b3Pos{-1.5f, 2.0f, 0.8f};
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateSphereShape(bodyId, &shapeDef, &sphere);
        jointDef.base.bodyIdB = bodyId;
        jointDef.base.localFrameA.p = b3Pos{-1.5f, -0.5f, 0.8f};
        jointDef.enableSteering = false;
        jointDef.enableSpinMotor = true;
        g_dr_rear_left = b3CreateWheelJoint(g_world, &jointDef);
    }
    {
        bodyDef.position = b3Pos{-1.5f, 2.0f, -0.8f};
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateSphereShape(bodyId, &shapeDef, &sphere);
        jointDef.base.bodyIdB = bodyId;
        jointDef.base.localFrameA.p = b3Pos{-1.5f, -0.5f, -0.8f};
        jointDef.enableSteering = false;
        jointDef.enableSpinMotor = true;
        g_dr_rear_right = b3CreateWheelJoint(g_world, &jointDef);
    }

    cam_third_person = false;
}

void destroy_driving() {
    if cam_third_person {
        cam_third_person = false;
        sapp_lock_mouse(false);
    }
    b3DestroyHeightField(g_dr_height_field);
}

void driving_keyboard(i32 key, i32 action, i32 mods) {
    ignore mods;
    if key == SAPP_KEYCODE_T && action == 1 {
        toggle_third_person();
    }
}

bool driving_controls() {
    ImGui_Text("Suspension");
    if ImGui_SliderFloat("Min##Suspension", &g_dr_lower_translation, -10.0f, 10.0f, "%.1f", 0) {
        g_dr_lower_translation = b3MinFloat(g_dr_lower_translation, g_dr_upper_translation);
        b3WheelJoint_SetSuspensionLimits(g_dr_front_left, g_dr_lower_translation,
                                         g_dr_upper_translation);
        b3WheelJoint_SetSuspensionLimits(g_dr_front_right, g_dr_lower_translation,
                                         g_dr_upper_translation);
        b3WheelJoint_SetSuspensionLimits(g_dr_rear_left, g_dr_lower_translation,
                                         g_dr_upper_translation);
        b3WheelJoint_SetSuspensionLimits(g_dr_rear_right, g_dr_lower_translation,
                                         g_dr_upper_translation);
    }
    if ImGui_SliderFloat("Max##Suspension", &g_dr_upper_translation, -10.0f, 10.0f, "%.1f", 0) {
        g_dr_upper_translation = b3MaxFloat(g_dr_upper_translation, g_dr_lower_translation);
        b3WheelJoint_SetSuspensionLimits(g_dr_front_left, g_dr_lower_translation,
                                         g_dr_upper_translation);
        b3WheelJoint_SetSuspensionLimits(g_dr_front_right, g_dr_lower_translation,
                                         g_dr_upper_translation);
        b3WheelJoint_SetSuspensionLimits(g_dr_rear_left, g_dr_lower_translation,
                                         g_dr_upper_translation);
        b3WheelJoint_SetSuspensionLimits(g_dr_rear_right, g_dr_lower_translation,
                                         g_dr_upper_translation);
    }
    if ImGui_SliderFloat("Hertz##Suspension", &g_dr_suspension_hertz, 0.0f, 10.0f, "%.1f", 0) {
        b3WheelJoint_SetSuspensionHertz(g_dr_front_left, g_dr_suspension_hertz);
        b3WheelJoint_SetSuspensionHertz(g_dr_front_right, g_dr_suspension_hertz);
        b3WheelJoint_SetSuspensionHertz(g_dr_rear_left, g_dr_suspension_hertz);
        b3WheelJoint_SetSuspensionHertz(g_dr_rear_right, g_dr_suspension_hertz);
    }
    if ImGui_SliderFloat("Damping##Suspension", &g_dr_suspension_damping_ratio, 0.0f, 2.0f,
                         "%.1f", 0) {
        b3WheelJoint_SetSuspensionDampingRatio(g_dr_front_left, g_dr_suspension_damping_ratio);
        b3WheelJoint_SetSuspensionDampingRatio(g_dr_front_right,
                                               g_dr_suspension_damping_ratio);
        b3WheelJoint_SetSuspensionDampingRatio(g_dr_rear_left, g_dr_suspension_damping_ratio);
        b3WheelJoint_SetSuspensionDampingRatio(g_dr_rear_right, g_dr_suspension_damping_ratio);
    }

    ImGui_Separator();
    ImGui_Text("Motor");
    ignore ImGui_SliderFloat("Max Torque", &g_dr_max_spin_torque, 0.0f, 100.0f, "%.0f", 0);
    ignore ImGui_SliderFloat("Speed", &g_dr_spin_speed, 0.0f, 100.0f, "%.0f", 0);

    ImGui_Separator();
    ImGui_Text("Steering");
    if ImGui_SliderFloat("Hertz##Steering", &g_dr_steering_hertz, 0.0f, 10.0f, "%.1f", 0) {
        b3WheelJoint_SetSteeringHertz(g_dr_front_left, g_dr_steering_hertz);
        b3WheelJoint_SetSteeringHertz(g_dr_front_right, g_dr_steering_hertz);
    }
    if ImGui_SliderFloat("Damping##Steering", &g_dr_steering_damping_ratio, 0.0f, 2.0f,
                         "%.1f", 0) {
        b3WheelJoint_SetSteeringDampingRatio(g_dr_front_left, g_dr_steering_damping_ratio);
        b3WheelJoint_SetSteeringDampingRatio(g_dr_front_right, g_dr_steering_damping_ratio);
    }
    if ImGui_SliderFloat("Torque##Steering", &g_dr_max_steering_torque, 0.0f, 20.0f,
                         "%.1f", 0) {
        b3WheelJoint_SetMaxSteeringTorque(g_dr_front_left, g_dr_max_steering_torque);
        b3WheelJoint_SetMaxSteeringTorque(g_dr_front_right, g_dr_max_steering_torque);
    }
    if ImGui_SliderFloat("Min Deg##Steering", &g_dr_lower_steering_degrees, -90.0f, 0.0f,
                         "%.0f", 0) {
        b3WheelJoint_SetSteeringLimits(g_dr_front_left,
                                       PI_F / 180.0f * g_dr_lower_steering_degrees,
                                       PI_F / 180.0f * g_dr_upper_steering_degrees);
        b3WheelJoint_SetSteeringLimits(g_dr_front_right,
                                       PI_F / 180.0f * g_dr_lower_steering_degrees,
                                       PI_F / 180.0f * g_dr_upper_steering_degrees);
    }
    if ImGui_SliderFloat("Max Deg##Steering", &g_dr_upper_steering_degrees, 0.0f, 90.0f,
                         "%.0f", 0) {
        b3WheelJoint_SetSteeringLimits(g_dr_front_left,
                                       PI_F / 180.0f * g_dr_lower_steering_degrees,
                                       PI_F / 180.0f * g_dr_upper_steering_degrees);
        b3WheelJoint_SetSteeringLimits(g_dr_front_right,
                                       PI_F / 180.0f * g_dr_lower_steering_degrees,
                                       PI_F / 180.0f * g_dr_upper_steering_degrees);
    }

    ImGui_Separator();
    bool thirdPerson = cam_third_person;
    if ImGui_Checkbox("Third Person (T)", &thirdPerson) {
        toggle_third_person();
    }
    return true;
}

void step_driving(f32 timeStep) {
    ignore timeStep;

    b3Vec2 throttle = b3Vec2{0.0f, 0.0f};
    if cam_third_person {
        if is_key_down(SAPP_KEYCODE_W) {
            throttle.x += 1.0f;
            b3Body_SetAwake(g_dr_chassis, true);
        }
        if is_key_down(SAPP_KEYCODE_S) {
            throttle.x -= 1.0f;
            b3Body_SetAwake(g_dr_chassis, true);
        }
        if is_key_down(SAPP_KEYCODE_A) {
            throttle.y += 1.0f;
            b3Body_SetAwake(g_dr_chassis, true);
        }
        if is_key_down(SAPP_KEYCODE_D) {
            throttle.y -= 1.0f;
            b3Body_SetAwake(g_dr_chassis, true);
        }
    }

    f32 maxSteeringAngle = 0.25f * PI_F;
    b3WheelJoint_SetTargetSteeringAngle(g_dr_front_left, maxSteeringAngle * throttle.y);
    b3WheelJoint_SetTargetSteeringAngle(g_dr_front_right, maxSteeringAngle * throttle.y);
    b3WheelJoint_SetSpinMotorSpeed(g_dr_rear_left, -g_dr_spin_speed * throttle.x);
    b3WheelJoint_SetSpinMotorSpeed(g_dr_rear_right, -g_dr_spin_speed * throttle.x);

    if cam_third_person {
        b3WorldTransform transform = b3Body_GetTransform(g_dr_chassis);
        cam_pivot = float3{transform.p.x, transform.p.y, transform.p.z};
        cam_rebuild_basis();
    }

    b3Vec3 velocity = b3Body_GetLinearVelocity(g_dr_chassis);
    b3Quat quat = b3Body_GetRotation(g_dr_chassis);
    b3Vec3 forward = b3RotateVector(quat, b3Vec3{-1.0f, 0.0f, 0.0f});
    f32 speed = b3Dot(velocity, forward);

    u8[128] buf;
    ignore snprintf(cast(u8*, &buf), 128, "speed = %.1f", cast(f64, speed));
    draw_text_line(cast(u8*, &buf));

    f32 leftSpeed = b3WheelJoint_GetSpinSpeed(g_dr_rear_left);
    f32 rightSpeed = b3WheelJoint_GetSpinSpeed(g_dr_rear_right);
    ignore snprintf(cast(u8*, &buf), 128, "spin speed = %.1f/%.1f", cast(f64, leftSpeed),
                    cast(f64, rightSpeed));
    draw_text_line(cast(u8*, &buf));

    f32 leftSpinTorque = b3WheelJoint_GetSpinTorque(g_dr_rear_left);
    f32 rightSpinTorque = b3WheelJoint_GetSpinTorque(g_dr_rear_right);
    ignore snprintf(cast(u8*, &buf), 128, "spin torque = %.1f/%.1f",
                    cast(f64, leftSpinTorque), cast(f64, rightSpinTorque));
    draw_text_line(cast(u8*, &buf));

    f32 angleLeft = b3WheelJoint_GetSteeringAngle(g_dr_front_left);
    f32 angleRight = b3WheelJoint_GetSteeringAngle(g_dr_front_right);
    ignore snprintf(cast(u8*, &buf), 128, "steering degrees = %.1f/%.1f",
                    cast(f64, 180.0f / PI_F * angleLeft),
                    cast(f64, 180.0f / PI_F * angleRight));
    draw_text_line(cast(u8*, &buf));

    f32 leftSteerTorque = b3WheelJoint_GetSteeringTorque(g_dr_front_left);
    f32 rightSteerTorque = b3WheelJoint_GetSteeringTorque(g_dr_front_right);
    ignore snprintf(cast(u8*, &buf), 128, "steering torque = %.1f/%.1f",
                    cast(f64, leftSteerTorque), cast(f64, rightSteerTorque));
    draw_text_line(cast(u8*, &buf));

    b3WorldTransform transform = b3WorldTransform_identity;
    transform.p.y += 0.05f;
    dbg_axes(transform, 2.0f);
}
