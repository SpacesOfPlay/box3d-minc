// Issues scenes. Ports of samples/sample_issues.cpp.

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
import debug_adapter;
import sample;
import sample_benchmark;
import sample_bodies;
import sample_continuous;
import sample_robustness;
import sample_stacking;
import sample_joint;
import sample_compound;
import sample_world;
import sample_shapes;

// samples/sample_issues.cpp RestitutionOvershoot
const f32 RO_BOX_HALF = 0.5f;
const f32 RO_FLOOR_HALF_XZ = 0.375f;
const f32 RO_FLOOR_HALF_Y = 0.25f;
const f32 RO_DROP_HEIGHT = 10.0f;
const f32 RO_TOLERANCE = 0.05f;

b3BodyId g_ro_box;
f32 g_ro_current_y;
f32 g_ro_max_bounce_y;
bool g_ro_bounced;
bool g_ro_failed;

void build_restitution_overshoot() {
    b3BoxHull floor = b3MakeBoxHull(RO_FLOOR_HALF_XZ, RO_FLOOR_HALF_Y, RO_FLOOR_HALF_XZ);
    b3BodyDef floorDef = b3DefaultBodyDef();
    floorDef.type = b3_staticBody;
    floorDef.position = b3Pos{0.0f, 0.0f - RO_FLOOR_HALF_Y, 0.0f};
    b3BodyId floorBody = b3CreateBody(g_world, &floorDef);
    b3ShapeDef floorShape = b3DefaultShapeDef();
    ignore b3CreateHullShape(floorBody, &floorShape, &floor.base);

    b3BoxHull box = b3MakeBoxHull(RO_BOX_HALF, RO_BOX_HALF, RO_BOX_HALF);
    b3BodyDef boxDef = b3DefaultBodyDef();
    boxDef.type = b3_dynamicBody;
    boxDef.position = b3Pos{0.0f, RO_DROP_HEIGHT, 0.0f};
    g_ro_box = b3CreateBody(g_world, &boxDef);
    b3ShapeDef boxShape = b3DefaultShapeDef();
    boxShape.baseMaterial.restitution = 1.0f;
    ignore b3CreateHullShape(g_ro_box, &boxShape, &box.base);

    g_ro_current_y = RO_DROP_HEIGHT;
    g_ro_max_bounce_y = 0.0f;
    g_ro_bounced = false;
    g_ro_failed = false;
}

void step_restitution_overshoot(f32 timeStep) {
    ignore timeStep;
    b3Pos position = b3Body_GetPosition(g_ro_box);
    g_ro_current_y = position.y;
    b3Vec3 velocity = b3Body_GetLinearVelocity(g_ro_box);
    if !g_ro_bounced && velocity.y > 0.0f { g_ro_bounced = true; }
    if g_ro_bounced {
        if position.y > g_ro_max_bounce_y { g_ro_max_bounce_y = position.y; }
        if position.y > RO_DROP_HEIGHT + RO_TOLERANCE { g_ro_failed = true; }
    }

    b3Pos markerPoint = b3Pos{0.0f, RO_DROP_HEIGHT + RO_BOX_HALF, 0.0f};
    dbg_plane(b3Vec3_axisY, markerPoint, b3_colorYellow);

    u8[128] buf;
    ignore snprintf(cast(u8*, &buf), 128, "drop height = %.2f m", RO_DROP_HEIGHT);
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 128, "current y   = %.2f m", g_ro_current_y);
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 128, "max bounce  = %.2f m", g_ro_max_bounce_y);
    draw_text_line(cast(u8*, &buf));
    if !g_ro_bounced {
        draw_text_line("waiting for first bounce...");
    } else if g_ro_failed {
        draw_text_line("FAIL: box exceeded drop height");
    } else {
        draw_text_line("PASS: bounce stays at or below drop height");
    }
}

// samples/sample_issues.cpp SlideTwistOffCenterShape
void build_slide_twist_off_center() {
    ignore add_ground_box(50.0f);

    b3Quat orientation = b3MakeQuatFromAxisAngle(b3Vec3_axisX, 20.0f * B3_DEG_TO_RAD);
    b3BodyDef bodyDef = b3DefaultBodyDef();
    b3ShapeDef shapeDef = b3DefaultShapeDef();

    bodyDef.position = b3Pos{0.0f, 4.0f, 0.0f};
    bodyDef.rotation = orientation;
    b3BodyId planeBody = b3CreateBody(g_world, &bodyDef);
    b3BoxHull plane = b3MakeBoxHull(10.0f, 0.5f, 10.0f);
    shapeDef.baseMaterial.friction = 0.6f;
    ignore b3CreateHullShape(planeBody, &shapeDef, &plane.base);

    b3Vec3 boxLocalCenter = b3Vec3{1.0f, 0.5f, 1.0f};
    b3Vec3 boxOffset = b3RotateVector(orientation, boxLocalCenter);
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = b3Pos{0.0f - boxOffset.x, 5.0f - boxOffset.y, 0.0f - boxOffset.z};
    bodyDef.rotation = orientation;
    //bodyDef.angularVelocity = 25.0f * b3RotateVector( orientation, b3Vec3_axisY );
    b3BodyId boxBody = b3CreateBody(g_world, &bodyDef);
    b3BoxHull mBox = b3MakeOffsetBoxHull(1.0f, 0.5f, 1.0f, boxLocalCenter);
    shapeDef.baseMaterial.friction = 0.3f;
    ignore b3CreateHullShape(boxBody, &shapeDef, &mBox.base);
    b3Vec3 up = b3RotateVector(orientation, b3Vec3_axisY);
    b3Body_SetAngularVelocity(boxBody, b3Vec3{25.0f * up.x, 25.0f * up.y, 25.0f * up.z});
}

// samples/sample_issues.cpp MultiplePrismatic
void build_multiple_prismatic() {
    b3BodyId groundId;
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        groundId = b3CreateBody(g_world, &bodyDef);
    }

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    b3BoxHull box = b3MakeBoxHull(0.5f, 0.5f, 0.5f);
    b3PrismaticJointDef jointDef = b3DefaultPrismaticJointDef();
    jointDef.base.bodyIdA = groundId;
    jointDef.base.localFrameA.p = b3Vec3{0.0f, 0.0f, 0.0f};
    jointDef.base.localFrameB.p = b3Vec3{0.0f, -0.6f, 0.0f};
    jointDef.base.drawScale = 2.0f;
    jointDef.base.constraintHertz = 240.0f;
    jointDef.lowerTranslation = -6.0f;
    jointDef.upperTranslation = 6.0f;
    jointDef.enableLimit = true;

    for i32 i = 0; i < 6; i += 1 {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{0.0f, 0.6f + 1.2f * cast(f32, i), 0.0f};
        bodyDef.type = b3_dynamicBody;
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateHullShape(bodyId, &shapeDef, &box.base);

        jointDef.base.bodyIdB = bodyId;
        ignore b3CreatePrismaticJoint(g_world, &jointDef);

        jointDef.base.bodyIdA = bodyId;
        jointDef.base.localFrameA.p = b3Vec3{0.0f, 0.6f, 0.0f};
    }

    // huge mouse force
    g_mouse_force_scale = 1000000.0f;
}

// samples/sample_issues.cpp HullCrash
const i32 HC_CAPACITY = 64;
b3HullData* g_hc_hull;
b3Vec3[HC_CAPACITY] g_hc_points;
i32 g_hc_count;

void build_hull_crash() {
    g_hc_hull = null;

    b3Vec3[5] points;
    points[0] = b3Vec3{100.000000f, -142.292389f, 130.826111f};
    points[1] = b3Vec3{99.5354385f, -71.3011093f, 130.826111f};
    points[2] = b3Vec3{99.5930862f, -80.1112213f, -100.000000f};
    points[3] = b3Vec3{100.000000f, -142.292389f, -100.000000f};
    points[4] = b3Vec3{99.5930862f, -80.1112213f, 130.826111f};

    g_hc_count = 5;
    for i32 i = 0; i < g_hc_count; i += 1 {
        g_hc_points[i] = b3MulSV(0.01f, points[i]);
    }

    // This shift shouldn't be necessary but I'm doing it so the hull
    // appears on the screen.
    // for ( int i = 0; i < m_count; ++i )
    //{
    //	m_points[i] -= m_points[0];
    //	m_points[i] *= 0.01f;
    //}

    g_hc_hull = b3CreateHull(cast(b3Vec3*, &g_hc_points), g_hc_count, g_hc_count);
}

void step_hull_crash(f32 timeStep) {
    ignore timeStep;
    if g_hc_hull != null {
        dbg_hull(b3WorldTransform_identity, g_hc_hull, b3_colorYellow);
    } else {
        for i32 i = 0; i < g_hc_count; i += 1 {
            adapter_point(b3Pos{g_hc_points[i].x, g_hc_points[i].y, g_hc_points[i].z},
                          5.0f, b3_colorWhite, null);
        }
    }

    adapter_transform(b3WorldTransform_identity, null);
}

// samples/sample_issues.cpp ConvexJitter
void build_convex_jitter() {
    ignore add_ground_box(10.0f);

    f32 s = 0.01f;

    {
        b3Vec3 b = b3Vec3{-459.292877f, 217.398331f, 1.00115335f};
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{s * b.x, s * b.z + 2.0f, s * b.y};
        bodyDef.rotation = b3Quat{b3Vec3{0.0f, -0.707106769f, 0.0f}, 0.707106769f};
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);

        b3ShapeDef shapeDef = b3DefaultShapeDef();

        const i32 count = 16;
        b3Vec3[count] points;
        points[0] = b3Vec3{-44.8770714f, -91.6598053f, -1.92012548f};
        points[1] = b3Vec3{-92.5001831f, 51.0151291f, 15.8006573f};
        points[2] = b3Vec3{-91.0282211f, -9.44371605f, 15.6148796f};
        points[3] = b3Vec3{90.2375641f, 77.3870087f, 15.9356089f};
        points[4] = b3Vec3{-85.5353241f, 91.3750992f, -1.36629653f};
        points[5] = b3Vec3{88.9092178f, -87.2975464f, -1.86754704f};
        points[6] = b3Vec3{83.7932816f, -89.8572235f, 15.4168339f};
        points[7] = b3Vec3{87.0243988f, 88.9776535f, -1.32423306f};
        points[8] = b3Vec3{-91.6564941f, -85.4949493f, 15.3782759f};
        points[9] = b3Vec3{-90.2922516f, -87.2074127f, -1.92012548f};
        points[10] = b3Vec3{-87.2944870f, 89.9510498f, 15.9215889f};
        points[11] = b3Vec3{79.2338104f, 89.9690781f, 15.9724140f};
        points[12] = b3Vec3{-91.6744461f, 81.0823212f, -1.39959598f};
        points[13] = b3Vec3{90.3452759f, -76.4459610f, 15.4588966f};
        points[14] = b3Vec3{-87.4021912f, -89.2263107f, 15.3677588f};
        points[15] = b3Vec3{76.3258057f, 92.0059967f, 1.82873762f};

        for i32 i = 0; i < count; i += 1 {
            b3Vec3 p = points[i];
            points[i] = b3Vec3{s * p.x, s * p.z, s * p.y};
        }

        b3HullData* hull = b3CreateHull(cast(b3Vec3*, &points), count, count);

        ignore b3CreateHullShape(bodyId, &shapeDef, hull);
        b3DestroyHull(hull);
    }

    {
        b3Vec3 b = b3Vec3{-402.321838f, 157.310364f, 16.8169250f};
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{s * b.x, s * b.z + 2.0f, s * b.y};
        bodyDef.rotation = b3Quat{b3Vec3{0.0f, -0.00152086187f, 0.0f}, 0.999998868f};
        bodyDef.type = b3_dynamicBody;

        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);

        b3ShapeDef shapeDef = b3DefaultShapeDef();
        shapeDef.baseMaterial.rollingResistance = 0.1f;

        const i32 count = 18;
        b3Vec3[count] points;
        points[0] = b3Vec3{29.5000000f, 17.1488495f, 0.175081104f};
        points[1] = b3Vec3{29.5000000f, -17.2990532f, 0.125000000f};
        points[2] = b3Vec3{29.4840164f, -17.3057766f, 24.0200863f};
        points[3] = b3Vec3{29.4840164f, 17.1648350f, 24.1781254f};
        points[4] = b3Vec3{-29.1345520f, 17.5529804f, 0.125000000f};
        points[5] = b3Vec3{-29.1345520f, 17.5529804f, 23.7899799f};
        points[6] = b3Vec3{-29.1441040f, 16.9679585f, 24.3750000f};
        points[7] = b3Vec3{-29.1345520f, -17.2990532f, 24.3750000f};
        points[8] = b3Vec3{-29.1345520f, -17.2990532f, 0.175081253f};
        points[9] = b3Vec3{29.0720215f, 17.5529785f, 0.125000000f};
        points[10] = b3Vec3{29.0859070f, 17.5629406f, 23.8120594f};
        points[11] = b3Vec3{29.1401348f, -17.2990532f, 24.3750000f};
        points[12] = b3Vec3{29.1123581f, 16.9722290f, 24.4027710f};
        points[13] = b3Vec3{29.3944912f, 17.2543602f, 24.1206398f};
        points[14] = b3Vec3{-29.1345520f, -17.2990532f, 24.0759430f};
        points[15] = b3Vec3{-29.1345520f, -16.9722252f, 24.4027710f};
        points[16] = b3Vec3{29.1123619f, -16.9722271f, 24.4027729f};
        points[17] = b3Vec3{29.5000000f, 17.3429642f, 24.0000000f};

        for i32 i = 0; i < count; i += 1 {
            b3Vec3 p = points[i];
            points[i] = b3Vec3{s * p.x, s * p.z, s * p.y};
        }

        b3HullData* hull = b3CreateHull(cast(b3Vec3*, &points), count, count);

        ignore b3CreateHullShape(bodyId, &shapeDef, hull);
        b3DestroyHull(hull);
    }
}

void destroy_hull_crash() {
    if g_hc_hull != null {
        b3DestroyHull(g_hc_hull);
        g_hc_hull = null;
    }
}

// samples/sample_issues.cpp WheelStack
const i32 WHEEL_HULL_COUNT = 37;
const i32 WHEEL_VERT_COUNT = 317;
f32[WHEEL_VERT_COUNT * 3] g_wheel_verts;
i32[WHEEL_HULL_COUNT * 2] g_wheel_spans;   // offset, count
b3HullData*[WHEEL_HULL_COUNT] g_wheel_hulls;

void wheel_data_init() {
    g_wheel_verts[0..] = { 0.010279f, 0.086341f, 0.051768f, -0.010314f, -0.084708f, 0.051804f,
                           0.010279f, -0.084708f, 0.051768f, -0.010314f, 0.086341f, 0.051804f,
                           0.029138f, -0.084708f, -0.043911f, 0.043724f, -0.084708f, -0.029376f,
                           0.010098f, -0.084708f, -0.051759f, -0.010494f, -0.084708f, -0.051723f,
                           0.051638f, -0.084708f, -0.010364f, 0.051674f, -0.084708f, 0.010229f };
    g_wheel_verts[30..] = { 0.043827f, -0.084708f, 0.029268f, 0.029291f, -0.084708f, 0.043855f,
                           -0.029353f, -0.084708f, 0.043957f, -0.051889f, -0.084708f, -0.010183f,
                           -0.051853f, -0.084708f, 0.010409f, -0.043939f, -0.084708f, 0.029421f,
                           -0.029506f, -0.084708f, -0.043809f, -0.044042f, -0.084708f, -0.029222f,
                           -0.029353f, 0.086341f, 0.043957f, 0.043724f, 0.086341f, -0.029376f };
    g_wheel_verts[60..] = { 0.029138f, 0.086341f, -0.043911f, 0.010098f, 0.086341f, -0.051759f,
                           0.051638f, 0.086341f, -0.010364f, 0.051674f, 0.086341f, 0.010229f,
                           -0.044042f, 0.086341f, -0.029222f, 0.043827f, 0.086341f, 0.029268f,
                           -0.051889f, 0.086341f, -0.010183f, -0.043939f, 0.086341f, 0.029421f,
                           -0.029506f, 0.086341f, -0.043809f, 0.035354f, 0.070897f, 0.035658f };
    g_wheel_verts[90..] = { 0.035354f, -0.069397f, 0.035658f, -0.035498f, 0.070897f, 0.035658f,
                           -0.035498f, -0.069397f, 0.035658f, 0.035354f, -0.069397f, 0.365503f,
                           0.035354f, 0.070897f, 0.365503f, -0.035498f, 0.070897f, 0.365503f,
                           -0.035498f, -0.069397f, 0.365503f, 0.035354f, 0.070897f, -0.365033f,
                           0.035354f, -0.069397f, -0.365033f, -0.035498f, 0.070897f, -0.365033f };
    g_wheel_verts[120..] = { -0.035498f, -0.069397f, -0.365033f, 0.035354f, -0.069397f, -0.035187f,
                           0.035354f, 0.070897f, -0.035187f, -0.035498f, 0.070897f, -0.035187f,
                           -0.035498f, -0.069397f, -0.035187f, -0.035494f, -0.069397f, -0.035191f,
                           -0.365340f, -0.069397f, -0.035191f, -0.035494f, 0.070897f, -0.035191f,
                           -0.365340f, 0.070897f, -0.035191f, -0.365340f, -0.069397f, 0.035661f };
    g_wheel_verts[150..] = { -0.035494f, -0.069397f, 0.035661f, -0.035494f, 0.070897f, 0.035661f,
                           -0.365340f, 0.070897f, 0.035661f, 0.035350f, 0.070897f, -0.035191f,
                           0.365196f, 0.070897f, -0.035191f, 0.035350f, -0.069397f, -0.035191f,
                           0.365196f, -0.069397f, -0.035191f, 0.365196f, 0.070897f, 0.035661f,
                           0.035350f, 0.070897f, 0.035661f, 0.035350f, -0.069397f, 0.035661f };
    g_wheel_verts[180..] = { 0.365196f, -0.069397f, 0.035661f, -0.070802f, 0.086337f, 0.355420f,
                           -0.070802f, -0.084705f, 0.355420f, -0.097081f, 0.086335f, 0.487537f,
                           -0.097081f, -0.084704f, 0.487537f, -0.000108f, 0.086337f, 0.362383f,
                           -0.000108f, 0.086335f, 0.497088f, -0.000108f, -0.084704f, 0.497088f,
                           -0.000108f, -0.084705f, 0.362383f, -0.138787f, -0.084705f, 0.334799f };
    g_wheel_verts[210..] = { -0.138787f, 0.086337f, 0.334799f, -0.070810f, -0.084705f, 0.355419f,
                           -0.070810f, 0.086337f, 0.355419f, -0.097090f, 0.086335f, 0.487536f,
                           -0.190336f, 0.086335f, 0.459250f, -0.190336f, -0.084704f, 0.459250f,
                           -0.097090f, -0.084704f, 0.487536f, -0.138795f, 0.086337f, 0.334796f,
                           -0.138795f, -0.084705f, 0.334796f, -0.201442f, 0.086337f, 0.301310f };
    g_wheel_verts[240..] = { -0.201442f, -0.084705f, 0.301310f, -0.276280f, -0.084704f, 0.413313f,
                           -0.276280f, 0.086335f, 0.413313f, -0.190344f, 0.086335f, 0.459247f,
                           -0.190344f, -0.084704f, 0.459247f, -0.201450f, -0.084705f, 0.301306f,
                           -0.256361f, -0.084705f, 0.256241f, -0.201450f, 0.086337f, 0.301306f,
                           -0.256361f, 0.086337f, 0.256241f, -0.351612f, 0.086335f, 0.351492f };
    g_wheel_verts[270..] = { -0.351612f, -0.084704f, 0.351492f, -0.276288f, 0.086335f, 0.413309f,
                           -0.276288f, -0.084704f, 0.413309f, -0.301431f, -0.084705f, 0.201325f,
                           -0.413435f, -0.084704f, 0.276163f, -0.413435f, 0.086335f, 0.276163f,
                           -0.301431f, 0.086337f, 0.201325f, -0.256367f, -0.084705f, 0.256236f,
                           -0.256367f, 0.086337f, 0.256236f, -0.351618f, 0.086335f, 0.351487f };
    g_wheel_verts[300..] = { -0.351618f, -0.084704f, 0.351487f, -0.334922f, -0.084705f, 0.138670f,
                           -0.459374f, -0.084704f, 0.190220f, -0.459374f, 0.086335f, 0.190220f,
                           -0.334922f, 0.086337f, 0.138670f, -0.301436f, 0.086337f, 0.201318f,
                           -0.301436f, -0.084705f, 0.201318f, -0.413440f, 0.086335f, 0.276156f,
                           -0.413440f, -0.084704f, 0.276156f, -0.487663f, -0.084704f, 0.096966f };
    g_wheel_verts[330..] = { -0.355546f, -0.084705f, 0.070686f, -0.459377f, -0.084704f, 0.190212f,
                           -0.334926f, -0.084705f, 0.138663f, -0.355546f, 0.086337f, 0.070686f,
                           -0.334926f, 0.086337f, 0.138663f, -0.487663f, 0.086335f, 0.096966f,
                           -0.459377f, 0.086335f, 0.190212f, -0.362511f, -0.084705f, -0.000016f,
                           -0.355549f, -0.084705f, 0.070678f, -0.497217f, -0.084704f, -0.000016f };
    g_wheel_verts[360..] = { -0.487665f, -0.084704f, 0.096957f, -0.497217f, 0.086335f, -0.000016f,
                           -0.362511f, 0.086337f, -0.000016f, -0.355549f, 0.086337f, 0.070678f,
                           -0.487665f, 0.086335f, 0.096957f, -0.362512f, 0.086337f, -0.000024f,
                           -0.362512f, -0.084705f, -0.000024f, -0.355549f, 0.086337f, -0.070717f,
                           -0.355549f, -0.084705f, -0.070717f, -0.497217f, -0.084704f, -0.000024f };
    g_wheel_verts[390..] = { -0.487666f, -0.084704f, -0.096997f, -0.497217f, 0.086335f, -0.000024f,
                           -0.487666f, 0.086335f, -0.096997f, -0.334927f, -0.084705f, -0.138702f,
                           -0.355548f, 0.086337f, -0.070726f, -0.355548f, -0.084705f, -0.070726f,
                           -0.334927f, 0.086337f, -0.138702f, -0.487665f, -0.084704f, -0.097005f,
                           -0.459379f, -0.084704f, -0.190252f, -0.487665f, 0.086335f, -0.097005f };
    g_wheel_verts[420..] = { -0.459379f, 0.086335f, -0.190252f, -0.413442f, -0.084704f, -0.276196f,
                           -0.301439f, -0.084705f, -0.201358f, -0.334925f, -0.084705f, -0.138710f,
                           -0.459376f, -0.084704f, -0.190260f, -0.334925f, 0.086337f, -0.138710f,
                           -0.301439f, 0.086337f, -0.201358f, -0.459376f, 0.086335f, -0.190260f,
                           -0.413442f, 0.086335f, -0.276196f, -0.256370f, -0.084705f, -0.256276f };
    g_wheel_verts[450..] = { -0.301434f, 0.086337f, -0.201365f, -0.301434f, -0.084705f, -0.201365f,
                           -0.256370f, 0.086337f, -0.256276f, -0.413438f, 0.086335f, -0.276203f,
                           -0.351621f, 0.086335f, -0.351527f, -0.413438f, -0.084704f, -0.276203f,
                           -0.351621f, -0.084704f, -0.351527f, -0.256364f, -0.084705f, -0.256283f,
                           -0.201453f, 0.086337f, -0.301347f, -0.256364f, 0.086337f, -0.256283f };
    g_wheel_verts[480..] = { -0.201453f, -0.084705f, -0.301347f, -0.276292f, -0.084704f, -0.413350f,
                           -0.351615f, -0.084704f, -0.351534f, -0.351615f, 0.086335f, -0.351534f,
                           -0.276292f, 0.086335f, -0.413350f, -0.201447f, -0.084705f, -0.301352f,
                           -0.276285f, -0.084704f, -0.413355f, -0.138799f, -0.084705f, -0.334838f,
                           -0.190349f, -0.084704f, -0.459289f, -0.276285f, 0.086335f, -0.413355f };
    g_wheel_verts[510..] = { -0.201447f, 0.086337f, -0.301352f, -0.190349f, 0.086335f, -0.459289f,
                           -0.138799f, 0.086337f, -0.334838f, -0.190341f, 0.086335f, -0.459293f,
                           -0.190341f, -0.084704f, -0.459293f, -0.138791f, -0.084705f, -0.334842f,
                           -0.138791f, 0.086337f, -0.334842f, -0.070815f, 0.086337f, -0.355462f,
                           -0.070815f, -0.084705f, -0.355462f, -0.097095f, 0.086335f, -0.487579f };
    g_wheel_verts[540..] = { -0.097095f, -0.084704f, -0.487579f, -0.070807f, 0.086337f, -0.355464f,
                           -0.097086f, 0.086335f, -0.487581f, -0.097086f, -0.084704f, -0.487581f,
                           -0.070807f, -0.084705f, -0.355464f, -0.000113f, -0.084705f, -0.362427f,
                           -0.000113f, 0.086337f, -0.362427f, -0.000113f, 0.086335f, -0.497132f,
                           -0.000113f, -0.084704f, -0.497132f, 0.070588f, 0.086337f, -0.355465f };
    g_wheel_verts[570..] = { 0.096868f, -0.084704f, -0.487582f, 0.096868f, 0.086335f, -0.487582f,
                           0.070588f, -0.084705f, -0.355465f, -0.000105f, 0.086337f, -0.362427f,
                           -0.000105f, 0.086335f, -0.497133f, -0.000105f, -0.084704f, -0.497133f,
                           -0.000105f, -0.084705f, -0.362427f, 0.190123f, -0.084704f, -0.459294f,
                           0.190123f, 0.086335f, -0.459294f, 0.138573f, -0.084705f, -0.334843f };
    g_wheel_verts[600..] = { 0.138573f, 0.086337f, -0.334843f, 0.070597f, 0.086337f, -0.355463f,
                           0.070597f, -0.084705f, -0.355463f, 0.096876f, -0.084704f, -0.487580f,
                           0.096876f, 0.086335f, -0.487580f, 0.201229f, -0.084705f, -0.301354f,
                           0.276067f, -0.084704f, -0.413358f, 0.201229f, 0.086337f, -0.301354f,
                           0.138581f, -0.084705f, -0.334840f, 0.138581f, 0.086337f, -0.334840f };
    g_wheel_verts[630..] = { 0.190131f, 0.086335f, -0.459292f, 0.276067f, 0.086335f, -0.413358f,
                           0.190131f, -0.084704f, -0.459292f, 0.201236f, 0.086337f, -0.301350f,
                           0.256147f, -0.084705f, -0.256286f, 0.256147f, 0.086337f, -0.256286f,
                           0.201236f, -0.084705f, -0.301350f, 0.351398f, -0.084704f, -0.351537f,
                           0.351398f, 0.086335f, -0.351537f, 0.276074f, -0.084704f, -0.413353f };
    g_wheel_verts[660..] = { 0.276074f, 0.086335f, -0.413353f, 0.301218f, -0.084705f, -0.201369f,
                           0.301218f, 0.086337f, -0.201369f, 0.256154f, -0.084705f, -0.256280f,
                           0.256154f, 0.086337f, -0.256280f, 0.413221f, 0.086335f, -0.276207f,
                           0.413221f, -0.084704f, -0.276207f, 0.351405f, 0.086335f, -0.351531f,
                           0.351405f, -0.084704f, -0.351531f, 0.334709f, 0.086337f, -0.138715f };
    g_wheel_verts[690..] = { 0.301223f, 0.086337f, -0.201362f, 0.334709f, -0.084705f, -0.138715f,
                           0.301223f, -0.084705f, -0.201362f, 0.459160f, -0.084704f, -0.190264f,
                           0.459160f, 0.086335f, -0.190264f, 0.413226f, 0.086335f, -0.276200f,
                           0.413226f, -0.084704f, -0.276200f, 0.334713f, -0.084705f, -0.138707f,
                           0.355333f, -0.084705f, -0.070731f, 0.334713f, 0.086337f, -0.138707f };
    g_wheel_verts[720..] = { 0.355333f, 0.086337f, -0.070731f, 0.459164f, 0.086335f, -0.190257f,
                           0.487450f, 0.086335f, -0.097010f, 0.487450f, -0.084704f, -0.097010f,
                           0.459164f, -0.084704f, -0.190257f, 0.355335f, -0.084705f, -0.070722f,
                           0.362298f, -0.084705f, -0.000029f, 0.355335f, 0.086337f, -0.070722f,
                           0.362298f, 0.086337f, -0.000029f, 0.497003f, -0.084704f, -0.000029f };
    g_wheel_verts[750..] = { 0.497003f, 0.086335f, -0.000029f, 0.487452f, 0.086335f, -0.097002f,
                           0.487452f, -0.084704f, -0.097002f, 0.362298f, -0.084705f, -0.000021f,
                           0.497003f, -0.084704f, -0.000021f, 0.355336f, -0.084705f, 0.070673f,
                           0.487453f, -0.084704f, 0.096952f, 0.497003f, 0.086335f, -0.000021f,
                           0.362298f, 0.086337f, -0.000021f, 0.355336f, 0.086337f, 0.070673f };
    g_wheel_verts[780..] = { 0.487453f, 0.086335f, 0.096952f, 0.355334f, -0.084705f, 0.070681f,
                           0.355334f, 0.086337f, 0.070681f, 0.487451f, 0.086335f, 0.096961f,
                           0.487451f, -0.084704f, 0.096961f, 0.334714f, -0.084705f, 0.138658f,
                           0.459165f, -0.084704f, 0.190207f, 0.334714f, 0.086337f, 0.138658f,
                           0.459165f, 0.086335f, 0.190207f, 0.334711f, 0.086337f, 0.138666f };
    g_wheel_verts[810..] = { 0.301225f, 0.086337f, 0.201313f, 0.459163f, 0.086335f, 0.190215f,
                           0.413229f, 0.086335f, 0.276151f, 0.334711f, -0.084705f, 0.138666f,
                           0.301225f, -0.084705f, 0.201313f, 0.413229f, -0.084704f, 0.276151f,
                           0.459163f, -0.084704f, 0.190215f, 0.301221f, -0.084705f, 0.201321f,
                           0.256157f, -0.084705f, 0.256232f, 0.301221f, 0.086337f, 0.201321f };
    g_wheel_verts[840..] = { 0.256157f, 0.086337f, 0.256232f, 0.413224f, -0.084704f, 0.276159f,
                           0.413224f, 0.086335f, 0.276159f, 0.351408f, 0.086335f, 0.351483f,
                           0.351408f, -0.084704f, 0.351483f, 0.201240f, -0.084705f, 0.301302f,
                           0.201240f, 0.086337f, 0.301302f, 0.256151f, -0.084705f, 0.256238f,
                           0.256151f, 0.086337f, 0.256238f, 0.351402f, 0.086335f, 0.351489f };
    g_wheel_verts[870..] = { 0.351402f, -0.084704f, 0.351489f, 0.276078f, 0.086335f, 0.413305f,
                           0.276078f, -0.084704f, 0.413305f, 0.138586f, -0.084705f, 0.334793f,
                           0.201233f, -0.084705f, 0.301307f, 0.190135f, -0.084704f, 0.459244f,
                           0.276071f, -0.084704f, 0.413311f, 0.201233f, 0.086337f, 0.301307f,
                           0.138586f, 0.086337f, 0.334793f, 0.276071f, 0.086335f, 0.413311f };
    g_wheel_verts[900..] = { 0.190135f, 0.086335f, 0.459244f, 0.138578f, -0.084705f, 0.334797f,
                           0.070602f, -0.084705f, 0.355417f, 0.138578f, 0.086337f, 0.334797f,
                           0.070602f, 0.086337f, 0.355417f, 0.190127f, 0.086335f, 0.459248f,
                           0.190127f, -0.084704f, 0.459248f, 0.096881f, 0.086335f, 0.487534f,
                           0.096881f, -0.084704f, 0.487534f, 0.070593f, -0.084705f, 0.355419f };
    g_wheel_verts[930..] = { 0.096873f, 0.086335f, 0.487536f, 0.096873f, -0.084704f, 0.487536f,
                           0.070593f, 0.086337f, 0.355419f, -0.000100f, 0.086337f, 0.362382f,
                           -0.000100f, 0.086335f, 0.497087f, -0.000100f, -0.084704f, 0.497087f,
                           -0.000100f, -0.084705f, 0.362382f };

    g_wheel_spans[0..] = { 0, 29, 29, 8, 37, 8, 45, 8, 53, 8, 61, 8, 69, 8, 77, 8, 85, 8, 93, 8 };
    g_wheel_spans[20..] = { 101, 8, 109, 8, 117, 8, 125, 8, 133, 8, 141, 8, 149, 8, 157, 8, 165, 8, 173, 8 };
    g_wheel_spans[40..] = { 181, 8, 189, 8, 197, 8, 205, 8, 213, 8, 221, 8, 229, 8, 237, 8, 245, 8, 253, 8 };
    g_wheel_spans[60..] = { 261, 8, 269, 8, 277, 8, 285, 8, 293, 8, 301, 8, 309, 8 };
}

// 30 metal_wheel1 props (37-piece convex decompositions) stacked. Body-pair contact merging
// lets them settle and sleep instead of wobbling.
const i32 WHEEL_COUNT = 30;

void build_gmod_wheel_stack() {
    wheel_data_init();

    ignore add_ground_box(10.0f);

    const i32 N = 512;
    b3Vec3[N] buffer;
    i32 bufferCount = 0;
    for i32 h = 0; h < WHEEL_HULL_COUNT; h += 1 {
        i32 offset = g_wheel_spans[h * 2];
        i32 count = g_wheel_spans[h * 2 + 1];
        g_wheel_hulls[h] = b3CreateHull(cast(b3Vec3*, &g_wheel_verts[offset * 3]), count, count);

        b3Vec3* points = b3GetHullPoints(g_wheel_hulls[h]);
        for i32 j = 0; j < g_wheel_hulls[h].vertexCount && bufferCount < N; j += 1 {
            buffer[bufferCount] = points[j];
            bufferCount += 1;
        }
    }

    // Create a single hull that wraps the input hulls.
    b3HullData* wheelHull = b3CreateHull(cast(b3Vec3*, &buffer), bufferCount, bufferCount);

    const f32 height = 0.171f;
    const f32 spacing = height + 0.006f;
    const f32 startY = 0.5f * height + 0.004f;

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.baseMaterial.friction = 0.6f;

    for i32 i = 0; i < WHEEL_COUNT; i += 1 {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;
        bodyDef.name = "wheel";
        bodyDef.position = b3Pos{0.0f, startY + cast(f32, i) * spacing, 0.0f};
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);

        //for ( int h = 0; h < s_metalWheel1HullCount; ++h )
        //{
        //	b3CreateHullShape( bodyId, &shapeDef, m_hulls[h] );
        //}

        // Using a single hull improves the simulation.
        ignore b3CreateHullShape(bodyId, &shapeDef, wheelHull);
    }

    b3DestroyHull(wheelHull);

    b3World_SetContactTuning(g_world, 240.0f, 10.0f, 3.0f);
}

void destroy_gmod_wheel_stack() {
    for i32 h = 0; h < WHEEL_HULL_COUNT; h += 1 {
        b3DestroyHull(g_wheel_hulls[h]);
    }
}

void step_gmod_wheel_stack(f32 timeStep) {
    ignore timeStep;
    b3Profile p = b3World_GetProfile(g_world);
    f32 substepRate = g_hertz * cast(f32, g_substeps);

    u8[160] buf;
    ignore snprintf(cast(u8*, &buf), 160, "wheels %d, hull pieces/wheel %d (%d total shapes)",
                    WHEEL_COUNT, WHEEL_HULL_COUNT, WHEEL_COUNT * WHEEL_HULL_COUNT);
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 160, "step %.0f hz, sub-steps %d -> substep rate %.0f hz",
                    g_hertz, g_substeps, substepRate);
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 160, "eff contact hz = min(240, %.0f) = %.0f",
                    0.125f * substepRate, b3MinFloat(240.0f, 0.125f * substepRate));
    draw_text_line(cast(u8*, &buf));
    f32 collidePct = 0.0f;
    f32 solvePct = 0.0f;
    if p.step > 0.0f {
        collidePct = 100.0f * p.collide / p.step;
        solvePct = 100.0f * p.solve / p.step;
    }
    ignore snprintf(cast(u8*, &buf), 160,
                    "step %.3f ms | collide %.3f ms (%.0f%%) | solve %.3f ms (%.0f%%)",
                    p.step, p.collide, collidePct, p.solve, solvePct);
    draw_text_line(cast(u8*, &buf));
}
// samples/sample_issues.cpp Crash
b3BodyId g_crash_body1;
b3BodyId g_crash_body2;
b3MeshData* g_crash_grid_mesh;

void build_crash() {
    b3BodyId groundId;
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{0.0f, -1.0f, 0.0f};
        groundId = b3CreateBody(g_world, &bodyDef);

        b3ShapeDef shapeDef = b3DefaultShapeDef();
        g_crash_grid_mesh = b3CreateGridMesh(20, 20, 2.0f, 0, true);
        ignore b3CreateMeshShape(groundId, &shapeDef, g_crash_grid_mesh, b3Vec3_one);
    }

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = b3Pos{2.0f, 4.0f, 0.0f};
    g_crash_body1 = b3CreateBody(g_world, &bodyDef);

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    b3BoxHull box = b3MakeBoxHull(0.5f, 0.5f, 0.5f);
    ignore b3CreateHullShape(g_crash_body1, &shapeDef, &box.base);

    bodyDef.position = b3Pos{-2.0f, 4.0f, 0.0f};
    g_crash_body2 = b3CreateBody(g_world, &bodyDef);
    ignore b3CreateHullShape(g_crash_body2, &shapeDef, &box.base);
}

void destroy_crash() {
    b3DestroyMesh(g_crash_grid_mesh);
}

bool crash_controls() {
    if ImGui_Button("Add Joint", ImVec2{0.0f, 0.0f}) {
        b3WeldJointDef jointDef = b3DefaultWeldJointDef();
        jointDef.base.bodyIdA = g_crash_body1;
        jointDef.base.bodyIdB = g_crash_body2;
        ignore b3CreateWeldJoint(g_world, &jointDef);
    }
    return true;
}

// samples/sample_issues.cpp SBoxMover
b3MeshData* g_sm_box_mesh;
b3HeightFieldData* g_sm_height_field;
b3MeshData* g_sm_grid_mesh;

void build_sbox_mover() {
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{-10.0f, 0.0f, -10.0f};
        b3BodyId groundId = b3CreateBody(g_world, &bodyDef);

        b3ShapeDef shapeDef = b3DefaultShapeDef();
        g_sm_height_field = b3CreateGrid(40, 40, b3Vec3{0.5f, 1.0f, 0.5f}, false);
        //m_heightField = b3CreateWave( 40, 40, {1.0f, 2.0f, 1.0f}, 0.02f, 0.04f, false );
        ignore b3CreateHeightFieldShape(groundId, &shapeDef, g_sm_height_field);

        g_sm_grid_mesh = b3CreateGridMesh(40, 40, 0.5f, 1, true);
        //b3CreateMeshShape( groundId, &shapeDef, m_gridMesh, b3Vec3_one );
    }

    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        b3BodyId groundId = b3CreateBody(g_world, &bodyDef);

        // m_boxMesh = b3CreateBoxMesh( { 0.0f, 1.0f, 0.0f }, { 1.0f, 1.0f, 1.0f }, true );
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        g_sm_box_mesh = b3CreatePlatformMesh(b3Vec3{0.0f, 0.5f, 0.0f}, 1.0f, 2.0f, 5.0f);
        b3Vec3 scale = b3Vec3_one;
        ignore b3CreateMeshShape(groundId, &shapeDef, g_sm_box_mesh, scale);
    }

    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;
        bodyDef.position = b3Pos{0.0f, 3.5f, 0.0f};
        bodyDef.motionLocks.angularX = true;
        bodyDef.motionLocks.angularY = true;
        bodyDef.motionLocks.angularZ = true;
        bodyDef.enableContactRecycling = false;
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);

        b3ShapeDef shapeDef = b3DefaultShapeDef();
        b3BoxHull box = b3MakeBoxHull(0.25f, 1.0f, 0.25f);
        ignore b3CreateHullShape(bodyId, &shapeDef, &box.base);
    }
}

void destroy_sbox_mover() {
    b3DestroyMesh(g_sm_box_mesh);
    b3DestroyHeightField(g_sm_height_field);
    b3DestroyMesh(g_sm_grid_mesh);
}

void step_sbox_mover(f32 timeStep) {
    ignore timeStep;
    b3Transform transform = b3Transform{b3Vec3{0.0f, 1.1f, 0.0f}, b3Quat_identity};
    dbg_axes(b3MakeWorldTransform(transform), 3.0f);
}

// samples/sample_issues.cpp SBoxGhostCollisions
// s&box works in inches: 1 unit = 0.0254 m (40 units per meter)
const f32 SG_SRC = 0.0254f;

// Floor layout (s&box inches)
const i32 SG_HALF_LENGTH_U = 256; // strip half length along x
const i32 SG_HALF_WIDTH_U = 64;   // strip half width along z
const i32 SG_TILE_SIZE_U = 32;    // slab tile stride

// Beam section: beams run along z, the character walks along x across them.
// Tops are 12 wide with 1.5 chamfers, pits between are 10 wide and 24 deep.
// The 16 wide hull always spans the 13 gap between flat tops.
const f32 SG_BEAM_PITCH_U = 22.0f;
const f32 SG_BEAM_WIDTH_U = 12.0f;
const f32 SG_CHAMFER_WIDTH_U = 1.5f;
const f32 SG_CHAMFER_DROP_U = 1.0f;
const f32 SG_PIT_DEPTH_U = 24.0f;
const i32 SG_BEAM_COUNT = 9;
const f32 SG_BEAM_REGION0 = -94.0f; // first beam start
const f32 SG_BEAM_REGION1 = 94.0f;  // last beam end

// Character (s&box player: 16 wide zero radius box hull, 72 tall, mass 500)
const f32 SG_BODY_HALF_WIDTH = 16.0f * SG_SRC;
const f32 SG_BODY_HALF_HEIGHT = 36.0f * SG_SRC;
const f32 SG_CHARACTER_MASS = 500.0f;

const f32 SG_WALK_RANGE_X = 3.5f;     // turn around beyond +/- this x (meters)
const f32 SG_WALK_RANGE_Z = 0.5f;     // turn around beyond +/- this x (meters)
const f32 SG_LAUNCH_THRESHOLD = 0.5f; // upward m/s counted as a ghost launch (~20 inch/s)
const i32 SG_MARKER_CAPACITY = 64;

b3MeshData*[2] g_sg_chunk_mesh;
b3BodyId g_sg_character;
f32 g_sg_walk_direction_x;
f32 g_sg_walk_direction_z;
f32 g_sg_walk_speed_x;
f32 g_sg_walk_speed_z;
i32 g_sg_launch_count;
f32 g_sg_max_launch_speed;
bool g_sg_was_launched;
b3Pos[SG_MARKER_CAPACITY] g_sg_launch_markers;
i32 g_sg_launch_marker_count;

// upstream emits into std::vector; these stand in for them, and a null
// buffer makes an emit pass count without writing.
b3Vec3* g_sg_vertices;
i32* g_sg_indices;
i32 g_sg_vertex_count;
i32 g_sg_index_count;

// Deterministic integer hash for tile tessellation selection
u32 sg_hash(u32 x) {
    x = x ^ (x >> 16);
    x *= cast(u32, 0x7feb352d);
    x = x ^ (x >> 15);
    x *= cast(u32, 0x846ca68b);
    x = x ^ (x >> 16);
    return x;
}

void sg_emit_triangle(b3Vec3 a, b3Vec3 b, b3Vec3 c) {
    i32 base = g_sg_vertex_count;
    if g_sg_vertices != null {
        g_sg_vertices[base + 0] = a;
        g_sg_vertices[base + 1] = b;
        g_sg_vertices[base + 2] = c;
        g_sg_indices[g_sg_index_count + 0] = base;
        g_sg_indices[g_sg_index_count + 1] = base + 1;
        g_sg_indices[g_sg_index_count + 2] = base + 2;
    }
    g_sg_vertex_count += 3;
    g_sg_index_count += 3;
}

// Horizontal patch at height y spanning [x0,x1]x[z0,z1] (inches), normal +y
void sg_emit_patch(f32 x0, f32 x1, f32 z0, f32 z1, f32 y, f32 cell) {
    if x1 - x0 < 0.01f {
        return;
    }

    i32 countX = cast(i32, (x1 - x0) / cell + 0.99f);
    i32 countZ = cast(i32, (z1 - z0) / cell + 0.99f);

    for i32 ix = 0; ix < countX; ix += 1 {
        for i32 iz = 0; iz < countZ; iz += 1 {
            f32 cx0 = x0 + (x1 - x0) * cast(f32, ix) / cast(f32, countX);
            f32 cx1 = x0 + (x1 - x0) * cast(f32, ix + 1) / cast(f32, countX);
            f32 cz0 = z0 + (z1 - z0) * cast(f32, iz) / cast(f32, countZ);
            f32 cz1 = z0 + (z1 - z0) * cast(f32, iz + 1) / cast(f32, countZ);

            b3Vec3 a = b3Vec3{SG_SRC * cx0, SG_SRC * y, SG_SRC * cz0};
            b3Vec3 b = b3Vec3{SG_SRC * cx1, SG_SRC * y, SG_SRC * cz0};
            b3Vec3 c = b3Vec3{SG_SRC * cx1, SG_SRC * y, SG_SRC * cz1};
            b3Vec3 d = b3Vec3{SG_SRC * cx0, SG_SRC * y, SG_SRC * cz1};

            // Alternate the split diagonal like typical cooked map data
            if (ix + iz) & 1 != 0 {
                sg_emit_triangle(a, d, c);
                sg_emit_triangle(a, c, b);
            } else {
                sg_emit_triangle(a, d, b);
                sg_emit_triangle(b, d, c);
            }
        }
    }
}

// Sloped strip from edge (xLow, yLow) to edge (xHigh, yHigh) spanning the full z width
void sg_emit_slope(f32 xLow, f32 yLow, f32 xHigh, f32 yHigh, f32 zCell) {
    i32 countZ = cast(i32, 2.0f * cast(f32, SG_HALF_WIDTH_U) / zCell + 0.99f);
    for i32 iz = 0; iz < countZ; iz += 1 {
        f32 z0 = -cast(f32, SG_HALF_WIDTH_U)
               + 2.0f * cast(f32, SG_HALF_WIDTH_U) * cast(f32, iz) / cast(f32, countZ);
        f32 z1 = -cast(f32, SG_HALF_WIDTH_U)
               + 2.0f * cast(f32, SG_HALF_WIDTH_U) * cast(f32, iz + 1) / cast(f32, countZ);

        b3Vec3 l0 = b3Vec3{SG_SRC * xLow, SG_SRC * yLow, SG_SRC * z0};
        b3Vec3 l1 = b3Vec3{SG_SRC * xLow, SG_SRC * yLow, SG_SRC * z1};
        b3Vec3 h0 = b3Vec3{SG_SRC * xHigh, SG_SRC * yHigh, SG_SRC * z0};
        b3Vec3 h1 = b3Vec3{SG_SRC * xHigh, SG_SRC * yHigh, SG_SRC * z1};

        sg_emit_triangle(l0, l1, h1);
        sg_emit_triangle(l0, h1, h0);
    }
}

// Vertical wall at x from y0 (bottom) to y1 (top). facing = +1 faces +x, -1 faces -x
void sg_emit_wall(f32 x, f32 y0, f32 y1, i32 facing, f32 zCell) {
    i32 countZ = cast(i32, 2.0f * cast(f32, SG_HALF_WIDTH_U) / zCell + 0.99f);
    for i32 iz = 0; iz < countZ; iz += 1 {
        f32 z0 = -cast(f32, SG_HALF_WIDTH_U)
               + 2.0f * cast(f32, SG_HALF_WIDTH_U) * cast(f32, iz) / cast(f32, countZ);
        f32 z1 = -cast(f32, SG_HALF_WIDTH_U)
               + 2.0f * cast(f32, SG_HALF_WIDTH_U) * cast(f32, iz + 1) / cast(f32, countZ);

        b3Vec3 b0 = b3Vec3{SG_SRC * x, SG_SRC * y0, SG_SRC * z0};
        b3Vec3 b1 = b3Vec3{SG_SRC * x, SG_SRC * y0, SG_SRC * z1};
        b3Vec3 t0 = b3Vec3{SG_SRC * x, SG_SRC * y1, SG_SRC * z0};
        b3Vec3 t1 = b3Vec3{SG_SRC * x, SG_SRC * y1, SG_SRC * z1};

        if facing > 0 {
            sg_emit_triangle(b0, b1, t1);
            sg_emit_triangle(b0, t1, t0);
        } else {
            sg_emit_triangle(b0, t0, t1);
            sg_emit_triangle(b0, t1, b1);
        }
    }
}

// Clip [a0,a1] to [c0,c1]
bool sg_clip_span(f32 a0, f32 a1, f32 c0, f32 c1, f32* o0, f32* o1) {
    *o0 = a0 > c0 ? a0 : c0;
    *o1 = a1 < c1 ? a1 : c1;
    return *o1 - *o0 > 0.01f;
}

void sg_emit_chunk(i32 chunk, f32 x0U, f32 x1U) {
    f32 s0;
    f32 s1;

    // --- Concrete slabs at y = 0 outside the beam region ---
    // Tiles tessellate at a hash-picked resolution so neighbors meet with T-junctions,
    // like cooked s&box map collision.
    f32[4] slabSpans = { -cast(f32, SG_HALF_LENGTH_U), SG_BEAM_REGION0,
                         SG_BEAM_REGION1, cast(f32, SG_HALF_LENGTH_U) };
    for i32 i = 0; i < 2; i += 1 {
        if sg_clip_span(slabSpans[2 * i], slabSpans[2 * i + 1], x0U, x1U, &s0, &s1) == false {
            continue;
        }

        for f32 tx = s0; tx < s1; tx += cast(f32, SG_TILE_SIZE_U) {
            f32 tx1 = b3MinFloat(tx + cast(f32, SG_TILE_SIZE_U), s1);
            for i32 tz = -SG_HALF_WIDTH_U; tz < SG_HALF_WIDTH_U; tz += SG_TILE_SIZE_U {
                u32 h = sg_hash(cast(u32, cast(i32, tx) * 73856093) ^ cast(u32, tz * 19349663) ^
                                cast(u32, chunk) * cast(u32, 2654435761));
                f32[3] cells = { 4.0f, 8.0f, 16.0f };
                sg_emit_patch(tx, tx1, cast(f32, tz), cast(f32, tz + SG_TILE_SIZE_U), 0.0f,
                              cells[cast(i32, h % 3)]);
            }
        }
    }

    // --- Beam section: flat tops at y = 0, chamfers dropping to pits ---
    f32 pitTop = -SG_CHAMFER_DROP_U;
    f32 pitBottom = -SG_PIT_DEPTH_U;

    for i32 k = 0; k < SG_BEAM_COUNT; k += 1 {
        f32 bx = SG_BEAM_REGION0 + SG_BEAM_PITCH_U * cast(f32, k);
        bool pitLeft = k > 0;
        bool pitRight = k < SG_BEAM_COUNT - 1;

        // Flat top (flush with the slab on outer sides)
        f32 top0 = pitLeft ? bx + SG_CHAMFER_WIDTH_U : bx;
        f32 top1 = pitRight ? bx + SG_BEAM_WIDTH_U - SG_CHAMFER_WIDTH_U : bx + SG_BEAM_WIDTH_U;
        if sg_clip_span(top0, top1, x0U, x1U, &s0, &s1) {
            sg_emit_patch(s0, s1, -cast(f32, SG_HALF_WIDTH_U), cast(f32, SG_HALF_WIDTH_U),
                          0.0f, 8.0f);
        }

        // Chamfers sloping below the walkable plane
        if pitLeft && bx >= x0U && bx < x1U {
            sg_emit_slope(bx, pitTop, bx + SG_CHAMFER_WIDTH_U, 0.0f, 8.0f);
        }
        if pitRight && bx + SG_BEAM_WIDTH_U > x0U && bx + SG_BEAM_WIDTH_U <= x1U {
            sg_emit_slope(bx + SG_BEAM_WIDTH_U, pitTop, bx + SG_BEAM_WIDTH_U - SG_CHAMFER_WIDTH_U,
                          0.0f, 8.0f);
        }

        // Pit to the right of this beam
        if pitRight {
            f32 pitL = bx + SG_BEAM_WIDTH_U;
            f32 pitR = bx + SG_BEAM_PITCH_U;
            if pitL >= x0U && pitL < x1U {
                sg_emit_wall(pitL, pitBottom, pitTop, 1, 16.0f);
            }
            if pitR > x0U && pitR <= x1U {
                sg_emit_wall(pitR, pitBottom, pitTop, -1, 16.0f);
            }
            if sg_clip_span(pitL, pitR, x0U, x1U, &s0, &s1) {
                sg_emit_patch(s0, s1, -cast(f32, SG_HALF_WIDTH_U), cast(f32, SG_HALF_WIDTH_U),
                              pitBottom, 16.0f);
            }
        }
    }
}

void sg_create_floor_chunk(i32 chunk, f32 x0U, f32 x1U) {
    // Count first, then emit for real: the buffers replace upstream's vectors.
    g_sg_vertices = null;
    g_sg_indices = null;
    g_sg_vertex_count = 0;
    g_sg_index_count = 0;
    sg_emit_chunk(chunk, x0U, x1U);

    i32 vertexCount = g_sg_vertex_count;
    i32 indexCount = g_sg_index_count;
    g_sg_vertices = cast(b3Vec3*, alloc(cast(i64, vertexCount * 12)));
    g_sg_indices = cast(i32*, alloc(cast(i64, indexCount * 4)));
    g_sg_vertex_count = 0;
    g_sg_index_count = 0;
    sg_emit_chunk(chunk, x0U, x1U);

    b3MeshDef meshDef = b3MeshDef{};
    meshDef.vertices = g_sg_vertices;
    meshDef.indices = g_sg_indices;
    meshDef.vertexCount = vertexCount;
    meshDef.triangleCount = indexCount / 3;
    meshDef.weldVertices = true;
    meshDef.weldTolerance = 0.005f; // == B3_LINEAR_SLOP, same as s&box
    meshDef.identifyEdges = true;

    g_sg_chunk_mesh[chunk] = b3CreateMesh(&meshDef, null, 0);

    free(g_sg_vertices);
    free(g_sg_indices);
    g_sg_vertices = null;
    g_sg_indices = null;

    b3BodyDef bodyDef = b3DefaultBodyDef();
    b3BodyId body = b3CreateBody(g_world, &bodyDef);

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    ignore b3CreateMeshShape(body, &shapeDef, g_sg_chunk_mesh[chunk], b3Vec3_one);
}

void build_sbox_ghost_collisions() {
    // Two chunks meeting at x = 0, each its own body and mesh shape, so seam contacts live
    // in separate contact pairs like s&box world mesh chunks. A beam top straddles the seam.
    sg_create_floor_chunk(0, -cast(f32, SG_HALF_LENGTH_U), 0.0f);
    sg_create_floor_chunk(1, 0.0f, cast(f32, SG_HALF_LENGTH_U));

    // Character
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;
        bodyDef.position = b3Pos{-SG_WALK_RANGE_X, SG_BODY_HALF_HEIGHT + 0.1f, 0.0f};
        bodyDef.motionLocks.angularX = true;
        bodyDef.motionLocks.angularY = true;
        bodyDef.motionLocks.angularZ = true;
        bodyDef.enableSleep = false;
        bodyDef.enableContactRecycling = false;
        bodyDef.gravityScale = 2.03f; // s&box gravity: 800 inch/s^2
        bodyDef.name = "character";
        g_sg_character = b3CreateBody(g_world, &bodyDef);

        b3ShapeDef shapeDef = b3DefaultShapeDef();
        shapeDef.baseMaterial.friction = 0.0f;
        shapeDef.baseMaterial.restitution = 0.0f;
        //shapeDef.baseMaterial.customColor = b3_colorLimeGreen;

        f32 volume = 8.0f * SG_BODY_HALF_WIDTH * SG_BODY_HALF_HEIGHT * SG_BODY_HALF_WIDTH;
        shapeDef.density = SG_CHARACTER_MASS / volume;
        shapeDef.enableSpeculativeContact = false;

        b3BoxHull box = b3MakeBoxHull(SG_BODY_HALF_WIDTH, SG_BODY_HALF_HEIGHT,
                                      SG_BODY_HALF_WIDTH);
        ignore b3CreateHullShape(g_sg_character, &shapeDef, &box.base);
    }

    g_sg_walk_direction_x = 1.0f;
    g_sg_walk_direction_z = 1.0f;
    g_sg_walk_speed_x = 350.0f * SG_SRC; // s&box run speed
    g_sg_walk_speed_z = 20.0f * SG_SRC;
    g_sg_launch_count = 0;
    g_sg_max_launch_speed = 0.0f;
    g_sg_launch_marker_count = 0;
    g_sg_was_launched = false;
}

void destroy_sbox_ghost_collisions() {
    b3DestroyMesh(g_sg_chunk_mesh[0]);
    b3DestroyMesh(g_sg_chunk_mesh[1]);
}

// upstream SBoxGhostCollisions::Step, the half ahead of Sample::Step():
// pure velocity control, the way the s&box player controller moves — keep
// the solver's vertical velocity, set the horizontal.
void pre_step_sbox_ghost_collisions(f32 timeStep) {
    ignore timeStep;
    b3Pos position = b3Body_GetPosition(g_sg_character);
    if position.x > SG_WALK_RANGE_X {
        g_sg_walk_direction_x = -1.0f;
    } else if position.x < -SG_WALK_RANGE_X {
        g_sg_walk_direction_x = 1.0f;
    }

    if position.z > SG_WALK_RANGE_Z {
        g_sg_walk_direction_z = -1.0f;
    } else if position.z < -SG_WALK_RANGE_Z {
        g_sg_walk_direction_z = 1.0f;
    }

    b3Vec3 velocity = b3Body_GetLinearVelocity(g_sg_character);
    velocity.x = g_sg_walk_direction_x * g_sg_walk_speed_x;
    velocity.z = g_sg_walk_direction_z * g_sg_walk_speed_z;
    b3Body_SetLinearVelocity(g_sg_character, velocity);
}

void step_sbox_ghost_collisions(f32 timeStep) {
    // upstream m_didStep
    if timeStep > 0.0f {
        // The walkable plane is exactly y = 0, so the grounded body center never rises above
        // rest height. Any upward velocity spike while grounded is a ghost collision: there
        // is nothing to climb and nothing to bounce off.
        b3Pos p = b3Body_GetPosition(g_sg_character);
        b3Vec3 v = b3Body_GetLinearVelocity(g_sg_character);

        bool grounded = p.y < SG_BODY_HALF_HEIGHT + 0.01f + 4.0f * SG_SRC;
        bool launched = v.y > SG_LAUNCH_THRESHOLD;

        if grounded && launched && g_sg_was_launched == false {
            g_sg_launch_count += 1;
            g_sg_max_launch_speed = b3MaxFloat(g_sg_max_launch_speed, v.y);

            if g_sg_launch_marker_count < SG_MARKER_CAPACITY {
                g_sg_launch_markers[g_sg_launch_marker_count] = p;
                g_sg_launch_marker_count += 1;
            }
        }

        g_sg_was_launched = launched;
    }

    for i32 i = 0; i < g_sg_launch_marker_count; i += 1 {
        adapter_point(g_sg_launch_markers[i], 8.0f, b3_colorRed, null);
    }

    b3Vec3 currentVelocity = b3Body_GetLinearVelocity(g_sg_character);
    u8[160] buf;
    ignore snprintf(cast(u8*, &buf), 160, "ghost launches: %d, worst: %.2f m/s (%.0f inch/s)",
                    g_sg_launch_count, g_sg_max_launch_speed, g_sg_max_launch_speed / SG_SRC);
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 160, "vertical velocity: %.2f m/s", currentVelocity.y);
    draw_text_line(cast(u8*, &buf));
}

bool sbox_ghost_collisions_controls() {
    f32 speedUX = g_sg_walk_speed_x / SG_SRC;
    if ImGui_SliderFloat("Walk Speed X (inch/s)", &speedUX, 100.0f, 400.0f, "%.0f", 0) {
        g_sg_walk_speed_x = speedUX * SG_SRC;
    }

    f32 speedUZ = g_sg_walk_speed_z / SG_SRC;
    if ImGui_SliderFloat("Walk Speed Z (inch/s)", &speedUZ, 10.0f, 100.0f, "%.0f", 0) {
        g_sg_walk_speed_z = speedUZ * SG_SRC;
    }

    if ImGui_Button("Reset Counters", ImVec2{0.0f, 0.0f}) {
        g_sg_launch_count = 0;
        g_sg_max_launch_speed = 0.0f;
        g_sg_launch_marker_count = 0;
    }

    ImGui_Text("Launches: %d", g_sg_launch_count);
    return true;
}

// samples/sample_issues.cpp CapsuleMeshBug
b3MeshData* g_cmb_building;

void build_capsule_mesh() {
    // --- Ground plane ---
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        b3BodyId body = b3CreateBody(g_world, &bodyDef);

        b3ShapeDef shapeDef = b3DefaultShapeDef();
        b3BoxHull ground = b3MakeBoxHull(50.0f, 0.1f, 50.0f);
        ignore b3CreateHullShape(body, &shapeDef, &ground.base);
    }

    // --- Building mesh on top of ground ---
    g_cmb_building = create_mesh_data("data/meshes/building.obj", 1.0f, false, false, true, true);
    if g_cmb_building != null {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{0.0f, 0.1f, 0.0f};
        b3BodyId body = b3CreateBody(g_world, &bodyDef);

        b3ShapeDef shapeDef = b3DefaultShapeDef();
        ignore b3CreateMeshShape(body, &shapeDef, g_cmb_building, b3Vec3_one);
    }

    // --- Locked capsule (same setup as player controller body) ---
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;
        bodyDef.position = b3Pos{0.0f, 4.0f, 10.0f};
        bodyDef.motionLocks.angularX = true;
        bodyDef.motionLocks.angularY = true;
        bodyDef.motionLocks.angularZ = true;
        bodyDef.enableSleep = false;
        bodyDef.enableContactRecycling = false;
        b3BodyId body = b3CreateBody(g_world, &bodyDef);

        b3ShapeDef shapeDef = b3DefaultShapeDef();
        shapeDef.baseMaterial.friction = 0.3f;
        shapeDef.baseMaterial.customColor = b3_colorMagenta;

        b3Capsule capsule = b3Capsule{b3Pos{0.0f, -0.5f, 0.0f}, b3Pos{0.0f, 0.5f, 0.0f}, 0.3f};
        ignore b3CreateCapsuleShape(body, &shapeDef, &capsule);
    }
}

void destroy_capsule_mesh() {
    if g_cmb_building != null {
        b3DestroyMesh(g_cmb_building);
        g_cmb_building = null;
    }
}

void step_capsule_mesh(f32 timeStep) {
    ignore timeStep;
    if g_cmb_building == null {
        draw_text_line("data/meshes/building.obj could not be read");
    }
}
