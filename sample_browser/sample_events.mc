// Event scenes. Ports of samples/sample_events.cpp.

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
import sample_joint;
import sample_compound;
import sample_world;
import sample_shapes;
import sample_issues;
import sample_mesh;

// b3_nullJointId is private to the module; box3d's own header says zero
// initialization also gives a null id.
b3JointId g_null_joint_id;

// samples/sample_events.cpp SensorVisit
b3ShapeId g_sv_sensor_shape;

void build_sensor_visit() {
    // Visitor
    b3BoxHull dynamicBox = b3MakeBoxHull(0.5f, 0.5f, 0.5f);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = b3Pos{0.0f, 12.5f, 0.0f};
    b3BodyId dynamicBody = b3CreateBody(g_world, &bodyDef);

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.enableSensorEvents = true;
    ignore b3CreateHullShape(dynamicBody, &shapeDef, &dynamicBox.base);

    // Sensor
    b3BoxHull sensorBox = b3MakeBoxHull(2.0f, 2.0f, 2.0f);

    bodyDef.type = b3_kinematicBody;
    bodyDef.position = b3Pos{0.0f, 2.0f, 0.0f};
    b3BodyId sensorBody = b3CreateBody(g_world, &bodyDef);
    shapeDef.isSensor = true;
    shapeDef.enableSensorEvents = true;
    g_sv_sensor_shape = b3CreateHullShape(sensorBody, &shapeDef, &sensorBox.base);
}

void step_sensor_visit(f32 timeStep) {
    ignore timeStep;
    dbg_ground_grid(10);

    b3SensorEvents events = b3World_GetSensorEvents(g_world);

    for i32 i = 0; i < events.beginCount; i += 1 {
        b3SensorBeginTouchEvent* event = events.beginEvents + i;
        if event.sensorShapeId.index1 == g_sv_sensor_shape.index1
            && event.sensorShapeId.world0 == g_sv_sensor_shape.world0
            && event.sensorShapeId.generation == g_sv_sensor_shape.generation {
            b3BodyId bodyId = b3Shape_GetBody(event.visitorShapeId);
            b3DestroyBody(bodyId);
            break;
        }
    }
}

// samples/sample_events.cpp MoveEvent
b3BodyId g_me_body;
b3Vec3 g_me_local_pivot;

void build_move_event() {
    ignore add_ground_box(40.0f);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    b3Pos pivot = b3Pos{0.0f, 1.0f, 0.0f};
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = pivot;
    bodyDef.name = "big box";
    g_me_body = b3CreateBody(g_world, &bodyDef);

    g_me_local_pivot = b3Body_GetLocalPoint(g_me_body, pivot);

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.enableHitEvents = true;
    b3BoxHull dynamicBox = b3MakeTransformedBoxHull(0.5f, 10.0f, 0.5f,
        b3Transform{b3Vec3{0.0f, 10.0f, 0.0f}, b3Quat_identity});
    ignore b3CreateHullShape(g_me_body, &shapeDef, &dynamicBox.base);

    b3Pos center = b3Body_GetWorldCenter(g_me_body);

    b3Vec3 r = b3SubPos(pivot, center);
    f32 rr = b3LengthSquared(r);
    if rr > 0.0f {
        b3Vec3 v = b3Vec3{-10.0f, 0.0f, 0.0f};
        b3Vec3 omega = b3MulSV(1.0f / rr, b3Cross(v, r));
        b3Body_SetAngularVelocity(g_me_body, omega);
        b3Body_SetLinearVelocity(g_me_body, v);
    }
}

void step_move_event(f32 timeStep) {
    ignore timeStep;
    b3Vec3 vp = b3Body_GetLocalPointVelocity(g_me_body, g_me_local_pivot);
    b3Vec3 v = b3Body_GetLinearVelocity(g_me_body);
    b3Vec3 omega = b3Body_GetAngularVelocity(g_me_body);

    u8[192] buf;
    ignore snprintf(cast(u8*, &buf), 192,
                    "vp = [%.2f, %.2f, %.2f], v = [%.2f, %.2f, %.2f], w = [%.2f, %.2f, %.2f]",
                    vp.x, vp.y, vp.z, v.x, v.y, v.z, omega.x, omega.y, omega.z);
    draw_text_line(cast(u8*, &buf));

    b3BodyEvents moveEvents = b3World_GetBodyEvents(g_world);
    for i32 i = 0; i < moveEvents.moveCount; i += 1 {
        b3BodyId body = moveEvents.moveEvents[i].bodyId;
        if moveEvents.moveEvents[i].fellAsleep {
            ignore snprintf(cast(u8*, &buf), 192, "%s fell asleep", b3Body_GetName(body));
        } else {
            ignore snprintf(cast(u8*, &buf), 192, "%s moved", b3Body_GetName(body));
        }
        draw_text_line(cast(u8*, &buf));
    }
}

// samples/sample_events.cpp JointEvent
const i32 JE_COUNT = 6;
b3JointId[JE_COUNT] g_je_joints;

void build_joint_event() {
    ignore add_ground_box(20.0f);

    b3BodyId groundId;
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        groundId = b3CreateBody(g_world, &bodyDef);
    }

    for i32 i = 0; i < JE_COUNT; i += 1 {
        g_je_joints[i] = g_null_joint_id;
    }

    b3Vec3 position = b3Vec3{-12.5f, 10.0f, 0.0f};
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.enableSleep = false;

    b3BoxHull box = b3MakeBoxHull(1.0f, 1.0f, 0.5f);

    i32 index = 0;

    f32 forceThreshold = 3000.0f;
    f32 torqueThreshold = 10000.0f;

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.density = 1.0f;

    // distance joint
    {
        bodyDef.position = b3OffsetPos(b3Pos_zero, position);
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateHullShape(bodyId, &shapeDef, &box.base);

        f32 length = 2.0f;
        b3Pos pivot1 = b3Pos{position.x, position.y + 1.0f + length, 0.0f};
        b3Pos pivot2 = b3Pos{position.x, position.y + 1.0f, 0.0f};
        b3DistanceJointDef jointDef = b3DefaultDistanceJointDef();
        jointDef.base.bodyIdA = groundId;
        jointDef.base.bodyIdB = bodyId;
        jointDef.base.localFrameA.p = b3Body_GetLocalPoint(jointDef.base.bodyIdA, pivot1);
        jointDef.base.localFrameB.p = b3Body_GetLocalPoint(jointDef.base.bodyIdB, pivot2);
        jointDef.length = length;
        jointDef.base.forceThreshold = forceThreshold;
        jointDef.base.torqueThreshold = torqueThreshold;
        jointDef.base.collideConnected = true;
        jointDef.base.userData = cast(void*, cast(i64, index));
        g_je_joints[index] = b3CreateDistanceJoint(g_world, &jointDef);
    }

    position.x += 5.0f;
    index += 1;

    // motor joint
    g_je_joints[index] = g_null_joint_id;

    position.x += 5.0f;
    index += 1;

    // prismatic joint
    {
        bodyDef.position = b3OffsetPos(b3Pos_zero, position);
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateHullShape(bodyId, &shapeDef, &box.base);

        b3Pos pivot = b3Pos{position.x - 1.0f, position.y, 0.0f};
        b3PrismaticJointDef jointDef = b3DefaultPrismaticJointDef();
        jointDef.base.bodyIdA = groundId;
        jointDef.base.bodyIdB = bodyId;
        jointDef.base.localFrameA.p = b3Body_GetLocalPoint(jointDef.base.bodyIdA, pivot);
        jointDef.base.localFrameB.p = b3Body_GetLocalPoint(jointDef.base.bodyIdB, pivot);
        jointDef.base.forceThreshold = forceThreshold;
        jointDef.base.torqueThreshold = torqueThreshold;
        jointDef.base.collideConnected = true;
        jointDef.base.userData = cast(void*, cast(i64, index));
        g_je_joints[index] = b3CreatePrismaticJoint(g_world, &jointDef);
    }

    position.x += 5.0f;
    index += 1;

    // revolute joint
    {
        bodyDef.position = b3OffsetPos(b3Pos_zero, position);
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateHullShape(bodyId, &shapeDef, &box.base);

        b3Pos pivot = b3Pos{position.x - 1.0f, position.y, 0.0f};
        b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
        jointDef.base.bodyIdA = groundId;
        jointDef.base.bodyIdB = bodyId;
        jointDef.base.localFrameA.p = b3Body_GetLocalPoint(jointDef.base.bodyIdA, pivot);
        jointDef.base.localFrameB.p = b3Body_GetLocalPoint(jointDef.base.bodyIdB, pivot);
        jointDef.base.forceThreshold = forceThreshold;
        jointDef.base.torqueThreshold = torqueThreshold;
        jointDef.base.collideConnected = true;
        jointDef.base.userData = cast(void*, cast(i64, index));
        g_je_joints[index] = b3CreateRevoluteJoint(g_world, &jointDef);
    }

    position.x += 5.0f;
    index += 1;

    // weld joint
    {
        bodyDef.position = b3OffsetPos(b3Pos_zero, position);
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateHullShape(bodyId, &shapeDef, &box.base);

        b3Pos pivot = b3Pos{position.x - 1.0f, position.y, 0.0f};
        b3WeldJointDef jointDef = b3DefaultWeldJointDef();
        jointDef.base.bodyIdA = groundId;
        jointDef.base.bodyIdB = bodyId;
        jointDef.base.localFrameA.p = b3Body_GetLocalPoint(jointDef.base.bodyIdA, pivot);
        jointDef.base.localFrameB.p = b3Body_GetLocalPoint(jointDef.base.bodyIdB, pivot);
        jointDef.angularHertz = 2.0f;
        jointDef.angularDampingRatio = 0.5f;
        jointDef.base.forceThreshold = forceThreshold;
        jointDef.base.torqueThreshold = torqueThreshold;
        jointDef.base.collideConnected = true;
        jointDef.base.userData = cast(void*, cast(i64, index));
        g_je_joints[index] = b3CreateWeldJoint(g_world, &jointDef);
    }

    position.x += 5.0f;
    index += 1;

    // wheel joint
    g_je_joints[index] = g_null_joint_id;

    position.x += 5.0f;
    index += 1;
}

void step_joint_event(f32 timeStep) {
    ignore timeStep;

    // Process joint events
    b3JointEvents events = b3World_GetJointEvents(g_world);
    for i32 i = 0; i < events.count; i += 1 {
        // Destroy the joint if it is still valid
        b3JointEvent* event = events.jointEvents + i;

        if b3Joint_IsValid(event.jointId) {
            i32 index = cast(i32, cast(i64, event.userData));
            b3DestroyJoint(event.jointId, true);
            g_je_joints[index] = g_null_joint_id;
        }
    }
}

// samples/sample_events.cpp SensorHits
const i32 SH_TRANSFORM_CAPACITY = 20;

b3MeshData* g_sh_grid_mesh;
b3ShapeId g_sh_static_sensor;
b3ShapeId g_sh_kinematic_sensor;
b3ShapeId g_sh_dynamic_sensor;
b3BodyId g_sh_kinematic_body;
b3BodyId g_sh_dynamic_body;
b3JointId g_sh_joint;
b3BodyId g_sh_body;
bool g_sh_body_live;
b3ShapeId g_sh_shape;
i32 g_sh_transform_count;
b3WorldTransform[SH_TRANSFORM_CAPACITY] g_sh_transforms;
i32 g_sh_begin_count;
i32 g_sh_end_count;
bool g_sh_is_bullet;

void sensor_hits_launch() {
    if g_sh_body_live {
        b3DestroyBody(g_sh_body);
        g_sh_body_live = false;
    }
    g_sh_transform_count = 0;
    g_sh_begin_count = 0;
    g_sh_end_count = 0;
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = b3Pos{-26.7f, 6.0f, 0.0f};
    f32 speed = random_float_range(200.0f, 300.0f);
    bodyDef.linearVelocity = b3Vec3{speed, 0.0f, 0.0f};
    bodyDef.isBullet = g_sh_is_bullet;
    g_sh_body = b3CreateBody(g_world, &bodyDef);
    g_sh_body_live = true;
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.enableSensorEvents = true;
    shapeDef.baseMaterial.friction = 0.8f;
    shapeDef.baseMaterial.rollingResistance = 0.01f;
    b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, 0.25f};
    g_sh_shape = b3CreateSphereShape(g_sh_body, &shapeDef, &sphere);
}

void build_sensor_hits() {
    ignore add_ground_box(10.0f);
    b3BodyId groundId;
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.name = "ground";
        groundId = b3CreateBody(g_world, &bodyDef);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        b3BoxHull wallBox = b3MakeTransformedBoxHull(0.1f, 5.0f, 5.0f,
            b3Transform{b3Vec3{10.0f, 5.0f, 0.0f}, b3Quat_identity});
        ignore b3CreateHullShape(groundId, &shapeDef, &wallBox.base);
    }
    g_sh_grid_mesh = b3CreateGridMesh(2, 2, 5.0f, 0, true);
    // Static sensor
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.name = "static sensor";
        bodyDef.position = b3Pos{-4.0f, 6.0f, 0.0f};
        bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, 0.5f * PI_F);
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        shapeDef.isSensor = true;
        shapeDef.enableSensorEvents = true;
        g_sh_static_sensor = b3CreateMeshShape(bodyId, &shapeDef, g_sh_grid_mesh, b3Vec3_one);
    }
    // Kinematic sensor
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.name = "kinematic sensor";
        bodyDef.type = b3_kinematicBody;
        bodyDef.position = b3Pos{0.0f, 6.0f, 0.0f};
        bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, 0.5f * PI_F);
        bodyDef.linearVelocity = b3Vec3{0.5f, 0.0f, 0.0f};
        g_sh_kinematic_body = b3CreateBody(g_world, &bodyDef);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        shapeDef.isSensor = true;
        shapeDef.enableSensorEvents = true;
        g_sh_kinematic_sensor = b3CreateMeshShape(g_sh_kinematic_body, &shapeDef,
                                                  g_sh_grid_mesh, b3Vec3_one);
    }
    // Dynamic sensor
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.name = "dynamic sensor";
        bodyDef.type = b3_dynamicBody;
        bodyDef.position = b3Pos{4.0f, 1.0f, 0.0f};
        g_sh_dynamic_body = b3CreateBody(g_world, &bodyDef);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        shapeDef.isSensor = true;
        shapeDef.enableSensorEvents = true;
        b3Capsule capsule = b3Capsule{b3Pos{0.0f, 1.0f, 0.0f}, b3Pos{0.0f, 9.0f, 0.0f}, 0.1f};
        g_sh_dynamic_sensor = b3CreateCapsuleShape(g_sh_dynamic_body, &shapeDef, &capsule);
        b3Pos pivot = b3OffsetPos(bodyDef.position, b3Vec3{0.0f, 6.0f, 0.0f});
        b3PrismaticJointDef jointDef = b3DefaultPrismaticJointDef();
        jointDef.base.bodyIdA = groundId;
        jointDef.base.bodyIdB = g_sh_dynamic_body;
        jointDef.base.localFrameA.p = b3Body_GetLocalPoint(groundId, pivot);
        jointDef.base.localFrameB.p = b3Body_GetLocalPoint(g_sh_dynamic_body, pivot);
        jointDef.enableMotor = true;
        jointDef.maxMotorForce = 1000.0f;
        jointDef.motorSpeed = 0.5f;
        g_sh_joint = b3CreatePrismaticJoint(g_world, &jointDef);
    }
    g_sh_begin_count = 0;
    g_sh_end_count = 0;
    g_sh_body_live = false;
    g_sh_transform_count = 0;
    g_sh_is_bullet = true;
    sensor_hits_launch();
}

void destroy_sensor_hits() {
    b3DestroyMesh(g_sh_grid_mesh);
}

bool sensor_hits_controls() {
    ignore ImGui_Checkbox("Bullet", &g_sh_is_bullet);
    if ImGui_Button("Launch", ImVec2{0.0f, 0.0f}) || ImGui_IsKeyDown(ImGuiKey_B) {
        sensor_hits_launch();
    }
    return true;
}

void sensor_hits_collect_transforms(b3ShapeId sensorShapeId) {
    const i32 capacity = 5;
    b3ShapeId[capacity] visitorIds;
    i32 count = b3Shape_GetSensorData(sensorShapeId, cast(b3ShapeId*, &visitorIds), capacity);
    for i32 i = 0; i < count && g_sh_transform_count < SH_TRANSFORM_CAPACITY; i += 1 {
        b3BodyId sensorBodyId = b3Shape_GetBody(sensorShapeId);
        b3WorldTransform t = b3Body_GetTransform(sensorBodyId);
        t.p = b3Body_GetWorldCenter(sensorBodyId);
        g_sh_transforms[g_sh_transform_count] = t;
        g_sh_transform_count += 1;
    }
}

void step_sensor_hits(f32 timeStep) {
    ignore timeStep;
    b3Pos p = b3Body_GetPosition(g_sh_kinematic_body);
    if p.x > 1.0f {
        b3Body_SetLinearVelocity(g_sh_kinematic_body, b3Vec3{-0.5f, 0.0f, 0.0f});
    } else if p.x < -1.0f {
        b3Body_SetLinearVelocity(g_sh_kinematic_body, b3Vec3{0.5f, 0.0f, 0.0f});
    }
    f32 x = b3PrismaticJoint_GetTranslation(g_sh_joint);
    if x > 1.0f {
        b3PrismaticJoint_SetMotorSpeed(g_sh_joint, -0.5f);
    } else if x < -1.0f {
        b3PrismaticJoint_SetMotorSpeed(g_sh_joint, 0.5f);
    }
    for i32 i = 0; i < g_sh_transform_count; i += 1 {
        dbg_axes(g_sh_transforms[i], 0.1f);
    }
    b3SensorEvents sensorEvents = b3World_GetSensorEvents(g_world);
    g_sh_begin_count += sensorEvents.beginCount;
    g_sh_end_count += sensorEvents.endCount;
    for i32 i = 0; i < sensorEvents.beginCount; i += 1 {
        b3SensorBeginTouchEvent* event = sensorEvents.beginEvents + i;
        sensor_hits_collect_transforms(event.sensorShapeId);
    }
    u8[128] buf;
    ignore snprintf(cast(u8*, &buf), 128, "begin touch count = %d", g_sh_begin_count);
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 128, "end touch count = %d", g_sh_end_count);
    draw_text_line(cast(u8*, &buf));
}

// samples/sample_events.cpp PersistentContact
b3ContactId g_pc_contact;
b3MeshData* g_pc_mesh_data;

void build_persistent_contact() {
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        b3BodyId groundId = b3CreateBody(g_world, &bodyDef);
        g_pc_mesh_data = b3CreateGridMesh(20, 20, 2.0f, 2, true);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        ignore b3CreateMeshShape(groundId, &shapeDef, g_pc_mesh_data, b3Vec3_one);
    }
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;
        bodyDef.position = b3Pos{-18.0f, 1.0f, 0.5f};
        bodyDef.linearVelocity = b3Vec3{4.0f, 0.0f, 0.0f};
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        shapeDef.density = 20.0f;
        shapeDef.enableContactEvents = true;
        shapeDef.baseMaterial.rollingResistance = 0.01f;
        b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, 0.5f};
        ignore b3CreateSphereShape(bodyId, &shapeDef, &sphere);
    }
    g_pc_contact = b3ContactId{};
}

void destroy_persistent_contact() {
    b3DestroyMesh(g_pc_mesh_data);
}

void step_persistent_contact(f32 timeStep) {
    ignore timeStep;
    b3ContactEvents events = b3World_GetContactEvents(g_world);
    for i32 i = 0; i < events.beginCount && i < 1; i += 1 {
        g_pc_contact = events.beginEvents[i].contactId;
    }
    for i32 i = 0; i < events.endCount; i += 1 {
        if g_pc_contact.index1 == events.endEvents[i].contactId.index1
           && g_pc_contact.world0 == events.endEvents[i].contactId.world0
           && g_pc_contact.generation == events.endEvents[i].contactId.generation {
            g_pc_contact = b3ContactId{};
            break;
        }
    }
    if g_pc_contact.index1 != 0 && b3Contact_IsValid(g_pc_contact) {
        b3ContactData data = b3Contact_GetData(g_pc_contact);
        b3Pos centerOfMass = b3Body_GetWorldCenter(b3Shape_GetBody(data.shapeIdA));
        for i32 i = 0; i < data.manifoldCount; i += 1 {
            b3Manifold* manifold = data.manifolds + i;
            b3Vec3 normal = manifold.normal;
            for i32 j = 0; j < manifold.pointCount; j += 1 {
                b3ManifoldPoint* manifoldPoint = manifold.points + j;
                b3Pos p1 = b3OffsetPos(centerOfMass, manifoldPoint.anchorA);
                b3Pos p2 = b3OffsetPos(p1, b3MulSV(manifoldPoint.totalNormalImpulse, normal));
                dbg_line(p1, p2, b3_colorCrimson);
                adapter_point(p1, 6.0f, b3_colorCrimson, null);
                u8[64] buf;
                ignore snprintf(cast(u8*, &buf), 64, "%.2f",
                                cast(f64, manifoldPoint.totalNormalImpulse));
                dbg_string_3d(p1, make_color(b3_colorGray), cast(u8*, &buf));
            }
        }
    } else {
        g_pc_contact = b3ContactId{};
    }
}

// samples/sample_events.cpp HitEvent
const i32 HE_MAX_EVENTS = 32;
b3MeshData* g_he_grid_mesh;
b3ContactHitEvent[HE_MAX_EVENTS] g_he_events;
i32 g_he_event_count;

void build_hit_event() {
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        b3BodyId groundId = b3CreateBody(g_world, &bodyDef);

        const i32 materialCount = 6;
        g_he_grid_mesh = b3CreateGridMesh(20, 20, 8.0f, materialCount, true);

        b3ShapeDef shapeDef = b3DefaultShapeDef();
        b3SurfaceMaterial[materialCount] materials;
        for i32 i = 0; i < materialCount; i += 1 {
            materials[i] = b3DefaultSurfaceMaterial();
            materials[i].userMaterialId = cast(u64, i + 1);
        }
        shapeDef.materials = cast(b3SurfaceMaterial*, &materials);
        shapeDef.materialCount = materialCount;
        ignore b3CreateMeshShape(groundId, &shapeDef, g_he_grid_mesh, b3Vec3_one);

        // b3BoxHull groundBox = b3MakeTransformedBoxHull( 80.0f, 1.0f, 80.0f, { { 0.0f, -1.0f, 0.0f }, b3Quat_identity } );
        // b3CreateHullShape( groundId, &shapeDef, &groundBox.base );
    }

    b3WeldJointDef jointDef = b3DefaultWeldJointDef();
    jointDef.angularHertz = 10.0f;
    jointDef.angularDampingRatio = 2.0f;

    f32 r = 0.75f;
    f32 y = r;
    f32 l = 1.5f;
    f32 offset = 0.05f;

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.enableHitEvents = true;
    shapeDef.baseMaterial.rollingResistance = 0.2f;
    shapeDef.baseMaterial.userMaterialId = cast(u64, 42);
    shapeDef.updateBodyMass = false;

    b3Vec3 origin = b3Vec3{0.0f, 0.0f, 0.0f};

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = b3OffsetPos(b3Pos_zero, origin);

    b3BodyId prevBodyId = b3BodyId{};
    b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);

    i32 shapeCount = 22;
    f32 velocityScale = 0.5f;
    i32 shapesPerBody = 3;

    for i32 i = 0; i < shapeCount; i += 1 {
        b3Capsule capsule = b3Capsule{b3Pos{offset, y, 0.0f}, b3Pos{0.0f, y + l, -offset}, r};
        ignore b3CreateCapsuleShape(bodyId, &shapeDef, &capsule);

        if (i + 1) % shapesPerBody == 0 || i == shapeCount - 1 {
            b3Body_ApplyMassFromShapes(bodyId);

            b3Pos center = b3Body_GetWorldCenter(bodyId);
            b3Vec3 omega = b3Vec3{0.0f, 0.0f, -1.0f * velocityScale};
            b3Vec3 v = b3Cross(omega, b3SubPos(center, b3OffsetPos(b3Pos_zero, origin)));
            b3Body_SetAngularVelocity(bodyId, omega);
            b3Body_SetLinearVelocity(bodyId, v);

            if i < shapeCount - 1 {
                prevBodyId = bodyId;

                if i < shapeCount - 1 {
                    bodyId = b3CreateBody(g_world, &bodyDef);

                    if prevBodyId.index1 != 0 {
                        jointDef.base.bodyIdA = prevBodyId;
                        jointDef.base.bodyIdB = bodyId;
                        jointDef.base.localFrameA.p = b3Pos{0.0f, y + l + r, 0.0f};
                        jointDef.base.localFrameB.p = b3Pos{0.0f, y + l + r, 0.0f};
                        ignore b3CreateWeldJoint(g_world, &jointDef);
                    }

                    velocityScale *= 0.75f;
                }
            }
        }

        y += l + 2.0f * r;
        r = 0.95f * r;
        offset = -offset;
    }

    g_he_event_count = 0;
}

void destroy_hit_event() {
    b3DestroyMesh(g_he_grid_mesh);
}

void step_hit_event(f32 timeStep) {
    ignore timeStep;

    b3ContactEvents events = b3World_GetContactEvents(g_world);
    for i32 i = 0; i < events.hitCount && g_he_event_count < HE_MAX_EVENTS; i += 1 {
        g_he_events[g_he_event_count] = events.hitEvents[i];
        g_he_event_count += 1;
    }

    b3Transform transform = b3Transform{b3Vec3{0.0f, 0.1f, 0.0f}, b3Quat_identity};
    dbg_axes(b3MakeWorldTransform(transform), 4.0f);

    u8[128] buf;
    for i32 i = 0; i < g_he_event_count; i += 1 {
        // void DrawLine(Scene * Scene, b3Vector3 Vertex1, b3Vector3 Vertex2, b3Color Color);
        // void DrawPoint(Scene * Scene, b3Vector3 Point, b3Color Color, float Size);
        b3Pos p1 = g_he_events[i].point;
        b3Pos p2 = b3OffsetPos(p1, b3MulSV(-g_he_events[i].approachSpeed,
                                           g_he_events[i].normal));
        adapter_point(p1, 10.0f, b3_colorYellow, null);
        dbg_line(p1, p2, b3_colorYellow);
        ignore snprintf(cast(u8*, &buf), 128, "%.1f, %d",
                        cast(f64, g_he_events[i].approachSpeed),
                        cast(i32, g_he_events[i].userMaterialIdA));
        dbg_string_3d(p1, make_color(b3_colorWhite), cast(u8*, &buf));
    }
    ignore snprintf(cast(u8*, &buf), 128, "event count = %d", g_he_event_count);
    draw_text_line(cast(u8*, &buf));
}
