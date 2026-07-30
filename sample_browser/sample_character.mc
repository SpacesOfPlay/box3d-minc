// Character scenes. Ports of samples/sample_character.cpp.

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
import mesh_loader;
import mover_shim;
import box3d_mover;
import box3d_rbchar;

// samples/sample_character.cpp CapsulePlane
const i32 CP_PLANE_CAPACITY = 3;
b3WorldTransform g_cp_transform;
b3Capsule g_cp_capsule;
b3CollisionPlane[CP_PLANE_CAPACITY] g_cp_planes;
i32 g_cp_plane_count;
b3Pos g_cp_base_translation;
b3Pos g_cp_origin;
i32 g_cp_base_x;
i32 g_cp_base_y;
bool g_cp_tracking;

void build_capsule_plane() {
    g_cp_transform.p = b3Pos{0.0f, 1.0f, 0.4f};
    g_cp_transform.q = b3Quat_identity;
    g_cp_capsule = b3Capsule{b3Pos{0.0f, -0.5f, 0.0f}, b3Pos{0.0f, 0.5f, 0.0f}, 0.25f};

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.position = b3Pos{0.0f, 1.0f, 1.0f};
    b3BodyId body = b3CreateBody(g_world, &bodyDef);

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    b3BoxHull box = b3MakeBoxHull(0.5f, 0.5f, 0.5f);
    ignore b3CreateHullShape(body, &shapeDef, &box.base);

    g_cp_base_translation = b3Pos_zero;
    g_cp_base_x = 0;
    g_cp_base_y = 0;
    g_cp_origin = b3Pos_zero;
    g_cp_tracking = false;
    g_cp_plane_count = 0;
}

void capsule_plane_solve() {
    b3PlaneSolverResult result = b3SolvePlanes(b3Vec3_zero,
                                               cast(b3CollisionPlane*, &g_cp_planes),
                                               g_cp_plane_count);
    g_cp_transform.p = b3OffsetPos(g_cp_transform.p, result.delta);
}

bool capsule_plane_controls() {
    if ImGui_Button("Solve", ImVec2{0.0f, 0.0f}) {
        capsule_plane_solve();
    }
    return true;
}

bool capsule_plane_result_fcn(b3ShapeId shape, b3PlaneResult* results, i32 planeCount,
                              void* context) {
    ignore shape;
    ignore context;
    for i32 i = 0; i < planeCount && g_cp_plane_count < 3; i += 1 {
        g_cp_planes[g_cp_plane_count] = b3CollisionPlane{results[i].plane, FLT_MAX, 0.0f, true};
        g_cp_plane_count += 1;
    }
    return true;
}

bool capsule_plane_mouse_down(f32 px, f32 py, i32 button, i32 modifiers) {
    if button == 0 && (modifiers & 4) == 0 {
        PickRay pickRay = build_pick_ray(px, py);
        b3Vec3 dir = b3Normalize(b3Vec3{pickRay.translation.x, pickRay.translation.y,
                                        pickRay.translation.z});
        g_cp_origin = b3OffsetPos(b3Pos{pickRay.origin.x, pickRay.origin.y, pickRay.origin.z},
                                  b3MulSV(10.0f, dir));
        g_cp_base_translation = g_cp_transform.p;
        g_cp_tracking = true;
        return true;
    }
    return false;
}

void capsule_plane_mouse_up(f32 px, f32 py, i32 button) {
    ignore px; ignore py; ignore button;
    g_cp_tracking = false;
}

void capsule_plane_mouse_move(f32 px, f32 py) {
    if g_cp_tracking {
        PickRay pickRay = build_pick_ray(px, py);
        b3Vec3 dir = b3Normalize(b3Vec3{pickRay.translation.x, pickRay.translation.y,
                                        pickRay.translation.z});
        b3Pos origin = b3OffsetPos(b3Pos{pickRay.origin.x, pickRay.origin.y, pickRay.origin.z},
                                   b3MulSV(10.0f, dir));
        g_cp_transform.p = b3OffsetPos(g_cp_base_translation, b3SubPos(origin, g_cp_origin));
    }
}

void step_capsule_plane(f32 timeStep) {
    ignore timeStep;

    g_cp_plane_count = 0;

    dbg_solid_capsule(g_cp_transform, g_cp_capsule, make_color(b3_colorGreen));

    b3QueryFilter filter = b3DefaultQueryFilter();
    b3Capsule capsule = b3Capsule{g_cp_capsule.center1, g_cp_capsule.center2,
                                  g_cp_capsule.radius};
    b3World_CollideMover(g_world, g_cp_transform.p, &capsule, filter,
                         capsule_plane_result_fcn, null);

    for i32 i = 0; i < g_cp_plane_count; i += 1 {
        b3Plane plane = g_cp_planes[i].plane;
        b3Pos p1 = b3OffsetPos(g_cp_transform.p,
                               b3MulSV(plane.offset - g_cp_capsule.radius, plane.normal));
        b3Pos p2 = b3OffsetPos(p1, b3MulSV(0.1f, plane.normal));
        adapter_point(p1, 5.0f, b3_colorYellow, null);
        dbg_line(p1, p2, b3_colorYellow);
    }

    dbg_ground_grid(10);
    dbg_line(b3Pos_zero, b3OffsetPos(b3Pos_zero, b3MulSV(2.0f, b3Vec3_axisX)), b3_colorRed);
    dbg_line(b3Pos_zero, b3OffsetPos(b3Pos_zero, b3MulSV(2.0f, b3Vec3_axisY)), b3_colorGreen);
    dbg_line(b3Pos_zero, b3OffsetPos(b3Pos_zero, b3MulSV(2.0f, b3Vec3_axisZ)), b3_colorBlue);
}

// samples/sample_character.cpp MoverOverlap
const i32 MO_PLANE_CAPACITY = 32;
b3WorldTransform g_mo_transform;
b3Capsule g_mo_capsule;
b3PlaneResult[MO_PLANE_CAPACITY] g_mo_results;
i32 g_mo_plane_count;
i32 g_mo_zero_normal_count;
b3Pos g_mo_base_translation;
b3Pos g_mo_origin;
bool g_mo_tracking;

void build_mover_overlap() {
    g_mo_capsule = b3Capsule{b3Pos{0.0f, -0.5f, 0.0f}, b3Pos{0.0f, 0.5f, 0.0f}, 0.35f};
    g_mo_transform.p = b3Pos{0.0f, 3.5f, 0.0f};
    g_mo_transform.q = b3Quat_identity;

    b3ShapeDef shapeDef = b3DefaultShapeDef();

    // Static sphere
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{-3.0f, 1.0f, 0.0f};
        b3BodyId body = b3CreateBody(g_world, &bodyDef);
        b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, 0.6f};
        ignore b3CreateSphereShape(body, &shapeDef, &sphere);
    }

    // Static capsule
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{0.0f, 1.0f, 0.0f};
        b3BodyId body = b3CreateBody(g_world, &bodyDef);
        b3Capsule capsule = b3Capsule{b3Pos{0.0f, 0.0f, -0.7f}, b3Pos{0.0f, 0.0f, 0.7f}, 0.4f};
        ignore b3CreateCapsuleShape(body, &shapeDef, &capsule);
    }

    // Static box hull
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{3.0f, 1.0f, 0.0f};
        b3BodyId body = b3CreateBody(g_world, &bodyDef);
        b3BoxHull box = b3MakeBoxHull(0.6f, 0.6f, 0.6f);
        ignore b3CreateHullShape(body, &shapeDef, &box.base);
    }

    g_mo_base_translation = b3Pos_zero;
    g_mo_origin = b3Pos_zero;
    g_mo_tracking = false;
    g_mo_plane_count = 0;
    g_mo_zero_normal_count = 0;
}

bool mover_overlap_result_fcn(b3ShapeId shape, b3PlaneResult* results, i32 planeCount,
                              void* context) {
    ignore shape;
    ignore context;
    for i32 i = 0; i < planeCount && g_mo_plane_count < MO_PLANE_CAPACITY; i += 1 {
        g_mo_results[g_mo_plane_count] = results[i];
        g_mo_plane_count += 1;
    }
    return true;
}

bool mover_overlap_mouse_down(f32 px, f32 py, i32 button, i32 modifiers) {
    if button == 0 && (modifiers & 4) == 0 {
        PickRay pickRay = build_pick_ray(px, py);
        b3Vec3 dir = b3Normalize(b3Vec3{pickRay.translation.x, pickRay.translation.y,
                                        pickRay.translation.z});
        g_mo_origin = b3OffsetPos(b3Pos{pickRay.origin.x, pickRay.origin.y, pickRay.origin.z},
                                  b3MulSV(10.0f, dir));
        g_mo_base_translation = g_mo_transform.p;
        g_mo_tracking = true;
        return true;
    }
    return false;
}

void mover_overlap_mouse_up(f32 px, f32 py, i32 button) {
    ignore px; ignore py; ignore button;
    g_mo_tracking = false;
}

void mover_overlap_mouse_move(f32 px, f32 py) {
    if g_mo_tracking {
        PickRay pickRay = build_pick_ray(px, py);
        b3Vec3 dir = b3Normalize(b3Vec3{pickRay.translation.x, pickRay.translation.y,
                                        pickRay.translation.z});
        b3Pos origin = b3OffsetPos(b3Pos{pickRay.origin.x, pickRay.origin.y, pickRay.origin.z},
                                   b3MulSV(10.0f, dir));
        g_mo_transform.p = b3OffsetPos(g_mo_base_translation, b3SubPos(origin, g_mo_origin));
    }
}

bool mover_overlap_controls() {
    ImGui_Text("planes: %d", g_mo_plane_count);
    ImGui_Text("degenerate normals: %d", g_mo_zero_normal_count);
    return true;
}

void step_mover_overlap(f32 timeStep) {
    ignore timeStep;

    // Query overlapping shapes. The mover capsule is relative to the body origin.
    b3Capsule mover = b3Capsule{g_mo_capsule.center1, g_mo_capsule.center2,
                                g_mo_capsule.radius};
    g_mo_plane_count = 0;
    g_mo_zero_normal_count = 0;
    b3QueryFilter filter = b3DefaultQueryFilter();
    b3World_CollideMover(g_world, g_mo_transform.p, &mover, filter, mover_overlap_result_fcn,
                         null);

    // Mover at the queried position.
    dbg_solid_capsule(g_mo_transform, g_mo_capsule, make_color(b3_colorYellow));

    // One arrow per returned plane, drawn from the contact point along the
    // normal. A degenerate (zero) normal is drawn red to surface the bug.
    b3CollisionPlane[MO_PLANE_CAPACITY] solverPlanes;
    for i32 i = 0; i < g_mo_plane_count; i += 1 {
        b3PlaneResult r = g_mo_results[i];
        bool valid = b3IsNormalized(r.plane.normal);
        b3HexColor color = valid ? b3_colorLimeGreen : b3_colorRed;
        b3Pos rp = b3OffsetPos(g_mo_transform.p, r.point);
        adapter_point(rp, 6.0f, color, null);
        dbg_arrow(rp, b3OffsetPos(rp, b3MulSV(0.5f, r.plane.normal)), color);
        if valid == false {
            g_mo_zero_normal_count += 1;
        }
        solverPlanes[i] = b3CollisionPlane{r.plane, FLT_MAX, 0.0f, true};
    }

    // Solve the planes and show the pushed-out capsule pose.
    b3PlaneSolverResult solved = b3SolvePlanes(b3Vec3_zero,
                                               cast(b3CollisionPlane*, &solverPlanes),
                                               g_mo_plane_count);
    b3WorldTransform pushed;
    pushed.p = b3OffsetPos(g_mo_transform.p, solved.delta);
    pushed.q = g_mo_transform.q;
    dbg_solid_capsule(pushed, g_mo_capsule, make_color(b3_colorCyan));

    draw_text_line("drag the capsule with the left mouse to push it into the shapes");
    draw_text_line("yellow = queried pose, cyan = solved push-out, lime = valid plane normals");
    u8[128] buf;
    ignore snprintf(cast(u8*, &buf), 128, "planes: %d   degenerate normals: %d",
                    g_mo_plane_count, g_mo_zero_normal_count);
    draw_text_line(cast(u8*, &buf));

    dbg_ground_grid(12);
}

// samples/sample_character.cpp BasicMover. The controller itself is
// upstream's samples/mover.cpp, transpiled to lib/box3d_mover.mc and
// bound to the browser by mover_shim.mc.
CharacterMover g_bm_mover;
MoverShapeUserData g_bm_enemy_shape;
MoverShapeUserData g_bm_friendly_shape;
b3MeshData* g_bm_level_mesh;
b3MeshData* g_bm_stairs;
b3MeshData* g_bm_torus;
b3HeightFieldData* g_bm_height_field;
b3ShapeId g_bm_ignore_shape_id;
bool g_bm_clip_velocity;

// upstream `{ 0.6f, 0.0f, 0 }` etc: friction, restitution,
// rollingResistance.
void bm_fill_materials(b3SurfaceMaterial* materials) {
    materials[0] = b3SurfaceMaterial{};
    materials[0].friction = 0.6f;
    materials[0].restitution = 0.0f;
    materials[0].rollingResistance = 0.0f;
    materials[1] = b3SurfaceMaterial{};
    materials[1].friction = 0.6f;
    materials[1].restitution = 1.0f;
    materials[1].rollingResistance = 1.0f;
    materials[2] = b3SurfaceMaterial{};
    materials[2].friction = 0.1f;
    materials[2].restitution = 0.0f;
    materials[2].rollingResistance = 2.0f;
}

void build_basic_mover() {
    b3Pos moverPosition = b3Pos{7.5f, 0.75f, 9.0f};

    CharacterMover_Initialize(&g_bm_mover, mover_sample(), moverPosition);

    {
        g_bm_level_mesh = create_mesh_data("data/meshes/test_map01.obj", 1.0f,
                                           false, false, true, true);
        b3BodyDef bodyDef = b3DefaultBodyDef();
        b3BodyId body = b3CreateBody(g_world, &bodyDef);

        b3ShapeDef shapeDef = b3DefaultShapeDef();
        b3SurfaceMaterial[3] materials;
        bm_fill_materials(cast(b3SurfaceMaterial*, &materials));
        shapeDef.materials = cast(b3SurfaceMaterial*, &materials);
        shapeDef.materialCount = 3;

        ignore b3CreateMeshShape(body, &shapeDef, g_bm_level_mesh, b3Vec3_one);

        {
            b3Transform transform = b3Transform{b3Vec3{4.0f, 1.0f, 14.0f}, b3Quat_identity};
            b3BoxHull box = b3MakeTransformedBoxHull(1.0f, 1.0f, 1.0f, transform);
            ignore b3CreateHullShape(body, &shapeDef, &box.base);
        }
        {
            b3Transform transform = b3Transform{b3Vec3{4.0f, 1.0f, 13.95f}, b3Quat_identity};
            b3BoxHull box = b3MakeTransformedBoxHull(1.0f, 1.0f, 1.0f, transform);
            ignore b3CreateHullShape(body, &shapeDef, &box.base);
        }
        {
            b3Transform transform = b3Transform{b3Vec3{5.8f, 1.0f, 13.7f},
                                                b3MakeQuatFromAxisAngle(b3Vec3_axisY, 0.1f * PI_F)};
            b3BoxHull box = b3MakeTransformedBoxHull(1.0f, 1.0f, 1.0f, transform);
            ignore b3CreateHullShape(body, &shapeDef, &box.base);
        }
    }

    {
        g_bm_stairs = create_mesh_data("data/meshes/stairs.obj", 1.0f,
                                        false, false, true, true);

        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{-10.0f, 0.0f, 0.0f};

        b3ShapeDef shapeDef = b3DefaultShapeDef();
        b3BodyId body = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateMeshShape(body, &shapeDef, g_bm_stairs, b3Vec3{0.75f, 0.75f, -1.5f});
    }

    {
        g_bm_torus = b3CreateTorusMesh(10, 12, 2.0f, 1.0f);

        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{-10.0f, 1.0f, -8.0f};
        bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisY, 0.5f * PI_F);

        b3ShapeDef shapeDef = b3DefaultShapeDef();
        b3BodyId body = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateMeshShape(body, &shapeDef, g_bm_torus, b3Vec3{-0.75f, 1.5f, 0.5f});
    }

    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{20.0f, 0.0f, 0.0f};
        b3BodyId body = b3CreateBody(g_world, &bodyDef);

        g_bm_height_field = b3CreateWave(50, 50, b3Vec3_one, 0.02f, 0.04f, true);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        b3SurfaceMaterial[3] materials;
        bm_fill_materials(cast(b3SurfaceMaterial*, &materials));
        shapeDef.materials = cast(b3SurfaceMaterial*, &materials);
        shapeDef.materialCount = 3;

        ignore b3CreateHeightFieldShape(body, &shapeDef, g_bm_height_field);
    }

    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{0.0f, 1.4f, 6.0f};

        b3ShapeDef shapeDef = b3DefaultShapeDef();
        g_bm_enemy_shape.maxPush = 1.0f;
        g_bm_enemy_shape.clipVelocity = true;

        b3Capsule capsule = b3Capsule{
            b3Pos{0.0f, -0.5f, 0.0f},
            b3Pos{0.0f, 0.5f, 0.0f},
            0.3f,
        };

        shapeDef.userData = cast(void*, &g_bm_enemy_shape);
        shapeDef.baseMaterial.customColor = cast(u32, b3_colorMediumVioletRed);
        b3BodyId body = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateCapsuleShape(body, &shapeDef, &capsule);
    }

    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{0.0f, 1.4f, 5.0f};

        b3ShapeDef shapeDef = b3DefaultShapeDef();
        g_bm_friendly_shape.maxPush = 0.01f;
        g_bm_friendly_shape.clipVelocity = false;

        b3Capsule capsule = b3Capsule{
            b3Pos{0.0f, -0.5f, 0.0f},
            b3Pos{0.0f, 0.5f, 0.0f},
            0.3f,
        };

        shapeDef.filter = b3Filter{};
        shapeDef.filter.categoryBits = cast(u64, 2);
        shapeDef.filter.maskBits = ~cast(u64, 0);
        shapeDef.filter.groupIndex = 0;
        shapeDef.userData = cast(void*, &g_bm_friendly_shape);
        shapeDef.baseMaterial.customColor = cast(u32, b3_colorLimeGreen);
        b3BodyId body = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateCapsuleShape(body, &shapeDef, &capsule);
    }

    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;
        bodyDef.position = b3Pos{7.0f, 5.0f, 0.0f};
        b3BodyId body = b3CreateBody(g_world, &bodyDef);

        b3ShapeDef shapeDef = b3DefaultShapeDef();
        b3Sphere sphere = b3Sphere{b3Vec3_zero, 0.5f};
        ignore b3CreateSphereShape(body, &shapeDef, &sphere);
    }

    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{7.0f, 2.0f, -3.0f};
        b3BodyId body = b3CreateBody(g_world, &bodyDef);

        b3ShapeDef shapeDef = b3DefaultShapeDef();
        shapeDef.baseMaterial.customColor = cast(u32, b3_colorFloralWhite);
        b3BoxHull box = b3MakeBoxHull(0.5f, 0.25f, 0.5f);
        g_bm_ignore_shape_id = b3CreateHullShape(body, &shapeDef, &box.base);
    }

    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        b3BodyId groundId = b3CreateBody(g_world, &bodyDef);

        bodyDef.type = b3_dynamicBody;
        bodyDef.position = b3Pos{-2.0f, 1.6f, 0.0f};
        bodyDef.gravityScale = 2.0f;
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);

        b3ShapeDef shapeDef = b3DefaultShapeDef();
        shapeDef.density = 1000.0f;

        b3BoxHull box = b3MakeBoxHull(0.75f, 1.5f, 0.1f);
        ignore b3CreateHullShape(bodyId, &shapeDef, &box.base);

        b3Quat axisQuat = b3ComputeQuatBetweenUnitVectors(b3Vec3_axisZ, b3Vec3_axisY);
        b3Vec3 offset = b3Vec3{-0.75f, 0.0f, 0.0f};

        b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
        jointDef.base.bodyIdA = groundId;
        jointDef.base.bodyIdB = bodyId;
        jointDef.base.localFrameA.p = b3ToVec3(b3OffsetPos(bodyDef.position, offset));
        jointDef.base.localFrameA.q = axisQuat;
        jointDef.base.localFrameB.p = offset;
        jointDef.base.localFrameB.q = axisQuat;

        jointDef.enableLimit = true;
        jointDef.lowerAngle = PI_F / 180.0f * -90.0f;
        jointDef.upperAngle = PI_F / 180.0f * 90.0f;
        jointDef.enableSpring = true;
        jointDef.hertz = 1.0f;
        jointDef.dampingRatio = 0.5f;
        jointDef.enableMotor = false;
        jointDef.maxMotorTorque = 100.0f;
        jointDef.base.drawScale = 2.0f;

        ignore b3CreateRevoluteJoint(g_world, &jointDef);
    }

    cam_third_person = false;
    g_bm_clip_velocity = true;
}

void destroy_basic_mover() {
    cam_third_person = false;
    sapp_lock_mouse(false);
    b3DestroyMesh(g_bm_level_mesh);
    b3DestroyMesh(g_bm_stairs);
    b3DestroyMesh(g_bm_torus);
    b3DestroyHeightField(g_bm_height_field);
}

bool basic_mover_controls() {
    bool thirdPerson = cam_third_person;
    if ImGui_Checkbox("Third Person (T)", &thirdPerson) {
        toggle_third_person();
    }

    ignore ImGui_Checkbox("Clip Velocity", &g_bm_clip_velocity);

    return true;
}

void basic_mover_keyboard(i32 key, i32 action, i32 mods) {
    ignore mods;
    if key == SAPP_KEYCODE_T && action == 1 {
        toggle_third_person();
    }
}

void step_basic_mover(f32 timeStep) {
    ignore timeStep;
    mover_sync();
    CharacterMover_Step(&g_bm_mover, &g_bm_ignore_shape_id, 1, g_bm_clip_velocity);

    b3WorldTransform axes;
    axes.p = b3Pos{0.0f, 0.0f, 0.02f};
    axes.q = b3Quat_identity;
    dbg_axes(axes, 2.0f);

    u8[128] buf;
    ignore snprintf(cast(u8*, &buf), 128, "third person (T) = %d",
                    cam_third_person ? 1 : 0);
    draw_text_line(cast(u8*, &buf));
}

// samples/sample_character.cpp RigidBodyCharacter. The controller is
// upstream's RigidbodyCharacter, moved to its own translation unit
// (ext/box3d/samples/rigidbody_character.cpp), transpiled to
// lib/box3d_rbchar.mc and bound to the browser by mover_shim.mc.
RigidbodyCharacter g_rbc_character;
b3MeshData* g_rbc_level_mesh;
b3MeshData* g_rbc_stairs;
b3MeshData* g_rbc_building;
b3MeshData* g_rbc_voxel01;
b3MeshData* g_rbc_voxel02;
b3HeightFieldData* g_rbc_height_field;
bool g_rbc_show_debug;

void build_rigid_body_character() {
    b3Pos startPosition = b3Pos{7.5f, 2.0f, 9.0f};

    RigidbodyCharacter_Initialize(&g_rbc_character, mover_sample(), startPosition);

    {
        g_rbc_level_mesh = create_mesh_data("data/meshes/test_map01.obj", 1.0f,
                                            false, false, true, true);
        b3BodyDef bodyDef = b3DefaultBodyDef();
        b3BodyId body = b3CreateBody(g_world, &bodyDef);

        b3ShapeDef shapeDef = b3DefaultShapeDef();
        b3SurfaceMaterial[3] materials;
        bm_fill_materials(cast(b3SurfaceMaterial*, &materials));
        shapeDef.materials = cast(b3SurfaceMaterial*, &materials);
        shapeDef.materialCount = 3;

        ignore b3CreateMeshShape(body, &shapeDef, g_rbc_level_mesh, b3Vec3_one);
    }

    {
        g_rbc_stairs = create_mesh_data("data/meshes/stairs.obj", 1.0f,
                                        false, false, true, true);
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{-10.0f, 0.0f, 0.0f};
        b3BodyId body = b3CreateBody(g_world, &bodyDef);

        b3ShapeDef shapeDef = b3DefaultShapeDef();
        ignore b3CreateMeshShape(body, &shapeDef, g_rbc_stairs,
                                 b3Vec3{0.75f, 0.75f, -1.5f});
    }

    {
        g_rbc_building = create_mesh_data("data/meshes/building.obj", 1.0f,
                                          false, false, true, true);
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{-5.0f, 0.0f, -10.0f};
        b3BodyId body = b3CreateBody(g_world, &bodyDef);

        b3ShapeDef shapeDef = b3DefaultShapeDef();
        ignore b3CreateMeshShape(body, &shapeDef, g_rbc_building, b3Vec3_one);
    }

    // Deviates from upstream, which passes 1.0f, false here. The voxel
    // .obj files are Z-up at 100x; every other sample loads them at
    // 0.01f, true. Unchanged, they stand on edge 3200 units tall.
    {
        g_rbc_voxel01 = create_mesh_data("data/meshes/voxel_mesh_01.obj", 0.01f,
                                         true, false, true, true);
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{10.0f, 0.0f, -10.0f};
        b3BodyId body = b3CreateBody(g_world, &bodyDef);

        b3ShapeDef shapeDef = b3DefaultShapeDef();
        ignore b3CreateMeshShape(body, &shapeDef, g_rbc_voxel01, b3Vec3_one);
    }
    {
        g_rbc_voxel02 = create_mesh_data("data/meshes/voxel_mesh_02.obj", 0.01f,
                                         true, false, true, true);
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{10.0f, 0.0f, 10.0f};
        b3BodyId body = b3CreateBody(g_world, &bodyDef);

        b3ShapeDef shapeDef = b3DefaultShapeDef();
        ignore b3CreateMeshShape(body, &shapeDef, g_rbc_voxel02, b3Vec3_one);
    }

    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{20.0f, 0.0f, 0.0f};
        b3BodyId body = b3CreateBody(g_world, &bodyDef);

        g_rbc_height_field = b3CreateWave(50, 50, b3Vec3_one, 0.02f, 0.04f, true);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        b3SurfaceMaterial[3] materials;
        bm_fill_materials(cast(b3SurfaceMaterial*, &materials));
        shapeDef.materials = cast(b3SurfaceMaterial*, &materials);
        shapeDef.materialCount = 3;

        ignore b3CreateHeightFieldShape(body, &shapeDef, g_rbc_height_field);
    }

    b3ShapeDef hullShapeDef = b3DefaultShapeDef();
    hullShapeDef.baseMaterial.friction = 0.6f;

    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{6.0f, 1.0f, 4.0f};
        bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, -20.0f * B3_DEG_TO_RAD);
        b3BodyId body = b3CreateBody(g_world, &bodyDef);
        hullShapeDef.baseMaterial.customColor = cast(u32, b3_colorOliveDrab);
        b3BoxHull box = b3MakeBoxHull(3.0f, 0.15f, 1.5f);
        ignore b3CreateHullShape(body, &hullShapeDef, &box.base);
    }

    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{6.0f, 2.0f, -4.0f};
        bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, -50.0f * B3_DEG_TO_RAD);
        b3BodyId body = b3CreateBody(g_world, &bodyDef);
        hullShapeDef.baseMaterial.customColor = cast(u32, b3_colorIndianRed);
        b3BoxHull box = b3MakeBoxHull(2.5f, 0.15f, 1.5f);
        ignore b3CreateHullShape(body, &hullShapeDef, &box.base);
    }

    for i32 i = 0; i < 3; ++i {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{-4.0f + 3.5f * cast(f32, i), 1.2f, -5.0f};
        b3BodyId body = b3CreateBody(g_world, &bodyDef);
        hullShapeDef.baseMaterial.customColor = cast(u32, b3_colorSlateGray);
        b3BoxHull box = b3MakeBoxHull(1.2f, 0.15f, 1.2f);
        ignore b3CreateHullShape(body, &hullShapeDef, &box.base);
    }

    for i32 i = 0; i < 5; ++i {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        f32 lipHeight = 0.05f + 0.08f * cast(f32, i);
        bodyDef.position = b3Pos{-8.0f, lipHeight, -1.0f + 2.0f * cast(f32, i)};
        b3BodyId body = b3CreateBody(g_world, &bodyDef);
        hullShapeDef.baseMaterial.customColor = cast(u32, b3_colorCornflowerBlue);
        b3BoxHull box = b3MakeBoxHull(1.0f, lipHeight, 0.6f);
        ignore b3CreateHullShape(body, &hullShapeDef, &box.base);
    }

    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{0.0f, 1.5f, 10.0f};
        b3BodyId body = b3CreateBody(g_world, &bodyDef);
        hullShapeDef.baseMaterial.customColor = cast(u32, b3_colorDarkSlateGray);
        b3BoxHull box = b3MakeBoxHull(4.0f, 1.5f, 0.2f);
        ignore b3CreateHullShape(body, &hullShapeDef, &box.base);
    }

    for i32 i = 0; i < 3; ++i {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;
        bodyDef.position = b3Pos{3.0f + 1.5f * cast(f32, i), 0.5f, 0.0f};
        b3BodyId body = b3CreateBody(g_world, &bodyDef);
        hullShapeDef.baseMaterial.customColor = cast(u32, b3_colorGold);
        b3BoxHull box = b3MakeBoxHull(0.4f, 0.4f, 0.4f);
        ignore b3CreateHullShape(body, &hullShapeDef, &box.base);
    }

    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;
        bodyDef.position = b3Pos{-3.0f, 1.0f, 0.0f};
        b3BodyId body = b3CreateBody(g_world, &bodyDef);
        hullShapeDef.baseMaterial.customColor = cast(u32, b3_colorOrange);
        b3Sphere sphere = b3Sphere{b3Vec3_zero, 0.5f};
        ignore b3CreateSphereShape(body, &hullShapeDef, &sphere);
    }

    cam_third_person = true;
    g_rbc_show_debug = true;
    sapp_lock_mouse(true);
}

void destroy_rigid_body_character() {
    cam_third_person = false;
    sapp_lock_mouse(false);
    b3DestroyMesh(g_rbc_level_mesh);
    b3DestroyMesh(g_rbc_stairs);
    b3DestroyMesh(g_rbc_building);
    b3DestroyMesh(g_rbc_voxel01);
    b3DestroyMesh(g_rbc_voxel02);
    b3DestroyHeightField(g_rbc_height_field);
}

void rigid_body_character_keyboard(i32 key, i32 action, i32 mods) {
    ignore mods;
    if key == SAPP_KEYCODE_T && action == 1 {
        toggle_third_person();
    }

    if key == SAPP_KEYCODE_V && action == 1 {
        g_rbc_show_debug = !g_rbc_show_debug;
    }
}

void step_rigid_body_character(f32 timeStep) {
    ignore timeStep;
    mover_sync();

    f32 hertz = g_hertz;
    f32 step = hertz > 0.0f ? 1.0f / hertz : 0.0f;

    b3Vec2 throttle = b3Vec2{0.0f, 0.0f};
    b3Vec3 forward = b3Neg(b3Vec3{cam_forward.x, cam_forward.y, cam_forward.z});
    b3Vec3 right = b3Vec3{cam_right.x, cam_right.y, cam_right.z};
    forward.y = 0.0f;

    f32 forwardLen = b3Length(forward);
    if forwardLen > 0.001f {
        forward = b3MulSV(1.0f / forwardLen, forward);
    }

    if cam_third_person {
        if is_key_down(SAPP_KEYCODE_W) { throttle.x += 1.0f; }
        if is_key_down(SAPP_KEYCODE_S) { throttle.x -= 1.0f; }
        if is_key_down(SAPP_KEYCODE_A) { throttle.y -= 1.0f; }
        if is_key_down(SAPP_KEYCODE_D) { throttle.y += 1.0f; }

        if is_key_down(SAPP_KEYCODE_SPACE) {
            RigidbodyCharacter_Jump(&g_rbc_character);
        }

        g_rbc_character.m_sprint = g_rbc_character.m_onGround
                                   && is_key_down(SAPP_KEYCODE_LEFT_SHIFT);
    }

    RigidbodyCharacter_Step(&g_rbc_character, step, forward, right, throttle);
}

void late_step_rigid_body_character(f32 timeStep) {
    ignore timeStep;
    f32 hertz = g_hertz;
    f32 step = hertz > 0.0f ? 1.0f / hertz : 0.0f;

    RigidbodyCharacter_LateStep(&g_rbc_character, step);

    b3Pos charPos = b3Body_GetPosition(g_rbc_character.m_bodyId);
    b3Pos pos = charPos;
    if cam_third_person {
        cam_pivot = float3{charPos.x, charPos.y, charPos.z};
        cam_rebuild_basis();

        // Keep the eye from clipping through geometry. Cast from the
        // character toward the eye and, on a hit, shorten the boom for
        // this frame only.
        f32 cameraRadius = 0.15f;
        b3Vec3 translation = b3SubPos(b3Pos{cam_eye.x, cam_eye.y, cam_eye.z}, charPos);
        f32 desiredDist = b3Length(translation);

        if desiredDist > 0.01f {
            b3QueryFilter filter = b3DefaultQueryFilter();
            b3RayResult rayResult = b3World_CastRayClosest(g_world, charPos,
                                                           translation, filter);

            if rayResult.hit {
                f32 clampedDist = rayResult.fraction * desiredDist - cameraRadius;
                if clampedDist < 0.1f {
                    clampedDist = 0.1f;
                }

                f32 savedRadius = cam_radius;
                cam_radius = b3MinFloat(savedRadius, clampedDist);
                cam_rebuild_basis();
                cam_radius = savedRadius;
            }
        }
    }

    if g_rbc_show_debug {
        RigidbodyCharacter_DrawDebug(&g_rbc_character);
    }

    b3WorldTransform axes;
    axes.p = b3Pos{0.0f, 0.0f, 0.02f};
    axes.q = b3Quat_identity;
    dbg_axes(axes, 2.0f);

    b3Vec3 vel = b3Body_GetLinearVelocity(g_rbc_character.m_bodyId);
    f32 speed = sqrtf(vel.x * vel.x + vel.z * vel.z);
    u8[160] buf;
    draw_text_line("Rigid Body Character (s&box-style)");
    ignore snprintf(cast(u8*, &buf), 160, "position: %.2f %.2f %.2f",
                    pos.x, pos.y, pos.z);
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 160,
                    "velocity: %.2f %.2f %.2f (horizontal: %.2f)",
                    vel.x, vel.y, vel.z, speed);
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 160, "on ground: %s | sprint: %s",
                    g_rbc_character.m_onGround ? "yes" : "no",
                    g_rbc_character.m_sprint ? "yes" : "no");
    draw_text_line(cast(u8*, &buf));
    draw_text_line("WASD=move Space=jump Shift=sprint T=camera V=debug");
}

bool rigid_body_character_controls() {
    bool thirdPerson = cam_third_person;
    if ImGui_Checkbox("Third Person (T)", &thirdPerson) {
        toggle_third_person();
    }

    ignore ImGui_Checkbox("Debug (V)", &g_rbc_show_debug);

    ImGui_Separator();
    u8[160] buf;
    ignore snprintf(cast(u8*, &buf), 160, "Ground: %s",
                    g_rbc_character.m_onGround ? "YES" : "NO");
    ImGui_Text("%s", cast(u8*, &buf));

    b3Vec3 vel = b3Body_GetLinearVelocity(g_rbc_character.m_bodyId);
    f32 hSpeed = sqrtf(vel.x * vel.x + vel.z * vel.z);
    ignore snprintf(cast(u8*, &buf), 160, "Speed: %.2f m/s", hSpeed);
    ImGui_Text("%s", cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 160, "Vertical: %.2f m/s", vel.y);
    ImGui_Text("%s", cast(u8*, &buf));

    b3Pos mc = g_rbc_character.m_massCenterWorld;
    b3Pos pos = b3Body_GetPosition(g_rbc_character.m_bodyId);
    ignore snprintf(cast(u8*, &buf), 160, "Mass center offset: %.2f", mc.y - pos.y);
    ImGui_Text("%s", cast(u8*, &buf));

    return true;
}
