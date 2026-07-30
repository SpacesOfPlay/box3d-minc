// Continuous-collision scenes: fast bodies against thin geometry.

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
import sample_mesh;
import debug_adapter;
import sample_benchmark;
import sample_bodies;
import sample_robustness;
import sample_stacking;
import sample_joint;
import sample_compound;
import sample_world;

void build_bullet_vs_stack() {
    ignore add_ground_box(50.0f);
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{0.0f, -1.0f, 0.0f};
        b3BodyId groundBodyId = b3CreateBody(g_world, &bodyDef);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        b3Transform transform;
        transform.p = b3Vec3{-1.0f, 5.0f, 0.0f};
        transform.q = b3Quat_identity;
        b3BoxHull wallBox = b3MakeTransformedBoxHull(0.1f, 5.0f, 10.0f, transform);
        ignore b3CreateHullShape(groundBodyId, &shapeDef, &wallBox.base);
    }
    b3BoxHull box = b3MakeBoxHull(0.5f, 0.5f, 0.5f);
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    for i32 row = 0; row < 10; row++ {
        bodyDef.position = b3Pos{0.0f, 0.5f + 1.1f * cast(f32, row), 0.0f};
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateHullShape(bodyId, &shapeDef, &box.base);
    }
    g_bullet_alive = false;
}

// BulletVersusStack::Launch, upstream key 'L'
void bullet_stack_launch() {
    if g_bullet_alive {
        b3DestroyBody(g_bullet_body);
        g_bullet_alive = false;
    }
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.isBullet = true;
    bodyDef.position = b3Pos{20.5f, 5.5f, 0.0f};
    bodyDef.linearVelocity = b3Vec3{-500.0f, 0.0f, 0.0f};
    g_bullet_body = b3CreateBody(g_world, &bodyDef);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.density *= 10.0f;
    b3Sphere sphere;
    sphere.center = b3Vec3{0.0f, 0.0f, 0.0f};
    sphere.radius = 0.25f;
    ignore b3CreateSphereShape(g_bullet_body, &shapeDef, &sphere);
    g_bullet_alive = true;
}

// samples/sample_continuous.cpp ThinWall
void build_thin_wall() {
    ignore add_ground_box(40.0f);
    b3BodyDef bodyDef = b3DefaultBodyDef();
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    bodyDef.position = b3Pos{0.0f, 10.0f, 0.0f};
    bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisX, 90.0f * PI_F / 180.0f);
    b3BodyId wallId = b3CreateBody(g_world, &bodyDef);
    b3BoxHull wallBox = b3MakeBoxHull(10.0f, 0.1f, 10.0f);
    ignore b3CreateHullShape(wallId, &shapeDef, &wallBox.base);

    bodyDef.type = b3_dynamicBody;
    bodyDef.rotation = b3Quat_identity;
    shapeDef.baseMaterial.rollingResistance = 0.1f;

    bodyDef.position = b3Pos{-5.0f, 10.0f, 20.0f};
    bodyDef.linearVelocity = b3Vec3{0.0f, 0.0f, -180.0f};
    bodyDef.angularVelocity = b3Vec3{20.0f, 0.0f, 0.0f};
    b3BodyId sphereBodyId = b3CreateBody(g_world, &bodyDef);
    b3Sphere sphere;
    sphere.center = b3Vec3{0.0f, 0.0f, 0.0f};
    sphere.radius = 0.1f;
    ignore b3CreateSphereShape(sphereBodyId, &shapeDef, &sphere);

    bodyDef.position = b3Pos{0.0f, 10.0f, 20.0f};
    bodyDef.linearVelocity = b3Vec3{0.0f, 0.0f, -180.0f};
    bodyDef.angularVelocity = b3Vec3{20.0f, -5.0f, 0.0f};
    b3BodyId capsuleBodyId = b3CreateBody(g_world, &bodyDef);
    b3Capsule capsule;
    capsule.center1 = b3Vec3{-0.3f, 0.0f, 0.0f};
    capsule.center2 = b3Vec3{0.3f, 0.0f, 0.0f};
    capsule.radius = 0.1f;
    ignore b3CreateCapsuleShape(capsuleBodyId, &shapeDef, &capsule);

    bodyDef.position = b3Pos{5.0f, 10.0f, 20.0f};
    bodyDef.linearVelocity = b3Vec3{0.0f, 0.0f, -180.0f};
    bodyDef.angularVelocity = b3Vec3{20.0f, 5.0f, 0.0f};
    b3BodyId boxBodyId = b3CreateBody(g_world, &bodyDef);
    b3BoxHull box = b3MakeBoxHull(0.4f, 0.1f, 0.1f);
    ignore b3CreateHullShape(boxBodyId, &shapeDef, &box.base);
}

bool bullet_controls() {
    if ImGui_Button("Launch", ImVec2{0.0f, 0.0f}) {
        bullet_stack_launch();
    }
    return true;
}

// sample_continuous.cpp BounceHouse: a frictionless, fully-elastic sphere
// with no gravity fired into a five-walled box arena, ricocheting forever.
void build_bounce_house() {
    ignore add_ground_box(10.0f);
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.position = b3Pos{0.0f, -1.0f, 0.0f};
    b3BodyId groundBodyId = b3CreateBody(g_world, &bodyDef);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    // four walls, baked at body-local offsets on the ground body
    bounce_wall(groundBodyId, &shapeDef, 0.1f, 5.0f, 10.0f, 10.0f, 5.0f, 0.0f);
    bounce_wall(groundBodyId, &shapeDef, 0.1f, 5.0f, 10.0f, -10.0f, 5.0f, 0.0f);
    bounce_wall(groundBodyId, &shapeDef, 10.0f, 5.0f, 0.1f, 0.0f, 5.0f, -10.0f);
    bounce_wall(groundBodyId, &shapeDef, 10.0f, 5.0f, 0.1f, 0.0f, 5.0f, 10.0f);

    bodyDef.type = b3_dynamicBody;
    bodyDef.gravityScale = 0.0f;
    bodyDef.position = b3Pos{-8.0f, 4.0f, 0.0f};
    bodyDef.linearVelocity = b3Vec3{120.0f, 0.0f, 120.0f};
    b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
    b3Sphere sphere;
    sphere.center = b3Vec3{0.0f, 0.0f, 0.0f};
    sphere.radius = 0.5f;
    shapeDef.baseMaterial.friction = 0.0f;
    shapeDef.baseMaterial.restitution = 1.0f;
    ignore b3CreateSphereShape(bodyId, &shapeDef, &sphere);
}

// One wall: a box hull baked at (ox,oy,oz) in the body frame, drawn at
// the matching draw-registry offset.
void bounce_wall(b3BodyId body, b3ShapeDef* sd, f32 hx, f32 hy, f32 hz,
                 f32 ox, f32 oy, f32 oz) {
    b3Transform t;
    t.p = b3Vec3{ox, oy, oz};
    t.q = b3Quat_identity;
    b3BoxHull wall = b3MakeTransformedBoxHull(hx, hy, hz, t);
    ignore b3CreateHullShape(body, sd, &wall.base);
}

// sample_continuous.cpp IsFast: three tall boxes, no gravity, spinning
// about the x, y and z axes — a fast-rotation stress test.
void build_is_fast() {
    ignore add_ground_box(40.0f);
    b3BoxHull box = b3MakeBoxHull(0.5f, 10.0f, 0.5f);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.gravityScale = 0.0f;

    bodyDef.position = b3Pos{-12.0f, 20.0f, 0.0f};
    bodyDef.angularVelocity = b3Vec3{0.0f, 0.0f, 4.0f};
    b3BodyId b1 = b3CreateBody(g_world, &bodyDef);
    ignore b3CreateHullShape(b1, &shapeDef, &box.base);

    bodyDef.position = b3Pos{0.0f, 20.0f, 0.0f};
    bodyDef.angularVelocity = b3Vec3{0.0f, 4.0f, 0.0f};
    b3BodyId b2 = b3CreateBody(g_world, &bodyDef);
    ignore b3CreateHullShape(b2, &shapeDef, &box.base);

    bodyDef.position = b3Pos{12.0f, 20.0f, 0.0f};
    bodyDef.angularVelocity = b3Vec3{4.0f, 0.0f, 0.0f};
    b3BodyId b3 = b3CreateBody(g_world, &bodyDef);
    ignore b3CreateHullShape(b3, &shapeDef, &box.base);
}

// samples/sample_continuous.cpp SpinningStick
void build_spinning_stick() {
    ignore add_ground_box(10.0f);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.position = b3Pos{0.0f, 0.5f, 0.0f};
    b3BodyId wallBodyId = b3CreateBody(g_world, &bodyDef);
    b3BoxHull wallBox = b3MakeBoxHull(0.125f, 0.5f, 10.0f);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    ignore b3CreateHullShape(wallBodyId, &shapeDef, &wallBox.base);

    bodyDef.type = b3_dynamicBody;
    bodyDef.position = b3Pos{0.0f, 20.0f, 0.5f};
    bodyDef.linearVelocity = b3Vec3{0.0f, -100.0f, 0.0f};
    b3Vec3 range = b3Vec3{50.0f, 50.0f, 50.0f};
    bodyDef.angularVelocity = random_vec3(
        b3Vec3{0.0f - range.x, 0.0f - range.y, 0.0f - range.z}, range);
    b3BodyId stickBodyId = b3CreateBody(g_world, &bodyDef);
    b3BoxHull stickBox = b3MakeBoxHull(2.0f, 0.1f, 0.1f);
    shapeDef.baseMaterial.rollingResistance = 0.1f;
    ignore b3CreateHullShape(stickBodyId, &shapeDef, &stickBox.base);
}

// samples/sample_continuous.cpp NeedleMesh
b3MeshData* g_nm_needle1;
b3MeshData* g_nm_needle2;
b3MeshData* g_nm_needle3;
b3MeshData* g_nm_needle4;

const i32 NEEDLE_SLICES_MAX = 64;

b3MeshData* create_needle(f32 height, f32 radius, b3Vec3 center, i32 slices) {
    i32 vertexCount = slices + 1;
    b3Vec3[NEEDLE_SLICES_MAX + 1] vertices;

    f32 alpha = 0.0f;
    f32 deltaAlpha = 2.0f * PI_F / cast(f32, slices);

    vertices[0] = b3Add(b3Vec3{0.0f, height, 0.0f}, center);
    for i32 index = 1; index < vertexCount; index += 1 {
        b3CosSin cs = b3ComputeCosSin(alpha);
        vertices[index] = b3Add(b3Vec3{radius * cs.cosine, 0.0f, radius * cs.sine}, center);
        alpha += deltaAlpha;
    }

    i32 triangleCount = slices;
    i32[3 * NEEDLE_SLICES_MAX] indexBase;

    i32 index1 = vertexCount - 1;
    for i32 index = 0; index < triangleCount; index += 1 {
        i32 index2 = index + 1;
        indexBase[3 * index + 0] = 0;
        indexBase[3 * index + 1] = index2;
        indexBase[3 * index + 2] = index1;
        index1 = index2;
    }

    b3MeshDef def = b3MeshDef{};
    def.vertexCount = vertexCount;
    def.vertices = cast(b3Vec3*, &vertices);
    def.triangleCount = triangleCount;
    def.indices = cast(i32*, &indexBase);
    def.useMedianSplit = true;

    return b3CreateMesh(&def, null, 0);
}

void build_needle_mesh() {
    i32 slices = 8;
    g_nm_needle1 = create_needle(0.99f, 0.1f, b3Vec3{0.2f, 0.0f, 0.2f}, slices);
    g_nm_needle2 = create_needle(1.01f, 0.1f, b3Vec3{0.2f, 0.0f, -0.2f}, slices);
    g_nm_needle3 = create_needle(0.98f, 0.1f, b3Vec3{-0.2f, 0.0f, -0.2f}, slices);
    g_nm_needle4 = create_needle(1.02f, 0.1f, b3Vec3{-0.2f, 0.0f, 0.2f}, slices);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    b3BodyId groundBodyId = b3CreateBody(g_world, &bodyDef);

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    ignore b3CreateMeshShape(groundBodyId, &shapeDef, g_nm_needle1, b3Vec3_one);
    ignore b3CreateMeshShape(groundBodyId, &shapeDef, g_nm_needle2, b3Vec3_one);
    ignore b3CreateMeshShape(groundBodyId, &shapeDef, g_nm_needle3, b3Vec3_one);
    ignore b3CreateMeshShape(groundBodyId, &shapeDef, g_nm_needle4, b3Vec3_one);

    bodyDef.type = b3_dynamicBody;
    bodyDef.position = b3Pos{0.0f, 5.0f, 0.0f};
    bodyDef.linearVelocity = b3Vec3{0.0f, -10.0f, 0.0f};
    b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);

    b3BoxHull box = b3MakeBoxHull(0.3f, 0.01f, 0.3f);
    ignore b3CreateHullShape(bodyId, &shapeDef, &box.base);
}

void destroy_needle_mesh() {
    b3DestroyMesh(g_nm_needle4);
    b3DestroyMesh(g_nm_needle3);
    b3DestroyMesh(g_nm_needle2);
    b3DestroyMesh(g_nm_needle1);
}

void step_needle_mesh(f32 timeStep) {
    ignore timeStep;
    dbg_ground_grid(10);
}

// samples/sample_continuous.cpp HumpMesh
b3MeshData* g_hm_hump;

b3MeshData* create_hump(f32 cellWidth) {
    b3Vec3[6] vertices;

    i32 index = 0;
    f32 x = -0.5f * cellWidth;
    for i32 ix = 0; ix <= 1; ix += 1 {
        f32 z = 0.0f - cellWidth;
        for i32 iz = 0; iz <= 2; iz += 1 {
            vertices[index] = b3Vec3{x, 0.0f, z};
            if iz == 1 {
                vertices[index].y = 0.05f * cellWidth;
            }
            z += cellWidth;
            index += 1;
        }
        x += cellWidth;
    }

    i32 triangleCount = 4;
    i32[12] indices;

    index = 0;
    for i32 ix = 0; ix < 1; ix += 1 {
        for i32 iz = 0; iz < 2; iz += 1 {
            i32 index1 = iz + 3 * ix;
            i32 index2 = index1 + 1;
            i32 index3 = index2 + 3;
            i32 index4 = index3 - 1;

            indices[index + 0] = index1;
            indices[index + 1] = index2;
            indices[index + 2] = index3;

            indices[index + 3] = index3;
            indices[index + 4] = index4;
            indices[index + 5] = index1;

            index += 6;
        }
    }

    b3MeshDef def = b3MeshDef{};
    def.vertexCount = 6;
    def.vertices = cast(b3Vec3*, &vertices);
    def.triangleCount = triangleCount;
    def.indices = cast(i32*, &indices);
    def.useMedianSplit = true;
    def.identifyEdges = true;

    return b3CreateMesh(&def, null, 0);
}

void build_hump_mesh() {
    ignore add_ground_box(20.0f);

    g_hm_hump = create_hump(8.0f);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    b3BodyId groundBodyId = b3CreateBody(g_world, &bodyDef);

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    ignore b3CreateMeshShape(groundBodyId, &shapeDef, g_hm_hump, b3Vec3_one);

    bodyDef.type = b3_dynamicBody;
    bodyDef.position = b3Pos{0.0f, 5.0f, 0.0f};
    bodyDef.linearVelocity = b3Vec3{0.0f, -50.0f, 0.0f};
    b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);

    b3BoxHull box = b3MakeBoxHull(0.5f, 0.05f, 1.0f);
    ignore b3CreateHullShape(bodyId, &shapeDef, &box.base);
}

void destroy_hump_mesh() {
    b3DestroyMesh(g_hm_hump);
}

// samples/sample_continuous.cpp Stall
b3MeshData* g_stall_mesh;
b3BodyId g_stall_bullet;
bool g_stall_bullet_live;
f32 g_stall_saved_threshold;

void stall_launch() {
    if g_stall_bullet_live {
        b3DestroyBody(g_stall_bullet);
        g_stall_bullet_live = false;
    }

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.isBullet = true;
    bodyDef.name = "rock";
    bodyDef.position = b3Pos{0.0f, 1.0f, -10.0f};
    bodyDef.linearVelocity = b3Vec3{0.0f, 0.0f, 600.0f};
    bodyDef.angularVelocity = b3Vec3{0.0f, 0.0f, 20.0f};
    g_stall_bullet = b3CreateBody(g_world, &bodyDef);
    g_stall_bullet_live = true;

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    b3HullData* rock = b3CreateRock(0.25f);
    ignore b3CreateHullShape(g_stall_bullet, &shapeDef, rock);
    b3DestroyHull(rock);
}

void build_stall() {
    ignore add_ground_box(500.0f);

    {
        g_stall_mesh = b3CreateTorusMesh(200, 200, 2.0f, 1.0f);
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.name = "torus";
        bodyDef.position.y = 2.0f;
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        ignore b3CreateMeshShape(bodyId, &shapeDef, g_stall_mesh, b3Vec3_one);
    }

    g_stall_saved_threshold = b3GetStallThreshold();
    b3SetStallThreshold(0.001f);

    g_stall_bullet_live = false;
    stall_launch();
}

void destroy_stall() {
    b3DestroyMesh(g_stall_mesh);
    b3SetStallThreshold(g_stall_saved_threshold);
}

bool stall_controls() {
    if ImGui_Button("Launch", ImVec2{0.0f, 0.0f}) {
        stall_launch();
    }
    return true;
}

// samples/sample_continuous.cpp MeshDrop
const i32 CMD_GRID_COUNT = 32;
const i32 CMD_BODY_COUNT = CMD_GRID_COUNT * CMD_GRID_COUNT;

const i32 CMD_SHAPE_BOX = 0;
const i32 CMD_SHAPE_CAPSULE = 1;
const i32 CMD_SHAPE_CYLINDER = 2;
const i32 CMD_SHAPE_SPHERE = 3;

b3BodyId g_cmd_ground;
bool g_cmd_ground_live;
b3MeshData* g_cmd_ground_mesh;
f32 g_cmd_ground_amplitude;
b3BodyId[CMD_BODY_COUNT] g_cmd_bodies;
bool[CMD_BODY_COUNT] g_cmd_body_live;
b3HullData* g_cmd_cylinder;
i32 g_cmd_shape_type;
i32 g_cmd_run_count;
bool g_cmd_failure;
bool g_cmd_auto_generate;
bool g_cmd_collide;
i32 g_cmd_step_count;

// upstream ConvertToUserData / ConvertToPair: two ints packed into the
// body's user data pointer.
void* cmd_to_user_data(i32 index1, i32 index2) {
    return cast(void*, (cast(i64, index2) << 32) | (cast(i64, index1) & cast(i64, 0xFFFFFFFF)));
}

i32 cmd_pair_index1(void* userData) { return cast(i32, cast(i64, userData)); }
i32 cmd_pair_index2(void* userData) { return cast(i32, cast(i64, userData) >> 32); }

void cmd_create_ground() {
    if g_cmd_ground_live {
        b3DestroyBody(g_cmd_ground);
        g_cmd_ground_live = false;
    }

    if g_cmd_ground_mesh != null {
        b3DestroyMesh(g_cmd_ground_mesh);
        g_cmd_ground_mesh = null;
    }

    b3BodyDef bodyDef = b3DefaultBodyDef();
    g_cmd_ground = b3CreateBody(g_world, &bodyDef);
    g_cmd_ground_live = true;

    i32 gridCount = 40;
    f32 cellWidth = 1.0f;
    f32 rowHz = 0.1f;
    f32 columnHz = 0.2f;

    g_cmd_ground_mesh = b3CreateWaveMesh(gridCount, gridCount, cellWidth,
                                         g_cmd_ground_amplitude, rowHz, columnHz);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.filter.categoryBits = 1;
    ignore b3CreateMeshShape(g_cmd_ground, &shapeDef, g_cmd_ground_mesh, b3Vec3_one);

    f32 extent = 0.5f * cast(f32, gridCount) * cellWidth;
    f32 halfHeight = 1.0f;

    {
        b3Transform transform;
        transform.p = b3Vec3{0.0f, halfHeight, -extent};
        transform.q = b3Quat_identity;
        b3BoxHull wallBox = b3MakeTransformedBoxHull(extent, halfHeight, 0.1f, transform);
        ignore b3CreateHullShape(g_cmd_ground, &shapeDef, &wallBox.base);
    }
    {
        b3Transform transform;
        transform.p = b3Vec3{0.0f, halfHeight, extent};
        transform.q = b3Quat_identity;
        b3BoxHull wallBox = b3MakeTransformedBoxHull(extent, halfHeight, 0.1f, transform);
        ignore b3CreateHullShape(g_cmd_ground, &shapeDef, &wallBox.base);
    }
    {
        b3Transform transform;
        transform.p = b3Vec3{-extent, halfHeight, 0.0f};
        transform.q = b3Quat_identity;
        b3BoxHull wallBox = b3MakeTransformedBoxHull(0.1f, halfHeight, extent, transform);
        ignore b3CreateHullShape(g_cmd_ground, &shapeDef, &wallBox.base);
    }
    {
        b3Transform transform;
        transform.p = b3Vec3{extent, halfHeight, 0.0f};
        transform.q = b3Quat_identity;
        b3BoxHull wallBox = b3MakeTransformedBoxHull(0.1f, halfHeight, extent, transform);
        ignore b3CreateHullShape(g_cmd_ground, &shapeDef, &wallBox.base);
    }
}

void cmd_generate() {
    for i32 i = 0; i < CMD_BODY_COUNT; i += 1 {
        if !g_cmd_body_live[i] { continue; }
        b3DestroyBody(g_cmd_bodies[i]);
        g_cmd_body_live[i] = false;
    }

    b3BoxHull box = b3MakeBoxHull(0.02f, 0.2f, 0.04f);
    b3Capsule capsule = b3Capsule{b3Pos{0.0f, -0.2f, 0.0f}, b3Pos{0.0f, 0.2f, 0.0f}, 0.05f};
    b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, 0.05f};

    i32 bodyIndex = 0;

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.baseMaterial.rollingResistance = g_cmd_shape_type == CMD_SHAPE_CAPSULE ? 0.4f : 0.1f;

    if g_cmd_collide == false {
        shapeDef.filter.categoryBits = 2;
        shapeDef.filter.maskBits = 1;
    }

    g_randomSeed = cast(u32, b3GetTicks());

    g_cmd_run_count += 1;
    g_cmd_step_count = 0;

    for i32 i = 0; i < CMD_GRID_COUNT; i += 1 {
        for i32 j = 0; j < CMD_GRID_COUNT; j += 1 {
            b3Vec3 linearVelocity = random_vec3_uniform(-1.0f, 1.0f);
            b3Vec3 angularVelocity = random_vec3_uniform(-5.0f, 5.0f);

            bodyDef.position = b3Pos{0.5f * (cast(f32, i) - 0.5f * cast(f32, CMD_GRID_COUNT)),
                                     5.0f,
                                     0.5f * (cast(f32, j) - 0.5f * cast(f32, CMD_GRID_COUNT))};
            bodyDef.linearVelocity = linearVelocity;
            bodyDef.angularVelocity = angularVelocity;
            bodyDef.userData = cmd_to_user_data(i, j);
            b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);

            if g_cmd_shape_type == CMD_SHAPE_BOX {
                ignore b3CreateHullShape(bodyId, &shapeDef, &box.base);
            } else if g_cmd_shape_type == CMD_SHAPE_CAPSULE {
                ignore b3CreateCapsuleShape(bodyId, &shapeDef, &capsule);
            } else if g_cmd_shape_type == CMD_SHAPE_CYLINDER {
                ignore b3CreateHullShape(bodyId, &shapeDef, g_cmd_cylinder);
            } else {
                ignore b3CreateSphereShape(bodyId, &shapeDef, &sphere);
            }

            g_cmd_bodies[bodyIndex] = bodyId;
            g_cmd_body_live[bodyIndex] = true;
            bodyIndex += 1;
        }
    }
}

void build_continuous_mesh_drop() {
    g_cmd_ground_live = false;
    g_cmd_ground_mesh = null;
    g_cmd_ground_amplitude = 0.5f;
    cmd_create_ground();

    for i32 i = 0; i < CMD_BODY_COUNT; i += 1 { g_cmd_body_live[i] = false; }

    g_cmd_cylinder = b3CreateCylinder(0.4f, 0.05f, 0.0f, 6);
    g_cmd_shape_type = CMD_SHAPE_BOX;
    g_cmd_run_count = 0;
    g_cmd_failure = false;
    g_cmd_auto_generate = false;
    g_cmd_collide = true;

    g_dbg_force_scale = 0.1f;

    cmd_generate();
}

void destroy_continuous_mesh_drop() {
    b3DestroyHull(g_cmd_cylinder);
    b3DestroyMesh(g_cmd_ground_mesh);
}

bool continuous_mesh_drop_controls() {
    u8*[4] shapeTypes;
    shapeTypes[0] = "box";
    shapeTypes[1] = "capsule";
    shapeTypes[2] = "cylinder";
    shapeTypes[3] = "sphere";
    i32 shapeType = g_cmd_shape_type;
    if ImGui_Combo("Type", &shapeType, cast(u8**, &shapeTypes), 4, -1) {
        g_cmd_step_count = 0;
        g_cmd_shape_type = shapeType;
        cmd_generate();
    }

    if ImGui_SliderFloat("Amplitude", &g_cmd_ground_amplitude, 0.0f, 1.0f, "%.3f", 0) {
        cmd_create_ground();
        cmd_generate();
    }

    if ImGui_Checkbox("Collide", &g_cmd_collide) {
        cmd_generate();
    }

    if ImGui_Button("Generate", ImVec2{0.0f, 0.0f}) {
        cmd_generate();
    }

    if ImGui_Button("Auto Generate", ImVec2{0.0f, 0.0f}) {
        g_cmd_auto_generate = !g_cmd_auto_generate;
        g_cmd_step_count = 0;
    }

    return true;
}

void step_continuous_mesh_drop(f32 timeStep) {
    {
        PickRay pickRay = build_pick_ray(g_mouse_screen_x, g_mouse_screen_y);
        b3Pos origin = b3Pos{pickRay.origin.x, pickRay.origin.y, pickRay.origin.z};
        b3Vec3 translation = b3Vec3{pickRay.translation.x, pickRay.translation.y,
                                    pickRay.translation.z};
        b3RayResult result = b3World_CastRayClosest(g_world, origin, translation,
                                                    b3DefaultQueryFilter());
        if result.hit {
            b3BodyId bodyId = b3Shape_GetBody(result.shapeId);
            void* userData = b3Body_GetUserData(bodyId);
            u8[96] buf;
            ignore snprintf(cast(u8*, &buf), 96, "indices: (%d, %d)",
                            cmd_pair_index1(userData), cmd_pair_index2(userData));
            draw_text_line(cast(u8*, &buf));
        }
    }

    for i32 i = 0; i < CMD_BODY_COUNT && g_cmd_failure == false; i += 1 {
        if !g_cmd_body_live[i] { continue; }
        b3Pos massCenter = b3Body_GetWorldCenter(g_cmd_bodies[i]);
        if massCenter.y < -2.0f {
            g_pause = true;
            g_cmd_failure = true;
            g_cmd_auto_generate = false;
        }
    }

    if g_cmd_auto_generate {
        b3BodyEvents bodyEvents = b3World_GetBodyEvents(g_world);
        if bodyEvents.moveCount == 0 {
            cmd_generate();
        }

        f32 step = 0.0f;
        if g_pause == false || g_single_step > 0 {
            step = g_hertz > 0.0f ? 1.0f / g_hertz : 0.0f;
            g_single_step = b3MaxInt(0, g_single_step - 1);
        }

        if step > 0.0f {
            i32 n = 20;
            for i32 i = 0; i < n; i += 1 {
                b3World_Step(g_world, step, g_substeps);
            }
            g_cmd_step_count += n;
        }

        i32 maxSteps = g_cmd_shape_type == CMD_SHAPE_CAPSULE ? 3000 : 1000;
        if g_cmd_step_count > maxSteps {
            g_pause = true;
            g_cmd_failure = true;
            g_cmd_auto_generate = false;
        }
    }
    ignore timeStep;
}
