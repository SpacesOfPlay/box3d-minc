// Mesh scenes. Ports of samples/sample_mesh.cpp.

import box3d;
import box3d_human;
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
import sample_issues;
import sample_events;
import sample_geometry;
import sample_collision;

// samples/sample_mesh.cpp ShapeType
const i32 MESH_SHAPE_SPHERE = 0;
const i32 MESH_SHAPE_CAPSULE = 1;
const i32 MESH_SHAPE_BOX = 2;
const i32 MESH_SHAPE_CYLINDER = 3;

// samples/sample_mesh.cpp GridMesh
b3HullData* g_gm_cylinder_hull;
i32 g_gm_shape_type;
b3BodyId g_gm_body;
bool g_gm_body_live;
b3MeshData* g_gm_grid_mesh;
b3ShapeId g_gm_grid_shape;
b3Vec3 g_gm_scale;

void grid_mesh_spawn() {
    if g_gm_body_live {
        b3DestroyBody(g_gm_body);
        g_gm_body_live = false;
    }

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = b3Pos{0.1f, 1.0f, -0.1f};
    bodyDef.angularDamping = g_gm_shape_type == MESH_SHAPE_CYLINDER ? 0.1f : 0.0f;
    g_gm_body = b3CreateBody(g_world, &bodyDef);
    g_gm_body_live = true;

    b3ShapeDef shapeDef = b3DefaultShapeDef();

    if g_gm_shape_type == MESH_SHAPE_SPHERE {
        shapeDef.baseMaterial.rollingResistance = 0.05f;
        b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, 0.5f};
        ignore b3CreateSphereShape(g_gm_body, &shapeDef, &sphere);
    } else if g_gm_shape_type == MESH_SHAPE_CAPSULE {
        shapeDef.baseMaterial.rollingResistance = 0.05f;
        b3Capsule capsule = b3Capsule{b3Pos{0.0f, 0.0f, 1.276f}, b3Pos{0.0f, 0.0f, 0.476f}, 0.15f};
        ignore b3CreateCapsuleShape(g_gm_body, &shapeDef, &capsule);
    } else if g_gm_shape_type == MESH_SHAPE_BOX {
        b3BoxHull box = b3MakeBoxHull(0.5f, 0.5f, 0.5f);
        ignore b3CreateHullShape(g_gm_body, &shapeDef, &box.base);
    } else {
        shapeDef.baseMaterial.rollingResistance = 0.02f;
        ignore b3CreateHullShape(g_gm_body, &shapeDef, g_gm_cylinder_hull);
    }
}

void build_grid_mesh() {
    b3BodyDef bodyDef = b3DefaultBodyDef();
    b3BodyId groundId = b3CreateBody(g_world, &bodyDef);

    g_gm_grid_mesh = b3CreateGridMesh(20, 20, 1.0f, 0, true);

    g_gm_scale = b3Vec3_one;
    g_gm_scale = b3Vec3{2.0f, 2.0f, 2.0f};

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    g_gm_grid_shape = b3CreateMeshShape(groundId, &shapeDef, g_gm_grid_mesh, g_gm_scale);

    g_gm_shape_type = MESH_SHAPE_CYLINDER;
    g_gm_body_live = false;
    g_gm_cylinder_hull = b3CreateCylinder(1.0f, 0.25f, 0.0f, 15);

    grid_mesh_spawn();

    g_dbg_force_scale = 0.01f;
}

void destroy_grid_mesh() {
    b3DestroyHull(g_gm_cylinder_hull);
    b3DestroyMesh(g_gm_grid_mesh);
}

bool grid_mesh_controls() {
    if ImGui_RadioButton("Sphere", g_gm_shape_type == MESH_SHAPE_SPHERE) {
        g_gm_shape_type = MESH_SHAPE_SPHERE;
        grid_mesh_spawn();
    }
    if ImGui_RadioButton("Capsule", g_gm_shape_type == MESH_SHAPE_CAPSULE) {
        g_gm_shape_type = MESH_SHAPE_CAPSULE;
        grid_mesh_spawn();
    }
    if ImGui_RadioButton("Box", g_gm_shape_type == MESH_SHAPE_BOX) {
        g_gm_shape_type = MESH_SHAPE_BOX;
        grid_mesh_spawn();
    }
    if ImGui_RadioButton("Cylinder", g_gm_shape_type == MESH_SHAPE_CYLINDER) {
        g_gm_shape_type = MESH_SHAPE_CYLINDER;
        grid_mesh_spawn();
    }

    b3Vec3 scale = g_gm_scale;
    bool changed = false;
    changed = ImGui_SliderFloat("Scale X", &scale.x, -2.0f, 2.0f, "%.1f", 0) || changed;
    changed = ImGui_SliderFloat("Scale Z", &scale.z, -2.0f, 2.0f, "%.1f", 0) || changed;

    if changed {
        g_gm_scale = scale;
        b3Shape_SetMesh(g_gm_grid_shape, g_gm_grid_mesh, g_gm_scale);
    }

    return true;
}

void step_grid_mesh(f32 timeStep) {
    ignore timeStep;
    u8[128] buf;
    ignore snprintf(cast(u8*, &buf), 128, "triangle count = %d, bytes = %d",
                    g_gm_grid_mesh.triangleCount, g_gm_grid_mesh.byteCount);
    draw_text_line(cast(u8*, &buf));
    b3Transform transform = b3Transform{b3Vec3{0.0f, 0.01f, 0.0f}, b3Quat_identity};
    dbg_axes(b3MakeWorldTransform(transform), 1.0f);
}

// samples/sample_mesh.cpp BoxMesh
b3MeshData* g_bm_box_mesh;
b3ShapeId g_bm_box_shape;
b3HullData* g_bm_cylinder_hull;
i32 g_bm_shape_type;
b3BodyId g_bm_body;
bool g_bm_body_live;
b3Vec3 g_bm_scale;

void box_mesh_spawn() {
    if g_bm_body_live {
        b3DestroyBody(g_bm_body);
        g_bm_body_live = false;
    }

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = b3Pos{0.0f, 1.5f, 0.0f};

    if g_bm_shape_type == MESH_SHAPE_CYLINDER {
        bodyDef.position.y -= 0.5f;
    }

    g_bm_body = b3CreateBody(g_world, &bodyDef);
    g_bm_body_live = true;

    b3ShapeDef shapeDef = b3DefaultShapeDef();

    if g_bm_shape_type == MESH_SHAPE_SPHERE {
        b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, 0.5f};
        ignore b3CreateSphereShape(g_bm_body, &shapeDef, &sphere);
    } else if g_bm_shape_type == MESH_SHAPE_CAPSULE {
        b3Capsule capsule = b3Capsule{b3Pos{-0.5f, 0.0f, 0.0f}, b3Pos{0.5f, 0.0f, 0.0f}, 0.1f};
        ignore b3CreateCapsuleShape(g_bm_body, &shapeDef, &capsule);
    } else if g_bm_shape_type == MESH_SHAPE_BOX {
        b3BoxHull box = b3MakeBoxHull(0.5f, 0.5f, 0.5f);
        ignore b3CreateHullShape(g_bm_body, &shapeDef, &box.base);
    } else {
        ignore b3CreateHullShape(g_bm_body, &shapeDef, g_bm_cylinder_hull);
    }
}

void build_box_mesh() {
    ignore add_ground_box(20.0f);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.position = b3Pos{0.0f, -1.0f, 0.0f};
    bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisY, 0.25f * PI_F);
    b3BodyId groundId = b3CreateBody(g_world, &bodyDef);

    b3ShapeDef shapeDef = b3DefaultShapeDef();

    g_bm_box_mesh = b3CreateBoxMesh(b3Vec3{0.0f, 1.0f, 0.0f}, b3Vec3{1.0f, 1.0f, 1.0f}, true);
    g_bm_scale = b3Vec3_one;
    g_bm_box_shape = b3CreateMeshShape(groundId, &shapeDef, g_bm_box_mesh, g_bm_scale);

    g_bm_cylinder_hull = b3CreateCylinder(1.0f, 0.75f, 0.0f, 8);

    g_bm_shape_type = MESH_SHAPE_BOX;
    g_bm_body_live = false;

    box_mesh_spawn();
}

void destroy_box_mesh() {
    b3DestroyMesh(g_bm_box_mesh);
    b3DestroyHull(g_bm_cylinder_hull);
}

bool box_mesh_controls() {
    if ImGui_RadioButton("Sphere", g_bm_shape_type == MESH_SHAPE_SPHERE) {
        g_bm_shape_type = MESH_SHAPE_SPHERE;
        box_mesh_spawn();
    }
    if ImGui_RadioButton("Capsule", g_bm_shape_type == MESH_SHAPE_CAPSULE) {
        g_bm_shape_type = MESH_SHAPE_CAPSULE;
        box_mesh_spawn();
    }
    if ImGui_RadioButton("Box", g_bm_shape_type == MESH_SHAPE_BOX) {
        g_bm_shape_type = MESH_SHAPE_BOX;
        box_mesh_spawn();
    }
    if ImGui_RadioButton("Cylinder", g_bm_shape_type == MESH_SHAPE_CYLINDER) {
        g_bm_shape_type = MESH_SHAPE_CYLINDER;
        box_mesh_spawn();
    }

    b3Vec3 scale = g_bm_scale;
    bool changed = false;
    changed = ImGui_SliderFloat("Scale X", &scale.x, -2.0f, 2.0f, "%.1f", 0) || changed;
    changed = ImGui_SliderFloat("Scale Z", &scale.z, -2.0f, 2.0f, "%.1f", 0) || changed;

    if changed {
        g_bm_scale = scale;
        b3Shape_SetMesh(g_bm_box_shape, g_bm_box_mesh, g_bm_scale);
    }

    return true;
}

// samples/sample_mesh.cpp BigBoxMesh
b3MeshData* g_bb_box_mesh;
b3ShapeId g_bb_grid_shape;
b3HullData* g_bb_cylinder_hull;
i32 g_bb_shape_type;
b3BodyId g_bb_body;
bool g_bb_body_live;
b3Vec3 g_bb_scale;

void big_box_mesh_spawn() {
    if g_bb_body_live {
        b3DestroyBody(g_bb_body);
        g_bb_body_live = false;
    }

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    // Deviation from upstream, which spawns at y = 0. The ground box
    // mesh's top face is also y = 0, so every shape starts embedded by
    // its own half height, and the sphere's 0.5 is deep enough that the
    // overlap never resolves — it sinks (measured: y = -125 after 240
    // steps, probe_bigbox_sphere.mc). 0.6 clears the largest shape.
    // The box, equally embedded, recovers; and at 0.6 the box, capsule
    // and cylinder settle at exactly the heights they did at 0.
    bodyDef.position = b3Pos{0.5f, 0.6f, 0.0f};
    g_bb_body = b3CreateBody(g_world, &bodyDef);
    g_bb_body_live = true;

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.baseMaterial.rollingResistance = 0.05f;

    if g_bb_shape_type == MESH_SHAPE_SPHERE {
        b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, 0.5f};
        ignore b3CreateSphereShape(g_bb_body, &shapeDef, &sphere);
    } else if g_bb_shape_type == MESH_SHAPE_CAPSULE {
        b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
        material.rollingResistance = 0.1f;
        shapeDef.materials = &material;
        shapeDef.materialCount = 1;
        b3Capsule capsule = b3Capsule{b3Pos{0.0f, 0.0f, 1.276f}, b3Pos{0.0f, 0.0f, 0.476f}, 0.15f};
        ignore b3CreateCapsuleShape(g_bb_body, &shapeDef, &capsule);
    } else if g_bb_shape_type == MESH_SHAPE_BOX {
        b3BoxHull box = b3MakeBoxHull(0.5f, 0.5f, 0.5f);
        ignore b3CreateHullShape(g_bb_body, &shapeDef, &box.base);
    } else {
        ignore b3CreateHullShape(g_bb_body, &shapeDef, g_bb_cylinder_hull);
    }
}

void build_big_box_mesh() {
    b3BodyDef bodyDef = b3DefaultBodyDef();
    b3BodyId groundId = b3CreateBody(g_world, &bodyDef);
    g_bb_box_mesh = b3CreateBoxMesh(b3Vec3{0.0f, -1.0f, 0.0f}, b3Vec3{50.0f, 1.0f, 50.0f}, true);
    g_bb_scale = b3Vec3_one;

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.baseMaterial.friction = 0.5f;
    g_bb_grid_shape = b3CreateMeshShape(groundId, &shapeDef, g_bb_box_mesh, g_bb_scale);

    g_bb_shape_type = MESH_SHAPE_CYLINDER;
    g_bb_body_live = false;
    g_bb_cylinder_hull = b3CreateCylinder(0.3f, 0.15f, 0.0f, 32);

    big_box_mesh_spawn();
}

void destroy_big_box_mesh() {
    b3DestroyHull(g_bb_cylinder_hull);
    b3DestroyMesh(g_bb_box_mesh);
}

bool big_box_mesh_controls() {
    if ImGui_RadioButton("Sphere", g_bb_shape_type == MESH_SHAPE_SPHERE) {
        g_bb_shape_type = MESH_SHAPE_SPHERE;
        big_box_mesh_spawn();
    }
    if ImGui_RadioButton("Capsule", g_bb_shape_type == MESH_SHAPE_CAPSULE) {
        g_bb_shape_type = MESH_SHAPE_CAPSULE;
        big_box_mesh_spawn();
    }
    if ImGui_RadioButton("Box", g_bb_shape_type == MESH_SHAPE_BOX) {
        g_bb_shape_type = MESH_SHAPE_BOX;
        big_box_mesh_spawn();
    }
    if ImGui_RadioButton("Cylinder", g_bb_shape_type == MESH_SHAPE_CYLINDER) {
        g_bb_shape_type = MESH_SHAPE_CYLINDER;
        big_box_mesh_spawn();
    }

    b3Vec3 scale = g_bb_scale;
    bool changed = false;
    changed = ImGui_SliderFloat("Scale X", &scale.x, -2.0f, 2.0f, "%.1f", 0) || changed;
    changed = ImGui_SliderFloat("Scale Z", &scale.z, -2.0f, 2.0f, "%.1f", 0) || changed;

    if changed {
        g_bb_scale = scale;
        b3Shape_SetMesh(g_bb_grid_shape, g_bb_box_mesh, g_bb_scale);
    }

    return true;
}

void step_big_box_mesh(f32 timeStep) {
    ignore timeStep;
    b3Transform transform = b3Transform{b3Vec3{0.0f, 0.01f, 0.0f}, b3Quat_identity};
    dbg_axes(b3MakeWorldTransform(transform), 1.0f);
}

// samples/sample_mesh.cpp HollowBox
b3MeshData* g_hb_mesh;

void build_hollow_box() {
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        b3BodyId groundId = b3CreateBody(g_world, &bodyDef);

        g_hb_mesh = b3CreateHollowBoxMesh(b3Vec3{0.0f, 0.0f, 0.0f}, b3Vec3{10.0f, 10.0f, 10.0f});
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        ignore b3CreateMeshShape(groundId, &shapeDef, g_hb_mesh, b3Vec3_one);
    }

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.gravityScale = 0.0f;
    bodyDef.enableSleep = false;
    b3ShapeDef shapeDef = b3DefaultShapeDef();

    {
        b3HullData* cylinderHull = b3CreateCylinder(1.0f, 0.25f, 0.0f, 8);

        b3Pos[6] positions;
        positions[0] = b3Pos{0.0f, -10.2f, 0.0f};
        positions[1] = b3Pos{0.0f, 9.2f, 0.0f};
        positions[2] = b3Pos{-9.8f, 0.0f, 0.0f};
        positions[3] = b3Pos{9.8f, 0.0f, 0.0f};
        positions[4] = b3Pos{0.0f, 0.0f, -9.8f};
        positions[5] = b3Pos{0.0f, 0.0f, 9.8f};

        for i32 i = 0; i < 6; i += 1 {
            bodyDef.position = positions[i];
            b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
            ignore b3CreateHullShape(bodyId, &shapeDef, cylinderHull);
        }

        b3DestroyHull(cylinderHull);
    }

    {
        b3Capsule capsule = b3Capsule{b3Pos{0.0f, 0.0f, 0.0f}, b3Pos{0.0f, 1.0f, 0.0f}, 0.25f};
        b3Pos[8] positions;
        positions[0] = b3Pos{0.0f, -10.2f, 2.0f};
        positions[1] = b3Pos{0.0f, 9.2f, 2.0f};
        positions[2] = b3Pos{0.0f, -9.9f, 4.0f};
        positions[3] = b3Pos{0.0f, 8.9f, 4.0f};
        positions[4] = b3Pos{-9.8f, 2.0f, 0.0f};
        positions[5] = b3Pos{9.8f, 2.0f, 0.0f};
        positions[6] = b3Pos{0.0f, 2.0f, -9.8f};
        positions[7] = b3Pos{0.0f, 2.0f, 9.8f};

        for i32 i = 0; i < 8; i += 1 {
            bodyDef.position = positions[i];
            b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
            ignore b3CreateCapsuleShape(bodyId, &shapeDef, &capsule);
        }
    }
}

void destroy_hollow_box() {
    b3DestroyMesh(g_hb_mesh);
}

void step_hollow_box(f32 timeStep) {
    ignore timeStep;
    b3Transform transform = b3Transform{b3Vec3{0.0f, 0.01f, 0.0f}, b3Quat_identity};
    dbg_axes(b3MakeWorldTransform(transform), 1.0f);
}

// samples/sample_mesh.cpp ComputeInternalSurfaceArea
f32 compute_internal_surface_area(b3MeshData* data) {
    b3MeshNode* nodes = b3GetMeshNodes(data);
    i32 nodeCount = data.nodeCount;
    f32 area = 0.0f;
    for i32 i = 1; i < nodeCount; i += 1 {
        b3MeshNode* node = nodes + i;
        if node.data.asLeaf.type == cast(u32, 3) {
            continue;
        }

        b3AABB box = b3AABB{node.lowerBound, node.upperBound};

        area += b3AABB_Area(box);
    }

    return area;
}

// samples/sample_mesh.cpp MeshCreationBenchmark
const i32 MCB_MESH_COUNT = 4;
TempMesh[MCB_MESH_COUNT] g_mcb_temp_meshes;
f32 g_mcb_time;

void build_mesh_creation_benchmark() {
    load_temp_mesh("data/meshes/voxel_mesh_01.obj", &g_mcb_temp_meshes[0], 0.01f, true);
    load_temp_mesh("data/meshes/voxel_mesh_02.obj", &g_mcb_temp_meshes[1], 0.01f, true);
    load_temp_mesh("data/meshes/voxel_mesh_03.obj", &g_mcb_temp_meshes[2], 0.01f, true);
    load_temp_mesh("data/meshes/voxel_mesh_04.obj", &g_mcb_temp_meshes[3], 0.01f, true);

    g_mcb_time = FLT_MAX;
}

void destroy_mesh_creation_benchmark() {
    for i32 i = 0; i < MCB_MESH_COUNT; i += 1 {
        destroy_temp_mesh(&g_mcb_temp_meshes[i]);
    }
}

void step_mesh_creation_benchmark(f32 timeStep) {
    ignore timeStep;

    for i32 j = 0; j < MCB_MESH_COUNT; j += 1 {
        if g_mcb_temp_meshes[j].vertexCount == 0 {
            draw_text_line("data/meshes/voxel_mesh_0*.obj could not be read");
            return;
        }
    }

    bool computeArea = false;
    i32 iterations = SAMPLE_IS_DEBUG ? 1 : 10;
    i32 meshCount = MCB_MESH_COUNT;
    i32 triangleCount = 0;
    f32 area = 0.0f;

    for i32 i = 0; i < iterations; i += 1 {
        u64 startTicks = b3GetTicks();

        for i32 j = 0; j < meshCount; j += 1 {
            TempMesh* mesh = &g_mcb_temp_meshes[j];
            b3MeshDef def = b3MeshDef{};
            def.vertices = mesh.vertices;
            def.vertexCount = mesh.vertexCount;
            def.indices = mesh.indices;
            def.triangleCount = mesh.triangleCount;
            def.materialIndices = mesh.materialIndices;
            def.useMedianSplit = true;
            def.identifyEdges = false;
            def.weldVertices = true;
            def.weldTolerance = 0.0015f;

            b3MeshData* meshData = b3CreateMesh(&def, null, 0);
            triangleCount += i == 0 ? meshData.triangleCount : 0;

            if computeArea && i == 0 {
                area += compute_internal_surface_area(meshData);
            }

            b3DestroyMesh(meshData);
        }

        f32 ms = b3GetMilliseconds(startTicks);
        g_mcb_time = b3MinFloat(g_mcb_time, ms);
    }

    u8[128] buf;
    ignore snprintf(cast(u8*, &buf), 128, "triangle count = %d, area = %g", triangleCount,
                    cast(f64, area));
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 128, "total time = %.4f ms", cast(f64, g_mcb_time));
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 128, "time per mesh = %.4f ms",
                    cast(f64, g_mcb_time / cast(f32, meshCount)));
    draw_text_line(cast(u8*, &buf));
}

// samples/sample_mesh.cpp VoxelMesh
b3MeshData* g_vm_mesh_data;

void build_voxel_mesh() {
    b3Pos origin = b3Pos{5000.0f, 3500.0f, -7000.0f};

    g_launch_speed_scale = 1.0f;

    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.name = "ground";
        bodyDef.position = origin;
        b3BodyId groundId = b3CreateBody(g_world, &bodyDef);

        f32 scale = 0.01f;
        TempMesh tempMesh;
        load_temp_mesh("data/meshes/collision_mesh_01.obj", &tempMesh, scale, true);

        b3MeshDef def = b3MeshDef{};
        def.vertices = tempMesh.vertices;
        def.vertexCount = tempMesh.vertexCount;
        def.indices = tempMesh.indices;
        def.triangleCount = tempMesh.triangleCount;
        def.materialIndices = tempMesh.materialIndices;
        def.useMedianSplit = true;

        // this has a big impact on stability because the faces are nearly coplanar
        def.identifyEdges = true;

        def.weldVertices = true;
        def.weldTolerance = 0.002f;

        g_vm_mesh_data = tempMesh.vertexCount > 0 ? b3CreateMesh(&def, null, 0) : null;

        if g_vm_mesh_data != null {
            b3ShapeDef shapeDef = b3DefaultShapeDef();
            ignore b3CreateMeshShape(groundId, &shapeDef, g_vm_mesh_data, b3Vec3_one);
        }

        destroy_temp_mesh(&tempMesh);
    }

    {
        b3Vec3[16] points;
        points[0] = b3Vec3{-3.13548756f, 3.81141949f, 237.289047f};
        points[1] = b3Vec3{-16.2333279f, -23.4977913f, 235.486603f};
        points[2] = b3Vec3{-13.8834839f, 6.20244455f, 23.7760544f};
        points[3] = b3Vec3{14.0794125f, 4.63170528f, 24.9530792f};
        points[4] = b3Vec3{3.98322797f, -16.4192238f, 236.704071f};
        points[5] = b3Vec3{-23.3520412f, -3.26714420f, 236.071594f};
        points[6] = b3Vec3{13.4517860f, -6.94963741f, 24.4085312f};
        points[7] = b3Vec3{-5.24953651f, 13.9316301f, 24.5058060f};
        points[8] = b3Vec3{-4.65071201f, -24.1484108f, 235.974121f};
        points[9] = b3Vec3{-14.5111103f, -5.37889385f, 23.2315063f};
        points[10] = b3Vec3{6.33307076f, 13.2810068f, 24.9935150f};
        points[11] = b3Vec3{4.81784487f, -14.6788225f, 23.6787796f};
        points[12] = b3Vec3{-14.7180958f, 4.46204281f, 236.801331f};
        points[13] = b3Vec3{-23.9796677f, -14.8484812f, 235.527039f};
        points[14] = b3Vec3{4.61085415f, -4.83788204f, 237.248611f};
        points[15] = b3Vec3{-6.76476669f, -14.0281992f, 23.1910706f};

        for i32 i = 0; i < 16; i += 1 {
            b3Vec3 p = points[i];
            points[i] = b3MulSV(0.01f, b3Vec3{p.y, p.z, p.x});
        }

        b3HullData* hull = b3CreateHull(cast(b3Vec3*, &points), 16, 16);
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.name = "cylinder";
        bodyDef.type = b3_dynamicBody;
        bodyDef.position = b3Pos{5020.27734f, 3506.22559f, -6986.48584f};
        bodyDef.rotation = b3Quat{b3Vec3{0.664546967f, 0.669287264f, 0.135021493f}, 0.303646326f};

        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);

        b3ShapeDef shapeDef = b3DefaultShapeDef();
        shapeDef.baseMaterial.rollingResistance = 0.1f;
        ignore b3CreateHullShape(bodyId, &shapeDef, hull);
        b3DestroyHull(hull);
    }

    // float jointFrictionTorque = 5.0f;
    // float jointHertz = 1.0f;
    // float jointDampingRatio = 0.7f;

    // CreateHuman( &m_human, m_worldId, b3Vec3{ 15.0f, 10.0f, 15.0f } + origin, jointFrictionTorque, jointHertz,
    // jointDampingRatio, 1, nullptr, 			 false );
}

void destroy_voxel_mesh() {
    if g_vm_mesh_data != null {
        b3DestroyMesh(g_vm_mesh_data);
        g_vm_mesh_data = null;
    }
}

void step_voxel_mesh(f32 timeStep) {
    ignore timeStep;
    if g_vm_mesh_data == null {
        draw_text_line("data/meshes/collision_mesh_01.obj could not be read");
    }
}

// samples/sample_mesh.cpp MeshReflection
const i32 MR_HUMAN_COUNT = 20;
b3Vec3 g_mr_scale;
b3MeshData* g_mr_grid_mesh;
b3MeshData* g_mr_building_mesh;
b3BodyId g_mr_mesh_body;
b3ShapeId g_mr_mesh_shape;
Human[MR_HUMAN_COUNT] g_mr_humans;

void build_mesh_reflection() {
    b3BodyDef bodyDef = b3DefaultBodyDef();
    b3ShapeDef shapeDef = b3DefaultShapeDef();

    {
        b3BodyId body = b3CreateBody(g_world, &bodyDef);
        g_mr_grid_mesh = b3CreateGridMesh(20, 20, 2.0f, 2, true);
        ignore b3CreateMeshShape(body, &shapeDef, g_mr_grid_mesh, b3Vec3_one);
    }

    {
        g_mr_building_mesh = create_mesh_data("data/meshes/building.obj", 1.0f, false, false,
                                              true, true);

        b3ShapeDef meshShapeDef = b3DefaultShapeDef();
        b3SurfaceMaterial[3] materials;
        materials[0] = b3SurfaceMaterial{};
        materials[0].friction = 0.6f;
        materials[1] = b3SurfaceMaterial{};
        materials[1].friction = 0.0f;
        materials[1].restitution = 0.95f;
        materials[1].userMaterialId = cast(u64, 1);
        materials[2] = b3SurfaceMaterial{};
        materials[2].friction = 0.2f;
        materials[2].restitution = 0.2f;
        materials[2].userMaterialId = cast(u64, 2);
        meshShapeDef.materials = cast(b3SurfaceMaterial*, &materials);
        meshShapeDef.materialCount = 3;

        bodyDef.position = b3Pos{-10.0f, 0.0f, 0.0f};
        b3BodyId body = b3CreateBody(g_world, &bodyDef);
        if g_mr_building_mesh != null {
            ignore b3CreateMeshShape(body, &meshShapeDef, g_mr_building_mesh, b3Vec3_one);
        }

        g_mr_scale = b3Vec3{-1.0f, 1.0f, 1.0f};
        bodyDef.position = b3Pos{10.0f, 0.0f, 0.0f};
        g_mr_mesh_body = b3CreateBody(g_world, &bodyDef);
        if g_mr_building_mesh != null {
            g_mr_mesh_shape = b3CreateMeshShape(g_mr_mesh_body, &meshShapeDef,
                                                g_mr_building_mesh, g_mr_scale);
        }
    }

    bodyDef.type = b3_dynamicBody;
    {
        bodyDef.position = b3Pos{6.0f, 15.0f, 0.0f};
        b3BodyId body = b3CreateBody(g_world, &bodyDef);
        b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, 0.5f};
        shapeDef.baseMaterial.rollingResistance = 0.2f;
        shapeDef.baseMaterial.userMaterialId = cast(u64, 42);
        ignore b3CreateSphereShape(body, &shapeDef, &sphere);
    }

    {
        bodyDef.position = b3Pos{9.0f, 15.0f, 0.0f};
        b3BodyId body = b3CreateBody(g_world, &bodyDef);
        b3Capsule capsule = b3Capsule{b3Pos{-0.5f, 0.5f, 0.0f}, b3Pos{0.5f, 0.0f, 0.0f}, 0.25f};
        shapeDef.baseMaterial.rollingResistance = 0.2f;
        shapeDef.baseMaterial.userMaterialId = cast(u64, 11);
        ignore b3CreateCapsuleShape(body, &shapeDef, &capsule);
    }

    {
        bodyDef.position = b3Pos{12.0f, 15.0f, 0.0f};
        b3BodyId body = b3CreateBody(g_world, &bodyDef);
        b3BoxHull hull = b3MakeBoxHull(0.25f, 0.5f, 0.75f);
        shapeDef.baseMaterial.userMaterialId = cast(u64, 555);
        ignore b3CreateHullShape(body, &shapeDef, &hull.base);
    }

    f32 frictionTorque = 5.0f;
    f32 hertz = 1.0f;
    f32 dampingRatio = 0.7f;
    bool colorize = false;
    for i32 humanIndex = 0; humanIndex < MR_HUMAN_COUNT; humanIndex += 1 {
        g_mr_humans[humanIndex] = Human{};

        i32 groupIndex = humanIndex;
        b3Pos position = b3Pos{-14.0f + 1.5f * cast(f32, humanIndex), 8.0f, 0.0f};
        CreateHuman(&g_mr_humans[humanIndex], g_world, position, frictionTorque, hertz,
                    dampingRatio, groupIndex, null, colorize);
    }
}

void destroy_mesh_reflection() {
    b3DestroyMesh(g_mr_grid_mesh);
    if g_mr_building_mesh != null {
        b3DestroyMesh(g_mr_building_mesh);
        g_mr_building_mesh = null;
    }

    for i32 i = 0; i < MR_HUMAN_COUNT; i += 1 {
        DestroyHuman(&g_mr_humans[i]);
    }
}

bool mesh_reflection_controls() {
    bool changed = false;
    b3Vec3 scale = g_mr_scale;
    if ImGui_RadioButton("Neg X", scale.x < 0.0f) {
        scale.x = -1.0f;
        changed = true;
    }

    ImGui_SameLine(0.0f, -1.0f);

    if ImGui_RadioButton("Pos X", scale.x > 0.0f) {
        scale.x = 1.0f;
        changed = true;
    }

    if ImGui_RadioButton("Neg Y", scale.y < 0.0f) {
        scale.y = -1.0f;
        changed = true;
    }

    ImGui_SameLine(0.0f, -1.0f);

    if ImGui_RadioButton("Pos Y", scale.y > 0.0f) {
        scale.y = 1.0f;
        changed = true;
    }

    if ImGui_RadioButton("Neg Z", scale.z < 0.0f) {
        scale.z = -1.0f;
        changed = true;
    }

    ImGui_SameLine(0.0f, -1.0f);

    if ImGui_RadioButton("Pos Z", scale.z > 0.0f) {
        scale.z = 1.0f;
        changed = true;
    }

    if changed {
        g_mr_scale = scale;

        if g_mr_building_mesh != null {
            b3Shape_SetMesh(g_mr_mesh_shape, g_mr_building_mesh, g_mr_scale);
        }
    }

    return true;
}

void step_mesh_reflection(f32 timeStep) {
    ignore timeStep;
    u8[128] buf;
    ignore snprintf(cast(u8*, &buf), 128, "scale = (%.2g, %.2g, %.2g)",
                    cast(f64, g_mr_scale.x), cast(f64, g_mr_scale.y), cast(f64, g_mr_scale.z));
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 128, "surface type = %d", cast(i32, g_user_material_id));
    draw_text_line(cast(u8*, &buf));
}

// samples/sample_mesh.cpp CastContext
// Renamed: sample_collision.cpp has an unrelated CastContext, and both
// land in one namespace here.
struct MeshCastContext {
    b3Pos point;
    b3Vec3 normal;
    f32 fraction;
    bool hit;
}

// samples/sample_mesh.cpp HeightField
b3BodyId g_hf_ground;
b3BodyId g_hf_body1;
b3BodyId g_hf_body2;
f32 g_hf_amplitude;
b3Vec3 g_hf_scale;
b3Vec3 g_hf_ray_origin;
b3Vec3 g_hf_ray_translation;
b3HeightFieldData* g_hf_height_field;
f32 g_hf_radius;
i32 g_hf_row_count;
i32 g_hf_column_count;
bool g_hf_holes;

void height_field_create_scene() {
    if g_hf_ground.index1 != 0 {
        b3DestroyBody(g_hf_ground);
        g_hf_ground = b3BodyId{};
    }

    if g_hf_body1.index1 != 0 {
        b3DestroyBody(g_hf_body1);
        g_hf_body1 = b3BodyId{};
    }

    if g_hf_body2.index1 != 0 {
        b3DestroyBody(g_hf_body2);
        g_hf_body2 = b3BodyId{};
    }

    if g_hf_height_field != null {
        b3DestroyHeightField(g_hf_height_field);
    }

    b3BodyDef bodyDef = b3DefaultBodyDef();
    b3ShapeDef shapeDef = b3DefaultShapeDef();

    g_hf_scale = b3Vec3{2.0f, 2.0f * g_hf_amplitude, 2.0f};

    if g_hf_amplitude == 0.0f {
        g_hf_height_field = b3CreateGrid(g_hf_row_count, g_hf_column_count, g_hf_scale,
                                         g_hf_holes);
    } else {
        g_hf_height_field = b3CreateWave(g_hf_row_count, g_hf_column_count, g_hf_scale, 0.1f,
                                         0.03333f, g_hf_holes);
    }

    bodyDef.position = b3Pos{
        -0.5f * g_hf_height_field.scale.x * cast(f32, g_hf_column_count - 1), 0.0f,
        -0.5f * g_hf_height_field.scale.z * cast(f32, g_hf_row_count - 1)};

    // bodyDef.rotation = b3MakeQuatFromAxisAngle( b3Vec3_axisY, 0.25f * B3_PI );

    g_hf_ground = b3CreateBody(g_world, &bodyDef);
    ignore b3CreateHeightFieldShape(g_hf_ground, &shapeDef, g_hf_height_field);
}

void build_mesh_height_field() {
    g_hf_row_count = 400;
    g_hf_column_count = 400;

    g_hf_ground = b3BodyId{};
    g_hf_body1 = b3BodyId{};
    g_hf_body2 = b3BodyId{};
    g_hf_amplitude = 0.75f;
    g_hf_holes = false;

    // m_rayOrigin = { 1.4f, 2.0f, 1.5f };
    // m_rayTranslation = { -3.0f, -4.0f, 3.0f };
    // m_rayOrigin = { 30.0f, 2.0f, 0.0f };
    // m_rayTranslation = { -60.0f, -4.0f, 0.0f };
    g_hf_ray_origin = b3Vec3{5.5f, 4.0f, 1.01f};
    g_hf_ray_translation = b3Vec3{0.0f, -8.0f, 0.0f};
    g_hf_radius = 0.2f;
    g_hf_height_field = null;

    height_field_create_scene();
}

void destroy_mesh_height_field() {
    b3DestroyHeightField(g_hf_height_field);
    g_hf_height_field = null;
}

bool mesh_height_field_controls() {
    ImGui_PushItemWidth(6.0f * ImGui_GetFontSize());

    if ImGui_SliderInt("columns", &g_hf_column_count, 1, 500, "%d", 0) {
        height_field_create_scene();
    }

    if ImGui_SliderInt("rows", &g_hf_row_count, 1, 500, "%d", 0) {
        height_field_create_scene();
    }

    if ImGui_SliderFloat("amplitude", &g_hf_amplitude, 0.0f, 2.0f, "%.3f", 0) {
        height_field_create_scene();
    }

    if ImGui_Checkbox("holes", &g_hf_holes) {
        height_field_create_scene();
    }

    ignore ImGui_SliderFloat("ray x", &g_hf_ray_origin.x,
                             -2.0f * g_hf_radius - 0.5f * g_hf_scale.x * cast(f32, g_hf_column_count - 1),
                             2.0f * g_hf_radius + 0.5f * g_hf_scale.x * cast(f32, g_hf_column_count - 1),
                             "%.3f", 0);
    ignore ImGui_SliderFloat("ray z", &g_hf_ray_origin.z,
                             -2.0f * g_hf_radius - 0.5f * g_hf_scale.z * cast(f32, g_hf_row_count - 1),
                             2.0f * g_hf_radius + 0.5f * g_hf_scale.z * cast(f32, g_hf_row_count - 1),
                             "%.3f", 0);
    ignore ImGui_SliderFloat("delta x", &g_hf_ray_translation.x,
                             -2.0f * g_hf_scale.x * cast(f32, g_hf_column_count - 1),
                             2.0f * g_hf_scale.x * cast(f32, g_hf_column_count - 1), "%.3f", 0);
    ignore ImGui_SliderFloat("delta z", &g_hf_ray_translation.z,
                             -2.0f * g_hf_scale.z * cast(f32, g_hf_row_count - 1),
                             2.0f * g_hf_scale.z * cast(f32, g_hf_row_count - 1), "%.3f", 0);
    ignore ImGui_SliderFloat("radius", &g_hf_radius, 0.0f, 1.0f, "%.3f", 0);

    ImGui_PopItemWidth();
    return true;
}

// This callback finds the closest hit.
f32 mesh_height_field_cast_callback(b3ShapeId shapeId, b3Pos point, b3Vec3 normal, f32 fraction,
                               u64 surfaceType, i32 triangleIndex, i32 childIndex,
                               void* context) {
    ignore shapeId;
    ignore surfaceType;
    ignore triangleIndex;
    ignore childIndex;

    MeshCastContext* castContext = cast(MeshCastContext*, context);
    castContext.point = point;
    castContext.normal = normal;
    castContext.fraction = fraction;
    castContext.hit = true;

    // By returning the current fraction, we instruct the calling code to clip the ray and
    // continue the ray-cast to the next shape. WARNING: do not assume that shapes
    // are reported in order. However, by clipping, we can always get the closest shape.
    return fraction;
}

void step_mesh_height_field(f32 timeStep) {
    ignore timeStep;

    b3Transform transform = b3Transform{b3Vec3{0.0f, 0.1f, 0.0f}, b3Quat_identity};
    dbg_axes(b3MakeWorldTransform(transform), 0.5f);

    if g_hf_radius == 0.0f {
        // m_rayOrigin = { 0.0f, -FLT_EPSILON, 0.0f };
        // m_rayTranslation = { -1000.0f, 0.0f, 0.0 };
        b3Pos origin = b3ToPos(g_hf_ray_origin);
        b3RayResult result = b3World_CastRayClosest(g_world, origin, g_hf_ray_translation,
                                                    b3DefaultQueryFilter());

        adapter_point(origin, 6.0f, b3_colorGreenYellow, null);
        adapter_point(b3OffsetPos(origin, g_hf_ray_translation), 6.0f, b3_colorRed, null);
        dbg_line(origin, b3OffsetPos(origin, g_hf_ray_translation), b3_colorGray);

        if result.hit {
            b3Pos point = result.point;
            dbg_line(point, b3OffsetPos(point, b3MulSV(0.5f, result.normal)), b3_colorGray);
            adapter_point(point, 10.0f, b3_colorOrange, null);
        }
    } else {
        b3ShapeProxy proxy;
        proxy.points = &g_hf_ray_origin;
        proxy.count = 1;
        proxy.radius = g_hf_radius;

        MeshCastContext result = MeshCastContext{};
        result.fraction = 1.0f;
        b3World_CastShape(g_world, b3Pos_zero, &proxy, g_hf_ray_translation,
                          b3DefaultQueryFilter(), mesh_height_field_cast_callback,
                          cast(void*, &result));

        b3Pos origin = b3ToPos(g_hf_ray_origin);
        adapter_point(origin, 2.0f, b3_colorGreen, null);
        adapter_point(b3OffsetPos(origin, g_hf_ray_translation), 2.0f, b3_colorRed, null);
        dbg_line(origin, b3OffsetPos(origin, g_hf_ray_translation), b3_colorYellow);

        b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, g_hf_radius};
        b3Transform sphereXf = b3Transform{
            b3Add(g_hf_ray_origin, b3MulSV(result.fraction, g_hf_ray_translation)),
            b3Quat_identity};
        dbg_solid_sphere(b3MakeWorldTransform(sphereXf), sphere, make_color(b3_colorOrange));

        if result.hit {
            b3Pos point = result.point;
            dbg_line(point, b3OffsetPos(point, b3MulSV(0.5f, result.normal)), b3_colorGreen);
            adapter_point(point, 6.0f, b3_colorPurple, null);
        }
    }
}

// samples/sample_mesh.cpp MeshViewer
const i32 MV_MESH_COUNT = 4;
const i32 MV_DEGENERATE_CAPACITY = 64;
// upstream walks the node tree with a std::queue; this is the same BFS
// over fixed storage, sized past the widest level any of these meshes
// reaches.
const i32 MV_QUEUE_CAPACITY = 8192;

TempMesh g_mv_temp_mesh;
b3MeshData* g_mv_mesh;
b3BodyId g_mv_body;
f32 g_mv_area;
f32 g_mv_weld_tolerance_millimeters;
f32 g_mv_build_time;
i32 g_mv_mesh_index;
i32[MV_DEGENERATE_CAPACITY] g_mv_degenerate_triangles;
i32 g_mv_draw_level;
i32 g_mv_height;
bool g_mv_median_split;
bool g_mv_concave_edges;
bool g_mv_weld_vertices;

b3MeshNode*[MV_QUEUE_CAPACITY] g_mv_queue_node;
i32[MV_QUEUE_CAPACITY] g_mv_queue_level;

void mesh_viewer_load_mesh() {
    if g_mv_body.index1 != 0 {
        b3DestroyBody(g_mv_body);
        g_mv_body = b3BodyId{};
    }

    if g_mv_mesh != null {
        b3DestroyMesh(g_mv_mesh);
        g_mv_mesh = null;
    }

    destroy_temp_mesh(&g_mv_temp_mesh);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    g_mv_body = b3CreateBody(g_world, &bodyDef);

    f32 scale = 0.01f;
    str[MV_MESH_COUNT] filesNames;
    filesNames[0] = "voxel_mesh_01.obj";
    filesNames[1] = "voxel_mesh_02.obj";
    filesNames[2] = "voxel_mesh_03.obj";
    filesNames[3] = "voxel_mesh_04.obj";

    u8[64] buffer;
    ignore snprintf(cast(u8*, &buffer), 64, "data/meshes/%s",
                    str_to_cstr(filesNames[g_mv_mesh_index]));
    load_temp_mesh(str_from_cstr(cast(u8*, &buffer)), &g_mv_temp_mesh, scale, true);
    if g_mv_temp_mesh.vertexCount == 0 { return; }

    b3MeshDef def = b3MeshDef{};
    def.vertices = g_mv_temp_mesh.vertices;
    def.vertexCount = g_mv_temp_mesh.vertexCount;
    def.indices = g_mv_temp_mesh.indices;
    def.triangleCount = g_mv_temp_mesh.triangleCount;
    def.materialIndices = g_mv_temp_mesh.materialIndices;
    def.useMedianSplit = g_mv_median_split;
    def.identifyEdges = g_mv_concave_edges;
    def.weldVertices = g_mv_weld_vertices;
    def.weldTolerance = 0.001f * g_mv_weld_tolerance_millimeters;

    u64 startTicks = b3GetTicks();
    g_mv_mesh = b3CreateMesh(&def, cast(i32*, &g_mv_degenerate_triangles),
                             MV_DEGENERATE_CAPACITY);
    g_mv_build_time = b3GetMilliseconds(startTicks);

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    ignore b3CreateMeshShape(g_mv_body, &shapeDef, g_mv_mesh, b3Vec3_one);

    g_mv_area = compute_internal_surface_area(g_mv_mesh);
    g_mv_height = b3GetHeight(g_mv_mesh);
    g_mv_draw_level = b3ClampInt(g_mv_draw_level, -1, g_mv_height);
}

void build_mesh_viewer() {
    g_mv_mesh_index = 0;
    g_mv_mesh = null;
    g_mv_body = b3BodyId{};
    g_mv_median_split = true;
    g_mv_concave_edges = true;
    g_mv_weld_vertices = true;
    g_mv_weld_tolerance_millimeters = 1.5f;
    g_mv_area = 0.0f;
    g_mv_draw_level = -1;
    g_mv_height = 0;
    g_mv_build_time = 0.0f;
    g_launch_speed_scale = 1.0f;
    g_mv_temp_mesh = TempMesh{};
    mesh_viewer_load_mesh();
}

void destroy_mesh_viewer() {
    if g_mv_mesh != null {
        b3DestroyMesh(g_mv_mesh);
        g_mv_mesh = null;
    }
    destroy_temp_mesh(&g_mv_temp_mesh);
}

void mesh_viewer_draw_nodes() {
    if g_mv_draw_level < 0 || g_mv_mesh == null {
        return;
    }

    i32 count = g_mv_mesh.nodeCount;
    if count == 0 {
        return;
    }

    const i32 colorCount = 20;
    b3HexColor[colorCount] colors = {
        b3_colorAliceBlue, b3_colorAntiqueWhite,   b3_colorAqua,           b3_colorAquamarine, b3_colorAzure,
        b3_colorBeige,     b3_colorBisque,         b3_colorBlanchedAlmond, b3_colorBlue,       b3_colorBlueViolet,
        b3_colorBrown,     b3_colorBurlywood,      b3_colorCadetBlue,      b3_colorChartreuse, b3_colorChocolate,
        b3_colorCoral,     b3_colorCornflowerBlue, b3_colorCornsilk,       b3_colorCrimson,    b3_colorCyan,
    };

    b3MeshNode* nodes = b3GetMeshNodes(g_mv_mesh);
    b3MeshNode* root = nodes + 0;

    // Start with the root at level 0
    i32 head = 0;
    i32 tail = 0;
    g_mv_queue_node[tail] = root;
    g_mv_queue_level[tail] = 0;
    tail += 1;

    while head < tail {
        b3MeshNode* node = g_mv_queue_node[head];
        i32 level = g_mv_queue_level[head];
        head += 1;

        // If the current level matches the target level, add the node's data to the result
        if level == g_mv_draw_level {
            b3AABB box = b3AABB{node.lowerBound, node.upperBound};
            adapter_bounds(box, colors[level % colorCount], null);

            i32 axis = cast(i32, node.data.asNode.axis);
            b3Vec3 center = b3AABB_Center(box);
            b3Pos centerPos = b3ToPos(center);
            if axis == 0 {
                dbg_arrow(centerPos, b3OffsetPos(centerPos, b3MulSV(0.1f, b3Vec3_axisX)),
                          b3_colorRed);
            } else if axis == 1 {
                dbg_arrow(centerPos, b3OffsetPos(centerPos, b3MulSV(0.1f, b3Vec3_axisY)),
                          b3_colorGreen);
            } else if axis == 2 {
                dbg_arrow(centerPos, b3OffsetPos(centerPos, b3MulSV(0.1f, b3Vec3_axisZ)),
                          b3_colorBlue);
            } else {
                b3Sphere sphere = b3Sphere{center, 0.03f};
                dbg_solid_sphere(b3WorldTransform_identity, sphere, make_color(b3_colorOrange));
            }
        }

        // If the current level exceeds the target level, stop processing
        if level > g_mv_draw_level {
            break;
        }

        bool isLeaf = node.data.asNode.axis == cast(u32, 3);
        if isLeaf == false && tail + 2 <= MV_QUEUE_CAPACITY {
            // Enqueue the left and right children with their respective levels
            // I'm using some internal knowledge of how the node data is organized.
            g_mv_queue_node[tail] = node + 1;
            g_mv_queue_level[tail] = level + 1;
            tail += 1;
            g_mv_queue_node[tail] = node + cast(i32, node.data.asNode.childOffset);
            g_mv_queue_level[tail] = level + 1;
            tail += 1;
        }
    }
}

bool mesh_viewer_controls() {
    if ImGui_SliderInt("index", &g_mv_mesh_index, 0, MV_MESH_COUNT - 1, "%d", 0) {
        mesh_viewer_load_mesh();
    }

    if ImGui_RadioButton("median split", g_mv_median_split) {
        g_mv_median_split = true;
        mesh_viewer_load_mesh();
    }

    if ImGui_RadioButton("sah binning", g_mv_median_split == false) {
        g_mv_median_split = false;
        mesh_viewer_load_mesh();
    }

    if ImGui_Checkbox("concave edges", &g_mv_concave_edges) {
        mesh_viewer_load_mesh();
    }

    if ImGui_Checkbox("weld vertices", &g_mv_weld_vertices) {
        mesh_viewer_load_mesh();
    }

    if ImGui_SliderFloat("tolerance", &g_mv_weld_tolerance_millimeters, 0.0f, 10.0f,
                         "%.1f", 0) {
        mesh_viewer_load_mesh();
    }

    ignore ImGui_SliderInt("draw level", &g_mv_draw_level, -1, g_mv_height, "%d", 0);
    return true;
}

void step_mesh_viewer(f32 timeStep) {
    ignore timeStep;
    if g_mv_mesh == null {
        draw_text_line("data/meshes/voxel_mesh_0*.obj could not be read");
        return;
    }

    dbg_axes(b3WorldTransform_identity, 1.0f);

    u8[128] buf;
    ignore snprintf(cast(u8*, &buf), 128, "triangle count = %d", g_mv_mesh.triangleCount);
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 128, "vertex count = %d", g_mv_mesh.vertexCount);
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 128, "degenerate count = %d", g_mv_mesh.degenerateCount);
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 128, "node area = %g", cast(f64, g_mv_area));
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 128, "height = %d", g_mv_height);
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 128, "build time (ms) = %g", cast(f64, g_mv_build_time));
    draw_text_line(cast(u8*, &buf));

    mesh_viewer_draw_nodes();

    b3Vec3 offset = b3Vec3{0.01f, 0.01f, 0.01f};
    for i32 i = 0; i < g_mv_mesh.degenerateCount && i < MV_DEGENERATE_CAPACITY; i += 1 {
        i32 triangleIndex = g_mv_degenerate_triangles[i];
        i32 i1 = g_mv_temp_mesh.indices[3 * triangleIndex + 0];
        i32 i2 = g_mv_temp_mesh.indices[3 * triangleIndex + 1];
        i32 i3 = g_mv_temp_mesh.indices[3 * triangleIndex + 2];
        b3Vec3 v1 = g_mv_temp_mesh.vertices[i1];
        b3Vec3 v2 = g_mv_temp_mesh.vertices[i2];
        b3Vec3 v3 = g_mv_temp_mesh.vertices[i3];
        b3Vec3 p = b3MulSV(1.0f / 3.0f, b3Add(b3Add(v1, v2), v3));
        b3Pos pPos = b3ToPos(p);
        adapter_point(pPos, 10.0f, b3_colorCyan, null);
        ignore snprintf(cast(u8*, &buf), 128, "%d", triangleIndex);
        dbg_string_3d(b3OffsetPos(pPos, offset), make_color(b3_colorOrange), cast(u8*, &buf));
        {
            b3Pos p1 = b3ToPos(v1);
            b3Pos p2 = b3ToPos(v2);
            b3Pos p3 = b3ToPos(v3);
            adapter_point(p1, 10.0f, b3_colorRed, null);
            adapter_point(p2, 10.0f, b3_colorGreen, null);
            adapter_point(p3, 10.0f, b3_colorBlue, null);
            ignore snprintf(cast(u8*, &buf), 128, "%d", i1);
            dbg_string_3d(b3OffsetPos(p1, offset), make_color(b3_colorRed), cast(u8*, &buf));
            ignore snprintf(cast(u8*, &buf), 128, "%d", i2);
            dbg_string_3d(b3OffsetPos(p2, offset), make_color(b3_colorGreen), cast(u8*, &buf));
            ignore snprintf(cast(u8*, &buf), 128, "%d", i3);
            dbg_string_3d(b3OffsetPos(p3, offset), make_color(b3_colorBlue), cast(u8*, &buf));
        }
    }
}
