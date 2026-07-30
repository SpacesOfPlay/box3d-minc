// Compound scenes. Ports of samples/sample_compound.cpp.

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
import sample_shapes;
import sample_collision;
import mesh_loader;
import mover_shim;
import box3d_mover;

// samples/sample_compound.cpp SimpleCompound
b3CompoundData* g_sc_compound;

void build_simple_compound() {
    {
        f32 a = 4.0f;
        b3Vec3 extents = b3Vec3{a, 0.125f * a, a};
        b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
        b3BoxHull box = b3MakeBoxHull(extents.x, extents.y, extents.z);
        b3Transform hullTransform;
        hullTransform.p = b3Vec3{1.0f, -0.125f * a, 0.0f};
        hullTransform.q = b3MakeQuatFromAxisAngle(
            b3Normalize(b3Vec3{1.0f, 0.0f, 1.0f}), 0.0f * PI_F);
        b3CompoundHullDef hullDef;
        hullDef.hull = &box.base;
        hullDef.transform = hullTransform;
        hullDef.material = material;
        b3CompoundDef def = b3CompoundDef{};
        def.hulls = &hullDef;
        def.hullCount = 1;
        g_sc_compound = b3CreateCompound(&def);

        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{2.0f, -1.0f, 0.0f};
        bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3{0.0f, 1.0f, 0.0f}, 0.25f * PI_F);
        b3BodyId groundId = b3CreateBody(g_world, &bodyDef);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        ignore b3CreateBakedCompoundShape(groundId, &shapeDef, g_sc_compound);
    }
    b3World_SetContactRecycleDistance(g_world, 0.0f);
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;
        bodyDef.position = b3Pos{0.0f, 2.0f, 0.0f};
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, 0.25f};
        ignore b3CreateSphereShape(bodyId, &shapeDef, &sphere);
    }
    //{
    //	b3BodyDef bodyDef = b3DefaultBodyDef();
    //	bodyDef.type = b3_dynamicBody;
    //	bodyDef.position = { 0.0f, 10.0f, 2.0f };
    //	b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
    //	b3ShapeDef shapeDef = b3DefaultShapeDef();
    //	b3BoxHull box = b3MakeBoxHull( 0.2f, 0.3f, 0.4f );
    //	b3CreateHullShape( bodyId, &shapeDef, &box.base );
    //}
}

// samples/sample_compound.cpp CompoundSpheres
const i32 CSPHERE_COUNT = 20;
b3CompoundData* g_cs_compound;

void build_compound_spheres() {
    f32 h = 10.0f;
    b3Vec3 lower = b3Vec3{-h, -h, -h};
    b3Vec3 upper = b3Vec3{h, h, h};
    b3CompoundSphereDef[CSPHERE_COUNT] spheres;
    for i32 i = 0; i < CSPHERE_COUNT; i++ {
        spheres[i].sphere.center = random_pos(lower, upper);
        spheres[i].sphere.radius = random_float_range(0.01f * h, 0.05f * h);
        spheres[i].material = b3DefaultSurfaceMaterial();
    }
    b3CompoundDef def = b3CompoundDef{};
    def.spheres = &spheres[0];
    def.sphereCount = CSPHERE_COUNT;
    g_cs_compound = b3CreateCompound(&def);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    b3BodyId groundId = b3CreateBody(g_world, &bodyDef);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    ignore b3CreateBakedCompoundShape(groundId, &shapeDef, g_cs_compound);
}

void step_compound_spheres(f32 timeStep) {
    ignore timeStep;
    i32 height = b3DynamicTree_GetHeight(&g_cs_compound.tree);
    u8[128] buf;
    ignore snprintf(cast(u8*, &buf), 128, "compound tree height = %d", height);
    draw_text_line(cast(u8*, &buf));
}

// samples/sample_compound.cpp CompoundHulls
const i32 CHULL_COUNT = 20;
b3CompoundData* g_ch_compound;

void build_compound_hulls() {
    f32 h = 10.0f;
    b3Vec3 lower = b3Vec3{-h, -h, -h};
    b3Vec3 upper = b3Vec3{h, h, h};
    b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
    b3BoxHull[CHULL_COUNT] boxHulls;
    b3CompoundHullDef[CHULL_COUNT] hulls;
    for i32 i = 0; i < CHULL_COUNT; i++ {
        b3Vec3 extents = b3Vec3{random_float_range(0.01f * h, 0.05f * h),
                                random_float_range(0.01f * h, 0.05f * h),
                                random_float_range(0.01f * h, 0.05f * h)};
        b3Transform transform;
        transform.p = random_pos(lower, upper);
        transform.q = random_quat();
        boxHulls[i] = b3MakeBoxHull(extents.x, extents.y, extents.z);
        hulls[i].hull = &boxHulls[i].base;
        hulls[i].transform = transform;
        hulls[i].material = material;
    }
    b3CompoundDef def = b3CompoundDef{};
    def.hulls = &hulls[0];
    def.hullCount = CHULL_COUNT;
    g_ch_compound = b3CreateCompound(&def);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    b3BodyId groundId = b3CreateBody(g_world, &bodyDef);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    ignore b3CreateBakedCompoundShape(groundId, &shapeDef, g_ch_compound);
}

void step_compound_hulls(f32 timeStep) {
    ignore timeStep;
    i32 height = b3DynamicTree_GetHeight(&g_ch_compound.tree);
    u8[128] buf;
    ignore snprintf(cast(u8*, &buf), 128, "compound tree height = %d", height);
    draw_text_line(cast(u8*, &buf));
}

// samples/sample_compound.cpp TileFloor
const i32 TF_GRID_COUNT = 50;
const i32 TF_BOX_COUNT = TF_GRID_COUNT * TF_GRID_COUNT;
b3CompoundData* g_tf_compound;
b3CompoundHullDef[TF_BOX_COUNT] g_tf_hulls;

void build_tile_floor() {
    {
        f32 a = 4.0f;
        b3Vec3 extents = b3Vec3{a, 0.5f * a, a};
        b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
        b3BoxHull box = b3MakeBoxHull(extents.x, extents.y, extents.z);
        b3Transform transform = b3Transform_identity;
        i32 index = 0;
        for i32 i = 0; i < TF_GRID_COUNT; i++ {
            transform.p.x = (2.0f * cast(f32, i) - cast(f32, TF_GRID_COUNT)) * a;
            for i32 j = 0; j < TF_GRID_COUNT; j++ {
                transform.p.z = (2.0f * cast(f32, j) - cast(f32, TF_GRID_COUNT)) * a;
                transform.p.y = random_float_range(-0.5f, 0.25f) * a;
                g_tf_hulls[index].hull = &box.base;
                g_tf_hulls[index].transform = transform;
                g_tf_hulls[index].material = material;
                index += 1;
            }
        }
        b3CompoundDef def = b3CompoundDef{};
        def.hulls = &g_tf_hulls[0];
        def.hullCount = TF_BOX_COUNT;
        g_tf_compound = b3CreateCompound(&def);

        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{-2.0f, 1.0f, -3.0f};
        bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Normalize(b3Vec3{1.0f, -1.0f, 0.5f}), 0.0f);
        b3BodyId groundId = b3CreateBody(g_world, &bodyDef);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        ignore b3CreateBakedCompoundShape(groundId, &shapeDef, g_tf_compound);
    }
    // b3World_SetContactRecycleDistance( m_worldId, 0.0f );
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;
        bodyDef.position = b3Pos{3.0f, 12.0f, 0.0f};
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, 0.25f};
        ignore b3CreateSphereShape(bodyId, &shapeDef, &sphere);
    }
}

void step_tile_floor(f32 timeStep) {
    ignore timeStep;
    u8[160] buf;
    ignore snprintf(cast(u8*, &buf), 160, "compound hull count = %d, mesh count = %d",
                    g_tf_compound.hullCount, g_tf_compound.meshCount);
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 160, "compound byte count = %d", g_tf_compound.byteCount);
    draw_text_line(cast(u8*, &buf));
    i32 treeBytes = b3DynamicTree_GetByteCount(&g_tf_compound.tree);
    i32 height = b3DynamicTree_GetHeight(&g_tf_compound.tree);
    ignore snprintf(cast(u8*, &buf), 160, "compound tree byte count = %d, height = %d",
                    treeBytes, height);
    draw_text_line(cast(u8*, &buf));
}

// samples/sample_compound.cpp MeshTile
b3CompoundData* g_mt_compound;

void build_mesh_tile() {
    {
        const i32 gridCount = 2;
        const i32 boxCount = gridCount * gridCount;

        f32 a = 4.0f;
        b3Vec3 extents = b3Vec3{a, 0.5f * a, a};
        b3SurfaceMaterial material = b3DefaultSurfaceMaterial();

        b3MeshData* box = b3CreateBoxMesh(b3Vec3{0.0f, 0.0f, 0.0f}, extents, true);
        b3CompoundMeshDef[boxCount] meshes;
        b3Transform transform = b3Transform_identity;
        transform.p.y = -0.5f * a;

        i32 index = 0;
        for i32 i = 0; i < gridCount; i += 1 {
            transform.p.x = (2.0f * cast(f32, i) - cast(f32, gridCount)) * a;

            for i32 j = 0; j < gridCount; j += 1 {
                transform.p.z = (2.0f * cast(f32, j) - cast(f32, gridCount)) * a;

                transform.p.y = random_float_range(-0.5f, 0.25f) * a;

                meshes[index].meshData = box;
                meshes[index].transform = transform;
                meshes[index].scale = b3Vec3{1.0f, 1.0f, 1.0f};
                meshes[index].materials = &material;
                meshes[index].materialCount = 1;

                index += 1;
            }
        }

        b3CompoundDef def = b3CompoundDef{};
        def.meshes = cast(b3CompoundMeshDef*, &meshes);
        def.meshCount = boxCount;

        g_mt_compound = b3CreateCompound(&def);
        b3DestroyMesh(box);
    }

    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        b3BodyId groundId = b3CreateBody(g_world, &bodyDef);

        b3ShapeDef shapeDef = b3DefaultShapeDef();
        ignore b3CreateBakedCompoundShape(groundId, &shapeDef, g_mt_compound);
    }
}

void destroy_mesh_tile() {
    b3DestroyCompound(g_mt_compound);
}

void step_mesh_tile(f32 timeStep) {
    ignore timeStep;
    b3Transform transform = b3Transform{b3Vec3{0.0f, 0.01f, 0.0f}, b3Quat_identity};
    dbg_axes(b3MakeWorldTransform(transform), 1.0f);

    u8[160] buf;
    ignore snprintf(cast(u8*, &buf), 160, "compound instance count = %d, byte count = %d",
                    g_mt_compound.meshCount, g_mt_compound.byteCount);
    draw_text_line(cast(u8*, &buf));

    i32 treeBytes = b3DynamicTree_GetByteCount(&g_mt_compound.tree);
    i32 height = b3DynamicTree_GetHeight(&g_mt_compound.tree);
    ignore snprintf(cast(u8*, &buf), 160, "compound tree byte count = %d, height = %d",
                    treeBytes, height);
    draw_text_line(cast(u8*, &buf));
}

// samples/sample_compound.cpp Village. The controller is upstream's
// samples/mover.cpp, transpiled to lib/box3d_mover.mc.
const i32 VILLAGE_GRID_COUNT = 200;
const f32 VILLAGE_A = 4.0f;
const i32 VILLAGE_MATERIAL_CAPACITY = 5;

bool village_overlap_result_fcn(b3ShapeId shapeId, void* context) {
    ignore shapeId;
    bool* overlap = cast(bool*, context);
    *overlap = true;
    return false;
}

b3CompoundData* g_vl_compound;
CharacterMover g_vl_mover;
f32 g_vl_world_width;
b3Pos g_vl_ray_origin;

void build_village() {
    i32 gridCount = VILLAGE_GRID_COUNT;
    f32 a = VILLAGE_A;
    g_vl_world_width = 2.0f * cast(f32, gridCount) * a;

    b3Pos position = b3Pos{0.0f, 10.0f, 0.0f};

    g_launch_speed_scale = 2.0f;
    CharacterMover_Initialize(&g_vl_mover, mover_sample(), position);

    {
        i32 capsuleCapacity = gridCount * gridCount / 8 + 1;
        i32 hullCount = gridCount * gridCount;
        i32 sphereCapacity = gridCount * gridCount / 8 + 1;

        b3Vec3 extents = b3Vec3{a, 0.5f * a, a};
        b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
        b3BoxHull box = b3MakeBoxHull(extents.x, extents.y, extents.z);

        // Must allocate to avoid stack overflow
        b3CompoundCapsuleDef* capsules = alloc<b3CompoundCapsuleDef>(capsuleCapacity);
        b3CompoundHullDef* hulls = alloc<b3CompoundHullDef>(hullCount);
        b3CompoundSphereDef* spheres = alloc<b3CompoundSphereDef>(sphereCapacity);

        b3Transform transform = b3Transform_identity;

        i32 capsuleIndex = 0;
        i32 hullIndex = 0;
        i32 sphereIndex = 0;

        for i32 i = 0; i < gridCount; ++i {
            transform.p.x = (2.0f * cast(f32, i) - cast(f32, gridCount)) * a;

            for i32 j = 0; j < gridCount; ++j {
                transform.p.z = (2.0f * cast(f32, j) - cast(f32, gridCount)) * a;
                transform.p.y = random_float_range(-0.25f, 0.125f) * a;

                if (i & 1) != 0 && (j & 1) != 0 {
                    b3Vec3 p1 = b3Add(transform.p, random_vec3(b3Vec3{-a, a, -a},
                                                               b3Vec3{a, 2.0f * a, a}));
                    b3Vec3 p2 = b3Add(transform.p, random_vec3(b3Vec3{-a, a, -a},
                                                               b3Vec3{a, 2.0f * a, a}));
                    f32 radius = random_float_range(0.1f, 0.5f);

                    if capsuleIndex < sphereIndex {
                        capsules[capsuleIndex].capsule = b3Capsule{p1, p2, radius};
                        capsules[capsuleIndex].material = material;
                        capsuleIndex += 1;
                    } else {
                        spheres[sphereIndex].sphere = b3Sphere{p1, radius};
                        spheres[sphereIndex].material = material;
                        sphereIndex += 1;
                    }
                }

                hulls[hullIndex].hull = &box.base;
                hulls[hullIndex].transform = transform;
                hulls[hullIndex].material = material;

                hullIndex += 1;
            }
        }

        i32 meshGridCount = gridCount / 4;
        i32 meshCount = meshGridCount * meshGridCount;
        f32 b = 4.0f * a;

        b3SurfaceMaterial[VILLAGE_MATERIAL_CAPACITY] meshMaterials;
        b3MeshData* buildingMesh = create_mesh_data("data/meshes/building.obj", 1.0f,
                                                    false, false, true, true);

        i32 materialCount = buildingMesh.materialCount;

        for i32 i = 0; i < materialCount; ++i {
            meshMaterials[i] = b3DefaultSurfaceMaterial();
            if i == 0 {
                meshMaterials[i].friction = 0.0f;
            } else if i == 1 {
                meshMaterials[i].restitution = 0.5f;
            }
            meshMaterials[i].userMaterialId = cast(u64, i + 42);
        }

        b3CompoundMeshDef* meshes = alloc<b3CompoundMeshDef>(meshCount);
        transform = b3Transform_identity;

        i32 meshIndex = 0;
        for i32 i = 0; i < meshGridCount; ++i {
            transform.p.x = (2.0f * cast(f32, i) - cast(f32, meshGridCount)) * b + 0.5f * b;

            for i32 j = 0; j < meshGridCount; ++j {
                transform.p.y = 0.5f * a;
                transform.p.z = (2.0f * cast(f32, j) - cast(f32, meshGridCount)) * b + 0.5f * b;
                transform.q = b3MakeQuatFromAxisAngle(b3Vec3{0.0f, 1.0f, 0.0f},
                                                      random_float_range(-PI_F, PI_F));

                meshes[meshIndex].meshData = buildingMesh;
                meshes[meshIndex].transform = transform;
                meshes[meshIndex].scale = random_vec3(b3Vec3{0.5f, 0.5f, 0.5f},
                                                      b3Vec3{2.0f, 2.0f, 2.0f});

                if (meshIndex & 1) != 0 {
                    meshes[meshIndex].scale.x = -meshes[meshIndex].scale.x;
                }

                if (meshIndex & 3) != 0 {
                    meshes[meshIndex].scale.z = -meshes[meshIndex].scale.z;
                }

                meshes[meshIndex].materials = cast(b3SurfaceMaterial*, &meshMaterials);
                meshes[meshIndex].materialCount = materialCount;

                meshIndex += 1;
            }
        }

        b3CompoundDef def = b3CompoundDef{};
        def.capsules = capsules;
        def.capsuleCount = capsuleIndex;
        def.hullCount = hullCount;
        def.hulls = hulls;
        def.hullCount = hullCount;
        def.meshes = meshes;
        def.meshCount = meshCount;
        def.spheres = spheres;
        def.sphereCount = sphereIndex;

        g_vl_compound = b3CreateCompound(&def);

        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{-1.0f, -0.5f, 2.0f};
        bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3{0.0f, 1.0f, 0.0f}, -1.15f * PI_F);
        b3BodyId groundId = b3CreateBody(g_world, &bodyDef);

        b3ShapeDef shapeDef = b3DefaultShapeDef();
        ignore b3CreateBakedCompoundShape(groundId, &shapeDef, g_vl_compound);

        free(capsules);
        free(hulls);
        free(meshes);
        free(spheres);

        b3DestroyMesh(buildingMesh);
    }

    g_vl_ray_origin = b3Pos{-0.45f * g_vl_world_width, 20.0f, -0.45f * g_vl_world_width};
}

void destroy_village() {
    b3DestroyCompound(g_vl_compound);
}

void village_keyboard(i32 key, i32 action, i32 mods) {
    ignore mods;
    if key == SAPP_KEYCODE_T && action == 1 {
        toggle_third_person();
    }
}

void step_village(f32 timeStepIn) {
    ignore timeStepIn;
    mover_sync();
    CharacterMover_Step(&g_vl_mover, cast(b3ShapeId*, null), 0, true);

    u8[160] buf;
    ignore snprintf(cast(u8*, &buf), 160, "third person (T) = %d",
                    cam_third_person ? 1 : 0);
    draw_text_line(cast(u8*, &buf));

    b3Vec3 translation = b3Vec3{10.0f, -40.0f, -5.0f};
    b3QueryFilter filter = b3DefaultQueryFilter();

    {
        CastContext context = CastContext{};
        ignore b3World_CastRay(g_world, g_vl_ray_origin, translation, filter,
                               ray_cast_closest_callback, cast(void*, &context));

        dbg_line(g_vl_ray_origin, b3OffsetPos(g_vl_ray_origin, translation),
                 b3_colorAliceBlue);
        if context.count > 0 {
            b3Pos p1 = context.points[0];
            b3Pos p2 = b3OffsetPos(p1, b3MulSV(0.5f, context.normals[0]));
            dbg_line(p1, p2, b3_colorYellow);
            adapter_point(p1, 8.0f, b3_colorLightCoral, null);
            ignore snprintf(cast(u8*, &buf), 160,
                            "ray hit triangle/child/material = %d / %d / %d",
                            context.triangleIndices[0], context.childIndices[0],
                            cast(i32, context.materialIds[0]));
            draw_text_line(cast(u8*, &buf));
        } else {
            draw_text_line("ray miss");
        }
    }

    {
        CastContext context = CastContext{};
        b3Pos origin = b3OffsetPos(g_vl_ray_origin, b3Vec3{-1.0f, 0.0f, -1.0f});
        b3Vec3 zero = b3Vec3_zero;
        b3ShapeProxy proxy = b3ShapeProxy{&zero, 1, 0.25f};
        b3World_CastShape(g_world, origin, &proxy, translation, filter,
                          ray_cast_closest_callback, cast(void*, &context));

        dbg_line(origin, b3OffsetPos(origin, translation), b3_colorAliceBlue);
        if context.count > 0 {
            b3Pos position = b3OffsetPos(origin, b3MulSV(context.fractions[0], translation));
            b3Pos p1 = context.points[0];
            b3Pos p2 = b3OffsetPos(p1, b3MulSV(0.5f, context.normals[0]));
            dbg_line(p1, p2, b3_colorYellow);
            adapter_point(p1, 8.0f, b3_colorLightCoral, null);
            b3Sphere sphere = b3Sphere{b3Vec3_zero, 0.25f};
            b3WorldTransform xf;
            xf.p = position;
            xf.q = b3Quat_identity;
            dbg_solid_sphere(xf, sphere, make_color(b3_colorOrchid));
            ignore snprintf(cast(u8*, &buf), 160,
                            "shape hit triangle/child/material = %d / %d / %d",
                            context.triangleIndices[0], context.childIndices[0],
                            cast(i32, context.materialIds[0]));
            draw_text_line(cast(u8*, &buf));
        } else {
            draw_text_line("shape miss");
        }
    }

    {
        bool overlap = false;
        b3Pos origin = b3Pos{g_vl_ray_origin.x - 1.0f, 2.0f, g_vl_ray_origin.z - 1.0f};
        b3Vec3 zero = b3Vec3_zero;
        b3ShapeProxy proxy = b3ShapeProxy{&zero, 1, 0.3f};
        b3World_OverlapShape(g_world, origin, &proxy, filter,
                             village_overlap_result_fcn, cast(void*, &overlap));

        b3HexColor color = overlap ? b3_colorDarkMagenta : b3_colorDarkSeaGreen;
        b3Sphere sphere = b3Sphere{b3Vec3_zero, 0.3f};
        b3WorldTransform xf;
        xf.p = origin;
        xf.q = b3Quat_identity;
        dbg_solid_sphere(xf, sphere, make_color(color));
    }

    if g_vl_ray_origin.x > 0.45f * g_vl_world_width {
        g_vl_ray_origin.x = -0.45f * g_vl_world_width;
        g_vl_ray_origin.z += 8.0f;
    }

    if g_vl_ray_origin.z > 0.45f * g_vl_world_width {
        g_vl_ray_origin.z = -0.45f * g_vl_world_width;
    }

    f32 timeStep = 0.0f;
    if g_pause == false || g_single_step > 0 {
        timeStep = g_hertz > 0.0f ? 1.0f / g_hertz : 0.0f;
    }

    g_vl_ray_origin.x += 2.0f * timeStep;

    b3Transform transform = b3Transform{b3Vec3{0.0f, 0.01f, 0.0f}, b3Quat_identity};
    dbg_axes(b3MakeWorldTransform(transform), 4.0f);

    ignore snprintf(cast(u8*, &buf), 160, "surface type = %d",
                    cast(i32, g_user_material_id));
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 160,
                    "compound capsules/hulls/meshes/sphere = %d / %d / %d / %d",
                    g_vl_compound.capsuleCount, g_vl_compound.hullCount,
                    g_vl_compound.meshCount, g_vl_compound.sphereCount);
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 160, "compound byte count = %d",
                    g_vl_compound.byteCount);
    draw_text_line(cast(u8*, &buf));

    i32 treeBytes = b3DynamicTree_GetByteCount(&g_vl_compound.tree);
    i32 height = b3DynamicTree_GetHeight(&g_vl_compound.tree);
    ignore snprintf(cast(u8*, &buf), 160,
                    "compound tree byte count = %d, height = %d", treeBytes, height);
    draw_text_line(cast(u8*, &buf));

    i32 total = 0;
    i32 drawn = compound_draw_stats(&total);
    ignore snprintf(cast(u8*, &buf), 160, "compound children drawn = %d / %d",
                    drawn, total);
    draw_text_line(cast(u8*, &buf));
}
