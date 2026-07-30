// Collision and determinism scenes. Ports of samples/sample_collision.cpp
// and samples/sample_determinism.cpp.

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
import sample_events;
import sample_geometry;
import sample_mesh;

// samples/sample_collision.cpp CapsuleCastRay
b3BodyId g_ccr_body;
b3Vec3 g_ccr_offset;
f32 g_ccr_scale;

void build_capsule_cast_ray() {
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_kinematicBody;
    g_ccr_body = b3CreateBody(g_world, &bodyDef);

    b3ShapeDef shapeDef = b3DefaultShapeDef();

    g_ccr_scale = 1.0f;

    // b3Capsule Capsule = {
    //	{ 1.88094640f, 1.13457823f, 30.2966614f }, { -1.37514091f, -2.26233172f, 848.671997f }, 51.7184296f
    // };

    b3Capsule capsule = b3Capsule{
        b3Pos{0.0f, 0.0f, 0.0f},
        b3Pos{0.0f, 1.0f, 0.0f},
        0.5f,
    };

    g_ccr_offset = b3Vec3{capsule.center1.x, capsule.center1.y, capsule.center1.z};

    ignore b3CreateCapsuleShape(g_ccr_body, &shapeDef, &capsule);
}

void step_capsule_cast_ray(f32 timeStep) {
    ignore timeStep;
    dbg_ground_grid(10);
    dbg_line(b3Pos_zero, b3OffsetPos(b3Pos_zero, b3MulSV(0.4f, b3Vec3_axisX)), b3_colorRed);
    dbg_line(b3Pos_zero, b3OffsetPos(b3Pos_zero, b3MulSV(0.4f, b3Vec3_axisY)), b3_colorGreen);
    dbg_line(b3Pos_zero, b3OffsetPos(b3Pos_zero, b3MulSV(0.4f, b3Vec3_axisZ)), b3_colorBlue);

    b3Pos origin = b3Pos{-1.0f, 0.5f, 0.0f};
    b3Vec3 translation = b3Vec3{2.0f, 0.0f, 0.0f};
    b3QueryFilter filter = b3DefaultQueryFilter();
    f32 maxFraction = 1.0f;
    b3WorldTransform bodyTransform = b3WorldTransform_identity;

    b3BodyCastResult result = b3Body_CastRay(g_ccr_body, origin, translation, filter,
                                             maxFraction, bodyTransform);

    b3Pos rayEnd = b3OffsetPos(origin, translation);
    dbg_line(origin, rayEnd, b3_colorGray);
    adapter_point(origin, 4.0f, b3_colorGreen, null);
    adapter_point(rayEnd, 4.0f, b3_colorRed, null);

    if result.hit {
        adapter_point(result.point, 4.0f, b3_colorOrange, null);
    }
}

// shared/determinism.c query driven spawning
//
// Query driven spawning in an empty zero gravity world. Each step casts a ray, overlaps an
// AABB, and casts a sphere, then spawns a shape whose position, type, and size depend on the
// query results. Any query divergence cascades into the final state.
const i32 QUERY_SPAWN_COUNT = 50;
const f32 QUERY_SPAWN_CAST_RADIUS = 0.5f;
const u32 B3_HASH_INIT = 5381;

b3BodyId[QUERY_SPAWN_COUNT] g_qs_bodies;
i32 g_qs_spawn_count;
i32 g_qs_query_hit_count;
u32 g_qs_query_hash;
i32 g_qs_step_count;
i32 g_qs_sleep_step;
u32 g_qs_hash;

// Last query cycle, recorded for visualization
b3Pos g_qs_ray_origin;
b3Vec3 g_qs_ray_translation;
b3Pos g_qs_ray_point;
b3Vec3 g_qs_ray_normal;
bool g_qs_ray_did_hit;
b3AABB g_qs_overlap_bounds;
i32 g_qs_overlap_count;
f32 g_qs_cast_fraction;
b3Pos g_qs_last_spawn_position;

i32 g_qs_frame_count;
bool g_qs_done;

bool query_spawn_overlap_callback(b3ShapeId shapeId, void* context) {
    ignore context;
    g_qs_overlap_count += 1;
    g_qs_query_hit_count += 1;
    g_qs_query_hash = b3Hash(g_qs_query_hash, cast(u8*, &shapeId.index1), 4);
    return true;
}

f32 query_spawn_cast_callback(b3ShapeId shapeId, b3Pos point, b3Vec3 normal, f32 fraction,
                              u64 userMaterialId, i32 triangleIndex, i32 childIndex,
                              void* context) {
    ignore shapeId; ignore point; ignore normal;
    ignore userMaterialId; ignore triangleIndex; ignore childIndex;
    f32* closest = cast(f32*, context);
    *closest = fraction;
    return fraction;
}

void query_spawn_once() {
    b3QueryFilter filter = b3DefaultQueryFilter();

    // Ray result decides the spawn position, a miss seeds the cloud near the origin.
    b3Pos rayOrigin = random_pos(b3Vec3{-12.0f, -12.0f, -12.0f}, b3Vec3{12.0f, 12.0f, 12.0f});
    b3Vec3 rayTranslation = b3MulSV(30.0f, random_unit_vector());

    b3Pos spawnPosition;
    b3RayResult ray = b3World_CastRayClosest(g_world, rayOrigin, rayTranslation, filter);
    if ray.hit {
        g_qs_query_hit_count += 1;
        g_qs_query_hash = b3Hash(g_qs_query_hash, cast(u8*, &ray.fraction), 4);
        g_qs_query_hash = b3Hash(g_qs_query_hash, cast(u8*, &ray.normal), 12);
        spawnPosition = b3OffsetPos(ray.point, b3MulSV(1.2f, ray.normal));
    } else {
        spawnPosition = random_pos(b3Vec3{-6.0f, -6.0f, -6.0f}, b3Vec3{6.0f, 6.0f, 6.0f});
    }

    g_qs_ray_origin = rayOrigin;
    g_qs_ray_translation = rayTranslation;
    g_qs_ray_did_hit = ray.hit;
    g_qs_ray_point = ray.hit ? ray.point : b3OffsetPos(rayOrigin, rayTranslation);
    g_qs_ray_normal = ray.hit ? ray.normal : b3Vec3_zero;

    // Overlap count picks the shape type.
    b3Vec3 center = random_vec3_uniform(-10.0f, 10.0f);
    f32 extent = random_float_range(1.0f, 4.0f);
    b3AABB aabb;
    aabb.lowerBound = b3Vec3{center.x - extent, center.y - extent, center.z - extent};
    aabb.upperBound = b3Vec3{center.x + extent, center.y + extent, center.z + extent};

    g_qs_overlap_count = 0;
    ignore b3World_OverlapAABB(g_world, aabb, filter, query_spawn_overlap_callback, null);

    g_qs_overlap_bounds = aabb;

    // Sphere cast fraction sets the spawn size.
    f32 fraction = 1.0f;
    b3Vec3 proxyPoint = b3Vec3_zero;
    b3ShapeProxy proxy = b3ShapeProxy{&proxyPoint, 1, QUERY_SPAWN_CAST_RADIUS};
    ignore b3World_CastShape(g_world, rayOrigin, &proxy, rayTranslation, filter,
                             query_spawn_cast_callback, cast(void*, &fraction));
    if fraction < 1.0f {
        g_qs_query_hit_count += 1;
        g_qs_query_hash = b3Hash(g_qs_query_hash, cast(u8*, &fraction), 4);
    }

    g_qs_cast_fraction = fraction;

    f32 size = 0.3f + 0.2f * fraction;

    // Damping guarantees everything comes to rest.
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = spawnPosition;
    bodyDef.rotation = random_quat();
    bodyDef.linearVelocity = random_vec3_uniform(-0.2f, 0.2f);
    bodyDef.angularVelocity = random_vec3_uniform(-0.5f, 0.5f);
    bodyDef.linearDamping = 1.0f;
    bodyDef.angularDamping = 1.0f;

    b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.baseMaterial.rollingResistance = 0.2f;

    i32 kind = (g_qs_spawn_count + g_qs_overlap_count) % 3;
    if kind == 0 {
        b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, size};
        ignore b3CreateSphereShape(bodyId, &shapeDef, &sphere);
    } else if kind == 1 {
        b3Capsule capsule = b3Capsule{b3Pos{0.0f, -size, 0.0f}, b3Pos{0.0f, size, 0.0f}, 0.7f * size};
        ignore b3CreateCapsuleShape(bodyId, &shapeDef, &capsule);
    } else {
        b3BoxHull box = b3MakeBoxHull(size, 0.7f * size, 0.5f * size);
        ignore b3CreateHullShape(bodyId, &shapeDef, &box.base);
    }

    g_qs_bodies[g_qs_spawn_count] = bodyId;
    g_qs_spawn_count += 1;
    g_qs_last_spawn_position = spawnPosition;
}

void create_query_spawn() {
    g_qs_spawn_count = 0;
    g_qs_query_hit_count = 0;
    g_qs_query_hash = 0;
    g_qs_step_count = 0;
    g_qs_sleep_step = 0;
    g_qs_hash = 0;
    g_qs_overlap_count = 0;
    g_qs_cast_fraction = 0.0f;
    g_qs_ray_did_hit = false;

    g_randomSeed = 71689;

    // Empty space, motion comes only from spawn velocities and overlap pushes.
    b3World_SetGravity(g_world, b3Vec3_zero);
}

bool update_query_spawn() {
    if g_qs_spawn_count < QUERY_SPAWN_COUNT {
        query_spawn_once();
    } else if g_qs_hash == 0 && b3World_GetAwakeBodyCount(g_world) == 0 {
        g_qs_hash = B3_HASH_INIT;
        for i32 i = 0; i < QUERY_SPAWN_COUNT; i += 1 {
            b3WorldTransform xf = b3Body_GetTransform(g_qs_bodies[i]);
            g_qs_hash = b3Hash(g_qs_hash, cast(u8*, &xf), cast(i32, sizeof(b3WorldTransform)));
        }

        g_qs_sleep_step = g_qs_step_count;
    }

    g_qs_step_count += 1;

    return g_qs_hash != 0;
}

// samples/sample_determinism.cpp QuerySpawn
void build_query_spawn() {
    create_query_spawn();
    g_qs_frame_count = 0;
    g_qs_done = false;
}

void step_query_spawn(f32 timeStep) {
    u8[192] buf;

    if g_qs_done == false {
        // Advance every 10th step so each query lingers on screen. This diverges from
        // the unit test cadence, so the hashes differ from the pinned constants.
        if timeStep > 0.0f {
            g_qs_frame_count += 1;
            if g_qs_frame_count % 10 == 1 {
                g_qs_done = update_query_spawn();
            }
        }

        ignore snprintf(cast(u8*, &buf), 192, "spawned = %d of %d, query hits = %d",
                        g_qs_spawn_count, QUERY_SPAWN_COUNT, g_qs_query_hit_count);
        draw_text_line(cast(u8*, &buf));
        draw_text_line("ray = yellow, overlap box = magenta, sphere cast = cyan, spawn = white");
    } else {
        ignore snprintf(cast(u8*, &buf), 192, "sleep step = %d, hash = 0x%08X",
                        g_qs_sleep_step, g_qs_hash);
        draw_text_line(cast(u8*, &buf));
        ignore snprintf(cast(u8*, &buf), 192, "query hits = %d, query hash = 0x%08X",
                        g_qs_query_hit_count, g_qs_query_hash);
        draw_text_line(cast(u8*, &buf));
    }

    if g_qs_spawn_count == 0 { return; }

    // Ray with hit point and normal, red end point on a miss
    dbg_line(g_qs_ray_origin, g_qs_ray_point, b3_colorYellow);
    adapter_point(g_qs_ray_origin, 5.0f, b3_colorYellow, null);
    if g_qs_ray_did_hit {
        adapter_point(g_qs_ray_point, 8.0f, b3_colorGreen, null);
        dbg_line(g_qs_ray_point, b3OffsetPos(g_qs_ray_point, g_qs_ray_normal), b3_colorGreen);
    } else {
        adapter_point(g_qs_ray_point, 5.0f, b3_colorRed, null);
    }

    adapter_bounds(g_qs_overlap_bounds, b3_colorMagenta, null);

    // Swept sphere stopped at its cast fraction, gray when it reached full length
    b3Pos castCenter = b3OffsetPos(g_qs_ray_origin,
                                   b3MulSV(g_qs_cast_fraction, g_qs_ray_translation));
    b3Sphere castSphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, QUERY_SPAWN_CAST_RADIUS};
    b3HexColor castColor = g_qs_cast_fraction < 1.0f ? b3_colorCyan : b3_colorGray;
    dbg_wire_sphere(b3Transform{b3Vec3{castCenter.x, castCenter.y, castCenter.z}, b3Quat_identity},
                    &castSphere, 24, castColor);

    dbg_cross(g_qs_last_spawn_position, 0.5f, b3_colorWhite);
}

// shared/determinism.c CreateWavePile
const i32 WAVE_PILE_BODY_COUNT = 100;
const i32 WAVE_PILE_GRID = 5;
const i32 WAVE_PILE_LAYERS = 4;

b3BodyId[WAVE_PILE_BODY_COUNT] g_wp_bodies;
b3HeightFieldData* g_wp_height_field;
i32 g_wp_step_count;
i32 g_wp_sleep_step;
u32 g_wp_hash;
bool g_wp_done;

void create_wave_pile() {
    g_wp_step_count = 0;
    g_wp_sleep_step = 0;
    g_wp_hash = 0;

    g_randomSeed = 52977;

    // Height fields grow from a corner, offset the body to center the patch.
    i32 fieldCount = 21;
    b3Vec3 fieldScale = b3Vec3{1.0f, 0.6f, 1.0f};
    g_wp_height_field = b3CreateWave(fieldCount, fieldCount, fieldScale, 0.08f, 0.06f, false);

    {
        f32 extent = fieldScale.x * cast(f32, fieldCount - 1);
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position.x = -0.5f * extent;
        bodyDef.position.z = -0.5f * extent;
        b3BodyId groundId = b3CreateBody(g_world, &bodyDef);

        b3ShapeDef shapeDef = b3DefaultShapeDef();
        ignore b3CreateHeightFieldShape(groundId, &shapeDef, g_wp_height_field);
    }

    b3HullData* rock = b3CreateRock(0.55f);
    b3BoxHull box = b3MakeBoxHull(0.45f, 0.3f, 0.55f);
    b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, 0.5f};
    b3Capsule capsule = b3Capsule{b3Pos{0.0f, -0.3f, 0.0f}, b3Pos{0.0f, 0.3f, 0.0f}, 0.35f};

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;

    // Rolling resistance so the pile sleeps quickly.
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.baseMaterial.rollingResistance = 0.3f;

    f32 spacing = 1.7f;
    i32 index = 0;
    for i32 layer = 0; layer < WAVE_PILE_LAYERS; layer += 1 {
        for i32 i = 0; i < WAVE_PILE_GRID; i += 1 {
            for i32 j = 0; j < WAVE_PILE_GRID; j += 1 {
                b3Vec3 jitter = random_vec3_uniform(-0.3f, 0.3f);
                bodyDef.position.x = spacing * (cast(f32, i) - 0.5f * cast(f32, WAVE_PILE_GRID - 1)) + jitter.x;
                bodyDef.position.y = 2.5f + 1.6f * cast(f32, layer) + 0.3f * jitter.y;
                bodyDef.position.z = spacing * (cast(f32, j) - 0.5f * cast(f32, WAVE_PILE_GRID - 1)) + jitter.z;
                bodyDef.rotation = random_quat();

                b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
                g_wp_bodies[index] = bodyId;

                i32 kind = index % 4;
                if kind == 0 {
                    ignore b3CreateSphereShape(bodyId, &shapeDef, &sphere);
                } else if kind == 1 {
                    ignore b3CreateCapsuleShape(bodyId, &shapeDef, &capsule);
                } else if kind == 2 {
                    ignore b3CreateHullShape(bodyId, &shapeDef, &box.base);
                } else {
                    ignore b3CreateHullShape(bodyId, &shapeDef, rock);
                }

                index += 1;
            }
        }
    }

    // The world keeps its own copy.
    b3DestroyHull(rock);
}

bool update_wave_pile() {
    if g_wp_hash == 0 {
        if b3World_GetAwakeBodyCount(g_world) == 0 {
            g_wp_hash = B3_HASH_INIT;
            for i32 i = 0; i < WAVE_PILE_BODY_COUNT; i += 1 {
                b3WorldTransform xf = b3Body_GetTransform(g_wp_bodies[i]);
                g_wp_hash = b3Hash(g_wp_hash, cast(u8*, &xf), cast(i32, sizeof(b3WorldTransform)));
            }
            g_wp_sleep_step = g_wp_step_count;
        }
    }

    g_wp_step_count += 1;
    return g_wp_hash != 0;
}

// samples/sample_determinism.cpp WavePile
void build_wave_pile() {
    create_wave_pile();
    g_wp_done = false;
}

void destroy_wave_pile() {
    b3DestroyHeightField(g_wp_height_field);
}

void step_wave_pile(f32 timeStep) {
    if g_wp_done == false {
        if timeStep > 0.0f {
            g_wp_done = update_wave_pile();
        }
    } else {
        u8[128] buf;
        ignore snprintf(cast(u8*, &buf), 128, "sleep step = %d, hash = 0x%08X",
                        g_wp_sleep_step, g_wp_hash);
        draw_text_line(cast(u8*, &buf));
    }
}

// shared/stability.c CreateMeshDrop — thin fast boxes dropped on a wave
// mesh. Stresses continuous collision and mesh contact stability, and
// doubles as a determinism scenario via the sleep hash. Shared by
// Determinism/Mesh Drop and World/Far Mesh Drop.
const i32 MESH_DROP_GRID_COUNT = 20;

b3MeshData* g_md_mesh;
b3BodyId[MESH_DROP_GRID_COUNT * MESH_DROP_GRID_COUNT] g_md_bodies;
i32 g_md_step_count;
i32 g_md_sleep_step;
u32 g_md_hash;

void create_mesh_drop(b3Pos origin) {
    g_md_step_count = 0;
    g_md_sleep_step = 0;
    g_md_hash = 0;

    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = origin;
        b3BodyId groundId = b3CreateBody(g_world, &bodyDef);

        i32 gridCount = 40;
        f32 cellWidth = 1.0f;
        f32 rowHz = 0.1f;
        f32 columnHz = 0.2f;
        f32 groundAmplitude = 0.5f;

        g_md_mesh = b3CreateWaveMesh(gridCount, gridCount, cellWidth, groundAmplitude,
                                     rowHz, columnHz);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        shapeDef.filter.categoryBits = 1;
        ignore b3CreateMeshShape(groundId, &shapeDef, g_md_mesh, b3Vec3_one);
    }

    {
        b3BoxHull box = b3MakeBoxHull(0.02f, 0.2f, 0.04f);

        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;

        b3ShapeDef shapeDef = b3DefaultShapeDef();
        shapeDef.baseMaterial.rollingResistance = 0.1f;

        // Don't allow shapes to collide with each other.
        shapeDef.filter.categoryBits = 2;
        shapeDef.filter.maskBits = 1;

        g_randomSeed = 3963634789;

        i32 gridCount = MESH_DROP_GRID_COUNT;

        for i32 i = 0; i < gridCount; i += 1 {
            for i32 j = 0; j < gridCount; j += 1 {
                b3Vec3 linearVelocity = random_vec3_uniform(-1.0f, 1.0f);
                b3Vec3 angularVelocity = random_vec3_uniform(-5.0f, 5.0f);

                bodyDef.position = b3OffsetPos(origin,
                    b3Vec3{0.5f * (cast(f32, i) - 0.5f * cast(f32, gridCount)), 5.0f,
                           0.5f * (cast(f32, j) - 0.5f * cast(f32, gridCount))});
                bodyDef.linearVelocity = linearVelocity;
                bodyDef.angularVelocity = angularVelocity;
                b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
                g_md_bodies[i * gridCount + j] = bodyId;

                ignore b3CreateHullShape(bodyId, &shapeDef, &box.base);
            }
        }
    }
}

bool update_mesh_drop() {
    if g_md_hash == 0 {
        if b3World_GetAwakeBodyCount(g_world) == 0 {
            g_md_hash = B3_HASH_INIT;
            i32 bodyCount = MESH_DROP_GRID_COUNT * MESH_DROP_GRID_COUNT;
            for i32 i = 0; i < bodyCount; i += 1 {
                b3WorldTransform xf = b3Body_GetTransform(g_md_bodies[i]);
                g_md_hash = b3Hash(g_md_hash, cast(u8*, &xf), cast(i32, sizeof(b3WorldTransform)));
            }
            g_md_sleep_step = g_md_step_count;
        }
    }

    g_md_step_count += 1;
    return g_md_hash != 0;
}

void destroy_mesh_drop() {
    b3DestroyMesh(g_md_mesh);
}

// samples/sample_determinism.cpp MeshDropDeterminism
bool g_mdd_done;

void build_mesh_drop_determinism() {
    create_mesh_drop(b3Pos_zero);
    g_mdd_done = false;
}

void step_mesh_drop_determinism(f32 timeStep) {
    if g_mdd_done == false {
        if timeStep > 0.0f {
            g_mdd_done = update_mesh_drop();
        }
    } else {
        u8[128] buf;
        ignore snprintf(cast(u8*, &buf), 128, "sleep step = %d, hash = 0x%08X",
                        g_md_sleep_step, g_md_hash);
        draw_text_line(cast(u8*, &buf));
    }
}

// samples/sample_world.cpp FarMeshDrop
const f32 FMD_OFFSET_KM = 1000.0f;
bool g_fmd_failed;

void build_far_mesh_drop() {
    b3Pos base = b3Pos{1000.0f * FMD_OFFSET_KM, 0.0f, 1000.0f * FMD_OFFSET_KM};
    create_mesh_drop(base);
    g_fmd_failed = false;
    g_dbg_force_scale = 0.1f;
}

void step_far_mesh_drop(f32 timeStep) {
    ignore timeStep;
    if g_fmd_failed == false && g_step_count >= 300 {
        b3BodyEvents bodyEvents = b3World_GetBodyEvents(g_world);
        if bodyEvents.moveCount > 0 {
            g_fmd_failed = true;
        }
    }

    u8[160] buf;
    ignore snprintf(cast(u8*, &buf), 160, "double precision: %s",
                    b3IsDoublePrecision() ? "ON" : "OFF");
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 160,
                    "mesh drop running %.0f km from the world origin", FMD_OFFSET_KM);
    draw_text_line(cast(u8*, &buf));
    if g_fmd_failed {
        draw_text_line("failed!");
    }
}

// samples/sample_collision.cpp RayCurtain
b3MeshData* g_rc_mesh;
f32 g_rc_offset;
f32 g_rc_abs_speed;
f32 g_rc_speed;

void build_ray_curtain() {
    b3BoxHull box = b3MakeBoxHull(0.6f, 0.6f, 0.6f);

    g_rc_mesh = b3CreateTorusMesh(10, 12, 0.65f, 0.35f);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_kinematicBody;

    b3ShapeDef shapeDef = b3DefaultShapeDef();

    bodyDef.position = b3Pos{-6.0f, 3.0f, 0.0f};
    bodyDef.angularVelocity = b3Vec3{0.8f, 0.4f, 0.8f};
    b3BodyId sphereBody = b3CreateBody(g_world, &bodyDef);
    b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, 0.9f};
    ignore b3CreateSphereShape(sphereBody, &shapeDef, &sphere);

    bodyDef.position = b3Pos{-2.0f, 3.0f, 0.0f};
    bodyDef.angularVelocity = b3Vec3{0.8f, 0.4f, 0.8f};
    b3BodyId capsuleBody = b3CreateBody(g_world, &bodyDef);
    b3Capsule capsule = b3Capsule{b3Pos{-0.5f, 0.0f, 0.0f}, b3Pos{0.5f, 0.0f, 0.0f}, 0.8f};
    ignore b3CreateCapsuleShape(capsuleBody, &shapeDef, &capsule);

    bodyDef.position = b3Pos{2.0f, 3.0f, 0.0f};
    bodyDef.angularVelocity = b3Vec3{0.8f, 0.4f, 0.8f};
    b3BodyId hullBody = b3CreateBody(g_world, &bodyDef);
    ignore b3CreateHullShape(hullBody, &shapeDef, &box.base);

    bodyDef.position = b3Pos{6.0f, 3.0f, 0.0f};
    bodyDef.angularVelocity = b3Vec3{0.8f, 0.4f, 0.8f};
    b3BodyId meshBody = b3CreateBody(g_world, &bodyDef);
    ignore b3CreateMeshShape(meshBody, &shapeDef, g_rc_mesh, b3Vec3_one);

    g_rc_abs_speed = 0.015f;
    g_rc_offset = 2.0f;
    g_rc_speed = 0.0f - g_rc_abs_speed;
}

void destroy_ray_curtain() {
    b3DestroyMesh(g_rc_mesh);
}

void step_ray_curtain(f32 timeStep) {
    ignore timeStep;

    dbg_ground_grid(10);
    dbg_line(b3Pos_zero, b3OffsetPos(b3Pos_zero, b3MulSV(0.4f, b3Vec3_axisX)), b3_colorRed);
    dbg_line(b3Pos_zero, b3OffsetPos(b3Pos_zero, b3MulSV(0.4f, b3Vec3_axisY)), b3_colorGreen);
    dbg_line(b3Pos_zero, b3OffsetPos(b3Pos_zero, b3MulSV(0.4f, b3Vec3_axisZ)), b3_colorBlue);

    f32 x = -8.0f;
    while x <= 8.0f {
        b3Pos rayOrigin = b3Pos{x, 8.0f, g_rc_offset};
        b3Pos rayEnd = b3Pos{x, 0.0f, g_rc_offset};
        b3Vec3 rayTranslation = b3SubPos(rayEnd, rayOrigin);

        b3RayResult result = b3World_CastRayClosest(g_world, rayOrigin, rayTranslation,
                                                    b3DefaultQueryFilter());
        if result.hit {
            dbg_line(result.point, b3OffsetPos(result.point, b3MulSV(0.5f, result.normal)),
                     b3_colorGreen);
        }

        adapter_point(rayOrigin, 4.0f, b3_colorGreen, null);
        adapter_point(b3OffsetPos(rayOrigin, rayTranslation), 4.0f, b3_colorRed, null);
        dbg_line(rayOrigin, b3OffsetPos(rayOrigin, rayTranslation), b3_colorYellow);

        x += 0.1f;
    }

    if g_rc_offset > 2.0f {
        g_rc_speed = 0.0f - g_rc_abs_speed;
    } else if g_rc_offset < -2.0f {
        g_rc_speed = g_rc_abs_speed;
    }

    if g_pause == false {
        g_rc_offset += g_rc_speed;
    }
}

// samples/sample_collision.cpp LongRayCast
const i32 LRC_SHAPE_COUNT = 5;
const i32 LRC_TRAIL_COUNT = 180;

b3HullData* g_lrc_hull;
b3MeshData* g_lrc_mesh;
b3HeightFieldData* g_lrc_height_field;
b3Pos[LRC_SHAPE_COUNT] g_lrc_targets;
b3Pos[LRC_SHAPE_COUNT * LRC_TRAIL_COUNT] g_lrc_trail;
i32[LRC_SHAPE_COUNT] g_lrc_trail_next;
i32[LRC_SHAPE_COUNT] g_lrc_trail_count;
f32[LRC_SHAPE_COUNT] g_lrc_fail_rate;
f32 g_lrc_ray_length_km;
f32 g_lrc_cone_angle;
f32 g_lrc_phase;

b3Pos g_lrc_cast_point;
b3Vec3 g_lrc_cast_normal;
f32 g_lrc_cast_fraction;
i32 g_lrc_cast_count;

// This callback finds the closest hit. This is the most common callback used in games.
f32 lrc_ray_cast_closest_callback(b3ShapeId shapeId, b3Pos point, b3Vec3 normal, f32 fraction,
                                  u64 materialId, i32 triangleIndex, i32 childIndex,
                                  void* context) {
    ignore materialId; ignore triangleIndex; ignore childIndex; ignore context;

    // Check for initial overlap
    if fraction == 0.0f {
        // By returning -1, we instruct the calling code to ignore this shape and
        // continue the ray-cast to the next shape.
        return -1.0f;
    }

    // Ignore a specific shape. Also ignore initial overlap.
    i64 ignoreFlag = cast(i64, b3Shape_GetUserData(shapeId));
    if ignoreFlag == 1 {
        // By returning -1, we instruct the calling code to ignore this shape and
        // continue the ray-cast to the next shape.
        return -1.0f;
    }

    g_lrc_cast_point = point;
    g_lrc_cast_normal = normal;
    g_lrc_cast_fraction = fraction;
    g_lrc_cast_count = 1;

    // By returning the current fraction, we instruct the calling code to clip the ray and
    // continue the ray-cast to the next shape. WARNING: do not assume that shapes
    // are reported in order.
    return fraction;
}

b3Pos g_lrc_hit_point;
b3Vec3 g_lrc_hit_normal;
bool g_lrc_hit;

// Cast a ray that passes through the aim point along a cone direction, starting at the given
// distance above it. Returns the closest hit.
void lrc_cast_along(b3Pos aim, b3Vec3 coneDir, f32 distance, f32 reach, b3QueryFilter filter) {
    b3Pos origin = b3OffsetPos(aim, b3MulSV(distance, coneDir));
    b3Vec3 translation = b3MulSV(0.0f - (distance + reach), coneDir);
    g_lrc_cast_count = 0;
    ignore b3World_CastRay(g_world, origin, translation, filter,
                           lrc_ray_cast_closest_callback, null);
    g_lrc_hit = g_lrc_cast_count > 0;
    if g_lrc_hit {
        g_lrc_hit_point = g_lrc_cast_point;
        g_lrc_hit_normal = g_lrc_cast_normal;
    }
}

void build_long_ray_cast() {
    b3World_SetGravity(g_world, b3Vec3_zero);
    g_lrc_hull = b3CreateRock(1.0f);
    g_lrc_mesh = b3CreateWaveMesh(8, 8, 0.5f, 0.25f, 0.2f, 0.2f);
    i32 hfCount = 9;
    b3Vec3 hfScale = b3MulSV(0.5f, b3Vec3_one);
    g_lrc_height_field = b3CreateWave(hfCount, hfCount, hfScale, 0.08f, 0.16f, false);

    // Aim each ray at a point above its shape so the cone sweeps the hit across the surface.
    f32 spacing = 5.0f;
    f32 aimHeight = 2.5f;
    for i32 i = 0; i < LRC_SHAPE_COUNT; i += 1 {
        f32 x = cast(f32, i - 2) * spacing;
        g_lrc_targets[i] = b3Pos{x, aimHeight, 0.0f};
        g_lrc_trail_next[i] = 0;
        g_lrc_trail_count[i] = 0;
        g_lrc_fail_rate[i] = 0.0f;
    }

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_staticBody;
    b3ShapeDef shapeDef = b3DefaultShapeDef();

    // Sphere
    bodyDef.position.x = g_lrc_targets[0].x;
    {
        b3BodyId body = b3CreateBody(g_world, &bodyDef);
        b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, 1.0f};
        ignore b3CreateSphereShape(body, &shapeDef, &sphere);
    }

    // Capsule along the x-axis
    bodyDef.position.x = g_lrc_targets[1].x;
    {
        b3BodyId body = b3CreateBody(g_world, &bodyDef);
        b3Capsule capsule = b3Capsule{b3Pos{-1.0f, 0.0f, 0.0f}, b3Pos{1.0f, 0.0f, 0.0f}, 0.7f};
        ignore b3CreateCapsuleShape(body, &shapeDef, &capsule);
    }

    // Hull
    bodyDef.position.x = g_lrc_targets[2].x;
    {
        b3BodyId body = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateHullShape(body, &shapeDef, g_lrc_hull);
    }

    // Wave mesh, centered on its origin and facing up
    bodyDef.position.x = g_lrc_targets[3].x;
    {
        b3BodyId body = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateMeshShape(body, &shapeDef, g_lrc_mesh, b3Vec3_one);
    }

    // Height field grows from a corner, so offset the body to center the patch under the ray
    {
        f32 extentX = hfScale.x * cast(f32, hfCount - 1);
        f32 extentZ = hfScale.z * cast(f32, hfCount - 1);
        bodyDef.position = b3Pos{g_lrc_targets[4].x - 0.5f * extentX, 0.0f, -0.5f * extentZ};
        b3BodyId body = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateHeightFieldShape(body, &shapeDef, g_lrc_height_field);
    }

    g_lrc_ray_length_km = 1.0f;
    g_lrc_cone_angle = 5.0f;
    g_lrc_phase = 0.0f;
    for i32 i = 0; i < LRC_SHAPE_COUNT * LRC_TRAIL_COUNT; i += 1 {
        g_lrc_trail[i] = b3Pos{0.0f, 0.0f, 0.0f};
    }
}

void destroy_long_ray_cast() {
    b3DestroyHull(g_lrc_hull);
    b3DestroyMesh(g_lrc_mesh);
    b3DestroyHeightField(g_lrc_height_field);
}

bool long_ray_cast_controls() {
    ignore ImGui_SliderFloat("Ray Length", &g_lrc_ray_length_km, 1.0f, 10000.0f, "%.0f km",
                             ImGuiSliderFlags_Logarithmic);
    ignore ImGui_SliderFloat("Cone Angle", &g_lrc_cone_angle, 0.0f, 12.0f, "%.1f deg", 0);
    return true;
}

void step_long_ray_cast(f32 timeStep) {
    ignore timeStep;

    // Advance the cone so it completes one loop per trail buffer.
    g_lrc_phase += 2.0f * PI_F / cast(f32, LRC_TRAIL_COUNT);
    if g_lrc_phase > 2.0f * PI_F {
        g_lrc_phase -= 2.0f * PI_F;
    }

    // Unit direction precessing on a small cone about the up axis.
    f32 halfAngle = g_lrc_cone_angle * (PI_F / 180.0f);
    b3Vec3 tilted = b3RotateVector(b3MakeQuatFromAxisAngle(b3Vec3_axisX, halfAngle), b3Vec3_axisY);
    b3Vec3 coneDir = b3RotateVector(b3MakeQuatFromAxisAngle(b3Vec3_axisY, g_lrc_phase), tilted);

    f32 reach = 5.0f;
    f32 farDistance = 1000.0f * g_lrc_ray_length_km;
    b3QueryFilter filter = b3DefaultQueryFilter();

    for i32 i = 0; i < LRC_SHAPE_COUNT; i += 1 {
        // The long ray under test and a short ray on the same line. The short ray is near
        // enough to be accurate, so it is the ground truth: if it hits where the long ray
        // misses, the miss is an accuracy failure, not the cone tilting off the shape.
        lrc_cast_along(g_lrc_targets[i], coneDir, 50.0f, reach, filter);
        bool truthHit = g_lrc_hit;
        b3Pos truthPoint = g_lrc_hit_point;

        lrc_cast_along(g_lrc_targets[i], coneDir, farDistance, reach, filter);
        bool castHit = g_lrc_hit;
        b3Pos castPoint = g_lrc_hit_point;
        b3Vec3 castNormal = g_lrc_hit_normal;

        f32 fail = 0.0f;
        if castHit {
            // Color the hit by how far it drifts from the ground truth.
            f32 error = truthHit ? b3Length(b3SubPos(castPoint, truthPoint)) : 0.0f;
            b3HexColor color = error < 0.05f ? b3_colorGreen : b3_colorOrange;
            g_lrc_trail[i * LRC_TRAIL_COUNT + g_lrc_trail_next[i]] = castPoint;
            g_lrc_trail_next[i] = (g_lrc_trail_next[i] + 1) % LRC_TRAIL_COUNT;
            if g_lrc_trail_count[i] < LRC_TRAIL_COUNT {
                g_lrc_trail_count[i] += 1;
            }
            dbg_line(b3OffsetPos(castPoint, b3MulSV(3.0f, coneDir)), castPoint, b3_colorAqua);
            dbg_line(castPoint, b3OffsetPos(castPoint, b3MulSV(1.5f, castNormal)), b3_colorYellow);
            adapter_point(castPoint, 8.0f, color, null);
        } else if truthHit {
            // Accuracy failure: the line does hit, but single precision lost it at distance.
            // Mark where the hit should have been and slash the empty ray red.
            fail = 1.0f;
            b3Pos expected = truthPoint;
            adapter_point(expected, 14.0f, b3_colorRed, null);
            dbg_line(b3OffsetPos(expected, b3MulSV(2.0f, coneDir)),
                     b3OffsetPos(expected, b3MulSV(-2.0f, coneDir)), b3_colorRed);
        } else {
            // Geometric miss: the cone tilted the ray off the shape. Not an accuracy problem.
            b3Pos aim = g_lrc_targets[i];
            dbg_line_alpha(b3OffsetPos(aim, b3MulSV(2.0f, coneDir)),
                           b3OffsetPos(aim, b3MulSV(-4.0f, coneDir)), b3_colorGray, 0.4f);
        }

        g_lrc_fail_rate[i] = 0.95f * g_lrc_fail_rate[i] + 0.05f * fail;

        // Fade the trail from oldest to newest so the loop reads as a path.
        i32 start = (g_lrc_trail_next[i] - g_lrc_trail_count[i] + LRC_TRAIL_COUNT) % LRC_TRAIL_COUNT;
        for i32 j = 0; j < g_lrc_trail_count[i]; j += 1 {
            i32 index = (start + j) % LRC_TRAIL_COUNT;
            f32 alpha = cast(f32, j + 1) / cast(f32, g_lrc_trail_count[i]);
            dbg_point_alpha(g_lrc_trail[i * LRC_TRAIL_COUNT + index], 4.0f, b3_colorGreen, alpha);
        }
    }

    u8[192] buf;
    draw_text_line("Long ray casts");
    ignore snprintf(cast(u8*, &buf), 192,
                    "Origin %.0f km. Green: accurate, Orange: drifting, Red: miss.",
                    g_lrc_ray_length_km);
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 192,
                    "Failures: sphere %.0f%%  capsule %.0f%%  hull %.0f%%  mesh %.0f%%  hf %.0f%%",
                    100.0f * g_lrc_fail_rate[0], 100.0f * g_lrc_fail_rate[1],
                    100.0f * g_lrc_fail_rate[2], 100.0f * g_lrc_fail_rate[3],
                    100.0f * g_lrc_fail_rate[4]);
    draw_text_line(cast(u8*, &buf));
}

// samples/sample_collision.cpp CastContext + RayCastClosestCallback
struct CastContext {
    b3Pos[3] points;
    b3Vec3[3] normals;
    f32[3] fractions;
    u64[3] materialIds;
    i32[3] triangleIndices;
    i32[3] childIndices;
    i32 count;
    bool initialOverlap;
}

f32 ray_cast_closest_callback(b3ShapeId shapeId, b3Pos point, b3Vec3 normal, f32 fraction,
                              u64 materialId, i32 triangleIndex, i32 childIndex,
                              void* context) {
    CastContext* rayContext = cast(CastContext*, context);

    // Check for initial overlap
    if rayContext.initialOverlap == false && fraction == 0.0f {
        // By returning -1, we instruct the calling code to ignore this shape and
        // continue the ray-cast to the next shape.
        return -1.0f;
    }

    // Ignore a specific shape. Also ignore initial overlap.
    i64 ignoreFlag = cast(i64, b3Shape_GetUserData(shapeId));
    if ignoreFlag == cast(i64, 1) {
        // By returning -1, we instruct the calling code to ignore this shape and
        // continue the ray-cast to the next shape.
        return -1.0f;
    }

    rayContext.points[0] = point;
    rayContext.normals[0] = normal;
    rayContext.fractions[0] = fraction;
    rayContext.materialIds[0] = materialId;
    rayContext.triangleIndices[0] = triangleIndex;
    rayContext.childIndices[0] = childIndex;
    rayContext.count = 1;

    // By returning the current fraction, we instruct the calling code to clip the ray and
    // continue the ray-cast to the next shape. WARNING: do not assume that shapes
    // are reported in order. However, by clipping, we can always get the closest shape.
    return fraction;
}

// samples/sample_collision.cpp ShapeCastDebug
b3Transform g_scd_transform;
b3Vec3 g_scd_translation;
b3Capsule g_scd_capsule;
b3HullData* g_scd_box;
b3Vec3[8] g_scd_points;
b3Vec3[3] g_scd_triangle;

void build_shape_cast_debug() {
    // this triggers an assert with scale of 1 and b3_lengthUnitsPerMeter of 100
    f32 scale = 0.01f;
    g_scd_triangle[0] = b3MulSV(scale, b3Vec3{0.0f, 0.0f, 0.0f});
    g_scd_triangle[1] = b3MulSV(scale, b3Vec3{0.0f, -6400.0f, 0.0f});
    g_scd_triangle[2] = b3MulSV(scale, b3Vec3{6400.0f, 0.0f, 22.609375f});

    b3Vec3 origin = g_scd_triangle[0];
    g_scd_triangle[0] = b3Vec3_zero;
    g_scd_triangle[1] = b3Sub(g_scd_triangle[1], origin);
    g_scd_triangle[2] = b3Sub(g_scd_triangle[2], origin);

    g_scd_points[0] = b3MulSV(scale, b3Vec3{200.305283f, 200.460999f, 9.53760529f});
    g_scd_points[1] = b3MulSV(scale, b3Vec3{-200.305283f, 200.460999f, 9.53760529f});
    g_scd_points[2] = b3MulSV(scale, b3Vec3{-200.305283f, -200.460999f, 9.53760529f});
    g_scd_points[3] = b3MulSV(scale, b3Vec3{200.305283f, -200.460999f, 9.53760529f});
    g_scd_points[4] = b3MulSV(scale, b3Vec3{200.305283f, 200.460999f, -9.53760529f});
    g_scd_points[5] = b3MulSV(scale, b3Vec3{-200.305283f, 200.460999f, -9.53760529f});
    g_scd_points[6] = b3MulSV(scale, b3Vec3{-200.305283f, -200.460999f, -9.53760529f});
    g_scd_points[7] = b3MulSV(scale, b3Vec3{200.305283f, -200.460999f, -9.53760529f});

    g_scd_capsule.center1 = b3MulSV(scale, b3Vec3{43616.2109375f, -100213.0f, 132631.8125f});
    g_scd_capsule.center2 = b3MulSV(scale, b3Vec3{342231.96875f, 359711.6875f, 132631.8125f});
    g_scd_capsule.radius = scale * 1.0f;

    g_scd_transform.p = b3Sub(b3MulSV(scale, b3Vec3{-115200.0f, -19200.0f, -202755.0f}), origin);
    g_scd_transform.q = b3Quat{b3Vec3{0.0f, 0.0f, 0.0f}, 1.0f};

    g_scd_translation = b3MulSV(scale, b3Vec3{0.008614914f, 0.0f, 72267.1171875f});

    g_scd_box = b3CreateHull(cast(b3Vec3*, &g_scd_points), 8, 8);
}

void destroy_shape_cast_debug() {
    b3DestroyHull(g_scd_box);
}

void step_shape_cast_debug(f32 timeStep) {
    ignore timeStep;

    dbg_ground_grid(10);
    dbg_axes(b3WorldTransform_identity, 1.0f);

    b3ShapeCastPairInput input;
    input.proxyA = b3ShapeProxy{cast(b3Vec3*, &g_scd_triangle), 3, 0.0f};
    // input.proxyB = { m_points, 8, 0.0f };
    input.proxyB = b3ShapeProxy{&g_scd_capsule.center1, 2, g_scd_capsule.radius};
    input.transform = g_scd_transform;
    input.translationB = g_scd_translation;
    input.maxFraction = 0.970617533f;
    input.canEncroach = false;

    b3CastOutput output = b3ShapeCast(&input);

    dbg_triangle(b3WorldTransform_identity, g_scd_triangle[0], g_scd_triangle[1],
                 g_scd_triangle[2], b3_colorCyan);
    // DrawHull( m_scene, m_transform, m_box, b3_colorGreen, false );
    dbg_solid_capsule(b3MakeWorldTransform(g_scd_transform), g_scd_capsule,
                      make_color(b3_colorGreen));

    if output.hit {
        // final position with overlap resolution
        b3Vec3 shapeEnd = b3Add(g_scd_transform.p, b3MulSV(output.fraction, g_scd_translation));
        // DrawHull( m_scene, { shapeEnd, m_transform.q }, m_box, b3_colorRed, false );
        dbg_solid_capsule(b3MakeWorldTransform(b3Transform{shapeEnd, g_scd_transform.q}),
                          g_scd_capsule, make_color(b3_colorRed));
    }

    b3Vec3 shapeEnd = b3Add(g_scd_transform.p, g_scd_translation);
    // DrawHull( m_scene, { shapeEnd, m_transform.q }, m_box, b3_colorGray, false );
    dbg_solid_capsule(b3MakeWorldTransform(b3Transform{shapeEnd, g_scd_transform.q}),
                      g_scd_capsule, make_color(b3_colorGray));
}

// samples/sample_collision.cpp InitialOverlap
b3MeshData* g_io_mesh;
bool g_io_initial_overlap;

void build_initial_overlap() {
    i32[6] indices = { 0, 1, 2, 2, 3, 0 };
    b3Vec3[4] vertices = { b3Vec3{-0.5f, 0.5f, 0.5f}, b3Vec3{-0.5f, 0.5f, -0.5f},
                           b3Vec3{-0.5f, -0.5f, -0.5f}, b3Vec3{-0.5f, -0.5f, 0.5f} };
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.position = b3Pos{0.0f, 0.0f, 0.0f};
    bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, 10.0f * B3_DEG_TO_RAD);
    b3BodyId body = b3CreateBody(g_world, &bodyDef);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    // m_mesh = b3CreateBoxMesh( { 0.0f, 0.0f, 0.0f }, { 0.5f, 0.5f, 0.5f } );
    b3MeshDef def = b3MeshDef{};
    def.triangleCount = 2;
    def.indices = cast(i32*, &indices);
    def.vertexCount = 4;
    def.vertices = cast(b3Vec3*, &vertices);
    def.materialIndices = null;
    def.useMedianSplit = false;
    g_io_mesh = b3CreateMesh(&def, null, 0);
    ignore b3CreateMeshShape(body, &shapeDef, g_io_mesh, b3MulSV(4.0f, b3Vec3_one));
    g_io_initial_overlap = true;
}

void destroy_initial_overlap() {
    b3DestroyMesh(g_io_mesh);
}

bool initial_overlap_controls() {
    ignore ImGui_Checkbox("initial overlap", &g_io_initial_overlap);
    return true;
}

void step_initial_overlap(f32 timeStep) {
    ignore timeStep;
    dbg_axes(b3WorldTransform_identity, 1.0f);

    b3Vec3 offset = b3Vec3{-2.1f, -0.8f, 0.95f};
    CastContext context = CastContext{};
    context.initialOverlap = g_io_initial_overlap;
    b3Capsule capsule;
    capsule.center1 = offset;
    capsule.center2 = b3Add(offset, b3Vec3{0.0f, 1.0f, 0.0f});
    capsule.radius = 0.25f;
    // zero length cast
    b3Vec3 translation = b3Vec3{0.0f, 0.0f, 0.0f};
    b3ShapeProxy proxy = b3ShapeProxy{&capsule.center1, 2, capsule.radius};
    dbg_solid_capsule(b3WorldTransform_identity, capsule, make_color(b3_colorGreen));
    b3World_CastShape(g_world, b3Pos_zero, &proxy, translation, b3DefaultQueryFilter(),
                      ray_cast_closest_callback, cast(void*, &context));
    f32 fraction = context.count > 0 ? context.fractions[0] : 1.0f;
    b3WorldTransform shapeEnd = b3MakeWorldTransform(
        b3Transform{b3MulSV(fraction, translation), b3Quat_identity});
    dbg_solid_capsule(shapeEnd, capsule,
                      make_color(context.count > 0 ? b3_colorRed : b3_colorGreen));
    if context.count > 0 {
        b3Pos p = context.points[0];
        b3Vec3 n = context.normals[0];
        dbg_line(p, b3OffsetPos(p, b3MulSV(0.5f, n)), b3_colorAliceBlue);
        adapter_point(p, 8.0f, b3_colorAliceBlue, null);
    }
}

// samples/sample_collision.cpp MeshScale
b3Vec3 g_ms_scale;
b3MeshData* g_ms_mesh;
b3BodyId g_ms_mesh_body;
b3ShapeId g_ms_mesh_shape;
b3Pos g_ms_start;
bool g_ms_sphere_cast;

void build_mesh_scale() {
    g_ms_mesh = b3CreateBoxMesh(b3Vec3{0.0f, 0.0f, 0.0f}, b3Vec3{0.5f, 0.5f, 0.5f}, true);
    g_ms_scale = b3Vec3{1.0f, 1.0f, 1.0f};
    b3BodyDef bodyDef = b3DefaultBodyDef();
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    g_ms_mesh_body = b3CreateBody(g_world, &bodyDef);
    g_ms_mesh_shape = b3CreateMeshShape(g_ms_mesh_body, &shapeDef, g_ms_mesh, g_ms_scale);
    g_ms_start = b3Pos{-2.0f, 0.0f, 0.0f};
    g_ms_sphere_cast = true;
}

void destroy_mesh_scale() {
    b3DestroyMesh(g_ms_mesh);
}

bool mesh_scale_controls() {
    b3Vec3 scale = g_ms_scale;
    bool changed = false;
    changed = changed || ImGui_SliderFloat("Scale X", &scale.x, -2.0f, 2.0f, "%.1f", 0);
    changed = changed || ImGui_SliderFloat("Scale Y", &scale.y, -2.0f, 2.0f, "%.1f", 0);
    changed = changed || ImGui_SliderFloat("Scale Z", &scale.z, -2.0f, 2.0f, "%.1f", 0);
    if changed {
        g_ms_scale = scale;
        b3Shape_SetMesh(g_ms_mesh_shape, g_ms_mesh, g_ms_scale);
    }
    b3Vec3 delta = b3SubPos(g_ms_start, b3Pos_zero);
    ignore ImGui_SliderFloat("Start Y", &delta.y, -2.0f, 2.0f, "%.1f", 0);
    ignore ImGui_SliderFloat("Start Z", &delta.z, -2.0f, 2.0f, "%.1f", 0);
    g_ms_start = b3OffsetPos(b3Pos_zero, delta);
    ignore ImGui_Checkbox("sphere Cast", &g_ms_sphere_cast);
    return true;
}

void step_mesh_scale(f32 timeStep) {
    ignore timeStep;
    b3Pos rayOrigin = g_ms_start;
    b3Vec3 rayTranslation = b3Vec3{4.0f, 0.0f, 0.0f};
    adapter_point(rayOrigin, 8.0f, b3_colorGreen, null);
    adapter_point(b3OffsetPos(rayOrigin, rayTranslation), 8.0f, b3_colorRed, null);
    dbg_line(rayOrigin, b3OffsetPos(rayOrigin, rayTranslation), b3_colorWhite);
    if g_ms_sphere_cast {
        b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, 0.25f};
        CastContext context = CastContext{};
        b3ShapeProxy proxy = b3ShapeProxy{&sphere.center, 1, sphere.radius};
        b3World_CastShape(g_world, g_ms_start, &proxy, rayTranslation, b3DefaultQueryFilter(),
                          ray_cast_closest_callback, cast(void*, &context));
        if context.count > 0 {
            b3WorldTransform transform = b3MakeWorldTransform(
                b3Transform{b3MulSV(context.fractions[0], rayTranslation), b3Quat_identity});
            dbg_solid_sphere(transform, sphere, make_color(b3_colorYellow));
            b3Pos point = context.points[0];
            dbg_line(point, b3OffsetPos(point, b3MulSV(0.5f, context.normals[0])), b3_colorGreen);
            adapter_point(point, 5.0f, b3_colorYellow, null);
        } else {
            b3WorldTransform transform = b3MakeWorldTransform(
                b3Transform{rayTranslation, b3Quat_identity});
            dbg_solid_sphere(transform, sphere, make_color(b3_colorGray));
        }
    } else {
        b3RayResult result = b3World_CastRayClosest(g_world, rayOrigin, rayTranslation,
                                                    b3DefaultQueryFilter());
        if result.hit {
            dbg_line(result.point, b3OffsetPos(result.point, b3MulSV(0.5f, result.normal)),
                     b3_colorGreen);
            adapter_point(result.point, 5.0f, b3_colorYellow, null);
        }
    }
}

// samples/sample_collision.cpp OverlapWorld
b3BoxHull g_ow_box;
b3MeshData* g_ow_mesh;
b3HeightFieldData* g_ow_height_field;
i32 g_ow_base_x;
f32 g_ow_cast_offset;
bool g_ow_tracking;
b3HexColor g_ow_color;

void build_overlap_world() {
    b3World_SetGravity(g_world, b3Vec3_zero);

    g_ow_box = b3MakeBoxHull(0.6f, 0.6f, 0.6f);
    g_ow_mesh = b3CreateTorusMesh(10, 12, 0.65f, 0.35f);
    g_ow_height_field = b3CreateWave(10, 10, b3MulSV(0.2f, b3Vec3_one), 0.03f, 0.09f, false);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    b3ShapeDef shapeDef = b3DefaultShapeDef();

    for i32 index = 0; index < 3; index += 1 {
        b3BodyType type = cast(b3BodyType, index);

        bodyDef.type = type;
        bodyDef.position = b3Pos{-6.0f, 3.0f + 2.0f * cast(f32, index), 0.0f};
        bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisX, 0.5f * PI_F);
        b3BodyId sphereBody = b3CreateBody(g_world, &bodyDef);
        b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, 0.8f};
        ignore b3CreateSphereShape(sphereBody, &shapeDef, &sphere);

        bodyDef.type = type;
        bodyDef.position = b3Pos{-3.0f, 3.0f + 2.0f * cast(f32, index), 0.0f};
        bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, 0.25f * PI_F);
        b3BodyId capsuleBody = b3CreateBody(g_world, &bodyDef);
        b3Capsule capsule = b3Capsule{b3Pos{-0.5f, 0.0f, 0.0f}, b3Pos{0.5f, 0.0f, 0.0f}, 0.5f};
        ignore b3CreateCapsuleShape(capsuleBody, &shapeDef, &capsule);

        bodyDef.type = type;
        bodyDef.position = b3Pos{0.0f, 3.0f + 2.0f * cast(f32, index), 0.0f};
        bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, 0.25f * PI_F);
        b3BodyId hullBody = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateHullShape(hullBody, &shapeDef, &g_ow_box.base);

        bodyDef.type = type;
        bodyDef.position = b3Pos{3.0f, 3.0f + 2.0f * cast(f32, index), 0.0f};
        bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisX, 0.5f * PI_F);
        b3BodyId meshBody = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateMeshShape(meshBody, &shapeDef, g_ow_mesh,
                                 b3Vec3{-0.5f, 1.5f, -1.0f});

        // Height fields only on static bodies
        bodyDef.type = b3_staticBody;
        bodyDef.position = b3Pos{5.0f, 2.0f + 2.0f * cast(f32, index), 0.0f};
        bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisX, -0.5f * PI_F);
        b3BodyId heightBody = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateHeightFieldShape(heightBody, &shapeDef, g_ow_height_field);
    }

    g_ow_base_x = 0;
    g_ow_cast_offset = 0.0f;
    g_ow_tracking = false;
}

void destroy_overlap_world() {
    b3DestroyHeightField(g_ow_height_field);
    b3DestroyMesh(g_ow_mesh);
}

bool overlap_world_result_fcn(b3ShapeId shapeId, void* context) {
    ignore shapeId;
    b3HexColor* color = cast(b3HexColor*, context);
    *color = b3_colorRed;
    // terminate the query
    return false;
}

void overlap_world_spheres() {
    for i32 i = 0; i < 5; i += 1 {
        b3Sphere sphere = b3Sphere{b3Pos{-6.0f + 3.0f * cast(f32, i), 3.0f,
                                         -5.0f + g_ow_cast_offset}, 0.3f};
        b3ShapeProxy proxy = b3ShapeProxy{&sphere.center, 1, sphere.radius};
        g_ow_color = b3_colorGreen;
        b3World_OverlapShape(g_world, b3Pos_zero, &proxy, b3DefaultQueryFilter(),
                             overlap_world_result_fcn, cast(void*, &g_ow_color));
        dbg_solid_sphere(b3WorldTransform_identity, sphere, make_color(g_ow_color));
    }
}

void overlap_world_capsules() {
    for i32 i = 0; i < 5; i += 1 {
        b3Vec3 offset = b3Vec3{-6.0f + 3.0f * cast(f32, i), 5.0f, -5.0f + g_ow_cast_offset};
        b3Capsule capsule = b3Capsule{b3Pos{-0.2f, -0.2f, -0.2f}, b3Pos{0.2f, 0.2f, 0.2f},
                                      0.2f};
        capsule.center1 = b3OffsetPos(capsule.center1, offset);
        capsule.center2 = b3OffsetPos(capsule.center2, offset);
        b3ShapeProxy proxy = b3ShapeProxy{&capsule.center1, 2, capsule.radius};
        g_ow_color = b3_colorGreen;
        b3World_OverlapShape(g_world, b3Pos_zero, &proxy, b3DefaultQueryFilter(),
                             overlap_world_result_fcn, cast(void*, &g_ow_color));
        dbg_solid_capsule(b3WorldTransform_identity, capsule, make_color(g_ow_color));
    }
}

void overlap_world_hulls() {
    for i32 i = 0; i < 5; i += 1 {
        b3Transform transform = b3Transform{
            b3Vec3{-6.0f + 3.0f * cast(f32, i), 7.0f, -5.0f + g_ow_cast_offset},
            b3Quat_identity};
        b3BoxHull box = b3MakeTransformedBoxHull(0.3f, 0.3f, 0.3f, transform);
        b3ShapeProxy proxy = b3ShapeProxy{cast(b3Vec3*, &box.boxPoints),
                                          box.base.vertexCount, 0.0f};
        g_ow_color = b3_colorGreen;
        b3World_OverlapShape(g_world, b3Pos_zero, &proxy, b3DefaultQueryFilter(),
                             overlap_world_result_fcn, cast(void*, &g_ow_color));
        dbg_hull(b3WorldTransform_identity, &box.base, g_ow_color);
    }
}

bool overlap_world_mouse_down(f32 px, f32 py, i32 button, i32 modifiers) {
    ignore py;
    if button == 0 {
        if modifiers == 1 {
            g_ow_tracking = true;
            g_ow_base_x = cast(i32, px);
            return true;
        }
    }
    return false;
}

void overlap_world_mouse_up(f32 px, f32 py, i32 button) {
    ignore px; ignore py;
    if button == 0 {
        g_ow_tracking = false;
    }
}

void overlap_world_mouse_move(f32 px, f32 py) {
    ignore py;
    if g_ow_tracking {
        g_ow_cast_offset = 0.05f * (cast(f32, g_ow_base_x) - px);
    }
}

void step_overlap_world(f32 timeStep) {
    ignore timeStep;
    overlap_world_spheres();
    overlap_world_capsules();
    overlap_world_hulls();
    draw_text_line("Shift + LMB and drag to move shapes");
    dbg_ground_grid(10);
    dbg_line(b3Pos_zero, b3OffsetPos(b3Pos_zero, b3MulSV(0.4f, b3Vec3_axisX)), b3_colorRed);
    dbg_line(b3Pos_zero, b3OffsetPos(b3Pos_zero, b3MulSV(0.4f, b3Vec3_axisY)), b3_colorGreen);
    dbg_line(b3Pos_zero, b3OffsetPos(b3Pos_zero, b3MulSV(0.4f, b3Vec3_axisZ)), b3_colorBlue);
}

// samples/sample_collision.cpp TimeOfImpact
const i32 TOI_BOX = 0;
const i32 TOI_CAPSULE = 1;
const i32 TOI_TRIANGLE = 2;

b3BoxHull g_toi_box;
b3Vec3[3] g_toi_triangle;
b3Capsule g_toi_capsule;
i32 g_toi_type_a;
i32 g_toi_type_b;
b3ShapeProxy g_toi_proxy_a;
b3ShapeProxy g_toi_proxy_b;
b3Sweep g_toi_sweep_a;
b3Sweep g_toi_sweep_b;

b3ShapeProxy toi_make_proxy(i32 type) {
    b3ShapeProxy proxy = b3ShapeProxy{};
    if type == TOI_CAPSULE {
        proxy.points = &g_toi_capsule.center1;
        proxy.radius = g_toi_capsule.radius;
        proxy.count = 2;
    } else if type == TOI_TRIANGLE {
        proxy.points = cast(b3Vec3*, &g_toi_triangle);
        proxy.count = 3;
    } else if type == TOI_BOX {
        proxy.points = b3GetHullPoints(&g_toi_box.base);
        proxy.count = g_toi_box.base.vertexCount;
    }
    return proxy;
}

void build_time_of_impact() {
    g_toi_box = b3MakeBoxHull(0.02f, 0.2f, 0.04f);
    g_toi_capsule = b3Capsule{b3Pos{0.0f, -0.2f, 0.0f}, b3Pos{0.0f, 0.2f, 0.0f}, 0.02f};
    g_toi_triangle[0] = b3Vec3{-4.0f, 0.0f, -4.0f};
    g_toi_triangle[1] = b3Vec3{-4.0f, 0.0f, -8.0f};
    g_toi_triangle[2] = b3Vec3{-8.0f, 0.0f, -8.0f};

    g_toi_type_a = TOI_TRIANGLE;
    g_toi_type_b = TOI_CAPSULE;
    g_toi_proxy_a = toi_make_proxy(g_toi_type_a);
    g_toi_proxy_b = toi_make_proxy(g_toi_type_b);

    g_toi_sweep_a = b3Sweep{};
    g_toi_sweep_a.q1.s = 1.0f;
    g_toi_sweep_a.q2.s = 1.0f;

    g_toi_sweep_b = b3Sweep{};
    g_toi_sweep_b.c1 = b3Pos{-4.06512070f, 0.101333618f, -7.87591267f};
    g_toi_sweep_b.c2 = b3Pos{-4.15895557f, 0.0356027633f, -7.69682646f};
    g_toi_sweep_b.q1 = b3Quat{b3Vec3{-0.860495985f, -0.272824734f, 0.0724888667f},
                              0.424097389f};
    g_toi_sweep_b.q2 = b3Quat{b3Vec3{-0.604184389f, -0.424355596f, 0.0457959622f},
                              0.672894001f};
}

void toi_draw_shape(i32 type, b3WorldTransform transform, b3HexColor color) {
    if type == TOI_CAPSULE {
        dbg_solid_capsule(transform, g_toi_capsule, make_color(color));
        b3Vec3 center = b3Lerp(g_toi_capsule.center1, g_toi_capsule.center2, 0.5f);
        b3WorldTransform xf;
        xf.p = b3TransformWorldPoint(transform, center);
        xf.q = transform.q;
        dbg_axes(xf, 0.025f);
    } else if type == TOI_TRIANGLE {
        b3Pos p1 = b3TransformWorldPoint(transform, g_toi_triangle[0]);
        b3Pos p2 = b3TransformWorldPoint(transform, g_toi_triangle[1]);
        b3Pos p3 = b3TransformWorldPoint(transform, g_toi_triangle[2]);
        dbg_line(p1, p2, color);
        dbg_line(p2, p3, color);
        dbg_line(p3, p1, color);
        dbg_string_3d(p1, make_color(b3_colorWhite), "0");
        dbg_string_3d(p2, make_color(b3_colorWhite), "1");
        dbg_string_3d(p3, make_color(b3_colorWhite), "2");
    } else if type == TOI_BOX {
        dbg_hull(transform, &g_toi_box.base, color);
    }
}

bool time_of_impact_controls() {
    u8*[4] shapeTypes;
    shapeTypes[0] = "point";
    shapeTypes[1] = "segment";
    shapeTypes[2] = "triangle";
    shapeTypes[3] = "box";

    i32 shapeType = g_toi_type_a;
    if ImGui_Combo("shape A", &shapeType, cast(u8**, &shapeTypes), 4, -1) {
        g_toi_type_a = shapeType;
        g_toi_proxy_a = toi_make_proxy(g_toi_type_a);
    }

    shapeType = g_toi_type_b;
    if ImGui_Combo("shape B", &shapeType, cast(u8**, &shapeTypes), 4, -1) {
        g_toi_type_b = shapeType;
        g_toi_proxy_b = toi_make_proxy(g_toi_type_b);
    }
    return true;
}

void step_time_of_impact(f32 timeStep) {
    ignore timeStep;

    b3TOIInput input = b3TOIInput{};
    input.proxyA = g_toi_proxy_a;
    input.proxyB = g_toi_proxy_b;
    input.sweepA = g_toi_sweep_a;
    input.sweepB = g_toi_sweep_b;
    input.maxFraction = 1.0f;

    b3TOIOutput output = b3TimeOfImpact(&input);

    toi_draw_shape(g_toi_type_a, b3WorldTransform_identity, b3_colorCyan);

    b3Transform transform1 = b3GetSweepTransform(&g_toi_sweep_b, 0.0f);
    b3Transform transform2 = b3GetSweepTransform(&g_toi_sweep_b, 1.0f);

    // qr = inv(q1) * q2
    b3Quat qr = b3InvMulQuat(transform1.q, transform2.q);
    f32 angle;
    ignore b3GetAxisAngle(&angle, qr);

    u8[128] buf;
    ignore snprintf(cast(u8*, &buf), 128, "angle = %g", cast(f64, 180.0f * angle / PI_F));
    draw_text_line(cast(u8*, &buf));

    toi_draw_shape(g_toi_type_b, b3MakeWorldTransform(transform1), b3_colorLightGreen);
    toi_draw_shape(g_toi_type_b, b3MakeWorldTransform(transform2), b3_colorLightCoral);

    if output.fraction < 1.0f {
        b3Transform transformHit = b3GetSweepTransform(&g_toi_sweep_b, output.fraction);
        toi_draw_shape(g_toi_type_b, b3MakeWorldTransform(transformHit), b3_colorLightCyan);
    }

    if output.state == b3_toiStateHit || output.state == b3_toiStateFailed {
        b3Pos p = b3ToPos(output.point);
        dbg_line(p, b3OffsetPos(p, b3MulSV(0.5f, output.normal)), b3_colorDimGray);
        adapter_point(p, 10.0f, b3_colorLightGreen, null);
    }

    if output.state == b3_toiStateUnknown {
        draw_text_line("unknown");
    } else if output.state == b3_toiStateFailed {
        draw_text_line("failed");
    } else if output.state == b3_toiStateOverlapped {
        draw_text_line("overlapped");
    } else if output.state == b3_toiStateHit {
        ignore snprintf(cast(u8*, &buf), 128, "hit %g", cast(f64, output.fraction));
        draw_text_line(cast(u8*, &buf));
    } else if output.state == b3_toiStateSeparated {
        draw_text_line("separated");
    }

    ignore snprintf(cast(u8*, &buf), 128, "iterations / push / root = %d / %d / %d",
                    output.distanceIterations, output.pushBackIterations,
                    output.rootIterations);
    draw_text_line(cast(u8*, &buf));

    dbg_axes(b3WorldTransform_identity, 0.5f);
}

// samples/sample_collision.cpp DistanceDebug
b3WorldTransform g_dd_transform_a;
b3WorldTransform g_dd_transform_b;
b3HullData* g_dd_box;
b3BoxHull g_dd_box_a;
b3BoxHull g_dd_box_b;
b3Vec3[8] g_dd_points;
b3Vec3[3] g_dd_triangle;
b3Simplex[32] g_dd_simplexes;
i32 g_dd_simplex_index;
i32 g_dd_simplex_count;

void build_distance_debug() {
    // this triggers an assert with scale of 1 and b3_lengthUnitsPerMeter of 100
    f32 scale = 0.01f;
    g_dd_triangle[0] = b3MulSV(scale, b3Vec3{1400.0f, 1600.0f, 70.1534424f});
    g_dd_triangle[1] = b3MulSV(scale, b3Vec3{1400.0f, 1500.0f, 66.125f});
    g_dd_triangle[2] = b3MulSV(scale, b3Vec3{1500.0f, 1600.0f, 72.350708f});

    // shifting to the origin give more accuracy
    b3Vec3 origin = g_dd_triangle[0];
    g_dd_triangle[0] = b3Vec3_zero;
    g_dd_triangle[1] = b3Sub(g_dd_triangle[1], origin);
    g_dd_triangle[2] = b3Sub(g_dd_triangle[2], origin);

    g_dd_points[0] = b3MulSV(scale, b3Vec3{200.305283f, 200.460999f, 9.53760529f});
    g_dd_points[1] = b3MulSV(scale, b3Vec3{-200.305283f, 200.460999f, 9.53760529f});
    g_dd_points[2] = b3MulSV(scale, b3Vec3{-200.305283f, -200.460999f, 9.53760529f});
    g_dd_points[3] = b3MulSV(scale, b3Vec3{200.305283f, -200.460999f, 9.53760529f});
    g_dd_points[4] = b3MulSV(scale, b3Vec3{200.305283f, 200.460999f, -9.53760529f});
    g_dd_points[5] = b3MulSV(scale, b3Vec3{-200.305283f, 200.460999f, -9.53760529f});
    g_dd_points[6] = b3MulSV(scale, b3Vec3{-200.305283f, -200.460999f, -9.53760529f});
    g_dd_points[7] = b3MulSV(scale, b3Vec3{200.305283f, -200.460999f, -9.53760529f});

    g_dd_box_a = b3MakeBoxHull(40.0f, 1.0f, 40.0f);
    g_dd_box_b = b3MakeTransformedBoxHull(0.5f, 10.0f, 0.5f,
                                          b3Transform{b3Vec3{0.0f, 10.0f, 0.0f},
                                                      b3Quat_identity});

    // m_transformA = {
    //	.p = { 0.0f, -1.0f, 0.0f },
    //	.q = b3Quat_identity,
    // };
    g_dd_transform_a = b3WorldTransform_identity;
    g_dd_transform_b.p = b3Pos{-1.64657831e-06f, 1.00989532471f, 0.0f};
    g_dd_transform_b.q = b3Quat{b3Vec3{0.0f, 0.0f, 0.004947796f}, 0.999987781f};

    g_dd_box = b3CreateHull(cast(b3Vec3*, &g_dd_points), 8, 8);
    g_dd_simplex_count = 0;
    g_dd_simplex_index = 0;
    memset(cast(void*, &g_dd_simplexes), 0,
           cast(i64, 32 * cast(i32, sizeof(b3Simplex))));
}

void destroy_distance_debug() {
    b3DestroyHull(g_dd_box);
}

bool distance_debug_controls() {
    ignore ImGui_SliderInt("simplex index", &g_dd_simplex_index, 0, g_dd_simplex_count - 1,
                           "%d", 0);
    return true;
}

void distance_debug_witness_points(b3Simplex* simplex, b3Vec3* vertexA, b3Vec3* vertexB) {
    b3SimplexVertex* vs = simplex.vertices;
    i32 count = simplex.count;
    if count == 1 {
        *vertexA = vs[0].wA;
        *vertexB = vs[0].wB;
    } else if count == 2 {
        *vertexA = b3Add(b3MulSV(vs[0].a, vs[0].wA), b3MulSV(vs[1].a, vs[1].wA));
        *vertexB = b3Add(b3MulSV(vs[0].a, vs[0].wB), b3MulSV(vs[1].a, vs[1].wB));
    } else if count == 3 {
        *vertexA = b3Add(b3Add(b3MulSV(vs[0].a, vs[0].wA), b3MulSV(vs[1].a, vs[1].wA)),
                         b3MulSV(vs[2].a, vs[2].wA));
        *vertexB = b3Add(b3Add(b3MulSV(vs[0].a, vs[0].wB), b3MulSV(vs[1].a, vs[1].wB)),
                         b3MulSV(vs[2].a, vs[2].wB));
    } else if count == 4 {
        // Force identical points and *zero* distance
        b3Vec3 v = b3Add(b3Add(b3MulSV(vs[0].a, vs[0].wA), b3MulSV(vs[1].a, vs[1].wA)),
                         b3Add(b3MulSV(vs[2].a, vs[2].wA), b3MulSV(vs[3].a, vs[3].wA)));
        *vertexA = v;
        *vertexB = v;
    }
}

b3Vec3 distance_debug_closest_point(b3Simplex* simplex) {
    i32 count = simplex.count;
    b3SimplexVertex* vs = simplex.vertices;
    if count == 1 {
        return simplex.vertices[0].w;
    } else if count == 2 {
        return b3Add(b3MulSV(vs[0].a, vs[0].w), b3MulSV(vs[1].a, vs[1].w));
    } else if count == 3 {
        return b3Add(b3Add(b3MulSV(vs[0].a, vs[0].w), b3MulSV(vs[1].a, vs[1].w)),
                     b3MulSV(vs[2].a, vs[2].w));
    } else if count == 4 {
        return b3Add(b3Add(b3MulSV(vs[0].a, vs[0].w), b3MulSV(vs[1].a, vs[1].w)),
                     b3Add(b3MulSV(vs[2].a, vs[2].w), b3MulSV(vs[3].a, vs[3].w)));
    }
    return b3Vec3_zero;
}

void step_distance_debug(f32 timeStep) {
    ignore timeStep;

    b3DistanceInput input;
    input.proxyA = b3ShapeProxy{cast(b3Vec3*, &g_dd_box_a.boxPoints), 8, 0.0f};
    input.proxyB = b3ShapeProxy{cast(b3Vec3*, &g_dd_box_b.boxPoints), 8, 0.0f};
    input.transform = b3InvMulWorldTransforms(g_dd_transform_a, g_dd_transform_b);
    input.useRadii = false;

    b3SimplexCache cache = b3SimplexCache{};
    b3DistanceOutput output = b3ShapeDistance(&input, &cache,
                                              cast(b3Simplex*, &g_dd_simplexes), 32);
    g_dd_simplex_count = output.simplexCount;

    // DrawFace( m_scene, m_triangle[0], m_triangle[1], m_triangle[2], b3_colorCyan );
    // DrawHull( m_scene, m_transformB, m_box, b3_colorGreen, false );
    dbg_hull(g_dd_transform_a, &g_dd_box_a.base, b3_colorGreen);
    dbg_hull(g_dd_transform_b, &g_dd_box_b.base, b3_colorCyan);

    u8[160] buf;
    for i32 i = 0; i < g_dd_box_b.base.vertexCount; i += 1 {
        b3Pos p = b3TransformWorldPoint(g_dd_transform_b, g_dd_box_b.boxPoints[i]);
        ignore snprintf(cast(u8*, &buf), 160, " %d", i);
        dbg_string_3d(p, make_color(b3_colorAliceBlue), cast(u8*, &buf));
    }

    b3Pos pA = b3TransformWorldPoint(g_dd_transform_a, output.pointA);
    b3Pos pB = b3TransformWorldPoint(g_dd_transform_a, output.pointB);
    adapter_point(pA, 5.0f, b3_colorWhite, null);
    adapter_point(pB, 5.0f, b3_colorWhite, null);

    b3Vec3 normal = b3RotateVector(g_dd_transform_a.q, output.normal);
    dbg_line(pA, b3OffsetPos(pA, b3MulSV(1.0f, normal)), b3_colorWhite);
    ignore snprintf(cast(u8*, &buf), 160, "distance = %g, normal = %g, %g, %g",
                    cast(f64, output.distance), cast(f64, normal.x), cast(f64, normal.y),
                    cast(f64, normal.z));
    draw_text_line(cast(u8*, &buf));

    if g_dd_simplex_count > 0 {
        b3Simplex* simplex = cast(b3Simplex*, &g_dd_simplexes) + g_dd_simplex_index;
        b3Vec3 v1;
        b3Vec3 v2;
        distance_debug_witness_points(simplex, &v1, &v2);
        b3Pos wv1 = b3TransformWorldPoint(g_dd_transform_a, v1);
        b3Pos wv2 = b3TransformWorldPoint(g_dd_transform_a, v2);
        adapter_point(wv1, 10.0f, b3_colorGreen, null);
        adapter_point(wv2, 10.0f, b3_colorGreen, null);

        for i32 i = 0; i < simplex.count; i += 1 {
            b3Pos wA = b3TransformWorldPoint(g_dd_transform_a, simplex.vertices[i].wA);
            b3Pos wB = b3TransformWorldPoint(g_dd_transform_a, simplex.vertices[i].wB);
            adapter_point(wA, 5.0f, b3_colorRed, null);
            adapter_point(wB, 5.0f, b3_colorRed, null);
        }

        b3Vec3 p = distance_debug_closest_point(simplex);
        f32 distance = b3Length(p);
        ignore snprintf(cast(u8*, &buf), 160, "current distance = %.5f", cast(f64, distance));
        draw_text_line(cast(u8*, &buf));
    }

    for i32 i = 0; i < g_dd_simplex_count; i += 1 {
        b3Simplex* simplex = cast(b3Simplex*, &g_dd_simplexes) + i;
        if simplex.count == 1 {
            b3SimplexVertex* v1 = simplex.vertices + 0;
            ignore snprintf(cast(u8*, &buf), 160, "v1s : %d, v2s : %d",
                            cast(i32, v1.indexA), cast(i32, v1.indexB));
            draw_text_line(cast(u8*, &buf));
        } else if simplex.count == 2 {
            b3SimplexVertex* v1 = simplex.vertices + 0;
            b3SimplexVertex* v2 = simplex.vertices + 1;
            ignore snprintf(cast(u8*, &buf), 160, "v1s : %d/%d, v2s : %d/%d",
                            cast(i32, v1.indexA), cast(i32, v2.indexA),
                            cast(i32, v1.indexB), cast(i32, v2.indexB));
            draw_text_line(cast(u8*, &buf));
        } else if simplex.count == 3 {
            b3SimplexVertex* v1 = simplex.vertices + 0;
            b3SimplexVertex* v2 = simplex.vertices + 1;
            b3SimplexVertex* v3 = simplex.vertices + 2;
            ignore snprintf(cast(u8*, &buf), 160, "v1s : %d/%d/%d, v2s : %d/%d/%d",
                            cast(i32, v1.indexA), cast(i32, v2.indexA), cast(i32, v3.indexA),
                            cast(i32, v1.indexB), cast(i32, v2.indexB), cast(i32, v3.indexB));
            draw_text_line(cast(u8*, &buf));
        }
    }

    dbg_ground_grid(10);
    dbg_axes(b3WorldTransform_identity, 1.0f);
}

// samples/sample_collision.cpp ShapeCast
b3MeshData* g_sc_mesh;
i32 g_sc_base_x;
i32 g_sc_base_y;
b3Vec3 g_sc_cast_offset;
bool g_sc_tracking_x;
bool g_sc_tracking_y;
bool g_sc_initial_overlap;

void build_shape_cast() {
    b3BoxHull box = b3MakeBoxHull(0.6f, 0.6f, 0.6f);
    g_sc_mesh = b3CreateTorusMesh(10, 12, 0.65f, 0.35f);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    b3ShapeDef shapeDef = b3DefaultShapeDef();

    for i32 index = 0; index < 3; index += 1 {
        bodyDef.position = b3Pos{-6.0f, 3.0f + 2.0f * cast(f32, index), 0.0f};
        bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisX, 0.5f * PI_F);
        b3BodyId sphereBody = b3CreateBody(g_world, &bodyDef);
        b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, 0.9f};
        ignore b3CreateSphereShape(sphereBody, &shapeDef, &sphere);

        bodyDef.position = b3Pos{-2.0f, 3.0f + 2.0f * cast(f32, index), 0.0f};
        bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, 0.25f * PI_F);
        b3BodyId capsuleBody = b3CreateBody(g_world, &bodyDef);
        b3Capsule capsule = b3Capsule{b3Pos{-0.5f, 0.0f, 0.0f}, b3Pos{0.5f, 0.0f, 0.0f}, 0.7f};
        ignore b3CreateCapsuleShape(capsuleBody, &shapeDef, &capsule);

        bodyDef.position = b3Pos{2.0f, 3.0f + 2.0f * cast(f32, index), 0.0f};
        bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, 0.25f * PI_F);
        b3BodyId hullBody = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateHullShape(hullBody, &shapeDef, &box.base);

        bodyDef.position = b3Pos{6.0f, 3.0f + 2.0f * cast(f32, index), 0.0f};
        bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisX, 0.5f * PI_F);
        b3BodyId meshBody = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateMeshShape(meshBody, &shapeDef, g_sc_mesh, b3Vec3_one);

        // todo add height field
    }

    g_sc_base_x = 0;
    g_sc_base_y = 0;
    g_sc_cast_offset = b3Vec3{0.0f, 0.0f, 0.0f};
    g_sc_tracking_x = false;
    g_sc_tracking_y = false;
    g_sc_initial_overlap = false;
}

void destroy_shape_cast() {
    b3DestroyMesh(g_sc_mesh);
}

void shape_cast_spheres() {
    for i32 castIndex = 0; castIndex < 4; castIndex += 1 {
        // World space sweep
        b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, 0.3f};
        b3Vec3 offset = b3Add(b3Vec3{-6.0f + 4.0f * cast(f32, castIndex), 3.0f, -5.0f},
                              g_sc_cast_offset);
        sphere.center = b3OffsetPos(sphere.center, offset);
        b3Vec3 translation = b3MulSV(10.0f, b3Vec3_axisZ);
        b3ShapeProxy proxy = b3ShapeProxy{&sphere.center, 1, sphere.radius};
        CastContext context = CastContext{};
        context.initialOverlap = g_sc_initial_overlap;
        b3World_CastShape(g_world, b3Pos_zero, &proxy, translation, b3DefaultQueryFilter(),
                          ray_cast_closest_callback, cast(void*, &context));
        dbg_solid_sphere(b3WorldTransform_identity, sphere, make_color(b3_colorGreen));
        if context.count > 0 {
            b3Pos point = context.points[0];
            b3Vec3 normal = context.normals[0];
            f32 fraction = context.fractions[0];
            // final position with overlap resolution
            dbg_solid_sphere(b3MakeWorldTransform(
                                 b3Transform{b3MulSV(fraction, translation), b3Quat_identity}),
                             sphere, make_color(b3_colorRed));
            adapter_point(point, 2.0f, b3_colorRed, null);
            dbg_line(point, b3OffsetPos(point, b3MulSV(0.2f, normal)), b3_colorYellow);
        } else {
            dbg_solid_sphere(b3MakeWorldTransform(b3Transform{translation, b3Quat_identity}),
                             sphere, make_color(b3_colorGray));
        }
    }
}

void shape_cast_capsules() {
    for i32 castIndex = 0; castIndex < 4; castIndex += 1 {
        b3Capsule capsule = b3Capsule{b3Pos{-0.2f, -0.2f, -0.2f}, b3Pos{0.2f, 0.2f, 0.2f},
                                      0.2f};
        b3Vec3 offset = b3Add(b3Vec3{-6.0f + 4.0f * cast(f32, castIndex), 5.0f, -5.0f},
                              g_sc_cast_offset);
        capsule.center1 = b3OffsetPos(capsule.center1, offset);
        capsule.center2 = b3OffsetPos(capsule.center2, offset);
        b3Vec3 translation = b3MulSV(10.0f, b3Vec3_axisZ);
        b3ShapeProxy proxy = b3ShapeProxy{&capsule.center1, 2, capsule.radius};
        CastContext context = CastContext{};
        context.initialOverlap = g_sc_initial_overlap;
        b3World_CastShape(g_world, b3Pos_zero, &proxy, translation, b3DefaultQueryFilter(),
                          ray_cast_closest_callback, cast(void*, &context));
        dbg_solid_capsule(b3WorldTransform_identity, capsule, make_color(b3_colorGreen));
        if context.count > 0 {
            b3Pos point = context.points[0];
            b3Vec3 normal = context.normals[0];
            f32 fraction = context.fractions[0];
            dbg_solid_capsule(b3MakeWorldTransform(
                                  b3Transform{b3MulSV(fraction, translation), b3Quat_identity}),
                              capsule, make_color(b3_colorRed));
            adapter_point(point, 2.0f, b3_colorRed, null);
            dbg_line(point, b3OffsetPos(point, b3MulSV(0.2f, normal)), b3_colorYellow);
        } else {
            dbg_solid_capsule(b3MakeWorldTransform(b3Transform{translation, b3Quat_identity}),
                              capsule, make_color(b3_colorGray));
        }
    }
}

void shape_cast_hulls() {
    for i32 castIndex = 0; castIndex < 4; castIndex += 1 {
        b3Vec3 offset = b3Add(b3Vec3{-6.0f + 4.0f * cast(f32, castIndex), 7.0f, -5.0f},
                              g_sc_cast_offset);
        b3Quat qx = b3MakeQuatFromAxisAngle(b3Vec3_axisX, 0.25f * PI_F);
        b3Quat qy = b3MakeQuatFromAxisAngle(b3Vec3_axisY, 0.25f * PI_F);
        b3Quat qz = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, 0.25f * PI_F);
        b3Quat q = b3MulQuat(qx, b3MulQuat(qy, qz));
        b3Transform transform = b3Transform{offset, q};
        b3BoxHull box = b3MakeTransformedBoxHull(0.3f, 0.3f, 0.3f, transform);
        b3Vec3 translation = b3MulSV(10.0f, b3Vec3_axisZ);
        b3ShapeProxy proxy = b3ShapeProxy{cast(b3Vec3*, &box.boxPoints),
                                          box.base.vertexCount, 0.0f};
        CastContext context = CastContext{};
        context.initialOverlap = g_sc_initial_overlap;
        b3World_CastShape(g_world, b3Pos_zero, &proxy, translation, b3DefaultQueryFilter(),
                          ray_cast_closest_callback, cast(void*, &context));
        dbg_hull(b3WorldTransform_identity, &box.base, b3_colorGreen);
        if context.count > 0 {
            b3Pos point = context.points[0];
            b3Vec3 normal = context.normals[0];
            f32 fraction = context.fractions[0];
            // final position with overlap resolution
            dbg_hull(b3MakeWorldTransform(
                         b3Transform{b3MulSV(fraction, translation), b3Quat_identity}),
                     &box.base, b3_colorRed);
            adapter_point(point, 2.0f, b3_colorRed, null);
            dbg_line(point, b3OffsetPos(point, b3MulSV(0.2f, normal)), b3_colorYellow);
        } else {
            dbg_hull(b3MakeWorldTransform(b3Transform{translation, b3Quat_identity}),
                     &box.base, b3_colorGray);
        }
    }
}

bool shape_cast_controls() {
    ignore ImGui_Checkbox("Initial Overlap", &g_sc_initial_overlap);
    return true;
}

bool shape_cast_mouse_down(f32 px, f32 py, i32 button, i32 modifiers) {
    if button == 0 {
        if modifiers == 1 {
            g_sc_tracking_x = true;
            g_sc_base_x = cast(i32, px);
            return true;
        } else if modifiers == 2 {
            g_sc_tracking_y = true;
            g_sc_base_y = cast(i32, py);
            return true;
        }
    }
    return false;
}

void shape_cast_mouse_up(f32 px, f32 py, i32 button) {
    ignore px; ignore py;
    if button == 0 {
        g_sc_tracking_x = false;
        g_sc_tracking_y = false;
    }
}

void shape_cast_mouse_move(f32 px, f32 py) {
    if g_sc_tracking_x {
        g_sc_cast_offset.z = 0.05f * (cast(f32, g_sc_base_x) - px);
    }
    if g_sc_tracking_y {
        g_sc_cast_offset.y = 0.05f * (cast(f32, g_sc_base_y) - py);
    }
}

void step_shape_cast(f32 timeStep) {
    ignore timeStep;
    shape_cast_spheres();
    shape_cast_capsules();
    shape_cast_hulls();
    draw_text_line("Shift + LMB and drag to shift start position");
    dbg_ground_grid(10);
    dbg_axes(b3WorldTransform_identity, 1.0f);
}

// samples/sample_collision.cpp RayCastAnyCallback
f32 ray_cast_any_callback(b3ShapeId shapeId, b3Pos point, b3Vec3 normal, f32 fraction,
                          u64 materialId, i32 triangleIndex, i32 childIndex, void* context) {
    CastContext* rayContext = cast(CastContext*, context);

    // Check for initial overlap
    if rayContext.initialOverlap == false && fraction == 0.0f {
        // By returning -1, we instruct the calling code to ignore this shape and
        // continue the ray-cast to the next shape.
        return -1.0f;
    }

    i64 ignoreFlag = cast(i64, b3Shape_GetUserData(shapeId));
    if ignoreFlag == cast(i64, 1) {
        // By returning -1, we instruct the calling code to ignore this shape and
        // continue the ray-cast to the next shape.
        return -1.0f;
    }

    rayContext.points[0] = point;
    rayContext.normals[0] = normal;
    rayContext.fractions[0] = fraction;
    rayContext.materialIds[0] = materialId;
    rayContext.triangleIndices[0] = triangleIndex;
    rayContext.childIndices[0] = childIndex;
    rayContext.count = 1;

    // At this point we have a hit, so we know the ray is obstructed.
    // By returning 0, we instruct the calling code to terminate the ray-cast.
    return 0.0f;
}

// samples/sample_collision.cpp RayCastMultipleCallback
f32 ray_cast_multiple_callback(b3ShapeId shapeId, b3Pos point, b3Vec3 normal, f32 fraction,
                               u64 materialId, i32 triangleIndex, i32 childIndex,
                               void* context) {
    CastContext* rayContext = cast(CastContext*, context);

    // Check for initial overlap
    if rayContext.initialOverlap == false && fraction == 0.0f {
        // By returning -1, we instruct the calling code to ignore this shape and
        // continue the ray-cast to the next shape.
        return -1.0f;
    }

    i64 ignoreFlag = cast(i64, b3Shape_GetUserData(shapeId));
    if ignoreFlag == cast(i64, 1) {
        // By returning -1, we instruct the calling code to ignore this shape and
        // continue the ray-cast to the next shape.
        return -1.0f;
    }

    i32 count = rayContext.count;

    rayContext.points[count] = point;
    rayContext.normals[count] = normal;
    rayContext.fractions[count] = fraction;
    rayContext.materialIds[count] = materialId;
    rayContext.triangleIndices[count] = triangleIndex;
    rayContext.childIndices[count] = childIndex;
    rayContext.count = count + 1;

    if rayContext.count == 3 {
        // At this point the buffer is full.
        // By returning 0, we instruct the calling code to terminate the ray-cast.
        return 0.0f;
    }

    // By returning 1, we instruct the caller to continue without clipping the ray.
    return 1.0f;
}

// samples/sample_collision.cpp RayCastSortedCallback
f32 ray_cast_sorted_callback(b3ShapeId shapeId, b3Pos point, b3Vec3 normal, f32 fraction,
                             u64 materialId, i32 triangleIndex, i32 childIndex,
                             void* context) {
    CastContext* rayContext = cast(CastContext*, context);

    // Check for initial overlap
    if rayContext.initialOverlap == false && fraction == 0.0f {
        // By returning -1, we instruct the calling code to ignore this shape and
        // continue the ray-cast to the next shape.
        return -1.0f;
    }

    i64 ignoreFlag = cast(i64, b3Shape_GetUserData(shapeId));
    if ignoreFlag == cast(i64, 1) {
        // By returning -1, we instruct the calling code to ignore this shape and
        // continue the ray-cast to the next shape.
        return -1.0f;
    }

    i32 count = rayContext.count;

    i32 index = 3;
    while fraction < rayContext.fractions[index - 1] {
        index -= 1;

        if index == 0 {
            break;
        }
    }

    if index == 3 {
        // not closer, continue but tell the caller not to consider fractions further than the largest fraction acquired
        // this only happens once the buffer is full
        return rayContext.fractions[2];
    }

    for i32 j = 2; j > index; j -= 1 {
        rayContext.points[j] = rayContext.points[j - 1];
        rayContext.normals[j] = rayContext.normals[j - 1];
        rayContext.fractions[j] = rayContext.fractions[j - 1];
        rayContext.materialIds[j] = rayContext.materialIds[j - 1];
        rayContext.triangleIndices[j] = rayContext.triangleIndices[j - 1];
        rayContext.childIndices[j] = rayContext.childIndices[j - 1];
    }

    rayContext.points[index] = point;
    rayContext.normals[index] = normal;
    rayContext.fractions[index] = fraction;
    rayContext.materialIds[index] = materialId;
    rayContext.triangleIndices[index] = triangleIndex;
    rayContext.childIndices[index] = childIndex;
    rayContext.count = count < 3 ? count + 1 : 3;

    if rayContext.count == 3 {
        return rayContext.fractions[2];
    }

    // By returning 1, we instruct the caller to continue without clipping the ray.
    return 1.0f;
}

// samples/sample_collision.cpp CastWorld
const i32 CW_ANY = 0;
const i32 CW_CLOSEST = 1;
const i32 CW_MULTIPLE = 2;
const i32 CW_SORTED = 3;

const i32 CW_RAY_CAST = 0;
const i32 CW_SPHERE_CAST = 1;
const i32 CW_CAPSULE_CAST = 2;
const i32 CW_BOX_CAST = 3;

const i32 CW_MAX_COUNT = 64;
const i32 CW_IGNORE_BASE = 7;

b3Capsule g_cw_capsule;
b3Sphere g_cw_sphere;
b3BoxHull g_cw_box;
b3MeshData* g_cw_mesh;
b3HeightFieldData* g_cw_height_field;
b3BodyId[CW_MAX_COUNT] g_cw_bodies;
i32 g_cw_body_index;
i32 g_cw_mode;
i32 g_cw_cast_type;
f32 g_cw_cast_radius;
bool g_cw_initial_overlap;
b3Pos g_cw_origin;
b3Vec3 g_cw_translation;
CastContext g_cw_cast_context;

void build_cast_world() {
    g_cw_sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, 0.9f};
    g_cw_capsule = b3Capsule{b3Pos{-0.5f, 0.0f, 0.0f}, b3Pos{0.5f, 0.0f, 0.0f}, 0.8f};
    g_cw_box = b3MakeBoxHull(0.6f, 0.6f, 0.6f);
    g_cw_mesh = b3CreateTorusMesh(10, 12, 0.65f, 0.35f);
    g_cw_height_field = b3CreateWave(10, 10, b3MulSV(0.5f, b3Vec3_one), 0.03f, 0.09f, false);

    g_cw_body_index = 0;
    for i32 i = 0; i < CW_MAX_COUNT; i += 1 {
        g_cw_bodies[i] = b3BodyId{};
    }

    g_cw_mode = CW_CLOSEST;
    g_cw_cast_type = CW_RAY_CAST;
    g_cw_cast_radius = 0.5f;
    g_cw_origin = b3Pos{-20.0f, 10.0f, 0.0f};
    g_cw_translation = b3Vec3{20.0f, 10.0f, 0.0f};
    g_cw_cast_context = CastContext{};
    g_cw_initial_overlap = false;
}

void destroy_cast_world() {
    b3DestroyMesh(g_cw_mesh);
    b3DestroyHeightField(g_cw_height_field);
}

void cast_world_create_shapes(b3ShapeType shapeType, i32 count) {
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.gravityScale = 0.0f;

    for i32 i = 0; i < count; i += 1 {
        if g_cw_bodies[g_cw_body_index].index1 != 0 {
            b3DestroyBody(g_cw_bodies[g_cw_body_index]);
            g_cw_bodies[g_cw_body_index] = b3BodyId{};
        }

        if g_cw_body_index % 3 == 0 {
            bodyDef.type = b3_kinematicBody;
        } else if g_cw_body_index % 2 == 0 {
            bodyDef.type = b3_dynamicBody;
        } else {
            bodyDef.type = b3_staticBody;
        }

        // Heightfield must be a static body
        if shapeType == b3_heightShape {
            bodyDef.type = b3_staticBody;
        }

        bodyDef.position = b3OffsetPos(b3Pos_zero, random_vec3_uniform(-20.0f, 20.0f));
        b3Vec3 axis = random_vec3_uniform(-1.0f, 1.0f);
        axis = b3Normalize(axis);
        f32 angle = random_float_range(-PI_F, PI_F);
        bodyDef.rotation = b3MakeQuatFromAxisAngle(axis, angle);
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);

        i32 flag = (g_cw_body_index & CW_IGNORE_BASE) == CW_IGNORE_BASE ? 1 : 0;
        shapeDef.userData = cast(void*, cast(i64, flag));

        if shapeType == b3_sphereShape {
            shapeDef.baseMaterial.userMaterialId = cast(u64, 11);
            ignore b3CreateSphereShape(bodyId, &shapeDef, &g_cw_sphere);
        } else if shapeType == b3_capsuleShape {
            shapeDef.baseMaterial.userMaterialId = cast(u64, 22);
            ignore b3CreateCapsuleShape(bodyId, &shapeDef, &g_cw_capsule);
        } else if shapeType == b3_hullShape {
            shapeDef.baseMaterial.userMaterialId = cast(u64, 33);
            ignore b3CreateHullShape(bodyId, &shapeDef, &g_cw_box.base);
        } else if shapeType == b3_meshShape {
            shapeDef.baseMaterial.userMaterialId = cast(u64, 44);
            ignore b3CreateMeshShape(bodyId, &shapeDef, g_cw_mesh,
                                     b3Vec3{4.0f, 3.0f, -2.0f});
        } else if shapeType == b3_heightShape {
            shapeDef.baseMaterial.userMaterialId = cast(u64, 55);
            b3SurfaceMaterial[3] materials;
            materials[0] = b3SurfaceMaterial{};
            materials[0].userMaterialId = cast(u64, 111);
            materials[1] = b3SurfaceMaterial{};
            materials[1].userMaterialId = cast(u64, 222);
            materials[2] = b3SurfaceMaterial{};
            materials[2].userMaterialId = cast(u64, 333);
            shapeDef.materials = cast(b3SurfaceMaterial*, &materials);
            shapeDef.materialCount = 3;
            ignore b3CreateHeightFieldShape(bodyId, &shapeDef, g_cw_height_field);
            shapeDef.materials = null;
            shapeDef.materialCount = 0;
        }

        g_cw_bodies[g_cw_body_index] = bodyId;
        g_cw_body_index = (g_cw_body_index + 1) % CW_MAX_COUNT;
    }
}

void cast_world_destroy_body() {
    for i32 i = 0; i < CW_MAX_COUNT; i += 1 {
        if g_cw_bodies[i].index1 != 0 {
            b3DestroyBody(g_cw_bodies[i]);
            g_cw_bodies[i] = b3BodyId{};
            return;
        }
    }
}

bool cast_world_mouse_down(f32 px, f32 py, i32 button, i32 modifiers) {
    if button == 0 && modifiers == 2 {
        PickRay pickRay = build_pick_ray(px, py);
        g_cw_origin = b3Pos{pickRay.origin.x, pickRay.origin.y, pickRay.origin.z};
        g_cw_translation = b3MulSV(100.0f, b3Normalize(b3Vec3{pickRay.translation.x,
                                                              pickRay.translation.y,
                                                              pickRay.translation.z}));
        return true;
    }
    return false;
}

bool cast_world_controls() {
    u8*[4] castTypes;
    castTypes[0] = "Ray";
    castTypes[1] = "Sphere";
    castTypes[2] = "Capsule";
    castTypes[3] = "Box";
    ignore ImGui_Combo("Cast Type", &g_cw_cast_type, cast(u8**, &castTypes), 4, -1);

    if g_cw_cast_type != CW_RAY_CAST && g_cw_cast_type != CW_BOX_CAST {
        ignore ImGui_SliderFloat("Radius", &g_cw_cast_radius, 0.1f, 2.0f, "%.1f", 0);
    }

    u8*[4] modes;
    modes[0] = "Any";
    modes[1] = "Closest";
    modes[2] = "Multiple";
    modes[3] = "Sorted";
    ignore ImGui_Combo("Mode", &g_cw_mode, cast(u8**, &modes), 4, -1);

    ignore ImGui_Checkbox("Initial Overlap", &g_cw_initial_overlap);

    if ImGui_Button("Spheres", ImVec2{0.0f, 0.0f}) {
        cast_world_create_shapes(b3_sphereShape, 10);
    }
    if ImGui_Button("Capsules", ImVec2{0.0f, 0.0f}) {
        cast_world_create_shapes(b3_capsuleShape, 10);
    }
    if ImGui_Button("Hulls", ImVec2{0.0f, 0.0f}) {
        cast_world_create_shapes(b3_hullShape, 10);
    }
    if ImGui_Button("Meshes", ImVec2{0.0f, 0.0f}) {
        cast_world_create_shapes(b3_meshShape, 1);
    }
    if ImGui_Button("Height Field", ImVec2{0.0f, 0.0f}) {
        cast_world_create_shapes(b3_heightShape, 1);
    }
    if ImGui_Button("Destroy Shape", ImVec2{0.0f, 0.0f}) {
        cast_world_destroy_body();
    }
    return true;
}

void step_cast_world(f32 timeStep) {
    ignore timeStep;

    b3HexColor color1 = b3_colorOrange;
    b3HexColor color2 = b3_colorAqua;
    b3HexColor green = b3_colorGreen;
    b3HexColor gray = b3_colorGray;

    b3CastResultFcn modeFcn = ray_cast_closest_callback;
    if g_cw_mode == CW_ANY { modeFcn = ray_cast_any_callback; }
    else if g_cw_mode == CW_MULTIPLE { modeFcn = ray_cast_multiple_callback; }
    else if g_cw_mode == CW_SORTED { modeFcn = ray_cast_sorted_callback; }

    g_cw_cast_context = CastContext{};
    g_cw_cast_context.initialOverlap = g_cw_initial_overlap;

    // Must initialize fractions for sorting
    g_cw_cast_context.fractions[0] = FLT_MAX;
    g_cw_cast_context.fractions[1] = FLT_MAX;
    g_cw_cast_context.fractions[2] = FLT_MAX;

    b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, g_cw_cast_radius};
    b3Capsule capsule = b3Capsule{b3Pos{0.0f, 0.0f, 0.0f}, b3Pos{0.0f, 1.0f, 0.0f},
                                  g_cw_cast_radius};
    b3BoxHull box = b3BoxHull{};
    b3Vec3[2] pointBuffer;
    b3ShapeProxy proxy = b3ShapeProxy{};

    if g_cw_cast_type == CW_RAY_CAST {
        proxy.count = 0;
    } else if g_cw_cast_type == CW_SPHERE_CAST {
        proxy.count = 1;
        proxy.radius = g_cw_cast_radius;
        proxy.points = &sphere.center;
    } else if g_cw_cast_type == CW_CAPSULE_CAST {
        proxy.count = 2;
        proxy.radius = g_cw_cast_radius;
        pointBuffer[0] = capsule.center1;
        pointBuffer[1] = capsule.center2;
        proxy.points = cast(b3Vec3*, &pointBuffer);
    } else if g_cw_cast_type == CW_BOX_CAST {
        b3Vec3 extent = b3Vec3{g_cw_cast_radius, 0.5f * g_cw_cast_radius,
                               0.25f * g_cw_cast_radius};
        b3Transform boxXf = b3Transform{b3Vec3_zero, b3Quat_identity};
        box = b3MakeTransformedBoxHull(extent.x, extent.y, extent.z, boxXf);
        proxy.points = cast(b3Vec3*, &box.boxPoints);
        proxy.count = box.base.vertexCount;
    }

    b3QueryFilter filter = b3DefaultQueryFilter();
    filter.name = "cast_world";

    if g_cw_cast_type == CW_RAY_CAST {
        b3World_CastRay(g_world, g_cw_origin, g_cw_translation, filter, modeFcn,
                        cast(void*, &g_cw_cast_context));
    } else {
        b3World_CastShape(g_world, g_cw_origin, &proxy, g_cw_translation, filter, modeFcn,
                          cast(void*, &g_cw_cast_context));
    }

    if g_cw_cast_context.count > 0 {
        b3HexColor[3] colors = { b3_colorRed, b3_colorGreen, b3_colorBlue };
        dbg_line(g_cw_origin, b3OffsetPos(g_cw_origin, g_cw_translation), color2);
        for i32 i = 0; i < g_cw_cast_context.count; i += 1 {
            b3Pos point = g_cw_cast_context.points[i];
            b3Vec3 normal = g_cw_cast_context.normals[i];
            adapter_point(point, 10.0f, colors[i], null);
            b3Pos head = b3OffsetPos(point, b3MulSV(0.5f, normal));
            b3WorldTransform transform;
            transform.p = b3OffsetPos(g_cw_origin,
                                      b3MulSV(g_cw_cast_context.fractions[i],
                                              g_cw_translation));
            transform.q = b3Quat_identity;
            if g_cw_cast_type == CW_RAY_CAST {
                dbg_line(point, head, colors[i]);
            } else if g_cw_cast_type == CW_SPHERE_CAST {
                dbg_line(point, head, color1);
                // DrawWireSphere( transform, &sphere, 32, MakeColorAlpha( colors[i], 0.5f ) );
                dbg_solid_sphere(transform, sphere, make_color_alpha(colors[i], 0.5f));
            } else if g_cw_cast_type == CW_CAPSULE_CAST {
                dbg_line(point, head, color1);
                dbg_solid_capsule(transform, capsule, make_color(colors[i]));
            } else if g_cw_cast_type == CW_BOX_CAST {
                dbg_line(point, head, color1);
                dbg_hull(transform, &box.base, colors[i]);
            }
        }
    } else {
        dbg_line(g_cw_origin, b3OffsetPos(g_cw_origin, g_cw_translation), color2);
        b3WorldTransform transform;
        transform.p = b3OffsetPos(g_cw_origin, g_cw_translation);
        transform.q = b3Quat_identity;
        if g_cw_cast_type == CW_SPHERE_CAST {
            dbg_solid_sphere(transform, sphere, make_color(gray));
        } else if g_cw_cast_type == CW_CAPSULE_CAST {
            dbg_solid_capsule(transform, capsule, make_color(gray));
        } else if g_cw_cast_type == CW_BOX_CAST {
            dbg_hull(transform, &box.base, gray);
        }
    }

    adapter_point(g_cw_origin, 10.0f, green, null);
    draw_text_line("Ctrl + left mouse to cast through cursor");
    draw_text_line("Shapes drawn in yellow boxes are ignored by the ray");

    // Outline the bodies the cast ignores
    for i32 i = 0; i < CW_MAX_COUNT; i += 1 {
        if (i & CW_IGNORE_BASE) == CW_IGNORE_BASE && g_cw_bodies[i].index1 != 0 {
            b3AABB bounds = b3Body_ComputeAABB(g_cw_bodies[i]);
            adapter_bounds(bounds, b3_colorYellow, null);
        }
    }

    if g_cw_mode == CW_ANY {
        draw_text_line("Cast mode: any - check for obstruction - unsorted");
    } else if g_cw_mode == CW_CLOSEST {
        draw_text_line("Cast mode: closest - find closest shape along the cast");
    } else if g_cw_mode == CW_MULTIPLE {
        draw_text_line("Cast mode: multiple - gather multiple shapes - unsorted");
    } else if g_cw_mode == CW_SORTED {
        draw_text_line("Cast mode: sorted - gather multiple shapes sorted by closeness");
    }

    u8[128] buf;
    for i32 i = 0; i < g_cw_cast_context.count; i += 1 {
        ignore snprintf(cast(u8*, &buf), 128, "material = %d, triangle = %d",
                        cast(i32, g_cw_cast_context.materialIds[i]),
                        g_cw_cast_context.triangleIndices[i]);
        draw_text_line(cast(u8*, &buf));
    }

    dbg_ground_grid(10);
    dbg_axes(b3WorldTransform_identity, 1.0f);
}

// samples/sample_collision.cpp ShapeDistance
const i32 SD_POINT = 0;
const i32 SD_SEGMENT = 1;
const i32 SD_TRIANGLE = 2;
const i32 SD_BOX = 3;
const i32 SD_SIMPLEX_CAPACITY = 20;

b3BoxHull g_sd_box;
b3Vec3[3] g_sd_triangle;
b3Vec3 g_sd_point;
b3Vec3[2] g_sd_segment;

i32 g_sd_type_a;
i32 g_sd_type_b;
f32 g_sd_radius_a;
f32 g_sd_radius_b;
b3ShapeProxy g_sd_proxy_a;
b3ShapeProxy g_sd_proxy_b;

b3SimplexCache g_sd_cache;
b3Simplex[SD_SIMPLEX_CAPACITY] g_sd_simplexes;
i32 g_sd_simplex_count;
i32 g_sd_simplex_index;

b3WorldTransform g_sd_transform_a;
b3WorldTransform g_sd_transform_b;

b3Pos g_sd_base_position;
b3Pos g_sd_drag_start;

b3Quat g_sd_base_quat;
f32 g_sd_rotate_start;

bool g_sd_dragging;
bool g_sd_rotating;
bool g_sd_show_indices;
bool g_sd_use_cache;
bool g_sd_draw_simplex;

b3ShapeProxy sd_make_proxy(i32 type, f32 radius) {
    b3ShapeProxy proxy = b3ShapeProxy{};
    proxy.radius = radius;

    if type == SD_POINT {
        proxy.points = &g_sd_point;
        proxy.count = 1;
    } else if type == SD_SEGMENT {
        proxy.points = cast(b3Vec3*, &g_sd_segment);
        proxy.count = 2;
    } else if type == SD_TRIANGLE {
        proxy.points = cast(b3Vec3*, &g_sd_triangle);
        proxy.count = 3;
    } else if type == SD_BOX {
        proxy.points = b3GetHullPoints(&g_sd_box.base);
        proxy.count = g_sd_box.base.vertexCount;
    }

    return proxy;
}

void sd_draw_shape(i32 type, b3WorldTransform transform, f32 radius, b3HexColor color) {
    if type == SD_POINT {
        if radius > 0.0f {
            b3Sphere sphere = b3Sphere{};
            sphere.center = g_sd_point;
            sphere.radius = radius;
            dbg_solid_sphere(transform, sphere, make_color(color));
        } else {
            b3Pos p = b3TransformWorldPoint(transform, g_sd_point);
            adapter_point(p, 5.0f, color, null);
        }
    } else if type == SD_SEGMENT {
        if radius > 0.0f {
            b3Capsule capsule = b3Capsule{};
            capsule.center1 = g_sd_segment[0];
            capsule.center2 = g_sd_segment[1];
            capsule.radius = radius;
            dbg_solid_capsule(transform, capsule, make_color(color));
        } else {
            b3Pos p1 = b3TransformWorldPoint(transform, g_sd_segment[0]);
            b3Pos p2 = b3TransformWorldPoint(transform, g_sd_segment[1]);
            dbg_line(p1, p2, color);
        }
    } else if type == SD_TRIANGLE {
        b3Pos p1 = b3TransformWorldPoint(transform, g_sd_triangle[0]);
        b3Pos p2 = b3TransformWorldPoint(transform, g_sd_triangle[1]);
        b3Pos p3 = b3TransformWorldPoint(transform, g_sd_triangle[2]);
        dbg_line(p1, p2, color);
        dbg_line(p2, p3, color);
        dbg_line(p3, p1, color);
    } else if type == SD_BOX {
        dbg_hull(transform, &g_sd_box.base, color);
    }
}

void build_shape_distance() {
    g_sd_point = b3Vec3_zero;
    g_sd_segment[0] = b3Vec3{-0.5f, 0.0f, 0.0f};
    g_sd_segment[1] = b3Vec3{0.5f, 0.0f, 0.0f};

    g_sd_triangle[0] = b3Vec3{-1.5f, 0.0f, 0.0f};
    g_sd_triangle[1] = b3Vec3{1.5f, 0.0f, 0.0f};
    g_sd_triangle[2] = b3Vec3{0.0f, 0.0f, 2.0f};
    g_sd_box = b3MakeBoxHull(0.125f, 0.25f, 0.5f);

    g_sd_transform_a = b3WorldTransform_identity;
    g_sd_transform_b.p = b3Pos{0.0f, 1.0f, 0.0f};
    g_sd_transform_b.q = b3Quat_identity;

    g_sd_cache = b3SimplexCache{};
    g_sd_simplex_count = 0;
    g_sd_simplex_index = 0;
    g_sd_drag_start = b3Pos{0.0f, 0.0f, 0.0f};
    g_sd_base_position = b3Pos{0.0f, 0.0f, 0.0f};
    g_sd_base_quat = b3Quat{};
    g_sd_base_quat.v.z = 1.0f;

    g_sd_dragging = false;
    g_sd_rotating = false;
    g_sd_show_indices = false;
    g_sd_use_cache = false;
    g_sd_draw_simplex = false;

    g_sd_type_a = SD_TRIANGLE;
    g_sd_type_b = SD_BOX;
    g_sd_radius_a = 0.0f;
    g_sd_radius_b = 0.0f;

    g_sd_proxy_a = sd_make_proxy(g_sd_type_a, g_sd_radius_a);
    g_sd_proxy_b = sd_make_proxy(g_sd_type_b, g_sd_radius_b);
}

bool shape_distance_controls() {
    u8*[4] shapeTypes = {cast(u8*, "point"), cast(u8*, "segment"),
                         cast(u8*, "triangle"), cast(u8*, "box")};
    i32 shapeType = g_sd_type_a;
    if ImGui_Combo("shape A", &shapeType, cast(u8**, &shapeTypes), 4, -1) {
        g_sd_type_a = shapeType;
        g_sd_proxy_a = sd_make_proxy(g_sd_type_a, g_sd_radius_a);
    }

    if (g_sd_type_a == SD_POINT || g_sd_type_a == SD_SEGMENT)
       && ImGui_SliderFloat("radius A", &g_sd_radius_a, 0.0f, 0.5f, "%.2f", 0) {
        g_sd_proxy_a.radius = g_sd_radius_a;
    }

    shapeType = g_sd_type_b;
    if ImGui_Combo("shape B", &shapeType, cast(u8**, &shapeTypes), 4, -1) {
        g_sd_type_b = shapeType;
        g_sd_proxy_b = sd_make_proxy(g_sd_type_b, g_sd_radius_b);
    }

    if (g_sd_type_b == SD_POINT || g_sd_type_b == SD_SEGMENT)
       && ImGui_SliderFloat("radius B", &g_sd_radius_b, 0.0f, 0.5f, "%.2f", 0) {
        g_sd_proxy_b.radius = g_sd_radius_b;
    }

    ImGui_Separator();

    ignore ImGui_Checkbox("show indices", &g_sd_show_indices);
    ignore ImGui_Checkbox("use cache", &g_sd_use_cache);

    ImGui_Separator();

    if ImGui_Checkbox("draw simplex", &g_sd_draw_simplex) {
        g_sd_simplex_index = 0;
    }

    if g_sd_draw_simplex && g_sd_simplex_count > 0 {
        ignore ImGui_SliderInt("index", &g_sd_simplex_index, 0, g_sd_simplex_count - 1, "%d", 0);
        g_sd_simplex_index = b3ClampInt(g_sd_simplex_index, 0, g_sd_simplex_count - 1);
    }

    return true;
}

b3Pos sd_plane_point(f32 px, f32 py) {
    PickRay ray = build_pick_ray(px, py);
    b3Pos origin = b3Pos{ray.origin.x, ray.origin.y, ray.origin.z};
    b3Vec3 d = b3Normalize(b3Vec3{ray.translation.x, ray.translation.y, ray.translation.z});
    f32 t = b3Dot(b3SubPos(origin, b3Pos_zero), d);
    return b3OffsetPos(origin, b3MulSV(0.0f - t, d));
}

bool shape_distance_mouse_down(f32 px, f32 py, i32 button, i32 modifiers) {
    if button != SAPP_MOUSEBUTTON_LEFT { return false; }
    bool shift = (modifiers & SAPP_MODIFIER_SHIFT) != 0;
    if !shift && g_sd_rotating == false {
        g_sd_dragging = true;
        g_sd_drag_start = sd_plane_point(px, py);
        g_sd_base_position = g_sd_transform_b.p;
        return true;
    } else if shift && g_sd_dragging == false {
        g_sd_rotating = true;
        g_sd_rotate_start = px;
        g_sd_base_quat = g_sd_transform_b.q;
        return true;
    }
    return false;
}

void shape_distance_mouse_up(f32 px, f32 py, i32 button) {
    ignore px; ignore py;
    if button == SAPP_MOUSEBUTTON_LEFT {
        g_sd_dragging = false;
        g_sd_rotating = false;
    }
}

void shape_distance_mouse_move(f32 px, f32 py) {
    if g_sd_dragging {
        b3Pos p = sd_plane_point(px, py);
        b3Vec3 delta = b3SubPos(p, g_sd_drag_start);
        g_sd_transform_b.p = b3OffsetPos(g_sd_base_position, delta);
    } else if g_sd_rotating && sapp_widthf() > 0.0f {
        f32 dx = (px - g_sd_rotate_start) / sapp_widthf();
        f32 angle = b3ClampFloat(2.0f * dx, -PI_F, PI_F);
        b3Vec3 axis = b3Vec3{cam_forward.x, cam_forward.y, cam_forward.z};
        b3Quat deltaQuat = b3MakeQuatFromAxisAngle(axis, angle);
        g_sd_transform_b.q = b3MulQuat(deltaQuat, g_sd_base_quat);
    }
}

void sd_compute_witness_points(b3Vec3* a, b3Vec3* b, b3Simplex* s) {
    b3SimplexVertex* vs = cast(b3SimplexVertex*, &s.vertices);
    i32 count = s.count;

    if count == 1 {
        *a = vs[0].wA;
        *b = vs[0].wB;
    } else if count == 2 {
        *a = b3Add(b3MulSV(vs[0].a, vs[0].wA), b3MulSV(vs[1].a, vs[1].wA));
        *b = b3Add(b3MulSV(vs[0].a, vs[0].wB), b3MulSV(vs[1].a, vs[1].wB));
    } else if count == 3 {
        *a = b3Add(b3Add(b3MulSV(vs[0].a, vs[0].wA), b3MulSV(vs[1].a, vs[1].wA)),
                   b3MulSV(vs[2].a, vs[2].wA));
        *b = b3Add(b3Add(b3MulSV(vs[0].a, vs[0].wB), b3MulSV(vs[1].a, vs[1].wB)),
                   b3MulSV(vs[2].a, vs[2].wB));
    } else if count == 4 {
        // Force identical points and *zero* distance
        b3Vec3 p = b3Add(b3Add(b3MulSV(vs[0].a, vs[0].wA), b3MulSV(vs[1].a, vs[1].wA)),
                         b3Add(b3MulSV(vs[2].a, vs[2].wA), b3MulSV(vs[3].a, vs[3].wA)));
        *a = p;
        *b = p;
    }
}

void step_shape_distance(f32 timeStep) {
    ignore timeStep;

    b3DistanceInput input = b3DistanceInput{};
    input.proxyA = g_sd_proxy_a;
    input.proxyB = g_sd_proxy_b;
    input.transform = b3InvMulWorldTransforms(g_sd_transform_a, g_sd_transform_b);
    input.useRadii = g_sd_radius_a > 0.0f || g_sd_radius_b > 0.0f;

    if g_sd_use_cache == false {
        g_sd_cache.count = 0;
    }

    b3DistanceOutput output = b3ShapeDistance(&input, &g_sd_cache,
                                              cast(b3Simplex*, &g_sd_simplexes),
                                              SD_SIMPLEX_CAPACITY);

    g_sd_simplex_count = output.simplexCount;

    sd_draw_shape(g_sd_type_a, b3WorldTransform_identity, g_sd_radius_a, b3_colorCyan);
    sd_draw_shape(g_sd_type_b, g_sd_transform_b, g_sd_radius_b, b3_colorBisque);

    u8[128] buf;

    if g_sd_draw_simplex {
        b3Simplex* simplex = &g_sd_simplexes[g_sd_simplex_index];
        b3SimplexVertex* vertices = cast(b3SimplexVertex*, &simplex.vertices);

        if g_sd_simplex_index > 0 {
            // The first recorded simplex does not have valid barycentric coordinates
            b3Vec3 pointA;
            b3Vec3 pointB;
            sd_compute_witness_points(&pointA, &pointB, simplex);

            b3Pos pA = b3TransformWorldPoint(g_sd_transform_a, pointA);
            b3Pos pB = b3TransformWorldPoint(g_sd_transform_a, pointB);
            dbg_line(pA, pB, b3_colorWhite);
            adapter_point(pA, 10.0f, b3_colorLightGreen, null);
            adapter_point(pB, 10.0f, b3_colorLightBlue, null);
        }

        b3HexColor[3] colors = {b3_colorRed, b3_colorGreen, b3_colorBlue};

        for i32 i = 0; i < simplex.count; ++i {
            b3SimplexVertex* vertex = &vertices[i];
            b3Pos wA = b3TransformWorldPoint(g_sd_transform_a, vertex.wA);
            b3Pos wB = b3TransformWorldPoint(g_sd_transform_a, vertex.wB);
            adapter_point(wA, 10.0f, colors[i], null);
            adapter_point(wB, 10.0f, colors[i], null);
        }
    } else {
        b3Pos pA = b3TransformWorldPoint(g_sd_transform_a, output.pointA);
        b3Pos pB = b3TransformWorldPoint(g_sd_transform_a, output.pointB);
        dbg_line(pA, pB, b3_colorDimGray);
        adapter_point(pA, 10.0f, b3_colorLightGreen, null);
        adapter_point(pB, 10.0f, b3_colorLightBlue, null);

        b3Vec3 normal = b3RotateVector(g_sd_transform_a.q, output.normal);
        dbg_line(pA, b3OffsetPos(pA, b3MulSV(0.5f, normal)), b3_colorYellow);
    }

    if g_sd_show_indices {
        for i32 i = 0; i < g_sd_proxy_a.count; ++i {
            b3Pos p = b3TransformWorldPoint(g_sd_transform_a, g_sd_proxy_a.points[i]);
            ignore snprintf(cast(u8*, &buf), 128, " %d", i);
            dbg_string_3d(p, make_color(b3_colorWhite), cast(u8*, &buf));
        }

        for i32 i = 0; i < g_sd_proxy_b.count; ++i {
            b3Pos p = b3TransformWorldPoint(g_sd_transform_b, g_sd_proxy_b.points[i]);
            ignore snprintf(cast(u8*, &buf), 128, " %d", i);
            dbg_string_3d(p, make_color(b3_colorWhite), cast(u8*, &buf));
        }
    }

    dbg_axes(b3WorldTransform_identity, 0.5f);

    draw_text_line("mouse button 1: drag");
    draw_text_line("mouse button 1 + shift: rotate");
    ignore snprintf(cast(u8*, &buf), 128, "distance = %.4f, iterations = %d",
                    cast(f64, output.distance), output.iterations);
    draw_text_line(cast(u8*, &buf));

    if g_sd_cache.count == 1 {
        ignore snprintf(cast(u8*, &buf), 128, "cache = {%d}, {%d}",
                        cast(i32, g_sd_cache.indexA[0]), cast(i32, g_sd_cache.indexB[0]));
        draw_text_line(cast(u8*, &buf));
    } else if g_sd_cache.count == 2 {
        ignore snprintf(cast(u8*, &buf), 128, "cache = {%d, %d}, {%d, %d}",
                        cast(i32, g_sd_cache.indexA[0]), cast(i32, g_sd_cache.indexA[1]),
                        cast(i32, g_sd_cache.indexB[0]), cast(i32, g_sd_cache.indexB[1]));
        draw_text_line(cast(u8*, &buf));
    } else if g_sd_cache.count == 3 {
        ignore snprintf(cast(u8*, &buf), 128, "cache = {%d, %d, %d}, {%d, %d, %d}",
                        cast(i32, g_sd_cache.indexA[0]), cast(i32, g_sd_cache.indexA[1]),
                        cast(i32, g_sd_cache.indexA[2]), cast(i32, g_sd_cache.indexB[0]),
                        cast(i32, g_sd_cache.indexB[1]), cast(i32, g_sd_cache.indexB[2]));
        draw_text_line(cast(u8*, &buf));
    }
}
