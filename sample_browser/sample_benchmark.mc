// Benchmark scenes — large body counts.

import box3d;
import box3d_human;
import box3d_shared;
import sokol_all;
import sokol_imgui;
import math;
import linear;
import str;
import camera;
import gui;
import renderer;
import sample;
import sample_bodies;
import sample_continuous;
import sample_robustness;
import sample_stacking;
import sample_joint;
import sample_compound;
import sample_world;
import sample_shapes;
import sample_issues;
import debug_adapter;

void build_large_pyramid() {
    b3World_EnableSleeping(g_world, false);
    i32 baseCount = 100;
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{0.0f, -1.0f, 0.0f};
        b3BodyId groundId = b3CreateBody(g_world, &bodyDef);
        b3BoxHull gbox = b3MakeBoxHull(400.0f, 1.0f, 400.0f);
        b3ShapeDef gsd = b3DefaultShapeDef();
        b3ShapeId groundShapeId = b3CreateHullShape(groundId, &gsd, &gbox.base);
        set_ground_shape(groundShapeId);
    }
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.density = 100.0f;
    f32 h = 0.5f;
    b3BoxHull box = b3MakeBoxHull(h, h, h);
    f32 shift = 1.0f * h;
    for i32 i = 0; i < baseCount; i++ {
        f32 y = (2.0f * cast(f32, i) + 1.0f) * shift;
        for i32 j = i; j < baseCount; j++ {
            f32 x = (cast(f32, i) + 1.0f) * shift
                + 2.0f * cast(f32, j - i) * shift - h * cast(f32, baseCount);
            bodyDef.position = b3Pos{x, y, 0.0f};
            b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
            ignore b3CreateHullShape(bodyId, &shapeDef, &box.base);
        }
    }
}

// shared/benchmarks.c CreateWidePyramid (upstream BenchmarkWidePyramid)
void build_wide_pyramid() {
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{0.0f, -1.0f, 0.0f};
        b3BodyId groundId = b3CreateBody(g_world, &bodyDef);
        b3BoxHull gbox = b3MakeBoxHull(100.0f, 1.0f, 100.0f);
        b3ShapeDef gsd = b3DefaultShapeDef();
        b3ShapeId groundShapeId = b3CreateHullShape(groundId, &gsd, &gbox.base);
        set_ground_shape(groundShapeId);
    }
    f32 boxSize = 2.0f;
    f32 boxSeparation = 0.5f;
    f32 halfBoxSize = 0.5f * boxSize;
    i32 pyramidHeight = 15;
    f32 h = halfBoxSize - 0.025f;
    b3BoxHull box = b3MakeBoxHull(h, h, h);
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    for i32 i = 0; i < pyramidHeight; i++ {
        for i32 j = i / 2; j < pyramidHeight - (i + 1) / 2; j++ {
            for i32 k = i / 2; k < pyramidHeight - (i + 1) / 2; k++ {
                f32 x = -cast(f32, pyramidHeight) + boxSize * cast(f32, j)
                    + ((i & 1) != 0 ? halfBoxSize : 0.0f);
                f32 y = 1.0f + (boxSize + boxSeparation) * cast(f32, i);
                f32 z = -cast(f32, pyramidHeight) + boxSize * cast(f32, k)
                    + ((i & 1) != 0 ? halfBoxSize : 0.0f);
                bodyDef.position = b3Pos{x, y, z};
                b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
                ignore b3CreateHullShape(bodyId, &shapeDef, &box.base);
            }
        }
    }
}

// shared/benchmarks.c CreateManyPyramids (upstream BenchmarkManyPyramids)
void many_small_pyramid(i32 baseCount, f32 extent, f32 centerX, f32 baseZ) {
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.enableSleep = false;
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.density = 100.0f;
    b3BoxHull box = b3MakeBoxHull(extent, extent, extent);
    for i32 i = 0; i < baseCount; i++ {
        f32 y = (2.0f * cast(f32, i) + 1.0f) * extent;
        for i32 j = i; j < baseCount; j++ {
            f32 x = (cast(f32, i) + 1.0f) * extent
                + 2.0f * cast(f32, j - i) * extent + centerX - 0.5f;
            bodyDef.position = b3Pos{x, y, baseZ};
            b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
            ignore b3CreateHullShape(bodyId, &shapeDef, &box.base);
        }
    }
}

void build_many_pyramids() {
    i32 baseCount = 10;
    f32 extent = 0.5f;
    i32 rowCount = 14;
    i32 columnCount = 14;
    f32 groundExtent = extent * cast(f32, columnCount) * (cast(f32, baseCount) + 1.0f);
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{0.0f, -1.0f, 0.0f};
        b3BodyId groundId = b3CreateBody(g_world, &bodyDef);
        b3ShapeDef gsd = b3DefaultShapeDef();
        b3BoxHull gbox = b3MakeBoxHull(groundExtent, 1.0f, groundExtent);
        b3ShapeId groundShapeId = b3CreateHullShape(groundId, &gsd, &gbox.base);
        set_ground_shape(groundShapeId);
    }
    f32 baseWidth = 2.0f * extent * cast(f32, baseCount);
    f32 baseZ = -groundExtent + 2.0f * extent;
    f32 deltaZ = 2.0f * (groundExtent - 2.0f * extent) / (cast(f32, rowCount) - 1.0f);
    for i32 i = 0; i < rowCount; i++ {
        for i32 j = 0; j < columnCount; j++ {
            f32 centerX = -groundExtent + cast(f32, j) * (baseWidth + 2.0f * extent) + 2.0f * extent;
            many_small_pyramid(baseCount, extent, centerX, baseZ);
        }
        baseZ += deltaZ;
    }
}

// samples/sample_benchmark.cpp FallingBoxes
void build_falling_boxes() {
    ignore add_ground_box(100.0f);
    i32 n = 50;
    f32 a = 0.5f;
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    b3BoxHull box = b3MakeCubeHull(a);
    for i32 i = 0; i < n; i++ {
        for i32 j = 0; j < 8; j++ {
            for i32 k = 0; k < 8; k++ {
                bodyDef.position = b3Pos{
                    -16.0f * a + 4.0f * a * cast(f32, j),
                    4.0f * a * cast(f32, i) + 5.0f * a,
                    -16.0f * a + 4.0f * a * cast(f32, k)};
                b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
                ignore b3CreateHullShape(bodyId, &shapeDef, &box.base);
            }
        }
    }
}

// samples/sample_robustness.cpp TinyPyramid

// shared/benchmarks.c CreateJointGrid
const i32 JOINT_GRID_N = 100;
b3BodyId[JOINT_GRID_N * JOINT_GRID_N] g_jg_bodies;

void create_joint_grid() {
    b3World_EnableSleeping(g_world, false);

    i32 n = JOINT_GRID_N;

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.filter.categoryBits = 2;
    shapeDef.filter.maskBits = ~cast(u64, 2);

    b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, 0.4f};

    b3SphericalJointDef jointDef = b3DefaultSphericalJointDef();
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.enableSleep = false;

    i32 index = 0;

    for i32 k = 0; k < n; k += 1 {
        for i32 i = 0; i < n; i += 1 {
            f32 fk = cast(f32, k);
            f32 fi = cast(f32, i);

            if i == 0 {
                bodyDef.type = b3_staticBody;
            } else {
                bodyDef.type = b3_dynamicBody;
            }

            bodyDef.position = b3Pos{fk, -fi, 0.0f};

            b3BodyId body = b3CreateBody(g_world, &bodyDef);

            ignore b3CreateSphereShape(body, &shapeDef, &sphere);

            if i > 0 {
                jointDef.base.bodyIdA = g_jg_bodies[index - 1];
                jointDef.base.bodyIdB = body;
                jointDef.base.localFrameA.p = b3Vec3{0.0f, -0.5f, 0.0f};
                jointDef.base.localFrameB.p = b3Vec3{0.0f, 0.5f, 0.0f};
                ignore b3CreateSphericalJoint(g_world, &jointDef);
            }

            if k > 0 {
                jointDef.base.bodyIdA = g_jg_bodies[index - n];
                jointDef.base.bodyIdB = body;
                jointDef.base.localFrameA.p = b3Vec3{0.5f, 0.0f, 0.0f};
                jointDef.base.localFrameB.p = b3Vec3{-0.5f, 0.0f, 0.0f};
                ignore b3CreateSphericalJoint(g_world, &jointDef);
            }

            g_jg_bodies[index] = body;
            index += 1;
        }
    }
}

// samples/sample_benchmark.cpp BenchmarkJointGrid
void build_joint_grid() {
    create_joint_grid();
}

void step_joint_grid(f32 timeStep) {
    ignore timeStep;
    b3Transform t = b3Transform{b3Vec3{0.0f, 0.1f, 0.0f}, b3Quat_identity};
    dbg_axes(b3MakeWorldTransform(t), 4.0f);
}

// shared/benchmarks.c large-world floor
const f32 STATIC_FLOOR_CELL_SIZE = 10.0f;
const i32 STATIC_FLOOR_GRID = 1000;
const i32 STATIC_FLOOR_SPHERES = 100;
const i32 STATIC_FLOOR_DROP_INTERVAL = 5;

i32 g_spheres_dropped;

void get_large_world_capacity(b3Capacity* capacity) {
    i32 floorCount = STATIC_FLOOR_GRID * STATIC_FLOOR_GRID;
    capacity.staticShapeCount = floorCount;
    capacity.staticBodyCount = floorCount;
    capacity.dynamicShapeCount = STATIC_FLOOR_SPHERES;
    capacity.dynamicBodyCount = STATIC_FLOOR_SPHERES;
    capacity.contactCount = b3MaxInt(1024, 8 * STATIC_FLOOR_SPHERES);
}

void create_large_world() {
    g_spheres_dropped = 0;

    f32 cell = STATIC_FLOOR_CELL_SIZE;
    i32 gridCount = STATIC_FLOOR_GRID;
    f32 halfSpan = 0.5f * cell * cast(f32, gridCount);

    b3BoxHull box = b3MakeBoxHull(0.5f * cell, 0.25f, 0.5f * cell);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    b3ShapeDef shapeDef = b3DefaultShapeDef();

    // The trigger: every static shape gets buffered into the move set on creation.
    shapeDef.invokeContactCreation = true;

    for i32 i = 0; i < gridCount; i += 1 {
        f32 x = -halfSpan + (cast(f32, i) + 0.5f) * cell;
        for i32 j = 0; j < gridCount; j += 1 {
            f32 z = -halfSpan + (cast(f32, j) + 0.5f) * cell;
            bodyDef.position = b3Pos{x, 0.0f, z};
            b3BodyId body = b3CreateBody(g_world, &bodyDef);
            ignore b3CreateHullShape(body, &shapeDef, &box.base);
        }
    }
}

void step_large_world_world(i32 stepCount) {
    if g_spheres_dropped >= STATIC_FLOOR_SPHERES { return; }
    if stepCount == 0 { return; }
    if (stepCount % STATIC_FLOOR_DROP_INTERVAL) != 0 { return; }

    // Spread spheres in a coarse grid across the floor so they don't all pile on one box.
    i32 side = 1;
    while side * side < STATIC_FLOOR_SPHERES { side += 1; }

    i32 idx = g_spheres_dropped;
    i32 gi = idx % side;
    i32 gj = idx / side;

    f32 halfSpan = 0.5f * STATIC_FLOOR_CELL_SIZE * cast(f32, STATIC_FLOOR_GRID);
    // Confine drops to the inner 80% of the floor so spheres can't roll off the edge.
    f32 inset = 0.1f * 2.0f * halfSpan;
    f32 usable = 2.0f * halfSpan - 2.0f * inset;
    f32 x = -halfSpan + inset + (cast(f32, gi) + 0.5f) * (usable / cast(f32, side));
    f32 z = -halfSpan + inset + (cast(f32, gj) + 0.5f) * (usable / cast(f32, side));

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = b3Pos{x, 1.5f, z};

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, 0.5f};

    b3BodyId body = b3CreateBody(g_world, &bodyDef);
    ignore b3CreateSphereShape(body, &shapeDef, &sphere);

    g_spheres_dropped += 1;
}

// samples/sample_benchmark.cpp BenchmarkLargeWorld
void build_large_world() {
    create_large_world();
}

void step_large_world(f32 timeStep) {
    ignore timeStep;
    step_large_world_world(g_step_count);
}

// shared/benchmarks.c CreateWasher
void get_washer_capacity(b3Capacity* capacity) {
    capacity.staticShapeCount = 16;
    capacity.dynamicShapeCount = 10000;
    capacity.staticBodyCount = 16;
    capacity.dynamicBodyCount = 10000;
    capacity.contactCount = 60000;
}

void create_washer() {
    bool kinematic = true;

    b3BodyId groundId;
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position.y = -1.0f;
        groundId = b3CreateBody(g_world, &bodyDef);

        b3BoxHull box = b3MakeBoxHull(60.0f, 1.0f, 60.0f);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        set_ground_shape(b3CreateHullShape(groundId, &shapeDef, &box.base));
    }

    {
        f32 motorSpeed = 25.0f;

        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{0.0f, 21.0f, 0.0f};

        if kinematic == true {
            bodyDef.type = b3_kinematicBody;
            bodyDef.angularVelocity = b3Vec3{0.0f, 0.0f, (PI_F / 180.0f) * motorSpeed};
            bodyDef.linearVelocity = b3Vec3{0.001f, -0.002f, 0.0f};
        } else {
            bodyDef.type = b3_dynamicBody;
        }

        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);

        b3ShapeDef shapeDef = b3DefaultShapeDef();

        f32 r0 = 14.0f;
        f32 r1 = 16.0f;
        f32 r2 = 18.0f;
        b3Vec3 nd = b3Vec3{0.0f, 0.0f, -10.0f};
        b3Vec3 pd = b3Vec3{0.0f, 0.0f, 10.0f};

        f32 angle = PI_F / 18.0f;
        b3Quat q = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, angle);
        b3Quat qo = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, 0.1f * angle);
        b3Vec3 u1 = b3Vec3{1.0f, 0.0f, 0.0f};
        for i32 i = 0; i < 36; i += 1 {
            b3Vec3 u2;
            if i == 35 {
                u2 = b3Vec3{1.0f, 0.0f, 0.0f};
            } else {
                u2 = b3RotateVector(q, u1);
            }

            {
                b3Vec3 a1 = b3InvRotateVector(qo, u1);
                b3Vec3 a2 = b3RotateVector(qo, u2);
                b3Vec3[8] points;
                points[0] = b3MulAdd(nd, r1, a1);
                points[1] = b3MulAdd(nd, r2, a1);
                points[2] = b3MulAdd(nd, r1, a2);
                points[3] = b3MulAdd(nd, r2, a2);
                points[4] = b3MulAdd(pd, r1, a1);
                points[5] = b3MulAdd(pd, r2, a1);
                points[6] = b3MulAdd(pd, r1, a2);
                points[7] = b3MulAdd(pd, r2, a2);
                b3HullData* hull = b3CreateHull(cast(b3Vec3*, &points), 8, 8);
                ignore b3CreateHullShape(bodyId, &shapeDef, hull);
                b3DestroyHull(hull);
            }

            if i % 9 == 0 {
                b3Vec3[8] points;
                points[0] = b3MulAdd(nd, r0, u1);
                points[1] = b3MulAdd(nd, r1, u1);
                points[2] = b3MulAdd(nd, r0, u2);
                points[3] = b3MulAdd(nd, r1, u2);
                points[4] = b3MulAdd(pd, r0, u1);
                points[5] = b3MulAdd(pd, r1, u1);
                points[6] = b3MulAdd(pd, r0, u2);
                points[7] = b3MulAdd(pd, r1, u2);
                b3HullData* hull = b3CreateHull(cast(b3Vec3*, &points), 8, 8);
                ignore b3CreateHullShape(bodyId, &shapeDef, hull);
                b3DestroyHull(hull);
            }

            u1 = u2;
        }

        if kinematic == false {
            b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
            jointDef.base.bodyIdA = groundId;
            jointDef.base.bodyIdB = bodyId;
            jointDef.base.localFrameA.p.y = 10.0f;
            jointDef.motorSpeed = (PI_F / 180.0f) * motorSpeed;
            jointDef.maxMotorTorque = 100000000.0f;
            jointDef.enableMotor = true;
            ignore b3CreateRevoluteJoint(g_world, &jointDef);
        }
    }

    i32 gridCount = 20;
    f32 a = 0.2f;

    b3BoxHull cube = b3MakeBoxHull(a, a, a);
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    b3ShapeDef shapeDef = b3DefaultShapeDef();

    f32 x = -2.0f * a * cast(f32, gridCount);
    for i32 i = 0; i < gridCount; i += 1 {
        f32 y = -2.0f * a * cast(f32, gridCount) + 21.0f;
        for i32 j = 0; j < gridCount; j += 1 {
            f32 z = -2.0f * a * cast(f32, gridCount);
            for i32 k = 0; k < gridCount; k += 1 {
                bodyDef.position = b3Pos{x, y, z};
                b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);

                ignore b3CreateHullShape(bodyId, &shapeDef, &cube.base);
                z += 4.0f * a;
            }

            y += 4.0f * a;
        }

        x += 4.0f * a;
    }
}

// samples/sample_benchmark.cpp BenchmarkWasher
void build_washer() {
    create_washer();
}

// samples/sample_benchmark.cpp BenchmarkHull
const i32 BH_CAPACITY = 64;
b3HullData* g_bh_hull;
b3HullData* g_bh_transformed_hull;
b3Vec3 g_bh_scale;
b3Vec3[BH_CAPACITY] g_bh_points;
i32 g_bh_count;

void build_benchmark_hull() {
    g_randomSeed = 42;

    g_bh_count = 64;
    for i32 i = 0; i < g_bh_count; i += 1 {
        g_bh_points[i] = random_vec3(b3Vec3{-1.0f, -1.0f, -1.0f}, b3Vec3{1.0f, 1.0f, 1.0f});
    }

    g_bh_hull = b3CreateHull(cast(b3Vec3*, &g_bh_points), g_bh_count, g_bh_count);
    g_bh_scale = b3Vec3{-1.0f, 1.0f, 1.0f};
    g_bh_transformed_hull = b3CloneAndTransformHull(g_bh_hull, b3Transform_identity, g_bh_scale);
}

void destroy_benchmark_hull() {
    b3DestroyHull(g_bh_hull);
    b3DestroyHull(g_bh_transformed_hull);
}

void step_benchmark_hull(f32 timeStep) {
    ignore timeStep;
    b3Transform t1 = b3Transform{b3Vec3{-2.0f, 0.0f, 0.0f}, b3Quat_identity};
    b3Transform t2 = b3Transform{b3Vec3{2.0f, 0.0f, 0.0f}, b3Quat_identity};

    dbg_hull(b3MakeWorldTransform(t1), g_bh_hull, b3_colorGreen);
    dbg_hull(b3MakeWorldTransform(t2), g_bh_transformed_hull, b3_colorYellow);

    u64 startTick = b3GetTicks();
    f32 area = 0.0f;
    i32 trials = 2000;

    for i32 i = 0; i < trials; i += 1 {
        b3HullData* hull = b3CreateHull(cast(b3Vec3*, &g_bh_points), g_bh_count, g_bh_count);
        area += hull.surfaceArea;
        b3DestroyHull(hull);
    }

    f32 createTime = b3GetMillisecondsAndReset(&startTick);

    f32 scaledArea = 0.0f;
    for i32 i = 0; i < trials; i += 1 {
        b3HullData* hull = b3CloneAndTransformHull(g_bh_hull, b3Transform_identity, g_bh_scale);
        scaledArea += hull.surfaceArea;
        b3DestroyHull(hull);
    }

    f32 cloneTime = b3GetMilliseconds(startTick);

    u8[160] buf;
    ignore snprintf(cast(u8*, &buf), 160, "trials = %d", trials);
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 160, "createTime (us) = %.2f, area = %.2f",
                    1000.0f * createTime / cast(f32, trials), area / cast(f32, trials));
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 160, "cloneTime (us) = %.2f, area = %.2f",
                    1000.0f * cloneTime / cast(f32, trials), scaledArea / cast(f32, trials));
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 160, "createTime / cloneTime = %.2f", createTime / cloneTime);
    draw_text_line(cast(u8*, &buf));
}

// shared/benchmarks.c CreateJunkyard
b3BodyId g_junkyard_pusher;
f32 g_junkyard_degrees;
f32 g_junkyard_radius;

void get_junkyard_capacity(b3Capacity* capacity) {
    capacity.staticShapeCount = 16;
    capacity.dynamicShapeCount = 20 * 20 * 24 + 1;
    capacity.staticBodyCount = 16;
    capacity.dynamicBodyCount = 20 * 20 * 24 + 1;
    capacity.contactCount = 250 * 1024;
}

void create_junkyard() {
    b3BodyId groundId;
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position.y = -1.0f;
        groundId = b3CreateBody(g_world, &bodyDef);
    }

    {
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        {
            b3BoxHull box = b3MakeBoxHull(120.0f, 1.0f, 120.0f);
            set_ground_shape(b3CreateHullShape(groundId, &shapeDef, &box.base));
        }
        {
            b3Vec3 offset = b3Vec3{-50.0f, 8.0f, 0.0f};
            b3BoxHull box = b3MakeOffsetBoxHull(1.0f, 8.0f, 50.0f, offset);
            ignore b3CreateHullShape(groundId, &shapeDef, &box.base);
        }
        {
            b3Vec3 offset = b3Vec3{50.0f, 8.0f, 0.0f};
            b3BoxHull box = b3MakeOffsetBoxHull(1.0f, 8.0f, 50.0f, offset);
            ignore b3CreateHullShape(groundId, &shapeDef, &box.base);
        }
        {
            b3Vec3 offset = b3Vec3{0.0f, 8.0f, -50.0f};
            b3BoxHull box = b3MakeOffsetBoxHull(50.0f, 8.0f, 1.0f, offset);
            ignore b3CreateHullShape(groundId, &shapeDef, &box.base);
        }
        {
            b3Vec3 offset = b3Vec3{0.0f, 8.0f, 50.0f};
            b3BoxHull box = b3MakeOffsetBoxHull(50.0f, 8.0f, 1.0f, offset);
            ignore b3CreateHullShape(groundId, &shapeDef, &box.base);
        }
    }
    {
        b3HullData* rockHull = b3CreateRock(1.5f);

        i32 count = 24;
        f32 height = 24.0f;
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        for i32 Y = 0; Y < count; Y += 1 {
            for i32 X = 0; X <= 20; X += 1 {
                for i32 Z = 0; Z <= 20; Z += 1 {
                    bodyDef.position.x = -40.0f + 4.0f * cast(f32, X);
                    bodyDef.position.y = 4.0f * cast(f32, Y) + height + 1.0f;
                    bodyDef.position.z = -40.0f + 4.0f * cast(f32, Z);
                    b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
                    ignore b3CreateHullShape(bodyId, &shapeDef, rockHull);
                }
            }
        }

        b3DestroyHull(rockHull);
    }

    g_junkyard_radius = 35.0f;
    f32 mHeight = 24.0f;

    b3HullData* hull = b3CreateCylinder(mHeight, 4.0f, 0.0f, 16);
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_kinematicBody;
    bodyDef.position = b3Pos{g_junkyard_radius, 0.0f, 0.0f};
    g_junkyard_pusher = b3CreateBody(g_world, &bodyDef);
    g_junkyard_degrees = 0.0f;
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    ignore b3CreateHullShape(g_junkyard_pusher, &shapeDef, hull);
    b3DestroyHull(hull);
}

void step_junkyard_world() {
    f32 timeStep = 1.0f / 60.0f;
    const f32 omega = -6.0f;
    g_junkyard_degrees += omega * timeStep;
    b3CosSin cs = b3ComputeCosSin(g_junkyard_degrees * PI_F / 180.0f);
    f32 r = g_junkyard_radius;
    b3Pos targetPos = b3Pos{r * cs.cosine, 0.0f, r * cs.sine};
    b3WorldTransform target = b3WorldTransform{targetPos, b3Quat_identity};
    b3Body_SetTargetTransform(g_junkyard_pusher, target, timeStep, false);
}

// samples/sample_benchmark.cpp BenchmarkJunkyard
void build_junkyard() {
    g_dbg_joints = false;
    create_junkyard();
}

void step_junkyard(f32 timeStep) {
    ignore timeStep;
    if g_pause == false || g_single_step == 0 {
        step_junkyard_world();
    }
}

// Huge pile of large convexes, ported from PEEL. Each convex is the hull of 32 random points on a
// sphere. A fixed LCG seed makes the hull identical across runs so results compare directly.

void get_convex_pile_capacity(b3Capacity* capacity) {
    capacity.dynamicShapeCount = 5120;
    capacity.dynamicBodyCount = 5120;
    capacity.contactCount = 50 * 1024;
}

// PEEL's BasicRandom, kept verbatim so the point set matches the original
u32 g_cp_rng_state;

u32 next_convex_pile_random() {
    g_cp_rng_state = g_cp_rng_state * cast(u32, 2147001325) + cast(u32, 715136305);
    return g_cp_rng_state;
}

// Float in [-0.5, 0.5]
f32 convex_pile_random_float() {
    return cast(f32, next_convex_pile_random() & cast(u32, 65535)) / 65535.0f - 0.5f;
}

// Uniform random direction, rejection sampled inside the unit sphere then pushed to the surface
b3Vec3 unit_random_point() {
    b3Vec3 point;
    f32 lengthSq;
    while true {
        point.x = convex_pile_random_float();
        point.y = convex_pile_random_float();
        point.z = convex_pile_random_float();
        lengthSq = b3Dot(point, point);
        if lengthSq <= 0.25f { break; }
    }

    return b3Normalize(point);
}

void create_convex_pile() {
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{0.0f, -1.0f, 0.0f};
        b3BodyId groundId = b3CreateBody(g_world, &bodyDef);

        b3BoxHull box = b3MakeBoxHull(250.0f, 1.0f, 250.0f);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        set_ground_shape(b3CreateHullShape(groundId, &shapeDef, &box.base));
    }

    i32 countX = 8;
    i32 countZ = 8;
    i32 layers = 80;
    f32 amplitude = 2.0f;
    i32 pointCount = 32;
    f32 scatter = 2.0f * amplitude;

    // Hull around random points on a sphere of radius amplitude
    b3Vec3[64] points;
    g_cp_rng_state = 42;
    for i32 i = 0; i < pointCount; i += 1 {
        points[i] = b3MulSV(amplitude, unit_random_point());
    }

    b3HullData* convex = b3CreateHull(cast(b3Vec3*, &points), pointCount, pointCount);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    b3ShapeDef shapeDef = b3DefaultShapeDef();

    // Grid tall enough to collapse into a pile
    for i32 layer = 0; layer < layers; layer += 1 {
        for i32 z = 0; z < countZ; z += 1 {
            for i32 x = 0; x < countX; x += 1 {
                f32 posX = (cast(f32, x) - 0.5f * cast(f32, countX)) * scatter;
                f32 posZ = (cast(f32, z) - 0.5f * cast(f32, countZ)) * scatter;
                f32 posY = amplitude + 2.0f * amplitude * cast(f32, layer);

                bodyDef.position = b3Pos{posX, posY, posZ};
                b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
                ignore b3CreateHullShape(bodyId, &shapeDef, convex);
            }
        }
    }

    b3DestroyHull(convex);
}

// samples/sample_benchmark.cpp BenchmarkConvexPile
void build_convex_pile() {
    create_convex_pile();
}

// samples/sample_benchmark.cpp CandyCups
b3HullData* g_cc_convex;

b3HullData* candy_cups_create_convex(f32 radius1, f32 height1, f32 radius2, f32 height2) {
    const i32 sideCount = 8;
    const f32 deltaAlpha = 2.0f * PI_F / cast(f32, sideCount);

    i32 vertexCount = 2 * sideCount;
    b3Vec3[2 * sideCount] vertexBase;

    f32 alpha = 0.0f;
    for i32 sideIndex = 0; sideIndex < sideCount; sideIndex += 1 {
        b3CosSin cs = b3ComputeCosSin(alpha);

        f32 x1 = radius1 * cs.cosine;
        f32 z1 = radius1 * cs.sine;
        f32 x2 = radius2 * cs.cosine;
        f32 z2 = radius2 * cs.sine;

        vertexBase[2 * sideIndex + 0] = b3Vec3{x1, height1, z1};
        vertexBase[2 * sideIndex + 1] = b3Vec3{x2, height2, z2};
        alpha += deltaAlpha;
    }

    return b3CreateHull(cast(b3Vec3*, &vertexBase), vertexCount, vertexCount);
}

void build_candy_cups() {
    ignore add_ground_box(60.0f);

    {
        const i32 n = 16;
        const i32 m = 16;

        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        g_cc_convex = candy_cups_create_convex(0.6f, 0.0f, 0.95f, 1.0f);
        for i32 i = 0; i < n; i += 1 {
            for i32 j = 0; j < m; j += 1 {
                for i32 k = 0; k < m; k += 1 {
                    bodyDef.position = b3Pos{-10.0f + 2.5f * cast(f32, j),
                                             1.0f * cast(f32, i),
                                             -10.0f + 2.5f * cast(f32, k)};
                    b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
                    ignore b3CreateHullShape(bodyId, &shapeDef, g_cc_convex);
                }
            }
        }
    }
}

void destroy_candy_cups() {
    b3DestroyHull(g_cc_convex);
}

// samples/sample_benchmark.cpp BenchmarkSensor
struct ShapeUserData {
    i32 row;
    bool active;
}

const i32 BS_COLUMN_COUNT = 40;
const i32 BS_ROW_COUNT = 40;
i32 g_bs_max_begin_count;
i32 g_bs_max_end_count;
ShapeUserData[BS_ROW_COUNT] g_bs_passive_sensors;
ShapeUserData g_bs_active_sensor;
i32 g_bs_last_step_count;
i32 g_bs_filter_row;

bool benchmark_sensor_filter(b3ShapeId idA, b3ShapeId idB, void* context) {
    ignore context;
    ShapeUserData* userData = null;
    if b3Shape_IsSensor(idA) {
        userData = cast(ShapeUserData*, b3Shape_GetUserData(idA));
    } else if b3Shape_IsSensor(idB) {
        userData = cast(ShapeUserData*, b3Shape_GetUserData(idB));
    }

    if userData != null {
        return userData.active == true || userData.row != g_bs_filter_row;
    }

    return true;
}

void benchmark_sensor_create_row(f32 y) {
    f32 shift = 5.0f;
    f32 xCenter = 0.5f * shift * cast(f32, BS_COLUMN_COUNT);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.gravityScale = 0.0f;
    bodyDef.linearVelocity = b3Vec3{0.0f, -5.0f, 0.0f};

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.enableSensorEvents = true;

    b3Sphere sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, 0.5f};
    for i32 i = 0; i < BS_COLUMN_COUNT; i += 1 {
        // stagger bodies to avoid bunching up events into a single update
        f32 yOffset = random_float_range(-1.0f, 1.0f);
        bodyDef.position = b3Pos{shift * cast(f32, i) - xCenter, y + yOffset, 0.0f};
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);

        ignore b3CreateSphereShape(bodyId, &shapeDef, &sphere);
    }
}

void build_benchmark_sensor() {
    b3World_SetCustomFilterCallback(g_world, benchmark_sensor_filter, null);

    g_bs_active_sensor.row = 0;
    g_bs_active_sensor.active = true;

    {
        f32 gridSize = 3.0f;

        // These destroy anything they touch, including themselves.
        b3BoxHull box = b3MakeCubeHull(0.48f * gridSize);
        b3BodyDef bodyDef = b3DefaultBodyDef();

        b3ShapeDef shapeDef = b3DefaultShapeDef();
        shapeDef.isSensor = true;
        shapeDef.enableSensorEvents = true;
        shapeDef.userData = cast(void*, &g_bs_active_sensor);
        shapeDef.baseMaterial.customColor = cast(b3HexColor,
            b3MakeDebugColor(cast(b3HexColor, 0x505050), b3_debugMaterialMetallic));

        f32 y = 0.0f;
        f32 x = -40.0f * gridSize;
        for i32 i = 0; i < 81; i += 1 {
            bodyDef.position = b3Pos{x, y, 0.0f};
            b3BodyId groundId = b3CreateBody(g_world, &bodyDef);
            ignore b3CreateHullShape(groundId, &shapeDef, &box.base);
            x += gridSize;
        }
    }

    {
        g_randomSeed = 42;

        f32 shift = 5.0f;
        f32 xCenter = 0.5f * shift * cast(f32, BS_COLUMN_COUNT);

        b3BodyDef bodyDef = b3DefaultBodyDef();

        b3BoxHull box = b3MakeCubeHull(0.5f);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        shapeDef.isSensor = true;
        shapeDef.enableSensorEvents = true;

        f32 yStart = 10.0f;
        g_bs_filter_row = BS_ROW_COUNT >> 1;

        for i32 j = 0; j < BS_ROW_COUNT; j += 1 {
            g_bs_passive_sensors[j].row = j;
            g_bs_passive_sensors[j].active = false;
            shapeDef.userData = cast(void*, &g_bs_passive_sensors[j]);

            if j == g_bs_filter_row {
                shapeDef.enableCustomFiltering = true;
                shapeDef.baseMaterial.customColor = b3_colorFuchsia;
            } else {
                shapeDef.enableCustomFiltering = false;
                shapeDef.baseMaterial.customColor = cast(b3HexColor, 0);
            }

            f32 y = cast(f32, j) * shift + yStart;
            for i32 i = 0; i < BS_COLUMN_COUNT; i += 1 {
                f32 x = cast(f32, i) * shift - xCenter;
                bodyDef.position = b3Pos{x, y, 0.0f};
                b3BodyId groundId = b3CreateBody(g_world, &bodyDef);
                ignore b3CreateHullShape(groundId, &shapeDef, &box.base);
            }
        }
    }

    g_bs_max_begin_count = 0;
    g_bs_max_end_count = 0;
    g_bs_last_step_count = 0;
}

const i32 BS_ZOMBIE_MAX = 1024;
b3BodyId[BS_ZOMBIE_MAX] g_bs_zombies;

void step_benchmark_sensor(f32 timeStep) {
    ignore timeStep;
    if g_step_count == g_bs_last_step_count { return; }

    // a std::set upstream; a linear scan dedups the same way at this size
    i32 zombieCount = 0;

    b3SensorEvents events = b3World_GetSensorEvents(g_world);
    for i32 i = 0; i < events.beginCount; i += 1 {
        b3SensorBeginTouchEvent* event = events.beginEvents + i;

        // shapes on begin touch are always valid

        ShapeUserData* userData = cast(ShapeUserData*, b3Shape_GetUserData(event.sensorShapeId));
        if userData.active {
            b3BodyId victim = b3Shape_GetBody(event.visitorShapeId);
            bool seen = false;
            for i32 z = 0; z < zombieCount; z += 1 {
                if g_bs_zombies[z].index1 == victim.index1
                    && g_bs_zombies[z].world0 == victim.world0
                    && g_bs_zombies[z].generation == victim.generation {
                    seen = true;
                    break;
                }
            }
            if !seen && zombieCount < BS_ZOMBIE_MAX {
                g_bs_zombies[zombieCount] = victim;
                zombieCount += 1;
            }
        } else {
            // Modify color while overlapped with a sensor
            b3SurfaceMaterial surfaceMaterial = b3Shape_GetSurfaceMaterial(event.visitorShapeId);
            surfaceMaterial.customColor = b3_colorLime;
            b3Shape_SetSurfaceMaterial(event.visitorShapeId, surfaceMaterial);
        }
    }

    for i32 i = 0; i < events.endCount; i += 1 {
        b3SensorEndTouchEvent* event = events.endEvents + i;

        if b3Shape_IsValid(event.visitorShapeId) == false { continue; }

        // Restore color to default
        b3SurfaceMaterial surfaceMaterial = b3Shape_GetSurfaceMaterial(event.visitorShapeId);
        surfaceMaterial.customColor = cast(b3HexColor, 0);
        b3Shape_SetSurfaceMaterial(event.visitorShapeId, surfaceMaterial);
    }

    for i32 i = 0; i < zombieCount; i += 1 {
        b3DestroyBody(g_bs_zombies[i]);
    }

    i32 delay = 31;

    if (g_step_count & delay) == 0 {
        benchmark_sensor_create_row(10.0f + cast(f32, BS_ROW_COUNT) * 5.0f);
    }

    g_bs_last_step_count = g_step_count;

    g_bs_max_begin_count = b3MaxInt(events.beginCount, g_bs_max_begin_count);
    g_bs_max_end_count = b3MaxInt(events.endCount, g_bs_max_end_count);
    u8[128] buf;
    ignore snprintf(cast(u8*, &buf), 128, "max begin touch events = %d", g_bs_max_begin_count);
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 128, "max end touch events = %d", g_bs_max_end_count);
    draw_text_line(cast(u8*, &buf));
}

// samples/sample_benchmark.cpp BenchmarkHeightField
b3HeightFieldData* g_hfb_field;
i32 g_hfb_column_count;
i32 g_hfb_row_count;
f32 g_hfb_radius;

// the cast callback's result, upstream's Context
b3Pos g_hfb_point;
b3Vec3 g_hfb_normal;
f32 g_hfb_fraction;
bool g_hfb_hit;

f32 height_field_cast_callback(b3ShapeId shapeId, b3Pos point, b3Vec3 normal, f32 fraction,
                               u64 userMaterialId, i32 triangleIndex, i32 childIndex,
                               void* context) {
    ignore shapeId; ignore userMaterialId; ignore triangleIndex; ignore childIndex;
    ignore context;
    g_hfb_point = point;
    g_hfb_normal = normal;
    g_hfb_fraction = fraction;
    g_hfb_hit = true;
    return fraction;
}

void build_benchmark_height_field() {
    g_hfb_column_count = 50;
    g_hfb_row_count = 50;
    g_hfb_radius = 0.1f;

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.position = b3Pos{-0.5f * cast(f32, g_hfb_column_count), 0.0f,
                             -0.5f * cast(f32, g_hfb_row_count)};
    b3BodyId body = b3CreateBody(g_world, &bodyDef);

    g_hfb_field = b3CreateWave(50, 50, b3Vec3_one, 0.02f, 0.04f, true);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    ignore b3CreateHeightFieldShape(body, &shapeDef, g_hfb_field);
}

void destroy_benchmark_height_field() {
    b3DestroyHeightField(g_hfb_field);
}

bool benchmark_height_field_controls() {
    ignore ImGui_SliderFloat("Radius", &g_hfb_radius, 0.0f, 1.0f, "%.1f", 0);
    return true;
}

void step_benchmark_height_field(f32 timeStep) {
    ignore timeStep;

    dbg_line(b3Pos_zero, b3OffsetPos(b3Pos_zero, b3MulSV(0.4f, b3Vec3_axisX)), b3_colorRed);
    dbg_line(b3Pos_zero, b3OffsetPos(b3Pos_zero, b3MulSV(0.4f, b3Vec3_axisY)), b3_colorGreen);
    dbg_line(b3Pos_zero, b3OffsetPos(b3Pos_zero, b3MulSV(0.4f, b3Vec3_axisZ)), b3_colorBlue);

    i32 hitCount = 0;
    i32 iterationCount = 0;
    i32 innerIterationCount = 0;

    f32 delta = 0.4f;

    f32 spanX = 0.94f * 0.5f * cast(f32, g_hfb_column_count);
    f32 spanZ = 0.96f * 0.5f * cast(f32, g_hfb_row_count);

    u64 startTick = b3GetTicks();

    b3Vec3 rayTranslation = b3Vec3{80000.0f, -80000.0f, 8.0f};
    i32 castCount = 0;

    f32 x = 0.0f - spanX;
    while x <= spanX {
        f32 z = 0.0f - spanZ;
        while z <= spanZ {
            b3Pos rayOrigin = b3Pos{x, 2.0f, z};

            bool hit = false;
            if g_hfb_radius == 0.0f {
                b3RayResult result = b3World_CastRayClosest(g_world, rayOrigin, rayTranslation,
                                                            b3DefaultQueryFilter());
                hit = result.hit;
            } else {
                g_hfb_hit = false;
                b3Vec3 proxyPoint = b3Vec3_zero;
                b3ShapeProxy proxy = b3ShapeProxy{&proxyPoint, 1, g_hfb_radius};
                ignore b3World_CastShape(g_world, rayOrigin, &proxy, rayTranslation,
                                         b3DefaultQueryFilter(), height_field_cast_callback, null);
                hit = g_hfb_hit;
            }

            castCount += 1;
            if hit { hitCount += 1; }
            z += delta;
        }
        x += delta;
    }

    f32 milliseconds = b3GetMilliseconds(startTick);
    u64 tickCount = b3GetTicks() - startTick;
    f32 aveIterations = cast(f32, iterationCount) / cast(f32, castCount);
    f32 aveInner = cast(f32, innerIterationCount) / cast(f32, castCount);
    f32 aveCastTime = 1000.0f * milliseconds / cast(f32, castCount);

    u8[224] buf;
    ignore snprintf(cast(u8*, &buf), 224,
                    "count = %d, hit count = %d, iterations = %d, inner = %d, ticks = %lld",
                    castCount, hitCount, iterationCount, innerIterationCount, tickCount);
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 224,
                    "ave iterations = %.1f, ave inner = %.1f, ave cast us %.3f",
                    aveIterations, aveInner, aveCastTime);
    draw_text_line(cast(u8*, &buf));
}

// samples/sample_benchmark.cpp BenchmarkExplosion
b3MeshData* g_bex_grid_mesh;
b3HullData* g_bex_cylinder;
f32 g_bex_impulse;

void build_benchmark_explosion() {
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    g_bex_grid_mesh = b3CreateGridMesh(40, 40, 1.0f, 0, true);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    b3BodyId groundId = b3CreateBody(g_world, &bodyDef);
    ignore b3CreateMeshShape(groundId, &shapeDef, g_bex_grid_mesh, b3Vec3_one);

    f32 hy = 1.0f;

    {
        b3Transform transform;
        transform.p = b3Vec3{0.0f, hy, -20.0f};
        transform.q = b3Quat_identity;
        b3BoxHull wallBox = b3MakeTransformedBoxHull(20.0f, hy, 0.1f, transform);
        ignore b3CreateHullShape(groundId, &shapeDef, &wallBox.base);
    }
    {
        b3Transform transform;
        transform.p = b3Vec3{0.0f, hy, 20.0f};
        transform.q = b3Quat_identity;
        b3BoxHull wallBox = b3MakeTransformedBoxHull(20.0f, hy, 0.1f, transform);
        ignore b3CreateHullShape(groundId, &shapeDef, &wallBox.base);
    }
    {
        b3Transform transform;
        transform.p = b3Vec3{-20.0f, hy, 0.0f};
        transform.q = b3Quat_identity;
        b3BoxHull wallBox = b3MakeTransformedBoxHull(0.1f, hy, 20.0f, transform);
        ignore b3CreateHullShape(groundId, &shapeDef, &wallBox.base);
    }
    {
        b3Transform transform;
        transform.p = b3Vec3{20.0f, hy, 0.0f};
        transform.q = b3Quat_identity;
        b3BoxHull wallBox = b3MakeTransformedBoxHull(0.1f, hy, 20.0f, transform);
        ignore b3CreateHullShape(groundId, &shapeDef, &wallBox.base);
    }

    g_bex_cylinder = b3CreateCylinder(0.5f, 0.2f, 0.0f, 15);

    const i32 n = 16;

    bodyDef.type = b3_dynamicBody;
    shapeDef.explosionScale = 2.0f;

    for i32 i = -n; i <= n; i += 1 {
        for i32 k = -n; k <= n; k += 1 {
            bodyDef.position = b3Pos{1.0f * cast(f32, i), 0.0f, 1.0f * cast(f32, k)};
            b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
            ignore b3CreateHullShape(bodyId, &shapeDef, g_bex_cylinder);
        }
    }

    g_bex_impulse = 1000.0f;
}

void destroy_benchmark_explosion() {
    b3DestroyHull(g_bex_cylinder);
    b3DestroyMesh(g_bex_grid_mesh);
}

void benchmark_explosion_explode() {
    b3ExplosionDef def = b3DefaultExplosionDef();
    def.radius = 16.0f;
    def.position = b3Pos{0.0f, -4.0f, 0.0f};
    def.impulsePerArea = g_bex_impulse;
    b3World_Explode(g_world, &def);
}

bool benchmark_explosion_controls() {
    ignore ImGui_SliderFloat("Magnitude", &g_bex_impulse, 0.0f, 2000.0f, "%.0f", 0);
    if ImGui_Button("Explode", ImVec2{0.0f, 0.0f}) {
        benchmark_explosion_explode();
    }
    return true;
}

// samples/sample_benchmark.cpp BenchmarkChains
const i32 CHAINS_GRID_COUNT = 25;
b3ShapeId[CHAINS_GRID_COUNT * CHAINS_GRID_COUNT] g_chain_shapes;
b3MeshData* g_chain_mesh;
b3Vec3 g_chain_noise;

void build_benchmark_chains() {
    b3BodyDef bodyDef = b3DefaultBodyDef();
    b3BodyId groundId = b3CreateBody(g_world, &bodyDef);

    g_chain_mesh = b3CreateWaveMesh(80, 80, 1.0f, 0.5f, 0.05f, 0.01f);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    ignore b3CreateMeshShape(groundId, &shapeDef, g_chain_mesh, b3Vec3_one);

    f32 linkRadius = 0.125f;
    f32 linkExtent = 0.25f;
    b3Capsule capsule = b3Capsule{b3Pos{0.0f, -linkExtent, 0.0f}, b3Pos{0.0f, linkExtent, 0.0f},
                                  linkRadius};

    bodyDef.enableSleep = false;

    i32 linkCount = 4;

    b3SphericalJointDef jointDef = b3DefaultSphericalJointDef();
    jointDef.base.localFrameA = b3Transform{b3Vec3{0.0f, -linkExtent, 0.0f}, b3Quat_identity};
    jointDef.base.localFrameB = b3Transform{b3Vec3{0.0f, linkExtent, 0.0f}, b3Quat_identity};
    jointDef.enableSpring = true;
    jointDef.hertz = 1.0f;
    jointDef.dampingRatio = 0.7f;
    jointDef.enableMotor = true;
    jointDef.maxMotorTorque = 1.0f;

    i32 shapeIndex = 0;

    f32 x = -1.0f * cast(f32, CHAINS_GRID_COUNT);
    for i32 rowIndex = 0; rowIndex < CHAINS_GRID_COUNT; rowIndex += 1 {
        f32 z = -1.0f * cast(f32, CHAINS_GRID_COUNT);
        for i32 columnIndex = 0; columnIndex < CHAINS_GRID_COUNT; columnIndex += 1 {
            for i32 i = 0; i < linkCount; i += 1 {
                bodyDef.position = b3Pos{x, (1.0f - 2.0f * cast(f32, i)) * linkExtent + 3.0f, z};
                bodyDef.type = i == 0 ? b3_staticBody : b3_dynamicBody;

                b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);

                b3ShapeId shapeId = b3CreateCapsuleShape(bodyId, &shapeDef, &capsule);
                if i == linkCount - 1 {
                    g_chain_shapes[shapeIndex] = shapeId;
                    shapeIndex += 1;
                }

                if i > 0 {
                    jointDef.base.bodyIdB = bodyId;
                    ignore b3CreateSphericalJoint(g_world, &jointDef);
                }

                jointDef.base.bodyIdA = bodyId;
            }
            z += 2.0f;
        }
        x += 2.0f;
    }

    g_chain_noise = b3Vec3{0.0f, 0.0f, 0.0f};
}

void destroy_benchmark_chains() {
    b3DestroyMesh(g_chain_mesh);
}

void step_benchmark_chains(f32 timeStep) {
    ignore timeStep;
    b3Vec3 baseWind = b3Vec3{20.0f, 0.0f, 0.0f};
    f32 speed;
    b3Vec3 direction = b3GetLengthAndNormalize(&speed, baseWind);
    b3Vec3 wind = b3MulSV(speed, b3Add(direction, g_chain_noise));

    for i32 i = 0; i < CHAINS_GRID_COUNT * CHAINS_GRID_COUNT; i += 1 {
        b3Shape_ApplyWind(g_chain_shapes[i], wind, 1.0f, 1.0f, 20.0f, false);
    }

    b3Vec3 rand = random_vec3(b3Vec3{-0.3f, -0.3f, -0.3f}, b3Vec3{0.3f, 0.3f, 0.3f});
    g_chain_noise = b3Lerp(g_chain_noise, rand, 0.05f);
}

// shared/benchmarks.c CreateTrees
b3MeshData* g_tree_mesh;

void create_trees(i32 scale) {
    f32 tilt = 0.0f * PI_F;
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.position = b3Pos{0.0f, 0.0f, 0.0f};
    bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3{1.0f, 0.0f, 0.0f}, tilt);
    b3BodyId groundId = b3CreateBody(g_world, &bodyDef);

    i32 xCount = scale * 150;
    i32 zCount = scale * 200;

    f32 cellWidth = 1.0f / cast(f32, scale);
    f32 amplitude = 0.4f;
    f32 rowHz = 0.05f;
    f32 columnHz = 0.1f;

    g_tree_mesh = b3CreateWaveMesh(xCount, zCount, cellWidth, amplitude, rowHz, columnHz);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    ignore b3CreateMeshShape(groundId, &shapeDef, g_tree_mesh, b3Vec3_one);

    bodyDef.type = b3_dynamicBody;
    bodyDef.sleepThreshold = 0.2f;
    bodyDef.rotation = b3Quat_identity;

    i32 bodyCount = 50;

    shapeDef.baseMaterial.friction = 0.9f;
    shapeDef.baseMaterial.rollingResistance = 0.05f;
    shapeDef.updateBodyMass = false;
    shapeDef.density = 1.0f;

    const i32 hullCount = 22;
    b3HullData*[hullCount] hulls;

    f32 y = 1.0f;
    f32 r = 0.75f;
    f32 l = 1.5f;
    for i32 i = 0; i < hullCount; i += 1 {
        hulls[i] = b3CreateCylinder(l + 2.0f * r, r, y - r, 6);
        y += l + 2.0f * r;
        r = 0.95f * r;
    }

    f32 angularVelocity = -0.5f;
    f32 z = -70.0f;
    b3CosSin cs = b3ComputeCosSin(tilt);
    f32 yTilt = cs.sine / cs.cosine;
    for i32 bodyIndex = 0; bodyIndex < bodyCount; bodyIndex += 1 {
        bodyDef.position = b3Pos{0.0f, 1.0f - z * yTilt, z};
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);

        for i32 shapeIndex = 0; shapeIndex < 22; shapeIndex += 1 {
            ignore b3CreateHullShape(bodyId, &shapeDef, hulls[shapeIndex]);
        }

        f32 velocityScale = 0.5f + (0.5f * cast(f32, bodyIndex)) / cast(f32, bodyCount);
        b3Body_ApplyMassFromShapes(bodyId);
        b3Pos center = b3Body_GetWorldCenter(bodyId);
        b3Vec3 omega = b3Vec3{0.0f, 0.0f, velocityScale * angularVelocity};
        b3Vec3 v = b3Cross(omega, b3SubPos(center, bodyDef.position));
        b3Body_SetAngularVelocity(bodyId, omega);
        b3Body_SetLinearVelocity(bodyId, v);

        z += 3.0f;
        angularVelocity = 0.0f - angularVelocity;
    }

    for i32 i = 0; i < hullCount; i += 1 {
        b3DestroyHull(hulls[i]);
    }
}

// samples/sample_benchmark.cpp BenchmarkFallingTrees
// The radio buttons rebuild the scene keeping the choice; a fresh load
// starts at 100cm, as upstream's constructor does. Same idiom as
// FarStack's presets (g_fs_keep).
i32 g_trees_grid_size;
bool g_trees_keep;

void build_falling_trees() {
    if !g_trees_keep { g_trees_grid_size = 100; }
    g_trees_keep = false;

    i32 scale = 1;
    if g_trees_grid_size == 50 { scale = 2; }
    else if g_trees_grid_size == 25 { scale = 4; }
    create_trees(scale);
}

void destroy_falling_trees() {
    b3DestroyMesh(g_tree_mesh);
}

bool falling_trees_controls() {
    // Upstream rebuilds the world from DrawControls; ours rebuilds the
    // sample, which is the same thing through the framework.
    if ImGui_RadioButton("100cm", g_trees_grid_size == 100) {
        g_trees_grid_size = 100; g_reset_pending = true; g_trees_keep = true;
    }
    if ImGui_RadioButton("50cm", g_trees_grid_size == 50) {
        g_trees_grid_size = 50; g_reset_pending = true; g_trees_keep = true;
    }
    if ImGui_RadioButton("25cm", g_trees_grid_size == 25) {
        g_trees_grid_size = 25; g_reset_pending = true; g_trees_keep = true;
    }
    return true;
}

// samples/sample_benchmark.cpp BenchmarkDestruction
const i32 BD_GRID_COUNT = 20;
const f32 BD_EXTENT = 2.5f;
const i32 BD_BODY_CAPACITY = BD_GRID_COUNT * BD_GRID_COUNT * BD_GRID_COUNT;

b3BodyId[BD_BODY_CAPACITY] g_bd_bodies;
i32 g_bd_body_count;
b3ExplosionDef g_bd_explosion;
b3MeshData* g_bd_grid_mesh;
f32 g_bd_spawn_ms;
f32 g_bd_destroy_ms;

void get_destruction_capacity(b3Capacity* capacity) {
    capacity.dynamicShapeCount = BD_BODY_CAPACITY;
    capacity.dynamicBodyCount = BD_BODY_CAPACITY;
    capacity.contactCount = 50000;
}

void bd_destroy_bodies() {
    u64 ticks = b3GetTicks();

    for i32 i = 0; i < g_bd_body_count; i += 1 {
        b3DestroyBody(g_bd_bodies[i]);
    }

    g_bd_destroy_ms = b3GetMilliseconds(ticks);
}

void bd_spawn() {
    u64 ticks = b3GetTicks();

    f32 a = BD_EXTENT / cast(f32, BD_GRID_COUNT);
    b3BoxHull box = b3MakeBoxHull(0.8f * a, 0.8f * a, 0.8f * a);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;

    b3ShapeDef shapeDef = b3DefaultShapeDef();

    i32 randomRange = 2;
    g_bd_body_count = 0;
    for i32 i = 0; i < BD_GRID_COUNT; i += 1 {
        for i32 j = 0; j < BD_GRID_COUNT; j += 1 {
            for i32 k = 0; k < BD_GRID_COUNT; k += 1 {
                // upstream RandomIntRange( 1, randomRange )
                if 1 + random_int() % randomRange == 1 { continue; }

                bodyDef.position = b3Pos{
                    (2.0f * cast(f32, i) - cast(f32, BD_GRID_COUNT) + 1.0f) * a,
                    (2.0f * cast(f32, j) + 1.0f) * a,
                    (2.0f * cast(f32, k) - cast(f32, BD_GRID_COUNT) + 1.0f) * a};

                b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
                ignore b3CreateHullShape(bodyId, &shapeDef, &box.base);

                g_bd_bodies[g_bd_body_count] = bodyId;
                g_bd_body_count += 1;
            }
        }
    }

    b3World_Explode(g_world, &g_bd_explosion);

    g_bd_spawn_ms = b3GetMilliseconds(ticks);
}

void build_destruction() {
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    g_bd_grid_mesh = b3CreateGridMesh(40, 40, 1.0f, 0, true);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    b3BodyId groundId = b3CreateBody(g_world, &bodyDef);
    ignore b3CreateMeshShape(groundId, &shapeDef, g_bd_grid_mesh, b3Vec3_one);

    g_bd_body_count = 0;

    g_bd_explosion = b3DefaultExplosionDef();
    g_bd_explosion.radius = BD_EXTENT;
    g_bd_explosion.falloff = 0.5f * BD_EXTENT;
    g_bd_explosion.position = b3Pos{0.0f, 2.0f * BD_EXTENT, 0.0f};
    g_bd_explosion.impulsePerArea = 1000.0f;

    g_bd_spawn_ms = 0.0f;
    g_bd_destroy_ms = 0.0f;

    bd_spawn();
}

void destroy_destruction() {
    b3DestroyMesh(g_bd_grid_mesh);
}

void step_destruction(f32 timeStep) {
    ignore timeStep;

    u8[128] buf;
    ignore snprintf(cast(u8*, &buf), 128, "spawn = %.2f ms", g_bd_spawn_ms);
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 128, "destroy = %.2f ms", g_bd_destroy_ms);
    draw_text_line(cast(u8*, &buf));

    f32 r = g_bd_explosion.radius;
    b3Sphere sphere1 = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, r};
    dbg_wire_sphere(b3Transform{b3Vec3{g_bd_explosion.position.x, g_bd_explosion.position.y,
                                       g_bd_explosion.position.z}, b3Quat_identity},
                    &sphere1, 24, b3_colorAqua);

    f32 rf = r + g_bd_explosion.falloff;
    b3Sphere sphere2 = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, rf};
    dbg_wire_sphere(b3Transform{b3Vec3{g_bd_explosion.position.x, g_bd_explosion.position.y,
                                       g_bd_explosion.position.z}, b3Quat_identity},
                    &sphere2, 24, b3_colorCornsilk);

    i32 spawnStep = 140;
    if g_step_count % spawnStep == 0 {
        bd_destroy_bodies();
        bd_spawn();
    }
}

// samples/sample_benchmark.cpp BenchmarkRain
void get_rain_capacity_fn(b3Capacity* capacity) {
    GetRainCapacity(capacity);
}

void build_benchmark_rain() {
    g_dbg_joints = false;
    CreateRain(g_world);
}

void destroy_benchmark_rain() {
    DestroyRain();
}

void step_benchmark_rain(f32 timeStep) {
    ignore timeStep;
    StepRain(g_world, g_step_count);

    // This is for testing adjustable worker count
    // if (m_stepCount == 200)
    //{
    //	m_scheduler->Initialize( 3 );
    //	b3World_SetWorkerCount( m_worldId, 3 );
    //}

    b3Transform t = b3Transform{b3Vec3{0.0f, 0.1f, 0.0f}, b3Quat_identity};
    dbg_axes(b3MakeWorldTransform(t), 2.0f);
}

// samples/sample_determinism.cpp FallingRagdolls
FallingRagdollData g_fd_data;
bool g_fd_done;

void build_falling_ragdolls() {
    g_fd_data = CreateFallingRagdolls(g_world);
    g_fd_done = false;
}

void destroy_falling_ragdolls() {
    DestroyFallingRagdolls(&g_fd_data);
}

void step_falling_ragdolls(f32 timeStep) {
    u8[128] buf;
    if g_fd_done == false {
        // Only advance the scenario on real steps, else pausing would skew
        // the step count. upstream m_didStep.
        if timeStep > 0.0f {
            g_fd_done = UpdateFallingRagdolls(g_world, &g_fd_data);
            if g_fd_done {
                ignore snprintf(cast(u8*, &buf), 128, "sleep step = %d, hash = 0x%08X",
                                g_fd_data.sleepStep, g_fd_data.hash);
                eprint("{}\n", str_from_cstr(cast(u8*, &buf)));
            }
        }
    } else {
        ignore snprintf(cast(u8*, &buf), 128, "sleep step = %d, hash = 0x%08X",
                        g_fd_data.sleepStep, g_fd_data.hash);
        draw_text_line(cast(u8*, &buf));
    }
}
