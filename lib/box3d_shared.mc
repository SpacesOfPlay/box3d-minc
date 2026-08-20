// box3d_shared.mc — scene builders box3d's samples share
// (shared/benchmarks.c, determinism.c, stability.c, overflow_color.c),
// transpiled.
//
import box3d;
import box3d_human;
import math;

// zero-init global
b3ShapeId b3_nullShapeId;
// transminc: C #define values surfaced as compile-time configuration
@define "B3_HASH_INIT" 5381

type errno_t = i32;
struct Group {
    Human[3] humans;
}

struct RainData {
    Group* groups;
    b3MeshData* gridMesh;
    b3MeshData* torusMesh;
    i32 columnCount;
    i32 columnIndex;
}

// Static Floor: a huge grid of static box bodies with b3ShapeDef::invokeContactCreation = true,
// plus a small number of dynamic spheres that drop onto it staggered over time. The point of this
// benchmark is to exercise the b3BroadPhase move buffer at steady state when it was once populated
// with ~staticShapeCount entries and now sees only a handful of dynamic moves per step.
struct StaticFloorData {
    i32 spheresDropped;
}

struct g_treeData_t {
    b3MeshData* meshData;
}

struct JunkyardData {
    b3BodyId pusherId;
    f32 degrees;
    f32 radius;
}

// PEEL's BasicRandom, kept verbatim so the point set matches the original
struct ConvexPileRandom {
    u32 state;
}

// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
struct RagdollGroup {
    Human[2] humans;
}

struct FallingRagdollData {
    RagdollGroup[4] groups;
    b3MeshData* gridMesh;
    b3MeshData* torusMesh;
    i32 columnCount;
    i32 columnIndex;
    i32 stepCount;
    i32 sleepStep;
    u32 hash;
}

// Convex pile dropped on a wave height field. Spheres, capsules, boxes, and rocks with
// rolling resistance so the pile sleeps quickly.
struct WavePileData {
    b3BodyId[100] bodies;
    b3HeightFieldData* heightField;
    i32 stepCount;
    i32 sleepStep;
    u32 hash;
}

// Query driven spawning in an empty zero gravity world. Each step casts a ray, overlaps an
// AABB, and casts a sphere, then spawns a shape whose position, type, and size depend on the
// query results. Any query divergence cascades into the final state.
struct QuerySpawnData {
    b3BodyId[50] bodies;
    i32 spawnCount;
    i32 queryHitCount;
    u32 queryHash;
    i32 stepCount;
    i32 sleepStep;
    u32 hash;
    b3Pos rayOrigin;
    b3Vec3 rayTranslation;
    b3Pos rayPoint;
    b3Vec3 rayNormal;
    bool rayDidHit;
    b3AABB overlapBounds;
    i32 overlapCount;
    f32 castFraction;
    b3Pos lastSpawnPosition;
}

struct QuerySpawnOverlapContext {
    QuerySpawnData* data;
    i32 count;
}

// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Thin fast boxes dropped on a wave mesh. Stresses continuous collision and mesh contact
// stability, and doubles as a determinism scenario via the sleep hash.
struct MeshDropData {
    b3MeshData* mesh;
    b3BodyId[400] bodies;
    i32 stepCount;
    i32 sleepStep;
    u32 hash;
}

// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// One heavy dynamic hub surrounded by enough dynamic neighbors that the hub's
// degree in the dyn-dyn contact graph exceeds B3_DYNAMIC_COLOR_COUNT (= 20).
// The excess contacts land in the overflow color (B3_GRAPH_COLOR_COUNT - 1),
// exercising the b3*_Overflow solver path that no other scene reaches.
struct OverflowColorPileData {
    b3ShapeId groundShapeId;
    b3BodyId hubId;
    i32 neighborCount;
}

private { b3ShapeId g_groundShapeId; }

b3ShapeId GetGroundShapeId() {
    return g_groundShapeId;
}

void ResetGroundShapeId() {
    g_groundShapeId = b3_nullShapeId;
}

void CreateJointGrid(b3WorldId worldId) {
    b3World_EnableSleeping(worldId, false);
    i32 n = 0 != 0 ? 10 : 100;
    var bodies = new(b3BodyId[n * n]);
    i32 index = 0;
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.filter.categoryBits = 2;
    shapeDef.filter.maskBits = cast(u64, ~2);
    var sphere = b3Sphere{b3Vec3{0.0f, 0.0f, 0.0f}, 0.4f};
    b3SphericalJointDef jointDef = b3DefaultSphericalJointDef();
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.enableSleep = false;
    for i32 k = 0; k < n; ++k {
        for i32 i = 0; i < n; ++i {
            var fk = cast(f32, k);
            var fi = cast(f32, i);
            if i == 0 {
                bodyDef.type = b3_staticBody;
            } else {
                bodyDef.type = b3_dynamicBody;
            }
            bodyDef.position = b3Pos{fk, -fi, 0.0f};
            b3BodyId body = b3CreateBody(worldId, &bodyDef);
            b3CreateSphereShape(body, &shapeDef, &sphere);
            if i > 0 {
                jointDef.base.bodyIdA = bodies[index - 1];
                jointDef.base.bodyIdB = body;
                jointDef.base.localFrameA.p = b3Vec3{0.0f, -0.5f, 0.0f};
                jointDef.base.localFrameB.p = b3Vec3{0.0f, 0.5f, 0.0f};
                b3CreateSphericalJoint(worldId, &jointDef);
            }
            if k > 0 {
                jointDef.base.bodyIdA = bodies[index - n];
                jointDef.base.bodyIdB = body;
                jointDef.base.localFrameA.p = b3Vec3{0.5f, 0.0f, 0.0f};
                jointDef.base.localFrameB.p = b3Vec3{-0.5f, 0.0f, 0.0f};
                b3CreateSphericalJoint(worldId, &jointDef);
            }
            bodies[index++] = body;
        }
    }
    free(bodies);
}

// The 80 block version falls over after 1000 steps.
void CreateLargePyramid(b3WorldId worldId) {
    b3World_EnableSleeping(worldId, false);
    i32 baseCount = 0 != 0 ? 20 : 100;
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{0.0f, -1.0f, 0.0f};
        b3BodyId groundId = b3CreateBody(worldId, &bodyDef);
        b3BoxHull box = b3MakeBoxHull(400.0f, 1.0f, 400.0f);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        g_groundShapeId = b3CreateHullShape(groundId, &shapeDef, &box.base);
    }
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.density = 100.0f;
    f32 h = 0.5f;
    b3BoxHull box = b3MakeBoxHull(h, h, h);
    f32 shift = 1.0f * h;
    for i32 i = 0; i < baseCount; ++i {
        f32 y = (2.0f * cast(f32, i) + 1.0f) * shift;
        for i32 j = i; j < baseCount; ++j {
            f32 x = (cast(f32, i) + 1.0f) * shift + 2.0f * cast(f32, j - i) * shift - h * cast(f32, baseCount);
            bodyDef.position = b3Pos{x, y, 0.0f};
            b3BodyId bodyId = b3CreateBody(worldId, &bodyDef);
            b3CreateHullShape(bodyId, &shapeDef, &box.base);
        }
    }
}

void CreateWidePyramid(b3WorldId worldId) {
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{0.0f, -1.0f, 0.0f};
        b3BodyId groundId = b3CreateBody(worldId, &bodyDef);
        b3BoxHull box = b3MakeBoxHull(100.0f, 1.0f, 100.0f);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        g_groundShapeId = b3CreateHullShape(groundId, &shapeDef, &box.base);
    }
    f32 boxSize = 2.0f;
    f32 boxSeparation = 0.5f;
    f32 halfBoxSize = 0.5f * boxSize;
    i32 pyramidHeight = 0 != 0 ? 5 : 15;
    var h = cast(f32, halfBoxSize - 0.025);
    b3BoxHull box = b3MakeBoxHull(h, h, h);
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    for i32 i = 0; i < pyramidHeight; ++i {
        for i32 j = i / 2; j < pyramidHeight - (i + 1) / 2; ++j {
            for i32 k = i / 2; k < pyramidHeight - (i + 1) / 2; ++k {
                f32 x = cast(f32, -pyramidHeight) + boxSize * cast(f32, j) + ((i & 1) != 0 ? halfBoxSize : 0.0f);
                f32 y = 1.0f + (boxSize + boxSeparation) * cast(f32, i);
                f32 z = cast(f32, -pyramidHeight) + boxSize * cast(f32, k) + ((i & 1) != 0 ? halfBoxSize : 0.0f);
                bodyDef.position = b3Pos{x, y, z};
                b3BodyId bodyId = b3CreateBody(worldId, &bodyDef);
                b3CreateHullShape(bodyId, &shapeDef, &box.base);
            }
        }
    }
}

private {
void CreateSmallPyramid(b3WorldId worldId, i32 baseCount, f32 extent, f32 centerX, f32 baseZ) {
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.enableSleep = false;
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.density = 100.0f;
    b3BoxHull box = b3MakeBoxHull(extent, extent, extent);
    for i32 i = 0; i < baseCount; ++i {
        f32 y = (2.0f * cast(f32, i) + 1.0f) * extent;
        for i32 j = i; j < baseCount; ++j {
            f32 x = (cast(f32, i) + 1.0f) * extent + 2.0f * cast(f32, j - i) * extent + centerX - 0.5f;
            bodyDef.position = b3Pos{x, y, baseZ};
            b3BodyId bodyId = b3CreateBody(worldId, &bodyDef);
            b3CreateHullShape(bodyId, &shapeDef, &box.base);
        }
    }
}
}

void CreateManyPyramids(b3WorldId worldId) {
    i32 baseCount = 10;
    f32 extent = 0.5f;
    i32 rowCount = 0 != 0 ? 3 : 14;
    i32 columnCount = 0 != 0 ? 3 : 14;
    f32 groundExtent = extent * cast(f32, columnCount) * (cast(f32, baseCount) + 1.0f);
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{0.0f, -1.0f, 0.0f};
        b3BodyId groundId = b3CreateBody(worldId, &bodyDef);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        b3BoxHull box = b3MakeBoxHull(groundExtent, 1.0f, groundExtent);
        g_groundShapeId = b3CreateHullShape(groundId, &shapeDef, &box.base);
    }
    f32 baseWidth = 2.0f * extent * cast(f32, baseCount);
    f32 baseZ = -groundExtent + 2.0f * extent;
    f32 deltaZ = 2.0f * (groundExtent - 2.0f * extent) / (cast(f32, rowCount) - 1.0f);
    for i32 i = 0; i < rowCount; ++i {
        for i32 j = 0; j < columnCount; ++j {
            f32 centerX = -groundExtent + cast(f32, j) * (baseWidth + 2.0f * extent) + 2.0f * extent;
            CreateSmallPyramid(worldId, baseCount, extent, centerX, baseZ);
        }
        baseZ += deltaZ;
    }
}
RainData g_rainData;

void GetRainCapacity(b3Capacity* capacity) {
}

void CreateRain(b3WorldId worldId) {
    memset(&g_rainData, 0, cast(u64, sizeof(g_rainData)));
    g_rainData.groups = new(Group[10 * 10]);
    memset(g_rainData.groups, 0, cast(u64, 10 * 10 * sizeof(Group)));
    i32 halfMeshGridRows = 4;
    f32 meshGridCellWidth = 15.0f / (2.0f * cast(f32, halfMeshGridRows));
    g_rainData.gridMesh = b3CreateGridMesh(2 * halfMeshGridRows, 2 * halfMeshGridRows, meshGridCellWidth, 1, true);
    g_rainData.torusMesh = b3CreateTorusMesh(16, 16, 0.25f * 15.0f, 1.0f);
    f32 span = 15.0f * 10.0f;
    b3BodyDef bodyDef = b3DefaultBodyDef();
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    bodyDef.position.x = -0.5f * span + 0.5f * 15.0f;
    for i32 i = 0; i < 10; ++i {
        bodyDef.position.z = -0.5f * span + 0.5f * 15.0f;
        for i32 j = 0; j < 10; ++j {
            b3BodyId body = b3CreateBody(worldId, &bodyDef);
            b3CreateMeshShape(body, &shapeDef, g_rainData.gridMesh, b3Vec3_one);
            b3CreateMeshShape(body, &shapeDef, g_rainData.torusMesh, b3Vec3_one);
            bodyDef.position.z += 15.0f;
        }
        bodyDef.position.x += 15.0f;
    }
}

void DestroyRain() {
    b3DestroyMesh(g_rainData.gridMesh);
    b3DestroyMesh(g_rainData.torusMesh);
    free(g_rainData.groups);
    g_rainData.groups = null;
}

void CreateGroup(b3WorldId worldId, i32 rowIndex, i32 columnIndex) {
    i32 groupIndex = rowIndex * 10 + columnIndex;
    f32 span = 10.0f * 15.0f;
    f32 groupDistance = 1.0f * span / 10.0f;
    noinit b3Pos position;
    position.x = -0.5f * span + groupDistance * (cast(f32, columnIndex) + 0.5f);
    position.y = 20.0f;
    position.z = -0.5f * span + groupDistance * (cast(f32, rowIndex) + 0.5f);
    f32 frictionTorque = 5.0f;
    f32 hertz = 1.0f;
    f32 dampingRatio = 0.7f;
    bool colorize = false;
    for i32 i = 0; i < 3; ++i {
        Human* human = g_rainData.groups[groupIndex].humans + i;
        CreateHuman(human, worldId, position, frictionTorque, hertz, dampingRatio, groupIndex, null, colorize);
        position.x += 0.75f;
    }
}

void DestroyGroup(i32 rowIndex, i32 columnIndex) {
    i32 groupIndex = rowIndex * 10 + columnIndex;
    for i32 i = 0; i < 3; ++i {
        DestroyHuman(g_rainData.groups[groupIndex].humans + i);
    }
}

void StepRain(b3WorldId worldId, i32 stepCount) {
    i32 delay = 0 != 0 ? 0x7F : 0x2F;
    i32 increment = 0 != 0 ? 100 : 1;
    if (stepCount & delay) == 0 {
        if g_rainData.columnCount < 10 {
            for i32 i = 0; i < 10; i += increment {
                CreateGroup(worldId, i, g_rainData.columnCount);
            }
            g_rainData.columnCount = b3MinInt(g_rainData.columnCount + increment, 10);
        } else {
            for i32 i = 0; i < 10; i += increment {
                DestroyGroup(i, g_rainData.columnIndex);
                CreateGroup(worldId, i, g_rainData.columnIndex);
            }
            g_rainData.columnIndex = g_rainData.columnIndex + increment;
            if g_rainData.columnIndex >= 10 {
                g_rainData.columnIndex = 0;
            }
        }
    }
}
private { StaticFloorData g_staticFloorData; }

void GetLargeWorldCapacity(b3Capacity* capacity) {
    i32 floorCount = 1000 * 1000;
    capacity.staticShapeCount = floorCount;
    capacity.staticBodyCount = floorCount;
    capacity.dynamicShapeCount = 100;
    capacity.dynamicBodyCount = 100;
    capacity.contactCount = b3MaxInt(1024, 8 * 100);
}

void CreateLargeWorld(b3WorldId worldId) {
    memset(&g_staticFloorData, 0, cast(u64, sizeof(g_staticFloorData)));
    f32 cell = 10.0f;
    i32 gridCount = 1000;
    f32 halfSpan = 0.5f * cell * cast(f32, gridCount);
    b3BoxHull box = b3MakeBoxHull(0.5f * cell, 0.25f, 0.5f * cell);
    b3BodyDef bodyDef = b3DefaultBodyDef();
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.invokeContactCreation = true;
    for i32 i = 0; i < gridCount; ++i {
        f32 x = -halfSpan + (cast(f32, i) + 0.5f) * cell;
        for i32 j = 0; j < gridCount; ++j {
            f32 z = -halfSpan + (cast(f32, j) + 0.5f) * cell;
            bodyDef.position = b3Pos{x, 0.0f, z};
            b3BodyId body = b3CreateBody(worldId, &bodyDef);
            b3CreateHullShape(body, &shapeDef, &box.base);
        }
    }
}

void StepLargeWorld(b3WorldId worldId, i32 stepCount) {
    if g_staticFloorData.spheresDropped >= 100 {
        return;
    }
    if stepCount == 0 {
        return;
    }
    if stepCount % 5 != 0 {
        return;
    }
    i32 side = 1;
    while side * side < 100 {
        side += 1;
    }
    i32 idx = g_staticFloorData.spheresDropped;
    i32 gi = idx % side;
    i32 gj = idx / side;
    f32 halfSpan = 0.5f * 10.0f * 1000.0f;
    f32 inset = 0.1f * 2.0f * halfSpan;
    f32 usable = 2.0f * halfSpan - 2.0f * inset;
    f32 x = -halfSpan + inset + (cast(f32, gi) + 0.5f) * (usable / cast(f32, side));
    f32 z = -halfSpan + inset + (cast(f32, gj) + 0.5f) * (usable / cast(f32, side));
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = b3Pos{x, 1.5f, z};
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    var sphere = b3Sphere{b3Vec3{0.0f, 0.0f, 0.0f}, 0.5f};
    b3BodyId body = b3CreateBody(worldId, &bodyDef);
    b3CreateSphereShape(body, &shapeDef, &sphere);
    g_staticFloorData.spheresDropped += 1;
}

void GetWasherCapacity(b3Capacity* capacity) {
    capacity.staticShapeCount = 16;
    capacity.dynamicShapeCount = 10000;
    capacity.staticBodyCount = 16;
    capacity.dynamicBodyCount = 10000;
    capacity.contactCount = 60000;
}

void CreateWasher(b3WorldId worldId) {
    bool kinematic = true;
    noinit b3BodyId groundId;
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position.y = -1.0f;
        groundId = b3CreateBody(worldId, &bodyDef);
        b3BoxHull box = b3MakeBoxHull(60.0f, 1.0f, 60.0f);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        g_groundShapeId = b3CreateHullShape(groundId, &shapeDef, &box.base);
    }
    {
        f32 motorSpeed = 25.0f;
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{0.0f, 21.0f, 0.0f};
        if kinematic == true {
            bodyDef.type = b3_kinematicBody;
            bodyDef.angularVelocity = b3Vec3{0.0f, 0.0f, 3.14159265359f / 180.0f * motorSpeed};
            bodyDef.linearVelocity = b3Vec3{0.001f, -0.002f, 0.0f};
        } else {
            bodyDef.type = b3_dynamicBody;
        }
        b3BodyId bodyId = b3CreateBody(worldId, &bodyDef);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        f32 r0 = 14.0f;
        f32 r1 = 16.0f;
        f32 r2 = 18.0f;
        var nd = b3Vec3{0.0f, 0.0f, -10.0f};
        var pd = b3Vec3{0.0f, 0.0f, 10.0f};
        f32 angle = 3.14159265359f / 18.0f;
        b3Quat q = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, angle);
        b3Quat qo = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, 0.1f * angle);
        var u1 = b3Vec3{1.0f, 0.0f, 0.0f};
        for i32 i = 0; i < 36; ++i {
            noinit b3Vec3 u2;
            if i == 35 {
                u2 = b3Vec3{1.0f, 0.0f, 0.0f};
            } else {
                u2 = b3RotateVector(q, u1);
            }
            {
                b3Vec3 a1 = b3InvRotateVector(qo, u1);
                b3Vec3 a2 = b3RotateVector(qo, u2);
                b3Vec3 p1 = b3MulAdd(nd, r1, a1);
                b3Vec3 p2 = b3MulAdd(nd, r2, a1);
                b3Vec3 p3 = b3MulAdd(nd, r1, a2);
                b3Vec3 p4 = b3MulAdd(nd, r2, a2);
                b3Vec3 p5 = b3MulAdd(pd, r1, a1);
                b3Vec3 p6 = b3MulAdd(pd, r2, a1);
                b3Vec3 p7 = b3MulAdd(pd, r1, a2);
                b3Vec3 p8 = b3MulAdd(pd, r2, a2);
                b3Vec3[8] points = {p1, p2, p3, p4, p5, p6, p7, p8};
                b3HullData* hull = b3CreateHull(points, 8, 8);
                b3CreateHullShape(bodyId, &shapeDef, hull);
                b3DestroyHull(hull);
            }
            if i % 9 == 0 {
                b3Vec3 p1 = b3MulAdd(nd, r0, u1);
                b3Vec3 p2 = b3MulAdd(nd, r1, u1);
                b3Vec3 p3 = b3MulAdd(nd, r0, u2);
                b3Vec3 p4 = b3MulAdd(nd, r1, u2);
                b3Vec3 p5 = b3MulAdd(pd, r0, u1);
                b3Vec3 p6 = b3MulAdd(pd, r1, u1);
                b3Vec3 p7 = b3MulAdd(pd, r0, u2);
                b3Vec3 p8 = b3MulAdd(pd, r1, u2);
                b3Vec3[8] points = {p1, p2, p3, p4, p5, p6, p7, p8};
                b3HullData* hull = b3CreateHull(points, 8, 8);
                b3CreateHullShape(bodyId, &shapeDef, hull);
                b3DestroyHull(hull);
            }
            u1 = u2;
        }
        if kinematic == false {
            b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
            jointDef.base.bodyIdA = groundId;
            jointDef.base.bodyIdB = bodyId;
            jointDef.base.localFrameA.p.y = 10.0f;
            jointDef.motorSpeed = 3.14159265359f / 180.0f * motorSpeed;
            jointDef.maxMotorTorque = 100000000.0f;
            jointDef.enableMotor = true;
            b3CreateRevoluteJoint(worldId, &jointDef);
        }
    }
    i32 gridCount = 0 != 0 ? 8 : 20;
    f32 a = 0.2f;
    b3BoxHull cube = b3MakeBoxHull(a, a, a);
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    f32 x = -2.0f * a * cast(f32, gridCount);
    for i32 i = 0; i < gridCount; ++i {
        f32 y = -2.0f * a * cast(f32, gridCount) + 21.0f;
        for i32 j = 0; j < gridCount; ++j {
            f32 z = -2.0f * a * cast(f32, gridCount);
            for i32 k = 0; k < gridCount; ++k {
                bodyDef.position = b3Pos{x, y, z};
                b3BodyId bodyId = b3CreateBody(worldId, &bodyDef);
                b3CreateHullShape(bodyId, &shapeDef, &cube.base);
                z += 4.0f * a;
            }
            y += 4.0f * a;
        }
        x += 4.0f * a;
    }
}
g_treeData_t g_treeData;

private {
void CreateTrees(b3WorldId worldId, i32 scale) {
    memset(&g_treeData, 0, cast(u64, sizeof(g_treeData)));
    f32 tilt = 0.0f * 3.14159265359f;
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.position = b3Pos{0.0f, 0.0f, 0.0f};
    bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3{1.0f, 0.0f, 0.0f}, tilt);
    b3BodyId groundId = b3CreateBody(worldId, &bodyDef);
    i32 xCount = scale * 150;
    i32 zCount = scale * 200;
    f32 cellWidth = 1.0f / cast(f32, scale);
    f32 amplitude = 0.4f;
    f32 rowHz = 0.05f;
    f32 columnHz = 0.1f;
    g_treeData.meshData = b3CreateWaveMesh(xCount, zCount, cellWidth, amplitude, rowHz, columnHz);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    b3CreateMeshShape(groundId, &shapeDef, g_treeData.meshData, b3Vec3_one);
    bodyDef.type = b3_dynamicBody;
    bodyDef.sleepThreshold = 0.2f;
    bodyDef.rotation = b3Quat_identity;
    i32 bodyCount = 0 != 0 ? 10 : 50;
    shapeDef.baseMaterial.friction = 0.9f;
    shapeDef.baseMaterial.rollingResistance = 0.05f;
    shapeDef.updateBodyMass = false;
    shapeDef.density = 1.0f;
    i32 hullCount = 22;
    b3HullData*[22] hulls;
    f32 y = 1.0f;
    f32 r = 0.75f;
    f32 l = 1.5f;
    for i32 i = 0; i < hullCount; ++i {
        hulls[i] = b3CreateCylinder(l + 2.0f * r, r, y - r, 6);
        y += l + 2.0f * r;
        r = 0.95f * r;
    }
    f32 angularVelocity = -0.5f;
    f32 z = 0 != 0 ? -15.0f : -70.0f;
    b3CosSin cs = b3ComputeCosSin(tilt);
    f32 yTilt = cs.sine / cs.cosine;
    for i32 bodyIndex = 0; bodyIndex < bodyCount; ++bodyIndex {
        bodyDef.position = b3Pos{0.0f, 1.0f - z * yTilt, z};
        b3BodyId bodyId = b3CreateBody(worldId, &bodyDef);
        for i32 shapeIndex = 0; shapeIndex < 22; ++shapeIndex {
            b3CreateHullShape(bodyId, &shapeDef, hulls[shapeIndex]);
        }
        f32 velocityScale = 0.5f + 0.5f * cast(f32, bodyIndex) / cast(f32, bodyCount);
        b3Body_ApplyMassFromShapes(bodyId);
        b3Pos center = b3Body_GetWorldCenter(bodyId);
        var omega = b3Vec3{0.0f, 0.0f, velocityScale * angularVelocity};
        b3Vec3 v = b3Cross(omega, b3SubPos(center, bodyDef.position));
        b3Body_SetAngularVelocity(bodyId, omega);
        b3Body_SetLinearVelocity(bodyId, v);
        z += 3.0f;
        angularVelocity = -angularVelocity;
    }
    for i32 i = 0; i < hullCount; ++i {
        b3DestroyHull(hulls[i]);
    }
}
}

void CreateTrees25(b3WorldId worldId) {
    CreateTrees(worldId, 4);
}

void CreateTrees50(b3WorldId worldId) {
    CreateTrees(worldId, 2);
}

void CreateTrees100(b3WorldId worldId) {
    CreateTrees(worldId, 1);
}

void DestroyTrees() {
    b3DestroyMesh(g_treeData.meshData);
    memset(&g_treeData, 0, cast(u64, sizeof(g_treeData)));
}
JunkyardData g_junkyardData;

void CreateJunkyard(b3WorldId worldId) {
    noinit b3BodyId groundId;
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position.y = -1.0f;
        groundId = b3CreateBody(worldId, &bodyDef);
    }
    {
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        {
            b3BoxHull box = b3MakeBoxHull(120.0f, 1.0f, 120.0f);
            g_groundShapeId = b3CreateHullShape(groundId, &shapeDef, &box.base);
        }
        {
            var offset = b3Vec3{-50.0f, 8.0f, 0.0f};
            b3BoxHull box = b3MakeOffsetBoxHull(1.0f, 8.0f, 50.0f, offset);
            b3CreateHullShape(groundId, &shapeDef, &box.base);
        }
        {
            var offset = b3Vec3{50.0f, 8.0f, 0.0f};
            b3BoxHull box = b3MakeOffsetBoxHull(1.0f, 8.0f, 50.0f, offset);
            b3CreateHullShape(groundId, &shapeDef, &box.base);
        }
        {
            var offset = b3Vec3{0.0f, 8.0f, -50.0f};
            b3BoxHull box = b3MakeOffsetBoxHull(50.0f, 8.0f, 1.0f, offset);
            b3CreateHullShape(groundId, &shapeDef, &box.base);
        }
        {
            var offset = b3Vec3{0.0f, 8.0f, 50.0f};
            b3BoxHull box = b3MakeOffsetBoxHull(50.0f, 8.0f, 1.0f, offset);
            b3CreateHullShape(groundId, &shapeDef, &box.base);
        }
    }
    {
        b3HullData* rockHull = b3CreateRock(1.5f);
        i32 count = 0 != 0 ? 2 : 24;
        f32 height = 24.0f;
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        for i32 Y = 0; Y < count; ++Y {
            for i32 X = 0; X <= 20; ++X {
                for i32 Z = 0; Z <= 20; ++Z {
                    bodyDef.position.x = -40.0f + 4.0f * cast(f32, X);
                    bodyDef.position.y = 4.0f * cast(f32, Y) + height + 1.0f;
                    bodyDef.position.z = -40.0f + 4.0f * cast(f32, Z);
                    b3BodyId bodyId = b3CreateBody(worldId, &bodyDef);
                    b3CreateHullShape(bodyId, &shapeDef, rockHull);
                }
            }
        }
        b3DestroyHull(rockHull);
    }
    g_junkyardData.radius = 35.0f;
    f32 mHeight = 24.0f;
    b3HullData* hull = b3CreateCylinder(mHeight, 4.0f, 0.0f, 16);
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_kinematicBody;
    bodyDef.position = b3Pos{g_junkyardData.radius, 0.0f, 0.0f};
    g_junkyardData.pusherId = b3CreateBody(worldId, &bodyDef);
    g_junkyardData.degrees = 0.0f;
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    b3CreateHullShape(g_junkyardData.pusherId, &shapeDef, hull);
    b3DestroyHull(hull);
}

void GetJunkyardCapacity(b3Capacity* capacity) {
    capacity.staticShapeCount = 16;
    capacity.dynamicShapeCount = 20 * 20 * 24 + 1;
    capacity.staticBodyCount = 16;
    capacity.dynamicBodyCount = 20 * 20 * 24 + 1;
    capacity.contactCount = 250 * 1024;
}

void StepJunkyard(b3WorldId worldId, i32 stepCount) {
    ignore worldId;
    ignore stepCount;
    f32 timeStep = 1.0f / 60.0f;
    f32 omega = -6.0f;
    g_junkyardData.degrees += omega * timeStep;
    b3CosSin cs = b3ComputeCosSin(g_junkyardData.degrees * 3.14159265359f / 180.0f);
    f32 r = g_junkyardData.radius;
    var targetPos = b3Pos{r * cs.cosine, 0.0f, r * cs.sine};
    var target = b3WorldTransform{.p = targetPos, .q = b3Quat_identity};
    b3Body_SetTargetTransform(g_junkyardData.pusherId, target, timeStep, false);
}

// Huge pile of large convexes, ported from PEEL. Each convex is the hull of 32 random points on a
// sphere. A fixed LCG seed makes the hull identical across runs so results compare directly.
void GetConvexPileCapacity(b3Capacity* capacity) {
    capacity.dynamicShapeCount = 5120;
    capacity.dynamicBodyCount = 5120;
    capacity.contactCount = 50 * 1024;
}

private {
u32 NextConvexPileRandom(ConvexPileRandom* rng) {
    rng.state = rng.state * 2147001325 + 715136305;
    return rng.state;
}

// Float in [-0.5, 0.5]
f32 ConvexPileRandomFloat(ConvexPileRandom* rng) {
    return cast(f32, NextConvexPileRandom(rng) & 0xffff) / 65535.0f - 0.5f;
}

// Uniform random direction, rejection sampled inside the unit sphere then pushed to the surface
b3Vec3 UnitRandomPoint(ConvexPileRandom* rng) {
    noinit b3Vec3 point;
    f32 lengthSq;
    while true {
        point.x = ConvexPileRandomFloat(rng);
        point.y = ConvexPileRandomFloat(rng);
        point.z = ConvexPileRandomFloat(rng);
        lengthSq = b3Dot(point, point);
        if !(lengthSq > 0.25f) { break; }
    }
    return b3Normalize(point);
}
}

void CreateConvexPile(b3WorldId worldId) {
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{0.0f, -1.0f, 0.0f};
        b3BodyId groundId = b3CreateBody(worldId, &bodyDef);
        b3BoxHull box = b3MakeBoxHull(250.0f, 1.0f, 250.0f);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        g_groundShapeId = b3CreateHullShape(groundId, &shapeDef, &box.base);
    }
    i32 countX = 8;
    i32 countZ = 8;
    i32 layers = 0 != 0 ? 10 : 80;
    f32 amplitude = 2.0f;
    i32 pointCount = 32;
    f32 scatter = 2.0f * amplitude;
    noinit b3Vec3[64] points;
    var rng = ConvexPileRandom{42};
    for i32 i = 0; i < pointCount; ++i {
        points[i] = b3MulSV(amplitude, UnitRandomPoint(&rng));
    }
    b3HullData* convex = b3CreateHull(points, pointCount, pointCount);
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    for i32 layer = 0; layer < layers; ++layer {
        for i32 z = 0; z < countZ; ++z {
            for i32 x = 0; x < countX; ++x {
                f32 posX = (cast(f32, x) - 0.5f * cast(f32, countX)) * scatter;
                f32 posZ = (cast(f32, z) - 0.5f * cast(f32, countZ)) * scatter;
                f32 posY = amplitude + 2.0f * amplitude * cast(f32, layer);
                bodyDef.position = b3Pos{posX, posY, posZ};
                b3BodyId bodyId = b3CreateBody(worldId, &bodyDef);
                b3CreateHullShape(bodyId, &shapeDef, convex);
            }
        }
    }
    b3DestroyHull(convex);
}

private {
void CreateGroup(FallingRagdollData* data, b3WorldId worldId, i32 rowIndex, i32 columnIndex) {
    i32 groupIndex = rowIndex * 2 + columnIndex;
    f32 span = 2.0f * 15.0f;
    f32 groupDistance = 1.0f * span / 2.0f;
    noinit b3Pos position;
    position.x = -0.5f * span + groupDistance * (cast(f32, columnIndex) + 0.5f);
    position.y = 15.0f;
    position.z = -0.5f * span + groupDistance * (cast(f32, rowIndex) + 0.5f);
    f32 frictionTorque = 5.0f;
    f32 hertz = 1.0f;
    f32 dampingRatio = 0.7f;
    bool colorize = false;
    for i32 i = 0; i < 2; ++i {
        Human* human = data.groups[groupIndex].humans + i;
        CreateHuman(human, worldId, position, frictionTorque, hertz, dampingRatio, groupIndex, null, colorize);
        position.x += 0.75f;
    }
}
}

FallingRagdollData CreateFallingRagdolls(b3WorldId worldId) {
    FallingRagdollData data;
    i32 halfMeshGridRows = 4;
    f32 meshGridCellWidth = 15.0f / (2.0f * cast(f32, halfMeshGridRows));
    data.gridMesh = b3CreateGridMesh(2 * halfMeshGridRows, 2 * halfMeshGridRows, meshGridCellWidth, 0, true);
    data.torusMesh = b3CreateTorusMesh(16, 16, 0.25f * 15.0f, 1.0f);
    f32 span = 15.0f * 2.0f;
    b3BodyDef bodyDef = b3DefaultBodyDef();
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    bodyDef.position.x = -0.5f * span + 0.5f * 15.0f;
    for i32 i = 0; i < 2; ++i {
        bodyDef.position.z = -0.5f * span + 0.5f * 15.0f;
        for i32 j = 0; j < 2; ++j {
            b3BodyId body = b3CreateBody(worldId, &bodyDef);
            b3CreateMeshShape(body, &shapeDef, data.gridMesh, b3Vec3_one);
            b3CreateMeshShape(body, &shapeDef, data.torusMesh, b3Vec3_one);
            CreateGroup(&data, worldId, i, j);
            bodyDef.position.z += 15.0f;
        }
        bodyDef.position.x += 15.0f;
    }
    return data;
}

bool UpdateFallingRagdolls(b3WorldId worldId, FallingRagdollData* data) {
    if data.hash == 0 {
        b3BodyEvents bodyEvents = b3World_GetBodyEvents(worldId);
        if bodyEvents.moveCount == 0 {
            unused i32 awakeCount = b3World_GetAwakeBodyCount(worldId);
            data.hash = B3_HASH_INIT;
            for i32 i = 0; i < 2; ++i {
                for i32 j = 0; j < 2; ++j {
                    for i32 k = 0; k < 2; ++k {
                        i32 groupIndex = i * 2 + j;
                        Human* human = data.groups[groupIndex].humans + k;
                        for i32 b = 0; b < bone_count; ++b {
                            b3BodyId bodyId = human.bones[b].bodyId;
                            b3WorldTransform xf = b3Body_GetTransform(bodyId);
                            data.hash = b3Hash(data.hash, cast(u8*, &xf), cast(i32, sizeof(b3WorldTransform)));
                        }
                    }
                }
            }
            data.sleepStep = data.stepCount;
        }
    }
    data.stepCount += 1;
    return data.hash != 0;
}

void DestroyFallingRagdolls(FallingRagdollData* data) {
    b3DestroyMesh(data.gridMesh);
    b3DestroyMesh(data.torusMesh);
    data.gridMesh = null;
    data.torusMesh = null;
}

WavePileData CreateWavePile(b3WorldId worldId) {
    WavePileData data;
    g_randomSeed = 52977;
    i32 fieldCount = 21;
    var fieldScale = b3Vec3{1.0f, 0.6f, 1.0f};
    data.heightField = b3CreateWave(fieldCount, fieldCount, fieldScale, 0.08f, 0.06f, false);
    {
        f32 extent = fieldScale.x * cast(f32, fieldCount - 1);
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position.x = -0.5f * extent;
        bodyDef.position.z = -0.5f * extent;
        b3BodyId groundId = b3CreateBody(worldId, &bodyDef);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        b3CreateHeightFieldShape(groundId, &shapeDef, data.heightField);
    }
    b3HullData* rock = b3CreateRock(0.55f);
    b3BoxHull box = b3MakeBoxHull(0.45f, 0.3f, 0.55f);
    var sphere = b3Sphere{b3Vec3_zero, 0.5f};
    var capsule = b3Capsule{b3Vec3{0.0f, -0.3f, 0.0f}, b3Vec3{0.0f, 0.3f, 0.0f}, 0.35f};
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.baseMaterial.rollingResistance = 0.3f;
    f32 spacing = 1.7f;
    i32 index = 0;
    for i32 layer = 0; layer < 4; ++layer {
        for i32 i = 0; i < 5; ++i {
            for i32 j = 0; j < 5; ++j {
                b3Vec3 jitter = RandomVec3Uniform(-0.3f, 0.3f);
                bodyDef.position.x = spacing * (cast(f32, i) - 0.5f * cast(f32, 5 - 1)) + jitter.x;
                bodyDef.position.y = 2.5f + 1.6f * cast(f32, layer) + 0.3f * jitter.y;
                bodyDef.position.z = spacing * (cast(f32, j) - 0.5f * cast(f32, 5 - 1)) + jitter.z;
                bodyDef.rotation = RandomQuat();
                b3BodyId bodyId = b3CreateBody(worldId, &bodyDef);
                data.bodies[index] = bodyId;
                switch index % 4 {
                    case 0: {
                        b3CreateSphereShape(bodyId, &shapeDef, &sphere);
                    }
                    case 1: {
                        b3CreateCapsuleShape(bodyId, &shapeDef, &capsule);
                    }
                    case 2: {
                        b3CreateHullShape(bodyId, &shapeDef, &box.base);
                    }
                    default: {
                        b3CreateHullShape(bodyId, &shapeDef, rock);
                    }
                }
                index += 1;
            }
        }
    }
    b3DestroyHull(rock);
    return data;
}

bool UpdateWavePile(b3WorldId worldId, WavePileData* data) {
    if data.hash == 0 {
        if b3World_GetAwakeBodyCount(worldId) == 0 {
            data.hash = B3_HASH_INIT;
            for i32 i = 0; i < 100; ++i {
                b3WorldTransform xf = b3Body_GetTransform(data.bodies[i]);
                data.hash = b3Hash(data.hash, cast(u8*, &xf), cast(i32, sizeof(b3WorldTransform)));
            }
            data.sleepStep = data.stepCount;
        }
    }
    data.stepCount += 1;
    return data.hash != 0;
}

void DestroyWavePile(WavePileData* data) {
    b3DestroyHeightField(data.heightField);
    data.heightField = null;
}

private {
bool QuerySpawnOverlapCallback(b3ShapeId shapeId, void* context) {
    QuerySpawnOverlapContext* overlap = context;
    overlap.count += 1;
    overlap.data.queryHitCount += 1;
    overlap.data.queryHash = b3Hash(overlap.data.queryHash, cast(u8*, &shapeId.index1), cast(i32, sizeof(shapeId.index1)));
    return true;
}

f32 QuerySpawnCastCallback(b3ShapeId shapeId, b3Pos point, b3Vec3 normal, f32 fraction, u64 userMaterialId, i32 triangleIndex, i32 childIndex, void* context) {
    ignore shapeId;
    ignore point;
    ignore normal;
    ignore userMaterialId;
    ignore triangleIndex;
    ignore childIndex;
    f32* closest = context;
    *closest = fraction;
    return fraction;
}

void QuerySpawnOnce(b3WorldId worldId, QuerySpawnData* data) {
    b3QueryFilter filter = b3DefaultQueryFilter();
    b3Pos rayOrigin = RandomPos(b3Vec3{-12.0f, -12.0f, -12.0f}, b3Vec3{12.0f, 12.0f, 12.0f});
    b3Vec3 rayTranslation = b3MulSV(30.0f, RandomUnitVector());
    noinit b3Pos spawnPosition;
    b3RayResult ray = b3World_CastRayClosest(worldId, rayOrigin, rayTranslation, filter);
    if ray.hit != 0 {
        data.queryHitCount += 1;
        data.queryHash = b3Hash(data.queryHash, cast(u8*, &ray.fraction), cast(i32, sizeof(ray.fraction)));
        data.queryHash = b3Hash(data.queryHash, cast(u8*, &ray.normal), cast(i32, sizeof(ray.normal)));
        spawnPosition = b3OffsetPos(ray.point, b3MulSV(1.2f, ray.normal));
    } else {
        spawnPosition = RandomPos(b3Vec3{-6.0f, -6.0f, -6.0f}, b3Vec3{6.0f, 6.0f, 6.0f});
    }
    data.rayOrigin = rayOrigin;
    data.rayTranslation = rayTranslation;
    data.rayDidHit = ray.hit;
    data.rayPoint = ray.hit != 0 ? ray.point : b3OffsetPos(rayOrigin, rayTranslation);
    data.rayNormal = ray.hit != 0 ? ray.normal : b3Vec3_zero;
    b3Vec3 center = RandomVec3Uniform(-10.0f, 10.0f);
    f32 extent = RandomFloatRange(1.0f, 4.0f);
    noinit b3AABB aabb;
    aabb.lowerBound = b3Vec3{center.x - extent, center.y - extent, center.z - extent};
    aabb.upperBound = b3Vec3{center.x + extent, center.y + extent, center.z + extent};
    var overlap = QuerySpawnOverlapContext{data, 0};
    b3World_OverlapAABB(worldId, aabb, filter, cast(b3OverlapResultFcn, QuerySpawnOverlapCallback), &overlap);
    data.overlapBounds = aabb;
    data.overlapCount = overlap.count;
    f32 fraction = 1.0f;
    b3Vec3 proxyPoint = b3Vec3_zero;
    var proxy = b3ShapeProxy{&proxyPoint, 1, 0.5f};
    b3World_CastShape(worldId, rayOrigin, &proxy, rayTranslation, filter, cast(b3CastResultFcn, QuerySpawnCastCallback), &fraction);
    if fraction < 1.0f {
        data.queryHitCount += 1;
        data.queryHash = b3Hash(data.queryHash, cast(u8*, &fraction), cast(i32, sizeof(fraction)));
    }
    data.castFraction = fraction;
    f32 size = 0.3f + 0.2f * fraction;
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = spawnPosition;
    bodyDef.rotation = RandomQuat();
    bodyDef.linearVelocity = RandomVec3Uniform(-0.2f, 0.2f);
    bodyDef.angularVelocity = RandomVec3Uniform(-0.5f, 0.5f);
    bodyDef.linearDamping = 1.0f;
    bodyDef.angularDamping = 1.0f;
    b3BodyId bodyId = b3CreateBody(worldId, &bodyDef);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.baseMaterial.rollingResistance = 0.2f;
    switch (data.spawnCount + overlap.count) % 3 {
        case 0: {
            {
                var sphere = b3Sphere{b3Vec3_zero, size};
                b3CreateSphereShape(bodyId, &shapeDef, &sphere);
                break case;
            }
        }
        case 1: {
            {
                var capsule = b3Capsule{
                    b3Vec3{0.0f, -size, 0.0f},
                    b3Vec3{0.0f, size, 0.0f},
                    0.7f * size,
                };
                b3CreateCapsuleShape(bodyId, &shapeDef, &capsule);
                break case;
            }
        }
        default: {
            {
                b3BoxHull box = b3MakeBoxHull(size, 0.7f * size, 0.5f * size);
                b3CreateHullShape(bodyId, &shapeDef, &box.base);
                break case;
            }
        }
    }
    data.bodies[data.spawnCount] = bodyId;
    data.spawnCount += 1;
    data.lastSpawnPosition = spawnPosition;
}
}

QuerySpawnData CreateQuerySpawn(b3WorldId worldId) {
    QuerySpawnData data;
    g_randomSeed = 71689;
    b3World_SetGravity(worldId, b3Vec3_zero);
    return data;
}

bool UpdateQuerySpawn(b3WorldId worldId, QuerySpawnData* data) {
    if data.spawnCount < 50 {
        QuerySpawnOnce(worldId, data);
    } else if data.hash == 0 && b3World_GetAwakeBodyCount(worldId) == 0 {
        data.hash = B3_HASH_INIT;
        for i32 i = 0; i < 50; ++i {
            b3WorldTransform xf = b3Body_GetTransform(data.bodies[i]);
            data.hash = b3Hash(data.hash, cast(u8*, &xf), cast(i32, sizeof(b3WorldTransform)));
        }
        data.sleepStep = data.stepCount;
    }
    data.stepCount += 1;
    return data.hash != 0;
}

void DestroyQuerySpawn(QuerySpawnData* data) {
    ignore data;
}

MeshDropData CreateMeshDrop(b3WorldId worldId, b3Pos origin) {
    MeshDropData data;
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = origin;
        b3BodyId groundId = b3CreateBody(worldId, &bodyDef);
        i32 gridCount = 40;
        f32 cellWidth = 1.0f;
        f32 rowHz = 0.1f;
        f32 columnHz = 0.2f;
        f32 groundAmplitude = 0.5f;
        data.mesh = b3CreateWaveMesh(gridCount, gridCount, cellWidth, groundAmplitude, rowHz, columnHz);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        shapeDef.filter.categoryBits = 1;
        b3CreateMeshShape(groundId, &shapeDef, data.mesh, b3Vec3_one);
    }
    {
        b3BoxHull box = b3MakeBoxHull(0.02f, 0.2f, 0.04f);
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        shapeDef.baseMaterial.rollingResistance = 0.1f;
        shapeDef.filter.categoryBits = 2;
        shapeDef.filter.maskBits = 1;
        g_randomSeed = 3963634789;
        i32 gridCount = 20;
        for i32 i = 0; i < gridCount; ++i {
            for i32 j = 0; j < gridCount; ++j {
                b3Vec3 linearVelocity = RandomVec3Uniform(-1.0f, 1.0f);
                b3Vec3 angularVelocity = RandomVec3Uniform(-5.0f, 5.0f);
                bodyDef.position = b3OffsetPos(origin, b3Vec3{
                    0.5f * (cast(f32, i) - 0.5f * cast(f32, gridCount)), 5.0f,
                    0.5f * (cast(f32, j) - 0.5f * cast(f32, gridCount)),
                });
                bodyDef.linearVelocity = linearVelocity;
                bodyDef.angularVelocity = angularVelocity;
                b3BodyId bodyId = b3CreateBody(worldId, &bodyDef);
                data.bodies[i * gridCount + j] = bodyId;
                b3CreateHullShape(bodyId, &shapeDef, &box.base);
            }
        }
    }
    return data;
}

bool UpdateMeshDrop(b3WorldId worldId, MeshDropData* data) {
    if data.hash == 0 {
        if b3World_GetAwakeBodyCount(worldId) == 0 {
            data.hash = B3_HASH_INIT;
            i32 bodyCount = 20 * 20;
            for i32 i = 0; i < bodyCount; ++i {
                b3WorldTransform xf = b3Body_GetTransform(data.bodies[i]);
                data.hash = b3Hash(data.hash, cast(u8*, &xf), cast(i32, sizeof(b3WorldTransform)));
            }
            data.sleepStep = data.stepCount;
        }
    }
    data.stepCount += 1;
    return data.hash != 0;
}

void DestroyMeshDrop(MeshDropData* data) {
    b3DestroyMesh(data.mesh);
}

OverflowColorPileData CreateOverflowColorPile(b3WorldId worldId) {
    OverflowColorPileData data;
    data.neighborCount = 5 * 5;
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{0.0f, -1.0f, 0.0f};
        b3BodyId groundId = b3CreateBody(worldId, &bodyDef);
        b3BoxHull box = b3MakeBoxHull(20.0f, 1.0f, 20.0f);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        data.groundShapeId = b3CreateHullShape(groundId, &shapeDef, &box.base);
    }
    f32 hubHalfX = 0.5f;
    f32 hubHalfY = 2.5f;
    f32 hubHalfZ = 0.5f;
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;
        bodyDef.position = b3Pos{0.0f, hubHalfY, 0.0f};
        data.hubId = b3CreateBody(worldId, &bodyDef);
        b3BoxHull box = b3MakeBoxHull(hubHalfX, hubHalfY, hubHalfZ);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        shapeDef.density = 50.0f;
        b3CreateHullShape(data.hubId, &shapeDef, &box.base);
    }
    f32 neighborHalf = 0.2f;
    f32 ringRadius = hubHalfX + neighborHalf - 0.03f;
    b3BoxHull neighborBox = b3MakeBoxHull(neighborHalf, neighborHalf, neighborHalf);
    b3ShapeDef neighborShape = b3DefaultShapeDef();
    f32 ringSpacing = 0.5f;
    f32 baseY = neighborHalf + 0.05f;
    for i32 ring = 0; ring < 5; ++ring {
        f32 y = baseY + ringSpacing * cast(f32, ring);
        f32 thetaOffset = (ring & 1) != 0 ? 3.14159265359f / 5.0f : 0.0f;
        for i32 slot = 0; slot < 5; ++slot {
            f32 theta = thetaOffset + 2.0f * 3.14159265359f * cast(f32, slot) / 5.0f;
            b3BodyDef bodyDef = b3DefaultBodyDef();
            bodyDef.type = b3_dynamicBody;
            bodyDef.position = b3Pos{
                ringRadius * cast(f32, cosf(theta)), y, ringRadius * cast(f32, sinf(theta)),
            };
            b3BodyId bodyId = b3CreateBody(worldId, &bodyDef);
            b3CreateHullShape(bodyId, &neighborShape, &neighborBox.base);
        }
    }
    return data;
}
