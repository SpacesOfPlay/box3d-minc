// box3d_human.mc — ragdoll helper from box3d's samples (shared/human.c), transpiled.
// 14 bones, 13 joints.
//
import box3d;
import math;

u32 g_randomSeed = 12345;

// Zero-init globals
b3BodyId b3_nullBodyId;
b3JointId b3_nullJointId;
// transminc: C #define values surfaced as compile-time configuration
@define "__x86_64__" 1
@define "NDEBUG" 1
@define "_MSC_VER" 1900
@define "FILTER_JOINT_COUNT" 8
@define "RAND_LIMIT" 32767
@define "RAND_SEED" 12345
@define "B3_ENABLE_VALIDATION" 0
@define "B3_NULL_INDEX" -1
@define "B3_HASH_INIT" 5381
@define "B3_MAX_WORKERS" 32
@define "B3_MAX_TASKS" 256
@define "B3_GRAPH_COLOR_COUNT" 24
@define "B3_CONTACT_MANIFOLD_COUNT_BUCKETS" 8
@define "B3_MAX_WORLDS" 128
@define "B3_MAX_MANIFOLD_POINTS" 4
@define "B3_MAX_SHAPE_CAST_POINTS" 64
@define "B3_GYROSCOPIC_ITERATIONS" 1
@define "B3_MAX_HULL_VERTICES" 128
@define "B3_MAX_HULL_FACES" 128
@define "B3_MAX_HULL_EDGES" 128
@define "B3_SHAPE_POWER" 22
@define "B3_RESTITUTION_ITERATIONS" 1
@define "B3_DYNAMIC_TREE_VERSION" -7787375179321898166
@define "B3_HULL_VERSION" -2715301031560262655
@define "B3_MESH_VERSION" -6066037853393090451
@define "B3_HEIGHT_FIELD_HOLE" 255
@define "B3_HEIGHT_FIELD_VERSION" -8423759003537458044
@define "B3_MAX_COMPOUND_MESH_MATERIALS" 4

// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
enum BoneId {
    bone_pelvis = 0,
    bone_spine_01 = 1,
    bone_spine_02 = 2,
    bone_spine_03 = 3,
    bone_neck = 4,
    bone_head = 5,
    bone_thigh_l = 6,
    bone_calf_l = 7,
    bone_thigh_r = 8,
    bone_calf_r = 9,
    bone_upper_arm_l = 10,
    bone_lower_arm_l = 11,
    bone_upper_arm_r = 12,
    bone_lower_arm_r = 13,
    bone_count = 14,
}

type errno_t = i32;
struct Bone {
    b3BodyId bodyId;
    b3JointId jointId;
    b3BodyId anchorId;
    b3JointId anchorJointId;
    b3Transform localFrameA;
    b3Transform localFrameB;
    b3Transform referenceFrame;
    b3JointType jointType;
    f32 swingLimit;
    b3Vec2 twistLimit;
    f32 jointFriction;
    i32 parentIndex;
}

// This must be zero initialized
struct Human {
    Bone[14] bones;
    b3JointId[8] filterJoints;
    i32 filterJointCount;
    f32 frictionTorque;
    bool isSpawned;
}

// void Human_EnablePoseControl( Human* human, float springHertz, bool enablePoseControl );
// void Human_AdjustPoseControl( Human* human, float springHertz );
// void Human_DriveBase( Human* human, b3Transform transform, float timeStep );
// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Global seed for simple random number generator.
// Simple random number generator. Using this instead of rand() for cross platform determinism.
i32 RandomInt() {
    u32 x = g_randomSeed;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    g_randomSeed = x;
    return cast(i32, x % cast(u32, 32767 + 1));
}
// Random integer in range [lo, hi]
f32 RandomIntRange(i32 lo, i32 hi) {
    return cast(f32, lo + RandomInt() % (hi - lo + 1));
}
// Random number in range [-1,1]
f32 RandomFloat() {
    var r = cast(f32, RandomInt() & 32767);
    r /= 32767.0f;
    r = 2.0f * r - 1.0f;
    return r;
}
// Random floating point number in range [lo, hi]
f32 RandomFloatRange(f32 lo, f32 hi) {
    var r = cast(f32, RandomInt() & 32767);
    r /= 32767.0f;
    r = (hi - lo) * r + lo;
    return r;
}
// Random vector with coordinates in range [lo, hi]
b3Vec3 RandomVec3(b3Vec3 lo, b3Vec3 hi) {
    noinit b3Vec3 v;
    v.x = RandomFloatRange(lo.x, hi.x);
    v.y = RandomFloatRange(lo.y, hi.y);
    v.z = RandomFloatRange(lo.z, hi.z);
    return v;
}
// Random world position with coordinates in range [lo, hi]
b3Pos RandomPos(b3Vec3 lo, b3Vec3 hi) {
    noinit b3Pos v;
    v.x = RandomFloatRange(lo.x, hi.x);
    v.y = RandomFloatRange(lo.y, hi.y);
    v.z = RandomFloatRange(lo.z, hi.z);
    return v;
}
b3Vec3 RandomVec3Uniform(f32 lo, f32 hi) {
    noinit b3Vec3 v;
    v.x = RandomFloatRange(lo, hi);
    v.y = RandomFloatRange(lo, hi);
    v.z = RandomFloatRange(lo, hi);
    return v;
}
b3Vec3 RandomUnitVector() {
    f32 u1 = RandomFloatRange(0.0f, 1.0f);
    f32 u2 = RandomFloatRange(0.0f, 2.0f * 3.14159265359f);
    f32 u3 = RandomFloatRange(0.0f, 2.0f * 3.14159265359f);
    f32 sqrt1MinusU1 = sqrtf(1.0f - u1);
    f32 sqrtU1 = sqrtf(u1);
    b3CosSin cs2 = b3ComputeCosSin(u2);
    b3CosSin cs3 = b3ComputeCosSin(u3);
    noinit b3Vec3 v;
    v.x = sqrt1MinusU1 * cs2.sine;
    v.y = sqrt1MinusU1 * cs2.cosine;
    v.z = sqrtU1 * cs3.sine;
    return v;
}
b3Quat RandomQuat() {
    f32 u1 = RandomFloatRange(0.0f, 1.0f);
    f32 u2 = RandomFloatRange(0.0f, 2.0f * 3.14159265359f);
    f32 u3 = RandomFloatRange(0.0f, 2.0f * 3.14159265359f);
    f32 sqrt1MinusU1 = sqrtf(1.0f - u1);
    f32 sqrtU1 = sqrtf(u1);
    b3CosSin cs2 = b3ComputeCosSin(u2);
    b3CosSin cs3 = b3ComputeCosSin(u3);
    noinit b3Quat q;
    q.v.x = sqrt1MinusU1 * cs2.sine;
    q.v.y = sqrt1MinusU1 * cs2.cosine;
    q.v.z = sqrtU1 * cs3.sine;
    q.s = sqrtU1 * cs3.cosine;
    return q;
}
void CreateHuman(Human* human, b3WorldId worldId, b3Pos position, f32 frictionTorque, f32 hertz, f32 dampingRatio, i32 groupIndex, void* userData, bool colorize) {
    for i32 i = 0; i < bone_count; ++i {
        human.bones[i].bodyId = b3_nullBodyId;
        human.bones[i].anchorId = b3_nullBodyId;
        human.bones[i].jointId = b3_nullJointId;
        human.bones[i].jointFriction = 1.0f;
        human.bones[i].parentIndex = -1;
    }
    for i32 i = 0; i < 8; ++i {
        human.filterJoints[i] = b3_nullJointId;
    }
    human.frictionTorque = frictionTorque;
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.userData = userData;
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.baseMaterial.rollingResistance = 0.2f;
    b3HexColor shirtColor = b3_colorMediumTurquoise;
    b3HexColor pantColor = b3_colorDodgerBlue;
    b3HexColor[4] skinColors = {b3_colorNavajoWhite, b3_colorLightYellow, b3_colorPeru, b3_colorTan};
    b3HexColor skinColor = skinColors[groupIndex % 4];
    {
        Bone* bone = human.bones + bone_pelvis;
        bone.parentIndex = -1;
        bodyDef.name = "pelvis";
        bone.referenceFrame = b3Transform{b3Vec3{0.0f, 0.9320870000000001f, -0.05170800000000001f}, b3Quat{b3Vec3{0.7391690000000001f, 0.0f, 0.0f}, 0.6735200000000001f}};
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = b3CreateBody(worldId, &bodyDef);
        var capsule = b3Capsule{b3Vec3{0.07f, 0.0f, -0.08f}, b3Vec3{-0.07f, 0.0f, -0.08f}, 0.13f};
        shapeDef.filter.groupIndex = 0;
        shapeDef.baseMaterial.customColor = colorize != 0 ? pantColor : 0;
        b3CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule);
    }
    {
        Bone* bone = human.bones + bone_spine_01;
        bone.parentIndex = bone_pelvis;
        bodyDef.name = "spine_01";
        bone.referenceFrame = b3Transform{b3Vec3{0.0f, 1.113505f, -0.03481f}, b3Quat{b3Vec3{0.7399730000000001f, 0.0f, 0.0f}, 0.6726370000000002f}};
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = b3CreateBody(worldId, &bodyDef);
        bodyDef.type = b3_dynamicBody;
        var capsule = b3Capsule{b3Vec3{0.06f, -0.0f, -0.05226400000000001f}, b3Vec3{-0.06f, 0.0f, -0.05226400000000001f}, 0.12f};
        shapeDef.filter.groupIndex = -groupIndex;
        shapeDef.baseMaterial.customColor = colorize != 0 ? shirtColor : 0;
        b3CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule);
        bone.jointType = b3_sphericalJoint;
        bone.localFrameA = b3Transform{b3Vec3{0.0f, 0.0f, -0.18220400000000003f}, b3Quat{b3Vec3{-0.9999990000000002f, 0.0f, -0.0f}, 0.0011940000000000002f}};
        bone.localFrameB = b3Transform{b3Vec3{0.0f, 0.0f, -0.0077360000000000016f}, b3Quat{b3Vec3{-1.0f, 0.0f, -0.0f}, 0.0f}};
        bone.swingLimit = 25.0f * 0.017453292510000006f;
        bone.twistLimit = b3Vec2{-15.0f * 0.017453292510000006f, 15.0f * 0.017453292510000006f};
    }
    {
        Bone* bone = human.bones + bone_spine_02;
        bone.parentIndex = bone_spine_01;
        bone.referenceFrame = b3Transform{b3Vec3{0.0f, 1.194336f, -0.027087000000000003f}, b3Quat{b3Vec3{0.7036110000000001f, 0.0f, 0.0f}, 0.7105860000000002f}};
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = b3CreateBody(worldId, &bodyDef);
        var capsule = b3Capsule{b3Vec3{0.08f, -0.015133000000000002f, -0.09180100000000002f}, b3Vec3{-0.08f, -0.015133000000000002f, -0.09180100000000002f}, 0.1f};
        shapeDef.filter.groupIndex = 0;
        shapeDef.baseMaterial.customColor = colorize != 0 ? shirtColor : 0;
        b3CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule);
        bone.jointType = b3_sphericalJoint;
        bone.localFrameA = b3Transform{b3Vec3{0.0f, -0.0f, -0.08893500000000001f}, b3Quat{b3Vec3{-0.9986190000000001f, -0.0f, 0.0f}, -0.05254000000000001f}};
        bone.localFrameB = b3Transform{b3Vec3{-0.0f, 0.0f, -0.008199000000000001f}, b3Quat{b3Vec3{-1.0f, 0.0f, -0.0f}, 0.0f}};
        bone.swingLimit = 25.0f * 0.017453292510000006f;
        bone.twistLimit = b3Vec2{-15.0f * 0.017453292510000006f, 15.0f * 0.017453292510000006f};
    }
    {
        Bone* bone = human.bones + bone_spine_03;
        bone.parentIndex = bone_spine_02;
        bodyDef.name = "spine_03";
        bone.referenceFrame = b3Transform{b3Vec3{-0.0f, 1.31043f, -0.028232000000000004f}, b3Quat{b3Vec3{0.6698560000000001f, 1.0000000000000002e-6f, -1.0000000000000002e-6f}, 0.7424910000000001f}};
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = b3CreateBody(worldId, &bodyDef);
        var capsule = b3Capsule{b3Vec3{0.11f, -0.039753000000000004f, -0.13f}, b3Vec3{-0.11f, -0.039753000000000004f, -0.13f}, 0.145f};
        shapeDef.filter.groupIndex = 0;
        shapeDef.baseMaterial.customColor = colorize != 0 ? shirtColor : 0;
        b3CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule);
        bone.jointType = b3_sphericalJoint;
        bone.localFrameA = b3Transform{b3Vec3{-0.0f, 0.0f, -0.12429800000000002f}, b3Quat{b3Vec3{-0.9989210000000002f, 1.0000000000000002e-6f, -1.0000000000000002e-6f}, -0.04643400000000001f}};
        bone.localFrameB = b3Transform{b3Vec3{0.0f, 0.0f, 0.0f}, b3Quat{b3Vec3{-1.0f, 0.0f, -1.0000000000000002e-6f}, 0.0f}};
        bone.swingLimit = 15.0f * 0.017453292510000006f;
        bone.twistLimit = b3Vec2{-10.0f * 0.017453292510000006f, 10.0f * 0.017453292510000006f};
    }
    {
        Bone* bone = human.bones + bone_neck;
        bone.parentIndex = bone_spine_03;
        bodyDef.name = "neck";
        bone.referenceFrame = b3Transform{b3Vec3{0.0f, 1.575582f, -0.05583700000000001f}, b3Quat{b3Vec3{0.8799220000000001f, 0.0f, 0.0f}, 0.4751180000000001f}};
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = b3CreateBody(worldId, &bodyDef);
        var capsule = b3Capsule{b3Vec3{-1.0000000000000002e-6f, -0.0f, -0.02f}, b3Vec3{0.0f, -0.005f, -0.08f}, 0.07f};
        shapeDef.filter.groupIndex = 0;
        shapeDef.baseMaterial.customColor = colorize != 0 ? skinColor : 0;
        b3CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule);
        bone.jointType = b3_sphericalJoint;
        bone.localFrameA = b3Transform{b3Vec3{1.0000000000000002e-6f, -0.00025900000000000006f, -0.26658500000000007f}, b3Quat{b3Vec3{-0.9421920000000001f, -1.0000000000000002e-6f, 0.0f}, 0.33507400000000004f}};
        bone.localFrameB = b3Transform{b3Vec3{0.0f, 0.0f, 0.0f}, b3Quat{b3Vec3{-1.0f, 0.0f, -1.0000000000000002e-6f}, 0.0f}};
        bone.swingLimit = 45.0f * 0.017453292510000006f;
        bone.twistLimit = b3Vec2{-15.0f * 0.017453292510000006f, 15.0f * 0.017453292510000006f};
        bone.jointFriction = 0.8f;
    }
    {
        Bone* bone = human.bones + bone_head;
        bone.parentIndex = bone_neck;
        bodyDef.name = "head";
        bone.referenceFrame = b3Transform{b3Vec3{0.0f, 1.653348f, -0.0032410000000000004f}, b3Quat{b3Vec3{0.7502880000000002f, 0.0f, 0.0f}, 0.6611110000000001f}};
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = b3CreateBody(worldId, &bodyDef);
        var capsule = b3Capsule{b3Vec3{-1.0000000000000002e-6f, 0.016892000000000004f, -0.058690000000000006f}, b3Vec3{0.0f, -0.0036290000000000007f, -0.11507200000000002f}, 0.0975f};
        shapeDef.filter.groupIndex = 0;
        shapeDef.baseMaterial.customColor = colorize != 0 ? skinColor : 0;
        b3CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule);
        bone.jointType = b3_sphericalJoint;
        bone.localFrameA = b3Transform{b3Vec3{0.0f, 0.0013210000000000001f, -0.09387300000000001f}, b3Quat{b3Vec3{-0.9743010000000002f, -0.0f, -0.0f}, -0.22525100000000003f}};
        bone.localFrameB = b3Transform{b3Vec3{0.0f, 0.0012680000000000002f, -0.005104000000000001f}, b3Quat{b3Vec3{-1.0f, 0.0f, -0.0f}, 0.0f}};
        bone.swingLimit = 15.0f * 0.017453292510000006f;
        bone.twistLimit = b3Vec2{-15.0f * 0.017453292510000006f, 15.0f * 0.017453292510000006f};
        bone.jointFriction = 0.4f;
    }
    {
        Bone* bone = human.bones + bone_thigh_l;
        bone.parentIndex = bone_pelvis;
        bodyDef.name = "thigh_l";
        bone.referenceFrame = b3Transform{b3Vec3{0.09041600000000001f, 0.9861040000000002f, -0.03509f}, b3Quat{b3Vec3{-0.7032870000000001f, -0.07071500000000001f, 0.05386600000000001f}, 0.7053270000000001f}};
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = b3CreateBody(worldId, &bodyDef);
        var capsule = b3Capsule{b3Vec3{0.023719000000000004f, 0.006008000000000001f, -0.039068000000000006f}, b3Vec3{-0.06449200000000001f, -0.004664000000000001f, -0.4247180000000001f}, 0.09f};
        shapeDef.filter.groupIndex = -groupIndex;
        shapeDef.baseMaterial.customColor = colorize != 0 ? pantColor : 0;
        b3CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule);
        bone.jointType = b3_sphericalJoint;
        bone.localFrameA = b3Transform{b3Vec3{0.05f, 0.011537000000000002f, -0.055325000000000006f}, b3Quat{b3Vec3{-0.7148960000000001f, -0.022305000000000005f, -0.6983610000000001f}, -0.026790000000000005f}};
        bone.localFrameB = b3Transform{b3Vec3{0.0f, 0.0f, 0.0f}, b3Quat{b3Vec3{-0.0020640000000000003f, 0.7589870000000001f, 0.017046000000000002f}, 0.6508800000000001f}};
        bone.swingLimit = 10.0f * 0.017453292510000006f;
        bone.twistLimit = b3Vec2{-60.0f * 0.017453292510000006f, 40.0f * 0.017453292510000006f};
    }
    {
        Bone* bone = human.bones + bone_calf_l;
        bone.parentIndex = bone_thigh_l;
        bodyDef.name = "calf_l";
        bone.referenceFrame = b3Transform{b3Vec3{0.10119800000000001f, 0.5270270000000001f, -0.037374000000000004f}, b3Quat{b3Vec3{-0.6533280000000001f, -0.06686000000000002f, 0.05858200000000001f}, 0.7518380000000001f}};
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = b3CreateBody(worldId, &bodyDef);
        var capsule = b3Capsule{b3Vec3{0.0017780000000000003f, 0.0f, 0.009841f}, b3Vec3{-0.07857700000000001f, 0.014707000000000003f, -0.41816000000000003f}, 0.075f};
        shapeDef.filter.groupIndex = 0;
        shapeDef.baseMaterial.customColor = colorize != 0 ? pantColor : 0;
        b3CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule);
        bone.jointType = b3_revoluteJoint;
        bone.localFrameA = b3Transform{b3Vec3{-0.06998900000000001f, 0.000253f, -0.4538440000000001f}, b3Quat{b3Vec3{-0.0006770000000000001f, 0.7600870000000002f, 0.10567400000000002f}, 0.6411710000000002f}};
        bone.localFrameB = b3Transform{b3Vec3{0.0f, 0.0f, 0.0f}, b3Quat{b3Vec3{-0.04458900000000001f, 0.7655400000000001f, 0.053368000000000006f}, 0.6396190000000002f}};
        bone.twistLimit = b3Vec2{-5.0f * 0.017453292510000006f, 45.0f * 0.017453292510000006f};
    }
    {
        Bone* bone = human.bones + bone_thigh_r;
        bone.parentIndex = bone_pelvis;
        bodyDef.name = "thigh_r";
        bone.referenceFrame = b3Transform{b3Vec3{-0.09041600000000001f, 0.9861040000000002f, -0.03509f}, b3Quat{b3Vec3{-0.7032870000000001f, 0.07071500000000001f, -0.05386500000000001f}, 0.7053260000000001f}};
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = b3CreateBody(worldId, &bodyDef);
        var capsule = b3Capsule{b3Vec3{-0.023719000000000004f, 0.006008000000000001f, -0.039068000000000006f}, b3Vec3{0.06449200000000001f, -0.004664000000000001f, -0.4247180000000001f}, 0.09f};
        shapeDef.filter.groupIndex = -groupIndex;
        shapeDef.baseMaterial.customColor = colorize != 0 ? pantColor : 0;
        b3CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule);
        bone.jointType = b3_sphericalJoint;
        bone.localFrameA = b3Transform{b3Vec3{-0.05f, 0.011537000000000002f, -0.05532600000000001f}, b3Quat{b3Vec3{-0.039089000000000006f, -0.7140940000000001f, 0.04317700000000001f}, 0.6976230000000001f}};
        bone.localFrameB = b3Transform{b3Vec3{0.0f, 0.0f, 0.0f}, b3Quat{b3Vec3{0.7588050000000002f, -0.019886000000000004f, -0.6510120000000001f}, -0.0017590000000000004f}};
        bone.swingLimit = 10.0f * 0.017453292510000006f;
        bone.twistLimit = b3Vec2{-30.0f * 0.017453292510000006f, 60.0f * 0.017453292510000006f};
    }
    {
        Bone* bone = human.bones + bone_calf_r;
        bone.parentIndex = bone_thigh_r;
        bodyDef.name = "calf_r";
        bone.referenceFrame = b3Transform{b3Vec3{-0.10119800000000001f, 0.5270270000000001f, -0.037373f}, b3Quat{b3Vec3{-0.6533270000000001f, 0.06686f, -0.05858200000000001f}, 0.7518390000000001f}};
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = b3CreateBody(worldId, &bodyDef);
        var capsule = b3Capsule{b3Vec3{-0.0018200000000000002f, 0.0f, 0.010071000000000002f}, b3Vec3{0.07788300000000001f, 0.014825000000000003f, -0.41804700000000006f}, 0.075f};
        shapeDef.filter.groupIndex = 0;
        shapeDef.baseMaterial.customColor = colorize != 0 ? pantColor : 0;
        b3CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule);
        bone.jointType = b3_revoluteJoint;
        bone.localFrameA = b3Transform{b3Vec3{0.06998800000000001f, 0.000253f, -0.4538440000000001f}, b3Quat{b3Vec3{0.7600860000000002f, -0.0006750000000000001f, -0.6411710000000002f}, -0.10567600000000002f}};
        bone.localFrameB = b3Transform{b3Vec3{0.0f, 0.0f, 0.0f}, b3Quat{b3Vec3{0.7655400000000001f, -0.04458900000000001f, -0.6396190000000002f}, -0.053368000000000006f}};
        bone.twistLimit = b3Vec2{-45.0f * 0.017453292510000006f, 5.0f * 0.017453292510000006f};
    }
    {
        Bone* bone = human.bones + bone_upper_arm_l;
        bone.parentIndex = bone_spine_03;
        bodyDef.name = "upper_arm_l";
        bone.referenceFrame = b3Transform{b3Vec3{0.20378000000000002f, 1.484275f, -0.11589700000000001f}, b3Quat{b3Vec3{0.14308200000000001f, 0.6959800000000002f, -0.6901300000000001f}, 0.13733f}};
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = b3CreateBody(worldId, &bodyDef);
        var capsule = b3Capsule{b3Vec3{0.0f, 0.0f, 0.0f}, b3Vec3{-0.09111800000000002f, 0.037775f, 0.22971900000000003f}, 0.075f};
        shapeDef.filter.groupIndex = 0;
        shapeDef.baseMaterial.customColor = colorize != 0 ? shirtColor : 0;
        b3CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule);
        bone.jointType = b3_sphericalJoint;
        bone.localFrameA = b3Transform{b3Vec3{0.20378000000000004f, -0.06936900000000001f, -0.18192100000000003f}, b3Quat{b3Vec3{-0.27848600000000007f, 0.44560000000000005f, -0.09701400000000002f}, 0.8452660000000002f}};
        bone.localFrameB = b3Transform{b3Vec3{0.0f, 0.0f, 0.0f}, b3Quat{b3Vec3{-0.20139600000000005f, -0.0015860000000000002f, 0.9018500000000002f}, 0.3822340000000001f}};
        bone.swingLimit = 60.0f * 0.017453292510000006f;
        bone.twistLimit = b3Vec2{-5.0f * 0.017453292510000006f, 5.0f * 0.017453292510000006f};
    }
    {
        Bone* bone = human.bones + bone_lower_arm_l;
        bone.parentIndex = bone_upper_arm_l;
        bodyDef.name = "lower_arm_l";
        bone.referenceFrame = b3Transform{b3Vec3{0.30561400000000005f, 1.2429080000000001f, -0.11759900000000002f}, b3Quat{b3Vec3{0.16504800000000003f, 0.5634370000000001f, -0.8020020000000001f}, 0.10995900000000002f}};
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = b3CreateBody(worldId, &bodyDef);
        var capsule = b3Capsule{b3Vec3{0.0f, 0.0f, 0.0f}, b3Vec3{-0.14240600000000003f, 0.039392f, 0.26109200000000005f}, 0.05f};
        shapeDef.filter.groupIndex = 0;
        shapeDef.baseMaterial.customColor = colorize != 0 ? skinColor : 0;
        b3CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule);
        bone.jointType = b3_revoluteJoint;
        bone.localFrameA = b3Transform{b3Vec3{-0.09548200000000001f, 0.03958400000000001f, 0.24072300000000005f}, b3Quat{b3Vec3{0.5124870000000001f, -0.18062900000000004f, 0.8394740000000002f}, 0.0037420000000000005f}};
        bone.localFrameB = b3Transform{b3Vec3{0.0f, 0.0f, 0.0f}, b3Quat{b3Vec3{0.5038030000000001f, -0.029831000000000003f, 0.8581680000000002f}, 0.09401700000000002f}};
        bone.twistLimit = b3Vec2{-5.0f * 0.017453292510000006f, 60.0f * 0.017453292510000006f};
    }
    {
        Bone* bone = human.bones + bone_upper_arm_r;
        bone.parentIndex = bone_spine_03;
        bodyDef.name = "upper_arm_r";
        bone.referenceFrame = b3Transform{b3Vec3{-0.20378000000000002f, 1.4842760000000002f, -0.11589900000000002f}, b3Quat{b3Vec3{0.14308300000000002f, -0.6959780000000001f, 0.6901320000000001f}, 0.13732900000000003f}};
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = b3CreateBody(worldId, &bodyDef);
        var capsule = b3Capsule{b3Vec3{0.0f, 0.0f, 0.0f}, b3Vec3{0.09111800000000002f, 0.037775f, 0.22971800000000003f}, 0.075f};
        shapeDef.filter.groupIndex = 0;
        shapeDef.baseMaterial.customColor = colorize != 0 ? shirtColor : 0;
        b3CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule);
        bone.jointType = b3_sphericalJoint;
        bone.localFrameA = b3Transform{b3Vec3{-0.20377900000000004f, -0.06937100000000002f, -0.18192200000000003f}, b3Quat{b3Vec3{-0.25362100000000004f, -0.41484200000000004f, 0.10696200000000002f}, 0.8672610000000002f}};
        bone.localFrameB = b3Transform{b3Vec3{0.0f, 0.0f, 0.0f}, b3Quat{b3Vec3{-0.20139700000000002f, 0.0015870000000000003f, -0.9018500000000002f}, 0.38223300000000004f}};
        bone.swingLimit = 60.0f * 0.017453292510000006f;
        bone.twistLimit = b3Vec2{-5.0f * 0.017453292510000006f, 5.0f * 0.017453292510000006f};
    }
    {
        Bone* bone = human.bones + bone_lower_arm_r;
        bone.parentIndex = bone_upper_arm_r;
        bodyDef.name = "lower_arm_r";
        bone.referenceFrame = b3Transform{b3Vec3{-0.30561400000000005f, 1.242907f, -0.11759900000000002f}, b3Quat{b3Vec3{0.16504800000000003f, -0.5634370000000001f, 0.8020020000000001f}, 0.10995900000000002f}};
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = b3CreateBody(worldId, &bodyDef);
        var capsule = b3Capsule{b3Vec3{0.0f, 0.0f, 0.0f}, b3Vec3{0.14240600000000003f, 0.039392f, 0.26109200000000005f}, 0.05f};
        shapeDef.filter.groupIndex = 0;
        shapeDef.baseMaterial.customColor = colorize != 0 ? skinColor : 0;
        b3CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule);
        bone.jointType = b3_revoluteJoint;
        bone.localFrameA = b3Transform{b3Vec3{0.09548400000000001f, 0.03958500000000001f, 0.24072300000000005f}, b3Quat{b3Vec3{-0.18062700000000004f, 0.5124870000000001f, -0.003744000000000001f}, -0.8394740000000002f}};
        bone.localFrameB = b3Transform{b3Vec3{0.0f, 0.0f, 0.0f}, b3Quat{b3Vec3{-0.029831000000000003f, 0.5038030000000001f, -0.09401700000000002f}, -0.8581690000000002f}};
        bone.twistLimit = b3Vec2{-60.0f * 0.017453292510000006f, 5.0f * 0.017453292510000006f};
    }
    for i32 i = 1; i < bone_count; ++i {
        Bone* bone = human.bones + i;
        Bone* parent = human.bones + bone.parentIndex;
        b3BodyId bodyIdA = parent.bodyId;
        b3BodyId bodyIdB = bone.bodyId;
        bone.localFrameA.q = b3NormalizeQuat(bone.localFrameA.q);
        bone.localFrameB.q = b3NormalizeQuat(bone.localFrameB.q);
        if bone.jointType == b3_revoluteJoint {
            b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
            jointDef.base.bodyIdA = bodyIdA;
            jointDef.base.bodyIdB = bodyIdB;
            jointDef.base.localFrameA = bone.localFrameA;
            jointDef.base.localFrameB = bone.localFrameB;
            jointDef.enableLimit = true;
            jointDef.lowerAngle = bone.twistLimit.x;
            jointDef.upperAngle = bone.twistLimit.y;
            jointDef.enableSpring = hertz > 0.0f;
            jointDef.hertz = hertz;
            jointDef.dampingRatio = dampingRatio;
            jointDef.enableMotor = true;
            jointDef.maxMotorTorque = bone.jointFriction * frictionTorque;
            bone.jointId = b3CreateRevoluteJoint(worldId, &jointDef);
        } else if bone.jointType == b3_sphericalJoint {
            b3SphericalJointDef jointDef = b3DefaultSphericalJointDef();
            jointDef.base.bodyIdA = bodyIdA;
            jointDef.base.bodyIdB = bodyIdB;
            jointDef.base.localFrameA = bone.localFrameA;
            jointDef.base.localFrameB = bone.localFrameB;
            jointDef.enableConeLimit = true;
            jointDef.coneAngle = bone.swingLimit;
            jointDef.enableTwistLimit = true;
            jointDef.lowerTwistAngle = bone.twistLimit.x;
            jointDef.upperTwistAngle = bone.twistLimit.y;
            jointDef.enableSpring = hertz > 0.0f;
            jointDef.hertz = hertz;
            jointDef.dampingRatio = dampingRatio;
            jointDef.enableMotor = true;
            jointDef.maxMotorTorque = bone.jointFriction * frictionTorque;
            bone.jointId = b3CreateSphericalJoint(worldId, &jointDef);
        }
    }
    b3FilterJointDef filterDef = b3DefaultFilterJointDef();
    filterDef.base.bodyIdA = human.bones[bone_thigh_l].bodyId;
    filterDef.base.bodyIdB = human.bones[bone_thigh_r].bodyId;
    human.filterJoints[0] = b3CreateFilterJoint(worldId, &filterDef);
    human.filterJointCount = 1;
    human.isSpawned = true;
}
void DestroyHuman(Human* human) {
    for i32 i = 0; i < human.filterJointCount; ++i {
        b3DestroyJoint(human.filterJoints[i], false);
        human.filterJoints[i] = b3_nullJointId;
    }
    for i32 i = 0; i < bone_count; ++i {
        if human.bones[i].jointId.index1 == 0 {
            continue;
        }
        b3DestroyJoint(human.bones[i].jointId, false);
        human.bones[i].jointId = b3_nullJointId;
    }
    for i32 i = 0; i < bone_count; ++i {
        if human.bones[i].bodyId.index1 == 0 {
            continue;
        }
        b3DestroyBody(human.bones[i].bodyId);
        human.bones[i].bodyId = b3_nullBodyId;
    }
    human.isSpawned = false;
}
void Human_SetVelocity(Human* human, b3Vec3 velocity) {
    for i32 i = 0; i < bone_count; ++i {
        b3BodyId bodyId = human.bones[i].bodyId;
        if bodyId.index1 == 0 {
            continue;
        }
        b3Body_SetLinearVelocity(bodyId, velocity);
    }
}
void Human_ApplyRandomAngularImpulse(Human* human, f32 magnitude) {
    var range = b3Vec3{magnitude, magnitude, magnitude};
    b3Vec3 impulse = RandomVec3(b3Neg(range), range);
    b3Body_ApplyAngularImpulse(human.bones[bone_spine_01].bodyId, impulse, true);
}
void Human_SetJointFrictionTorque(Human* human, f32 torque) {
    human.frictionTorque = torque;
    for i32 i = 1; i < bone_count; ++i {
        Bone* bone = human.bones + i;
        if bone.jointType == b3_revoluteJoint {
            b3RevoluteJoint_SetMaxMotorTorque(bone.jointId, bone.jointFriction * torque);
        } else {
            b3SphericalJoint_SetMaxMotorTorque(bone.jointId, bone.jointFriction * torque);
        }
    }
}
void Human_SetJointSpringHertz(Human* human, f32 hertz) {
    for i32 i = 1; i < bone_count; ++i {
        Bone* bone = human.bones + i;
        if bone.jointType == b3_revoluteJoint {
            b3RevoluteJoint_SetSpringHertz(bone.jointId, hertz);
        } else {
            b3SphericalJoint_SetSpringHertz(bone.jointId, hertz);
        }
    }
}
void Human_SetJointDampingRatio(Human* human, f32 dampingRatio) {
    for i32 i = 1; i < bone_count; ++i {
        Bone* bone = human.bones + i;
        if bone.jointType == b3_revoluteJoint {
            b3RevoluteJoint_SetSpringDampingRatio(bone.jointId, dampingRatio);
        } else {
            b3SphericalJoint_SetSpringDampingRatio(bone.jointId, dampingRatio);
        }
    }
}
void Human_AlignSpring(Human* human, b3WorldId worldId, b3BodyId groundId, f32 hertz, f32 dampingRatio) {
    Bone* bone = human.bones + bone_pelvis;
    b3Quat q = b3ComputeQuatBetweenUnitVectors(b3Vec3_axisZ, b3Vec3_axisY);
    b3Quat qb = b3Body_GetRotation(bone.bodyId);
    b3ParallelJointDef jointDef = b3DefaultParallelJointDef();
    jointDef.base.bodyIdA = groundId;
    jointDef.base.bodyIdB = bone.bodyId;
    jointDef.base.localFrameA.q = q;
    jointDef.base.localFrameB.q = b3InvMulQuat(qb, q);
    jointDef.base.drawScale = 2.0f;
    jointDef.base.collideConnected = true;
    jointDef.hertz = hertz;
    jointDef.dampingRatio = dampingRatio;
    bone.jointId = b3CreateParallelJoint(worldId, &jointDef);
}
void Human_CreateMotorAnchors(Human* human, b3WorldId worldId) {
    b3BodyDef anchorDef = b3DefaultBodyDef();
    anchorDef.type = b3_kinematicBody;
    b3MotorJointDef motorDef = b3DefaultMotorJointDef();
    motorDef.angularHertz = 5.0f;
    motorDef.angularDampingRatio = 1.0f;
    motorDef.linearHertz = 5.0f;
    motorDef.linearDampingRatio = 1.0f;
    motorDef.maxSpringForce = FLT_MAX;
    motorDef.maxSpringTorque = FLT_MAX;
    for i32 i = 0; i < bone_count; ++i {
        Bone* bone = human.bones + i;
        b3WorldTransform bodyTransform = b3Body_GetTransform(bone.bodyId);
        anchorDef.position = bodyTransform.p;
        anchorDef.rotation = bodyTransform.q;
        bone.anchorId = b3CreateBody(worldId, &anchorDef);
        motorDef.base.bodyIdA = bone.anchorId;
        motorDef.base.bodyIdB = bone.bodyId;
        bone.anchorJointId = b3CreateMotorJoint(worldId, &motorDef);
    }
}
void Human_CreateParallelAnchors(Human* human, b3WorldId worldId) {
    b3BodyDef anchorDef = b3DefaultBodyDef();
    anchorDef.type = b3_kinematicBody;
    b3Quat qFrameWorld = b3ComputeQuatBetweenUnitVectors(b3Vec3_axisZ, b3Vec3_axisY);
    b3ParallelJointDef jointDef = b3DefaultParallelJointDef();
    jointDef.hertz = 8.0f;
    jointDef.dampingRatio = 1.0f;
    jointDef.maxTorque = 800.0f;
    for i32 i = 0; i < bone_count; ++i {
        Bone* bone = human.bones + i;
        b3WorldTransform bodyTransform = b3Body_GetTransform(bone.bodyId);
        anchorDef.position = bodyTransform.p;
        anchorDef.rotation = bodyTransform.q;
        bone.anchorId = b3CreateBody(worldId, &anchorDef);
        jointDef.base.bodyIdA = bone.anchorId;
        jointDef.base.bodyIdB = bone.bodyId;
        b3Quat frameQuat = b3InvMulQuat(bodyTransform.q, qFrameWorld);
        jointDef.base.localFrameA.q = frameQuat;
        jointDef.base.localFrameB.q = frameQuat;
        bone.anchorJointId = b3CreateParallelJoint(worldId, &jointDef);
    }
}
void Human_SetBullet(Human* human, bool flag) {
    for i32 i = 0; i < bone_count; ++i {
        Bone* bone = human.bones + i;
        b3Body_SetBullet(bone.bodyId, flag);
    }
}
