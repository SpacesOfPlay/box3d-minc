// box3d_human.mc — ragdoll helper from box3d's samples (shared/human.c), transpiled.
// 14 bones, 13 joints.
//
import box3d;
import math;

u32 g_randomSeed = 12345;

// Zero-init globals
b3BodyId b3_nullBodyId;
b3JointId b3_nullJointId;
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
    f32 sqrt1MinusU1 = sqrt(1.0f - u1);
    f32 sqrtU1 = sqrt(u1);
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
    f32 sqrt1MinusU1 = sqrt(1.0f - u1);
    f32 sqrtU1 = sqrt(u1);
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
        bone.referenceFrame = b3Transform{
            b3Vec3{0.0f, 0.932087f, -0.051708f},
            b3Quat{b3Vec3{0.739169f, 0.0f, 0.0f}, 0.67352f},
        };
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = b3CreateBody(worldId, &bodyDef);
        var capsule = b3Capsule{b3Vec3{0.07f, 0.0f, -0.08f}, b3Vec3{-0.07f, 0.0f, -0.08f}, 0.13f};
        shapeDef.filter.groupIndex = 0;
        shapeDef.baseMaterial.customColor = cast(u32, colorize != 0 ? pantColor : 0);
        b3CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule);
    }
    {
        Bone* bone = human.bones + bone_spine_01;
        bone.parentIndex = bone_pelvis;
        bodyDef.name = "spine_01";
        bone.referenceFrame = b3Transform{
            b3Vec3{0.0f, 1.113505f, -0.03481f},
            b3Quat{b3Vec3{0.739973f, 0.0f, 0.0f}, 0.672637f},
        };
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = b3CreateBody(worldId, &bodyDef);
        bodyDef.type = b3_dynamicBody;
        var capsule = b3Capsule{
            b3Vec3{0.06f, -0.0f, -0.052264f},
            b3Vec3{-0.06f, 0.0f, -0.052264f},
            0.12f,
        };
        shapeDef.filter.groupIndex = -groupIndex;
        shapeDef.baseMaterial.customColor = cast(u32, colorize != 0 ? shirtColor : 0);
        b3CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule);
        bone.jointType = b3_sphericalJoint;
        bone.localFrameA = b3Transform{
            b3Vec3{0.0f, 0.0f, -0.182204f},
            b3Quat{b3Vec3{-0.999999f, 0.0f, -0.0f}, 0.001194f},
        };
        bone.localFrameB = b3Transform{
            b3Vec3{0.0f, 0.0f, -0.007736f},
            b3Quat{b3Vec3{-1.0f, 0.0f, -0.0f}, 0.0f},
        };
        bone.swingLimit = 25.0f * 0.01745329251f;
        bone.twistLimit = b3Vec2{-15.0f * 0.01745329251f, 15.0f * 0.01745329251f};
    }
    {
        Bone* bone = human.bones + bone_spine_02;
        bone.parentIndex = bone_spine_01;
        bone.referenceFrame = b3Transform{
            b3Vec3{0.0f, 1.194336f, -0.027087f},
            b3Quat{b3Vec3{0.703611f, 0.0f, 0.0f}, 0.710586f},
        };
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = b3CreateBody(worldId, &bodyDef);
        var capsule = b3Capsule{
            b3Vec3{0.08f, -0.015133f, -0.091801f},
            b3Vec3{-0.08f, -0.015133f, -0.091801f},
            0.1f,
        };
        shapeDef.filter.groupIndex = 0;
        shapeDef.baseMaterial.customColor = cast(u32, colorize != 0 ? shirtColor : 0);
        b3CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule);
        bone.jointType = b3_sphericalJoint;
        bone.localFrameA = b3Transform{
            b3Vec3{0.0f, -0.0f, -0.088935f},
            b3Quat{b3Vec3{-0.998619f, -0.0f, 0.0f}, -0.05254f},
        };
        bone.localFrameB = b3Transform{
            b3Vec3{-0.0f, 0.0f, -0.008199f},
            b3Quat{b3Vec3{-1.0f, 0.0f, -0.0f}, 0.0f},
        };
        bone.swingLimit = 25.0f * 0.01745329251f;
        bone.twistLimit = b3Vec2{-15.0f * 0.01745329251f, 15.0f * 0.01745329251f};
    }
    {
        Bone* bone = human.bones + bone_spine_03;
        bone.parentIndex = bone_spine_02;
        bodyDef.name = "spine_03";
        bone.referenceFrame = b3Transform{
            b3Vec3{-0.0f, 1.31043f, -0.028232f},
            b3Quat{b3Vec3{0.669856f, 1.0e-6f, -1.0e-6f}, 0.742491f},
        };
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = b3CreateBody(worldId, &bodyDef);
        var capsule = b3Capsule{
            b3Vec3{0.11f, -0.039753f, -0.13f},
            b3Vec3{-0.11f, -0.039753f, -0.13f},
            0.145f,
        };
        shapeDef.filter.groupIndex = 0;
        shapeDef.baseMaterial.customColor = cast(u32, colorize != 0 ? shirtColor : 0);
        b3CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule);
        bone.jointType = b3_sphericalJoint;
        bone.localFrameA = b3Transform{
            b3Vec3{-0.0f, 0.0f, -0.124298f},
            b3Quat{b3Vec3{-0.998921f, 1.0e-6f, -1.0e-6f}, -0.046434f},
        };
        bone.localFrameB = b3Transform{
            b3Vec3{0.0f, 0.0f, 0.0f},
            b3Quat{b3Vec3{-1.0f, 0.0f, -1.0e-6f}, 0.0f},
        };
        bone.swingLimit = 15.0f * 0.01745329251f;
        bone.twistLimit = b3Vec2{-10.0f * 0.01745329251f, 10.0f * 0.01745329251f};
    }
    {
        Bone* bone = human.bones + bone_neck;
        bone.parentIndex = bone_spine_03;
        bodyDef.name = "neck";
        bone.referenceFrame = b3Transform{
            b3Vec3{0.0f, 1.575582f, -0.055837f},
            b3Quat{b3Vec3{0.879922f, 0.0f, 0.0f}, 0.475118f},
        };
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = b3CreateBody(worldId, &bodyDef);
        var capsule = b3Capsule{
            b3Vec3{-1.0e-6f, -0.0f, -0.02f},
            b3Vec3{0.0f, -0.005f, -0.08f},
            0.07f,
        };
        shapeDef.filter.groupIndex = 0;
        shapeDef.baseMaterial.customColor = cast(u32, colorize != 0 ? skinColor : 0);
        b3CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule);
        bone.jointType = b3_sphericalJoint;
        bone.localFrameA = b3Transform{
            b3Vec3{1.0e-6f, -0.000259f, -0.266585f},
            b3Quat{b3Vec3{-0.942192f, -1.0e-6f, 0.0f}, 0.335074f},
        };
        bone.localFrameB = b3Transform{
            b3Vec3{0.0f, 0.0f, 0.0f},
            b3Quat{b3Vec3{-1.0f, 0.0f, -1.0e-6f}, 0.0f},
        };
        bone.swingLimit = 45.0f * 0.01745329251f;
        bone.twistLimit = b3Vec2{-15.0f * 0.01745329251f, 15.0f * 0.01745329251f};
        bone.jointFriction = 0.8f;
    }
    {
        Bone* bone = human.bones + bone_head;
        bone.parentIndex = bone_neck;
        bodyDef.name = "head";
        bone.referenceFrame = b3Transform{
            b3Vec3{0.0f, 1.653348f, -0.003241f},
            b3Quat{b3Vec3{0.750288f, 0.0f, 0.0f}, 0.661111f},
        };
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = b3CreateBody(worldId, &bodyDef);
        var capsule = b3Capsule{
            b3Vec3{-1.0e-6f, 0.016892f, -0.05869f},
            b3Vec3{0.0f, -0.003629f, -0.115072f},
            0.0975f,
        };
        shapeDef.filter.groupIndex = 0;
        shapeDef.baseMaterial.customColor = cast(u32, colorize != 0 ? skinColor : 0);
        b3CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule);
        bone.jointType = b3_sphericalJoint;
        bone.localFrameA = b3Transform{
            b3Vec3{0.0f, 0.001321f, -0.093873f},
            b3Quat{b3Vec3{-0.974301f, -0.0f, -0.0f}, -0.225251f},
        };
        bone.localFrameB = b3Transform{
            b3Vec3{0.0f, 0.001268f, -0.005104f},
            b3Quat{b3Vec3{-1.0f, 0.0f, -0.0f}, 0.0f},
        };
        bone.swingLimit = 15.0f * 0.01745329251f;
        bone.twistLimit = b3Vec2{-15.0f * 0.01745329251f, 15.0f * 0.01745329251f};
        bone.jointFriction = 0.4f;
    }
    {
        Bone* bone = human.bones + bone_thigh_l;
        bone.parentIndex = bone_pelvis;
        bodyDef.name = "thigh_l";
        bone.referenceFrame = b3Transform{
            b3Vec3{0.090416f, 0.986104f, -0.03509f},
            b3Quat{b3Vec3{-0.703287f, -0.070715f, 0.053866f}, 0.705327f},
        };
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = b3CreateBody(worldId, &bodyDef);
        var capsule = b3Capsule{
            b3Vec3{0.023719f, 0.006008f, -0.039068f},
            b3Vec3{-0.064492f, -0.004664f, -0.424718f},
            0.09f,
        };
        shapeDef.filter.groupIndex = -groupIndex;
        shapeDef.baseMaterial.customColor = cast(u32, colorize != 0 ? pantColor : 0);
        b3CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule);
        bone.jointType = b3_sphericalJoint;
        bone.localFrameA = b3Transform{
            b3Vec3{0.05f, 0.011537f, -0.055325f},
            b3Quat{b3Vec3{-0.714896f, -0.022305f, -0.698361f}, -0.02679f},
        };
        bone.localFrameB = b3Transform{
            b3Vec3{0.0f, 0.0f, 0.0f},
            b3Quat{b3Vec3{-0.002064f, 0.758987f, 0.017046f}, 0.65088f},
        };
        bone.swingLimit = 10.0f * 0.01745329251f;
        bone.twistLimit = b3Vec2{-60.0f * 0.01745329251f, 40.0f * 0.01745329251f};
    }
    {
        Bone* bone = human.bones + bone_calf_l;
        bone.parentIndex = bone_thigh_l;
        bodyDef.name = "calf_l";
        bone.referenceFrame = b3Transform{
            b3Vec3{0.101198f, 0.527027f, -0.037374f},
            b3Quat{b3Vec3{-0.653328f, -0.06686f, 0.058582f}, 0.751838f},
        };
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = b3CreateBody(worldId, &bodyDef);
        var capsule = b3Capsule{
            b3Vec3{0.001778f, 0.0f, 0.009841f},
            b3Vec3{-0.078577f, 0.014707f, -0.41816f},
            0.075f,
        };
        shapeDef.filter.groupIndex = 0;
        shapeDef.baseMaterial.customColor = cast(u32, colorize != 0 ? pantColor : 0);
        b3CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule);
        bone.jointType = b3_revoluteJoint;
        bone.localFrameA = b3Transform{
            b3Vec3{-0.069989f, 0.000253f, -0.453844f},
            b3Quat{b3Vec3{-0.000677f, 0.760087f, 0.105674f}, 0.641171f},
        };
        bone.localFrameB = b3Transform{
            b3Vec3{0.0f, 0.0f, 0.0f},
            b3Quat{b3Vec3{-0.044589f, 0.76554f, 0.053368f}, 0.639619f},
        };
        bone.twistLimit = b3Vec2{-5.0f * 0.01745329251f, 45.0f * 0.01745329251f};
    }
    {
        Bone* bone = human.bones + bone_thigh_r;
        bone.parentIndex = bone_pelvis;
        bodyDef.name = "thigh_r";
        bone.referenceFrame = b3Transform{
            b3Vec3{-0.090416f, 0.986104f, -0.03509f},
            b3Quat{b3Vec3{-0.703287f, 0.070715f, -0.053865f}, 0.705326f},
        };
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = b3CreateBody(worldId, &bodyDef);
        var capsule = b3Capsule{
            b3Vec3{-0.023719f, 0.006008f, -0.039068f},
            b3Vec3{0.064492f, -0.004664f, -0.424718f},
            0.09f,
        };
        shapeDef.filter.groupIndex = -groupIndex;
        shapeDef.baseMaterial.customColor = cast(u32, colorize != 0 ? pantColor : 0);
        b3CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule);
        bone.jointType = b3_sphericalJoint;
        bone.localFrameA = b3Transform{
            b3Vec3{-0.05f, 0.011537f, -0.055326f},
            b3Quat{b3Vec3{-0.039089f, -0.714094f, 0.043177f}, 0.697623f},
        };
        bone.localFrameB = b3Transform{
            b3Vec3{0.0f, 0.0f, 0.0f},
            b3Quat{b3Vec3{0.758805f, -0.019886f, -0.651012f}, -0.001759f},
        };
        bone.swingLimit = 10.0f * 0.01745329251f;
        bone.twistLimit = b3Vec2{-30.0f * 0.01745329251f, 60.0f * 0.01745329251f};
    }
    {
        Bone* bone = human.bones + bone_calf_r;
        bone.parentIndex = bone_thigh_r;
        bodyDef.name = "calf_r";
        bone.referenceFrame = b3Transform{
            b3Vec3{-0.101198f, 0.527027f, -0.037373f},
            b3Quat{b3Vec3{-0.653327f, 0.06686f, -0.058582f}, 0.751839f},
        };
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = b3CreateBody(worldId, &bodyDef);
        var capsule = b3Capsule{
            b3Vec3{-0.00182f, 0.0f, 0.010071f},
            b3Vec3{0.077883f, 0.014825f, -0.418047f},
            0.075f,
        };
        shapeDef.filter.groupIndex = 0;
        shapeDef.baseMaterial.customColor = cast(u32, colorize != 0 ? pantColor : 0);
        b3CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule);
        bone.jointType = b3_revoluteJoint;
        bone.localFrameA = b3Transform{
            b3Vec3{0.069988f, 0.000253f, -0.453844f},
            b3Quat{b3Vec3{0.760086f, -0.000675f, -0.641171f}, -0.105676f},
        };
        bone.localFrameB = b3Transform{
            b3Vec3{0.0f, 0.0f, 0.0f},
            b3Quat{b3Vec3{0.76554f, -0.044589f, -0.639619f}, -0.053368f},
        };
        bone.twistLimit = b3Vec2{-45.0f * 0.01745329251f, 5.0f * 0.01745329251f};
    }
    {
        Bone* bone = human.bones + bone_upper_arm_l;
        bone.parentIndex = bone_spine_03;
        bodyDef.name = "upper_arm_l";
        bone.referenceFrame = b3Transform{
            b3Vec3{0.20378f, 1.484275f, -0.115897f},
            b3Quat{b3Vec3{0.143082f, 0.69598f, -0.69013f}, 0.13733f},
        };
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = b3CreateBody(worldId, &bodyDef);
        var capsule = b3Capsule{
            b3Vec3{0.0f, 0.0f, 0.0f},
            b3Vec3{-0.091118f, 0.037775f, 0.229719f},
            0.075f,
        };
        shapeDef.filter.groupIndex = 0;
        shapeDef.baseMaterial.customColor = cast(u32, colorize != 0 ? shirtColor : 0);
        b3CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule);
        bone.jointType = b3_sphericalJoint;
        bone.localFrameA = b3Transform{
            b3Vec3{0.20378f, -0.069369f, -0.181921f},
            b3Quat{b3Vec3{-0.278486f, 0.4456f, -0.097014f}, 0.845266f},
        };
        bone.localFrameB = b3Transform{
            b3Vec3{0.0f, 0.0f, 0.0f},
            b3Quat{b3Vec3{-0.201396f, -0.001586f, 0.90185f}, 0.382234f},
        };
        bone.swingLimit = 60.0f * 0.01745329251f;
        bone.twistLimit = b3Vec2{-5.0f * 0.01745329251f, 5.0f * 0.01745329251f};
    }
    {
        Bone* bone = human.bones + bone_lower_arm_l;
        bone.parentIndex = bone_upper_arm_l;
        bodyDef.name = "lower_arm_l";
        bone.referenceFrame = b3Transform{
            b3Vec3{0.305614f, 1.242908f, -0.117599f},
            b3Quat{b3Vec3{0.165048f, 0.563437f, -0.802002f}, 0.109959f},
        };
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = b3CreateBody(worldId, &bodyDef);
        var capsule = b3Capsule{
            b3Vec3{0.0f, 0.0f, 0.0f},
            b3Vec3{-0.142406f, 0.039392f, 0.261092f},
            0.05f,
        };
        shapeDef.filter.groupIndex = 0;
        shapeDef.baseMaterial.customColor = cast(u32, colorize != 0 ? skinColor : 0);
        b3CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule);
        bone.jointType = b3_revoluteJoint;
        bone.localFrameA = b3Transform{
            b3Vec3{-0.095482f, 0.039584f, 0.240723f},
            b3Quat{b3Vec3{0.512487f, -0.180629f, 0.839474f}, 0.003742f},
        };
        bone.localFrameB = b3Transform{
            b3Vec3{0.0f, 0.0f, 0.0f},
            b3Quat{b3Vec3{0.503803f, -0.029831f, 0.858168f}, 0.094017f},
        };
        bone.twistLimit = b3Vec2{-5.0f * 0.01745329251f, 60.0f * 0.01745329251f};
    }
    {
        Bone* bone = human.bones + bone_upper_arm_r;
        bone.parentIndex = bone_spine_03;
        bodyDef.name = "upper_arm_r";
        bone.referenceFrame = b3Transform{
            b3Vec3{-0.20378f, 1.484276f, -0.115899f},
            b3Quat{b3Vec3{0.143083f, -0.695978f, 0.690132f}, 0.137329f},
        };
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = b3CreateBody(worldId, &bodyDef);
        var capsule = b3Capsule{
            b3Vec3{0.0f, 0.0f, 0.0f},
            b3Vec3{0.091118f, 0.037775f, 0.229718f},
            0.075f,
        };
        shapeDef.filter.groupIndex = 0;
        shapeDef.baseMaterial.customColor = cast(u32, colorize != 0 ? shirtColor : 0);
        b3CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule);
        bone.jointType = b3_sphericalJoint;
        bone.localFrameA = b3Transform{
            b3Vec3{-0.203779f, -0.069371f, -0.181922f},
            b3Quat{b3Vec3{-0.253621f, -0.414842f, 0.106962f}, 0.867261f},
        };
        bone.localFrameB = b3Transform{
            b3Vec3{0.0f, 0.0f, 0.0f},
            b3Quat{b3Vec3{-0.201397f, 0.001587f, -0.90185f}, 0.382233f},
        };
        bone.swingLimit = 60.0f * 0.01745329251f;
        bone.twistLimit = b3Vec2{-5.0f * 0.01745329251f, 5.0f * 0.01745329251f};
    }
    {
        Bone* bone = human.bones + bone_lower_arm_r;
        bone.parentIndex = bone_upper_arm_r;
        bodyDef.name = "lower_arm_r";
        bone.referenceFrame = b3Transform{
            b3Vec3{-0.305614f, 1.242907f, -0.117599f},
            b3Quat{b3Vec3{0.165048f, -0.563437f, 0.802002f}, 0.109959f},
        };
        bodyDef.rotation = bone.referenceFrame.q;
        bodyDef.position = b3OffsetPos(position, bone.referenceFrame.p);
        bone.bodyId = b3CreateBody(worldId, &bodyDef);
        var capsule = b3Capsule{
            b3Vec3{0.0f, 0.0f, 0.0f},
            b3Vec3{0.142406f, 0.039392f, 0.261092f},
            0.05f,
        };
        shapeDef.filter.groupIndex = 0;
        shapeDef.baseMaterial.customColor = cast(u32, colorize != 0 ? skinColor : 0);
        b3CreateCapsuleShape(bone.bodyId, &shapeDef, &capsule);
        bone.jointType = b3_revoluteJoint;
        bone.localFrameA = b3Transform{
            b3Vec3{0.095484f, 0.039585f, 0.240723f},
            b3Quat{b3Vec3{-0.180627f, 0.512487f, -0.003744f}, -0.839474f},
        };
        bone.localFrameB = b3Transform{
            b3Vec3{0.0f, 0.0f, 0.0f},
            b3Quat{b3Vec3{-0.029831f, 0.503803f, -0.094017f}, -0.858169f},
        };
        bone.twistLimit = b3Vec2{-60.0f * 0.01745329251f, 5.0f * 0.01745329251f};
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
