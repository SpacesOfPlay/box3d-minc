// Ragdoll scenes. Ports of samples/sample_ragdoll.cpp.

import box3d;
import box3d_human;
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

// samples/sample_ragdoll.cpp RagdollOnBox
Human g_rb_human;
f32 g_rb_joint_friction_torque;
f32 g_rb_joint_hertz;
f32 g_rb_joint_damping_ratio;

void ragdoll_on_box_spawn() {
    CreateHuman(&g_rb_human, g_world, b3Pos{0.0f, 2.0f, 0.0f}, g_rb_joint_friction_torque,
                g_rb_joint_hertz, g_rb_joint_damping_ratio, 1, null, false);
    // Human_ApplyRandomAngularImpulse( &m_human, 10.0f );
}

void build_ragdoll_on_box() {
    ignore add_ground_box(20.0f);

    g_rb_joint_friction_torque = 5.0f;
    g_rb_joint_hertz = 1.0f;
    g_rb_joint_damping_ratio = 0.7f;

    g_rb_human = Human{};

    ragdoll_on_box_spawn();
}

bool ragdoll_on_box_controls() {
    ImGui_PushItemWidth(6.0f * ImGui_GetFontSize());

    if ImGui_SliderFloat("Joint Friction", &g_rb_joint_friction_torque, 0.0f, 20.0f, "%3.0f", 0) {
        Human_SetJointFrictionTorque(&g_rb_human, g_rb_joint_friction_torque);
    }

    if ImGui_SliderFloat("Hertz", &g_rb_joint_hertz, 0.0f, 20.0f, "%3.1f", 0) {
        Human_SetJointSpringHertz(&g_rb_human, g_rb_joint_hertz);
    }

    if ImGui_SliderFloat("Damping", &g_rb_joint_damping_ratio, 0.0f, 4.0f, "%3.1f", 0) {
        Human_SetJointDampingRatio(&g_rb_human, g_rb_joint_damping_ratio);
    }

    if ImGui_Button("Respawn", ImVec2{0.0f, 0.0f}) {
        DestroyHuman(&g_rb_human);
        ragdoll_on_box_spawn();
    }
    ImGui_PopItemWidth();
    return true;
}

// samples/sample_ragdoll.cpp RagdollOnMesh
b3MeshData* g_rm_ground_mesh;
b3BodyId g_rm_ground;
Human g_rm_human;
f32 g_rm_joint_friction_torque;
f32 g_rm_joint_hertz;
f32 g_rm_joint_damping_ratio;

void ragdoll_on_mesh_spawn() {
    CreateHuman(&g_rm_human, g_world, b3Pos{0.0f, 1.0f, 0.0f}, g_rm_joint_friction_torque,
                g_rm_joint_hertz, g_rm_joint_damping_ratio, 1, null, false);
    // Human_AlignSpring( &m_human, m_worldId, m_groundId, 25.0f, 1.0f );
    //  Human_ApplyRandomAngularImpulse( &m_human, 10.0f );
    // b3Body_SetType( m_human.bones[bone_thigh_l].bodyId, b3_kinematicBody );
    // b3Body_SetType( m_human.bones[bone_thigh_r].bodyId, b3_kinematicBody );
    // b3Body_SetType( m_human.bones[bone_pelvis].bodyId, b3_kinematicBody );
    // Human_CreateMotorAnchors( &m_human, m_worldId );
    Human_CreateParallelAnchors(&g_rm_human, g_world);
}

void build_ragdoll_on_mesh() {
    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        g_rm_ground = b3CreateBody(g_world, &bodyDef);

        b3ShapeDef shapeDef = b3DefaultShapeDef();
        g_rm_ground_mesh = b3CreateGridMesh(20, 20, 2.0f, 2, true);
        ignore b3CreateMeshShape(g_rm_ground, &shapeDef, g_rm_ground_mesh, b3Vec3_one);
    }

    {
        b3Transform transform;
        transform.p = b3Vec3{0.0f, 5.0f, -20.0f};
        transform.q = b3Quat_identity;
        b3BoxHull wallBox = b3MakeTransformedBoxHull(20.0f, 5.0f, 0.1f, transform);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        ignore b3CreateHullShape(g_rm_ground, &shapeDef, &wallBox.base);
    }

    {
        b3Transform transform;
        transform.p = b3Vec3{0.0f, 5.0f, 20.0f};
        transform.q = b3Quat_identity;
        b3BoxHull wallBox = b3MakeTransformedBoxHull(20.0f, 5.0f, 0.1f, transform);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        ignore b3CreateHullShape(g_rm_ground, &shapeDef, &wallBox.base);
    }

    {
        b3Transform transform;
        transform.p = b3Vec3{-20.0f, 5.0f, 0.0f};
        transform.q = b3Quat_identity;
        b3BoxHull wallBox = b3MakeTransformedBoxHull(0.1f, 5.0f, 20.0f, transform);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        ignore b3CreateHullShape(g_rm_ground, &shapeDef, &wallBox.base);
    }

    {
        b3Transform transform;
        transform.p = b3Vec3{20.0f, 5.0f, 0.0f};
        transform.q = b3Quat_identity;
        b3BoxHull wallBox = b3MakeTransformedBoxHull(0.1f, 5.0f, 20.0f, transform);
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        ignore b3CreateHullShape(g_rm_ground, &shapeDef, &wallBox.base);
    }

    g_rm_joint_friction_torque = 5.0f;
    g_rm_joint_hertz = 2.0f;
    g_rm_joint_damping_ratio = 0.7f;

    g_rm_human = Human{};

    ragdoll_on_mesh_spawn();
}

void destroy_ragdoll_on_mesh() {
    b3DestroyMesh(g_rm_ground_mesh);
}

bool ragdoll_on_mesh_controls() {
    ImGui_PushItemWidth(6.0f * ImGui_GetFontSize());

    if ImGui_SliderFloat("Joint Friction", &g_rm_joint_friction_torque, 0.0f, 20.0f, "%3.0f", 0) {
        Human_SetJointFrictionTorque(&g_rm_human, g_rm_joint_friction_torque);
    }

    if ImGui_SliderFloat("Hertz", &g_rm_joint_hertz, 0.0f, 20.0f, "%3.1f", 0) {
        Human_SetJointSpringHertz(&g_rm_human, g_rm_joint_hertz);
    }

    if ImGui_SliderFloat("Damping", &g_rm_joint_damping_ratio, 0.0f, 4.0f, "%3.1f", 0) {
        Human_SetJointDampingRatio(&g_rm_human, g_rm_joint_damping_ratio);
    }

    if ImGui_Button("Respawn", ImVec2{0.0f, 0.0f}) {
        DestroyHuman(&g_rm_human);
        ragdoll_on_mesh_spawn();
    }
    ImGui_PopItemWidth();
    return true;
}

// samples/sample_ragdoll.cpp RagdollPile
const i32 RP_COUNT = 20;
b3MeshData* g_rp_ground_mesh;
Human[RP_COUNT] g_rp_humans;

void build_ragdoll_pile() {
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.position = b3Pos{0.0f, -1.0f, 0.0f};
    b3BodyId groundId = b3CreateBody(g_world, &bodyDef);

    b3ShapeDef shapeDef = b3DefaultShapeDef();
    g_rp_ground_mesh = b3CreateGridMesh(20, 20, 1.0f, 1, true);
    ignore b3CreateMeshShape(groundId, &shapeDef, g_rp_ground_mesh, b3Vec3_one);

    g_randomSeed = cast(u32, 42);
    f32 a = 0.1f * cast(f32, RP_COUNT);
    b3Vec3 lower = b3Vec3{-a, -a, -a};
    b3Vec3 upper = b3Vec3{a, a, a};
    for i32 i = 0; i < RP_COUNT; i += 1 {
        g_rp_humans[i] = Human{};
        b3Vec3 offset = random_vec3(lower, upper);
        b3Pos position = b3Pos{offset.x, 2.0f, offset.z};
        f32 torque = 10.0f;
        f32 hertz = 0.5f;
        f32 damping = 0.7f;
        i32 groupIndex = i;
        void* userData = null;
        bool colorize = false;
        CreateHuman(&g_rp_humans[i], g_world, position, torque, hertz, damping, groupIndex,
                    userData, colorize);
    }
}

void destroy_ragdoll_pile() {
    b3DestroyMesh(g_rp_ground_mesh);
}

// samples/sample_ragdoll.cpp RagdollIncline
b3MeshData* g_ri_ground_mesh;
Human g_ri_human;
f32 g_ri_time;
bool g_ri_motorized;

void build_ragdoll_incline() {
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    g_ri_ground_mesh = b3CreateGridMesh(4, 4, 2.0f, 1, true);

    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{-10.0f, 2.0f, 0.0f};
        bodyDef.rotation = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, -0.2f * PI_F);
        b3BodyId groundId = b3CreateBody(g_world, &bodyDef);
        ignore b3CreateMeshShape(groundId, &shapeDef, g_ri_ground_mesh, b3Vec3_one);
    }

    {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.position = b3Pos{0.0f, 0.0f, 0.0f};
        b3BodyId groundId = b3CreateBody(g_world, &bodyDef);
        b3Vec3 scale = b3Vec3{4.0f, 4.0f, 4.0f};
        ignore b3CreateMeshShape(groundId, &shapeDef, g_ri_ground_mesh, scale);
    }

    g_ri_human = Human{};
    b3Pos position = b3Pos{-12.0f, 6.0f, 0.0f};
    f32 torque = 10.0f;
    f32 hertz = 2.0f;
    f32 damping = 0.7f;
    i32 groupIndex = 1;
    void* userData = null;
    bool colorize = false;
    CreateHuman(&g_ri_human, g_world, position, torque, hertz, damping, groupIndex, userData,
                colorize);
    g_ri_time = 0.0f;
    g_ri_motorized = true;
}

void destroy_ragdoll_incline() {
    b3DestroyMesh(g_ri_ground_mesh);
}

void step_ragdoll_incline(f32 timeStep) {
    ignore timeStep;
    if g_ri_time > 2.0f && g_ri_motorized == true {
        Human_SetJointFrictionTorque(&g_ri_human, 0.5f);
        Human_SetJointSpringHertz(&g_ri_human, 0.5f);
        g_ri_motorized = false;
    }

    g_ri_time += g_hertz > 0.0f ? 1.0f / g_hertz : 0.0f;
}
