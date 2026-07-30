// Sample framework: the world and solver state every scene shares,
// the sample registry, mouse picking and the projectile launcher.

// --- world / sample state -------------------------------------------

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
import sample_mesh;
import sample_ragdoll;
import sample_manifold;
import sample_character;
import sample_tree;
import debug_adapter;

b3WorldId g_world;
i32 g_frame;
f32 g_sample_time;

// upstream Sample::m_isDebug, a static constexpr set by the build type.
// The dist ships release.
const bool SAMPLE_IS_DEBUG = false;

// upstream Sample::m_launchSpeedScale. A sample built at a small world
// scale turns it down so a launched body does not leave the scene.
const f32 DEFAULT_LAUNCH_SPEED_SCALE = 5.0f;
f32 g_launch_speed_scale = DEFAULT_LAUNCH_SPEED_SCALE;

// camera-shot bullets (SPACE), round-robin pool
const i32 MAX_SHOTS = 16;
b3BodyId[MAX_SHOTS] g_shots;
i32 g_shot_count;
i32 g_shot_next;
float3 g_eye;
bool g_reset_pending;
i32 g_switch_pending;    // -1 prev, +1 next, 0 none

// Runtime, not compile-time: cpu_count() is 1 on a wasm build without
// --threads (and on a one-core machine), so workers pin to 1 and the
// Workers slider grays out exactly when parallel stepping can't help.
bool g_single_threaded;
i32 g_max_workers = 32;   // min(32, cpu_count()) — oversubscribing the
                          // machine wins nothing and can wedge browsers

// upstream SampleContext knobs (sample.cpp Solver panel)
bool g_pause;
i32 g_single_step;
i32 g_substeps = 4;
f32 g_hertz = 60.0f;
i32 g_workers = 1;
f32 g_recycle_distance = 0.05f;   // B3_CONTACT_RECYCLE_DISTANCE = 10 * 0.005
bool g_enable_sleep = true;
bool g_enable_warm = true;
bool g_enable_continuous = true;

// upstream Sample::m_stepWhilePaused, true by default. Paused, the world
// still steps with dt = 0 — not a no-op: b3World_Step clears the move,
// sensor and contact event buffers, so a sample that reads them does not
// re-read the last step's events every frame.
bool g_step_while_paused = true;

// upstream IsKeyDown / SetKeyDown: which keys are held this frame, for
// samples that steer something rather than react to a press.
const i32 KEY_STATE_MAX = 512;
bool[KEY_STATE_MAX] g_key_down;

bool is_key_down(i32 key) {
    return key >= 0 && key < KEY_STATE_MAX ? g_key_down[key] : false;
}

void set_key_down(i32 key, bool down) {
    if key >= 0 && key < KEY_STATE_MAX { g_key_down[key] = down; }
}

// upstream Sample::ToggleThirdPerson.
void toggle_third_person() {
    if cam_third_person {
        sapp_lock_mouse(false);
        cam_third_person = false;
    } else {
        sapp_lock_mouse(true);
        cam_third_person = true;
        g_selected_valid = false;
    }
}

// upstream Sample::m_triangleIndex / m_userMaterialId: what the cursor
// is over this frame, refreshed in the frame loop after the step.
i32 g_triangle_index = -1;
u64 g_user_material_id;

// per-sample extra state
b3BodyId g_kinematic_body;
b3BodyId g_bullet_body;      // BulletVersusStack's bullet
bool g_bullet_alive;
b3BodyId g_single_box_body;  // SingleBox position readout

// JengaStack shape type (upstream m_shapeType). The radio keeps its
// choice across the rebuild; a fresh load starts on hulls.
bool g_jenga_capsule;
bool g_jenga_keep;

// FarStack offset preset; the buttons rebuild the scene keeping it
bool g_fs_keep;

// OverlapRecovery params (upstream m_* fields). Sliders rebuild the
// scene keeping the values; a plain restart resets to the defaults.
bool g_or_keep;
i32 g_or_base_count;
f32 g_or_overlap;
f32 g_or_extent;
f32 g_or_speed;
f32 g_or_hertz;
f32 g_or_damping;

// A macro in box3d/math_functions.h, so it does not reach the module.
// Upstream's literal rather than PI_F / 180, to round the same way.
const f32 B3_DEG_TO_RAD = 0.01745329251f;

// --- upstream shared/utils.h RNG -------------------------------------
//
// XorShift32 rather than rand(), for the same stream on every platform.
// Sample::Sample reseeds before each scene is built (sample.cpp:320).
// g_randomSeed itself comes from box3d_human — it is shared/utils.c's
// global, and the transpiled shared/human.c reads it through utils.h's
// inline helpers. One seed, the way upstream has it.
const i32 RAND_LIMIT = 32767;
const u32 RAND_SEED = 12345;

i32 random_int() {
    u32 x = g_randomSeed;
    x = x ^ (x << 13);
    x = x ^ (x >> 17);
    x = x ^ (x << 5);
    g_randomSeed = x;
    return cast(i32, x % (cast(u32, RAND_LIMIT) + 1));
}

// [-1, 1]
f32 random_float() {
    f32 r = cast(f32, random_int() & RAND_LIMIT);
    r /= cast(f32, RAND_LIMIT);
    return 2.0f * r - 1.0f;
}

// Random integer in range [lo, hi]
i32 random_int_range(i32 lo, i32 hi) {
    return lo + random_int() % (hi - lo + 1);
}

f32 random_float_range(f32 lo, f32 hi) {
    f32 r = cast(f32, random_int() & RAND_LIMIT);
    r /= cast(f32, RAND_LIMIT);
    return (hi - lo) * r + lo;
}

b3Vec3 random_vec3(b3Vec3 lo, b3Vec3 hi) {
    return b3Vec3{random_float_range(lo.x, hi.x),
                  random_float_range(lo.y, hi.y),
                  random_float_range(lo.z, hi.z)};
}

b3Vec3 random_vec3_uniform(f32 lo, f32 hi) {
    return b3Vec3{random_float_range(lo, hi),
                  random_float_range(lo, hi),
                  random_float_range(lo, hi)};
}

// Generate uniformly distributed random quaternion using Shoemake's method
// Reference: "Uniform Random Rotations", Ken Shoemake, Graphics Gems III, 1992
b3Quat random_quat() {
    f32 u1 = random_float_range(0.0f, 1.0f);
    f32 u2 = random_float_range(0.0f, 2.0f * PI_F);
    f32 u3 = random_float_range(0.0f, 2.0f * PI_F);

    f32 sqrt1MinusU1 = sqrtf(1.0f - u1);
    f32 sqrtU1 = sqrtf(u1);

    b3CosSin cs2 = b3ComputeCosSin(u2);
    b3CosSin cs3 = b3ComputeCosSin(u3);

    b3Quat q;
    q.v.x = sqrt1MinusU1 * cs2.sine;
    q.v.y = sqrt1MinusU1 * cs2.cosine;
    q.v.z = sqrtU1 * cs3.sine;
    q.s = sqrtU1 * cs3.cosine;
    return q;
}

// Generate uniformly distributed random quaternion using Shoemake's method
// Reference: "Uniform Random Rotations", Ken Shoemake, Graphics Gems III, 1992
b3Vec3 random_unit_vector() {
    f32 u1 = random_float_range(0.0f, 1.0f);
    f32 u2 = random_float_range(0.0f, 2.0f * PI_F);
    f32 u3 = random_float_range(0.0f, 2.0f * PI_F);

    f32 sqrt1MinusU1 = sqrtf(1.0f - u1);
    f32 sqrtU1 = sqrtf(u1);

    b3CosSin cs2 = b3ComputeCosSin(u2);
    b3CosSin cs3 = b3ComputeCosSin(u3);

    b3Vec3 v;
    v.x = sqrt1MinusU1 * cs2.sine;
    v.y = sqrt1MinusU1 * cs2.cosine;
    v.z = sqrtU1 * cs3.sine;

    return v;
}

b3Pos random_pos(b3Vec3 lo, b3Vec3 hi) {
    return b3Pos{random_float_range(lo.x, hi.x),
                 random_float_range(lo.y, hi.y),
                 random_float_range(lo.z, hi.z)};
}

// upstream Sample::AddGroundBox(extent): ground body at y=-1 with a
// (extent, 1, extent) hull.
b3BodyId add_ground_box(f32 extent) {
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.position = b3Pos{0.0f, -1.0f, 0.0f};
    b3BodyId groundId = b3CreateBody(g_world, &bodyDef);
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    b3BoxHull hull = b3MakeBoxHull(extent, 1.0f, extent);
    b3ShapeId groundShapeId = b3CreateHullShape(groundId, &shapeDef, &hull.base);
    set_ground_shape(groundShapeId);
    return groundId;
}

// One entry per upstream sample: category/name, the camera SetView
// arguments from its constructor (yawDeg, pitchDeg, radius, pivot),
// the build function, and optional per-step logic.

type BuildFn = fn(): void;
type StepFn = fn(f32): void;
type ControlsFn = fn(): bool;
// upstream Sample::MouseDown( b3Vec2 p, int button, int modifiers ).
// Returning false runs sample_mouse_down, the base implementation.
type MouseDownFn = fn(f32, f32, i32, i32): bool;
// upstream Sample::MouseUp( b3Vec2 p, int button ) and
// Sample::MouseMove( b3Vec2 p ). Unlike MouseDown these have no base
// implementation to fall back to, so they return nothing: the framework
// runs its own picking either way.
type MouseUpFn = fn(f32, f32, i32): void;
type MouseMoveFn = fn(f32, f32): void;
// upstream Sample::Keyboard( int key, int action, int modifiers ).
type KeyboardFn = fn(i32, i32, i32): void;
// upstream ~Sample(): release what the sample allocated outside the world.
type DestroyFn = fn(): void;
// upstream Sample::CreateWorld( b3Capacity* ): allocation hints applied
// to the world def before b3CreateWorld.
type CapacityFn = fn(b3Capacity*): void;

struct SampleDef {
    str category;
    str name;
    f32 yawDeg;
    f32 pitchDeg;
    f32 radius;
    float3 pivot;
    BuildFn build;
    // Upstream's Step() override straddles Sample::Step(). Work ahead of
    // it — velocity control, kinematic drives, spawners — goes in
    // preStep; everything after it, which is most samples and all of
    // Render(), goes in step.
    StepFn preStep;
    StepFn step;
    ControlsFn controls;
    MouseDownFn mouseDown;
    MouseUpFn mouseUp;
    MouseMoveFn mouseMove;
    KeyboardFn keyboard;
    DestroyFn destroy;
    CapacityFn capacity;
    // upstream Sample::HasSolverControls: a debug sample hides the
    // Solver section. Inverted here so the default is upstream's true.
    bool hideSolverControls;
}

const i32 NUM_SAMPLES = 157;
SampleDef[NUM_SAMPLES] g_samples;
i32 g_sample;
bool g_sample_loaded;

void def_sample(i32 i, str category, str name, f32 yawDeg, f32 pitchDeg,
                f32 radius, float3 pivot, BuildFn build, StepFn step) {
    g_samples[i] = SampleDef{category, name, yawDeg, pitchDeg, radius, pivot, build, null, step, null, null, null, null, null, null, null, false};
}

void setup_samples() {
    StepFn nostep = null;
    def_sample(0, "Stacking", "Single Box",       0.0f, 25.0f, 10.0f, float3{0.0f, 0.0f, 0.0f}, build_single_box, step_single_box);
    def_sample(1, "Stacking", "Box Stack",        0.0f, 15.0f, 50.0f, float3{0.0f, 20.0f, 0.0f}, build_box_stack, nostep);
    def_sample(2, "Stacking", "Sphere Stack",     0.0f, 15.0f, 50.0f, float3{0.0f, 10.0f, 0.0f}, build_sphere_stack, nostep);
    def_sample(3, "Stacking", "Capsule Stack",    0.0f, 15.0f, 50.0f, float3{0.0f, 10.0f, 0.0f}, build_capsule_stack, nostep);
    def_sample(4, "Stacking", "Jenga Stack",      35.0f, 15.0f, 12.0f, float3{0.0f, 2.0f, 0.0f}, build_jenga_stack, nostep);
    def_sample(5, "Stacking", "Card House",       30.0f, 10.0f, 3.0f, float3{0.75f, 1.0f, 0.4f}, build_card_house, nostep);
    def_sample(6, "Stacking", "Dominoes",         0.0f, 15.0f, 75.0f, float3{0.0f, 0.0f, 0.0f}, build_dominoes, nostep);
    def_sample(7, "Stacking", "Double Domino",    0.0f, 15.0f, 15.0f, float3{0.0f, 0.5f, 1.0f}, build_double_domino, nostep);
    def_sample(8, "Stacking", "Pyramid2D",        0.0f, 30.0f, 50.0f, float3{0.0f, 5.0f, 0.0f}, build_pyramid_2d, nostep);
    def_sample(9, "Benchmark", "Large Pyramid",   40.0f, -10.0f, 110.0f, float3{0.0f, 40.0f, 0.0f}, build_large_pyramid, nostep);
    def_sample(10, "Benchmark", "Wide Pyramid",    0.0f, 5.0f, 80.0f, float3{0.0f, 18.0f, 0.0f}, build_wide_pyramid, nostep);
    def_sample(11, "Benchmark", "Many Pyramids",   -10.0f, 10.0f, 120.0f, float3{0.0f, 5.0f, 0.0f}, build_many_pyramids, nostep);
    def_sample(12, "Benchmark", "Falling Boxes",   45.0f, 10.0f, 80.0f, float3{0.0f, 20.0f, 0.0f}, build_falling_boxes, nostep);
    def_sample(13, "Robustness", "Tiny Pyramid",   -30.0f, 20.0f, 10.0f, float3{0.0f, 0.5f, 0.0f}, build_tiny_pyramid, step_tiny_pyramid);
    def_sample(14, "Robustness", "HighMassRatio1", 30.0f, 15.0f, 70.0f, float3{0.0f, 0.0f, 0.0f}, build_high_mass_ratio, nostep);
    def_sample(15, "Robustness", "Overlap Recovery", 45.0f, 20.0f, 15.0f, float3{0.0f, 0.0f, 0.0f}, build_overlap_recovery, nostep);
    def_sample(16, "Bodies", "Kinematic",          0.0f, 30.0f, 10.0f, float3{0.0f, 1.5f, 0.0f}, build_kinematic, step_kinematic);
    def_sample(17, "Bodies", "Spinning Book",      0.0f, 30.0f, 10.0f, float3{0.0f, 1.0f, 0.0f}, build_spinning_books, nostep);
    def_sample(18, "Continuous", "Bullet vs Stack", 15.0f, 20.0f, 30.0f, float3{0.0f, 2.0f, 0.0f}, build_bullet_vs_stack, nostep);
    def_sample(19, "Continuous", "Thin Wall",      45.0f, 30.0f, 30.0f, float3{0.0f, 0.0f, 0.0f}, build_thin_wall, nostep);
    def_sample(20, "Continuous", "Bounce House",   45.0f, 45.0f, 50.0f, float3{0.0f, 0.0f, 0.0f}, build_bounce_house, nostep);
    def_sample(21, "Continuous", "Is Fast",        0.0f, 15.0f, 50.0f, float3{0.0f, 15.0f, 0.0f}, build_is_fast, nostep);
    def_sample(22, "Shapes", "Restitution",        0.0f, 25.0f, 85.0f, float3{0.0f, 20.0f, 0.0f}, build_restitution, nostep);
    def_sample(23, "Shapes", "Inclined Plane",     -55.0f, 30.0f, 60.0f, float3{0.0f, 7.5f, 0.0f}, build_inclined_plane, nostep);
    def_sample(24, "Shapes", "Isotropic Friction", 45.0f, 30.0f, 150.0f, float3{0.0f, 0.0f, 0.0f}, build_isotropic_friction, nostep);
    def_sample(25, "Shapes", "Rolling Resistance", -140.0f, 17.0f, 60.0f, float3{0.0f, 7.5f, 0.0f}, build_rolling_resistance, nostep);
    def_sample(26, "Shapes", "High Resistance",    0.0f, 5.0f, 40.0f, float3{0.0f, 7.5f, 0.0f}, build_high_resistance, nostep);

    def_sample(27, "Stacking", "Edge Crossing",    0.0f, 25.0f, 10.0f, float3{0.0f, 0.0f, 0.0f}, build_edge_crossing, nostep);
    def_sample(28, "Bodies", "Lock Mixing",      45.0f, 30.0f, 40.0f, float3{0.0f, 0.0f, 0.0f}, build_lock_mixing, nostep);
    def_sample(29, "Bodies", "Fixed Rotation",   0.0f, 15.0f, 10.0f, float3{0.0f, 0.0f, 0.0f}, build_fixed_rotation, nostep);
    def_sample(30, "Bodies", "Disable",          45.0f, 25.0f, 10.0f, float3{0.0f, 0.0f, 0.0f}, build_disable, step_disable);
    def_sample(31, "Bodies", "Body Type",        0.0f, 30.0f, 30.0f, float3{0.0f, 1.5f, 0.0f}, build_body_type, step_body_type);
    def_sample(32, "Robustness", "Overflow Color Pile", 30.0f, 35.0f, 15.0f, float3{0.0f, 0.0f, 0.0f}, build_overflow_color_pile, nostep);
    def_sample(33, "Continuous", "Spinning Stick",  45.0f, 25.0f, 20.0f, float3{0.0f, 2.0f, 0.0f}, build_spinning_stick, nostep);
    def_sample(34, "Shapes", "Slide Twist",      -30.0f, 17.0f, 30.0f, float3{0.0f, 5.0f, 0.0f}, build_slide_twist, nostep);
    def_sample(35, "Shapes", "Static Invoke",    0.0f, 25.0f, 10.0f, float3{0.0f, 1.0f, 0.0f}, build_static_invoke, step_static_invoke);
    def_sample(36, "Stacking", "Cylinder",       0.0f, 15.0f, 10.0f, float3{0.0f, 0.0f, 0.0f}, build_cylinder, nostep);
    def_sample(37, "Stacking", "Cylinder Stack", 0.0f, 15.0f, 15.0f, float3{0.0f, 5.0f, 0.0f}, build_cylinder_stack, nostep);
    def_sample(38, "Stacking", "Wedge",          75.0f, 10.0f, 10.0f, float3{0.0f, 0.0f, 0.0f}, build_wedge, nostep);
    def_sample(39, "Stacking", "Arch",           25.0f, 10.0f, 30.0f, float3{0.0f, 5.0f, 0.0f}, build_arch, nostep);
    def_sample(40, "Joints", "Filter",           45.0f, 30.0f, 15.0f, float3{0.0f, 2.0f, 0.0f}, build_filter_joint, nostep);
    def_sample(41, "Joints", "Ball and Chain",   180.0f, 15.0f, 50.0f, float3{0.0f, -20.0f, 0.0f}, build_ball_and_chain, nostep);
    def_sample(42, "Joints", "Bridge",           0.0f, 20.0f, 35.0f, float3{0.0f, 10.0f, 0.0f}, build_bridge, nostep);
    def_sample(43, "Joints", "Weld",             45.0f, 30.0f, 15.0f, float3{0.0f, 2.0f, 0.0f}, build_weld_joint, nostep);
    def_sample(44, "Joints", "Distance Joint",   0.0f, 0.0f, 40.0f, float3{0.0f, 10.0f, 0.0f}, build_distance_joint, nostep);
    def_sample(45, "Joints", "Revolute",         45.0f, 30.0f, 15.0f, float3{0.0f, 2.0f, 0.0f}, build_revolute_joint, step_revolute_joint);
    def_sample(46, "Joints", "Motor Joint",      0.0f, 0.0f, 25.0f, float3{0.0f, 8.0f, 0.0f}, build_motor_joint, step_motor_joint);
    def_sample(47, "Joints", "Prismatic",        45.0f, 30.0f, 15.0f, float3{0.0f, 2.0f, 0.0f}, build_prismatic_joint, nostep);
    def_sample(48, "Joints", "Spherical",        45.0f, 30.0f, 15.0f, float3{0.0f, 2.0f, 0.0f}, build_spherical_joint, nostep);
    def_sample(49, "Joints", "Parallel Spring",  45.0f, 30.0f, 15.0f, float3{0.0f, 2.0f, 0.0f}, build_parallel_joint, nostep);
    def_sample(50, "Joints", "Top Down Friction", 0.0f, 0.0f, 26.0f, float3{0.0f, 10.0f, 0.0f}, build_top_down_friction, nostep);
    def_sample(51, "Joints", "Wheel",            25.0f, 20.0f, 7.0f, float3{0.0f, 2.0f, 0.0f}, build_wheel_joint, nostep);
    def_sample(52, "Joints", "Motion Locks",     0.0f, 30.0f, 40.0f, float3{0.0f, 5.0f, 0.0f}, build_motion_locks, nostep);
    def_sample(53, "Joints", "Door",             45.0f, 30.0f, 15.0f, float3{0.0f, 2.0f, 0.0f}, build_door, step_door);
    def_sample(54, "Compound", "Simple",         45.0f, 30.0f, 45.0f, float3{0.0f, 0.0f, 0.0f}, build_simple_compound, nostep);
    def_sample(55, "Compound", "Spheres",        45.0f, 30.0f, 45.0f, float3{0.0f, 0.0f, 0.0f}, build_compound_spheres, step_compound_spheres);
    def_sample(56, "Compound", "Hulls",          45.0f, 30.0f, 45.0f, float3{0.0f, 0.0f, 0.0f}, build_compound_hulls, step_compound_hulls);
    def_sample(57, "Compound", "Tile Floor",     45.0f, 30.0f, 45.0f, float3{0.0f, 0.0f, 0.0f}, build_tile_floor, step_tile_floor);
    def_sample(58, "World", "Far Stack",         0.0f, 8.0f, 16.0f, float3{0.0f, 2.0f, 0.0f}, build_far_stack, step_far_stack);
    def_sample(59, "World", "Far Pyramid",       40.0f, -10.0f, 60.0f, float3{10000000.0f, 20.0f, 0.0f}, build_far_pyramid, nostep);
    def_sample(60, "Bodies", "Gyroscopic Torque", 0.0f, 20.0f, 4.0f, float3{0.0f, 2.0f, 0.0f}, build_gyroscopic_torque, step_gyroscopic_torque);
    def_sample(61, "Bodies", "Weeble",           45.0f, 25.0f, 25.0f, float3{0.0f, 0.0f, 0.0f}, build_weeble, step_weeble);
    def_sample(62, "Issues", "Restitution Overshoot", 20.0f, 0.0f, 28.0f, float3{0.0f, 10.5f, 0.0f}, build_restitution_overshoot, step_restitution_overshoot);
    def_sample(63, "Issues", "Slide Twist Off Center Shape", -30.0f, 17.0f, 30.0f, float3{0.0f, 5.0f, 0.0f}, build_slide_twist_off_center, nostep);
    def_sample(64, "Issues", "Multiple Prismatic", 0.0f, 0.0f, 25.0f, float3{0.0f, 5.0f, 0.0f}, build_multiple_prismatic, nostep);
    def_sample(65, "Issues", "Hull Crash",        0.0f, 15.0f, 5.0f, float3{0.0f, 0.0f, 0.0f}, build_hull_crash, step_hull_crash);
    def_sample(66, "Issues", "Convex Jitter",     0.0f, 15.0f, 10.0f, float3{0.0f, 2.0f, 0.0f}, build_convex_jitter, nostep);
    def_sample(67, "Issues", "GMod Wheel Stack",  0.0f, 12.0f, 5.0f, float3{0.0f, 0.85f, 0.0f}, build_gmod_wheel_stack, step_gmod_wheel_stack);
    def_sample(68, "Benchmark", "Joint Grid",     -25.0f, 25.0f, 94.0f, float3{30.0f, -30.0f, 30.0f}, build_joint_grid, step_joint_grid);
    def_sample(69, "Benchmark", "Candy Cups",     45.0f, 20.0f, 70.0f, float3{0.0f, 0.0f, 0.0f}, build_candy_cups, nostep);
    def_sample(70, "Benchmark", "Washer",         15.0f, 20.0f, 60.0f, float3{0.0f, 15.0f, 0.0f}, build_washer, nostep);
    def_sample(71, "Benchmark", "Large World",    0.0f, 10.0f, 250.0f, float3{0.0f, 0.0f, 0.0f}, build_large_world, step_large_world);
    def_sample(72, "Benchmark", "Hull",           0.0f, 15.0f, 5.0f, float3{0.0f, 0.0f, 0.0f}, build_benchmark_hull, step_benchmark_hull);
    def_sample(73, "Benchmark", "Junkyard",       45.0f, 30.0f, 125.0f, float3{0.0f, 0.0f, 0.0f}, build_junkyard, step_junkyard);
    def_sample(74, "Benchmark", "Convex Pile",    45.0f, 20.0f, 150.0f, float3{0.0f, 15.0f, 0.0f}, build_convex_pile, nostep);
    def_sample(75, "Shapes", "Conveyor Belt",    0.0f, 25.0f, 40.0f, float3{0.0f, 1.0f, 0.0f}, build_conveyor_belt, nostep);
    def_sample(76, "Shapes", "Wind",             0.0f, 0.0f, 5.0f, float3{0.0f, 1.0f, 0.0f}, build_wind, step_wind);
    def_sample(77, "Shapes", "Wind Drop",        -45.0f, 15.0f, 20.0f, float3{0.0f, 5.0f, 0.0f}, build_wind_drop, step_wind_drop);
    def_sample(78, "Shapes", "Wind Flap",        -35.0f, 15.0f, 65.0f, float3{0.0f, 5.0f, 10.0f}, build_wind_flap, step_wind_flap);
    def_sample(79, "Events", "Sensor Visit",     0.0f, 30.0f, 20.0f, float3{0.0f, 5.0f, 0.0f}, build_sensor_visit, step_sensor_visit);
    def_sample(80, "Events", "Move",             0.0f, 30.0f, 40.0f, float3{0.0f, 5.0f, 0.0f}, build_move_event, step_move_event);
    def_sample(81, "Events", "Joint",            0.0f, 30.0f, 40.0f, float3{0.0f, 5.0f, 0.0f}, build_joint_event, step_joint_event);
    def_sample(82, "Geometry", "Box Hull",       0.0f, 15.0f, 5.0f, float3{0.0f, 0.0f, 0.0f}, build_box_hull, step_box_hull);
    def_sample(83, "Geometry", "Hull",           0.0f, 15.0f, 5.0f, float3{0.0f, 0.0f, 0.0f}, build_geometry_hull, step_geometry_hull);
    def_sample(84, "Geometry", "Hull Reduction", 0.0f, 15.0f, 5.0f, float3{0.0f, 0.0f, 0.0f}, build_hull_reduction, step_hull_reduction);
    def_sample(85, "Geometry", "Hull Transform", 0.0f, 15.0f, 5.0f, float3{0.0f, 0.0f, 0.0f}, build_hull_transform, step_hull_transform);
    def_sample(86, "Benchmark", "Sensor",       0.0f, 0.0f, 250.0f, float3{0.0f, 110.0f, 0.0f}, build_benchmark_sensor, step_benchmark_sensor);
    def_sample(87, "Bodies", "Gyroscopic Precession", 40.0f, 30.0f, 75.0f, float3{0.0f, 2.0f, 0.0f}, build_gyroscopic_precession, step_gyroscopic_precession);
    def_sample(88, "Collision", "Capsule Cast Ray", 120.0f, 30.0f, 20.0f, float3{0.0f, 1.5f, 0.0f}, build_capsule_cast_ray, step_capsule_cast_ray);
    def_sample(89, "Determinism", "Query Spawn", 45.0f, 25.0f, 30.0f, float3{0.0f, 0.0f, 0.0f}, build_query_spawn, step_query_spawn);
    def_sample(90, "Mesh", "Grid",               45.0f, 30.0f, 6.0f, float3{0.0f, 0.0f, 0.0f}, build_grid_mesh, step_grid_mesh);
    def_sample(91, "Benchmark", "Height Field", 0.0f, 20.0f, 50.0f, float3{0.0f, 0.0f, 0.0f}, build_benchmark_height_field, step_benchmark_height_field);
    def_sample(92, "Mesh", "Box",                45.0f, 30.0f, 6.0f, float3{0.0f, 0.0f, 0.0f}, build_box_mesh, nostep);
    def_sample(93, "Mesh", "Big Box",            45.0f, 30.0f, 6.0f, float3{0.0f, 0.0f, 0.0f}, build_big_box_mesh, step_big_box_mesh);
    def_sample(94, "Mesh", "Hollow Box",         45.0f, 30.0f, 30.0f, float3{0.0f, 0.0f, 0.0f}, build_hollow_box, step_hollow_box);
    def_sample(95, "Continuous", "Needle Mesh", 45.0f, 25.0f, 4.0f, float3{0.0f, 1.2f, 0.0f}, build_needle_mesh, step_needle_mesh);
    def_sample(96, "Continuous", "Hump Mesh",   45.0f, 25.0f, 10.0f, float3{0.0f, 1.2f, 0.0f}, build_hump_mesh, nostep);
    def_sample(97, "Continuous", "Stall",       130.0f, 15.0f, 15.0f, float3{0.0f, 2.0f, 0.0f}, build_stall, nostep);
    def_sample(98, "Benchmark", "Explosion",    45.0f, 20.0f, 30.0f, float3{0.0f, 0.0f, 0.0f}, build_benchmark_explosion, nostep);
    def_sample(99, "Benchmark", "Chains",       0.0f, 15.0f, 50.0f, float3{0.0f, 5.0f, 0.0f}, build_benchmark_chains, step_benchmark_chains);
    def_sample(100, "Benchmark", "Falling Trees", 20.0f, 0.0f, 140.0f, float3{0.0f, 15.0f, 0.0f}, build_falling_trees, nostep);
    def_sample(101, "Compound", "Mesh Tile",    45.0f, 30.0f, 45.0f, float3{0.0f, 0.0f, 0.0f}, build_mesh_tile, step_mesh_tile);
    def_sample(102, "Determinism", "Wave Pile", 45.0f, 25.0f, 25.0f, float3{0.0f, 0.0f, 0.0f}, build_wave_pile, step_wave_pile);
    def_sample(103, "Determinism", "Mesh Drop", 0.0f, 30.0f, 20.0f, float3{0.0f, 0.0f, 0.0f}, build_mesh_drop_determinism, step_mesh_drop_determinism);
    def_sample(104, "World", "Far Mesh Drop",   0.0f, 30.0f, 20.0f, float3{1000000.0f, 0.0f, 1000000.0f}, build_far_mesh_drop, step_far_mesh_drop);
    def_sample(105, "Collision", "Ray Curtain", 45.0f, 30.0f, 20.0f, float3{0.0f, 0.0f, 0.0f}, build_ray_curtain, step_ray_curtain);
    def_sample(106, "Issues", "Crash",          45.0f, 30.0f, 15.0f, float3{0.0f, 2.0f, 0.0f}, build_crash, nostep);
    def_sample(107, "Continuous", "Mesh Drop", 0.0f, 30.0f, 20.0f, float3{0.0f, 0.0f, 0.0f}, build_continuous_mesh_drop, step_continuous_mesh_drop);
    def_sample(108, "Benchmark", "Destruction", 0.0f, 40.0f, 30.0f, float3{0.0f, 0.0f, 0.0f}, build_destruction, step_destruction);
    def_sample(109, "Events", "Sensor Hits",   0.0f, 30.0f, 40.0f, float3{0.0f, 5.0f, 0.0f}, build_sensor_hits, step_sensor_hits);
    def_sample(110, "Collision", "Long Ray Cast", -35.0f, 22.0f, 34.0f, float3{0.0f, 1.0f, 0.0f}, build_long_ray_cast, step_long_ray_cast);
    def_sample(111, "Joints", "Gear Lift",    18.0f, 12.0f, 17.0f, float3{-1.5f, 4.5f, 0.0f}, build_gear_lift, nostep);
    def_sample(112, "Issues", "s&box mover",  45.0f, 30.0f, 12.0f, float3{0.0f, 0.0f, 0.0f}, build_sbox_mover, step_sbox_mover);
    def_sample(113, "Issues", "s&box Ghost Collisions", 90.0f, 25.0f, 10.0f, float3{0.0f, 1.0f, 0.0f}, build_sbox_ghost_collisions, step_sbox_ghost_collisions);
    def_sample(114, "Issues", "Capsule Mesh", 20.0f, 10.0f, 30.0f, float3{0.0f, 2.0f, 0.0f}, build_capsule_mesh, step_capsule_mesh);
    def_sample(115, "Mesh", "Creation Benchmark", 45.0f, 30.0f, 40.0f, float3{0.0f, 0.0f, 0.0f}, build_mesh_creation_benchmark, step_mesh_creation_benchmark);
    def_sample(117, "Ragdoll", "Box",       45.0f, 30.0f, 6.0f, float3{0.0f, 0.0f, 0.0f}, build_ragdoll_on_box, nostep);
    def_sample(118, "Ragdoll", "Mesh",      45.0f, 30.0f, 6.0f, float3{0.0f, 0.0f, 0.0f}, build_ragdoll_on_mesh, nostep);
    def_sample(119, "Ragdoll", "Pile",      180.0f, 30.0f, 20.0f, float3{0.0f, 0.0f, 0.0f}, build_ragdoll_pile, nostep);
    def_sample(120, "Ragdoll", "Incline",   -20.0f, 30.0f, 25.0f, float3{0.0f, 0.0f, 0.0f}, build_ragdoll_incline, step_ragdoll_incline);
    def_sample(121, "World", "Far Ragdolls", 180.0f, 30.0f, 20.0f, float3{1000000.0f, 0.0f, 0.0f}, build_far_ragdolls, step_far_ragdolls);
    def_sample(130, "Manifold", "Sphere vs Sphere",   35.0f, 30.0f, 50.0f, float3{0.0f, 5.0f, 0.0f}, build_sphere_and_sphere, step_sphere_and_sphere);
    def_sample(131, "Manifold", "Capsule vs Sphere",  35.0f, 30.0f, 50.0f, float3{0.0f, 5.0f, 0.0f}, build_capsule_and_sphere, step_capsule_and_sphere);
    def_sample(132, "Manifold", "Capsule vs Capsule", 35.0f, 30.0f, 50.0f, float3{0.0f, 5.0f, 0.0f}, build_capsule_and_capsule, step_capsule_and_capsule);
    def_sample(151, "Joints", "Driving", 25.0f, 20.0f, 7.0f, float3{0.0f, 2.0f, 0.0f}, build_driving, step_driving);
    def_sample(150, "Collision", "Cast World", 45.0f, 30.0f, 20.0f, float3{0.0f, 0.0f, 0.0f}, build_cast_world, step_cast_world);
    def_sample(149, "Mesh", "Viewer", 45.0f, 30.0f, 50.0f, float3{0.0f, 0.0f, 0.0f}, build_mesh_viewer, step_mesh_viewer);
    def_sample(147, "Collision", "Shape Cast",     120.0f, 30.0f, 20.0f, float3{0.0f, 1.5f, 0.0f}, build_shape_cast, step_shape_cast);
    def_sample(148, "Collision", "Distance Debug", 120.0f, 30.0f, 20.0f, float3{0.0f, 1.5f, 0.0f}, build_distance_debug, step_distance_debug);
    def_sample(145, "Collision", "Overlap World",  120.0f, 30.0f, 20.0f, float3{0.0f, 1.5f, 0.0f}, build_overlap_world, step_overlap_world);
    def_sample(146, "Collision", "Time of Impact", -90.0f, 0.0f, 10.0f, float3{0.0f, 0.0f, 0.0f}, build_time_of_impact, step_time_of_impact);
    def_sample(143, "Shapes", "Conveyor Mesh",   65.0f, 25.0f, 28.0f, float3{0.0f, 1.0f, 0.0f}, build_conveyor_mesh, step_conveyor_mesh);
    def_sample(144, "Character", "MoverOverlap", 120.0f, 25.0f, 10.0f, float3{0.0f, 1.0f, 0.0f}, build_mover_overlap, step_mover_overlap);
    def_sample(152, "Character", "Mover", 120.0f, 30.0f, 5.0f, float3{7.5f, 0.75f, 9.0f}, build_basic_mover, step_basic_mover);
    def_sample(153, "Compound", "Village", 45.0f, 10.0f, 5.0f, float3{0.0f, 10.0f, 0.0f}, build_village, step_village);
    def_sample(154, "Collision", "Shape Distance", -45.0f, 10.0f, 5.0f, float3{0.0f, 0.0f, 0.0f}, build_shape_distance, step_shape_distance);
    def_sample(155, "Character", "Rigid Body", 120.0f, 30.0f, 5.0f, float3{7.5f, 2.0f, 9.0f}, build_rigid_body_character, step_rigid_body_character);
    def_sample(156, "Tree", "Benchmark", 45.0f, 45.0f, 250.0f, float3{0.0f, 0.0f, 0.0f}, build_tree_benchmark, step_tree_benchmark);
    def_sample(141, "Character", "CapsulePlane", 120.0f, 30.0f, 20.0f, float3{0.0f, 1.5f, 0.0f}, build_capsule_plane, step_capsule_plane);
    def_sample(142, "Bodies", "Cast",            120.0f, 30.0f, 20.0f, float3{0.0f, 1.5f, 0.0f}, build_body_cast, step_body_cast);
    def_sample(139, "Events", "Hit",        0.0f, 30.0f, 100.0f, float3{0.0f, 5.0f, 0.0f}, build_hit_event, step_hit_event);
    def_sample(140, "Mesh", "Height Field", 45.0f, 30.0f, 40.0f, float3{0.0f, 0.0f, 0.0f}, build_mesh_height_field, step_mesh_height_field);
    def_sample(133, "Manifold", "Hull vs Sphere",     35.0f, 30.0f, 50.0f, float3{0.0f, 5.0f, 0.0f}, build_hull_and_sphere, step_hull_and_sphere);
    def_sample(134, "Manifold", "Capsule vs Hull",    0.0f, 30.0f, 5.0f, float3{0.0f, 0.0f, 0.0f}, build_capsule_and_hull, step_capsule_and_hull);
    def_sample(135, "Manifold", "Hull vs Hull",       0.0f, 15.0f, 4.0f, float3{0.0f, 0.0f, 0.0f}, build_hull_and_hull, step_hull_and_hull);
    def_sample(136, "Manifold", "Triangle vs Sphere", 0.0f, 30.0f, 10.0f, float3{0.0f, 0.0f, 0.0f}, build_triangle_and_sphere, step_triangle_and_sphere);
    def_sample(137, "Manifold", "Triangle vs Capsule", 0.0f, 30.0f, 10.0f, float3{0.0f, 0.0f, 0.0f}, build_triangle_and_capsule, step_triangle_and_capsule);
    def_sample(138, "Manifold", "Triangle vs Hull",   0.0f, 30.0f, 3.0f, float3{0.0f, 0.0f, 0.0f}, build_triangle_and_hull, step_triangle_and_hull);
    def_sample(129, "Events", "Persistent Contact", 0.0f, 30.0f, 40.0f, float3{0.0f, 5.0f, 0.0f}, build_persistent_contact, step_persistent_contact);
    def_sample(127, "Collision", "Mesh Scale", 45.0f, 30.0f, 20.0f, float3{0.0f, 0.0f, 0.0f}, build_mesh_scale, step_mesh_scale);
    def_sample(128, "Geometry", "Capsule Mass", 0.0f, 15.0f, 5.0f, float3{0.0f, 0.0f, 0.0f}, build_capsule_mass, step_capsule_mass);
    def_sample(125, "Benchmark", "Rain",    25.0f, 10.0f, 70.0f, float3{0.0f, 0.0f, 0.0f}, build_benchmark_rain, step_benchmark_rain);
    def_sample(126, "Determinism", "Falling Ragdolls", 45.0f, 30.0f, 40.0f, float3{0.0f, 0.0f, 0.0f}, build_falling_ragdolls, step_falling_ragdolls);
    def_sample(123, "Collision", "Shape Cast Debug", 120.0f, 30.0f, 20.0f, float3{0.0f, 1.5f, 0.0f}, build_shape_cast_debug, step_shape_cast_debug);
    def_sample(124, "Collision", "Initial Overlap", -140.0f, 10.0f, 10.0f, float3{0.0f, 0.0f, 0.0f}, build_initial_overlap, step_initial_overlap);
    def_sample(122, "Mesh", "Reflection",   45.0f, 30.0f, 40.0f, float3{0.0f, 0.0f, 0.0f}, build_mesh_reflection, step_mesh_reflection);
    def_sample(116, "Mesh", "Voxel", -115.0f, 5.0f, 5.0f, float3{5000.0f, 3510.0f, -7000.0f}, build_voxel_mesh, step_voxel_mesh);

    // Samples whose upstream Step() override does its work AHEAD of
    // Sample::Step() — velocity control, kinematic drives, spawners and
    // the wind pass. Everything else stays in `step`, which now runs
    // after the world advances the way upstream's does.
    g_samples[16].preStep = step_kinematic;          // Bodies / Kinematic
    g_samples[16].step = null;
    g_samples[30].preStep = step_disable;            // Bodies / Disable
    g_samples[30].step = null;
    g_samples[31].preStep = step_body_type;          // Bodies / Body Type
    g_samples[31].step = null;
    g_samples[46].preStep = step_motor_joint;        // Joints / Motor Joint
    g_samples[46].step = null;
    g_samples[72].preStep = step_benchmark_hull;     // Benchmark / Hull
    g_samples[72].step = null;
    g_samples[73].preStep = step_junkyard;           // Benchmark / Junkyard
    g_samples[73].step = null;
    g_samples[99].preStep = step_benchmark_chains;   // Benchmark / Chains
    g_samples[99].step = null;
    g_samples[109].preStep = step_sensor_hits;       // Events / Sensor Hits
    g_samples[109].step = null;
    g_samples[120].preStep = step_ragdoll_incline;   // Ragdoll / Incline
    g_samples[120].step = null;
    g_samples[125].preStep = step_benchmark_rain;    // Benchmark / Rain
    g_samples[125].step = null;
    g_samples[151].preStep = step_driving;           // Joints / Driving
    g_samples[151].step = null;
    // Issues / s&box Ghost Collisions straddles it.
    g_samples[113].preStep = pre_step_sbox_ghost_collisions;

    // assigned by index, so before the sort
    g_samples[4].controls = jenga_controls;
    g_samples[15].controls = or_controls;
    g_samples[18].controls = bullet_controls;
    g_samples[30].controls = disable_controls;
    g_samples[31].controls = bt_controls;
    g_samples[35].controls = si_controls;
    g_samples[42].controls = bridge_controls;
    g_samples[43].controls = weld_controls;
    g_samples[44].controls = dj_controls;
    g_samples[45].controls = revolute_controls;
    g_samples[46].controls = motor_controls;
    g_samples[47].controls = prismatic_controls;
    g_samples[48].controls = spherical_controls;
    g_samples[49].controls = parallel_controls;
    g_samples[50].controls = top_down_friction_controls;
    g_samples[51].controls = wheel_controls;
    g_samples[52].controls = motion_locks_controls;
    g_samples[53].controls = door_controls;
    g_samples[53].mouseDown = door_mouse_down;
    g_samples[58].controls = far_stack_controls;
    g_samples[61].controls = weeble_controls;
    g_samples[65].destroy = destroy_hull_crash;
    g_samples[67].destroy = destroy_gmod_wheel_stack;
    g_samples[69].destroy = destroy_candy_cups;
    g_samples[70].capacity = get_washer_capacity;
    g_samples[71].capacity = get_large_world_capacity;
    g_samples[72].destroy = destroy_benchmark_hull;
    g_samples[73].capacity = get_junkyard_capacity;
    g_samples[74].capacity = get_convex_pile_capacity;
    g_samples[90].controls = grid_mesh_controls;
    g_samples[91].controls = benchmark_height_field_controls;
    g_samples[92].controls = box_mesh_controls;
    g_samples[92].destroy = destroy_box_mesh;
    g_samples[93].controls = big_box_mesh_controls;
    g_samples[93].destroy = destroy_big_box_mesh;
    g_samples[94].destroy = destroy_hollow_box;
    g_samples[95].destroy = destroy_needle_mesh;
    g_samples[96].destroy = destroy_hump_mesh;
    g_samples[97].controls = stall_controls;
    g_samples[98].controls = benchmark_explosion_controls;
    g_samples[98].destroy = destroy_benchmark_explosion;
    g_samples[99].destroy = destroy_benchmark_chains;
    g_samples[100].controls = falling_trees_controls;
    g_samples[101].destroy = destroy_mesh_tile;
    g_samples[102].destroy = destroy_wave_pile;
    g_samples[103].destroy = destroy_mesh_drop;
    g_samples[104].destroy = destroy_mesh_drop;
    g_samples[105].destroy = destroy_ray_curtain;
    g_samples[106].controls = crash_controls;
    g_samples[107].controls = continuous_mesh_drop_controls;
    g_samples[108].capacity = get_destruction_capacity;
    g_samples[109].controls = sensor_hits_controls;
    g_samples[110].controls = long_ray_cast_controls;
    g_samples[110].destroy = destroy_long_ray_cast;
    g_samples[111].controls = gear_lift_controls;
    g_samples[111].destroy = destroy_gear_lift;
    g_samples[112].destroy = destroy_sbox_mover;
    g_samples[113].controls = sbox_ghost_collisions_controls;
    g_samples[113].destroy = destroy_sbox_ghost_collisions;
    g_samples[114].destroy = destroy_capsule_mesh;
    g_samples[115].destroy = destroy_mesh_creation_benchmark;
    g_samples[116].destroy = destroy_voxel_mesh;
    g_samples[117].controls = ragdoll_on_box_controls;
    g_samples[118].controls = ragdoll_on_mesh_controls;
    g_samples[118].destroy = destroy_ragdoll_on_mesh;
    g_samples[119].destroy = destroy_ragdoll_pile;
    g_samples[120].destroy = destroy_ragdoll_incline;
    g_samples[121].destroy = destroy_far_ragdolls;
    g_samples[122].controls = mesh_reflection_controls;
    g_samples[122].destroy = destroy_mesh_reflection;
    g_samples[123].destroy = destroy_shape_cast_debug;
    g_samples[124].controls = initial_overlap_controls;
    g_samples[124].destroy = destroy_initial_overlap;
    g_samples[124].hideSolverControls = true;
    g_samples[125].capacity = get_rain_capacity_fn;
    g_samples[125].destroy = destroy_benchmark_rain;
    g_samples[126].destroy = destroy_falling_ragdolls;
    g_samples[127].controls = mesh_scale_controls;
    g_samples[127].destroy = destroy_mesh_scale;
    g_samples[127].hideSolverControls = true;
    g_samples[128].controls = capsule_mass_controls;
    g_samples[128].destroy = destroy_capsule_mass;
    g_samples[128].hideSolverControls = true;
    g_samples[129].destroy = destroy_persistent_contact;
    g_samples[138].destroy = destroy_triangle_and_hull;
    g_samples[139].destroy = destroy_hit_event;
    g_samples[140].controls = mesh_height_field_controls;
    g_samples[140].destroy = destroy_mesh_height_field;
    g_samples[141].controls = capsule_plane_controls;
    g_samples[141].mouseDown = capsule_plane_mouse_down;
    g_samples[141].mouseUp = capsule_plane_mouse_up;
    g_samples[141].mouseMove = capsule_plane_mouse_move;
    g_samples[142].destroy = destroy_body_cast;
    g_samples[142].mouseDown = body_cast_mouse_down;
    g_samples[142].mouseUp = body_cast_mouse_up;
    g_samples[142].mouseMove = body_cast_mouse_move;
    g_samples[143].destroy = destroy_conveyor_mesh;
    g_samples[144].controls = mover_overlap_controls;
    g_samples[144].mouseDown = mover_overlap_mouse_down;
    g_samples[144].mouseUp = mover_overlap_mouse_up;
    g_samples[144].mouseMove = mover_overlap_mouse_move;
    g_samples[145].destroy = destroy_overlap_world;
    g_samples[145].mouseDown = overlap_world_mouse_down;
    g_samples[145].mouseUp = overlap_world_mouse_up;
    g_samples[145].mouseMove = overlap_world_mouse_move;
    g_samples[146].controls = time_of_impact_controls;
    g_samples[146].hideSolverControls = true;
    g_samples[147].controls = shape_cast_controls;
    g_samples[147].destroy = destroy_shape_cast;
    g_samples[147].mouseDown = shape_cast_mouse_down;
    g_samples[147].mouseUp = shape_cast_mouse_up;
    g_samples[147].mouseMove = shape_cast_mouse_move;
    g_samples[147].hideSolverControls = true;
    g_samples[148].controls = distance_debug_controls;
    g_samples[148].destroy = destroy_distance_debug;
    g_samples[148].hideSolverControls = true;
    g_samples[149].controls = mesh_viewer_controls;
    g_samples[149].destroy = destroy_mesh_viewer;
    g_samples[150].controls = cast_world_controls;
    g_samples[150].destroy = destroy_cast_world;
    g_samples[150].mouseDown = cast_world_mouse_down;
    g_samples[151].controls = driving_controls;
    g_samples[151].destroy = destroy_driving;
    g_samples[151].keyboard = driving_keyboard;
    // upstream BasicMover::Step and Village::Step are wholly ahead of
    // Sample::Step() — the mover drives velocity before the solver runs.
    g_samples[152].preStep = step_basic_mover;
    g_samples[152].step = null;
    g_samples[152].controls = basic_mover_controls;
    g_samples[152].keyboard = basic_mover_keyboard;
    g_samples[152].destroy = destroy_basic_mover;
    g_samples[153].preStep = step_village;
    g_samples[153].step = null;
    g_samples[153].keyboard = village_keyboard;
    g_samples[153].destroy = destroy_village;
    g_samples[154].controls = shape_distance_controls;
    g_samples[154].mouseDown = shape_distance_mouse_down;
    g_samples[154].mouseUp = shape_distance_mouse_up;
    g_samples[154].mouseMove = shape_distance_mouse_move;
    g_samples[154].hideSolverControls = true;
    // upstream RigidBodyCharacter::Step straddles Sample::Step(): the
    // character manipulates velocity before, corrects and follows after.
    g_samples[155].preStep = step_rigid_body_character;
    g_samples[155].step = late_step_rigid_body_character;
    g_samples[155].controls = rigid_body_character_controls;
    g_samples[155].keyboard = rigid_body_character_keyboard;
    g_samples[155].destroy = destroy_rigid_body_character;
    g_samples[156].controls = tree_benchmark_controls;
    g_samples[156].destroy = destroy_tree_benchmark;
    g_samples[156].hideSolverControls = true;
    for i32 i = 130; i <= 138; i++ {
        g_samples[i].controls = mf_controls;
        g_samples[i].mouseDown = mf_mouse_down;
        g_samples[i].mouseUp = mf_mouse_up;
        g_samples[i].mouseMove = mf_mouse_move;
        g_samples[i].hideSolverControls = true;
    }
    g_samples[109].destroy = destroy_sensor_hits;
    g_samples[108].destroy = destroy_destruction;
    g_samples[107].destroy = destroy_continuous_mesh_drop;
    g_samples[106].destroy = destroy_crash;
    g_samples[100].destroy = destroy_falling_trees;
    g_samples[97].destroy = destroy_stall;
    g_samples[91].destroy = destroy_benchmark_height_field;
    g_samples[90].destroy = destroy_grid_mesh;
    g_samples[76].controls = wind_controls;
    g_samples[82].controls = box_hull_controls;
    g_samples[82].destroy = destroy_box_hull;
    g_samples[82].hideSolverControls = true;
    g_samples[83].destroy = destroy_geometry_hull;
    g_samples[84].controls = hull_reduction_controls;
    g_samples[84].destroy = destroy_hull_reduction;
    g_samples[84].hideSolverControls = true;
    g_samples[85].controls = hull_transform_controls;
    g_samples[85].destroy = destroy_hull_transform;
    g_samples[85].hideSolverControls = true;

    sort_samples();
}

// upstream main.cpp SortSamples/CompareSamples: category then name, not
// registration order. The menu builder walks runs of equal category.
void sort_samples() {
    // insertion sort, once at startup
    for i32 i = 1; i < NUM_SAMPLES; i++ {
        SampleDef key = g_samples[i];
        i32 j = i - 1;
        while j >= 0 {
            i32 c = str_compare(g_samples[j].category, key.category);
            if c == 0 { c = str_compare(g_samples[j].name, key.name); }
            if c <= 0 { break; }
            g_samples[j + 1] = g_samples[j];
            j--;
        }
        g_samples[j + 1] = key;
    }
}

// Sorted index of a sample, or 0 if the name is not registered.
// Sorting runs after registration, so a literal index would shift every
// time a sample is added.
i32 find_sample(str category, str name) {
    for i32 i = 0; i < NUM_SAMPLES; i++ {
        if str_compare(g_samples[i].category, category) == 0
            && str_compare(g_samples[i].name, name) == 0 {
            return i;
        }
    }
    return 0;
}

void load_sample(i32 index, bool restart) {
    if g_sample_loaded && g_samples[g_sample].destroy != null {
        g_samples[g_sample].destroy();
    }
    g_sample = index;
    // upstream reseeds in the Sample constructor
    g_randomSeed = RAND_SEED;
    g_mouse_joint_valid = false;
    g_mouse_body_valid = false;
    g_selected_valid = false;
    g_mouse_fraction = 0.0f;
    g_step_count = 0;
    g_profile_read = 0;
    g_profile_write = 0;
    g_profile_current = 0;
    g_shot_count = 0;
    g_shot_next = 0;
    g_bullet_alive = false;
    g_sample_time = 0.0f;
    g_mouse_force_scale = 100.0f;
    g_launch_speed_scale = DEFAULT_LAUNCH_SPEED_SCALE;
    // Each sample gets a fresh world, so shape ids restart: a stale
    // ground id from the previous sample would match whatever is
    // created first here and shade it as ground. Only a sample that
    // asks for it gets the grid.
    g_ground_shape_valid = false;
    if g_single_threaded { g_workers = 1; }
    // release mesh references before the shapes are destroyed
    adapter_reset();
    b3WorldDef wd = b3DefaultWorldDef();
    wd.workerCount = g_workers;
    wd.enableSleep = g_enable_sleep;
    adapter_attach_world_def(&wd);
    if g_samples[index].capacity != null {
        b3Capacity cap = b3Capacity{0, 0, 0, 0, 0};
        g_samples[index].capacity(&cap);
        wd.capacity = cap;
    }
    g_world = b3CreateWorld(&wd);
    b3World_SetContactRecycleDistance(g_world, g_recycle_distance);
    g_samples[index].build();
    g_sample_loaded = true;
    // upstream sample.cpp: fit the shadow range to the world bounds.
    b3AABB shadowBounds = b3World_GetBounds(g_world);
    f32 diagonal = b3Distance(shadowBounds.lowerBound, shadowBounds.upperBound);
    g_shadow_split_far = b3ClampFloat(diagonal, SHADOW_SPLIT_FAR, SHADOW_SPLIT_FAR_MAX);
    // upstream: SetView only when restart == false (R keeps the camera)
    if !restart {
        SampleDef* sd = &g_samples[index];
        cam_set_view(sd.yawDeg, sd.pitchDeg, sd.radius, sd.pivot);
    }
}

// --- mouse picking ---------------------------------------------------
//
// Upstream Sample::MouseDown/MouseUp/MouseMove: plain left-click
// selects, Ctrl+left grabs a dynamic body with a kinematic mouse body
// + motor joint (linearHertz 7.5, damping 1, maxSpringForce 100*m*g,
// velocity torque from the inertia-trace lever), Shift+left launches
// the bullet ball along the click ray. The pick ray comes from the
// camera basis + fov; upstream unprojects its reverse-Z pair, same ray
// (doc/PLAN_box3d_demo_gui.md decision 3).

struct PickRay {
    float3 origin;
    float3 translation;
}

PickRay build_pick_ray(f32 px, f32 py) {
    f32 w = sapp_widthf();
    f32 h = sapp_heightf();
    PickRay ray;
    ray.origin = float3{0.0f, 0.0f, 0.0f};
    ray.translation = float3{0.0f, 0.0f, 0.0f};
    if w <= 0.0f || h <= 0.0f { return ray; }
    f32 ndcX = (2.0f * px / w) - 1.0f;
    f32 ndcY = 1.0f - (2.0f * py / h);
    f32 tanHalf = tanf(0.5f * 60.0f * PI_F / 180.0f);
    f32 aspect = w / h;
    // look = -forward; lift by right/up scaled to the frustum
    f32 dx = -cam_forward.x + ndcX * tanHalf * aspect * cam_right.x + ndcY * tanHalf * cam_up.x;
    f32 dy = -cam_forward.y + ndcX * tanHalf * aspect * cam_right.y + ndcY * tanHalf * cam_up.y;
    f32 dz = -cam_forward.z + ndcX * tanHalf * aspect * cam_right.z + ndcY * tanHalf * cam_up.z;
    f32 dl = sqrtf(dx * dx + dy * dy + dz * dz);
    dx /= dl; dy /= dl; dz /= dl;
    // upstream: origin on the near plane, translation spans near -> far
    f32 near = 0.1f;
    ray.origin = float3{cam_eye.x + dx * near, cam_eye.y + dy * near, cam_eye.z + dz * near};
    f32 span = CAM_VIEW_DISTANCE - near;
    ray.translation = float3{dx * span, dy * span, dz * span};
    return ray;
}

// grab/selection state (upstream Sample fields)
bool g_mouse_joint_valid;
b3JointId g_mouse_joint;
bool g_mouse_body_valid;
b3BodyId g_mouse_body;
float3 g_mouse_point;
f32 g_mouse_fraction;
bool g_selected_valid;
b3BodyId g_selected_body;
f32 g_mouse_force_scale;                // upstream m_mouseForceScale

void mouse_release() {
    if g_mouse_joint_valid {
        b3DestroyJoint(g_mouse_joint, true);
        g_mouse_joint_valid = false;
    }
    if g_mouse_body_valid {
        b3DestroyBody(g_mouse_body);
        g_mouse_body_valid = false;
    }
    g_mouse_fraction = 0.0f;
}

// Sample::MouseDown, Ctrl arm: grab a dynamic body under the cursor.
void mouse_grab(f32 px, f32 py) {
    PickRay ray = build_pick_ray(px, py);
    b3QueryFilter filter = b3DefaultQueryFilter();
    b3RayResult result = b3World_CastRayClosest(g_world,
        b3Pos{ray.origin.x, ray.origin.y, ray.origin.z},
        b3Vec3{ray.translation.x, ray.translation.y, ray.translation.z}, filter);
    if !result.hit { return; }
    b3BodyId bodyId = b3Shape_GetBody(result.shapeId);
    if b3Body_GetType(bodyId) != b3_dynamicBody { return; }

    g_mouse_point = float3{result.point.x, result.point.y, result.point.z};

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_kinematicBody;
    bodyDef.position = result.point;
    bodyDef.enableSleep = false;
    g_mouse_body = b3CreateBody(g_world, &bodyDef);
    g_mouse_body_valid = true;

    g_selected_body = bodyId;
    g_selected_valid = true;

    b3MotorJointDef jointDef = b3DefaultMotorJointDef();
    jointDef.base.bodyIdA = g_mouse_body;
    jointDef.base.bodyIdB = bodyId;
    jointDef.base.localFrameB.p = b3Body_GetLocalPoint(bodyId, result.point);
    jointDef.linearHertz = 7.5f;
    jointDef.linearDampingRatio = 1.0f;

    b3MassData massData = b3Body_GetMassData(bodyId);
    b3Vec3 gv = b3World_GetGravity(g_world);
    f32 g = sqrtf(gv.x * gv.x + gv.y * gv.y + gv.z * gv.z);
    f32 mg = massData.mass * g;
    jointDef.maxSpringForce = g_mouse_force_scale * mg;
    if massData.mass > 0.0f {
        // acts like angular friction
        f32 trace = massData.inertia.cx.x + massData.inertia.cy.y + massData.inertia.cz.z;
        f32 lever = sqrtf(trace / (3.0f * massData.mass));
        jointDef.maxVelocityTorque = 0.5f * lever * mg;
    }

    g_mouse_joint = b3CreateMotorJoint(g_world, &jointDef);
    g_mouse_joint_valid = true;
    b3Body_SetAwake(bodyId, true);
    g_mouse_fraction = result.fraction;
}

// Sample::MouseDown, plain arm: selection.
void mouse_select(f32 px, f32 py) {
    PickRay ray = build_pick_ray(px, py);
    b3QueryFilter filter = b3DefaultQueryFilter();
    b3RayResult result = b3World_CastRayClosest(g_world,
        b3Pos{ray.origin.x, ray.origin.y, ray.origin.z},
        b3Vec3{ray.translation.x, ray.translation.y, ray.translation.z}, filter);
    if result.hit {
        g_selected_body = b3Shape_GetBody(result.shapeId);
        g_selected_valid = true;
    } else {
        g_selected_valid = false;
    }
}

// Sample::MouseMove: slide the grab target along the pick ray.
// upstream SampleContext::mouseX/mouseY: the cursor, tracked every
// frame whether or not a body is being dragged, so a sample can query
// what is under it.
f32 g_mouse_screen_x;
f32 g_mouse_screen_y;

void mouse_move(f32 px, f32 py) {
    g_mouse_screen_x = px;
    g_mouse_screen_y = py;
    if !g_mouse_joint_valid { return; }
    PickRay ray = build_pick_ray(px, py);
    g_mouse_point = float3{
        ray.origin.x + g_mouse_fraction * ray.translation.x,
        ray.origin.y + g_mouse_fraction * ray.translation.y,
        ray.origin.z + g_mouse_fraction * ray.translation.z};
}

// upstream Sample::MouseDown: plain left selects, Ctrl+left grabs,
// Shift+left launches.
void sample_mouse_down(f32 px, f32 py, i32 button, i32 modifiers) {
    // Nothing picks while the pointer is locked to a third-person camera.
    if cam_third_person { return; }
    if button == SAPP_MOUSEBUTTON_LEFT && modifiers == 0 { mouse_select(px, py); }
    else if button == SAPP_MOUSEBUTTON_LEFT && modifiers == SAPP_MODIFIER_CTRL {
        mouse_grab(px, py);
    }
    // Upstream tests the shift bit and does not require a button.
    else if (modifiers & SAPP_MODIFIER_SHIFT) != 0 { launch_at(px, py, modifiers); }
}

// SPACE: the sample framework's projectile launch (sample.cpp
// MouseDown, plain shift-click arm): sphere r=0.25, density x4,
// isBullet, 100 m/s along the aim, spawned 2 units out.
void shoot_dir(f32 dx, f32 dy, f32 dz) {

    if g_shot_count == MAX_SHOTS {
        b3DestroyBody(g_shots[g_shot_next]);
    }
    b3BodyDef bd = b3DefaultBodyDef();
    bd.type = b3_dynamicBody;
    bd.position = b3Pos{g_eye.x + dx * 2.0f, g_eye.y + dy * 2.0f, g_eye.z + dz * 2.0f};
    f32 speed = 20.0f * g_launch_speed_scale;
    bd.linearVelocity = b3Vec3{dx * speed, dy * speed, dz * speed};
    bd.isBullet = true;
    b3BodyId ball = b3CreateBody(g_world, &bd);
    b3Sphere sph;
    sph.center = b3Vec3{0.0f, 0.0f, 0.0f};
    sph.radius = 0.25f;
    b3ShapeDef sd = b3DefaultShapeDef();
    sd.density *= 4.0f;
    ignore b3CreateSphereShape(ball, &sd, &sph);

    g_shots[g_shot_next] = ball;
    g_shot_next = (g_shot_next + 1) % MAX_SHOTS;
    if g_shot_count < MAX_SHOTS { g_shot_count++; }
}

void shoot() {
    shoot_dir(-cam_forward.x, -cam_forward.y, -cam_forward.z);
}

// upstream Sample::MouseDown, shift arm. Three projectiles: Shift+Ctrl a
// cylinder, Shift+Alt a ragdoll, Shift alone the sphere above. The first
// two are untracked, as upstream leaves them — only the sphere goes in
// the recycle pool, which SPACE can otherwise fill without bound.
void launch_at(f32 px, f32 py, i32 modifiers) {
    PickRay ray = build_pick_ray(px, py);
    f32 dl = sqrtf(ray.translation.x * ray.translation.x
        + ray.translation.y * ray.translation.y
        + ray.translation.z * ray.translation.z);
    if dl <= 0.0f { return; }
    b3Vec3 dir = b3Vec3{ray.translation.x / dl, ray.translation.y / dl,
                        ray.translation.z / dl};
    // upstream spawns from the pick ray's near-plane origin, not the eye.
    b3Pos spawn = b3OffsetPos(b3Pos{ray.origin.x, ray.origin.y, ray.origin.z},
                              b3MulSV(2.0f, dir));

    b3ShapeDef shapeDef = b3DefaultShapeDef();

    if (modifiers & SAPP_MODIFIER_CTRL) != 0 {
        b3BodyDef bodyDef = b3DefaultBodyDef();
        bodyDef.type = b3_dynamicBody;
        bodyDef.position = spawn;
        bodyDef.linearVelocity = b3MulSV(10.0f * g_launch_speed_scale, dir);
        bodyDef.isBullet = true;
        b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);

        b3HullData* hull = b3CreateCylinder(2.0f, 0.15f, 0.0f, 6);
        ignore b3CreateHullShape(bodyId, &shapeDef, hull);
        b3DestroyHull(hull);
    } else if (modifiers & SAPP_MODIFIER_ALT) != 0 {
        Human human = Human{};
        CreateHuman(&human, g_world, spawn, 1.0f, 1.0f, 1.0f, 0, null, true);
        Human_SetBullet(&human, true);
        Human_SetVelocity(&human, b3MulSV(10.0f * g_launch_speed_scale, dir));
    } else {
        shoot_dir(dir.x, dir.y, dir.z);
    }
}
