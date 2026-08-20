// box3d_bench.mc — headless timing harness for the hero demo's scene.
//
// Same pyramid and same projectile cadence as apps/box3d_hero.mc, but
// no renderer and no sokol: just b3World_Step in a loop. That isolates
// the solver so native and wasm can be compared without GL in the way.
//
// Build it the same way as the hero (cwd = the box3d-minc checkout so
// `import box3d;` resolves there):
//   scripts/build_box3d_hero.ps1 -Bench

import box3d;
import math;

const i32 PYRAMID_BASE  = 16;
const f32 BOX_HALF      = 0.5f;
const f32 STEP_DT       = 1.0f / 60.0f;
const i32 SUBSTEPS      = 4;
const i32 STEP_COUNT    = 1080;     // 18 s at 60 Hz: one demo cycle
const f32 FIRST_SHOT_AT = 3.0f;
const f32 SHOT_INTERVAL = 1.7f;
const f32 SHOT_SPEED    = 30.0f;
const f32 SHOT_RADIUS   = 0.42f;
const f32 SHOT_DENSITY  = 2.0f;
const i32 MAX_SHOTS     = 12;

b3WorldId g_world;
b3BodyId[MAX_SHOTS] g_shots;
i32 g_shot_count;
i32 g_shot_next;
i32 g_shot_index;

void build_scene() {
    b3WorldDef wd = b3DefaultWorldDef();
    when defined(MINC_THREADS) {
        // Parallel step: workerCount > 1 makes the world create the
        // in-tree scheduler (real threads under wasm --threads and on
        // native with -DMINC_THREADS). BOX3D_W1/W2 pin the count for
        // the determinism fence; W1 takes the serial path.
        when defined(BOX3D_W1) {
            wd.workerCount = 1;
        } else {
            when defined(BOX3D_W2) {
                wd.workerCount = 2;
            } else {
                when defined(BOX3D_W8) {
                    wd.workerCount = 8;
                } else {
                    wd.workerCount = 4;
                }
            }
        }
    }
    g_world = b3CreateWorld(&wd);
    b3World_EnableSleeping(g_world, true);
    g_shot_count = 0;
    g_shot_next = 0;
    g_shot_index = 0;

    b3BodyDef groundDef = b3DefaultBodyDef();
    groundDef.position = b3Pos{0.0f, -1.0f, 0.0f};
    b3BodyId groundId = b3CreateBody(g_world, &groundDef);
    b3BoxHull gbox = b3MakeBoxHull(200.0f, 1.0f, 200.0f);
    b3ShapeDef gsd = b3DefaultShapeDef();
    ignore b3CreateHullShape(groundId, &gsd, &gbox.base);

    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    b3ShapeDef shapeDef = b3DefaultShapeDef();
    shapeDef.density = 100.0f;
    f32 h = BOX_HALF;
    b3BoxHull box = b3MakeBoxHull(h, h, h);
    f32 shift = 1.0f * h;
    for i32 i = 0; i < PYRAMID_BASE; i++ {
        f32 y = (2.0f * cast(f32, i) + 1.0f) * shift;
        for i32 j = i; j < PYRAMID_BASE; j++ {
            f32 x = (cast(f32, i) + 1.0f) * shift
                + 2.0f * cast(f32, j - i) * shift - h * cast(f32, PYRAMID_BASE);
            bodyDef.position = b3Pos{x, y, 0.0f};
            b3BodyId bodyId = b3CreateBody(g_world, &bodyDef);
            ignore b3CreateHullShape(bodyId, &shapeDef, &box.base);
        }
    }
}

// Camera-equivalent launch point: the demo fires from a sweeping orbit,
// so reproduce the same geometry without needing the renderer.
void shoot(f32 t) {
    if g_shot_count == MAX_SHOTS { b3DestroyBody(g_shots[g_shot_next]); }
    f32 aimY = 13.0f - 1.5f * cast(f32, g_shot_index);
    if aimY < 1.5f { aimY = 1.5f; }
    g_shot_index++;

    f32 yaw = 0.60f * sinf(t * (2.0f * 3.14159265f / 27.0f));
    f32 pitch = 0.13f;
    f32 cp = cosf(pitch);
    f32 ex = cp * sinf(yaw) * 29.0f;
    f32 ey = 5.0f + sinf(pitch) * 29.0f;
    f32 ez = cp * cosf(yaw) * 29.0f;

    f32 dx = 0.0f - ex;
    f32 dy = aimY - ey;
    f32 dz = 0.0f - ez;
    f32 dl = sqrtf(dx * dx + dy * dy + dz * dz);
    dx /= dl; dy /= dl; dz /= dl;

    b3BodyDef bd = b3DefaultBodyDef();
    bd.type = b3_dynamicBody;
    bd.position = b3Pos{ex + dx * 2.0f, ey + dy * 2.0f, ez + dz * 2.0f};
    bd.linearVelocity = b3Vec3{dx * SHOT_SPEED, dy * SHOT_SPEED, dz * SHOT_SPEED};
    bd.isBullet = true;
    b3BodyId ball = b3CreateBody(g_world, &bd);
    b3Sphere sph;
    sph.center = b3Vec3{0.0f, 0.0f, 0.0f};
    sph.radius = SHOT_RADIUS;
    b3ShapeDef sd = b3DefaultShapeDef();
    sd.density *= SHOT_DENSITY;
    ignore b3CreateSphereShape(ball, &sd, &sph);

    g_shots[g_shot_next] = ball;
    g_shot_next = (g_shot_next + 1) % MAX_SHOTS;
    if g_shot_count < MAX_SHOTS { g_shot_count++; }
}

i32 main() {
    build_scene();

    // Optional step-count override: `box3d_bench <steps>`. The default
    // run is 0.18 s, too short for the sampling profiler to collect
    // anything, so profiling used to need a recompiled copy with a
    // larger STEP_COUNT. Timing runs are unaffected — no argument keeps
    // the 1080-step cadence the benchmark table is calibrated on.
    i32 steps = STEP_COUNT;
    if get_argc() > 1 {
        u8* a = get_arg(1);
        i32 n = 0;
        i32 k = 0;
        while *(a + k) >= 48 && *(a + k) <= 57 {
            n = n * 10 + (cast(i32, *(a + k)) - 48);
            k = k + 1;
        }
        if n > 0 { steps = n; }
    }

    i64 freq = qpf();
    i64 start = qpc();

    f32 t = 0.0f;
    f32 next_shot = FIRST_SHOT_AT;
    for i32 i = 0; i < steps; i++ {
        if t >= next_shot {
            shoot(t);
            next_shot += SHOT_INTERVAL;
        }
        b3World_Step(g_world, STEP_DT, SUBSTEPS);
        t += STEP_DT;
    }

    i64 end = qpc();
    i64 us = (end - start) * 1_000_000 / freq;

    // A cheap checksum so the optimiser cannot elide the simulation and
    // so native/wasm divergence shows up immediately.
    b3AABB bounds = b3World_GetBounds(g_world);
    i64 check = cast(i64, bounds.upperBound.y * 1000.0f);

    print("box3d_bench: steps={} check={} time={} us\n", steps, check, us);
    print("  per-step: {} us\n", us / cast(i64, steps));
    return 0;
}
