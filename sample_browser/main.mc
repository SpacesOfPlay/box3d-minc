// box3d-minc sample browser: a port of Box3D's samples application.
//
// Controls
//   [ / ]    previous / next sample (the menu bar has the full list)
//   P        pause; O single-step (+Shift: 5 steps); R restart
//   F        frame the scene; M metrics drawer; Tab hide the UI
//   SPACE    launch a ball along the view direction
//   Ctrl+Q / Esc  quit
//
//   left-click        select a body
//   Ctrl+left-drag    grab a dynamic body
//   Shift+left-click  launch a ball along the click ray
//   Alt+left-drag     orbit      Alt+middle-drag  pan
//   Alt+right-drag    zoom       scroll           zoom
//   right-drag        fly look; WASD moves, scroll tunes speed
//
// Scene setups, solver parameters, camera and GUI mirror the upstream
// samples app. This file holds the entry points.

import box3d;
import sokol_all;
import sokol_imgui;
import math;
import linear;
import str;
import camera;
import gui;
import renderer;
import sample;
import debug_adapter;
import sample_benchmark;
import sample_bodies;
import sample_continuous;
import sample_robustness;
import sample_stacking;
import sample_joint;
import sample_compound;
import sample_world;
import sample_shapes;
import mesh_loader;
import mover_shim;
import box3d_mover;
import sample_issues;
import sample_events;
import sample_geometry;
import sample_collision;
import sample_mesh;

void init() {
    // A cached mesh takes two buffers, vertices and edges, so a full
    // cache is 2 * MESH_CACHE_MAX. The fixed ones on top are the sphere
    // and capsule impostors, the sky triangle, the overlay line buffer,
    // the edge corners and the instance stream, plus two for sokol_imgui
    // — eight, and the rest of the slack absorbs any added later.
    // sokol's default pool is 128, which Joints/Gear Lift overruns at 74
    // live meshes: sg_make_buffer starts handing back invalid ids and the
    // first bind of one faults.
    sg_setup(&sg_desc{
        .environment = sglue_environment(),
        .logger = sglue_logger(),
        .buffer_pool_size = MESH_CACHE_MAX * 2 + 32,
    });
    simgui_setup(&simgui_desc_t{});
    gui_apply_style();

    build_sphere();
    build_capsule();
    setup_samples();
    // upstream main.cpp: workerCount = clamp(cores / 2, 1, 8)
    i32 cores = cpu_count();
    g_single_threaded = cores <= 1;
    g_max_workers = cores;
    if g_max_workers > 32 { g_max_workers = 32; }
    if g_max_workers < 1 { g_max_workers = 1; }
    g_workers = cores / 2;
    if g_workers < 1 { g_workers = 1; }
    if g_workers > 8 { g_workers = 8; }
    when os(wasm) {
        // Browser thread wakes are pricier than native and the sample
        // scenes cap near four workers; past that the extra spinners
        // cost more than they add.
        if g_workers > 4 { g_workers = 4; }
    }
    // Upstream opens a first run on the replay viewer, which needs mesh
    // shapes and 3D text; without it, its persisted default is sorted
    // index 0. Named rather than numbered so adding a sample cannot
    // silently move the startup scene.
    load_sample(find_sample("Stacking", "Box Stack"), restart: false);
    g_sph_vbuf = sg_make_buffer(&sg_buffer_desc{
        .data.ptr = &g_sph_verts, .data.size = sizeof(g_sph_verts) });
    g_cap_vbuf = sg_make_buffer(&sg_buffer_desc{
        .data.ptr = &g_cap_verts, .data.size = sizeof(g_cap_verts) });
    // fullscreen triangle for the sky (covers NDC with one tri)
    f32[9] skyVerts = { -1.0f, -1.0f, 0.0f,  3.0f, -1.0f, 0.0f,  -1.0f, 3.0f, 0.0f };
    g_sky_vbuf = sg_make_buffer(&sg_buffer_desc{
        .data.ptr = &skyVerts, .data.size = sizeof(skyVerts) });

    sg_shader eshd = sokol_make_shader(&b3_edge_vs_shader, &b3_edge_fs_shader);
    // one InstRec per shape, refilled every frame
    g_inst_vbuf = sg_make_buffer(&sg_buffer_desc{
        .size = cast(i64, INST_MAX * INST_STRIDE),
        .usage.stream_update = true });
    // debug channel: a stream-updated line buffer
    g_dbg_vbuf = sg_make_buffer(&sg_buffer_desc{
        .size = cast(i64, DBG_VERT_MAX * DBG_FLOATS_PER_VERT * 4),
        .usage.stream_update = true });
    sg_shader dbgshd = sokol_make_shader(&b3_dbg_vs_shader, &b3_dbg_fs_shader);
    // Depth ALWAYS, as upstream's overlay pipeline (gfx/overlay.c
    // EnsurePipelines): joint markers sit inside the bodies they join.
    g_pip_dbg = sg_make_pipeline(&sg_pipeline_desc{
        .shader = dbgshd,
        .layout.attrs[0].format = SG_VERTEXFORMAT_FLOAT3,
        .layout.attrs[1].format = SG_VERTEXFORMAT_FLOAT4,
        .index_type = SG_INDEXTYPE_NONE,
        .primitive_type = SG_PRIMITIVETYPE_LINES,
        .depth.compare = SG_COMPAREFUNC_ALWAYS,
        .depth.write_enabled = false,
        .colors[0].blend.enabled = true,
        .colors[0].blend.src_factor_rgb = SG_BLENDFACTOR_SRC_ALPHA,
        .colors[0].blend.dst_factor_rgb = SG_BLENDFACTOR_ONE_MINUS_SRC_ALPHA,
        .colors[0].blend.src_factor_alpha = SG_BLENDFACTOR_ONE,
        .colors[0].blend.dst_factor_alpha = SG_BLENDFACTOR_ONE_MINUS_SRC_ALPHA,
    });

    // hull-edge overlay: 1 px lines, alpha-blended, depth-tested but not
    // writing depth
    // Screen-space expanded quads, six vertices per edge, so triangles
    // rather than lines. Premultiplied blend to pair with the shader's
    // colour * coverage output, as upstream gfx/edges.c.
    // buffer 0 is the six shared quad corners, buffer 1 one endpoint
    // pair per edge, stepped per instance
    g_edge_corner_vbuf = sg_make_buffer(&sg_buffer_desc{
        .data.ptr = &g_edge_corners, .data.size = sizeof(g_edge_corners) });
    g_pip_lines_ns = sg_make_pipeline(&sg_pipeline_desc{
        .shader = eshd,
        .layout.buffers[1].step_func = SG_VERTEXSTEP_PER_INSTANCE,
        .layout.attrs[0].buffer_index = 0,
        .layout.attrs[0].format = SG_VERTEXFORMAT_FLOAT,
        .layout.attrs[1].buffer_index = 1,
        .layout.attrs[1].format = SG_VERTEXFORMAT_FLOAT3,
        .layout.attrs[2].buffer_index = 1,
        .layout.attrs[2].format = SG_VERTEXFORMAT_FLOAT3,
        .index_type = SG_INDEXTYPE_NONE,
        .primitive_type = SG_PRIMITIVETYPE_TRIANGLES,
        .depth.compare = SG_COMPAREFUNC_LESS_EQUAL,
        .depth.write_enabled = false,
        .colors[0].blend.enabled = true,
        .colors[0].blend.src_factor_rgb = SG_BLENDFACTOR_ONE,
        .colors[0].blend.dst_factor_rgb = SG_BLENDFACTOR_ONE_MINUS_SRC_ALPHA,
        .colors[0].blend.src_factor_alpha = SG_BLENDFACTOR_ONE,
        .colors[0].blend.dst_factor_alpha = SG_BLENDFACTOR_ONE_MINUS_SRC_ALPHA,
    });

    // sky: far-plane fullscreen triangle, fills only untouched depth
    sg_shader skyshd = sokol_make_shader(&b3_sky_vs_shader, &b3_sky_fs_shader);
    g_pip_sky = sg_make_pipeline(&sg_pipeline_desc{
        .shader = skyshd,
        .layout.attrs[0].format = SG_VERTEXFORMAT_FLOAT3,
        .depth.compare = SG_COMPAREFUNC_LESS_EQUAL,
        .depth.write_enabled = false,
    });

    // Solids: buffer 0 is the shape mesh, buffer 1 the instance stream.
    sg_shader ishd = sokol_make_shader(&b3_inst_vs_shader, &b3_fs_shader);
    g_pip_inst = sg_make_pipeline(&sg_pipeline_desc{
        .shader = ishd,
        .layout.buffers[1].step_func = SG_VERTEXSTEP_PER_INSTANCE,
        .layout.attrs[0].format = SG_VERTEXFORMAT_FLOAT3,
        .layout.attrs[1].format = SG_VERTEXFORMAT_FLOAT3,
        .layout.attrs[2].buffer_index = 1,
        .layout.attrs[2].format = SG_VERTEXFORMAT_FLOAT4,
        .layout.attrs[3].buffer_index = 1,
        .layout.attrs[3].format = SG_VERTEXFORMAT_FLOAT4,
        .layout.attrs[4].buffer_index = 1,
        .layout.attrs[4].format = SG_VERTEXFORMAT_FLOAT4,
        .layout.attrs[5].buffer_index = 1,
        .layout.attrs[5].format = SG_VERTEXFORMAT_FLOAT4,
        .layout.attrs[6].buffer_index = 1,
        .layout.attrs[6].format = SG_VERTEXFORMAT_FLOAT4,
        .index_type = SG_INDEXTYPE_NONE,
        .face_winding = SG_FACEWINDING_CCW,
        .cull_mode = SG_CULLMODE_BACK,
        .depth.compare = SG_COMPAREFUNC_LESS_EQUAL,
        .depth.write_enabled = true,
    });

    // upstream gfx/geometry_registry.c: a negative-determinant scale
    // reverses winding, so those draws are front-culled instead.
    // Character / Mover mirrors the stairs (-z) and the torus (-x).
    g_pip_inst_mirror = sg_make_pipeline(&sg_pipeline_desc{
        .shader = ishd,
        .layout.buffers[1].step_func = SG_VERTEXSTEP_PER_INSTANCE,
        .layout.attrs[0].format = SG_VERTEXFORMAT_FLOAT3,
        .layout.attrs[1].format = SG_VERTEXFORMAT_FLOAT3,
        .layout.attrs[2].buffer_index = 1,
        .layout.attrs[2].format = SG_VERTEXFORMAT_FLOAT4,
        .layout.attrs[3].buffer_index = 1,
        .layout.attrs[3].format = SG_VERTEXFORMAT_FLOAT4,
        .layout.attrs[4].buffer_index = 1,
        .layout.attrs[4].format = SG_VERTEXFORMAT_FLOAT4,
        .layout.attrs[5].buffer_index = 1,
        .layout.attrs[5].format = SG_VERTEXFORMAT_FLOAT4,
        .layout.attrs[6].buffer_index = 1,
        .layout.attrs[6].format = SG_VERTEXFORMAT_FLOAT4,
        .index_type = SG_INDEXTYPE_NONE,
        .face_winding = SG_FACEWINDING_CCW,
        .cull_mode = SG_CULLMODE_FRONT,
        .depth.compare = SG_COMPAREFUNC_LESS_EQUAL,
        .depth.write_enabled = true,
    });

    // --- shadow map ---
    // One array image, a layer per cascade. The depth pass renders into a
    // per-slice attachment view; the shader samples the whole array.
    g_shadow_img = sg_make_image(&sg_image_desc{
        .type = SG_IMAGETYPE_ARRAY,
        .width = SHADOW_DIM,
        .height = SHADOW_DIM,
        .num_slices = SHADOW_CASCADES,
        .pixel_format = SG_PIXELFORMAT_DEPTH,
        .sample_count = 1,
        .usage.depth_stencil_attachment = true,
    });
    for i32 i = 0; i < SHADOW_CASCADES; i++ {
        g_shadow_att[i] = sg_make_view(&sg_view_desc{
            .depth_stencil_attachment.image = g_shadow_img,
            .depth_stencil_attachment.slice = i });
    }
    g_shadow_tex = sg_make_view(&sg_view_desc{ .texture.image = g_shadow_img });
    // a comparison sampler makes each tap a depth test
    g_shadow_smp = sg_make_sampler(&sg_sampler_desc{
        .min_filter = SG_FILTER_LINEAR,
        .mag_filter = SG_FILTER_LINEAR,
        .wrap_u = SG_WRAP_CLAMP_TO_EDGE,
        .wrap_v = SG_WRAP_CLAMP_TO_EDGE,
        .compare = SG_COMPAREFUNC_LESS_EQUAL,
    });

    // Depth-only pipelines, one per mesh kind. color_count 0 keeps the
    // pass depth-only. Shadow bias is per-pixel, in b3_fs.
    sg_shader dshd = sokol_make_shader(&b3_inst_depth_vs_shader, &b3_depth_fs_shader);
    g_pip_inst_depth = sg_make_pipeline(&sg_pipeline_desc{
        .shader = dshd,
        .layout.buffers[1].step_func = SG_VERTEXSTEP_PER_INSTANCE,
        .layout.attrs[0].format = SG_VERTEXFORMAT_FLOAT3,
        .layout.attrs[1].format = SG_VERTEXFORMAT_FLOAT3,
        .layout.attrs[2].buffer_index = 1,
        .layout.attrs[2].format = SG_VERTEXFORMAT_FLOAT4,
        .layout.attrs[3].buffer_index = 1,
        .layout.attrs[3].format = SG_VERTEXFORMAT_FLOAT4,
        .layout.attrs[4].buffer_index = 1,
        .layout.attrs[4].format = SG_VERTEXFORMAT_FLOAT4,
        .layout.attrs[5].buffer_index = 1,
        .layout.attrs[5].format = SG_VERTEXFORMAT_FLOAT4,
        .layout.attrs[6].buffer_index = 1,
        .layout.attrs[6].format = SG_VERTEXFORMAT_FLOAT4,
        .index_type = SG_INDEXTYPE_NONE,
        .face_winding = SG_FACEWINDING_CCW,
        .cull_mode = SG_CULLMODE_BACK,
        .color_count = 0,
        .depth.pixel_format = SG_PIXELFORMAT_DEPTH,
        .depth.compare = SG_COMPAREFUNC_LESS_EQUAL,
        .depth.write_enabled = true,
    });
    sg_shader dcshd = sokol_make_shader(&b3_inst_cap_depth_vs_shader, &b3_depth_fs_shader);
    g_pip_inst_depth_cap = sg_make_pipeline(&sg_pipeline_desc{
        .shader = dcshd,
        .layout.buffers[1].step_func = SG_VERTEXSTEP_PER_INSTANCE,
        .layout.attrs[0].format = SG_VERTEXFORMAT_FLOAT3,
        .layout.attrs[1].format = SG_VERTEXFORMAT_FLOAT3,
        .layout.attrs[2].buffer_index = 1,
        .layout.attrs[2].format = SG_VERTEXFORMAT_FLOAT4,
        .layout.attrs[3].buffer_index = 1,
        .layout.attrs[3].format = SG_VERTEXFORMAT_FLOAT4,
        .layout.attrs[4].buffer_index = 1,
        .layout.attrs[4].format = SG_VERTEXFORMAT_FLOAT4,
        .layout.attrs[5].buffer_index = 1,
        .layout.attrs[5].format = SG_VERTEXFORMAT_FLOAT4,
        .layout.attrs[6].buffer_index = 1,
        .layout.attrs[6].format = SG_VERTEXFORMAT_FLOAT4,
        .index_type = SG_INDEXTYPE_NONE,
        .face_winding = SG_FACEWINDING_CCW,
        .cull_mode = SG_CULLMODE_BACK,
        .color_count = 0,
        .depth.pixel_format = SG_PIXELFORMAT_DEPTH,
        .depth.compare = SG_COMPAREFUNC_LESS_EQUAL,
        .depth.write_enabled = true,
    });

    // capsules: the same state, but a vertex shader that sizes the unit
    // mesh from the half-length/radius attribute
    sg_shader cshd = sokol_make_shader(&b3_inst_cap_vs_shader, &b3_fs_shader);
    g_pip_inst_cap = sg_make_pipeline(&sg_pipeline_desc{
        .shader = cshd,
        .layout.buffers[1].step_func = SG_VERTEXSTEP_PER_INSTANCE,
        .layout.attrs[0].format = SG_VERTEXFORMAT_FLOAT3,
        .layout.attrs[1].format = SG_VERTEXFORMAT_FLOAT3,
        .layout.attrs[2].buffer_index = 1,
        .layout.attrs[2].format = SG_VERTEXFORMAT_FLOAT4,
        .layout.attrs[3].buffer_index = 1,
        .layout.attrs[3].format = SG_VERTEXFORMAT_FLOAT4,
        .layout.attrs[4].buffer_index = 1,
        .layout.attrs[4].format = SG_VERTEXFORMAT_FLOAT4,
        .layout.attrs[5].buffer_index = 1,
        .layout.attrs[5].format = SG_VERTEXFORMAT_FLOAT4,
        .layout.attrs[6].buffer_index = 1,
        .layout.attrs[6].format = SG_VERTEXFORMAT_FLOAT4,
        .index_type = SG_INDEXTYPE_NONE,
        .face_winding = SG_FACEWINDING_CCW,
        .cull_mode = SG_CULLMODE_BACK,
        .depth.compare = SG_COMPAREFUNC_LESS_EQUAL,
        .depth.write_enabled = true,
    });
}

void frame() {
    g_frame++;
    if g_frame_ticks != 0 {
        f32 ms = b3GetMilliseconds(g_frame_ticks);
        // light EMA
        if g_frame_ms == 0.0f { g_frame_ms = ms; }
        else { g_frame_ms = 0.9f * g_frame_ms + 0.1f * ms; }
    }
    g_frame_ticks = b3GetTicks();
    f32 dt = g_frame_ms / 1000.0f;
    if dt > 0.05f { dt = 0.05f; }

    if g_reset_pending || g_switch_pending != 0 {
        // upstream clamps at the catalogue ends (no wrap)
        i32 next = g_sample + g_switch_pending;
        if next < 0 { next = 0; }
        if next > NUM_SAMPLES - 1 { next = NUM_SAMPLES - 1; }
        bool restart = g_reset_pending && next == g_sample;
        g_reset_pending = false;
        g_switch_pending = 0;
        load_sample(next, restart);
    }

    simgui_new_frame(&simgui_frame_desc_t{
        .width = sapp_width(),
        .height = sapp_height(),
        .delta_time = sapp_frame_duration(),
        .dpi_scale = sapp_dpi_scale(),
    });
    if g_show_ui {
        draw_menu_bar();
        draw_info_panel();
        draw_metrics();
    } else {
        draw_hud();
    }
    // upstream ResetText: under the menu bar, or high when UI hidden
    if g_show_ui {
        g_text_line = ImGui_GetFrameHeight() + 0.5f * ImGui_GetFontSize();
    } else {
        g_text_line = 3.0f * ImGui_GetFontSize();
    }

    // upstream step semantics: timeStep = 1/hertz; pause holds the
    // world unless single-step credits are pending
    f32 timeStep = g_hertz > 0.0f ? 1.0f / g_hertz : 0.0f;
    if g_pause {
        if g_single_step > 0 { g_single_step--; }
        else { timeStep = 0.0f; }
    }
    // Upstream clears the overlay at frame start, so b3World_Draw and
    // whatever a sample draws itself both land in it. Clearing later
    // would discard everything the sample's step emitted.
    dbg_reset();
    // upstream main.cpp: camera input is folded in before the sample
    // advances, so a sample reading the basis this frame gets this
    // frame's — the character controllers build their throttle from it.
    cam_update(dt);
    // upstream main.cpp: sync the draw origin to the camera eye once per
    // frame, before any drawing. Must hold even for a sample that drives
    // its own step, so it sits here rather than in the adapter.
    set_draw_origin(b3Pos{cam_eye.x, cam_eye.y, cam_eye.z});
    // The half of an upstream Step() override that runs ahead of
    // Sample::Step(): velocity control, kinematic drives, spawners.
    if g_samples[g_sample].preStep != null {
        g_samples[g_sample].preStep(timeStep);
    }
    // upstream Sample::Step: recover if the grabbed body/world vanished
    if g_mouse_joint_valid && !b3Joint_IsValid(g_mouse_joint) {
        g_mouse_joint_valid = false;
        if g_mouse_body_valid {
            b3DestroyBody(g_mouse_body);
            g_mouse_body_valid = false;
        }
    }
    if g_mouse_body_valid && timeStep > 0.0f {
        b3WorldTransform mt;
        mt.p = b3Pos{g_mouse_point.x, g_mouse_point.y, g_mouse_point.z};
        mt.q = b3Quat{b3Vec3{0.0f, 0.0f, 0.0f}, 1.0f};
        b3Body_SetTargetTransform(g_mouse_body, mt, timeStep, true);
    }
    // upstream applies the enable flags every frame before stepping
    b3World_EnableSleeping(g_world, g_enable_sleep);
    b3World_EnableWarmStarting(g_world, g_enable_warm);
    b3World_EnableContinuous(g_world, g_enable_continuous);

    if timeStep > 0.0f || g_step_while_paused {
        b3World_Step(g_world, timeStep, g_substeps);
    }

    if timeStep > 0.0f {
        g_sample_time += timeStep;
        g_step_count++;
        // profile ring (upstream Sample::Step)
        if g_profile_write - g_profile_read == PROFILE_CAP { g_profile_read++; }
        g_profile_current = g_profile_write & (PROFILE_CAP - 1);
        g_profiles[g_profile_current] = b3World_GetProfile(g_world);
        g_profile_write++;
    }

    // upstream Sample::Step: a cursor pick that feeds the surface-type
    // readout only. Nothing is highlighted from it.
    g_triangle_index = -1;
    g_user_material_id = cast(u64, 0);
    // Not while the pointer is locked to a third-person camera: there is
    // no cursor to read a surface under.
    if !cam_third_person {
        PickRay pickRay = build_pick_ray(g_mouse_screen_x, g_mouse_screen_y);
        b3QueryFilter filter = b3DefaultQueryFilter();
        filter.name = "pick";
        b3RayResult result = b3World_CastRayClosest(g_world,
            b3Pos{pickRay.origin.x, pickRay.origin.y, pickRay.origin.z},
            b3Vec3{pickRay.translation.x, pickRay.translation.y, pickRay.translation.z},
            filter);
        if result.hit {
            if b3Shape_GetType(result.shapeId) == b3_meshShape {
                g_triangle_index = result.triangleIndex;
            }
            g_user_material_id = result.userMaterialId;
        }
    }

    // The rest of an upstream Step() override, plus Render(): both run
    // after Sample::Step(), so a sample draws the state the world just
    // advanced to rather than the one before it.
    if g_samples[g_sample].step != null {
        g_samples[g_sample].step(timeStep);
    }
    g_eye = cam_eye;
    float4x4 proj = perspective(60.0f * PI_F / 180.0f, sapp_widthf() / sapp_heightf(), 0.1f, CAM_VIEW_DISTANCE);
    float4x4 view = cam_view_matrix();
    float4x4 viewproj = mul(proj, view);

    // --- shadow pass: depth from the light, casters only ---------------
    // One pass per cascade, each into its own slice.
    update_light_matrix();
    // one walk of the world per frame, then one instance upload; every
    // pass below draws out of it
    adapter_collect();
    adapter_build_instances(&viewproj);
    for i32 c = 0; c < SHADOW_CASCADES; c++ {
        sg_begin_pass(&sg_pass{
            .action.depth.load_action = SG_LOADACTION_CLEAR,
            .action.depth.clear_value = 1.0f,
            // STORE: the lit pass samples this map. sokol defaults an
            // unset depth store action to DONTCARE.
            .action.depth.store_action = SG_STOREACTION_STORE,
            .attachments.depth_stencil = g_shadow_att[c],
        });
        adapter_draw_depth(c);
        sg_end_pass();
    }

    // per-pass shadow data, uploaded once per lit pipeline below
    ShadowUni shu;
    for i32 c = 0; c < SHADOW_CASCADES; c++ { shu.cascade[c] = g_light_mat[c]; }
    shu.cascadeFar = float4{g_cascade_far[0], g_cascade_far[1], g_cascade_far[2], 0.0f};
    // The eye is the draw origin, so it is the origin of the frame the
    // shader works in.
    shu.camPos = float4{0.0f, 0.0f, 0.0f, 0.0f};
    // cam_forward points pivot -> eye, so the view direction negates it
    shu.viewDir = float4{0.0f - cam_forward.x, 0.0f - cam_forward.y, 0.0f - cam_forward.z, 0.0f};
    // x is the PCF tap offset, with the spacing folded in
    shu.shParams = float4{SHADOW_PCF_SPACING / cast(f32, SHADOW_DIM), 0.35f, 0.0f, 0.0f};
    // upstream main.cpp: wrap the origin to the grid period in double,
    // before it narrows to float. A float cannot resolve a 1 m cell at
    // 1e7 m, so feeding the raw origin to the grid would shatter the
    // lines. The pattern repeats every 10 cells, so the wrapped offset
    // draws identical lines at any distance. zw carries the full origin
    // for the axes, which have to sit at true x = 0 / z = 0.
    f64 gridPeriod = 10.0 * cast(f64, GROUND_GRID_CELL);
    shu.gridOffset = float4{
        cast(f32, fmod(cast(f64, cam_eye.x), gridPeriod)),
        cast(f32, fmod(cast(f64, cam_eye.z), gridPeriod)),
        cam_eye.x, cam_eye.z};

    sg_begin_pass(&sg_pass{
        .swapchain = sglue_swapchain(),
        .action.colors[0].load_action = SG_LOADACTION_CLEAR,
        .action.colors[0].clear_value = sg_color{0.09f, 0.10f, 0.13f, 1.0f},
    });
    adapter_draw_lit(&viewproj, &shu);

    // Sky before the overlays. It is depth-tested but writes no depth,
    // and neither overlay writes depth either, so it still shades only
    // the pixels the lit pass left empty — but it no longer paints over
    // lines drawn into that empty space. A sample with no bodies is
    // otherwise all sky.
    sg_apply_pipeline(g_pip_sky);
    sg_apply_bindings(&sg_bindings{ .vertex_buffers[0] = g_sky_vbuf });
    f32 skyTanHalf = tanf(0.5f * 60.0f * PI_F / 180.0f);
    f32 skyAspect = sapp_widthf() / sapp_heightf();
    SkyPass sp;
    sp.fwd = float4{cam_forward.x, cam_forward.y, cam_forward.z, 0.0f};
    sp.right = float4{cam_right.x, cam_right.y, cam_right.z, skyTanHalf * skyAspect};
    sp.up = float4{cam_up.x, cam_up.y, cam_up.z, skyTanHalf};
    sp.sun = float4{0.4880f, 0.7807f, 0.3904f, 0.0f};   // normalize(0.5, 0.8, 0.4)
    sg_apply_uniforms(0, &sg_range{ .ptr = &sp, .size = sizeof(sp) });
    sg_draw(0, 3, 1);

    adapter_draw_outlines(&viewproj);

    // debug channel
    adapter_draw_debug_lines(&viewproj);

    // upstream drains the label arena in the GUI shell, after the scene:
    // world positions project with the camera that just rendered.
    label_flush(&viewproj);

    simgui_render();
    sg_end_pass();
    sg_commit();
}

void on_event(sapp_event* ev) {
    ignore simgui_handle_event(ev);
    if ev.type == SAPP_EVENTTYPE_QUIT_REQUESTED { sapp_request_quit(); }

    // camera input, gated on imgui capture (upstream main.cpp routing)
    ImGuiIO* cio = ImGui_GetIO();
    bool mouse_ev = ev.type == SAPP_EVENTTYPE_MOUSE_DOWN
        || ev.type == SAPP_EVENTTYPE_MOUSE_UP
        || ev.type == SAPP_EVENTTYPE_MOUSE_MOVE
        || ev.type == SAPP_EVENTTYPE_MOUSE_SCROLL;
    bool key_ev = ev.type == SAPP_EVENTTYPE_KEY_DOWN || ev.type == SAPP_EVENTTYPE_KEY_UP;
    if mouse_ev && !cio.WantCaptureMouse { cam_on_event(ev); }
    if key_ev && !cio.WantCaptureKeyboard { cam_on_event(ev); }
    if ev.type == SAPP_EVENTTYPE_UNFOCUSED { cam_on_event(ev); }

    // sample mouse interaction (upstream Sample::MouseDown/Up/Move)
    if mouse_ev && !cio.WantCaptureMouse {
        u32 smods = ev.modifiers & (SAPP_MODIFIER_SHIFT | SAPP_MODIFIER_CTRL | SAPP_MODIFIER_ALT);
        if ev.type == SAPP_EVENTTYPE_MOUSE_DOWN {
            i32 button = cast(i32, ev.mouse_button);
            i32 mods = cast(i32, smods);
            bool handled = false;
            if g_samples[g_sample].mouseDown != null {
                handled = g_samples[g_sample].mouseDown(ev.mouse_x, ev.mouse_y, button, mods);
            }
            if !handled { sample_mouse_down(ev.mouse_x, ev.mouse_y, button, mods); }
        }
        if ev.type == SAPP_EVENTTYPE_MOUSE_UP {
            if g_samples[g_sample].mouseUp != null {
                g_samples[g_sample].mouseUp(ev.mouse_x, ev.mouse_y, cast(i32, ev.mouse_button));
            }
            if ev.mouse_button == SAPP_MOUSEBUTTON_LEFT { mouse_release(); }
        }
        if ev.type == SAPP_EVENTTYPE_MOUSE_MOVE {
            if g_samples[g_sample].mouseMove != null {
                g_samples[g_sample].mouseMove(ev.mouse_x, ev.mouse_y);
            }
            mouse_move(ev.mouse_x, ev.mouse_y);
        }
    }

    // upstream SetKeyDown / Sample::Keyboard: the held-key table and the
    // per-sample press hook, both before the browser's own shortcuts.
    if ev.type == SAPP_EVENTTYPE_KEY_DOWN || ev.type == SAPP_EVENTTYPE_KEY_UP {
        ImGuiIO* kio = ImGui_GetIO();
        if !kio.WantCaptureKeyboard {
            bool down = ev.type == SAPP_EVENTTYPE_KEY_DOWN;
            set_key_down(cast(i32, ev.key_code), down);
            if g_samples[g_sample].keyboard != null {
                i32 action = down ? 1 : 0;
                g_samples[g_sample].keyboard(cast(i32, ev.key_code), action,
                                             cast(i32, ev.modifiers));
            }
        }
    }

    if ev.type == SAPP_EVENTTYPE_KEY_DOWN {
        ImGuiIO* io = ImGui_GetIO();
        if io.WantCaptureKeyboard { return; }
        bool ctrl = (ev.modifiers & SAPP_MODIFIER_CTRL) != 0;
        bool shift = (ev.modifiers & SAPP_MODIFIER_SHIFT) != 0;
        if ev.key_code == SAPP_KEYCODE_ESCAPE { sapp_request_quit(); }
        if ev.key_code == SAPP_KEYCODE_Q && ctrl { sapp_request_quit(); }
        // Third person gives space to the character's jump.
        if ev.key_code == SAPP_KEYCODE_SPACE && !cam_third_person { shoot(); }
        if ev.key_code == SAPP_KEYCODE_R { g_reset_pending = true; }
        if ev.key_code == SAPP_KEYCODE_P { g_pause = !g_pause; }
        if ev.key_code == SAPP_KEYCODE_O && !ctrl {
            g_single_step += shift ? 5 : 1;
        }
        if ev.key_code == SAPP_KEYCODE_RIGHT_BRACKET { g_switch_pending = 1; }
        if ev.key_code == SAPP_KEYCODE_LEFT_BRACKET { g_switch_pending = -1; }
        if ev.key_code == SAPP_KEYCODE_F { cam_frame(); }
        if ev.key_code == SAPP_KEYCODE_TAB { g_show_ui = !g_show_ui; }
        if ev.key_code == SAPP_KEYCODE_M { g_show_metrics = !g_show_metrics; }
    }
    return;
}

void cleanup() {
    b3DestroyWorld(g_world);
    simgui_shutdown();
    sg_shutdown();
}

sapp_desc sokol_main() {
    return sapp_desc{
        .init_cb = init,
        .frame_cb = frame,
        .cleanup_cb = cleanup,
        .event_cb = on_event,
        .width = 1280,
        .height = 720,
        .high_dpi = false,
        .icon.sokol_default = true,
        .window_title = "box3d samples (minc)",
    };
}
