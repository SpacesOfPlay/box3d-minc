// Dear ImGui overlay: the upstream theme, menu bar, info panel,
// on-screen text and the metrics drawer.

// text overlay (upstream Sample::DrawTextLine): white labels on the
// background draw list, advancing down from under the frame bar

import box3d;
import sokol_all;
import sokol_imgui;
import math;
import linear;
import str;
import camera;
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

f32 g_text_line;
const f32 TEXT_INCREMENT = 22.0f;   // upstream m_textIncrement

void draw_text_line(u8* text) {
    ImDrawList* dl = ImGui_GetBackgroundDrawList(ImGui_GetMainViewport());
    ImDrawList_AddText(dl, null, 0.0f, ImVec2{5.0f, g_text_line}, 0xFFFFFFFF, text, null, 0.0f, null);
    g_text_line += TEXT_INCREMENT;
}

bool g_show_ui = true;   // upstream showUI (Tab hides the panels)

// Measured wall-clock frame time. sapp_frame_duration() is sokol's
// spike-filtered average and pins under bimodal frame timing, so the
// readout and the camera use a measured delta. Upstream reads
// sapp_frame_duration().
u64 g_frame_ticks;
f32 g_frame_ms;

// metrics drawer state (upstream Sample::DrawMetrics, M key)
bool g_show_metrics;
i32 g_step_count;
const i32 PROFILE_CAP = 512;        // upstream m_profileCapacity
b3Profile[PROFILE_CAP] g_profiles;
i32 g_profile_read;
i32 g_profile_write;
i32 g_profile_current;
bool[22] g_profile_row_open;    // upstream s_rowOpen
bool g_profile_show_plots;      // upstream s_showPlots
// per-row histories, 22 rows x PROFILE_CAP. File scope: the wasm
// target caps a function frame at 32 KB. Slots beyond the ring count
// are not read.
f32[11264] g_prof_hist;

// upstream DrawMetrics row colors (IM_COL32, little-endian ABGR)
const u32 PROF_COLOR_STEP = 0xFFFF9966;      // 102,153,255
const u32 PROF_COLOR_PAIRS = 0xFFDCDCDC;     // 220,220,220
const u32 PROF_COLOR_COLLIDE = 0xFF338CFF;   // 255,140,51
const u32 PROF_COLOR_SOLVE = 0xFF66CC66;     // 102,204,102
const u32 PROF_COLOR_SENSORS = 0xFFDC78C8;   // 200,120,220
const u32 PROF_COLOR_OTHER = 0xFF5A5A5A;     // 90,90,90
const u32 PROF_COLOR_DEFAULT = 0xFFDCDCDC;

// --- imgui overlay ---------------------------------------------------
//
// Upstream's Dear ImGui theme, ported verbatim from
// samples/host/gui.cpp guiApplyStyle: neutral charcoal surfaces, one
// steel-blue accent at three brightnesses, a 4px/3px rounding system.

void gui_apply_style() {
    ImGuiStyle* style = ImGui_GetStyle();
    style.WindowPadding = ImVec2{10.0f, 10.0f};
    style.FramePadding = ImVec2{8.0f, 4.0f};
    style.CellPadding = ImVec2{6.0f, 4.0f};
    style.ItemSpacing = ImVec2{8.0f, 7.0f};
    style.ItemInnerSpacing = ImVec2{7.0f, 4.0f};
    style.IndentSpacing = 18.0f;
    style.ScrollbarSize = 12.0f;
    style.GrabMinSize = 10.0f;

    style.WindowBorderSize = 1.0f;
    style.FrameBorderSize = 0.0f;
    style.PopupBorderSize = 1.0f;
    style.TabBorderSize = 0.0f;
    style.SeparatorTextBorderSize = 1.0f;

    style.WindowRounding = 4.0f;
    style.ChildRounding = 4.0f;
    style.PopupRounding = 4.0f;
    style.FrameRounding = 3.0f;
    style.GrabRounding = 3.0f;
    style.ScrollbarRounding = 3.0f;
    style.TabRounding = 3.0f;

    style.WindowTitleAlign = ImVec2{0.0f, 0.5f};
    style.FontSizeBase = 13.0f;

    float4 accent4 = float4{0.28f, 0.48f, 0.66f, 1.0f};
    ImVec4 accent = ImVec4{0.28f, 0.48f, 0.66f, 1.00f};
    ImVec4 accentHi = ImVec4{0.38f, 0.60f, 0.80f, 1.00f};
    ImVec4 accentLo = ImVec4{0.22f, 0.36f, 0.50f, 1.00f};
    ignore accent4;

    style.Colors[ImGuiCol_Text] = ImVec4{0.90f, 0.91f, 0.93f, 1.00f};
    style.Colors[ImGuiCol_TextDisabled] = ImVec4{0.49f, 0.51f, 0.55f, 1.00f};
    style.Colors[ImGuiCol_WindowBg] = ImVec4{0.110f, 0.115f, 0.125f, 0.97f};
    style.Colors[ImGuiCol_ChildBg] = ImVec4{0.00f, 0.00f, 0.00f, 0.00f};
    style.Colors[ImGuiCol_PopupBg] = ImVec4{0.100f, 0.105f, 0.115f, 0.98f};
    style.Colors[ImGuiCol_Border] = ImVec4{0.00f, 0.00f, 0.00f, 0.45f};
    style.Colors[ImGuiCol_BorderShadow] = ImVec4{0.00f, 0.00f, 0.00f, 0.00f};
    style.Colors[ImGuiCol_FrameBg] = ImVec4{0.18f, 0.19f, 0.21f, 1.00f};
    style.Colors[ImGuiCol_FrameBgHovered] = ImVec4{0.24f, 0.26f, 0.29f, 1.00f};
    style.Colors[ImGuiCol_FrameBgActive] = ImVec4{0.29f, 0.32f, 0.36f, 1.00f};
    style.Colors[ImGuiCol_TitleBg] = ImVec4{0.090f, 0.095f, 0.105f, 1.00f};
    style.Colors[ImGuiCol_TitleBgActive] = ImVec4{0.14f, 0.16f, 0.19f, 1.00f};
    style.Colors[ImGuiCol_TitleBgCollapsed] = ImVec4{0.090f, 0.095f, 0.105f, 0.75f};
    style.Colors[ImGuiCol_MenuBarBg] = ImVec4{0.13f, 0.14f, 0.16f, 1.00f};
    style.Colors[ImGuiCol_ScrollbarBg] = ImVec4{0.06f, 0.06f, 0.07f, 0.55f};
    style.Colors[ImGuiCol_ScrollbarGrab] = ImVec4{0.28f, 0.30f, 0.33f, 1.00f};
    style.Colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4{0.36f, 0.39f, 0.43f, 1.00f};
    style.Colors[ImGuiCol_ScrollbarGrabActive] = accent;
    style.Colors[ImGuiCol_CheckMark] = accentHi;
    style.Colors[ImGuiCol_SliderGrab] = accent;
    style.Colors[ImGuiCol_SliderGrabActive] = accentHi;
    style.Colors[ImGuiCol_Button] = ImVec4{0.22f, 0.24f, 0.27f, 1.00f};
    style.Colors[ImGuiCol_ButtonHovered] = accentLo;
    style.Colors[ImGuiCol_ButtonActive] = accent;
    style.Colors[ImGuiCol_Header] = ImVec4{0.19f, 0.21f, 0.24f, 1.00f};
    style.Colors[ImGuiCol_HeaderHovered] = accentLo;
    style.Colors[ImGuiCol_HeaderActive] = accent;
    style.Colors[ImGuiCol_Separator] = ImVec4{1.00f, 1.00f, 1.00f, 0.09f};
    style.Colors[ImGuiCol_SeparatorHovered] = accentLo;
    style.Colors[ImGuiCol_SeparatorActive] = accent;
    style.Colors[ImGuiCol_ResizeGrip] = ImVec4{1.00f, 1.00f, 1.00f, 0.06f};
    style.Colors[ImGuiCol_ResizeGripHovered] = accentLo;
    style.Colors[ImGuiCol_ResizeGripActive] = accent;
    style.Colors[ImGuiCol_Tab] = ImVec4{0.15f, 0.16f, 0.18f, 1.00f};
    style.Colors[ImGuiCol_TabHovered] = accentLo;
    style.Colors[ImGuiCol_TabSelected] = accent;
    style.Colors[ImGuiCol_TabSelectedOverline] = accentHi;
    style.Colors[ImGuiCol_TabDimmed] = ImVec4{0.12f, 0.13f, 0.14f, 1.00f};
    style.Colors[ImGuiCol_TabDimmedSelected] = accentLo;
    style.Colors[ImGuiCol_TextSelectedBg] = ImVec4{0.28f, 0.48f, 0.66f, 0.40f};
    style.Colors[ImGuiCol_DragDropTarget] = accentHi;
    style.Colors[ImGuiCol_NavCursor] = accentHi;
    style.Colors[ImGuiCol_PlotLines] = ImVec4{0.70f, 0.72f, 0.75f, 1.00f};
    style.Colors[ImGuiCol_PlotLinesHovered] = accentHi;
    style.Colors[ImGuiCol_PlotHistogram] = accent;
    style.Colors[ImGuiCol_PlotHistogramHovered] = accentHi;
}

// upstream main menu bar (sample.cpp ~1578): Sim / View / Samples.
void draw_menu_bar() {
    if !ImGui_BeginMainMenuBar() { return; }
    if ImGui_BeginMenu("Sim", true) {
        ignore ImGui_MenuItem("Pause", "P", &g_pause, true);
        if ImGui_MenuItem("Single Step", "O", false, true) { g_single_step += 1; }
        if ImGui_MenuItem("Restart", "R", false, true) { g_reset_pending = true; }
        ImGui_Separator();
        if ImGui_MenuItem("Previous Sample", "[", false, true) { g_switch_pending = -1; }
        if ImGui_MenuItem("Next Sample", "]", false, true) { g_switch_pending = 1; }
        ImGui_Separator();
        if ImGui_MenuItem("Reset Profile", null, false, true) {
            g_step_count = 0;
            g_profile_read = 0;
            g_profile_write = 0;
            g_profile_current = 0;
        }
        if ImGui_MenuItem("Dump Mem Stats", null, false, true) {
            b3World_DumpMemoryStats(g_world);
        }
        ImGui_Separator();
        if ImGui_MenuItem("Quit", "Esc", false, true) { sapp_request_quit(); }
        ImGui_EndMenu();
    }
    if ImGui_BeginMenu("View", true) {
        if ImGui_MenuItem("Hide UI", "Tab", false, true) { g_show_ui = false; }
        if ImGui_MenuItem("Frame Camera", null, false, true) { cam_frame(); }
        ImGui_Separator();
        ignore ImGui_MenuItem("Diagnostics", "M", &g_show_metrics, true);
        ImGui_PushItemWidth(8.0f * ImGui_GetFontSize());
        ignore ImGui_SliderFloat("Draw Distance", &cam_draw_distance, 10.0f, CAM_VIEW_DISTANCE, "%.0f m", 0);
        ImGui_PopItemWidth();

        // upstream sample.cpp:1618 View menu, same items and order.
        ImGui_Separator();
        ignore ImGui_MenuItem("Shapes", null, &g_dbg_shapes, true);
        ignore ImGui_MenuItem("Joints", null, &g_dbg_joints, true);
        ignore ImGui_MenuItem("Joint Extras", null, &g_dbg_joint_extras, true);
        ignore ImGui_MenuItem("Bounds", null, &g_dbg_bounds, true);
        ignore ImGui_MenuItem("Mass", null, &g_dbg_mass, true);
        ignore ImGui_MenuItem("Sleep", null, &g_dbg_sleep, true);
        ignore ImGui_MenuItem("Graph Colors", null, &g_dbg_graph_colors, true);
        ignore ImGui_MenuItem("Islands", null, &g_dbg_islands, true);
        ImGui_Separator();
        ignore ImGui_MenuItem("Contact Points", null, &g_dbg_contacts, true);
        ignore ImGui_MenuItem("Contact Normals", null, &g_dbg_contact_normals, true);
        ignore ImGui_MenuItem("Contact Forces", null, &g_dbg_contact_forces, true);
        // upstream sample.cpp:1659
        ImGui_Separator();
        ImGui_PushItemWidth(6.0f * ImGui_GetFontSize());
        ignore ImGui_InputFloat("Joint", &g_dbg_joint_scale, 0.0f, 0.0f, "%.2f", 0);
        ignore ImGui_InputFloat("Force", &g_dbg_force_scale, 0.0f, 0.0f, "%.6f", 0);
        ImGui_PopItemWidth();
        ImGui_EndMenu();
    }
    if ImGui_BeginMenu("Samples", true) {
        i32 i = 0;
        while i < NUM_SAMPLES {
            str category = g_samples[i].category;
            if ImGui_BeginMenu(category.data, true) {
                i32 j = i;
                while j < NUM_SAMPLES && str_equal(g_samples[j].category, category) {
                    if ImGui_MenuItem(g_samples[j].name.data, null, j == g_sample, true) {
                        g_switch_pending = j - g_sample;
                    }
                    j++;
                }
                ImGui_EndMenu();
            }
            while i < NUM_SAMPLES && str_equal(g_samples[i].category, category) { i++; }
        }
        ImGui_EndMenu();
    }
    ImGui_EndMainMenuBar();
}

// upstream DrawHud: minimal overlay when the UI is hidden (Tab)
void draw_hud() {
    f32 fontSize = ImGui_GetFontSize();
    ImDrawList* dl = ImGui_GetBackgroundDrawList(ImGui_GetMainViewport());
    noinit u8[160] buf;
    SampleDef* sd = &g_samples[g_sample];
    ignore snprintf(cast(u8*, &buf), 160, "%s : %s", sd.category.data, sd.name.data);
    ImDrawList_AddText(dl, null, 0.0f, ImVec2{5.0f, 1.5f * fontSize}, 0xFF00FFFF, cast(u8*, &buf), null, 0.0f, null);
    if g_pause {
        ImDrawList_AddText(dl, null, 0.0f, ImVec2{5.0f, 3.0f * fontSize}, 0xFF0000FF, "****PAUSED****", null, 0.0f, null);
    }
    ImVec2 disp = ImGui_GetIO().DisplaySize;
    ignore snprintf(cast(u8*, &buf), 160, "%.1f ms  step %d", g_frame_ms, g_step_count);
    ImDrawList_AddText(dl, null, 0.0f, ImVec2{5.0f, disp.y - 1.5f * fontSize}, 0xFF578B2E, cast(u8*, &buf), null, 0.0f, null);
}

// upstream Info panel (sample.cpp DrawInfoPanel): full-height at the
// right edge. Name/category, pause state, camera readout, then the
// per-sample controls and the Solver panel.
void draw_info_panel() {
    f32 fontSize = ImGui_GetFontSize();
    f32 menuWidth = 20.0f * fontSize;   // upstream INFO_PANEL_WIDTH
    // imgui display space, not sapp framebuffer pixels: the wasm canvas
    // is devicePixelRatio-scaled and the two differ there
    ImVec2 disp = ImGui_GetIO().DisplaySize;
    f32 menuBarHeight = ImGui_GetFrameHeight();
    ImGui_SetNextWindowPos(
        ImVec2{disp.x - menuWidth - 0.5f * fontSize, menuBarHeight + 0.5f * fontSize},
        ImGuiCond_None, ImVec2{0.0f, 0.0f});
    ImGui_SetNextWindowSize(ImVec2{menuWidth, disp.y - menuBarHeight - fontSize}, ImGuiCond_None);
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize
        | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar;
    if !ImGui_Begin("Info", null, flags) {
        ImGui_End();
        return;
    }

    SampleDef* sd = &g_samples[g_sample];
    ImGui_TextColored(ImVec4{0.855f, 0.647f, 0.125f, 1.0f}, "%s", sd.name.data);
    ImGui_TextColored(ImVec4{0.827f, 0.827f, 0.827f, 1.0f}, "%s", sd.category.data);
    ImGui_Separator();

    if g_pause {
        ImGui_TextColored(ImVec4{1.0f, 0.0f, 0.0f, 1.0f}, "PAUSED");
        ImGui_SameLine(0.0f, -1.0f);
        ImGui_TextDisabled("(p)");
        ImGui_Separator();
    }

    ImGui_TextColored(ImVec4{0.180f, 0.545f, 0.341f, 1.0f}, "%.1f ms", g_frame_ms);
    ImGui_TextColored(ImVec4{0.180f, 0.545f, 0.341f, 1.0f}, "step %d", g_step_count);
    ImGui_Separator();

    ImGui_TextColored(ImVec4{0.180f, 0.545f, 0.341f, 1.0f}, "pivot m (%.1f, %.1f, %.1f)",
                      cam_pivot.x, cam_pivot.y, cam_pivot.z);
    f32 yawDeg = cam_yaw * 180.0f / PI_F;
    f32 pitchDeg = cam_pitch * 180.0f / PI_F;
    ImGui_TextColored(ImVec4{0.180f, 0.545f, 0.341f, 1.0f}, "yaw/pitch (%.1f, %.1f)", yawDeg, pitchDeg);
    ImGui_TextColored(ImVec4{0.180f, 0.545f, 0.341f, 1.0f}, "radius m %.1f, speed m/s %.1f", cam_radius, cam_speed);
    ImGui_Separator();

    // per-sample DrawControls slot (upstream draws them between the
    // camera readout and the Solver header)
    ImGui_PushItemWidth(6.0f * fontSize);
    if g_samples[g_sample].controls != null && g_samples[g_sample].controls() {
        ImGui_Separator();
    }
    ImGui_PopItemWidth();

    // Solver panel, verbatim from sample.cpp ~1953
    if !g_samples[g_sample].hideSolverControls
        && ImGui_CollapsingHeader("Solver", ImGuiTreeNodeFlags_DefaultOpen) {
        ImGui_PushItemWidth(6.0f * fontSize);
        ignore ImGui_SliderInt("Sub-steps##Solver", &g_substeps, 1, 50, null, 0);
        ignore ImGui_SliderFloat("Hertz##Solver", &g_hertz, 5.0f, 240.0f, "%.0f hz", 0);
        ImGui_BeginDisabled(g_single_threaded);
        if ImGui_SliderInt("Workers##Solver", &g_workers, 1, g_max_workers, null, 0) {
            g_reset_pending = true;
        }
        ImGui_EndDisabled();
        f32 recyclingCentimeters = 100.0f * g_recycle_distance;
        if ImGui_SliderFloat("Recycle##Solver", &recyclingCentimeters, 0.0f, 10.0f, "%.1f cm", 0) {
            g_recycle_distance = 0.01f * recyclingCentimeters;
            b3World_SetContactRecycleDistance(g_world, g_recycle_distance);
        }
        ImGui_PopItemWidth();

        ignore ImGui_Checkbox("Sleep##Solver", &g_enable_sleep);
        ignore ImGui_Checkbox("Warm Starting##Solver", &g_enable_warm);
        ignore ImGui_Checkbox("Continuous##Solver", &g_enable_continuous);

        if ImGui_Button("Restart", ImVec2{0.0f, 0.0f}) {
            g_reset_pending = true;
        }
    }

    ImGui_End();
}

// upstream AddSegment: one span of the Profile flame strip
f32 flame_segment(ImDrawList* dl, f32 availWidth, f32 t, f32 stepNow,
                  u32 col, f32 x, ImVec2 cursor, f32 barHeight) {
    f32 w = availWidth * (t / stepNow);
    if w > 0.0f {
        ImDrawList_AddRectFilled(dl, ImVec2{x, cursor.y},
                                 ImVec2{x + w, cursor.y + barHeight}, col, 0.0f, 0);
        x += w;
    }
    return x;
}

// Metrics drawer (upstream Sample::DrawMetrics): bottom-anchored tab
// bar. Profile and Counters follow upstream's stock-imgui code. The
// Frame Time chart is ImPlot upstream; this uses three PlotLines.
void draw_metrics() {
    if !g_show_metrics { return; }
    f32 fontSize = ImGui_GetFontSize();
    f32 menuWidth = 20.0f * fontSize;
    f32 drawerHeight = 16.0f * fontSize;
    ImVec2 mdisp = ImGui_GetIO().DisplaySize;
    f32 drawerWidth = mdisp.x - menuWidth - 1.5f * fontSize;
    ImGui_SetNextWindowPos(
        ImVec2{0.5f * fontSize, mdisp.y - drawerHeight - 0.5f * fontSize},
        ImGuiCond_None, ImVec2{0.0f, 0.0f});
    ImGui_SetNextWindowSize(ImVec2{drawerWidth, drawerHeight}, ImGuiCond_None);
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize
        | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar;
    if !ImGui_Begin("Metrics", null, flags) {
        ImGui_End();
        return;
    }
    if !ImGui_BeginTabBar("MetricsTabs", 0) {
        ImGui_End();
        return;
    }

    i32 count = g_profile_write - g_profile_read;

    if ImGui_BeginTabItem("Profile", null, 0) {
        // upstream rows: name, indent, color, b3Profile field index
        // (field order per struct; sensorHits is unlisted upstream)
        const i32 PROWS = 22;
        u8*[PROWS] rowName;
        i32[PROWS] rowIndent;
        u32[PROWS] rowColor;
        i32[PROWS] rowField;
        rowName[0] = "step";          rowIndent[0] = 0;  rowColor[0] = PROF_COLOR_STEP;     rowField[0] = 0;
        rowName[1] = "pairs";         rowIndent[1] = 0;  rowColor[1] = PROF_COLOR_PAIRS;    rowField[1] = 1;
        rowName[2] = "collide";       rowIndent[2] = 0;  rowColor[2] = PROF_COLOR_COLLIDE;  rowField[2] = 2;
        rowName[3] = "solve";         rowIndent[3] = 0;  rowColor[3] = PROF_COLOR_SOLVE;    rowField[3] = 3;
        rowName[4] = "setup";         rowIndent[4] = 1;  rowColor[4] = PROF_COLOR_DEFAULT;  rowField[4] = 4;
        rowName[5] = "constraints";   rowIndent[5] = 1;  rowColor[5] = PROF_COLOR_DEFAULT;  rowField[5] = 5;
        rowName[6] = "prepare";       rowIndent[6] = 2;  rowColor[6] = PROF_COLOR_DEFAULT;  rowField[6] = 6;
        rowName[7] = "velocities";    rowIndent[7] = 2;  rowColor[7] = PROF_COLOR_DEFAULT;  rowField[7] = 7;
        rowName[8] = "warm start";    rowIndent[8] = 2;  rowColor[8] = PROF_COLOR_DEFAULT;  rowField[8] = 8;
        rowName[9] = "bias";          rowIndent[9] = 2;  rowColor[9] = PROF_COLOR_DEFAULT;  rowField[9] = 9;
        rowName[10] = "positions";    rowIndent[10] = 2; rowColor[10] = PROF_COLOR_DEFAULT; rowField[10] = 10;
        rowName[11] = "relax";        rowIndent[11] = 2; rowColor[11] = PROF_COLOR_DEFAULT; rowField[11] = 11;
        rowName[12] = "restitution";  rowIndent[12] = 2; rowColor[12] = PROF_COLOR_DEFAULT; rowField[12] = 12;
        rowName[13] = "store";        rowIndent[13] = 2; rowColor[13] = PROF_COLOR_DEFAULT; rowField[13] = 13;
        rowName[14] = "split islands"; rowIndent[14] = 2; rowColor[14] = PROF_COLOR_DEFAULT; rowField[14] = 14;
        rowName[15] = "transforms";   rowIndent[15] = 1; rowColor[15] = PROF_COLOR_DEFAULT; rowField[15] = 15;
        rowName[16] = "joint events"; rowIndent[16] = 1; rowColor[16] = PROF_COLOR_DEFAULT; rowField[16] = 17;
        rowName[17] = "hit events";   rowIndent[17] = 1; rowColor[17] = PROF_COLOR_DEFAULT; rowField[17] = 18;
        rowName[18] = "refit BVH";    rowIndent[18] = 1; rowColor[18] = PROF_COLOR_DEFAULT; rowField[18] = 19;
        rowName[19] = "sleep";        rowIndent[19] = 1; rowColor[19] = PROF_COLOR_DEFAULT; rowField[19] = 21;
        rowName[20] = "bullets";      rowIndent[20] = 1; rowColor[20] = PROF_COLOR_DEFAULT; rowField[20] = 20;
        rowName[21] = "sensors";      rowIndent[21] = 0; rowColor[21] = PROF_COLOR_SENSORS; rowField[21] = 22;

        // unroll the ring into per-row histories (g_prof_hist)
        f32[PROWS] totals;
        for i32 i = 0; i < count; i++ {
            i32 idx = (g_profile_read + i) & (PROFILE_CAP - 1);
            f32* pf = cast(f32*, &g_profiles[idx]);
            for i32 r = 0; r < PROWS; r++ {
                f32 v = *(pf + rowField[r]);
                g_prof_hist[r * PROFILE_CAP + i] = v;
                totals[r] += v;
            }
        }

        // smoothed over the last few frames so bars don't jitter
        const i32 NOW_WINDOW = 10;
        f32[PROWS] now;
        i32 nwin = count < NOW_WINDOW ? count : NOW_WINDOW;
        if nwin > 0 {
            f32 inv = 1.0f / cast(f32, nwin);
            for i32 r = 0; r < PROWS; r++ {
                f32 sum = 0.0f;
                for i32 i = count - nwin; i < count; i++ {
                    sum += g_prof_hist[r * PROFILE_CAP + i];
                }
                now[r] = sum * inv;
            }
        }

        f32[PROWS] avg;
        if count > 0 {
            f32 scale = 1.0f / cast(f32, count);
            for i32 r = 0; r < PROWS; r++ { avg[r] = scale * totals[r]; }
        }

        f32[PROWS] rowMax;
        for i32 r = 0; r < PROWS; r++ {
            for i32 i = 0; i < count; i++ {
                if g_prof_hist[r * PROFILE_CAP + i] > rowMax[r] { rowMax[r] = g_prof_hist[r * PROFILE_CAP + i]; }
            }
        }

        // derive parent/child links from the indent levels
        i32[PROWS] parents;
        bool[PROWS] hasChildren;
        {
            i32[8] pstack;
            i32 pstackSize = 0;
            for i32 i = 0; i < PROWS; i++ {
                while pstackSize > 0 && rowIndent[pstack[pstackSize - 1]] >= rowIndent[i] {
                    pstackSize--;
                }
                parents[i] = pstackSize > 0 ? pstack[pstackSize - 1] : 0 - 1;
                pstack[pstackSize] = i;
                pstackSize++;
                if parents[i] >= 0 { hasChildren[parents[i]] = true; }
            }
        }

        // bars scale to the step row so proportions stay consistent
        f32 stepNow = now[0] > 0.001f ? now[0] : 0.001f;

        if ImGui_Button("Reset", ImVec2{0.0f, 0.0f}) {
            // upstream ResetProfile
            g_step_count = 0;
            g_profile_read = 0;
            g_profile_write = 0;
            g_profile_current = 0;
        }
        ImGui_SameLine(0.0f, -1.0f);
        ignore ImGui_Checkbox("Show plots", &g_profile_show_plots);
        ImGui_SameLine(0.0f, -1.0f);
        ImGui_Text("   step %.2f ms", now[0]);

        // flame strip: step subdivided by top-level children
        {
            f32 pairsT = now[1];
            f32 collideT = now[2];
            f32 solveT = now[3];
            f32 sensorsT = now[21];
            f32 otherT = stepNow - pairsT - collideT - solveT - sensorsT;
            if otherT < 0.0f { otherT = 0.0f; }
            f32 availWidth = ImGui_GetContentRegionAvail().x;
            f32 barHeight = 1.5f * fontSize;
            ImDrawList* dl = ImGui_GetWindowDrawList();
            ImVec2 fcursor = ImGui_GetCursorScreenPos();
            f32 x = fcursor.x;
            x = flame_segment(dl, availWidth, pairsT, stepNow, PROF_COLOR_PAIRS, x, fcursor, barHeight);
            x = flame_segment(dl, availWidth, collideT, stepNow, PROF_COLOR_COLLIDE, x, fcursor, barHeight);
            x = flame_segment(dl, availWidth, solveT, stepNow, PROF_COLOR_SOLVE, x, fcursor, barHeight);
            x = flame_segment(dl, availWidth, sensorsT, stepNow, PROF_COLOR_SENSORS, x, fcursor, barHeight);
            x = flame_segment(dl, availWidth, otherT, stepNow, PROF_COLOR_OTHER, x, fcursor, barHeight);
            ImGui_Dummy(ImVec2{availWidth, barHeight});
        }

        ImGuiTableFlags profFlags = ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg
            | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_ScrollY;
        i32 colCount = g_profile_show_plots ? 6 : 5;
        ImVec2 tableSize = ImGui_GetContentRegionAvail();
        if ImGui_BeginTable("profile", colCount, profFlags, tableSize, 0.0f) {
            ImGui_TableSetupColumn("section", ImGuiTableColumnFlags_WidthFixed, 8.0f * fontSize, 0);
            ImGui_TableSetupColumn("now", ImGuiTableColumnFlags_WidthFixed, 3.0f * fontSize, 0);
            ImGui_TableSetupColumn("avg", ImGuiTableColumnFlags_WidthFixed, 3.0f * fontSize, 0);
            ImGui_TableSetupColumn("max", ImGuiTableColumnFlags_WidthFixed, 3.0f * fontSize, 0);
            ImGui_TableSetupColumn("% step", ImGuiTableColumnFlags_WidthFixed, 8.0f * fontSize, 0);
            if g_profile_show_plots {
                ImGui_TableSetupColumn("history", ImGuiTableColumnFlags_WidthFixed, 16.0f * fontSize, 0);
            }
            ImGui_TableHeadersRow();

            f32 rowHeight = 1.5f * fontSize;

            for i32 r = 0; r < PROWS; r++ {
                bool visible = true;
                for i32 pr = parents[r]; pr >= 0; pr = parents[pr] {
                    if !g_profile_row_open[pr] { visible = false; break; }
                }
                if !visible { continue; }

                // hide leaf rows that are entirely zero
                if !hasChildren[r] && now[r] == 0.0f && avg[r] == 0.0f && rowMax[r] == 0.0f {
                    continue;
                }

                ImGui_TableNextRow(0, 0.0f);

                ignore ImGui_TableNextColumn();
                if rowIndent[r] > 0 { ImGui_Indent(cast(f32, rowIndent[r]) * fontSize); }
                if hasChildren[r] {
                    ImGuiTreeNodeFlags tnf = ImGuiTreeNodeFlags_OpenOnArrow
                        | ImGuiTreeNodeFlags_OpenOnDoubleClick | ImGuiTreeNodeFlags_NoTreePushOnOpen;
                    ImGui_PushStyleColor(ImGuiCol_Text, rowColor[r]);
                    g_profile_row_open[r] = ImGui_TreeNodeEx(rowName[r], tnf);
                    ImGui_PopStyleColor(1);
                } else {
                    f32 leafIndent = ImGui_GetTreeNodeToLabelSpacing();
                    ImGui_Indent(leafIndent);
                    ImGui_PushStyleColor(ImGuiCol_Text, rowColor[r]);
                    ImGui_TextUnformatted(rowName[r], null);
                    ImGui_PopStyleColor(1);
                    ImGui_Unindent(leafIndent);
                }
                if rowIndent[r] > 0 { ImGui_Unindent(cast(f32, rowIndent[r]) * fontSize); }

                ignore ImGui_TableNextColumn();
                ImGui_Text("%.2f", now[r]);
                ignore ImGui_TableNextColumn();
                ImGui_Text("%.2f", avg[r]);
                ignore ImGui_TableNextColumn();
                ImGui_Text("%.2f", rowMax[r]);

                ignore ImGui_TableNextColumn();
                f32 frac = now[r] / stepNow;
                if frac > 1.0f { frac = 1.0f; }
                if frac < 0.0f { frac = 0.0f; }
                ImGui_PushStyleColor(ImGuiCol_PlotHistogram, rowColor[r]);
                ImGui_ProgressBar(frac, ImVec2{-0.001f, 0.0f}, "");
                ImGui_PopStyleColor(1);

                if g_profile_show_plots {
                    ignore ImGui_TableNextColumn();
                    if count > 1 {
                        u8[16] plotId;
                        ignore snprintf(cast(u8*, &plotId), 16, "##h%d", r);
                        ImGui_PushStyleColor(ImGuiCol_PlotLines, rowColor[r]);
                        ImGui_PlotLines(cast(u8*, &plotId), &g_prof_hist[r * PROFILE_CAP], count, 0,
                                        null, 0.0f, rowMax[r] * 1.05f + 0.001f,
                                        ImVec2{-0.001f, rowHeight}, 4);
                        ImGui_PopStyleColor(1);
                    }
                }
            }
            ImGui_EndTable();
        }

        ImGui_EndTabItem();
    }

    if ImGui_BeginTabItem("Counters", null, 0) {
        ignore ImGui_BeginChild("##counters_scroll", ImVec2{0.0f, 0.0f}, 0, 0);
        b3Counters s = b3World_GetCounters(g_world);
        const i32 colorCount = 24;
        const i32 overflowIndex = 23;
        const i32 manifoldBucketCount = 8;

        i32 totalCount = 0;
        i32 maxCount = 1;
        for i32 i = 0; i < colorCount; i++ {
            totalCount += s.colorCounts[i];
            if i != overflowIndex && s.colorCounts[i] > maxCount { maxCount = s.colorCounts[i]; }
        }
        i32 totalManifolds = 0;
        i32 maxManifolds = 1;
        for i32 i = 0; i < manifoldBucketCount; i++ {
            totalManifolds += s.manifoldCounts[i];
            if s.manifoldCounts[i] > maxManifolds { maxManifolds = s.manifoldCounts[i]; }
        }

        ImGui_Text("bodies/shapes/contacts/joints = %d/%d/%d/%d",
                   s.bodyCount, s.shapeCount, s.contactCount, s.jointCount);
        {
            f32 frac = 0.0f;
            if s.awakeContactCount > 0 {
                frac = cast(f32, s.recycledContactCount) / cast(f32, s.awakeContactCount);
                if frac > 1.0f { frac = 1.0f; }
            }
            u8[32] overlay;
            ignore snprintf(cast(u8*, &overlay), 32, "%d / %d", s.recycledContactCount, s.awakeContactCount);
            ImGui_TextUnformatted("recycled contacts", null);
            ImGui_SameLine(0.0f, -1.0f);
            ImGui_ProgressBar(frac, ImVec2{-0.001f, 0.0f}, cast(u8*, &overlay));
        }
        ImGui_Text("islands/tasks = %d/%d", s.islandCount, s.taskCount);
        ImGui_Text("tree height static/movable = %d/%d", s.staticTreeHeight, s.treeHeight);
        ImGui_Text("sat call/hit = %d/%d", s.satCallCount, s.satCacheHitCount);
        ImGui_Text("toi d/p/r = %d/%d/%d", s.distanceIterations, s.pushBackIterations, s.rootIterations);
        ImGui_Text("stack allocator size = %d K", s.stackUsed / 1024);
        ImGui_Text("arena capacity = %d K", s.arenaCapacity / 1024);
        ImGui_Text("total allocation = %d K", s.byteCount / 1024);

        ImGui_Separator();
        b3Capacity c = b3World_GetMaxCapacity(g_world);
        ImGui_Text("max capacities");
        ImGui_BulletText("static shapes/bodies = %d/%d", c.staticShapeCount, c.staticBodyCount);
        ImGui_BulletText("dynamic shapes/bodies = %d/%d", c.dynamicShapeCount, c.dynamicBodyCount);
        ImGui_BulletText("contacts = %d", c.contactCount);

        ImGui_Separator();
        ImGui_Text("%d constraints across %d colors", totalCount, colorCount);

        ImGuiTableFlags tableFlags = ImGuiTableFlags_BordersInnerV
            | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit;
        if ImGui_BeginTable("graphColors", 3, tableFlags, ImVec2{0.0f, 0.0f}, 0.0f) {
            ImGui_TableSetupColumn("color", ImGuiTableColumnFlags_WidthFixed, 3.5f * fontSize, 0);
            ImGui_TableSetupColumn("count", ImGuiTableColumnFlags_WidthFixed, 5.0f * fontSize, 0);
            ImGui_TableSetupColumn("share", ImGuiTableColumnFlags_WidthFixed, 16.0f * fontSize, 0);
            ImGui_TableHeadersRow();
            f32 invMax = 1.0f / cast(f32, maxCount);
            for i32 i = 0; i < colorCount; i++ {
                i32 cnt = s.colorCounts[i];
                bool isOverflow = i == overflowIndex;
                // skip empty slots, but always show overflow
                if cnt == 0 && !isOverflow { continue; }
                u32 hex = cast(u32, b3GetGraphColor(i));
                u32 swatch = ((hex >> 16) & 0xFF) | (((hex >> 8) & 0xFF) << 8)
                    | ((hex & 0xFF) << 16) | 0xFF000000;
                u32 red = 220 | (60 << 8) | (60 << 16) | 0xFF000000;
                u32 barColor = isOverflow ? red : swatch;
                ImGui_TableNextRow(0, 0.0f);
                ignore ImGui_TableNextColumn();
                if isOverflow {
                    ImGui_PushStyleColor(ImGuiCol_Text, red);
                    ImGui_TextUnformatted("over", null);
                    ImGui_PopStyleColor(1);
                } else {
                    ImGui_PushStyleColor(ImGuiCol_Text, swatch);
                    ImGui_Text("%d", i);
                    ImGui_PopStyleColor(1);
                }
                ignore ImGui_TableNextColumn();
                ImGui_Text("%d", cnt);
                ignore ImGui_TableNextColumn();
                f32 frac = cast(f32, cnt) * invMax;
                if frac > 1.0f { frac = 1.0f; }
                ImGui_PushStyleColor(ImGuiCol_PlotHistogram, barColor);
                ImGui_ProgressBar(frac, ImVec2{-0.001f, 0.0f}, "");
                ImGui_PopStyleColor(1);
            }
            ImGui_EndTable();
        }

        ImGui_Separator();
        ImGui_Text("%d manifolds across %d buckets", totalManifolds, manifoldBucketCount);
        if ImGui_BeginTable("manifolds", 3, tableFlags, ImVec2{0.0f, 0.0f}, 0.0f) {
            ImGui_TableSetupColumn("manifolds", ImGuiTableColumnFlags_WidthFixed, 3.5f * fontSize, 0);
            ImGui_TableSetupColumn("count", ImGuiTableColumnFlags_WidthFixed, 5.0f * fontSize, 0);
            ImGui_TableSetupColumn("share", ImGuiTableColumnFlags_WidthFixed, 16.0f * fontSize, 0);
            ImGui_TableHeadersRow();
            f32 invMax = 1.0f / cast(f32, maxManifolds);
            for i32 i = 0; i < manifoldBucketCount; i++ {
                i32 cnt = s.manifoldCounts[i];
                if cnt == 0 { continue; }
                ImGui_TableNextRow(0, 0.0f);
                ignore ImGui_TableNextColumn();
                ImGui_Text("%d", i + 1);
                ignore ImGui_TableNextColumn();
                ImGui_Text("%d", cnt);
                ignore ImGui_TableNextColumn();
                f32 frac = cast(f32, cnt) * invMax;
                if frac > 1.0f { frac = 1.0f; }
                ImGui_ProgressBar(frac, ImVec2{-0.001f, 0.0f}, "");
            }
            ImGui_EndTable();
        }
        ImGui_EndChild();
        ImGui_EndTabItem();
    }

    if ImGui_BeginTabItem("Frame Time", null, 0) {
        f32 maxValue = 0.0f;
        f32[PROFILE_CAP] stepTimes;
        f32[PROFILE_CAP] collideTimes;
        f32[PROFILE_CAP] solveTimes;
        for i32 i = 0; i < count; i++ {
            i32 idx = (g_profile_read + i) & (PROFILE_CAP - 1);
            stepTimes[i] = g_profiles[idx].step;
            collideTimes[i] = g_profiles[idx].collide;
            solveTimes[i] = g_profiles[idx].solve;
            if stepTimes[i] > maxValue { maxValue = stepTimes[i]; }
        }
        f32 maxScale = (maxValue > 1.0f ? maxValue : 1.0f) * 1.05f;
        ImVec2 avail = ImGui_GetContentRegionAvail();
        f32 plotW = avail.x - 4.0f * fontSize;
        f32 plotH = (avail.y - fontSize) / 3.0f;
        ImGui_PushStyleColor(ImGuiCol_PlotLines, PROF_COLOR_STEP);
        ImGui_PlotLines("step", cast(f32*, &stepTimes), count, 0, null, 0.0f, maxScale, ImVec2{plotW, plotH}, 4);
        ImGui_PopStyleColor(1);
        ImGui_PushStyleColor(ImGuiCol_PlotLines, PROF_COLOR_COLLIDE);
        ImGui_PlotLines("collide", cast(f32*, &collideTimes), count, 0, null, 0.0f, maxScale, ImVec2{plotW, plotH}, 4);
        ImGui_PopStyleColor(1);
        ImGui_PushStyleColor(ImGuiCol_PlotLines, PROF_COLOR_SOLVE);
        ImGui_PlotLines("solve", cast(f32*, &solveTimes), count, 0, null, 0.0f, maxScale, ImVec2{plotW, plotH}, 4);
        ImGui_PopStyleColor(1);
        ImGui_EndTabItem();
    }

    ImGui_EndTabBar();
    ImGui_End();
}
