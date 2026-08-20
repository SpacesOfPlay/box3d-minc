// Orbit + fly camera, from samples/host/camera.cpp/.h.
//
// Two modes share one state: ORBIT (Alt held: left-drag orbit,
// middle-drag pan, right-drag radial zoom) and FLY (right mouse without
// Alt: FPS look + WASD at cam_speed, scroll tunes speed). Bare scroll is
// multiplicative zoom. OnEvent accumulates deltas, cam_update consumes
// them per frame. forward is pivot -> eye (+view-Z); the look direction
// is -forward.

import box3d;
import sokol_all;
import sokol_imgui;
import math;
import linear;
import str;
import gui;
import renderer;
import sample;
import sample_benchmark;
import sample_bodies;
import sample_continuous;
import sample_robustness;
import sample_stacking;
import sample_joint;
import sample_compound;
import sample_world;

const f32 CAM_VIEW_DISTANCE = 1000.0f;
const f32 CAM_ORBIT_SENS = 0.005f;
const f32 CAM_FLY_LOOK_SENS = 0.005f;
const f32 CAM_THIRD_PERSON_MIN_RADIUS = 1.0f;
const f32 CAM_PAN_SENS = 0.005f;
const f32 CAM_RADIAL_ZOOM_SENS = 0.02f;
const f32 CAM_FLY_SPEED_STEP = 1.0f;
const f32 CAM_ZOOM_STEP = 0.9f;
const f32 CAM_MIN_DIST = 0.1f;
const f32 CAM_MIN_SPEED = 0.06f;
const f32 CAM_MAX_SPEED = 30000.0f;

float3 cam_pivot;
f32 cam_yaw;          // radians, around Y
f32 cam_pitch;        // radians, camera-frame X
f32 cam_radius;       // meters from pivot
f32 cam_speed = 10.0f;
// upstream drawDistance: cull box half extent around the eye, meters.
// Separate from CAM_VIEW_DISTANCE, which is the far plane / zoom cap.
// Reset on sample load; a sample may pin a shorter one.
f32 cam_draw_distance = CAM_VIEW_DISTANCE;
float3 cam_right;
float3 cam_up;
float3 cam_forward;   // pivot -> eye
float3 cam_eye;

// per-frame input deltas (OnEvent accumulates, cam_update consumes)
f32 cam_orbit_dx; f32 cam_orbit_dy;
f32 cam_pan_dx; f32 cam_pan_dy;
f32 cam_radial_dy;
f32 cam_scroll;
f32 cam_speed_scroll;
// sticky input state
bool cam_left_down; bool cam_right_down; bool cam_middle_down;
bool cam_alt_down;
// upstream Camera::m_thirdPerson: input locks to wheel zoom and the sample
// drives cam_pivot to the body it follows.
bool cam_third_person;
bool cam_w_down; bool cam_a_down; bool cam_s_down; bool cam_d_down;

f32 cam_pitch_lim() { return 0.5f * PI_F - 0.01f; }

f32 cam_clamp(f32 v, f32 lo, f32 hi) {
    if v < lo { return lo; }
    if v > hi { return hi; }
    return v;
}

float3 cam_forward_from_angles(f32 yaw, f32 pitch) {
    f32 cp = cosf(pitch);
    return float3{sinf(yaw) * cp, sinf(pitch), cosf(yaw) * cp};
}

// RebuildBasisAndView: worldUp re-orthogonalized against forward;
// right = up x forward. Right-handed view basis, eye on +forward.
void cam_rebuild_basis() {
    cam_forward = cam_forward_from_angles(cam_yaw, cam_pitch);
    f32 d = cam_forward.y;   // dot(worldUp, forward)
    float3 upun = float3{-d * cam_forward.x, 1.0f - d * cam_forward.y, -d * cam_forward.z};
    f32 ul = sqrtf(upun.x * upun.x + upun.y * upun.y + upun.z * upun.z);
    cam_up = float3{upun.x / ul, upun.y / ul, upun.z / ul};
    float3 r = float3{cam_up.y * cam_forward.z - cam_up.z * cam_forward.y,
                      cam_up.z * cam_forward.x - cam_up.x * cam_forward.z,
                      cam_up.x * cam_forward.y - cam_up.y * cam_forward.x};
    f32 rl = sqrtf(r.x * r.x + r.y * r.y + r.z * r.z);
    cam_right = float3{r.x / rl, r.y / rl, r.z / rl};
    cam_eye = float3{cam_pivot.x + cam_radius * cam_forward.x,
                     cam_pivot.y + cam_radius * cam_forward.y,
                     cam_pivot.z + cam_radius * cam_forward.z};
}

void cam_set_orbit(f32 yaw, f32 pitch, f32 radius) {
    cam_yaw = yaw;
    cam_pitch = cam_clamp(pitch, -cam_pitch_lim(), cam_pitch_lim());
    cam_radius = cam_clamp(radius, CAM_MIN_DIST, CAM_VIEW_DISTANCE);
    cam_rebuild_basis();
}

// Box3D sample signature: degrees + pivot (Camera::SetView).
void cam_set_view(f32 yawDeg, f32 pitchDeg, f32 radius, float3 pivot) {
    cam_pivot = pivot;
    cam_set_orbit(yawDeg * PI_F / 180.0f, pitchDeg * PI_F / 180.0f, radius);
}

// Camera::OnEvent, verbatim: route deltas by button + Alt state.
void cam_on_event(sapp_event* ev) {
    u32 mods = ev.modifiers & (SAPP_MODIFIER_SHIFT | SAPP_MODIFIER_CTRL | SAPP_MODIFIER_ALT);
    bool canOrbit = mods == SAPP_MODIFIER_ALT;

    if ev.type == SAPP_EVENTTYPE_MOUSE_DOWN {
        if ev.mouse_button == SAPP_MOUSEBUTTON_LEFT { cam_left_down = true; }
        if ev.mouse_button == SAPP_MOUSEBUTTON_RIGHT { cam_right_down = true; }
        if ev.mouse_button == SAPP_MOUSEBUTTON_MIDDLE { cam_middle_down = true; }
    } else if ev.type == SAPP_EVENTTYPE_MOUSE_UP {
        if ev.mouse_button == SAPP_MOUSEBUTTON_LEFT { cam_left_down = false; }
        if ev.mouse_button == SAPP_MOUSEBUTTON_RIGHT { cam_right_down = false; }
        if ev.mouse_button == SAPP_MOUSEBUTTON_MIDDLE { cam_middle_down = false; }
    } else if ev.type == SAPP_EVENTTYPE_MOUSE_MOVE {
        if cam_third_person {
            // Pointer is locked: deltas are the look input, no button.
            cam_orbit_dx += ev.mouse_dx;
            cam_orbit_dy += ev.mouse_dy;
        } else if canOrbit && cam_left_down {
            cam_orbit_dx += ev.mouse_dx;
            cam_orbit_dy += ev.mouse_dy;
        } else if canOrbit && cam_middle_down {
            cam_pan_dx += ev.mouse_dx;
            cam_pan_dy += ev.mouse_dy;
        } else if canOrbit && cam_right_down {
            cam_radial_dy += ev.mouse_dy;
        } else if !canOrbit && cam_right_down {
            // fly look shares the orbit accumulators
            cam_orbit_dx += ev.mouse_dx;
            cam_orbit_dy += ev.mouse_dy;
        }
    } else if ev.type == SAPP_EVENTTYPE_MOUSE_SCROLL {
        if !canOrbit && cam_right_down { cam_speed_scroll += ev.scroll_y; }
        else { cam_scroll += ev.scroll_y; }
    } else if ev.type == SAPP_EVENTTYPE_KEY_DOWN {
        if ev.key_code == SAPP_KEYCODE_W { cam_w_down = true; }
        if ev.key_code == SAPP_KEYCODE_A { cam_a_down = true; }
        if ev.key_code == SAPP_KEYCODE_S { cam_s_down = true; }
        if ev.key_code == SAPP_KEYCODE_D { cam_d_down = true; }
        if ev.key_code == SAPP_KEYCODE_LEFT_ALT || ev.key_code == SAPP_KEYCODE_RIGHT_ALT {
            cam_alt_down = true;
        }
    } else if ev.type == SAPP_EVENTTYPE_KEY_UP {
        if ev.key_code == SAPP_KEYCODE_W { cam_w_down = false; }
        if ev.key_code == SAPP_KEYCODE_A { cam_a_down = false; }
        if ev.key_code == SAPP_KEYCODE_S { cam_s_down = false; }
        if ev.key_code == SAPP_KEYCODE_D { cam_d_down = false; }
        if ev.key_code == SAPP_KEYCODE_LEFT_ALT || ev.key_code == SAPP_KEYCODE_RIGHT_ALT {
            cam_alt_down = false;
        }
    } else if ev.type == SAPP_EVENTTYPE_UNFOCUSED {
        // drop held input on focus loss, so a drag does not persist
        cam_left_down = false;
        cam_right_down = false;
        cam_middle_down = false;
        cam_w_down = false;
        cam_a_down = false;
        cam_s_down = false;
        cam_d_down = false;
        cam_alt_down = false;
    }
}

// Camera::Update, verbatim minus the sim->display transform (identity
// for us).
void cam_update(f32 dt) {
    if cam_third_person {
        // Follow cam: the sample drives cam_pivot to the tracked body and
        // calls cam_rebuild. The only camera input is wheel zoom on the
        // radius.
        if cam_scroll != 0.0f {
            cam_radius -= cam_scroll;
            if cam_radius < CAM_THIRD_PERSON_MIN_RADIUS {
                cam_radius = CAM_THIRD_PERSON_MIN_RADIUS;
            }
        }
        // Mouse look.
        if cam_orbit_dx != 0.0f || cam_orbit_dy != 0.0f {
            cam_yaw -= cam_orbit_dx * CAM_FLY_LOOK_SENS;
            cam_pitch += cam_orbit_dy * CAM_FLY_LOOK_SENS;
            cam_pitch = cam_clamp(cam_pitch, -cam_pitch_lim(), cam_pitch_lim());
        }
        // Drop the other accumulators so a return to orbit/fly starts clean.
        cam_orbit_dx = 0.0f;
        cam_orbit_dy = 0.0f;
        cam_pan_dx = 0.0f;
        cam_pan_dy = 0.0f;
        cam_radial_dy = 0.0f;
        cam_scroll = 0.0f;
        cam_speed_scroll = 0.0f;
        cam_rebuild_basis();
        return;
    }

    bool flyMode = cam_right_down && !cam_alt_down;

    if flyMode {
        // snapshot the eye before rotating: FPS look pivots on the eye
        float3 fwd0 = cam_forward_from_angles(cam_yaw, cam_pitch);
        float3 eyeBefore = float3{cam_pivot.x + cam_radius * fwd0.x,
                                  cam_pivot.y + cam_radius * fwd0.y,
                                  cam_pivot.z + cam_radius * fwd0.z};

        if cam_orbit_dx != 0.0f || cam_orbit_dy != 0.0f {
            cam_yaw -= cam_orbit_dx * CAM_FLY_LOOK_SENS;
            cam_pitch += cam_orbit_dy * CAM_FLY_LOOK_SENS;
            cam_pitch = cam_clamp(cam_pitch, -cam_pitch_lim(), cam_pitch_lim());
        }

        float3 forward = cam_forward_from_angles(cam_yaw, cam_pitch);

        f32 wasdF = 0.0f;   // +forward = backwards (forward = pivot->eye)
        f32 wasdR = 0.0f;
        if cam_w_down { wasdF -= 1.0f; }
        if cam_s_down { wasdF += 1.0f; }
        if cam_d_down { wasdR += 1.0f; }
        if cam_a_down { wasdR -= 1.0f; }

        float3 eye = eyeBefore;
        if wasdF != 0.0f || wasdR != 0.0f {
            float3 r = float3{forward.z, 0.0f, -forward.x};   // cross(worldUp, forward)
            f32 rlen = sqrtf(r.x * r.x + r.z * r.z);
            if rlen > 0.000001f { r.x /= rlen; r.z /= rlen; }
            f32 step = cam_speed * dt;
            eye.x += forward.x * wasdF * step + r.x * wasdR * step;
            eye.y += forward.y * wasdF * step;
            eye.z += forward.z * wasdF * step + r.z * wasdR * step;
        }

        // back-derive pivot so returning to orbit keeps the look direction
        cam_pivot = float3{eye.x - forward.x * cam_radius,
                           eye.y - forward.y * cam_radius,
                           eye.z - forward.z * cam_radius};

        if cam_speed_scroll != 0.0f {
            cam_speed += cam_speed_scroll * CAM_FLY_SPEED_STEP;
            cam_speed = cam_clamp(cam_speed, CAM_MIN_SPEED, CAM_MAX_SPEED);
        }
    } else {
        if cam_orbit_dx != 0.0f || cam_orbit_dy != 0.0f {
            cam_yaw -= cam_orbit_dx * CAM_ORBIT_SENS;
            cam_pitch -= cam_orbit_dy * CAM_ORBIT_SENS;
            cam_pitch = cam_clamp(cam_pitch, -cam_pitch_lim(), cam_pitch_lim());
        }

        if cam_pan_dx != 0.0f || cam_pan_dy != 0.0f {
            // pivot moves along the previous frame's right/up, the
            // screen that was dragged against
            f32 panScale = CAM_PAN_SENS * cam_radius;
            cam_pivot.x -= cam_right.x * cam_pan_dx * panScale;
            cam_pivot.y -= cam_right.y * cam_pan_dx * panScale;
            cam_pivot.z -= cam_right.z * cam_pan_dx * panScale;
            cam_pivot.x += cam_up.x * cam_pan_dy * panScale;
            cam_pivot.y += cam_up.y * cam_pan_dy * panScale;
            cam_pivot.z += cam_up.z * cam_pan_dy * panScale;
        }

        if cam_radial_dy != 0.0f {
            // drag down -> zoom in
            cam_radius -= CAM_RADIAL_ZOOM_SENS * cam_radial_dy;
            cam_radius = cam_clamp(cam_radius, CAM_MIN_DIST, CAM_VIEW_DISTANCE);
        }

        if cam_scroll != 0.0f {
            cam_radius *= powf(CAM_ZOOM_STEP, cam_scroll);
            cam_radius = cam_clamp(cam_radius, CAM_MIN_DIST, CAM_VIEW_DISTANCE);
        }
    }

    // keep yaw bounded across long sessions
    while cam_yaw > PI_F { cam_yaw -= 2.0f * PI_F; }
    while cam_yaw <= -PI_F { cam_yaw += 2.0f * PI_F; }

    cam_orbit_dx = 0.0f;
    cam_orbit_dy = 0.0f;
    cam_pan_dx = 0.0f;
    cam_pan_dy = 0.0f;
    cam_radial_dy = 0.0f;
    cam_scroll = 0.0f;
    cam_speed_scroll = 0.0f;

    cam_rebuild_basis();
}

// View matrix from the cached basis (column-major; same shape as
// lib look_at, with forward = pivot->eye = +view-Z).
// upstream RebuildBasisAndView sets m_position = b3Vec3_zero and builds
// the view from that: the eye IS the draw origin, so the view carries no
// translation and stays exact far from the origin. Geometry arrives
// already shifted against the same origin.
float4x4 cam_view_matrix() {
    return float4x4{
        cam_right.x, cam_up.x, cam_forward.x, 0.0f,
        cam_right.y, cam_up.y, cam_forward.y, 0.0f,
        cam_right.z, cam_up.z, cam_forward.z, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    };
}

// upstream Camera::Frame: fit the camera distance to an AABB keeping
// yaw/pitch. Frame Camera menu item and the F key use world bounds.
void cam_frame() {
    b3AABB aabb = b3World_GetBounds(g_world);
    float3 center = float3{0.5f * (aabb.lowerBound.x + aabb.upperBound.x),
                           0.5f * (aabb.lowerBound.y + aabb.upperBound.y),
                           0.5f * (aabb.lowerBound.z + aabb.upperBound.z)};
    f32 ex = 0.5f * (aabb.upperBound.x - aabb.lowerBound.x);
    f32 ey = 0.5f * (aabb.upperBound.y - aabb.lowerBound.y);
    f32 ez = 0.5f * (aabb.upperBound.z - aabb.lowerBound.z);
    f32 r = sqrtf(ex * ex + ey * ey + ez * ez);
    cam_pivot = center;
    if r < 0.000001f {
        cam_rebuild_basis();
        return;
    }
    f32 w = sapp_widthf();
    f32 h = sapp_heightf();
    f32 aspect = h > 0.0f ? w / h : 1.0f;
    f32 invTan = 1.0f / tanf(0.5f * 60.0f * PI_F / 180.0f);
    f32 distV = r * invTan;
    f32 distH = r * invTan / aspect;
    f32 d = distV > distH ? distV : distH;
    d *= 0.75f;
    cam_set_orbit(cam_yaw, cam_pitch, d);
}
