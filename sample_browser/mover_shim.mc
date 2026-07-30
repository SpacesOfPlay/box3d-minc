import box3d;
import camera;
import sample;
import debug_adapter;

// Host for the transpiled CharacterMover (lib/box3d_mover.mc).
//
// The module was transpiled against ext/box3d/samples/mover_host.h, a
// stub naming the things mover.cpp reads through its `Sample*` back
// reference and the draw / input calls it makes. Nothing there was
// emitted — this file binds those names to the browser's own camera,
// world and debug channel.
//
// Upstream offsets debug drawing by a camera origin for
// far-from-origin precision. This port draws in world space, so
// DrawOrigin is the origin and SetDrawOrigin is a sink.

struct Vec4 { f32 x; f32 y; f32 z; f32 w; }
struct SampleContext { f32 hertz; }
struct Camera { b3Pos m_pivot; bool m_thirdPerson; }
struct Sample { SampleContext* m_context; Camera* m_camera; b3WorldId m_worldId; }

// The mover keeps the `Sample*` from Initialize, so these mirror the
// browser's globals and must be refreshed before every Step — the
// mover gates its keyboard reads on m_thirdPerson.
Camera g_mover_camera;
SampleContext g_mover_context;
Sample g_mover_sample;

void mover_sync() {
    g_mover_context.hertz = g_hertz;
    g_mover_camera.m_thirdPerson = cam_third_person;
    g_mover_camera.m_pivot = b3Pos{cam_pivot.x, cam_pivot.y, cam_pivot.z};
    g_mover_sample.m_context = &g_mover_context;
    g_mover_sample.m_camera = &g_mover_camera;
    g_mover_sample.m_worldId = g_world;
}

Sample* mover_sample() {
    mover_sync();
    return &g_mover_sample;
}

b3Vec3 Camera_GetForward(Camera* self) {
    ignore self;
    return b3Vec3{cam_forward.x, cam_forward.y, cam_forward.z};
}

b3Vec3 Camera_GetRight(Camera* self) {
    ignore self;
    return b3Vec3{cam_right.x, cam_right.y, cam_right.z};
}

b3Pos Camera_DrawOrigin(Camera* self) {
    ignore self;
    return b3Pos{0.0f, 0.0f, 0.0f};
}

// Third person follows the mover: it writes m_pivot then calls this.
void Camera_UpdateTransform(Camera* self) {
    cam_pivot = float3{self.m_pivot.x, self.m_pivot.y, self.m_pivot.z};
    cam_rebuild_basis();
}

Vec4 MakeColor(b3HexColor hexColor) {
    float4 v = make_color(hexColor);
    return Vec4{v.x, v.y, v.z, v.w};
}

void SetDrawOrigin(b3Pos origin) { ignore origin; }

void DrawLine(b3Pos a, b3Pos b, Vec4 color) {
    dbg_line_rgba(a.x, a.y, a.z, b.x, b.y, b.z,
                  color.x, color.y, color.z, color.w);
}

void DrawPoint(b3Pos p, f32 size, Vec4 color) {
    f32 k = 0.002f * size * cam_radius;
    dbg_line_rgba(p.x - k, p.y, p.z, p.x + k, p.y, p.z,
                  color.x, color.y, color.z, color.w);
    dbg_line_rgba(p.x, p.y - k, p.z, p.x, p.y + k, p.z,
                  color.x, color.y, color.z, color.w);
    dbg_line_rgba(p.x, p.y, p.z - k, p.x, p.y, p.z + k,
                  color.x, color.y, color.z, color.w);
}

void DrawSolidCapsule(b3WorldTransform transform, b3Capsule capsule, Vec4 color) {
    dbg_solid_capsule(transform, capsule, float4{color.x, color.y, color.z, color.w});
}

bool IsKeyDown(i32 key) { return is_key_down(key); }

// The operator overloads the stub declares, onto box3d's C API.
b3Vec3 op_sub_b3Vec3(b3Vec3 a) { return b3Neg(a); }
b3Vec3 op_add_b3Vec3_b3Vec3(b3Vec3 a, b3Vec3 b) { return b3Add(a, b); }
b3Vec3 op_sub_b3Vec3_b3Vec3(b3Vec3 a, b3Vec3 b) { return b3Sub(a, b); }
b3Vec3 op_mul_float_b3Vec3(f32 s, b3Vec3 a) { return b3MulSV(s, a); }
b3Vec3 op_mul_b3Vec3_float(b3Vec3 a, f32 s) { return b3MulSV(s, a); }
b3Vec3 op_mul_b3Vec3_b3Vec3(b3Vec3 a, b3Vec3 b) { return b3Mul(a, b); }
b3Pos op_add_b3Pos_b3Vec3(b3Pos a, b3Vec3 b) { return b3OffsetPos(a, b); }
b3Pos op_sub_b3Pos_b3Vec3(b3Pos a, b3Vec3 b) { return b3OffsetPos(a, b3Neg(b)); }
b3Vec3 op_sub_b3Pos_b3Pos(b3Pos a, b3Pos b) { return b3SubPos(a, b); }
