// Rendering: the shaders, the sphere and capsule meshes, and the shadow
// cascade fit.

// --- shaders --------------------------------------------------------

import box3d;
import sokol_all;
import sokol_imgui;
import math;
import linear;
import str;
import camera;
import gui;
import sample;
import sample_benchmark;
import sample_bodies;
import sample_continuous;
import sample_robustness;
import sample_stacking;
import sample_joint;
import sample_compound;
import sample_world;

struct PerDraw {
    float4x4 mvp;
    float4x4 model;
    float4 tint;
    float4 params;   // x: ground-grid cell size (0 = off)
}

// Per-pass shadow data (uniform slot 1). Each cascade matrix folds in
// the clip-space-to-texture-space remap, so the fragment shader divides
// by w and uses xy as the lookup, z as the reference depth.
const i32 SHADOW_CASCADES = 3;

struct ShadowUni {
    float4x4[SHADOW_CASCADES] cascade;
    float4 cascadeFar;   // xyz: far view depth of each cascade
    float4 camPos;       // xyz: camera position, zero in the relative frame
    float4 viewDir;      // xyz: camera looking direction
    float4 shParams;     // x: PCF tap offset in UV, y: unshadowed floor
    // upstream geom.glsl grid_offset: xy = draw origin wrapped to the grid
    // period, for the lines; zw = the full origin x/z, for the axes. wpos
    // is eye-relative, so the lines add a small wrapped offset and stay
    // precise, while the axes reconstruct absolute x/z where they must.
    float4 gridOffset;
}

struct VsOut {
    float4 pos;
    float3 normal;
    float4 tint;
    float3 wpos;
    f32 cell;
}

struct EdgeOut {
    float4 pos;
    float4 tint;
}

// Edge overlay, a port of upstream shaders/shapes/edge.glsl.
//
// Upstream draws no vertex buffer at all: six vertices per edge indexed
// off gl_VertexIndex, endpoints read from a storage buffer. Storage
// buffers need GLES 3.1 and our wasm target is WebGL2, so the six
// corners come from a shared per-vertex buffer and the endpoints from a
// per-instance one — 24 bytes an edge, where carrying both endpoints on
// all six vertices cost 168. The screen-space expansion, the SDF
// coverage and the constants below are upstream's.
//
// Corner layout, matching upstream:
//   0 -> A,-1   1 -> B,-1   2 -> B,+1
//   3 -> A,-1   4 -> B,+1   5 -> A,+1
struct EdgeVsOut {
    float4 pos;
    float4 tint;
    @noperspective f32 distFromAxisPx;   // long edges thin in the middle
    @flat f32 halfWidthPx;
}

// The six quad corners, shared by every edge and stepped per vertex
// while the endpoints step per instance.
f32[6] g_edge_corners = { 0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f };
sg_buffer g_edge_corner_vbuf;

const f32 EDGE_THICKNESS_PX = 1.5f;   // upstream edges.c thicknessPx
// The quad is expanded past the true half-width so the coverage falloff
// has geometry to ramp down inside. fwidth is an L1 norm and peaks at
// sqrt(2) on a 45 degree line, so the pad must be >= ~1.42.
const f32 EDGE_AA_PAD_PX = 1.5f;

@shader vertex
VsOut b3_vs(
    @attr(0) float3 position,
    @attr(1) float3 normal,
    @uniform(0) PerDraw d
) {
    VsOut o;
    o.pos = mul(d.mvp, float4{position.x, position.y, position.z, 1.0f});
    // upstream shapes/geom.glsl: the world normal is R * (normal /
    // scale). model carries R*S here, so divide by scale twice.
    // params.yzw is 1/scale^2 per axis.
    float4 n4 = mul(d.model, float4{normal.x * d.params.y,
                                    normal.y * d.params.z,
                                    normal.z * d.params.w, 0.0f});
    o.normal = normalize(float3{n4.x, n4.y, n4.z});
    o.tint = d.tint;
    float4 w = mul(d.model, float4{position.x, position.y, position.z, 1.0f});
    o.wpos = float3{w.x, w.y, w.z};
    o.cell = d.params.x;
    return o;
}

@shader fragment
float4 b3_fs(
    VsOut input,
    @uniform(1) ShadowUni sh,
    @texture(0) Texture2DArray shadowMap,
    @sampler(0) Sampler shadowSmp
) {
    // sun dir: upstream renderer's normalize(0.5, 0.8, 0.4)
    float3 nn = normalize(input.normal);
    f32 nl = max(dot(nn, float3{0.4880f, 0.7807f, 0.3904f}), 0.0f);

    // Shadow lookup: pick the cascade by view depth, sample that layer.
    // Anything outside every cascade stays lit.
    float3 rel = float3{input.wpos.x - sh.camPos.x,
                        input.wpos.y - sh.camPos.y,
                        input.wpos.z - sh.camPos.z};
    f32 viewZ = dot(rel, float3{sh.viewDir.x, sh.viewDir.y, sh.viewDir.z});
    i32 c = 0;
    if viewZ > sh.cascadeFar.x { c = 1; }
    if viewZ > sh.cascadeFar.y { c = 2; }

    f32 shadow = 1.0f;
    float4 wp = float4{input.wpos.x, input.wpos.y, input.wpos.z, 1.0f};
    float4 lc = mul(sh.cascade[c], wp);
    if lc.w > 0.0f {
        float3 sp = float3{lc.x / lc.w, lc.y / lc.w, lc.z / lc.w};
        if sp.x > 0.0f && sp.x < 1.0f && sp.y > 0.0f && sp.y < 1.0f
           && sp.z > 0.0f && sp.z < 1.0f {
            // Slope-scaled bias in shadow-map depth units:
            // tan(angle to the sun) = sqrt(1 - n.l^2) / n.l.
            f32 ndl = clamp(nl, 0.001f, 1.0f);
            f32 tanAngle = sqrt(1.0f - ndl * ndl) / ndl;
            f32 zbias = clamp(0.001f * tanAngle, 0.0005f, 0.05f);
            f32 refz = sp.z - zbias;
            f32 tap = sh.shParams.x;
            f32 layer = cast(f32, c);
            f32 sum = 0.0f;
            for i32 j = 0; j < 3; j++ {
                for i32 i = 0; i < 3; i++ {
                    float3 st = float3{sp.x + (cast(f32, i) - 1.0f) * tap,
                                       sp.y + (cast(f32, j) - 1.0f) * tap,
                                       layer};
                    sum += sample_cmp(shadowMap, shadowSmp, st, refz);
                }
            }
            // shParams.y keeps shadowed faces from going fully black
            shadow = sh.shParams.y + (1.0f - sh.shParams.y) * (sum / 9.0f);
        }
    }

    f32 lit = 0.35f + 0.65f * nl * shadow;
    float3 baseColor = float3{input.tint.x, input.tint.y, input.tint.z};
    // upstream common/pbr.glsl proceduralGrid: minor cells at 0.70x,
    // 10x major cells at 0.35x, red +X / blue +Z axes, all
    // fwidth-antialiased; the blend fades as the surface tilts
    if input.cell > 0.0f {
        // upstream geom.glsl: line_xz = world_pos.xz + grid_offset.xy,
        // axis_xz = world_pos.xz + grid_offset.zw. wpos is eye-relative,
        // so the lines ride a wrapped offset and never lose resolution,
        // while the axes reconstruct true x/z.
        f32 lineX = input.wpos.x + sh.gridOffset.x;
        f32 lineZ = input.wpos.z + sh.gridOffset.y;
        f32 axisXc = input.wpos.x + sh.gridOffset.z;
        f32 axisZc = input.wpos.z + sh.gridOffset.w;
        f32 cmx = lineX / input.cell;
        f32 cmz = lineZ / input.cell;
        f32 gmx = abs(fract(cmx - 0.5f) - 0.5f) / max(fwidth(cmx), 0.000001f);
        f32 gmz = abs(fract(cmz - 0.5f) - 0.5f) / max(fwidth(cmz), 0.000001f);
        f32 lineMinor = 1.0f - clamp(min(gmx, gmz), 0.0f, 1.0f);

        f32 cMx = lineX / (input.cell * 10.0f);
        f32 cMz = lineZ / (input.cell * 10.0f);
        f32 gMx = abs(fract(cMx - 0.5f) - 0.5f) / max(fwidth(cMx), 0.000001f);
        f32 gMz = abs(fract(cMz - 0.5f) - 0.5f) / max(fwidth(cMz), 0.000001f);
        f32 lineMajor = 1.0f - clamp(min(gMx, gMz), 0.0f, 1.0f);

        float3 gridColor = mix(baseColor, baseColor * 0.70f, lineMinor);
        gridColor = mix(gridColor, baseColor * 0.35f, lineMajor);

        // colored axes on the positive halves, pixel-wide
        f32 axisX = 1.0f - clamp(abs(axisZc) / max(fwidth(axisZc), 0.000001f), 0.0f, 1.0f);
        if axisXc < 0.0f { axisX = 0.0f; }
        f32 axisZ = 1.0f - clamp(abs(axisXc) / max(fwidth(axisXc), 0.000001f), 0.0f, 1.0f);
        if axisZc < 0.0f { axisZ = 0.0f; }
        gridColor = mix(gridColor, float3{1.0f, 0.0f, 0.0f}, axisX);
        gridColor = mix(gridColor, float3{0.0f, 0.0f, 1.0f}, axisZ);

        f32 gridBlend = pow(clamp(nn.y, 0.0f, 1.0f), 8.0f);
        baseColor = mix(baseColor, gridColor, gridBlend);
    }
    return float4{baseColor.x * lit, baseColor.y * lit, baseColor.z * lit, 1.0f};
}

// Debug channel. The vertex layout is the two float3s the other meshes
// use, with the second slot carrying colour instead of a normal.
struct DbgUni {
    float4x4 viewproj;
}

struct DbgOut {
    float4 pos;
    float4 color;
}

@shader vertex
DbgOut b3_dbg_vs(
    @attr(0) float3 position,
    @attr(1) float4 color,
    @uniform(0) DbgUni u
) {
    DbgOut o;
    o.pos = mul(u.viewproj, float4{position.x, position.y, position.z, 1.0f});
    o.color = color;
    return o;
}

@shader fragment
float4 b3_dbg_fs(DbgOut input) {
    return input.color;
}

// Capsule. Vertices store the unit direction in `normal` and that
// direction offset by the cap sign along x in `position`, so a point
// rebuilds as normal*radius + capSign*halfLength. Sized here rather
// than by a model-matrix scale, which squashes the caps. params.y is
// the half-length, params.z the radius.
@shader vertex
VsOut b3_cap_vs(
    @attr(0) float3 position,
    @attr(1) float3 normal,
    @uniform(0) PerDraw d
) {
    f32 halfLen = d.params.y;
    f32 radius = d.params.z;
    f32 capOff = position.x >= 0.0f ? halfLen : 0.0f - halfLen;
    float3 lp = float3{normal.x * radius + capOff, normal.y * radius, normal.z * radius};
    VsOut o;
    o.pos = mul(d.mvp, float4{lp.x, lp.y, lp.z, 1.0f});
    float4 n4 = mul(d.model, float4{normal.x, normal.y, normal.z, 0.0f});
    o.normal = normalize(float3{n4.x, n4.y, n4.z});
    o.tint = d.tint;
    float4 w = mul(d.model, float4{lp.x, lp.y, lp.z, 1.0f});
    o.wpos = float3{w.x, w.y, w.z};
    o.cell = 0.0f;
    return o;
}

// Shadow pass: depth only. PerDraw.mvp carries the light
// view-projection times the model matrix.
@shader vertex
EdgeOut b3_depth_vs(
    @attr(0) float3 position,
    @attr(1) float3 normal,
    @uniform(0) PerDraw d
) {
    EdgeOut o;
    o.pos = mul(d.mvp, float4{position.x, position.y, position.z, 1.0f});
    o.tint = d.tint;
    return o;
}

// Capsule depth: same in-shader sizing as b3_cap_vs.
@shader vertex
EdgeOut b3_cap_depth_vs(
    @attr(0) float3 position,
    @attr(1) float3 normal,
    @uniform(0) PerDraw d
) {
    f32 halfLen = d.params.y;
    f32 radius = d.params.z;
    f32 capOff = position.x >= 0.0f ? halfLen : 0.0f - halfLen;
    float3 lp = float3{normal.x * radius + capOff, normal.y * radius, normal.z * radius};
    EdgeOut o;
    o.pos = mul(d.mvp, float4{lp.x, lp.y, lp.z, 1.0f});
    return o;
}

@shader fragment
float4 b3_depth_fs(EdgeOut input) {
    return float4{0.0f, 0.0f, 0.0f, 1.0f};
}

// hull-edge overlay: thin translucent lines over the solid boxes
// (upstream gfx/edges.c hullEdgeColor 0.5,0.5,0.5,0.5)
// Edge overlay. Upstream (shaders/shapes/edge.glsl) breaks the depth tie
// with a clip-space nudge toward the camera rather than by moving the
// geometry: `clip.z += zBias * clip.w`, reverse-Z, so toward the camera
// is +z there and -z here. A geometric nudge cannot work for a flat
// mesh — scaling a plane at y = 0 leaves it at y = 0, so its edges stay
// exactly coplanar and lose the tie.
const f32 EDGE_Z_BIAS = 0.000001f;   // upstream edges.c zBias

@shader vertex
EdgeVsOut b3_edge_vs(
    @attr(0) f32 cornerIndex,       // per vertex, 0..5, shared by every edge
    @attr(1) float3 endpointA,      // per instance
    @attr(2) float3 endpointB,
    @uniform(0) PerDraw d
) {
    i32 corner = cast(i32, cornerIndex);
    bool isB = corner == 1 || corner == 2 || corner == 4;
    f32 side = (corner == 2 || corner == 4 || corner == 5) ? 1.0f : -1.0f;

    float4 clipA = mul(d.mvp, float4{endpointA.x, endpointA.y, endpointA.z, 1.0f});
    float4 clipB = mul(d.mvp, float4{endpointB.x, endpointB.y, endpointB.z, 1.0f});
    float4 clipEnd = isB ? clipB : clipA;

    // pixel coords of both endpoints, NDC [-1,1] -> [0, viewport]
    float2 viewport = float2{d.params.x, d.params.y};
    float2 ndcA = float2{clipA.x / clipA.w, clipA.y / clipA.w};
    float2 ndcB = float2{clipB.x / clipB.w, clipB.y / clipB.w};
    float2 sa = float2{(ndcA.x * 0.5f + 0.5f) * viewport.x, (ndcA.y * 0.5f + 0.5f) * viewport.y};
    float2 sb = float2{(ndcB.x * 0.5f + 0.5f) * viewport.x, (ndcB.y * 0.5f + 0.5f) * viewport.y};

    float2 dirPx = float2{sb.x - sa.x, sb.y - sa.y};
    f32 lenPx = sqrt(dirPx.x * dirPx.x + dirPx.y * dirPx.y);
    float2 dirN = lenPx > 0.000001f ? float2{dirPx.x / lenPx, dirPx.y / lenPx}
                                    : float2{1.0f, 0.0f};
    float2 perpN = float2{0.0f - dirN.y, dirN.x};

    // floor the half-width at 0.5 px so thin requests do not vanish
    f32 halfWidthPx = max(EDGE_THICKNESS_PX * 0.5f, 0.5f);
    f32 quadHalfPx = halfWidthPx + EDGE_AA_PAD_PX;

    float2 offsetPx = float2{perpN.x * side * quadHalfPx, perpN.y * side * quadHalfPx};
    float2 offsetNdc = float2{offsetPx.x * 2.0f / viewport.x, offsetPx.y * 2.0f / viewport.y};

    EdgeVsOut o;
    o.pos = clipEnd;
    o.pos.x += offsetNdc.x * clipEnd.w;
    o.pos.y += offsetNdc.y * clipEnd.w;
    // upstream nudges +z toward the camera in reverse-Z; ours is normal-Z
    o.pos.z -= d.params.w * o.pos.w;
    o.tint = d.tint;
    o.distFromAxisPx = side * quadHalfPx;
    o.halfWidthPx = halfWidthPx;
    return o;
}

@shader fragment
float4 b3_edge_fs(EdgeVsOut input) {
    // Coverage from the capsule SDF: 1 -> 0 over a 2*aa band straddling
    // the true edge. fwidth of the raw signed distance, not the folded
    // one, to avoid the kink at the abs.
    f32 dist = abs(input.distFromAxisPx);
    f32 aa = fwidth(input.distFromAxisPx);
    f32 coverage = 1.0f - smoothstep(input.halfWidthPx - aa, input.halfWidthPx + aa, dist);
    // premultiplied, to pair with SRC_ONE / ONE_MINUS_SRC_ALPHA
    return float4{input.tint.x * input.tint.w * coverage,
                  input.tint.y * input.tint.w * coverage,
                  input.tint.z * input.tint.w * coverage,
                  input.tint.w * coverage};
}

// Sky backdrop: upstream ibl/sky.glsl + common/preetham.glsl. Upstream
// draws its fullscreen triangle at the reverse-Z far plane; this
// renderer is normal-Z, so the triangle sits at z = 1 with LESS_EQUAL
// against the cleared depth. The view ray comes from the camera basis
// rather than an inverse view-projection. Turbidity 2.2 and the 0.06
// luminance scale are upstream's.

struct SkyPass {
    float4 fwd;      // camera forward (pivot -> eye); view dir = -fwd
    float4 right;    // xyz: camera right, w: tanHalfFov * aspect
    float4 up;       // xyz: camera up,    w: tanHalfFov
    float4 sun;      // xyz: world dir to sun
}

struct SkyOut {
    float4 pos;
    float3 ray;
    float3 sun;
}

@shader vertex
SkyOut b3_sky_vs(
    @attr(0) float3 position,
    @uniform(0) SkyPass sp
) {
    SkyOut o;
    o.pos = float4{position.x, position.y, 1.0f, 1.0f};
    o.ray = float3{
        0.0f - sp.fwd.x + position.x * sp.right.w * sp.right.x + position.y * sp.up.w * sp.up.x,
        0.0f - sp.fwd.y + position.x * sp.right.w * sp.right.y + position.y * sp.up.w * sp.up.y,
        0.0f - sp.fwd.z + position.x * sp.right.w * sp.right.z + position.y * sp.up.w * sp.up.z};
    o.sun = float3{sp.sun.x, sp.sun.y, sp.sun.z};
    return o;
}

@shader fragment
float4 b3_sky_fs(SkyOut input) {
    float3 v = normalize(input.ray);
    float3 sun = normalize(input.sun);

    f32 sun_y = clamp(sun.y, 0.0f, 1.0f);
    float3 vc = normalize(float3{v.x, max(v.y, 0.0f) + 0.01f, v.z});
    f32 cos_theta = max(vc.y, 0.0f);
    f32 cos_gamma = clamp(dot(sun, vc), 0.0f - 1.0f, 1.0f);
    f32 gamma = acos(cos_gamma);
    f32 sun_theta = acos(sun_y);

    // Perez coefficients at turbidity T (Preetham table 2)
    f32 T = 2.2f;
    f32 T2 = T * T;
    f32 A_Y = 0.1787f * T - 1.4630f;
    f32 B_Y = 0.0f - 0.3554f * T + 0.4275f;
    f32 C_Y = 0.0f - 0.0227f * T + 5.3251f;
    f32 D_Y = 0.1206f * T - 2.5771f;
    f32 E_Y = 0.0f - 0.0670f * T + 0.3703f;
    f32 A_x = 0.0f - 0.0193f * T - 0.2592f;
    f32 B_x = 0.0f - 0.0665f * T + 0.0008f;
    f32 C_x = 0.0f - 0.0004f * T + 0.2125f;
    f32 D_x = 0.0f - 0.0641f * T - 0.8989f;
    f32 E_x = 0.0f - 0.0033f * T + 0.0452f;
    f32 A_y = 0.0f - 0.0167f * T - 0.2608f;
    f32 B_y = 0.0f - 0.0950f * T + 0.0092f;
    f32 C_y = 0.0f - 0.0079f * T + 0.2102f;
    f32 D_y = 0.0f - 0.0441f * T - 1.6537f;
    f32 E_y = 0.0f - 0.0109f * T + 0.0529f;

    // zenith chromaticities and luminance (Preetham appendix)
    f32 ts = sun_theta;
    f32 ts2 = ts * ts;
    f32 ts3 = ts2 * ts;
    f32 x_z = (0.00166f * ts3 - 0.00375f * ts2 + 0.00209f * ts) * T2
        + (0.0f - 0.02903f * ts3 + 0.06377f * ts2 - 0.03202f * ts + 0.00394f) * T
        + (0.11693f * ts3 - 0.21196f * ts2 + 0.06052f * ts + 0.25886f);
    f32 y_z = (0.00275f * ts3 - 0.00610f * ts2 + 0.00317f * ts) * T2
        + (0.0f - 0.04214f * ts3 + 0.08970f * ts2 - 0.04153f * ts + 0.00516f) * T
        + (0.15346f * ts3 - 0.26756f * ts2 + 0.06670f * ts + 0.26688f);
    f32 chi = (4.0f / 9.0f - T / 120.0f) * (3.14159265f - 2.0f * sun_theta);
    f32 Y_z = (4.0453f * T - 4.9710f) * tan(chi) - 0.2155f * T + 2.4192f;

    // Perez F(theta, gamma) for the view and zenith directions
    f32 ct = max(cos_theta, 0.01f);
    f32 cg2 = cos_gamma * cos_gamma;
    f32 czg2 = sun_y * sun_y;
    f32 pY_v = (1.0f + A_Y * exp(B_Y / ct)) * (1.0f + C_Y * exp(D_Y * gamma) + E_Y * cg2);
    f32 px_v = (1.0f + A_x * exp(B_x / ct)) * (1.0f + C_x * exp(D_x * gamma) + E_x * cg2);
    f32 py_v = (1.0f + A_y * exp(B_y / ct)) * (1.0f + C_y * exp(D_y * gamma) + E_y * cg2);
    f32 pY_z = (1.0f + A_Y * exp(B_Y)) * (1.0f + C_Y * exp(D_Y * sun_theta) + E_Y * czg2);
    f32 px_z = (1.0f + A_x * exp(B_x)) * (1.0f + C_x * exp(D_x * sun_theta) + E_x * czg2);
    f32 py_z = (1.0f + A_y * exp(B_y)) * (1.0f + C_y * exp(D_y * sun_theta) + E_y * czg2);

    f32 Y = max(Y_z * pY_v / max(pY_z, 0.00001f), 0.0f);
    f32 x = x_z * px_v / max(px_z, 0.00001f);
    f32 y = y_z * py_v / max(py_z, 0.00001f);

    // xyY -> XYZ -> linear sRGB
    f32 yy = max(y, 0.00001f);
    float3 XYZ = float3{Y * x / yy, Y, Y * (1.0f - x - y) / yy};
    f32 r = dot(XYZ, float3{3.2404542f, 0.0f - 1.5371385f, 0.0f - 0.4985314f});
    f32 g = dot(XYZ, float3{0.0f - 0.9692660f, 1.8760108f, 0.0415560f});
    f32 b = dot(XYZ, float3{0.0556434f, 0.0f - 0.2040259f, 1.0572252f});

    // upstream tone pipeline (post/tonemap.glsl): luminance scale,
    // exposure pow(2, -2.5), then Minimal AgX (inset matrix, log2
    // encode, sigmoid fit, outset matrix). Saturation look is identity.
    f32 ev = 0.176777f;   // pow(2, -2.5)
    r = max(r, 0.0f) * 0.06f * ev;
    g = max(g, 0.0f) * 0.06f * ev;
    b = max(b, 0.0f) * 0.06f * ev;

    // AgX inset (linear sRGB -> AgX primaries)
    f32 ax = 0.8424791f * r + 0.0784336f * g + 0.0792237f * b;
    f32 ay = 0.0423282f * r + 0.8784686f * g + 0.0791661f * b;
    f32 az = 0.0423757f * r + 0.0784336f * g + 0.8791430f * b;

    // log2 encode into [0,1] over [-12.47393, 4.026069] EV
    f32 spanInv = 1.0f / 16.500001f;
    ax = (clamp(log2(max(ax, 0.0000000001f)), 0.0f - 12.47393f, 4.026069f) + 12.47393f) * spanInv;
    ay = (clamp(log2(max(ay, 0.0000000001f)), 0.0f - 12.47393f, 4.026069f) + 12.47393f) * spanInv;
    az = (clamp(log2(max(az, 0.0000000001f)), 0.0f - 12.47393f, 4.026069f) + 12.47393f) * spanInv;

    // 6th-order AgX sigmoid fit (Sobotka), per channel
    f32 x2 = ax * ax;
    f32 x4 = x2 * x2;
    ax = 15.5f * x4 * x2 - 40.14f * x4 * ax + 31.96f * x4 - 6.868f * x2 * ax + 0.4298f * x2 + 0.1191f * ax - 0.00232f;
    x2 = ay * ay;
    x4 = x2 * x2;
    ay = 15.5f * x4 * x2 - 40.14f * x4 * ay + 31.96f * x4 - 6.868f * x2 * ay + 0.4298f * x2 + 0.1191f * ay - 0.00232f;
    x2 = az * az;
    x4 = x2 * x2;
    az = 15.5f * x4 * x2 - 40.14f * x4 * az + 31.96f * x4 - 6.868f * x2 * az + 0.4298f * x2 + 0.1191f * az - 0.00232f;

    // AgX outset -> display-encoded sRGB, written verbatim
    r = 1.1968790f * ax - 0.0980209f * ay - 0.0990297f * az;
    g = 0.0f - 0.0528969f * ax + 1.1519031f * ay - 0.0989612f * az;
    b = 0.0f - 0.0529716f * ax - 0.0980435f * ay + 1.1510737f * az;
    return float4{clamp(r, 0.0f, 1.0f), clamp(g, 0.0f, 1.0f), clamp(b, 0.0f, 1.0f), 1.0f};
}

// --- meshes ---------------------------------------------------------

// Unit cube, per-face normals: 24 verts x (pos3 + normal3), 36 indices.
i32[36] g_cube_idx;
// face outlines: 4 lines per face (shared cube edges draw twice)
i32[48] g_cube_line_idx;
// Icosphere (1 subdivision, flat-shaded): 80 faces x 3 verts.
f32[240 * 6] g_sph_verts;
i32 g_sph_nverts;

f32[36] g_ico_v;     // 12 base verts x 3

void sph_emit_tri(f32 ax, f32 ay, f32 az, f32 bx, f32 by, f32 bz, f32 cx, f32 cy, f32 cz) {
    f32 nx = (ax + bx + cx) / 3.0f;
    f32 ny = (ay + by + cy) / 3.0f;
    f32 nz = (az + bz + cz) / 3.0f;
    f32 nl = sqrtf(nx * nx + ny * ny + nz * nz);
    nx /= nl; ny /= nl; nz /= nl;
    i32 v = g_sph_nverts * 6;
    g_sph_verts[v..] = { ax, ay, az, nx, ny, nz,
                         bx, by, bz, nx, ny, nz,
                         cx, cy, cz, nx, ny, nz };
    g_sph_nverts += 3;
}

void sph_norm(f32* x, f32* y, f32* z) {
    f32 l = sqrtf(*x * *x + *y * *y + *z * *z);
    *x /= l; *y /= l; *z /= l;
}

void sph_subdiv(f32 ax, f32 ay, f32 az, f32 bx, f32 by, f32 bz, f32 cx, f32 cy, f32 cz) {
    f32 abx = (ax + bx) * 0.5f; f32 aby = (ay + by) * 0.5f; f32 abz = (az + bz) * 0.5f;
    f32 bcx = (bx + cx) * 0.5f; f32 bcy = (by + cy) * 0.5f; f32 bcz = (bz + cz) * 0.5f;
    f32 cax = (cx + ax) * 0.5f; f32 cay = (cy + ay) * 0.5f; f32 caz = (cz + az) * 0.5f;
    sph_norm(&abx, &aby, &abz);
    sph_norm(&bcx, &bcy, &bcz);
    sph_norm(&cax, &cay, &caz);
    sph_emit_tri(ax, ay, az, abx, aby, abz, cax, cay, caz);
    sph_emit_tri(bx, by, bz, bcx, bcy, bcz, abx, aby, abz);
    sph_emit_tri(cx, cy, cz, cax, cay, caz, bcx, bcy, bcz);
    sph_emit_tri(abx, aby, abz, bcx, bcy, bcz, cax, cay, caz);
}

void build_sphere() {
    f32 t = (1.0f + sqrtf(5.0f)) * 0.5f;
    g_ico_v[0..] = { -1.0f,t,0.0f,  1.0f,t,0.0f,  -1.0f,-t,0.0f,  1.0f,-t,0.0f,
                     0.0f,-1.0f,t,  0.0f,1.0f,t,  0.0f,-1.0f,-t,  0.0f,1.0f,-t,
                     t,0.0f,-1.0f,  t,0.0f,1.0f,  -t,0.0f,-1.0f,  -t,0.0f,1.0f };
    for i32 i = 0; i < 12; i++ {
        f32* px = &g_ico_v[i * 3];
        sph_norm(px, px + 1, px + 2);
    }
    i32[60] faces = { 0,11,5, 0,5,1, 0,1,7, 0,7,10, 0,10,11,
                      1,5,9, 5,11,4, 11,10,2, 10,7,6, 7,1,8,
                      3,9,4, 3,4,2, 3,2,6, 3,6,8, 3,8,9,
                      4,9,5, 2,4,11, 6,2,10, 8,6,7, 9,8,1 };
    g_sph_nverts = 0;
    for i32 f = 0; f < 20; f++ {
        i32 a = faces[f * 3] * 3;
        i32 b = faces[f * 3 + 1] * 3;
        i32 c = faces[f * 3 + 2] * 3;
        sph_subdiv(g_ico_v[a], g_ico_v[a + 1], g_ico_v[a + 2],
                   g_ico_v[b], g_ico_v[b + 1], g_ico_v[b + 2],
                   g_ico_v[c], g_ico_v[c + 1], g_ico_v[c + 2]);
    }
}

// Unit capsule along x: two hemispheres of radius 1 centred at
// x = +/-1 plus the band joining them. b3_cap_vs rebuilds the real
// point from the normal and position together.
const i32 CAP_SEGS = 16;    // longitude steps
const i32 CAP_RINGS = 4;    // rings per hemisphere
f32[900 * 6] g_cap_verts;
i32 g_cap_nverts;

void cap_v(f32 ux, f32 uy, f32 uz, f32 cap) {
    i32 v = g_cap_nverts * 6;
    g_cap_verts[v..] = { ux + cap, uy, uz, ux, uy, uz };
    g_cap_nverts++;
}

void build_capsule() {
    g_cap_nverts = 0;
    f32 dphi = 2.0f * PI_F / cast(f32, CAP_SEGS);
    f32 dth = 0.5f * PI_F / cast(f32, CAP_RINGS);
    for i32 hemi = 0; hemi < 2; hemi++ {
        f32 cap = hemi == 0 ? 1.0f : 0.0f - 1.0f;
        for i32 i = 0; i < CAP_RINGS; i++ {
            f32 t0 = cast(f32, i) * dth;
            f32 t1 = cast(f32, i + 1) * dth;
            f32 x0 = cap * cosf(t0);
            f32 x1 = cap * cosf(t1);
            f32 r0 = sinf(t0);
            f32 r1 = sinf(t1);
            for i32 j = 0; j < CAP_SEGS; j++ {
                f32 p0 = cast(f32, j) * dphi;
                f32 p1 = cast(f32, j + 1) * dphi;
                f32 c0 = cosf(p0); f32 s0 = sinf(p0);
                f32 c1 = cosf(p1); f32 s1 = sinf(p1);
                // A(t0,p0) B(t0,p1) C(t1,p1) D(t1,p0). The +x hemisphere
                // needs the reversed winding of the -x one. The i == 0
                // ring degenerates at the pole and rasterizes nothing.
                if hemi == 0 {
                    cap_v(x0, r0 * c0, r0 * s0, cap);
                    cap_v(x1, r1 * c1, r1 * s1, cap);
                    cap_v(x0, r0 * c1, r0 * s1, cap);

                    cap_v(x0, r0 * c0, r0 * s0, cap);
                    cap_v(x1, r1 * c0, r1 * s0, cap);
                    cap_v(x1, r1 * c1, r1 * s1, cap);
                } else {
                    cap_v(x0, r0 * c0, r0 * s0, cap);
                    cap_v(x0, r0 * c1, r0 * s1, cap);
                    cap_v(x1, r1 * c1, r1 * s1, cap);

                    cap_v(x0, r0 * c0, r0 * s0, cap);
                    cap_v(x1, r1 * c1, r1 * s1, cap);
                    cap_v(x1, r1 * c0, r1 * s0, cap);
                }
            }
        }
    }
    // band between the two equators
    for i32 j = 0; j < CAP_SEGS; j++ {
        f32 p0 = cast(f32, j) * dphi;
        f32 p1 = cast(f32, j + 1) * dphi;
        f32 c0 = cosf(p0); f32 s0 = sinf(p0);
        f32 c1 = cosf(p1); f32 s1 = sinf(p1);
        cap_v(0.0f, c0, s0, 0.0f - 1.0f);
        cap_v(0.0f, c1, s1, 1.0f);
        cap_v(0.0f, c0, s0, 1.0f);

        cap_v(0.0f, c0, s0, 0.0f - 1.0f);
        cap_v(0.0f, c1, s1, 0.0f - 1.0f);
        cap_v(0.0f, c1, s1, 1.0f);
    }
}

const i32 DRAW_BOX = 0;
const i32 DRAW_SPHERE = 1;
const i32 DRAW_CAPSULE = 2;

// upstream debug_adapter.h BOX3D_GROUND_GRID_CELL_SIZE: world-space
// size of a minor grid cell, major lines every 10 inside the shader
const f32 GROUND_GRID_CELL = 1.0f;

// Which shape draws with the procedural ground grid.
b3ShapeId g_ground_shape;
bool g_ground_shape_valid;

// upstream Sample::SetGroundShape( b3ShapeId )
void set_ground_shape(b3ShapeId shapeId) {
    g_ground_shape = shapeId;
    g_ground_shape_valid = true;
}

// --- shadow map ------------------------------------------------------
//
// Three cascades, refitted to the camera each frame. Scenes here span
// roughly 400x in scale, so one fixed region does not fit all.

// The array costs SHADOW_DIM^2 * SHADOW_CASCADES * 4 bytes: 192 MB at
// 4096. WebGL2 follows GLES 3.0, which guarantees a maximum texture size
// of only 2048, so the web build uses that.
when os(wasm) {
    const i32 SHADOW_DIM = 2048;
}
when os(windows) || os(linux) || os(macos) || os(ios) || os(android) {
    const i32 SHADOW_DIM = 4096;
}

// Debug channel: the segment buffer and its line pipeline.
sg_pipeline g_pip_dbg;
sg_buffer g_dbg_vbuf;

// hull outlines: the edge shader, non-indexed
sg_pipeline g_pip_lines_ns;

// PCF tap spacing in texels. Each tap is a bilinear 2x2 through the
// comparison sampler, so 3x3 taps at spacing S spread over (2 + 2S)
// texels: blur radius 1 + S. Below about 0.5 the taps stop
// antialiasing the edge.
const f32 SHADOW_PCF_SPACING = 0.75f;
sg_image g_shadow_img;
sg_view[SHADOW_CASCADES] g_shadow_att;   // one depth attachment per slice
sg_view g_shadow_tex;      // texture view over the whole array
sg_sampler g_shadow_smp;   // comparison sampler
sg_pipeline g_pip_depth_ns;   // spheres (non-indexed)
sg_pipeline g_pip_depth_cap;  // capsules (non-indexed)
float4x4[SHADOW_CASCADES] g_light_clip;   // world -> light clip (depth pass)
float4x4[SHADOW_CASCADES] g_light_mat;    // world -> shadow texture space
float3[SHADOW_CASCADES] g_cascade_centre; // fitted sphere, for culling
f32[SHADOW_CASCADES] g_cascade_radius;
f32[SHADOW_CASCADES] g_cascade_far;       // far view depth of each cascade

// Orthographic projection, column-major, matching linear.mc's
// per-target clip-space convention (D3D/Metal z in [0,1], GL z in
// [-1,1]).
float4x4 ortho_proj(f32 halfW, f32 halfH, f32 near, f32 far) {
    f32 zScale = 0.0f;
    f32 zOffset = 0.0f;
    when os(windows) || os(macos) || os(ios) {
        zScale = 1.0f / (near - far);
        zOffset = near / (near - far);
    }
    when os(linux) || os(wasm) || os(android) {
        zScale = 2.0f / (near - far);
        zOffset = (far + near) / (near - far);
    }
    return float4x4{
        1.0f / halfW, 0.0f,         0.0f,   0.0f,
        0.0f,         1.0f / halfH, 0.0f,   0.0f,
        0.0f,         0.0f,         zScale, 0.0f,
        0.0f,         0.0f,         zOffset, 1.0f
    };
}

// Clip space -> shadow-map texture space, folded into each cascade
// matrix. D3D and Metal sample top-left with clip z in [0,1]; GL
// samples bottom-left and needs z remapped from [-1,1].
float4x4 clip_to_texture() {
    f32 yScale = 0.0f;
    f32 zScale = 0.0f;
    f32 zOffset = 0.0f;
    when os(windows) || os(macos) || os(ios) {
        yScale = -0.5f;
        zScale = 1.0f;
    }
    when os(linux) || os(wasm) || os(android) {
        yScale = 0.5f;
        zScale = 0.5f;
        zOffset = 0.5f;
    }
    return float4x4{
        0.5f, 0.0f,   0.0f,   0.0f,
        0.0f, yScale, 0.0f,   0.0f,
        0.0f, 0.0f,   zScale, 0.0f,
        0.5f, 0.5f,   zOffset, 1.0f
    };
}

// upstream gfx/shadow.{h,c}: the cascade range is the scene's, not the
// camera's — sample.cpp fits far to the world bounds diagonal.
const f32 SHADOW_SPLIT_NEAR = 0.1f;
// Room on the light's near side for casters standing above a cascade slice.
const f32 CASTER_MARGIN = 50.0f;
const f32 SHADOW_SPLIT_FAR = 50.0f;
const f32 SHADOW_SPLIT_FAR_MAX = 200.0f;
const f32 PSSM_LAMBDA = 0.5f;
const f32 PSSM_LAMBDA_MAX = 0.9f;
f32 g_shadow_split_far = SHADOW_SPLIT_FAR;

// Toward log as the range widens, so the near cascade stays tight.
f32 split_lambda_for_range(f32 nearZ, f32 farZ) {
    f32 decades = logf((farZ / nearZ) / (SHADOW_SPLIT_FAR / SHADOW_SPLIT_NEAR)) / logf(10.0f);
    return b3ClampFloat(PSSM_LAMBDA + 0.4f * decades, PSSM_LAMBDA, PSSM_LAMBDA_MAX);
}

// Fit one cascade to the view slice between near and far. The bounding
// sphere's radius depends only on the slice depths and the field of
// view, not on where the camera points, so texel size holds still as
// the camera turns.
void fit_cascade(i32 index, f32 nearZ, f32 farZ) {
    float3 sun = float3{0.4880f, 0.7807f, 0.3904f};
    f32 tanHalf = tanf(0.5f * 60.0f * PI_F / 180.0f);
    f32 w = sapp_widthf();
    f32 h = sapp_heightf();
    f32 aspect = h > 0.0f ? w / h : 1.0f;
    float3 vdir = float3{0.0f - cam_forward.x, 0.0f - cam_forward.y, 0.0f - cam_forward.z};

    // bound the slice by its eight corners
    f32[8] cx; f32[8] cy; f32[8] cz;
    float3 centre = float3{0.0f, 0.0f, 0.0f};
    i32 n = 0;
    for i32 e = 0; e < 2; e++ {
        f32 z = e == 0 ? nearZ : farZ;
        f32 hy = z * tanHalf;
        f32 hx = hy * aspect;
        for i32 sy = 0; sy < 2; sy++ {
            for i32 sx = 0; sx < 2; sx++ {
                f32 fx = sx == 0 ? 0.0f - hx : hx;
                f32 fy = sy == 0 ? 0.0f - hy : hy;
                // Eye-relative, like everything else the GPU sees: the eye
                // is the draw origin, so it contributes nothing here.
                cx[n] = vdir.x * z + cam_right.x * fx + cam_up.x * fy;
                cy[n] = vdir.y * z + cam_right.y * fx + cam_up.y * fy;
                cz[n] = vdir.z * z + cam_right.z * fx + cam_up.z * fy;
                centre.x += cx[n]; centre.y += cy[n]; centre.z += cz[n];
                n++;
            }
        }
    }
    centre.x /= 8.0f; centre.y /= 8.0f; centre.z /= 8.0f;
    f32 radius = 0.0f;
    for i32 i = 0; i < 8; i++ {
        f32 dx = cx[i] - centre.x;
        f32 dy = cy[i] - centre.y;
        f32 dz = cz[i] - centre.z;
        f32 d = sqrtf(dx * dx + dy * dy + dz * dz);
        if d > radius { radius = d; }
    }
    if radius < 0.5f { radius = 0.5f; }
    g_cascade_centre[index] = centre;
    g_cascade_radius[index] = radius;

    // The light eye sits CASTER_MARGIN in front of the sphere, and the
    // depth range ends at its back face: casters up to CASTER_MARGIN above
    // the slice reach the map, and no depth range is spent behind it.
    f32 back = radius + CASTER_MARGIN;
    float3 eye = float3{centre.x + sun.x * back,
                        centre.y + sun.y * back,
                        centre.z + sun.z * back};
    float3 up = float3{0.0f, 1.0f, 0.0f};
    if sun.y > 0.99f { up = float3{0.0f, 0.0f, 1.0f}; }
    float4x4 view = look_at(eye, centre, up);
    float4x4 proj = ortho_proj(radius, radius, 0.0f, 2.0f * radius + CASTER_MARGIN);

    // Texel snapping: project the world origin, round to the nearest
    // texel, translate the projection by the remainder. Without it the
    // grid slides under the geometry and edges crawl.
    float4x4 snap = mul(proj, view);
    float4 origin = mul(snap, float4{0.0f, 0.0f, 0.0f, 1.0f});
    f32 halfDim = 0.5f * cast(f32, SHADOW_DIM);
    f32 tx = origin.x * halfDim;
    f32 ty = origin.y * halfDim;
    f32* pe = cast(f32*, &proj);
    pe[12] += (roundf(tx) - tx) / halfDim;
    pe[13] += (roundf(ty) - ty) / halfDim;

    // the depth pass rasterizes in clip space; the lookup uses texture
    // space
    g_light_clip[index] = mul(proj, view);
    g_light_mat[index] = mul(clip_to_texture(), g_light_clip[index]);
}

// Split the shadowed range into cascades and fit each.
void update_light_matrix() {
    f32 far = g_shadow_split_far;
    f32 near = SHADOW_SPLIT_NEAR;
    f32 lambda = split_lambda_for_range(near, far);
    f32 prev = near;
    for i32 i = 0; i < SHADOW_CASCADES; i++ {
        f32 t = cast(f32, i + 1) / cast(f32, SHADOW_CASCADES);
        f32 even = near + (far - near) * t;
        f32 logarithmic = near * powf(far / near, t);
        f32 split = lambda * logarithmic + (1.0f - lambda) * even;
        fit_cascade(i, prev, split);
        g_cascade_far[i] = split;
        prev = split;
    }
}

// --- rendering ------------------------------------------------------

sg_pipeline g_pip_sky;
sg_buffer g_sky_vbuf;
sg_pipeline g_pip_sph;
sg_pipeline g_pip_sph_mirror;
sg_pipeline g_pip_cap;
sg_buffer g_sph_vbuf;
sg_buffer g_cap_vbuf;

// Column-major model matrix: translate * rotate(q) * scale(s), with an
// optional body-local offset (rotated by q) for baked-transform hulls.
float4x4 make_model(b3Quat q, f32 px, f32 py, f32 pz, f32 sx, f32 sy, f32 sz) {
    f32 x = q.v.x; f32 y = q.v.y; f32 z = q.v.z; f32 w = q.s;
    f32 x2 = x + x; f32 y2 = y + y; f32 z2 = z + z;
    f32 xx = x * x2; f32 xy = x * y2; f32 xz = x * z2;
    f32 yy = y * y2; f32 yz = y * z2; f32 zz = z * z2;
    f32 wx = w * x2; f32 wy = w * y2; f32 wz = w * z2;
    return float4x4{
        (1.0f - (yy + zz)) * sx, (xy + wz) * sx,          (xz - wy) * sx,          0.0f,
        (xy - wz) * sy,          (1.0f - (xx + zz)) * sy, (yz + wx) * sy,          0.0f,
        (xz + wy) * sz,          (yz - wx) * sz,          (1.0f - (xx + yy)) * sz, 0.0f,
        px,                      py,                      pz,                      1.0f
    };
}
