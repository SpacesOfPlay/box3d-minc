// Rendering: the shaders, the impostor proxy cube, and the shadow
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

// Instanced solids. One record per shape, stepped per instance; row0..2
// are the rows of the model matrix's affine 3x4, tint and params carry
// what PerDraw carries.
struct InstRec {
    float4 row0;
    float4 row1;
    float4 row2;
    float4 tint;
    float4 params;
}

// Clip matrix for a whole pass, applied once per pipeline bind.
struct InstPass {
    float4x4 viewproj;
}

// Per-pass shadow data (uniform slot 1). Each cascade matrix folds in
// the clip-space-to-texture-space remap, so the fragment shader divides
// by w and uses xy as the lookup, z as the reference depth.
//
// Four cascades, upstream's count. Per-cascade values ride in float4
// lanes, so four is also the ceiling. More cascades cannot sharpen the
// last one, which always has to reach the far split; they buy a finer
// near cascade and a smaller step at each boundary, at the cost of
// another shadow pass.
const i32 SHADOW_CASCADES = 4;

struct ShadowUni {
    float4x4[SHADOW_CASCADES] cascade;
    float4 cascadeFar;         // far view depth of each cascade
    float4 cascadeTexelWorld;  // one shadow texel in world units, per cascade
    float4 cascadeDepthBias;   // residual comparison bias in depth units, per cascade
    float4 camPos;             // xyz: camera position, zero in the relative frame
    float4 viewDir;            // xyz: camera looking direction
    float4 shParams;           // x: one texel in UV, y: unshadowed floor
    // Rows of the view-projection the impostors rebuild their depth
    // from, and the clip-to-window depth remap of the target.
    float4 clipZ;
    float4 clipW;
    float4 depthRemap;         // x: scale, y: offset
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

// Lane i of a per-cascade float4.
f32 shadow_lane(float4 v, i32 i) {
    if i == 0 { return v.x; }
    if i == 1 { return v.y; }
    if i == 2 { return v.z; }
    return v.w;
}

// The shadow lookup, shared by the mesh and impostor fragment shaders
// after upstream's common/shadow_cascade.glsl, common/shadow_pcf.glsl
// and the sampleCascade in shapes/cube.glsl. A helper cannot take the
// texture, so the sixteen taps themselves stay in each shader; these
// produce everything around them.

struct ShadowSel {
    i32 c;       // cascade whose slice holds this view depth
    f32 alpha;   // blend weight into the next cascade, 0 outside the band
}

// The cascade is picked by view depth; the last one is a fallback
// rather than a range, its map footprint runs well past the final
// split. The last 10% of each cascade's range blends with the next:
// texel density jumps by roughly 2.5x at every boundary, so even with
// matched bias the penumbra width changes there, and without the blend
// that reads as a polygonal step on the ground.
ShadowSel shadow_select(float4 cascadeFar, f32 viewZ) {
    ShadowSel s;
    s.c = SHADOW_CASCADES - 1;
    s.alpha = 0.0f;
    if viewZ < cascadeFar.x { s.c = 0; }
    else if viewZ < cascadeFar.y { s.c = 1; }
    else if viewZ < cascadeFar.z { s.c = 2; }
    if s.c < SHADOW_CASCADES - 1 {
        f32 farZ = shadow_lane(cascadeFar, s.c);
        f32 nearZ = s.c == 0 ? 0.0f : shadow_lane(cascadeFar, s.c - 1);
        f32 blendStart = mix(farZ, nearZ, 0.1f);
        if viewZ > blendStart {
            s.alpha = (viewZ - blendStart) / (farZ - blendStart);
        }
    }
    return s;
}

// Where cascade cc reads for a surface point: the lookup moved
// sideways along the normal (normal-offset shadows, which keep contact
// shadows welded where a depth bias would detach them), projected
// into the cascade's texture space. The offset has to clear the whole
// filter: 3 texels is the scaled 7x7 half-width, plus the sampling
// snap, and the root two reaches the kernel's corner tap.
float3 shadow_lookup(float4x4 cascadeMat, float3 wpos, float3 nn, f32 sinTheta,
                     f32 scale, f32 texelWorld) {
    f32 offset = (3.0f * scale + 0.5f) * 1.41421f * sinTheta * texelWorld;
    float4 lc = mul(cascadeMat, float4{wpos.x + nn.x * offset,
                                       wpos.y + nn.y * offset,
                                       wpos.z + nn.z * offset, 1.0f});
    return float3{lc.x / lc.w, lc.y / lc.w, lc.z / lc.w};
}

// The 7x7 uniform kernel from 16 bilinear comparison taps. Tap
// positions and weights steer the hardware 2x2 compare so the fetches
// reproduce the full 49-texel kernel with triangle-filtered sub-texel
// placement: a moving shadow edge slides instead of stepping texel by
// texel. The optimized PCF from The Witness (Ignacio Castano), via
// MJP's shadows sample.
struct PcfTaps {
    f32 bx; f32 by;        // nearest texel corner, in UV
    f32 step;              // one texel in UV, scaled for the cascade
    f32 refz; f32 layer;
    f32 u0; f32 u1; f32 u2; f32 u3;
    f32 v0; f32 v1; f32 v2; f32 v3;
    f32 uw0; f32 uw1; f32 uw2; f32 uw3;
    f32 vw0; f32 vw1; f32 vw2; f32 vw3;
    // Weight of the sampled result against fully lit. Zero past the
    // kernel's reach outside the map, where nothing needs sampling.
    // Inside, taps past a cascade edge compare against the sampler's
    // white border and come back lit, so the boundary dissolves across
    // the kernel rather than flipping whole pixels as the texel snap
    // steps the map. WebGL2, WebGPU and iOS have no border mode and
    // clamp to the edge texel instead; the fade over the band the
    // kernel can reach past the edge stands in for the border there
    // and changes little where the border exists.
    f32 fade;
}

PcfTaps shadow_taps(float3 sp, f32 depthBias, f32 scale, f32 invSize, i32 cc) {
    PcfTaps t;
    t.fade = 0.0f;
    // The 3 * scale here and in shadow_lookup both encode the 7x7
    // kernel's three-texel radius.
    f32 reach = (3.0f * scale + 1.0f) * invSize;
    if sp.z < 0.0f || sp.z > 1.0f
       || sp.x <= 0.0f - reach || sp.x >= 1.0f + reach
       || sp.y <= 0.0f - reach || sp.y >= 1.0f + reach {
        return t;
    }
    f32 edge = min(min(sp.x, 1.0f - sp.x), min(sp.y, 1.0f - sp.y));
    t.fade = clamp((edge + reach) / (2.0f * reach), 0.0f, 1.0f);

    f32 size = 1.0f / invSize;
    t.layer = cast(f32, cc);
    t.refz = sp.z - depthBias;
    f32 tux = sp.x * size;
    f32 tuy = sp.y * size;
    f32 bx = floor(tux + 0.5f);
    f32 by = floor(tuy + 0.5f);
    f32 fs = tux + 0.5f - bx;
    f32 ft = tuy + 0.5f - by;
    t.bx = (bx - 0.5f) * invSize;
    t.by = (by - 0.5f) * invSize;
    t.step = invSize * scale;

    t.uw0 = 5.0f * fs - 6.0f;
    t.uw1 = 11.0f * fs - 28.0f;
    t.uw2 = 0.0f - (11.0f * fs + 17.0f);
    t.uw3 = 0.0f - (5.0f * fs + 1.0f);
    t.u0 = (4.0f * fs - 5.0f) / t.uw0 - 3.0f;
    t.u1 = (4.0f * fs - 16.0f) / t.uw1 - 1.0f;
    t.u2 = 0.0f - (7.0f * fs + 5.0f) / t.uw2 + 1.0f;
    t.u3 = 0.0f - fs / t.uw3 + 3.0f;

    t.vw0 = 5.0f * ft - 6.0f;
    t.vw1 = 11.0f * ft - 28.0f;
    t.vw2 = 0.0f - (11.0f * ft + 17.0f);
    t.vw3 = 0.0f - (5.0f * ft + 1.0f);
    t.v0 = (4.0f * ft - 5.0f) / t.vw0 - 3.0f;
    t.v1 = (4.0f * ft - 16.0f) / t.vw1 - 1.0f;
    t.v2 = 0.0f - (7.0f * ft + 5.0f) / t.vw2 + 1.0f;
    t.v3 = 0.0f - ft / t.vw3 + 3.0f;
    return t;
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

    // Shadow lookup: see shadow_select and shadow_taps above.
    f32 viewZ = dot(float3{input.wpos.x - sh.camPos.x,
                           input.wpos.y - sh.camPos.y,
                           input.wpos.z - sh.camPos.z},
                    float3{sh.viewDir.x, sh.viewDir.y, sh.viewDir.z});
    ShadowSel sel = shadow_select(sh.cascadeFar, viewZ);
    // Sine of the angle between surface and light: zero facing the sun,
    // where no offset is wanted, growing toward grazing.
    f32 sinTheta = sqrt(max(1.0f - nl * nl, 0.0f));
    f32 shadow = 1.0f;
    for i32 k = 0; k < 2; k++ {
        if k == 0 || sel.alpha > 0.0f {
            i32 cc = sel.c + k;
            f32 texelWorld = shadow_lane(sh.cascadeTexelWorld, cc);
            // Shrink the kernel in the coarser cascades so the penumbra
            // keeps a constant world width instead of going crisp to
            // blurry at every boundary. The floor leaves the taps about
            // two texels apart, close enough that the hardware 2x2
            // compare still overlaps them.
            f32 scale = clamp(sh.cascadeTexelWorld.x / texelWorld, 0.25f, 1.0f);
            float3 sp = shadow_lookup(sh.cascade[cc], input.wpos, nn, sinTheta, scale, texelWorld);
            PcfTaps tp = shadow_taps(sp, shadow_lane(sh.cascadeDepthBias, cc), scale, sh.shParams.x, cc);
            f32 s = 1.0f;
            if tp.fade > 0.0f {
                f32 sum = 0.0f;
                sum += tp.uw0 * tp.vw0 * sample_cmp(shadowMap, shadowSmp, float3{tp.bx + tp.u0 * tp.step, tp.by + tp.v0 * tp.step, tp.layer}, tp.refz);
                sum += tp.uw1 * tp.vw0 * sample_cmp(shadowMap, shadowSmp, float3{tp.bx + tp.u1 * tp.step, tp.by + tp.v0 * tp.step, tp.layer}, tp.refz);
                sum += tp.uw2 * tp.vw0 * sample_cmp(shadowMap, shadowSmp, float3{tp.bx + tp.u2 * tp.step, tp.by + tp.v0 * tp.step, tp.layer}, tp.refz);
                sum += tp.uw3 * tp.vw0 * sample_cmp(shadowMap, shadowSmp, float3{tp.bx + tp.u3 * tp.step, tp.by + tp.v0 * tp.step, tp.layer}, tp.refz);
                sum += tp.uw0 * tp.vw1 * sample_cmp(shadowMap, shadowSmp, float3{tp.bx + tp.u0 * tp.step, tp.by + tp.v1 * tp.step, tp.layer}, tp.refz);
                sum += tp.uw1 * tp.vw1 * sample_cmp(shadowMap, shadowSmp, float3{tp.bx + tp.u1 * tp.step, tp.by + tp.v1 * tp.step, tp.layer}, tp.refz);
                sum += tp.uw2 * tp.vw1 * sample_cmp(shadowMap, shadowSmp, float3{tp.bx + tp.u2 * tp.step, tp.by + tp.v1 * tp.step, tp.layer}, tp.refz);
                sum += tp.uw3 * tp.vw1 * sample_cmp(shadowMap, shadowSmp, float3{tp.bx + tp.u3 * tp.step, tp.by + tp.v1 * tp.step, tp.layer}, tp.refz);
                sum += tp.uw0 * tp.vw2 * sample_cmp(shadowMap, shadowSmp, float3{tp.bx + tp.u0 * tp.step, tp.by + tp.v2 * tp.step, tp.layer}, tp.refz);
                sum += tp.uw1 * tp.vw2 * sample_cmp(shadowMap, shadowSmp, float3{tp.bx + tp.u1 * tp.step, tp.by + tp.v2 * tp.step, tp.layer}, tp.refz);
                sum += tp.uw2 * tp.vw2 * sample_cmp(shadowMap, shadowSmp, float3{tp.bx + tp.u2 * tp.step, tp.by + tp.v2 * tp.step, tp.layer}, tp.refz);
                sum += tp.uw3 * tp.vw2 * sample_cmp(shadowMap, shadowSmp, float3{tp.bx + tp.u3 * tp.step, tp.by + tp.v2 * tp.step, tp.layer}, tp.refz);
                sum += tp.uw0 * tp.vw3 * sample_cmp(shadowMap, shadowSmp, float3{tp.bx + tp.u0 * tp.step, tp.by + tp.v3 * tp.step, tp.layer}, tp.refz);
                sum += tp.uw1 * tp.vw3 * sample_cmp(shadowMap, shadowSmp, float3{tp.bx + tp.u1 * tp.step, tp.by + tp.v3 * tp.step, tp.layer}, tp.refz);
                sum += tp.uw2 * tp.vw3 * sample_cmp(shadowMap, shadowSmp, float3{tp.bx + tp.u2 * tp.step, tp.by + tp.v3 * tp.step, tp.layer}, tp.refz);
                sum += tp.uw3 * tp.vw3 * sample_cmp(shadowMap, shadowSmp, float3{tp.bx + tp.u3 * tp.step, tp.by + tp.v3 * tp.step, tp.layer}, tp.refz);
                s = mix(1.0f, sum / 2704.0f, tp.fade);
            }
            if k == 0 { shadow = s; }
            else { shadow = mix(shadow, s, sel.alpha); }
        }
    }
    // shParams.y keeps shadowed faces from going fully black
    shadow = sh.shParams.y + (1.0f - sh.shParams.y) * shadow;

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

@shader fragment
float4 b3_depth_fs(EdgeOut input) {
    return float4{0.0f, 0.0f, 0.0f, 1.0f};
}

// --- solids -----------------------------------------------------------
//
// The model matrix and the shape's tint/params arrive as per-instance
// attributes; only the pass matrix is a uniform.

@shader vertex
VsOut b3_inst_vs(
    @attr(0) float3 position,
    @attr(1) float3 normal,
    @attr(2) float4 irow0,
    @attr(3) float4 irow1,
    @attr(4) float4 irow2,
    @attr(5) float4 itint,
    @attr(6) float4 iparams,
    @uniform(0) InstPass u
) {
    f32 wx = irow0.x * position.x + irow0.y * position.y + irow0.z * position.z + irow0.w;
    f32 wy = irow1.x * position.x + irow1.y * position.y + irow1.z * position.z + irow1.w;
    f32 wz = irow2.x * position.x + irow2.y * position.y + irow2.z * position.z + irow2.w;
    // rows carry R*S, so the normal divides by scale twice; params.yzw
    // is 1/scale^2 per axis
    f32 nx = normal.x * iparams.y;
    f32 ny = normal.y * iparams.z;
    f32 nz = normal.z * iparams.w;
    VsOut o;
    o.pos = mul(u.viewproj, float4{wx, wy, wz, 1.0f});
    o.normal = normalize(float3{irow0.x * nx + irow0.y * ny + irow0.z * nz,
                                irow1.x * nx + irow1.y * ny + irow1.z * nz,
                                irow2.x * nx + irow2.y * ny + irow2.z * nz});
    o.tint = itint;
    o.wpos = float3{wx, wy, wz};
    o.cell = iparams.x;
    return o;
}

// Sphere and capsule impostors, after upstream shapes/sphere.glsl and
// capsule.glsl. The vertex stage rasterizes the shape's oriented
// bounding box; the fragment stage casts the eye ray at the exact
// surface, so both shapes are round at any zoom and need no mesh. A
// sphere is a capsule of zero half-length, so one shader serves both:
// the cylinder part vanishes and the two cap spheres coincide. The
// instance rows carry the rotation only; params.y is the half-length,
// params.z the radius, params.w 1 for a sphere (three equator rings)
// and 0 for a capsule (two).
//
// Writing depth from the fragment shader turns off early depth
// rejection for these pipelines. Accepted for impostors, as upstream.
struct ImpOut {
    float4 pos;
    float3 wpos;            // proxy surface point, eye-relative
    @flat float3 center;
    @flat float3 rrow0;     // rotation rows; their columns are R^T's rows
    @flat float3 rrow1;
    @flat float3 rrow2;
    @flat float4 tint;
    @flat float4 params;
}

struct ImpFrag {
    float4 color;
    @depth f32 depth;
}

@shader vertex
ImpOut b3_imp_vs(
    @attr(0) float3 position,
    @attr(1) float3 normal,
    @attr(2) float4 irow0,
    @attr(3) float4 irow1,
    @attr(4) float4 irow2,
    @attr(5) float4 itint,
    @attr(6) float4 iparams,
    @uniform(0) InstPass u
) {
    f32 halfLen = iparams.y;
    f32 radius = iparams.z;
    // unit-cube corners at half-extent 0.5, scaled to the box that
    // encloses the shape about its local +X axis
    float3 lp = float3{position.x * 2.0f * (halfLen + radius),
                       position.y * 2.0f * radius,
                       position.z * 2.0f * radius};
    f32 wx = irow0.x * lp.x + irow0.y * lp.y + irow0.z * lp.z + irow0.w;
    f32 wy = irow1.x * lp.x + irow1.y * lp.y + irow1.z * lp.z + irow1.w;
    f32 wz = irow2.x * lp.x + irow2.y * lp.y + irow2.z * lp.z + irow2.w;
    ImpOut o;
    o.pos = mul(u.viewproj, float4{wx, wy, wz, 1.0f});
    o.wpos = float3{wx, wy, wz};
    o.center = float3{irow0.w, irow1.w, irow2.w};
    o.rrow0 = float3{irow0.x, irow0.y, irow0.z};
    o.rrow1 = float3{irow1.x, irow1.y, irow1.z};
    o.rrow2 = float3{irow2.x, irow2.y, irow2.z};
    o.tint = itint;
    o.params = iparams;
    return o;
}

@shader fragment
ImpFrag b3_imp_fs(
    ImpOut input,
    @uniform(1) ShadowUni sh,
    @texture(0) Texture2DArray shadowMap,
    @sampler(0) Sampler shadowSmp
) {
    // The eye is the origin of the frame the GPU sees, so the ray starts
    // there and the proxy point is its direction.
    float3 rd = normalize(input.wpos);
    f32 halfLen = input.params.y;
    f32 radius = input.params.z;
    // local +X in world is the first column of R
    float3 axis = float3{input.rrow0.x, input.rrow1.x, input.rrow2.x};
    float3 pa = float3{input.center.x - halfLen * axis.x,
                       input.center.y - halfLen * axis.y,
                       input.center.z - halfLen * axis.z};
    float3 pb = float3{input.center.x + halfLen * axis.x,
                       input.center.y + halfLen * axis.y,
                       input.center.z + halfLen * axis.z};

    // Ray-cylinder, Quilez closed form, with the origin at zero.
    // Reference: https://iquilezles.org/articles/intersectors/
    float3 ba = float3{pb.x - pa.x, pb.y - pa.y, pb.z - pa.z};
    float3 oa = float3{0.0f - pa.x, 0.0f - pa.y, 0.0f - pa.z};
    f32 baba = dot(ba, ba);
    f32 bard = dot(ba, rd);
    f32 baoa = dot(ba, oa);
    f32 rdoa = dot(rd, oa);
    f32 oaoa = dot(oa, oa);
    f32 ac = baba - bard * bard;
    f32 bc = baba * rdoa - baoa * bard;
    f32 cc2 = baba * oaoa - baoa * baoa - radius * radius * baba;
    f32 hc = bc * bc - ac * cc2;

    f32 t = 0.0f - 1.0f;
    float3 n = float3{0.0f, 0.0f, 0.0f};
    // ac collapses when the ray runs along the axis and baba when the
    // half-length is zero; the caps cover both.
    if hc >= 0.0f && ac > 0.00000001f {
        f32 tb = (0.0f - bc - sqrt(hc)) / ac;
        f32 y = baoa + tb * bard;
        if tb > 0.0f && y > 0.0f && y < baba {
            float3 hit = float3{rd.x * tb, rd.y * tb, rd.z * tb};
            f32 f = y / baba;
            float3 axisPt = float3{pa.x + ba.x * f, pa.y + ba.y * f, pa.z + ba.z * f};
            n = float3{(hit.x - axisPt.x) / radius, (hit.y - axisPt.y) / radius, (hit.z - axisPt.z) / radius};
            t = tb;
        }
    }
    // Cap spheres at both ends, keeping the nearer hit. Only the half
    // of each cap outside the cylinder is a surface.
    for i32 i = 0; i < 2; i++ {
        float3 p = i == 0 ? pa : pb;
        f32 side = i == 0 ? 0.0f - 1.0f : 1.0f;
        float3 oc = float3{0.0f - p.x, 0.0f - p.y, 0.0f - p.z};
        f32 b = dot(oc, rd);
        f32 c = dot(oc, oc) - radius * radius;
        f32 h = b * b - c;
        if h >= 0.0f {
            f32 tc = 0.0f - b - sqrt(h);
            if tc > 0.0f && (t < 0.0f || tc < t) {
                float3 hit = float3{rd.x * tc, rd.y * tc, rd.z * tc};
                float3 hp = float3{hit.x - p.x, hit.y - p.y, hit.z - p.z};
                if dot(hp, axis) * side >= 0.0f {
                    n = float3{hp.x / radius, hp.y / radius, hp.z / radius};
                    t = tc;
                }
            }
        }
    }
    if t < 0.0f {
        // box corner past the silhouette, or the shape behind the eye
        discard;
    }

    float3 hit = float3{rd.x * t, rd.y * t, rd.z * t};
    float3 nn = normalize(n);

    // Object-space normal via R^T, for the surface pattern: a north/
    // south hemisphere split and great-circle rings at the equators of
    // the object axes, so spin is visible. fwidth on the signed
    // component keeps each ring about 1.5 px wide at any zoom; abs has
    // a kink at the equator that would speckle it.
    float3 nObj = float3{
        input.rrow0.x * nn.x + input.rrow1.x * nn.y + input.rrow2.x * nn.z,
        input.rrow0.y * nn.x + input.rrow1.y * nn.y + input.rrow2.y * nn.z,
        input.rrow0.z * nn.x + input.rrow1.z * nn.y + input.rrow2.z * nn.z};
    f32 hemi = nObj.y > 0.0f ? 1.0f : 0.65f;
    f32 ringY = 1.0f - smoothstep(0.0f, fwidth(nObj.y) * 1.5f, abs(nObj.y));
    f32 ringZ = 1.0f - smoothstep(0.0f, fwidth(nObj.z) * 1.5f, abs(nObj.z));
    f32 ring = max(ringY, ringZ);
    if input.params.w > 0.5f {
        f32 ringX = 1.0f - smoothstep(0.0f, fwidth(nObj.x) * 1.5f, abs(nObj.x));
        ring = max(ring, ringX);
    }
    float3 base = float3{input.tint.x * hemi, input.tint.y * hemi, input.tint.z * hemi};
    float3 surface = mix(base, base * 0.35f, ring);

    // sun dir: upstream renderer's normalize(0.5, 0.8, 0.4)
    f32 nl = max(dot(nn, float3{0.4880f, 0.7807f, 0.3904f}), 0.0f);

    // Shadow lookup: see shadow_select and shadow_taps above.
    f32 viewZ = dot(hit, float3{sh.viewDir.x, sh.viewDir.y, sh.viewDir.z});
    ShadowSel sel = shadow_select(sh.cascadeFar, viewZ);
    f32 sinTheta = sqrt(max(1.0f - nl * nl, 0.0f));
    f32 shadow = 1.0f;
    for i32 k = 0; k < 2; k++ {
        if k == 0 || sel.alpha > 0.0f {
            i32 cc = sel.c + k;
            f32 texelWorld = shadow_lane(sh.cascadeTexelWorld, cc);
            f32 scale = clamp(sh.cascadeTexelWorld.x / texelWorld, 0.25f, 1.0f);
            float3 sp = shadow_lookup(sh.cascade[cc], hit, nn, sinTheta, scale, texelWorld);
            PcfTaps tp = shadow_taps(sp, shadow_lane(sh.cascadeDepthBias, cc), scale, sh.shParams.x, cc);
            f32 s = 1.0f;
            if tp.fade > 0.0f {
                f32 sum = 0.0f;
                sum += tp.uw0 * tp.vw0 * sample_cmp(shadowMap, shadowSmp, float3{tp.bx + tp.u0 * tp.step, tp.by + tp.v0 * tp.step, tp.layer}, tp.refz);
                sum += tp.uw1 * tp.vw0 * sample_cmp(shadowMap, shadowSmp, float3{tp.bx + tp.u1 * tp.step, tp.by + tp.v0 * tp.step, tp.layer}, tp.refz);
                sum += tp.uw2 * tp.vw0 * sample_cmp(shadowMap, shadowSmp, float3{tp.bx + tp.u2 * tp.step, tp.by + tp.v0 * tp.step, tp.layer}, tp.refz);
                sum += tp.uw3 * tp.vw0 * sample_cmp(shadowMap, shadowSmp, float3{tp.bx + tp.u3 * tp.step, tp.by + tp.v0 * tp.step, tp.layer}, tp.refz);
                sum += tp.uw0 * tp.vw1 * sample_cmp(shadowMap, shadowSmp, float3{tp.bx + tp.u0 * tp.step, tp.by + tp.v1 * tp.step, tp.layer}, tp.refz);
                sum += tp.uw1 * tp.vw1 * sample_cmp(shadowMap, shadowSmp, float3{tp.bx + tp.u1 * tp.step, tp.by + tp.v1 * tp.step, tp.layer}, tp.refz);
                sum += tp.uw2 * tp.vw1 * sample_cmp(shadowMap, shadowSmp, float3{tp.bx + tp.u2 * tp.step, tp.by + tp.v1 * tp.step, tp.layer}, tp.refz);
                sum += tp.uw3 * tp.vw1 * sample_cmp(shadowMap, shadowSmp, float3{tp.bx + tp.u3 * tp.step, tp.by + tp.v1 * tp.step, tp.layer}, tp.refz);
                sum += tp.uw0 * tp.vw2 * sample_cmp(shadowMap, shadowSmp, float3{tp.bx + tp.u0 * tp.step, tp.by + tp.v2 * tp.step, tp.layer}, tp.refz);
                sum += tp.uw1 * tp.vw2 * sample_cmp(shadowMap, shadowSmp, float3{tp.bx + tp.u1 * tp.step, tp.by + tp.v2 * tp.step, tp.layer}, tp.refz);
                sum += tp.uw2 * tp.vw2 * sample_cmp(shadowMap, shadowSmp, float3{tp.bx + tp.u2 * tp.step, tp.by + tp.v2 * tp.step, tp.layer}, tp.refz);
                sum += tp.uw3 * tp.vw2 * sample_cmp(shadowMap, shadowSmp, float3{tp.bx + tp.u3 * tp.step, tp.by + tp.v2 * tp.step, tp.layer}, tp.refz);
                sum += tp.uw0 * tp.vw3 * sample_cmp(shadowMap, shadowSmp, float3{tp.bx + tp.u0 * tp.step, tp.by + tp.v3 * tp.step, tp.layer}, tp.refz);
                sum += tp.uw1 * tp.vw3 * sample_cmp(shadowMap, shadowSmp, float3{tp.bx + tp.u1 * tp.step, tp.by + tp.v3 * tp.step, tp.layer}, tp.refz);
                sum += tp.uw2 * tp.vw3 * sample_cmp(shadowMap, shadowSmp, float3{tp.bx + tp.u2 * tp.step, tp.by + tp.v3 * tp.step, tp.layer}, tp.refz);
                sum += tp.uw3 * tp.vw3 * sample_cmp(shadowMap, shadowSmp, float3{tp.bx + tp.u3 * tp.step, tp.by + tp.v3 * tp.step, tp.layer}, tp.refz);
                s = mix(1.0f, sum / 2704.0f, tp.fade);
            }
            if k == 0 { shadow = s; }
            else { shadow = mix(shadow, s, sel.alpha); }
        }
    }
    shadow = sh.shParams.y + (1.0f - sh.shParams.y) * shadow;
    f32 lit = 0.35f + 0.65f * nl * shadow;

    ImpFrag o;
    o.color = float4{surface.x * lit, surface.y * lit, surface.z * lit, 1.0f};
    // The proxy's depth is the box face; the surface sits behind it.
    float4 hp4 = float4{hit.x, hit.y, hit.z, 1.0f};
    o.depth = dot(sh.clipZ, hp4) / dot(sh.clipW, hp4) * sh.depthRemap.x + sh.depthRemap.y;
    return o;
}

@shader vertex
EdgeOut b3_inst_depth_vs(
    @attr(0) float3 position,
    @attr(1) float3 normal,
    @attr(2) float4 irow0,
    @attr(3) float4 irow1,
    @attr(4) float4 irow2,
    @attr(5) float4 itint,
    @attr(6) float4 iparams,
    @uniform(0) InstPass u
) {
    f32 wx = irow0.x * position.x + irow0.y * position.y + irow0.z * position.z + irow0.w;
    f32 wy = irow1.x * position.x + irow1.y * position.y + irow1.z * position.z + irow1.w;
    f32 wz = irow2.x * position.x + irow2.y * position.y + irow2.z * position.z + irow2.w;
    EdgeOut o;
    o.pos = mul(u.viewproj, float4{wx, wy, wz, 1.0f});
    o.tint = itint;
    return o;
}

// Impostor shadow casters, after upstream shadow_caster_sphere.glsl
// and shadow_caster_capsule.glsl. The same box as the lit impostor,
// ray-cast along the light so caster and receiver agree on where the
// surface is. Only the entry point matters, not the normal: the
// capsule is the union of a cylinder and two cap spheres, and the
// entry into a union is the nearest entry across its parts, so the
// hemisphere rejection the lit shader needs is unnecessary.
struct ImpDepthOut {
    float4 pos;
    float3 wpos;            // proxy surface point
    @flat float3 center;
    @flat float3 axis;      // local +X in world
    @flat float4 params;
}

struct ImpDepthFrag {
    float4 color;           // no target is bound; the pipeline is depth-only
    @depth f32 depth;
}

// Per-cascade data for the caster fragment stage.
struct CasterPass {
    float4 lightDir;        // xyz: direction the light travels
    float4 depthRow;        // world position to window depth, w = 1
}

@shader vertex
ImpDepthOut b3_imp_depth_vs(
    @attr(0) float3 position,
    @attr(1) float3 normal,
    @attr(2) float4 irow0,
    @attr(3) float4 irow1,
    @attr(4) float4 irow2,
    @attr(5) float4 itint,
    @attr(6) float4 iparams,
    @uniform(0) InstPass u
) {
    f32 halfLen = iparams.y;
    f32 radius = iparams.z;
    float3 lp = float3{position.x * 2.0f * (halfLen + radius),
                       position.y * 2.0f * radius,
                       position.z * 2.0f * radius};
    f32 wx = irow0.x * lp.x + irow0.y * lp.y + irow0.z * lp.z + irow0.w;
    f32 wy = irow1.x * lp.x + irow1.y * lp.y + irow1.z * lp.z + irow1.w;
    f32 wz = irow2.x * lp.x + irow2.y * lp.y + irow2.z * lp.z + irow2.w;
    ImpDepthOut o;
    o.pos = mul(u.viewproj, float4{wx, wy, wz, 1.0f});
    o.wpos = float3{wx, wy, wz};
    o.center = float3{irow0.w, irow1.w, irow2.w};
    o.axis = float3{irow0.x, irow1.x, irow2.x};
    o.params = iparams;
    return o;
}

@shader fragment
ImpDepthFrag b3_imp_depth_fs(ImpDepthOut input, @uniform(1) CasterPass cp) {
    // The light is directional, so every ray is parallel: it starts on
    // the proxy face and travels with the light.
    float3 ro = input.wpos;
    float3 rd = float3{cp.lightDir.x, cp.lightDir.y, cp.lightDir.z};
    f32 halfLen = input.params.y;
    f32 radius = input.params.z;
    float3 axis = input.axis;
    float3 pa = float3{input.center.x - halfLen * axis.x,
                       input.center.y - halfLen * axis.y,
                       input.center.z - halfLen * axis.z};
    float3 pb = float3{input.center.x + halfLen * axis.x,
                       input.center.y + halfLen * axis.y,
                       input.center.z + halfLen * axis.z};

    float3 ba = float3{pb.x - pa.x, pb.y - pa.y, pb.z - pa.z};
    float3 oa = float3{ro.x - pa.x, ro.y - pa.y, ro.z - pa.z};
    f32 baba = dot(ba, ba);
    f32 bard = dot(ba, rd);
    f32 baoa = dot(ba, oa);
    f32 rdoa = dot(rd, oa);
    f32 oaoa = dot(oa, oa);
    f32 ac = baba - bard * bard;
    f32 bc = baba * rdoa - baoa * bard;
    f32 cc2 = baba * oaoa - baoa * baoa - radius * radius * baba;
    f32 hc = bc * bc - ac * cc2;

    f32 t = 0.0f - 1.0f;
    // Entry into each part, clamped forward rather than rejected. The
    // proxy faces are tangent to the surface along a line, and rounding
    // there puts the origin a hair inside, which drives the near root
    // negative. Treating that as a miss punches a dashed hole down the
    // middle of the shadow. A part whose far root is behind us too is a
    // genuine miss, which is what the end faces need.
    if hc >= 0.0f && ac > 0.00000001f {
        f32 q = sqrt(hc);
        if 0.0f - bc + q >= 0.0f {
            f32 tb = max((0.0f - bc - q) / ac, 0.0f);
            f32 y = baoa + tb * bard;
            if y > 0.0f && y < baba { t = tb; }
        }
    }
    for i32 i = 0; i < 2; i++ {
        float3 p = i == 0 ? pa : pb;
        float3 oc = float3{ro.x - p.x, ro.y - p.y, ro.z - p.z};
        f32 b = dot(oc, rd);
        f32 c = dot(oc, oc) - radius * radius;
        f32 h = b * b - c;
        if h >= 0.0f {
            f32 q = sqrt(h);
            if 0.0f - b + q >= 0.0f {
                f32 tc = max(0.0f - b - q, 0.0f);
                if t < 0.0f || tc < t { t = tc; }
            }
        }
    }
    if t < 0.0f {
        // box corner, the ray clears the shape
        discard;
    }
    ImpDepthFrag o;
    o.color = float4{0.0f, 0.0f, 0.0f, 1.0f};
    // orthographic light, so the clip w is 1 and the divide drops out
    o.depth = dot(cp.depthRow, float4{ro.x + t * rd.x, ro.y + t * rd.y, ro.z + t * rd.z, 1.0f});
    return o;
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

// --- proxy cube --------------------------------------------------------

// The impostors rasterize a unit cube, half-extent 0.5, in the same
// pos3 + normal3 layout the meshes use. 36 vertices, outward faces
// wound counter-clockwise, so the back-culled pipelines keep the faces
// toward the viewer or the light.
const i32 PROXY_NVERTS = 36;
f32[PROXY_NVERTS * 6] g_proxy_verts;

void proxy_face(i32 face, f32 nx, f32 ny, f32 nz, f32 ux, f32 uy, f32 uz, f32 vx, f32 vy, f32 vz) {
    // corners c = 0.5 * (n + s * u + t * v), two triangles
    f32[4] sx = { -1.0f, 1.0f, 1.0f, -1.0f };
    f32[4] tx = { -1.0f, -1.0f, 1.0f, 1.0f };
    i32[6] order = { 0, 1, 2, 0, 2, 3 };
    for i32 i = 0; i < 6; i++ {
        f32 s = sx[order[i]];
        f32 tt = tx[order[i]];
        i32 v = (face * 6 + i) * 6;
        g_proxy_verts[v..] = { 0.5f * (nx + s * ux + tt * vx),
                               0.5f * (ny + s * uy + tt * vy),
                               0.5f * (nz + s * uz + tt * vz),
                               nx, ny, nz };
    }
}

void build_proxy_cube() {
    // (u, v) chosen so u x v = n, which makes the winding counter-clockwise
    // seen from outside
    proxy_face(0,  1.0f, 0.0f, 0.0f,   0.0f, 1.0f, 0.0f,   0.0f, 0.0f, 1.0f);
    proxy_face(1, -1.0f, 0.0f, 0.0f,   0.0f, 0.0f, 1.0f,   0.0f, 1.0f, 0.0f);
    proxy_face(2,  0.0f, 1.0f, 0.0f,   0.0f, 0.0f, 1.0f,   1.0f, 0.0f, 0.0f);
    proxy_face(3,  0.0f, -1.0f, 0.0f,  1.0f, 0.0f, 0.0f,   0.0f, 0.0f, 1.0f);
    proxy_face(4,  0.0f, 0.0f, 1.0f,   1.0f, 0.0f, 0.0f,   0.0f, 1.0f, 0.0f);
    proxy_face(5,  0.0f, 0.0f, -1.0f,  0.0f, 1.0f, 0.0f,   1.0f, 0.0f, 0.0f);
}

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
// Four cascades, refitted to the camera each frame. Scenes here span
// roughly 400x in scale, so one fixed region does not fit all.

// The array costs SHADOW_DIM^2 * SHADOW_CASCADES * 4 bytes: 256 MB at
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

sg_image g_shadow_img;
sg_view[SHADOW_CASCADES] g_shadow_att;   // one depth attachment per slice
sg_view g_shadow_tex;      // texture view over the whole array
sg_sampler g_shadow_smp;   // comparison sampler
float4x4[SHADOW_CASCADES] g_light_clip;   // world -> light clip (depth pass)
float4x4[SHADOW_CASCADES] g_light_mat;    // world -> shadow texture space
f32[SHADOW_CASCADES] g_cascade_far;       // far view depth of each cascade
f32[SHADOW_CASCADES] g_cascade_texel;     // one shadow texel in world units
f32[SHADOW_CASCADES] g_cascade_bias;      // residual comparison bias, depth units

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

// Clip depth to window depth for the target: D3D and Metal rasterize
// clip z in [0,1] as is, GL maps [-1,1] onto it. The impostors write
// their depth from the fragment stage and need the same mapping.
float4 clip_depth_remap() {
    f32 scale = 1.0f;
    f32 offset = 0.0f;
    when os(linux) || os(wasm) || os(android) {
        scale = 0.5f;
        offset = 0.5f;
    }
    return float4{scale, offset, 0.0f, 0.0f};
}

// upstream gfx/shadow.{h,c}. The split range is a pair of view-space
// distances the cascades are distributed over. The near end sits
// nowhere near the camera near plane on purpose: it says where the
// split distribution starts, not where shadows start. The first
// cascade's bounding sphere reaches back past the eye whatever this
// is, so close content is covered either way. Starting at working
// distance puts the whole volume being looked at inside the first
// cascade, rather than spending it on the metre in front of the lens
// and leaving two boundaries in the middle of the scene.
const f32 SHADOW_SPLIT_NEAR = 10.0f;
const f32 SHADOW_SPLIT_FAR = 50.0f;
// Ceiling on widening the far end to a scene. The last cascade always
// has to reach the far split, so its texel size is set by this number
// alone. A scene that needs more reach asks for it in its SampleDef
// rather than having it inferred from how big its ground is.
const f32 SHADOW_SPLIT_FAR_MAX = 80.0f;
// Blend between the uniform split scheme (each cascade covers an equal
// slice of distance) and the logarithmic one (an equal multiplicative
// step). 0.5 is the standard sweet spot; raising it tightens the near
// cascades but drags every boundary toward the camera.
const f32 PSSM_LAMBDA = 0.5f;
// Room on the light's near side for casters standing above a cascade slice.
const f32 CASTER_MARGIN = 50.0f;
// What remains of the comparison bias once the lookup is offset along
// the surface normal: it only has to cover depth quantization, so it
// is measured in texels of the cascade that owns it.
const f32 DEPTH_BIAS_TEXELS = 0.5f;
// The sun, normalize(0.5, 0.8, 0.4), shared with the lit shader.
const f32 SUN_X = 0.4880f;
const f32 SUN_Y = 0.7807f;
const f32 SUN_Z = 0.3904f;

f32 g_shadow_split_near = SHADOW_SPLIT_NEAR;
f32 g_shadow_split_far = SHADOW_SPLIT_FAR;
f32 g_shadow_split_lambda = PSSM_LAMBDA;

void set_shadow_splits(f32 nearZ, f32 farZ) {
    if nearZ <= 0.0f || farZ <= nearZ {
        nearZ = SHADOW_SPLIT_NEAR;
        farZ = SHADOW_SPLIT_FAR;
    }
    g_shadow_split_near = nearZ;
    g_shadow_split_far = farZ;
}

void set_shadow_split_lambda(f32 lambda) {
    g_shadow_split_lambda = b3ClampFloat(lambda, 0.0f, 1.0f);
}

// Boundary i of the split scheme, i in 0..SHADOW_CASCADES.
f32 split_distance(i32 i) {
    f32 nearZ = g_shadow_split_near;
    f32 farZ = g_shadow_split_far;
    f32 fi = cast(f32, i) / cast(f32, SHADOW_CASCADES);
    f32 u = nearZ + (farZ - nearZ) * fi;
    f32 l = nearZ * powf(farZ / nearZ, fi);
    return g_shadow_split_lambda * l + (1.0f - g_shadow_split_lambda) * u;
}

// Bound the view slice between near and far by a sphere: centroid of
// its eight corners plus the largest distance to any of them. The
// radius depends only on the slice depths and the field of view, not
// on where the camera points, so texel size holds still as the camera
// turns. Eye-relative, like everything else the GPU sees: the eye is
// the draw origin, so it contributes nothing here.
void slice_bounding_sphere(f32 nearZ, f32 farZ, float3* centreOut, f32* radiusOut) {
    f32 tanHalf = tanf(0.5f * 60.0f * PI_F / 180.0f);
    f32 w = sapp_widthf();
    f32 h = sapp_heightf();
    f32 aspect = h > 0.0f ? w / h : 1.0f;
    float3 vdir = float3{0.0f - cam_forward.x, 0.0f - cam_forward.y, 0.0f - cam_forward.z};

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
    *centreOut = centre;
    *radiusOut = radius;
}

// Fit one cascade to the view slice between near and far.
//
// Texel snapping: the sphere centre is quantized onto a world-anchored
// lattice aligned with the light's own axes, one texel of world per
// step, and the light matrices are built around the snapped centre.
// Camera motion then slides the cascade in whole texels, so shadow
// edges step instead of crawling. It has to happen on the centre and
// not by projecting it through the finished matrix: the centre is the
// look-at target, so it lands on clip (0, 0) by construction.
void fit_cascade(i32 index, f32 nearZ, f32 farZ) {
    float3 sun = float3{SUN_X, SUN_Y, SUN_Z};
    float3 centre;
    f32 radius;
    slice_bounding_sphere(nearZ, farZ, &centre, &radius);

    float3 up = float3{0.0f, 1.0f, 0.0f};
    if sun.y > 0.99f { up = float3{0.0f, 0.0f, 1.0f}; }

    // Light axes, matching the basis look_at derives below. The eye
    // sits along +sun from the target, so the light looks along -sun.
    // These depend only on the sun and the up choice, never on the
    // camera, which is what makes the lattice hold still.
    float3 fwd = float3{0.0f - sun.x, 0.0f - sun.y, 0.0f - sun.z};
    float3 lr = float3{fwd.y * up.z - fwd.z * up.y,
                       fwd.z * up.x - fwd.x * up.z,
                       fwd.x * up.y - fwd.y * up.x};
    f32 lrl = sqrtf(lr.x * lr.x + lr.y * lr.y + lr.z * lr.z);
    lr = float3{lr.x / lrl, lr.y / lrl, lr.z / lrl};
    float3 lu = float3{lr.y * fwd.z - lr.z * fwd.y,
                       lr.z * fwd.x - lr.x * fwd.z,
                       lr.x * fwd.y - lr.y * fwd.x};

    // Radius is fixed for a given split, field of view and aspect, so
    // the lattice step is fixed too and the same world points keep
    // landing on the same texels frame to frame.
    f32 texelWorld = 2.0f * radius / cast(f32, SHADOW_DIM);

    // Geometry reaches the GPU shifted against the draw origin, which
    // tracks the camera, so the frame slides whenever the camera does.
    // Anchoring the lattice there would quantize against a moving
    // reference, which is the crawl the snap exists to remove. Fold the
    // eye back in and snap the absolute coordinate, in double so the
    // fraction within a texel survives far from the origin.
    f64 ox = cast(f64, lr.x) * cast(f64, cam_eye.x) + cast(f64, lr.y) * cast(f64, cam_eye.y)
        + cast(f64, lr.z) * cast(f64, cam_eye.z);
    f64 oy = cast(f64, lu.x) * cast(f64, cam_eye.x) + cast(f64, lu.y) * cast(f64, cam_eye.y)
        + cast(f64, lu.z) * cast(f64, cam_eye.z);
    f64 step = cast(f64, texelWorld);
    f64 ax = ox + cast(f64, lr.x * centre.x + lr.y * centre.y + lr.z * centre.z);
    f64 ay = oy + cast(f64, lu.x * centre.x + lu.y * centre.y + lu.z * centre.z);
    f32 sx = cast(f32, round(ax / step) * step - ax);
    f32 sy = cast(f32, round(ay / step) * step - ay);
    centre = float3{centre.x + sx * lr.x + sy * lu.x,
                    centre.y + sx * lr.y + sy * lu.y,
                    centre.z + sx * lr.z + sy * lu.z};

    // The light eye sits CASTER_MARGIN in front of the sphere, and the
    // depth range ends at its back face: casters up to CASTER_MARGIN
    // above the slice reach the map, and no depth range is spent
    // behind it.
    f32 back = radius + CASTER_MARGIN;
    float3 eye = float3{centre.x + sun.x * back,
                        centre.y + sun.y * back,
                        centre.z + sun.z * back};
    float4x4 view = look_at(eye, centre, up);
    f32 depthRange = 2.0f * radius + CASTER_MARGIN;
    float4x4 proj = ortho_proj(radius, radius, 0.0f, depthRange);

    // the depth pass rasterizes in clip space; the lookup uses texture
    // space
    g_light_clip[index] = mul(proj, view);
    g_light_mat[index] = mul(clip_to_texture(), g_light_clip[index]);
    g_cascade_far[index] = farZ;
    g_cascade_texel[index] = texelWorld;
    // Texture depth spans the whole orthographic range, and the caster
    // margin inflates that range without covering any more receiver.
    // Sizing the bias in world units here decouples it, so the margin
    // costs caster headroom and nothing else.
    g_cascade_bias[index] = (DEPTH_BIAS_TEXELS * texelWorld) / depthRange;
}

// Split the shadowed range into cascades and fit each.
void update_light_matrix() {
    for i32 i = 0; i < SHADOW_CASCADES; i++ {
        fit_cascade(i, split_distance(i), split_distance(i + 1));
    }
}

// Eye-relative box enclosing every shape that can cast into the
// cascades: each slice's bounding sphere swept toward the sun by the
// caster margin. It reaches well outside the camera frustum, which is
// the point. A draw set culled to the visible volume drops casters
// that belong in the shadow map, and their shadows vanish. Derived
// from the camera alone, so it can be queried before the fit runs.
void shadow_caster_bounds(float3* loOut, float3* hiOut) {
    float3 lo = float3{FLT_MAX, FLT_MAX, FLT_MAX};
    float3 hi = float3{0.0f - FLT_MAX, 0.0f - FLT_MAX, 0.0f - FLT_MAX};
    for i32 i = 0; i < SHADOW_CASCADES; i++ {
        float3 centre;
        f32 radius;
        slice_bounding_sphere(split_distance(i), split_distance(i + 1), &centre, &radius);
        // A caster can only darken a receiver in this sphere if it sits
        // between the two, so sweep the sphere toward the sun rather
        // than growing it in every direction. Sweeping by the caster
        // margin lands the front face exactly on the cascade's near
        // plane.
        float3 swept = float3{centre.x + CASTER_MARGIN * SUN_X,
                              centre.y + CASTER_MARGIN * SUN_Y,
                              centre.z + CASTER_MARGIN * SUN_Z};
        lo.x = b3MinFloat(lo.x, b3MinFloat(centre.x, swept.x) - radius);
        lo.y = b3MinFloat(lo.y, b3MinFloat(centre.y, swept.y) - radius);
        lo.z = b3MinFloat(lo.z, b3MinFloat(centre.z, swept.z) - radius);
        hi.x = b3MaxFloat(hi.x, b3MaxFloat(centre.x, swept.x) + radius);
        hi.y = b3MaxFloat(hi.y, b3MaxFloat(centre.y, swept.y) + radius);
        hi.z = b3MaxFloat(hi.z, b3MaxFloat(centre.z, swept.z) + radius);
    }
    *loOut = lo;
    *hiOut = hi;
}

// --- rendering ------------------------------------------------------

sg_pipeline g_pip_sky;
sg_buffer g_sky_vbuf;
sg_buffer g_proxy_vbuf;

// Solids. Meshes draw their vertices; spheres and capsules draw the
// proxy cube through the impostor pipelines.
sg_pipeline g_pip_inst;
sg_pipeline g_pip_inst_mirror;
sg_pipeline g_pip_imp;
sg_pipeline g_pip_inst_depth;
sg_pipeline g_pip_imp_depth;

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
