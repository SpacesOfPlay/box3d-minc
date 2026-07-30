// Manifold scenes. Ports of samples/sample_manifold.cpp.
//
// Upstream shares one base class across the category: it owns the two
// transforms, the drag-and-rotate mouse handling, and the render that
// labels every manifold point. Here that base is the mf_* helpers, and
// each scene supplies its shapes and the collide call.

import box3d;
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

const i32 MF_POINT_CAPACITY = 64;
b3LocalManifold g_mf_manifold;
b3LocalManifoldPoint[MF_POINT_CAPACITY] g_mf_points;
b3WorldTransform g_mf_transform_a;
b3WorldTransform g_mf_transform_b;
b3Pos g_mf_base_translation;
b3Quat g_mf_base_quaternion;
b3Pos g_mf_origin;
b3SimplexCache g_mf_simplex_cache;
b3SATCache g_mf_sat_cache;
i32 g_mf_manual_feature;
i32 g_mf_base_x;
i32 g_mf_base_y;
bool g_mf_use_cache;
bool g_mf_tracking;
bool g_mf_rotating;

void mf_init() {
    memset(cast(void*, &g_mf_points), 0,
           cast(i64, MF_POINT_CAPACITY * cast(i32, sizeof(b3LocalManifoldPoint))));
    g_mf_manifold = b3LocalManifold{};
    g_mf_manifold.points = cast(b3LocalManifoldPoint*, &g_mf_points);

    g_mf_transform_a.p = b3Pos{3.5f, 0.5f, 0.0f};
    g_mf_transform_a.q = b3MakeQuatFromAxisAngle(b3Vec3{0.0f, 1.0f, 0.0f}, 0.5f * PI_F);

    g_mf_transform_b.p = b3Pos{0.0f, 1.5f, 3.5f};
    g_mf_transform_b.q = b3Quat_identity;

    g_mf_simplex_cache = b3SimplexCache{};
    g_mf_sat_cache = b3SATCache{};
    g_mf_manual_feature = 0;
    g_mf_base_translation = b3Pos_zero;
    g_mf_base_quaternion = b3Quat_identity;
    g_mf_base_x = 0;
    g_mf_base_y = 0;
    g_mf_origin = b3Pos_zero;
    g_mf_use_cache = false;
    g_mf_tracking = false;
    g_mf_rotating = false;
}

// upstream Manifold::Render, run after each scene has drawn its shapes.
void mf_render() {
    u8[128] buf;
    ignore snprintf(cast(u8*, &buf), 128, "origin: %g %g %g", cast(f64, g_mf_origin.x),
                    cast(f64, g_mf_origin.y), cast(f64, g_mf_origin.z));
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 128, "count = %d", g_mf_manifold.pointCount);
    draw_text_line(cast(u8*, &buf));

    dbg_axes(b3WorldTransform_identity, 1.0f);

    if g_mf_manifold.pointCount == 0 {
        return;
    }

    f32 length = 0.5f * b3GetLengthUnitsPerMeter();
    b3Vec3 normal = b3RotateVector(g_mf_transform_a.q, g_mf_manifold.normal);

    for i32 pointIndex = 0; pointIndex < g_mf_manifold.pointCount; pointIndex += 1 {
        b3LocalManifoldPoint* manifoldPoint = g_mf_manifold.points + pointIndex;

        b3Pos point = b3TransformWorldPoint(g_mf_transform_a, manifoldPoint.point);

        dbg_line(point, b3OffsetPos(point, b3MulSV(length, normal)), b3_colorWhite);

        if manifoldPoint.separation > 0.0f {
            adapter_point(point, 10.0f, b3_colorWhite, null);
        } else {
            adapter_point(point, 10.0f, b3_colorYellow, null);
        }

        ignore snprintf(cast(u8*, &buf), 128, "   %.3f", cast(f64, manifoldPoint.separation));
        dbg_string_3d(point, make_color(b3_colorWhite), cast(u8*, &buf));

        b3Vec3 perp = b3Perp(normal);
        b3FeaturePair pair = manifoldPoint.pair;
        ignore snprintf(cast(u8*, &buf), 128, "  %X:%X %X:%X", pair.owner1, pair.index1,
                        pair.owner2, pair.index2);
        dbg_string_3d(b3OffsetPos(b3OffsetPos(point, b3MulSV(0.025f, normal)),
                                  b3MulSV(0.05f, perp)),
                      make_color(b3_colorPapayaWhip), cast(u8*, &buf));
    }
}

bool mf_controls() {
    ignore ImGui_Checkbox("Use cache", &g_mf_use_cache);
    if g_mf_use_cache {
        ignore ImGui_RadioButton("auto", &g_mf_manual_feature, 0);
        ignore ImGui_RadioButton("faceA", &g_mf_manual_feature, 1);
        ignore ImGui_RadioButton("faceB", &g_mf_manual_feature, 2);
        ignore ImGui_RadioButton("edgePair", &g_mf_manual_feature, 3);
    }
    return true;
}

bool mf_mouse_down(f32 px, f32 py, i32 button, i32 modifiers) {
    if button == 0 && (modifiers & 4) == 0 {
        if (modifiers & 1) != 0 {
            g_mf_base_x = cast(i32, px);
            g_mf_base_y = cast(i32, py);
            g_mf_base_quaternion = g_mf_transform_b.q;
            g_mf_rotating = true;
        } else {
            PickRay pickRay = build_pick_ray(px, py);
            b3Vec3 dir = b3Normalize(b3Vec3{pickRay.translation.x, pickRay.translation.y,
                                            pickRay.translation.z});
            g_mf_origin = b3OffsetPos(b3Pos{pickRay.origin.x, pickRay.origin.y,
                                            pickRay.origin.z}, b3MulSV(10.0f, dir));
            g_mf_base_translation = g_mf_transform_b.p;
            g_mf_tracking = true;
        }
        return true;
    }
    return false;
}

void mf_mouse_up(f32 px, f32 py, i32 button) {
    ignore px; ignore py; ignore button;
    g_mf_tracking = false;
    g_mf_rotating = false;
}

void mf_mouse_move(f32 px, f32 py) {
    if g_mf_tracking {
        PickRay pickRay = build_pick_ray(px, py);
        b3Vec3 dir = b3Normalize(b3Vec3{pickRay.translation.x, pickRay.translation.y,
                                        pickRay.translation.z});
        b3Pos origin = b3OffsetPos(b3Pos{pickRay.origin.x, pickRay.origin.y, pickRay.origin.z},
                                   b3MulSV(10.0f, dir));
        g_mf_transform_b.p = b3OffsetPos(g_mf_base_translation, b3SubPos(origin, g_mf_origin));
    }

    if g_mf_rotating {
        i32 x = cast(i32, px);
        i32 y = cast(i32, py);

        b3Quat qx = b3MakeQuatFromAxisAngle(b3Vec3_axisY, 0.01f * cast(f32, x - g_mf_base_x));
        b3Quat qz = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, 0.01f * cast(f32, y - g_mf_base_y));
        g_mf_transform_b.q = b3NormalizeQuat(b3MulQuat(g_mf_base_quaternion,
                                                       b3MulQuat(qx, qz)));
    }
}

// samples/sample_manifold.cpp SphereAndSphere
b3Sphere g_ss_sphere;

void build_sphere_and_sphere() {
    mf_init();
    g_ss_sphere = b3Sphere{b3Pos{0.5f, 0.0f, -0.25f}, 2.0f};
}

void step_sphere_and_sphere(f32 timeStep) {
    ignore timeStep;
    b3Transform transformBtoA = b3InvMulWorldTransforms(g_mf_transform_a, g_mf_transform_b);
    b3CollideSpheres(&g_mf_manifold, MF_POINT_CAPACITY, &g_ss_sphere, &g_ss_sphere,
                     transformBtoA);

    dbg_solid_sphere(g_mf_transform_a, g_ss_sphere, make_color(b3_colorGreen));
    dbg_solid_sphere(g_mf_transform_b, g_ss_sphere, make_color(b3_colorCyan));
    mf_render();
}

// samples/sample_manifold.cpp CapsuleAndSphere
b3Sphere g_cs_sphere;
b3Capsule g_cs_capsule;

void build_capsule_and_sphere() {
    mf_init();
    g_cs_capsule = b3Capsule{b3Pos{-2.0f, 0.0f, 0.0f}, b3Pos{2.0f, 0.0f, 0.0f}, 1.0f};
    g_cs_sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, 2.0f};

    g_mf_transform_a.p = b3Pos{0.0f, 0.0f, 0.0f};
    g_mf_transform_a.q = b3Quat_identity;
    g_mf_transform_b.p = b3Pos{-4.0f, 0.0f, 0.0f};
    g_mf_transform_b.q = b3Quat_identity;
}

void step_capsule_and_sphere(f32 timeStep) {
    ignore timeStep;
    b3Transform transformBtoA = b3InvMulWorldTransforms(g_mf_transform_a, g_mf_transform_b);
    b3CollideCapsuleAndSphere(&g_mf_manifold, MF_POINT_CAPACITY, &g_cs_capsule, &g_cs_sphere,
                              transformBtoA);

    dbg_solid_capsule(g_mf_transform_a, g_cs_capsule, make_color(b3_colorCyan));
    dbg_solid_sphere(g_mf_transform_b, g_cs_sphere, make_color(b3_colorGreen));
    mf_render();
}

// samples/sample_manifold.cpp CapsuleAndCapsule
b3Capsule g_cc_capsule;

void build_capsule_and_capsule() {
    mf_init();
    g_cc_capsule = b3Capsule{b3Pos{-2.0f, 0.0f, 0.0f}, b3Pos{2.0f, 0.0f, 0.0f}, 1.0f};

    g_mf_transform_a.p = b3Pos{1.0f, 1.0f, 0.0f};
    g_mf_transform_a.q = b3Quat_identity;
    g_mf_transform_b.p = b3Pos{-4.0f, 1.0f, 0.0f};
    g_mf_transform_b.q = b3Quat_identity;
}

void step_capsule_and_capsule(f32 timeStep) {
    ignore timeStep;
    b3Transform transformBtoA = b3InvMulWorldTransforms(g_mf_transform_a, g_mf_transform_b);
    b3CollideCapsules(&g_mf_manifold, MF_POINT_CAPACITY, &g_cc_capsule, &g_cc_capsule,
                      transformBtoA);

    dbg_solid_capsule(g_mf_transform_a, g_cc_capsule, make_color(b3_colorGreen));
    dbg_solid_capsule(g_mf_transform_b, g_cc_capsule, make_color(b3_colorCyan));
    mf_render();
}

// samples/sample_manifold.cpp TriangleManifold
//
// The second base in the file: same drag and rotate, but the manifold is
// reported in frame B and the triangle itself is drawn with its winding
// labelled and its normal arrowed.
b3Vec3[3] g_tmf_triangle;

void tmf_init() {
    mf_init();
}

void tmf_render() {
    u8[128] buf;
    ignore snprintf(cast(u8*, &buf), 128, "origin: %g %g %g", cast(f64, g_mf_origin.x),
                    cast(f64, g_mf_origin.y), cast(f64, g_mf_origin.z));
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 128, "count = %d", g_mf_manifold.pointCount);
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 128, "feature = %d", g_mf_manifold.feature);
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 128, "cache hit = %d", cast(i32, g_mf_sat_cache.hit));
    draw_text_line(cast(u8*, &buf));

    dbg_axes(b3WorldTransform_identity, 1.0f);

    if g_mf_manifold.pointCount > 0 {
        f32 length = 0.5f * b3GetLengthUnitsPerMeter();
        b3Vec3 normal = b3RotateVector(g_mf_transform_b.q, g_mf_manifold.normal);
        for i32 pointIndex = 0; pointIndex < g_mf_manifold.pointCount; pointIndex += 1 {
            b3LocalManifoldPoint* manifoldPoint = g_mf_manifold.points + pointIndex;
            b3Pos point = b3TransformWorldPoint(g_mf_transform_b, manifoldPoint.point);
            dbg_line(point, b3OffsetPos(point, b3MulSV(length, normal)), b3_colorWhite);
            if manifoldPoint.separation > 0.0f {
                adapter_point(point, 10.0f, b3_colorWhite, null);
            } else {
                adapter_point(point, 10.0f, b3_colorYellow, null);
            }
            ignore snprintf(cast(u8*, &buf), 128, "  %.2f",
                            cast(f64, 100.0f * manifoldPoint.separation));
            dbg_string_3d(point, make_color(b3_colorWhite), cast(u8*, &buf));
            b3FeaturePair pair = manifoldPoint.pair;
            ignore snprintf(cast(u8*, &buf), 128, "  %X:%X %X:%X", pair.owner1, pair.index1,
                            pair.owner2, pair.index2);
            dbg_string_3d(b3OffsetPos(point, b3MulSV(0.025f, normal)),
                          make_color(b3_colorPapayaWhip), cast(u8*, &buf));
        }
    }

    dbg_triangle(g_mf_transform_a, g_tmf_triangle[0], g_tmf_triangle[1], g_tmf_triangle[2],
                 b3_colorCyan);
    b3Pos p1 = b3TransformWorldPoint(g_mf_transform_a, g_tmf_triangle[0]);
    b3Pos p2 = b3TransformWorldPoint(g_mf_transform_a, g_tmf_triangle[1]);
    b3Pos p3 = b3TransformWorldPoint(g_mf_transform_a, g_tmf_triangle[2]);
    dbg_string_3d(p1, make_color(b3_colorWhite), "0");
    dbg_string_3d(p2, make_color(b3_colorWhite), "1");
    dbg_string_3d(p3, make_color(b3_colorWhite), "2");
    b3Vec3 e1 = b3SubPos(p2, p1);
    b3Vec3 e2 = b3SubPos(p3, p1);
    b3Vec3 normal = b3Normalize(b3Cross(e1, e2));
    b3Pos center = b3OffsetPos(p1, b3MulSV(1.0f / 3.0f, b3Add(e1, e2)));
    dbg_arrow(center, b3OffsetPos(center, b3MulSV(0.5f, normal)), b3_colorMediumPurple);
}

// The manual-feature radio drives the SAT cache in the hull scenes.
void mf_apply_sat_cache() {
    if g_mf_use_cache == true {
        if g_mf_manual_feature == 1 {
            g_mf_sat_cache.type = b3_manualFaceAxisA;
        } else if g_mf_manual_feature == 2 {
            g_mf_sat_cache.type = b3_manualFaceAxisB;
        } else if g_mf_manual_feature == 3 {
            g_mf_sat_cache.type = b3_manualEdgePairAxis;
        }
    } else {
        g_mf_sat_cache = b3SATCache{};
    }
}

// samples/sample_manifold.cpp HullAndSphere
b3Sphere g_hs_sphere;
b3BoxHull g_hs_hull;

void build_hull_and_sphere() {
    mf_init();
    g_hs_sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, 1.0f};
    g_hs_hull = b3MakeBoxHull(2.0f, 0.5f, 0.5f);
    g_mf_transform_a.p = b3Pos{0.0f, 0.0f, 0.0f};
    g_mf_transform_a.q = b3Quat_identity;
    g_mf_transform_b.p = b3Pos{1.5f, 0.0f, 0.0f};
    g_mf_transform_b.q = b3Quat_identity;
}

void step_hull_and_sphere(f32 timeStep) {
    ignore timeStep;
    if g_mf_use_cache == false {
        g_mf_simplex_cache = b3SimplexCache{};
    }
    b3Transform transformBtoA = b3InvMulWorldTransforms(g_mf_transform_a, g_mf_transform_b);
    b3CollideHullAndSphere(&g_mf_manifold, MF_POINT_CAPACITY, &g_hs_hull.base, &g_hs_sphere,
                           transformBtoA, &g_mf_simplex_cache);

    dbg_hull(g_mf_transform_a, &g_hs_hull.base, b3_colorCyan);
    dbg_solid_sphere(g_mf_transform_b, g_hs_sphere, make_color(b3_colorGreen));
    mf_render();
}

// samples/sample_manifold.cpp CapsuleAndHull
b3BoxHull g_ch_hull;
b3Capsule g_ch_capsule;

void build_capsule_and_hull() {
    mf_init();
    g_ch_capsule = b3Capsule{b3Pos{-1.0f, 0.0f, 0.0f}, b3Pos{1.0f, 0.0f, 0.0f}, 0.15f};
    g_ch_hull = b3MakeBoxHull(1.0f, 0.5f, 0.5f);
    g_mf_transform_a.p = b3Pos{0.0f, 0.0f, 0.0f};
    g_mf_transform_a.q = b3Quat_identity;
    // m_capsule = {
    //	{ 0.0799999982, -0.0151330000, -0.0918010026 }, { -0.0799999982, -0.0151330000, -0.0918010026 }, 0.100000001 };
    // m_hull = b3CreateBox( { 0.5f, 1.0f, 1.5f } );
    g_mf_transform_b.p = b3Pos{1.58523774f, 0.729615569f, 0.451690674f};
    g_mf_transform_b.q = b3Quat{b3Vec3{-0.00256555085f, -0.0201825816f, 0.126076236f},
                                0.991811991f};
}

void step_capsule_and_hull(f32 timeStep) {
    ignore timeStep;
    if g_mf_use_cache == false {
        g_mf_simplex_cache = b3SimplexCache{};
    }
    b3Transform transformBtoA = b3InvMulWorldTransforms(g_mf_transform_a, g_mf_transform_b);
    b3CollideHullAndCapsule(&g_mf_manifold, MF_POINT_CAPACITY, &g_ch_hull.base, &g_ch_capsule,
                            transformBtoA, &g_mf_simplex_cache);

    dbg_hull(g_mf_transform_a, &g_ch_hull.base, b3_colorCyan);
    dbg_solid_capsule(g_mf_transform_b, g_ch_capsule, make_color(b3_colorGreen));
    mf_render();
}

// samples/sample_manifold.cpp HullAndHull
b3BoxHull g_hh_box_a;
b3BoxHull g_hh_box_b;

void build_hull_and_hull() {
    mf_init();
    // m_boxA = b3MakeTransformedBoxHull( 0.5f, 0.5f, 0.5f, { { 0.0f, -0.5f, 0.0f }, b3Quat_identity } );
    b3Transform transform = b3Transform{b3Vec3{1.0f, 0.5f, 0.0f}, b3Quat_identity};
    g_hh_box_a = b3MakeTransformedBoxHull(0.5f, 1.0f, 1.0f, transform);
    g_hh_box_b = b3MakeBoxHull(0.5f, 0.5f, 0.5f);
    g_mf_transform_a.p = b3Pos{0.0f, 0.0f, 0.0f};
    g_mf_transform_a.q = b3Quat_identity;
    g_mf_transform_b.p = b3Pos{0.0f, 0.0f, 0.0f};
    g_mf_transform_b.q = b3Quat_identity;
    g_mf_sat_cache = b3SATCache{};
}

void step_hull_and_hull(f32 timeStep) {
    ignore timeStep;
    mf_apply_sat_cache();
    b3Transform transformBtoA = b3InvMulWorldTransforms(g_mf_transform_a, g_mf_transform_b);
    b3CollideHulls(&g_mf_manifold, MF_POINT_CAPACITY, &g_hh_box_a.base, &g_hh_box_b.base,
                   transformBtoA, &g_mf_sat_cache);
    u8[64] buf;
    ignore snprintf(cast(u8*, &buf), 64, "SAT type: %d", cast(i32, g_mf_sat_cache.type));
    draw_text_line(cast(u8*, &buf));

    dbg_hull(g_mf_transform_a, &g_hh_box_a.base, b3_colorGreen);
    dbg_hull(g_mf_transform_b, &g_hh_box_b.base, b3_colorCyan);
    mf_render();
}

// samples/sample_manifold.cpp TriangleAndSphere
b3Sphere g_ts_sphere;

void build_triangle_and_sphere() {
    tmf_init();
    g_ts_sphere = b3Sphere{b3Pos{0.0f, 0.0f, 0.0f}, 0.25f};
    g_tmf_triangle[0] = b3Vec3{0.0f, 0.0f, 0.0f};
    g_tmf_triangle[1] = b3Vec3{4.0f, 0.0f, 4.0f};
    g_tmf_triangle[2] = b3Vec3{4.0f, 0.0f, 0.0f};
    // b3Quat qA = b3MakeQuatFromAxisAngle( { 0.0f, 1.0f, 0.0f }, 2.0f );
    // m_transformA = { { 1.0f, 1.0f, 0.0f }, qA };
    g_mf_transform_a = b3WorldTransform_identity;
    g_mf_transform_b.p = b3Pos{2.0f, 0.5f, 1.0f};
    g_mf_transform_b.q = b3Quat_identity;
}

void step_triangle_and_sphere(f32 timeStep) {
    ignore timeStep;
    // Convert triangle to frame B
    b3Transform xf = b3InvMulWorldTransforms(g_mf_transform_b, g_mf_transform_a);
    b3Vec3[3] localTriangle;
    localTriangle[0] = b3TransformPoint(xf, g_tmf_triangle[0]);
    localTriangle[1] = b3TransformPoint(xf, g_tmf_triangle[1]);
    localTriangle[2] = b3TransformPoint(xf, g_tmf_triangle[2]);
    b3CollideTriangleAndSphere(&g_mf_manifold, MF_POINT_CAPACITY,
                               cast(b3Vec3*, &localTriangle), &g_ts_sphere);

    dbg_solid_sphere(g_mf_transform_b, g_ts_sphere, make_color_alpha(b3_colorGreen, 0.5f));
    tmf_render();
}

// samples/sample_manifold.cpp TriangleAndCapsule
b3Capsule g_tc_capsule;

void build_triangle_and_capsule() {
    tmf_init();
    g_tc_capsule = b3Capsule{b3Pos{-0.5f, 0.0f, 0.0f}, b3Pos{0.5f, 0.0f, 0.0f}, 0.05f};
    g_tmf_triangle[0] = b3Vec3{-4.0f, 0.0f, -4.0f};
    g_tmf_triangle[1] = b3Vec3{-4.0f, 0.0f, 0.0f};
    g_tmf_triangle[2] = b3Vec3{0.0f, 0.0f, 0.0f};
    g_mf_transform_a = b3WorldTransform_identity;
    g_mf_transform_b.p = b3Pos{-1.0f, 0.0f, -1.0f};
    g_mf_transform_b.q = b3Quat_identity;
}

void step_triangle_and_capsule(f32 timeStep) {
    ignore timeStep;
    if g_mf_use_cache == false {
        g_mf_simplex_cache = b3SimplexCache{};
    }
    // Put triangle into capsule coordinates
    b3Transform xf = b3InvMulWorldTransforms(g_mf_transform_b, g_mf_transform_a);
    b3Vec3[3] localTriangle;
    localTriangle[0] = b3TransformPoint(xf, g_tmf_triangle[0]);
    localTriangle[1] = b3TransformPoint(xf, g_tmf_triangle[1]);
    localTriangle[2] = b3TransformPoint(xf, g_tmf_triangle[2]);
    b3CollideTriangleAndCapsule(&g_mf_manifold, MF_POINT_CAPACITY,
                                cast(b3Vec3*, &localTriangle), &g_tc_capsule,
                                &g_mf_simplex_cache);

    dbg_solid_capsule(g_mf_transform_b, g_tc_capsule, make_color_alpha(b3_colorGreen, 0.5f));
    dbg_axes(g_mf_transform_b, 0.1f);
    tmf_render();
}

// samples/sample_manifold.cpp TriangleAndHull
i32 g_th_flags;
b3BoxHull g_th_box_hull;
b3HullData* g_th_cylinder;

void build_triangle_and_hull() {
    tmf_init();
    //m_triangle[0] = { 1.00000000, 0, 1.00000000 };
    //m_triangle[1] = { 1.00000000, 0, 0.00000000 };
    //m_triangle[2] = { 0.00000000, 0, 0.00000000 };
    g_tmf_triangle[0] = b3Vec3{0.299769998f, -1.01549578f, -0.744717002f};
    g_tmf_triangle[1] = b3Vec3{0.299769998f, -1.01549578f, 1.28728306f};
    g_tmf_triangle[2] = b3Vec3{0.299769998f, -0.913895786f, 0.271283031f};
    f32 bodyHalfWidth = 0.304800004f;
    f32 bodyHalfHeight = 0.914399981f;
    g_th_box_hull = b3MakeBoxHull(bodyHalfWidth, bodyHalfHeight, bodyHalfWidth);
    g_mf_transform_a = b3WorldTransform_identity;
    g_mf_transform_b = b3WorldTransform_identity;
    //m_transformB.p = { -2.16650009f, 0.912535489f, 0.00000000f };
    // b3MeshEdgeFlags
    g_th_flags = 0;
    g_mf_sat_cache = b3SATCache{};
    g_mf_use_cache = false;
    g_th_cylinder = b3CreateCylinder(0.4f, 0.05f, 0.0f, 6);
}

void destroy_triangle_and_hull() {
    b3DestroyHull(g_th_cylinder);
}

void step_triangle_and_hull(f32 timeStep) {
    ignore timeStep;
    mf_apply_sat_cache();
    b3Transform xf = b3InvMulWorldTransforms(g_mf_transform_b, g_mf_transform_a);
    b3Vec3[3] localTriangle;
    localTriangle[0] = b3TransformPoint(xf, g_tmf_triangle[0]);
    localTriangle[1] = b3TransformPoint(xf, g_tmf_triangle[1]);
    localTriangle[2] = b3TransformPoint(xf, g_tmf_triangle[2]);
    b3CollideTriangleAndHull(&g_mf_manifold, MF_POINT_CAPACITY, localTriangle[0],
                             localTriangle[1], localTriangle[2], g_th_flags,
                             &g_th_box_hull.base, &g_mf_sat_cache, true);

    dbg_hull(g_mf_transform_b, &g_th_box_hull.base, b3_colorGreen);
    b3WorldTransform xf2;
    xf2.p = b3TransformWorldPoint(g_mf_transform_b, g_th_box_hull.base.center);
    xf2.q = g_mf_transform_b.q;
    dbg_axes(xf2, 0.1f);
    tmf_render();
}
