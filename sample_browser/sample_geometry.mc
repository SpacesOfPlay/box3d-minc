// Geometry scenes. Ports of samples/sample_geometry.cpp.

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

// samples/sample_geometry.cpp BoxHull
b3Vec3 g_bx_half_widths;
b3Vec3 g_bx_rotation;
b3Transform g_bx_transform;
b3Vec3 g_bx_post_scale;
b3BoxHull g_bx_box;
b3HullData* g_bx_hull;

void box_hull_update_rotation() {
    b3Quat qx = b3MakeQuatFromAxisAngle(b3Vec3_axisX, B3_DEG_TO_RAD * g_bx_rotation.x);
    b3Quat qy = b3MakeQuatFromAxisAngle(b3Vec3_axisY, B3_DEG_TO_RAD * g_bx_rotation.y);
    b3Quat qz = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, B3_DEG_TO_RAD * g_bx_rotation.z);

    g_bx_transform.q = b3MulQuat(qz, b3MulQuat(qy, qx));
}

void box_hull_create_hulls(b3Vec3 h, b3Transform transform, b3Vec3 postScale) {
    if g_bx_hull != null {
        b3DestroyHull(g_bx_hull);
        g_bx_hull = null;
    }

    b3Vec3 scale = b3SafeScale(postScale);
    b3Vec3[8] points;

    points[0] = b3Mul(scale, b3TransformPoint(transform, b3Vec3{h.x, h.y, h.z}));
    points[1] = b3Mul(scale, b3TransformPoint(transform, b3Vec3{h.x, h.y, -h.z}));
    points[2] = b3Mul(scale, b3TransformPoint(transform, b3Vec3{h.x, -h.y, h.z}));
    points[3] = b3Mul(scale, b3TransformPoint(transform, b3Vec3{h.x, -h.y, -h.z}));
    points[4] = b3Mul(scale, b3TransformPoint(transform, b3Vec3{-h.x, h.y, h.z}));
    points[5] = b3Mul(scale, b3TransformPoint(transform, b3Vec3{-h.x, h.y, -h.z}));
    points[6] = b3Mul(scale, b3TransformPoint(transform, b3Vec3{-h.x, -h.y, h.z}));
    points[7] = b3Mul(scale, b3TransformPoint(transform, b3Vec3{-h.x, -h.y, -h.z}));

    g_bx_hull = b3CreateHull(cast(b3Vec3*, &points), 8, 8);
    g_bx_box = b3MakeScaledBoxHull(h, transform, postScale);
}

void build_box_hull() {
    g_bx_hull = null;

    g_bx_half_widths = b3Vec3{1.0f, 0.5f, 0.25f};
    g_bx_post_scale = b3Vec3{1.0f, 1.0f, 1.0f};
    g_bx_rotation = b3Vec3{0.0f, 0.0f, 0.0f};
    g_bx_transform = b3Transform_identity;

    box_hull_update_rotation();
    box_hull_create_hulls(g_bx_half_widths, g_bx_transform, g_bx_post_scale);
}

void destroy_box_hull() {
    if g_bx_hull != null {
        b3DestroyHull(g_bx_hull);
        g_bx_hull = null;
    }
}

bool box_hull_controls() {
    f32 fontSize = ImGui_GetFontSize();
    ImGui_PushItemWidth(10.0f * fontSize);

    if ImGui_SliderFloat3("h", &g_bx_half_widths.x, 0.1f, 2.0f, "%.1f", 0) {
        box_hull_create_hulls(g_bx_half_widths, g_bx_transform, g_bx_post_scale);
    }

    if ImGui_SliderFloat3("c", &g_bx_transform.p.x, -2.0f, 2.0f, "%.1f", 0) {
        box_hull_create_hulls(g_bx_half_widths, g_bx_transform, g_bx_post_scale);
    }

    if ImGui_SliderFloat3("r", &g_bx_rotation.x, -180.0f, 180.0f, "%.0f", 0) {
        box_hull_update_rotation();
        box_hull_create_hulls(g_bx_half_widths, g_bx_transform, g_bx_post_scale);
    }

    if ImGui_SliderFloat3("s", &g_bx_post_scale.x, -2.0f, 2.0f, "%.1f", 0) {
        box_hull_create_hulls(g_bx_half_widths, g_bx_transform, g_bx_post_scale);
    }

    if ImGui_Button("Refresh", ImVec2{0.0f, 0.0f}) {
        box_hull_create_hulls(g_bx_half_widths, g_bx_transform, g_bx_post_scale);
    }

    ImGui_PopItemWidth();

    return true;
}

void step_box_hull(f32 timeStep) {
    ignore timeStep;
    dbg_hull(b3WorldTransform_identity, g_bx_hull, b3_colorYellow);
    dbg_hull(b3WorldTransform_identity, &g_bx_box.base, b3_colorCyan);

    dbg_axes(b3WorldTransform_identity, 1.0f);
}

// samples/sample_geometry.cpp Hull
const i32 GH_CAPACITY = 64;
b3HullData* g_gh_hull;
b3Vec3[GH_CAPACITY] g_gh_points;
i32 g_gh_count;

void build_geometry_hull() {
    g_gh_hull = null;

    // this fails because it generates too many edges
    b3Vec3[48] points;
    points[0] = b3Vec3{-3.9866004f, 75.4595108f, 28.3783073f};
    points[1] = b3Vec3{-13.1079493f, 73.080368f, 28.296587f};
    points[2] = b3Vec3{-18.6611958f, 72.0040894f, 16.9292431f};
    points[3] = b3Vec3{4.82537603f, 79.2908554f, 22.2369995f};
    points[4] = b3Vec3{-12.7315464f, 79.2187576f, 2.94275379f};
    points[5] = b3Vec3{-21.806488f, 78.7758865f, 0.985544085f};
    points[6] = b3Vec3{-27.7619209f, 73.3481522f, 11.9647141f};
    points[7] = b3Vec3{-22.3994541f, 72.2203826f, 21.4116211f};
    points[8] = b3Vec3{-25.3797474f, 76.7417755f, 27.9124985f};
    points[9] = b3Vec3{-22.7552319f, 77.0559006f, 29.4733639f};
    points[10] = b3Vec3{-6.81736374f, 78.3484726f, 36.8649979f};
    points[11] = b3Vec3{3.62397718f, 85.5270843f, 29.2077713f};
    points[12] = b3Vec3{7.90363788f, 84.121231f, 18.2612896f};
    points[13] = b3Vec3{-12.3809223f, 84.5280533f, -0.43230924f};
    points[14] = b3Vec3{5.83599472f, 95.2908325f, 4.4423275f};
    points[15] = b3Vec3{-22.5541401f, 89.9094467f, -4.87791252f};
    points[16] = b3Vec3{-43.9060402f, 78.5287094f, 1.32877088f};
    points[17] = b3Vec3{-42.6015129f, 76.7829742f, 7.67437983f};
    points[18] = b3Vec3{-25.735527f, 78.1218796f, 27.908411f};
    points[19] = b3Vec3{-23.5183544f, 77.6326675f, 29.1178799f};
    points[20] = b3Vec3{2.0977366f, 100.430191f, 34.3929482f};
    points[21] = b3Vec3{1.09743047f, 103.952553f, 35.5656395f};
    points[22] = b3Vec3{8.50175952f, 96.0529861f, 8.73674774f};
    points[23] = b3Vec3{2.52570295f, 103.303696f, 32.2314339f};
    points[24] = b3Vec3{-20.099781f, 89.4923248f, -4.15468454f};
    points[25] = b3Vec3{2.8092947f, 123.516098f, -1.12693477f};
    points[26] = b3Vec3{-43.9318161f, 79.1106186f, 1.39006138f};
    points[27] = b3Vec3{-23.358511f, 90.9599686f, -4.25683546f};
    points[28] = b3Vec3{2.10804915f, 123.603645f, -1.38435471f};
    points[29] = b3Vec3{-44.1329117f, 78.7192383f, 1.54941654f};
    points[30] = b3Vec3{-42.4365158f, 77.725357f, 8.14835929f};
    points[31] = b3Vec3{-43.204792f, 77.5811691f, 7.14319515f};
    points[32] = b3Vec3{-44.17416f, 78.7810363f, 2.50146222f};
    points[33] = b3Vec3{-32.8975143f, 99.1221771f, 7.55588436f};
    points[34] = b3Vec3{-0.624746263f, 110.070351f, 32.7381058f};
    points[35] = b3Vec3{0.00431228895f, 109.14341f, 33.6411133f};
    points[36] = b3Vec3{-0.58865279f, 122.980537f, 16.6554794f};
    points[37] = b3Vec3{2.18539238f, 124.324593f, -0.620266676f};
    points[38] = b3Vec3{-1.02177501f, 123.881721f, 16.8230057f};
    points[39] = b3Vec3{1.9842999f, 124.571777f, -0.321986318f};
    points[40] = b3Vec3{1.86570692f, 124.365791f, -0.599836588f};
    points[41] = b3Vec3{-43.591507f, 78.1373291f, 6.1135149f};
    points[42] = b3Vec3{-43.8235397f, 79.2239074f, 3.48619604f};
    points[43] = b3Vec3{-43.591507f, 78.50811f, 5.54555655f};
    points[44] = b3Vec3{1.21086729f, 124.49453f, 1.07543683f};
    points[45] = b3Vec3{-1.86223853f, 124.195847f, 15.6257992f};
    points[46] = b3Vec3{-1.46520972f, 124.355492f, 16.9864483f};
    points[47] = b3Vec3{1.654302f, 124.612976f, 0.621887207f};

    g_gh_count = 48;
    for i32 i = 0; i < g_gh_count; i += 1 {
        g_gh_points[i] = b3MulSV(0.01f, points[i]);
    }

    // This shift shouldn't be necessary but I'm doing it so the hull
    // appears on the screen.
    // for ( int i = 0; i < m_count; ++i )
    //{
    //	m_points[i] -= m_points[0];
    //	m_points[i] *= 0.01f;
    //}

    g_gh_hull = b3CreateHull(cast(b3Vec3*, &g_gh_points), g_gh_count, 16);
}

void destroy_geometry_hull() {
    if g_gh_hull != null {
        b3DestroyHull(g_gh_hull);
        g_gh_hull = null;
    }
}

void step_geometry_hull(f32 timeStep) {
    ignore timeStep;
    if g_gh_hull != null {
        dbg_hull(b3WorldTransform_identity, g_gh_hull, b3_colorYellow);
    }

    dbg_axes(b3WorldTransform_identity, 1.0f);
}

// samples/sample_geometry.cpp HullReduction
const i32 HR_CAPACITY = 128;
const i32 HR_BOX = 0;
const i32 HR_SPHERE = 1;

i32 g_hr_type;
b3HullData* g_hr_hull;
b3Vec3[HR_CAPACITY] g_hr_points;
i32 g_hr_count;

void hull_reduction_generate_points() {
    g_randomSeed = 42;

    if g_hr_type == HR_BOX {
        b3Vec3 lower = b3Vec3{-2.0f, -2.0f, -2.0f};
        b3Vec3 upper = b3Vec3{2.0f, 2.0f, 2.0f};
        b3AABB box = b3AABB{b3Vec3{-1.0f, -1.0f, -1.0f}, b3Vec3{1.0f, 1.0f, 1.0f}};
        f32 a = 0.001f;
        b3AABB noise = b3AABB{b3Vec3{-a, -a, -a}, b3Vec3{a, a, a}};

        for i32 i = 0; i < HR_CAPACITY; i += 1 {
            b3Vec3 p = random_vec3(lower, upper);
            p = b3Clamp(p, box.lowerBound, box.upperBound);
            b3Vec3 f = random_vec3(noise.lowerBound, noise.upperBound);
            g_hr_points[i] = b3Add(p, f);
        }
    } else {
        for i32 i = 0; i < HR_CAPACITY; i += 1 {
            g_hr_points[i] = random_unit_vector();
        }
    }
}

void hull_reduction_generate_hull() {
    if g_hr_hull != null {
        b3DestroyHull(g_hr_hull);
    }

    g_hr_hull = b3CreateHull(cast(b3Vec3*, &g_hr_points), HR_CAPACITY, g_hr_count);
}

void build_hull_reduction() {
    g_hr_hull = null;

    g_hr_type = HR_SPHERE;
    g_hr_count = 16;
    hull_reduction_generate_points();
    hull_reduction_generate_hull();
}

void destroy_hull_reduction() {
    if g_hr_hull != null {
        b3DestroyHull(g_hr_hull);
        g_hr_hull = null;
    }
}

bool hull_reduction_controls() {
    if ImGui_RadioButton("Box", g_hr_type == HR_BOX) {
        g_hr_type = HR_BOX;
        hull_reduction_generate_points();
        hull_reduction_generate_hull();
    }

    if ImGui_RadioButton("Sphere", g_hr_type == HR_SPHERE) {
        g_hr_type = HR_SPHERE;
        hull_reduction_generate_points();
        hull_reduction_generate_hull();
    }

    if ImGui_SliderInt("count", &g_hr_count, 4, HR_CAPACITY, null, 0) {
        hull_reduction_generate_hull();
    }

    return true;
}

void step_hull_reduction(f32 timeStep) {
    ignore timeStep;
    if g_hr_hull != null {
        dbg_hull(b3WorldTransform_identity, g_hr_hull, b3_colorYellow);

        u8[128] buf;
        ignore snprintf(cast(u8*, &buf), 128, "v/f/e = %d/%d/%d",
                        g_hr_hull.vertexCount, g_hr_hull.faceCount, g_hr_hull.edgeCount / 2);
        draw_text_line(cast(u8*, &buf));
    }

    dbg_axes(b3WorldTransform_identity, 1.0f);
}

// samples/sample_geometry.cpp HullTransform
b3BoxHull g_ht_box;
b3HullData* g_ht_original;
b3HullData* g_ht_hull;
b3Vec3 g_ht_angles;
b3Vec3 g_ht_scale;
b3Vec3 g_ht_offset;

void build_hull_transform() {
    g_ht_original = b3CreateCylinder(1.0f, 0.5f, 0.0f, 9);
    g_ht_box = b3MakeBoxHull(0.5f, 0.5f, 0.5f);
    // m_original = &m_box.base;
    g_ht_scale = b3Vec3{1.0f, 1.0f, 1.0f};
    g_ht_angles = b3Vec3_zero;
    g_ht_offset = b3Vec3_zero;
    g_ht_hull = b3CloneAndTransformHull(g_ht_original, b3Transform_identity, g_ht_scale);
}

void destroy_hull_transform() {
    b3DestroyHull(g_ht_original);
    b3DestroyHull(g_ht_hull);
}

void hull_transform_update_hull() {
    b3DestroyHull(g_ht_hull);

    b3Quat qx = b3MakeQuatFromAxisAngle(b3Vec3_axisX, g_ht_angles.x * PI_F / 180.0f);
    b3Quat qy = b3MakeQuatFromAxisAngle(b3Vec3_axisY, g_ht_angles.y * PI_F / 180.0f);
    b3Quat qz = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, g_ht_angles.z * PI_F / 180.0f);
    b3Quat q = b3MulQuat(qz, b3MulQuat(qy, qx));

    g_ht_hull = b3CloneAndTransformHull(g_ht_original, b3Transform{g_ht_offset, q}, g_ht_scale);
}

void step_hull_transform(f32 timeStep) {
    ignore timeStep;
    b3Transform transform1 = b3Transform{b3Vec3{-2.0f, 0.0f, 0.0f}, b3Quat_identity};
    b3Transform transform2 = b3Transform{b3Vec3{2.0f, 0.0f, 0.0f}, b3Quat_identity};

    dbg_hull(b3MakeWorldTransform(transform1), g_ht_original, b3_colorGreen);
    dbg_hull(b3MakeWorldTransform(transform2), g_ht_hull, b3_colorYellow);
    dbg_axes(b3WorldTransform_identity, 1.0f);

    u8[160] buf;
    ignore snprintf(cast(u8*, &buf), 160, "hull 1: area = %g, volume = %g, radius = %g",
                    g_ht_original.surfaceArea, g_ht_original.volume, g_ht_original.innerRadius);
    draw_text_line(cast(u8*, &buf));
    ignore snprintf(cast(u8*, &buf), 160, "hull 2: area = %g, volume = %g, radius = %g",
                    g_ht_hull.surfaceArea, g_ht_hull.volume, g_ht_hull.innerRadius);
    draw_text_line(cast(u8*, &buf));
}

bool hull_transform_controls() {
    if ImGui_SliderFloat("sx", &g_ht_scale.x, -2.0f, 2.0f, "%.1f", 0) {
        hull_transform_update_hull();
    }

    if ImGui_SliderFloat("sy", &g_ht_scale.y, -2.0f, 2.0f, "%.1f", 0) {
        hull_transform_update_hull();
    }

    if ImGui_SliderFloat("sz", &g_ht_scale.z, -2.0f, 2.0f, "%.1f", 0) {
        hull_transform_update_hull();
    }

    if ImGui_SliderFloat("rx", &g_ht_angles.x, -180.0f, 180.0f, "%.0f", 0) {
        hull_transform_update_hull();
    }

    if ImGui_SliderFloat("ry", &g_ht_angles.y, -180.0f, 180.0f, "%.0f", 0) {
        hull_transform_update_hull();
    }

    if ImGui_SliderFloat("rz", &g_ht_angles.z, -180.0f, 180.0f, "%.0f", 0) {
        hull_transform_update_hull();
    }

    if ImGui_SliderFloat("px", &g_ht_offset.x, -1.0f, 1.0f, "%.1f", 0) {
        hull_transform_update_hull();
    }

    if ImGui_SliderFloat("py", &g_ht_offset.y, -1.0f, 1.0f, "%.1f", 0) {
        hull_transform_update_hull();
    }

    if ImGui_SliderFloat("pz", &g_ht_offset.z, -1.0f, 1.0f, "%.1f", 0) {
        hull_transform_update_hull();
    }

    return true;
}

// samples/sample_geometry.cpp CapsuleMass
// Limited by edge count
const i32 CM_MAX_SIDES = 6;
b3Capsule g_cm_capsule;
b3BoxHull g_cm_box;
b3HullData* g_cm_hull;
f32 g_cm_radius;
f32 g_cm_length;
i32 g_cm_sides;

void capsule_mass_create_hull(i32 sides) {
    if g_cm_hull != null {
        b3DestroyHull(g_cm_hull);
        g_cm_hull = null;
    }

    const i32 N = CM_MAX_SIDES;
    const i32 M = 2 * N * N;
    b3Vec3[M] points;
    if sides > N {
        return;
    }

    i32 count = 2 * sides * sides;
    f32 d = PI_F / (cast(f32, sides) - 1.0f);
    f32 angle1 = -0.5f * PI_F;
    i32 index = 0;
    for i32 i = 0; i < sides; i += 1 {
        f32 s1 = sinf(angle1);
        f32 c1 = cosf(angle1);
        f32 angle2 = -0.5f * PI_F;
        for i32 j = 0; j < sides; j += 1 {
            points[index].x = 1.0f + g_cm_radius * c1;
            points[index].y = g_cm_radius * s1 * cosf(angle2);
            points[index].z = g_cm_radius * s1 * sinf(angle2);
            angle2 += d;
            index += 1;
        }
        angle1 += d;
    }
    angle1 = 0.5f * PI_F;
    for i32 i = 0; i < sides; i += 1 {
        f32 s1 = sinf(angle1);
        f32 c1 = cosf(angle1);
        f32 angle2 = -0.5f * PI_F;
        for i32 j = 0; j < sides; j += 1 {
            points[index].x = -1.0f + g_cm_radius * c1;
            points[index].y = g_cm_radius * s1 * cosf(angle2);
            points[index].z = g_cm_radius * s1 * sinf(angle2);
            angle2 += d;
            index += 1;
        }
        angle1 += d;
    }
    g_cm_hull = b3CreateHull(cast(b3Vec3*, &points), count, count);
}

void build_capsule_mass() {
    g_cm_radius = 1.0f;
    g_cm_length = 2.0f;
    g_cm_sides = 6;
    g_cm_capsule = b3Capsule{b3Pos{-0.5f * g_cm_length, 0.0f, 0.0f},
                             b3Pos{0.5f * g_cm_length, 0.0f, 0.0f}, g_cm_radius};
    g_cm_box = b3MakeBoxHull(g_cm_radius + 0.5f * g_cm_length, g_cm_radius, g_cm_radius);
    g_cm_hull = null;
    capsule_mass_create_hull(g_cm_sides);
}

void destroy_capsule_mass() {
    if g_cm_hull != null {
        b3DestroyHull(g_cm_hull);
        g_cm_hull = null;
    }
}

bool capsule_mass_controls() {
    if ImGui_SliderInt("sides", &g_cm_sides, 3, CM_MAX_SIDES, "%d", 0) {
        capsule_mass_create_hull(g_cm_sides);
    }
    return true;
}

void step_capsule_mass(f32 timeStep) {
    ignore timeStep;
    // The lit path is opaque, so the 0.8 alpha upstream asks for here
    // does not read through yet; the colour is otherwise upstream's.
    dbg_solid_capsule(b3WorldTransform_identity, g_cm_capsule,
                      make_color_alpha(b3_colorAqua, 0.8f));
    dbg_hull(b3WorldTransform_identity, &g_cm_box.base, b3_colorBlueViolet);
    if g_cm_hull != null {
        dbg_hull(b3WorldTransform_identity, g_cm_hull, b3_colorYellow);
    }
    dbg_axes(b3WorldTransform_identity, 1.0f);
    if g_cm_hull != null {
        b3MassData lowerMassData = b3ComputeHullMass(g_cm_hull, 1.0f);
        b3MassData massData = b3ComputeCapsuleMass(&g_cm_capsule, 1.0f);
        b3MassData upperMassData = b3ComputeHullMass(&g_cm_box.base, 1.0f);
        u8[128] buf;
        ignore snprintf(cast(u8*, &buf), 128, "mass hull:    %g", cast(f64, lowerMassData.mass));
        draw_text_line(cast(u8*, &buf));
        ignore snprintf(cast(u8*, &buf), 128, "mass capsule: %g", cast(f64, massData.mass));
        draw_text_line(cast(u8*, &buf));
        ignore snprintf(cast(u8*, &buf), 128, "mass box:     %g", cast(f64, upperMassData.mass));
        draw_text_line(cast(u8*, &buf));
        draw_text_line("");
        ignore snprintf(cast(u8*, &buf), 128, "Ixx hull:    %g",
                        cast(f64, lowerMassData.inertia.cx.x));
        draw_text_line(cast(u8*, &buf));
        ignore snprintf(cast(u8*, &buf), 128, "Ixx capsule: %g", cast(f64, massData.inertia.cx.x));
        draw_text_line(cast(u8*, &buf));
        ignore snprintf(cast(u8*, &buf), 128, "Ixx box:     %g",
                        cast(f64, upperMassData.inertia.cx.x));
        draw_text_line(cast(u8*, &buf));
        draw_text_line("");
        ignore snprintf(cast(u8*, &buf), 128, "Iyy hull:    %g",
                        cast(f64, lowerMassData.inertia.cy.y));
        draw_text_line(cast(u8*, &buf));
        ignore snprintf(cast(u8*, &buf), 128, "Iyy capsule: %g", cast(f64, massData.inertia.cy.y));
        draw_text_line(cast(u8*, &buf));
        ignore snprintf(cast(u8*, &buf), 128, "Iyy box:     %g",
                        cast(f64, upperMassData.inertia.cy.y));
        draw_text_line(cast(u8*, &buf));
        draw_text_line("");
        ignore snprintf(cast(u8*, &buf), 128, "Izz hull:    %g",
                        cast(f64, lowerMassData.inertia.cz.z));
        draw_text_line(cast(u8*, &buf));
        ignore snprintf(cast(u8*, &buf), 128, "Izz capsule: %g", cast(f64, massData.inertia.cz.z));
        draw_text_line(cast(u8*, &buf));
        ignore snprintf(cast(u8*, &buf), 128, "Izz box:     %g",
                        cast(f64, upperMassData.inertia.cz.z));
        draw_text_line(cast(u8*, &buf));
    }
}
