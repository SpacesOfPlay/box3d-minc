// box3d_rbchar.mc — the force-based character controller from
// samples/sample_character.cpp, transpiled.
//
import box3d;
import math;
// transminc: C #define values surfaced as compile-time configuration
@define "__x86_64__" 1
@define "NDEBUG" 1
@define "_MSC_VER" 1900
@define "KEY_W" 87
@define "KEY_S" 83
@define "KEY_A" 65
@define "KEY_D" 68
@define "KEY_SPACE" 32
@define "KEY_LEFT_SHIFT" 340
@define "B3_ENABLE_VALIDATION" 0
@define "B3_NULL_INDEX" -1
@define "B3_HASH_INIT" 5381
@define "B3_MAX_WORKERS" 32
@define "B3_MAX_TASKS" 256
@define "B3_GRAPH_COLOR_COUNT" 24
@define "B3_CONTACT_MANIFOLD_COUNT_BUCKETS" 8
@define "B3_MAX_WORLDS" 128
@define "B3_MAX_MANIFOLD_POINTS" 4
@define "B3_MAX_SHAPE_CAST_POINTS" 64
@define "B3_GYROSCOPIC_ITERATIONS" 1
@define "B3_MAX_HULL_VERTICES" 128
@define "B3_MAX_HULL_FACES" 128
@define "B3_MAX_HULL_EDGES" 128
@define "B3_SHAPE_POWER" 22
@define "B3_RESTITUTION_ITERATIONS" 1
@define "B3_DYNAMIC_TREE_VERSION" -7787375179321898166
@define "B3_HULL_VERSION" -2715301031560262655
@define "B3_MESH_VERSION" -6066037853393090451
@define "B3_HEIGHT_FIELD_HOLE" 255
@define "B3_HEIGHT_FIELD_VERSION" -8423759003537458044
@define "B3_MAX_COMPOUND_MESH_MATERIALS" 4

type errno_t = i32;
// Vendored extraction: samples/sample_character.cpp lines 593-1312,
// moved verbatim so RigidbodyCharacter is its own translation unit and
// can be transpiled. No edits to the moved code.
struct ClosestShapeCastContext {
    b3ShapeId[16] ignoreShapes;
    i32 ignoreCount;
    f32 closestFraction;
    b3Vec3 closestNormal;
    b3Pos closestPoint;
    b3ShapeId closestShape;
    bool hit;
    bool startedSolid;
}

// --- Trace result matching s&box's SceneTraceResult ---
struct TraceResult {
    b3Pos endPosition;
    b3Vec3 normal;
    b3Pos hitPoint;
    f32 fraction;
    bool hit;
    bool startedSolid;
}

// --- RigidbodyCharacter ---
struct RigidbodyCharacter {
    b3BodyId m_bodyId;
    b3ShapeId m_bodyCapsuleId;
    b3ShapeId m_feetBoxId;
    b3ShapeId[4] m_ownShapes;
    i32 m_ownShapeCount;
    b3Vec3 m_groundNormal;
    b3Vec3 m_groundVelocity;
    f32 m_jumpCooldown;
    bool m_onGround;
    bool m_sprint;
    bool m_didStep;
    b3Pos m_stepPosition;
    b3Vec3 m_lastWishVelocity;
    b3Pos m_massCenterWorld;
    Sample* m_sample;
}

private {
f32 ClosestShapeCastCallback(b3ShapeId shapeId, b3Pos point, b3Vec3 normal, f32 fraction, u64 userMaterialId, i32 triangleIndex, i32 childIndex, void* context) {
    var ctx = cast(ClosestShapeCastContext*, context);
    for i32 i = 0; i < ctx.ignoreCount; ++i {
        if shapeId.index1 == ctx.ignoreShapes[i].index1 && shapeId.world0 == ctx.ignoreShapes[i].world0 && shapeId.generation == ctx.ignoreShapes[i].generation {
            return -1.0f;
        }
    }
    if fraction == 0.0f {
        ctx.startedSolid = true;
        return -1.0f;
    }
    if fraction < ctx.closestFraction {
        ctx.closestFraction = fraction;
        ctx.closestNormal = normal;
        ctx.closestPoint = point;
        ctx.closestShape = shapeId;
        ctx.hit = true;
    }
    return ctx.closestFraction;
}
}
f32 RigidbodyCharacter_SRC = 0.025400000000000002f;
f32 RigidbodyCharacter_m_walkSpeed = 230.0f * 0.025400000000000002f;
f32 RigidbodyCharacter_m_runSpeed = 350.0f * 0.025400000000000002f;
f32 RigidbodyCharacter_m_jumpSpeed = 300.0f * 0.025400000000000002f;
f32 RigidbodyCharacter_m_maxSlopeAngle = 45.0f;
f32 RigidbodyCharacter_m_characterGravity = 15.0f;
f32 RigidbodyCharacter_m_characterMass = 500.0f;
f32 RigidbodyCharacter_m_jumpCooldownTime = 0.2f;
f32 RigidbodyCharacter_m_stepUpHeight = 18.0f * 0.025400000000000002f;
f32 RigidbodyCharacter_m_stepDownHeight = 18.0f * 0.025400000000000002f;
f32 RigidbodyCharacter_m_skin = 0.095f * 0.025400000000000002f;
f32 RigidbodyCharacter_m_brakePower = 0.2f;
f32 RigidbodyCharacter_m_surfaceFriction = 0.6000000000000001f;
f32 RigidbodyCharacter_m_airFriction = 0.1f;
f32 RigidbodyCharacter_m_bodyRadius = 16.0f * 0.025400000000000002f;
f32 RigidbodyCharacter_m_totalHeight = 72.0f * 0.025400000000000002f;
f32 RigidbodyCharacter_m_feetHeight = 72.0f * 0.025400000000000002f * 0.5f;
void RigidbodyCharacter_Initialize(RigidbodyCharacter* self, Sample* sample, b3Pos position) {
    self.m_sample = sample;
    self.m_onGround = false;
    self.m_sprint = false;
    self.m_jumpCooldown = 0.0f;
    self.m_groundNormal = b3Vec3_axisY;
    self.m_groundVelocity = b3Vec3_zero;
    self.m_lastWishVelocity = b3Vec3_zero;
    self.m_didStep = false;
    self.m_stepPosition = b3Pos_zero;
    b3BodyDef bodyDef = b3DefaultBodyDef();
    bodyDef.type = b3_dynamicBody;
    bodyDef.position = position;
    self.m_massCenterWorld = bodyDef.position;
    bodyDef.motionLocks.angularX = true;
    bodyDef.motionLocks.angularY = true;
    bodyDef.motionLocks.angularZ = true;
    bodyDef.enableSleep = false;
    bodyDef.enableContactRecycling = false;
    bodyDef.name = "character";
    bodyDef.gravityScale = RigidbodyCharacter_m_characterGravity / 10.0f;
    self.m_bodyId = b3CreateBody(sample.m_worldId, &bodyDef);
    {
        f32 halfExtX = RigidbodyCharacter_m_bodyRadius * 0.5f;
        f32 halfExtY = RigidbodyCharacter_m_feetHeight * 0.5f;
        f32 halfExtZ = RigidbodyCharacter_m_bodyRadius * 0.5f;
        b3ShapeDef shapeDef = b3DefaultShapeDef();
        shapeDef.baseMaterial.friction = 0.0f;
        shapeDef.baseMaterial.restitution = 0.0f;
        shapeDef.baseMaterial.customColor = cast(u32, b3_colorLimeGreen);
        f32 feetVolume = 8.0f * halfExtX * halfExtY * halfExtZ;
        shapeDef.density = RigidbodyCharacter_m_characterMass * 0.4f / feetVolume;
        var feetTransform = b3Transform{b3Vec3{0.0f, -RigidbodyCharacter_m_totalHeight * 0.5f + halfExtY, 0.0f}, b3Quat_identity};
        b3BoxHull feetBox = b3MakeTransformedBoxHull(halfExtX, halfExtY, halfExtZ, feetTransform);
        self.m_feetBoxId = b3CreateHullShape(self.m_bodyId, &shapeDef, &feetBox.base);
    }
    {
        f32 capsuleRadius = RigidbodyCharacter_m_bodyRadius * 0.707f;
        f32 capsuleBottom = -RigidbodyCharacter_m_totalHeight * 0.5f + RigidbodyCharacter_m_feetHeight * 0.5f + capsuleRadius;
        f32 capsuleTop = RigidbodyCharacter_m_totalHeight * 0.5f - capsuleRadius;
        if capsuleTop > capsuleBottom {
            var capsule = b3Capsule{b3Vec3{0.0f, capsuleBottom, 0.0f}, b3Vec3{0.0f, capsuleTop, 0.0f}, capsuleRadius};
            b3ShapeDef shapeDef = b3DefaultShapeDef();
            shapeDef.baseMaterial.friction = 0.0f;
            shapeDef.baseMaterial.restitution = 0.0f;
            shapeDef.baseMaterial.customColor = cast(u32, b3_colorCornflowerBlue);
            f32 h = capsuleTop - capsuleBottom;
            f32 r = capsuleRadius;
            f32 capsuleVolume = 3.14159265359f * r * r * (h + 4.0f * r / 3.0f);
            shapeDef.density = RigidbodyCharacter_m_characterMass * 0.6000000000000001f / capsuleVolume;
            self.m_bodyCapsuleId = b3CreateCapsuleShape(self.m_bodyId, &shapeDef, &capsule);
        }
    }
    self.m_ownShapeCount = b3Body_GetShapes(self.m_bodyId, self.m_ownShapes, 4);
    RigidbodyCharacter_UpdateMassCenter(self, 0.0f);
}
// --- TraceBody: box shape cast matching s&box's TraceBody ---
// Casts a box from `from` to `to` with given radius and height scale.
TraceResult RigidbodyCharacter_TraceBody(RigidbodyCharacter* self, b3Pos from_var, b3Pos to, f32 radiusScale, f32 heightScale) {
    TraceResult result;
    result.endPosition = to;
    result.normal = b3Vec3_axisY;
    result.hitPoint = to;
    result.fraction = 1.0f;
    result.hit = false;
    result.startedSolid = false;
    b3Vec3 translation = op_sub_b3Pos_b3Pos(to, from_var);
    f32 translationLen = b3Length(translation);
    if translationLen < 1.0000000000000002e-6f {
        return result;
    }
    f32 halfW = RigidbodyCharacter_m_bodyRadius * 0.5f * radiusScale;
    f32 halfH = RigidbodyCharacter_m_totalHeight * heightScale * 0.5f;
    f32 halfD = RigidbodyCharacter_m_bodyRadius * 0.5f * radiusScale;
    f32 boxMinY = 0.0f;
    f32 boxMaxY = RigidbodyCharacter_m_totalHeight * heightScale;
    f32 boxCenterY = (boxMinY + boxMaxY) * 0.5f;
    b3Vec3[8] points;
    for i32 i = 0; i < 8; ++i {
        f32 sx = (i & 1) != 0 ? halfW : -halfW;
        f32 sy = (i & 2) != 0 ? halfH : -halfH;
        f32 sz = (i & 4) != 0 ? halfD : -halfD;
        points[i] = b3Vec3{sx, boxCenterY + sy, sz};
    }
    b3ShapeProxy proxy;
    proxy.points = points;
    proxy.count = 8;
    proxy.radius = 0.0f;
    ClosestShapeCastContext ctx;
    for i32 i = 0; i < self.m_ownShapeCount; ++i {
        ctx.ignoreShapes[i] = self.m_ownShapes[i];
    }
    ctx.ignoreCount = self.m_ownShapeCount;
    ctx.closestFraction = 1.0f;
    ctx.hit = false;
    ctx.startedSolid = false;
    ctx.closestShape = b3_nullShapeId;
    b3QueryFilter filter = b3DefaultQueryFilter();
    b3World_CastShape(self.m_sample.m_worldId, from_var, &proxy, translation, filter, cast(b3CastResultFcn, ClosestShapeCastCallback), &ctx);
    result.startedSolid = ctx.startedSolid;
    if ctx.hit != 0 {
        result.hit = true;
        result.fraction = ctx.closestFraction;
        result.normal = ctx.closestNormal;
        result.hitPoint = ctx.closestPoint;
        result.endPosition = op_add_b3Pos_b3Vec3(from_var, op_mul_float_b3Vec3(ctx.closestFraction, translation));
    }
    return result;
}
bool RigidbodyCharacter_IsStandableSurface(RigidbodyCharacter* self, b3Vec3 normal) {
    f32 maxSlopeCos = cosf(RigidbodyCharacter_m_maxSlopeAngle * 3.14159265359f / 180.0f);
    return b3Dot(normal, b3Vec3_axisY) >= maxSlopeCos;
}
// Get feet position from body center position
b3Pos RigidbodyCharacter_GetFeetPosition(RigidbodyCharacter* self) {
    b3Pos pos = b3Body_GetPosition(self.m_bodyId);
    return b3Pos{pos.x, pos.y - RigidbodyCharacter_m_totalHeight * 0.5f, pos.z};
}
// --- CategorizeGround: s&box-style box cast with radius shrinking ---
void RigidbodyCharacter_CategorizeGround(RigidbodyCharacter* self) {
    b3Pos feet = RigidbodyCharacter_GetFeetPosition(self);
    var from_var = b3Pos{feet.x, feet.y + 4.0f * RigidbodyCharacter_SRC, feet.z};  // renamed from: from
    var to = b3Pos{feet.x, feet.y - 2.0f * RigidbodyCharacter_SRC, feet.z};
    f32 radiusScale = 1.0f;
    TraceResult tr = RigidbodyCharacter_TraceBody(self, from_var, to, radiusScale, 0.5f);
    while tr.startedSolid || tr.hit && !RigidbodyCharacter_IsStandableSurface(self, tr.normal) {
        radiusScale -= 0.1f;
        if radiusScale < 0.7000000000000001f {
            RigidbodyCharacter_UpdateGround(self, false, b3Vec3_axisY);
            DrawLine(from_var, to, MakeColor(b3_colorRed));
            return;
        }
        tr = RigidbodyCharacter_TraceBody(self, from_var, to, radiusScale, 0.5f);
    }
    if !tr.startedSolid && tr.hit && RigidbodyCharacter_IsStandableSurface(self, tr.normal) && self.m_jumpCooldown <= 0.0f {
        RigidbodyCharacter_UpdateGround(self, true, tr.normal);
        DrawLine(from_var, tr.hitPoint, MakeColor(b3_colorGreen));
        DrawPoint(tr.hitPoint, 5.0f, MakeColor(b3_colorGreen));
    } else {
        RigidbodyCharacter_UpdateGround(self, false, b3Vec3_axisY);
        DrawLine(from_var, to, MakeColor(b3_colorGray));
    }
}
void RigidbodyCharacter_UpdateGround(RigidbodyCharacter* self, bool onGround, b3Vec3 normal) {
    self.m_onGround = onGround;
    self.m_groundNormal = normal;
    if onGround == 0 {
        self.m_groundVelocity = b3Vec3_zero;
    }
}
// --- Reground / StickToGround: snap character to surface when on ground ---
void RigidbodyCharacter_Reground(RigidbodyCharacter* self, f32 stepSize) {
    if self.m_onGround == 0 {
        return;
    }
    b3Pos pos = b3Body_GetPosition(self.m_bodyId);
    var from_var = b3Pos{pos.x, pos.y + 0.05f, pos.z};  // renamed from: from
    var to = b3Pos{pos.x, pos.y - stepSize, pos.z};
    f32 radiusScale = 1.0f;
    TraceResult tr = RigidbodyCharacter_TraceBody(self, from_var, to, radiusScale, 0.5f);
    while tr.startedSolid != 0 {
        radiusScale -= 0.1f;
        if radiusScale < 0.7000000000000001f {
            return;
        }
        tr = RigidbodyCharacter_TraceBody(self, from_var, to, radiusScale, 0.5f);
    }
    if tr.hit != 0 {
        var targetPos = b3Pos{tr.endPosition.x, tr.endPosition.y + 0.01f, tr.endPosition.z};
        f32 deltaY = targetPos.y - pos.y;
        b3Quat rot = b3Body_GetRotation(self.m_bodyId);
        b3Body_SetTransform(self.m_bodyId, targetPos, rot);
        if deltaY > 0.01f {
            b3Vec3 vel = b3Body_GetLinearVelocity(self.m_bodyId);
            vel.y = 0.0f;
            b3Body_SetLinearVelocity(self.m_bodyId, vel);
        }
        DrawLine(from_var, tr.endPosition, MakeColor(b3_colorCyan));
    }
}
// --- TryStep: 4-phase trace-based step-up algorithm ---
// Returns true if a step was taken and m_stepPosition was set.
bool RigidbodyCharacter_TryStep(RigidbodyCharacter* self, f32 maxStepHeight) {
    b3Pos pos = b3Body_GetPosition(self.m_bodyId);
    b3Vec3 vel = b3Body_GetLinearVelocity(self.m_bodyId);
    if self.m_onGround == 0 {
        return false;
    }
    var hVel = b3Vec3{vel.x, 0.0f, vel.z};
    f32 hSpeed = b3Length(hVel);
    if hSpeed < 0.01f {
        return false;
    }
    var moveDir = b3Vec3{hVel.x / hSpeed, 0.0f, hVel.z / hSpeed};
    f32 forwardDist = hSpeed * (1.0f / 60.0f) + RigidbodyCharacter_m_bodyRadius;
    b3Pos forwardFrom = op_sub_b3Pos_b3Vec3(pos, op_mul_float_b3Vec3(RigidbodyCharacter_m_skin, moveDir));
    b3Pos forwardTo = op_add_b3Pos_b3Vec3(pos, op_mul_float_b3Vec3(forwardDist, moveDir));
    f32 radiusScale = 1.0f;
    TraceResult trForward = RigidbodyCharacter_TraceBody(self, forwardFrom, forwardTo, radiusScale, 1.0f);
    while trForward.startedSolid != 0 {
        radiusScale -= 0.1f;
        if radiusScale < 0.6000000000000001f {
            DrawLine(forwardFrom, forwardTo, MakeColor(b3_colorRed));
            return false;
        }
        trForward = RigidbodyCharacter_TraceBody(self, forwardFrom, forwardTo, radiusScale, 1.0f);
    }
    if trForward.hit == 0 {
        return false;
    }
    DrawLine(forwardFrom, trForward.endPosition, MakeColor(b3_colorYellow));
    b3Pos hitPos = trForward.endPosition;
    b3Pos upFrom = hitPos;
    var upTo = b3Pos{hitPos.x, hitPos.y + maxStepHeight, hitPos.z};
    TraceResult trUp = RigidbodyCharacter_TraceBody(self, upFrom, upTo, radiusScale, 1.0f);
    if trUp.startedSolid != 0 {
        DrawLine(upFrom, upTo, MakeColor(b3_colorRed));
        return false;
    }
    b3Pos topPos = trUp.hit != 0 ? trUp.endPosition : upTo;
    f32 upDistance = topPos.y - upFrom.y;
    if upDistance < 0.005f {
        DrawLine(upFrom, topPos, MakeColor(b3_colorRed));
        return false;
    }
    DrawLine(upFrom, topPos, MakeColor(b3_colorYellow));
    f32 acrossDist = forwardDist * (1.0f - trForward.fraction) + RigidbodyCharacter_m_bodyRadius * 0.5f;
    b3Pos acrossFrom = topPos;
    b3Pos acrossTo = op_add_b3Pos_b3Vec3(topPos, op_mul_float_b3Vec3(acrossDist, moveDir));
    TraceResult trAcross = RigidbodyCharacter_TraceBody(self, acrossFrom, acrossTo, radiusScale, 1.0f);
    if trAcross.startedSolid != 0 {
        DrawLine(acrossFrom, acrossTo, MakeColor(b3_colorRed));
        return false;
    }
    b3Pos acrossPos = trAcross.hit != 0 ? trAcross.endPosition : acrossTo;
    DrawLine(acrossFrom, acrossPos, MakeColor(b3_colorYellow));
    b3Pos downFrom = acrossPos;
    var downTo = b3Pos{acrossPos.x, acrossPos.y - maxStepHeight, acrossPos.z};
    TraceResult trDown = RigidbodyCharacter_TraceBody(self, downFrom, downTo, radiusScale, 1.0f);
    if trDown.hit == 0 {
        DrawLine(downFrom, downTo, MakeColor(b3_colorRed));
        return false;
    }
    if RigidbodyCharacter_IsStandableSurface(self, trDown.normal) == 0 {
        DrawLine(downFrom, trDown.endPosition, MakeColor(b3_colorRed));
        return false;
    }
    f32 stepHeight = trDown.endPosition.y - pos.y;
    if stepHeight < 0.01f {
        return false;
    }
    DrawLine(downFrom, trDown.endPosition, MakeColor(b3_colorYellow));
    DrawPoint(trDown.endPosition, 8.0f, MakeColor(b3_colorYellow));
    var stepPos = b3Pos{trDown.endPosition.x, trDown.endPosition.y + 0.01f, trDown.endPosition.z};
    b3Quat rot = b3Body_GetRotation(self.m_bodyId);
    b3Body_SetTransform(self.m_bodyId, stepPos, rot);
    b3Vec3 newVel = b3Body_GetLinearVelocity(self.m_bodyId);
    newVel.x *= 0.9f;
    newVel.y = 0.0f;
    newVel.z *= 0.9f;
    b3Body_SetLinearVelocity(self.m_bodyId, newVel);
    self.m_stepPosition = stepPos;
    return true;
}
void RigidbodyCharacter_RestoreStep(RigidbodyCharacter* self) {
    if self.m_didStep == 0 {
        return;
    }
    b3Quat rot = b3Body_GetRotation(self.m_bodyId);
    b3Body_SetTransform(self.m_bodyId, self.m_stepPosition, rot);
    self.m_didStep = false;
}
b3Vec3 RigidbodyCharacter_AddClamped(b3Vec3 current, b3Vec3 add, f32 maxAddLength) {
    f32 addLen = b3Length(add);
    if addLen > maxAddLength && addLen > 0.0f {
        add = op_mul_float_b3Vec3(maxAddLength / addLen, add);
    }
    return op_add_b3Vec3_b3Vec3(current, add);
}
// --- UpdateMassCenter: s&box formula ---
void RigidbodyCharacter_UpdateMassCenter(RigidbodyCharacter* self, f32 wishSpeed) {
    b3MassData massData = b3Body_GetMassData(self.m_bodyId);
    f32 halfHeight = RigidbodyCharacter_m_totalHeight * 0.5f;
    if self.m_onGround != 0 {
        f32 centerOffset = b3ClampFloat(wishSpeed, 0.0f, halfHeight);
        massData.center = b3Vec3{0.0f, centerOffset - halfHeight, 0.0f};
    } else {
        massData.center = b3Vec3{0.0f, 0.0f, 0.0f};
    }
    b3Body_SetMassData(self.m_bodyId, massData);
}
// --- UpdateBody: set friction, gravity, damping per s&box ---
void RigidbodyCharacter_UpdateBody(RigidbodyCharacter* self, b3Vec3 wishVelocity) {
    f32 wishLen = b3Length(wishVelocity);
    b3Vec3 vel = b3Body_GetLinearVelocity(self.m_bodyId);
    f32 velLen = b3Length(vel);
    f32 feetFriction = 0.0f;
    if self.m_onGround != 0 {
        bool wantsBrakes = wishLen < 5.0f * RigidbodyCharacter_SRC || wishLen < velLen * 0.9f;
        if wantsBrakes != 0 {
            feetFriction = 1.0f + 100.0f * RigidbodyCharacter_m_brakePower * RigidbodyCharacter_m_surfaceFriction;
        }
    }
    b3Shape_SetFriction(self.m_feetBoxId, feetFriction);
    RigidbodyCharacter_UpdateMassCenter(self, wishLen);
    bool wantsGravity = false;
    if self.m_onGround == 0 {
        wantsGravity = true;
    }
    if velLen > 1.0f * RigidbodyCharacter_SRC {
        wantsGravity = true;
    }
    if b3Length(self.m_groundVelocity) > 1.0f * RigidbodyCharacter_SRC {
        wantsGravity = true;
    }
    b3Body_SetGravityScale(self.m_bodyId, wantsGravity != 0 ? RigidbodyCharacter_m_characterGravity / 10.0f : 0.0f);
    bool wantsDamping = self.m_onGround && wishLen < 1.0f * RigidbodyCharacter_SRC && b3Length(self.m_groundVelocity) < 1.0f * RigidbodyCharacter_SRC;
    b3Body_SetLinearDamping(self.m_bodyId, wantsDamping != 0 ? 10.0f * RigidbodyCharacter_m_brakePower : RigidbodyCharacter_m_airFriction);
}
// --- AddVelocity: s&box's MoveMode.Walk velocity model ---
void RigidbodyCharacter_AddVelocity(RigidbodyCharacter* self, b3Vec3 wishVelocity) {
    var wish = b3Vec3{wishVelocity.x, 0.0f, wishVelocity.z};
    f32 wishLen = b3Length(wish);
    if wishLen < 0.001f {
        return;
    }
    f32 groundFrictionFactor = 0.25f + RigidbodyCharacter_m_surfaceFriction * 10.0f;
    b3Vec3 vel = b3Body_GetLinearVelocity(self.m_bodyId);
    f32 savedY = vel.y;
    b3Vec3 velocity = op_sub_b3Vec3_b3Vec3(vel, self.m_groundVelocity);
    f32 speed = b3Length(velocity);
    f32 maxSpeed = b3MaxFloat(wishLen, speed);
    if self.m_onGround != 0 {
        f32 amount = 1.0f * groundFrictionFactor;
        velocity = RigidbodyCharacter_AddClamped(velocity, op_mul_float_b3Vec3(amount, wish), wishLen * amount);
    } else {
        f32 amount = 0.05f;
        velocity = RigidbodyCharacter_AddClamped(velocity, op_mul_float_b3Vec3(amount, wish), wishLen);
    }
    f32 newSpeed = b3Length(velocity);
    if newSpeed > maxSpeed && newSpeed > 0.0f {
        velocity = op_mul_float_b3Vec3(maxSpeed / newSpeed, velocity);
    }
    velocity = op_add_b3Vec3_b3Vec3(velocity, self.m_groundVelocity);
    if self.m_onGround != 0 {
        velocity.y = savedY;
    }
    b3Body_SetLinearVelocity(self.m_bodyId, velocity);
}
// --- PreStep: UpdateBody + AddVelocity + TryStep ---
void RigidbodyCharacter_PreStep(RigidbodyCharacter* self, f32 timeStep, b3Vec3 forward, b3Vec3 right, b3Vec2 throttle) {
    if self.m_jumpCooldown > 0.0f {
        self.m_jumpCooldown -= timeStep;
    }
    f32 maxSpeed = self.m_sprint != 0 ? RigidbodyCharacter_m_runSpeed : RigidbodyCharacter_m_walkSpeed;
    b3Vec3 wishVelocity = op_add_b3Vec3_b3Vec3(op_mul_float_b3Vec3(maxSpeed, op_mul_float_b3Vec3(throttle.x, forward)), op_mul_float_b3Vec3(maxSpeed, op_mul_float_b3Vec3(throttle.y, right)));
    f32 wishSpeed = b3Length(wishVelocity);
    if wishSpeed > maxSpeed {
        wishVelocity = op_mul_float_b3Vec3(maxSpeed / wishSpeed, wishVelocity);
    }
    self.m_lastWishVelocity = wishVelocity;
    RigidbodyCharacter_UpdateBody(self, wishVelocity);
    RigidbodyCharacter_AddVelocity(self, wishVelocity);
    self.m_didStep = RigidbodyCharacter_TryStep(self, RigidbodyCharacter_m_stepUpHeight);
    self.m_massCenterWorld = b3Body_GetWorldCenter(self.m_bodyId);
}
// --- PostStep: RestoreStep + Reground + CategorizeGround ---
void RigidbodyCharacter_PostStep(RigidbodyCharacter* self, f32 timeStep) {
    RigidbodyCharacter_RestoreStep(self);
    RigidbodyCharacter_Reground(self, RigidbodyCharacter_m_stepDownHeight);
    RigidbodyCharacter_CategorizeGround(self);
}
void RigidbodyCharacter_Jump(RigidbodyCharacter* self) {
    if self.m_onGround && self.m_jumpCooldown <= 0.0f {
        b3Vec3 velocity = b3Body_GetLinearVelocity(self.m_bodyId);
        velocity.y = RigidbodyCharacter_m_jumpSpeed;
        b3Body_SetLinearVelocity(self.m_bodyId, velocity);
        self.m_onGround = false;
        self.m_jumpCooldown = RigidbodyCharacter_m_jumpCooldownTime;
    }
}
void RigidbodyCharacter_Step(RigidbodyCharacter* self, f32 timeStep, b3Vec3 forward, b3Vec3 right, b3Vec2 throttle) {
    RigidbodyCharacter_PreStep(self, timeStep, forward, right, throttle);
}
void RigidbodyCharacter_LateStep(RigidbodyCharacter* self, f32 timeStep) {
    RigidbodyCharacter_PostStep(self, timeStep);
}
void RigidbodyCharacter_DrawDebug(RigidbodyCharacter* self) {
    b3Pos pos = b3Body_GetPosition(self.m_bodyId);
    b3Vec3 vel = b3Body_GetLinearVelocity(self.m_bodyId);
    DrawLine(pos, b3OffsetPos(pos, vel), MakeColor(b3_colorPurple));
    DrawLine(pos, b3OffsetPos(pos, self.m_lastWishVelocity), MakeColor(b3_colorOrange));
    DrawPoint(self.m_massCenterWorld, 8.0f, MakeColor(b3_colorYellow));
    if self.m_onGround != 0 {
        var bottom = b3Pos{pos.x, pos.y - RigidbodyCharacter_m_totalHeight * 0.5f, pos.z};
        DrawLine(bottom, b3OffsetPos(bottom, op_mul_float_b3Vec3(0.30000000000000004f, self.m_groundNormal)), MakeColor(b3_colorGreen));
    }
}
