// box3d_mover.mc — the character controller box3d's samples share
// (samples/mover.cpp), transpiled.
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
struct MoverShapeUserData {
    f32 maxPush;
    bool clipVelocity;
}

struct PlaneExtra {
    b3Pos point;
    b3ShapeId shapeId;
}

struct CharacterMover {
    Sample* m_sample;
    b3WorldTransform m_transform;
    b3Vec3 m_velocity;
    b3Capsule m_capsule;
    b3CollisionPlane[8] m_planes;
    PlaneExtra[8] m_planeExtras;
    i32 m_planeCount;
    i32 m_totalIterations;
    f32 m_pogoVelocity;
    bool m_onGround;
    bool m_sprint;
    b3ShapeId* m_ignoreShapeIds;
    i32 m_ignoreCount;
}

i32 CharacterMover_m_planeCapacity = 8;
f32 CharacterMover_m_jumpSpeed = 5.0f;
f32 CharacterMover_m_maxSpeed = 6.0f;
f32 CharacterMover_m_minSpeed = 0.01f;
f32 CharacterMover_m_stopSpeed = 1.0f;
f32 CharacterMover_m_accelerate = 30.0f;
f32 CharacterMover_m_friction = 4.0f;
f32 CharacterMover_m_gravity = 15.0f;
private {
bool MoverFilterCallback(b3ShapeId shapeId, void* context) {
    var self = cast(CharacterMover*, context);
    for i32 i = 0; i < self.m_ignoreCount; ++i {
        if shapeId.index1 == self.m_ignoreShapeIds[i].index1 && shapeId.world0 == self.m_ignoreShapeIds[i].world0 && shapeId.generation == self.m_ignoreShapeIds[i].generation {
            return false;
        }
    }
    return true;
}
}
void CharacterMover_Initialize(CharacterMover* self, Sample* sample, b3Pos position) {
    self.m_sample = sample;
    self.m_transform.p = position;
    self.m_transform.q = b3Quat_identity;
    self.m_velocity = b3Vec3{0.0f, 0.0f, 0.0f};
    self.m_capsule = b3Capsule{b3Vec3{0.0f, -0.5f, 0.0f}, b3Vec3{0.0f, 0.5f, 0.0f}, 0.30000000000000004f};
    self.m_planeCount = 0;
    self.m_totalIterations = 0;
    self.m_pogoVelocity = 0.0f;
    self.m_onGround = false;
    self.m_sprint = false;
    self.m_ignoreShapeIds = null;
    self.m_ignoreCount = 0;
}
private {
bool PlaneResultFcn(b3ShapeId shapeId, b3PlaneResult* planeResults, i32 planeCount, void* context) {
    if MoverFilterCallback(shapeId, context) == false {
        return true;
    }
    var self = cast(CharacterMover*, context);
    f32 maxPush = 3.4028234659999994e38f;
    bool clipVelocity = true;
    var userData = cast(MoverShapeUserData*, b3Shape_GetUserData(shapeId));
    if userData != null {
        maxPush = userData.maxPush;
        clipVelocity = userData.clipVelocity;
    }
    for i32 i = 0; i < planeCount && self.m_planeCount < CharacterMover_m_planeCapacity; ++i {
        self.m_planes[self.m_planeCount] = b3CollisionPlane{.plane = planeResults[i].plane, .pushLimit = maxPush, .push = 0.0f, .clipVelocity = clipVelocity};
        self.m_planeExtras[self.m_planeCount] = PlaneExtra{.point = b3OffsetPos(self.m_transform.p, planeResults[i].point), .shapeId = shapeId};
        self.m_planeCount += 1;
    }
    return true;
}
}
void CharacterMover_SolveMove(CharacterMover* self, f32 timeStep, b3Vec3 forward, b3Vec3 right, b3Vec2 throttle, bool clipVelocity) {
    f32 speed = b3Length(self.m_velocity);
    if speed < CharacterMover_m_minSpeed {
        self.m_velocity.x = 0.0f;
        self.m_velocity.z = 0.0f;
    } else {
        f32 control = speed < CharacterMover_m_stopSpeed ? CharacterMover_m_stopSpeed : speed;
        f32 drop = control * CharacterMover_m_friction * timeStep;
        f32 newSpeed = b3MaxFloat(0.0f, speed - drop);
        f32 ratio = newSpeed / speed;
        self.m_velocity.x *= ratio;
        self.m_velocity.z *= ratio;
    }
    f32 maxSpeed = self.m_sprint != 0 ? 1.5f * CharacterMover_m_maxSpeed : CharacterMover_m_maxSpeed;
    b3Vec3 desiredVelocity = op_add_b3Vec3_b3Vec3(op_mul_float_b3Vec3(maxSpeed, op_mul_float_b3Vec3(throttle.x, forward)), op_mul_float_b3Vec3(maxSpeed, op_mul_float_b3Vec3(throttle.y, right)));
    f32 desiredSpeed;
    b3Vec3 desiredDirection = b3GetLengthAndNormalize(&desiredSpeed, desiredVelocity);
    if desiredSpeed > maxSpeed {
        desiredVelocity = op_mul_b3Vec3_float(desiredVelocity, maxSpeed / desiredSpeed);
        desiredSpeed = maxSpeed;
    }
    if self.m_onGround != 0 {
        self.m_velocity.y = 0.0f;
    }
    f32 currentSpeed = b3Dot(self.m_velocity, desiredDirection);
    f32 addSpeed = desiredSpeed - currentSpeed;
    if addSpeed > 0.0f {
        f32 accelSpeed = CharacterMover_m_accelerate * maxSpeed * timeStep;
        if accelSpeed > addSpeed {
            accelSpeed = addSpeed;
        }
        self.m_velocity = op_add_b3Vec3_b3Vec3(self.m_velocity, op_mul_float_b3Vec3(accelSpeed, desiredDirection));
    }
    self.m_velocity.y -= CharacterMover_m_gravity * timeStep;
    b3WorldId worldId = self.m_sample.m_worldId;
    f32 pogoRestLength = 3.0f * self.m_capsule.radius;
    f32 rayLength = pogoRestLength + self.m_capsule.radius;
    b3Pos rayOrigin = b3TransformWorldPoint(self.m_transform, self.m_capsule.center1);
    b3Vec3 rayTranslation = op_sub_b3Vec3(op_mul_float_b3Vec3(rayLength, b3Vec3_axisY));
    var skipTeamFilter = b3QueryFilter{.categoryBits = 1, .maskBits = cast(u64, ~2)};
    skipTeamFilter.name = "pogo";
    b3RayResult rayResult = b3World_CastRayClosest(worldId, rayOrigin, rayTranslation, skipTeamFilter);
    bool suppressPogo = self.m_velocity.y > 0.0f;
    if rayResult.hit == false || suppressPogo {
        self.m_onGround = false;
        self.m_pogoVelocity = 0.0f;
        DrawLine(rayOrigin, b3OffsetPos(rayOrigin, rayTranslation), MakeColor(b3_colorGray));
    } else {
        self.m_onGround = true;
        f32 pogoCurrentLength = rayResult.fraction * rayLength;
        f32 zeta = 0.7000000000000001f;
        f32 hertz = 4.0f;
        f32 omega = 2.0f * 3.14159265359f * hertz;
        f32 omegaH = omega * timeStep;
        self.m_pogoVelocity = (self.m_pogoVelocity - omega * omegaH * (pogoCurrentLength - pogoRestLength)) / (1.0f + 2.0f * zeta * omegaH + omegaH * omegaH);
        DrawLine(rayOrigin, rayResult.point, MakeColor(b3_colorGreen));
    }
    b3Pos startPosition = self.m_transform.p;
    b3Pos target = op_add_b3Pos_b3Vec3(op_add_b3Pos_b3Vec3(self.m_transform.p, op_mul_float_b3Vec3(timeStep, self.m_velocity)), op_mul_float_b3Vec3(timeStep, op_mul_float_b3Vec3(self.m_pogoVelocity, b3Vec3_axisY)));
    var moverFilter = b3QueryFilter{.categoryBits = 1, .maskBits = cast(u64, ~0), .id = 1, .name = "mover_collide"};
    var castFilter = b3QueryFilter{.categoryBits = 1, .maskBits = cast(u64, ~2), .id = 1, .name = "mover_cast"};
    self.m_totalIterations = 0;
    f32 tolerance = 0.01f;
    for i32 iteration = 0; iteration < 5; ++iteration {
        self.m_planeCount = 0;
        b3Capsule mover;
        mover.center1 = self.m_capsule.center1;
        mover.center2 = self.m_capsule.center2;
        mover.radius = self.m_capsule.radius;
        b3World_CollideMover(worldId, self.m_transform.p, &mover, moverFilter, cast(b3PlaneResultFcn, PlaneResultFcn), self);
        b3Vec3 targetDelta = op_sub_b3Pos_b3Pos(target, self.m_transform.p);
        b3PlaneSolverResult result = b3SolvePlanes(targetDelta, self.m_planes, self.m_planeCount);
        self.m_totalIterations += result.iterationCount;
        b3Vec3 delta = result.delta;
        f32 fraction = b3World_CastMover(worldId, self.m_transform.p, &mover, delta, castFilter, cast(b3MoverFilterFcn, MoverFilterCallback), self);
        delta = op_mul_b3Vec3_float(delta, fraction);
        self.m_transform.p = op_add_b3Pos_b3Vec3(self.m_transform.p, delta);
        if b3LengthSquared(delta) < tolerance * tolerance {
            break;
        }
    }
    for i32 i = 0; i < self.m_planeCount; ++i {
        b3BodyId bodyId = b3Shape_GetBody(self.m_planeExtras[i].shapeId);
        b3BodyType bodyType = b3Body_GetType(bodyId);
        if bodyType != b3_dynamicBody {
            continue;
        }
        b3Pos point = self.m_planeExtras[i].point;
        b3Vec3 normal = b3Neg(self.m_planes[i].plane.normal);
        f32 invMassA = 0.0f;
        f32 invMassB = b3Body_GetInverseMass(bodyId);
        b3Matrix3 invIB = b3Body_GetWorldInverseRotationalInertia(bodyId);
        b3Pos pB = b3Body_GetWorldCenter(bodyId);
        b3Vec3 rB = b3SubPos(point, pB);
        b3Vec3 rnB = b3Cross(rB, normal);
        f32 kNormal = invMassA + invMassB + b3Dot(rnB, b3MulMV(invIB, rnB));
        f32 normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;
        b3Vec3 vB = b3Body_GetLinearVelocity(bodyId);
        b3Vec3 omegaB = b3Body_GetAngularVelocity(bodyId);
        b3Vec3 vrB = b3Add(vB, b3Cross(omegaB, rB));
        f32 vn = b3Dot(b3Sub(vrB, self.m_velocity), normal);
        f32 impulse = b3MaxFloat(-normalMass * vn, 0.0f);
        b3Vec3 P = b3MulSV(impulse, normal);
        self.m_velocity = b3MulSub(self.m_velocity, invMassA, P);
        b3Body_ApplyLinearImpulse(bodyId, P, point, true);
    }
    if clipVelocity != 0 {
        self.m_velocity = b3ClipVector(self.m_velocity, self.m_planes, self.m_planeCount);
    } else if timeStep > 0.0f {
        self.m_velocity = op_mul_float_b3Vec3(1.0f / timeStep, op_sub_b3Pos_b3Pos(self.m_transform.p, startPosition));
    }
}
void CharacterMover_Step(CharacterMover* self, b3ShapeId* ignoreShapes, i32 ignoreCount, bool clipVelocity) {
    self.m_ignoreShapeIds = ignoreShapes;
    self.m_ignoreCount = ignoreCount;
    var throttle = b3Vec2{0.0f, 0.0f};
    b3Vec3 forward = op_sub_b3Vec3(Camera_GetForward(self.m_sample.m_camera));
    b3Vec3 right = Camera_GetRight(self.m_sample.m_camera);
    forward.y = 0.0f;
    forward = b3Normalize(forward);
    if self.m_sample.m_camera.m_thirdPerson != 0 {
        if IsKeyDown(87) != 0 {
            throttle.x += 1.0f;
        }
        if IsKeyDown(83) != 0 {
            throttle.x -= 1.0f;
        }
        if IsKeyDown(65) != 0 {
            throttle.y -= 1.0f;
        }
        if IsKeyDown(68) != 0 {
            throttle.y += 1.0f;
        }
        if IsKeyDown(32) && self.m_onGround == true {
            self.m_velocity.y = CharacterMover_m_jumpSpeed;
            self.m_onGround = false;
        }
        if self.m_onGround == true {
            self.m_sprint = IsKeyDown(340);
        } else {
            self.m_sprint = false;
        }
    }
    f32 hertz = self.m_sample.m_context.hertz;
    f32 timeStep = hertz > 0.0f ? 1.0f / hertz : 0.0f;
    CharacterMover_SolveMove(self, timeStep, forward, right, throttle, clipVelocity);
    b3Pos position = self.m_transform.p;
    if self.m_sample.m_camera.m_thirdPerson != 0 {
        self.m_sample.m_camera.m_pivot = position;
        Camera_UpdateTransform(self.m_sample.m_camera);
    }
    SetDrawOrigin(Camera_DrawOrigin(self.m_sample.m_camera));
    i32 count = self.m_planeCount;
    for i32 i = 0; i < count; ++i {
        b3Plane plane = self.m_planes[i].plane;
        b3Pos p1 = op_add_b3Pos_b3Vec3(position, op_mul_float_b3Vec3(plane.offset - self.m_capsule.radius, plane.normal));
        b3Pos p2 = op_add_b3Pos_b3Vec3(p1, op_mul_float_b3Vec3(0.1f, plane.normal));
        DrawPoint(p1, 5.0f, MakeColor(b3_colorYellow));
        DrawLine(p1, p2, MakeColor(b3_colorYellow));
    }
    DrawSolidCapsule(self.m_transform, self.m_capsule, MakeColor(b3_colorBlue));
    DrawLine(position, op_add_b3Pos_b3Vec3(position, self.m_velocity), MakeColor(b3_colorPurple));
    self.m_ignoreShapeIds = null;
    self.m_ignoreCount = 0;
}
