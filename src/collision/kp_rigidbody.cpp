//Copyright(C) 2026 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#include "collision/kp_rigidbody.hpp"
#include "collision/kp_collider.hpp"
#include "core/kp_core.hpp"
#include "core/kp_collision.hpp"

using KalaPhysics::Core::KalaPhysicsCore;
using KalaPhysics::Core::CollisionManager;

namespace KalaPhysics::Collision
{
    RigidBody* RigidBody::Initialize(const Transform3D& transform)
    {

    }

    u8 RigidBody::GetMovementRotationLock() const { return moveRotLock; }
    void RigidBody::SetMovementRotationLock(u8 newValue) { moveRotLock = newValue; }

    Transform3D& RigidBody::GetTransform() { return transform; }

    StationaryData& RigidBody::GetStationaryData() { return stationaryData; }
    LinearMotionData& RigidBody::GetLinearMotionData() { return linearMotionData; }
    AngularMotionData& RigidBody::GetAngularMotionData() { return angularMotionData; }
    const RuntimeData& RigidBody::GetRuntimeData() const { return runtimeData; }

    void RigidBody::Destroy()
    {

    }

    void RigidBody::Update()
    {

    }

    RigidBody::~RigidBody()
    {

    }
}