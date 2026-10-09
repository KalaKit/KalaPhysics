//Copyright(C) 2026 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#include "collision/kp_collider.hpp"
#include "collision/kp_collision.hpp"

namespace KalaPhysics::Collision
{
    Collider* Collider::Initialize(
        ColliderType colliderType,
        const Transform3D& transform,
        const vector<vec3>& vertices,
        const vector<u32>& indices)
    {
        if (!CollisionHandler::IsInitialized())
        {
            //not allowed...
            return nullptr;
        }
    }

    u32 Collider::GetID() const { return ID; }

    u8 Collider::GetMovementRotationLock() const { return moveRotLock; }
    void Collider::SetMovementRotationLock(u8 newValue) { moveRotLock = newValue; }

    const bitset<256>& Collider::GetCollisionLayers() const { return collisionLayers; }
    //Override all collision layer states
    void Collider::SetCollisionLayers(bitset<256>&& newValue)
    {

    }
    void Collider::SetCollisionLayerState(
        u8 newValue,
        bool state)
    {

    }

    bool Collider::IsEnabled() const { return isEnabled; }
    void Collider::SetEnabledState(bool newValue)
    {
        if (isEnabled == newValue)
        {
            //not allowed...
            return;
        }

        isEnabled = newValue;
    }

    bool Collider::IsTrigger() const { return isTrigger; }
    void Collider::SetTriggerState(bool newValue)
    {
        if (isTrigger == newValue)
        {
            //not allowed...
            return;
        }

        isTrigger = newValue;
    }

    ColliderType Collider::GetColliderType() const { return colliderType; }
    void Collider::SetColliderType(ColliderType newValue)
    {
        if (newValue == colliderType)
        {
            //not allowed...
            return;
        }

        colliderType = newValue;
    }

    Transform3D& Collider::GetTransform() { return transform; }

    const vector<vec3>& Collider::GetVertices() const { return vertices; }
    void Collider::SetVertices(const vector<vec3>& newValue)
    {
        if (colliderType != ColliderType::CT_MESH)
        {
            //not allowed...
            return;
        }
        if (newValue.empty())
        {
            //not allowed...
            return;
        }
        if (vertices == newValue)
        {
            //not allowed...
            return;
        }

        vertices = newValue;
    }

    const vector<u32>& Collider::GetIndices() const { return indices; }
    void Collider::SetIndices(const vector<u32>& newValue)
    {
        if (colliderType != ColliderType::CT_MESH)
        {
            //not allowed...
            return;
        }
        if (newValue.empty())
        {
            //not allowed...
            return;
        }
        if (indices == newValue)
        {
            //not allowed...
            return;
        }

        indices = newValue;
    }

    void Collider::OnCollisionEvent(const function<void(u32)>& newValue)
    {
        collisionEvent = newValue;
    }
    void Collider::OnTriggerEnterEvent(const function<void(u32)>& newValue)
    {
        triggerEnterEvent = newValue;
    }
        void Collider::OnTriggerStayEvent(const function<void(u32)>& newValue)
    {
        triggerStayEvent = newValue;
    }
        void Collider::OnTriggerExitEvent(const function<void(u32)>& newValue)
    {
        triggerExitEvent = newValue;
    }

    StationaryData& Collider::GetStationaryData() { return stationaryData; }
    LinearMotionData& Collider::GetLinearMotionData() { return linearMotionData; }
    AngularMotionData& Collider::GetAngularMotionData() { return angularMotionData; }
    const RuntimeData& Collider::GetRuntimeData() const { return runtimeData; }

    void Collider::Destroy()
    {

    }

    void Collider::Update()
    {

    }

    Collider::~Collider()
    {

    }
}