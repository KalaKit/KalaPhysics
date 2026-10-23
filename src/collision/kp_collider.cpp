//Copyright(C) 2026 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#include "log_utils.hpp"

#include "collision/kp_collider.hpp"
#include "collision/kp_rigidbody.hpp"
#include "core/kp_core.hpp"
#include "core/kp_collision.hpp"

using KalaHeaders::KalaLog::Log;
using KalaHeaders::KalaLog::LogType;

using KalaPhysics::Core::KalaPhysicsCore;
using KalaPhysics::Core::CollisionManager;

using std::string;
using std::to_string;
using std::unique_ptr;
using std::make_unique;

namespace KalaPhysics::Collision
{
    static KalaPhysicsRegistry<Collider> registry{};

    KalaPhysicsRegistry<Collider>& Collider::GetRegistry() { return registry; }

    Collider* Collider::Initialize(
        ColliderType colliderType,
        const Transform3D& transform)
    {
        if (!CollisionManager::IsInitialized())
        {
            Log::Print(
                "Failed to initialize collider because the collision manager has not yet been initialized!",
                "KP_COLLIDER",
                LogType::LOG_WARNING);

            return nullptr;
        }

		u32 newID = KalaPhysicsCore::GetGlobalID() + 1;
		KalaPhysicsCore::SetGlobalID(newID);

		unique_ptr<Collider> newCollider = make_unique<Collider>();
		Collider* cPtr = newCollider.get();

		cPtr->ID = newID;
        cPtr->colliderType = colliderType;
        cPtr->transform = transform;

		string err = registry.AddContent(
			newID, 
			std::move(newCollider));
		if (!err.empty())
		{
			KalaPhysicsCore::ForceClose(
				"KalaPhysics collider error",
				"Failed to initialize collider! Reason: " + err);
		}

		Log::Print(
			"Created new collider '" + to_string(newID) + "'!",
			"KP_COLLIDER",
			LogType::LOG_SUCCESS);

		return cPtr;
    }

    u32 Collider::GetID() const { return ID; }

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

    Transform3D& Collider::GetTransform() { return transform; }

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

    const vector<Vertex>& Collider::GetVertices() const { return vertices; }
    void Collider::SetVertices(const vector<Vertex>& newValue)
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

    void Collider::SetOnCollisionEnterEvent(const function<void(u32)>& newValue)
    {
        onCollisionEnter = newValue;
    }
    void Collider::SetOnCollisionStayEvent(const function<void(u32)>& newValue)
    {
        onCollisionStay = newValue;
    }
    void Collider::SetOnCollisionExitEvent(const function<void(u32)>& newValue)
    {
        onCollisionExit = newValue;
    }
    void Collider::SetOnTriggerEnterEvent(const function<void(u32)>& newValue)
    {
        onTriggerEnter = newValue;
    }
    void Collider::SetOnTriggerStayEvent(const function<void(u32)>& newValue)
    {
        onTriggerStay = newValue;
    }
    void Collider::SetOnTriggerExitEvent(const function<void(u32)>& newValue)
    {
        onTriggerExit = newValue;
    }

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