//Copyright(C) 2026 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#include "log_utils.hpp"

#include "collision/kp_rigidbody.hpp"
#include "collision/kp_collider.hpp"
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
    static KalaPhysicsRegistry<RigidBody> registry{};

    KalaPhysicsRegistry<RigidBody>& RigidBody::GetRegistry() { return registry; }

    RigidBody* RigidBody::Initialize(const Transform3D& transform)
    {
        if (!CollisionManager::IsInitialized())
        {
            Log::Print(
                "Failed to initialize rigidbody because the collision manager has not yet been initialized!",
                "KP_RIGIDBODY",
                LogType::LOG_WARNING);

            return nullptr;
        }

		u32 newID = KalaPhysicsCore::GetGlobalID() + 1;
		KalaPhysicsCore::SetGlobalID(newID);

		unique_ptr<RigidBody> newRigidBody = make_unique<RigidBody>();
		RigidBody* rbPtr = newRigidBody.get();

		rbPtr->ID = newID;
        rbPtr->transform = transform;

		string err = registry.AddContent(
			newID, 
			std::move(newRigidBody));
		if (!err.empty())
		{
			KalaPhysicsCore::ForceClose(
				"KalaPhysics rigidbody error",
				"Failed to initialize rigidbody! Reason: " + err);
		}

		Log::Print(
			"Created new rigidbody '" + to_string(newID) + "'!",
			"KP_RIGIDBODY",
			LogType::LOG_SUCCESS);

		return rbPtr;
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