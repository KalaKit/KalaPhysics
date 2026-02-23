//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#include <string>

#include "physics/kp_rigidbody.hpp"

using std::to_string;

namespace KalaPhysics::Physics
{
	static KalaPhysicsRegistry<RigidBody> registry{};

	KalaPhysicsRegistry<RigidBody>& RigidBody::GetRegistry() { return registry; }

	RigidBody* RigidBody::Initialize()
	{
		return nullptr;
	}

	bool RigidBody::IsInitialized() const { return isInitialized; }

	u32 RigidBody::GetID() const { return ID; }

	void RigidBody::AddCollider(u32 colliderID)
	{
		//skip if max collider count was reached
		if (colliderCount >= MAX_COLLIDERS)
		{
			Log::Print(
				"Cannot add a new collider '" + to_string(colliderID) + "' for rigidbody '" + to_string(ID) + "' because the max layer count of '" + to_string(MAX_COLLIDERS) + "' colliders has been reached!",
				"COLLIDER",
				LogType::LOG_ERROR,
				2);

			return;
		}

		//cannot add what already exists
		for (u8 i = 0; i < colliderCount; i++)
		{
			if (colliders[i] == colliderID)
			{
				Log::Print(
					"Cannot add a new collider '" + to_string(colliderID) + "' for rigidbody '" + to_string(ID) + "' because the collider has already been added!",
					"COLLIDER",
					LogType::LOG_ERROR,
					2);

				return;
			}
		}

		colliders[colliderCount++] = colliderID;
	}
	void RigidBody::RemoveCollider(u32 colliderID)
	{
		for (u8 i = 0; i < colliderCount; i++)
		{
			if (colliders[i] == colliderID)
			{
				colliders[i] = colliders[--colliderCount];
				return;
			}
		}

		Log::Print(
			"Cannot remove an existing collider '" + to_string(colliderID) + "' from rigidbody '" + to_string(ID) + "' because that collider ID doesn't exist!",
			"COLLIDER",
			LogType::LOG_ERROR,
			2);
	}
	void RigidBody::RemoveAllColliders()
	{
		colliderCount = 0;
	}

	const array<u32, MAX_COLLIDERS>& RigidBody::GetAllColliders() const { return colliders; }
	u8 RigidBody::GetColliderCount() const { return colliderCount; }

	bool RigidBody::IsSleeping() const { return vars.isSleeping; }
	bool RigidBody::IsCCD() const { return vars.ccd; }

	f32 RigidBody::GetMass() const { return vars.mass; }
	void RigidBody::SetMass(f32 newValue)
	{
		vars.mass = clamp(
			newValue,
			0.0f,
			MAX_MASS);
	}

	f32 RigidBody::GetRestitution() const { return vars.restitution; }
	void RigidBody::SetRestitution(f32 newValue)
	{
		vars.restitution = clamp(
			newValue,
			0.0f,
			1.0f);
	}

	f32 RigidBody::GetLinearDamp() const { return vars.linearDamp; }
	void RigidBody::SetLinearDamp(f32 newValue)
	{
		vars.linearDamp = clamp(
			newValue,
			0.0f,
			1.0f);
	}

	f32 RigidBody::GetAngularDamp() const { return vars.angularDamp; }
	void RigidBody::SetAngularDamp(f32 newValue)
	{
		vars.angularDamp = clamp(
			newValue,
			0.0f,
			1.0f);
	}

	const vec3& RigidBody::GetGravityScale() const { return vars.gravityScale; }
	void RigidBody::SetGravityScale(const vec3& newValue)
	{
		vars.gravityScale = kclamp(
			newValue,
			vec3(0.0f),
			MAX_GRAVITY_SCALE);
	}

	const vec3& RigidBody::GetVelocity() const { return vars.velocity; }
	void RigidBody::SetVelocity(const vec3& newValue)
	{
		vars.velocity = kclamp(
			newValue,
			vec3(0.0f),
			MAX_VELOCITY);
	}

	const vec3& RigidBody::GetAngularVelocity() const { return vars.angularVelocity; }
	void RigidBody::SetAngularVelocity(const vec3& newValue)
	{
		vars.angularVelocity = kclamp(
			newValue,
			vec3(0.0f),
			MAX_ANGULAR_VELOCITY);
	}

	const mat3& RigidBody::GetInertiaTensor() const { return vars.inertiaTensor; }

	f32 RigidBody::GetAccumulatedForce() const { return vars.accumForce; }

	f32 RigidBody::GetAccumulatedTorque() const { return vars.accumTorque; }
	
	RigidBody::~RigidBody()
	{

	}
}