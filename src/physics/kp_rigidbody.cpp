//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#include <string>

#include "physics/kp_rigidbody.hpp"
#include "physics/kp_collider.hpp"

using std::to_string;

namespace KalaPhysics::Physics
{
	RigidBody* RigidBody::Initialize()
	{
		return nullptr;
	}

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
	
	RigidBody::~RigidBody()
	{

	}
}