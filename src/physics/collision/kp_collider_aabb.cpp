//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#include <vector>
#include <memory>

#include "KalaHeaders/math_utils.hpp"

#include "physics/collision/kp_collider_aabb.hpp"
#include "physics/kp_rigidbody.hpp"
#include "core/kp_core.hpp"

using KalaHeaders::KalaMath::vec3;

using KalaPhysics::Physics::RigidBody;
using KalaPhysics::Physics::MAX_COLLIDERS;
using KalaPhysics::Core::KalaPhysicsCore;

using std::vector;
using std::to_string;
using std::make_unique;
using std::unique_ptr;

static vector<vec3> GenerateCube(
	const vec3& minCorner,
	const vec3& maxCorner);

namespace KalaPhysics::Physics::Collision
{
	Collider_AABB* Collider_AABB::Initialize(
		u32 parentRigidBody,
		const vec3& minCorner,
		const vec3& maxCorner)
	{
		u32 newID = ++KalaPhysicsCore::globalID;
		unique_ptr<Collider_AABB> newCol = make_unique<Collider_AABB>();
		Collider_AABB* colPtr = newCol.get();

		Log::Print(
			"Creating new AABB collider with ID '" + to_string(newID) + "'.",
			"AABB_COLLIDER",
			LogType::LOG_DEBUG);

		colPtr->ID = newID;

		if (parentRigidBody != 0)
		{
			RigidBody* rb = RigidBody::registry.GetContent(parentRigidBody);

			if (rb == nullptr)
			{
				Log::Print(
					"Cannot add parent rigidbody for AABB collider with ID '" + to_string(newID) + "' because that rigidbody does not exist!",
					"AABB_COLLIDER",
					LogType::LOG_ERROR,
					2);
			}
			else
			{
				if (rb->GetColliderCount() >= MAX_COLLIDERS)
				{
					Log::Print(
						"Cannot add parent rigidbody for AABB collider with ID '" + to_string(newID) + "' because that rigidbody already has a max number of colliders!",
						"AABB_COLLIDER",
						LogType::LOG_ERROR,
						2);
				}
				else
				{
					colPtr->parentRigidBody = parentRigidBody;
					rb->AddCollider(newID);

					Log::Print(
						"Added AABB collider with ID '" + to_string(newID) + "' to rigidbody with ID '" + to_string(parentRigidBody) + "'!",
						"AABB_COLLIDER",
						LogType::LOG_SUCCESS);
				}
			}
		}

		colPtr->SetMinCorner(minCorner);
		colPtr->SetMaxCorner(maxCorner);
		colPtr->vertices = move(GenerateCube(colPtr->minCorner, colPtr->maxCorner));

		registry.AddContent(newID, move(newCol));

		colPtr->isInitialized = true;

		Log::Print(
			"Created new AABB collider with ID '" + to_string(newID) + "'!",
			"AABB_COLLIDER",
			LogType::LOG_SUCCESS);

		return colPtr;
	}

	void Collider_AABB::Update(Collider* c, f32 deltaTime)
	{

	}

	Collider_AABB::~Collider_AABB()
	{

	}
}

vector<vec3> GenerateCube(
	const vec3& minCorner,
	const vec3& maxCorner)
{
	return{};
}