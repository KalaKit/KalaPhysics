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
		u32 newID = KalaPhysicsCore::GetGlobalID() + 1;
		KalaPhysicsCore::SetGlobalID(newID);

		unique_ptr<Collider_AABB> newCol = make_unique<Collider_AABB>();
		Collider_AABB* colPtr = newCol.get();

		Log::Print(
			"Creating new AABB collider with ID '" + to_string(newID) + "'.",
			"AABB_COLLIDER",
			LogType::LOG_DEBUG);

		colPtr->ID = newID;

		if (parentRigidBody != 0)
		{
			RigidBody* rb = RigidBody::GetRegistry().GetContent(parentRigidBody);

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
		colPtr->vertices = std::move(GenerateCube(colPtr->minCorner, colPtr->maxCorner));

		GetRegistry().AddContent(newID, std::move(newCol));

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

	const vec3& Collider_AABB::GetMinCorner() const { return minCorner; }
	void Collider_AABB::SetMinCorner(const vec3& newValue)
	{
		minCorner = kclamp(newValue, MIN_AABB_CORNER, MAX_AABB_CORNER);
		maxCorner = kclamp(maxCorner, minCorner + MIN_AABB_CORNER_DISTANCE, MAX_AABB_CORNER);
	}

	const vec3& Collider_AABB::GetMaxCorner() const { return maxCorner; }
	void Collider_AABB::SetMaxCorner(const vec3& newValue)
	{
		maxCorner = kclamp(newValue, MIN_AABB_CORNER, MAX_AABB_CORNER);
		minCorner = kclamp(minCorner, MIN_AABB_CORNER, maxCorner - MIN_AABB_CORNER_DISTANCE);
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