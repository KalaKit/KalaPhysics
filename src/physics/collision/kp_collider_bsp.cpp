//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#include <vector>
#include <memory>

#include "KalaHeaders/math_utils.hpp"

#include "physics/collision/kp_collider_bsp.hpp"
#include "physics/kp_rigidbody.hpp"
#include "core/kp_core.hpp"

using KalaHeaders::KalaMath::vec3;
using KalaHeaders::KalaMath::PI;

using KalaPhysics::Physics::Collision::SPHERE_QUALITY;
using KalaPhysics::Physics::RigidBody;
using KalaPhysics::Core::KalaPhysicsCore;

using std::vector;
using std::to_string;
using std::make_unique;
using std::unique_ptr;

static vector<vec3> GenerateSphere(f32 radius);

namespace KalaPhysics::Physics::Collision
{
	Collider_BSP* Collider_BSP::Initialize(
		u32 parentRigidBody,
		const vec3& center,
		f32 radius)
	{
		u32 newID = KalaPhysicsCore::GetGlobalID() + 1;
		KalaPhysicsCore::SetGlobalID(newID);

		unique_ptr<Collider_BSP> newCol = make_unique<Collider_BSP>();
		Collider_BSP* colPtr = newCol.get();

		Log::Print(
			"Creating new BSP collider with ID '" + to_string(newID) + "'.",
			"BSP_COLLIDER",
			LogType::LOG_DEBUG);

		colPtr->ID = newID;

		if (parentRigidBody != 0)
		{
			RigidBody* rb = RigidBody::GetRegistry().GetContent(parentRigidBody);

			if (rb == nullptr)
			{
				Log::Print(
					"Cannot add parent rigidbody for BSP collider with ID '" + to_string(newID) + "' because that rigidbody does not exist!",
					"BSP_COLLIDER",
					LogType::LOG_ERROR,
					2);
			}
			else
			{
				if (rb->GetColliderCount() >= MAX_COLLIDERS)
				{
					Log::Print(
						"Cannot add parent rigidbody for BSP collider with ID '" + to_string(newID) + "' because that rigidbody already has a max number of colliders!",
						"BSP_COLLIDER",
						LogType::LOG_ERROR,
						2);
				}
				else
				{
					colPtr->parentRigidBody = parentRigidBody;
					rb->AddCollider(newID);

					Log::Print(
						"Added BSP collider with ID '" + to_string(newID) + "' to rigidbody with ID '" + to_string(parentRigidBody) + "'!",
						"BSP_COLLIDER",
						LogType::LOG_SUCCESS);
				}
			}
		}

		colPtr->SetCenter(center);
		colPtr->SetRadius(radius);
		colPtr->vertices = std::move(GenerateSphere(colPtr->radius));

		GetRegistry().AddContent(newID, std::move(newCol));

		colPtr->isInitialized = true;

		Log::Print(
			"Created new BSP collider with ID '" + to_string(newID) + "'!",
			"BSP_COLLIDER",
			LogType::LOG_SUCCESS);

		return colPtr;
	}

	void Collider_BSP::Update(Collider* c, f32 deltaTime)
	{

	}

	const vec3& Collider_BSP::GetCenter() const { return center; }
	void Collider_BSP::SetCenter(const vec3& newValue)
	{
		center = kclamp(newValue, MIN_BSP_CENTER, MAX_BSP_CENTER);
	}

	f32 Collider_BSP::GetRadius() const { return radius; }
	void Collider_BSP::SetRadius(f32 newValue)
	{
		radius = clamp(newValue, MIN_BSP_RADIUS, MAX_BSP_RADIUS);
	}

	Collider_BSP::~Collider_BSP()
	{

	}
}

vector<vec3> GenerateSphere(f32 radius)
{
	vector<vec3> vertices{};

	for (int i = 0; i < SPHERE_QUALITY; ++i)
	{
		//vertex angle
		f32 theta = scast<f32>(i) / SPHERE_QUALITY * 2.0f * PI;

		//x, y and z coordinates

		f32 x = radius * sinf(theta) * cosf(0.0f);
		f32 y = radius * sinf(theta) * sinf(0.0f);
		f32 z = radius * cosf(theta);

		vertices.push_back({x, y, z});
	}

	return vertices;
}