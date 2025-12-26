//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#include "physics/collision/kp_collider_aabb.hpp"

namespace KalaPhysics::Physics::Collision
{
	Collider_AABB* Collider_AABB::Initialize(
		u32 parentRigidbody,
		const vec3& minCorner,
		const vec3& maxCorner)
	{
		return nullptr;
	}

	void Collider_AABB::Update(Collider* c, f32 deltaTime)
	{

	}

	Collider_AABB::~Collider_AABB()
	{

	}
}