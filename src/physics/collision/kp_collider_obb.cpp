//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#include "physics/collision/kp_collider_obb.hpp"

namespace KalaPhysics::Physics::Collision
{
	Collider_OBB* Collider_OBB::Initialize(
		u32 parentRigidbody,
		const vec3& pos,
		const quat& rot,
		const vec3& halfExtents,
		ColliderType type)
	{
		return nullptr;
	}

	void Collider_OBB::Update(Collider* c, f32 deltaTime)
	{

	}

	Collider_OBB::~Collider_OBB()
	{

	}
}