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

	const vec3& Collider_OBB::GetPos() const { return pos; }
	void Collider_OBB::SetPos(const vec3& newValue)
	{
		pos = kclamp(newValue, MIN_OBB_POS, MAX_OBB_POS);
	}

	const quat& Collider_OBB::GetRot() const { return rot; }
	void Collider_OBB::SetRot(const quat& newValue)
	{
		rot = normalize_q(newValue);
	}

	const vec3& Collider_OBB::GetHalfExtents() const { return halfExtents; }
	void Collider_OBB::SetHalfExtents(const vec3& newValue)
	{
		halfExtents = kclamp(newValue, MIN_OBB_HALF_EXTENTS, MAX_OBB_HALF_EXTENTS);
	}

	Collider_OBB::~Collider_OBB()
	{

	}
}