//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#include "physics/collision/kp_collider_bch.hpp"

namespace KalaPhysics::Physics::Collision
{
	Collider_BCH* Collider_BCH::Initialize(
		u32 parentRigidbody,
		const vec3& pos,
		const quat& rot,
		const vector<vec3>& vertices)
	{
		return nullptr;
	}

	void Collider_BCH::Update(Collider* c, f32 deltaTime)
	{

	}

	const vec3& Collider_BCH::GetPos() const { return pos; }
	void Collider_BCH::SetPos(const vec3& newValue)
	{
		pos = kclamp(newValue, MIN_BCH_POS, MAX_BCH_POS);
	}

	const quat& Collider_BCH::GetRot() const { return rot; }
	void Collider_BCH::SetRot(const quat& newValue)
	{
		rot = normalize_q(newValue);
	}

	const vector<vec3>& Collider_BCH::GetVertices() const { return vertices; }

	Collider_BCH::~Collider_BCH()
	{

	}
}