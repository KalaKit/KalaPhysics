//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#include "physics/collision/kp_collider_kdop.hpp"

namespace KalaPhysics::Physics::Collision
{
	Collider_KDOP* Collider_KDOP::Initialize(
		u32 parentRigidbody,
		const vec3& pos,
		const quat& rot,
		const vector<vec3>& vertices,
		KDOPShape shape)
	{
		return nullptr;
	}

	void Collider_KDOP::Update(Collider* c, f32 deltaTime)
	{

	}

	const vec3& Collider_KDOP::GetPos() const { return pos; }
	void Collider_KDOP::SetPos(const vec3& newValue)
	{
		pos = kclamp(newValue, MIN_KDOP_POS, MAX_KDOP_POS);
	}

	const quat& Collider_KDOP::GetRot() const { return rot; }
	void Collider_KDOP::SetRot(const quat& newValue)
	{
		rot = normalize_q(newValue);
	}

	const vector<vec3>& Collider_KDOP::GetVertices() const { return vertices; }

	KDOPShape Collider_KDOP::GetKDOPShape() const { return kdopShape; }

	Collider_KDOP::~Collider_KDOP()
	{

	}
}