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

	void Collider_KDOP::Update(f32 deltaTime)
	{

	}

	Collider_KDOP::~Collider_KDOP()
	{

	}
}