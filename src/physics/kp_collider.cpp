//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.'

#include "physics/kp_collider.hpp"

namespace KalaPhysics::Physics
{
	Collider* Collider::Initialize(
		ColliderShape shape,
		ColliderType type,
		u32 parentRigidbody,
		const vector<vec3>& vertices,
		const Transform3D& transform)
	{
		return nullptr;
	}
	
	Collider::~Collider()
	{
		
	}
}