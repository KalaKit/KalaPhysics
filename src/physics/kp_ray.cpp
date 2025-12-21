//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#include "physics/kp_ray.hpp"
#include "physics/kp_collider.hpp"

namespace KalaPhysics::Physics
{
	bool Ray::HitAny(
		const vec3& origin,
		const vec3& direction,
		f32 maxDistance)
	{
		return false;
	}

	Collider* Ray::HitCollider(
		const vec3& origin,
		const vec3& direction,
		f32 maxDistance)
	{
		return nullptr;
	}
}