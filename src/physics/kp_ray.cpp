//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#include "physics/kp_ray.hpp"
#include "physics/collision/kp_collider.hpp"

using KalaPhysics::Core::PhysicsWorld;

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

	void Ray::AddLayerToMask(const string& layer)
	{
		u8 foundLayer = PhysicsWorld::GetLayer(layer);

		if (foundLayer == 255)
		{
			Log::Print(
				"Cannot add layer with name '" + layer + "' because it does not exist!",
				"RAY",
				LogType::LOG_ERROR,
				2);

			return;
		}

		mask |= (1ULL << foundLayer);
	}
	void Ray::RemoveLayerFromMask(const string& layer)
	{
		u8 foundLayer = PhysicsWorld::GetLayer(layer);

		if (foundLayer == 255)
		{
			Log::Print(
				"Cannot remove layer with name '" + layer + "' because it does not exist!",
				"RAY",
				LogType::LOG_ERROR,
				2);

			return;
		}

		mask &= ~(1ULL << foundLayer);
	}
}