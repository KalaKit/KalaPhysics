//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#include "physics/kp_delayed_ray.hpp"

using KalaPhysics::Core::PhysicsWorld;

namespace KalaPhysics::Physics
{
	DelayedRay* DelayedRay::Initialize()
	{
		return nullptr;
	}

	void DelayedRay::AddLayerToMask(const string& layer)
	{
		u8 foundLayer = PhysicsWorld::GetLayer(layer);

		if (foundLayer == 255)
		{
			Log::Print(
				"Cannot add layer with name '" + layer + "' because it does not exist!",
				"DELAYED_RAY",
				LogType::LOG_ERROR,
				2);

			return;
		}

		mask |= (1ULL << foundLayer);
	}
	void DelayedRay::RemoveLayerFromMask(const string& layer)
	{
		u8 foundLayer = PhysicsWorld::GetLayer(layer);

		if (foundLayer == 255)
		{
			Log::Print(
				"Cannot remove layer with name '" + layer + "' because it does not exist!",
				"DELAYED_RAY",
				LogType::LOG_ERROR,
				2);

			return;
		}

		mask &= ~(1ULL << foundLayer);
	}

	DelayedRay::~DelayedRay()
	{
		
	}

	void DelayedRay::Update(f32 deltaTime)
	{

	}
}