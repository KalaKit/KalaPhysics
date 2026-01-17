//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#include "physics/kp_delayed_ray.hpp"

using KalaPhysics::Core::PhysicsWorld;

using std::to_string;

namespace KalaPhysics::Physics
{
	static KalaPhysicsRegistry<DelayedRay> registry{};

	KalaPhysicsRegistry<DelayedRay>& GetRegistry() { return registry; }

	u32 DelayedRay::MakeMaskFromLayers(initializer_list<u8> layers)
	{
		u32 m = 0ULL;
		for (u8 l : layers)
		{
			if (l < MAX_LAYERS) m |= (1ULL << l);
		}

		return m;
	}

	DelayedRay* DelayedRay::Initialize()
	{
		return nullptr;
	}

	bool DelayedRay::IsInitialized() const { return isInitialized; }

	u32 DelayedRay::GetID() const { return ID; }

	void DelayedRay::SetMask(u32 m) { mask = m; }
	void DelayedRay::ClearMask() { mask = 0ULL; }

	void DelayedRay::AddLayerToMask(const string& layer)
	{
		u8 foundLayer = PhysicsWorld::GetLayer(layer);

		if (foundLayer == 255)
		{
			Log::Print(
				"Cannot add layer with name '" + layer + "' for delayed ray '" + to_string(ID) + "' because the layer does not exist!",
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
				"Cannot remove layer with name '" + layer + "' for delayed ray '" + to_string(ID) + "' because the layer does not exist!",
				"DELAYED_RAY",
				LogType::LOG_ERROR,
				2);

			return;
		}

		mask &= ~(1ULL << foundLayer);
	}

	u32 DelayedRay::GetMask() const { return mask; }

	DelayedRay::~DelayedRay()
	{
		
	}

	void DelayedRay::Update(f32 deltaTime)
	{

	}
}