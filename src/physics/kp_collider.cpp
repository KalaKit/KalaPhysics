//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.'

#include "physics/kp_collider.hpp"
#include "core/kp_physics_world.hpp"

using KalaPhysics::Core::PhysicsWorld;

using std::to_string;

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

	void Collider::SetLayer(const string& newLayer)
	{
		u8 foundLayer = PhysicsWorld::GetLayer(newLayer);
		if (foundLayer == 255)
		{
			Log::Print(
				"Cannot set layer with name '" + newLayer + "' for collider '" + to_string(ID) + "' because the layer does not exist!",
				"COLLIDER",
				LogType::LOG_ERROR,
				2);

			return;
		}

		layer = foundLayer;
	}
	string Collider::GetLayer()
	{
		if (layer == 255) return "NONE";

		string foundLayer = PhysicsWorld::GetLayer(layer);
		if (foundLayer == "NONE")
		{
			Log::Print(
				"Cannot get layer for collider '" + to_string(ID) + "' because the layer does not exist!",
				"COLLIDER",
				LogType::LOG_ERROR,
				2);
		}

		return foundLayer;
	}

	void Collider::Update(f32 deltaTime)
	{

	}
	
	Collider::~Collider()
	{
		
	}
}