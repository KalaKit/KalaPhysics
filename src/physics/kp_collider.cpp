//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.'

#include "physics/kp_collider.hpp"
#include "core/kp_physics_world.hpp"

using KalaPhysics::Core::PhysicsWorld;

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
				"Cannot set layer with name '" + newLayer + "' because it does not exist!",
				"COLLIDER",
				LogType::LOG_ERROR,
				2);

			return;
		}

		layer = foundLayer;
	}
	void Collider::ClearLayer()
	{ 
		layer = 0;
	}

	void Collider::Update(f32 deltaTime)
	{

	}
	
	Collider::~Collider()
	{
		
	}
}