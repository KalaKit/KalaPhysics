//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

//physics
#include "collision/collider.hpp"

namespace KalaKit::Physics::Collision
{
	Collider::Collider(
		ColliderType type, 
		const GameObjectHandle& h) : 
		type(type), 
		handle(h) {}

	BoxCollider::BoxCollider(
		const GameObjectHandle& h) : 
		Collider(ColliderType::BOX, h) {}

	SphereCollider::SphereCollider(
		const GameObjectHandle& h) : 
		Collider(ColliderType::SPHERE, h) {}
}