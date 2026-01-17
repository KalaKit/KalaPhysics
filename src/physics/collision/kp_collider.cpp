//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.'

#include "physics/collision/kp_collider.hpp"
#include "core/kp_physics_world.hpp"

using KalaPhysics::Core::PhysicsWorld;

using std::to_string;

namespace KalaPhysics::Physics::Collision
{
	static KalaPhysicsRegistry<Collider> registry{};

	KalaPhysicsRegistry<Collider>& Collider::GetRegistry() { return registry; }

	bool Collider::IsInitialized() const { return isInitialized; }

	u32 Collider::GetID() const { return ID; }

	void Collider::SetStaticState(bool newValue) { isStatic = newValue; }
	bool Collider::IsStatic() const { return isStatic; }

	void Collider::SetTriggerState(bool newValue) { isTrigger = newValue; }
	bool Collider::IsTrigger() const { return isTrigger; }

	void Collider::SetParentRigidBody(u32 newValue) { parentRigidBody = newValue; }
	u32 Collider::GetParentRigidBody() const { return parentRigidBody; }

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

	ColliderShape Collider::GetColliderShape() const { return shape; }
	ColliderType Collider::GetColliderType() const { return type; }

	const vector<vec3>& Collider::GetVertices() const { return vertices; }
	const Transform3D& Collider::GetTransform() const { return transform; }

	void Collider::SetOnTriggerEnter(const function<void()>& func) { if (func) onTriggerEnter = func; }
	void Collider::SetOnTriggerExit(const function<void()>& func) { if (func) onTriggerExit = func; }
	void Collider::SetOnTriggerStay(const function<void()>& func) { if (func) onTriggerStay = func; }

	void Collider::ClearOnTriggerEnter() { onTriggerEnter = nullptr; }
	void Collider::ClearOnTriggerExit() { onTriggerExit = nullptr; }
	void Collider::ClearOnTriggerStay() { onTriggerStay = nullptr; }
}