//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#include <vector>

#include "core/kp_physics_world.hpp"
#include "physics/kp_rigidbody.hpp"
#include "physics/collision/kp_collider.hpp"

using KalaPhysics::Physics::RigidBody;
using KalaPhysics::Physics::Collision::Collider;
using KalaPhysics::Physics::Collision::ColliderShape;

using std::vector;

static vector<Collider*> activeColliders{};

namespace KalaPhysics::Core
{
	void PhysicsWorld::Update(
		f32 deltaTime,
		u8 substeps)
	{
		//Can we even collide with this collider
		auto _can_collide = [&](Collider* c)
			{
				return !c || c->isTrigger;
			};

		//Can this collider move?
		auto _can_move = [&](Collider* c)
			{
				//fast path to quickly check legitimate static models
				if (!_can_collide(c)
					|| isnear(gravity)
					|| c->isStatic
					|| c->parentRigidBody == 0)
				{
					return false;
				}

				RigidBody* rb = RigidBody::registry.GetContent(c->parentRigidBody);

				//did not find parent rb so dont even bother with further checks
				if (!rb) return false;

				//don't calculate movement for collider if
				//no mass or gravity scale exists for its rigidbody
				if (rb->vars.mass == 0
					|| isnear(rb->vars.gravityScale)) return false;

				return true;
			};

		//Can this collider rotate?
		auto _can_rotate = [&](Collider* c)
			{
				//early skip if this collider can't even move
				if (!_can_move(c)) return false;

				//bounding sphere and aabb can never rotate
				if (c->shape == ColliderShape::COLLIDER_BSP
					|| c->shape == ColliderShape::COLLIDER_AABB)
				{
					return false;
				}

				RigidBody* rb = RigidBody::registry.GetContent(c->parentRigidBody);

				//cannot rotate if inertia tensor is near 0
				if (isnear(rb->vars.inertiaTensor)) return false;

				return true;
			};

		//
		// ENSURE ACTIVE COLLIDERS LIST IS UP TO DATE
		//

		//remove all existing invalid colliders from active colliders list
		activeColliders.erase(remove_if(
			activeColliders.begin(),
			activeColliders.end(),
			[&](Collider* c)
			{
				return !c;
			}), activeColliders.end());

		//add new valid colliders to active colliders list
		for (const auto& c : Collider::registry.runtimeContent)
		{
			if (c
				&& find(
					activeColliders.begin(),
					activeColliders.end(),
					c)
					== activeColliders.end())
			{
				activeColliders.push_back(c);
			}
		}

		//
		// BEGIN COLLISION HANDLING
		//

		//early exit if no colliders exist
		if (activeColliders.empty()) return;

		for (const auto& c : activeColliders)
		{

		}
	}
}