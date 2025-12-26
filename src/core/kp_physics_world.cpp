//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#include <vector>

#include "core/kp_physics_world.hpp"
#include "physics/kp_rigidbody.hpp"
#include "physics/collision/kp_collider.hpp"
#include "physics/collision/kp_collider_bsp.hpp"
#include "physics/collision/kp_collider_aabb.hpp"
#include "physics/collision/kp_collider_obb.hpp"
#include "physics/collision/kp_collider_bcp.hpp"
#include "physics/collision/kp_collider_kdop.hpp"
#include "physics/collision/kp_collider_bch.hpp"

using KalaPhysics::Physics::RigidBody;
using KalaPhysics::Physics::Collision::Collider;
using KalaPhysics::Physics::Collision::ColliderShape;
using KalaPhysics::Physics::Collision::Collider_BSP;
using KalaPhysics::Physics::Collision::Collider_AABB;
using KalaPhysics::Physics::Collision::Collider_OBB;
using KalaPhysics::Physics::Collision::Collider_BCP;
using KalaPhysics::Physics::Collision::Collider_KDOP;
using KalaPhysics::Physics::Collision::Collider_BCH;

using std::vector;

static vector<Collider*> activeColliders{};

namespace KalaPhysics::Core
{
	void PhysicsWorld::Update(f32 deltaTime)
	{
		//Can we even collide with this collider
		auto _can_collide = [](Collider* c)
			{
				return !c || c->isTrigger;
			};

		//Can this collider move?
		auto _can_move = [&_can_collide](Collider* c)
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

				if (!rb) return false;

				if (rb->vars.isSleeping
					|| rb->vars.mass == 0
					|| isnear(rb->vars.gravityScale)) return false;

				return true;
			};

		//Can this collider rotate?
		auto _can_rotate = [&_can_move](Collider* c)
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

				if (isnear(rb->vars.inertiaTensor)) return false;

				return true;
			};

		//
		// ENSURE ACTIVE COLLIDERS LIST IS UP TO DATE
		//

		//TODO: consider more efficient lookups in the future (unordered_set etc)

		//remove all existing invalid colliders from active colliders list
		activeColliders.erase(remove_if(
			activeColliders.begin(),
			activeColliders.end(),
			[](Collider* c)
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
		// BEGIN COLLISION AND MOTION
		//

		struct ColliderPair
		{
			Collider* a{};
			Collider* b{};
		};

		vector<ColliderPair> realCollisions{};

		//handles real collisions and motion
		auto _collide = [&realCollisions, &deltaTime](u8 substeps)
			{
				int ss = substeps;
			
				while (ss > 0)
				{
					for (auto& c : realCollisions)
					{
						switch (c.a->shape)
						{
						case ColliderShape::COLLIDER_BSP:
						{
							Collider_BSP* col = scast<Collider_BSP*>(c.a);
							col->Update(c.b, deltaTime);
							break;
						}
						case ColliderShape::COLLIDER_AABB:
						{
							Collider_AABB* col = scast<Collider_AABB*>(c.a);
							col->Update(c.b, deltaTime);
							break;
						}
						case ColliderShape::COLLIDER_OBB:
						{
							Collider_OBB* col = scast<Collider_OBB*>(c.a);
							col->Update(c.b, deltaTime);
							break;
						}
						case ColliderShape::COLLIDER_BCP:
						{
							Collider_BCP* col = scast<Collider_BCP*>(c.a);
							col->Update(c.b, deltaTime);
							break;
						}

						case ColliderShape::COLLIDER_KDOP_10_X:
						case ColliderShape::COLLIDER_KDOP_10_Y:
						case ColliderShape::COLLIDER_KDOP_10_Z:
						case ColliderShape::COLLIDER_KDOP_18:
						case ColliderShape::COLLIDER_KDOP_26:
						{
							Collider_KDOP* col = scast<Collider_KDOP*>(c.a);
							col->Update(c.b, deltaTime);
							break;
						}

						case ColliderShape::COLLIDER_BCH:
						{
							Collider_BCH* col = scast<Collider_BCH*>(c.a);
							col->Update(c.b, deltaTime);
							break;
						}
						}
					}
				}
			};

		//TODO: consider faster iterations in the future than this pairwise loop

		//only handle collisions if there are more than 1 colliders active
		if (activeColliders.size() > 1)
		{
			for (const auto& c1 : activeColliders)
			{
				for (const auto& c2 : activeColliders)
				{
					if (c1 == c2) continue; //skip self-collision

					if (CanCollide(c1->layer, c2->layer))
					{
						realCollisions.push_back({ c1, c2 });
					}
				}
			}
		}

		u8 substeps = 1;

		if (realCollisions.size() >= COLLISION_THRESHOLD)
		{
			substeps = scast<u8>(substeps * SUBSTEP_GROWTH_FACTOR);
			substeps = min(substeps, MAX_SUBSTEPS);
		}

		_collide(substeps);
	}
}