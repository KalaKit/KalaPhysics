//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

//main log macro
#define WRITE_LOG(type, msg) std::cout << "[KALAKIT_PHYSICS | " << type << "] " << msg << "\n"

//log types
#if KALAPHYSICS_DEBUG
	#define LOG_DEBUG(msg) WRITE_LOG("DEBUG", msg)
#else
	#define LOG_DEBUG(msg)
#endif
#define LOG_SUCCESS(msg) WRITE_LOG("SUCCESS", msg)
#define LOG_ERROR(msg) WRITE_LOG("ERROR", msg)

//external
#include "glm.hpp"

//physics
#include "simulation/stepsimulation.hpp"
#include "simulation/contactsolver.hpp"
#include "simulation/frictionsolver.hpp"
#include "simulation/handlemotion.hpp"

using glm::degrees;
using glm::radians;
using glm::clamp;
using glm::min;
using glm::max;
using glm::quat;

namespace KalaKit::Physics::Simulation
{
	ContactSolver contactSolver;
	FrictionSolver frictionSolver;

	bool StepSimulation::IsValidCollision(RigidBody& bodyA, RigidBody& bodyB)
	{
		//avoid sleeping bodies

		if (bodyA.isSleeping
			&& bodyB.isSleeping)
		{
			return false;
		}

		//avoid non-collider bodies

		if (!bodyA.collider
			|| !bodyB.collider)
		{
			return false;
		}

		//avoid same body

		if (&bodyA == &bodyB)
		{
			return false;
		}

		//avoid non-dynamic and non-gravity bodies

		bool bodyMoves =
			bodyA.isDynamic
			|| bodyA.useGravity;
		bool otherMoves =
			bodyB.isDynamic
			|| bodyB.useGravity;
		if (!bodyMoves
			&& !otherMoves)
		{
			return false;
		}

		//bounding sphere distance culling

		vec3 posA = bodyA.position;
		vec3 posB = bodyB.position;

		float radiusA = bodyA.collider->boundingRadius;
		float radiusB = bodyB.collider->boundingRadius;

		if (length(posA - posB) > (radiusA + radiusB))
		{
			return false;
		}

		return true;
	}

	void StepSimulation::Step(PhysicsWorld& world, float deltaTime)
	{
		auto& bodies = world.GetBodies();
		if (bodies.size() == 0) return;

		for (RigidBody* body : world.GetBodies())
		{
			if (body 
				&& body->collider)
			{
				body->collider->SetGroundedState(false);
				body->collider->SetGroundNormal(vec3(0.0f, 1.0f, 0.0f));
			}
		}

		for (size_t i = 0; i < bodies.size(); i++)
		{
			RigidBody& bodyA = *bodies[i];

			for (size_t j = i + 1; j < bodies.size(); j++)
			{
				RigidBody& bodyB = *bodies[j];

				if (!IsValidCollision(bodyA, bodyB)) continue;

				auto& colA = *bodyA.collider;
				auto& colB = *bodyB.collider;

				auto manifold = colA.GenerateContacts(bodyA, bodyB, colB);
				if (manifold.contacts.empty()) continue;

				for (const auto& contact : manifold.contacts)
				{
					auto& contactSolv = contactSolver.AddContact(
						world,
						&bodyA,
						&bodyB,
						contact.point,
						contact.normal,
						contact.penetration,
						deltaTime);

					frictionSolver.AddFrictionPair(
						&bodyA,
						&bodyB,
						contact.point,
						contact.normal,
						bodyA.staticFriction,
						bodyA.dynamicFriction,
						&contactSolv);

					const float angleToUp = 
						degrees(acos(clamp(
						dot(contact.normal, vec3(0, 1, 0)), 
						-1.0f, 1.0f)));

					if (angleToUp <= world.GetAngleLimit()
						&& bodyA.isDynamic)
					{
						if (bodyA.collider)
						{
							bodyA.collider->SetGroundedState(true);
							bodyA.collider->SetGroundNormal(contact.normal);
						}
					}
					else
					{
						const float reverseAngle = 
							degrees(acos(clamp(
							dot(-contact.normal, vec3(0, 1, 0)), 
							-1.0f, 1.0f)));

						if (reverseAngle <= world.GetAngleLimit()
							&& bodyB.isDynamic)
						{
							if (bodyB.collider)
							{
								bodyB.collider->SetGroundedState(true);
								bodyB.collider->SetGroundNormal(-contact.normal);
							}
						}
					}
				}
			}
		}

		//loop over all contact points 10 times, adjusting the objects' 
		//velocities and impulses a bit each time until everything stabilizes

		contactSolver.Solve(deltaTime, 10);
		frictionSolver.Solve(deltaTime, 10);

		contactSolver.Clear();
		frictionSolver.Clear();

		//and finally handle all the forces, dampening and sleep 

		HandleMotion::Run(world, deltaTime);
	}
}