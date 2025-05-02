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
					contactSolver.AddContact(
						&bodyA,
						&bodyB,
						contact.point,
						contact.normal,
						contact.penetration);

					frictionSolver.AddFrictionPair(
						&bodyA,
						&bodyB,
						contact.point,
						contact.normal,
						bodyA.staticFriction,
						bodyA.dynamicFriction);
				}
			}
		}

		//loop over all contact points 10 times, adjusting the objects' 
		//velocities and impulses a bit each time until everything stabilizes

		contactSolver.Solve(deltaTime, 10);
		frictionSolver.Solve(deltaTime, 10);

		contactSolver.Clear();
		frictionSolver.Clear();

		ApplyPhysicsIntegration(world, deltaTime);
	}

	void StepSimulation::ApplyPhysicsIntegration(PhysicsWorld& world, float deltaTime)
	{
		auto& bodies = world.GetBodies();
		for (auto& bodyPtr : bodies)
		{
			RigidBody& body = *bodyPtr;

			if (!body.isDynamic
				|| !body.useGravity
				|| !body.collider)
			{
				continue;
			}

			body.UpdateCenterOfGravity();

			if (body.useGravity)
			{
				vec3 gravityImpulse = (world.GetGravity() * body.gravityFactor) * deltaTime;
				body.ApplyImpulse(gravityImpulse * body.mass);
			}

			//predict future rotation using angular velocity
			quat angularRotation = quat(
				0,
				body.angularVelocity.x,
				body.angularVelocity.y,
				body.angularVelocity.z)
				* 0.5f * deltaTime;

			quat futureRotation = normalize(
				body.rotation
				+ angularRotation
				* body.rotation);

			PredictCollision(world, bodyPtr, body, deltaTime);

			//apply simple Euler integration
			body.position += body.velocity * deltaTime;

			//apply angular velocity
			if (length(body.angularVelocity) > 0.001f)
			{
				body.rotation = futureRotation;
			}

			body.tiltTimer += deltaTime;
			if (body.tiltTimer >= 0.05f)
			{
				if (CanTilt(body)) TiltBody(body);
				body.tiltTimer = 0.0f;
			}

			//dampen angular velocity over time
			float constantDampingFactor = 0.95f;
			body.angularVelocity *= pow(constantDampingFactor, deltaTime * 60.0f);

			//small damping at close to face
			if (body.angleToFlat >= 2.0f
				&& body.angleToFlat < 15.0f)
			{
				body.angularVelocity *= 0.90f;
			}

			//apply small residual damping if the rotation is minimal
			if (length(body.angularVelocity) < 0.05f)
			{
				body.angularVelocity *= 0.85f;
			}

			//apply linear damping (reduces endless sliding)
			float linearDampingFactor = pow(0.99f, deltaTime * 60.0f);
			body.velocity *= linearDampingFactor;

			if (length(body.velocity) < body.sleepThreshold
				&& length(body.angularVelocity) < body.sleepThreshold)
			{
				body.sleepTimer += deltaTime;
				if (body.sleepTimer > 2.0f)
				{
					body.Sleep();
				}
			}
			else
			{
				body.sleepTimer = 0.0f;
				body.WakeUp();
			}
		}
	}

	void StepSimulation::PredictCollision(
		PhysicsWorld& world,
		RigidBody* bodyPtr,
		RigidBody& body,
		float deltaTime)
	{
		auto& bodies = world.GetBodies();

		vec3 originalVelocity = body.velocity;
		const float maxWalkableAngle = radians(world.GetAngleLimit());

		const float groundingProbeDistance = 0.05f;

		//if the body is not moving, thest slightly below to stay grounded on slopes
		if (dot(originalVelocity, originalVelocity) < 0.0001f)
		{
			vec3 probePos = body.position + vec3(0, -groundingProbeDistance, 0);

			for (auto& otherBodyPtr : bodies)
			{
				RigidBody& otherBody = *otherBodyPtr;
				if (!IsValidCollision(body, otherBody)) continue;

				RigidBody tempBody = body;
				tempBody.position = probePos;

				auto& colA = *tempBody.collider;
				auto& colB = *otherBody.collider;
				auto manifold = colA.GenerateContacts(tempBody, otherBody, colB);

				if (!manifold.contacts.empty())
				{
					float smallestAngle = FLT_MAX;

					//find the smallest angle between any contact normal and 'up'
					for (const auto& contact : manifold.contacts)
					{
						vec3 n = normalize(contact.normal);
						float angleToUp = acos(clamp(dot(n, vec3(0, 1, 0)), -1.0f, 1.0f));
						smallestAngle = min(smallestAngle, angleToUp);
					}

					if (smallestAngle < maxWalkableAngle)
					{
						body.velocity.y = 0.0f;
						body.position.y -= groundingProbeDistance;
					}
				}
			}
		}

		//test each axis separately to allow natural sliding along walls or slopes
		for (int axis = 0; axis < 3; ++axis)
		{
			vec3 axisVelocity(0.0f);
			axisVelocity[axis] = originalVelocity[axis];

			vec3 testPos = body.position + axisVelocity * deltaTime;

			bool cancelAxis = false;

			for (auto& otherBodyPtr : bodies)
			{
				RigidBody& otherBody = *otherBodyPtr;
				if (!IsValidCollision(body, otherBody)) continue;

				RigidBody tempBody = body;
				tempBody.position = testPos;

				auto& colA = *tempBody.collider;
				auto& colB = *otherBody.collider;
				auto manifold = colA.GenerateContacts(tempBody, otherBody, colB);

				if (!manifold.contacts.empty())
				{
					float smallestAngle = FLT_MAX;

					//find the smallest angle between any contact normal and 'up'
					for (const auto& contact : manifold.contacts)
					{
						vec3 n = normalize(contact.normal);
						float angleToUp = acos(clamp(dot(n, vec3(0, 1, 0)), -1.0f, 1.0f));
						smallestAngle = min(smallestAngle, angleToUp);
					}

					if (smallestAngle >= maxWalkableAngle)
					{
						//block movement if against wall or steep slope
						cancelAxis = true;
					}
					else
					{
						//snap down to stay grounded while not moving
						if (dot(originalVelocity, originalVelocity) < 0.0001f)
						{
							body.velocity.y = 0.0f;
							body.position.y -= groundingProbeDistance;
						}
						break;
					}
				}
			}

			if (cancelAxis) body.velocity[axis] = 0.0f;
		}
	}

	bool StepSimulation::CanTilt(RigidBody& body)
	{
		//calculate the six candidate directions (in world space)
		vec3 possibleUps[6] =
		{
			body.rotation * vec3(0, 1, 0),
			body.rotation * vec3(0, -1, 0),
			body.rotation * vec3(1, 0, 0),
			body.rotation * vec3(-1, 0, 0),
			body.rotation * vec3(0, 0, 1),
			body.rotation * vec3(0, 0, -1)
		};

		vec3 bestUp = possibleUps[0];
		for (int i = 1; i < 6; i++)
		{
			if (abs(dot(possibleUps[i], vec3(0, 1, 0)))
				> abs(dot(bestUp, vec3(0, 1, 0))))
			{
				bestUp = possibleUps[i];
			}
		}

		body.closestUp = bestUp;

		//calculate the angle difference in degrees between the best candidate and world up
		float angleDiff = acos(
			clamp(dot(bestUp, vec3(0, 1, 0)), -1.0f, 1.0f))
			* (180.0f / 3.14159f);

		return angleDiff > 1.0f;
	}

	void StepSimulation::TiltBody(RigidBody& body)
	{
		//get tilt axis
		vec3 tiltAxis = normalize(cross(body.closestUp, vec3(0, 1, 0)));

		//if tiltAxis is too small, set a default rotation axis (prevents divide-by-zero issues)
		if (length(tiltAxis) < 1e-5f)
		{
			tiltAxis = vec3(1, 0, 0);
			LOG_ERROR("Small tilt axis detected, using default (1, 0, 0)");
		}

		float upAlignment = clamp(fabs(dot(body.closestUp, vec3(0, 1, 0))), 0.0f, 1.0f);
		body.angleToFlat = acos(upAlignment) * (180.0f / 3.14159f);
		LOG_DEBUG("Clamped Up Alignment: " << upAlignment
			<< " | Angle to Flat: " << body.angleToFlat << "");

		float correctionScale = clamp(body.angleToFlat / 15.0f, 0.0f, 1.0f);
		vec3 tiltTorque = tiltAxis * correctionScale * 5.0f;

		//moderate correction
		if (body.angleToFlat >= 15.0f)
		{
			body.ApplyTorque(tiltTorque);
			LOG_DEBUG("tilt a");
		}
		//faster correction
		else if (body.angleToFlat >= 2.0f)
		{
			body.ApplyTorque(tiltTorque * 1.25f);
			LOG_DEBUG("tilt b");
		}
		//stop rotation
		else if (body.angleToFlat < 2.0f
			&& length(body.angularVelocity) < 0.1f)
		{
			body.angularVelocity = vec3(0.0f);
			body.rotation = normalize(body.rotation);
			LOG_DEBUG("tilt snap");
		}
	}
}