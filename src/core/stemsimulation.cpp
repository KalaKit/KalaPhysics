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
#include "core/stepsimulation.hpp"

using glm::radians;
using glm::clamp;
using glm::min;
using glm::max;

namespace KalaKit::Physics::Core
{
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
					ResolveCollision(world, bodyA, bodyB, contact.normal, contact.point, contact.penetration);
				}
			}
		}

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

	void StepSimulation::ResolveCollision(
		PhysicsWorld& world,
		RigidBody& bodyA,
		RigidBody& bodyB,
		const vec3& collisionNormal,
		const vec3& contactPoint,
		float penetration)
	{
		//correctly handle ramp and slope detection before other collision types

		float upDot = dot(collisionNormal, vec3(0, 1, 0));
		float walkLimitCos = cos(radians(world.GetAngleLimit()));
		if (upDot > walkLimitCos)
		{
			ApplyFriction(world, bodyA, bodyB, collisionNormal, contactPoint);

			vec3& v = bodyA.velocity;
			v = v - dot(v, collisionNormal) * collisionNormal;

			return;
		}

		//world space center of gravity
		vec3 worldCoGA =
			bodyA.position
			+ mat3_cast(bodyA.rotation)
			* bodyA.centerOfGravity;
		vec3 worldCoGB =
			bodyB.position
			+ mat3_cast(bodyB.rotation)
			* bodyB.centerOfGravity;

		//constant offsets based on world cog
		vec3 rA = contactPoint - worldCoGA;
		vec3 rB = contactPoint - worldCoGB;

		//compute relative velocity
		vec3 relativeVelocity = bodyB.velocity - bodyA.velocity;

		//compute velocity along the collision normal
		float velocityAlongNormal = dot(relativeVelocity, collisionNormal);

		//if objects are separating, do nothing
		if (velocityAlongNormal > 0.0f) return;

		//combute combied restitution (take the minimum)
		float restitution = clamp(min(bodyA.restitution, bodyB.restitution), 0.0f, 0.5f);

		//compute inverse mass and inverse inertia tensors
		float invMassA = (bodyA.mass > 0.0f) ? (1.0f / bodyA.mass) : 0.0f;
		float invMassB = (bodyB.mass > 0.0f) ? (1.0f / bodyB.mass) : 0.0f;

		vec3 invInertiaA = (bodyA.mass > 0.0f) ? (1.0f / bodyA.inertiaTensor) : vec3(0.0f);
		vec3 invInertiaB = (bodyB.mass > 0.0f) ? (1.0f / bodyB.inertiaTensor) : vec3(0.0f);

		//compute impulse scalar
		vec3 crossRA_N = cross(rA, collisionNormal);
		vec3 crossRB_N = cross(rB, collisionNormal);

		float denominator = invMassA + invMassB
			+ dot(crossRA_N * invInertiaA, crossRA_N)
			+ dot(crossRB_N * invInertiaB, crossRB_N);

		if (denominator < 0.0001f) return;

		float impulseScalar = -(1.0f + restitution) * velocityAlongNormal / denominator;

		//clamp the impulse to prevent excessive velocity spikes
		const float maxImpulse = 50.0f;
		impulseScalar = clamp(impulseScalar, -maxImpulse, maxImpulse);

		vec3 impulse = impulseScalar * collisionNormal;

		bodyA.ApplyImpulse(-impulse);
		bodyB.ApplyImpulse(impulse);

		//position correction for penetration
		float corrected = max(0.0f, penetration - world.GetMinPenetrationThreshold());
		if (corrected > 0.0f)
		{
			float totalMass = bodyA.mass + bodyB.mass;
			if (totalMass > 0.00001f)
			{
				float ratioA = (bodyB.mass / totalMass);
				float ratioB = (bodyA.mass / totalMass);

				vec3 correctionVec = collisionNormal * (corrected * world.GetCorrectionFactor());

				bodyA.position -= correctionVec * ratioA;
				bodyB.position += correctionVec * ratioB;
			}
		}
	}

	void StepSimulation::ApplyFriction(
		PhysicsWorld& world,
		RigidBody& bodyA,
		RigidBody& bodyB,
		const vec3& collisionNormal,
		const vec3& contactPoint)
	{
		//compute relative velocity at the contact point
		vec3 rA = contactPoint - bodyA.position;
		vec3 rB = contactPoint - bodyB.position;

		vec3 vA = bodyA.velocity + cross(bodyA.angularVelocity, rA);
		vec3 vB = bodyB.velocity + cross(bodyB.angularVelocity, rB);
		vec3 relativeVelocity = vB - vA;

		//apply friction only if there's significant tangential velocity
		if (length(relativeVelocity) < 0.01f) return;

		//tangential direction
		vec3 tangent = relativeVelocity - dot(relativeVelocity, collisionNormal) * collisionNormal;
		if (length(tangent) < 1e-6f) return;
		tangent = normalize(tangent);

		//compute static and dynamic friction coefficients (use average)
		float staticFriction = (bodyA.staticFriction + bodyB.staticFriction) * world.GetFrictionMultiplier();
		float dynamicFriction = (bodyA.dynamicFriction + bodyB.dynamicFriction) * world.GetFrictionMultiplier();

		//compute friction impulse magnitude
		float frictionImpulseScalar = -dot(relativeVelocity, tangent);
		frictionImpulseScalar /= (1.0f / bodyA.mass) + (1.0f / bodyB.mass);

		vec3 frictionImpulse = frictionImpulseScalar * tangent;

		//static friction check
		float maxStaticFriction = staticFriction * length(frictionImpulse);
		if (abs(frictionImpulseScalar) > maxStaticFriction)
		{
			frictionImpulse = dynamicFriction * frictionImpulse;
		}

		//clamp friction force to avoid cancelling small impulses
		float maxFrictionForce = max(bodyA.mass, bodyB.mass);
		if (length(frictionImpulse) > maxFrictionForce)
		{
			frictionImpulse = normalize(frictionImpulse) * maxFrictionForce;
		}

		bodyA.ApplyImpulse(-frictionImpulse);
		bodyB.ApplyImpulse(frictionImpulse);

		vec3 torqueA = cross(rA, -frictionImpulse);
		vec3 torqueB = cross(rB, frictionImpulse);
		bodyA.ApplyTorque(torqueA);
		bodyB.ApplyTorque(torqueB);

		if (length(bodyA.velocity) < 0.01f)
		{
			bodyA.angularVelocity *= world.GetLowAngularVelocityFactor();
		}

		if (length(bodyB.velocity) < 0.01f)
		{
			bodyB.angularVelocity *= world.GetLowAngularVelocityFactor();
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