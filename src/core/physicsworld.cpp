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

#include <string>

//external
#include "gtc/quaternion.hpp"

//physics
#include "core/physicsworld.hpp"

using KalaKit::Physics::Shape::ColliderType;
using KalaKit::Physics::Collision::ContactManifold;

using glm::normalize;
using glm::length;
using std::min;
using std::max;
using glm::clamp;
using std::string;
using std::to_string;
using std::swap;
using glm::acos;
using glm::quat;
using glm::cross;
using std::fabs;
using glm::radians;

namespace KalaKit::Physics::Core
{
	PhysicsWorld& PhysicsWorld::GetInstance()
	{
		static PhysicsWorld instance;
		return instance;
	}

	PhysicsWorld::PhysicsWorld() :
		gravity(),
		isInitialized(false) {}
	PhysicsWorld::~PhysicsWorld()
	{
		if (!isInitialized)
		{
			LOG_ERROR("Cannot shut down Elypso Physics because it has not yet been initialized!");
			return;
		}

		for (auto* rb : bodies)
		{
			if (rb)
			{
				GameObjectHandle handle = rb->handle;
				RemoveRigidBody(handle);
			}
		}

		bodies.clear();        //clear the bodies vector
		bodyMap.clear();       //clear the body map
		generations.clear();   //clear the generations map
		isInitialized = false;

		LOG_SUCCESS("Shutdown completed!");
	}

	void PhysicsWorld::InitializePhysics(const vec3& newGravity)
	{
		if (isInitialized)
		{
			LOG_ERROR("Elypso Physics is already initialized!");
			return;
		}

		bodies.clear();
		bodyMap.clear();
		generations.clear();

		gravity = newGravity;

		isInitialized = true;

		LOG_SUCCESS("Initialization completed!");
	}

	GameObjectHandle PhysicsWorld::CreateRigidBody(
		const vec3& position,
		const quat& rotation,
		ColliderType colliderType,
		const vec3& colliderSizeOrRadius,
		float mass,
		float restitution,
		float staticFriction,
		float dynamicFriction,
		float gravityFactor,
		bool useGravity)
	{
		if (!isInitialized)
		{
			LOG_ERROR("Cannot create a RigidBody if Elypso Physics isnt initialized!");
			return GameObjectHandle(UINT32_MAX, UINT32_MAX);
		}

		uint32_t index = static_cast<uint32_t>(bodies.size());
		uint32_t generation = generations.size() > index ? generations[index] : 0;

		GameObjectHandle handle(index, generation);

		//create the rigidbody
		RigidBody* rb = new RigidBody(
			handle,
			position,
			rotation,
			mass,
			restitution,
			staticFriction,
			dynamicFriction,
			gravityFactor);

		//assign collider based on collider type
		rb->SetCollider(colliderType);

		bodies.push_back(rb);
		bodyMap[handle] = index;

		if (generations.size() <= index) generations.push_back(0);

		LOG_SUCCESS("Created rigidbody(" + to_string(index) + ", " + to_string(generation) + ")!");

		return handle;
	}

	RigidBody* PhysicsWorld::GetRigidBody(const GameObjectHandle& handle)
	{
		auto it = bodyMap.find(handle);
		if (it != bodyMap.end())
		{
			return bodies[it->second];
		}
		return nullptr;
	}

	void PhysicsWorld::RemoveRigidBody(const GameObjectHandle& handle)
	{
		auto it = bodyMap.find(handle);
		if (it != bodyMap.end())
		{
			size_t index = it->second;

			if (bodies[index])
			{
				//delete collider first if it exists
				if (bodies[index]->collider)
				{
					delete bodies[index]->collider;
					bodies[index]->collider = nullptr;
				}

				//delete the rigid body
				delete bodies[index];
				bodies[index] = nullptr;
			}

			generations[handle.index]++;
			bodyMap.erase(it);
			if (index < bodies.size() - 1)
			{
				swap(bodies[index], bodies.back());
				bodyMap[bodies[index]->handle] = index;
			}

			uint32_t idx = handle.index;
			uint32_t gen = handle.generation;
			LOG_SUCCESS("Removed rigidbody(" + to_string(idx) + ", " + to_string(gen) + ")!");

			bodies.pop_back();
		}
	}

	bool PhysicsWorld::IsValidCollision(RigidBody& bodyA, RigidBody& bodyB)
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

	void PhysicsWorld::StepSimulation(float deltaTime)
	{
		if (bodyMap.size() == 0) return;

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
					ResolveCollision(bodyA, bodyB, contact.normal, contact.point, contact.penetration);
				}
			}
		}

		ApplyPhysicsIntegration(deltaTime);
	}

	void PhysicsWorld::ApplyPhysicsIntegration(float deltaTime)
	{
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
				vec3 gravityImpulse = (gravity * body.gravityFactor) * deltaTime;
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

			PredictCollision(bodyPtr, body, deltaTime);

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

	void PhysicsWorld::PredictCollision(
		RigidBody* bodyPtr, 
		RigidBody& body,
		float deltaTime)
	{
		vec3 originalVelocity = body.velocity;
		const float maxWalkableAngle = radians(angleLimit);

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

	void PhysicsWorld::ResolveCollision(
		RigidBody& bodyA, 
		RigidBody& bodyB, 
		const vec3& collisionNormal,
		const vec3& contactPoint,
		float penetration)
	{
		//correctly handle ramp and slope detection before other collision types

		float upDot = dot(collisionNormal, vec3(0, 1, 0));
		float walkLimitCos = cos(radians(angleLimit));
		if (upDot > walkLimitCos)
		{
			ApplyFriction(bodyA, bodyB, collisionNormal, contactPoint);

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
		float corrected = max(0.0f, penetration - minPenetrationThreshold);
		if (corrected > 0.0f)
		{
			float totalMass = bodyA.mass + bodyB.mass;
			if (totalMass > 0.00001f)
			{
				float ratioA = (bodyB.mass / totalMass);
				float ratioB = (bodyA.mass / totalMass);

				vec3 correctionVec = collisionNormal * (corrected * correctionFactor);

				bodyA.position -= correctionVec * ratioA;
				bodyB.position += correctionVec * ratioB;
			}
		}
	}

	void PhysicsWorld::ApplyFriction(
		RigidBody& bodyA, 
		RigidBody& bodyB, 
		const vec3& collisionNormal,
		const vec3& contactPoint) const
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
		float staticFriction = (bodyA.staticFriction + bodyB.staticFriction) * frictionMultiplier;
		float dynamicFriction = (bodyA.dynamicFriction + bodyB.dynamicFriction) * frictionMultiplier;

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
			bodyA.angularVelocity *= lowAngularVelocityFactor;
		}

		if (length(bodyB.velocity) < 0.01f)
		{
			bodyB.angularVelocity *= lowAngularVelocityFactor;
		}
	}

	bool PhysicsWorld::CanTilt(RigidBody& body)
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

	void PhysicsWorld::TiltBody(RigidBody& body)
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