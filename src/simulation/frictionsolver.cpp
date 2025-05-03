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
#include "gtc/quaternion.hpp"

//physics
#include "simulation/frictionsolver.hpp"
#include "simulation/contactsolver.hpp"

using glm::mat3_cast;
using glm::clamp;

namespace KalaKit::Physics::Simulation
{
	void FrictionSolver::AddFrictionPair(
		RigidBody* bodyA,
		RigidBody* bodyB,
		const vec3& point,
		const vec3& normal,
		float staticFriction,
		float dynamicFriction,
		Contact* linkedContact)
	{
		vec3 rA = point
			- (bodyA->position
			+ mat3_cast(bodyA->rotation)
			* bodyA->centerOfGravity);
		vec3 rB = point
			- (bodyB->position
			+ mat3_cast(bodyB->rotation)
			* bodyB->centerOfGravity);

		vec3 vA = bodyA->velocity + cross(bodyA->angularVelocity, rA);
		vec3 vB = bodyB->velocity + cross(bodyB->angularVelocity, rB);
		vec3 relVel = vB - vA;

		//try to get tangent1 from relative velocity

		vec3 tangent1 = relVel - dot(relVel, normal) * normal;
		if (dot(tangent1, tangent1) < 1e-6f)
		{
			//nearly aligned with normal,
			//so fallback to any perpendicular direction

			tangent1 = abs(normal.x) < 0.9f
				? cross(normal, vec3(1, 0, 0))
				: cross(normal, vec3(0, 1, 0));
		}
		tangent1 = normalize(tangent1);

		//always compute tangent2 as orthogonal to normal and tangent1

		vec3 tangent2 = normalize(cross(normal, tangent1));

		float invMassA = (bodyA->mass > 0.0f) ? 1.0f / bodyA->mass : 0.0f;
		float invMassB = (bodyB->mass > 0.0f) ? 1.0f / bodyB->mass : 0.0f;
		vec3 invInertiaA = (bodyA->mass > 0.0f) 
			? (1.0f / bodyA->inertiaTensor) : vec3(0.0f);
		vec3 invInertiaB = (bodyB->mass > 0.0f) 
			? (1.0f / bodyB->inertiaTensor) : vec3(0.0f);

		vec3 crossRA1 = cross(rA, tangent1);
		vec3 crossRB1 = cross(rB, tangent1);
		float angularA1 = dot(crossRA1 * invInertiaA, crossRA1);
		float angularB1 = dot(crossRB1 * invInertiaB, crossRB1);
		float effMass1 = invMassA + invMassB + angularA1 + angularB1;
		if (effMass1 >= 1e-5f)
		{
			FrictionConstraint fc1{};
			fc1.bodyA = bodyA;
			fc1.bodyB = bodyB;
			fc1.point = point;
			fc1.normal = normal;
			fc1.tangent = tangent1;
			fc1.rA = rA;
			fc1.rB = rB;
			fc1.effectiveMass = 1.0f / effMass1;
			fc1.tangentImpulse = dynamicFriction;
			fc1.linkedContact = linkedContact;
			constraints.push_back(fc1);
		}

		vec3 crossRA2 = cross(rA, tangent2);
		vec3 crossRB2 = cross(rB, tangent2);
		float angularA2 = dot(crossRA2 * invInertiaA, crossRA2);
		float angularB2 = dot(crossRB2 * invInertiaB, crossRB2);
		float effMass2 = invMassA + invMassB + angularA2 + angularB2;
		if (effMass2 >= 1e-5f)
		{
			FrictionConstraint fc2{};
			fc2.bodyA = bodyA;
			fc2.bodyB = bodyB;
			fc2.point = point;
			fc2.normal = normal;
			fc2.tangent = tangent2;
			fc2.rA = rA;
			fc2.rB = rB;
			fc2.effectiveMass = 1.0f / effMass2;
			fc2.tangentImpulse = dynamicFriction;
			fc2.linkedContact = linkedContact;
			constraints.push_back(fc2);
		}
	}

	void FrictionSolver::Solve(float deltaTime, int iterations)
	{
		for (int i = 0; i < iterations; ++i)
		{
			for (auto& fc : constraints)
			{
				vec3 vA = fc.bodyA->velocity + cross(fc.bodyA->angularVelocity, fc.rA);
				vec3 vB = fc.bodyB->velocity + cross(fc.bodyB->angularVelocity, fc.rB);
				vec3 relVel = vB - vA;

				float tangentVel = dot(relVel, fc.tangent);
				float lambda = -tangentVel * fc.effectiveMass;

				//accumulate impulse (clamp to avoid pulling)

				float oldImpulse = fc.accumulatedImpulse;

				float maxFrictionImpulse{};
				if (fc.linkedContact)
				{
					maxFrictionImpulse = fc.linkedContact->accumulatedImpulse * fc.tangentImpulse;
				}

				fc.accumulatedImpulse = clamp(
					oldImpulse + lambda, 
					-maxFrictionImpulse,
					maxFrictionImpulse);
				float actualImpulse = fc.accumulatedImpulse - oldImpulse;

				vec3 impulse = actualImpulse * fc.tangent;

				fc.bodyA->ApplyImpulse(-impulse);
				fc.bodyB->ApplyImpulse(impulse);
			}
		}
	}

	void FrictionSolver::Clear()
	{
		constraints.clear();
	}
}