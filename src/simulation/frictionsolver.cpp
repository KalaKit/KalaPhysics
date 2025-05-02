//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

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

		vec3 tangent = relVel - dot(relVel, normal) * normal;
		if (dot(tangent, tangent) < 1e-6f) return; //no tangential movement
		tangent = normalize(tangent);

		float invMassA = (bodyA->mass > 0.0f) ? 1.0f / bodyA->mass : 0.0f;
		float invMassB = (bodyB->mass > 0.0f) ? 1.0f / bodyB->mass : 0.0f;
		vec3 invInertiaA = (bodyA->mass > 0.0f) 
			? (1.0f / bodyA->inertiaTensor) : vec3(0.0f);
		vec3 invInertiaB = (bodyB->mass > 0.0f) 
			? (1.0f / bodyB->inertiaTensor) : vec3(0.0f);

		vec3 crossRA = cross(rA, tangent);
		vec3 crossRB = cross(rB, tangent);
		float angularA = dot(crossRA * invInertiaA, crossRA);
		float angularB = dot(crossRB * invInertiaB, crossRB);
		float effMass = invMassA + invMassB + angularA + angularB;

		if (effMass < 1e-5f) return;

		FrictionConstraint fc{};
		fc.bodyA = bodyA;
		fc.bodyB = bodyB;
		fc.point = point;
		fc.normal = normal;
		fc.tangent = tangent;
		fc.rA = rA;
		fc.rB = rB;
		fc.effectiveMass = 1.0f / effMass;
		fc.tangentImpulse = dynamicFriction;
		fc.linkedContact = linkedContact;

		constraints.push_back(fc);
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