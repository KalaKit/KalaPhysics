//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

//external
#include "gtc/quaternion.hpp"

//physics
#include "simulation/contactsolver.hpp"

using glm::mat3_cast;
using glm::max;

namespace KalaKit::Physics::Simulation
{
	Contact& ContactSolver::AddContact(
		PhysicsWorld& world,
		RigidBody* bodyA,
		RigidBody* bodyB,
		const vec3& point,
		const vec3& normal,
		float penetration,
		float deltaTime)
	{
		Contact contact{};
		contact.bodyA = bodyA;
		contact.bodyB = bodyB;
		contact.point = point;
		contact.normal = normal;
		contact.penetration = penetration;

		//gently pushes bodies apart when they're penetrating, without adding bounce

		float penetrationBias = max(0.0f, penetration - world.GetBaumgarteSlop());
		contact.bias = (world.GetBaumgarteFactor() / deltaTime) * penetrationBias;

		//local lever arms

		vec3 worldCoGa = bodyA->position + mat3_cast(bodyA->rotation) * bodyA->centerOfGravity;
		vec3 worldCoGb = bodyB->position + mat3_cast(bodyB->rotation) * bodyB->centerOfGravity;
		contact.rA = point - worldCoGa;
		contact.rB = point - worldCoGb;

		//inverse mass and inertia
		
		float invMassA = (bodyA->mass > 0.0f) ? 1.0f / bodyA->mass : 0.0f;
		float invMassB = (bodyB->mass > 0.0f) ? 1.0f / bodyB->mass : 0.0f;
		vec3 invInertiaA = (bodyA->mass > 0.0f) ? (1.0f / bodyA->inertiaTensor) : vec3(0.0f);
		vec3 invInertiaB = (bodyB->mass > 0.0f) ? (1.0f / bodyB->inertiaTensor) : vec3(0.0f);

		//compute effective mass

		vec3 crossRA = cross(contact.rA, contact.normal);
		vec3 crossRB = cross(contact.rB, contact.normal);
		float angularA = dot(crossRA * invInertiaA, crossRA);
		float angularB = dot(crossRB * invInertiaB, crossRB);
		float totalInvMass = invMassA + invMassB + angularA + angularB;

		contact.effectiveMass = (totalInvMass > 0.0f) ? 1.0f / totalInvMass : 0.0f;

		contacts.push_back(contact);
		return contacts.back();
	}

	void ContactSolver::Solve(float deltaTime, int iterations)
	{
		//start each frame with a good approximation from the previous frame,
		//boosting stack ability and reducing jitter
		for (auto& c : contacts)
		{
			vec3 impulse = c.accumulatedImpulse * c.normal;
			c.bodyA->ApplyImpulse(-impulse);
			c.bodyB->ApplyImpulse(impulse);
		}

		for (int i = 0; i < iterations; ++i)
		{
			for (auto& c : contacts)
			{
				vec3 vA = c.bodyA->velocity + cross(c.bodyA->angularVelocity, c.rA);
				vec3 vB = c.bodyB->velocity + cross(c.bodyB->angularVelocity, c.rB);
				vec3 relVel = vB - vA;

				float normalVel = dot(relVel, c.normal);
				float lambda = -(normalVel + c.bias) * c.effectiveMass;

				//accumulate impulse (clamp to avoid pulling)

				float oldImpulse = c.accumulatedImpulse;
				c.accumulatedImpulse = max(oldImpulse + lambda, 0.0f);
				float actualImpulse = c.accumulatedImpulse - oldImpulse;

				vec3 impulse = actualImpulse * c.normal;

				c.bodyA->ApplyImpulse(-impulse);
				c.bodyB->ApplyImpulse(impulse);
			}
		}
	}

	void ContactSolver::Clear()
	{
		contacts.clear();
	}
}