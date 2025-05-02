//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

//physics
#include "simulation/handlemotion.hpp"

namespace KalaKit::Physics::Simulation
{
	void HandleMotion::Run(
		PhysicsWorld& world,
		float deltaTime)
	{
		for (auto& bodyPtr : world.GetBodies())
		{
			RigidBody& body = *bodyPtr;

			if (!body.isDynamic
				|| !body.collider
				|| !body.useGravity
				|| body.isSleeping)
			{
				continue;
			}

			ApplyForces(world, body, deltaTime);
			ProjectAgainstSurfaces(world, bodyPtr, body, deltaTime);
			Integrate(world, body, deltaTime);
			ApplyDamping(body, deltaTime);
			UpdateSleepState(body, deltaTime);

			CanTilt(body);
			TiltBody(body);
		}
	}

	void HandleMotion::ApplyForces(
		PhysicsWorld& world,
		RigidBody& body,
		float deltaTime)
	{

	}

	void HandleMotion::ProjectAgainstSurfaces(
		PhysicsWorld& world,
		RigidBody* bodyPtr,
		RigidBody& body,
		float deltaTime)
	{

	}

	void HandleMotion::Integrate(
		PhysicsWorld& world,
		RigidBody& body,
		float deltaTime)
	{

	}

	void HandleMotion::ApplyDamping(
		RigidBody& body,
		float deltaTime)
	{

	}

	void HandleMotion::UpdateSleepState(
		RigidBody& body,
		float deltaTime)
	{

	}

	bool HandleMotion::CanTilt(RigidBody& body)
	{
		return false;
	}
	void HandleMotion::TiltBody(RigidBody& body)
	{

	}
}