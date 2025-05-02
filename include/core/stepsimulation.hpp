//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#pragma once

#ifdef _WIN32
	#ifdef KALAPHYSICS_DLL_EXPORT
		#define KALAPHYSICS_API __declspec(dllexport)
	#else
		#define KALAPHYSICS_API __declspec(dllimport)
	#endif
#else
	#define KALAPHYSICS_API
#endif

//physics
#include "core/rigidbody.hpp"
#include "core/physicsworld.hpp"
#include "core/rigidbody.hpp"

namespace KalaKit::Physics::Core
{
	class KALAPHYSICS_API StepSimulation
	{
	public:
		/// <summary>
		/// Update physics simulation
		/// </summary>
		static void Step(PhysicsWorld& world, float deltaTime);
	private:
		static bool IsValidCollision(RigidBody& bodyA, RigidBody& bodyB);

		static void ApplyPhysicsIntegration(PhysicsWorld& world, float deltaTime);

		static void PredictCollision(
			PhysicsWorld& world,
			RigidBody* bodyPtr,
			RigidBody& body,
			float deltaTime);

		/// <summary>
		/// Resolves a collision by applying impulse forces to separate the bodies and simulate realistic response
		/// </summary>
		static void ResolveCollision(
			PhysicsWorld& world,
			RigidBody& bodyA,
			RigidBody& bodyB,
			const vec3& collisionNormal,
			const vec3& contactPoint,
			float penetration);

		/// <summary>
		/// Applies frictional forces to reduce sliding and simulate surface resistance after a collision
		/// </summary>
		static void ApplyFriction(
			PhysicsWorld& world,
			RigidBody& bodyA,
			RigidBody& bodyB,
			const vec3& collisionNormal,
			const vec3& contactPoint);

		static bool CanTilt(RigidBody& body);
		static void TiltBody(RigidBody& body);
	};
}