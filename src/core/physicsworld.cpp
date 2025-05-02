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
			LOG_ERROR("Cannot shut down KalaPhysics because it has not yet been initialized!");
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
			LOG_ERROR("KalaPhysics is already initialized!");
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
		const vec3& scale,
		bool isDynamic,
		bool useGravity,
		ColliderType colliderType,
		float mass,
		float restitution,
		float staticFriction,
		float dynamicFriction,
		float gravityFactor)
	{
		if (!isInitialized)
		{
			LOG_ERROR("Cannot create a RigidBody if KalaPhysics isnt initialized!");
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
			scale,
			isDynamic,
			useGravity,
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
}