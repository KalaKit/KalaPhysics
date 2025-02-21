//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#pragma once

#ifdef _WIN32
	#ifdef PHYSICS_DLL_EXPORT
		#define PHYSICS_API __declspec(dllexport)
	#else
		#define PHYSICS_API __declspec(dllimport)
	#endif
#else
	#define PHYSICS_API
#endif

#include <string>
#include <iostream>

//external
#include "glm.hpp"

//physics
#include "gameobjecthandle.hpp"

namespace ElypsoPhysics
{
	using glm::vec3;
	using std::string;
	using std::to_string;
	using std::cout;

	enum class ColliderType
	{
		BOX,
		SPHERE
	};

	class PHYSICS_API Collider
	{
	public:
		ColliderType type;
		GameObjectHandle handle;
		float boundingRadius = 0.0f;

		explicit Collider(ColliderType type, const GameObjectHandle& h);
		
		virtual ~Collider() = default;

		virtual void CalculateBoundingRadius() = 0;
		virtual void UpdateScale(const vec3& scale) = 0;

		Collider(const Collider&) = delete;
		Collider& operator=(const Collider&) = delete;
	};

	class PHYSICS_API BoxCollider : public Collider
	{
	public:
		BoxCollider(const GameObjectHandle& h, const vec3& size);
		void UpdateScale(const vec3& scale) override
		{
			halfExtents = scale * 0.5f;
			CalculateBoundingRadius();

#ifdef NDEBUG
#else
			uint32_t index = handle.index;
			uint32_t gen = handle.generation;
			string sizeString = to_string(scale.x) + ", " + to_string(scale.y) + ", " + to_string(scale.z);
			string message = "[ELYPSO-PHYSICS | SUCCESS] Updated box scale to '" + sizeString + "' for rigidbody (" + to_string(index) + ", " + to_string(gen) + ")!\n";
			cout << message;
#endif
		}
		void CalculateBoundingRadius() override
		{
			boundingRadius = length(halfExtents) * 2.0f * 0.5f; //half diagonal
		}
		//half size of box in each axis
		vec3 halfExtents;
	};

	class PHYSICS_API SphereCollider : public Collider
	{
	public:
		SphereCollider(const GameObjectHandle& h, float r);
		void UpdateScale(const vec3& scale) override
		{
			radius = scale.x;
			CalculateBoundingRadius();

#ifdef NDEBUG
#else
			uint32_t index = handle.index;
			uint32_t gen = handle.generation;
			string radius = to_string(scale.x);
			string message = "[ELYPSO-PHYSICS | SUCCESS] Updated sphere radius to '" + radius + "' for rigidbody (" + to_string(index) + ", " + to_string(gen) + ")!\n";
			cout << message;
#endif
		}
		void CalculateBoundingRadius() override
		{
			boundingRadius = radius;
		}

		float radius;
	};
}