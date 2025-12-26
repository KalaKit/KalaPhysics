//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#pragma once

#include "KalaHeaders/core_utils.hpp"

#include "physics/collision/kp_collider.hpp"

namespace KalaPhysics::Core
{
	class PhysicsWorld;
}

namespace KalaPhysics::Physics::Collision
{
	using KalaHeaders::KalaMath::kclamp;

	inline const vec3 MIN_AABB_CORNER = vec3(-10000.0f);
	inline const vec3 MAX_AABB_CORNER = vec3(10000.0f);
	inline const vec3 MIN_AABB_CORNER_DISTANCE = vec3(epsilon);

	class LIB_API Collider_AABB : public Collider
	{
		friend class KalaPhysics::Core::PhysicsWorld;
	public:
		//Initializes a broadphase-only AABB collider
		static Collider_AABB* Initialize(
			u32 parentRigidbody,
			const vec3& minCorner,
			const vec3& maxCorner);

		inline const vec3& GetMinCorner() const { return minCorner; }
		inline void SetMinCorner(const vec3& newValue)
		{
			minCorner = kclamp(newValue, MIN_AABB_CORNER, MAX_AABB_CORNER);
			maxCorner = kclamp(maxCorner, minCorner + MIN_AABB_CORNER_DISTANCE, MAX_AABB_CORNER);
		}

		inline const vec3& GetMaxCorner() const { return maxCorner; }
		inline void SetMaxCorner(const vec3& newValue)
		{
			maxCorner = kclamp(newValue, MIN_AABB_CORNER, MAX_AABB_CORNER);
			minCorner = kclamp(minCorner, MIN_AABB_CORNER, maxCorner - MIN_AABB_CORNER_DISTANCE);
		}

		~Collider_AABB() override;
	private:
		void Update(Collider* c, f32 deltaTime) override;

		vec3 minCorner{};
		vec3 maxCorner{};
	};
}