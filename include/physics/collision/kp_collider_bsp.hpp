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

	inline const vec3 MIN_CENTER = vec3(-10000.0f);
	inline const vec3 MAX_CENTER = vec3(10000.0f);

	constexpr f32 MIN_RADIUS = epsilon;
	constexpr f32 MAX_RADIUS = 10000.0f;

	class LIB_API Collider_BSP : public Collider
	{
		friend class KalaPhysics::Core::PhysicsWorld;
	public:
		//Initializes a broadphase-only BSP collider
		static Collider_BSP* Initialize(
			u32 parentRigidbody,
			const vec3& center,
			f32 radius);

		inline const vec3& GetCenter() const { return center; }
		inline void SetCenter(const vec3& newValue)
		{
			center = kclamp(newValue, MIN_CENTER, MAX_CENTER);
		}

		inline f32 GetRadius() const { return radius; }
		inline void SetRadius(f32 newValue)
		{
			radius = clamp(newValue, MIN_RADIUS, MAX_RADIUS);
		}

		~Collider_BSP() override;
	private:
		void Update(f32 deltaTime);

		vec3 center{};
		f32 radius{};
	};
}