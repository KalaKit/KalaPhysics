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

	//Represents the detail level of the sphere,
	//the higher the number, the more complex the sphere and its calculations will be.
	//Allowed values: 8, 12, 16, 24, 32, 48, 64
	constexpr u8 SPHERE_QUALITY = 24;

	inline const vec3 MIN_BSP_CENTER = vec3(-10000.0f);
	inline const vec3 MAX_BSP_CENTER = vec3(10000.0f);

	constexpr f32 MIN_BSP_RADIUS = epsilon;
	constexpr f32 MAX_BSP_RADIUS = 10000.0f;

	class LIB_API Collider_BSP : public Collider
	{
		friend class KalaPhysics::Core::PhysicsWorld;
	public:
		//Initializes a broadphase-only BSP collider
		static Collider_BSP* Initialize(
			u32 parentRigidBody,
			const vec3& center,
			f32 radius);

		inline const vec3& GetCenter() const { return center; }
		inline void SetCenter(const vec3& newValue)
		{
			center = kclamp(newValue, MIN_BSP_CENTER, MAX_BSP_CENTER);
		}

		inline f32 GetRadius() const { return radius; }
		inline void SetRadius(f32 newValue)
		{
			radius = clamp(newValue, MIN_BSP_RADIUS, MAX_BSP_RADIUS);
		}

		~Collider_BSP() override;
	private:
		void Update(Collider* c, f32 deltaTime) override;

		vec3 center{};
		f32 radius{};
	};
}