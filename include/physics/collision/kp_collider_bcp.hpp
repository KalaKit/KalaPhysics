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

	inline const vec3 MIN_BCP_POS = vec3(-10000.0f);
	inline const vec3 MAX_BCP_POS = vec3(10000.0f);

	constexpr f32 MIN_BCP_HEIGHT = epsilon;
	constexpr f32 MAX_BCP_HEIGHT = 10000.0f;

	constexpr f32 MIN_BCP_RADIUS = epsilon;
	constexpr f32 MAX_BCP_RADIUS = 10000.0f;

	class LIB_API Collider_BCP : public Collider
	{
		friend class KalaPhysics::Core::PhysicsWorld;
	public:
		//Initializes a broadphase or narrowphase BCP collider
		static Collider_BCP* Initialize(
			u32 parentRigidBody,
			const vec3& pos,
			f32 height,
			f32 radius,
			ColliderType type);

		inline const vec3& GetPos() const { return pos; }
		inline void SetPos(const vec3& newValue)
		{
			pos = kclamp(newValue, MIN_BCP_POS, MAX_BCP_POS);
		}

		inline f32 GetHeight() const { return height; }
		inline void SetHeight(f32 newValue)
		{
			height = clamp(newValue, MIN_BCP_HEIGHT, MAX_BCP_HEIGHT);
			radius = min(radius, height * 0.5f);
		}

		inline f32 GetRadius() const { return radius; }
		inline void SetRadius(f32 newValue)
		{
			radius = clamp(newValue, MIN_BCP_RADIUS, MAX_BCP_RADIUS);
			height = max(height, 2 * radius);
		}

		~Collider_BCP() override;
	private:
		void Update(Collider* c, f32 deltaTime) override;

		vec3 pos{};
		f32 height{};
		f32 radius{};
	};
}