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
	using KalaHeaders::KalaMath::normalize_q;

	inline const vec3 MIN_OBB_POS = vec3(-10000.0f);
	inline const vec3 MAX_OBB_POS = vec3(10000.0f);

	inline const vec3 MIN_OBB_HALF_EXTENTS = vec3(epsilon);
	inline const vec3 MAX_OBB_HALF_EXTENTS = vec3(10000.0f);

	class LIB_API Collider_OBB : public Collider
	{
		friend class KalaPhysics::Core::PhysicsWorld;
	public:
		//Initializes a broadphase or narrowphase OBB collider
		static Collider_OBB* Initialize(
			u32 parentRigidBody,
			const vec3& pos,
			const quat& rot,
			const vec3& halfExtents,
			ColliderType type);

		inline const vec3& GetPos() const { return pos; }
		inline void SetPos(const vec3& newValue)
		{
			pos = kclamp(newValue, MIN_OBB_POS, MAX_OBB_POS);
		}

		inline const quat& GetRot() const { return rot; }
		inline void SetRot(const quat& newValue)
		{ 
			rot = normalize_q(newValue);
		}

		inline const vec3& GetHalfExtents() const { return halfExtents; }
		inline void SetHalfExtents(const vec3& newValue)
		{
			halfExtents = kclamp(newValue, MIN_OBB_HALF_EXTENTS, MAX_OBB_HALF_EXTENTS);
		}

		~Collider_OBB() override;
	private:
		void Update(Collider* c, f32 deltaTime) override;

		vec3 pos{};
		quat rot{};
		vec3 halfExtents{};
	};
}