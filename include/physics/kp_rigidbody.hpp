//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#pragma once

#include <array>
#include <algorithm>

#include "KalaHeaders/core_utils.hpp"
#include "KalaHeaders/log_utils.hpp"
#include "KalaHeaders/math_utils.hpp"

#include "core/kp_registry.hpp"

namespace KalaPhysics::Core
{
	class PhysicsWorld;
}

namespace KalaPhysics::Physics
{
	using std::array;
	using std::clamp;
	
	using KalaHeaders::KalaLog::Log;
	using KalaHeaders::KalaLog::LogType;
	using KalaHeaders::KalaMath::vec3;
	using KalaHeaders::KalaMath::mat3;

	using KalaPhysics::Core::KalaPhysicsRegistry;
	
	constexpr u8 MAX_COLLIDERS = 50;

	constexpr f32 MAX_MASS = 10000.0f;
	inline const vec3 MAX_GRAVITY_SCALE = 10000.0f;
	inline const vec3 MAX_VELOCITY = 10000.0f;
	inline const vec3 MAX_ANGULAR_VELOCITY = 10000.0f;

	struct LIB_API RigidBodyVars
	{
		bool isSleeping{}; //is this body currently sleeping
		bool ccd{};	       //is continuous collision enabled

		f32 mass{};
		f32 restitution{};
		f32 linearDamp{};
		f32 angularDamp{};

		vec3 gravityScale = 1.0f; //per-rb multiplier
		vec3 velocity{};
		vec3 angularVelocity{};
		mat3 inertiaTensor{};

		f32 accumForce{};
		f32 accumTorque{};
	};
	
	class LIB_API RigidBody
	{
		friend class KalaPhysics::Core::PhysicsWorld;
	public:
		static inline KalaPhysicsRegistry<RigidBody> registry{};
		
		static RigidBody* Initialize();

		inline bool IsInitialized() const { return isInitialized; }

		inline u32 GetID() const { return ID; }
		
		//Add a new collider by its ID to this rigidbody
		void AddCollider(u32 colliderID);
		//Remove an existing collider by ID from this rigidbody
		void RemoveCollider(u32 colliderID);
		//Reset colliders count, doesn't waste time removing actual values
		void RemoveAllColliders();

		inline const array<u32, MAX_COLLIDERS>& GetAllColliders() const { return colliders; }
		inline u8 GetColliderCount() const { return colliderCount; }

		inline bool IsSleeping() const { return vars.isSleeping; }
		inline bool IsCCD() const { return vars.ccd; }

		inline f32 GetMass() const { return vars.mass; }
		inline void SetMass(f32 newValue)
		{
			vars.mass = clamp(
				newValue,
				0.0f,
				MAX_MASS);
		}

		inline f32 GetRestitution() const { return vars.restitution; }
		inline void SetRestitution(f32 newValue)
		{
			vars.restitution = clamp(
				newValue,
				0.0f,
				1.0f);
		}

		inline f32 GetLinearDamp() const { return vars.linearDamp; }
		inline void SetLinearDamp(f32 newValue)
		{
			vars.linearDamp = clamp(
				newValue,
				0.0f,
				1.0f);
		}

		inline f32 GetAngularDamp() const { return vars.angularDamp; }
		inline void SetAngularDamp(f32 newValue)
		{ 
			vars.angularDamp = clamp(
				newValue,
				0.0f,
				1.0f);
		}

		inline const vec3& GetGravityScale() const { return vars.gravityScale; }
		inline void SetGravityScale(const vec3& newValue)
		{ 
			vars.gravityScale = clamp(
				newValue,
				vec3(0.0f),
				MAX_GRAVITY_SCALE);
		}

		inline const vec3& GetVelocity() const { return vars.velocity; }
		inline void SetVelocity(const vec3& newValue)
		{
			vars.velocity = clamp(
				newValue,
				vec3(0.0f),
				MAX_VELOCITY);
		}

		inline const vec3& GetAngularVelocity() const { return vars.angularVelocity; }
		inline void SetAngularVelocity(const vec3& newValue)
		{
			vars.angularVelocity = clamp(
				newValue,
				vec3(0.0f),
				MAX_ANGULAR_VELOCITY);
		}

		inline const mat3& GetInertiaTensor() const { return vars.inertiaTensor; }

		inline f32 GetAccumulatedForce() const { return vars.accumForce; }

		inline f32 GetAccumulatedTorque() const { return vars.accumTorque; }
		
		~RigidBody();
	private:
		bool isInitialized{};

		u32 ID{};

		array<u32, MAX_COLLIDERS> colliders{};
		u8 colliderCount{};

		RigidBodyVars vars{};
	};
}