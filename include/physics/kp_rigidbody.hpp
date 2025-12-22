//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#pragma once

#include <array>

#include "KalaHeaders/core_utils.hpp"
#include "KalaHeaders/log_utils.hpp"

#include "core/kp_registry.hpp"

namespace KalaPhysics::Core
{
	class PhysicsWorld;
}

namespace KalaPhysics::Physics
{
	using std::array;
	
	using u8 = uint8_t;
	using u32 = uint32_t;
	
	using KalaHeaders::KalaLog::Log;
	using KalaHeaders::KalaLog::LogType;

	using KalaPhysics::Core::KalaPhysicsRegistry;
	
	constexpr u8 MAX_COLLIDERS = 50;
	
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
		
		~RigidBody();
	private:
		bool isInitialized{};

		u32 ID{};

		array<u32, MAX_COLLIDERS> colliders{};
		u8 colliderCount{};
	};
}