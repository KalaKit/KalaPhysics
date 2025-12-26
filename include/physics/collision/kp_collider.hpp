//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#pragma once

#include <vector>
#include <string>
#include <functional>

#include "KalaHeaders/core_utils.hpp"
#include "KalaHeaders/math_utils.hpp"
#include "KalaHeaders/log_utils.hpp"

#include "core/kp_registry.hpp"

namespace KalaPhysics::Core
{
	class PhysicsWorld;
}

namespace KalaPhysics::Physics::Collision
{
	using std::vector;
	using std::string;
	using std::function;

	using KalaHeaders::KalaMath::vec3;
	using KalaHeaders::KalaMath::quat;
	using KalaHeaders::KalaMath::Transform3D;
	using KalaHeaders::KalaLog::Log;
	using KalaHeaders::KalaLog::LogType;
	
	using KalaPhysics::Core::KalaPhysicsRegistry;

	enum class ColliderShape : u8
	{
		COLLIDER_BSP = 0,  //bounding sphere
		COLLIDER_AABB = 1, //axis-aligned bounding box
		COLLIDER_OBB = 2,  //oriented bounding box
		COLLIDER_BCP = 3,  //bounding capsule

		COLLIDER_KDOP_10_X = 4, //10-face Z-axis discrete oriented polytope
		COLLIDER_KDOP_10_Y = 5, //10-face Y-axis discrete oriented polytope
		COLLIDER_KDOP_10_Z = 6, //10-face Z-axis discrete oriented polytope
		COLLIDER_KDOP_18 = 7,   //18-face discrete oriented polytope
		COLLIDER_KDOP_26 = 8,   //26-face discrete oriented polytope

		COLLIDER_BCH = 9 //bounding convex hull
	};

	enum class ColliderType : u8
	{
		COLLIDER_TYPE_BP = 0, //broadphase
		COLLIDER_TYPE_NP = 1  //narrowphase
	};

	class LIB_API Collider
	{	
		friend class KalaPhysics::Core::PhysicsWorld;
	public:
		static inline KalaPhysicsRegistry<Collider> registry{};

		inline bool IsInitialized() const { return isInitialized; }

		inline u32 GetID() const { return ID; }

		//If true, then this collider wont be affected by its parent rigidbody,
		//a collider without a parent rigidbody is always treated as static, regardless of the static state
		inline void SetStaticState(bool newValue) { isStatic = newValue; }
		inline bool IsStatic() const { return isStatic; }
			
		//If true, then this collider allows other colliders to pass through it
		//and it will register all trigger events provided by the end user
		inline void SetTriggerState(bool newValue) { isTrigger = newValue; }
		inline bool IsTrigger() const { return isTrigger; }

		//Assign a new parent rigidbody to this collider, or 0 if no parent
		inline void SetParentRigidBody(u32 newValue) { parentRigidBody = newValue; }
		inline u32 GetParentRigidBody() const { return parentRigidBody; }

		//Set collider layer by name, use "NONE" to remove the layer completely
		void SetLayer(const string& layer);
		string GetLayer();

		inline ColliderShape GetColliderShape() const { return shape; }
		inline ColliderType GetColliderType() const { return type; }

		//Returns a reference to this collider vertices
		inline const vector<vec3>& GetVertices() const { return vertices; }
		//Returns a reference to this collider transform
		inline const Transform3D& GetTransform() const { return transform; }
		
		inline void SetOnTriggerEnter(const function<void()>& func) { if (func) onTriggerEnter = func; }
		inline void SetOnTriggerExit(const function<void()>& func) { if (func) onTriggerExit = func; }
		inline void SetOnTriggerStay(const function<void()>& func) { if (func) onTriggerStay = func; }
		
		inline void ClearOnTriggerEnter() { onTriggerEnter = nullptr; }
		inline void ClearOnTriggerExit() { onTriggerExit = nullptr; }
		inline void ClearOnTriggerStay() { onTriggerStay = nullptr; }
		
		virtual ~Collider() = default;
	private:
		virtual void Update(Collider* c, f32 deltaTime) {};

		bool isInitialized{};

		u32 ID{};
	
		bool isStatic{};
		bool isTrigger{};

		u32 parentRigidBody{};

		u8 layer = 255;
	
		ColliderShape shape{};
		ColliderType type{};

		vector<vec3> vertices;
		Transform3D transform;

		function<void()> onTriggerEnter{};
		function<void()> onTriggerExit{};
		function<void()> onTriggerStay{};
	};
}