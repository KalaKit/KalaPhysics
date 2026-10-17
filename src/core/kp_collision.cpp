//Copyright(C) 2026 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#include "log_utils.hpp"

#include "core/kp_collision.hpp"
#include "core/kp_core.hpp"
#include "collision/kp_collider.hpp"
#include "collision/kp_rigidbody.hpp"

using KalaHeaders::KalaLog::Log;
using KalaHeaders::KalaLog::LogType;

using KalaHeaders::KalaMath::vec3;

using KalaPhysics::Core::DEFAULT_GLOBAL_GRAVITY;
using KalaPhysics::Core::CollisionMatrix;
using KalaPhysics::Core::IgnoreList;
using KalaPhysics::Collision::Collider;
using KalaPhysics::Collision::RigidBody;

static bool isInitialized{};
static vec3 globalGravity = DEFAULT_GLOBAL_GRAVITY;
static CollisionMatrix collisionMatrix{};
static IgnoreList ignoreList{};

namespace KalaPhysics::Core
{
    bool CollisionManager::IsInitialized() { return isInitialized; }
    void CollisionManager::Initialize()
    {
        if (isInitialized)
        {
            Log::Print(
                "Failed to initialize collision manager because it has already been initialized!",
                "KP_COLLISION_MANAGER",
                LogType::LOG_WARNING);

            return;
        }
    }

    void CollisionManager::Update(u8 substeps)
    {
        if (!isInitialized)
        {
            Log::Print(
                "Failed to update collision manager because it has not yet been initialized!",
                "KP_COLLISION_MANAGER",
                LogType::LOG_WARNING);

            return;
        }
    }

    const vec3& CollisionManager::GetGlobalGravity() { return globalGravity; }
    void CollisionManager::SetGlobalGravity(const vec3& newValue) { globalGravity = newValue; }

    const CollisionMatrix& CollisionManager::GetCollisionMatrix() { return collisionMatrix; }
    void CollisionManager::SetCollisionMatrix(CollisionMatrix&& newValue)
    {
        if (!isInitialized)
        {
            Log::Print(
                "Failed to set collision matrix because the collision manager has not yet been initialized!",
                "KP_COLLISION_MANAGER",
                LogType::LOG_WARNING);

            return;
        }
    }

    const IgnoreList& CollisionManager::GetIgnoreList() { return ignoreList; }
    void CollisionManager::SetIgnoreList(IgnoreList&& newValue)
    {
        if (!isInitialized)
        {
            Log::Print(
                "Failed to set ignore list because the collision manager has not yet been initialized!",
                "KP_COLLISION_MANAGER",
                LogType::LOG_WARNING);

            return;
        }
    }
    void CollisionManager::AddToIgnoreList(IgnorePair&& newValue)
    {
        if (!isInitialized)
        {
            Log::Print(
                "Failed to add to ignore list because the collision manager has not yet been initialized!",
                "KP_COLLISION_MANAGER",
                LogType::LOG_WARNING);

            return;
        }
    }
    void CollisionManager::RemoveFromIgnoreList(IgnorePair&& newValue)
    {
        if (!isInitialized)
        {
            Log::Print(
                "Failed to remove from ignore list because the collision manager has not yet been initialized!",
                "KP_COLLISION_MANAGER",
                LogType::LOG_WARNING);

            return;
        }
    }
    
    void CollisionManager::Shutdown()
    {
        if (!isInitialized)
        {
            Log::Print(
                "Failed to shut down collision manager because it has not yet been initialized!",
                "KP_COLLISION_MANAGER",
                LogType::LOG_WARNING);

            return;
        }
    }
}