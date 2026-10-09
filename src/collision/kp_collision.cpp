//Copyright(C) 2026 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#include "collision/kp_collision.hpp"

using KalaHeaders::KalaMath::vec3;

using KalaPhysics::Collision::DEFAULT_GLOBAL_GRAVITY;
using KalaPhysics::Collision::CollisionMatrix;
using KalaPhysics::Collision::IgnoreList;

static bool isInitialized{};
static vec3 globalGravity = DEFAULT_GLOBAL_GRAVITY;
static CollisionMatrix collisionMatrix{};
static IgnoreList ignoreList{};

namespace KalaPhysics::Collision
{
    bool CollisionHandler::IsInitialized() { return isInitialized; }
    void CollisionHandler::Initialize()
    {
        if (isInitialized)
        {
            //not allowed...
            return;
        }
    }

    void CollisionHandler::Update(u8 substeps)
    {
        if (!isInitialized)
        {
            //not allowed...
            return;
        }
    }

    const vec3& CollisionHandler::GetGlobalGravity() { return globalGravity; }
    void CollisionHandler::SetGlobalGravity(const vec3& newValue) { globalGravity = newValue; }

    const CollisionMatrix& CollisionHandler::GetCollisionMatrix() { return collisionMatrix; }
    void CollisionHandler::SetCollisionMatrix(CollisionMatrix&& newValue)
    {
        if (!isInitialized)
        {
            //not allowed...
            return;
        }
    }

    const IgnoreList& CollisionHandler::GetIgnoreList() { return ignoreList; }
    void CollisionHandler::SetIgnoreList(IgnoreList&& newValue)
    {
        if (!isInitialized)
        {
            //not allowed...
            return;
        }
    }
    void CollisionHandler::AddToIgnoreList(IgnorePair&& newValue)
    {
        if (!isInitialized)
        {
            //not allowed...
            return;
        }
    }
    void CollisionHandler::RemoveFromIgnoreList(IgnorePair&& newValue)
    {
        if (!isInitialized)
        {
            //not allowed...
            return;
        }
    }
    
    void CollisionHandler::Shutdown()
    {
        if (!isInitialized)
        {
            //not allowed...
            return;
        }
    }
}