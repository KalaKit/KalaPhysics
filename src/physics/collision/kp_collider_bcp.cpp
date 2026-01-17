//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#include "physics/collision/kp_collider_bcp.hpp"

namespace KalaPhysics::Physics::Collision
{
	Collider_BCP* Collider_BCP::Initialize(
		u32 parentRigidbody,
		const vec3& pos,
		f32 height,
		f32 radius,
		ColliderType type)
	{
		return nullptr;
	}

	void Collider_BCP::Update(Collider* c, f32 deltaTime)
	{

	}

	const vec3& Collider_BCP::GetPos() const { return pos; }
	void Collider_BCP::SetPos(const vec3& newValue)
	{
		pos = kclamp(newValue, MIN_BCP_POS, MAX_BCP_POS);
	}

	f32 Collider_BCP::GetHeight() const { return height; }
	void Collider_BCP::SetHeight(f32 newValue)
	{
		height = clamp(newValue, MIN_BCP_HEIGHT, MAX_BCP_HEIGHT);
		radius = min(radius, height * 0.5f);
	}

	f32 Collider_BCP::GetRadius() const { return radius; }
	void Collider_BCP::SetRadius(f32 newValue)
	{
		radius = clamp(newValue, MIN_BCP_RADIUS, MAX_BCP_RADIUS);
		height = max(height, 2 * radius);
	}

	Collider_BCP::~Collider_BCP()
	{

	}
}