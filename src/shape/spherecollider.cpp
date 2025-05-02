//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

//physics
#include "shape/spherecollider.hpp"
#include "collision/sat.hpp"
#include "collision/contactgenerator.hpp"

using KalaKit::Physics::Collision::SAT;
using KalaKit::Physics::Collision::ContactGenerator;

namespace KalaKit::Physics::Shape
{
	SphereCollider::SphereCollider(
		const vec3& offsetScale,
		const vec3& combinedScale,
		const GameObjectHandle& handle)
		: Collider(offsetScale,
				   combinedScale,
				   ColliderType::SPHERE,
				   handle),
		radius(combinedScale.x * 0.5f)
	{
		CalculateBoundingRadius();
	}

	void SphereCollider::UpdateScale(const vec3& newCombinedScale)
	{
		combinedScale = newCombinedScale;
		radius = combinedScale.x * 0.5f;
		CalculateBoundingRadius();
	}

	void SphereCollider::CalculateBoundingRadius()
	{
		boundingRadius = radius;
	}

	SATResult SphereCollider::SATAgainst(
		const RigidBody& self,
		const RigidBody& other,
		const Collider& otherCol) const
	{
		if (otherCol.GetColliderType() != ColliderType::SPHERE)
		{
			return {};
		}

		const auto& otherSphere = static_cast<const SphereCollider&>(otherCol);
		return SAT::PerformSphereSAT(self, other, *this, otherSphere);
	}

	ContactManifold SphereCollider::GenerateContacts(
		const RigidBody& self,
		const RigidBody& other,
		const Collider& otherCol) const
	{
		if (otherCol.GetColliderType() != ColliderType::SPHERE)
		{
			return {};
		}

		const auto& otherSphere = static_cast<const SphereCollider&>(otherCol);
		auto sat = SAT::PerformSphereSAT(self, other, *this, otherSphere);

		if (!sat.colliding)
		{
			return {};
		}

		return ContactGenerator::GenerateSphereContacts(self, other, *this, otherSphere, sat);
	}
}