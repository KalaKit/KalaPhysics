//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

//physics
#include "shape/boxcollider.hpp"
#include "collision/sat.hpp"
#include "collision/contactgenerator.hpp"

using glm::length;

using KalaKit::Physics::Collision::SAT;
using KalaKit::Physics::Collision::ContactGenerator;

namespace KalaKit::Physics::Shape
{
	BoxCollider::BoxCollider(
		const vec3& offsetScale,
		const vec3& combinedScale,
		const GameObjectHandle& handle)
		: Collider(offsetScale,
				   combinedScale,
			       ColliderType::BOX,
				   handle),
		halfExtents(combinedScale * 0.5f)
	{
		CalculateBoundingRadius();
	}

	void BoxCollider::UpdateScale(const vec3& newCombinedScale)
	{
		combinedScale = newCombinedScale;
		halfExtents = combinedScale * 0.5f;
		CalculateBoundingRadius();
	}

	void BoxCollider::CalculateBoundingRadius()
	{
		boundingRadius = length(halfExtents);
	}

	SATResult BoxCollider::SATAgainst(
		const RigidBody& self,
		const RigidBody& other,
		const Collider& otherCol) const
	{
		if (otherCol.GetColliderType() != ColliderType::BOX)
		{
			return{};
		}

		const auto& otherBox = static_cast<const BoxCollider&>(otherCol);
		return SAT::PerformBoxSAT(self, other, *this, otherBox);
	}

	ContactManifold BoxCollider::GenerateContacts(
		const RigidBody& self,
		const RigidBody& other,
		const Collider& otherCol) const
	{
		if (otherCol.GetColliderType() != ColliderType::BOX)
		{
			return{};
		}

		const auto& otherBox = static_cast<const BoxCollider&>(otherCol);
		SATResult sat = SAT::PerformBoxSAT(self, other, *this, otherBox);

		if (!sat.colliding)
		{
			return {};
		}

		return ContactGenerator::GenerateBoxContacts(self, other, *this, otherBox, sat);
	}
}