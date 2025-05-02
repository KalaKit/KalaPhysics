//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

//physics
#include "shape/boxcollider.hpp"
#include "shape/spherecollider.hpp"
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

	ContactManifold BoxCollider::GenerateContacts(
		const RigidBody& self,
		const RigidBody& other,
		const Collider& otherCol) const
	{
		switch (otherCol.GetColliderType())
		{
		case ColliderType::BOX :
		{
			const auto& otherBox = static_cast<const BoxCollider&>(otherCol);
			auto sat = SAT::PerformBoxSAT(self, other, *this, otherBox);

			if (!sat.colliding) return {};

			return ContactGenerator::GenerateBoxContacts(self, other, *this, otherBox, sat);
		}
		case ColliderType::SPHERE:
		{
			const auto& sphere = static_cast<const SphereCollider&>(otherCol);
			return ContactGenerator::GenerateBoxSphereContacts(
				self, 
				other, 
				*this, 
				sphere);
		}
		}

		return {};
	}
}