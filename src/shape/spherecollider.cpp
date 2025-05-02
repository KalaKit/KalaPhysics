//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

//physics
#include "shape/spherecollider.hpp"
#include "shape/boxcollider.hpp"
#include "collision/sat.hpp"
#include "collision/contactgenerator.hpp"

using KalaKit::Physics::Collision::SAT;
using KalaKit::Physics::Collision::ContactGenerator;

namespace KalaKit::Physics::Shape
{
	SphereCollider::SphereCollider(
		const GameObjectHandle& handle)
		: Collider(ColliderType::SPHERE, handle)
	{
		CalculateBoundingRadius();
	}

	void SphereCollider::CalculateBoundingRadius()
	{
		boundingRadius = radius;
	}

	ContactManifold SphereCollider::GenerateContacts(
		const RigidBody& self,
		const RigidBody& other,
		const Collider& otherCol) const
	{
		switch (otherCol.GetColliderType())
		{
		case ColliderType::BOX:
		{
			const auto& box = static_cast<const BoxCollider&>(otherCol);
			ContactManifold manifold = ContactGenerator::GenerateBoxSphereContacts(
				self, 
				other, 
				box, 
				*this);

			//flip normal to point from this sphere
			for (auto& contact : manifold.contacts)
			{
				contact.normal = -contact.normal;
			}
			return manifold;
		}
		case ColliderType::SPHERE:
		{
			const auto& otherSphere = static_cast<const SphereCollider&>(otherCol);
			auto sat = SAT::PerformSphereSAT(self, other, *this, otherSphere);

			if (!sat.colliding) return {};

			return ContactGenerator::GenerateSphereContacts(self, other, *this, otherSphere, sat);
		}
		}

		return {};
	}
}