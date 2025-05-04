//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#include <string>

//physics
#include "core/rigidbody.hpp"
#include "collision/collider.hpp"

using std::string;
using std::to_string;
using glm::max;

using KalaKit::Physics::Collision::BoxCollider;
using KalaKit::Physics::Collision::SphereCollider;

namespace KalaKit::Physics::Core
{
	RigidBody::RigidBody(
		GameObjectHandle h,
		const vec3& position,
		const quat& rotation,
		const vec3& scale,
		bool isDynamic,
		bool useGravity,
		float m,
		float rest,
		float staticFrict,
		float dynamicFrict,
		float gFactor) :
		handle(h),
		position(position),
		rotation(rotation),
		scale(scale),
		isDynamic(isDynamic),
		useGravity(useGravity),
		mass(m),
		restitution(rest),
		staticFriction(staticFrict),
		dynamicFriction(dynamicFrict),
		gravityFactor(gFactor) {}

	void RigidBody::ApplyForce(const vec3& force)
	{
		//static objects cant move
		if (!isDynamic) return;
		//objects with no mass cant move
		if (mass <= 0.0f) return;

		WakeUp();

		vec3 acceleration = force / mass;
		velocity += acceleration;
	}

	void RigidBody::ApplyImpulse(const vec3& impulse)
	{
		//static objects cant move
		if (!isDynamic) return;
		//objects with no mass cant move
		if (mass <= 0.0f) return;

		WakeUp();

		velocity += impulse / mass;
	}

	void RigidBody::ApplyTorque(const vec3& torque)
	{
		//static objects cant move
		if (!isDynamic) return;

		WakeUp();

		angularVelocity += torque / inertiaTensor;
	}

	void RigidBody::ComputeInertiaTensor()
	{
		if (!collider) return;

		if (collider->type == ColliderType::BOX)
		{
			BoxCollider* box = static_cast<BoxCollider*>(collider);
			vec3 halfExtents = box->halfExtents;

			float I_x = (1.0f / 12.0f) * mass * (halfExtents.y * halfExtents.y + halfExtents.z * halfExtents.z);
			float I_y = (1.0f / 12.0f) * mass * (halfExtents.x * halfExtents.x + halfExtents.z * halfExtents.z);
			float I_z = (1.0f / 12.0f) * mass * (halfExtents.x * halfExtents.x + halfExtents.y * halfExtents.y);

			inertiaTensor = vec3(I_x, I_y, I_z);
		}
		else if (collider->type == ColliderType::SPHERE)
		{
			SphereCollider* sphere = static_cast<SphereCollider*>(collider);
			float inertia = (2.0f / 5.0f) * mass * (sphere->radius * sphere->radius);
			inertiaTensor = vec3(inertia);
		}
	}

	void RigidBody::UpdateCenterOfGravity()
	{
		if (!collider 
			|| collider->type != ColliderType::BOX)
		{
			centerOfGravity = vec3(0.0f);
			return;
		}

		const BoxCollider* box = static_cast<const BoxCollider*>(collider);
		vec3 halfExtents = box->halfExtents;

		//find the largest axis
		float maxExtent = 
			max(halfExtents.x, 
			max(halfExtents.y, 
				halfExtents.z));

		//wide along X (horizontal, sideways object)
		if (halfExtents.x > halfExtents.y 
			&& halfExtents.x > halfExtents.z)
		{
			centerOfGravity = vec3(halfExtents.x * 0.2f, 0.0f, 0.0f);
		}
		//tall along Y (standing vertical object)
		else if (halfExtents.y > halfExtents.x 
			&& halfExtents.y > halfExtents.z)
		{
			centerOfGravity = vec3(0.0f, -halfExtents.y * 0.2f, 0.0f);
		}
		//deep along Z (lying forward/backward)
		else if (halfExtents.z > halfExtents.x 
			&& halfExtents.z > halfExtents.y)
		{
			centerOfGravity = vec3(0.0f, 0.0f, halfExtents.z * 0.2f);
		}
		//almost uniform cube shape, keep CoG centered
		else centerOfGravity = vec3(0.0f);
	}

	void RigidBody::SetScale(const vec3& newScale)
	{
		scale = newScale;

		BoxCollider* box = static_cast<BoxCollider*>(collider);
		SphereCollider* sphere = static_cast<SphereCollider*>(collider);

		if (box != nullptr)
		{
			box->halfExtents = vec3(scale * 0.5f);
			box->boundingRadius = length(box->halfExtents) * 2.0f * 0.5f;
		}
		else if (sphere != nullptr)
		{
			sphere->radius = scale.x;
			sphere->boundingRadius = sphere->radius;
		}

		UpdateCenterOfGravity();
		ComputeInertiaTensor();
	}

	void RigidBody::SetCollider(ColliderType type)
	{
		if (collider) delete collider;

		uint32_t index = handle.index;
		uint32_t gen = handle.generation;
		string sizeString{};
		string colliderType{};

		if (type == ColliderType::BOX)
		{
			collider = new BoxCollider(handle);

			sizeString = to_string(scale.x) + ", " + to_string(scale.y) + ", " + to_string(scale.z);
			colliderType = "box";
		}
		else if (type == ColliderType::SPHERE)
		{
			collider = new SphereCollider(handle);

			sizeString = to_string(scale.x);
			colliderType = "sphere";
		}

		LOG_SUCCESS("Set size to '" + sizeString 
			+ "' and collider to " + colliderType 
			+ " for rigidbody(" + to_string(index) + ", " + to_string(gen) + ")!");
	}

	void RigidBody::WakeUp()
	{
		isSleeping = false;
		sleepTimer = 0.0f;
	}

	void RigidBody::Sleep()
	{
		isSleeping = true;
		velocity = vec3(0.0f);
		angularVelocity = vec3(0.0f);
	}
}