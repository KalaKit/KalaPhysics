//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

//main log macro
#define WRITE_LOG(type, msg) std::cout << "[KALAKIT_PHYSICS | " << type << "] " << msg << "\n"

//log types
#if KALAPHYSICS_DEBUG
	#define LOG_DEBUG(msg) WRITE_LOG("DEBUG", msg)
#else
	#define LOG_DEBUG(msg)
#endif
#define LOG_SUCCESS(msg) WRITE_LOG("SUCCESS", msg)
#define LOG_ERROR(msg) WRITE_LOG("ERROR", msg)

//external
#include "glm.hpp"

//physics
#include "simulation/handlemotion.hpp"

using glm::vec3;
using glm::quat;
using glm::clamp;
using glm::degrees;

namespace KalaKit::Physics::Simulation
{
	void HandleMotion::Run(
		PhysicsWorld& world,
		float deltaTime)
	{
		for (auto& bodyPtr : world.GetBodies())
		{
			RigidBody& body = *bodyPtr;

			if (!body.isDynamic
				|| !body.collider
				|| !body.useGravity
				|| body.isSleeping)
			{
				continue;
			}

			ApplyForces(world, body, deltaTime);
			ProjectAgainstSurfaces(world, bodyPtr, body, deltaTime);
			Integrate(world, body, deltaTime);
			ApplyDamping(body, deltaTime);
			UpdateSleepState(body, deltaTime);

			body.tiltTimer += deltaTime;
			if (body.tiltTimer >= 0.05f)
			{
				if (CanTilt(body)) TiltBody(body);
				body.tiltTimer = 0.0f;
			}
		}
	}

	void HandleMotion::ApplyForces(
		PhysicsWorld& world,
		RigidBody& body,
		float deltaTime)
	{
		if (!body.useGravity
			|| !body.collider)
		{
			return;
		}

		vec3 gravityImpulse = world.GetGravity() * body.gravityFactor * deltaTime;
		body.ApplyImpulse(gravityImpulse * body.mass);
	}

	void HandleMotion::ProjectAgainstSurfaces(
		PhysicsWorld& world,
		RigidBody* bodyPtr,
		RigidBody& body,
		float deltaTime)
	{
		if (!body.collider
			|| !body.collider->IsGrounded())
		{
			return;
		}

		const vec3 groundNormal = body.collider->GetGroundNormal();
		const float intoSurface = dot(body.velocity, groundNormal);

		if (intoSurface < 0.0f)
		{
			body.velocity -= groundNormal * intoSurface;
		}
	}

	void HandleMotion::Integrate(
		PhysicsWorld& world,
		RigidBody& body,
		float deltaTime)
	{
		body.position += body.velocity * deltaTime;

		if (dot(body.angularVelocity, body.angularVelocity) > 1e-6f)
		{
			const quat spin(
				0,
				body.angularVelocity.x,
				body.angularVelocity.y,
				body.angularVelocity.z);

			body.rotation = normalize(body.rotation + 0.5f * spin * body.rotation * deltaTime);
		}
	}

	void HandleMotion::ApplyDamping(
		RigidBody& body,
		float deltaTime)
	{
		//linear damp

		const float linearDamp = pow(0.99f, deltaTime * 60.0f);
		body.velocity *= linearDamp;

		//angular damp

		if (body.angleToFlat >= 2.0f
			&& body.angleToFlat < 15.0f)
		{
			body.angularVelocity *= 0.90f;
		}

		if (dot(body.angularVelocity, body.angularVelocity) < 0.0025f)
		{
			body.angularVelocity *= 0.85f;
		}
		else
		{
			body.angularVelocity *= pow(0.95f, deltaTime * 60.0f);
		}
	}

	void HandleMotion::UpdateSleepState(
		RigidBody& body,
		float deltaTime)
	{
		const float velocity2 = dot(body.velocity, body.velocity);
		const float angular2 = dot(body.angularVelocity, body.angularVelocity);
		const float threshold2 = body.sleepThreshold * body.sleepThreshold;

		if (velocity2 < threshold2
			&& angular2 < threshold2)
		{
			body.sleepTimer += deltaTime;
			if (body.sleepTimer > 2.0f) body.Sleep();
		}
		else
		{
			body.sleepTimer = 0.0f;
			body.WakeUp();
		}
	}

	bool HandleMotion::CanTilt(RigidBody& body)
	{
		const vec3 faces[6] =
		{
			body.rotation * vec3( 0,  1,  0),
			body.rotation * vec3( 0, -1,  0),
			body.rotation * vec3( 1,  0,  0),
			body.rotation * vec3(-1,  0,  0),
			body.rotation * vec3( 0,  0,  1),
			body.rotation * vec3( 0,  0, -1)
		};

		vec3 best = faces[0];
		for (int i = 1; i < 6; ++i)
		{
			if (abs(dot(faces[i], vec3(0, 1, 0))) > abs(dot(best, vec3(0, 1, 0))))
			{
				best = faces[i];
			}
		}

		body.closestUp = best;
		const float upDot = clamp(dot(best, vec3(0, 1, 0)), -1.0f, 1.0f);
		body.angleToFlat = degrees(acos(upDot));

		return body.angleToFlat > 1.0f;
	}
	void HandleMotion::TiltBody(RigidBody& body)
	{
		vec3 tiltAxis = cross(body.closestUp, vec3(0, 1, 0));
		if (dot(tiltAxis, tiltAxis) < 1e-6f)
		{
			tiltAxis = vec3(1, 0, 0);
		}

		const float scale = clamp(body.angleToFlat / 15.0f, 0.0f, 1.0f);
		const vec3 torque = tiltAxis * scale * 5.0f;

		if (body.angleToFlat >= 15.0f)
		{
			body.ApplyTorque(torque);
		}
		else if (body.angleToFlat >= 2.0f)
		{
			body.ApplyTorque(torque * 1.25f);
		}
		else if (body.angleToFlat < 2.0f
				 && dot(body.angularVelocity, body.angularVelocity) < 0.01f)
		{
			body.angularVelocity = vec3(0.0f);
			body.rotation = normalize(body.rotation);
		}
	}
}