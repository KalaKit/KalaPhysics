//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#include <cmath>

//physics
#include "collisiondetection.hpp"

using glm::dot;
using glm::mat3;
using glm::mat3_cast;
using std::abs;
using glm::max;

namespace KalaKit
{
	ContactManifold CollisionDetection::GenerateOBBContactManifold(const RigidBody& a, const RigidBody& b)
	{
		ContactManifold manifold{};

		bool hit = CheckOBBCollision(a, b);
		if (!hit)
		{
			manifold.colliding = false;
			return manifold;
		}
		manifold.colliding = true;

		mat3 rotA = mat3_cast(a.combinedRotation);
		mat3 rotB = mat3_cast(b.combinedRotation);

		vec3 extentsA = static_cast<const BoxCollider*>(a.collider)->halfExtents;
		vec3 extentsB = static_cast<const BoxCollider*>(b.collider)->halfExtents;

		//relative delta
		vec3 delta = b.combinedPosition - a.combinedPosition;

		//find the best penetration axis
		float minPenetration = FLT_MAX;
		vec3 collisionNormal{};

		//check face axes (A's local axes)
		for (int i = 0; i < 3; i++)
		{
			vec3 axis = rotA[i];
			float overlap =
				extentsA[i] + abs(dot(rotB[0], axis))
				* extentsB[0] + abs(dot(rotB[1], axis))
				* extentsB[1] + abs(dot(rotB[2], axis))
				* extentsB[2];

			float penetration = overlap - abs(dot(delta, axis));
			if (penetration < 0.0f) return {}; //no collision

			if (penetration < minPenetration)
			{
				minPenetration = penetration;
				collisionNormal = axis * (dot(delta, axis) > 0 ? 1.0f : -1.0f);
			}
		}

		//check face axes (B's local axes)
		for (int i = 0; i < 3; i++)
		{
			vec3 axis = rotB[i];
			float overlap = 
				extentsB[i] + abs(dot(rotA[0], axis)) 
				* extentsA[0] + abs(dot(rotA[1], axis)) 
				* extentsA[1] + abs(dot(rotA[2], axis)) 
				* extentsA[2];

			float penetration = overlap - abs(dot(delta, axis));
			if (penetration < 0.0f) return {}; //no collision

			if (penetration < minPenetration)
			{
				minPenetration = penetration;
				collisionNormal = axis * (dot(delta, axis) > 0 ? -1.0f : 1.0f);
			}
		}

		//compute contact points using face clipping
		vec3 contactPoints[8]{};
		int contactCount = 0;

		//generate potential contact points
		vec3 incidentFaceCenter = b.combinedPosition - collisionNormal * extentsB;
		for (int i = 0; i < 8; i++)
		{
			vec3 corner = incidentFaceCenter;
			corner += (i & 1) ? rotB[0] * extentsB.x : -rotB[0] * extentsB.x;
			corner += (i & 2) ? rotB[1] * extentsB.y : -rotB[1] * extentsB.y;
			corner += (i & 4) ? rotB[2] * extentsB.z : -rotB[2] * extentsB.z;

			//max 4 contacts for OBB-OBB
			if (contactCount < 4)
			{
				contactPoints[contactCount++] = corner;
			}
		}

		//add contacts to the manifold
		for (int i = 0; i < contactCount; i++)
		{
			Contact c{};
			c.normal = normalize(collisionNormal);
			c.penetration = minPenetration;
			c.point = contactPoints[i];
			manifold.contacts.push_back(c);
		}

		return manifold;
	}

	bool CollisionDetection::CheckOBBCollision(const RigidBody& a, const RigidBody& b)
	{
		//no collision if no colliders
		if (!a.collider
			|| !b.collider)
		{
			return false;
		}

		if (a.collider->type != ColliderType::BOX
			|| b.collider->type != ColliderType::BOX)
		{
			return false;
		}

		const BoxCollider* boxA = static_cast<BoxCollider*>(a.collider);
		const BoxCollider* boxB = static_cast<BoxCollider*>(b.collider);

		const vec3& centerA = a.combinedPosition;
		const vec3& centerB = b.combinedPosition;
		const vec3& extentsA = boxA->halfExtents;
		const vec3& extentsB = boxB->halfExtents;
		const mat3& rotationA = mat3_cast(a.combinedRotation);;
		const mat3& rotationB = mat3_cast(b.combinedRotation);;

		//combute the rotation matrix expressing B in A's coordinate frame
		mat3 R{}, AbsR{};
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				R[i][j] = dot(rotationA[i], rotationB[j]);
				AbsR[i][j] = abs(R[i][j]) + 1e-6f;
			}
		}

		//compute translation vector t
		vec3 t = centerB - centerA;
		//express t in A's coordinate frame
		t = vec3(
			dot(t, rotationA[0]), 
			dot(t, rotationA[1]),
			dot(t, rotationA[2]));

		//test axes of A
		for (int i = 0; i < 3; i++)
		{
			float ra = extentsA[i];
			float rb =
				extentsB[0] * AbsR[i][0]
				+ extentsB[1] * AbsR[i][1]
				+ extentsB[2] * AbsR[i][2];

			if (abs(t[i]) > ra + rb) return false;
		}

		//test axes of B
		for (int i = 0; i < 3; i++)
		{
			float ra =
				extentsA[0] * AbsR[i][0]
				+ extentsA[1] * AbsR[i][1]
				+ extentsA[2] * AbsR[i][2];
			float rb = extentsB[i];

			if (abs(
				t[0] * R[0][i]
				+ t[1] * R[1][i]
				+ t[2] * R[2][i])
				> ra + rb)
			{
				return false;
			}
		}

		//test cross products of axes
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				float ra = 
					extentsA[(i + 1) % 3] * AbsR[(i + 2) % 3][j] 
					+ extentsA[(i + 2) % 3] * AbsR[(i + 1) % 3][j];
				float rb = 
					extentsB[(j + 1) % 3] * AbsR[i][(j + 2) % 3] 
					+ extentsB[(j + 2) % 3] * AbsR[i][(j + 1) % 3];

				if (abs(
					t[(i + 1) % 3] * R[(i + 2) % 3][j]
					- t[(i + 2) % 3] * R[(i + 1) % 3][j])
					> ra + rb)
				{
					return false;
				}
			}
		}

		return true; //no separating axis found, OBBs are colliding
	}

	bool CollisionDetection::CheckOBBCollisionAt(
		const RigidBody& movingBody, 
		const vec3& futurePosition, 
		const RigidBody& otherBody,
		float deltaTime)
	{
		RigidBody tempBody = movingBody;

		//update its position to the future position.
		tempBody.combinedPosition = futurePosition;

		vec3 futureAngularVelocity = movingBody.angularVelocity * deltaTime;
		tempBody.combinedRotation = normalize(movingBody.combinedRotation + quat(
			0,
			futureAngularVelocity.x,
			futureAngularVelocity.y,
			futureAngularVelocity.z) * 0.5f);

		//reuse the existing OBB collision detection.
		return CheckOBBCollision(tempBody, otherBody);
	}
}