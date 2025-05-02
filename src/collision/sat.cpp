//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

//external
#include "gtx/quaternion.hpp"

//physics
#include "collision/sat.hpp"
#include "core/rigidbody.hpp"
#include "shape/boxcollider.hpp"
#include "shape/spherecollider.hpp"

using glm::dot;
using glm::sqrt;
using glm::abs;
using glm::mat3_cast;
using glm::vec3;
using glm::mat3;

namespace KalaKit::Physics::Collision
{
	SATResult SAT::PerformBoxSAT(
		const RigidBody& bodyA,
		const RigidBody& bodyB,
		const BoxCollider& boxA,
		const BoxCollider& boxB)
	{
		const vec3& centerA = bodyA.combinedPosition;
		const vec3& centerB = bodyB.combinedPosition;
		const vec3& extentsA = boxA.halfExtents;
		const vec3& extentsB = boxB.halfExtents;

		const mat3 rotA = mat3_cast(bodyA.combinedRotation);
		const mat3 rotB = mat3_cast(bodyB.combinedRotation);

		//build a rotation matrix to compare directions of the two boxes.
		//AbsR is used to test distances safely
		//without negative values or floating point errors

		mat3 R{}, AbsR{};
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				R[i][j] = dot(rotA[i], rotB[j]);

				//add a tiny bias to prevent divide-by zero problems
				AbsR[i][j] = abs(R[i][j]) + 1e-6f;
			}
		}

		//figure out how far apart the boxes are, from A to B,
		//using box A's local coordinate system

		vec3 t = centerB - centerA;
		t = vec3(dot(t, rotA[0]), dot(t, rotA[1]), dot(t, rotA[2]));

		float minPenetration = FLT_MAX;
		vec3 bestAxis = vec3(0.0f);

		//check A's local axes to see if there's
		//a gap between the boxes on any of them
		for (int i = 0; i < 3; ++i)
		{
			//how far each box extends along axis i

			float ra = extentsA[i];
			float rb =
				extentsB[0] * AbsR[i][0]
				+ extentsB[1] * AbsR[i][1]
				+ extentsB[2] * AbsR[i][2];

			float distance = abs(t[i]);
			float overlap = ra + rb - distance;

			//if there's a gap on this axis, the boxes are not touching
			if (overlap < 0.0f) return { false };

			//track the axis with the smallest overlap - 
			//it's the best one to push them apart with
			if (overlap < minPenetration)
			{
				minPenetration = overlap;
				bestAxis = rotA[i] * (t[i] < 0 ? -1.0f : 1.0f);
			}
		}

		//now check B's local axes the same way
		for (int i = 0; i < 3; ++i)
		{
			float ra =
				extentsA[0] * AbsR[0][i]
				+ extentsA[1] * AbsR[1][i]
				+ extentsA[2] * AbsR[2][i];
			float rb = extentsB[i];

			//project the distance vector onto B's axis

			float proj = dot(t, vec3(
				R[0][i], 
				R[1][i],
				R[2][i]));
			float distance = abs(proj);
			float overlap = ra + rb - distance;

			if (overlap < 0.0f) return { false };

			if (overlap < minPenetration)
			{
				minPenetration = overlap;
				bestAxis = rotB[i] * (proj < 0 ? -1.0f : 1.0f);
			}
		}

		//if no gaps were found, the boxes are colliding.
		//return the axis with the smallest overlap as the "separation axis"
		return SATResult{ true, bestAxis, minPenetration };
	}

	SATResult SAT::PerformSphereSAT(
		const RigidBody& bodyA,
		const RigidBody& bodyB,
		const SphereCollider& sphereA,
		const SphereCollider& sphereB)
	{
		//get the vector between the centers of the two spheres

		vec3 delta = bodyB.combinedPosition - bodyA.combinedPosition;

		//compare how far apart they are to how big they are combined

		float distSq = dot(delta, delta);
		float combinedRadius = sphereA.radius + sphereB.radius;
		float combinedRadiusSq = combinedRadius * combinedRadius;

		//if they are too far apart. they aren't colliding
		if (distSq > combinedRadiusSq)
		{
			return { false };
		}

		//they are overlapping - compute how deep
		//the overlap is and in which direction

		float distance = sqrt(distSq);

		//if they are exactly on top of each other, choose a default direction

		vec3 normal = (distance > 1e-5f) ? (delta / distance) : vec3(0, 1, 0);

		return SATResult{ true, normal, combinedRadius - distance };
	}
}