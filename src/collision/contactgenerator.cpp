//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#include <algorithm>

//external
#include "gtx/quaternion.hpp"

//physics
#include "collision/contactgenerator.hpp"
#include "core/rigidbody.hpp"
#include "shape/boxcollider.hpp"
#include "shape/spherecollider.hpp"
#include "collision/faceclipping.hpp"

using glm::length;
using glm::vec3;
using glm::mat3_cast;
using glm::mat3;
using glm::dot;
using std::sort;
using std::min;

namespace KalaKit::Physics::Collision
{
	ContactManifold ContactGenerator::GenerateBoxContacts(
		const RigidBody& bodyA,
		const RigidBody& bodyB,
		const BoxCollider& boxA,
		const BoxCollider& boxB,
		const SATResult& sat)
	{
		ContactManifold manifold{};
		manifold.colliding = true;

		//convert orientations into 3x3 rotation matrices

		mat3 rotA = mat3_cast(bodyA.combinedRotation);
		mat3 rotB = mat3_cast(bodyB.combinedRotation);

		//get each box's world center and local half extents

		vec3 centerA = bodyA.combinedPosition;
		vec3 centerB = bodyB.combinedPosition;
		vec3 extentsA = boxA.halfExtents;
		vec3 extentsB = boxB.halfExtents;

		//determine which box is the "preference" and which is the "incident",
		//this affects which face gets used for clipping

		bool flip = dot(sat.axis, centerB - centerA) < 0.0f;
		const RigidBody& refBody = flip ? bodyB : bodyA;
		const BoxCollider& refBox = flip ? boxB : boxA;
		mat3 refRot = flip ? rotB : rotA;
		vec3 refExtents = flip ? extentsB : extentsA;

		const RigidBody& incBody = flip ? bodyA : bodyB;
		const BoxCollider& incBox = flip ? boxA : boxB;
		mat3 incRot = flip ? rotA : rotB;
		vec3 incExtents = flip ? extentsA : extentsB;

		vec3 refNormal = flip ? -sat.axis : sat.axis;

		//find which face of the reference box is
		//most aligned with the collision normal

		int bestRefFace = 0;
		float maxDot = dot(refRot[0], refNormal);
		for (int i = 1; i < 3; ++i)
		{
			float d = dot(refRot[i], refNormal);
			if (d > maxDot)
			{
				maxDot = d;
				bestRefFace = i;
			}
		}

		//get the outward normal of that face in world space

		vec3 worldNormal = refRot[bestRefFace];
		if (dot(worldNormal, refNormal) < 0.0f)
		{
			worldNormal = -worldNormal;
		}

		//compute the plane offset (distance from origin along the face normal)

		vec3 refCenter = refBody.combinedPosition;
		float planeOffset = dot(worldNormal, refCenter + worldNormal * refExtents[bestRefFace]);

		//find the face of the incident box that is
		//most opposed to the reference normal

		int bestIncFace = 0;
		float minDot = dot(incRot[0], -refNormal);
		for (int i = 1; i < 3; ++i)
		{
			float d = dot(incRot[i], -refNormal);
			if (d < minDot)
			{
				minDot = d;
				bestIncFace = i;
			}
		}

		//compute the four corners of the incident face in world space

		vec3 axisU = incRot[(bestIncFace + 1) % 3];
		vec3 axisV = incRot[(bestIncFace + 2) % 3];
		vec3 faceCenter = incBody.combinedPosition - incRot[bestIncFace] * incExtents[bestIncFace];

		vector<vec3> incidentFace{};
		incidentFace.push_back(
			faceCenter + axisU * incExtents[(bestIncFace + 1) % 3] 
			+ axisV * incExtents[(bestIncFace + 2) % 3]);
		incidentFace.push_back(
			faceCenter - axisU * incExtents[(bestIncFace + 1) % 3]
			+ axisV * incExtents[(bestIncFace + 2) % 3]);
		incidentFace.push_back(
			faceCenter - axisU * incExtents[(bestIncFace + 1) % 3]
			- axisV * incExtents[(bestIncFace + 2) % 3]);
		incidentFace.push_back(
			faceCenter + axisU * incExtents[(bestIncFace + 1) % 3]
			- axisV * incExtents[(bestIncFace + 2) % 3]);

		//clip the incident face against the reference face's plane and edges

		FaceClipping clipper{};
		vector<vec3> clipped = clipper.ClipFaceAgainstPlane(
			incidentFace,
			worldNormal,
			planeOffset);

		vec3 refU = refRot[(bestRefFace + 1) % 3];
		vec3 refV = refRot[(bestRefFace + 2) % 3];
		float uExtent = refExtents[(bestRefFace + 1) % 3];
		float vExtent = refExtents[(bestRefFace + 2) % 3];

		//clip against four side planes of the reference face
		//to stay within its rectangle

		clipped = clipper.ClipFaceAgainstPlane(
			clipped, 
			-refU, 
			dot(refU, refCenter + refU * uExtent));
		clipped = clipper.ClipFaceAgainstPlane(
			clipped,  
			refU, 
			-dot(refU, refCenter - refU * uExtent));
		clipped = clipper.ClipFaceAgainstPlane(
			clipped, 
			-refV, 
			dot(refV, refCenter + refV * uExtent));
		clipped = clipper.ClipFaceAgainstPlane(
			clipped,  
			refV, 
			-dot(refV, refCenter - refV * uExtent));

		//for each clipped point, calculate how deep
		//it penetrated the reference face

		vector<Contact> allContacts;
		for (const vec3& pt : clipped)
		{
			float depth = dot(worldNormal, pt) - planeOffset;
			if (depth <= 0.0f)
			{
				Contact c{};
				c.point = pt;
				c.normal = refNormal;
				c.penetration = -depth;
				allContacts.push_back(c);
			}
		}

		//sort contacts by penetration depth and
		//take the four deepest for stability

		sort(allContacts.begin(), allContacts.end(),
			[](const Contact& a, const Contact& b)
			{
				return a.penetration > b.penetration;
			});

		//keep only the four deepest contacts for stability and performance
		for (size_t i = 0; i < min(size_t(4), allContacts.size()); ++i)
		{
			manifold.contacts.push_back(allContacts[i]);
		}

		return manifold;
	}

	ContactManifold ContactGenerator::GenerateSphereContacts(
		const RigidBody& bodyA,
		const RigidBody& bodyB,
		const SphereCollider& sphereA,
		const SphereCollider& sphereB,
		const SATResult& sat)
	{
		ContactManifold manifold{};
		manifold.colliding = true;

		vec3 posA = bodyA.combinedPosition;
		vec3 posB = bodyB.combinedPosition;

		vec3 delta = posB - posA;
		float distSq = dot(delta, delta);
		float combinedRadius = sphereA.radius + sphereB.radius;

		//spheres are not touching
		if (distSq > combinedRadius * combinedRadius) return {};

		float distance = sqrt(distSq);

		//if completely overlapping, use a fallback up vector to avoid NaNs
		vec3 normal = (distance > 1e-5f) ? (delta / distance) : vec3(0, 1, 0);

		//point of the surface of sphere A in the direction of collision
		vec3 contactPoint = posA + normal * sphereA.radius;

		Contact contact{};
		contact.point = contactPoint;
		contact.normal = normal;
		contact.penetration = combinedRadius - distance;

		manifold.contacts.push_back(contact);
		return manifold;
	}
}