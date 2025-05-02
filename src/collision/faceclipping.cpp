//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

//physics
#include "collision/faceclipping.hpp"

using glm::dot;

namespace KalaKit::Physics::Collision
{
	vector<vec3> FaceClipping::ClipFaceAgainstPlane(
		const vector<vec3>& face,
		const vec3& planeNormal,
		float planeOffset)
	{
		vector<vec3> clipped{};

		if (face.empty()) return clipped;

		vec3 prev = face.back();
		float prevDist = dot(planeNormal, prev) - planeOffset;

		for (const vec3& curr : face)
		{
			float currDist = dot(planeNormal, curr) - planeOffset;

			//current is inside
			if (currDist <= 0.0f)
			{
				//if previous was outside, clip the edge between them
				if (prevDist > 0.0f)
				{
					float t = prevDist / (prevDist - currDist);
					vec3 point = prev + t * (curr - prev);
					clipped.push_back(point);
				}
				clipped.push_back(curr);
			}
			//current is outside
			else if (prevDist <= 0.0f)
			{
				//previous was inside but current is outside,
				//clip edge and keep intersection point

				float t = prevDist / (prevDist - currDist);
				vec3 point = prev + t * (curr - prev);
				clipped.push_back(point);
			}

			prev = curr;
			prevDist = currDist;
		}

		return clipped;
	}
}