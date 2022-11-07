#pragma once
#include <cassert>
#include <fstream>
#include "Math.h"
#include "DataTypes.h"
#include <iostream>

namespace dae
{
	namespace GeometryUtils
	{
#pragma region Sphere HitTest
		//SPHERE HIT-TESTS
		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			hitRecord.didHit = false;
			
			const float A{ Vector3::Dot(ray.direction, ray.direction) };
			const float B{ Vector3::Dot(2 * ray.direction, (ray.origin - sphere.origin)) };
			const float C{ Vector3::Dot((ray.origin - sphere.origin), (ray.origin - sphere.origin)) - (sphere.radius * sphere.radius) };

            const float discriminant{ (B * B) - (4 * A * C) };

            if (discriminant < 0) return false;

            const float sqrtDiscriminant = std::sqrtf(discriminant);
			const float t{ (-B - sqrtDiscriminant) / (2 * A) };

			if (t < ray.min || t * t > ray.max) return false;

            if (ignoreHitRecord) return true;

            hitRecord.didHit = true;
            hitRecord.materialIndex = sphere.materialIndex;
            hitRecord.t = t;
            hitRecord.origin = ray.origin + (hitRecord.t * ray.direction);
			hitRecord.normal = (hitRecord.origin - sphere.origin).Normalized();

            return true;
		}

		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Sphere(sphere, ray, temp, true);
		}
#pragma endregion
#pragma region Plane HitTest
		//PLANE HIT-TESTS
		inline bool HitTest_Plane(const Plane& plane, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			hitRecord.didHit = false;

			const float dirDotNormal{ Vector3::Dot(ray.direction, plane.normal) };

			const float t{ Vector3::Dot((plane.origin - ray.origin), plane.normal) / dirDotNormal };

			if (t < ray.min || t * t > ray.max) return false;
			
			if (ignoreHitRecord) return true;

			hitRecord.didHit = true;
			hitRecord.materialIndex = plane.materialIndex;
			hitRecord.normal = plane.normal;

			hitRecord.origin = ray.origin + ray.direction * t;
			hitRecord.t = t;
			
			return true;
		}

		inline bool HitTest_Plane(const Plane& plane, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Plane(plane, ray, temp, true);
		}
#pragma endregion
#pragma region Triangle HitTest
		//TRIANGLE HIT-TESTS
		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			const Vector3 a { triangle.v1 - triangle.v0 };
			const Vector3 b { triangle.v2 - triangle.v0 };


			//Möller–Trumbore algorithm - faster than analytical method and "correct side" method
			const Vector3 tVec{ Vector3::Cross(ray.direction, b) };
			const float determinant{ Vector3::Dot(a, tVec) };

			//culling
			if (determinant > -.01f && determinant < .01f) return false; //ray is parallel
			const bool isBackFacing{ determinant < 0 };
			const TriangleCullMode cullMode{ ignoreHitRecord == true ? static_cast<TriangleCullMode>((static_cast<int>(triangle.cullMode) + 1) % 2) : triangle.cullMode };
			if (cullMode == TriangleCullMode::FrontFaceCulling && !isBackFacing) return false;
			if (cullMode == TriangleCullMode::BackFaceCulling && isBackFacing) return false;

			const float invDeterminant{ 1 / determinant };
			const Vector3 v0ToOrigin{ ray.origin - triangle.v0 };

			//calculate barycentric U parameter 
			const float u{ Vector3::Dot(v0ToOrigin, tVec) * invDeterminant };
			if (u < 0 || u > 1) return false;

			//prep V param calculation
			const Vector3 qVec{ Vector3::Cross(v0ToOrigin, a) };

			//calculate barycentric V parameter
			const float v{ Vector3::Dot(ray.direction,qVec) * invDeterminant };
			if (v < 0 || u + v > 1) return false;

			const float t{ Vector3::Dot(b, qVec) * invDeterminant };

			if (t < ray.min || t * t > ray.max) return false;



			/*   
			//"Correct side method"
			//flip cullmode for shadows
			const TriangleCullMode cullMode{ ignoreHitRecord == true ?  static_cast<TriangleCullMode>((static_cast<int>(triangle.cullMode) + 1) % 2) : triangle.cullMode };

			const float rayDotNormal{ Vector3::Dot(ray.direction, triangle.normal) };
			const bool isFrontView{ rayDotNormal < 0 };

			//culling
			if (rayDotNormal > -0.01f && rayDotNormal < .01f ) return false; //perpendicular to viewray
			if (cullMode == TriangleCullMode::FrontFaceCulling && isFrontView) return false;
			if (cullMode == TriangleCullMode::BackFaceCulling && !isFrontView) return false;

			//find hit point
			const Vector3 center{ (triangle.v0 + triangle.v1 + triangle.v2) / 3.0f };
			const Vector3 L{ center - ray.origin };

			const float t{ Vector3::Dot(L, triangle.normal) / rayDotNormal };
			if (t < ray.min || t > ray.max) return false;

			const Vector3 hitPoint{ ray.origin + t * ray.direction };

			//check if hit point is outside triangle
			const Vector3 edges[3] { a, 
				triangle.v2 - triangle.v1, 
				triangle.v0 - triangle.v2 };
			const Vector3 vertices[3] { triangle.v0, triangle.v1, triangle.v2 };
			Vector3 pointToSide{};
			for (int i = 0; i < 3; ++i)
			{
				pointToSide =  hitPoint - vertices[i] ;
				if (Vector3::Dot(triangle.normal, Vector3::Cross(edges[i], pointToSide)) < 0) return false;
			}
			 */

			/*//ANALYTICAL CHECK IF HITPOINT IS IN TRIANGLE https://www.youtube.com/watch?v=HYAgJN3x4GA, slower than our method
			const float Cy_Ay{ triangle.v2.y - triangle.v0.y };
			const float Cx_Ax{ triangle.v2.x - triangle.v0.x };

			const float w1{
				(triangle.v0.x * Cy_Ay + (hitPoint.y - triangle.v0.y) * Cx_Ax - hitPoint.x * Cy_Ay)
				/ ((triangle.v1.y - triangle.v0.y) * Cx_Ax - (triangle.v1.x - triangle.v0.x) * Cy_Ay)
			};
			const float w2{ (hitPoint.y - triangle.v0.y - w1 * (triangle.v1.y - triangle.v0.y)) / Cy_Ay };

			if (!(w1 >= 0 && w2 >= 0 && w1 + w2 < 1)) return false;*/


			if (ignoreHitRecord) return true;

			hitRecord.didHit = true;
			hitRecord.materialIndex = triangle.materialIndex;
			hitRecord.normal = triangle.normal;
			hitRecord.origin = ray.origin + ray.direction * t;
			hitRecord.t = t;

			return true;
		}

		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Triangle(triangle, ray, temp, true);
		}
#pragma endregion
#pragma region TriangeMesh HitTest

		inline bool SlabTest_TriangleMesh(const Vector3& minAABB, const Vector3& maxAABB, const Ray& ray)
		{
			const Vector3 originToMinAABB{ (minAABB - ray.origin) };
			const Vector3 originToMaxAABB{ (maxAABB - ray.origin) };
			const float tx1{ originToMinAABB.x * ray.reciproke.x };
			const float tx2{ originToMaxAABB.x * ray.reciproke.x };

			float tmin = std::min(tx1, tx2);
			float tmax = std::max(tx1, tx2);

			const float ty1{  originToMinAABB.y * ray.reciproke.y };
			const float ty2{  originToMaxAABB.y * ray.reciproke.y };

			tmin = std::max(tmin, std::min(ty1, ty2));
			tmax = std::min(tmax, std::max(ty1, ty2));

			const float tz1{  originToMinAABB.z * ray.reciproke.z };
			const float tz2{  originToMaxAABB.z * ray.reciproke.z };

			tmin = std::max(tmin, std::min(tz1, tz2));
			tmax = std::min(tmax, std::max(tz1, tz2));

			return tmax > 0 && tmax >= tmin;
		}

		inline bool IntersectBVH(const TriangleMesh& mesh,const Ray& ray, const uint32_t nodeIdx, HitRecord& hitRecord, bool ignoreHitRecord = false) {

			BVHNode node = mesh.bvhNodes[nodeIdx];

			HitRecord tmp{};

			if (!GeometryUtils::SlabTest_TriangleMesh(node.bounds.minAABB, node.bounds.maxAABB, ray)) 
				return false;

			if (node.isLeaf()) {
				for (uint32_t i = 0; i < node.primCount; ++i)
				{
					if(!GeometryUtils::HitTest_Triangle(*mesh.tris[node.firstPrim + i], ray, tmp, ignoreHitRecord))
						return false;

					if (ignoreHitRecord) return true;
					
					if (tmp.t < hitRecord.t)
					{
						hitRecord = tmp;
					}
					return hitRecord.didHit;
				}
			}
			else
			{
				IntersectBVH(mesh, ray, node.leftChild, hitRecord, ignoreHitRecord);
				IntersectBVH(mesh, ray, node.leftChild + 1, hitRecord, ignoreHitRecord);
			}

		}

		inline bool IntersectBVH(const TriangleMesh& mesh, const Ray& ray, const uint32_t nodeIdx) {
			HitRecord tmp{};
			return IntersectBVH(mesh, ray, nodeIdx,tmp, true);
		}

		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			if (!SlabTest_TriangleMesh(mesh.transformedMinAABB, mesh.transformedMaxAABB, ray))
				return false;
			
			Triangle t{};
			HitRecord tmp{};
			int normalIdx{ 0 };

			for (uint32_t i = 0; i < mesh.indices.size(); i += 3)
			{
				const uint32_t v0 = mesh.indices[i];
				const uint32_t v1 = mesh.indices[i + 1];
				const uint32_t v2 = mesh.indices[i + 2];
				
				//why doesn't this work
				/*t.v0 = mesh.transformedPositions[v0];
				t.v1 = mesh.transformedPositions[v1];
				t.v2 = mesh.transformedPositions[v2];
				t.normal = mesh.transformedNormals[normalIdx++];*/ 

				t = { //but this does
					mesh.transformedPositions[v0],
					mesh.transformedPositions[v1],
					mesh.transformedPositions[v2],
					mesh.transformedNormals[normalIdx++]
				};

				t.cullMode = mesh.cullMode;
				t.materialIndex = mesh.materialIndex;


				if (GeometryUtils::HitTest_Triangle(t, ray, tmp, ignoreHitRecord))
				{
					if (ignoreHitRecord) return true;

					if (tmp.t < hitRecord.t)
					{
						hitRecord = tmp;
					}
				}
			}

			return hitRecord.didHit;
		}

		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_TriangleMesh(mesh, ray, temp, true);
		}

#pragma endregion
	}

	namespace LightUtils
	{
		//Direction from target to light
		inline Vector3 GetDirectionToLight(const Light& light, const Vector3 origin)
		{
			return light.origin - origin;
		}

		inline ColorRGB GetRadiance(const Light& light, const Vector3& target)
		{
			ColorRGB c{};

			const float radiantIntensity{ light.intensity * PI_4 };
			const float irradiance { radiantIntensity / (PI_4 * (light.origin - target).SqrMagnitude()) };

			return light.color * irradiance;
		}
	}

	namespace Utils
	{
		//Just parses vertices and indices
#pragma warning(push)
#pragma warning(disable : 4505) //Warning unreferenced local function
		static bool ParseOBJ(const std::string& filename, std::vector<Vector3>& positions, std::vector<Vector3>& normals, std::vector<int>& indices)
		{
			std::ifstream file(filename);
			if (!file)
				return false;

			std::string sCommand;
			// start a while iteration ending when the end of file is reached (ios::eof)
			while (!file.eof())
			{
				//read the first word of the string, use the >> operator (istream::operator>>) 
				file >> sCommand;
				//use conditional statements to process the different commands	
				if (sCommand == "#")
				{
					// Ignore Comment
				}
				else if (sCommand == "v")
				{
					//Vertex
					float x, y, z;
					file >> x >> y >> z;
					positions.push_back({ x, y, z });
				}
				else if (sCommand == "f")
				{
					float i0, i1, i2;
					file >> i0 >> i1 >> i2;

					indices.push_back((int)i0 - 1);
					indices.push_back((int)i1 - 1);
					indices.push_back((int)i2 - 1);
				}
				//read till end of line and ignore all remaining chars
				file.ignore(1000, '\n');

				if (file.eof()) 
					break;
			}

			//Precompute normals
			for (uint64_t index = 0; index < indices.size(); index += 3)
			{
				uint32_t i0 = indices[index];
				uint32_t i1 = indices[index + 1];
				uint32_t i2 = indices[index + 2];

				Vector3 edgeV0V1 = positions[i1] - positions[i0];
				Vector3 edgeV0V2 = positions[i2] - positions[i0];
				Vector3 normal = Vector3::Cross(edgeV0V1, edgeV0V2);

				if(isnan(normal.x))
				{
					int k = 0;
				}

				normal.Normalize();
				if (isnan(normal.x))
				{
					int k = 0;
				}

				normals.push_back(normal);
			}

			return true;
		}
#pragma warning(pop)
	}
}