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

           /* if (t < ray.min)
            {
                t = (-B + sqrtDiscriminant) / (2 * A);
            }*/

			if (t < ray.min || t > ray.max) return false;


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
			const float t{ Vector3::Dot((plane.origin - ray.origin), plane.normal) / Vector3::Dot(ray.direction, plane.normal) };

			if (t < ray.min || t > ray.max) return false;
			
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
			const Vector3 n{ Vector3::Cross(a, b) };


			//TODO: optimize, switch/cycle(modulo) cullmode without if statements
			TriangleCullMode cullMode{ triangle.cullMode };
			if (ignoreHitRecord) {
				int cullModeInt{ (int)cullMode };
				++cullModeInt;
				cullModeInt = cullModeInt % 2;
				cullMode = (TriangleCullMode)cullModeInt;

				//if (cullMode == TriangleCullMode::BackFaceCulling)
				//{
				//	cullMode = TriangleCullMode::FrontFaceCulling;
				//}
				//else if (cullMode == TriangleCullMode::FrontFaceCulling) {
				//	cullMode = TriangleCullMode::BackFaceCulling;
				//}
			}

			const float rayDotNormal{ Vector3::Dot(ray.direction, n) };
			//culling
			if (rayDotNormal == 0) return false; //perpendicular to viewray
			if (cullMode == TriangleCullMode::FrontFaceCulling && rayDotNormal < 0) return false;
			if (cullMode == TriangleCullMode::BackFaceCulling && rayDotNormal > 0) return false;

			//find hit point
			const Vector3 center{ (triangle.v0 + triangle.v1 + triangle.v2) / 3 };
			const Vector3 L{ center - ray.origin };

			const float t{ Vector3::Dot(L, n) / rayDotNormal };
			if (t < ray.min || t > ray.max) return false;


			const Vector3 hitPoint{ ray.origin + t * ray.direction };


			//check if hit point is outside triangle
			const Vector3 edges[3] { a, 
				triangle.v2 - triangle.v1, 
				triangle.v0 - triangle.v2 };
			const Vector3 vertices[3] { triangle.v0, triangle.v1, triangle.v2 };
			for (int i = 0; i < 3; ++i)
			{
				Vector3 pointToSide = hitPoint - vertices[i];
				if (Vector3::Dot(n, Vector3::Cross(edges[i], pointToSide)) < 0) return false;
			}

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
		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{

			Triangle t{};
			HitRecord tmp{};
			int normalIdx{ 0 };

			for (uint32_t i = 0; i < mesh.indices.size(); i += 3)
			{
				/*t.v0 = mesh.transformedPositions[ mesh.indices[i] ];
				t.v1 = mesh.transformedPositions[ mesh.indices[i + 1] ];
				t.v2 = mesh.transformedPositions[ mesh.indices[i + 2] ];
				t.normal = mesh.transformedNormals[normalIdx++];*/

				uint32_t v0 = mesh.indices[i];
				uint32_t v1 = mesh.indices[i + 1];
				uint32_t v2 = mesh.indices[i + 2];


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
					else
					{
						if (tmp.t < hitRecord.t)
						{
							hitRecord = tmp;
						}
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
			//todo W3
			return light.origin - origin;
		}

		inline ColorRGB GetRadiance(const Light& light, const Vector3& target)
		{
			ColorRGB c{};
			//todo W3

			float radiantIntensity = light.intensity * (4 * PI);
			float irradiance = radiantIntensity / (4 * PI * (light.origin - target).SqrMagnitude());

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