#pragma once
#include <cassert>
#include <iostream>
#include "Math.h"
#include "vector"

namespace dae
{
#pragma region GEOMETRY

	struct Sphere
	{
		Vector3 origin{};
		float radius{};

		unsigned char materialIndex{ 0 };
	};

	struct Plane
	{
		Vector3 origin{};
		Vector3 normal{};

		unsigned char materialIndex{ 0 };
	};

	enum class TriangleCullMode
	{
		FrontFaceCulling,
		BackFaceCulling,
		NoCulling
	};

	enum class Axis 
	{
		X,
		Y,
		Z
	};

	struct Triangle
	{
		Triangle() = default;
		Triangle(const Vector3& _v0, const Vector3& _v1, const Vector3& _v2, const Vector3& _normal):
			v0{_v0}, v1{_v1}, v2{_v2}, normal{_normal.Normalized()}{}
		Triangle(const Vector3& _v0, const Vector3& _v1, const Vector3& _v2) :
			v0{ _v0 }, v1{ _v1 }, v2{ _v2 }
		{
			const Vector3 edgeV0V1 = v1 - v0;
			const Vector3 edgeV0V2 = v2 - v0;
			normal = Vector3::Cross(edgeV0V1, edgeV0V2).Normalized();
		}
		Triangle(const Vector3& _v0, const Vector3& _v1, const Vector3& _v2, const Vector3& _normal, const TriangleCullMode _cullMode, const unsigned char _materialIndex, const Vector3& _centroid) :
			v0{ _v0 }, v1{ _v1 }, v2{ _v2 }, normal{ _normal.Normalized() }, cullMode{ _cullMode }, materialIndex{ _materialIndex }, centroid{ _centroid }{}
		


		Vector3 v0{};
		Vector3 v1{};
		Vector3 v2{};

		Vector3 normal{};
		Vector3 centroid{};

		TriangleCullMode cullMode{};
		unsigned char materialIndex{};
	};

	struct AABB 
	{
		Vector3 minAABB;
		Vector3 maxAABB;
		void Grow(const Vector3& to) 
		{
			minAABB = Vector3::Min(minAABB, to);
			maxAABB = Vector3::Max(maxAABB, to);
		}
		float area()
		{
			Vector3 extent = maxAABB - minAABB;
			return extent.x * extent.y + extent.y * extent.z + extent.z * extent.x;
		}
	};

	struct BVHNode 
	{
		AABB bounds{};
		uint32_t leftFirst{}, triCount{};
		bool isLeaf() { return triCount > 0; };
	};

	struct TriangleMesh
	{
		TriangleMesh() = default;
		TriangleMesh(const std::vector<Vector3>& _positions, const std::vector<int>& _indices, TriangleCullMode _cullMode):
		positions(_positions), indices(_indices), cullMode(_cullMode)
		{
			//Calculate Normals
			CalculateNormals();

			//Update Transforms
			UpdateTransforms();
		}

		TriangleMesh(const std::vector<Vector3>& _positions, const std::vector<int>& _indices, const std::vector<Vector3>& _normals, TriangleCullMode _cullMode) :
			positions(_positions), indices(_indices), normals(_normals), cullMode(_cullMode)
		{
			UpdateTransforms();
		}

		std::vector<Vector3> positions{};
		std::vector<Vector3> normals{};
		std::vector<int> indices{};
		unsigned char materialIndex{};

		TriangleCullMode cullMode{TriangleCullMode::BackFaceCulling};

		Matrix rotationTransform{};
		Matrix translationTransform{};
		Matrix scaleTransform{};

		Vector3 minAABB;
		Vector3 maxAABB;

		Vector3 transformedMinAABB;
		Vector3 transformedMaxAABB;

		std::vector<Vector3> transformedPositions{};
		std::vector<Vector3> transformedNormals{};

		/*
		void GenerateBoundaryVolumeHierarchy()
		{
			

			//make root node to kick off algorithm
			GenerateAABB(BVHRoot.bounds, positions);
			BVHRoot.positions = positions;

			//SplitToMinimumChildren(BVHRoot);
		}

		void SplitAABBNode(BVHNode& currentNode) {

			std::vector<Vector3> child1Positions;
			std::vector<Vector3> child2Positions;

			BVHNode tmpChild1;
			BVHNode tmpChild2;

			const float xLength{ currentNode.bounds.maxAABB.x - currentNode.bounds.minAABB.x };
			const float yLength{ currentNode.bounds.maxAABB.y - currentNode.bounds.minAABB.y };
			const float zLength{ currentNode.bounds.maxAABB.z - currentNode.bounds.minAABB.z };

			Vector3 extents{ currentNode.bounds.maxAABB - currentNode.bounds.minAABB };

			//find longest axis
			Axis longestAxis{Axis::X};
			if (extents.y > extents.x)
				longestAxis = Axis::Y;
			if (extents.z > extents.z || extents.z > extents.x)
				longestAxis = Axis::Z;

			float axisCenter{};

			for (size_t i = 0; i < currentNode.positions.size(); ++i)
			{
				Vector3 currentPos{ currentNode.positions[i] };

				switch (longestAxis) 
				{
				case Axis::X:
					axisCenter = extents.x / 2;

					if (currentPos.x < axisCenter)
						child1Positions.push_back(currentPos);
					else
						child2Positions.push_back(currentPos);
					break;

				case Axis::Y:
					axisCenter = extents.y / 2;

					if (currentPos.y < axisCenter)
						child1Positions.push_back(currentPos);
					else
						child2Positions.push_back(currentPos);
					break;

				case Axis::Z:
					axisCenter = extents.z / 2;

					if (currentPos.z < axisCenter)
						child1Positions.push_back(currentPos);
					else
						child2Positions.push_back(currentPos);
					break;
				}
			}

			tmpChild1.positions = child1Positions;
			tmpChild2.positions = child2Positions;
			
			GenerateAABB(tmpChild1.bounds, tmpChild1.positions);
			GenerateAABB(tmpChild2.bounds, tmpChild2.positions);

			currentNode.children.push_back(tmpChild1);
			currentNode.children.push_back(tmpChild2);
		}

		void SplitToMinimumChildren(BVHNode& root) {
			
			bool isRootMinimized{ true };

			if (root.positions.size() > 2)
			{
				isRootMinimized = false;
			}

			if (isRootMinimized)
			{
				return;
			}
			else
			{
				SplitAABBNode(root); //split node, creates children

				for (size_t i = 0; i < root.children.size(); i++)
				{
					SplitToMinimumChildren(root.children[i]);
				}
			}
		}
		 bool SplitAABBNode(AABBNode & node) {

			if (node.positions.size() <= 4)
			{
				return true;
			}


			const float xLength{ node.box.maxAABB.x - node.box.minAABB.x };
			const float yLength{ node.box.maxAABB.y - node.box.minAABB.y };
			const float zLength{ node.box.maxAABB.z - node.box.minAABB.z };

			//minimum box size
			if (xLength < 1 || yLength < 1 || zLength < 1)
			{
				return true;
			}

			//find biggest length
			Axis longestAxis{};
			if (xLength > yLength && xLength > zLength)
				longestAxis = Axis::X;
			else if (yLength > xLength && yLength > zLength)
				longestAxis = Axis::Y;
			else if (zLength > xLength && zLength > yLength) 
				longestAxis = Axis::Z;



			float axisCenter{};
			node.split1 = new AABBNode();
			node.split2 = new AABBNode();

			for (size_t i = 0; i < node.positions.size(); ++i)
			{
				switch (longestAxis) {
				case Axis::X:
					axisCenter = xLength / 2 + node.box.minAABB.x;

					if (node.positions[i].x < axisCenter)
					{
						node.split1->positions.push_back(node.positions[i]);
					}
					else
					{
						node.split2->positions.push_back(node.positions[i]);
					}
					
					break;

				case Axis::Y:
					axisCenter = yLength / 2 + node.box.minAABB.y;

					if (node.positions[i].y < axisCenter)
					{
						node.split1->positions.push_back(node.positions[i]);
					}
					else
					{
						node.split2->positions.push_back(node.positions[i]);
					}
					break;

				case Axis::Z:
					axisCenter = zLength / 2 + node.box.minAABB.z;

					if (node.positions[i].z < axisCenter)
					{
						node.split1->positions.push_back(node.positions[i]);
					}
					else
					{
						node.split2->positions.push_back(node.positions[i]);
					}
					break;
				}
			}

			GenerateAABB(node.split1->box, node.split1->positions);
			GenerateAABB(node.split2->box, node.split2->positions);

			SplitAABBNode(*node.split1);
			SplitAABBNode(*node.split2);

			return false;
		}
		*/

		void GenerateAABB(AABB& aabb, std::vector<Vector3> positions) 
		{
			if (positions.size() > 0)
			{
				aabb.minAABB = positions[0];
				aabb.maxAABB = positions[0];

				for (auto& p : positions)
				{
					aabb.minAABB = Vector3::Min(p, minAABB);
					aabb.maxAABB = Vector3::Max(p, maxAABB);
				}
			}
		}

		void Translate(const Vector3& translation)
		{
			translationTransform = Matrix::CreateTranslation(translation);
		}

		void RotateY(float yaw)
		{
			rotationTransform = Matrix::CreateRotationY(yaw);
		}

		void Scale(const Vector3& scale)
		{
			scaleTransform = Matrix::CreateScale(scale);
		}

		void AppendTriangle(const Triangle& triangle, bool ignoreTransformUpdate = false)
		{
			int startIndex = static_cast<int>(positions.size());

			positions.push_back(triangle.v0);
			positions.push_back(triangle.v1);
			positions.push_back(triangle.v2);

			indices.push_back(startIndex);
			indices.push_back(++startIndex);
			indices.push_back(++startIndex);

			normals.push_back(triangle.normal);

			//Not ideal, but making sure all vertices are updated
			if(!ignoreTransformUpdate)
				UpdateTransforms();
		}

		void CalculateNormals()
		{
			//find vertrex triplets

			for (int i = 0; i < indices.size(); i += 3)
			{
				uint32_t v0 = indices[i];
				uint32_t v1 = indices[i+1];
				uint32_t v2 = indices[i+2];


				const Vector3 edge1{ positions[v1] - positions[v0]};
				const Vector3 edge2{ positions[v2] - positions[v0]};
				normals.emplace_back(Vector3::Cross(edge1, edge2));
			}

			
		}

		void UpdateTransforms()
		{
			//Calculate Final Transform 
			const auto finalTransform = scaleTransform * rotationTransform * translationTransform;

			transformedPositions.clear();
			transformedNormals.clear();
			transformedPositions.reserve(positions.size());
			transformedNormals.reserve(normals.size());

			//Transform Positions (positions > transformedPositions)
			//...
			for (unsigned int i = 0; i < positions.size(); ++i)
			{
				transformedPositions.emplace_back(finalTransform.TransformPoint(positions[i]));
			}


			//Transform Normals (normals > transformedNormals)
			//...
			for (unsigned int i = 0; i < normals.size(); ++i)
			{
				transformedNormals.emplace_back(finalTransform.TransformVector(normals[i]));
			}

			UpdateTransformedAABB(finalTransform);
		}

		void UpdateAABB()
		{
			if (positions.size()> 0)
			{
				minAABB = positions[0];
				maxAABB = positions[0];

				for(auto& p : positions)
				{
					minAABB = Vector3::Min(p, minAABB);
					maxAABB = Vector3::Max(p, maxAABB);
				}
			}
		}

		void UpdateTransformedAABB(const Matrix& finalTransform)
		{
			//AABB update: be careful -> trandform the 8 vertices of the aabb and calculate new min and max
			Vector3 tMinAABB = finalTransform.TransformPoint(minAABB);
			Vector3 tMaxAABB = tMinAABB;

			//(xmax, tmin, zmin)
			Vector3 tAABB = finalTransform.TransformPoint(maxAABB.x, minAABB.y, minAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);

			//xmax, ymin, zma
			tAABB = finalTransform.TransformPoint(maxAABB.x, minAABB.y, maxAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);

			//xmin, ymin, zmax
			tAABB = finalTransform.TransformPoint(minAABB.x, minAABB.y, maxAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);


			//xmin, ymax, zmin
			tAABB = finalTransform.TransformPoint(minAABB.x, maxAABB.y, minAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			

			//xmax ymax,zmin
			tAABB = finalTransform.TransformPoint(maxAABB.x, maxAABB.y, minAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);

			//max max max
			tAABB = finalTransform.TransformPoint(maxAABB);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);

			//xmin ymax zmax
			tAABB = finalTransform.TransformPoint(minAABB.x, maxAABB.y, minAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);

			transformedMinAABB = tMinAABB;
			transformedMaxAABB = tMaxAABB;
		}
	};
#pragma endregion
#pragma region LIGHT
	enum class LightType
	{
		Point,
		Directional
	};

	struct Light
	{
		Vector3 origin{};
		Vector3 direction{};
		ColorRGB color{};
		float intensity{};

		LightType type{};
	};
#pragma endregion
#pragma region MISC
	struct Ray
	{
		Vector3 origin{};
		Vector3 direction{};
		Vector3 reciproke{};

		float min{ 0.0001f };
		float max{ FLT_MAX };
	};

	struct HitRecord
	{
		Vector3 origin{};
		Vector3 normal{};
		float t = FLT_MAX;

		bool didHit{ false };
		unsigned char materialIndex{ 0 };
	};
#pragma endregion
}