#include "BVH.h"
#include "Utils.h"
#include <iostream>

dae::BVH::BVH(dae::TriangleMesh& mesh)
	: m_NTris{ static_cast<uint32_t>(mesh.normals.size()) }
	, m_Mesh{mesh}
{
	m_BvhNodes = new BVHNode[m_NTris * 2 - 1];
	m_Tris = new Triangle[m_NTris];
	m_TriIdx = new uint32_t[m_NTris];

	GenerateTriangles(mesh);
	for (uint32_t i = 0; i < m_NTris; ++i) m_TriIdx[i] = i;
	BuildBVH();
}

void dae::BVH::Update()
{
	UpdateTriangles();
	RefitBVH();
}

void dae::BVH::Intersect(const Ray& ray, const uint32_t nodeIdx, HitRecord& hitRecord, bool ignoreHitRecord)
{
	BVHNode& node = m_BvhNodes[nodeIdx];
	HitRecord tmp{};

	if (!GeometryUtils::SlabTest_TriangleMesh(node.bounds.minAABB, node.bounds.maxAABB, ray)) return;

	if (node.isLeaf()) {

		for (uint32_t i = 0; i < node.triCount; ++i)
		{
			if (GeometryUtils::HitTest_Triangle(m_Tris[m_TriIdx[node.leftFirst + i]], ray, tmp, ignoreHitRecord))
			{
				if (ignoreHitRecord) return;

				if (tmp.t < hitRecord.t)
				{
					hitRecord = tmp;
				}
			}
		}
	}
	else
	{
		Intersect(ray, node.leftFirst, hitRecord, ignoreHitRecord);
		Intersect(ray, node.leftFirst + 1, hitRecord, ignoreHitRecord);
	}
}

void dae::BVH::IntersectBVH(const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord)
{
	BVHNode* node = &m_BvhNodes[m_RootNodeIdx], *stack[64];
	uint32_t stackPtr{ 0 };

	HitRecord tmp{};

	//infinite loop completes when trying to pop from an empty stack
	while (1) 
	{
		if (node->isLeaf())
		{
			for (uint32_t i = 0; i < node->triCount; i++)
			{
				if (GeometryUtils::HitTest_Triangle(m_Tris[m_TriIdx[node->leftFirst + i]], ray, tmp, ignoreHitRecord))
				{
					if (ignoreHitRecord) return;

					if (tmp.t < hitRecord.t)
					{
						hitRecord = tmp;
					}
				}
			}


			if (stackPtr == 0) break;
			else node = stack[--stackPtr];
			continue;
		}

		BVHNode* child1 = &m_BvhNodes[node->leftFirst];
		BVHNode* child2 = &m_BvhNodes[node->leftFirst + 1];
		float dist1 = IntersectAABB( child1->bounds.minAABB, child1->bounds.maxAABB, ray);
		float dist2 = IntersectAABB(child2->bounds.minAABB, child2->bounds.maxAABB, ray);

		//using the distances to both child nodes we can sort them
		if (dist1 > dist2)
		{
			std::swap(dist1, dist2);
			std::swap(child1, child2);
		}

		if (dist1 == FLT_MAX)
		{
			if (stackPtr == 0) break;
			else node = stack[--stackPtr];
		}
		else
		{
			node = child1;
			if (dist2 != FLT_MAX) stack[stackPtr++] = child2;
		}
	}
}

float dae::BVH::IntersectAABB(const Vector3& bmin, const Vector3 bmax, const Ray& ray)
{
	float tx1 = (bmin.x - ray.origin.x) * ray.reciproke.x, tx2 = (bmax.x - ray.origin.x) * ray.reciproke.x;
	float tmin = std::min(tx1, tx2), tmax = std::max(tx1, tx2);
	float ty1 = (bmin.y - ray.origin.y) * ray.reciproke.y, ty2 = (bmax.y - ray.origin.y) * ray.reciproke.y;
	tmin = std::max(tmin, std::min(ty1, ty2)), tmax = std::min(tmax, std::max(ty1, ty2));
	float tz1 = (bmin.z - ray.origin.z) * ray.reciproke.z, tz2 = (bmax.z - ray.origin.z) * ray.reciproke.z;
	tmin = std::max(tmin, std::min(tz1, tz2)), tmax = std::min(tmax, std::max(tz1, tz2));
	if ((tmax >= tmin && tmax > 0)) return tmin;
	else return FLT_MAX;
}

void dae::BVH::BuildBVH()
{
	BVHNode& root = m_BvhNodes[m_RootNodeIdx];
	root.leftFirst = 0;
	root.triCount = m_NTris;

	UpdateNodeBounds(m_RootNodeIdx);
	Subdivide(m_RootNodeIdx);
}

void dae::BVH::GenerateTriangles(const TriangleMesh& mesh)
{
	Triangle t{};
	int normalIdx{ 0 };
	int triIdx{ 0 };
	for (uint32_t i = 0; i < mesh.indices.size(); i += 3)
	{
		const uint32_t v0 = mesh.indices[i];
		const uint32_t v1 = mesh.indices[i + 1];
		const uint32_t v2 = mesh.indices[i + 2];

		t = {
			mesh.transformedPositions[v0],
			mesh.transformedPositions[v1],
			mesh.transformedPositions[v2],
			mesh.transformedNormals[normalIdx++]
		};

		t.cullMode = mesh.cullMode;
		t.materialIndex = mesh.materialIndex;

		t.centroid = (t.v0 + t.v1 + t.v2) * .333f;
		
		m_Tris[triIdx] = t;
		++triIdx;
	}
}

void dae::BVH::UpdateTriangles()
{
	int normalIdx{ 0 };
	int triIdx{ 0 };
	for (uint32_t i = 0; i < m_Mesh.indices.size(); i += 3)
	{
		const uint32_t v0 = m_Mesh.indices[i];
		const uint32_t v1 = m_Mesh.indices[i + 1];
		const uint32_t v2 = m_Mesh.indices[i + 2];

		m_Tris[triIdx].v0 =	m_Mesh.transformedPositions[v0];
		m_Tris[triIdx].v1 =	m_Mesh.transformedPositions[v1];
		m_Tris[triIdx].v2 =	m_Mesh.transformedPositions[v2];
		m_Tris[triIdx].normal = m_Mesh.transformedNormals[normalIdx++];
		m_Tris[triIdx].centroid = (m_Tris[triIdx].v0 + m_Tris[triIdx].v1 + m_Tris[triIdx].v2) * .333f;
		++triIdx;
	}
}

void dae::BVH::UpdateNodeBounds(const uint32_t nodeIdx)
{
	BVHNode& node = m_BvhNodes[nodeIdx];

	node.bounds.minAABB = { FLT_MAX, FLT_MAX, FLT_MAX };
	node.bounds.maxAABB = { FLT_MIN, FLT_MIN, FLT_MIN };

	for (uint32_t first = node.leftFirst, i = 0; i < node.triCount; ++i) {

		uint32_t leafTriIdx = m_TriIdx[first + i];
		Triangle& leafTri = m_Tris[leafTriIdx];

		node.bounds.minAABB = Vector3::Min(node.bounds.minAABB, leafTri.v0);
		node.bounds.minAABB = Vector3::Min(node.bounds.minAABB, leafTri.v1);
		node.bounds.minAABB = Vector3::Min(node.bounds.minAABB, leafTri.v2);

		node.bounds.maxAABB = Vector3::Max(node.bounds.maxAABB, leafTri.v0);
		node.bounds.maxAABB = Vector3::Max(node.bounds.maxAABB, leafTri.v1);
		node.bounds.maxAABB = Vector3::Max(node.bounds.maxAABB, leafTri.v2);
	}
}

void dae::BVH::Subdivide(const uint32_t nodeIdx)
{
	BVHNode& node = m_BvhNodes[nodeIdx];
	if (node.triCount <= 2) return;

	//find split plane axis & position 
	//----MIDPOINT SPLIT
	/*const Vector3 extent{node.bounds.maxAABB - node.bounds.minAABB};
	int axis{ 0 };
	if (extent.y > extent.x) axis = 1;
	if (extent.z > extent[axis]) axis = 2;
	const float splitPos{ node.bounds.minAABB[axis] + extent[axis] * .5f };*/


	int axis = -1;
	float splitPos{ 0 };
	float splitCost{ FindBestSplitPlane(node, axis, splitPos) };

	//split the group in two halves
	int i = node.leftFirst;
	int j = i + node.triCount - 1;
	while (i <= j) {
		if (m_Tris[m_TriIdx[i]].centroid[axis] < splitPos)
			++i;
		else
			std::swap(m_TriIdx[i], m_TriIdx[j--]); //swap each primitive that is not on the left of the plane with a primitive at the end of the list.
	}

	//abort split if one of the sides is empty
	int leftCount = i - node.leftFirst;
	if (leftCount == 0 || leftCount == node.triCount) return; 

	//create child nodes for each half
	const uint32_t leftChildIdx{ m_NodesUsed++ };
	const uint32_t rightChildIdx{ m_NodesUsed++ };
	m_BvhNodes[leftChildIdx].leftFirst = node.leftFirst;
	m_BvhNodes[leftChildIdx].triCount = leftCount;
	m_BvhNodes[rightChildIdx].leftFirst = i;
	m_BvhNodes[rightChildIdx].triCount = node.triCount - leftCount;
	node.leftFirst = leftChildIdx;
	node.triCount = 0;

	UpdateNodeBounds(leftChildIdx);
	UpdateNodeBounds(rightChildIdx);

	Subdivide(leftChildIdx);
	Subdivide(rightChildIdx);
}

float dae::BVH::FindBestSplitPlane(BVHNode& node, int& axis, float& splitPos)
{
	float bestCost{ FLT_MAX };
	for (uint32_t a = 0; a < 3; a++) for (uint32_t i = 0; i < node.triCount; ++i)
	{
		Triangle& t = m_Tris[m_TriIdx[node.leftFirst + i]];
		float candidatePos = t.centroid[a];
		float cost = EvaluateSAH(node, a, candidatePos);
		if (cost < bestCost)
		{
			splitPos = candidatePos;
			axis = a;
			bestCost = cost;
		}
	}
	return bestCost;
}

void dae::BVH::RefitBVH()
{
	for (int i = m_NodesUsed - 1; i >= 0; i--) 
	{
		if (i != 1) 
		{
			BVHNode& node = m_BvhNodes[i];

			if (node.isLeaf())
			{
				//adjust leaf node bounds to contained triangles
				UpdateNodeBounds(i);
				continue;
			}

			//adjust interior node to child node bounds
			BVHNode& leftChild = m_BvhNodes[node.leftFirst];
			BVHNode& rightChild = m_BvhNodes[node.leftFirst + 1];
			node.bounds.minAABB = Vector3::Min(leftChild.bounds.minAABB, rightChild.bounds.minAABB);
			node.bounds.maxAABB = Vector3::Max(leftChild.bounds.maxAABB, rightChild.bounds.maxAABB);
		}
	}
}

float dae::BVH::EvaluateSAH(BVHNode& node, int axis, float pos)
{
	AABB leftBox, rightBox;
	int leftCount{ 0 }, rightCount{ 0 };
	 
	for (uint32_t i = 0; i < node.triCount; i++)
	{
		Triangle& t = m_Tris[m_TriIdx[node.leftFirst + i]];
		if (t.centroid[axis] < pos) {
			++leftCount;
			leftBox.Grow(t.v0);
			leftBox.Grow(t.v1);
			leftBox.Grow(t.v2);
		}
		else
		{
			++rightCount;
			rightBox.Grow(t.v0);
			rightBox.Grow(t.v1);
			rightBox.Grow(t.v2);
		}
	}

	const float cost{ leftCount * leftBox.area() + rightCount * rightBox.area() };
	return cost > 0 ? cost : FLT_MAX;
}




