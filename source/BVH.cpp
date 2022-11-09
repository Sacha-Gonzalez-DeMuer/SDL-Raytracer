#include "BVH.h"
#include "Utils.h"
#include <iostream>

dae::BVH::BVH(dae::TriangleMesh& mesh)
	: m_NTris{ static_cast<uint32_t>(mesh.normals.size()) }
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
	//UpdateTransformedTriangles();
}

void dae::BVH::Intersect(const Ray& ray, const uint32_t nodeIdx, HitRecord& hitRecord, bool ignoreHitRecord)
{

	//hitRecord.didHit = false;
	BVHNode& node = m_BvhNodes[nodeIdx];
	HitRecord tmp{};

	if (!IntersectAABB(node.bounds.minAABB, node.bounds.maxAABB, ray)) return;

	if (node.isLeaf()) {

		for (uint32_t i = 0; i < node.triCount; ++i)
		{
			if (GeometryUtils::HitTest_Triangle(m_Tris[m_TriIdx[node.firstTriIdx + i]], ray, tmp, ignoreHitRecord))
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
		Intersect(ray, node.leftNode, hitRecord, ignoreHitRecord);
		Intersect(ray, node.leftNode + 1, hitRecord, ignoreHitRecord);
	}
}

bool dae::BVH::IntersectAABB(const Vector3& bmin, const Vector3 bmax, const Ray& ray)
{
	float tx1 = (bmin.x - ray.origin.x) / ray.direction.x, tx2 = (bmax.x - ray.origin.x) / ray.direction.x;
	float tmin = std::min(tx1, tx2), tmax = std::max(tx1, tx2);
	float ty1 = (bmin.y - ray.origin.y) / ray.direction.y, ty2 = (bmax.y - ray.origin.y) / ray.direction.y;
	tmin = std::max(tmin, std::min(ty1, ty2)), tmax = std::min(tmax, std::max(ty1, ty2));
	float tz1 = (bmin.z - ray.origin.z) / ray.direction.z, tz2 = (bmax.z - ray.origin.z) / ray.direction.z;
	tmin = std::max(tmin, std::min(tz1, tz2)), tmax = std::min(tmax, std::max(tz1, tz2));
	return (tmax >= tmin && tmax > 0);
}

void dae::BVH::BuildBVH()
{
	BVHNode& root = m_BvhNodes[m_RootNodeIdx];
	root.leftNode = 0;
	root.firstTriIdx = 0;
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
			mesh.positions[v0],
			mesh.positions[v1],
			mesh.positions[v2],
			mesh.normals[normalIdx++]
		};

		t.cullMode = mesh.cullMode;
		t.materialIndex = mesh.materialIndex;

		t.centroid = (t.v0 + t.v1 + t.v2) * .333f;
		
		m_Tris[triIdx] = t;
		++triIdx;
	}
}

void dae::BVH::UpdateNodeBounds(const uint32_t nodeIdx)
{
	BVHNode& node = m_BvhNodes[nodeIdx];

	node.bounds.minAABB = { FLT_MAX, FLT_MAX, FLT_MAX };
	node.bounds.maxAABB = { FLT_MIN, FLT_MIN, FLT_MIN };

	for (uint32_t first = node.firstTriIdx, i = 0; i < node.triCount; ++i) {

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
	const Vector3 extent{ node.bounds.maxAABB - node.bounds.minAABB };
	int axis{ 0 };
	if (extent.y > extent.x) axis = 1;
	if (extent.z > extent[axis]) axis = 2;
	const float splitPos{ node.bounds.minAABB[axis] + extent[axis] * .5f };

	//split the group in two halves
	int i = node.firstTriIdx;
	int j = i + node.triCount - 1;
	while (i <= j) {
		if (m_Tris[m_TriIdx[i]].centroid[axis] < splitPos)
			++i;
		else
			std::swap(m_TriIdx[i], m_TriIdx[j--]); //swap each primitive that is not on the left of the plane with a primitive at the end of the list.
	}

	//abort split if one of the sides is empty
	int leftCount = i - node.firstTriIdx;
	if (leftCount == 0 || leftCount == node.triCount) return; 

	//create child nodes for each half
	int leftChildIdx = m_NodesUsed++;
	int rightChildIdx = m_NodesUsed++;

	node.leftNode = leftChildIdx;
	m_BvhNodes[leftChildIdx].firstTriIdx = node.firstTriIdx;
	m_BvhNodes[leftChildIdx].triCount = leftCount;
	m_BvhNodes[rightChildIdx].firstTriIdx = i;
	m_BvhNodes[rightChildIdx].triCount = node.triCount - leftCount;
	node.triCount = 0;

	UpdateNodeBounds(leftChildIdx);
	UpdateNodeBounds(rightChildIdx);

	Subdivide(leftChildIdx);
	Subdivide(rightChildIdx);
}



