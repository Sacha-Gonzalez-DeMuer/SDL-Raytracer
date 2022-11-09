#pragma once

#include "DataTypes.h"

namespace dae {

	class BVH
	{
	public:
		BVH(TriangleMesh& mesh);

		void Update();
		void Intersect(const Ray& ray, const uint32_t nodeIdx, HitRecord& hitRecord, bool ignoreHitRecord = false);
		bool IntersectAABB(const Vector3& bmin, const Vector3 bmax, const Ray& ray);
		uint32_t GetRootNodeIdx() const { return m_RootNodeIdx; };
	private:

		void BuildBVH();
		void GenerateTriangles(const TriangleMesh& mesh);
		void UpdateNodeBounds(const uint32_t nodeIdx);
		void Subdivide(const uint32_t nodeIdx);


		BVHNode* m_BvhNodes{};
		uint32_t m_RootNodeIdx{ 0 };
		uint32_t m_NodesUsed{ 1 };


		Triangle* m_Tris{};
		uint32_t* m_TriIdx{};

		uint32_t m_NTris;

	};


}

