#pragma once

#include "DataTypes.h"

namespace dae {

	class BVH
	{
	public:
		BVH(TriangleMesh& mesh);

		void Update();

		void Intersect(const Ray& ray, const uint32_t nodeIdx, HitRecord& hitRecord, bool ignoreHitRecord = false);
		void IntersectBVH(const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false);
		float IntersectAABB(const Vector3& bmin, const Vector3 bmax, const Ray& ray);
		uint32_t GetRootNodeIdx() const { return m_RootNodeIdx; };
	private:
		void BuildBVH();
		void GenerateTriangles(const TriangleMesh& mesh);
		void UpdateNodeBounds(const uint32_t nodeIdx);
		void Subdivide(const uint32_t nodeIdx);

		float FindBestSplitPlane(BVHNode& node, int& axis, float& splitPos);

		void UpdateTriangles();
		void RefitBVH();

		float EvaluateSAH(BVHNode& node, int axis, float pos);

		BVHNode* m_BvhNodes{};
		uint32_t m_RootNodeIdx{ 0 };
		uint32_t m_NodesUsed{ 1 };

		TriangleMesh& m_Mesh;
		Triangle* m_Tris{};
		uint32_t* m_TriIdx{};

		uint32_t m_NTris;
	};


}

