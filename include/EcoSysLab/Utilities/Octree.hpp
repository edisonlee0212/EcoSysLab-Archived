#pragma once

#include "ecosyslab_export.h"

using namespace UniEngine;
namespace EcoSysLab
{
	class OctreeNode
	{
	public:
		float m_radius = 0.0f;
		unsigned m_level = 0;
		glm::vec3 m_center = glm::vec3(0.0f);
		int m_children[8] = { -1 , -1, -1, -1, -1, -1, -1, -1};
		/*
		int m_leftUpBack = -1;
		int m_leftUpFront = -1;
		int m_leftDownBack = -1;
		int m_leftDownFront = -1;
		int m_rightUpBack = -1;
		int m_rightUpFront = -1;
		int m_rightDownBack = -1;
		int m_rightDownFront = -1;
		*/
	};

	class Octree
	{
		std::vector<OctreeNode> m_octreeNodes;
		int NewNode(float radius, unsigned level, const glm::vec3 &center);
		float m_fieldRadius = 16;
		unsigned m_maxSubdivisionLevel = 10;
		float m_minRadius = 0.015625f;
		glm::vec3 m_center;
	public:
		Octree();
		[[nodiscard]] float GetMinRadius() const;
		Octree(float radius, unsigned maxSubdivisionLevel, const glm::vec3& center);
		void IterateLeaves(const std::function<void(const OctreeNode& octreeNode)>& func) const;
		[[nodiscard]] bool Occupied(const glm::vec3& position) const;
		void Reset(float radius, unsigned maxSubdivisionLevel, const glm::vec3& center);
		[[nodiscard]] int GetIndex(const glm::vec3& position) const;
		const OctreeNode& RefNode(int index) const;
		void Occupy(const glm::vec3& position);
		void Occupy(const glm::vec3& position, const glm::quat& rotation, float length, float radius);
		void GetVoxels(std::vector<glm::mat4>& voxels) const;

		void TriangulateField(std::vector<Vertex>& vertices, std::vector<unsigned>& indices, bool removeDuplicate, int smoothMeshIteration) const;
	};
}