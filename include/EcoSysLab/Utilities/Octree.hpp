#pragma once

#include "ecosyslab_export.h"

using namespace UniEngine;
namespace EcoSysLab
{
	class OctreeNode
	{
	public:
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
		int NewNode();
		
		void IterateOccupied(const std::function<void(const glm::vec3& position, float radius)> &func, const glm::vec3 &center, int subdivision, float voxelRadius, int nodeIndex) const;
	public:
		Octree();
		glm::vec3 m_center;
		float m_radius = 16;
		int m_maxSubdivisionLevel = 10;
		[[nodiscard]] bool Occupied(const glm::vec3& position) const;
		void Reset();
		int GetIndex(const glm::vec3& position) const;
		OctreeNode& RefNode(int index);
		void Occupy(const glm::vec3& position);
		void Occupy(const glm::vec3& position, const glm::quat& rotation, float length, float radius);
		void GetVoxels(std::vector<glm::mat4>& voxels) const;

		void TriangulateField(std::vector<Vertex>& vertices, std::vector<unsigned>& indices, bool removeDuplicate) const;
	};
}