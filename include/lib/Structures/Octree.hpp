#pragma once

#include "glm/gtx/quaternion.hpp"
#include "MarchingCubes.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	typedef int OctreeNodeHandle;
	typedef int OctreeNodeDataHandle;
	class OctreeNode
	{
		float m_radius = 0.0f;
		unsigned m_level = 0;
		glm::vec3 m_center = glm::vec3(0.0f);
		OctreeNodeHandle m_children[8] = { -1, -1, -1, -1, -1, -1, -1, -1 };
		OctreeNodeDataHandle m_dataHandle = -1;
		template<typename ND>
		friend class Octree;
		bool m_recycled = true;
	public:
		[[nodiscard]] float GetRadius() const { return m_radius; }
		[[nodiscard]] unsigned GetLevel() const { return m_level; }
		[[nodiscard]] glm::vec3 GetCenter() const { return m_center; }
		[[nodiscard]] OctreeNodeDataHandle GetNodeDataHandle() const { return m_dataHandle; }
		[[nodiscard]] bool IsRecycled() const { return m_recycled; }
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


	template <typename NodeData>
	class Octree
	{
		std::vector<OctreeNode> m_octreeNodes = {};
		std::queue<size_t> m_nodePool = {};
		std::vector<NodeData> m_nodeData = {};
		std::queue<size_t> m_nodeDataPool = {};
		OctreeNodeHandle Allocate(float radius, unsigned level, const glm::vec3 &center);
		void Recycle(OctreeNodeHandle nodeHandle);
		float m_chunkRadius = 16;
		unsigned m_maxSubdivisionLevel = 10;
		float m_minimumNodeRadius = 0.015625f;
		glm::vec3 m_center;
	public:
		Octree();
		[[nodiscard]] float GetMinRadius() const;
		Octree(float radius, unsigned maxSubdivisionLevel, const glm::vec3& center);
		void IterateLeaves(const std::function<void(const OctreeNode& octreeNode)>& func) const;
		[[nodiscard]] bool Occupied(const glm::vec3& position) const;
		void Reset(float radius, unsigned maxSubdivisionLevel, const glm::vec3& center);
		[[nodiscard]] OctreeNodeHandle GetNodeHandle(const glm::vec3& position) const;
		[[nodiscard]] const OctreeNode& RefNode(OctreeNodeHandle nodeHandle) const;

		//void Expand(OctreeNodeHandle nodeHandle);
		//void Collapse(OctreeNodeHandle nodeHandle);

		void Occupy(const glm::vec3& position, const std::function<void(OctreeNode&)>& occupiedNodes);
		void Occupy(const glm::vec3& position, const glm::quat& rotation, float length, float radius, const std::function<void(OctreeNode&)>& occupiedNodes);
		void Occupy(const glm::vec3& min, const glm::vec3 &max, const std::function<bool(const glm::vec3& boxCenter)>& collisionHandle, const std::function<void(OctreeNode&)>& occupiedNodes);
		[[nodiscard]] NodeData& RefNodeData(OctreeNodeDataHandle nodeDataHandle);
		[[nodiscard]] const NodeData& PeekNodeData(OctreeNodeDataHandle nodeDataHandle) const;
		[[nodiscard]] NodeData& RefNodeData(const OctreeNode& octreeNode);
		[[nodiscard]] const NodeData& PeekNodeData(const OctreeNode& octreeNode) const;
		void GetVoxels(std::vector<glm::mat4>& voxels) const;
		void TriangulateField(std::vector<Vertex>& vertices, std::vector<unsigned>& indices, bool removeDuplicate) const;
	};

	template <typename NodeData>
	OctreeNodeHandle Octree<NodeData>::Allocate(const float radius, const unsigned level, const glm::vec3& center)
	{
		OctreeNodeHandle newNodeHandle;
		if(m_nodePool.empty())
		{
			newNodeHandle = m_octreeNodes.size();
			m_octreeNodes.emplace_back();
		}else
		{
			newNodeHandle = m_nodePool.front();
			m_nodePool.pop();
		}

		auto& node = m_octreeNodes.at(newNodeHandle);
		node.m_radius = radius;
		node.m_level = level;
		node.m_center = center;
		node.m_recycled = false;
		if(m_nodeDataPool.empty())
		{
			node.m_dataHandle = m_nodeData.size();
			m_nodeData.emplace_back();
		}else
		{
			node.m_dataHandle = m_nodeDataPool.front();
			m_nodeDataPool.pop();
		}
		m_nodeData.at(node.m_dataHandle) = {};
		return m_octreeNodes.size() - 1;
	}

	template <typename NodeData>
	void Octree<NodeData>::Recycle(const OctreeNodeHandle nodeHandle)
	{
		m_nodeDataPool.push(nodeHandle);
		auto& node = m_octreeNodes[nodeHandle];
		node.m_radius = 0;
		node.m_level = 0;
		node.m_center = {};
		node.m_recycled = true;

		m_nodeDataPool.push(node.m_dataHandle);
		node.m_dataHandle = -1;

	}

	template <typename NodeData>
	Octree<NodeData>::Octree()
	{
		Reset(16, 10, glm::vec3(0.0f));
	}
	template <typename NodeData>
	float Octree<NodeData>::GetMinRadius() const
	{
		return m_minimumNodeRadius;
	}
	template <typename NodeData>
	Octree<NodeData>::Octree(const float radius, const unsigned maxSubdivisionLevel, const glm::vec3& center)
	{
		Reset(radius, maxSubdivisionLevel, center);
	}


	template <typename NodeData>
	bool Octree<NodeData>::Occupied(const glm::vec3& position) const
	{
		float currentRadius = m_chunkRadius;
		glm::vec3 center = m_center;
		int octreeNodeIndex = 0;
		for (int subdivision = 0; subdivision < m_maxSubdivisionLevel; subdivision++)
		{
			currentRadius /= 2.f;
			const auto& octreeNode = m_octreeNodes[octreeNodeIndex];
			const int index = 4 * (position.x > center.x ? 0 : 1) + 2 * (position.y > center.y ? 0 : 1) + (position.z > center.z ? 0 : 1);
			if (octreeNode.m_children[index] == -1)
			{
				return false;
			}
			octreeNodeIndex = octreeNode.m_children[index];
			center.x += position.x > center.x ? currentRadius : -currentRadius;
			center.y += position.y > center.y ? currentRadius : -currentRadius;
			center.z += position.z > center.z ? currentRadius : -currentRadius;
		}
		return true;
	}
	template <typename NodeData>
	void Octree<NodeData>::Reset(float radius, unsigned maxSubdivisionLevel, const glm::vec3& center)
	{
		m_chunkRadius = m_minimumNodeRadius = radius;
		m_maxSubdivisionLevel = maxSubdivisionLevel;
		m_center = center;
		m_octreeNodes.clear();
		for (int subdivision = 0; subdivision < m_maxSubdivisionLevel; subdivision++)
		{
			m_minimumNodeRadius /= 2.f;
		}
		Allocate(m_chunkRadius, -1, center);
	}
	template <typename NodeData>
	OctreeNodeHandle Octree<NodeData>::GetNodeHandle(const glm::vec3& position) const
	{
		float currentRadius = m_chunkRadius;
		glm::vec3 center = m_center;
		OctreeNodeHandle octreeNodeIndex = 0;
		for (int subdivision = 0; subdivision < m_maxSubdivisionLevel; subdivision++)
		{
			currentRadius /= 2.f;
			const auto& octreeNode = m_octreeNodes[octreeNodeIndex];
			const int index = 4 * (position.x > center.x ? 0 : 1) + 2 * (position.y > center.y ? 0 : 1) + (position.z > center.z ? 0 : 1);
			if (octreeNode.m_children[index] == -1)
			{
				return -1;
			}
			octreeNodeIndex = octreeNode.m_children[index];
			center.x += position.x > center.x ? currentRadius : -currentRadius;
			center.y += position.y > center.y ? currentRadius : -currentRadius;
			center.z += position.z > center.z ? currentRadius : -currentRadius;
		}
		return octreeNodeIndex;
	}
	template <typename NodeData>
	const OctreeNode& Octree<NodeData>::RefNode(const OctreeNodeHandle nodeHandle) const
	{
		return m_octreeNodes[nodeHandle];
	}

	template <typename NodeData>
	void Octree<NodeData>::Occupy(const glm::vec3& position, const std::function<void(OctreeNode&)>& occupiedNodes)
	{
		float currentRadius = m_chunkRadius;
		glm::vec3 center = m_center;
		int octreeNodeIndex = 0;
		for (int subdivision = 0; subdivision < m_maxSubdivisionLevel; subdivision++)
		{
			currentRadius /= 2.f;
			const auto& octreeNode = m_octreeNodes[octreeNodeIndex];
			const int index = 4 * (position.x > center.x ? 0 : 1) + 2 * (position.y > center.y ? 0 : 1) + (position.z > center.z ? 0 : 1);
			center.x += position.x > center.x ? currentRadius : -currentRadius;
			center.y += position.y > center.y ? currentRadius : -currentRadius;
			center.z += position.z > center.z ? currentRadius : -currentRadius;
			if (octreeNode.m_children[index] == -1)
			{
				const auto newIndex = Allocate(currentRadius, subdivision, center);
				m_octreeNodes[octreeNodeIndex].m_children[index] = newIndex;
				octreeNodeIndex = newIndex;
			}
			else octreeNodeIndex = octreeNode.m_children[index];
		}
		occupiedNodes(m_octreeNodes[octreeNodeIndex]);
	}

	template <typename NodeData>
	void Octree<NodeData>::Occupy(const glm::vec3& position, const glm::quat& rotation, float length, float radius, const std::function<void(OctreeNode&)>& occupiedNodes)
	{
		const float maxRadius = glm::max(length, radius);
		Occupy(glm::vec3(position - glm::vec3(maxRadius)), glm::vec3(position + glm::vec3(maxRadius)), [&](const glm::vec3& boxCenter)
			{
				const auto relativePos = glm::rotate(glm::inverse(rotation), boxCenter - position);
				return glm::abs(relativePos.z) <= length && glm::length(glm::vec2(relativePos.x, relativePos.y)) <= radius;
			}, occupiedNodes);
	}

	template <typename NodeData>
	void Octree<NodeData>::Occupy(const glm::vec3& min, const glm::vec3& max,
		const std::function<bool(const glm::vec3& boxCenter)>& collisionHandle,
		const std::function<void(OctreeNode&)>& occupiedNodes)
	{
		for (float x = min.x - m_minimumNodeRadius; x < max.x + m_minimumNodeRadius; x += m_minimumNodeRadius)
		{
			for (float y = min.y - m_minimumNodeRadius; y < max.y + m_minimumNodeRadius; y += m_minimumNodeRadius)
			{
				for (float z = min.z - m_minimumNodeRadius; z < max.z + m_minimumNodeRadius; z += m_minimumNodeRadius)
				{
					if (collisionHandle(glm::vec3(x, y, z)))
					{
						Occupy(glm::vec3(x, y, z), occupiedNodes);
					}
				}
			}
		}
	}

	template <typename NodeData>
	NodeData& Octree<NodeData>::RefNodeData(int nodeDataHandle)
	{
		assert(nodeDataHandle > 0 && nodeDataHandle < m_nodeData.size());
		return m_nodeData[nodeDataHandle];
	}

	template <typename NodeData>
	const NodeData& Octree<NodeData>::PeekNodeData(int nodeDataHandle) const
	{
		assert(nodeDataHandle > 0 && nodeDataHandle < m_nodeData.size());
		return m_nodeData[nodeDataHandle];
	}

	template <typename NodeData>
	NodeData& Octree<NodeData>::RefNodeData(const OctreeNode& octreeNode)
	{
		assert(!octreeNode.m_recycled);
		return m_nodeData[octreeNode.m_dataHandle];
	}

	template <typename NodeData>
	const NodeData& Octree<NodeData>::PeekNodeData(const OctreeNode& octreeNode) const
	{
		assert(!octreeNode.m_recycled);
		return m_nodeData[octreeNode.m_dataHandle];
	}

	template <typename NodeData>
	void Octree<NodeData>::IterateLeaves(const std::function<void(const OctreeNode& octreeNode)>& func) const
	{
		for (const auto& node : m_octreeNodes)
		{
			if (node.m_level == m_maxSubdivisionLevel - 1)
			{
				func(node);
			}
		}
	}
	template <typename NodeData>
	void Octree<NodeData>::GetVoxels(std::vector<glm::mat4>& voxels) const
	{
		voxels.clear();
		IterateLeaves([&](const OctreeNode& octreeNode)
			{
				voxels.push_back(glm::translate(octreeNode.m_center) * glm::scale(glm::vec3(m_minimumNodeRadius)));
			});
	}
	template <typename NodeData>
	void Octree<NodeData>::TriangulateField(std::vector<Vertex>& vertices, std::vector<unsigned>& indices, const bool removeDuplicate) const
	{
		std::vector<TestingCell> testingCells;
		IterateLeaves([&](const OctreeNode& octreeNode)
			{
				TestingCell testingCell{};
				testingCell.m_position = octreeNode.m_center;
				testingCells.push_back(testingCell);
			});

		MarchingCubes::TriangulateField(m_center, [&](const glm::vec3& samplePoint)
			{
				return Occupied(samplePoint) ? 1.0f : 0.0f;
			}, 0.5f, m_minimumNodeRadius, testingCells, vertices, indices, removeDuplicate);
	}


}