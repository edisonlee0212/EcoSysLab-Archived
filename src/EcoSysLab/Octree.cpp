#include "Octree.hpp"

using namespace EcoSysLab;

int Octree::NewNode()
{
	m_octreeNodes.emplace_back();
	return m_octreeNodes.size() - 1;
}

Octree::Octree()
{
	Reset();
}

bool Octree::Occupied(const glm::vec3& position) const
{
	float currentRadius = m_radius;
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

void Octree::Reset()
{
	m_octreeNodes.clear();
	NewNode();
}

void Octree::Occupy(const glm::vec3& position)
{
	float currentRadius = m_radius;
	glm::vec3 center = m_center;
	int octreeNodeIndex = 0;
	for(int subdivision = 0; subdivision < m_maxSubdivisionLevel; subdivision++)
	{
		currentRadius /= 2.f;
		auto& octreeNode = m_octreeNodes[octreeNodeIndex];
		const int index = 4 * (position.x > center.x ? 0 : 1) + 2 * (position.y > center.y ? 0 : 1) + (position.z > center.z ? 0 : 1);
		if(octreeNode.m_children[index] == -1)
		{
			octreeNode.m_children[index] = octreeNodeIndex = NewNode();
		}else octreeNodeIndex = octreeNode.m_children[index];
		center.x += position.x > center.x ? currentRadius : -currentRadius;
		center.y += position.y > center.y ? currentRadius : -currentRadius;
		center.z += position.z > center.z ? currentRadius : -currentRadius;
		
	}
}

void Octree::GetVoxels(std::vector<glm::mat4>& voxels) const
{
	voxels.clear();
	float currentRadius = m_radius;
	for (int subdivision = 0; subdivision < m_maxSubdivisionLevel; subdivision++)
	{
		currentRadius /= 2.f;
	}

	for(float x = -m_radius; x < m_radius; x += currentRadius)
	{
		for (float y = -m_radius; y < m_radius; y += currentRadius)
		{
			for (float z = -m_radius; z < m_radius; z += currentRadius)
			{
				if(Occupied(m_center + glm::vec3(x, y, z)))
				{
					voxels.push_back(glm::translate(m_center + glm::vec3(x, y, z)) * glm::mat4_cast(glm::quat(glm::vec3(0))) * glm::scale(glm::vec3(currentRadius)));
				}
			}
		}
	}
}
