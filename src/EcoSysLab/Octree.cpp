#include "Octree.hpp"
#include "glm/gtx/quaternion.hpp"
#include "MarchingCubes.hpp"
using namespace EcoSysLab;

int Octree::NewNode(float radius, unsigned level, const glm::vec3& center)
{
	m_octreeNodes.emplace_back();
	m_octreeNodes.back().m_radius = radius;
	m_octreeNodes.back().m_level = level;
	m_octreeNodes.back().m_center = center;
	return m_octreeNodes.size() - 1;
}


Octree::Octree()
{
	Reset(16, 10, glm::vec3(0.0f));
}

float Octree::GetMinRadius() const
{
	return m_minRadius;
}

Octree::Octree(float radius, unsigned maxSubdivisionLevel, const glm::vec3& center)
{
	Reset(radius, maxSubdivisionLevel, center);
}



bool Octree::Occupied(const glm::vec3& position) const
{
	float currentRadius = m_fieldRadius;
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

void Octree::Reset(float radius, unsigned maxSubdivisionLevel, const glm::vec3& center)
{
	m_fieldRadius = m_minRadius = radius;
	m_maxSubdivisionLevel = maxSubdivisionLevel;
	m_center = center;
	m_octreeNodes.clear();
	for (int subdivision = 0; subdivision < m_maxSubdivisionLevel; subdivision++)
	{
		m_minRadius /= 2.f;
	}
	NewNode(m_fieldRadius, -1, center);
}

int Octree::GetIndex(const glm::vec3& position) const
{
	float currentRadius = m_fieldRadius;
	glm::vec3 center = m_center;
	int octreeNodeIndex = 0;
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

const OctreeNode& Octree::RefNode(const int index) const
{
	return m_octreeNodes[index];
}

void Octree::Occupy(const glm::vec3& position)
{
	float currentRadius = m_fieldRadius;
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
			const auto newIndex = NewNode(currentRadius, subdivision, center);
			m_octreeNodes[octreeNodeIndex].m_children[index] = newIndex;
			octreeNodeIndex = newIndex;
		}
		else octreeNodeIndex = octreeNode.m_children[index];
	}
}

bool CylinderCollision(const glm::vec3& center, const glm::vec3& position, const glm::quat& rotation, float length, float radius)
{
	const auto relativePos = glm::rotate(glm::inverse(rotation), center - position);
	return glm::abs(relativePos.z) <= length && glm::length(glm::vec2(relativePos.x, relativePos.y)) <= radius;
}

void Octree::Occupy(const glm::vec3& position, const glm::quat& rotation, float length, float radius)
{
	const float maxRadius = glm::max(length, radius);
	for (float x = -maxRadius + position.x - m_minRadius; x < maxRadius + position.x + m_minRadius; x += m_minRadius)
	{
		for (float y = -maxRadius + position.y - m_minRadius; y < maxRadius + position.y + m_minRadius; y += m_minRadius)
		{
			for (float z = -maxRadius + position.z - m_minRadius; z < maxRadius + position.z + m_minRadius; z += m_minRadius)
			{
				if (CylinderCollision(glm::vec3(x, y, z), position, rotation, length, radius))
				{
					Occupy(glm::vec3(x, y, z));
				}
			}
		}
	}
}

void Octree::IterateLeaves(const std::function<void(const OctreeNode& octreeNode)>& func) const
{
	for(const auto& node : m_octreeNodes)
	{
		if(node.m_level == m_maxSubdivisionLevel - 2)
		{
			func(node);
		}
	}
}

void Octree::GetVoxels(std::vector<glm::mat4>& voxels) const
{
	voxels.clear();
	IterateLeaves([&](const OctreeNode& octreeNode)
		{
			voxels.push_back(glm::translate(octreeNode.m_center) * glm::scale(glm::vec3(m_minRadius)));
		});
}

void Octree::TriangulateField(std::vector<Vertex>& vertices, std::vector<unsigned>& indices, const bool removeDuplicate, const int smoothMeshIteration) const
{
	std::vector<glm::vec3> testingCells;
	std::vector<glm::vec3> validateCells;
	IterateLeaves([&](const OctreeNode& octreeNode)
		{
			testingCells.push_back(octreeNode.m_center);
		});
		
	MarchingCubes::TriangulateField(m_center, [&](const glm::vec3& samplePoint)
		{
			return Occupied(samplePoint) ? 1.0f : 0.0f;
		}, 0.5f, m_minRadius * 2.0f, testingCells, vertices, indices, removeDuplicate, smoothMeshIteration);	
}
