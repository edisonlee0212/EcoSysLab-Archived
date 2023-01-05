#include "Octree.hpp"
#include "glm/gtx/quaternion.hpp"
#include "MarchingCubes.hpp"
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

int Octree::GetIndex(const glm::vec3& position) const
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
			return -1;
		}
		octreeNodeIndex = octreeNode.m_children[index];
		center.x += position.x > center.x ? currentRadius : -currentRadius;
		center.y += position.y > center.y ? currentRadius : -currentRadius;
		center.z += position.z > center.z ? currentRadius : -currentRadius;
	}
	return octreeNodeIndex;
}

OctreeNode& Octree::RefNode(int index)
{
	return m_octreeNodes[index];
}

void Octree::Occupy(const glm::vec3& position)
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
			const auto newIndex = NewNode();
			m_octreeNodes[octreeNodeIndex].m_children[index] = newIndex;
			octreeNodeIndex = newIndex;
		}
		else octreeNodeIndex = octreeNode.m_children[index];
		center.x += position.x > center.x ? currentRadius : -currentRadius;
		center.y += position.y > center.y ? currentRadius : -currentRadius;
		center.z += position.z > center.z ? currentRadius : -currentRadius;
	}
}

bool CylinderCollision(const glm::vec3& center, const glm::vec3& position, const glm::quat& rotation, float length, float radius)
{
	const auto relativePos = glm::rotate(glm::inverse(rotation), center - position);
	return glm::abs(relativePos.z) <= length && glm::length(glm::vec2(relativePos.x, relativePos.y)) <= radius;
}

void Octree::Occupy(const glm::vec3& position, const glm::quat& rotation, float length, float radius)
{
	float minRadius = m_radius;
	const float maxRadius = glm::max(length, radius);
	for (int subdivision = 0; subdivision < m_maxSubdivisionLevel; subdivision++)
	{
		minRadius /= 2.f;
	}
	for (float x = -maxRadius + position.x - minRadius; x < maxRadius + position.x + minRadius; x += minRadius)
	{
		for (float y = -maxRadius + position.y - minRadius; y < maxRadius + position.y + minRadius; y += minRadius)
		{
			for (float z = -maxRadius + position.z - minRadius; z < maxRadius + position.z + minRadius; z += minRadius)
			{
				if (CylinderCollision(glm::vec3(x, y, z), position, rotation, length, radius))
				{
					Occupy(glm::vec3(x, y, z));
				}
			}
		}
	}
}

void Octree::IterateOccupied(const std::function<void(const glm::vec3& position, float radius)>& func, const glm::vec3& center,
	int subdivision, float voxelRadius, int nodeIndex) const
{
	if (subdivision == m_maxSubdivisionLevel - 1)
	{
		func(center, voxelRadius);
	}
	else
	{
		voxelRadius /= 2.f;
		for (int i = 0; i < 8; i++) {
			const auto childIndex = m_octreeNodes[nodeIndex].m_children[i];
			if (childIndex != -1) {
				glm::vec3 newCenter = center;
				newCenter.x += (i / 4 == 1 ? -voxelRadius : voxelRadius);
				newCenter.y += (i / 2 % 2 == 1 ? -voxelRadius : voxelRadius);
				newCenter.z += (i % 2 == 1 ? -voxelRadius : voxelRadius);
				IterateOccupied(func, newCenter, subdivision + 1, voxelRadius, childIndex);
			}
		}
	}
}

void Octree::GetVoxels(std::vector<glm::mat4>& voxels) const
{
	voxels.clear();

	IterateOccupied([&](const glm::vec3& position, float radius)
		{
			voxels.push_back(glm::translate(position) * glm::scale(glm::vec3(radius)));
		}, m_center, 0, m_radius, 0);
}

void Octree::TriangulateField(std::vector<Vertex>& vertices, std::vector<unsigned>& indices, bool removeDuplicate, bool smoothMesh) const
{
	std::vector<glm::vec3> testingCells;
	IterateOccupied([&](const glm::vec3& position, float radius)
		{
			testingCells.push_back(position);
		}, m_center, 0, m_radius, 0);

	float minRadius = m_radius;
	for (int subdivision = 0; subdivision < m_maxSubdivisionLevel; subdivision++)
	{
		minRadius /= 2.f;
	}
	MarchingCubes::TriangulateField([&](const glm::vec3& samplePoint)
		{
			return Occupied(samplePoint) ? 1.0f : 0.0f;
		}, 0.5f, minRadius, testingCells, vertices, indices, removeDuplicate || smoothMesh);


	if (smoothMesh) {
		std::vector<std::vector<unsigned>> connectivity;
		connectivity.resize(vertices.size());
		for (int i = 0; i < indices.size() / 3; i++)
		{
			auto a = indices[3 * i];
			auto b = indices[3 * i + 1];
			auto c = indices[3 * i + 2];
			//a
			{
				bool found1 = false;
				bool found2 = false;
				for (const auto& index : connectivity.at(a))
				{
					if (b == index) found1 = true;
					if (c == index) found2 = true;
				}
				if (!found1)
				{
					connectivity.at(a).emplace_back(b);
				}
				if (!found2)
				{
					connectivity.at(a).emplace_back(c);
				}
			}
			//b
			{
				bool found1 = false;
				bool found2 = false;
				for (const auto& index : connectivity.at(b))
				{
					if (a == index) found1 = true;
					if (c == index) found2 = true;
				}
				if (!found1)
				{
					connectivity.at(b).emplace_back(a);
				}
				if (!found2)
				{
					connectivity.at(b).emplace_back(c);
				}
			}
			//c
			{
				bool found1 = false;
				bool found2 = false;
				for (const auto& index : connectivity.at(c))
				{
					if (a == index) found1 = true;
					if (b == index) found2 = true;
				}
				if (!found1)
				{
					connectivity.at(c).emplace_back(a);
				}
				if (!found2)
				{
					connectivity.at(c).emplace_back(b);
				}
			}
		}
		std::vector<glm::vec3> newPositions;
		for (int i = 0; i < vertices.size(); i++)
		{
			auto position = glm::vec3(0.0f);
			for (const auto& index : connectivity.at(i))
			{
				position += vertices.at(index).m_position;
			}
			if (!connectivity.at(i).size() > 3) newPositions.push_back(position / static_cast<float>(connectivity.at(i).size()));
			else
			{
				newPositions.push_back(vertices.at(i).m_position);
			}
		}
		for (int i = 0; i < vertices.size(); i++)
		{
			vertices[i].m_position = newPositions[i];
		}
	}
}
