#include "TreeOccupancyGrid.hpp"

using namespace EcoSysLab;


void TreeOccupancyGrid::Initialize(const glm::vec3& min, const glm::vec3& max, const float internodeLength,
	const float removalDistanceFactor, const float theta, const float detectionDistanceFactor)
{
	m_removalDistanceFactor = removalDistanceFactor;
	m_detectionDistanceFactor = detectionDistanceFactor;
	m_theta = theta;
	m_internodeLength = internodeLength;
	m_occupancyGrid.Initialize(m_removalDistanceFactor * internodeLength, min, max, {});
}

void TreeOccupancyGrid::Resize(const glm::vec3& min, const glm::vec3& max)
{
	m_occupancyGrid.Resize(min - m_occupancyGrid.GetMinBound() - m_detectionDistanceFactor * m_internodeLength, 
		max - m_occupancyGrid.GetMaxBound() + m_detectionDistanceFactor * m_internodeLength);
}

VoxelGrid<TreeOccupancyGridVoxelData>& TreeOccupancyGrid::RefGrid()
{
	return m_occupancyGrid;
}

glm::vec3 TreeOccupancyGrid::GetMin() const
{
	return m_occupancyGrid.GetMinBound();
}

glm::vec3 TreeOccupancyGrid::GetMax() const
{
	return m_occupancyGrid.GetMaxBound();
}
