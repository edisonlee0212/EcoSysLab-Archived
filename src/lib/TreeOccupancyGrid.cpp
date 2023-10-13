#include "TreeOccupancyGrid.hpp"

using namespace EcoSysLab;


void TreeOccupancyGrid::ResetMarkers()
{
	Jobs::ParallelFor(m_occupancyGrid.GetVoxelCount(), [&](unsigned i)
		{
			auto& voxelData = m_occupancyGrid.Ref(static_cast<int>(i));

			for(auto& marker : voxelData.m_markers)
			{
				marker.m_nodeHandle = -1;
			}
		}
	);
}

float TreeOccupancyGrid::GetRemovalDistanceFactor() const
{
	return m_removalDistanceFactor;
}

float TreeOccupancyGrid::GetTheta() const
{
	return m_theta;
}

float TreeOccupancyGrid::GetDetectionDistanceFactor() const
{
	return m_detectionDistanceFactor;
}

float TreeOccupancyGrid::GetInternodeLength() const
{
	return m_internodeLength;
}

size_t TreeOccupancyGrid::GetMarkersPerVoxel() const
{
	return m_markersPerVoxel;
}

void TreeOccupancyGrid::Initialize(const glm::vec3& min, const glm::vec3& max, const float internodeLength,
	const float removalDistanceFactor, const float theta, const float detectionDistanceFactor, size_t markersPerVoxel)
{
	m_removalDistanceFactor = removalDistanceFactor;
	m_detectionDistanceFactor = detectionDistanceFactor;
	m_theta = theta;
	m_internodeLength = internodeLength;
	m_markersPerVoxel = markersPerVoxel;
	m_occupancyGrid.Initialize(m_removalDistanceFactor * internodeLength, min, max, {});
	const auto voxelSize = m_occupancyGrid.GetVoxelSize();
	Jobs::ParallelFor(m_occupancyGrid.GetVoxelCount(), [&](unsigned i)
		{
			auto& voxelData = m_occupancyGrid.Ref(static_cast<int>(i));
			for (int v = 0; v < m_markersPerVoxel; v++)
			{
				auto& newMarker = voxelData.m_markers.emplace_back();
				newMarker.m_position = m_occupancyGrid.GetPosition(i) + glm::linearRand(-glm::vec3(voxelSize * 0.5f), glm::vec3(voxelSize * 0.5f));
			}
		}
	);
}

void TreeOccupancyGrid::Resize(const glm::vec3& min, const glm::vec3& max)
{
	const auto voxelSize = m_occupancyGrid.GetVoxelSize();
	const auto diffMin = glm::floor((min - m_occupancyGrid.GetMinBound() - m_detectionDistanceFactor * m_internodeLength) / voxelSize);
	const auto diffMax = glm::ceil((max - m_occupancyGrid.GetMaxBound() + m_detectionDistanceFactor * m_internodeLength) / voxelSize);
	m_occupancyGrid.Resize(-diffMin, diffMax);
	const auto newResolution = m_occupancyGrid.GetResolution();
	Jobs::ParallelFor(m_occupancyGrid.GetVoxelCount(), [&](unsigned i)
		{
			const auto coordinate = m_occupancyGrid.GetCoordinate(i);
			
			if (coordinate.x < -diffMin.x || coordinate.y < -diffMin.y || coordinate.z < -diffMin.z
				|| coordinate.x >= newResolution.x - diffMax.x || coordinate.y >= newResolution.y - diffMax.y || coordinate.z >= newResolution.z - diffMax.z)
			{
				auto& voxelData = m_occupancyGrid.Ref(static_cast<int>(i));
				for (int v = 0; v < m_markersPerVoxel; v++)
				{
					auto& newMarker = voxelData.m_markers.emplace_back();
					newMarker.m_position = m_occupancyGrid.GetPosition(i) + glm::linearRand(-glm::vec3(voxelSize * 0.5f), glm::vec3(voxelSize * 0.5f));
				}
			}
		}
	);
}

void TreeOccupancyGrid::Initialize(const VoxelGrid<TreeOccupancyGridBasicData>& srcGrid, const glm::vec3& min, const glm::vec3& max, const float internodeLength,
	const float removalDistanceFactor, const float theta, const float detectionDistanceFactor, const size_t markersPerVoxel)
{
	m_removalDistanceFactor = removalDistanceFactor;
	m_detectionDistanceFactor = detectionDistanceFactor;
	m_theta = theta;
	m_internodeLength = internodeLength;
	m_markersPerVoxel = markersPerVoxel;
	m_occupancyGrid.Initialize(m_removalDistanceFactor * internodeLength, min, max, {});
	const auto voxelSize = m_occupancyGrid.GetVoxelSize();

	Jobs::ParallelFor(m_occupancyGrid.GetVoxelCount(), [&](unsigned i)
		{
			const glm::vec3 normalizedPosition = glm::vec3(m_occupancyGrid.GetCoordinate(i)) / glm::vec3(m_occupancyGrid.GetResolution());
			const auto srcGridPosition = normalizedPosition * (srcGrid.GetMaxBound() - srcGrid.GetMinBound());
			if(srcGrid.Peek(srcGrid.GetIndex(srcGridPosition)).m_occupied || (normalizedPosition.y < 0.2f && glm::length(glm::vec2(normalizedPosition.x - 0.5f, normalizedPosition.z - 0.5f)) < 0.05f))
			{
				auto& voxelData = m_occupancyGrid.Ref(static_cast<int>(i));
				for (int v = 0; v < m_markersPerVoxel; v++)
				{
					auto& newMarker = voxelData.m_markers.emplace_back();
					newMarker.m_position = m_occupancyGrid.GetPosition(i) + glm::linearRand(-glm::vec3(voxelSize * 0.5f), glm::vec3(voxelSize * 0.5f));
				}
			}
		}
	);

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
