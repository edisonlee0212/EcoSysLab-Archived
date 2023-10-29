#pragma once
#include "Skeleton.hpp"
#include "VoxelGrid.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	class RadialBoundingVolume;

	struct OccupancyGridSettings
	{
		
	};

	struct TreeOccupancyGridMarker
	{
		glm::vec3 m_position = glm::vec3(0.0f);
		NodeHandle m_nodeHandle = -1;
	};

	struct TreeOccupancyGridVoxelData {
		std::vector<TreeOccupancyGridMarker> m_markers;
	};

	struct TreeOccupancyGridBasicData
	{
		bool m_occupied = false;
	};

	class TreeOccupancyGrid
	{
		VoxelGrid<TreeOccupancyGridVoxelData> m_occupancyGrid {};
		float m_removalDistanceFactor = 2;
		float m_theta = 90.0f;
		float m_detectionDistanceFactor = 4;
		float m_internodeLength = 1.0f;
		size_t m_markersPerVoxel = 5;
	public:
		void ResetMarkers();
		[[nodiscard]] float GetRemovalDistanceFactor() const;
		[[nodiscard]] float GetTheta() const;
		[[nodiscard]] float GetDetectionDistanceFactor() const;
		[[nodiscard]] float GetInternodeLength() const;
		[[nodiscard]] size_t GetMarkersPerVoxel() const;

		void Initialize(const glm::vec3& min, const glm::vec3& max, float internodeLength,
		                float removalDistanceFactor = 2.0f, float theta = 90.0f, float detectionDistanceFactor = 4.0f, size_t markersPerVoxel = 1);
		void Resize(const glm::vec3 &min, const glm::vec3& max);
		void Initialize(const VoxelGrid<TreeOccupancyGridBasicData>& srcGrid, const glm::vec3& min, const glm::vec3& max, float internodeLength,
			float removalDistanceFactor = 2.0f, float theta = 90.0f, float detectionDistanceFactor = 4.0f, size_t markersPerVoxel = 1);
		void Initialize(const std::shared_ptr<RadialBoundingVolume>& srcRadialBoundingVolume, const glm::vec3& min, const glm::vec3& max, float internodeLength,
			float removalDistanceFactor = 2.0f, float theta = 90.0f, float detectionDistanceFactor = 4.0f, size_t markersPerVoxel = 1);
		[[nodiscard]] VoxelGrid<TreeOccupancyGridVoxelData>& RefGrid();
		[[nodiscard]] glm::vec3 GetMin() const;
		[[nodiscard]] glm::vec3 GetMax() const;
	};
}