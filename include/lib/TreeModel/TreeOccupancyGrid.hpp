#pragma once
#include "VoxelGrid.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	struct OccupancyGridSettings
	{
		
	};

	struct TreeOccupancyGridMarker
	{
		glm::vec3 m_position;
	};

	struct TreeOccupancyGridVoxelData {
		std::vector<TreeOccupancyGridMarker> m_markers;
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
		[[nodiscard]] float GetRemovalDistanceFactor() const;
		[[nodiscard]] float GetTheta() const;
		[[nodiscard]] float GetDetectionDistanceFactor() const;
		[[nodiscard]] float GetInternodeLength() const;
		[[nodiscard]] size_t GetMarkersPerVoxel() const;

		void Initialize(const glm::vec3& min, const glm::vec3& max, float internodeLength,
		                float removalDistanceFactor = 2.0f, float theta = 90.0f, float detectionDistanceFactor = 4.0f, size_t markersPerVoxel = 5);
		void Resize(const glm::vec3 &min, const glm::vec3& max);

		[[nodiscard]] VoxelGrid<TreeOccupancyGridVoxelData>& RefGrid();
		[[nodiscard]] glm::vec3 GetMin() const;
		[[nodiscard]] glm::vec3 GetMax() const;
	};
}