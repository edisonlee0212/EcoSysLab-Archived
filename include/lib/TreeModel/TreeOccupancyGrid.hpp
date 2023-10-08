#pragma once
#include "VoxelGrid.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	struct OccupancyGridSettings
	{
		
	};

	struct TreeOccupancyGridVoxelData {
	};

	class TreeOccupancyGrid
	{
		VoxelGrid<TreeOccupancyGridVoxelData> m_occupancyGrid {};
		float m_removalDistanceFactor = 2;
		float m_theta = 90.0f;
		float m_detectionDistanceFactor = 4;
		float m_internodeLength = 1.0f;
	public:
		void Initialize(const glm::vec3& min, const glm::vec3& max, float internodeLength,
			float removalDistanceFactor = 2.0f, float theta = 90.0f, float detectionDistanceFactor = 4.0f);
		void Resize(const glm::vec3 &min, const glm::vec3& max);

		[[nodiscard]] VoxelGrid<TreeOccupancyGridVoxelData>& RefGrid();
		[[nodiscard]] glm::vec3 GetMin() const;
		[[nodiscard]] glm::vec3 GetMax() const;
	};
}