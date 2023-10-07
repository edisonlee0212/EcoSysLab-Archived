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
	public:
		void Initialize();
	};
}