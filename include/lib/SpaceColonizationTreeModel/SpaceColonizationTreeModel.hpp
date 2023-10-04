#pragma once
#include "SpaceColonizationTreeGrowthData.hpp"
#include "VoxelGrid.hpp"
using namespace EvoEngine;
namespace EcoSysLab {
	struct SpaceColonizationTreeGrowthController
	{
		float m_internodeLength;

		float m_occupancyZoneRadius;
		float m_perceptionVolumeRadius;
		float m_perceptionVolumeAngle;

	};


	struct SpaceColonizationGridData
	{
		std::vector<glm::vec3> m_markerPoints;
	};

	class SpaceColonizationTreeModel
	{
	public:
		VoxelGrid<SpaceColonizationGridData> m_markerGrid;

		SpaceColonizationTreeSkeleton m_skeleton {};
	};
};