#pragma once
#include "VoxelGrid.hpp"
#include "Skeleton.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	struct IlluminationEstimationSettings
	{
		float m_voxelSize = 0.1f;
		float m_minShadowIntensity = 0.02f;
		float m_maxShadowIntensity = 1.0f;
		float m_distancePowerFactor = 2.0f;
		float m_distanceMultiplier = 2.0f;

		float m_internodeShadowMultiplier = 0.5f;
		float m_fruitShadowMultiplier = 1.0f;
		float m_leafShadowMultiplier = 1.0f;

		float m_shadowIntensityMultiplier = 0.02f;
	};
	struct ShadowVolume
	{
		glm::vec3 m_position;
		float m_value;
	};

	struct InternodeVoxelRegistration
	{
		glm::vec3 m_position = glm::vec3(0.0f);
		NodeHandle m_nodeHandle = -1;
		unsigned m_treeModelIndex = 0;
		float m_thickness = 0.0f;
	};

	struct EnvironmentVoxel
	{
		glm::vec3 m_shadowDirection = glm::vec3(0.0f);
		float m_shadowIntensity = 0.0f;
		float m_totalBiomass = 0.0f;

		std::vector<InternodeVoxelRegistration> m_internodeVoxelRegistrations{};
	};

	class EnvironmentGrid
	{
	public:
		IlluminationEstimationSettings m_settings;
		VoxelGrid<EnvironmentVoxel> m_voxel;
		[[nodiscard]] float IlluminationEstimation(const glm::vec3& position, glm::vec3& lightDirection) const;
		void AddShadowVolume(const ShadowVolume& shadowVolume);
		void AddBiomass(const glm::vec3& position, float value);
		void AddNode(const InternodeVoxelRegistration& registration);
	};
}