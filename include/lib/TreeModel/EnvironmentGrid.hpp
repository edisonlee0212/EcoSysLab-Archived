#pragma once
#include "VoxelGrid.hpp"
#include "Skeleton.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	struct LightingEstimationSettings
	{
		float m_skylightIntensity = 1.f;

		float m_shadowDistanceLoss = 1.f;
		float m_detectionRadius = 0.5f;

		float m_environmentLightIntensity = 0.05f;

		int m_blurIteration = 0;
	};
	
	struct InternodeVoxelRegistration
	{
		glm::vec3 m_position = glm::vec3(0.0f);
		SkeletonNodeHandle m_nodeHandle = -1;
		unsigned m_treeModelIndex = 0;
		float m_thickness = 0.0f;
	};

	struct EnvironmentVoxel
	{
		glm::vec3 m_lightDirection = glm::vec3(0, 1, 0);
		float m_selfShadow = 0.0f;
		float m_lightIntensity = 1.0f;
		float m_totalBiomass = 0.0f;

		std::vector<InternodeVoxelRegistration> m_internodeVoxelRegistrations{};
	};

	class EnvironmentGrid
	{
	public:
		float m_voxelSize = 0.2f;
		LightingEstimationSettings m_settings;
		VoxelGrid<EnvironmentVoxel> m_voxel;
		[[nodiscard]] float Sample(const glm::vec3& position, glm::vec3& lightDirection) const;
		void AddShadowValue(const glm::vec3& position, float value);
		void LightPropagation();
		void AddBiomass(const glm::vec3& position, float value);
		void AddNode(const InternodeVoxelRegistration& registration);
	};
}