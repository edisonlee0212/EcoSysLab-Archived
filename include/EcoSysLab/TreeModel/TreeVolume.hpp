#pragma once
#include "Voxel.hpp"
using namespace UniEngine;
namespace EcoSysLab
{
	struct IlluminationEstimationSettings {
		int m_probeLayerAmount = 4;
		int m_probeSectorAmount = 4;
		float m_occlusion = 0.15f;
		float m_occlusionDistanceFactor = 2.5f;
		float m_overallIntensity = 2.0f;
		float m_layerAngleFactor = 0.9f;
	};
	class TreeSphericalVolume
	{
		std::vector<std::pair<float, int>> m_probe;
	public:
		std::vector<float> m_distances;
		glm::vec3 m_center = glm::vec3(0.0f);
		int m_layerAmount = 8;
		int m_sectorAmount = 8;
		bool m_hasData = false;
		float m_offset = 0;
		[[nodiscard]] int GetSectorIndex(const glm::vec3& position) const;
		void Clear();
		void TipPosition(int layerIndex, int sectorIndex, glm::vec3& position) const;
		void Smooth();
		[[nodiscard]] float IlluminationEstimation(const glm::vec3& position, const IlluminationEstimationSettings& settings, glm::vec3& lightDirection);
	};
	struct ShadowEstimationSettings
	{
		float m_voxelSize = 0.1f;
		float m_minShadowIntensity = 0.02f;
		float m_maxShadowIntensity = 1.0f;
		float m_distancePowerFactor = 2.0f;
		float m_distanceMultiplier = 2.0f;

		float m_shadowIntensityMultiplier = 10.0f;
	};
	struct ShadowVolume
	{
		glm::vec3 m_position;
		float m_size;
	};

	struct ShadowVoxel
	{
		glm::vec3 m_shadowDirection = glm::vec3(0.0f);
		float m_shadowIntensity = 0.0f;
	};

	class TreeShadowEstimator
	{
	public:
		ShadowEstimationSettings m_settings;
		Voxel<ShadowVoxel> m_voxel;
		[[nodiscard]] float IlluminationEstimation(const glm::vec3& position, glm::vec3& lightDirection) const;
		void AddShadowVolume(const ShadowVolume& shadowVolume);
	};
}