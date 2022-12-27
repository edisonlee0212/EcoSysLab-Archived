#pragma once

#include "ecosyslab_export.h"
using namespace UniEngine;
namespace EcoSysLab {
	class SoilParameters {
	public:
		// scaling factors for different forces
		float m_diffusionFactor = 1.0f;
		float m_gravityFactor = 1.0f;
		float m_f_cap = 1.0f;
		
		float m_deltaTime = 0.01f; // delta t, time between steps
	};

	class SoilModel {
		std::vector<float> m_waterDensity;
		std::vector<float> m_waterDensityBlur;
		std::vector<float> m_gradWaterDensityX1;
		std::vector<float> m_gradWaterDensityX2;
		std::vector<float> m_gradWaterDensityX3;

		std::vector<float> m_fluxX1; // three components of the total flux
		std::vector<float> m_fluxX2;
		std::vector<float> m_fluxX3;

		std::vector<float> m_divergenceX1; // divergence components before summing
		std::vector<float> m_divergenceX2;
		std::vector<float> m_divergenceX3;

		std::vector<float> m_divergence; // scalar divergence of the flux field

		std::vector<float> m_grad_cw;

		// nutrients
		std::vector<float> m_nutrientsDensity;
		float m_totalWaterDensity = 0;
		
		glm::uvec3 m_voxelResolution = glm::uvec3(64);

		float m_voxelDistance = 1.0f; // delta x, distance between two voxels
		bool m_initialized = false;
		float m_time = 0.0f; // time since start
	public:
		
		[[nodiscard]] float GetWater(const glm::vec3& position) const;
		[[nodiscard]] float GetDensity(const glm::vec3& position) const;
		[[nodiscard]] float GetNutrient(const glm::vec3& position) const;

		[[nodiscard]] float AddWater(const glm::vec3& position, float value);
		[[nodiscard]] float AddDensity(const glm::vec3& position, float value);
		[[nodiscard]] float AddNutrient(const glm::vec3& position, float value);
		static int Index(const glm::uvec3 &resolution, int x, int y, int z);

		void Initialize(const SoilParameters& soilParameters, const glm::uvec3& voxelResolution = { 64, 64, 64 }, float voxelDistance = 1.0f);

		virtual void Step(const SoilParameters& soilParameters);
		[[nodiscard]] glm::uvec3 GetVoxelResolution() const;
		[[nodiscard]] float GetVoxelDistance() const;
	};
}