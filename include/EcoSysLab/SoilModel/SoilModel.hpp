#pragma once

#include "ecosyslab_export.h"
using namespace UniEngine;
namespace EcoSysLab {
	class SoilParameters {
	public:
		// scaling factors for different forces
		float m_diffusionFactor = 1.0f;
		float m_gravityFactor = 1.0f;
		float m_capFactor = 1.0f;

		float m_deltaTime = 0.2f; // delta t, time between steps
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

		glm::uvec3 m_voxelResolution = glm::uvec3(65, 33, 65);

		float m_voxelSize = 1.0f; // delta x, distance between two voxels
		bool m_initialized = false;
		float m_time = 0.0f; // time since start

		friend class Soil;

		glm::vec3 m_startPosition = glm::vec3(-32.5f, -16.5f, -32.5f);
	public:
		glm::vec3 m_gravityDirection = glm::vec3(0, -1, 0);
		void Convolution3(const std::vector<float>& input, std::vector<float>& output, const std::vector<int>& indices, const std::vector<float>& weights);
		[[nodiscard]] float GetWater(const glm::vec3& position) const;
		[[nodiscard]] float GetDensity(const glm::vec3& position) const;
		[[nodiscard]] float GetNutrient(const glm::vec3& position) const;

		[[nodiscard]] float AddWater(const glm::vec3& position, float value);
		[[nodiscard]] float AddDensity(const glm::vec3& position, float value);
		[[nodiscard]] float AddNutrient(const glm::vec3& position, float value);
		[[nodiscard]] int Index(int x, int y, int z) const;


		[[nodiscard]] unsigned Index(const glm::uvec3& coordinate) const;
		[[nodiscard]] glm::uvec3 GetCoordinate(unsigned index) const;

		[[nodiscard]] glm::uvec3 GetCoordinate(const glm::vec3& position) const;
		[[nodiscard]] glm::vec3 GetCenter(const glm::uvec3& coordinate) const;


		void Initialize(const SoilParameters& soilParameters, const glm::uvec3& voxelResolution = glm::uvec3(65, 32, 65), float voxelDistance = 1.0f, const glm::vec3& minPosition = glm::vec3(-32.5f, -10.0f, -32.5f));

		void Reset();

		void TestSetup();
		
		void Step(const SoilParameters& soilParameters);

		[[nodiscard]] glm::uvec3 GetVoxelResolution() const;
		[[nodiscard]] float GetVoxelSize() const;
		[[nodiscard]] glm::vec3 GetStartPosition() const;
		[[nodiscard]] bool Initialized() const;
		[[nodiscard]] float GetTime() const;

	};
}