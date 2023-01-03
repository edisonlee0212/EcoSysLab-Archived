#pragma once

#include <glm/glm.hpp>
#include <vector>
#include "ecosyslab_export.h"

using namespace UniEngine;

namespace EcoSysLab {
	class SoilParameters {
	public:
		// scaling factors for different forces
		float m_densityGradientForceFactor = 1.0f;
		float m_gravityForceFactor = 1.0f;
		float m_capacityGradientForceFactor = 1.0f;

		float m_deltaTime = 0.2f; // delta t, time between steps
	};

	class SoilModel {
		friend class Soil;
	public:
		void Initialize(const SoilParameters& soilParameters = SoilParameters(),
			const glm::uvec3& voxelResolution = glm::uvec3(64, 64, 64),
			float voxelDistance = 1.0f,
			const glm::vec3& minPosition = glm::vec3(-31.5f, -31.5f, -31.5f));

		void Reset();
		void Step();
		void WaterLogic(); // can be called for each step to add some water to the volume

		[[nodiscard]] float GetWater(const glm::vec3& position) const;
		[[nodiscard]] float GetDensity(const glm::vec3& position) const;
		[[nodiscard]] float GetNutrient(const glm::vec3& position) const;

		[[nodiscard]] void ChangeWater(   const glm::vec3& position, float amount);
		[[nodiscard]] void ChangeDensity( const glm::vec3& position, float amount);
		[[nodiscard]] void ChangeNutrient(const glm::vec3& position, float amount);

		[[nodiscard]] static int Index(const glm::uvec3& resolution, int x, int y, int z);
		[[nodiscard]] int Index(int x, int y, int z) const;

		[[nodiscard]] static unsigned Index(const glm::uvec3& resolution, const glm::uvec3& coordinate);
		[[nodiscard]] unsigned Index(const glm::uvec3& coordinate) const;

		[[nodiscard]] glm::uvec3 GetCoordinate(unsigned index) const;

		[[nodiscard]] glm::uvec3 GetCoordinate(const glm::vec3& position) const;
		[[nodiscard]] glm::vec3 GetCenter(const glm::uvec3& coordinate) const;

		
		[[nodiscard]] glm::uvec3 GetVoxelResolution() const;
		[[nodiscard]] float GetVoxelSize() const;
		[[nodiscard]] glm::vec3 GetStartPosition() const;
		[[nodiscard]] bool Initialized() const;
		[[nodiscard]] float GetTime() const;

	protected:
		bool m_initialized = false;

		glm::uvec3 m_resolution;
		float m_voxelSize; // delta x, distance between two voxels
		float m_deltaTime; // delta t, time between steps
		float m_time = 0.0f; // time since start

		// scaling factors for different forces
		float m_diffusionForce;
		float m_gravityForce;
		float m_capacityForce;
		glm::vec3 m_gravityDirection = glm::vec3(0, -1, 0);

		// Fields:
		std::vector<float> m_waterDensity;
		std::vector<float> m_waterDensityBlur;
		std::vector<float> m_gradWaterDensityX;
		std::vector<float> m_gradWaterDensityY;
		std::vector<float> m_gradWaterDensityZ;

		std::vector<float> m_fluxX; // three components of the total flux
		std::vector<float> m_fluxY;
		std::vector<float> m_fluxZ;

		std::vector<float> m_divergenceX; // divergence components before summing
		std::vector<float> m_divergenceY;
		std::vector<float> m_divergenceZ;

		std::vector<float> m_divergence; // scalar divergence of the flux field

		std::vector<float> m_grad_cw;

		// nutrients
		std::vector<float> m_nutrientsDensity;
		/////////////////////////////////

		float m_totalWaterDensity = 0;

		// helper variables:
		std::vector<int>   m_blur_3x3_idx;
		std::vector<float> m_blur_3x3_weights;

		glm::vec3 m_volumePositionMin;

		void Convolution3(const std::vector<float>& input, std::vector<float>& output, const std::vector<int>& indices, const std::vector<float>& weights) const;
	};
}