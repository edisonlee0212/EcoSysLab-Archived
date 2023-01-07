#pragma once

#include <glm/glm.hpp>
#include <vector>
#include "ecosyslab_export.h"

using namespace UniEngine;

namespace EcoSysLab {
	class SoilParameters {
	public:
		// scaling factors for different forces
		float m_diffusionForce = 1.0f;
		glm::vec3 m_gravityForce = glm::vec3(0, 0, 0);
		float m_deltaX = 0.2f;
		float m_deltaTime = 0.2f; // delta t, time between steps
	};

	class SoilModel {
		friend class Soil;
	public:
		void Initialize(const SoilParameters& soilParameters, const glm::uvec3& voxelResolution,
			const glm::vec3& minPosition);

		void Reset();
		void Step();
		void WaterLogic(); // can be called for each step to add some water to the volume

		[[nodiscard]] float GetWater(const glm::vec3& position) const;
		[[nodiscard]] float GetDensity(const glm::vec3& position) const;
		[[nodiscard]] float GetNutrient(const glm::vec3& position) const;

		[[nodiscard]] void ChangeWater(   const glm::vec3& center, float amount, float width);
		[[nodiscard]] void ChangeDensity( const glm::vec3& position, float amount);
		[[nodiscard]] void ChangeNutrient(const glm::vec3& position, float amount);

		// negative indices are useful as relative offsets
		[[nodiscard]] static int Index(const glm::uvec3& resolution, int x, int y, int z);
		[[nodiscard]]        int Index(int x, int y, int z) const;
		[[nodiscard]] static int Index(const glm::uvec3& resolution, const glm::uvec3& coordinate);
		[[nodiscard]]        int Index(const glm::uvec3& coordinate) const;

		[[nodiscard]] glm::ivec3 GetCoordinateFromIndex(const int index) const;
		[[nodiscard]] glm::ivec3 GetCoordinateFromPosition(const glm::vec3& position) const;
		[[nodiscard]] glm::vec3  GetPositionFromCoordinate(const glm::ivec3& coordinate) const;
				
		[[nodiscard]] glm::uvec3 GetVoxelResolution() const;
		[[nodiscard]] float GetVoxelSize() const;
		[[nodiscard]] float GetTime() const;
		[[nodiscard]] glm::vec3 GetBoundingBoxCenter() const;
		[[nodiscard]] glm::vec3 GetBoundingBoxMin() const;
		[[nodiscard]] glm::vec3 GetBoundingBoxMax() const;
		[[nodiscard]] bool Initialized() const;
	protected:
		void update_w_sum();
		bool m_initialized = false;

		glm::uvec3 m_resolution;
		float m_dx; // delta x, distance between two voxels
		float m_dt; // delta t, time between steps
		float m_time = 0.0f; // time since start

		// scaling factors for different forces
		float m_diffusionForce;
		glm::vec3 m_gravityForce;

		// Fields:
		std::vector<float> m_w;

		// these field are temporary variables but kept so we don't have to reallocate them each step
		std::vector<float> m_tmpValue;
		std::vector<float> m_w_grad_x;
		std::vector<float> m_w_grad_y;
		std::vector<float> m_w_grad_z;

		std::vector<float> m_div_diff_x; // divergence components for diffusion process
		std::vector<float> m_div_diff_y;
		std::vector<float> m_div_diff_z;

		std::vector<float> m_div_grav_x; // divergence components for gravity
		std::vector<float> m_div_grav_y;
		std::vector<float> m_div_grav_z;

		std::vector<float> m_grad_cw;

		// nutrients
		std::vector<float> m_nutrientsDensity;
		/////////////////////////////////

		float m_w_sum = 0;

		// helper variables:
		std::vector<int>   m_blur_3x3_idx;
		std::vector<float> m_blur_3x3_weights;

		glm::vec3 m_boundingBoxMin;

		void Convolution3(const std::vector<float>& input, std::vector<float>& output, const std::vector<int>& indices, const std::vector<float>& weights) const;
	};
}