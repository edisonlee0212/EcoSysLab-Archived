#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <functional>
#include "ecosyslab_export.h"

using namespace UniEngine;

namespace EcoSysLab {
	class SoilParameters;

	class SoilModel {
		friend class Soil;
		friend class EcoSysLabLayer;
	public:
		enum class Boundary : int {sink, block, wrap};
		void Initialize(const SoilParameters& soilParameters);

		void Reset();
		void Step();
		void Irrigation(); // can be called for each step to add some water to the volume

		[[nodiscard]] float GetWater(const glm::vec3& position) const;
		[[nodiscard]] float GetDensity(const glm::vec3& position) const;
		[[nodiscard]] float GetNutrient(const glm::vec3& position) const;

		[[nodiscard]] void ChangeWater(   const glm::vec3& center, float amount, float width);
		[[nodiscard]] void ChangeDensity( const glm::vec3& center, float amount, float width);
		[[nodiscard]] void ChangeNutrient(const glm::vec3& center, float amount, float width);

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

		int m_version = 0; // TODO: what does this do?
	protected:
		void ChangeField(std::vector<float>& field, const glm::vec3& center, float amount, float width);
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

		std::vector<float> m_c; // the capacity of each cell
		std::vector<float> m_c_grad_x;
		std::vector<float> m_c_grad_y;
		std::vector<float> m_c_grad_z;

		// nutrients
		std::vector<float> m_nutrientsDensity;

		std::vector<float> m_soilDensity;
		Boundary m_boundary_x, m_boundary_y, m_boundary_z;

		/////////////////////////////////

		float m_w_sum = 0;

		// helper variables:
		std::vector<int>   m_blur_3x3_idx;
		std::vector<float> m_blur_3x3_weights;

		glm::vec3 m_boundingBoxMin;

		void Convolution3(const std::vector<float>& input, std::vector<float>& output, const std::vector<int>& indices, const std::vector<float>& weights) const;
	};

	class SoilParameters {
	public:
		glm::uvec3 m_voxelResolution = glm::uvec3(64, 48, 64);
		float m_deltaX = 0.2f;
		float m_deltaTime = 0.2f; // delta t, time between steps
		glm::vec3& m_boundingBoxMin = glm::vec3(-6.4, -6.4, -6.4);

		SoilModel::Boundary m_boundary_x = SoilModel::Boundary::sink;
		SoilModel::Boundary m_boundary_y = SoilModel::Boundary::sink;
		SoilModel::Boundary m_boundary_z = SoilModel::Boundary::sink;

		float m_diffusionForce = 1.0f;
		glm::vec3 m_gravityForce = glm::vec3(0, 0, 0);

		std::function<float(const glm::vec3& position)> m_soilDensitySampleFunc = [](const glm::vec3& position)
			{
				return position.y > 0 ? 0.f : 1.f;
			};
	};
}