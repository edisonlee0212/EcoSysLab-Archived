#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <functional>
#include <random>
#include <valarray>
#include <map>
#include "ecosyslab_export.h"

using namespace UniEngine;

namespace EcoSysLab {

	using Field = std::valarray<float>;

	class SoilParameters;


	struct SoilSurface
	{
		std::function<float(const glm::vec2& position)> m_height;
	};

	struct SoilPhysicalMaterial
	{
		int m_id = -1;
		std::function<float(const glm::vec3& position)> m_c; // capacity
		std::function<float(const glm::vec3& position)> m_p; // permeability
		std::function<float(const glm::vec3& position)> m_d; // density

		std::function<float(const glm::vec3& position)> m_n; // initial amount of nutrients
		std::function<float(const glm::vec3& position)> m_w; // initial amount of water
	};

	struct SoilLayer
	{
		SoilPhysicalMaterial m_mat;
		std::function<float(const glm::vec2& position)> m_thickness;
	};

	class SoilModel {
		friend class Soil;
		friend class EcoSysLabLayer;
	public:
		enum class Boundary : int {sink, block, wrap, absorb};

		class Source
		{
		public:
			std::vector<int> idx;
			std::vector<float> amounts;
			void Apply(Field& target);
		};


		void Initialize(const SoilParameters& p, const SoilSurface& soilSurface, const std::vector<SoilLayer>& soilLayers);

		void Reset();
		void Step();
		void Irrigation(); // can be called for each step to add some water to the volume

		[[nodiscard]] float GetWater(const glm::vec3& position) const;
		[[nodiscard]] float GetDensity(const glm::vec3& position) const;
		[[nodiscard]] float GetNutrient(const glm::vec3& position) const;
		[[nodiscard]] float GetCapacity(const glm::vec3& position) const;

		[[nodiscard]] void ChangeWater(   const glm::vec3& center, float amount, float width);
		[[nodiscard]] void ChangeDensity( const glm::vec3& center, float amount, float width);
		[[nodiscard]] void ChangeNutrient(const glm::vec3& center, float amount, float width);
		[[nodiscard]] void ChangeCapacity(const glm::vec3& center, float amount, float width);

		// negative indices are useful as relative offsets
		[[nodiscard]] static int Index(const glm::ivec3& resolution, int x, int y, int z);
		[[nodiscard]]        int Index(int x, int y, int z) const;
		[[nodiscard]] static int Index(const glm::ivec3& resolution, const glm::ivec3& coordinate);
		[[nodiscard]]        int Index(const glm::ivec3& coordinate) const;

		[[nodiscard]] glm::ivec3 GetCoordinateFromIndex(const int index) const;
		[[nodiscard]] glm::ivec3 GetCoordinateFromPosition(const glm::vec3& position) const;
		[[nodiscard]] glm::vec3  GetPositionFromCoordinate(const glm::ivec3& coordinate, float dx) const;
		[[nodiscard]] glm::vec3  GetPositionFromCoordinate(const glm::ivec3& coordinate) const; // uses m_dx as dx
				
		[[nodiscard]] glm::ivec3 GetVoxelResolution() const;
		[[nodiscard]] float GetVoxelSize() const;
		[[nodiscard]] float GetTime() const;
		[[nodiscard]] glm::vec3 GetBoundingBoxCenter() const;
		[[nodiscard]] glm::vec3 GetBoundingBoxMin() const;
		[[nodiscard]] glm::vec3 GetBoundingBoxMax() const;
		bool PositionInsideVolume(const glm::vec3& position) const;
		bool CoordinateInsideVolume(const glm::ivec3& coordinate) const;
		[[nodiscard]] bool Initialized() const;

		std::vector<glm::vec4> GetSoilTextureSlideZ(int slize_z, glm::uvec2 resolution, std::map<int, std::vector<glm::vec4>*> textures, float blur_width=1); // the output as well as all input textures must have the same resolution!

		int m_version = 0; // TODO: what does this do?
	protected:
		void BuildFromLayers(const SoilSurface& soilSurface, const std::vector<SoilLayer>& soil_layers); // helper function called inside initialize to set up soil layers
		void SetVoxel(const glm::ivec3& coordinate, const SoilPhysicalMaterial& material);

		float GetField(const Field& field, const glm::vec3& position, float default_value) const;
		void ChangeField(Field& field, const glm::vec3& center, float amount, float width);
		void SetField(Field& field, const glm::vec3& bb_min, const glm::vec3& bb_max, float value);
		void BlurField(Field& field); // for now there is just one standard kernel
		
		void Convolution3(   const Field& input, Field& output, const std::vector<int>& indices, const std::vector<float>& weights) const;

		// Boundary stuff

		void Boundary_Wrap_Axis(   const Field& input, Field& output, const std::vector<int>& indices_1D, const std::vector<float>& weights, int lim_a, int lim_b, int lim_f, std::function<int(int, int, int)> WrapIndex) const;
		void Boundary_Wrap_X(   const Field& input, Field& output, const std::vector<int>& indices_1D, const std::vector<float>& weights) const;
		void Boundary_Wrap_Y(   const Field& input, Field& output, const std::vector<int>& indices_1D, const std::vector<float>& weights) const;
		void Boundary_Wrap_Z(   const Field& input, Field& output, const std::vector<int>& indices_1D, const std::vector<float>& weights) const;
		
		void Boundary_Barrier_Axis(const Field& input, Field& output, const std::vector<int>& indices_1D, const std::vector<float>& weights, int lim_a, int lim_b, int lim_f, std::function<int(int, int, int)> WrapIndex) const;
		void Boundary_Barrier_X(   const Field& input, Field& output, const std::vector<int>& indices_1D, const std::vector<float>& weights) const;
		void Boundary_Barrier_Y(   const Field& input, Field& output, const std::vector<int>& indices_1D, const std::vector<float>& weights) const;
		void Boundary_Barrier_Z(   const Field& input, Field& output, const std::vector<int>& indices_1D, const std::vector<float>& weights) const;

		//////
		
		void AddSource(Source&& source);

		void UpdateStats(); // updates sum of water and max speeds
		bool m_initialized = false;

		glm::ivec3 m_resolution;
		float m_dx; // delta x, distance between two voxels
		float m_dt; // delta t, time between steps
		float m_time = 0.0f; // time since start

		// scaling factors for different forces
		float m_diffusionForce;
		glm::vec3 m_gravityForce;
		bool m_use_capacity = true;
		float m_nutrientForce;

		// Fields:
		std::valarray<int> m_material_id; // material id for each foxel in the soil volume

		Field m_w; // water content of each cell
		Field m_c; // the capacity of each cell
		Field m_l; // filling level of each cell, w/c
		Field m_p; // permeability of each cell


		// these field are temporary variables but kept so we don't have to reallocate them each step
		Field m_w_grad_x;
		Field m_w_grad_y;
		Field m_w_grad_z;

		Field m_div_diff_x; // divergence components for diffusion process
		Field m_div_diff_y;
		Field m_div_diff_z;

		Field m_div_diff_n_x; // divergence components for diffusion process of nutrients
		Field m_div_diff_n_y;
		Field m_div_diff_n_z;

		Field m_div_grav_x; // divergence components for gravity
		Field m_div_grav_y;
		Field m_div_grav_z;

		Field m_div_grav_n_x; // divergence components for gravity of nutrients
		Field m_div_grav_n_y;
		Field m_div_grav_n_z;
		
		// nutrients
		Field m_n;
		// soil density
		Field m_d;
		Boundary m_boundary_x, m_boundary_y, m_boundary_z;
		int m_absorption_width= 5;

		/////////////////////////////////

		glm::vec3 m_boundingBoxMin;

		float m_w_sum = 0;
		float m_max_speed_diff = 0.f;
		float m_max_speed_grav = 0.f;
		std::mt19937 m_rnd;
		float m_irrigationAmount = 1;

		std::vector<Source> m_water_sources;

		// helper variables:
		std::vector<glm::ivec3> m_blur_3x3_idx;
		std::vector<float> m_blur_3x3_weights;
	};



	class SoilParameters {
	public:
		glm::ivec3 m_voxelResolution = glm::ivec3(64, 48, 64);
		float m_deltaX = 0.2f;
		float m_deltaTime = 0.01f; // delta t, time between steps
		glm::vec3& m_boundingBoxMin = glm::vec3(-6.4, -6.4, -6.4);

		SoilModel::Boundary m_boundary_x = SoilModel::Boundary::sink;
		SoilModel::Boundary m_boundary_y = SoilModel::Boundary::sink;
		SoilModel::Boundary m_boundary_z = SoilModel::Boundary::sink;

		float m_diffusionForce = 0.1;
		glm::vec3 m_gravityForce = glm::vec3(0, -0.05, 0);
		float m_nutrientForce = 0.05;
	};
}
