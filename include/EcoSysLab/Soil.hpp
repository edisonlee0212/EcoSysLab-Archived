#pragma once

#include "ecosyslab_export.h"
#include "SoilModel.hpp"
using namespace UniEngine;
namespace EcoSysLab
{
	/**
	 * \brief The soil descriptor contains the procedural paremeters for soil model.
	 * It helps provide the user's control menu and serialization outside the portable soil model
	 */
	class SoilDescriptor : public IAsset {
	public:
		glm::uvec3 m_voxelResolution = glm::uvec3(65, 33, 65);
		float m_voxelSize = 0.1f; // delta x, distance between two voxels
		glm::vec3 m_startPosition = glm::vec3(-3.25f, -1.65f, -3.25f);

		SoilParameters m_soilParameters;
		/**ImGui menu goes to here.Also you can take care you visualization with Gizmos here.
		 * Note that the visualization will only be activated while you are inspecting the soil private component in the entity inspector.
		 */
		void OnInspect() override;

		void Serialize(YAML::Emitter& out) override;

		void Deserialize(const YAML::Node& in) override;
	};
	enum class SoilProperty
	{
		WaterDensityBlur,
		WaterDensity,
		WaterDensityGradient,
		Flux,
		Divergence,
		ScalarDivergence,
		NutrientDensity
	};
	/**
	 * \brief The soil is designed to be a private component of an entity.
	 * It holds the soil model and can be referenced by multiple trees.
	 * The soil will also take care of visualization and menu for soil model.
	 */
	class Soil : public IPrivateComponent {

	public:
		SoilModel m_soilModel;
		AssetRef m_soilDescriptor;

		/**ImGui menu goes to here.Also you can take care you visualization with Gizmos here.
		 * Note that the visualization will only be activated while you are inspecting the soil private component in the entity inspector.
		 */
		void OnInspect() override;


	};
}