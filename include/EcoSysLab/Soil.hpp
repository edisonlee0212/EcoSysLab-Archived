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
		SoilParameters m_soilParameters;
	};

	/**
	 * \brief The soil is designed to be a private component of an entity.
	 * It holds the soil model and can be referenced by multiple trees.
	 * The soil will also take care of visualization and menu for soil model.
	 */
	class Soil : public IPrivateComponent {

	public:
		SoilModel m_soilModel;

	};
}