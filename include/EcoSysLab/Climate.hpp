#pragma once

#include "ecosyslab_export.h"
#include "ClimateModel.hpp"
using namespace UniEngine;
namespace EcoSysLab
{
	class ClimateDescriptor : public IAsset {
	public:
		ClimateParameters m_climateParameters;
	};
	class Climate : public IPrivateComponent {

	public:
		ClimateModel m_climateModel;

	};
}