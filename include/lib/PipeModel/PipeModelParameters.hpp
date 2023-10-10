#pragma once
#include "PipeModelData.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	struct PipeModelParameters
	{
		float m_profileDefaultCellRadius = 0.01f;
		float m_damping = 0.01f;
		float m_gravityStrength = 1.0f;
		float m_simulationIterationFactor = 2;
	};
}