#pragma once
#include "PipeModelData.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	struct PipeModelParameters
	{
		float m_profileDefaultCellRadius = 0.005f;
		float m_damping = 0.01f;
		float m_gravityStrength = 100.0f;
		float m_simulationIterationCellFactor = 50;
		int m_minimumSimulationIteration = 100;
		float m_particleStabilizeSpeed = 10.0f;
	};
}