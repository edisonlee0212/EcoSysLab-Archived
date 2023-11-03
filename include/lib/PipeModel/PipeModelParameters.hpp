#pragma once
#include "PipeModelData.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	struct PipeModelParameters
	{
		float m_profileDefaultCellRadius = 0.001f;
		float m_damping = 0.01f;
		float m_gravityStrength = 200.0f;
		float m_simulationIterationCellFactor = 50;
		int m_minimumSimulationIteration = 10;
		float m_particleStabilizeSpeed = 2.0f;

		int m_endNodeStrands = 1;
	};
}