#pragma once
#include "PipeModelData.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	struct PipeModelParameters
	{
		float m_profileDefaultCellRadius = 0.001f;
		float m_damping = 0.01f;
		float m_gravityStrength = 400.0f;
		int m_maxSimulationIterationCellFactor = 100;
		int m_timeout = 10000;
		int m_stabilizationCheckIteration = 200;
		float m_stabilizationMovementDistance = 0.1f;

		int m_endNodeStrands = 3;
	};
}