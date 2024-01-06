#pragma once
#include "PipeModelData.hpp"
#include "ParticlePhysics2D.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	struct PipeModelParameters
	{
		float m_centerAttractionStrength = 400;
		float m_profileDefaultCellRadius = 0.002f;
		int m_maxSimulationIterationCellFactor = 100;
		int m_timeout = 10000;
		int m_stabilizationCheckIteration = 3000;
		float m_stabilizationMovementDistance = 0.1f;
		float m_splitRatioLimit = 0.1f;
		int m_endNodeStrands = 1;

		bool m_preMerge = true;
	};
}