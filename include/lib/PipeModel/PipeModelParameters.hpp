#pragma once
#include "PipeModelData.hpp"
#include "ParticlePhysics2D.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	struct PipeModelParameters
	{
		float m_boundaryStrength = 10000;
		float m_centerAttractionStrength = 1000;
		float m_profileDefaultCellRadius = 0.002f;
		int m_maxSimulationIterationCellFactor = 5;
		int m_timeout = 500;
		int m_timeoutWithBoundaries = 1500;
		float m_splitRatioLimit = 0.1f;
		int m_endNodeStrands = 1;

		bool m_preMerge = true;

		float m_frontControlPointRatio = 0.3f;
		float m_backControlPointRatio = 0.3f;
		bool m_triplePoints = false;
		float m_nodeMaxCount = -1;
	};
}