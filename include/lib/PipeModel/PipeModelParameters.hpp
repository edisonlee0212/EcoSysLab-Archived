#pragma once
#include <Plot2D.hpp>

#include "PipeModelData.hpp"
#include "PipeProfile.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	struct PipeModelParameters
	{
		float m_boundaryStrength = 40000;
		float m_centerAttractionStrength = 4000;

		int m_maxSimulationIterationCellFactor = 5;
		int m_branchProfilePackingMaxIteration = 50;
		int m_junctionProfilePackingMaxIteration = 500;
		int m_modifiedProfilePackingMaxIteration = 1500;

		float m_overlapThreshold = 0.1f;
		int m_endNodeStrands = 3;
		float m_branchTwistAngle = 0.0f;
		float m_junctionTwistAngle = 0.0f;
		bool m_preMerge = true;

		float m_controlPointRatio = 0.3f;
		bool m_triplePoints = false;
		int m_nodeMaxCount = -1;

		int m_boundaryPointDistance = 6;

		glm::vec4 m_boundaryPointColor = glm::vec4(0.6f, 0.3f, 0, 1);
		glm::vec4 m_contentPointColor = glm::vec4(0, 0.3, 0.0f, 1);

		float m_shiftPushRatio = 0.0f;
		float m_sidePushRatio = 1.0f;
		float m_frontPushRatio = 1.0f;
		float m_rotationPushRatio = 0.5f;

		PlottedDistribution<float> m_pipeRadiusDistribution {};

		ParticlePhysicsSettings m_profilePhysicsSettings{};
	};
}