#pragma once
#include <Plot2D.hpp>

#include "StrandModelData.hpp"
#include "StrandModelProfile.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	struct StrandModelParameters
	{
		float m_centerAttractionStrength = 40000;

		int m_maxSimulationIterationCellFactor = 5;
		int m_branchProfilePackingMaxIteration = 200;
		int m_junctionProfilePackingMaxIteration = 500;
		int m_modifiedProfilePackingMaxIteration = 1500;

		float m_overlapThreshold = 0.1f;
		int m_endNodeStrands = 1;
		int m_strandsAlongBranch = 0;
		bool m_preMerge = true;

		int m_nodeMaxCount = -1;

		int m_boundaryPointDistance = 6;

		glm::vec4 m_boundaryPointColor = glm::vec4(0.6f, 0.3f, 0, 1);
		glm::vec4 m_contentPointColor = glm::vec4(0, 0.3, 0.0f, 1);

		float m_sidePushFactor = 1.0f;
		float m_apicalSidePushFactor = 1.f;
		float m_rotationPushFactor = 1.f;
		float m_apicalBranchRotationPushFactor = 1.f;

		PlottedDistribution<float> m_branchTwistDistribution{};
		PlottedDistribution<float> m_junctionTwistDistribution{};
		PlottedDistribution<float> m_strandRadiusDistribution {};
		float m_cladoptosisRange = 10.0f;
		PlottedDistribution<float> m_cladoptosisDistribution{};

		ParticlePhysicsSettings m_profilePhysicsSettings{};
	};
}