#pragma once

#include "Tree.hpp"
#include "Soil.hpp"
#include "Climate.hpp"
#include "Strands.hpp"

using namespace EvoEngine;
namespace EcoSysLab {
	class SimulationSettings
	{
	public:
		int m_iteration = 0;
		float m_deltaTime = 0.0822f;
		bool m_soilSimulation = false;
		bool m_autoClearFruitAndLeaves = true;
		float m_crownShynessDistance = 0.15f;
		int m_maxNodeCount = 0;

		float m_minGrowthRate = 0.8f;
		float m_maxGrowthRate = 1.f;

		float m_minLowBranchPruning = 0.f;
		float m_maxLowBranchPruning = 0.f;

		LightingEstimationSettings m_lightingEstimationSettings;

		void Serialize(YAML::Emitter& out);
		void Deserialize(const YAML::Node& in);
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer);
	};
}