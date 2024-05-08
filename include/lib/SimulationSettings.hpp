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
		float m_deltaTime = 0.0822f;
		bool m_soilSimulation = false;
		bool m_autoClearFruitAndLeaves = true;
		float m_crownShynessDistance = 0.15f;
		int m_maxNodeCount = 0;

		


		float m_skylightIntensity = 1.f;

		float m_shadowDistanceLoss = 1.f;
		float m_detectionRadius = 0.5f;

		float m_environmentLightIntensity = 0.01f;

		int m_blurIteration = 0;

		void Save(const std::string& name, YAML::Emitter& out);
		void Load(const std::string& name, const YAML::Node& in);
		void Serialize(YAML::Emitter& out);
		void Deserialize(const YAML::Node& in);
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer);
	};
}