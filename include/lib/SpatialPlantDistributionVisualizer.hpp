#pragma once
#include "SpatialPlantDistribution.hpp"
using namespace EvoEngine;
namespace EcoSysLab {
	class SpatialPlantDistributionVisualizer : public IPrivateComponent {
	public:
		SpatialPlantDistribution m_distribution{};
		bool m_simulate = false;
		static void OnInspectSpatialPlantDistributionFunction(const SpatialPlantDistribution& spatialPlantDistribution,
			const std::function<void(glm::vec2 position)>& func, const std::function<void(ImVec2 origin, float zoomFactor, ImDrawList*)>& drawFunc);
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
		void FixedUpdate() override;
		void OnCreate() override;
	};

}