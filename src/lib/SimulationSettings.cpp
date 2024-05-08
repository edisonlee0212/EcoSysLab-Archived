#include "SimulationSettings.hpp"

using namespace EcoSysLab;

void SimulationSettings::Serialize(YAML::Emitter& out)
{
}

void SimulationSettings::Deserialize(const YAML::Node& in)
{
}

void SimulationSettings::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	if (ImGui::Button("Grow weekly")) m_deltaTime = 0.01918f;
	ImGui::SameLine();
	if (ImGui::Button("Grow monthly")) m_deltaTime = 0.0822f;
	ImGui::SameLine();
	ImGui::DragFloat("Delta time", &m_deltaTime, 0.00001f, 0, 1, "%.5f");
	ImGui::Checkbox("Auto clear fruit and leaves", &m_autoClearFruitAndLeaves);
	ImGui::DragFloat("Crown shyness", &m_crownShynessDistance, 0.01f, 0.0f, 1.0f);

	ImGui::DragFloat("Min growth rate", &m_minGrowthRate, 0.01f, 0.f, m_maxGrowthRate);
	ImGui::DragFloat("Max growth rate", &m_maxGrowthRate, 0.01f, m_minGrowthRate, 3.f);
	ImGui::DragFloat("Min low branch pruning", &m_minLowBranchPruning, 0.01f, 0.f, m_maxLowBranchPruning);
	ImGui::DragFloat("Max low branch pruning", &m_maxLowBranchPruning, 0.01f, m_minLowBranchPruning, 1.f);

	ImGui::Checkbox("Simulate soil", &m_soilSimulation);
	if (ImGui::TreeNode("Lighting Estimation Settings")) {
		bool settingsChanged = false;
		settingsChanged =
			ImGui::DragFloat("Skylight Intensity", &m_lightingEstimationSettings.m_skylightIntensity, 0.01f,
				0.0f, 10.0f) || settingsChanged;
		settingsChanged =
			ImGui::DragFloat("Environmental Intensity", &m_lightingEstimationSettings.m_environmentLightIntensity, 0.01f,
				0.0f, 10.0f) || settingsChanged;
		settingsChanged =
			ImGui::DragFloat("Shadow distance loss", &m_lightingEstimationSettings.m_shadowDistanceLoss, 0.01f,
				0.0f, 10.0f) || settingsChanged;
		settingsChanged =
			ImGui::DragFloat("Detection radius", &m_lightingEstimationSettings.m_detectionRadius, 0.001f,
				0.0f, 1.0f) || settingsChanged;
		settingsChanged =
			ImGui::DragInt("Blur iteration", &m_lightingEstimationSettings.m_blurIteration, 1, 0, 10) || settingsChanged;
		/*
		if (settingsChanged) {
			const auto climateCandidate = FindClimate();
			if (!climateCandidate.expired()) {
				const auto climate = climateCandidate.lock();
				for (const auto& i : *treeEntities) {
					const auto tree = scene->GetOrSetPrivateComponent<Tree>(i).lock();
					tree->m_climate = climate;
				}
				climate->PrepareForGrowth();
				for (const auto& i : *treeEntities) {
					const auto tree = scene->GetOrSetPrivateComponent<Tree>(i).lock();
					tree->m_treeModel.CalculateShootFlux(scene->GetDataComponent<GlobalTransform>(i).m_value, climate->m_climateModel, tree->m_shootGrowthController);
					tree->m_treeVisualizer.m_needUpdate = true;
				}
			}
		}*/
		ImGui::TreePop();
	}
}
