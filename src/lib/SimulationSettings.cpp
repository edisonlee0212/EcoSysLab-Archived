#include "SimulationSettings.hpp"

using namespace EcoSysLab;

void SimulationSettings::Save(const std::string& name, YAML::Emitter& out) const
{
	out << YAML::Key << name << YAML::Value << YAML::BeginMap;
	Serialize(out);
	out << YAML::EndMap;
}

void SimulationSettings::Load(const std::string& name, const YAML::Node& in)
{
	if (in[name]) Deserialize(in[name]);
}

void SimulationSettings::Serialize(YAML::Emitter& out) const
{
	out << YAML::Key << "m_deltaTime" << YAML::Value << m_deltaTime;
	out << YAML::Key << "m_soilSimulation" << YAML::Value << m_soilSimulation;
	out << YAML::Key << "m_autoClearFruitAndLeaves" << YAML::Value << m_autoClearFruitAndLeaves;
	out << YAML::Key << "m_crownShynessDistance" << YAML::Value << m_crownShynessDistance;
	out << YAML::Key << "m_maxNodeCount" << YAML::Value << m_maxNodeCount;


	out << YAML::Key << "m_skylightIntensity" << YAML::Value << m_skylightIntensity;
	out << YAML::Key << "m_shadowDistanceLoss" << YAML::Value << m_shadowDistanceLoss;
	out << YAML::Key << "m_detectionRadius" << YAML::Value << m_detectionRadius;
	out << YAML::Key << "m_environmentLightIntensity" << YAML::Value << m_environmentLightIntensity;
	out << YAML::Key << "m_blurIteration" << YAML::Value << m_blurIteration;
}

void SimulationSettings::Deserialize(const YAML::Node& in)
{
	if (in["m_deltaTime"]) m_deltaTime = in["m_deltaTime"].as<float>();
	if (in["m_soilSimulation"]) m_soilSimulation = in["m_soilSimulation"].as<bool>();
	if (in["m_autoClearFruitAndLeaves"]) m_autoClearFruitAndLeaves = in["m_autoClearFruitAndLeaves"].as<bool>();
	if (in["m_crownShynessDistance"]) m_crownShynessDistance = in["m_crownShynessDistance"].as<float>();
	if (in["m_maxNodeCount"]) m_maxNodeCount = in["m_maxNodeCount"].as<int>();

	if (in["m_skylightIntensity"]) m_skylightIntensity = in["m_skylightIntensity"].as<float>();
	if (in["m_shadowDistanceLoss"]) m_shadowDistanceLoss = in["m_shadowDistanceLoss"].as<float>();
	if (in["m_detectionRadius"]) m_detectionRadius = in["m_detectionRadius"].as<float>();
	if (in["m_environmentLightIntensity"]) m_environmentLightIntensity = in["m_environmentLightIntensity"].as<float>();

	if (in["m_blurIteration"]) m_blurIteration = in["m_blurIteration"].as<int>();
}

bool SimulationSettings::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	bool changed = false;
	if (ImGui::Button("Grow weekly")) {
		m_deltaTime = 0.01918f;
		changed = true;
	}
	ImGui::SameLine();
	if (ImGui::Button("Grow monthly")) {
		m_deltaTime = 0.0822f;
		changed = true;
	}
	ImGui::SameLine();
	if(ImGui::DragFloat("Delta time", &m_deltaTime, 0.00001f, 0, 1, "%.5f")) changed = true;
	if (ImGui::Checkbox("Auto clear fruit and leaves", &m_autoClearFruitAndLeaves)) changed = true;
	if (ImGui::DragFloat("Crown shyness", &m_crownShynessDistance, 0.01f, 0.0f, 1.0f)) changed = true;
	if (ImGui::Checkbox("Simulate soil", &m_soilSimulation)) changed = true;
	if (ImGui::TreeNode("Lighting Estimation Settings")) {
		changed =
			ImGui::DragFloat("Skylight Intensity", &m_skylightIntensity, 0.01f,
				0.0f, 10.0f) || changed;
		changed =
			ImGui::DragFloat("Environmental Intensity", &m_environmentLightIntensity, 0.01f,
				0.0f, 10.0f) || changed;
		changed =
			ImGui::DragFloat("Shadow distance loss", &m_shadowDistanceLoss, 0.01f,
				0.0f, 10.0f) || changed;
		changed =
			ImGui::DragFloat("Detection radius", &m_detectionRadius, 0.001f,
				0.0f, 1.0f) || changed;
		changed =
			ImGui::DragInt("Blur iteration", &m_blurIteration, 1, 0, 10) || changed;
		
		ImGui::TreePop();
	}
	return changed;
}
