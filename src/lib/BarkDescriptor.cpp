#include "BarkDescriptor.hpp"

using namespace EcoSysLab;

void BarkDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	ImGui::DragFloat("Bark X Frequency", &m_barkXFrequency, 0.1f, 0.0f, 100.0f);
	ImGui::DragFloat("Bark Y Frequency", &m_barkYFrequency, 0.1f, 0.0f, 100.0f);
	ImGui::DragFloat("Bark Depth", &m_barkDepth, 0.01f, 0.0f, 1.0f);

	ImGui::DragFloat("Base Frequency", &m_baseFrequency, 0.01f, 0.0f, 1.0f);
	ImGui::DragFloat("Base Max Distance", &m_baseMaxDistance, 0.01f, 0.0f, 1.0f);
	ImGui::DragFloat("Base Distance Decrease Factor", &m_baseDistanceDecreaseFactor, 0.01f, 0.0f, 1.0f);
	ImGui::DragFloat("Base Depth", &m_baseDepth, 0.01f, 0.0f, 1.0f);
}

float BarkDescriptor::GetValue(const float xFactor, const float distanceToRoot)
{
	float bark = m_barkDepth * 
		glm::perlin(glm::vec3(
			m_barkXFrequency * glm::sin(xFactor * 2.0f * glm::pi<float>()), 
			m_barkXFrequency * glm::cos(xFactor * 2.0f * glm::pi<float>()), 
			m_barkYFrequency * distanceToRoot));

	float base = m_baseDepth + m_baseDepth * glm::perlin(glm::vec3(
		m_baseFrequency * glm::sin(xFactor * 2.0f * glm::pi<float>()),
		m_baseFrequency * glm::cos(xFactor * 2.0f * glm::pi<float>()),
		0.0f));

	base *= glm::pow(glm::max(0.0f, (m_baseMaxDistance - distanceToRoot) / m_baseMaxDistance), m_baseDistanceDecreaseFactor);

	return bark + base;
}

void BarkDescriptor::Serialize(YAML::Emitter& out)
{
	out << YAML::Key << "m_barkXFrequency" << YAML::Value << m_barkXFrequency;
	out << YAML::Key << "m_barkYFrequency" << YAML::Value << m_barkYFrequency;
	out << YAML::Key << "m_barkDepth" << YAML::Value << m_barkDepth;
	out << YAML::Key << "m_baseFrequency" << YAML::Value << m_baseFrequency;
	out << YAML::Key << "m_baseMaxDistance" << YAML::Value << m_baseMaxDistance;
	out << YAML::Key << "m_baseDistanceDecreaseFactor" << YAML::Value << m_baseDistanceDecreaseFactor;
	out << YAML::Key << "m_baseDepth" << YAML::Value << m_baseDepth;
}

void BarkDescriptor::Deserialize(const YAML::Node& in)
{
	if (in["m_barkXFrequency"]) m_barkXFrequency = in["m_barkXFrequency"].as<float>();
	if (in["m_barkYFrequency"]) m_barkYFrequency = in["m_barkYFrequency"].as<float>();
	if (in["m_barkDepth"]) m_barkDepth = in["m_barkDepth"].as<float>();
	if (in["m_baseFrequency"]) m_baseFrequency = in["m_baseFrequency"].as<float>();
	if (in["m_baseMaxDistance"]) m_baseMaxDistance = in["m_baseMaxDistance"].as<float>();
	if (in["m_baseDistanceDecreaseFactor"]) m_baseDistanceDecreaseFactor = in["m_baseDistanceDecreaseFactor"].as<float>();
	if (in["m_baseDepth"]) m_baseDepth = in["m_baseDepth"].as<float>();

}
