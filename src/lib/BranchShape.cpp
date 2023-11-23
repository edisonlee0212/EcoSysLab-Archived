#include "BranchShape.hpp"

using namespace EcoSysLab;

void BranchShape::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	ImGui::DragFloat("Bark X Frequency", &m_barkXFrequency, 0.1f, 0.0f, 100.0f);
	ImGui::DragFloat("Bark Y Frequency", &m_barkYFrequency, 0.1f, 0.0f, 100.0f);
	ImGui::DragFloat("Bark Depth", &m_barkDepth, 0.01f, 0.0f, 1.0f);

	ImGui::DragFloat("Base Frequency", &m_baseFrequency, 0.01f, 0.0f, 1.0f);
	ImGui::DragFloat("Base Max Distance", &m_baseMaxDistance, 0.01f, 0.0f, 1.0f);
	ImGui::DragFloat("Base Distance Decrease Factor", &m_baseDistanceDecreaseFactor, 0.01f, 0.0f, 1.0f);
	ImGui::DragFloat("Base Depth", &m_baseDepth, 0.01f, 0.0f, 1.0f);
}

float BranchShape::GetValue(const float xFactor, const float distanceToRoot)
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

	return 1.0f + bark + base;
}
