#include "TreePipeNode.hpp"
using namespace EcoSysLab;

void TreePipeNode::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	static bool displayPhysics = true;

	static bool showGrid = false;
	ImGui::Checkbox("Show Physics", &displayPhysics);
	if (displayPhysics) ImGui::Checkbox("Show Grid", &showGrid);
	if (displayPhysics)
	{
		ImGui::SetNextWindowSize(ImVec2(300, 300), ImGuiCond_Appearing);
		const std::string startTag = "Profile";
		static int profileIndex = 0;
		ImGui::SliderInt("Profile Index", &profileIndex, 0, m_profiles.size());
		profileIndex = glm::clamp(profileIndex, 0, static_cast<int>(m_profiles.size()));
		if (ImGui::Begin(startTag.c_str()))
		{
			m_profiles.at(profileIndex)->m_particlePhysics2D.OnInspect([&](const glm::vec2 position) {},
				[&](const ImVec2 origin, const float zoomFactor, ImDrawList* drawList) {},
				showGrid);
		}
		ImGui::End();
	}
}
