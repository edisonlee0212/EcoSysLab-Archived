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
		const std::string startTag = "Start Profile";
		if (ImGui::Begin(startTag.c_str()))
		{
			m_startParticlePhysics2D.OnInspect([&](const glm::vec2 position) {},
				[&](const ImVec2 origin, const float zoomFactor, ImDrawList* drawList) {},
				showGrid);
		}
		ImGui::End();
		const std::string endTag = "End Profile";
		if (ImGui::Begin(endTag.c_str()))
		{
			m_endParticlePhysics2D.OnInspect([&](const glm::vec2 position) {},
				[&](const ImVec2 origin, const float zoomFactor, ImDrawList* drawList) {},
				showGrid);
		}
		ImGui::End();
	}
}
