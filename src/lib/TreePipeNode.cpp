#include "TreePipeNode.hpp"
using namespace EcoSysLab;

void TreePipeNode::InsertInterpolation(const float a)
{
	size_t prev = 0;
	for(int i = 0; i < m_profiles.size() - 1; i++)
	{
		if(m_profiles[i]->m_a < a)
		{
			prev = i;
		}else if(m_profiles[i]->m_a == a)
		{
			EVOENGINE_ERROR("Unable to insert with existing a");
			return;
		}else
		{
			break;
		}
	}
	m_profiles.insert(m_profiles.begin() + prev + 1, std::make_shared<TreePipeProfile>());
	const auto prevProfile = m_profiles.at(prev);
	const auto newProfile = m_profiles.at(prev + 1);
	newProfile->m_a = a;
	const auto nextProfile = m_profiles.at(prev + 2);

	const float actualA = (newProfile->m_a - prevProfile->m_a) / (nextProfile->m_a - prevProfile->m_a);
	newProfile->m_offset = glm::mix(prevProfile->m_offset, nextProfile->m_offset, actualA);
	//newProfile->m_profileTransform = glm::mix(prevProfile->m_profileTransform, nextProfile->m_profileTransform, actualA);
	for(const auto & [pipeHandle, prevParticleHandle] : prevProfile->m_particleMap)
	{
		const auto newParticleHandle = newProfile->m_particlePhysics2D.AllocateParticle();
		auto& newParticle = newProfile->m_particlePhysics2D.RefParticle(newParticleHandle);
		newProfile->m_particleMap.insert({ pipeHandle, newParticleHandle });
		
		newParticle.m_data.m_pipeHandle = pipeHandle;
		const auto& prevParticle = prevProfile->m_particlePhysics2D.PeekParticle(prevParticleHandle);
		const auto& nextParticle = nextProfile->m_particlePhysics2D.PeekParticle(nextProfile->m_particleMap.at(pipeHandle));
		newParticle.SetPosition(glm::mix(prevParticle.GetPosition(), nextParticle.GetPosition(), actualA));
		newParticle.SetColor(glm::mix(prevParticle.GetColor(), nextParticle.GetColor(), actualA));
		newParticle.SetDamping(glm::mix(prevParticle.GetDamping(), nextParticle.GetDamping(), actualA));
	}
}

void TreePipeNode::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	static bool displayPhysics = true;
	static bool showGrid = false;
	ImGui::Checkbox("Show Physics", &displayPhysics);
	if (displayPhysics) ImGui::Checkbox("Show Grid", &showGrid);
	if (displayPhysics && !m_profiles.empty())
	{
		static float a = 0.0f;
		ImGui::DragFloat("Preset A", &a, 0.01f, 0.01f, 0.99f);
		if (ImGui::Button("Insert a"))
		{
			InsertInterpolation(a);
		}
		static int profileIndex = 0;
		if (profileIndex != 0 && profileIndex != m_profiles.size() - 1 && ImGui::Button("Remove current profile")) {
			m_profiles.erase(m_profiles.begin() + profileIndex);
		}
		ImGui::SetNextWindowSize(ImVec2(300, 300), ImGuiCond_Appearing);
		const std::string startTag = "Profile";
		
		ImGui::SliderInt("Profile Index", &profileIndex, 0, static_cast<int>(m_profiles.size()) - 1);
		profileIndex = glm::clamp(profileIndex, 0, static_cast<int>(m_profiles.size()) - 1);
		
		if (ImGui::Begin(startTag.c_str()))
		{
			m_profiles.at(profileIndex)->m_particlePhysics2D.OnInspect([&](const glm::vec2 position) {},
				[&](const ImVec2 origin, const float zoomFactor, ImDrawList* drawList) {},
				showGrid);
		}
		ImGui::End();
	}
}
