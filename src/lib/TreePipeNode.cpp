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
	
	const auto prevProfile = m_profiles.at(prev);
	TreePipeProfile newProfile;
	newProfile.m_a = a;
	const auto nextProfile = m_profiles.at(prev + 2);

	const float actualA = (newProfile.m_a - prevProfile->m_a) / (nextProfile->m_a - prevProfile->m_a);
	for(const auto & [pipeHandle, prevParticleHandle] : prevProfile->m_particleMap)
	{
		const auto newParticleHandle = newProfile.m_particlePhysics2D.AllocateParticle();
		auto& newParticle = newProfile.m_particlePhysics2D.RefParticle(newParticleHandle);
		newProfile.m_particleMap.insert({ pipeHandle, newParticleHandle });
		
		newParticle.m_data.m_pipeHandle = pipeHandle;
		const auto& prevParticle = prevProfile->m_particlePhysics2D.PeekParticle(prevParticleHandle);
		const auto& nextParticle = nextProfile->m_particlePhysics2D.PeekParticle(nextProfile->m_particleMap.at(pipeHandle));
		newParticle.SetPosition(glm::mix(prevParticle.GetPosition(), nextParticle.GetPosition(), actualA));
		newParticle.SetColor(glm::mix(prevParticle.GetColor(), nextParticle.GetColor(), actualA));
		newParticle.SetDamping(glm::mix(prevParticle.GetDamping(), nextParticle.GetDamping(), actualA));
	}
	/*
	auto scene = GetScene();
	Entity middleEntity = scene->CreateEntity("Profile");
	auto tpn = scene->GetOrSetPrivateComponent<TreePipeNode>(middleEntity).lock();
	tpn->m_profiles.emplace_back(std::make_shared<TreePipeProfile>());
	tpn->m_profiles.back()->m_a = 0.0f;
	tpn->m_profiles.emplace_back(std::make_shared<TreePipeProfile>());
	tpn->m_profiles.back()->m_a = 1.0f;

	tpn->m_apical = flow.IsApical();
	auto mmr = scene->GetOrSetPrivateComponent<MeshRenderer>(newEntity).lock();
	mmr->m_mesh = Resources::GetResource<Mesh>("PRIMITIVE_SPHERE");
	mmr->m_material = nodeMaterial;
	nodeMaterial->m_materialProperties.m_albedoColor = glm::vec3(1, 0, 0);
	nodeMaterial->m_materialProperties.m_transmission = 0.5f;

	GlobalTransform globalTransform;
	const glm::quat rotation = lastNode.m_info.m_regulatedGlobalRotation;
	globalTransform.m_value =
		ownerGlobalTransform.m_value
		* (glm::translate(flow.m_info.m_globalEndPosition) * glm::mat4_cast(rotation) * glm::scale(glm::vec3(lastNode.m_info.m_thickness * 5.0f)));
	scene->SetDataComponent(newEntity, globalTransform);

	if (parentHandle == -1)
	{
		scene->SetParent(newEntity, owner);
	}
	else
	{
		scene->SetParent(newEntity, flowMap.at(parentHandle));
	}*/
}

void TreePipeNode::OnDestroy()
{
	m_profiles.clear();
	m_pipeHandle = -1;
	m_frontControlPointDistance = 0.0f;
	m_backControlPointDistance = 0.0f;
	m_centerDirectionRadius = 0.0f;
	m_apical = false;
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
