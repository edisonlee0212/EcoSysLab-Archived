#include "SinglePipeProfile.hpp"

using namespace EcoSysLab;


void SinglePipeProfile::OnCreate()
{
	m_profileHandle = -1;
	m_pipeModelBase.Clear();
}

void SinglePipeProfile::OnDestroy()
{
	if(m_pipeModelBase.Get<PipeModelBase>() && m_profileHandle != -1)
	{
		m_pipeModelBase.Get<PipeModelBase>()->m_pipeModel.m_pipeProfileGroup.RecycleProfile(m_profileHandle);
	}
}

void SinglePipeProfile::Update()
{
	if (m_showProfile)
	{
		const auto pipeModelBase = m_pipeModelBase.Get<PipeModelBase>();
		if (!pipeModelBase) return;
		auto& profile = pipeModelBase->m_pipeModel.m_pipeProfileGroup.RefProfile(m_profileHandle);
		const std::string tag = "Profile [" + std::to_string(GetOwner().GetIndex()) + "]";
		if (ImGui::Begin(tag.c_str()))
		{
			if (profile.OnInspect(true))
			{
				EVOENGINE_LOG("Profile Updated.");
			}
		}
		ImGui::End();
	}
}

void SinglePipeProfile::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	static bool displayPlane = true;

	ImGui::Checkbox("Display Plane", &displayPlane);

	const auto scene = GetScene();
	const auto owner = GetOwner();

	if (displayPlane)
	{
		editorLayer->DrawGizmoMesh(Resources::GetResource<Mesh>("PRIMITIVE_QUAD"), glm::vec4(1.0, 1.0, 1.0, 0.5),
			scene->GetDataComponent<GlobalTransform>(owner).m_value);
	}
	ImGui::Checkbox("Show Profile", &m_showProfile);

	ImGui::Text(std::string("Profile Handle " + std::to_string(m_profileHandle)).c_str());
}

