#include "TreePipeNode.hpp"
using namespace EcoSysLab;

void TreePipeNode::InsertInterpolation(const float a)
{
	const auto scene = GetScene();
	const auto owner = GetOwner();
	const auto parent = scene->GetParent(owner);
	const Entity middleEntity = scene->CreateEntity("Profile");
	const auto tpn = scene->GetOrSetPrivateComponent<TreePipeNode>(middleEntity).lock();

	tpn->m_apical = m_apical;
	const auto mmr = scene->GetOrSetPrivateComponent<MeshRenderer>(middleEntity).lock();
	mmr->m_mesh = Resources::GetResource<Mesh>("PRIMITIVE_SPHERE");
	mmr->m_material = scene->GetOrSetPrivateComponent<MeshRenderer>(owner).lock()->m_material;
	

	for (const auto& [pipeHandle, prevParticleHandle] : m_frontParticleMap)
	{
		const auto newFrontParticleHandle = tpn->m_frontParticlePhysics2D.AllocateParticle();
		auto& newFrontParticle = tpn->m_frontParticlePhysics2D.RefParticle(newFrontParticleHandle);
		tpn->m_frontParticleMap.insert({ pipeHandle, newFrontParticleHandle });

		newFrontParticle.m_data.m_pipeHandle = pipeHandle;
		const auto& prevParticle = m_frontParticlePhysics2D.PeekParticle(prevParticleHandle);
		const auto& nextParticle = m_backParticlePhysics2D.PeekParticle(m_backParticleMap.at(pipeHandle));
		newFrontParticle.SetPosition(glm::mix(prevParticle.GetPosition(), nextParticle.GetPosition(), a));
		newFrontParticle.SetColor(glm::mix(prevParticle.GetColor(), nextParticle.GetColor(), a));
		newFrontParticle.SetDamping(glm::mix(prevParticle.GetDamping(), nextParticle.GetDamping(), a));

		const auto newBackParticleHandle = tpn->m_backParticlePhysics2D.AllocateParticle();
		auto& newBackParticle = tpn->m_backParticlePhysics2D.RefParticle(newBackParticleHandle);
		tpn->m_backParticleMap.insert({ pipeHandle, newBackParticleHandle });

		newBackParticle.m_data.m_pipeHandle = pipeHandle;
		newBackParticle.SetPosition(glm::mix(prevParticle.GetPosition(), nextParticle.GetPosition(), a));
		newBackParticle.SetColor(glm::mix(prevParticle.GetColor(), nextParticle.GetColor(), a));
		newBackParticle.SetDamping(glm::mix(prevParticle.GetDamping(), nextParticle.GetDamping(), a));
	}

	const auto ownerGlobalTransform = scene->GetDataComponent<GlobalTransform>(owner);
	auto parentGlobalTransform = scene->GetDataComponent<GlobalTransform>(parent);
	
	GlobalTransform globalTransform;
	globalTransform.SetValue(glm::mix(ownerGlobalTransform.GetPosition(), parentGlobalTransform.GetPosition(), a),
		glm::mix(ownerGlobalTransform.GetEulerRotation(), scene->HasPrivateComponent<TreePipeNode>(parent) ? parentGlobalTransform.GetEulerRotation() : ownerGlobalTransform.GetEulerRotation(), a),
		glm::vec3(0.02f));

	scene->SetParent(middleEntity, parent);
	scene->SetParent(owner, middleEntity);
	scene->SetDataComponent(middleEntity, globalTransform);
	scene->SetDataComponent(owner, ownerGlobalTransform);

}

void TreePipeNode::OnDestroy()
{
	m_frontParticlePhysics2D = {};
	m_frontParticleMap = {};
	m_backParticlePhysics2D = {};
	m_backParticleMap = {};

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
	if (displayPhysics)
	{
		static float a = 0.0f;
		ImGui::DragFloat("Preset A", &a, 0.01f, 0.01f, 0.99f);
		if (ImGui::Button("Insert a"))
		{
			InsertInterpolation(a);
		}
		ImGui::SetNextWindowSize(ImVec2(300, 300), ImGuiCond_Appearing);
		const std::string frontTag = "Front Profile";
		if (ImGui::Begin(frontTag.c_str()))
		{
			m_frontParticlePhysics2D.OnInspect([&](const glm::vec2 position) {},
				[&](const ImVec2 origin, const float zoomFactor, ImDrawList* drawList) {},
				showGrid);
		}
		ImGui::End();
		const std::string backTag = "Back Profile";
		if (ImGui::Begin(backTag.c_str()))
		{
			m_backParticlePhysics2D.OnInspect([&](const glm::vec2 position) {},
				[&](const ImVec2 origin, const float zoomFactor, ImDrawList* drawList) {},
				showGrid);
		}
		ImGui::End();
	}
}
