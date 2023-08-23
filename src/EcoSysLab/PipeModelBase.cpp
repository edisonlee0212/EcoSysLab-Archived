#include "PipeModelBase.hpp"

#include "SinglePipeProfile.hpp"

using namespace EcoSysLab;

void PipeModelBase::Update()
{
	if (m_showProfile)
	{
		auto& profile = m_pipeModel.m_pipeProfileGroup.RefProfile(m_baseProfileHandle);
		const std::string tag = "Base Profile";
		if (ImGui::Begin(tag.c_str()))
		{
			if (profile.OnInspect(true))
			{
				EVOENGINE_LOG("Base Profile Updated.");
			}
		}
		ImGui::End();
	}
}

void PipeModelBase::AssignProfiles()
{
	const auto scene = GetScene();
	const auto owner = GetOwner();
	scene->ForEachDescendant(owner, [&](const Entity& entity)
		{
			if (scene->HasPrivateComponent<SinglePipeProfile>(entity))
			{
				const auto singlePipeProfile = scene->GetOrSetPrivateComponent<SinglePipeProfile>(entity).lock();
				if (singlePipeProfile->m_profileHandle == -1)
				{
					singlePipeProfile->m_pipeModelBase = scene->GetOrSetPrivateComponent<PipeModelBase>(owner).lock();
					singlePipeProfile->m_profileHandle = m_pipeModel.m_pipeProfileGroup.Allocate();
				}
			}
		});
}

void PipeModelBase::EstablishPipes()
{
	AssignProfiles();
	const auto scene = GetScene();
	const auto owner = GetOwner();

	m_pipeModel.m_skeleton = {};
	m_pipeModel.m_pipeGroup = {};

	m_pipeModel.m_skeleton.m_data.m_baseProfileHandle = m_baseProfileHandle;
	m_pipeModel.m_pipeProfileGroup.RefProfile(m_baseProfileHandle).m_data.m_nodeHandle = -1;

	
	scene->ForEachDescendant(owner, [&](const Entity& entity)
		{
			const auto parent = scene->GetParent(entity);
			if (scene->HasPrivateComponent<SinglePipeProfile>(entity)) {
				const auto singlePipeProfile = scene->GetOrSetPrivateComponent<SinglePipeProfile>(entity).lock();
				auto& profile = m_pipeModel.m_pipeProfileGroup.RefProfile(singlePipeProfile->m_profileHandle);
				if (scene->HasPrivateComponent<SinglePipeProfile>(parent))
				{
					const auto parentSinglePipeProfile = scene->GetOrSetPrivateComponent<SinglePipeProfile>(parent).lock();
					auto& parentProfile = m_pipeModel.m_pipeProfileGroup.RefProfile(parentSinglePipeProfile->m_profileHandle);
					NodeHandle parentNodeHandle = parentProfile.m_data.m_nodeHandle;
					if (parentNodeHandle == -1) return;
					profile.m_data.m_nodeHandle = m_pipeModel.m_skeleton.Extend(parentNodeHandle, true);
					auto& node = m_pipeModel.m_skeleton.RefNode(profile.m_data.m_nodeHandle);

					const auto transform = scene->GetDataComponent<Transform>(entity);
					const auto localPosition = transform.GetPosition();
					node.m_info.m_length = glm::length(localPosition);
					const auto front = glm::normalize(localPosition);
					node.m_info.m_localRotation = glm::quatLookAt(front, glm::vec3(front.y, front.z, front.x));

					node.m_data.m_profileHandle = singlePipeProfile->m_profileHandle;
				}
				else if (scene->HasPrivateComponent<PipeModelBase>(parent))
				{
					//There can only be one entity for this.
					const auto parentPipeModelBase = scene->GetOrSetPrivateComponent<PipeModelBase>(parent).lock();
					profile.m_data.m_nodeHandle = 0;
					auto& node = m_pipeModel.m_skeleton.RefNode(profile.m_data.m_nodeHandle);

					const auto transform = scene->GetDataComponent<Transform>(entity);
					const auto localPosition = transform.GetPosition();
					node.m_info.m_length = glm::length(localPosition);

					node.m_data.m_profileHandle = singlePipeProfile->m_profileHandle;
				}
			}
		}
	);
	m_pipeModel.m_skeleton.SortLists();
	m_pipeModel.m_skeleton.CalculateTransforms();
	m_pipeModel.m_skeleton.CalculateFlows();

	m_pipeModel.InitializePipes(m_pipeModelParameters);
}

void PipeModelBase::InitializeStrandRenderer() const
{
	const auto scene = GetScene();
	const auto owner = GetOwner();

	ClearStrands();
	const auto strandsEntity = scene->CreateEntity("Branch Strands");
	scene->SetParent(strandsEntity, owner);

	const auto renderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(strandsEntity).lock();
	const auto strandsAsset = ProjectManager::CreateTemporaryAsset<Strands>();
	std::vector<glm::uint> strandsList;
	std::vector<StrandPoint> points;
	m_pipeModel.m_pipeGroup.BuildStrands(strandsList, points);
	if (!points.empty()) strandsList.emplace_back(points.size());
	StrandPointAttributes strandPointAttributes{};
	strandPointAttributes.m_color = true;
	strandsAsset->SetStrands(strandPointAttributes, strandsList, points);
	renderer->m_strands = strandsAsset;

	const auto material = ProjectManager::CreateTemporaryAsset<Material>();
	renderer->m_material = material;
	material->m_vertexColorOnly = true;
}

void PipeModelBase::ClearStrands() const
{
	const auto scene = GetScene();
	const auto self = GetOwner();
	const auto children = scene->GetChildren(self);
	for (const auto& child : children) {
		auto name = scene->GetEntityName(child);
		if (name == "Branch Strands") {
			scene->DeleteEntity(child);
		}
		else if (name == "Root Strands") {
			scene->DeleteEntity(child);
		}
	}
}

void PipeModelBase::OnCreate()
{
	m_pipeModel = {};
	m_baseProfileHandle = m_pipeModel.m_pipeProfileGroup.Allocate();
}

void PipeModelBase::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	const auto scene = GetScene();
	const auto owner = GetOwner();

	ImGui::DragFloat("Pipe Model Profile Scale", &m_pipeModelParameters.m_profileScale);

	ImGui::Checkbox("Show Profile", &m_showProfile);

	if (ImGui::Button("Establish pipes"))
	{
		EstablishPipes();
	}

	if(ImGui::Button("Initialize Basic Y Shape"))
	{
		const auto children = scene->GetChildren(owner);
		for (const auto& i : children)
		{
			scene->DeleteEntity(i);
		}
		const auto centerChild = scene->CreateEntity("Center child");

		const auto leftChild = scene->CreateEntity("Left child");
		const auto rightChild = scene->CreateEntity("Right child");
		auto transform = Transform();

		scene->SetParent(centerChild, owner);
		scene->SetParent(leftChild, centerChild);
		scene->SetParent(rightChild, centerChild);
		transform.SetPosition(glm::vec3(0, 0, -2));
		scene->SetDataComponent(centerChild, transform);
		transform.SetPosition(glm::vec3(-1, 0, -4));
		scene->SetDataComponent(leftChild, transform);
		transform.SetPosition(glm::vec3(1, 0, -4));
		scene->SetDataComponent(rightChild, transform);
		
		scene->GetOrSetPrivateComponent<SinglePipeProfile>(centerChild);
		scene->GetOrSetPrivateComponent<SinglePipeProfile>(leftChild);
		scene->GetOrSetPrivateComponent<SinglePipeProfile>(rightChild);

		AssignProfiles();
	}

	if(ImGui::Button("Initialize Strands"))
	{
		InitializeStrandRenderer();
	}
}
