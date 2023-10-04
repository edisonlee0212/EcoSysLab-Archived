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

					const auto parentGlobalTransform = scene->GetDataComponent<GlobalTransform>(parent);
					const auto globalTransform = scene->GetDataComponent<GlobalTransform>(entity);
					node.m_info.m_globalPosition = parentGlobalTransform.GetPosition();
					node.m_info.m_globalRotation = parentGlobalTransform.GetRotation() * glm::quat(glm::radians(glm::vec3(90, 0 ,0)));
					node.m_data.m_globalEndRotation = globalTransform.GetRotation() * glm::quat(glm::radians(glm::vec3(90, 0, 0)));
					node.m_info.m_length = glm::distance(parentGlobalTransform.GetPosition(), globalTransform.GetPosition());
					node.m_info.m_globalDirection = glm::normalize(globalTransform.GetPosition() - parentGlobalTransform.GetPosition());

					node.m_data.m_profileHandle = singlePipeProfile->m_profileHandle;
				}
				else if (scene->HasPrivateComponent<PipeModelBase>(parent))
				{
					//There can only be one entity for this.
					const auto parentPipeModelBase = scene->GetOrSetPrivateComponent<PipeModelBase>(parent).lock();
					profile.m_data.m_nodeHandle = 0;
					auto& node = m_pipeModel.m_skeleton.RefNode(profile.m_data.m_nodeHandle);

					const auto parentGlobalTransform = scene->GetDataComponent<GlobalTransform>(parent);
					const auto globalTransform = scene->GetDataComponent<GlobalTransform>(entity);
					node.m_info.m_globalPosition = parentGlobalTransform.GetPosition();
					node.m_info.m_globalRotation = parentGlobalTransform.GetRotation() * glm::quat(glm::radians(glm::vec3(90, 0, 0)));
					node.m_data.m_globalEndRotation = globalTransform.GetRotation() * glm::quat(glm::radians(glm::vec3(90, 0, 0)));
					node.m_info.m_length = glm::distance(parentGlobalTransform.GetPosition(), globalTransform.GetPosition());
					node.m_info.m_globalDirection = glm::normalize(globalTransform.GetPosition() - parentGlobalTransform.GetPosition());

					node.m_data.m_profileHandle = singlePipeProfile->m_profileHandle;
				}
			}
		}
	);
	m_pipeModel.m_skeleton.SortLists();
	m_pipeModel.m_skeleton.CalculateRegulatedGlobalRotation();

	for (const auto& nodeHandle : m_pipeModel.m_skeleton.RefSortedNodeList()) {
		auto& node = m_pipeModel.m_skeleton.RefNode(nodeHandle);
		auto& nodeData = node.m_data;
		if (node.GetParentHandle() != -1) {
			auto& parentData = m_pipeModel.m_skeleton.RefNode(node.GetParentHandle()).m_data;
			auto front = nodeData.m_globalEndRotation * glm::vec3(0, 0, -1);
			auto parentRegulatedUp = parentData.m_regulatedGlobalEndRotation * glm::vec3(0, 1, 0);
			auto regulatedUp = glm::normalize(glm::cross(glm::cross(front, parentRegulatedUp), front));
			nodeData.m_regulatedGlobalEndRotation = glm::quatLookAt(front, regulatedUp);
		}
		else
		{
			nodeData.m_regulatedGlobalEndRotation = nodeData.m_globalEndRotation;
		}
	}

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
	strandPointAttributes.m_normal = true;
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

	ImGui::DragFloat("Default profile cell radius", &m_pipeModelParameters.m_profileDefaultCellRadius);

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
		transform.SetPosition(glm::vec3(0, 2, 0));
		scene->SetDataComponent(centerChild, transform);
		transform.SetPosition(glm::vec3(-1, 4, 0));
		scene->SetDataComponent(leftChild, transform);
		transform.SetPosition(glm::vec3(1, 4, 0));
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
