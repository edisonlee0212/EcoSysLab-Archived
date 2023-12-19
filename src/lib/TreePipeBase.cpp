#include "TreePipeBase.hpp"
#include "Tree.hpp"
#include "TransformGraph.hpp"
using namespace EcoSysLab;

void TreePipeBase::GatherChildrenEntities(std::vector<Entity>& list) const
{
	std::queue<Entity> entityQueue{};
	const auto scene = GetScene();
	const auto owner = GetOwner();
	for (const auto& i : scene->GetChildren(owner))
	{
		if (scene->HasPrivateComponent<TreePipeNode>(i))
		{
			entityQueue.push(i);
			list.push_back(i);
		}
	}
	while (!entityQueue.empty())
	{
		auto nextEntity = entityQueue.front();
		entityQueue.pop();
		const auto& children = scene->GetChildren(nextEntity);
		for (const auto& i : children)
		{
			if (scene->HasPrivateComponent<TreePipeNode>(i))
			{
				entityQueue.push(i);
				list.push_back(i);
			}
		}
	}
}

void TreePipeBase::ExtendPipesWithProfile(const glm::vec3& globalPosition,
	const glm::quat& globalRotation, const ParticlePhysics2D<CellParticlePhysicsData>& profile, const std::unordered_map<PipeHandle, ParticleHandle>& map)
{
	const auto currentUp = globalRotation * glm::vec3(0, 1, 0);
	const auto currentLeft = globalRotation * glm::vec3(1, 0, 0);
	for (const auto& [pipeHandle, particleHandle] : map)
	{
		const auto& particle = profile.PeekParticle(particleHandle);
		const auto newPipeSegmentHandle = m_pipeGroup.Extend(pipeHandle);
		auto& newPipeSegment = m_pipeGroup.RefPipeSegment(newPipeSegmentHandle);
		newPipeSegment.m_info.m_thickness = m_pipeModelParameters.m_profileDefaultCellRadius;
		newPipeSegment.m_info.m_globalPosition = globalPosition
			+ m_pipeModelParameters.m_profileDefaultCellRadius * particle.GetPosition().x * currentLeft
			+ m_pipeModelParameters.m_profileDefaultCellRadius * particle.GetPosition().y * currentUp;
		newPipeSegment.m_info.m_globalRotation = globalRotation;
	}
}

void TreePipeBase::InstantiateExample()
{
	m_skeleton = {};
	const auto trunkHandle = m_skeleton.Extend(0, false);
	const auto handle1 = m_skeleton.Extend(trunkHandle, false);
	const auto handle2 = m_skeleton.Extend(trunkHandle, true);
	const auto handle3 = m_skeleton.Extend(trunkHandle, true);
	
	const auto handle4 = m_skeleton.Extend(handle1, false);
	const auto handle5 = m_skeleton.Extend(handle2, true);
	const auto handle6 = m_skeleton.Extend(handle3, true);
	
	//Position, length, globalRotation
	auto& baseNode = m_skeleton.RefNode(0);
	auto& trunkNode = m_skeleton.RefNode(trunkHandle);
	auto& node1 = m_skeleton.RefNode(handle1);
	auto& node2 = m_skeleton.RefNode(handle2);
	auto& node3 = m_skeleton.RefNode(handle3);
	
	auto& node4 = m_skeleton.RefNode(handle4);
	auto& node5 = m_skeleton.RefNode(handle5);
	auto& node6 = m_skeleton.RefNode(handle6);
	
	baseNode.m_info.m_globalRotation = baseNode.m_info.m_regulatedGlobalRotation = glm::quatLookAt(glm::vec3(0, 1, 0), glm::vec3(0, 0, 1));
	baseNode.m_info.m_globalPosition = glm::vec3(0.0f);
	baseNode.m_info.m_length = 0.5f;

	trunkNode.m_info.m_globalRotation = trunkNode.m_info.m_regulatedGlobalRotation = glm::quatLookAt(glm::vec3(0, 1, 0), glm::vec3(0, 0, 1));
	trunkNode.m_info.m_globalPosition = glm::vec3(0.0f, 0.5f, 0.0f);
	trunkNode.m_info.m_length = 0.5f;

	node1.m_info.m_globalRotation = node1.m_info.m_regulatedGlobalRotation = glm::quatLookAt(glm::vec3(0, 1, 0), glm::vec3(0, 0, 1));
	node1.m_info.m_globalPosition = glm::vec3(0.0f, 1.f, 0.0f);
	node1.m_info.m_length = 0.5f;

	node2.m_info.m_globalRotation = node2.m_info.m_regulatedGlobalRotation = glm::quatLookAt(glm::vec3(1, 0, 0), glm::vec3(0, 0, 1));
	node2.m_info.m_globalPosition = glm::vec3(0.0f, 1.f, 0.0f);
	node2.m_info.m_length = 0.1f;
	
	node3.m_info.m_globalRotation = node3.m_info.m_regulatedGlobalRotation = glm::quatLookAt(glm::normalize(glm::vec3(-1, 1, 0)), glm::vec3(0, 0, 1));
	node3.m_info.m_globalPosition = glm::vec3(0.0f, 1.f, 0.0f);
	node3.m_info.m_length = 0.1f;
	
	
	node4.m_info.m_globalRotation = node4.m_info.m_regulatedGlobalRotation = glm::quatLookAt(glm::vec3(0, 1, 0), glm::vec3(0, 0, 1));
	node4.m_info.m_globalPosition = glm::vec3(0.0f, 1.1f, 0.0f);
	node4.m_info.m_length = 0.1f;

	node5.m_info.m_globalRotation = node5.m_info.m_regulatedGlobalRotation = glm::quatLookAt(glm::vec3(1, 0, 0), glm::vec3(0, 0, 1));
	node5.m_info.m_globalPosition = glm::vec3(0.1f, 1.f, 0.0f);
	node5.m_info.m_length = 0.1f;

	node6.m_info.m_globalRotation = node6.m_info.m_regulatedGlobalRotation = glm::quatLookAt(glm::normalize(glm::vec3(-1, 1, 0)), glm::vec3(0, 0, 1));
	node6.m_info.m_globalPosition = glm::vec3(0.0f, 1.f, 0.0f) + glm::normalize(glm::vec3(-1, 1, 0)) * 0.1f;
	node6.m_info.m_length = 0.1f;
	
	m_skeleton.SortLists();
	m_skeleton.CalculateFlows();
}

void TreePipeBase::ClearStrands() const
{
	const auto scene = GetScene();
	const auto self = GetOwner();
	const auto children = scene->GetChildren(self);
	for (const auto& child : children) {
		auto name = scene->GetEntityName(child);
		if (name == "Branch Strands") {
			scene->DeleteEntity(child);
		}
	}
}

void TreePipeBase::InitializeStrandRenderer(const float frontControlPointRatio, const float backControlPointRatio, int nodeMaxCount)
{
	const auto scene = GetScene();
	const auto owner = GetOwner();

	ClearStrands();
	const auto strandsEntity = scene->CreateEntity("Branch Strands");
	scene->SetParent(strandsEntity, owner);

	const auto renderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(strandsEntity).lock();
	const auto strandsAsset = ProjectManager::CreateTemporaryAsset<Strands>();

	BuildPipes();

	std::vector<glm::uint> strandsList;
	std::vector<StrandPoint> points;
	m_pipeGroup.BuildStrands(frontControlPointRatio, backControlPointRatio, strandsList, points, nodeMaxCount);
	if (!points.empty()) strandsList.emplace_back(points.size());
	StrandPointAttributes strandPointAttributes{};
	strandPointAttributes.m_color = true;
	strandsAsset->SetStrands(strandPointAttributes, strandsList, points);
	renderer->m_strands = strandsAsset;

	const auto material = ProjectManager::CreateTemporaryAsset<Material>();

	renderer->m_material = material;
	material->m_vertexColorOnly = true;
	material->m_materialProperties.m_albedoColor = glm::vec3(0.6f, 0.3f, 0.0f);
}

void TreePipeBase::OnCreate()
{
	const auto material = ProjectManager::CreateTemporaryAsset<Material>();
	m_nodeMaterial = material;
	material->m_materialProperties.m_albedoColor = glm::vec3(1, 0, 0);
	material->m_materialProperties.m_transmission = 0.5f;
	const auto mesh = Resources::GetResource<Mesh>("PRIMITIVE_SPHERE");
	m_nodeMesh = mesh;

}

void TreePipeBase::CalculateProfiles()
{
	float time = Times::Now();
	std::vector<Entity> sortedEntityList;
	GatherChildrenEntities(sortedEntityList);
	const auto scene = GetScene();
	m_pipeGroup = {};
	auto& pipeGroup = m_pipeGroup;

	for (int sortedEntityIndex = 0; sortedEntityIndex < sortedEntityList.size(); sortedEntityIndex++)
	{
		const auto& entity = sortedEntityList[sortedEntityIndex];
		const auto node = scene->GetOrSetPrivateComponent<TreePipeNode>(entity).lock();
		auto& frontPhysics2D = node->m_frontParticlePhysics2D;
		auto& backPhysics2D = node->m_backParticlePhysics2D;
		
		frontPhysics2D.Reset(0.002f);
		backPhysics2D.Reset(0.002f);
		frontPhysics2D.m_settings = m_particlePhysicsSettings;
		backPhysics2D.m_settings = m_particlePhysicsSettings;

		Entity parentEntity{};
		bool onlyChild = true;
		if (sortedEntityIndex != 0)
		{
			parentEntity = scene->GetParent(entity);
			auto siblingsEntities = scene->GetChildren(parentEntity);
			for (auto siblingsEntity : siblingsEntities)
			{
				if (siblingsEntity.GetIndex() != entity.GetIndex() && scene->HasPrivateComponent<TreePipeNode>(
					siblingsEntity))
				{
					onlyChild = false;
				}
			}
		}
		if (node->m_apical || onlyChild)
		{
			if (sortedEntityIndex == 0)
			{
				for (int i = 0; i < m_pipeModelParameters.m_endNodeStrands; i++) {
					const auto newPipeHandle = pipeGroup.AllocatePipe();

					const auto newStartParticleHandle = frontPhysics2D.AllocateParticle();
					auto& newStartParticle = frontPhysics2D.RefParticle(newStartParticleHandle);
					newStartParticle.m_data.m_pipeHandle = newPipeHandle;
					const auto newEndParticleHandle = backPhysics2D.AllocateParticle();
					auto& newEndParticle = backPhysics2D.RefParticle(newEndParticleHandle);
					newEndParticle.m_data.m_pipeHandle = newPipeHandle;
				}
			}
			else
			{
				const auto parentNode = scene->GetOrSetPrivateComponent<TreePipeNode>(parentEntity).lock();
				for (const auto& cell : parentNode->m_frontParticlePhysics2D.PeekParticles())
				{
					const auto newStartParticleHandle = frontPhysics2D.AllocateParticle();
					auto& newStartParticle = frontPhysics2D.RefParticle(newStartParticleHandle);
					newStartParticle.m_data.m_pipeHandle = cell.m_data.m_pipeHandle;

					const auto newEndParticleHandle = backPhysics2D.AllocateParticle();
					auto& newEndParticle = backPhysics2D.RefParticle(newEndParticleHandle);
					newEndParticle.m_data.m_pipeHandle = cell.m_data.m_pipeHandle;
				}
			}
		}
		else
		{
			//If this node is formed from branching, we need to construct new pipe.
			//First, we need to collect a chain of nodes from current node to the root.
			const auto parentNode = scene->GetOrSetPrivateComponent<TreePipeNode>(parentEntity).lock();
			std::vector<std::shared_ptr<TreePipeNode>> parentNodeToRootChain;
			Entity hierarchyWalkerEntity = parentEntity;
			while (scene->HasPrivateComponent<TreePipeNode>(hierarchyWalkerEntity))
			{
				parentNodeToRootChain.emplace_back(scene->GetOrSetPrivateComponent<TreePipeNode>(hierarchyWalkerEntity));
				hierarchyWalkerEntity = scene->GetParent(hierarchyWalkerEntity);
			}
			for (int i = 0; i < m_pipeModelParameters.m_endNodeStrands; i++) {
				const auto pipeHandle = pipeGroup.AllocatePipe();
				for (auto it = parentNodeToRootChain.rbegin(); it != parentNodeToRootChain.rend(); ++it) {

					const auto newStartParticleHandle = (*it)->m_frontParticlePhysics2D.AllocateParticle();
					auto& newStartParticle = (*it)->m_frontParticlePhysics2D.RefParticle(newStartParticleHandle);
					newStartParticle.m_data.m_pipeHandle = pipeHandle;

					const auto newEndParticleHandle = (*it)->m_backParticlePhysics2D.AllocateParticle();
					auto& newEndParticle = (*it)->m_backParticlePhysics2D.RefParticle(newEndParticleHandle);
					newEndParticle.m_data.m_pipeHandle = pipeHandle;
				}
				const auto newStartParticleHandle = frontPhysics2D.AllocateParticle();
				auto& newStartParticle = frontPhysics2D.RefParticle(newStartParticleHandle);
				newStartParticle.m_data.m_pipeHandle = pipeHandle;

				const auto newEndParticleHandle = backPhysics2D.AllocateParticle();
				auto& newEndParticle = backPhysics2D.RefParticle(newEndParticleHandle);
				newEndParticle.m_data.m_pipeHandle = pipeHandle;
			}
		}
	}

	for (const auto& entity : sortedEntityList)
	{
		const auto node = scene->GetOrSetPrivateComponent<TreePipeNode>(entity).lock();
		node->m_frontParticleMap.clear();
		node->m_backParticleMap.clear();
		auto& startPhysics2D = node->m_frontParticlePhysics2D;

		for (auto& particle : startPhysics2D.RefParticles())
		{
			node->m_frontParticleMap.insert({ particle.m_data.m_pipeHandle, particle.GetHandle() });
		}
		auto& endPhysics2D = node->m_backParticlePhysics2D;

		for (auto& particle : endPhysics2D.RefParticles())
		{
			node->m_backParticleMap.insert({ particle.m_data.m_pipeHandle, particle.GetHandle() });
		}
	}
	for (const auto& entity : sortedEntityList)
	{
		const auto node = scene->GetOrSetPrivateComponent<TreePipeNode>(entity).lock();
		const auto childrenEntities = scene->GetChildren(entity);
		std::vector<std::shared_ptr<TreePipeNode>> childrenNodes;

		std::shared_ptr<TreePipeNode> mainChildNode{};
		for (auto childEntity : childrenEntities)
		{
			if (scene->HasPrivateComponent<TreePipeNode>(childEntity))
			{
				const auto childNode = scene->GetOrSetPrivateComponent<TreePipeNode>(childEntity).lock();
				if (childNode->m_apical) mainChildNode = childNode;
				childrenNodes.push_back(childNode);
			}
		}
		for (const auto& childNode : childrenNodes)
		{
			if (childNode == mainChildNode) childNode->m_split = false;
			const auto childSize = static_cast<float>(childNode->m_frontParticleMap.size());
			const auto totalSize = static_cast<float>(node->m_frontParticleMap.size());
			if (childSize > totalSize * m_pipeModelParameters.m_splitRatioLimit)
			{
				childNode->m_split = true;
			}
			else
			{
				childNode->m_split = false;
			}
		}
	}
	m_numOfProfiles = 0;
	m_numOfParticles = 0;
	m_numOfProfiles = sortedEntityList.size();
	if (m_parallelScheduling) {
		for (auto it = sortedEntityList.rbegin(); it != sortedEntityList.rend(); ++it)
		{
			const auto node = scene->GetOrSetPrivateComponent<TreePipeNode>(*it).lock();
			node->CalculateProfile(m_pipeModelParameters, true);
			m_numOfParticles += node->m_frontParticlePhysics2D.PeekParticles().size();
		}
		if (!sortedEntityList.empty())
		{
			auto firstEntity = sortedEntityList.front();
			const auto node = scene->GetOrSetPrivateComponent<TreePipeNode>(firstEntity).lock();
			node->Wait();
		}
	}else
	{
		for (auto it = sortedEntityList.rbegin(); it != sortedEntityList.rend(); ++it)
		{
			const auto node = scene->GetOrSetPrivateComponent<TreePipeNode>(*it).lock();
			node->CalculateProfile(m_pipeModelParameters, false);
			node->Wait();
			m_numOfParticles += node->m_frontParticlePhysics2D.PeekParticles().size();
		}
	}

	m_profileCalculationTime = Times::Now() - time;
}

void TreePipeBase::AdjustGraph() const
{
	std::vector<Entity> sortedEntityList;
	GatherChildrenEntities(sortedEntityList);
	const auto scene = GetScene();
	for (auto entity : sortedEntityList)
	{
		const auto node = scene->GetOrSetPrivateComponent<TreePipeNode>(entity).lock();
		scene->SetDataComponent(entity, node->m_desiredGlobalTransform);
	}
	TransformGraph::CalculateTransformGraphForDescendents(scene, GetOwner());
	for (auto entity : sortedEntityList)
	{
		auto parent = scene->GetParent(entity);
		const auto node = scene->GetOrSetPrivateComponent<TreePipeNode>(entity).lock();
		auto baseProfile = entity == sortedEntityList.front();
		const auto parentGlobalTransform = baseProfile ? scene->GetDataComponent<GlobalTransform>(entity) : scene->GetDataComponent<GlobalTransform>(parent);
		auto globalTransform = parentGlobalTransform;
		if (!baseProfile) globalTransform.m_value *= scene->GetDataComponent<Transform>(entity).m_value;

		const auto parentGlobalRotation = parentGlobalTransform.GetRotation();
		const auto parentUp = parentGlobalRotation * glm::vec3(0, 1, 0);
		const auto parentLeft = parentGlobalRotation * glm::vec3(1, 0, 0);
		const auto parentFront = parentGlobalRotation * glm::vec3(0, 0, -1);

		auto globalPosition = globalTransform.GetPosition();
		auto globalRotation = globalTransform.GetRotation();
		const auto front = globalRotation * glm::vec3(0, 0, -1);
		const float offsetLength = glm::length(node->m_offset);
		float maxDistanceToCenter = node->m_frontParticlePhysics2D.GetMaxDistanceToCenter();
		const float cosFront = glm::dot(front, parentFront); //Horizontal
		const float sinFront = glm::sin(glm::acos(glm::clamp(cosFront, -1.0f, 1.0f))); //Vertical
		if (!node->m_apical && offsetLength > glm::epsilon<float>()) {
			const float outerRadius = node->m_frontParticlePhysics2D.GetDistanceToCenter(glm::normalize(node->m_offset));
			const float innerRadius = node->m_frontParticlePhysics2D.GetDistanceToCenter(-glm::normalize(node->m_offset));
			const auto offsetDirection = glm::normalize(node->m_offset);
			const auto newOffset = (offsetLength + innerRadius + (outerRadius - outerRadius * cosFront) * m_graphAdjustmentSettings.m_rotationPushRatio) * offsetDirection;
			globalPosition += parentUp * newOffset.y * m_graphAdjustmentSettings.m_sidePushRatio * m_pipeModelParameters.m_profileDefaultCellRadius;
			globalPosition += parentLeft * newOffset.x * m_graphAdjustmentSettings.m_sidePushRatio * m_pipeModelParameters.m_profileDefaultCellRadius;
			globalPosition += parentFront * (sinFront * outerRadius * m_graphAdjustmentSettings.m_rotationPushRatio) * m_pipeModelParameters.m_profileDefaultCellRadius;
		}
		globalPosition += parentUp * node->m_shift.y * m_graphAdjustmentSettings.m_shiftPushRatio * m_pipeModelParameters.m_profileDefaultCellRadius;
		globalPosition += parentLeft * node->m_shift.x * m_graphAdjustmentSettings.m_shiftPushRatio * m_pipeModelParameters.m_profileDefaultCellRadius;
		globalPosition += parentFront * sinFront * maxDistanceToCenter * m_graphAdjustmentSettings.m_frontPushRatio * m_pipeModelParameters.m_profileDefaultCellRadius;

		globalTransform.SetPosition(globalPosition);
		scene->SetDataComponent(entity, globalTransform);
	}
	TransformGraph::CalculateTransformGraphForDescendents(scene, GetOwner());
}

void TreePipeBase::RestoreGraph() const
{
	std::vector<Entity> sortedEntityList;
	GatherChildrenEntities(sortedEntityList);
	const auto scene = GetScene();
	for (auto entity : sortedEntityList)
	{
		const auto node = scene->GetOrSetPrivateComponent<TreePipeNode>(entity).lock();
		scene->SetDataComponent(entity, node->m_desiredGlobalTransform);
	}
	TransformGraph::CalculateTransformGraphForDescendents(scene, GetOwner());
}

void TreePipeBase::BuildPipes()
{
	for (const auto& pipe : m_pipeGroup.RefPipes())
	{
		if (!pipe.PeekPipeSegmentHandles().empty()) m_pipeGroup.RecyclePipeSegment(pipe.PeekPipeSegmentHandles().front());
	}

	std::vector<Entity> sortedEntityList;
	GatherChildrenEntities(sortedEntityList);
	const auto scene = GetScene();
	auto& pipeGroup = m_pipeGroup;
	const auto modelGlobalTransform = scene->GetDataComponent<GlobalTransform>(GetOwner());
	for (auto entity : sortedEntityList)
	{
		auto parent = scene->GetParent(entity);
		const auto node = scene->GetOrSetPrivateComponent<TreePipeNode>(entity).lock();
		auto parentGlobalTransform = scene->GetDataComponent<GlobalTransform>(parent);
		bool baseNode = entity.GetIndex() == sortedEntityList.front().GetIndex();
		//parentGlobalTransform.m_value = glm::inverse(modelGlobalTransform.m_value) * parentGlobalTransform.m_value;
		auto parentGlobalRotation = parentGlobalTransform.GetRotation();
		if (baseNode)
		{
			parentGlobalRotation = glm::quatLookAt(parentGlobalRotation * glm::vec3(0, 1, 0),
				parentGlobalRotation * glm::vec3(0, 0, -1));
		}
		const auto parentGlobalPosition = parentGlobalTransform.GetPosition();
		auto globalTransform = scene->GetDataComponent<GlobalTransform>(entity);
		//globalTransform.m_value = glm::inverse(modelGlobalTransform.m_value) * globalTransform.m_value;
		const auto globalPosition = globalTransform.GetPosition();
		const auto globalRotation = globalTransform.GetRotation();
		if (baseNode)
		{
			const auto currentUp = parentGlobalRotation * glm::vec3(0, -1, 0);
			const auto currentLeft = parentGlobalRotation * glm::vec3(-1, 0, 0);
			for (const auto& [pipeHandle, particleHandle] : node->m_frontParticleMap)
			{
				const auto& particle = node->m_frontParticlePhysics2D.PeekParticle(particleHandle);
				auto& pipe = pipeGroup.RefPipe(pipeHandle);
				pipe.m_info.m_baseInfo.m_thickness = m_pipeModelParameters.m_profileDefaultCellRadius;
				pipe.m_info.m_baseInfo.m_globalPosition = parentGlobalPosition
					+ m_pipeModelParameters.m_profileDefaultCellRadius * particle.GetPosition().x * currentLeft
					+ m_pipeModelParameters.m_profileDefaultCellRadius * particle.GetPosition().y * currentUp;
				pipe.m_info.m_baseInfo.m_globalRotation = parentGlobalRotation;
			}
		}
		ExtendPipesWithProfile(globalPosition, globalRotation, node->m_backParticlePhysics2D, node->m_backParticleMap);
	}
}

void TreePipeBase::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	ImGui::DragFloat("Default profile cell radius", &m_pipeModelParameters.m_profileDefaultCellRadius, 0.001f, 0.001f, 1.0f);
	ImGui::DragFloat("Physics damping", &m_particlePhysicsSettings.m_damping, 0.01f, 0.0f, 1.0f);
	ImGui::DragFloat("Physics max speed", &m_particlePhysicsSettings.m_maxSpeed, 0.01f, 0.0f, 100.0f);
	ImGui::DragFloat("Physics particle softness", &m_particlePhysicsSettings.m_particleSoftness, 0.01f, 0.0f, 1.0f);

	ImGui::DragFloat("Center attraction strength", &m_pipeModelParameters.m_centerAttractionStrength, 0.01f, 0.0f, 10.0f);
	ImGui::DragInt("Max iteration cell factor", &m_pipeModelParameters.m_maxSimulationIterationCellFactor, 1, 0, 500);
	ImGui::DragInt("Stabilization check iteration", &m_pipeModelParameters.m_stabilizationCheckIteration, 1, 0, 500);
	ImGui::DragInt("Simulation timeout", &m_pipeModelParameters.m_timeout, 1, 0, 50000);
	ImGui::DragFloat("Stabilization movement distance", &m_pipeModelParameters.m_stabilizationMovementDistance, 0.01f, 0.0f, 1.0f);
	ImGui::DragFloat("Split limit", &m_pipeModelParameters.m_splitRatioLimit, 0.01f, 0.0f, 1.0f);
	ImGui::DragInt("End node strand count", &m_pipeModelParameters.m_endNodeStrands, 1, 1, 50);
	ImGui::Checkbox("Pre-merge", &m_pipeModelParameters.m_preMerge);

	static PrivateComponentRef tempTree{};
	ImGui::Checkbox("Parallel Scheduling", &m_parallelScheduling);
	if (editorLayer->DragAndDropButton<Tree>(tempTree, "Target tree"))
	{
		if (const auto tree = tempTree.Get<Tree>())
		{
			m_skeleton.Clone(tree->m_treeModel.PeekShootSkeleton(), [&](NodeHandle srcNodeHandle, NodeHandle dstNodeHandle) {});
			InitializeNodes();
		}
		tempTree.Clear();
	}

	if(ImGui::Button("Y Shape Example"))
	{
		InstantiateExample();
		InitializeNodes();
	}
	if (ImGui::Button("Calculate Profiles"))
	{
		CalculateProfiles();
	}

	ImGui::Text(("Last calculation time: " + std::to_string(m_profileCalculationTime)).c_str());
	ImGui::Text(("Strand count: " + std::to_string(m_pipeGroup.PeekPipes().size())).c_str());
	ImGui::Text(("Profile count: " + std::to_string(m_numOfProfiles)).c_str());
	ImGui::Text(("Total particle count: " + std::to_string(m_numOfParticles)).c_str());

	static float frontControlPointRatio = 0.4f;
	static float backControlPointRatio = 0.4f;
	ImGui::DragFloat("Front Control Point Ratio", &frontControlPointRatio, 0.01f, 0.01f, 0.5f);
	ImGui::DragFloat("Back Control Point Ratio", &backControlPointRatio, 0.01f, 0.01f, 0.5f);

	static bool adjustment = true;
	ImGui::Checkbox("Graph adjustment", &adjustment);
	if(adjustment && ImGui::TreeNodeEx("Graph Adjustment settings"))
	{
		ImGui::DragFloat("Shift push ratio", &m_graphAdjustmentSettings.m_shiftPushRatio, 0.01f, 0.0f, 2.0f);
		ImGui::DragFloat("Side push ratio", &m_graphAdjustmentSettings.m_sidePushRatio, 0.01f, 0.0f, 2.0f);
		ImGui::DragFloat("Front push ratio", &m_graphAdjustmentSettings.m_frontPushRatio, 0.01f, 0.0f, 2.0f);
		ImGui::DragFloat("Rotation push ratio", &m_graphAdjustmentSettings.m_rotationPushRatio, 0.01f, 0.0f, 2.0f);
		ImGui::TreePop();
	}

	if (ImGui::Button("Build Strands"))
	{
		if (adjustment) AdjustGraph();
		else RestoreGraph();
		InitializeStrandRenderer(frontControlPointRatio, backControlPointRatio);
	}

	if(ImGui::Button("Clear Strands"))
	{
		ClearStrands();
	}

	editorLayer->DragAndDropButton<Material>(m_nodeMaterial, "Node Material");
	editorLayer->DragAndDropButton<Mesh>(m_nodeMesh, "Node Mesh");
}

void TreePipeBase::InitializeNodes()
{
	const auto scene = GetScene();
	const auto owner = GetOwner();
	const auto children = scene->GetChildren(owner);
	for (const auto& i : children)
	{
		scene->DeleteEntity(i);
	}
	const auto ownerGlobalTransform = scene->GetDataComponent<GlobalTransform>(owner);
	const auto& srcSkeletonSortedNodeList = m_skeleton.RefSortedNodeList();
	std::unordered_map<NodeHandle, Entity> nodeMap{};
	for (const auto& nodeHandle : srcSkeletonSortedNodeList)
	{
		const auto& node = m_skeleton.PeekNode(nodeHandle);
		const auto newEntity = scene->CreateEntity("Profile");
		const auto parentHandle = node.GetParentHandle();
		const auto tpn = scene->GetOrSetPrivateComponent<TreePipeNode>(newEntity).lock();
		tpn->m_apical = node.IsApical();
		const auto mmr = scene->GetOrSetPrivateComponent<MeshRenderer>(newEntity).lock();
		mmr->m_mesh = m_nodeMesh;
		mmr->m_material = m_nodeMaterial;

		GlobalTransform globalTransform;
		const glm::quat rotation = node.m_info.m_regulatedGlobalRotation;
		globalTransform.m_value =
			ownerGlobalTransform.m_value
			* (glm::translate(node.m_info.GetGlobalEndPosition()) * glm::mat4_cast(rotation) * glm::scale(glm::vec3(0.02f)));
		scene->SetDataComponent(newEntity, globalTransform);
		tpn->m_desiredGlobalTransform = globalTransform;
		if (parentHandle == -1)
		{
			scene->SetParent(newEntity, owner);
		}
		else
		{
			scene->SetParent(newEntity, nodeMap.at(parentHandle));
		}
		nodeMap.insert({ nodeHandle, newEntity });

	}
	m_pipeGroup = {};
	TransformGraph::CalculateTransformGraphForDescendents(scene, owner);
}