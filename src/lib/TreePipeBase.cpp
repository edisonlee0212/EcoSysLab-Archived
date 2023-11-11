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

void TreePipeBase::Packing()
{
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
			particle.SetDamping(m_pipeModelParameters.m_damping);
			node->m_frontParticleMap.insert({ particle.m_data.m_pipeHandle, particle.GetHandle() });
		}
		auto& endPhysics2D = node->m_backParticlePhysics2D;

		for (auto& particle : endPhysics2D.RefParticles())
		{
			particle.SetDamping(m_pipeModelParameters.m_damping);
			node->m_backParticleMap.insert({ particle.m_data.m_pipeHandle, particle.GetHandle() });
		}
	}

	for (auto it = sortedEntityList.rbegin(); it != sortedEntityList.rend(); ++it)
	{
		const auto node = scene->GetOrSetPrivateComponent<TreePipeNode>(*it).lock();
		node->Merge(m_pipeModelParameters);
		node->Pack(m_pipeModelParameters);
	}
}

void TreePipeBase::AdjustGraph() const
{
	std::vector<Entity> sortedEntityList;
	GatherChildrenEntities(sortedEntityList);
	const auto scene = GetScene();
	for (auto entity : sortedEntityList)
	{
		auto parent = scene->GetParent(entity);
		const auto node = scene->GetOrSetPrivateComponent<TreePipeNode>(entity).lock();
		const auto parentGlobalTransform = scene->GetDataComponent<GlobalTransform>(parent);
		const auto parentGlobalRotation = parentGlobalTransform.GetRotation();
		const auto parentUp = parentGlobalRotation * glm::vec3(0, 1, 0);
		const auto parentLeft = parentGlobalRotation * glm::vec3(1, 0, 0);
		const auto parentFront = parentGlobalRotation * glm::vec3(0, 0, -1);

		auto globalTransform = parentGlobalTransform;
		globalTransform.m_value *= scene->GetDataComponent<Transform>(entity).m_value;
		auto globalPosition = globalTransform.GetPosition();
		auto globalRotation = globalTransform.GetRotation();
		const auto front = globalRotation * glm::vec3(0, 0, -1);
		float maxDistanceToCenter = node->m_frontParticlePhysics2D.GetMaxDistanceToCenter();
		const float offsetLength = glm::length(node->m_offset);
		if (offsetLength > glm::epsilon<float>()) {
			const float cosFront = glm::dot(front, parentFront); //Horizontal
			const float sinFront = glm::sin(glm::acos(cosFront)); //Vertical
			const auto offsetDirection = glm::normalize(node->m_offset);
			globalPosition += parentUp * offsetDirection.y * offsetLength * m_pipeModelParameters.m_profileDefaultCellRadius;
			globalPosition += parentLeft * offsetDirection.x * offsetLength * m_pipeModelParameters.m_profileDefaultCellRadius;
			globalPosition += parentFront * (sinFront * node->m_centerDirectionRadius + maxDistanceToCenter) * m_pipeModelParameters.m_profileDefaultCellRadius;
		}
		globalTransform.SetPosition(globalPosition);
		scene->SetDataComponent(entity, globalTransform);
	}
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
		if(baseNode)
		{
			parentGlobalRotation = glm::quatLookAt(parentGlobalRotation * glm::vec3(0, 1, 0),
				parentGlobalRotation * glm::vec3(0, 0, -1));
		}
		const auto parentGlobalPosition = parentGlobalTransform.GetPosition();
		auto globalTransform = scene->GetDataComponent<GlobalTransform>(entity);
		//globalTransform.m_value = glm::inverse(modelGlobalTransform.m_value) * globalTransform.m_value;
		const auto globalPosition = globalTransform.GetPosition();
		const auto globalRotation = globalTransform.GetRotation();
		if(baseNode)
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
	ImGui::DragFloat("Physics damping", &m_pipeModelParameters.m_damping, 0.01f, 0.0f, 1.0f);
	ImGui::DragFloat("Physics attraction strength", &m_pipeModelParameters.m_gravityStrength, 0.01f, 0.0f, 10.0f);
	ImGui::DragFloat("Physics simulation iteration cell factor", &m_pipeModelParameters.m_simulationIterationCellFactor, 0.1f, 0.0f, 50.0f);
	ImGui::DragInt("Physics simulation minimum iteration", &m_pipeModelParameters.m_minimumSimulationIteration, 1, 0, 50);
	ImGui::DragFloat("Physics simulation particle stabilize speed", &m_pipeModelParameters.m_particleStabilizeSpeed, 0.1f, 0.0f, 100.0f);
	static bool adjustGraph = true;
	ImGui::Checkbox("Adjust graph", &adjustGraph);
	static PrivateComponentRef tempTree{};
	if (editorLayer->DragAndDropButton<Tree>(tempTree, "Target tree"))
	{
		if (const auto tree = tempTree.Get<Tree>())
		{
			InitializeNodesWithSkeleton(tree->m_treeModel.PeekShootSkeleton());
			Packing();
			TransformGraph::CalculateTransformGraphForDescendents(GetScene(), GetOwner());
			if (adjustGraph) AdjustGraph();
		}
		tempTree.Clear();
	}
	static float frontControlPointRatio = 0.2f;
	static float backControlPointRatio = 0.2f;
	ImGui::DragFloat("Front Control Point Ratio", &frontControlPointRatio, 0.01f, 0.01f, 0.5f);
	ImGui::DragFloat("Back Control Point Ratio", &backControlPointRatio, 0.01f, 0.01f, 0.5f);
	if (ImGui::Button("Build Strands"))
	{
		InitializeStrandRenderer(frontControlPointRatio, backControlPointRatio);
	}

	editorLayer->DragAndDropButton(m_nodeMaterial, "Node Material");
	editorLayer->DragAndDropButton(m_nodeMesh, "Node Mesh");
}
