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
		node->m_profiles.front()->m_particlePhysics2D.Reset(0.002f);
		node->m_profiles.back()->m_particlePhysics2D.Reset(0.002f);
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
				const auto newPipeHandle = pipeGroup.AllocatePipe();

				const auto newStartParticleHandle = node->m_profiles.front()->m_particlePhysics2D.AllocateParticle();
				auto& newStartParticle = node->m_profiles.front()->m_particlePhysics2D.RefParticle(newStartParticleHandle);
				newStartParticle.m_data.m_pipeHandle = newPipeHandle;

				const auto newEndParticleHandle = node->m_profiles.back()->m_particlePhysics2D.AllocateParticle();
				auto& newEndParticle = node->m_profiles.back()->m_particlePhysics2D.RefParticle(newEndParticleHandle);
				newEndParticle.m_data.m_pipeHandle = newPipeHandle;

				node->m_pipeHandle = newPipeHandle;
			}
			else
			{
				const auto parentNode = scene->GetOrSetPrivateComponent<TreePipeNode>(parentEntity).lock();
				for (const auto& cell : parentNode->m_profiles.front()->m_particlePhysics2D.PeekParticles())
				{
					const auto newStartParticleHandle = node->m_profiles.front()->m_particlePhysics2D.AllocateParticle();
					auto& newStartParticle = node->m_profiles.front()->m_particlePhysics2D.RefParticle(newStartParticleHandle);
					newStartParticle.m_data.m_pipeHandle = cell.m_data.m_pipeHandle;

					const auto newEndParticleHandle = node->m_profiles.back()->m_particlePhysics2D.AllocateParticle();
					auto& newEndParticle = node->m_profiles.back()->m_particlePhysics2D.RefParticle(newEndParticleHandle);
					newEndParticle.m_data.m_pipeHandle = cell.m_data.m_pipeHandle;
				}

				node->m_pipeHandle = parentNode->m_pipeHandle;
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
			node->m_pipeHandle = pipeGroup.AllocatePipe();
			for (auto it = parentNodeToRootChain.rbegin(); it != parentNodeToRootChain.rend(); ++it) {

				const auto newStartParticleHandle = (*it)->m_profiles.front()->m_particlePhysics2D.AllocateParticle();
				auto& newStartParticle = (*it)->m_profiles.front()->m_particlePhysics2D.RefParticle(newStartParticleHandle);
				newStartParticle.m_data.m_pipeHandle = node->m_pipeHandle;

				const auto newEndParticleHandle = (*it)->m_profiles.back()->m_particlePhysics2D.AllocateParticle();
				auto& newEndParticle = (*it)->m_profiles.back()->m_particlePhysics2D.RefParticle(newEndParticleHandle);
				newEndParticle.m_data.m_pipeHandle = node->m_pipeHandle;
			}
			const auto newStartParticleHandle = node->m_profiles.front()->m_particlePhysics2D.AllocateParticle();
			auto& newStartParticle = node->m_profiles.front()->m_particlePhysics2D.RefParticle(newStartParticleHandle);
			newStartParticle.m_data.m_pipeHandle = node->m_pipeHandle;

			const auto newEndParticleHandle = node->m_profiles.back()->m_particlePhysics2D.AllocateParticle();
			auto& newEndParticle = node->m_profiles.back()->m_particlePhysics2D.RefParticle(newEndParticleHandle);
			newEndParticle.m_data.m_pipeHandle = node->m_pipeHandle;
		}
	}

	for (const auto& entity : sortedEntityList)
	{
		const auto node = scene->GetOrSetPrivateComponent<TreePipeNode>(entity).lock();
		node->m_profiles.front()->m_particleMap.clear();
		node->m_profiles.back()->m_particleMap.clear();
		auto& startPhysics2D = node->m_profiles.front()->m_particlePhysics2D;

		for (auto& particle : startPhysics2D.RefParticles())
		{
			particle.SetDamping(m_pipeModelParameters.m_damping);
			node->m_profiles.front()->m_particleMap.insert({ particle.m_data.m_pipeHandle, particle.GetHandle() });
		}
		auto& endPhysics2D = node->m_profiles.front()->m_particlePhysics2D;

		for (auto& particle : endPhysics2D.RefParticles())
		{
			particle.SetDamping(m_pipeModelParameters.m_damping);
			node->m_profiles.back()->m_particleMap.insert({ particle.m_data.m_pipeHandle, particle.GetHandle() });
		}
	}


	for (auto it = sortedEntityList.rbegin(); it != sortedEntityList.rend(); ++it)
	{
		auto childrenEntities = scene->GetChildren(*it);
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
		if (!childrenNodes.empty() && !mainChildNode) mainChildNode = childrenNodes.front();
		const auto node = scene->GetOrSetPrivateComponent<TreePipeNode>(*it).lock();
		node->m_centerDirectionRadius = 0.0f;
		if (childrenNodes.empty())
		{
			assert(node->m_profiles.front()->m_particlePhysics2D.RefParticles().size() == 1);
			assert(node->m_profiles.back()->m_particlePhysics2D.RefParticles().size() == 1);
			//For flow start, set only particle at the center.
			for (auto& particle : node->m_profiles.front()->m_particlePhysics2D.RefParticles())
			{
				particle.SetColor(glm::vec4(1.0f));
				particle.SetPosition(glm::vec2(0.0f));

				particle.m_data.m_mainChild = true;
			}
			//For flow end, set only particle at the center.
			for (auto& particle : node->m_profiles.back()->m_particlePhysics2D.RefParticles())
			{
				particle.SetColor(glm::vec4(1.0f));
				particle.SetPosition(glm::vec2(0.0f));

				particle.m_data.m_mainChild = true;
			}
		}
		else if (childrenNodes.size() == 1)
		{
			//Copy from child flow start to self flow start
			const auto& childNode = childrenNodes.front();
			const auto& childPhysics2D = childNode->m_profiles.front()->m_particlePhysics2D;
			assert(node->m_profiles.front()->m_particlePhysics2D.PeekParticles().size() == childPhysics2D.PeekParticles().size());
			assert(node->m_profiles.back()->m_particlePhysics2D.PeekParticles().size() == childPhysics2D.PeekParticles().size());
			for (const auto& childParticle : childPhysics2D.PeekParticles())
			{
				const auto nodeStartParticleHandle = node->m_profiles.front()->m_particleMap.at(childParticle.m_data.m_pipeHandle);
				const auto nodeEndParticleHandle = node->m_profiles.back()->m_particleMap.at(childParticle.m_data.m_pipeHandle);
				auto& nodeStartParticle = node->m_profiles.front()->m_particlePhysics2D.RefParticle(nodeStartParticleHandle);
				auto& nodeEndParticle = node->m_profiles.back()->m_particlePhysics2D.RefParticle(nodeEndParticleHandle);
				nodeStartParticle.SetColor(childParticle.GetColor());
				nodeStartParticle.SetPosition(childParticle.GetPosition());
				nodeEndParticle.SetColor(childParticle.GetColor());
				nodeEndParticle.SetPosition(childParticle.GetPosition());

				nodeStartParticle.m_data.m_mainChild = nodeEndParticle.m_data.m_mainChild = true;
			}
		}
		else {
			const auto& mainChildPhysics2D = mainChildNode->m_profiles.front()->m_particlePhysics2D;
			for (const auto& mainChildParticle : mainChildPhysics2D.PeekParticles())
			{
				const auto nodeStartParticleHandle = node->m_profiles.front()->m_particleMap.at(mainChildParticle.m_data.m_pipeHandle);
				const auto nodeEndParticleHandle = node->m_profiles.back()->m_particleMap.at(mainChildParticle.m_data.m_pipeHandle);
				auto& nodeStartParticle = node->m_profiles.front()->m_particlePhysics2D.RefParticle(nodeStartParticleHandle);
				auto& nodeEndParticle = node->m_profiles.back()->m_particlePhysics2D.RefParticle(nodeEndParticleHandle);
				nodeStartParticle.SetColor(mainChildParticle.GetColor());
				nodeStartParticle.SetPosition(mainChildParticle.GetPosition());
				nodeEndParticle.SetColor(mainChildParticle.GetColor());
				nodeEndParticle.SetPosition(mainChildParticle.GetPosition());

				nodeStartParticle.m_data.m_mainChild = nodeEndParticle.m_data.m_mainChild = true;
			}
			int index = 0;
			const auto nodeGlobalTransform = scene->GetDataComponent<GlobalTransform>(*it);
			for (const auto& childNode : childrenNodes)
			{
				if (childNode == mainChildNode) continue;
				auto& childPhysics2D = childNode->m_profiles.front()->m_particlePhysics2D;
				auto childNodeGlobalTransform = scene->GetDataComponent<GlobalTransform>(childNode->GetOwner());
				auto childNodeFront = glm::inverse(nodeGlobalTransform.GetRotation()) * childNodeGlobalTransform.GetRotation() * glm::vec3(0, 0, -1);
				auto offset = glm::normalize(glm::vec2(childNodeFront.x, childNodeFront.y));
				if (glm::isnan(offset.x) || glm::isnan(offset.y))
				{
					offset = glm::vec2(1, 0);
				}
				childNode->m_centerDirectionRadius = childPhysics2D.GetDistanceToCenter(-offset) + 1.0f;
				offset = (mainChildPhysics2D.GetDistanceToCenter(offset) + childNode->m_centerDirectionRadius + 1.0f) * offset;
				childNode->m_offset = offset;
				for (auto& childParticle : childPhysics2D.RefParticles())
				{
					childParticle.SetPosition(childParticle.GetPosition() + offset);

					const auto nodeStartParticleHandle = node->m_profiles.front()->m_particleMap.at(childParticle.m_data.m_pipeHandle);
					const auto nodeEndParticleHandle = node->m_profiles.back()->m_particleMap.at(childParticle.m_data.m_pipeHandle);
					auto& nodeStartParticle = node->m_profiles.front()->m_particlePhysics2D.RefParticle(nodeStartParticleHandle);
					auto& nodeEndParticle = node->m_profiles.back()->m_particlePhysics2D.RefParticle(nodeEndParticleHandle);
					nodeStartParticle.SetColor(childParticle.GetColor());
					nodeStartParticle.SetPosition(childParticle.GetPosition());
					nodeEndParticle.SetColor(childParticle.GetColor());
					nodeEndParticle.SetPosition(childParticle.GetPosition());

					nodeStartParticle.m_data.m_mainChild = nodeEndParticle.m_data.m_mainChild = false;
				}
				index++;
			}
			const auto iterations = m_pipeModelParameters.m_simulationIterationCellFactor * node->m_profiles.front()->m_particlePhysics2D.RefParticles().size();
			for (int i = 0; i < iterations; i++) {
				node->m_profiles.front()->m_particlePhysics2D.Simulate(1, [&](auto& particle)
					{
						//Apply gravity
						particle.SetPosition(particle.GetPosition() - node->m_profiles.front()->m_particlePhysics2D.GetMassCenter());
						if (glm::length(particle.GetPosition()) > 0.0f) {
							const glm::vec2 acceleration = m_pipeModelParameters.m_gravityStrength * -glm::normalize(particle.GetPosition());
							particle.SetAcceleration(acceleration);
						}
					}
				);
				if (i > m_pipeModelParameters.m_minimumSimulationIteration && node->m_profiles.front()->m_particlePhysics2D.GetMaxParticleVelocity() < m_pipeModelParameters.m_particleStabilizeSpeed)
				{
					break;
				}
			}
		}
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
		float maxDistanceToCenter = node->m_profiles.back()->m_particlePhysics2D.GetMaxDistanceToCenter();
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
			auto& profile = node->m_profiles.front();
			const auto currentUp = parentGlobalRotation * glm::vec3(0, -1, 0);
			const auto currentLeft = parentGlobalRotation * glm::vec3(-1, 0, 0);
			for (const auto& [pipeHandle, particleHandle] : profile->m_particleMap)
			{
				BezierCurve bezierCurve{};
				bezierCurve.m_p0 = parentGlobalPosition;
				bezierCurve.m_p3 = globalPosition;
				float distance = glm::distance(bezierCurve.m_p0, bezierCurve.m_p3);
				bezierCurve.m_p1 = bezierCurve.m_p0 + distance * 0.25f * (parentGlobalRotation * glm::vec3(0, 0, -1));
				bezierCurve.m_p2 = bezierCurve.m_p3 - distance * 0.25f * (globalRotation * glm::vec3(0, 0, -1));

				const auto& particle = profile->m_particlePhysics2D.PeekParticle(particleHandle);
				auto& pipe = pipeGroup.RefPipe(pipeHandle);
				pipe.m_info.m_baseInfo.m_thickness = m_pipeModelParameters.m_profileDefaultCellRadius;
				pipe.m_info.m_baseInfo.m_globalPosition = bezierCurve.GetPoint(profile->m_a)
					+ m_pipeModelParameters.m_profileDefaultCellRadius * particle.GetPosition().x * currentLeft
					+ m_pipeModelParameters.m_profileDefaultCellRadius * particle.GetPosition().y * currentUp;
				pipe.m_info.m_baseInfo.m_globalRotation = parentGlobalRotation;
			}
		}
		for (int i = 1; i < node->m_profiles.size(); i++)
		{
			auto& profile = node->m_profiles.at(i);
			const auto currentRotation = glm::mix(parentGlobalRotation, globalRotation, profile->m_a);
			const auto currentUp = currentRotation * glm::vec3(0, 1, 0);
			const auto currentLeft = currentRotation * glm::vec3(1, 0, 0);
			for (const auto& [pipeHandle, particleHandle] : profile->m_particleMap)
			{
				BezierCurve bezierCurve{};
				bezierCurve.m_p0 = parentGlobalPosition;
				bezierCurve.m_p3 = globalPosition;
				float distance = glm::distance(bezierCurve.m_p0, bezierCurve.m_p3);
				bezierCurve.m_p1 = bezierCurve.m_p0 + distance * 0.25f * (parentGlobalRotation * glm::vec3(0, 0, -1));
				bezierCurve.m_p2 = bezierCurve.m_p3 - distance * 0.25f * (globalRotation * glm::vec3(0, 0, -1));

				const auto& particle = profile->m_particlePhysics2D.PeekParticle(particleHandle);
				const auto newPipeSegmentHandle = pipeGroup.Extend(pipeHandle);
				auto& newPipeSegment = pipeGroup.RefPipeSegment(newPipeSegmentHandle);
				newPipeSegment.m_info.m_thickness = m_pipeModelParameters.m_profileDefaultCellRadius;
				newPipeSegment.m_info.m_globalPosition = bezierCurve.GetPoint(profile->m_a)
					+ m_pipeModelParameters.m_profileDefaultCellRadius * particle.GetPosition().x * currentLeft
					+ m_pipeModelParameters.m_profileDefaultCellRadius * particle.GetPosition().y * currentUp;
				newPipeSegment.m_info.m_globalRotation = currentRotation;
			}
		}
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
}
