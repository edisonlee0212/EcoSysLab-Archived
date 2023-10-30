#include "TreePipeBase.hpp"
#include "Tree.hpp"
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

void TreePipeBase::Packing(const PipeModelParameters& pipeModelParameters)
{
	std::vector<Entity> sortedEntityList;
	GatherChildrenEntities(sortedEntityList);
	const auto scene = GetScene();
	auto& profileGroup = m_pipeProfileGroup;
	auto& pipeGroup = m_pipeGroup;

	m_baseProfileHandle = profileGroup.Allocate();
	for (int sortedEntityIndex = 0; sortedEntityIndex < sortedEntityList.size(); sortedEntityIndex++)
	{
		const auto& entity = sortedEntityList[sortedEntityIndex];
		const auto node = scene->GetOrSetPrivateComponent<TreePipeNode>(entity).lock();
		node->m_startProfileHandle = profileGroup.Allocate();
		node->m_endProfileHandle = profileGroup.Allocate();
		auto& newStartProfile = profileGroup.RefProfile(node->m_startProfileHandle);
		auto& newEndProfile = profileGroup.RefProfile(node->m_endProfileHandle);
		newStartProfile.m_info.m_cellRadius = newEndProfile.m_info.m_cellRadius = pipeModelParameters.m_profileDefaultCellRadius;
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
				auto& baseProfile = profileGroup.RefProfile(m_baseProfileHandle);
				const auto baseCellHandle = baseProfile.AllocateCell();
				const auto newPipeHandle = pipeGroup.AllocatePipe();
				auto& baseCell = baseProfile.RefCell(baseCellHandle);
				baseCell.m_data.m_pipeHandle = newPipeHandle;

				const auto newStartCellHandle = newStartProfile.AllocateCell();
				auto& newStartCell = newStartProfile.RefCell(newStartCellHandle);
				newStartCell.m_data.m_pipeHandle = newPipeHandle;
				newStartCell.m_info.m_offset = glm::vec2(0.0f);

				const auto newEndCellHandle = newEndProfile.AllocateCell();
				auto& newEndCell = newEndProfile.RefCell(newEndCellHandle);
				newEndCell.m_data.m_pipeHandle = newPipeHandle;
				newEndCell.m_info.m_offset = glm::vec2(0.0f);

				node->m_pipeHandle = newPipeHandle;
			}
			else
			{
				const auto parentNode = scene->GetOrSetPrivateComponent<TreePipeNode>(parentEntity).lock();
				const auto& parentStartNodeProfile = profileGroup.PeekProfile(parentNode->m_startProfileHandle);
				for (const auto& cell : parentStartNodeProfile.PeekCells())
				{
					const auto newStartCellHandle = newStartProfile.AllocateCell();
					auto& newStartCell = newStartProfile.RefCell(newStartCellHandle);
					newStartCell.m_data.m_pipeHandle = cell.m_data.m_pipeHandle;
					newStartCell.m_info.m_offset = glm::vec2(0.0f);

					const auto newEndCellHandle = newEndProfile.AllocateCell();
					auto& newEndCell = newEndProfile.RefCell(newEndCellHandle);
					newEndCell.m_data.m_pipeHandle = cell.m_data.m_pipeHandle;
					newEndCell.m_info.m_offset = glm::vec2(0.0f);
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
			auto& baseProfile = profileGroup.RefProfile(m_baseProfileHandle);
			const auto baseCellHandle = baseProfile.AllocateCell();
			node->m_pipeHandle = pipeGroup.AllocatePipe();

			auto& baseCell = baseProfile.RefCell(baseCellHandle);
			baseCell.m_data.m_pipeHandle = node->m_pipeHandle;

			baseCell.m_info.m_offset = glm::vec2(0.0f);
			for (auto it = parentNodeToRootChain.rbegin(); it != parentNodeToRootChain.rend(); ++it) {

				auto& startProfile = profileGroup.RefProfile((*it)->m_startProfileHandle);
				auto& endProfile = profileGroup.RefProfile((*it)->m_endProfileHandle);
				const auto newStartCellHandle = startProfile.AllocateCell();
				auto& newStartCell = startProfile.RefCell(newStartCellHandle);
				newStartCell.m_data.m_pipeHandle = node->m_pipeHandle;
				newStartCell.m_info.m_offset = glm::vec2(0.0f);

				const auto newEndCellHandle = endProfile.AllocateCell();
				auto& newEndCell = endProfile.RefCell(newEndCellHandle);
				newEndCell.m_data.m_pipeHandle = node->m_pipeHandle;
				newEndCell.m_info.m_offset = glm::vec2(0.0f);
			}
			const auto newStartCellHandle = newStartProfile.AllocateCell();
			auto& newStartCell = newStartProfile.RefCell(newStartCellHandle);
			newStartCell.m_data.m_pipeHandle = node->m_pipeHandle;
			newStartCell.m_info.m_offset = glm::vec2(0.0f);

			const auto newEndCellHandle = newEndProfile.AllocateCell();
			auto& newEndCell = newEndProfile.RefCell(newEndCellHandle);
			newEndCell.m_data.m_pipeHandle = node->m_pipeHandle;
			newEndCell.m_info.m_offset = glm::vec2(0.0f);
		}
	}

	for (const auto& entity : sortedEntityList)
	{
		const auto node = scene->GetOrSetPrivateComponent<TreePipeNode>(entity).lock();
		node->m_startParticleMap.clear();
		node->m_endParticleMap.clear();
		auto& startProfile = profileGroup.RefProfile(node->m_startProfileHandle);
		auto& startPhysics2D = node->m_startParticlePhysics2D;
		startPhysics2D.Reset(0.002f);
		for (auto& cell : startProfile.RefCells())
		{
			auto newParticleHandle = startPhysics2D.AllocateParticle();
			auto& newParticle = startPhysics2D.RefParticle(newParticleHandle);
			newParticle.m_data.m_pipeHandle = cell.m_data.m_pipeHandle;
			newParticle.SetDamping(pipeModelParameters.m_damping);
			newParticle.SetPosition(cell.m_info.m_offset);
			node->m_startParticleMap.insert({ cell.m_data.m_pipeHandle, newParticleHandle });
		}
		auto& endProfile = profileGroup.RefProfile(node->m_endProfileHandle);
		auto& endPhysics2D = node->m_endParticlePhysics2D;
		endPhysics2D.Reset(0.002f);
		for (auto& cell : endProfile.RefCells())
		{
			auto newParticleHandle = endPhysics2D.AllocateParticle();
			auto& newParticle = endPhysics2D.RefParticle(newParticleHandle);
			newParticle.m_data.m_pipeHandle = cell.m_data.m_pipeHandle;
			newParticle.SetDamping(pipeModelParameters.m_damping);
			newParticle.SetPosition(cell.m_info.m_offset);
			node->m_endParticleMap.insert({ cell.m_data.m_pipeHandle, newParticleHandle });
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
		if (childrenNodes.empty())
		{
			assert(node->m_startParticlePhysics2D.RefParticles().size() == 1);
			assert(node->m_endParticlePhysics2D.RefParticles().size() == 1);
			//For flow start, set only particle at the center.
			for (auto& particle : node->m_startParticlePhysics2D.RefParticles())
			{
				particle.SetColor(glm::vec4(1.0f));
				particle.SetPosition(glm::vec2(0.0f));
			}
			//For flow end, set only particle at the center.
			for (auto& particle : node->m_endParticlePhysics2D.RefParticles())
			{
				particle.SetColor(glm::vec4(1.0f));
				particle.SetPosition(glm::vec2(0.0f));
			}
		}
		else if (childrenNodes.size() == 1)
		{
			//Copy from child flow start to self flow start
			const auto& childNode = childrenNodes.front();
			const auto& childPhysics2D = childNode->m_startParticlePhysics2D;
			assert(node->m_startParticlePhysics2D.PeekParticles().size() == childPhysics2D.PeekParticles().size());
			assert(node->m_endParticlePhysics2D.PeekParticles().size() == childPhysics2D.PeekParticles().size());
			for (const auto& childParticle : childPhysics2D.PeekParticles())
			{
				const auto nodeStartParticleHandle = node->m_startParticleMap.at(childParticle.m_data.m_pipeHandle);
				const auto nodeEndParticleHandle = node->m_endParticleMap.at(childParticle.m_data.m_pipeHandle);
				node->m_startParticlePhysics2D.RefParticle(nodeStartParticleHandle).SetColor(childParticle.GetColor());
				node->m_startParticlePhysics2D.RefParticle(nodeStartParticleHandle).SetPosition(childParticle.GetPosition());
				node->m_endParticlePhysics2D.RefParticle(nodeEndParticleHandle).SetColor(childParticle.GetColor());
				node->m_endParticlePhysics2D.RefParticle(nodeEndParticleHandle).SetPosition(childParticle.GetPosition());
			}
		}
		else {
			const auto& mainChildPhysics2D = mainChildNode->m_startParticlePhysics2D;
			for (const auto& mainChildParticle : mainChildPhysics2D.PeekParticles())
			{
				const auto nodeStartParticleHandle = node->m_startParticleMap.at(mainChildParticle.m_data.m_pipeHandle);
				const auto nodeEndParticleHandle = node->m_endParticleMap.at(mainChildParticle.m_data.m_pipeHandle);
				node->m_startParticlePhysics2D.RefParticle(nodeStartParticleHandle).SetColor(mainChildParticle.GetColor());
				node->m_startParticlePhysics2D.RefParticle(nodeStartParticleHandle).SetPosition(mainChildParticle.GetPosition());
				node->m_endParticlePhysics2D.RefParticle(nodeEndParticleHandle).SetColor(mainChildParticle.GetColor());
				node->m_endParticlePhysics2D.RefParticle(nodeEndParticleHandle).SetPosition(mainChildParticle.GetPosition());
			}
			int index = 0;
			for (const auto& childNode : childrenNodes)
			{
				if (childNode == mainChildNode) continue;
				auto& childPhysics2D = childNode->m_startParticlePhysics2D;
				auto childNodeFront = glm::inverse(node->m_endRegulatedRotation) * childNode->m_startRegulatedRotation * glm::vec3(0, 0, -1);
				auto offset = glm::normalize(glm::vec2(childNodeFront.x, childNodeFront.y));
				if(glm::isnan(offset.x) || glm::isnan(offset.y))
				{
					offset = glm::vec2(1, 0);
				}
				offset = (mainChildPhysics2D.GetDistanceToCenter(offset) + childPhysics2D.GetDistanceToCenter(-offset) + 2.0f) * offset;
				childNode->m_offset = offset;
				for (const auto& childParticle : childPhysics2D.PeekParticles())
				{
					const auto nodeStartParticleHandle = node->m_startParticleMap.at(childParticle.m_data.m_pipeHandle);
					const auto nodeEndParticleHandle = node->m_endParticleMap.at(childParticle.m_data.m_pipeHandle);
					node->m_startParticlePhysics2D.RefParticle(nodeStartParticleHandle).SetColor(childParticle.GetColor());
					node->m_startParticlePhysics2D.RefParticle(nodeStartParticleHandle).SetPosition(childParticle.GetPosition() + offset);
					node->m_endParticlePhysics2D.RefParticle(nodeEndParticleHandle).SetColor(childParticle.GetColor());
					node->m_endParticlePhysics2D.RefParticle(nodeEndParticleHandle).SetPosition(childParticle.GetPosition() + offset);
				}
				index++;
			}
			const auto iterations = pipeModelParameters.m_simulationIterationCellFactor * node->m_startParticlePhysics2D.RefParticles().size();
			for (int i = 0; i < iterations; i++) {
				node->m_startParticlePhysics2D.Simulate(1, [&](auto& particle)
					{
						//Apply gravity
						particle.SetPosition(particle.GetPosition() - node->m_startParticlePhysics2D.GetMassCenter());
						if (glm::length(particle.GetPosition()) > 0.0f) {
							const glm::vec2 acceleration = pipeModelParameters.m_gravityStrength * -glm::normalize(particle.GetPosition());
							particle.SetAcceleration(acceleration);
						}
					}
				);
				if (i > pipeModelParameters.m_minimumSimulationIteration && node->m_startParticlePhysics2D.GetMaxParticleVelocity() < pipeModelParameters.m_particleStabilizeSpeed)
				{
					break;
				}
			}
		}
	}
}


void TreePipeBase::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	static PrivateComponentRef tempTree{};
	if (editorLayer->DragAndDropButton<Tree>(tempTree, "Target tree"))
	{
		if (const auto tree = tempTree.Get<Tree>())
		{
			InitializeNodesWithSkeleton(tree->m_treeModel.PeekShootSkeleton());
		}
		tempTree.Clear();
	}
	static PipeModelParameters pipeModelParameters{};

	if(ImGui::Button("Packing"))
	{
		Packing(pipeModelParameters);
	}
}
