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
			particle.SetDamping(pipeModelParameters.m_damping);
			node->m_profiles.front()->m_particleMap.insert({ particle.m_data.m_pipeHandle, particle.GetHandle() });
		}
		auto& endPhysics2D = node->m_profiles.front()->m_particlePhysics2D;
		
		for (auto& particle : endPhysics2D.RefParticles())
		{
			particle.SetDamping(pipeModelParameters.m_damping);
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
		node->m_profiles.front()->m_offset  = glm::vec2(0.0f);
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
			}
			//For flow end, set only particle at the center.
			for (auto& particle : node->m_profiles.back()->m_particlePhysics2D.RefParticles())
			{
				particle.SetColor(glm::vec4(1.0f));
				particle.SetPosition(glm::vec2(0.0f));
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
				node->m_profiles.front()->m_particlePhysics2D.RefParticle(nodeStartParticleHandle).SetColor(childParticle.GetColor());
				node->m_profiles.front()->m_particlePhysics2D.RefParticle(nodeStartParticleHandle).SetPosition(childParticle.GetPosition());
				node->m_profiles.back()->m_particlePhysics2D.RefParticle(nodeEndParticleHandle).SetColor(childParticle.GetColor());
				node->m_profiles.back()->m_particlePhysics2D.RefParticle(nodeEndParticleHandle).SetPosition(childParticle.GetPosition());
			}
		}
		else {
			const auto& mainChildPhysics2D = mainChildNode->m_profiles.front()->m_particlePhysics2D;
			for (const auto& mainChildParticle : mainChildPhysics2D.PeekParticles())
			{
				const auto nodeStartParticleHandle = node->m_profiles.front()->m_particleMap.at(mainChildParticle.m_data.m_pipeHandle);
				const auto nodeEndParticleHandle = node->m_profiles.back()->m_particleMap.at(mainChildParticle.m_data.m_pipeHandle);
				node->m_profiles.front()->m_particlePhysics2D.RefParticle(nodeStartParticleHandle).SetColor(mainChildParticle.GetColor());
				node->m_profiles.front()->m_particlePhysics2D.RefParticle(nodeStartParticleHandle).SetPosition(mainChildParticle.GetPosition());
				node->m_profiles.back()->m_particlePhysics2D.RefParticle(nodeEndParticleHandle).SetColor(mainChildParticle.GetColor());
				node->m_profiles.back()->m_particlePhysics2D.RefParticle(nodeEndParticleHandle).SetPosition(mainChildParticle.GetPosition());
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
				if(glm::isnan(offset.x) || glm::isnan(offset.y))
				{
					offset = glm::vec2(1, 0);
				}
				childNode->m_centerDirectionRadius = childPhysics2D.GetDistanceToCenter(-offset) + 1.0f;
				offset = (mainChildPhysics2D.GetDistanceToCenter(offset) + childNode->m_centerDirectionRadius + 1.0f) * offset;
				childNode->m_profiles.front()->m_offset = offset;
				for (const auto& childParticle : childPhysics2D.PeekParticles())
				{
					const auto nodeStartParticleHandle = node->m_profiles.front()->m_particleMap.at(childParticle.m_data.m_pipeHandle);
					const auto nodeEndParticleHandle = node->m_profiles.back()->m_particleMap.at(childParticle.m_data.m_pipeHandle);
					node->m_profiles.front()->m_particlePhysics2D.RefParticle(nodeStartParticleHandle).SetColor(childParticle.GetColor());
					node->m_profiles.front()->m_particlePhysics2D.RefParticle(nodeStartParticleHandle).SetPosition(childParticle.GetPosition() + offset);
					node->m_profiles.back()->m_particlePhysics2D.RefParticle(nodeEndParticleHandle).SetColor(childParticle.GetColor());
					node->m_profiles.back()->m_particlePhysics2D.RefParticle(nodeEndParticleHandle).SetPosition(childParticle.GetPosition() + offset);
				}
				index++;
			}
			const auto iterations = pipeModelParameters.m_simulationIterationCellFactor * node->m_profiles.front()->m_particlePhysics2D.RefParticles().size();
			for (int i = 0; i < iterations; i++) {
				node->m_profiles.front()->m_particlePhysics2D.Simulate(1, [&](auto& particle)
					{
						//Apply gravity
						particle.SetPosition(particle.GetPosition() - node->m_profiles.front()->m_particlePhysics2D.GetMassCenter());
						if (glm::length(particle.GetPosition()) > 0.0f) {
							const glm::vec2 acceleration = pipeModelParameters.m_gravityStrength * -glm::normalize(particle.GetPosition());
							particle.SetAcceleration(acceleration);
						}
					}
				);
				if (i > pipeModelParameters.m_minimumSimulationIteration && node->m_profiles.front()->m_particlePhysics2D.GetMaxParticleVelocity() < pipeModelParameters.m_particleStabilizeSpeed)
				{
					break;
				}
			}
		}
	}
}

void TreePipeBase::AdjustGraph(const PipeModelParameters& pipeModelParameters)
{
	std::vector<Entity> sortedEntityList;
	GatherChildrenEntities(sortedEntityList);
	const auto scene = GetScene();
	auto& pipeGroup = m_pipeGroup;
	for (auto entity : sortedEntityList)
	{
		auto parent = scene->GetParent(entity);
		const auto node = scene->GetOrSetPrivateComponent<TreePipeNode>(entity).lock();
		const auto parentGlobalTransform = scene->GetDataComponent<GlobalTransform>(parent);
		const auto parentGlobalRotation = parentGlobalTransform.GetRotation();
		const auto parentUp = parentGlobalRotation * glm::vec3(0, 1, 0);
		const auto parentLeft = parentGlobalRotation * glm::vec3(1, 0, 0);
		const auto parentFront = parentGlobalRotation * glm::vec3(0, 0, -1);

		auto globalTransform = scene->GetDataComponent<GlobalTransform>(entity);
		auto globalPosition = globalTransform.GetPosition();
		auto globalRotation = globalTransform.GetRotation();
		const auto front = globalRotation * glm::vec3(0, 0, -1);

		const float offsetLength = glm::length(node->m_profiles.front()->m_offset);
		if (offsetLength > glm::epsilon<float>()) {
			const float cosFront = glm::dot(front, parentFront); //Horizontal
			const float sinFront = glm::sin(glm::acos(cosFront)); //Vertical
			const auto offsetDirection = glm::normalize(node->m_profiles.front()->m_offset);
			globalPosition += parentUp * offsetDirection.y * (offsetLength + cosFront * node->m_centerDirectionRadius) * m_pipeModelParameters.m_profileDefaultCellRadius;
			globalPosition += parentLeft * offsetDirection.x * (offsetLength + cosFront * node->m_centerDirectionRadius) * m_pipeModelParameters.m_profileDefaultCellRadius;
			globalPosition += parentFront * (sinFront * node->m_centerDirectionRadius) * m_pipeModelParameters.m_profileDefaultCellRadius;
		}
		globalTransform.SetPosition(globalPosition);
		scene->SetDataComponent(entity, globalTransform);
	}
}

void TreePipeBase::BuildPipes(const PipeModelParameters& pipeModelParameters)
{
	for(const auto& pipe : m_pipeGroup.RefPipes())
	{
		if(!pipe.PeekPipeSegmentHandles().empty()) m_pipeGroup.RecyclePipeSegment(pipe.PeekPipeSegmentHandles().front());
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
	ImGui::DragFloat("Default profile cell radius", &m_pipeModelParameters.m_profileDefaultCellRadius, 0.001f, 0.001f, 1.0f);
	ImGui::DragFloat("Physics damping", &m_pipeModelParameters.m_damping, 0.01f, 0.0f, 1.0f);
	ImGui::DragFloat("Physics attraction strength", &m_pipeModelParameters.m_gravityStrength, 0.01f, 0.0f, 10.0f);
	ImGui::DragFloat("Physics simulation iteration cell factor", &m_pipeModelParameters.m_simulationIterationCellFactor, 0.1f, 0.0f, 50.0f);
	ImGui::DragInt("Physics simulation minimum iteration", &m_pipeModelParameters.m_minimumSimulationIteration, 1, 0, 50);
	ImGui::DragFloat("Physics simulation particle stabilize speed", &m_pipeModelParameters.m_particleStabilizeSpeed, 0.1f, 0.0f, 100.0f);

	if(ImGui::Button("Packing"))
	{
		Packing(m_pipeModelParameters);
	}
	if (ImGui::Button("Adjust Graph"))
	{
		AdjustGraph(m_pipeModelParameters);
	}
}
