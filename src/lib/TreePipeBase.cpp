#include "TreePipeBase.hpp"
#include "Tree.hpp"
using namespace EcoSysLab;

void TreePipeBase::GatherChildrenEntities(std::vector<Entity>& list) const
{
	std::queue<Entity> entityQueue{};
	const auto scene = GetScene();
	const auto owner = GetOwner();
	for(const auto& i : scene->GetChildren(owner))
	{
		if(scene->HasPrivateComponent<TreePipeNode>(i))
		{
			entityQueue.push(i);
			list.push_back(i);
		}
	}
	while(!entityQueue.empty())
	{
		auto nextEntity = entityQueue.front();
		entityQueue.pop();
		const auto& children = scene->GetChildren(nextEntity);
		for(const auto& i : children)
		{
			if (scene->HasPrivateComponent<TreePipeNode>(i))
			{
				entityQueue.push(i);
				list.push_back(i);
			}
		}
	}
}

void TreePipeBase::Packing()
{
	std::vector<Entity> sortedEntityList{};
	const auto scene = GetScene();
	GatherChildrenEntities(sortedEntityList);
	for (auto it = sortedEntityList.rbegin(); it != sortedEntityList.rend(); ++it)
	{
		auto childrenEntities = scene->GetChildren(*it);
		std::vector<std::shared_ptr<TreePipeNode>> childrenNodes;
		int mainChildNodeIndex = -1;
		for (int i = 0; i < childrenEntities.size(); i++)
		{
			if (scene->HasPrivateComponent<TreePipeNode>(childrenEntities.at(i)))
			{
				const auto childNode = scene->GetOrSetPrivateComponent<TreePipeNode>(childrenEntities.at(i)).lock();
				if (childNode->m_apical) mainChildNodeIndex = i;
				childrenNodes.push_back(childNode);
			}
		}
		const auto node = scene->GetOrSetPrivateComponent<TreePipeNode>(*it).lock();
		if(childrenNodes.empty())
		{
			//For flow start, set only particle at the center.
			const auto startParticleHandle = node->m_startParticlePhysics2D.AllocateParticle();
			auto& startParticle = node->m_startParticlePhysics2D.RefParticle(startParticleHandle);
			startParticle.SetPosition(glm::vec2(0.0f));
			startParticle.SetColor(glm::vec4(1.0f));

			const auto endParticleHandle = node->m_endParticlePhysics2D.AllocateParticle();
			auto& endParticle = node->m_endParticlePhysics2D.RefParticle(endParticleHandle);
			endParticle.SetPosition(glm::vec2(0.0f));
			endParticle.SetColor(glm::vec4(1.0f));
		}else
		{

			if(mainChildNodeIndex != 0)
			{
				
			}
		}
	}
}

void TreePipeBase::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	static PrivateComponentRef tempTree {};
	if(editorLayer->DragAndDropButton<Tree>(tempTree, "Target tree"))
	{
		if(const auto tree = tempTree.Get<Tree>())
		{
			InitializeNodesWithSkeleton(tree->m_treeModel.PeekShootSkeleton());
		}
		tempTree.Clear();
	}
}
