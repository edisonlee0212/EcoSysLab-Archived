#pragma once

#include "TreePipeModel.hpp"
#include "TreeVisualizer.hpp"
#include "TreeMeshGenerator.hpp"
#include "LSystemString.hpp"
#include "TreeGraph.hpp"
#include "TreeGrowthParameters.hpp"

#include "TreePipeNode.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	class TreePipeBase : public IPrivateComponent
	{
		void GatherChildrenEntities(std::vector<Entity>& list) const;

	public:
		PipeModelPipeGroup m_pipeGroup;
		PipeModelPipeProfileGroup m_pipeProfileGroup;
		PipeModelParameters m_pipeModelParameters{};

		ParticlePhysics2D<CellParticlePhysicsData> m_baseParticlePhysics2D;
		ProfileHandle m_baseProfileHandle = -1;
		template<typename SkeletonData, typename FlowData, typename NodeData>
		void InitializeNodesWithSkeleton(const Skeleton<SkeletonData, FlowData, NodeData>& srcSkeleton);

		void Packing(const PipeModelParameters& pipeModelParameters);
		void AdjustGraph(const PipeModelParameters& pipeModelParameters);
		void BuildPipes(const PipeModelParameters& pipeModelParameters);
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
	};

	template <typename SkeletonData, typename FlowData, typename NodeData>
	void TreePipeBase::InitializeNodesWithSkeleton(const Skeleton<SkeletonData, FlowData, NodeData>& srcSkeleton)
	{
		const auto scene = GetScene();
		const auto owner = GetOwner();
		const auto children = scene->GetChildren(owner);
		for(const auto& i : children)
		{
			scene->DeleteEntity(i);
		}
		const auto ownerGlobalTransform = scene->GetDataComponent<GlobalTransform>(owner);
		const auto& srcSkeletonSortedFlowList = srcSkeleton.RefSortedFlowList();
		std::unordered_map<FlowHandle, Entity> flowMap{};
		const auto nodeMaterial = ProjectManager::CreateTemporaryAsset<Material>();
		for (const auto& flowHandle : srcSkeletonSortedFlowList)
		{
			const auto& flow = srcSkeleton.PeekFlow(flowHandle);
			const auto newEntity = scene->CreateEntity("Node " + std::to_string(flowHandle));
			const auto parentHandle = flow.GetParentHandle();
			if(parentHandle == -1)
			{
				scene->SetParent(newEntity, owner);
			}else
			{
				scene->SetParent(newEntity, flowMap.at(parentHandle));
			}
			flowMap.insert({ flowHandle, newEntity });
			
			auto tpn = scene->GetOrSetPrivateComponent<TreePipeNode>(newEntity).lock();
			tpn->m_apical = flow.IsApical();
			auto& lastNode = srcSkeleton.PeekNode(flow.RefNodeHandles().back());
			tpn->m_endRegulatedRotation = lastNode.m_info.m_regulatedGlobalRotation;
			auto& firstNode = srcSkeleton.PeekNode(flow.RefNodeHandles().front());
			tpn->m_startRegulatedRotation = firstNode.m_info.m_regulatedGlobalRotation;
			auto mmr = scene->GetOrSetPrivateComponent<MeshRenderer>(newEntity).lock();
			mmr->m_mesh = Resources::GetResource<Mesh>("PRIMITIVE_CUBE");
			mmr->m_material = nodeMaterial;

			GlobalTransform globalTransform;
			//const auto diff = flow.m_info.m_globalEndPosition - flow.m_info.m_globalStartPosition;
			const glm::quat rotation = tpn->m_endRegulatedRotation;
			globalTransform.m_value =
				ownerGlobalTransform.m_value
				* (glm::translate(flow.m_info.m_globalEndPosition) * glm::mat4_cast(rotation) * glm::scale(glm::vec3(flow.m_info.m_startThickness * 5.0f, flow.m_info.m_startThickness, flow.m_info.m_startThickness * 5.0f)));
			scene->SetDataComponent(newEntity, globalTransform);
		}
		m_pipeGroup = {};
		m_pipeProfileGroup = {};
		m_baseProfileHandle = -1;
	}
}
