#pragma once
#include "PipeModelData.hpp"
#include "PipeModelParameters.hpp"
#include "TreeVisualizer.hpp"
#include "TreeMeshGenerator.hpp"
#include "LSystemString.hpp"
#include "TreePipeNode.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	struct GraphAdjustmentSettings
	{
		float m_sidePushRatio = 1.0f;
		float m_frontPushRatio = 1.0f;
		float m_rotationPushRatio = 0.5f;
	};

	class TreePipeBase : public IPrivateComponent
	{
		void GatherChildrenEntities(std::vector<Entity>& list) const;
		void ExtendPipesWithProfile(
			const glm::vec3& globalPosition,
			const glm::quat& globalRotation,
			const ParticlePhysics2D<CellParticlePhysicsData>& profile, const std::unordered_map<PipeHandle, ParticleHandle>& map);
	public:
		bool m_parallelScheduling = true;
		void InstantiateExample();
		PipeModelPipeGroup m_pipeGroup;
		PipeModelParameters m_pipeModelParameters{};
		GraphAdjustmentSettings m_graphAdjustmentSettings;
		AssetRef m_nodeMaterial{};
		AssetRef m_nodeMesh{};
		template<typename SkeletonData, typename FlowData, typename NodeData>
		void InitializeNodesWithSkeleton(const Skeleton<SkeletonData, FlowData, NodeData>& srcSkeleton);
		void ClearStrands() const;
		void InitializeStrandRenderer(float frontControlPointRatio, float backControlPointRatio, int nodeMaxCount = -1);
		void OnCreate() override;
		void Packing();
		void AdjustGraph() const;
		void RestoreGraph() const;
		void BuildPipes();
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
	};

	template <typename SkeletonData, typename FlowData, typename NodeData>
	void TreePipeBase::InitializeNodesWithSkeleton(const Skeleton<SkeletonData, FlowData, NodeData>& srcSkeleton)
	{
		const auto scene = GetScene();
		const auto owner = GetOwner();
		const auto children = scene->GetChildren(owner);
		for (const auto& i : children)
		{
			scene->DeleteEntity(i);
		}
		const auto ownerGlobalTransform = scene->GetDataComponent<GlobalTransform>(owner);
		const auto& srcSkeletonSortedFlowList = srcSkeleton.RefSortedFlowList();
		std::unordered_map<FlowHandle, Entity> flowMap{};
		for (const auto& flowHandle : srcSkeletonSortedFlowList)
		{
			const auto& flow = srcSkeleton.PeekFlow(flowHandle);
			const auto newEntity = scene->CreateEntity("Profile");
			const auto parentHandle = flow.GetParentHandle();

			const auto tpn = scene->GetOrSetPrivateComponent<TreePipeNode>(newEntity).lock();

			auto& firstNode = srcSkeleton.PeekNode(flow.RefNodeHandles().front());
			auto& lastNode = srcSkeleton.PeekNode(flow.RefNodeHandles().back());

			tpn->m_apical = flow.IsApical();
			const auto mmr = scene->GetOrSetPrivateComponent<MeshRenderer>(newEntity).lock();
			mmr->m_mesh = m_nodeMesh;
			mmr->m_material = m_nodeMaterial;

			GlobalTransform globalTransform;
			const glm::quat rotation = lastNode.m_info.m_regulatedGlobalRotation;
			globalTransform.m_value =
				ownerGlobalTransform.m_value
				* (glm::translate(flow.m_info.m_globalEndPosition) * glm::mat4_cast(rotation) * glm::scale(glm::vec3(0.02f)));
			scene->SetDataComponent(newEntity, globalTransform);
			tpn->m_desiredGlobalTransform = globalTransform;
			if (parentHandle == -1)
			{
				scene->SetParent(newEntity, owner);
			}
			else
			{
				scene->SetParent(newEntity, flowMap.at(parentHandle));
			}
			flowMap.insert({ flowHandle, newEntity });

		}
		m_pipeGroup = {};
	}
}
