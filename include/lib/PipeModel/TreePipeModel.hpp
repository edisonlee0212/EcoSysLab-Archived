#pragma once

#include "TreeModel.hpp"
#include "PipeModel.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	class TreePipeModel
	{
	public:
		PipeModel m_shootPipeModel{};
		template<typename SkeletonData, typename FlowData, typename NodeData>
		void InitializePipeSkeleton(const Skeleton<SkeletonData, FlowData, NodeData>& srcSkeleton);
		void ShiftSkeleton();
		void UpdatePipeModels(const TreeModel& targetTreeModel, const PipeModelParameters& pipeModelParameters);
		void ApplySimulationResults(const PipeModelParameters& pipeModelParameters);
	};

	template <typename SkeletonData, typename FlowData, typename NodeData>
	void TreePipeModel::InitializePipeSkeleton(const Skeleton<SkeletonData, FlowData, NodeData>& srcSkeleton)
	{
		auto& dstSkeleton = m_shootPipeModel.m_skeleton;
		const auto& srcSkeletonSortedFlowList = srcSkeleton.RefSortedFlowList();
		std::unordered_map<FlowHandle, NodeHandle> flowMap{};
		flowMap.insert({ 0, 0 });
		for (const auto& flowHandle : srcSkeletonSortedFlowList)
		{
			const auto& flow = srcSkeleton.PeekFlow(flowHandle);
			const auto newNodeHandle = dstSkeleton.Extend(flowMap.at(flow.GetParentHandle()), flow.IsApical());
			auto& newNode = dstSkeleton.RefNode(newNodeHandle);
			flowMap.insert({flowHandle, newNodeHandle});
			newNode.m_info.m_globalPosition = flow.m_info.m_globalStartPosition;
			const auto diff = flow.m_info.m_globalEndPosition - flow.m_info.m_globalStartPosition;
			newNode.m_info.m_globalDirection = glm::normalize(diff);
			newNode.m_info.m_globalRotation = glm::normalize(diff);
			newNode.m_info.m_length = glm::length(diff);
		}

	}
}
