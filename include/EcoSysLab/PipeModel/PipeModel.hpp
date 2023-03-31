#pragma once
#include "PipeModelParameters.hpp"
using namespace UniEngine;
namespace EcoSysLab
{
	class PipeModel
	{
		void CalculatePipeTransforms(PipeModelSkeleton& targetSkeleton, const PipeModelParameters& pipeModelParameters) const;
		void DistributePipes(PipeModelSkeleton& targetSkeleton, const PipeModelParameters& pipeModelParameters);
	public:
		PipeModelHexagonGrid m_baseGrid;
		PipeModelSkeleton m_shootSkeleton;
		PipeModelSkeleton m_rootSkeleton;
		template<typename SkeletonData, typename FlowData, typename NodeData>
		void InitializeSkeleton(const Skeleton<SkeletonData, FlowData, NodeData>& srcSkeleton, PipeModelSkeleton& dstSkeleton);
		void BuildGraph(const PipeModelParameters& pipeModelParameters);
	};

	template <typename SkeletonData, typename FlowData, typename NodeData>
	void PipeModel::InitializeSkeleton(const Skeleton<SkeletonData, FlowData, NodeData>& srcSkeleton, PipeModelSkeleton& dstSkeleton)
	{
		dstSkeleton = {};
		const auto& sortedSrcNodeList = srcSkeleton.RefSortedNodeList();
		std::unordered_map<NodeHandle, NodeHandle> nodeHandleMap;
		nodeHandleMap[0] = 0;
		for (const auto& srcNodeHandle : sortedSrcNodeList)
		{
			if(srcNodeHandle == 0)
			{
				dstSkeleton.RefNode(0).m_info = srcSkeleton.PeekNode(0).m_info;
			}else
			{
				const auto& srcNode = srcSkeleton.PeekNode(srcNodeHandle);
				auto dstNodeHandle = dstSkeleton.Extend(nodeHandleMap.at(srcNode.GetParentHandle()), !srcNode.IsApical(), false);
				nodeHandleMap[srcNodeHandle] = dstNodeHandle;
				auto& dstNode = dstSkeleton.RefNode(dstNodeHandle);
				dstNode.m_info = srcNode.m_info;
			}
		}
	}
}
