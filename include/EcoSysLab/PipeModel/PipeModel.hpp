#pragma once
#include "PipeModelParameters.hpp"
using namespace UniEngine;
namespace EcoSysLab
{
	class PipeModel
	{
		void CalculatePipeLocalPositions(const PipeModelSkeleton& targetSkeleton, const PipeModelParameters& pipeModelParameters);
		void CalculatePipeTransforms(const PipeModelSkeleton& targetSkeleton, const PipeModelParameters& pipeModelParameters);
		void DistributePipes(PipeModelBaseHexagonGrid baseGrid, PipeModelSkeleton& targetSkeleton, const PipeModelParameters& pipeModelParameters);

		void SplitPipes(std::unordered_map<NodeHandle, HexagonGridHandle>& gridHandleMap, PipeModelHexagonGridGroup& gridGroup, 
			PipeModelSkeleton& targetSkeleton, NodeHandle nodeHandle, HexagonGridHandle newGridHandle, const PipeModelParameters& pipeModelParameters);
	public:
		PipeModelPipeGroup m_pipeGroup;
		template <typename SkeletonData, typename FlowData, typename NodeData>
		void InitializePipes(const Skeleton<SkeletonData, FlowData, NodeData> &targetSkeleton, PipeModelBaseHexagonGrid baseGrid, const PipeModelParameters& pipeModelParameters);
	};

	template <typename SkeletonData, typename FlowData, typename NodeData>
	void PipeModel::InitializePipes(const Skeleton<SkeletonData, FlowData, NodeData>& targetSkeleton, PipeModelBaseHexagonGrid baseGrid,
		const PipeModelParameters& pipeModelParameters)
	{
		if (baseGrid.GetCellCount() == 0) return;
		PipeModelSkeleton clonedSkeleton;
		clonedSkeleton.Clone<SkeletonData, FlowData, NodeData>(targetSkeleton, [&](NodeHandle srcNodeHandle, NodeHandle dstNodeHandle){});
		const auto& flowList = clonedSkeleton.RefSortedFlowList();
		if (!flowList.empty())
		{
			DistributePipes(baseGrid, clonedSkeleton, pipeModelParameters);
			CalculatePipeLocalPositions(clonedSkeleton, pipeModelParameters);
			clonedSkeleton.CalculateTransforms();
			CalculatePipeTransforms(clonedSkeleton, pipeModelParameters);
		}
	}
}
