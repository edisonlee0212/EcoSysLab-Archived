#pragma once
#include "Skeleton.hpp"
#include "PipeGroup.hpp"
#include "HexagonProfileData.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	struct PipeModelPipeGroupData
	{
	};

	struct PipeModelPipeData
	{
		PipeSegmentInfo m_baseInfo;
	};

	struct PipeModelPipeNodeData
	{
		NodeHandle m_nodeHandle = -1;
	};

	typedef PipeGroup<PipeModelPipeGroupData, PipeModelPipeData, PipeModelPipeNodeData> PipeModelPipeGroup;


	struct PipeCellData
	{
		
	};

	struct PipeProfileData
	{
		
	};

	struct PipeModelNodeData
	{
		int m_endNodeCount = 0;

		PipeProfile<PipeProfileData, PipeCellData> m_profile;
	};

	struct PipeModelFlowData
	{
		
	};

	struct PipeModelSkeletonData
	{
	};
	typedef Skeleton<PipeModelSkeletonData, PipeModelFlowData, PipeModelNodeData> PipeModelSkeleton;
}