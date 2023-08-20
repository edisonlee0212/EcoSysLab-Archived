#pragma once
#include "Skeleton.hpp"
#include "PipeGroup.hpp"
#include "HexagonProfileData.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	struct HexagonProfileGroupData {

	};

	typedef PipeProfile<HexagonProfileData, HexagonCellData> PipeModelBaseHexagonProfile;
	typedef PipeProfile<HexagonProfileData, HexagonCellData> PipeModelHexagonProfile;
	typedef PipeProfileGroup<HexagonProfileGroupData, HexagonProfileData, HexagonCellData> PipeModelHexagonProfileGroup;

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

	struct PipeModelNodeData
	{
		int m_endNodeCount = 0;
	};

	struct PipeModelFlowData
	{
		
	};

	struct PipeModelSkeletonData
	{
	};
	typedef Skeleton<PipeModelSkeletonData, PipeModelFlowData, PipeModelNodeData> PipeModelSkeleton;
}