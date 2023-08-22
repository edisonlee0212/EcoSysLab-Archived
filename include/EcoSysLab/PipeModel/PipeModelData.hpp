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
	};

	struct PipeModelPipeSegmentData
	{
		NodeHandle m_nodeHandle = -1;
	};

	typedef PipeGroup<PipeModelPipeGroupData, PipeModelPipeData, PipeModelPipeSegmentData> PipeModelPipeGroup;


	struct PipeCellData
	{
		PipeHandle m_pipeHandle = -1;
		PipeSegmentHandle m_pipeSegmentHandle = -1;
	};

	struct PipeProfileData
	{
		NodeHandle m_nodeHandle = -1;
	};

	struct PipeProfileGroupData
	{
		
	};

	typedef PipeProfile<PipeProfileData, PipeCellData> PipeModelPipeProfile;
	typedef PipeProfileGroup<PipeProfileGroupData, PipeProfileData, PipeCellData> PipeModelPipeProfileGroup;
	struct PipeModelNodeData
	{
		int m_endNodeCount = 0;

		ProfileHandle m_profileHandle = -1;
	};

	struct PipeModelFlowData
	{
		
	};

	struct PipeModelSkeletonData
	{
		ProfileHandle m_baseProfileHandle = -1;
	};

	typedef Skeleton<PipeModelSkeletonData, PipeModelFlowData, PipeModelNodeData> PipeModelSkeleton;
}