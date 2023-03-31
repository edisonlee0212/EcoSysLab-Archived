#pragma once
#include "PlantStructure.hpp"
#include "PipeStructure.hpp"
#include "HexagonGrid.hpp"
using namespace UniEngine;
namespace EcoSysLab
{
	struct HexagonGridCellData
	{
		PipeHandle m_pipeHandle = -1;
	};

	struct HexagonGridData
	{
		NodeHandle m_nodeHandle = -1;
	};

	typedef HexagonGrid<HexagonGridData, HexagonGridCellData> PipeModelHexagonGrid;
	typedef HexagonGridGroup<HexagonGridData, HexagonGridCellData> PipeModelHexagonGridGroup;

	struct PipeModelPipeGroupData
	{
		glm::vec4 m_innerColor = glm::vec4(233, 216, 201, 255) / 255.0f;
		glm::vec4 m_outerColor = glm::vec4(44, 32, 21, 255) / 255.0f;
	};

	struct PipeModelPipeData
	{
		float m_startAge = 0.0f;

		PipeNodeInfo m_baseInfo;
	};

	struct PipeModelPipeNodeData
	{
		NodeHandle m_nodeHandle = -1;
		HexagonCellHandle m_cellHandle = -1;
	};

	typedef PipeGroup<PipeModelPipeGroupData, PipeModelPipeData, PipeModelPipeNodeData> PipeModelPipeGroup;

	struct PipeModelNodeData
	{
		HexagonGridHandle m_gridHandle = -1;
		std::vector<PipeNodeHandle> m_pipeNodeHandles;
	};

	struct PipeModelFlowData
	{
		
	};

	struct PipeModelSkeletonData
	{
		PipeModelPipeGroup m_pipeGroup;
		PipeModelHexagonGridGroup m_hexagonGridGroup;
	};
	typedef Skeleton<PipeModelSkeletonData, PipeModelFlowData, PipeModelNodeData> PipeModelSkeleton;
}