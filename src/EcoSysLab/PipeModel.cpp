#include "PipeModel.hpp"

using namespace EcoSysLab;

void PipeModel::CalculatePipeTransforms(PipeModelSkeleton& targetSkeleton, const PipeModelParameters& pipeModelParameters) const
{
	auto& pipeGroup = targetSkeleton.m_data.m_pipeGroup;
	const auto& gridGroup = targetSkeleton.m_data.m_hexagonGridGroup;
	for (auto& pipeNode : pipeGroup.RefPipeNodes())
	{
		if (pipeNode.IsRecycled()) continue;
		const auto& node = m_shootSkeleton.PeekNode(pipeNode.m_data.m_nodeHandle);
		const auto& nodeInfo = node.m_info;
		const glm::vec3 left = nodeInfo.m_regulatedGlobalRotation * glm::vec3(1, 0, 0);
		const glm::vec3 up = nodeInfo.m_regulatedGlobalRotation * glm::vec3(0, 1, 0);
		const glm::vec3 front = nodeInfo.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
		auto& pipeInfo = pipeNode.m_info;
		const auto& grid = gridGroup.PeekGrid(node.m_data.m_gridHandle);
		const auto& cell = grid.PeekCell(pipeNode.m_data.m_cellHandle);

		pipeInfo.m_localPosition = pipeModelParameters.m_endNodeThickness * 2.0f * grid.GetPosition(cell.GetCoordinate());
		pipeInfo.m_thickness = pipeModelParameters.m_endNodeThickness;
		pipeInfo.m_globalPosition = nodeInfo.m_globalPosition + front * nodeInfo.m_length + left * pipeInfo.m_localPosition.x + up * pipeInfo.m_localPosition.y;
		pipeInfo.m_globalRotation = nodeInfo.m_regulatedGlobalRotation;
	}

	for (auto& pipe : pipeGroup.RefPipes())
	{
		if (pipe.IsRecycled()) continue;
		pipe.m_info.m_color = glm::mix(pipeGroup.m_data.m_innerColor, pipeGroup.m_data.m_outerColor, pipe.m_data.m_startAge - glm::floor(pipe.m_data.m_startAge));
	}
}

void PipeModel::DistributePipes(bool isShoot, const PipeModelParameters& pipeModelParameters)
{
	//1. Reverse calculate number of distribution ratio.
	auto& targetSkeleton = isShoot ? m_shootSkeleton : m_rootSkeleton;
	const auto nodeList = targetSkeleton.RefSortedNodeList();
	for (auto it = nodeList.rbegin(); it != nodeList.rend(); it++)
	{
		auto& node = targetSkeleton.RefNode(*it);
		auto& nodeData = node.m_data;
		nodeData.m_endNodeCount = 0;
		if (node.IsEndNode()) nodeData.m_endNodeCount = 1;
		else
		{
			for (const auto& childNodeHandle : node.RefChildHandles())
			{
				const auto& childNode = targetSkeleton.RefNode(childNodeHandle);
				node.m_data.m_endNodeCount += childNode.m_data.m_endNodeCount;
			}
		}
	}

	targetSkeleton.m_data.m_hexagonGridGroup = {};
	targetSkeleton.m_data.m_pipeGroup = {};
	auto& pipeGroup = targetSkeleton.m_data.m_pipeGroup;
	auto& gridGroup = targetSkeleton.m_data.m_hexagonGridGroup;
	//2. Allocate pipe for target skeleton.
	const auto firstGridHandle = gridGroup.Allocate();
	auto& firstGrid = gridGroup.RefGrid(firstGridHandle);
	for (const auto& readOnlyCell : m_baseGrid.PeekCells())
	{
		if (readOnlyCell.IsRecycled()) continue;
		auto& cell = m_baseGrid.RefCell(readOnlyCell.GetHandle());
		const auto newPipeHandle = pipeGroup.AllocatePipe();
		if (isShoot) cell.m_data.m_shootPipeHandle = newPipeHandle;
		else cell.m_data.m_rootPipeHandle = newPipeHandle;

		auto& firstNode = targetSkeleton.RefNode(0);
		const auto firstGridCellHandle = firstGrid.Allocate(cell.GetCoordinate());
		auto& firstGridCell = firstGrid.RefCell(firstGridCellHandle);
		firstGridCell.m_data.m_pipeHandle = newPipeHandle;
		firstNode.m_data.m_gridHandle = firstGridHandle;
		const auto firstPipeNodeHandle = pipeGroup.Extend(newPipeHandle);
		auto& firstPipeNode = pipeGroup.RefPipeNode(firstPipeNodeHandle);
		firstPipeNode.m_data.m_cellHandle = firstGridCellHandle;
		firstPipeNode.m_data.m_nodeHandle = 0;
		firstNode.m_data.m_pipeNodeHandles.emplace_back(firstPipeNodeHandle);
	}
	//3. Create traverse graph and setup pipes.
	for (const auto& nodeHandle : nodeList)
	{
		auto& node = targetSkeleton.RefNode(nodeHandle);
		auto& nodeData = node.m_data;
		if (nodeHandle == 0) continue;
		const auto& parentNode = targetSkeleton.PeekNode(node.GetParentHandle());
		//Create a hexagon grid for every node that has multiple child, and a hexagon grid for each child.
		if (node.RefChildHandles().size() > 1) {
			const auto parentGridHandle = parentNode.m_data.m_gridHandle;
			const auto newGridHandle = gridGroup.Allocate();
			const auto& parentGrid = gridGroup.PeekGrid(parentGridHandle);
			auto& newGrid = gridGroup.RefGrid(newGridHandle);
			//Copy all cells from parent grid.
			for(const auto& parentCell : parentGrid.PeekCells())
			{
				if(parentCell.IsRecycled()) continue;

			}
			continue;
		}
		if(parentNode.RefChildHandles().size() == 1)
		{
			node.m_data.m_gridHandle = parentNode.m_data.m_gridHandle;
			//Extend all pipe nodes from parent.
		}

	}
}

void PipeModel::BuildGraph(const PipeModelParameters& pipeModelParameters)
{
	const auto& baseGridCellCount = m_baseGrid.GetCellCount();
	const auto& shootFlowList = m_shootSkeleton.RefSortedFlowList();
	const auto& rootFlowList = m_rootSkeleton.RefSortedFlowList();
	if (baseGridCellCount >= shootFlowList.size())
	{
		DistributePipes(true, pipeModelParameters);
		CalculatePipeTransforms(m_shootSkeleton, pipeModelParameters);
	}
	if (baseGridCellCount >= rootFlowList.size())
	{
		DistributePipes(false, pipeModelParameters);
		CalculatePipeTransforms(m_rootSkeleton, pipeModelParameters);
	}
}
