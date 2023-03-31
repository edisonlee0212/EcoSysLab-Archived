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
	for (auto it = nodeList.rbegin(); it != nodeList.rend(); ++it)
	{
		auto& node = targetSkeleton.RefNode(*it);
		auto& nodeData = node.m_data;
		if (node.IsEndNode()) nodeData.m_endNodeCount = 1;
		else
		{
			nodeData.m_endNodeCount = 0;
			for (const auto& childNodeHandle : node.RefChildHandles())
			{
				const auto& childNode = targetSkeleton.RefNode(childNodeHandle);
				nodeData.m_endNodeCount += childNode.m_data.m_endNodeCount;
			}
		}
	}

	targetSkeleton.m_data.m_hexagonGridGroup = {};
	targetSkeleton.m_data.m_pipeGroup = {};
	auto& pipeGroup = targetSkeleton.m_data.m_pipeGroup;
	auto& gridGroup = targetSkeleton.m_data.m_hexagonGridGroup;
	//2. Allocate pipe for target skeleton. Also create new grid for first node and copy cells from base grid.
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
			nodeData.m_gridHandle = newGridHandle;
			const auto& parentGrid = gridGroup.PeekGrid(parentGridHandle);
			auto& newGrid = gridGroup.RefGrid(newGridHandle);
			//Copy all cells from parent grid.
			for(const auto& parentCell : parentGrid.PeekCells())
			{
				if(parentCell.IsRecycled()) continue;
				const auto newCellHandle = newGrid.Allocate(parentCell.GetCoordinate());
				auto& newCell = newGrid.RefCell(newCellHandle);
				newCell.m_data.m_pipeHandle = parentCell.m_data.m_pipeHandle;
				const auto newPipeNodeHandle = pipeGroup.Extend(newCell.m_data.m_pipeHandle);
				auto& newPipeNode = pipeGroup.RefPipeNode(newPipeNodeHandle);
				newPipeNode.m_data.m_cellHandle = newCellHandle;
				newPipeNode.m_data.m_nodeHandle = nodeHandle;

			}
			//Sort child by their end node sizes.
			int totalAllocatedCellCount = 0;
			std::map<int, std::pair<NodeHandle, int>> childNodeHandles;
			for (const auto& childNodeHandle : node.RefChildHandles())
			{
				const auto& childNode = targetSkeleton.RefNode(childNodeHandle);
				int cellCount = static_cast<float>(newGrid.GetCellCount()) * childNode.m_data.m_endNodeCount / node.m_data.m_endNodeCount;
				childNodeHandles[childNode.m_data.m_endNodeCount] = { childNodeHandle , cellCount };
				totalAllocatedCellCount += cellCount;
			}
			childNodeHandles.end()->second.second += newGrid.GetCellCount() - totalAllocatedCellCount;

			std::map<std::pair<int, int>, HexagonCellHandle> remainingCells = newGrid.PeekCellMap();
			for (auto it = childNodeHandles.rbegin(); it != childNodeHandles.rend(); ++it)
			{
				auto& childNode = targetSkeleton.RefNode(it->second.first);
				auto& childNodeData = childNode.m_data;
				const auto childNewGridHandle = gridGroup.Allocate();
				childNodeData.m_gridHandle = childNewGridHandle;
				auto& childNewGrid = gridGroup.RefGrid(childNewGridHandle);
				const auto cellCount = it->second.second;
				//1. Find the start cell.
				//2. Sort cell based on distance to the start cell
				//3. Extract cells based on distance.
				for(int i = 0; i < cellCount; i++)
				{
					const auto allocatedCellHandle = remainingCells.begin()->second;
					remainingCells.erase(remainingCells.begin());
					const auto& allocatedCell = newGrid.PeekCell(allocatedCellHandle);
					const auto childNewCellHandle = childNewGrid.Allocate(allocatedCell.GetCoordinate());
					auto& childNewCell = childNewGrid.RefCell(childNewCellHandle);
					childNewCell.m_data.m_pipeHandle = allocatedCell.m_data.m_pipeHandle;

					const auto childNewPipeNodeHandle = pipeGroup.Extend(childNewCell.m_data.m_pipeHandle);
					auto& childNewPipeNode = pipeGroup.RefPipeNode(childNewPipeNodeHandle);
					childNewPipeNode.m_data.m_cellHandle = childNewCellHandle;
					childNewPipeNode.m_data.m_nodeHandle = it->second.first;

				}
			}

			continue;
		}
		if(parentNode.RefChildHandles().size() == 1)
		{
			//Extend all pipe nodes from parent.
			const auto parentGridHandle = parentNode.m_data.m_gridHandle;
			nodeData.m_gridHandle = parentGridHandle;
			const auto& parentGrid = gridGroup.PeekGrid(parentGridHandle);
			for (const auto& parentCell : parentGrid.PeekCells())
			{
				if (parentCell.IsRecycled()) continue;
				const auto newPipeNodeHandle = pipeGroup.Extend(parentCell.m_data.m_pipeHandle);
				auto& newPipeNode = pipeGroup.RefPipeNode(newPipeNodeHandle);
				newPipeNode.m_data.m_cellHandle = parentCell.GetHandle();
				newPipeNode.m_data.m_nodeHandle = nodeHandle;
			}
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
