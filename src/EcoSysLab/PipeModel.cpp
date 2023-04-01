#include "PipeModel.hpp"

using namespace EcoSysLab;

void PipeModel::CalculatePipeTransforms(PipeModelSkeleton& targetSkeleton, const PipeModelParameters& pipeModelParameters)
{
	auto& pipeGroup = targetSkeleton.m_data.m_pipeGroup;
	const auto& gridGroup = targetSkeleton.m_data.m_hexagonGridGroup;
	for (auto& pipeNode : pipeGroup.RefPipeNodes())
	{
		if (pipeNode.IsRecycled()) continue;
		const auto& node = targetSkeleton.PeekNode(pipeNode.m_data.m_nodeHandle);
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
		pipe.m_info.m_color = glm::vec4(1.0f);
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
		nodeData.m_gridHandle = -1;
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
		//Create a hexagon grid for every node that has multiple child, and a hexagon grid for each child.
		const auto currentGridHandle = nodeData.m_gridHandle;
		//No pipe left for this node.
		if(currentGridHandle < 0) continue;
		if (node.RefChildHandles().size() > 1) {
			const auto newGridHandle = gridGroup.Allocate();
			nodeData.m_gridHandle = newGridHandle;
			auto& newGrid = gridGroup.RefGrid(newGridHandle);
			//Copy all cells from parent grid.
			const auto& previousGrid = gridGroup.PeekGrid(currentGridHandle);
			for(const auto& parentCell : previousGrid.PeekCells())
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
			std::multimap<int, std::pair<NodeHandle, int>> childNodeHandles;
			for (const auto& childNodeHandle : node.RefChildHandles())
			{
				const auto& childNode = targetSkeleton.RefNode(childNodeHandle);
				int cellCount = static_cast<float>(newGrid.GetCellCount()) * childNode.m_data.m_endNodeCount / node.m_data.m_endNodeCount;
				childNodeHandles.insert({ childNode.m_data.m_endNodeCount, { childNodeHandle , cellCount } });
				totalAllocatedCellCount += cellCount;
			}
			auto last = childNodeHandles.end();
			--last;
			last->second.second += newGrid.GetCellCount() - totalAllocatedCellCount;
			auto newGridCellMap = newGrid.PeekCellMap();
			for (auto it = childNodeHandles.rbegin(); it != childNodeHandles.rend(); ++it)
			{
				const auto cellCount = it->second.second;
				auto& childNode = targetSkeleton.RefNode(it->second.first);
				auto& childNodeData = childNode.m_data;
				if(cellCount == 0) continue;
				
				const auto childNewGridHandle = gridGroup.Allocate();
				childNodeData.m_gridHandle = childNewGridHandle;
				auto& childNewGrid = gridGroup.RefGrid(childNewGridHandle);
				const auto& prevGrid = gridGroup.RefGrid(newGridHandle);
				//1. Find the start cell.
				glm::vec2 direction = glm::diskRand(1.0f);
				std::map<int, std::map<float, HexagonCellHandle>> sortedKnots;
				for (const auto& i : newGridCellMap) {
					const auto& prevCell = prevGrid.PeekCell(i.second);
					auto position = prevGrid.GetPosition(prevCell.GetCoordinate());
					if (position == glm::vec2(0.0f)) continue;
					int angle = glm::degrees(
						glm::acos(glm::clamp(glm::dot(glm::normalize(position), glm::normalize(direction)), 0.0f,
							1.0f))) /
						10.0f;
					float distance = glm::length(position);
					auto search = sortedKnots.find(angle);
					if (search == sortedKnots.end()) {
						sortedKnots[angle] = { {distance, prevCell.GetHandle()} };
					}
					else {
						search->second[distance] = prevCell.GetHandle();
					}
				}
				//2. Sort cell based on distance to the start cell
				auto baseCellHandle = sortedKnots.begin()->second.rbegin()->second;
				const auto& baseCell = prevGrid.PeekCell(baseCellHandle);
				std::multimap<float, HexagonCellHandle> distanceSortedCellHandles;
				auto basePosition = prevGrid.GetPosition(baseCell.GetCoordinate());
				for (const auto& i : newGridCellMap) {
					const auto& prevCell = prevGrid.PeekCell(i.second);
					auto position = prevGrid.GetPosition(prevCell.GetCoordinate());
					float distance = glm::distance(position, basePosition);
					distanceSortedCellHandles.insert({ distance, prevCell.GetHandle() });
				}

				//3. Extract cells based on distance.
				glm::ivec2 sumCoordinate = glm::ivec2(0, 0);
				for(int i = 0; i < cellCount; i++)
				{
					const auto allocatedCellHandle = distanceSortedCellHandles.begin()->second;
					distanceSortedCellHandles.erase(distanceSortedCellHandles.begin());
					const auto& allocatedCell = prevGrid.PeekCell(allocatedCellHandle);
					auto coordinate = allocatedCell.GetCoordinate();
					newGridCellMap.erase({ coordinate.x, coordinate.y });
					sumCoordinate += coordinate;
					const auto childNewCellHandle = childNewGrid.Allocate(coordinate);
					auto& childNewCell = childNewGrid.RefCell(childNewCellHandle);
					childNewCell.m_data.m_pipeHandle = allocatedCell.m_data.m_pipeHandle;

					const auto childNewPipeNodeHandle = pipeGroup.Extend(childNewCell.m_data.m_pipeHandle);
					auto& childNewPipeNode = pipeGroup.RefPipeNode(childNewPipeNodeHandle);
					childNewPipeNode.m_data.m_cellHandle = childNewCellHandle;
					childNewPipeNode.m_data.m_nodeHandle = it->second.first;

				}
				sumCoordinate /= cellCount;
				childNewGrid.ShiftCoordinate(-sumCoordinate);
			}
		}
		else if(node.RefChildHandles().size() == 1)
		{
			auto childNodeHandle = node.RefChildHandles()[0];
			auto& childNode = targetSkeleton.RefNode(childNodeHandle);
			auto& childNodeData = childNode.m_data;
			//Extend all pipe nodes from parent.
			childNodeData.m_gridHandle = currentGridHandle;
			const auto& previousGrid = gridGroup.PeekGrid(currentGridHandle);
			for (const auto& parentCell : previousGrid.PeekCells())
			{
				if (parentCell.IsRecycled()) continue;

				const auto newPipeNodeHandle = pipeGroup.Extend(parentCell.m_data.m_pipeHandle);
				auto& newPipeNode = pipeGroup.RefPipeNode(newPipeNodeHandle);
				newPipeNode.m_data.m_cellHandle = parentCell.GetHandle();
				newPipeNode.m_data.m_nodeHandle = childNodeHandle;
			}
		}
	}
}

void PipeModel::BuildGraph(const PipeModelParameters& pipeModelParameters)
{
	const auto& baseGridCellCount = m_baseGrid.GetCellCount();
	const auto& shootFlowList = m_shootSkeleton.RefSortedFlowList();
	const auto& rootFlowList = m_rootSkeleton.RefSortedFlowList();
	if (!shootFlowList.empty())
	{
		DistributePipes(true, pipeModelParameters);
		CalculatePipeTransforms(m_shootSkeleton, pipeModelParameters);
	}
	if (!rootFlowList.empty())
	{
		DistributePipes(false, pipeModelParameters);
		CalculatePipeTransforms(m_rootSkeleton, pipeModelParameters);
	}
}
