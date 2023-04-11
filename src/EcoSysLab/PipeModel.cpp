#include "PipeModel.hpp"

using namespace EcoSysLab;

void PipeModel::CalculatePipeLocalPositions(PipeModelSkeleton& targetSkeleton,
	const PipeModelParameters& pipeModelParameters)
{
	auto& pipeGroup = targetSkeleton.m_data.m_pipeGroup;
	for (auto& pipeNode : pipeGroup.RefPipeNodes())
	{
		if (pipeNode.IsRecycled()) continue;
		const auto& node = targetSkeleton.PeekNode(pipeNode.m_data.m_nodeHandle);
		auto& pipeInfo = pipeNode.m_info;
		pipeInfo.m_thickness = pipeModelParameters.m_endNodeThickness;
	}

	for (auto& pipe : pipeGroup.RefPipes())
	{
		if (pipe.IsRecycled()) continue;
		pipe.m_info.m_color = glm::vec4(1.0f);
	}
}

void PipeModel::CalculatePipeTransforms(PipeModelSkeleton& targetSkeleton, const PipeModelParameters& pipeModelParameters)
{
	auto& pipeGroup = targetSkeleton.m_data.m_pipeGroup;
	for (auto& pipeNode : pipeGroup.RefPipeNodes())
	{
		if (pipeNode.IsRecycled()) continue;
		const auto& node = targetSkeleton.PeekNode(pipeNode.m_data.m_nodeHandle);
		const auto& nodeInfo = node.m_info;
		const glm::vec3 left = nodeInfo.m_regulatedGlobalRotation * glm::vec3(1, 0, 0);
		const glm::vec3 up = nodeInfo.m_regulatedGlobalRotation * glm::vec3(0, 1, 0);
		const glm::vec3 front = nodeInfo.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
		auto& pipeInfo = pipeNode.m_info;
		pipeInfo.m_globalPosition = nodeInfo.m_globalPosition + front * nodeInfo.m_length + left * pipeInfo.m_localPosition.x + up * pipeInfo.m_localPosition.y;
		pipeInfo.m_globalRotation = nodeInfo.m_regulatedGlobalRotation;
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
	targetSkeleton.m_data.m_pipeGroup = {};
	auto& pipeGroup = targetSkeleton.m_data.m_pipeGroup;
	PipeModelHexagonGridGroup gridGroup;
	std::unordered_map<NodeHandle, HexagonGridHandle> gridHandleMap;
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
		gridHandleMap[0] = firstGridHandle;
		const auto firstPipeNodeHandle = pipeGroup.Extend(newPipeHandle);
		auto& firstPipeNode = pipeGroup.RefPipeNode(firstPipeNodeHandle);
		firstPipeNode.m_info.m_localPosition = pipeModelParameters.m_endNodeThickness * 2.0f * firstGrid.GetPosition(cell.GetCoordinate());
		firstPipeNode.m_data.m_nodeHandle = 0;

	}
	//3. Create traverse graph and setup pipes.
	for (const auto& nodeHandle : nodeList)
	{
		auto& node = targetSkeleton.RefNode(nodeHandle);
		auto& nodeData = node.m_data;
		//Create a hexagon grid for every node that has multiple child, and a hexagon grid for each child.
		const auto currentGridHandle = gridHandleMap.at(nodeHandle);
		//No pipe left for this node.
		if(currentGridHandle < 0) continue;
		if (node.RefChildHandles().size() > 1) {
			const auto newGridHandle = gridGroup.Allocate();
			gridHandleMap[nodeHandle] = newGridHandle;
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

				newPipeNode.m_info.m_localPosition = pipeModelParameters.m_endNodeThickness * 2.0f * firstGrid.GetPosition(newCell.GetCoordinate());
				newPipeNode.m_data.m_nodeHandle = nodeHandle;
			}
			SplitPipes(gridHandleMap, gridGroup, targetSkeleton, nodeHandle, newGridHandle, pipeModelParameters);
		}
		else if(node.RefChildHandles().size() == 1)
		{
			const auto childNodeHandle = node.RefChildHandles()[0];
			auto& childNode = targetSkeleton.RefNode(childNodeHandle);
			auto& childNodeData = childNode.m_data;
			//Extend all pipe nodes from parent.
			gridHandleMap[childNodeHandle] = currentGridHandle;
			const auto& previousGrid = gridGroup.PeekGrid(currentGridHandle);
			for (const auto& parentCell : previousGrid.PeekCells())
			{
				if (parentCell.IsRecycled()) continue;

				const auto newPipeNodeHandle = pipeGroup.Extend(parentCell.m_data.m_pipeHandle);
				auto& newPipeNode = pipeGroup.RefPipeNode(newPipeNodeHandle);
				newPipeNode.m_info.m_localPosition = pipeModelParameters.m_endNodeThickness * 2.0f * firstGrid.GetPosition(parentCell.GetCoordinate());
				newPipeNode.m_data.m_nodeHandle = childNodeHandle;
			}
		}
	}
}

void PipeModel::SplitPipes(std::unordered_map<NodeHandle, HexagonGridHandle>& gridHandleMap, 
	PipeModelHexagonGridGroup& gridGroup, PipeModelSkeleton& targetSkeleton, 
	NodeHandle nodeHandle, HexagonGridHandle newGridHandle, const PipeModelParameters& pipeModelParameters) const
{
	auto& pipeGroup = targetSkeleton.m_data.m_pipeGroup;
	auto& node = targetSkeleton.RefNode(nodeHandle);
	auto& newGrid = gridGroup.RefGrid(newGridHandle);
	const auto nodeUp = node.m_info.m_regulatedGlobalRotation * glm::vec3(0, 1, 0);
	const auto nodeLeft = node.m_info.m_regulatedGlobalRotation * glm::vec3(1, 0, 0);
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
		if (cellCount == 0) continue;

		const auto childNewGridHandle = gridGroup.Allocate();
		gridHandleMap[it->second.first] = childNewGridHandle;
		auto& childNewGrid = gridGroup.RefGrid(childNewGridHandle);
		const auto& prevGrid = gridGroup.RefGrid(newGridHandle);
		//1. Find the start cell.
		auto childNodeFront = glm::inverse(node.m_info.m_regulatedGlobalRotation) * childNode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
		glm::vec2 direction = glm::normalize(glm::vec2(childNodeFront.x, childNodeFront.y));
		/*
		childNodeFront = childNode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
		glm::vec2 direction2 = glm::normalize(glm::vec2(glm::dot(childNodeFront, nodeLeft) / glm::length(nodeLeft),
			glm::dot(childNodeFront, nodeUp) / glm::length(nodeUp)));
		UNIENGINE_LOG(std::to_string(glm::dot(direction, direction2)));
		*/
		std::multimap<float, HexagonCellHandle> sortedKnots;
		auto avgPosition = glm::vec2(0.0f);
		glm::ivec2 sumCoordinate = glm::ivec2(0, 0);
		for (const auto& i : newGridCellMap) {
			const auto& prevCell = prevGrid.PeekCell(i.second);
			avgPosition += prevGrid.GetPosition(prevCell.GetCoordinate());
			sumCoordinate += prevCell.GetCoordinate();
		}
		avgPosition /= newGridCellMap.size();
		for (const auto& i : newGridCellMap) {
			const auto& prevCell = prevGrid.PeekCell(i.second);
			auto position = prevGrid.GetPosition(prevCell.GetCoordinate()) - avgPosition;
			if (position == glm::vec2(0.0f)) continue;
			float distance = glm::dot(position, direction) / glm::length(direction);
			sortedKnots.insert({ -distance, prevCell.GetHandle() });
		}
		//2. Sort cell based on distance to the start cell
		auto baseCellHandle = sortedKnots.begin()->second;
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
		glm::ivec2 extractedCellSumCoordinate = glm::ivec2(0, 0);
		for (int i = 0; i < cellCount; i++)
		{
			const auto allocatedCellHandle = distanceSortedCellHandles.begin()->second;
			distanceSortedCellHandles.erase(distanceSortedCellHandles.begin());
			const auto& allocatedCell = prevGrid.PeekCell(allocatedCellHandle);
			auto coordinate = allocatedCell.GetCoordinate();
			newGridCellMap.erase({ coordinate.x, coordinate.y });
			if (it != childNodeHandles.rbegin()) {
				const auto& map = childNewGrid.PeekCellMap();
				coordinate = glm::ivec2(0, 0);
				auto search = map.find({ 0, 0 });
				if (search != map.end())
				{
					coordinate = childNewGrid.FindAvailableCoordinate(search->second, glm::circularRand(1.0f));
				}
			}
			extractedCellSumCoordinate += coordinate;
			const auto childNewCellHandle = childNewGrid.Allocate(coordinate);
			auto& childNewCell = childNewGrid.RefCell(childNewCellHandle);
			childNewCell.m_data.m_pipeHandle = allocatedCell.m_data.m_pipeHandle;
		}
		const auto shiftPosition = childNewGrid.GetPosition(extractedCellSumCoordinate / cellCount);
		childNewGrid.ShiftCoordinate(-extractedCellSumCoordinate / cellCount);

		childNode.m_info.m_localPosition = pipeModelParameters.m_endNodeThickness * 2.0f * (nodeLeft * shiftPosition.x + nodeUp * shiftPosition.y);
		for (const auto& cell : childNewGrid.PeekCells())
		{
			if (cell.IsRecycled()) continue;
			auto& childNewCell = childNewGrid.RefCell(cell.GetHandle());
			const auto childNewPipeNodeHandle = pipeGroup.Extend(childNewCell.m_data.m_pipeHandle);
			auto& childNewPipeNode = pipeGroup.RefPipeNode(childNewPipeNodeHandle);
			childNewPipeNode.m_info.m_localPosition = pipeModelParameters.m_endNodeThickness * 2.0f * childNewGrid.GetPosition(childNewCell.GetCoordinate());
			childNewPipeNode.m_data.m_nodeHandle = it->second.first;
		}
	}
}

void PipeModel::InitializePipes(const PipeModelParameters& pipeModelParameters)
{
	if(m_baseGrid.GetCellCount() == 0) return;
	const auto& shootFlowList = m_shootSkeleton.RefSortedFlowList();
	const auto& rootFlowList = m_rootSkeleton.RefSortedFlowList();
	if (!shootFlowList.empty())
	{
		DistributePipes(true, pipeModelParameters);
		CalculatePipeLocalPositions(m_shootSkeleton, pipeModelParameters);
		m_shootSkeleton.CalculateTransforms();
		CalculatePipeTransforms(m_shootSkeleton, pipeModelParameters);
	}
	if (!rootFlowList.empty())
	{
		DistributePipes(false, pipeModelParameters);
		CalculatePipeLocalPositions(m_rootSkeleton, pipeModelParameters);
		m_rootSkeleton.CalculateTransforms();
		CalculatePipeTransforms(m_rootSkeleton, pipeModelParameters);
	}
}
