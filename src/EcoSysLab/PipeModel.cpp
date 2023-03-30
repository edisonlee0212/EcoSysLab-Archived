#include "PipeModel.hpp"

using namespace EcoSysLab;

void PipeModel::CalculateGraph(const PipeModelParameters& pipeModelParameters)
{
	{
		auto& pipeGroup = m_shootSkeleton.m_data.m_pipeGroup;
		const auto& gridGroup = m_shootSkeleton.m_data.m_hexagonGridGroup;
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
			pipeInfo.m_globalPosition = nodeInfo.m_globalPosition + left * pipeInfo.m_localPosition.x + up * pipeInfo.m_localPosition.y;
			pipeInfo.m_globalRotation = nodeInfo.m_regulatedGlobalRotation;
		}

		for (auto& pipe : pipeGroup.RefPipes())
		{
			if (pipe.IsRecycled()) continue;
			pipe.m_info.m_color = glm::mix(pipeGroup.m_data.m_innerColor, pipeGroup.m_data.m_outerColor, pipe.m_data.m_startAge - glm::floor(pipe.m_data.m_startAge));
		}
	}
	{
		auto& pipeGroup = m_rootSkeleton.m_data.m_pipeGroup;
		const auto& gridGroup = m_rootSkeleton.m_data.m_hexagonGridGroup;
		for (auto& pipeNode : pipeGroup.RefPipeNodes())
		{
			if (pipeNode.IsRecycled()) continue;
			const auto& node = m_rootSkeleton.PeekNode(pipeNode.m_data.m_nodeHandle);
			const auto& nodeInfo = node.m_info;
			const glm::vec3 left = nodeInfo.m_regulatedGlobalRotation * glm::vec3(1, 0, 0);
			const glm::vec3 up = nodeInfo.m_regulatedGlobalRotation * glm::vec3(0, 1, 0);
			const glm::vec3 front = nodeInfo.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
			auto& pipeInfo = pipeNode.m_info;
			const auto& grid = gridGroup.PeekGrid(node.m_data.m_gridHandle);
			const auto& cell = grid.PeekCell(pipeNode.m_data.m_cellHandle);

			pipeInfo.m_localPosition = pipeModelParameters.m_endNodeThickness * 2.0f * grid.GetPosition(cell.GetCoordinate());
			pipeInfo.m_thickness = pipeModelParameters.m_endNodeThickness;
			pipeInfo.m_globalPosition = nodeInfo.m_globalPosition + left * pipeInfo.m_localPosition.x + up * pipeInfo.m_localPosition.y;
			pipeInfo.m_globalRotation = nodeInfo.m_regulatedGlobalRotation;
		}

		for (auto& pipe : pipeGroup.RefPipes())
		{
			if (pipe.IsRecycled()) continue;
			pipe.m_info.m_color = glm::mix(pipeGroup.m_data.m_innerColor, pipeGroup.m_data.m_outerColor, pipe.m_data.m_startAge - glm::floor(pipe.m_data.m_startAge));
		}
	}
}
