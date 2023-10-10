#include "PipeModel.hpp"
#include "algorithm"
#include "PipeProfile.hpp"

using namespace EcoSysLab;

void PipeModel::CalculatePipeSegmentInfos(const PipeModelParameters& pipeModelParameters)
{
	auto& baseProfile = m_pipeProfileGroup.RefProfile(m_skeleton.m_data.m_baseProfileHandle);
	const auto& baseNode = m_skeleton.RefNode(0);
	for (const auto& readOnlyCell : baseProfile.PeekCells())
	{
		if (readOnlyCell.IsRecycled()) continue;
		auto& cell = baseProfile.RefCell(readOnlyCell.GetHandle());
		auto& pipe = m_pipeGroup.RefPipe(cell.m_data.m_pipeHandle);
		const glm::vec3 left = baseNode.m_info.m_regulatedGlobalRotation * glm::vec3(1, 0, 0);
		const glm::vec3 up = baseNode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 1, 0);
		const auto localPosition = cell.m_info.m_offset * baseProfile.m_info.m_cellRadius;
		pipe.m_info.m_baseInfo.m_globalPosition = baseNode.m_info.m_globalPosition + left * localPosition.x + up * localPosition.y;
		pipe.m_info.m_baseInfo.m_globalRotation = baseNode.m_info.m_regulatedGlobalRotation;
		pipe.m_info.m_baseInfo.m_thickness = baseProfile.m_info.m_cellRadius;
	}
	const auto nodeList = m_skeleton.RefSortedNodeList();
	for (const auto& i : nodeList)
	{
		const auto& node = m_skeleton.PeekNode(i);
		const auto profileHandle = node.m_data.m_profileHandle;
		auto& profile = m_pipeProfileGroup.RefProfile(profileHandle);
		for (auto& cell : profile.RefCells())
		{
			if (cell.IsRecycled()) continue;
			if (cell.m_data.m_pipeHandle == -1) continue;
			auto& pipeSegment = m_pipeGroup.RefPipeSegment(cell.m_data.m_pipeSegmentHandle);
			pipeSegment.m_data.m_nodeHandle = i;
			pipeSegment.m_info.m_localPosition = cell.m_info.m_offset * profile.m_info.m_cellRadius;
			pipeSegment.m_info.m_thickness = profile.m_info.m_cellRadius;
		}
	}

	for (auto& pipeSegment : m_pipeGroup.RefPipeSegments())
	{
		if (pipeSegment.IsRecycled()) continue;
		const auto& node = m_skeleton.PeekNode(pipeSegment.m_data.m_nodeHandle);
		const auto& nodeInfo = node.m_info;
		const glm::vec3 left = nodeInfo.m_regulatedGlobalRotation * glm::vec3(1, 0, 0);
		const glm::vec3 up = nodeInfo.m_regulatedGlobalRotation * glm::vec3(0, 1, 0);
		auto& pipeSegmentInfo = pipeSegment.m_info;
		pipeSegmentInfo.m_globalPosition = nodeInfo.m_globalPosition + nodeInfo.m_globalDirection * nodeInfo.m_length + left * pipeSegmentInfo.m_localPosition.x + up * pipeSegmentInfo.m_localPosition.y;
		pipeSegmentInfo.m_globalRotation = nodeInfo.m_regulatedGlobalRotation;
	}
}