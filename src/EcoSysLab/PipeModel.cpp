#include "PipeModel.hpp"
#include "algorithm"
#include "PipeProfile.hpp"

using namespace EcoSysLab;

void PipeModel::MapProfiles(ProfileHandle srcProfileHandle, const std::vector<ProfileHandle>& dstProfileHandles)
{
	const auto& srcProfile = m_pipeProfileGroup.PeekProfile(srcProfileHandle);
	//Simply assign profile cell one by one.
	std::vector<PipeHandle> srcPipeHandles;


	std::vector<std::pair<ProfileHandle, CellHandle>> dstCells;
	for (const auto& dstProfileHandle : dstProfileHandles)
	{
		auto& dstProfile = m_pipeProfileGroup.RefProfile(dstProfileHandle);
		for (auto& dstCell : dstProfile.RefCells())
		{
			if (dstCell.IsRecycled()) continue;
			dstCell.m_data.m_pipeHandle = -1;
			dstCells.emplace_back(dstProfileHandle, dstCell.GetHandle());
		}
	}
	std::random_device rd;
	std::mt19937 g(rd());
	std::shuffle(dstCells.begin(), dstCells.end(), g);
	int dstCellIndex = 0;
	for (const auto& srcCell : srcProfile.PeekCells())
	{
		if (srcCell.IsRecycled() || srcCell.m_data.m_pipeHandle == -1) continue;
		if (dstCellIndex >= dstCells.size()) break;
		auto& dstCell = m_pipeProfileGroup.RefProfile(dstCells[dstCellIndex].first).RefCell(dstCells[dstCellIndex].second);
		dstCell.m_data.m_pipeHandle = srcCell.m_data.m_pipeHandle;
		dstCellIndex++;

		/*
		//Break if we ran out of dst profile handles.
		while (dstProfileIndex < dstProfileHandles.size()
			&& dstProfileCellIndex >= m_pipeProfileGroup.RefProfile(dstProfileHandles[dstProfileIndex]).PeekCells().size())
		{
			dstProfileIndex++;
			dstProfileCellIndex = 0;
		}
		if (dstProfileIndex >= dstProfileHandles.size()) break;
		auto& dstCell = m_pipeProfileGroup.RefProfile(dstProfileHandles[dstProfileIndex]).RefCell(dstProfileCellIndex);
		dstProfileCellIndex++;
		dstCell.m_data.m_pipeHandle = srcCell.m_data.m_pipeHandle;
		*/
	}
}

void PipeModel::InitializePipes(const PipeModelParameters& pipeModelParameters)
{
	const auto nodeList = m_skeleton.RefSortedNodeList();
	for (auto it = nodeList.rbegin(); it != nodeList.rend(); ++it)
	{
		auto& node = m_skeleton.RefNode(*it);
		auto& nodeData = node.m_data;
		if (node.IsEndNode()) nodeData.m_endNodeCount = 1;
		else
		{
			nodeData.m_endNodeCount = 0;
			for (const auto& childNodeHandle : node.RefChildHandles())
			{
				const auto& childNode = m_skeleton.RefNode(childNodeHandle);
				nodeData.m_endNodeCount += childNode.m_data.m_endNodeCount;
			}
		}
	}
	m_pipeGroup = {};

	//0. Allocate pipes for target skeleton.
	auto& baseProfile = m_pipeProfileGroup.RefProfile(m_skeleton.m_data.m_baseProfileHandle);
	auto& baseNode = m_skeleton.RefNode(0);
	for (const auto& readOnlyCell : baseProfile.PeekCells())
	{
		if (readOnlyCell.IsRecycled()) continue;
		auto& cell = baseProfile.RefCell(readOnlyCell.GetHandle());
		const auto newPipeHandle = m_pipeGroup.AllocatePipe();
		cell.m_data.m_pipeHandle = newPipeHandle;

		auto& pipe = m_pipeGroup.RefPipe(newPipeHandle);
		const glm::vec3 left = baseNode.m_info.m_regulatedGlobalRotation * glm::vec3(1, 0, 0);
		const glm::vec3 up = baseNode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 1, 0);
		auto localPosition = pipeModelParameters.m_profileScale * cell.m_info.m_offset;
		pipe.m_info.m_baseInfo.m_globalPosition = baseNode.m_info.m_globalPosition + left * localPosition.x + up * localPosition.y;
		pipe.m_info.m_baseInfo.m_globalRotation = baseNode.m_info.m_regulatedGlobalRotation;
		pipe.m_info.m_baseInfo.m_thickness = pipeModelParameters.m_profileScale * cell.m_info.m_radius;
	}

	//1. Map first node profile from base profile.
	const std::vector firstProfileHandle = { m_skeleton.RefNode(0).m_data.m_profileHandle };
	MapProfiles(m_skeleton.m_data.m_baseProfileHandle, firstProfileHandle);

	//2. Map all profiles.
	for (const auto& i : nodeList)
	{
		const auto& node = m_skeleton.RefNode(i);
		std::vector<ProfileHandle> childProfileHandles;
		for (const auto& childNodeHandle : node.RefChildHandles())
		{
			const auto& childNode = m_skeleton.RefNode(childNodeHandle);
			childProfileHandles.emplace_back(childNode.m_data.m_profileHandle);
		}
		MapProfiles(node.m_data.m_profileHandle, childProfileHandles);
	}

	//3. Extend pipes according to the profile.
	for (const auto& i : nodeList)
	{
		const auto& node = m_skeleton.PeekNode(i);
		const auto profileHandle = node.m_data.m_profileHandle;
		auto& profile = m_pipeProfileGroup.RefProfile(profileHandle);
		for (auto& cell : profile.RefCells())
		{
			if (cell.IsRecycled()) continue;
			if (cell.m_data.m_pipeHandle == -1) continue;
			cell.m_data.m_pipeSegmentHandle = m_pipeGroup.Extend(cell.m_data.m_pipeHandle);
			auto& pipeSegment = m_pipeGroup.RefPipeSegment(cell.m_data.m_pipeSegmentHandle);
			pipeSegment.m_data.m_nodeHandle = i;
			pipeSegment.m_info.m_localPosition = pipeModelParameters.m_profileScale * cell.m_info.m_offset;
			pipeSegment.m_info.m_thickness = pipeModelParameters.m_profileScale * cell.m_info.m_radius;
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
		pipeSegmentInfo.m_globalPosition = nodeInfo.m_globalPosition + left * pipeSegmentInfo.m_localPosition.x + up * pipeSegmentInfo.m_localPosition.y;
		pipeSegmentInfo.m_globalRotation = nodeInfo.m_regulatedGlobalRotation;
	}
}
