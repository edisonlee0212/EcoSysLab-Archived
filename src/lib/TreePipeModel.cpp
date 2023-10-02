#include "TreePipeModel.hpp"

using namespace EcoSysLab;

void TreePipeModel::UpdatePipeModels(const TreeModel& targetTreeModel)
{
	m_shootPipeModel = {};
	m_rootPipeModel = {};

	float endNodeThickness = 0.1f;
	auto& skeleton = m_shootPipeModel.m_skeleton;
	auto& profileGroup = m_shootPipeModel.m_pipeProfileGroup;
	auto& pipeGroup = m_shootPipeModel.m_pipeGroup;
	
	skeleton.Clone(targetTreeModel.PeekShootSkeleton(), [&](NodeHandle srcNodeHandle, NodeHandle dstNodeHandle) {});
	skeleton.m_data.m_baseProfileHandle = profileGroup.Allocate();
	
	std::map<int, NodeHandle> shootNewNodeList;
	for(const auto& i : skeleton.RefRawNodes())
	{
		shootNewNodeList[i.GetIndex()] = i.GetHandle();
	}
	for(const auto& nodePair : shootNewNodeList)
	{
		auto& node = skeleton.RefNode(nodePair.second);
		node.m_data.m_profileHandle = profileGroup.Allocate();
		auto& baseProfile = profileGroup.RefProfile(skeleton.m_data.m_baseProfileHandle);
		auto& newProfile = profileGroup.RefProfile(node.m_data.m_profileHandle);
		auto parentNodeHandle = node.GetParentHandle();
		if(node.IsApical())
		{
			//If this node is formed from elongation, we simply extend the pipe.
			if(parentNodeHandle == -1)
			{
				//Root node. We establish first pipe and allocate cell for both base profile and new profile.
				const auto baseCellHandle = baseProfile.AllocateCell();
				const auto newPipeHandle = pipeGroup.AllocatePipe();
				const auto newPipeSegmentHandle = pipeGroup.Extend(newPipeHandle);
				auto& newPipeSegment = pipeGroup.RefPipeSegment(newPipeSegmentHandle);
				newPipeSegment.m_data.m_nodeHandle = nodePair.second;
				
				auto& baseCell = baseProfile.RefCell(baseCellHandle);
				baseCell.m_data.m_pipeHandle = newPipeHandle;
				baseCell.m_data.m_pipeSegmentHandle = -1;
				
				const auto newCellHandle = newProfile.AllocateCell();
				auto& newCell = newProfile.RefCell(newCellHandle);
				newCell.m_data.m_pipeHandle = newPipeHandle;
				newCell.m_data.m_pipeSegmentHandle = newPipeSegmentHandle;
				newCell.m_info.m_offset = glm::vec2(0.0f);
				newCell.m_info.m_radius = endNodeThickness;

				newPipeSegment.m_data.m_cellHandle = newCellHandle;

				node.m_data.m_pipeHandle = newPipeHandle;
			}else
			{
				const auto& parentNodeProfile = profileGroup.PeekProfile(skeleton.PeekNode(parentNodeHandle).m_data.m_profileHandle);
				for(const auto& cell : parentNodeProfile.PeekCells())
				{
					const auto newPipeSegmentHandle = pipeGroup.Extend(cell.m_data.m_pipeHandle);
					auto& newPipeSegment = pipeGroup.RefPipeSegment(newPipeSegmentHandle);
					newPipeSegment.m_data.m_nodeHandle = nodePair.second;

					const auto newCellHandle = newProfile.AllocateCell();
					auto& newCell = newProfile.RefCell(newCellHandle);
					newCell.m_data.m_pipeHandle = cell.m_data.m_pipeHandle;
					newCell.m_data.m_pipeSegmentHandle = newPipeSegmentHandle;
					newCell.m_info.m_offset = cell.m_info.m_offset;
					newCell.m_info.m_radius = endNodeThickness;

					newPipeSegment.m_data.m_cellHandle = newCellHandle;
				}
				const auto& parentNode = skeleton.PeekNode(parentNodeHandle);
				node.m_data.m_pipeHandle = parentNode.m_data.m_pipeHandle;
			}
		}else
		{
			//If this node is formed from branching, we need to construct new pipe.
			//First, we need to collect a chain of nodes from current node to the root.
			const auto& parentNode = skeleton.PeekNode(parentNodeHandle);
			const auto& parentPipe = pipeGroup.RefPipe(parentNode.m_data.m_pipeHandle);
			const auto& parentPipeSegments = parentPipe.PeekPipeSegmentHandles();
			std::vector<NodeHandle> parentNodeToRootChain;
			std::vector<PipeSegmentHandle> rootToParentNodePipeSegmentChain;
			int segmentIndex = 0;
			while(parentNodeHandle != -1)
			{
				parentNodeToRootChain.emplace_back(parentNodeHandle);
				parentNodeHandle = skeleton.PeekNode(parentNodeHandle).GetParentHandle();

				rootToParentNodePipeSegmentChain.emplace_back(parentPipeSegments[segmentIndex]);
				segmentIndex++;
			}

			auto direction = glm::circularRand(1.0f);

			const auto baseCellHandle = baseProfile.AllocateCell();
			node.m_data.m_pipeHandle = pipeGroup.AllocatePipe();
			
			auto& baseCell = baseProfile.RefCell(baseCellHandle);
			baseCell.m_data.m_pipeHandle = node.m_data.m_pipeHandle;
			baseCell.m_data.m_pipeSegmentHandle = -1;
			baseCell.m_info.m_radius = endNodeThickness;
			baseCell.m_info.m_offset = baseProfile.FindAvailablePosition(baseProfile.RefCell(pipeGroup.RefPipeSegment(rootToParentNodePipeSegmentChain[0]).m_data.m_cellHandle).m_info.m_offset, direction, endNodeThickness);
			segmentIndex = 0;
			for (auto it = parentNodeToRootChain.rbegin(); it != parentNodeToRootChain.rend(); it++) {
				const auto newPipeSegmentHandle = pipeGroup.Extend(node.m_data.m_pipeHandle);
				auto& newPipeSegment = pipeGroup.RefPipeSegment(newPipeSegmentHandle);
				newPipeSegment.m_data.m_nodeHandle = *it;

				const auto& nodeInChain = skeleton.PeekNode(*it);
				auto& profile = profileGroup.RefProfile(nodeInChain.m_data.m_profileHandle);
				const auto newCellHandle = profile.AllocateCell();
				auto& newCell = profile.RefCell(newCellHandle);
				newCell.m_data.m_pipeHandle = node.m_data.m_pipeHandle;
				newCell.m_data.m_pipeSegmentHandle = newPipeSegmentHandle;
				newCell.m_info.m_radius = endNodeThickness;
				newCell.m_info.m_offset = profile.FindAvailablePosition(profile.RefCell(pipeGroup.RefPipeSegment(rootToParentNodePipeSegmentChain[segmentIndex]).m_data.m_cellHandle).m_info.m_offset, direction, endNodeThickness); //glm::ballRand(glm::pow(static_cast<float>(profile.PeekCells().size()), 0.5f)) * endNodeThickness;

				newPipeSegment.m_data.m_cellHandle = newCellHandle;

				segmentIndex++;
			}

			const auto newPipeSegmentHandle = pipeGroup.Extend(node.m_data.m_pipeHandle);
			auto& newPipeSegment = pipeGroup.RefPipeSegment(newPipeSegmentHandle);
			newPipeSegment.m_data.m_nodeHandle = nodePair.second;

			const auto newCellHandle = newProfile.AllocateCell();
			auto& newCell = newProfile.RefCell(newCellHandle);
			newCell.m_data.m_pipeHandle = node.m_data.m_pipeHandle;
			newCell.m_data.m_pipeSegmentHandle = newPipeSegmentHandle;
			newCell.m_info.m_offset = glm::vec2(0.0f);
			newCell.m_info.m_radius = endNodeThickness;

			newPipeSegment.m_data.m_cellHandle = newCellHandle;
		}
		
	}
	
}
