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
				baseCell.m_data.m_pipeSegmentHandle = newPipeSegmentHandle;
				
				const auto newCellHandle = newProfile.AllocateCell();
				auto& newCell = newProfile.RefCell(newCellHandle);
				newCell.m_data.m_pipeHandle = newPipeHandle;
				newCell.m_data.m_pipeSegmentHandle = newPipeSegmentHandle;
				newCell.m_info.m_offset = glm::vec2(0.0f);
				newCell.m_info.m_radius = endNodeThickness;
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
				}
			}
		}else
		{
			//If this node is formed from branching, we need to construct new pipe.
			//First, we need to collect a chain of nodes from current node to the root.
			std::vector<NodeHandle> parentNodeToRootChain;
			while(parentNodeHandle != -1)
			{
				parentNodeToRootChain.emplace_back(parentNodeHandle);
				parentNodeHandle = skeleton.PeekNode(parentNodeHandle).GetParentHandle();
			}
			const auto baseCellHandle = baseProfile.AllocateCell();
			const auto newPipeHandle = pipeGroup.AllocatePipe();
			const auto newBasePipeSegmentHandle = pipeGroup.Extend(newPipeHandle);
			auto& newBasePipeSegment = pipeGroup.RefPipeSegment(newBasePipeSegmentHandle);
			newBasePipeSegment.m_data.m_nodeHandle = parentNodeToRootChain.back();

			auto& baseCell = baseProfile.RefCell(baseCellHandle);
			baseCell.m_data.m_pipeHandle = newPipeHandle;
			baseCell.m_data.m_pipeSegmentHandle = newBasePipeSegmentHandle;
			for (auto it = parentNodeToRootChain.rbegin(); it != parentNodeToRootChain.rend(); it++) {
				const auto newPipeSegmentHandle = pipeGroup.Extend(newPipeHandle);
				auto& newPipeSegment = pipeGroup.RefPipeSegment(newPipeSegmentHandle);
				newPipeSegment.m_data.m_nodeHandle = *it;

				const auto& nodeInChain = skeleton.PeekNode(*it);
				auto& profile = profileGroup.RefProfile(nodeInChain.m_data.m_profileHandle);
				const auto newCellHandle = profile.AllocateCell();
				auto& newCell = profile.RefCell(newCellHandle);
				newCell.m_data.m_pipeHandle = newPipeHandle;
				newCell.m_data.m_pipeSegmentHandle = newPipeSegmentHandle;
				newCell.m_info.m_radius = endNodeThickness;
				newCell.m_info.m_offset = glm::ballRand(glm::pow(static_cast<float>(profile.PeekCells().size()), 0.5f)) * endNodeThickness;
			}

			const auto newPipeSegmentHandle = pipeGroup.Extend(newPipeHandle);
			auto& newPipeSegment = pipeGroup.RefPipeSegment(newPipeSegmentHandle);
			newPipeSegment.m_data.m_nodeHandle = nodePair.second;

			const auto newCellHandle = newProfile.AllocateCell();
			auto& newCell = newProfile.RefCell(newCellHandle);
			newCell.m_data.m_pipeHandle = newPipeHandle;
			newCell.m_data.m_pipeSegmentHandle = newPipeSegmentHandle;
			newCell.m_info.m_offset = glm::vec2(0.0f);
			newCell.m_info.m_radius = endNodeThickness;
		}
		
	}
	m_shootPipeModel.CalculatePipeSegmentInfos(m_pipeModelParameters);
}
