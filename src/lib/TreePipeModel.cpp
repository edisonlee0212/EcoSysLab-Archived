#include "TreePipeModel.hpp"

#include "EcoSysLabLayer.hpp"

using namespace EcoSysLab;


void TreePipeModel::UpdatePipeModels(const TreeModel& targetTreeModel, const PipeModelParameters& pipeModelParameters)
{
	m_shootPipeModel = {};
	m_rootPipeModel = {};

	auto& skeleton = m_shootPipeModel.m_skeleton;
	auto& profileGroup = m_shootPipeModel.m_pipeProfileGroup;
	auto& pipeGroup = m_shootPipeModel.m_pipeGroup;

	skeleton.Clone(targetTreeModel.PeekShootSkeleton(), [&](NodeHandle srcNodeHandle, NodeHandle dstNodeHandle)
		{
			skeleton.m_data.m_nodeMap[srcNodeHandle] = dstNodeHandle;
		});
	skeleton.m_data.m_baseProfileHandle = profileGroup.Allocate();
	profileGroup.RefProfile(skeleton.m_data.m_baseProfileHandle).m_info.m_cellRadius = pipeModelParameters.m_profileDefaultCellRadius;

	std::map<int, NodeHandle> shootNewNodeList;
	for (const auto& i : skeleton.RefRawNodes())
	{
		shootNewNodeList[i.GetIndex()] = i.GetHandle();
	}
	if (shootNewNodeList.empty()) return;
	for (const auto& nodePair : shootNewNodeList)
	{
		auto& node = skeleton.RefNode(nodePair.second);
		node.m_data.m_profileHandle = profileGroup.Allocate();
		auto& baseProfile = profileGroup.RefProfile(skeleton.m_data.m_baseProfileHandle);
		auto& newProfile = profileGroup.RefProfile(node.m_data.m_profileHandle);
		newProfile.m_info.m_cellRadius = pipeModelParameters.m_profileDefaultCellRadius;
		auto parentNodeHandle = node.GetParentHandle();
		if (node.IsApical())
		{
			//If this node is formed from elongation, we simply extend the pipe.
			if (parentNodeHandle == -1)
			{
				//Root node. We establish first pipe and allocate cell for both base profile and new profile.
				const auto baseCellHandle = baseProfile.AllocateCell();
				const auto newPipeHandle = pipeGroup.AllocatePipe();
				const auto newPipeSegmentHandle = pipeGroup.Extend(newPipeHandle);
				auto& newPipeSegment = pipeGroup.RefPipeSegment(newPipeSegmentHandle);
				newPipeSegment.m_data.m_nodeHandle = nodePair.second;
				newPipeSegment.m_data.m_flowHandle = node.GetFlowHandle();
				auto& baseCell = baseProfile.RefCell(baseCellHandle);
				baseCell.m_data.m_pipeHandle = newPipeHandle;
				baseCell.m_data.m_pipeSegmentHandle = -1;

				const auto newCellHandle = newProfile.AllocateCell();
				auto& newCell = newProfile.RefCell(newCellHandle);
				newCell.m_data.m_pipeHandle = newPipeHandle;
				newCell.m_data.m_pipeSegmentHandle = newPipeSegmentHandle;
				newCell.m_info.m_offset = glm::vec2(0.0f);

				newPipeSegment.m_data.m_cellHandle = newCellHandle;

				node.m_data.m_pipeHandle = newPipeHandle;
			}
			else
			{
				const auto& parentNodeProfile = profileGroup.PeekProfile(skeleton.PeekNode(parentNodeHandle).m_data.m_profileHandle);
				for (const auto& cell : parentNodeProfile.PeekCells())
				{
					const auto newPipeSegmentHandle = pipeGroup.Extend(cell.m_data.m_pipeHandle);
					auto& newPipeSegment = pipeGroup.RefPipeSegment(newPipeSegmentHandle);
					newPipeSegment.m_data.m_nodeHandle = nodePair.second;
					newPipeSegment.m_data.m_flowHandle = node.GetFlowHandle();
					const auto newCellHandle = newProfile.AllocateCell();
					auto& newCell = newProfile.RefCell(newCellHandle);
					newCell.m_data.m_pipeHandle = cell.m_data.m_pipeHandle;
					newCell.m_data.m_pipeSegmentHandle = newPipeSegmentHandle;
					newCell.m_info.m_offset = cell.m_info.m_offset;

					newPipeSegment.m_data.m_cellHandle = newCellHandle;
				}
				const auto& parentNode = skeleton.PeekNode(parentNodeHandle);
				node.m_data.m_pipeHandle = parentNode.m_data.m_pipeHandle;
			}
		}
		else
		{
			//If this node is formed from branching, we need to construct new pipe.
			//First, we need to collect a chain of nodes from current node to the root.
			const auto& parentNode = skeleton.PeekNode(parentNodeHandle);
			const auto& parentPipe = pipeGroup.RefPipe(parentNode.m_data.m_pipeHandle);
			const auto& parentPipeSegments = parentPipe.PeekPipeSegmentHandles();
			std::vector<NodeHandle> parentNodeToRootChain;
			std::vector<PipeSegmentHandle> rootToParentNodePipeSegmentChain;
			int segmentIndex = 0;
			while (parentNodeHandle != -1)
			{
				parentNodeToRootChain.emplace_back(parentNodeHandle);
				parentNodeHandle = skeleton.PeekNode(parentNodeHandle).GetParentHandle();

				rootToParentNodePipeSegmentChain.emplace_back(parentPipeSegments[segmentIndex]);
				segmentIndex++;
			}
			const auto baseCellHandle = baseProfile.AllocateCell();
			node.m_data.m_pipeHandle = pipeGroup.AllocatePipe();

			auto& baseCell = baseProfile.RefCell(baseCellHandle);
			baseCell.m_data.m_pipeHandle = node.m_data.m_pipeHandle;
			baseCell.m_data.m_pipeSegmentHandle = -1;

			baseCell.m_info.m_offset = glm::vec2(0.0f);
			segmentIndex = 0;
			for (auto it = parentNodeToRootChain.rbegin(); it != parentNodeToRootChain.rend(); ++it) {
				const auto newPipeSegmentHandle = pipeGroup.Extend(node.m_data.m_pipeHandle);
				auto& newPipeSegment = pipeGroup.RefPipeSegment(newPipeSegmentHandle);
				const auto& nodeInChain = skeleton.PeekNode(*it);
				newPipeSegment.m_data.m_nodeHandle = *it;
				newPipeSegment.m_data.m_flowHandle = nodeInChain.GetFlowHandle();

				auto& profile = profileGroup.RefProfile(nodeInChain.m_data.m_profileHandle);
				const auto newCellHandle = profile.AllocateCell();
				auto& newCell = profile.RefCell(newCellHandle);
				newCell.m_data.m_pipeHandle = node.m_data.m_pipeHandle;
				newCell.m_data.m_pipeSegmentHandle = newPipeSegmentHandle;

				newCell.m_info.m_offset = glm::vec2(0.0f);
				newPipeSegment.m_data.m_cellHandle = newCellHandle;

				segmentIndex++;
			}

			const auto newPipeSegmentHandle = pipeGroup.Extend(node.m_data.m_pipeHandle);
			auto& newPipeSegment = pipeGroup.RefPipeSegment(newPipeSegmentHandle);
			newPipeSegment.m_data.m_nodeHandle = nodePair.second;
			newPipeSegment.m_data.m_flowHandle = node.GetFlowHandle();
			const auto newCellHandle = newProfile.AllocateCell();

			auto& newCell = newProfile.RefCell(newCellHandle);
			newCell.m_data.m_pipeHandle = node.m_data.m_pipeHandle;
			newCell.m_data.m_pipeSegmentHandle = newPipeSegmentHandle;
			newCell.m_info.m_offset = glm::vec2(0.0f);
			newPipeSegment.m_data.m_cellHandle = newCellHandle;
		}

	}

	for (auto& profile : profileGroup.RefProfiles())
	{
		auto& profileData = profile.m_data;
		auto& physics2D = profileData.m_particlePhysics2D;
		physics2D.Reset(0.002f);
		for (auto& cell : profile.RefCells())
		{
			auto newParticleHandle = physics2D.AllocateParticle();
			auto& newParticle = physics2D.RefParticle(newParticleHandle);
			newParticle.m_data.m_cellHandle = cell.GetHandle();
			newParticle.SetDamping(pipeModelParameters.m_damping);
			newParticle.SetPosition(cell.m_info.m_offset);
			cell.m_data.m_particleHandle = newParticleHandle;
		}
	}
	const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
	const auto& randomColor = ecoSysLabLayer->RandomColors();
	const auto& sortedNodeList = skeleton.RefSortedNodeList();
	const auto& sortedFlowList = skeleton.RefSortedFlowList();
	//1. Packing
	for (auto it = sortedFlowList.rbegin(); it != sortedFlowList.rend(); ++it)
	{
		auto& flow = skeleton.RefFlow(*it);
		const auto& childFlowHandles = flow.RefChildHandles();
		const auto flowStartNodeHandle = flow.RefNodeHandles().front();
		const auto& flowStartNode = skeleton.RefNode(flowStartNodeHandle);
		const auto& flowEndNode = skeleton.RefNode(flow.RefNodeHandles().back());
		auto& profile = profileGroup.RefProfile(flowStartNode.m_data.m_profileHandle);
		auto& profileData = profile.m_data;
		auto& physics2D = profileData.m_particlePhysics2D;
		if (childFlowHandles.empty())
		{
			//For flow start, set only particle at the center.
			for (auto& particle : physics2D.RefParticles())
			{
				particle.SetColor(glm::vec4(1.0f));
				particle.SetPosition(glm::vec2(0.0f));
			}
		}
		else if (childFlowHandles.size() == 1)
		{
			//Copy from child flow start to self flow start
			auto& childNode = skeleton.RefNode(flowEndNode.RefChildHandles().at(0));
			auto& childProfile = profileGroup.RefProfile(childNode.m_data.m_profileHandle);
			auto& childPhysics2D = childProfile.m_data.m_particlePhysics2D;
			for (const auto& cell : profile.RefCells())
			{
				const auto& pipeSegment = pipeGroup.RefPipeSegment(cell.m_data.m_pipeSegmentHandle);
				const auto& childPipeSegment = pipeGroup.RefPipeSegment(pipeSegment.GetNextHandle());
				const auto& childCell = childProfile.PeekCell(childPipeSegment.m_data.m_cellHandle);
				const auto& childParticle = childPhysics2D.RefParticle(childCell.m_data.m_particleHandle);
				auto& particle = physics2D.RefParticle(cell.m_data.m_particleHandle);
				particle.SetColor(childParticle.GetColor());
				particle.SetPosition(childParticle.GetPosition());
			}
		}
		else {
			//Simulate flow start physics
			std::vector<NodeHandle> childNodeHandles = flowEndNode.RefChildHandles();
			NodeHandle mainChildHandle = -1;
			for (const auto& childHandle : childNodeHandles)
			{
				auto& childNode = skeleton.RefNode(childHandle);
				if (childNode.IsApical()) {
					mainChildHandle = childHandle;
					break;
				}
			}
			const auto& mainChildNode = skeleton.RefNode(mainChildHandle);
			auto& mainChildProfile = profileGroup.RefProfile(mainChildNode.m_data.m_profileHandle);
			auto& mainChildPhysics2D = mainChildProfile.m_data.m_particlePhysics2D;
			//Copy cell offset from main child.
			for (const auto& childCell : mainChildProfile.RefCells())
			{
				const auto& mainChildPipeSegment = pipeGroup.RefPipeSegment(childCell.m_data.m_pipeSegmentHandle);
				const auto& mainChildParticle = mainChildPhysics2D.RefParticle(childCell.m_data.m_particleHandle);

				const auto& pipeSegment = pipeGroup.RefPipeSegment(mainChildPipeSegment.GetPrevHandle());
				const auto& cell = profile.PeekCell(pipeSegment.m_data.m_cellHandle);
				auto& particle = physics2D.RefParticle(cell.m_data.m_particleHandle);
				particle.SetPosition(mainChildParticle.GetPosition());
				particle.SetColor(glm::vec4(1.0f));
			}
			int index = 0;
			for (const auto& childHandle : childNodeHandles)
			{
				auto& childNode = skeleton.RefNode(childHandle);
				if (childNode.IsApical()) {
					continue;
				}
				auto& childProfile = profileGroup.RefProfile(childNode.m_data.m_profileHandle);
				auto& childPhysics2D = childProfile.m_data.m_particlePhysics2D;
				auto childNodeFront = glm::inverse(flowEndNode.m_info.m_regulatedGlobalRotation) * childNode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
				auto offset = glm::normalize(glm::vec2(childNodeFront.x, childNodeFront.y));
				offset = (mainChildPhysics2D.GetDistanceToCenter(offset) + childPhysics2D.GetDistanceToCenter(-offset) + 2.0f) * offset;
				for (const auto& childCell : childProfile.RefCells())
				{
					const auto& childPipeSegment = pipeGroup.RefPipeSegment(childCell.m_data.m_pipeSegmentHandle);
					const auto& childParticle = childPhysics2D.RefParticle(childCell.m_data.m_particleHandle);

					const auto& pipeSegment = pipeGroup.RefPipeSegment(childPipeSegment.GetPrevHandle());
					const auto& cell = profile.PeekCell(pipeSegment.m_data.m_cellHandle);
					auto& particle = physics2D.RefParticle(cell.m_data.m_particleHandle);
					particle.SetPosition(childParticle.GetPosition() + offset);
					particle.SetColor(glm::vec4(randomColor[index], 1.0f));
				}
				index++;
			}
			const auto iterations = pipeModelParameters.m_simulationIterationCellFactor * physics2D.RefParticles().size();
			for (int i = 0; i < iterations; i++) {
				physics2D.Simulate(1, [&](auto& particle)
					{
						//Apply gravity
						particle.SetPosition(particle.GetPosition() - physics2D.GetMassCenter());
						if (glm::length(particle.GetPosition()) > 0.0f) {
							const glm::vec2 acceleration = pipeModelParameters.m_gravityStrength * -glm::normalize(particle.GetPosition());
							particle.SetAcceleration(acceleration);
						}
					}
				);
				if (i > pipeModelParameters.m_minimumSimulationIteration && physics2D.GetMaxParticleVelocity() < pipeModelParameters.m_particleStabilizeSpeed)
				{
					break;
				}
			}
		}
	}
	//2. Splitting
	for (const auto& flowHandle : sortedFlowList)
	{
		auto& flow = skeleton.RefFlow(flowHandle);
		if (flow.RefNodeHandles().size() == 1) continue;
		const auto& childFlowHandles = flow.RefChildHandles();
		const auto flowStartNodeHandle = flow.RefNodeHandles().front();
		const auto& flowStartNode = skeleton.RefNode(flowStartNodeHandle);
		const auto& flowEndNode = skeleton.RefNode(flow.RefNodeHandles().back());
		auto& flowStartProfile = profileGroup.RefProfile(flowStartNode.m_data.m_profileHandle);
		auto& flowStartProfileData = flowStartProfile.m_data;
		auto& flowStartPhysics2D = flowStartProfileData.m_particlePhysics2D;
		auto& flowEndProfile = profileGroup.RefProfile(flowEndNode.m_data.m_profileHandle);
		auto& flowEndProfileData = flowEndProfile.m_data;
		auto& flowEndPhysics2D = flowEndProfileData.m_particlePhysics2D;

		if (flow.GetParentHandle() != -1)
		{
			//First, copy parent end to self start.
			auto& parentFlow = skeleton.RefFlow(flow.GetParentHandle());
			const auto& parentFlowEndNode = skeleton.RefNode(parentFlow.RefNodeHandles().back());
			auto& parentFlowEndProfile = profileGroup.RefProfile(parentFlowEndNode.m_data.m_profileHandle);
			auto& parentFlowEndProfileData = parentFlowEndProfile.m_data;
			auto& parentFlowEndPhysics2D = parentFlowEndProfileData.m_particlePhysics2D;
			for (auto& flowStartParticle : flowStartPhysics2D.RefParticles())
			{
				const auto flowStartNodeCellHandle = flowStartParticle.m_data.m_cellHandle;
				auto& flowStartNodeCell = flowStartProfile.RefCell(flowStartNodeCellHandle);
				auto& flowStartPipeSegment = pipeGroup.RefPipeSegment(flowStartNodeCell.m_data.m_pipeSegmentHandle);
				auto& parentFlowEndPipeSegment = pipeGroup.RefPipeSegment(flowStartPipeSegment.GetPrevHandle());
				auto& parentFlowEndCell = parentFlowEndProfile.RefCell(parentFlowEndPipeSegment.m_data.m_cellHandle);
				flowStartParticle.SetPosition(parentFlowEndPhysics2D.RefParticle(parentFlowEndCell.m_data.m_particleHandle).GetPosition());
			}
		}
		//Copy self start to self end.
		std::unordered_map<PipeHandle, ParticleHandle> flowStartParticleMap{};
		for (const auto& flowStartParticle : flowStartPhysics2D.RefParticles())
		{
			const auto flowStartNodeCellHandle = flowStartParticle.m_data.m_cellHandle;
			auto& flowStartNodeCell = flowStartProfile.RefCell(flowStartNodeCellHandle);
			flowStartParticleMap[flowStartNodeCell.m_data.m_pipeHandle] = flowStartParticle.GetHandle();
		}
		for (auto& flowEndParticle : flowEndPhysics2D.RefParticles())
		{
			const auto flowEndNodeCellHandle = flowEndParticle.m_data.m_cellHandle;
			auto& flowEndNodeCell = flowEndProfile.RefCell(flowEndNodeCellHandle);
			flowEndParticle.SetPosition(flowStartPhysics2D.RefParticle(flowStartParticleMap.at(flowEndNodeCell.m_data.m_pipeHandle)).GetPosition());
			flowEndParticle.SetColor(flowStartPhysics2D.RefParticle(flowStartParticleMap.at(flowEndNodeCell.m_data.m_pipeHandle)).GetColor());
		}
		flowEndPhysics2D.CalculateMinMax();
		const auto currentCenter = flowEndPhysics2D.GetMassCenter();
		if (flow.GetParentHandle() != -1)
		{
			//Sqeeze end.
			const auto iterations = pipeModelParameters.m_simulationIterationCellFactor * flowEndPhysics2D.RefParticles().size();
			for (int i = 0; i < iterations; i++) {
				flowEndPhysics2D.Simulate(1, [&](auto& particle)
					{
						//Apply gravity
						const auto diff = particle.GetPosition() - currentCenter;
						if (glm::length(diff) > 0.0f) {
							const glm::vec2 acceleration = pipeModelParameters.m_gravityStrength * -glm::normalize(diff);
							particle.SetAcceleration(acceleration);
						}
					}
				);
				if (i > pipeModelParameters.m_minimumSimulationIteration && flowEndPhysics2D.GetMaxParticleVelocity() < pipeModelParameters.m_particleStabilizeSpeed)
				{
					break;
				}
			}
		}
	}

	auto& baseProfile = profileGroup.RefProfile(skeleton.m_data.m_baseProfileHandle);
	auto& basePhysics2D = baseProfile.m_data.m_particlePhysics2D;
	auto& childProfile = profileGroup.RefProfile(skeleton.RefNode(sortedNodeList[0]).m_data.m_profileHandle);

	for (const auto& cell : baseProfile.RefCells())
	{
		const auto& childPipeSegment = pipeGroup.RefPipeSegment(pipeGroup.RefPipe(cell.m_data.m_pipeHandle).PeekPipeSegmentHandles().front());
		const auto& childCell = childProfile.PeekCell(childPipeSegment.m_data.m_cellHandle);
		const auto& childParticle = childProfile.m_data.m_particlePhysics2D.RefParticle(childCell.m_data.m_particleHandle);
		auto& particle = basePhysics2D.RefParticle(cell.m_data.m_particleHandle);
		particle.SetColor(childParticle.GetColor());
		particle.SetPosition(childParticle.GetPosition());
	}
}

void TreePipeModel::ApplySimulationResults(const PipeModelParameters& pipeModelParameters)
{
	auto& skeleton = m_shootPipeModel.m_skeleton;
	auto& profileGroup = m_shootPipeModel.m_pipeProfileGroup;
	auto& pipeGroup = m_shootPipeModel.m_pipeGroup;
	const auto sortedFlowList = skeleton.RefSortedFlowList();
	for (const auto flowHandle : sortedFlowList)
	{
		auto& flow = skeleton.RefFlow(flowHandle);
		const auto& nodeHandles = flow.RefNodeHandles();
		const auto& srcNode = skeleton.RefNode(nodeHandles.front());
		auto& srcNodeProfile = profileGroup.RefProfile(srcNode.m_data.m_profileHandle);
		std::unordered_map<PipeHandle, CellHandle> srcNodeMap{};
		auto& srcNodePhysics2D = srcNodeProfile.m_data.m_particlePhysics2D;
		for (const auto& srcNodeParticle : srcNodePhysics2D.RefParticles())
		{
			const auto srcNodeCellHandle = srcNodeParticle.m_data.m_cellHandle;
			auto& srcNodeCell = srcNodeProfile.RefCell(srcNodeCellHandle);
			srcNodeMap[srcNodeCell.m_data.m_pipeHandle] = srcNodeCellHandle;
			srcNodeCell.m_info.m_offset = srcNodePhysics2D.RefParticle(srcNodeCell.m_data.m_particleHandle).GetPosition();
		}

		if (nodeHandles.size() == 1) continue;
		std::unordered_map<PipeHandle, std::pair<ProfileHandle, CellHandle>> dstNodeMap{};
		const auto& lastNode = skeleton.RefNode(nodeHandles.back());
		const auto& lastNodeChildHandles = lastNode.RefChildHandles();
		if(lastNodeChildHandles.empty())
		{
			const auto& dstNode = skeleton.RefNode(nodeHandles.back());
			const auto dstNodeProfileHandle = dstNode.m_data.m_profileHandle;
			auto& dstNodeProfile = profileGroup.RefProfile(dstNodeProfileHandle);
			auto& dstNodePhysics2D = dstNodeProfile.m_data.m_particlePhysics2D;
			for (const auto& dstNodeParticle : dstNodePhysics2D.RefParticles())
			{
				const auto cellHandle = dstNodeParticle.m_data.m_cellHandle;
				auto& cell = dstNodeProfile.RefCell(cellHandle);
				dstNodeMap[cell.m_data.m_pipeHandle] = { dstNodeProfileHandle, cellHandle };
			}
		}
		else {
			for (const auto& dstNodeHandle : lastNodeChildHandles) {
				const auto& dstNode = skeleton.RefNode(dstNodeHandle);
				const auto dstNodeProfileHandle = dstNode.m_data.m_profileHandle;
				auto& dstNodeProfile = profileGroup.RefProfile(dstNodeProfileHandle);
				auto& dstNodePhysics2D = dstNodeProfile.m_data.m_particlePhysics2D;
				for (const auto& dstNodeParticle : dstNodePhysics2D.RefParticles())
				{
					const auto cellHandle = dstNodeParticle.m_data.m_cellHandle;
					auto& cell = dstNodeProfile.RefCell(cellHandle);
					dstNodeMap[cell.m_data.m_pipeHandle] = { dstNodeProfileHandle, cellHandle };
					cell.m_info.m_offset = dstNodePhysics2D.RefParticle(cell.m_data.m_particleHandle).GetPosition();
				}
			}
		}
		const auto nodeSize = nodeHandles.size();
		for (int i = 1; i < nodeHandles.size(); i++)
		{
			const float a = static_cast<float>(i) / (nodeSize - 1);
			const auto& node = skeleton.RefNode(nodeHandles[i]);
			auto& profile = profileGroup.RefProfile(node.m_data.m_profileHandle);
			for (auto& cell : profile.RefCells())
			{
				const auto pipeHandle = cell.m_data.m_pipeHandle;
				cell.m_info.m_offset =
					glm::mix(srcNodeProfile.RefCell(srcNodeMap.at(pipeHandle)).m_info.m_offset,
						profileGroup.RefProfile(dstNodeMap.at(pipeHandle).first).RefCell(dstNodeMap.at(pipeHandle).second).m_info.m_offset, a);
				cell.m_info.m_color = glm::mix(srcNodeProfile.RefCell(srcNodeMap.at(pipeHandle)).m_info.m_color,
					profileGroup.RefProfile(dstNodeMap.at(pipeHandle).first).RefCell(dstNodeMap.at(pipeHandle).second).m_info.m_color, a);
			}
		}
	}
	auto& baseProfile = profileGroup.RefProfile(skeleton.m_data.m_baseProfileHandle);
	auto& basePhysics2D = baseProfile.m_data.m_particlePhysics2D;
	for (auto& cell : baseProfile.RefCells())
	{
		auto& particle = basePhysics2D.RefParticle(cell.m_data.m_particleHandle);
		cell.m_info.m_offset = particle.GetPosition();
	}
}
