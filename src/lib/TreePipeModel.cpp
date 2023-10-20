#include "TreePipeModel.hpp"

#include "EcoSysLabLayer.hpp"

using namespace EcoSysLab;


void TreePipeModel::ShiftSkeleton()
{
	auto& skeleton = m_shootPipeModel.m_skeleton;
	for (const auto& nodeHandle : skeleton.RefSortedNodeList()) {
		auto& node = skeleton.RefNode(nodeHandle);
		auto& nodeInfo = node.m_info;
		auto& nodeData = node.m_data;
		if (node.GetParentHandle() != -1) {
			auto& parentInfo = skeleton.RefNode(node.GetParentHandle()).m_info;
			nodeInfo.m_globalRotation =
				parentInfo.m_globalRotation * nodeData.m_localRotation;
			nodeInfo.m_globalDirection = glm::normalize(nodeInfo.m_globalRotation * glm::vec3(0, 0, -1));
			nodeInfo.m_globalPosition =
				parentInfo.m_globalPosition
				+ parentInfo.m_length * parentInfo.m_globalDirection + nodeData.m_localPosition;
			auto parentRegulatedUp = parentInfo.m_regulatedGlobalRotation * glm::vec3(0, 1, 0);
			auto regulatedUp = glm::normalize(glm::cross(glm::cross(nodeInfo.m_globalDirection, parentRegulatedUp), nodeInfo.m_globalDirection));
			nodeInfo.m_regulatedGlobalRotation = glm::quatLookAt(nodeInfo.m_globalDirection, regulatedUp);
		}
		skeleton.m_min = glm::min(skeleton.m_min, nodeInfo.m_globalPosition);
		skeleton.m_max = glm::max(skeleton.m_max, nodeInfo.m_globalPosition);
		const auto endPosition = nodeInfo.m_globalPosition
			+ nodeInfo.m_length * nodeInfo.m_globalDirection;
		skeleton.m_min = glm::min(skeleton.m_min, endPosition);
		skeleton.m_max = glm::max(skeleton.m_max, endPosition);
	}
}

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
		if(parentNodeHandle != -1)
		{
			node.m_data.m_localRotation = glm::inverse(skeleton.RefNode(parentNodeHandle).m_info.m_globalRotation) * node.m_info.m_globalRotation;
		}

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

	const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
	const auto& randomColor = ecoSysLabLayer->RandomColors();
	const auto& sortedFlowList = skeleton.RefSortedFlowList();

	for (const auto& flowHandle : sortedFlowList)
	{
		auto& flow = skeleton.RefFlow(flowHandle);
		const auto& nodeHandles = flow.RefNodeHandles();
		if (!nodeHandles.empty()) {
			auto &startNode = skeleton.RefNode(nodeHandles.front());
			auto& startProfile = profileGroup.RefProfile(startNode.m_data.m_profileHandle);
			auto& startPhysics2D = flow.m_data.m_startParticlePhysics2D;
			startPhysics2D.Reset(0.002f);
			for (auto& cell : startProfile.RefCells())
			{
				auto newParticleHandle = startPhysics2D.AllocateParticle();
				auto& newParticle = startPhysics2D.RefParticle(newParticleHandle);
				newParticle.m_data.m_pipeHandle = cell.m_data.m_pipeHandle;
				newParticle.SetDamping(pipeModelParameters.m_damping);
				newParticle.SetPosition(cell.m_info.m_offset);
			}
			auto& endNode = skeleton.RefNode(nodeHandles.back());
			auto& endProfile = profileGroup.RefProfile(endNode.m_data.m_profileHandle);
			auto& endPhysics2D = flow.m_data.m_endParticlePhysics2D;
			endPhysics2D.Reset(0.002f);
			for (auto& cell : endProfile.RefCells())
			{
				auto newParticleHandle = endPhysics2D.AllocateParticle();
				auto& newParticle = endPhysics2D.RefParticle(newParticleHandle);
				newParticle.m_data.m_pipeHandle = cell.m_data.m_pipeHandle;
				newParticle.SetDamping(pipeModelParameters.m_damping);
				newParticle.SetPosition(cell.m_info.m_offset);
			}
		}

	}
	
	//1. Packing
	for (auto it = sortedFlowList.rbegin(); it != sortedFlowList.rend(); ++it)
	{
		auto& flow = skeleton.RefFlow(*it);
		const auto& childFlowHandles = flow.RefChildHandles();
		auto& flowStartPhysics2D = flow.m_data.m_startParticlePhysics2D;
		auto& flowEndPhysics2D = flow.m_data.m_endParticlePhysics2D;
		if (childFlowHandles.empty())
		{
			//For flow start, set only particle at the center.
			for (auto& particle : flowStartPhysics2D.RefParticles())
			{
				particle.SetColor(glm::vec4(1.0f));
				particle.SetPosition(glm::vec2(0.0f));
			}
			//For flow end, set only particle at the center.
			for (auto& particle : flowEndPhysics2D.RefParticles())
			{
				particle.SetColor(glm::vec4(1.0f));
				particle.SetPosition(glm::vec2(0.0f));
			}
		}
		else if (childFlowHandles.size() == 1)
		{
			//Copy from child flow start to self flow start
			const auto& childFlow = skeleton.PeekFlow(childFlowHandles.front());
			const auto& childPhysics2D = childFlow.m_data.m_startParticlePhysics2D;
			std::unordered_map<PipeHandle, ParticleHandle> particleMap{};
			for (const auto& childParticle : childPhysics2D.PeekParticles())
			{
				particleMap.insert({ childParticle.m_data.m_pipeHandle, childParticle.GetHandle() });
			}
			for (auto& particle : flowStartPhysics2D.RefParticles())
			{
				const auto& childParticle = childPhysics2D.PeekParticle(particleMap.at(particle.m_data.m_pipeHandle));
				particle.SetColor(childParticle.GetColor());
				particle.SetPosition(childParticle.GetPosition());
			}
			for (auto& particle : flowEndPhysics2D.RefParticles())
			{
				const auto& childParticle = childPhysics2D.PeekParticle(particleMap.at(particle.m_data.m_pipeHandle));
				particle.SetColor(childParticle.GetColor());
				particle.SetPosition(childParticle.GetPosition());
			}
		}
		else {
			//Simulate flow start physics
			FlowHandle mainChildHandle = -1;
			for (const auto& childHandle : childFlowHandles)
			{
				const auto& childFlow = skeleton.PeekFlow(childHandle);
				if (childFlow.IsApical()) {
					mainChildHandle = childHandle;
					break;
				}
			}
			auto& mainChildFlow = skeleton.RefFlow(mainChildHandle);
			mainChildFlow.m_data.m_offset = glm::vec2(0.0f);
			const auto& mainChildPhysics2D = mainChildFlow.m_data.m_startParticlePhysics2D;
			//Copy cell offset from main child.
			std::unordered_map<PipeHandle, ParticleHandle> flowStartParticleMap{};
			std::unordered_map<PipeHandle, ParticleHandle> flowEndParticleMap{};
			for (const auto& flowStartParticle : flowStartPhysics2D.PeekParticles())
			{
				flowStartParticleMap.insert({ flowStartParticle.m_data.m_pipeHandle, flowStartParticle.GetHandle() });
			}
			for (const auto& flowEndParticle : flowEndPhysics2D.PeekParticles())
			{
				flowEndParticleMap.insert({ flowEndParticle.m_data.m_pipeHandle, flowEndParticle.GetHandle() });
			}
			if(mainChildHandle != -1){
				for (const auto& mainChildParticle : mainChildPhysics2D.PeekParticles())
				{
					const auto particleHandle = flowStartParticleMap.at(mainChildParticle.m_data.m_pipeHandle);
					flowStartPhysics2D.RefParticle(particleHandle).SetColor(mainChildParticle.GetColor());
					flowStartPhysics2D.RefParticle(particleHandle).SetPosition(mainChildParticle.GetPosition());
					flowEndPhysics2D.RefParticle(particleHandle).SetColor(mainChildParticle.GetColor());
					flowEndPhysics2D.RefParticle(particleHandle).SetPosition(mainChildParticle.GetPosition());
				}
			}
			int index = 0;
			for (const auto& childHandle : childFlowHandles)
			{
				if(childHandle == mainChildHandle) continue;
				auto& childFlow = skeleton.RefFlow(childHandle);
				const auto& flowEndNode = skeleton.RefNode(flow.RefNodeHandles().back());
				const auto& childStartNode = skeleton.PeekNode(childFlow.RefNodeHandles().front());
				auto& childPhysics2D = childFlow.m_data.m_startParticlePhysics2D;
				auto childNodeFront = glm::inverse(flowEndNode.m_info.m_regulatedGlobalRotation) * childStartNode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
				auto offset = glm::normalize(glm::vec2(childNodeFront.x, childNodeFront.y));
				offset = (mainChildPhysics2D.GetDistanceToCenter(offset) + childPhysics2D.GetDistanceToCenter(-offset) + 2.0f) * offset;
				childFlow.m_data.m_offset = offset;
				for (const auto& childParticle : childPhysics2D.PeekParticles())
				{
					const auto particleHandle = flowStartParticleMap.at(childParticle.m_data.m_pipeHandle);
					flowStartPhysics2D.RefParticle(particleHandle).SetColor(childParticle.GetColor());
					flowStartPhysics2D.RefParticle(particleHandle).SetPosition(childParticle.GetPosition() + offset);
					flowEndPhysics2D.RefParticle(particleHandle).SetColor(childParticle.GetColor());
					flowEndPhysics2D.RefParticle(particleHandle).SetPosition(childParticle.GetPosition() + offset);
				}
				index++;
			}
			const auto iterations = pipeModelParameters.m_simulationIterationCellFactor * flowStartPhysics2D.RefParticles().size();
			for (int i = 0; i < iterations; i++) {
				flowStartPhysics2D.Simulate(1, [&](auto& particle)
					{
						//Apply gravity
						particle.SetPosition(particle.GetPosition() - flowStartPhysics2D.GetMassCenter());
						if (glm::length(particle.GetPosition()) > 0.0f) {
							const glm::vec2 acceleration = pipeModelParameters.m_gravityStrength * -glm::normalize(particle.GetPosition());
							particle.SetAcceleration(acceleration);
						}
					}
				);
				if (i > pipeModelParameters.m_minimumSimulationIteration && flowStartPhysics2D.GetMaxParticleVelocity() < pipeModelParameters.m_particleStabilizeSpeed)
				{
					break;
				}
			}
		}
	}
	//2. Shift Skeleton
	//ShiftSkeleton();
	skeleton.CalculateFlows();
}

void TreePipeModel::ApplySimulationResults(const PipeModelParameters& pipeModelParameters)
{
	auto& skeleton = m_shootPipeModel.m_skeleton;
	auto& profileGroup = m_shootPipeModel.m_pipeProfileGroup;
	const auto sortedFlowList = skeleton.RefSortedFlowList();
	for (const auto flowHandle : sortedFlowList)
	{
		auto& flow = skeleton.RefFlow(flowHandle);
		std::unordered_map<PipeHandle, ParticleHandle> flowStartParticleMap{};
		const auto& flowStartPhysics2D = flow.m_data.m_startParticlePhysics2D;
		const auto& flowEndStartPhysics2D = flow.m_data.m_endParticlePhysics2D;
		for (const auto& particle : flowStartPhysics2D.PeekParticles())
		{
			flowStartParticleMap.insert({ particle.m_data.m_pipeHandle, particle.GetHandle() });
		}
		std::unordered_map<PipeHandle, ParticleHandle> flowEndParticleMap{};
		for (const auto& particle : flowEndStartPhysics2D.PeekParticles())
		{
			flowEndParticleMap.insert({ particle.m_data.m_pipeHandle, particle.GetHandle() });
		}
		const auto& nodeHandles = flow.RefNodeHandles();
		const auto nodeSize = nodeHandles.size();
		for (int i = 0; i < nodeHandles.size(); i++)
		{
			const float a = static_cast<float>(i) / (nodeSize - 1);
			const auto& node = skeleton.RefNode(nodeHandles[i]);
			auto& profile = profileGroup.RefProfile(node.m_data.m_profileHandle);
			for (auto& cell : profile.RefCells())
			{
				const auto pipeHandle = cell.m_data.m_pipeHandle;
				cell.m_info.m_offset =
					glm::mix(flowStartPhysics2D.PeekParticle(flowStartParticleMap.at(pipeHandle)).GetPosition(),
						flowEndStartPhysics2D.PeekParticle(flowEndParticleMap.at(pipeHandle)).GetPosition(), a);
				cell.m_info.m_color = glm::mix(flowStartPhysics2D.PeekParticle(flowStartParticleMap.at(pipeHandle)).GetColor(),
					flowEndStartPhysics2D.PeekParticle(flowEndParticleMap.at(pipeHandle)).GetColor(), a);
			}
		}
	}
	auto& baseProfile = profileGroup.RefProfile(skeleton.m_data.m_baseProfileHandle);
	auto& basePhysics2D = skeleton.RefFlow(0).m_data.m_startParticlePhysics2D;
	std::unordered_map<PipeHandle, ParticleHandle> particleMap{};
	for (const auto& baseParticle : basePhysics2D.PeekParticles())
	{
		particleMap.insert({ baseParticle.m_data.m_pipeHandle, baseParticle.GetHandle() });
	}
	for (auto& cell : baseProfile.RefCells())
	{
		auto& particle = basePhysics2D.RefParticle(particleMap.at(cell.m_data.m_pipeHandle));
		cell.m_info.m_offset = particle.GetPosition();
	}
}
