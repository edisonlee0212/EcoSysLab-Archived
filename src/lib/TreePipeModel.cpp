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
	if(shootNewNodeList.empty()) return;
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

			auto direction = glm::circularRand(1.0f);
			const auto baseCellHandle = baseProfile.AllocateCell();
			node.m_data.m_pipeHandle = pipeGroup.AllocatePipe();

			auto& baseCell = baseProfile.RefCell(baseCellHandle);
			baseCell.m_data.m_pipeHandle = node.m_data.m_pipeHandle;
			baseCell.m_data.m_pipeSegmentHandle = -1;

			baseCell.m_info.m_offset = glm::vec2(0.0f);
			baseCell.m_info.m_offset = baseProfile.FindAvailablePosition(baseProfile.RefCell(pipeGroup.RefPipeSegment(rootToParentNodePipeSegmentChain[0]).m_data.m_cellHandle).m_info.m_offset, direction * pipeModelParameters.m_profileDefaultCellRadius);
			segmentIndex = 0;
			for (auto it = parentNodeToRootChain.rbegin(); it != parentNodeToRootChain.rend(); ++it) {
				const auto newPipeSegmentHandle = pipeGroup.Extend(node.m_data.m_pipeHandle);
				auto& newPipeSegment = pipeGroup.RefPipeSegment(newPipeSegmentHandle);
				newPipeSegment.m_data.m_nodeHandle = *it;

				const auto& nodeInChain = skeleton.PeekNode(*it);
				auto& profile = profileGroup.RefProfile(nodeInChain.m_data.m_profileHandle);
				const auto newCellHandle = profile.AllocateCell();
				auto& newCell = profile.RefCell(newCellHandle);
				newCell.m_data.m_pipeHandle = node.m_data.m_pipeHandle;
				newCell.m_data.m_pipeSegmentHandle = newPipeSegmentHandle;

				newCell.m_info.m_offset = glm::vec2(0.0f);
				newCell.m_info.m_offset = profile.FindAvailablePosition(profile.RefCell(pipeGroup.RefPipeSegment(rootToParentNodePipeSegmentChain[segmentIndex]).m_data.m_cellHandle).m_info.m_offset, direction * pipeModelParameters.m_profileDefaultCellRadius);
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
	for (auto it = sortedNodeList.rbegin(); it != sortedNodeList.rend(); ++it) {
		auto& node = skeleton.RefNode(*it);
		auto& profile = profileGroup.RefProfile(node.m_data.m_profileHandle);
		auto& profileData = profile.m_data;
		auto& physics2D = profileData.m_particlePhysics2D;
		const auto& childNodeHandles = node.RefChildHandles();
		if (childNodeHandles.empty()) {
			for (auto& particle : physics2D.RefParticles())
			{
				particle.SetColor(glm::vec4(1.0f));
				particle.SetPosition(glm::vec2(0.0f));
			}
		}
		else if (childNodeHandles.size() == 1)
		{
			auto& childNode = skeleton.RefNode(childNodeHandles[0]);
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
			auto parentNodeHandle = node.GetParentHandle();
			bool chainStart = false;
			if (parentNodeHandle == -1 || skeleton.PeekNode(parentNodeHandle).RefChildHandles().size() > 1) chainStart = true;
			if(chainStart)
			{
				physics2D.Simulate(pipeModelParameters.m_simulationIterationFactor * physics2D.RefParticles().size(), [&](auto& particle)
					{
						//Apply gravity
						particle.SetPosition(particle.GetPosition() - physics2D.GetMassCenter());
						if (glm::length(particle.GetPosition()) > 0.0f) {
							const glm::vec2 acceleration = pipeModelParameters.m_gravityStrength * -glm::normalize(particle.GetPosition());
							particle.SetAcceleration(acceleration);
						}
					}
				);
			}
		}
		else {
#pragma region Color
			for (auto& particle : physics2D.RefParticles())
			{
				const auto cellHandle = particle.m_data.m_cellHandle;
				const auto pipeSegmentHandle = profile.PeekCell(cellHandle).m_data.m_pipeSegmentHandle;
				const auto& nextPipeSegment = pipeGroup.PeekPipeSegment(pipeGroup.PeekPipeSegment(pipeSegmentHandle).GetNextHandle());
				const auto targetChildNodeHandle = nextPipeSegment.m_data.m_nodeHandle;
				for (int i = 0; i < childNodeHandles.size(); i++)
				{
					if (childNodeHandles[i] == targetChildNodeHandle)
					{
						particle.SetColor(glm::vec4(randomColor[i], 1.0f));
					}
				}
			}
#pragma endregion
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
				physics2D.RefParticle(cell.m_data.m_particleHandle).SetPosition(mainChildParticle.GetPosition());
			}
			int index = 0;
			float randAngle = glm::linearRand(0.0f, 360.0f);
			for (const auto& childHandle : childNodeHandles)
			{
				auto& childNode = skeleton.RefNode(childHandle);
				if (childNode.IsApical()) {
					continue;
				}
				auto& childProfile = profileGroup.RefProfile(childNode.m_data.m_profileHandle);
				auto& childPhysics2D = childProfile.m_data.m_particlePhysics2D;
				auto offset = glm::normalize(glm::vec2(glm::cos(glm::radians(randAngle + index * 120.0f)), glm::sin(glm::radians(randAngle + index * 120.0f))));
				offset = (mainChildPhysics2D.GetDistanceToCenter(offset) + childPhysics2D.GetDistanceToCenter(-offset) + 2.0f) * offset;
				for (const auto& childCell : childProfile.RefCells())
				{
					const auto& childPipeSegment = pipeGroup.RefPipeSegment(childCell.m_data.m_pipeSegmentHandle);
					const auto& childParticle = childPhysics2D.RefParticle(childCell.m_data.m_particleHandle);

					const auto& pipeSegment = pipeGroup.RefPipeSegment(childPipeSegment.GetPrevHandle());
					const auto& cell = profile.PeekCell(pipeSegment.m_data.m_cellHandle);
					physics2D.RefParticle(cell.m_data.m_particleHandle).SetPosition(childParticle.GetPosition() + offset);
				}
				index++;
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
	const auto sortedFlowList = skeleton.RefSortedFlowList();
	for (const auto flowHandle : sortedFlowList)
	{
		auto& flow = skeleton.RefFlow(flowHandle);
		const auto& nodeHandles = flow.RefNodeHandles();
		const auto& firstNode = skeleton.RefNode(nodeHandles.front());
		auto& firstNodeProfile = profileGroup.RefProfile(firstNode.m_data.m_profileHandle);
		auto& firstNodePhysics2D = firstNodeProfile.m_data.m_particlePhysics2D;
		std::unordered_map<PipeHandle, CellHandle> firstNodeMap{};
		for (const auto& firstNodeParticle : firstNodePhysics2D.RefParticles())
		{
			const auto cellHandle = firstNodeParticle.m_data.m_cellHandle;
			auto& cell = firstNodeProfile.RefCell(cellHandle);
			cell.m_info.m_offset = firstNodeParticle.GetPosition();
			firstNodeMap[cell.m_data.m_pipeHandle] = cellHandle;
		}
		if (nodeHandles.size() == 1) continue;
		std::unordered_map<PipeHandle, CellHandle> lastNodeMap{};
		const auto& lastNode = skeleton.RefNode(nodeHandles.back());
		auto& lastNodeProfile = profileGroup.RefProfile(lastNode.m_data.m_profileHandle);
		auto& lastNodePhysics2D = lastNodeProfile.m_data.m_particlePhysics2D;
		for (const auto& lastNodeParticle : lastNodePhysics2D.RefParticles())
		{
			const auto cellHandle = lastNodeParticle.m_data.m_cellHandle;
			auto& cell = lastNodeProfile.RefCell(cellHandle);
			cell.m_info.m_offset = lastNodeParticle.GetPosition();
			lastNodeMap[cell.m_data.m_pipeHandle] = cellHandle;
		}
		if (nodeHandles.size() == 2) continue;
		const auto nodeSize = nodeHandles.size();
		for (int i = 1; i < nodeHandles.size() - 1; i++)
		{
			const float a = static_cast<float>(i) / (nodeSize - 1);
			const auto& node = skeleton.RefNode(nodeHandles[i]);
			auto& profile = profileGroup.RefProfile(node.m_data.m_profileHandle);
			for (auto& cell : profile.RefCells())
			{
				const auto pipeHandle = cell.m_data.m_pipeHandle;
				cell.m_info.m_offset =
					glm::mix(firstNodeProfile.RefCell(firstNodeMap.at(pipeHandle)).m_info.m_offset,
						lastNodeProfile.RefCell(lastNodeMap.at(pipeHandle)).m_info.m_offset, a);
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
