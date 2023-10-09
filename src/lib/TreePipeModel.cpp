#include "TreePipeModel.hpp"

using namespace EcoSysLab;

void TreePipeModel::UpdatePipeModels(const TreeModel& targetTreeModel, const PipeModelParameters& pipeModelParameters)
{
	m_shootPipeModel = {};
	m_rootPipeModel = {};

	auto& skeleton = m_shootPipeModel.m_skeleton;
	auto& profileGroup = m_shootPipeModel.m_pipeProfileGroup;
	auto& pipeGroup = m_shootPipeModel.m_pipeGroup;

	skeleton.Clone(targetTreeModel.PeekShootSkeleton(), [&](NodeHandle srcNodeHandle, NodeHandle dstNodeHandle) {});
	skeleton.m_data.m_baseProfileHandle = profileGroup.Allocate();
	profileGroup.RefProfile(skeleton.m_data.m_baseProfileHandle).m_info.m_cellRadius = pipeModelParameters.m_profileDefaultCellRadius;

	std::map<int, NodeHandle> shootNewNodeList;
	for (const auto& i : skeleton.RefRawNodes())
	{
		shootNewNodeList[i.GetIndex()] = i.GetHandle();
	}
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

			baseCell.m_info.m_offset = baseProfile.FindAvailablePosition(
				baseProfile.RefCell(pipeGroup.RefPipeSegment(rootToParentNodePipeSegmentChain[0]).m_data.m_cellHandle).m_info.m_offset,
				direction * pipeModelParameters.m_profileDefaultCellRadius);

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

				newCell.m_info.m_offset = profile.FindAvailablePosition(
					profile.RefCell(pipeGroup.RefPipeSegment(rootToParentNodePipeSegmentChain[segmentIndex]).m_data.m_cellHandle).m_info.m_offset,
					direction * pipeModelParameters.m_profileDefaultCellRadius);

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
		physics2D.m_particleRadius = profile.m_info.m_cellRadius;
		for (auto& cell : profile.RefCells())
		{
			auto newParticleHandle = physics2D.AllocateParticle();
			auto& newParticle = physics2D.RefParticle(newParticleHandle);
			newParticle.m_data.m_cellHandle = cell.GetHandle();
			newParticle.SetPosition(cell.m_info.m_offset);
			newParticle.SetDamping(pipeModelParameters.m_damping);

			cell.m_data.m_particleHandle = newParticleHandle;
		}
	}
}

void TreePipeModel::SimulateAllProfiles(const size_t minCellCount, const size_t iteration, const PipeModelParameters& pipeModelParameters)
{
	auto& skeleton = m_shootPipeModel.m_skeleton;
	auto& profileGroup = m_shootPipeModel.m_pipeProfileGroup;
	auto& pipeGroup = m_shootPipeModel.m_pipeGroup;

	const auto& sortedNodeList = skeleton.RefSortedNodeList();
	for(const auto& nodeHandle : sortedNodeList)
	{
		auto& node = skeleton.RefNode(nodeHandle);
		auto& profile = profileGroup.RefProfile(node.m_data.m_profileHandle);
		const auto parentNodeHandle = node.GetParentHandle();
		if(parentNodeHandle == -1)
		{
			auto& baseProfile = profileGroup.RefProfile(skeleton.m_data.m_baseProfileHandle);
			auto& baseProfileData = baseProfile.m_data;
			auto& basePhysics2D = baseProfileData.m_particlePhysics2D;
			if (basePhysics2D.RefParticles().size() <= minCellCount) continue;
			basePhysics2D.Simulate(iteration, [&](auto& particle)
				{
					//Apply gravity
					particle.SetPosition(particle.GetPosition() - basePhysics2D.GetMassCenter());
					if (glm::length(particle.GetPosition()) > 0.0f) {
						const glm::vec2 acceleration = pipeModelParameters.m_gravityStrength * -glm::normalize(particle.GetPosition());
						particle.SetAcceleration(acceleration);
					}
				}
			);
			auto& profileData = profile.m_data;
			auto& physics2D = profileData.m_particlePhysics2D;
			//Copy cell offset from parent.
			for (const auto& baseCell : baseProfile.RefCells())
			{
				const auto& pipe = pipeGroup.RefPipe(baseCell.m_data.m_pipeHandle);
				if (pipe.PeekPipeSegmentHandles().empty()) continue;
				const auto pipeSegmentHandle = pipe.PeekPipeSegmentHandles().at(0);
				const auto& pipeSegment = pipeGroup.RefPipeSegment(pipeSegmentHandle);
				const auto& cell = profile.RefCell(pipeSegment.m_data.m_cellHandle);

				const auto& parentParticle = basePhysics2D.RefParticle(baseCell.m_data.m_particleHandle);
				physics2D.RefParticle(cell.m_data.m_particleHandle).SetPosition(parentParticle.GetPosition());
			}
		}else
		{
			const auto& parentNode = skeleton.RefNode(parentNodeHandle);
			
			if(parentNode.RefChildHandles().size() > 1)
			{
				auto& profileData = profile.m_data;
				auto& physics2D = profileData.m_particlePhysics2D;
				if (physics2D.RefParticles().size() <= minCellCount) continue;
				physics2D.Simulate(iteration, [&](auto& particle)
					{
						//Apply gravity
						particle.SetPosition(particle.GetPosition() - physics2D.GetMassCenter());
						if (glm::length(particle.GetPosition()) > 0.0f) {
							const glm::vec2 acceleration = pipeModelParameters.m_gravityStrength * -glm::normalize(particle.GetPosition());
							particle.SetAcceleration(acceleration);
						}
					}
				);
			}else
			{
				auto& profileData = profile.m_data;
				auto& physics2D = profileData.m_particlePhysics2D;
				//Copy cell offset from parent.
				auto& parentProfile = profileGroup.RefProfile(parentNode.m_data.m_profileHandle);
				for(const auto& cell : profile.RefCells())
				{
					const auto& pipeSegment = pipeGroup.RefPipeSegment(cell.m_data.m_pipeSegmentHandle);
					const auto& parentPipeSegment = pipeGroup.RefPipeSegment(pipeSegment.GetPrevHandle());
					const auto& parentCell = parentProfile.PeekCell(parentPipeSegment.m_data.m_cellHandle);
					const auto& parentParticle = parentProfile.m_data.m_particlePhysics2D.RefParticle(parentCell.m_data.m_particleHandle);
					physics2D.RefParticle(cell.m_data.m_particleHandle).SetPosition(parentParticle.GetPosition());
				}
			}
		}
	}
}

void TreePipeModel::ApplySimulationResults(const PipeModelParameters& pipeModelParameters)
{
	auto& profileGroup = m_shootPipeModel.m_pipeProfileGroup;
	for (auto& profile : profileGroup.RefProfiles())
	{
		auto& profileData = profile.m_data;
		auto& physics2D = profileData.m_particlePhysics2D;
		for(const auto& particle : physics2D.RefParticles())
		{
			auto& cell = profile.RefCell(particle.m_data.m_cellHandle);
			cell.m_info.m_offset = particle.GetPosition();
		}
	}
}
