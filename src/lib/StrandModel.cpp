#include "StrandModel.hpp"

using namespace EcoSysLab;


void StrandModel::ResetAllProfiles(const StrandModelParameters& strandModelParameters)
{
	m_strandModelSkeleton.m_data.m_strandGroup = {};
	const auto& sortedInternodeList = m_strandModelSkeleton.PeekSortedNodeList();
	for (const auto& internodeHandle : sortedInternodeList)
	{
		auto& internode = m_strandModelSkeleton.RefNode(internodeHandle);
		auto& frontPhysics2D = internode.m_data.m_frontProfile;
		auto& backPhysics2D = internode.m_data.m_backProfile;
		frontPhysics2D.m_settings = strandModelParameters.m_profilePhysicsSettings;
		backPhysics2D.m_settings = strandModelParameters.m_profilePhysicsSettings;
		frontPhysics2D.Reset(0.001f);
		backPhysics2D.Reset(0.001f);
		internode.m_data.m_frontParticleMap.clear();
		internode.m_data.m_backParticleMap.clear();
		internode.m_data.m_strandCount = 0;
	}
}

void StrandModel::InitializeProfiles(const StrandModelParameters& strandModelParameters)
{
	m_strandModelSkeleton.m_data.m_strandGroup = {};
	auto& strandGroup = m_strandModelSkeleton.m_data.m_strandGroup;
	const auto& sortedInternodeList = m_strandModelSkeleton.PeekSortedNodeList();
	for (const auto& internodeHandle : sortedInternodeList)
	{
		auto& internode = m_strandModelSkeleton.RefNode(internodeHandle);
		auto& frontPhysics2D = internode.m_data.m_frontProfile;
		auto& backPhysics2D = internode.m_data.m_backProfile;
		frontPhysics2D.m_settings = strandModelParameters.m_profilePhysicsSettings;
		backPhysics2D.m_settings = strandModelParameters.m_profilePhysicsSettings;
		if (internode.IsEndNode()) continue;
		frontPhysics2D.Reset(0.001f);
		backPhysics2D.Reset(0.001f);
	}


	for (const auto& internodeHandle : sortedInternodeList)
	{
		auto& internode = m_strandModelSkeleton.RefNode(internodeHandle);
		auto& frontPhysics2D = internode.m_data.m_frontProfile;
		auto& backPhysics2D = internode.m_data.m_backProfile;

		std::vector<NodeHandle> parentNodeToRootChain;
		NodeHandle walker = internode.GetParentHandle();
		while (walker >= 0)
		{
			parentNodeToRootChain.emplace_back(walker);
			walker = m_strandModelSkeleton.PeekNode(walker).GetParentHandle();
		}
		if (!internode.IsEndNode())
		{
			for (int i = 0; i < strandModelParameters.m_strandsAlongBranch; i++) {
				const auto strandHandle = strandGroup.AllocateStrand();
				for (auto it = parentNodeToRootChain.rbegin(); it != parentNodeToRootChain.rend(); ++it) {
					const auto newstrandSegmentHandle = strandGroup.Extend(strandHandle);
					auto& nodeOnChain = m_strandModelSkeleton.RefNode(*it);
					const auto newStartParticleHandle = nodeOnChain.m_data.m_frontProfile.AllocateParticle();
					auto& newStartParticle = nodeOnChain.m_data.m_frontProfile.RefParticle(newStartParticleHandle);
					newStartParticle.m_data.m_strandHandle = strandHandle;
					newStartParticle.m_data.m_strandSegmentHandle = newstrandSegmentHandle;
					newStartParticle.m_data.m_base = false;
					const auto newEndParticleHandle = nodeOnChain.m_data.m_backProfile.AllocateParticle();
					auto& newEndParticle = nodeOnChain.m_data.m_backProfile.RefParticle(newEndParticleHandle);
					newEndParticle.m_data.m_strandHandle = strandHandle;
					newEndParticle.m_data.m_strandSegmentHandle = newstrandSegmentHandle;
					newEndParticle.m_data.m_base = false;

					auto& newSegment = strandGroup.RefStrandSegment(newstrandSegmentHandle);
					newSegment.m_data.m_nodeHandle = *it;
					newSegment.m_data.m_frontProfileParticleHandle = newStartParticleHandle;
					newSegment.m_data.m_backProfileParticleHandle = newEndParticleHandle;
				}
				const auto position = glm::vec3(0.0f);
				const auto newstrandSegmentHandle = strandGroup.Extend(strandHandle);
				const auto newStartParticleHandle = frontPhysics2D.AllocateParticle();
				auto& newStartParticle = frontPhysics2D.RefParticle(newStartParticleHandle);
				newStartParticle.m_data.m_strandHandle = strandHandle;
				newStartParticle.m_data.m_strandSegmentHandle = newstrandSegmentHandle;
				newStartParticle.m_data.m_base = true;
				newStartParticle.SetPosition(position);
				const auto newEndParticleHandle = backPhysics2D.AllocateParticle();
				auto& newEndParticle = backPhysics2D.RefParticle(newEndParticleHandle);
				newEndParticle.m_data.m_strandHandle = strandHandle;
				newEndParticle.m_data.m_strandSegmentHandle = newstrandSegmentHandle;
				newEndParticle.m_data.m_base = true;
				newStartParticle.SetPosition(position);
				auto& newSegment = strandGroup.RefStrandSegment(newstrandSegmentHandle);
				newSegment.m_data.m_nodeHandle = internodeHandle;
				newSegment.m_data.m_frontProfileParticleHandle = newStartParticleHandle;
				newSegment.m_data.m_backProfileParticleHandle = newEndParticleHandle;
			}
		}
		else {
			if (frontPhysics2D.RefParticles().empty()) {
				for (int i = 0; i < strandModelParameters.m_endNodeStrands; i++) {
					const auto strandHandle = strandGroup.AllocateStrand();
					for (auto it = parentNodeToRootChain.rbegin(); it != parentNodeToRootChain.rend(); ++it) {
						const auto newstrandSegmentHandle = strandGroup.Extend(strandHandle);
						auto& nodeOnChain = m_strandModelSkeleton.RefNode(*it);
						const auto newStartParticleHandle = nodeOnChain.m_data.m_frontProfile.AllocateParticle();
						auto& newStartParticle = nodeOnChain.m_data.m_frontProfile.RefParticle(newStartParticleHandle);
						newStartParticle.m_data.m_strandHandle = strandHandle;
						newStartParticle.m_data.m_strandSegmentHandle = newstrandSegmentHandle;
						newStartParticle.m_data.m_base = false;
						const auto newEndParticleHandle = nodeOnChain.m_data.m_backProfile.AllocateParticle();
						auto& newEndParticle = nodeOnChain.m_data.m_backProfile.RefParticle(newEndParticleHandle);
						newEndParticle.m_data.m_strandHandle = strandHandle;
						newEndParticle.m_data.m_strandSegmentHandle = newstrandSegmentHandle;
						newEndParticle.m_data.m_base = false;

						auto& newSegment = strandGroup.RefStrandSegment(newstrandSegmentHandle);
						newSegment.m_data.m_nodeHandle = *it;
						newSegment.m_data.m_frontProfileParticleHandle = newStartParticleHandle;
						newSegment.m_data.m_backProfileParticleHandle = newEndParticleHandle;
					}
					const auto position = glm::diskRand(glm::sqrt(static_cast<float>(strandModelParameters.m_endNodeStrands)));
					const auto newstrandSegmentHandle = strandGroup.Extend(strandHandle);
					const auto newStartParticleHandle = frontPhysics2D.AllocateParticle();
					auto& newStartParticle = frontPhysics2D.RefParticle(newStartParticleHandle);
					newStartParticle.m_data.m_strandHandle = strandHandle;
					newStartParticle.m_data.m_strandSegmentHandle = newstrandSegmentHandle;
					newStartParticle.m_data.m_base = true;
					newStartParticle.SetPosition(position);
					const auto newEndParticleHandle = backPhysics2D.AllocateParticle();
					auto& newEndParticle = backPhysics2D.RefParticle(newEndParticleHandle);
					newEndParticle.m_data.m_strandHandle = strandHandle;
					newEndParticle.m_data.m_strandSegmentHandle = newstrandSegmentHandle;
					newEndParticle.m_data.m_base = true;
					newStartParticle.SetPosition(position);
					auto& newSegment = strandGroup.RefStrandSegment(newstrandSegmentHandle);
					newSegment.m_data.m_nodeHandle = internodeHandle;
					newSegment.m_data.m_frontProfileParticleHandle = newStartParticleHandle;
					newSegment.m_data.m_backProfileParticleHandle = newEndParticleHandle;
				}
			}
			else
			{
				for (ParticleHandle particleHandle = 0; particleHandle < frontPhysics2D.RefParticles().size(); particleHandle++)
				{
					const auto strandHandle = strandGroup.AllocateStrand();
					for (auto it = parentNodeToRootChain.rbegin(); it != parentNodeToRootChain.rend(); ++it) {
						const auto newstrandSegmentHandle = strandGroup.Extend(strandHandle);
						auto& nodeOnChain = m_strandModelSkeleton.RefNode(*it);
						const auto newStartParticleHandle = nodeOnChain.m_data.m_frontProfile.AllocateParticle();
						auto& newStartParticle = nodeOnChain.m_data.m_frontProfile.RefParticle(newStartParticleHandle);
						newStartParticle.m_data.m_strandHandle = strandHandle;
						newStartParticle.m_data.m_strandSegmentHandle = newstrandSegmentHandle;
						newStartParticle.m_data.m_base = false;
						const auto newEndParticleHandle = nodeOnChain.m_data.m_backProfile.AllocateParticle();
						auto& newEndParticle = nodeOnChain.m_data.m_backProfile.RefParticle(newEndParticleHandle);
						newEndParticle.m_data.m_strandHandle = strandHandle;
						newEndParticle.m_data.m_strandSegmentHandle = newstrandSegmentHandle;
						newEndParticle.m_data.m_base = false;

						auto& newSegment = strandGroup.RefStrandSegment(newstrandSegmentHandle);
						newSegment.m_data.m_nodeHandle = *it;
						newSegment.m_data.m_frontProfileParticleHandle = newStartParticleHandle;
						newSegment.m_data.m_backProfileParticleHandle = newEndParticleHandle;
					}
					const auto newstrandSegmentHandle = strandGroup.Extend(strandHandle);
					auto& frontParticle = frontPhysics2D.RefParticle(particleHandle);
					frontParticle.m_data.m_strandHandle = strandHandle;
					frontParticle.m_data.m_strandSegmentHandle = newstrandSegmentHandle;
					frontParticle.m_data.m_base = true;
					auto& backParticle = backPhysics2D.RefParticle(particleHandle);
					backParticle.m_data.m_strandHandle = strandHandle;
					backParticle.m_data.m_strandSegmentHandle = newstrandSegmentHandle;
					backParticle.m_data.m_base = true;
					frontParticle.SetPosition(backParticle.GetPosition());
					auto& newSegment = strandGroup.RefStrandSegment(newstrandSegmentHandle);
					newSegment.m_data.m_nodeHandle = internodeHandle;
					newSegment.m_data.m_frontProfileParticleHandle = particleHandle;
					newSegment.m_data.m_backProfileParticleHandle = particleHandle;
				}
			}
		}
	}

	for (const auto& internodeHandle : sortedInternodeList)
	{
		auto& internode = m_strandModelSkeleton.RefNode(internodeHandle);
		internode.m_data.m_frontParticleMap.clear();
		internode.m_data.m_backParticleMap.clear();
		auto& startPhysics2D = internode.m_data.m_frontProfile;

		for (auto& particle : startPhysics2D.RefParticles())
		{
			internode.m_data.m_frontParticleMap.insert({ particle.m_data.m_strandHandle, particle.GetHandle() });
		}
		auto& endPhysics2D = internode.m_data.m_backProfile;

		for (auto& particle : endPhysics2D.RefParticles())
		{
			internode.m_data.m_backParticleMap.insert({ particle.m_data.m_strandHandle, particle.GetHandle() });
		}
		internode.m_data.m_strandCount = startPhysics2D.RefParticles().size();
	}
	m_strandModelSkeleton.m_data.m_numOfParticles = 0;
	for (const auto& internodeHandle : sortedInternodeList)
	{
		auto& internode = m_strandModelSkeleton.RefNode(internodeHandle);
		int maxChildSize = 0;
		NodeHandle maxChildHandle = -1;
		for (const auto& childHandle : internode.RefChildHandles())
		{
			auto& childInternode = m_strandModelSkeleton.RefNode(childHandle);
			const auto childSize = static_cast<float>(childInternode.m_data.m_frontParticleMap.size());
			if (childSize > maxChildSize)
			{
				maxChildSize = childSize;
				maxChildHandle = childHandle;
			}
		}
		for (const auto& childHandle : internode.RefChildHandles())
		{
			auto& childInternode = m_strandModelSkeleton.RefNode(childHandle);
			if (childHandle == maxChildHandle) childInternode.m_data.m_split = false;
			const auto childSize = static_cast<float>(childInternode.m_data.m_frontParticleMap.size());
			if (childSize > maxChildSize * strandModelParameters.m_overlapThreshold)
			{
				childInternode.m_data.m_split = true;
			}
			else
			{
				childInternode.m_data.m_split = false;
			}
		}
		m_strandModelSkeleton.m_data.m_numOfParticles += internode.m_data.m_frontProfile.PeekParticles().size();
	}
}

void StrandModel::CalculateProfiles(const StrandModelParameters& strandModelParameters)
{
	const float time = Times::Now();
	const auto& sortedInternodeList = m_strandModelSkeleton.PeekSortedNodeList();
	if (sortedInternodeList.empty()) return;

	Jobs::ParallelFor(sortedInternodeList.size(), [&](unsigned i)
		{
			auto& internode = m_strandModelSkeleton.RefNode(sortedInternodeList[i]);
			for (auto& particle : internode.m_data.m_backProfile.RefParticles()) {
				if (!internode.IsEndNode()) particle.SetPosition(glm::vec2(0.0f));
				particle.SetVelocity(glm::vec2(0.0f), 0.002f);
				particle.SetAcceleration(glm::vec2(0.0f));
			}
			for (auto& particle : internode.m_data.m_frontProfile.RefParticles())
			{
				if (!internode.IsEndNode()) particle.SetPosition(glm::vec2(0.0f));
				particle.SetVelocity(glm::vec2(0.0f), 0.002f);
				particle.SetAcceleration(glm::vec2(0.0f));
			}
		}
	);
	const auto& baseNode = m_strandModelSkeleton.PeekNode(0);
	const float maxRootDistance = baseNode.m_info.m_endDistance + baseNode.m_info.m_length;
	if (m_strandModelSkeleton.m_data.m_parallelScheduling) {
		for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); ++it)
		{
			CalculateProfile(maxRootDistance, *it, strandModelParameters, true);
			m_strandModelSkeleton.m_data.m_numOfParticles += m_strandModelSkeleton.RefNode(*it).m_data.m_frontProfile.PeekParticles().size();
		}
		if (!sortedInternodeList.empty())
		{
			Wait(sortedInternodeList.front());
		}
	}
	else
	{
		for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); ++it)
		{
			CalculateProfile(maxRootDistance, *it, strandModelParameters, false);
			Wait(*it);
			m_strandModelSkeleton.m_data.m_numOfParticles += m_strandModelSkeleton.RefNode(*it).m_data.m_frontProfile.PeekParticles().size();
		}
	}
	m_strandModelSkeleton.m_data.m_profileCalculationTime = Times::Now() - time;
}

void StrandModel::CalculateProfile(const float maxRootDistance, const NodeHandle nodeHandle, const StrandModelParameters& strandModelParameters, bool scheduling)
{
	if (scheduling) {
		m_strandModelSkeleton.RefNode(nodeHandle).m_data.m_tasks.emplace_back(Jobs::AddTask([&, nodeHandle, scheduling](unsigned threadIndex) {
			MergeTask(maxRootDistance, nodeHandle, strandModelParameters);
			auto& internode = m_strandModelSkeleton.RefNode(nodeHandle);
			if (internode.m_data.m_frontProfile.PeekParticles().size() > 1) {
				PackTask(nodeHandle, strandModelParameters, !scheduling);
				if (internode.RefChildHandles().empty()) CopyFrontToBackTask(nodeHandle);
				CalculateShiftTask(nodeHandle, strandModelParameters);
			}
			internode.m_data.m_frontProfile.CalculateBoundaries(true, strandModelParameters.m_boundaryPointDistance);
			internode.m_data.m_backProfile.CalculateBoundaries(true, strandModelParameters.m_boundaryPointDistance);
			}
		)
		);
	}
	else
	{
		MergeTask(maxRootDistance, nodeHandle, strandModelParameters);
		auto& internode = m_strandModelSkeleton.RefNode(nodeHandle);
		if (internode.m_data.m_frontProfile.PeekParticles().size() > 1) {
			PackTask(nodeHandle, strandModelParameters, !scheduling);
			if (internode.RefChildHandles().empty()) CopyFrontToBackTask(nodeHandle);
			CalculateShiftTask(nodeHandle, strandModelParameters);
		}
		internode.m_data.m_frontProfile.CalculateBoundaries(true, strandModelParameters.m_boundaryPointDistance);
		internode.m_data.m_backProfile.CalculateBoundaries(true, strandModelParameters.m_boundaryPointDistance);
	}
}

void StrandModel::Wait(const NodeHandle nodeHandle)
{
	auto& internode = m_strandModelSkeleton.RefNode(nodeHandle);
	if (internode.m_data.m_tasks.empty()) return;
	for (const auto& i : internode.m_data.m_tasks)
	{
		i.wait();
	}
	internode.m_data.m_tasks.clear();
}

void StrandModel::PackTask(const NodeHandle nodeHandle, const StrandModelParameters& strandModelParameters, const bool parallel)
{
	auto& internode = m_strandModelSkeleton.RefNode(nodeHandle);
	auto& internodeData = internode.m_data;
	internodeData.m_frontProfile.m_parallel = parallel;

	const auto iterations = internodeData.m_packingIteration;

	int timeout = strandModelParameters.m_junctionProfilePackingMaxIteration;
	if (!internodeData.m_profileConstraints.m_boundaries.empty()) timeout = strandModelParameters.m_modifiedProfilePackingMaxIteration;
	for (int i = 0; i < iterations; i++) {
		internodeData.m_frontProfile.Simulate(1,
			[&](auto& grid, bool gridResized)
			{
				if (gridResized || internodeData.m_boundariesUpdated) grid.ApplyBoundaries(internodeData.m_profileConstraints);
				internodeData.m_boundariesUpdated = false;
			},
			[&](auto& particle)
			{
				auto acceleration = glm::vec2(0.f);
				if (!internodeData.m_frontProfile.m_particleGrid2D.PeekCells().empty()) {
					const auto& cell = internodeData.m_frontProfile.m_particleGrid2D.RefCell(particle.GetPosition());
					if (glm::length(cell.m_target) > glm::epsilon<float>()) {
						acceleration += strandModelParameters.m_centerAttractionStrength * glm::normalize(cell.m_target);
					}
				}
				particle.SetAcceleration(acceleration);
			}
		);
		if (timeout > 0 && i > timeout) break;
	}
}

void StrandModel::MergeTask(float maxRootDistance, NodeHandle nodeHandle, const StrandModelParameters& strandModelParameters)
{
	auto& internode = m_strandModelSkeleton.RefNode(nodeHandle);
	auto& internodeData = internode.m_data;
	internodeData.m_twistAngle = 0.0f;
	const auto& childHandles = internode.RefChildHandles();
	int maxChildSize = -1;
	NodeHandle maxChildHandle = -1;
	for (const auto& childHandle : childHandles) {
		Wait(childHandle);
		auto& childInternode = m_strandModelSkeleton.RefNode(childHandle);
		const auto childSize = static_cast<float>(childInternode.m_data.m_frontParticleMap.size());
		if (childSize > maxChildSize)
		{
			maxChildSize = childSize;
			maxChildHandle = childHandle;
		}
	}
	internodeData.m_centerDirectionRadius = 0.0f;
	if (!internodeData.m_profileConstraints.m_boundaries.empty())
	{
		internodeData.m_packingIteration = glm::min(strandModelParameters.m_modifiedProfilePackingMaxIteration, static_cast<int>(internodeData.m_frontProfile.RefParticles().size()) * strandModelParameters.m_maxSimulationIterationCellFactor);
	}
	if (childHandles.empty())
	{
		if (internodeData.m_profileConstraints.m_boundaries.empty()) internode.m_data.m_packingIteration = glm::min(strandModelParameters.m_junctionProfilePackingMaxIteration, static_cast<int>(internodeData.m_frontProfile.RefParticles().size()) * strandModelParameters.m_maxSimulationIterationCellFactor);
		int particleIndex = 0;
		for (const auto& particle : internodeData.m_frontProfile.RefParticles())
		{
			const auto nodeStartParticleHandle = internodeData.m_frontParticleMap.at(particle.m_data.m_strandHandle);
			const auto nodeEndParticleHandle = internodeData.m_backParticleMap.at(particle.m_data.m_strandHandle);
			auto& nodeStartParticle = internodeData.m_frontProfile.RefParticle(nodeStartParticleHandle);
			auto& nodeEndParticle = internodeData.m_backProfile.RefParticle(nodeEndParticleHandle);
			particleIndex++;
			nodeStartParticle.SetColor(particle.GetColor());
			nodeEndParticle.SetColor(particle.GetColor());
			nodeStartParticle.m_data.m_mainChild = nodeEndParticle.m_data.m_mainChild = true;
		}
		return;
	}

	if (maxChildHandle == -1) maxChildHandle = childHandles.front();
	auto& mainChildNode = m_strandModelSkeleton.RefNode(maxChildHandle);
	const auto branchTwistAngle = strandModelParameters.m_branchTwistDistribution.GetValue(internode.m_info.m_rootDistance / maxRootDistance);
	const auto junctionTwistAngle = strandModelParameters.m_junctionTwistDistribution.GetValue(internode.m_info.m_rootDistance / maxRootDistance);
	if (childHandles.size() == 1)
	{
		//Copy from child flow start to self flow start
		auto& childNode = m_strandModelSkeleton.RefNode(childHandles.front());
		const auto& childPhysics2D = childNode.m_data.m_frontProfile;
		childNode.m_data.m_twistAngle = branchTwistAngle;

		for (const auto& childParticle : childPhysics2D.PeekParticles())
		{
			const auto nodeStartParticleHandle = internodeData.m_frontParticleMap.at(childParticle.m_data.m_strandHandle);
			const auto nodeEndParticleHandle = internodeData.m_backParticleMap.at(childParticle.m_data.m_strandHandle);
			auto& nodeStartParticle = internodeData.m_frontProfile.RefParticle(nodeStartParticleHandle);
			auto& nodeEndParticle = internodeData.m_backProfile.RefParticle(nodeEndParticleHandle);
			nodeStartParticle.SetColor(childParticle.GetColor());
			nodeEndParticle.SetColor(childParticle.GetColor());
			auto polarPosition = childParticle.GetPolarPosition();
			polarPosition.y += glm::radians(branchTwistAngle);
			nodeStartParticle.SetPolarPosition(polarPosition);
			nodeEndParticle.SetPolarPosition(polarPosition);

			nodeStartParticle.m_data.m_mainChild = nodeEndParticle.m_data.m_mainChild = true;
		}
		if (internodeData.m_profileConstraints.m_boundaries.empty()) internode.m_data.m_packingIteration = glm::min(strandModelParameters.m_branchProfilePackingMaxIteration, static_cast<int>(internodeData.m_frontProfile.RefParticles().size()) * strandModelParameters.m_maxSimulationIterationCellFactor);
		return;
	}
	if (internodeData.m_profileConstraints.m_boundaries.empty()) internode.m_data.m_packingIteration = glm::min(strandModelParameters.m_junctionProfilePackingMaxIteration, static_cast<int>(internodeData.m_frontProfile.RefParticles().size()) * strandModelParameters.m_maxSimulationIterationCellFactor);
	const auto& mainChildPhysics2D = mainChildNode.m_data.m_frontProfile;
	mainChildNode.m_data.m_twistAngle = junctionTwistAngle;
	for (const auto& mainChildParticle : mainChildPhysics2D.PeekParticles())
	{
		const auto nodeStartParticleHandle = internodeData.m_frontParticleMap.at(mainChildParticle.m_data.m_strandHandle);
		const auto nodeEndParticleHandle = internodeData.m_backParticleMap.at(mainChildParticle.m_data.m_strandHandle);
		auto& nodeStartParticle = internodeData.m_frontProfile.RefParticle(nodeStartParticleHandle);
		auto& nodeEndParticle = internodeData.m_backProfile.RefParticle(nodeEndParticleHandle);
		nodeStartParticle.SetColor(mainChildParticle.GetColor());
		nodeEndParticle.SetColor(mainChildParticle.GetColor());
		auto polarPosition = mainChildParticle.GetPolarPosition();
		polarPosition.y += glm::radians(junctionTwistAngle);
		nodeStartParticle.SetPolarPosition(polarPosition);
		nodeEndParticle.SetPolarPosition(polarPosition);

		nodeStartParticle.m_data.m_mainChild = nodeEndParticle.m_data.m_mainChild = true;
	}
	//const auto nodeGlobalRotation = scene->GetDataComponent<GlobalTransform>(owner);
	if (strandModelParameters.m_preMerge) {
		bool needSimulation = false;
		for (const auto& childHandle : childHandles)
		{
			if (childHandle == maxChildHandle) continue;
			auto& childNode = m_strandModelSkeleton.RefNode(childHandle);
			auto& childPhysics2D = childNode.m_data.m_frontProfile;
			if (!childNode.m_data.m_split)
			{
				needSimulation = true;
				const auto childNodeFront = glm::inverse(internode.m_info.m_regulatedGlobalRotation) * childNode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
				auto direction = glm::normalize(glm::vec2(childNodeFront.x, childNodeFront.y));
				if (glm::isnan(direction.x) || glm::isnan(direction.y))
				{
					direction = glm::vec2(1, 0);
				}
				childNode.m_data.m_centerDirectionRadius = childPhysics2D.GetDistanceToCenter(-direction);
				const auto mainChildRadius = mainChildPhysics2D.GetDistanceToCenter(direction);
				auto offset = glm::vec2(0.0f);
				offset = (mainChildRadius - childNode.m_data.m_centerDirectionRadius + 2.0f) * direction;
				childNode.m_data.m_offset = offset;
				for (auto& childParticle : childPhysics2D.RefParticles())
				{
					childParticle.SetPosition(childParticle.GetPosition() + offset);

					const auto nodeStartParticleHandle = internodeData.m_frontParticleMap.at(childParticle.m_data.m_strandHandle);
					const auto nodeEndParticleHandle = internodeData.m_backParticleMap.at(childParticle.m_data.m_strandHandle);
					auto& nodeStartParticle = internodeData.m_frontProfile.RefParticle(nodeStartParticleHandle);
					auto& nodeEndParticle = internodeData.m_backProfile.RefParticle(nodeEndParticleHandle);
					nodeStartParticle.SetColor(childParticle.GetColor());
					nodeEndParticle.SetColor(childParticle.GetColor());
					auto polarPosition = childParticle.GetPolarPosition();
					polarPosition.y += glm::radians(junctionTwistAngle);
					nodeStartParticle.SetPolarPosition(polarPosition);
					nodeEndParticle.SetPolarPosition(polarPosition);
					nodeStartParticle.m_enable = true;
					nodeStartParticle.m_data.m_mainChild = nodeEndParticle.m_data.m_mainChild = false;
				}
			}
			else
			{
				for (auto& childParticle : childPhysics2D.RefParticles())
				{
					const auto nodeStartParticleHandle = internodeData.m_frontParticleMap.at(childParticle.m_data.m_strandHandle);
					const auto nodeEndParticleHandle = internodeData.m_backParticleMap.at(childParticle.m_data.m_strandHandle);
					auto& nodeStartParticle = internodeData.m_frontProfile.RefParticle(nodeStartParticleHandle);
					auto& nodeEndParticle = internodeData.m_backProfile.RefParticle(nodeEndParticleHandle);
					nodeStartParticle.SetColor(childParticle.GetColor());
					nodeEndParticle.SetColor(childParticle.GetColor());
					auto polarPosition = childParticle.GetPolarPosition();
					polarPosition.y += glm::radians(junctionTwistAngle);
					nodeStartParticle.SetPolarPosition(polarPosition);
					nodeEndParticle.SetPolarPosition(polarPosition);
					nodeStartParticle.m_enable = false;
					nodeStartParticle.m_data.m_mainChild = nodeEndParticle.m_data.m_mainChild = false;
				}
			}
		}
		if (needSimulation) PackTask(nodeHandle, strandModelParameters, false);
		for (const auto& childHandle : childHandles)
		{
			if (childHandle == maxChildHandle) continue;
			auto& childNode = m_strandModelSkeleton.RefNode(childHandle);
			if (childNode.m_data.m_split)
			{
				auto& childPhysics2D = childNode.m_data.m_frontProfile;
				const auto childNodeFront = glm::inverse(internode.m_info.m_regulatedGlobalRotation) * childNode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
				auto direction = glm::normalize(glm::vec2(childNodeFront.x, childNodeFront.y));
				if (glm::isnan(direction.x) || glm::isnan(direction.y))
				{
					direction = glm::vec2(1, 0);
				}
				childNode.m_data.m_centerDirectionRadius = childPhysics2D.GetDistanceToCenter(-direction);
				const auto centerRadius = internodeData.m_frontProfile.GetDistanceToCenter(direction);
				auto offset = glm::vec2(0.0f);
				offset = (centerRadius + childNode.m_data.m_centerDirectionRadius + 2.0f) * direction;
				childNode.m_data.m_offset = offset;
				for (auto& childParticle : childPhysics2D.RefParticles())
				{
					auto polarPosition = childParticle.GetPolarPosition();
					polarPosition.y += glm::radians(junctionTwistAngle);
					childParticle.SetPolarPosition(polarPosition);
					childParticle.SetPosition(childParticle.GetPosition() + offset);

					const auto nodeStartParticleHandle = internodeData.m_frontParticleMap.at(childParticle.m_data.m_strandHandle);
					const auto nodeEndParticleHandle = internodeData.m_backParticleMap.at(childParticle.m_data.m_strandHandle);
					auto& nodeStartParticle = internodeData.m_frontProfile.RefParticle(nodeStartParticleHandle);
					auto& nodeEndParticle = internodeData.m_backProfile.RefParticle(nodeEndParticleHandle);
					nodeStartParticle.SetColor(childParticle.GetColor());
					nodeEndParticle.SetColor(childParticle.GetColor());
					nodeStartParticle.SetPosition(childParticle.GetPosition());
					nodeEndParticle.SetPosition(childParticle.GetPosition());

					nodeStartParticle.m_data.m_mainChild = nodeEndParticle.m_data.m_mainChild = false;
				}
			}
		}
		CopyFrontToBackTask(nodeHandle);
		internodeData.m_frontProfile.SetEnableAllParticles(true);
	}
	else {
		for (const auto& childHandle : childHandles)
		{
			if (childHandle == maxChildHandle) continue;
			auto& childNode = m_strandModelSkeleton.RefNode(childHandle);
			childNode.m_data.m_twistAngle = junctionTwistAngle;
			auto& childPhysics2D = childNode.m_data.m_frontProfile;
			const auto childNodeFront = glm::inverse(internode.m_info.m_regulatedGlobalRotation) * childNode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
			auto direction = glm::normalize(glm::vec2(childNodeFront.x, childNodeFront.y));
			if (glm::isnan(direction.x) || glm::isnan(direction.y))
			{
				direction = glm::vec2(1, 0);
			}
			childNode.m_data.m_centerDirectionRadius = childPhysics2D.GetDistanceToCenter(-direction);
			const auto mainChildRadius = mainChildPhysics2D.GetDistanceToCenter(direction);
			auto offset = glm::vec2(0.0f);
			if (childNode.m_data.m_split)
			{
				offset = (mainChildRadius + childNode.m_data.m_centerDirectionRadius + 2.0f) * direction;
			}
			else
			{
				offset = (mainChildRadius - childNode.m_data.m_centerDirectionRadius) * direction;
			}
			childNode.m_data.m_offset = offset;
			for (auto& childParticle : childPhysics2D.RefParticles())
			{
				childParticle.SetPosition(childParticle.GetPosition() + offset);

				const auto nodeStartParticleHandle = internodeData.m_frontParticleMap.at(childParticle.m_data.m_strandHandle);
				const auto nodeEndParticleHandle = internodeData.m_backParticleMap.at(childParticle.m_data.m_strandHandle);
				auto& nodeStartParticle = internodeData.m_frontProfile.RefParticle(nodeStartParticleHandle);
				auto& nodeEndParticle = internodeData.m_backProfile.RefParticle(nodeEndParticleHandle);
				nodeStartParticle.SetColor(childParticle.GetColor());
				nodeEndParticle.SetColor(childParticle.GetColor());
				auto polarPosition = childParticle.GetPolarPosition();
				polarPosition.y += glm::radians(strandModelParameters.m_junctionTwistDistribution.GetValue(internode.m_info.m_rootDistance / maxRootDistance));
				nodeStartParticle.SetPolarPosition(polarPosition);
				nodeEndParticle.SetPolarPosition(polarPosition);

				nodeStartParticle.m_data.m_mainChild = nodeEndParticle.m_data.m_mainChild = false;
			}
		}
	}
}

void StrandModel::CopyFrontToBackTask(const NodeHandle nodeHandle)
{
	auto& internode = m_strandModelSkeleton.RefNode(nodeHandle);
	auto& internodeData = internode.m_data;
	for (int i = 0; i < internodeData.m_frontProfile.RefParticles().size(); i++)
	{
		internodeData.m_backProfile.RefParticle(i).SetPosition(internodeData.m_frontProfile.RefParticle(i).GetPosition());
	}
}

void StrandModel::CalculateShiftTask(const NodeHandle nodeHandle, const StrandModelParameters& strandModelParameters)
{
	auto& internode = m_strandModelSkeleton.RefNode(nodeHandle);
	auto& internodeData = internode.m_data;
	const auto& childHandles = internode.RefChildHandles();
	int maxChildSize = -1;
	NodeHandle maxChildHandle = -1;
	for (const auto& childHandle : childHandles) {
		Wait(childHandle);
		auto& childInternode = m_strandModelSkeleton.RefNode(childHandle);
		const auto childSize = static_cast<float>(childInternode.m_data.m_frontParticleMap.size());
		if (childSize > maxChildSize)
		{
			maxChildSize = childSize;
			maxChildHandle = childHandle;
		}
	}
	internodeData.m_centerDirectionRadius = 0.0f;
	if (childHandles.empty())
	{
		internode.m_data.m_packingIteration = glm::min(strandModelParameters.m_junctionProfilePackingMaxIteration, static_cast<int>(internodeData.m_frontProfile.RefParticles().size()) * strandModelParameters.m_maxSimulationIterationCellFactor);
		return;
	}
	if (maxChildHandle == -1) maxChildHandle = childHandles.front();
	const auto& mainChildNode = m_strandModelSkeleton.RefNode(maxChildHandle);
	const auto& mainChildPhysics2D = mainChildNode.m_data.m_frontProfile;
	auto sum = glm::vec2(0.0f);
	for (const auto& mainChildParticle : mainChildPhysics2D.PeekParticles())
	{
		const auto nodeStartParticleHandle = internodeData.m_frontParticleMap.at(mainChildParticle.m_data.m_strandHandle);
		auto& nodeStartParticle = internodeData.m_frontProfile.RefParticle(nodeStartParticleHandle);
		sum += nodeStartParticle.GetPosition();
	}
	internodeData.m_shift = sum / static_cast<float>(mainChildPhysics2D.PeekParticles().size());

}

void StrandModel::ApplyProfile(const StrandModelParameters& strandModelParameters,
	const NodeHandle nodeHandle)
{
	const auto& node = m_strandModelSkeleton.RefNode(nodeHandle);
	const auto currentFront = node.m_data.m_adjustedGlobalRotation * glm::vec3(0, 0, -1);
	const auto currentUp = node.m_data.m_adjustedGlobalRotation * glm::vec3(0, 1, 0);
	const auto currentLeft = node.m_data.m_adjustedGlobalRotation * glm::vec3(1, 0, 0);
	const auto& parameters = strandModelParameters;
	bool wound = node.IsEndNode();
	for (const auto& [strandHandle, particleHandle] : node.m_data.m_backParticleMap)
	{
		const auto& particle = node.m_data.m_backProfile.PeekParticle(particleHandle);
		auto& newstrandSegment = m_strandModelSkeleton.m_data.m_strandGroup.RefStrandSegment(particle.m_data.m_strandSegmentHandle);
		newstrandSegment.m_info.m_thickness = node.m_data.m_strandRadius;
		newstrandSegment.m_info.m_globalPosition = node.m_data.m_adjustedGlobalPosition
			+ node.m_data.m_strandRadius * particle.GetPosition().x * currentLeft
			+ node.m_data.m_strandRadius * particle.GetPosition().y * currentUp;
		if (wound)
		{
			newstrandSegment.m_info.m_globalPosition += currentFront * glm::max(0.0f, strandModelParameters.m_cladoptosisDistribution.GetValue(glm::max(0.0f, (strandModelParameters.m_cladoptosisRange - particle.GetDistanceToBoundary()) / strandModelParameters.m_cladoptosisRange)));
		}
		newstrandSegment.m_info.m_globalRotation = node.m_data.m_adjustedGlobalRotation;
		newstrandSegment.m_info.m_color = particle.IsBoundary() ? parameters.m_boundaryPointColor : parameters.m_contentPointColor;
		newstrandSegment.m_info.m_isBoundary = particle.IsBoundary();
	}
}

void StrandModel::ApplyProfiles(const StrandModelParameters& strandModelParameters)
{
	auto& strandGroup = m_strandModelSkeleton.m_data.m_strandGroup;
	const auto& sortedInternodeList = m_strandModelSkeleton.PeekSortedNodeList();
	for (const auto& nodeHandle : sortedInternodeList)
	{
		const auto& node = m_strandModelSkeleton.RefNode(nodeHandle);
		glm::quat parentGlobalRotation;
		glm::vec3 parentGlobalPosition;
		if (node.GetParentHandle() == -1)
		{
			parentGlobalRotation = node.m_data.m_adjustedGlobalRotation;
			parentGlobalPosition = glm::vec3(0.0f);
		}
		else {
			const auto& parent = m_strandModelSkeleton.RefNode(node.GetParentHandle());
			parentGlobalRotation = parent.m_data.m_adjustedGlobalRotation;
			parentGlobalPosition = parent.m_data.m_adjustedGlobalPosition;
		}
		if (node.GetParentHandle() == -1)
		{
			const auto currentUp = parentGlobalRotation * glm::vec3(0, 1, 0);
			const auto currentLeft = parentGlobalRotation * glm::vec3(1, 0, 0);
			const auto baseRadius = strandModelParameters.m_strandRadiusDistribution.GetValue(0.0f);
			for (const auto& [strandHandle, particleHandle] : node.m_data.m_frontParticleMap)
			{
				const auto& particle = node.m_data.m_frontProfile.PeekParticle(particleHandle);
				auto& strand = strandGroup.RefStrand(strandHandle);
				strand.m_info.m_baseInfo.m_thickness = baseRadius;
				strand.m_info.m_baseInfo.m_globalPosition = parentGlobalPosition
					+ strand.m_info.m_baseInfo.m_thickness * particle.GetPosition().x * currentLeft
					+ strand.m_info.m_baseInfo.m_thickness * particle.GetPosition().y * currentUp;
				strand.m_info.m_baseInfo.m_globalRotation = parentGlobalRotation;
				strand.m_info.m_baseInfo.m_isBoundary = particle.IsBoundary();
				strand.m_info.m_baseInfo.m_color = particle.IsBoundary() ? strandModelParameters.m_boundaryPointColor : strandModelParameters.m_contentPointColor;
			}
		}
		ApplyProfile(strandModelParameters, nodeHandle);
	}
}

void StrandModel::CalculateStrandProfileAdjustedTransforms(const StrandModelParameters& strandModelParameters)
{
	const auto& sortedInternodeList = m_strandModelSkeleton.PeekSortedNodeList();
	float maxRootDistance = 0.0f;
	for (const auto& nodeHandle : sortedInternodeList)
	{
		auto& node = m_strandModelSkeleton.RefNode(nodeHandle);
		const auto parentHandle = node.GetParentHandle();
		if (parentHandle == -1)
		{
			node.m_data.m_adjustedGlobalPosition = node.m_info.GetGlobalEndPosition();
			node.m_data.m_adjustedGlobalRotation = node.m_info.m_regulatedGlobalRotation;
			maxRootDistance = node.m_info.m_endDistance + node.m_info.m_length;

			node.m_data.m_strandRadius = strandModelParameters.m_strandRadiusDistribution.GetValue(node.m_info.m_rootDistance / maxRootDistance);
			continue;
		}
		const auto& parentNode = m_strandModelSkeleton.PeekNode(parentHandle);
		glm::vec3 parentGlobalPosition = parentNode.m_data.m_adjustedGlobalPosition;
		glm::quat parentGlobalRotation = parentNode.m_data.m_adjustedGlobalRotation;
		node.m_data.m_strandRadius = strandModelParameters.m_strandRadiusDistribution.GetValue(node.m_info.m_rootDistance / maxRootDistance);
		node.m_data.m_adjustedGlobalPosition = parentGlobalPosition + node.m_info.GetGlobalEndPosition() - parentNode.m_info.GetGlobalEndPosition();
		node.m_data.m_adjustedGlobalRotation = parentGlobalRotation * (glm::inverse(parentNode.m_info.m_regulatedGlobalRotation) * node.m_info.m_regulatedGlobalRotation);

		const auto parentUp = parentGlobalRotation * glm::vec3(0, 1, 0);
		const auto parentLeft = parentGlobalRotation * glm::vec3(1, 0, 0);
		const auto parentFront = parentGlobalRotation * glm::vec3(0, 0, -1);


		const auto front = node.m_data.m_adjustedGlobalRotation * glm::vec3(0, 0, -1);
		const float offsetLength = glm::length(node.m_data.m_offset);
		float maxDistanceToCenter = node.m_data.m_frontProfile.GetMaxDistanceToCenter();
		const float cosFront = glm::dot(front, parentFront); //Horizontal
		const float sinFront = glm::sin(glm::acos(glm::clamp(cosFront, -1.0f, 1.0f))); //Vertical
		if (!node.m_data.m_apical && offsetLength > glm::epsilon<float>()) {
			const float outerRadius = node.m_data.m_frontProfile.GetDistanceToCenter(glm::normalize(node.m_data.m_offset));
			const float innerRadius = node.m_data.m_frontProfile.GetDistanceToCenter(-glm::normalize(node.m_data.m_offset));
			const auto offsetDirection = glm::normalize(node.m_data.m_offset);
			const auto newOffset = (offsetLength + innerRadius + (outerRadius - outerRadius * cosFront) * strandModelParameters.m_rotationPushRatio) * offsetDirection;
			node.m_data.m_adjustedGlobalPosition += parentUp * newOffset.y * strandModelParameters.m_sidePushRatio * node.m_data.m_strandRadius;
			node.m_data.m_adjustedGlobalPosition += parentLeft * newOffset.x * strandModelParameters.m_sidePushRatio * node.m_data.m_strandRadius;
			node.m_data.m_adjustedGlobalPosition += parentFront * (sinFront * outerRadius * strandModelParameters.m_rotationPushRatio) * node.m_data.m_strandRadius;
		}
		else if (node.m_data.m_apical)
		{
			//This part needs improvement.
			node.m_data.m_adjustedGlobalPosition += parentFront * sinFront * maxDistanceToCenter * strandModelParameters.m_frontPushRatio * node.m_data.m_strandRadius;
		}
		node.m_data.m_adjustedGlobalPosition += parentUp * node.m_data.m_shift.y * strandModelParameters.m_shiftPushRatio * node.m_data.m_strandRadius;
		node.m_data.m_adjustedGlobalPosition += parentLeft * node.m_data.m_shift.x * strandModelParameters.m_shiftPushRatio * node.m_data.m_strandRadius;

	}
}


EvoEngine::StrandPoint operator/(const EvoEngine::StrandPoint& lhs, const float& rhs) {
	EvoEngine::StrandPoint retVal;
	retVal.m_thickness = lhs.m_thickness / rhs;
	retVal.m_position = lhs.m_position / rhs;
	retVal.m_color = lhs.m_color / rhs;
	retVal.m_texCoord = lhs.m_texCoord / rhs;
	return retVal;
}

EvoEngine::StrandPoint operator*(const EvoEngine::StrandPoint& lhs, const float& rhs) {
	EvoEngine::StrandPoint retVal;
	retVal.m_thickness = lhs.m_thickness * rhs;
	retVal.m_position = lhs.m_position * rhs;
	retVal.m_color = lhs.m_color * rhs;
	retVal.m_texCoord = lhs.m_texCoord * rhs;
	return retVal;
}

EvoEngine::StrandPoint
operator+(const EvoEngine::StrandPoint& lhs, const EvoEngine::StrandPoint& rhs) {
	EvoEngine::StrandPoint retVal;
	retVal.m_thickness = lhs.m_thickness + rhs.m_thickness;
	retVal.m_position = lhs.m_position + rhs.m_position;
	retVal.m_color = lhs.m_color + rhs.m_color;
	retVal.m_texCoord = lhs.m_texCoord + rhs.m_texCoord;
	return retVal;
}

EvoEngine::StrandPoint
operator-(const EvoEngine::StrandPoint& lhs, const EvoEngine::StrandPoint& rhs) {
	EvoEngine::StrandPoint retVal;
	retVal.m_thickness = lhs.m_thickness - rhs.m_thickness;
	retVal.m_position = lhs.m_position - rhs.m_position;
	retVal.m_color = lhs.m_color - rhs.m_color;
	retVal.m_texCoord = lhs.m_texCoord - rhs.m_texCoord;
	return retVal;
}

glm::vec3 StrandModel::InterpolateStrandSegmentPosition(const StrandSegmentHandle strandSegmentHandle, const float a) const
{
	assert(strandSegmentHandle >= 0);
	assert(a >= 0.f && a <= 1.f);
	const auto& strandGroup = m_strandModelSkeleton.m_data.m_strandGroup;
	assert(strandGroup.PeekStrandSegments().size() > strandSegmentHandle);
	const auto& strandSegment = strandGroup.PeekStrandSegment(strandSegmentHandle);
	const auto& strand = strandGroup.PeekStrand(strandSegment.GetStrandHandle());
	auto& baseInfo = strand.m_info.m_baseInfo;

	const auto& strandSegmentHandles = strand.PeekStrandSegmentHandles();

	glm::vec3 p[4];

	p[2] = strandSegment.m_info.m_globalPosition;
	if (strandSegmentHandle == strandSegmentHandles.front())
	{
		p[1] = baseInfo.m_globalPosition;
		p[0] = p[1] * 2.0f - p[2];
	}
	else if (strandSegmentHandle == strandSegmentHandles.at(1))
	{
		p[0] = baseInfo.m_globalPosition;
		p[1] = strandGroup.PeekStrandSegment(strandSegmentHandles.front()).m_info.m_globalPosition;
	}
	else
	{
		const auto& prevSegment = strandGroup.PeekStrandSegment(strandSegment.GetPrevHandle());
		p[1] = prevSegment.m_info.m_globalPosition;
		const auto& prevPrevSegment = strandGroup.PeekStrandSegment(prevSegment.GetPrevHandle());
		p[0] = prevPrevSegment.m_info.m_globalPosition;
	}
	if (strandSegmentHandle == strandSegmentHandles.back())
	{
		p[3] = p[2] * 2.0f - p[1];
	}
	else
	{
		const auto& nextSegment = strandGroup.PeekStrandSegment(strandSegment.GetNextHandle());
		p[3] = nextSegment.m_info.m_globalPosition;
	}
	glm::vec3 position, tangent;
	Strands::CubicInterpolation(p[0], p[1], p[2], p[3], position, tangent, a);
	return position;
}

glm::vec3 StrandModel::InterpolateStrandSegmentAxis(StrandSegmentHandle strandSegmentHandle, const float a) const
{
	assert(strandSegmentHandle >= 0);
	assert(a >= 0.f && a <= 1.f);
	const auto& strandGroup = m_strandModelSkeleton.m_data.m_strandGroup;
	assert(strandGroup.PeekStrandSegments().size() > strandSegmentHandle);
	const auto& strandSegment = strandGroup.PeekStrandSegment(strandSegmentHandle);
	const auto& strand = strandGroup.PeekStrand(strandSegment.GetStrandHandle());
	auto& baseInfo = strand.m_info.m_baseInfo;

	const auto& strandSegmentHandles = strand.PeekStrandSegmentHandles();

	glm::vec3 p[4];

	p[2] = strandSegment.m_info.m_globalPosition;
	if (strandSegmentHandle == strandSegmentHandles.front())
	{
		p[1] = baseInfo.m_globalPosition;
		p[0] = p[1] * 2.0f - p[2];
	}
	else if (strandSegmentHandle == strandSegmentHandles.at(1))
	{
		p[0] = baseInfo.m_globalPosition;
		p[1] = strandGroup.PeekStrandSegment(strandSegmentHandles.front()).m_info.m_globalPosition;
	}
	else
	{
		const auto& prevSegment = strandGroup.PeekStrandSegment(strandSegment.GetPrevHandle());
		p[1] = prevSegment.m_info.m_globalPosition;
		const auto& prevPrevSegment = strandGroup.PeekStrandSegment(prevSegment.GetPrevHandle());
		p[0] = prevPrevSegment.m_info.m_globalPosition;
	}
	if (strandSegmentHandle == strandSegmentHandles.back())
	{
		p[3] = p[2] * 2.0f - p[1];
	}
	else
	{
		const auto& nextSegment = strandGroup.PeekStrandSegment(strandSegment.GetNextHandle());
		p[3] = nextSegment.m_info.m_globalPosition;
	}
	glm::vec3 position, tangent;
	Strands::CubicInterpolation(p[0], p[1], p[2], p[3], position, tangent, a);
	return tangent;
}

float StrandModel::InterpolateStrandSegmentRadius(StrandSegmentHandle strandSegmentHandle, const float a) const
{
	assert(strandSegmentHandle >= 0);
	assert(a >= 0.f && a <= 1.f);
	const auto& strandGroup = m_strandModelSkeleton.m_data.m_strandGroup;
	assert(strandGroup.PeekStrandSegments().size() > strandSegmentHandle);
	const auto& strandSegment = strandGroup.PeekStrandSegment(strandSegmentHandle);
	const auto& strand = strandGroup.PeekStrand(strandSegment.GetStrandHandle());
	auto& baseInfo = strand.m_info.m_baseInfo;

	const auto& strandSegmentHandles = strand.PeekStrandSegmentHandles();

	float p[4];

	p[2] = strandSegment.m_info.m_thickness;
	if (strandSegmentHandle == strandSegmentHandles.front())
	{
		p[1] = baseInfo.m_thickness;
		p[0] = p[1] * 2.0f - p[2];
	}
	else if (strandSegmentHandle == strandSegmentHandles.at(1))
	{
		p[0] = baseInfo.m_thickness;
		p[1] = strandGroup.PeekStrandSegment(strandSegmentHandles.front()).m_info.m_thickness;
	}
	else
	{
		const auto& prevSegment = strandGroup.PeekStrandSegment(strandSegment.GetPrevHandle());
		p[1] = prevSegment.m_info.m_thickness;
		const auto& prevPrevSegment = strandGroup.PeekStrandSegment(prevSegment.GetPrevHandle());
		p[0] = prevPrevSegment.m_info.m_thickness;
	}
	if (strandSegmentHandle == strandSegmentHandles.back())
	{
		p[3] = p[2] * 2.0f - p[1];
	}
	else
	{
		const auto& nextSegment = strandGroup.PeekStrandSegment(strandSegment.GetNextHandle());
		p[3] = nextSegment.m_info.m_thickness;
	}
	float radius, tangent;
	Strands::CubicInterpolation(p[0], p[1], p[2], p[3], radius, tangent, a);
	return radius;
}
#pragma endregion