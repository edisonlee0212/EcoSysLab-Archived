#include "StrandModel.hpp"

#include <any>

using namespace EcoSysLab;


void StrandModel::ResetAllProfiles(const StrandModelParameters& strandModelParameters)
{
	m_strandModelSkeleton.m_data.m_strandGroup = {};
	const auto& sortedInternodeList = m_strandModelSkeleton.PeekSortedNodeList();
	for (const auto& internodeHandle : sortedInternodeList)
	{
		auto& internode = m_strandModelSkeleton.RefNode(internodeHandle);
		auto& profile = internode.m_data.m_profile;
		profile.m_settings = strandModelParameters.m_profilePhysicsSettings;
		profile.Reset(0.001f);
		internode.m_data.m_particleMap.clear();
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
		auto& profile = internode.m_data.m_profile;
		profile.m_settings = strandModelParameters.m_profilePhysicsSettings;
		if (internode.IsEndNode()) continue;
		profile.Reset(0.001f);
	}


	for (const auto& internodeHandle : sortedInternodeList)
	{
		auto& internode = m_strandModelSkeleton.RefNode(internodeHandle);
		auto& profile = internode.m_data.m_profile;

		std::vector<SkeletonNodeHandle> parentNodeToRootChain;
		SkeletonNodeHandle walker = internode.GetParentHandle();
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
					const auto newStrandSegmentHandle = strandGroup.Extend(strandHandle);
					auto& nodeOnChain = m_strandModelSkeleton.RefNode(*it);
					const auto newParticleHandle = nodeOnChain.m_data.m_profile.AllocateParticle();
					auto& newParticle = nodeOnChain.m_data.m_profile.RefParticle(newParticleHandle);
					newParticle.m_strandHandle = strandHandle;
					newParticle.m_strandSegmentHandle = newStrandSegmentHandle;
					newParticle.m_base = false;

					auto& newSegment = strandGroup.RefStrandSegment(newStrandSegmentHandle);
					newSegment.m_data.m_nodeHandle = *it;
					newSegment.m_data.m_profileParticleHandle = newParticleHandle;
				}
				constexpr auto position = glm::vec3(0.0f);
				const auto newStrandSegmentHandle = strandGroup.Extend(strandHandle);
				const auto newParticleHandle = profile.AllocateParticle();
				auto& newParticle = profile.RefParticle(newParticleHandle);
				newParticle.m_strandHandle = strandHandle;
				newParticle.m_strandSegmentHandle = newStrandSegmentHandle;
				newParticle.m_base = true;
				newParticle.SetPosition(position);
				newParticle.SetInitialPosition(position);
				auto& newSegment = strandGroup.RefStrandSegment(newStrandSegmentHandle);
				newSegment.m_data.m_nodeHandle = internodeHandle;
				newSegment.m_data.m_profileParticleHandle = newParticleHandle;
			}
		}
		else {
			if (profile.RefParticles().empty()) {
				for (int i = 0; i < strandModelParameters.m_endNodeStrands; i++) {
					const auto strandHandle = strandGroup.AllocateStrand();
					for (auto it = parentNodeToRootChain.rbegin(); it != parentNodeToRootChain.rend(); ++it) {
						const auto newStrandSegmentHandle = strandGroup.Extend(strandHandle);
						auto& nodeOnChain = m_strandModelSkeleton.RefNode(*it);
						const auto newParticleHandle = nodeOnChain.m_data.m_profile.AllocateParticle();
						auto& newParticle = nodeOnChain.m_data.m_profile.RefParticle(newParticleHandle);
						newParticle.m_strandHandle = strandHandle;
						newParticle.m_strandSegmentHandle = newStrandSegmentHandle;
						newParticle.m_base = false;
						auto& newSegment = strandGroup.RefStrandSegment(newStrandSegmentHandle);
						newSegment.m_data.m_nodeHandle = *it;
						newSegment.m_data.m_profileParticleHandle = newParticleHandle;
					}
					const auto position = glm::diskRand(glm::sqrt(static_cast<float>(strandModelParameters.m_endNodeStrands)));
					const auto newStrandSegmentHandle = strandGroup.Extend(strandHandle);
					const auto newParticleHandle = profile.AllocateParticle();
					auto& newParticle = profile.RefParticle(newParticleHandle);
					newParticle.m_strandHandle = strandHandle;
					newParticle.m_strandSegmentHandle = newStrandSegmentHandle;
					newParticle.m_base = true;
					newParticle.SetPosition(position);
					newParticle.SetInitialPosition(position);
					auto& newSegment = strandGroup.RefStrandSegment(newStrandSegmentHandle);
					newSegment.m_data.m_nodeHandle = internodeHandle;
					newSegment.m_data.m_profileParticleHandle = newParticleHandle;
				}
			}
			else
			{
				for (ParticleHandle particleHandle = 0; particleHandle < profile.RefParticles().size(); particleHandle++)
				{
					const auto strandHandle = strandGroup.AllocateStrand();
					for (auto it = parentNodeToRootChain.rbegin(); it != parentNodeToRootChain.rend(); ++it) {
						const auto newStrandSegmentHandle = strandGroup.Extend(strandHandle);
						auto& nodeOnChain = m_strandModelSkeleton.RefNode(*it);
						const auto newParticleHandle = nodeOnChain.m_data.m_profile.AllocateParticle();
						auto& newParticle = nodeOnChain.m_data.m_profile.RefParticle(newParticleHandle);
						newParticle.m_strandHandle = strandHandle;
						newParticle.m_strandSegmentHandle = newStrandSegmentHandle;
						newParticle.m_base = false;
						auto& newSegment = strandGroup.RefStrandSegment(newStrandSegmentHandle);
						newSegment.m_data.m_nodeHandle = *it;
						newSegment.m_data.m_profileParticleHandle = newParticleHandle;
					}
					const auto newStrandSegmentHandle = strandGroup.Extend(strandHandle);
					auto& particle2D = profile.RefParticle(particleHandle);
					particle2D.m_strandHandle = strandHandle;
					particle2D.m_strandSegmentHandle = newStrandSegmentHandle;
					particle2D.m_base = true;
					particle2D.SetPosition(particle2D.GetInitialPosition());
					auto& newSegment = strandGroup.RefStrandSegment(newStrandSegmentHandle);
					newSegment.m_data.m_nodeHandle = internodeHandle;
					newSegment.m_data.m_profileParticleHandle = particleHandle;
				}
			}
		}
	}

	for (const auto& internodeHandle : sortedInternodeList)
	{
		auto& internode = m_strandModelSkeleton.RefNode(internodeHandle);
		internode.m_data.m_particleMap.clear();
		auto& profile = internode.m_data.m_profile;
		for (auto& particle : profile.RefParticles())
		{
			internode.m_data.m_particleMap.insert({ particle.m_strandHandle, particle.GetHandle() });
		}
		internode.m_data.m_strandCount = profile.RefParticles().size();
	}
	m_strandModelSkeleton.m_data.m_numOfParticles = 0;
	for (const auto& internodeHandle : sortedInternodeList)
	{
		auto& internode = m_strandModelSkeleton.RefNode(internodeHandle);
		int maxChildSize = 0;
		SkeletonNodeHandle maxChildHandle = -1;
		for (const auto& childHandle : internode.PeekChildHandles())
		{
			auto& childInternode = m_strandModelSkeleton.RefNode(childHandle);
			const auto childSize = static_cast<float>(childInternode.m_data.m_particleMap.size());
			if (childSize > maxChildSize)
			{
				maxChildSize = childSize;
				maxChildHandle = childHandle;
			}
		}
		for (const auto& childHandle : internode.PeekChildHandles())
		{
			auto& childInternode = m_strandModelSkeleton.RefNode(childHandle);
			if (childHandle == maxChildHandle) childInternode.m_data.m_split = false;
			const auto childSize = static_cast<float>(childInternode.m_data.m_particleMap.size());
			if (childSize > maxChildSize * strandModelParameters.m_overlapThreshold)
			{
				childInternode.m_data.m_split = true;
			}
			else
			{
				childInternode.m_data.m_split = false;
			}
		}
		m_strandModelSkeleton.m_data.m_numOfParticles += internode.m_data.m_profile.PeekParticles().size();
	}
}

void StrandModel::CalculateProfiles(const StrandModelParameters& strandModelParameters)
{
	
	const auto& sortedInternodeList = m_strandModelSkeleton.PeekSortedNodeList();
	if (sortedInternodeList.empty()) return;

	Jobs::ParallelFor(sortedInternodeList.size(), [&](unsigned i)
		{
			auto& internode = m_strandModelSkeleton.RefNode(sortedInternodeList[i]);
			for (auto& particle : internode.m_data.m_profile.RefParticles())
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
			m_strandModelSkeleton.m_data.m_numOfParticles += m_strandModelSkeleton.RefNode(*it).m_data.m_profile.PeekParticles().size();
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
			m_strandModelSkeleton.m_data.m_numOfParticles += m_strandModelSkeleton.RefNode(*it).m_data.m_profile.PeekParticles().size();
		}
	}
	
}

void StrandModel::CalculateProfile(const float maxRootDistance, const SkeletonNodeHandle nodeHandle, const StrandModelParameters& strandModelParameters, bool scheduling)
{
	if (scheduling) {
		m_strandModelSkeleton.RefNode(nodeHandle).m_data.m_tasks.emplace_back(Jobs::AddTask([&, nodeHandle, scheduling](unsigned threadIndex) {
			MergeTask(maxRootDistance, nodeHandle, strandModelParameters);
			auto& internode = m_strandModelSkeleton.RefNode(nodeHandle);
			if (internode.m_data.m_profile.PeekParticles().size() > 1) {
				PackTask(nodeHandle, strandModelParameters, !scheduling);
				if (internode.PeekChildHandles().empty()) CopyFrontToBackTask(nodeHandle);
			}
			internode.m_data.m_profile.CalculateBoundaries(true, strandModelParameters.m_boundaryPointDistance);
			}
		)
		);
	}
	else
	{
		MergeTask(maxRootDistance, nodeHandle, strandModelParameters);
		auto& internode = m_strandModelSkeleton.RefNode(nodeHandle);
		if (internode.m_data.m_profile.PeekParticles().size() > 1) {
			PackTask(nodeHandle, strandModelParameters, !scheduling);
			if (internode.PeekChildHandles().empty()) CopyFrontToBackTask(nodeHandle);
		}
		internode.m_data.m_profile.CalculateBoundaries(true, strandModelParameters.m_boundaryPointDistance);
	}
}

void StrandModel::Wait(const SkeletonNodeHandle nodeHandle)
{
	auto& internode = m_strandModelSkeleton.RefNode(nodeHandle);
	if (internode.m_data.m_tasks.empty()) return;
	for (const auto& i : internode.m_data.m_tasks)
	{
		i.wait();
	}
	internode.m_data.m_tasks.clear();
}

void StrandModel::PackTask(const SkeletonNodeHandle nodeHandle, const StrandModelParameters& strandModelParameters, const bool parallel)
{
	auto& internode = m_strandModelSkeleton.RefNode(nodeHandle);
	auto& internodeData = internode.m_data;
	internodeData.m_profile.m_parallel = parallel;

	const auto iterations = internodeData.m_packingIteration;

	int timeout = strandModelParameters.m_junctionProfilePackingMaxIteration;
	if (!internodeData.m_profileConstraints.m_boundaries.empty()) timeout = strandModelParameters.m_modifiedProfilePackingMaxIteration;
	for (int i = 0; i < iterations; i++) {
		internodeData.m_profile.Simulate(1,
			[&](auto& grid, bool gridResized)
			{
				if (gridResized || internodeData.m_boundariesUpdated) grid.ApplyBoundaries(internodeData.m_profileConstraints);
				internodeData.m_boundariesUpdated = false;
			},
			[&](auto& particle)
			{
				auto acceleration = glm::vec2(0.f);
				if (!internodeData.m_profile.m_particleGrid2D.PeekCells().empty()) {
					const auto& cell = internodeData.m_profile.m_particleGrid2D.RefCell(particle.GetPosition());
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

void StrandModel::MergeTask(float maxRootDistance, SkeletonNodeHandle nodeHandle, const StrandModelParameters& strandModelParameters)
{
	auto& internode = m_strandModelSkeleton.RefNode(nodeHandle);
	auto& internodeData = internode.m_data;
	internodeData.m_twistAngle = 0.0f;
	const auto& childHandles = internode.PeekChildHandles();
	int maxChildSize = -1;
	SkeletonNodeHandle maxChildHandle = -1;
	for (const auto& childHandle : childHandles) {
		Wait(childHandle);
		auto& childInternode = m_strandModelSkeleton.RefNode(childHandle);
		const auto childSize = static_cast<float>(childInternode.m_data.m_particleMap.size());
		if (childSize > maxChildSize)
		{
			maxChildSize = childSize;
			maxChildHandle = childHandle;
		}
	}

	internodeData.m_centerDirectionRadius = 0.0f;
	if (!internodeData.m_profileConstraints.m_boundaries.empty())
	{
		internodeData.m_packingIteration = glm::min(strandModelParameters.m_modifiedProfilePackingMaxIteration, static_cast<int>(internodeData.m_profile.RefParticles().size()) * strandModelParameters.m_maxSimulationIterationCellFactor);
	}
	if (childHandles.empty())
	{
		if (internodeData.m_profileConstraints.m_boundaries.empty()) internode.m_data.m_packingIteration = glm::min(strandModelParameters.m_junctionProfilePackingMaxIteration, static_cast<int>(internodeData.m_profile.RefParticles().size()) * strandModelParameters.m_maxSimulationIterationCellFactor);
		int particleIndex = 0;
		for (const auto& particle : internodeData.m_profile.RefParticles())
		{
			const auto nodeParticleHandle = internodeData.m_particleMap.at(particle.m_strandHandle);
			auto& nodeParticle = internodeData.m_profile.RefParticle(nodeParticleHandle);
			particleIndex++;
			nodeParticle.SetColor(particle.GetColor());
			nodeParticle.m_mainChild = true;
			nodeParticle.m_correspondingChildNodeHandle = -1;
		}
		return;
	}

	if (maxChildHandle == -1) maxChildHandle = childHandles.front();
	auto& mainChildNode = m_strandModelSkeleton.RefNode(maxChildHandle);
	const auto& mainChildPhysics2D = mainChildNode.m_data.m_profile;
	for (const auto& childHandle : childHandles)
	{
		auto& childNode = m_strandModelSkeleton.RefNode(childHandle);
		const auto childNodeFront = glm::inverse(internode.m_info.m_regulatedGlobalRotation) * childNode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
		auto direction = glm::normalize(glm::vec2(childNodeFront.x, childNodeFront.y));
		const auto mainChildRadius = mainChildPhysics2D.GetDistanceToOrigin(direction, glm::vec2(0.0f));
		auto offset = glm::vec2(0.0f);
		offset = (mainChildRadius - childNode.m_data.m_centerDirectionRadius + 2.0f) * direction;
		childNode.m_data.m_offset = offset;
	}
	const auto branchTwistAngle = strandModelParameters.m_branchTwistDistribution.GetValue(internode.m_info.m_rootDistance / maxRootDistance);
	const auto junctionTwistAngle = strandModelParameters.m_junctionTwistDistribution.GetValue(internode.m_info.m_rootDistance / maxRootDistance);
	if (childHandles.size() == 1)
	{
		//Copy from child flow start to self flow start
		auto& childNode = m_strandModelSkeleton.RefNode(childHandles.front());
		const auto& childPhysics2D = childNode.m_data.m_profile;
		childNode.m_data.m_twistAngle = branchTwistAngle;

		for (const auto& childParticle : childPhysics2D.PeekParticles())
		{
			const auto nodeParticleHandle = internodeData.m_particleMap.at(childParticle.m_strandHandle);
			auto& nodeParticle = internodeData.m_profile.RefParticle(nodeParticleHandle);
			nodeParticle.SetColor(childParticle.GetColor());
			auto polarPosition = childParticle.GetPolarPosition();
			polarPosition.y += glm::radians(branchTwistAngle);
			nodeParticle.SetPolarPosition(polarPosition);
			nodeParticle.SetInitialPosition(nodeParticle.GetPosition());
			nodeParticle.m_mainChild = true;
			nodeParticle.m_correspondingChildNodeHandle = childHandles.front();
		}
		if (internodeData.m_profileConstraints.m_boundaries.empty()) internode.m_data.m_packingIteration = glm::min(strandModelParameters.m_branchProfilePackingMaxIteration, static_cast<int>(internodeData.m_profile.RefParticles().size()) * strandModelParameters.m_maxSimulationIterationCellFactor);
		return;
	}
	if (internodeData.m_profileConstraints.m_boundaries.empty()) internode.m_data.m_packingIteration = glm::min(strandModelParameters.m_junctionProfilePackingMaxIteration, static_cast<int>(internodeData.m_profile.RefParticles().size()) * strandModelParameters.m_maxSimulationIterationCellFactor);

	mainChildNode.m_data.m_twistAngle = junctionTwistAngle;
	for (const auto& mainChildParticle : mainChildPhysics2D.PeekParticles())
	{
		const auto nodeParticleHandle = internodeData.m_particleMap.at(mainChildParticle.m_strandHandle);
		auto& nodeParticle = internodeData.m_profile.RefParticle(nodeParticleHandle);
		nodeParticle.SetColor(mainChildParticle.GetColor());
		auto polarPosition = mainChildParticle.GetPolarPosition();
		polarPosition.y += glm::radians(junctionTwistAngle);
		nodeParticle.SetPolarPosition(polarPosition);
		nodeParticle.SetInitialPosition(nodeParticle.GetPosition());

		nodeParticle.m_mainChild = true;
		nodeParticle.m_correspondingChildNodeHandle = maxChildHandle;
	}


	if (strandModelParameters.m_preMerge) {
		bool needSimulation = false;
		for (const auto& childHandle : childHandles)
		{
			if (childHandle == maxChildHandle) continue;
			auto& childNode = m_strandModelSkeleton.RefNode(childHandle);
			auto& childPhysics2D = childNode.m_data.m_profile;
			if (!childNode.m_data.m_split)
			{
				needSimulation = true;
				const auto childNodeFront = glm::inverse(internode.m_info.m_regulatedGlobalRotation) * childNode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
				auto direction = glm::normalize(glm::vec2(childNodeFront.x, childNodeFront.y));
				if (glm::isnan(direction.x) || glm::isnan(direction.y))
				{
					direction = glm::vec2(1, 0);
				}
				childNode.m_data.m_centerDirectionRadius = childPhysics2D.GetDistanceToOrigin(-direction, glm::vec2(0.0f));

				for (auto& childParticle : childPhysics2D.RefParticles())
				{
					childParticle.SetPosition(childParticle.GetPosition() + childNode.m_data.m_offset);

					const auto nodeParticleHandle = internodeData.m_particleMap.at(childParticle.m_strandHandle);
					auto& nodeParticle = internodeData.m_profile.RefParticle(nodeParticleHandle);
					nodeParticle.SetColor(childParticle.GetColor());
					auto polarPosition = childParticle.GetPolarPosition();
					polarPosition.y += glm::radians(junctionTwistAngle);
					nodeParticle.SetPolarPosition(polarPosition);
					nodeParticle.SetPolarPosition(nodeParticle.GetPosition());
					nodeParticle.m_enable = true;
					nodeParticle.m_mainChild = false;
					nodeParticle.m_correspondingChildNodeHandle = childHandle;
				}
			}
			else
			{
				for (auto& childParticle : childPhysics2D.RefParticles())
				{
					const auto nodeParticleHandle = internodeData.m_particleMap.at(childParticle.m_strandHandle);
					auto& nodeParticle = internodeData.m_profile.RefParticle(nodeParticleHandle);
					nodeParticle.SetColor(childParticle.GetColor());
					auto polarPosition = childParticle.GetPolarPosition();
					polarPosition.y += glm::radians(junctionTwistAngle);
					nodeParticle.SetPolarPosition(polarPosition);
					nodeParticle.SetPolarPosition(nodeParticle.GetPosition());
					nodeParticle.m_enable = false;
					nodeParticle.m_mainChild = false;
					nodeParticle.m_correspondingChildNodeHandle = childHandle;
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
				auto& childPhysics2D = childNode.m_data.m_profile;
				const auto childNodeFront = glm::inverse(internode.m_info.m_regulatedGlobalRotation) * childNode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
				auto direction = glm::normalize(glm::vec2(childNodeFront.x, childNodeFront.y));
				if (glm::isnan(direction.x) || glm::isnan(direction.y))
				{
					direction = glm::vec2(1, 0);
				}
				childNode.m_data.m_centerDirectionRadius = childPhysics2D.GetDistanceToOrigin(-direction, glm::vec2(0.0f));
				const auto centerRadius = internodeData.m_profile.GetDistanceToOrigin(direction, glm::vec2(0.0f));
				auto offset = glm::vec2(0.0f);
				offset = (centerRadius + childNode.m_data.m_centerDirectionRadius + 2.0f) * direction;
				childNode.m_data.m_offset = offset;
				for (auto& childParticle : childPhysics2D.RefParticles())
				{
					auto polarPosition = childParticle.GetPolarPosition();
					polarPosition.y += glm::radians(junctionTwistAngle);
					childParticle.SetPolarPosition(polarPosition);
					childParticle.SetPosition(childParticle.GetPosition() + offset);

					const auto nodeParticleHandle = internodeData.m_particleMap.at(childParticle.m_strandHandle);
					auto& nodeParticle = internodeData.m_profile.RefParticle(nodeParticleHandle);
					nodeParticle.SetColor(childParticle.GetColor());
					nodeParticle.SetPosition(childParticle.GetPosition());
					nodeParticle.SetInitialPosition(childParticle.GetPosition());
					nodeParticle.m_mainChild = false;
					nodeParticle.m_correspondingChildNodeHandle = childHandle;
				}
			}
		}
		CopyFrontToBackTask(nodeHandle);
		internodeData.m_profile.SetEnableAllParticles(true);
	}
	else {
		for (const auto& childHandle : childHandles)
		{
			if (childHandle == maxChildHandle) continue;
			auto& childNode = m_strandModelSkeleton.RefNode(childHandle);
			childNode.m_data.m_twistAngle = junctionTwistAngle;
			auto& childPhysics2D = childNode.m_data.m_profile;
			const auto childNodeFront = glm::inverse(internode.m_info.m_regulatedGlobalRotation) * childNode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
			auto direction = glm::normalize(glm::vec2(childNodeFront.x, childNodeFront.y));
			if (glm::isnan(direction.x) || glm::isnan(direction.y))
			{
				direction = glm::vec2(1, 0);
			}
			childNode.m_data.m_centerDirectionRadius = childPhysics2D.GetDistanceToOrigin(-direction, glm::vec2(0.0f));
			const auto mainChildRadius = mainChildPhysics2D.GetDistanceToOrigin(direction, glm::vec2(0.0f));
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

				const auto nodeParticleHandle = internodeData.m_particleMap.at(childParticle.m_strandHandle);
				auto& nodeParticle = internodeData.m_profile.RefParticle(nodeParticleHandle);
				nodeParticle.SetColor(childParticle.GetColor());
				auto polarPosition = childParticle.GetPolarPosition();
				polarPosition.y += glm::radians(strandModelParameters.m_junctionTwistDistribution.GetValue(internode.m_info.m_rootDistance / maxRootDistance));
				nodeParticle.SetPolarPosition(polarPosition);
				nodeParticle.SetInitialPosition(nodeParticle.GetPosition());
				nodeParticle.m_mainChild = false;
				nodeParticle.m_correspondingChildNodeHandle = childHandle;
			}
		}
	}
}

void StrandModel::CopyFrontToBackTask(const SkeletonNodeHandle nodeHandle)
{
	auto& internode = m_strandModelSkeleton.RefNode(nodeHandle);
	auto& internodeData = internode.m_data;
	for (int i = 0; i < internodeData.m_profile.RefParticles().size(); i++)
	{
		internodeData.m_profile.RefParticle(i).SetInitialPosition(internodeData.m_profile.RefParticle(i).GetPosition());
	}
}

void StrandModel::ApplyProfile(const StrandModelParameters& strandModelParameters,
	const SkeletonNodeHandle nodeHandle)
{
	auto& node = m_strandModelSkeleton.RefNode(nodeHandle);
	const auto currentFront = node.m_info.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
	const auto currentUp = node.m_info.m_regulatedGlobalRotation * glm::vec3(0, 1, 0);
	const auto currentLeft = node.m_info.m_regulatedGlobalRotation * glm::vec3(1, 0, 0);
	const auto& parameters = strandModelParameters;
	const bool wound = node.IsEndNode();
	for (const auto& [strandHandle, particleHandle] : node.m_data.m_particleMap)
	{
		const auto& particle = node.m_data.m_profile.PeekParticle(particleHandle);
		auto& newStrandSegment = m_strandModelSkeleton.m_data.m_strandGroup.RefStrandSegment(particle.m_strandSegmentHandle);
		newStrandSegment.m_info.m_thickness = node.m_data.m_strandRadius;
		newStrandSegment.m_info.m_globalPosition = node.m_info.GetGlobalEndPosition()
			+ node.m_data.m_strandRadius * particle.GetInitialPosition().x * currentLeft
			+ node.m_data.m_strandRadius * particle.GetInitialPosition().y * currentUp;
		if (glm::any(glm::isnan(newStrandSegment.m_info.m_globalPosition)))
		{
			EVOENGINE_ERROR("Nan!");
		}
		if (wound)
		{
			newStrandSegment.m_info.m_globalPosition += currentFront * glm::max(0.0f, strandModelParameters.m_cladoptosisDistribution.GetValue(glm::max(0.0f, (strandModelParameters.m_cladoptosisRange - particle.GetDistanceToBoundary()) / strandModelParameters.m_cladoptosisRange)));
		}
		newStrandSegment.m_info.m_color = particle.IsBoundary() ? parameters.m_boundaryPointColor : parameters.m_contentPointColor;
		newStrandSegment.m_info.m_isBoundary = particle.IsBoundary();
	}
}

void StrandModel::ApplyProfiles(const StrandModelParameters& strandModelParameters)
{
	auto& strandGroup = m_strandModelSkeleton.m_data.m_strandGroup;
	const auto& sortedInternodeList = m_strandModelSkeleton.PeekSortedNodeList();
	for (const auto& nodeHandle : sortedInternodeList)
	{
		const auto& node = m_strandModelSkeleton.RefNode(nodeHandle);
		if (node.GetParentHandle() == -1)
		{
			const auto parentGlobalRotation = node.m_info.m_regulatedGlobalRotation;

			const auto currentUp = parentGlobalRotation * glm::vec3(0, 1, 0);
			const auto currentLeft = parentGlobalRotation * glm::vec3(1, 0, 0);
			const auto baseRadius = strandModelParameters.m_strandRadiusDistribution.GetValue(0.0f);
			for (const auto& [strandHandle, particleHandle] : node.m_data.m_particleMap)
			{
				const auto& particle = node.m_data.m_profile.PeekParticle(particleHandle);
				auto& strand = strandGroup.RefStrand(strandHandle);
				strand.m_info.m_baseInfo.m_thickness = baseRadius;
				strand.m_info.m_baseInfo.m_globalPosition =
					strand.m_info.m_baseInfo.m_thickness * particle.GetPosition().x * currentLeft
					+ strand.m_info.m_baseInfo.m_thickness * particle.GetPosition().y * currentUp;
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
		if (parentHandle != -1) break;
		node.m_info.m_globalPosition = node.m_info.m_globalPosition;
		node.m_info.m_globalRotation = node.m_info.m_regulatedGlobalRotation;
		maxRootDistance = glm::max(maxRootDistance, node.m_info.m_endDistance + node.m_info.m_length);
	}

	for (const auto& nodeHandle : sortedInternodeList)
	{
		auto& node = m_strandModelSkeleton.RefNode(nodeHandle);
		const auto parentHandle = node.GetParentHandle();
		if (parentHandle == -1)
		{
			node.m_info.m_globalPosition = node.m_info.m_globalPosition;
			node.m_info.m_globalRotation = node.m_info.m_regulatedGlobalRotation;
			node.m_data.m_strandRadius = strandModelParameters.m_strandRadiusDistribution.GetValue(node.m_info.m_rootDistance / maxRootDistance);
			continue;
		}
		const auto& parentNode = m_strandModelSkeleton.PeekNode(parentHandle);
		node.m_info.m_globalPosition = parentNode.m_info.GetGlobalEndPosition();
		glm::quat parentGlobalRotation = parentNode.m_info.m_globalRotation;
		node.m_data.m_strandRadius = strandModelParameters.m_strandRadiusDistribution.GetValue(node.m_info.m_rootDistance / maxRootDistance);
		const auto globalDirection = node.m_info.GetGlobalDirection();
		auto localPosition = globalDirection * node.m_info.m_length;
		auto newGlobalEndPosition = node.m_info.m_globalPosition;
		node.m_info.m_globalRotation = parentGlobalRotation * (glm::inverse(parentNode.m_info.m_regulatedGlobalRotation) * node.m_info.m_regulatedGlobalRotation);

		const auto parentUp = parentGlobalRotation * glm::vec3(0, 1, 0);
		const auto parentLeft = parentGlobalRotation * glm::vec3(1, 0, 0);
		const auto parentFront = parentGlobalRotation * glm::vec3(0, 0, -1);

		const auto front = node.m_info.m_globalRotation * glm::vec3(0, 0, -1);
		const float offsetLength = glm::length(node.m_data.m_offset);
		const float cosFront = glm::dot(front, parentFront); //Horizontal
		const float sinFront = glm::sin(glm::acos(glm::clamp(cosFront, -1.0f, 1.0f))); //Vertical

		glm::vec2 parentCenter = glm::vec2(0.0f);
		int particleCount = 0;

		for (const auto& parentParticle : parentNode.m_data.m_profile.PeekParticles())
		{
			if (parentParticle.m_correspondingChildNodeHandle == nodeHandle)
			{
				parentCenter += parentParticle.GetPosition();
				particleCount++;
			}
		}
		if (particleCount > 0) {
			parentCenter /= particleCount;
		}
		glm::vec3 sideShift = glm::vec3(0.f);
		if (node.IsApical() && strandModelParameters.m_sidePushFactor > 0.f) {

			sideShift += parentUp * parentCenter.y * strandModelParameters.m_sidePushFactor * node.m_data.m_strandRadius;
			sideShift += parentLeft * parentCenter.x * strandModelParameters.m_sidePushFactor * node.m_data.m_strandRadius;
		}else if(!node.IsApical() && strandModelParameters.m_apicalSidePushFactor > 0.f){
			sideShift += parentUp * parentCenter.y * strandModelParameters.m_apicalSidePushFactor * node.m_data.m_strandRadius;
			sideShift += parentLeft * parentCenter.x * strandModelParameters.m_apicalSidePushFactor * node.m_data.m_strandRadius;
		}
		glm::vec3 rotationShift = glm::vec3(0.f);
		if (offsetLength > glm::epsilon<float>()) {
			const auto offsetDirection = glm::normalize(node.m_data.m_offset);
			float maxRadius = node.m_data.m_profile.GetMaxDistanceToCenter();
			for (const auto& parentParticle : parentNode.m_data.m_profile.PeekParticles())
			{
				if (parentParticle.m_correspondingChildNodeHandle == nodeHandle)
				{
					const auto distance = glm::length(glm::closestPointOnLine(parentParticle.GetPosition(), parentCenter, parentCenter + node.m_data.m_offset * 1000.0f));
					maxRadius = glm::max(maxRadius, distance);
				}
			}
			for (const auto& particle : parentNode.m_data.m_profile.PeekParticles())
			{
				const auto distance = glm::length(glm::closestPointOnLine(particle.GetPosition(), glm::vec2(0.f), node.m_data.m_offset * 1000.0f));
				maxRadius = glm::max(maxRadius, distance);
			}
			
			if (node.IsApical())
			{
				rotationShift += parentUp * offsetDirection.y * cosFront * maxRadius * strandModelParameters.m_apicalBranchRotationPushFactor * node.m_data.m_strandRadius;
				rotationShift += parentLeft * offsetDirection.x * cosFront * maxRadius * strandModelParameters.m_apicalBranchRotationPushFactor * node.m_data.m_strandRadius;
				rotationShift += parentFront * sinFront * maxRadius * strandModelParameters.m_apicalBranchRotationPushFactor * node.m_data.m_strandRadius;
			}
			else
			{

				rotationShift += parentUp * offsetDirection.y * cosFront * maxRadius * strandModelParameters.m_rotationPushFactor * node.m_data.m_strandRadius;
				rotationShift += parentLeft * offsetDirection.x * cosFront * maxRadius * strandModelParameters.m_rotationPushFactor * node.m_data.m_strandRadius;
				rotationShift += parentFront * sinFront * maxRadius * strandModelParameters.m_rotationPushFactor * node.m_data.m_strandRadius;
			}
		}
		if (!node.IsApical()) {
			const auto projectedShift = globalDirection * glm::dot(rotationShift + sideShift, globalDirection);
			if (glm::length(projectedShift) > glm::length(localPosition))
			{
				newGlobalEndPosition += projectedShift;
			}
			else
			{
				newGlobalEndPosition += localPosition;
			}
		}else
		{
			newGlobalEndPosition += localPosition;
		}
		assert(!glm::any(glm::isnan(node.m_info.m_globalPosition)));
		assert(!glm::any(glm::isnan(newGlobalEndPosition)));

		const auto diff = newGlobalEndPosition - node.m_info.m_globalPosition;
		node.m_info.m_length = glm::length(diff);
	}
	m_strandModelSkeleton.CalculateRegulatedGlobalRotation();
	m_strandModelSkeleton.CalculateDistance();
	for (const auto& nodeHandle : sortedInternodeList)
	{
		auto& node = m_strandModelSkeleton.RefNode(nodeHandle);
		node.m_info.m_globalRotation = node.m_info.m_regulatedGlobalRotation;
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

glm::vec3 StrandModel::InterpolateStrandSegmentPosition(const StrandSegmentHandle strandSegmentHandle, float a) const
{
	assert(strandSegmentHandle >= 0);
	assert(a >= 0.f && a <= 1.f);
	a = glm::clamp(a, 0.0f, 1.0f);
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
	if (glm::any(glm::isnan(position)))
	{
		EVOENGINE_ERROR("nan");
	}
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