//
// Created by lllll on 10/21/2022.
//

#include "TreeModel.hpp"

using namespace EcoSysLab;
void ReproductiveModule::Reset()
{
	m_maturity = 0.0f;
	m_health = 1.0f;
	m_transform = glm::mat4(0.0f);
}

void TreeModel::ResetReproductiveModule()
{
	const auto& sortedInternodeList = m_shootSkeleton.RefSortedNodeList();
	for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); ++it) {
		auto& internode = m_shootSkeleton.RefNode(*it);
		auto& internodeData = internode.m_data;
		auto& buds = internodeData.m_buds;
		for (auto& bud : buds)
		{
			if (bud.m_status == BudStatus::Removed) continue;
			if (bud.m_type == BudType::Fruit || bud.m_type == BudType::Leaf)
			{
				bud.m_status = BudStatus::Dormant;
				bud.m_reproductiveModule.Reset();
			}
		}
	}
	m_fruitCount = m_leafCount = 0;
}

void TreeModel::RegisterVoxel(const glm::mat4& globalTransform, ClimateModel& climateModel, const ShootGrowthController& shootGrowthController)
{
	const auto& sortedInternodeList = m_shootSkeleton.RefSortedNodeList();
	auto& environmentGrid = climateModel.m_environmentGrid;
	for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); ++it) {
		const auto& internode = m_shootSkeleton.RefNode(*it);
		const auto& internodeInfo = internode.m_info;
		const float shadowSize = shootGrowthController.m_internodeShadowFactor;
		const float biomass = internodeInfo.m_thickness;
		const glm::vec3 worldPosition = globalTransform * glm::vec4(internodeInfo.m_globalPosition, 1.0f);
		environmentGrid.AddShadowValue(worldPosition, shadowSize);
		environmentGrid.AddBiomass(worldPosition, biomass);
		if (internode.IsEndNode()) {
			InternodeVoxelRegistration registration;
			registration.m_position = worldPosition;
			registration.m_nodeHandle = *it;
			registration.m_treeModelIndex = m_index;
			registration.m_thickness = internodeInfo.m_thickness;
			environmentGrid.AddNode(registration);
		}
	}
}

void TreeModel::PruneInternode(NodeHandle internodeHandle)
{
	auto& internode = m_shootSkeleton.RefNode(internodeHandle);

	m_shootSkeleton.RecycleNode(internodeHandle,
		[&](FlowHandle flowHandle) {},
		[&](NodeHandle nodeHandle)
		{
			const auto& node = m_shootSkeleton.RefNode(nodeHandle);
			const auto& physics2D = node.m_data.m_frontProfile;
			for (const auto& particle : physics2D.PeekParticles())
			{
				if (!m_shootSkeleton.m_data.m_pipeGroup.PeekPipeSegment(particle.m_data.m_pipeSegmentHandle).IsRecycled()) m_shootSkeleton.m_data.m_pipeGroup.RecyclePipeSegment(particle.m_data.m_pipeSegmentHandle);
			}
		});
}

void TreeModel::HarvestFruits(const std::function<bool(const ReproductiveModule& fruit)>& harvestFunction)
{
	const auto& sortedInternodeList = m_shootSkeleton.RefSortedNodeList();
	m_fruitCount = 0;

	for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); ++it) {
		auto& internode = m_shootSkeleton.RefNode(*it);
		auto& internodeData = internode.m_data;
		auto& buds = internodeData.m_buds;
		for (auto& bud : buds)
		{
			if (bud.m_type != BudType::Fruit || bud.m_status != BudStatus::Died) continue;

			if (harvestFunction(bud.m_reproductiveModule)) {
				bud.m_reproductiveModule.Reset();
			}
			else if (bud.m_reproductiveModule.m_maturity > 0) m_fruitCount++;

		}
	}
}

void TreeModel::ApplyTropism(const glm::vec3& targetDir, float tropism, glm::vec3& front, glm::vec3& up) {
	const glm::vec3 dir = glm::normalize(targetDir);
	const float dotP = glm::abs(glm::dot(front, dir));
	if (dotP < 0.99f && dotP > -0.99f) {
		const glm::vec3 left = glm::cross(front, dir);
		const float maxAngle = glm::acos(dotP);
		const float rotateAngle = maxAngle * tropism;
		front = glm::normalize(
			glm::rotate(front, glm::min(maxAngle, rotateAngle), left));
		up = glm::normalize(glm::cross(glm::cross(front, up), front));
	}
}

void TreeModel::ApplyTropism(const glm::vec3& targetDir, float tropism, glm::quat& rotation) {
	auto front = rotation * glm::vec3(0, 0, -1);
	auto up = rotation * glm::vec3(0, 1, 0);
	ApplyTropism(targetDir, tropism, front, up);
	rotation = glm::quatLookAt(front, up);
}

bool TreeModel::Grow(const float deltaTime, const NodeHandle baseInternodeHandle, const glm::mat4& globalTransform, ClimateModel& climateModel,
	const ShootGrowthController& shootGrowthController,
	const bool pruning, const float overrideGrowthRate)
{
	m_currentDeltaTime = deltaTime;
	m_age += m_currentDeltaTime;
	bool treeStructureChanged = false;
	if (!m_initialized) {
		Initialize(shootGrowthController);
		treeStructureChanged = true;
	}
	if (m_shootSkeleton.RefRawNodes().size() <= baseInternodeHandle) return false;

	m_shootSkeleton.SortLists();
	CalculateShootFlux(globalTransform, climateModel, shootGrowthController);
	SampleTemperature(globalTransform, climateModel);
	if (pruning) {
		const bool anyBranchPruned = PruneInternodes(globalTransform, climateModel, shootGrowthController);
		if (anyBranchPruned) m_shootSkeleton.SortLists();
		treeStructureChanged = treeStructureChanged || anyBranchPruned;
	}
	bool anyBranchGrown = false;
	const auto sortedSubTreeInternodeList = m_shootSkeleton.GetSubTree(baseInternodeHandle);

	if (!m_treeGrowthSettings.m_useSpaceColonization) {
		const auto totalShootFlux = CollectShootFlux(sortedSubTreeInternodeList);
		const float requiredVigor = CalculateDesiredGrowthRate(sortedSubTreeInternodeList, shootGrowthController);
		float growthRate = totalShootFlux.m_value / requiredVigor;
		if (overrideGrowthRate > 0.0f) growthRate = overrideGrowthRate;
		AdjustGrowthRate(sortedSubTreeInternodeList, growthRate);
	}
	for (auto it = sortedSubTreeInternodeList.rbegin(); it != sortedSubTreeInternodeList.rend(); ++it) {
		const bool graphChanged = GrowInternode(climateModel, *it, shootGrowthController);
		anyBranchGrown = anyBranchGrown || graphChanged;
	}
	if (anyBranchGrown) m_shootSkeleton.SortLists();
	treeStructureChanged = treeStructureChanged || anyBranchGrown;
	ShootGrowthPostProcess(shootGrowthController);

	const int year = climateModel.m_time;
	if (year != m_ageInYear)
	{
		ResetReproductiveModule();
		m_ageInYear = year;
	}
	m_iteration++;

	return treeStructureChanged;
}

void TreeModel::Initialize(const ShootGrowthController& shootGrowthController) {
	if (m_initialized) Clear();
	{
		auto& firstInternode = m_shootSkeleton.RefNode(0);
		firstInternode.m_info.m_thickness = shootGrowthController.m_endNodeThickness;
		firstInternode.m_data.m_internodeLength = 0.0f;
		firstInternode.m_data.m_buds.emplace_back();
		auto& apicalBud = firstInternode.m_data.m_buds.back();
		apicalBud.m_type = BudType::Apical;
		apicalBud.m_status = BudStatus::Dormant;
		apicalBud.m_localRotation = glm::vec3(0.0f);
	}

	if (m_treeGrowthSettings.m_useSpaceColonization && m_treeGrowthSettings.m_spaceColonizationAutoResize) {
		const auto gridRadius = m_treeGrowthSettings.m_spaceColonizationDetectionDistanceFactor * shootGrowthController.m_internodeLength;
		m_treeOccupancyGrid.Initialize(glm::vec3(-gridRadius, 0.0f, -gridRadius), glm::vec3(gridRadius), shootGrowthController.m_internodeLength,
			m_treeGrowthSettings.m_spaceColonizationRemovalDistanceFactor, m_treeGrowthSettings.m_spaceColonizationTheta, m_treeGrowthSettings.m_spaceColonizationDetectionDistanceFactor);
	}

	m_currentSeedValue = m_seed;
	m_initialized = true;
}

void TreeModel::CalculateShootFlux(const glm::mat4& globalTransform, ClimateModel& climateModel, const ShootGrowthController& shootGrowthController)
{
	auto& shootData = m_shootSkeleton.m_data;
	shootData.m_maxMarkerCount = 0;
	const auto& sortedInternodeList = m_shootSkeleton.RefSortedNodeList();
	if (m_treeGrowthSettings.m_useSpaceColonization)
	{
		if (m_treeGrowthSettings.m_spaceColonizationAutoResize) {
			auto minBound = m_shootSkeleton.m_data.m_desiredMin;
			auto maxBound = m_shootSkeleton.m_data.m_desiredMax;
			const auto originalMin = m_treeOccupancyGrid.GetMin();
			const auto originalMax = m_treeOccupancyGrid.GetMax();
			const float detectionRange = m_treeGrowthSettings.m_spaceColonizationDetectionDistanceFactor * shootGrowthController.m_internodeLength;
			if (minBound.x - detectionRange < originalMin.x || minBound.y < originalMin.y || minBound.z - detectionRange < originalMin.z
				|| maxBound.x + detectionRange > originalMax.x || maxBound.y + detectionRange > originalMax.y || maxBound.z + detectionRange > originalMax.z) {
				minBound -= glm::vec3(1.0f);
				maxBound += glm::vec3(1.0f);
				m_treeOccupancyGrid.Resize(minBound, maxBound);
			}
		}
		auto& voxelGrid = m_treeOccupancyGrid.RefGrid();
		for (const auto& internodeHandle : sortedInternodeList)
		{
			auto& internode = m_shootSkeleton.RefNode(internodeHandle);
			auto& internodeData = internode.m_data;
			for (auto& bud : internodeData.m_buds)
			{
				bud.m_markerDirection = glm::vec3(0.0f);
				bud.m_markerCount = 0;
			}
			internodeData.m_lightDirection = glm::vec3(0.0f);
			const auto dotMin = glm::cos(glm::radians(m_treeOccupancyGrid.GetTheta()));
			voxelGrid.ForEach(internodeData.m_desiredGlobalPosition, m_treeGrowthSettings.m_spaceColonizationRemovalDistanceFactor * shootGrowthController.m_internodeLength,
				[&](TreeOccupancyGridVoxelData& voxelData)
				{
					for (auto& marker : voxelData.m_markers)
					{
						const auto diff = marker.m_position - internodeData.m_desiredGlobalPosition;
						const auto distance = glm::length(diff);
						const auto direction = glm::normalize(diff);
						if (distance < m_treeGrowthSettings.m_spaceColonizationDetectionDistanceFactor * shootGrowthController.m_internodeLength)
						{
							if (marker.m_nodeHandle != -1) continue;
							if (distance < m_treeGrowthSettings.m_spaceColonizationRemovalDistanceFactor * shootGrowthController.m_internodeLength)
							{
								marker.m_nodeHandle = internodeHandle;
							}
							else
							{
								for (auto& bud : internodeData.m_buds) {
									auto budDirection = glm::normalize(internode.m_info.m_globalRotation * bud.m_localRotation * glm::vec3(0, 0, -1));
									if (glm::dot(direction, budDirection) > dotMin)
									{
										bud.m_markerDirection += direction;
										bud.m_markerCount++;
									}
								}
							}
						}

					}
				}
			);
		}
	}
	for (const auto& internodeHandle : sortedInternodeList) {
		auto& internode = m_shootSkeleton.RefNode(internodeHandle);
		auto& internodeData = internode.m_data;
		auto& internodeInfo = internode.m_info;
		internodeData.m_lightIntensity = 0.0f;
		if (m_treeGrowthSettings.m_useSpaceColonization) {
			for (const auto& bud : internodeData.m_buds)
			{
				shootData.m_maxMarkerCount = glm::max(shootData.m_maxMarkerCount, bud.m_markerCount);
			}
		}
		const glm::vec3 position = globalTransform * glm::vec4(internodeInfo.m_globalPosition, 1.0f);
		internodeData.m_lightIntensity = climateModel.m_environmentGrid.IlluminationEstimation(position, internodeData.m_lightDirection);
		if (internodeData.m_lightIntensity <= glm::epsilon<float>())
		{
			internodeData.m_lightDirection = glm::normalize(internodeInfo.m_globalDirection);
		}
		internodeData.m_spaceOccupancy = climateModel.m_environmentGrid.m_voxel.Peek(position).m_totalBiomass;
	}
}
ShootFlux TreeModel::CollectShootFlux(const std::vector<NodeHandle>& sortedInternodeList)
{
	ShootFlux totalShootFlux;
	totalShootFlux.m_value = 0.0f;
	for (const auto& internodeHandle : sortedInternodeList) {
		auto& internode = m_shootSkeleton.PeekNode(internodeHandle);
		const auto& internodeData = internode.m_data;
		totalShootFlux.m_value += internodeData.m_lightIntensity;
	}
	return totalShootFlux;
}

void TreeModel::ShootGrowthPostProcess(const ShootGrowthController& shootGrowthController)
{
	{
		m_shootSkeleton.m_min = glm::vec3(FLT_MAX);
		m_shootSkeleton.m_max = glm::vec3(FLT_MIN);
		m_shootSkeleton.m_data.m_desiredMin = glm::vec3(FLT_MAX);
		m_shootSkeleton.m_data.m_desiredMax = glm::vec3(FLT_MIN);

		m_shootSkeleton.CalculateDistance();
		CalculateThickness(shootGrowthController);
		CalculateBiomass(shootGrowthController);
		CalculateLevel();
		CalculateTransform(shootGrowthController, true);

	};

	m_internodeOrderCounts.clear();
	m_fruitCount = m_leafCount = 0;
	{
		int maxOrder = 0;
		const auto& sortedFlowList = m_shootSkeleton.RefSortedFlowList();
		for (const auto& flowHandle : sortedFlowList) {
			auto& flow = m_shootSkeleton.RefFlow(flowHandle);
			auto& flowData = flow.m_data;
			if (flow.GetParentHandle() == -1) {
				flowData.m_order = 0;
			}
			else {
				auto& parentFlow = m_shootSkeleton.RefFlow(flow.GetParentHandle());
				if (flow.IsApical()) flowData.m_order = parentFlow.m_data.m_order;
				else flowData.m_order = parentFlow.m_data.m_order + 1;
			}
			maxOrder = glm::max(maxOrder, flowData.m_order);
		}
		m_internodeOrderCounts.resize(maxOrder + 1);
		std::fill(m_internodeOrderCounts.begin(), m_internodeOrderCounts.end(), 0);
		const auto& sortedInternodeList = m_shootSkeleton.RefSortedNodeList();
		for (const auto& internodeHandle : sortedInternodeList)
		{
			auto& internode = m_shootSkeleton.RefNode(internodeHandle);
			const auto order = m_shootSkeleton.RefFlow(internode.GetFlowHandle()).m_data.m_order;
			internode.m_data.m_order = order;
			m_internodeOrderCounts[order]++;

			for (const auto& bud : internode.m_data.m_buds)
			{
				if (bud.m_status != BudStatus::Died || bud.m_reproductiveModule.m_maturity <= 0) continue;
				if (bud.m_type == BudType::Fruit)
				{
					m_fruitCount++;
				}
				else if (bud.m_type == BudType::Leaf)
				{
					m_leafCount++;
				}
			}

		}
		m_shootSkeleton.CalculateFlows();
	}
}

float TreeModel::GetSubTreeMaxAge(const NodeHandle baseInternodeHandle) const
{
	const auto sortedSubTreeInternodeList = m_shootSkeleton.GetSubTree(baseInternodeHandle);
	float maxAge = 0.0f;

	for (const auto& internodeHandle : sortedSubTreeInternodeList)
	{
		const auto age = m_shootSkeleton.PeekNode(internodeHandle).m_data.m_startAge;
		maxAge = glm::max(age, maxAge);
	}
	return maxAge;
}

bool TreeModel::Reduce(const ShootGrowthController& shootGrowthController, const NodeHandle baseInternodeHandle, float targetAge)
{
	const auto sortedSubTreeInternodeList = m_shootSkeleton.GetSubTree(baseInternodeHandle);
	if (sortedSubTreeInternodeList.size() == 1) return false;
	bool reduced = false;
	for (auto it = sortedSubTreeInternodeList.rbegin(); it != sortedSubTreeInternodeList.rend(); ++it) {
		auto& internode = m_shootSkeleton.RefNode(*it);
		if (internode.m_data.m_startAge > targetAge)
		{
			const auto parentHandle = internode.GetParentHandle();
			if (parentHandle != -1)
			{
				auto& parent = m_shootSkeleton.RefNode(parentHandle);
				parent.m_info.m_thickness = shootGrowthController.m_endNodeThickness;
				parent.m_data.m_buds[internode.m_data.m_indexOfParentBud].m_status = BudStatus::Dormant;
			}
			PruneInternode(*it);
			reduced = true;
		}
	}

	if (reduced) m_shootSkeleton.SortLists();
	ShootGrowthPostProcess(shootGrowthController);
	return reduced;
}

void TreeModel::CalculateTransform(const ShootGrowthController& shootGrowthController, bool sagging)
{
	const auto& sortedInternodeList = m_shootSkeleton.RefSortedNodeList();
	for (const auto& internodeHandle : sortedInternodeList) {
		auto& internode = m_shootSkeleton.RefNode(internodeHandle);
		auto& internodeData = internode.m_data;
		auto& internodeInfo = internode.m_info;

		internodeInfo.m_length = internodeData.m_internodeLength * glm::pow(internodeInfo.m_thickness / shootGrowthController.m_endNodeThickness, shootGrowthController.m_internodeLengthThicknessFactor);

		if (internode.GetParentHandle() == -1) {
			internodeInfo.m_globalPosition = internodeData.m_desiredGlobalPosition = glm::vec3(0.0f);
			internodeData.m_desiredLocalRotation = glm::vec3(0.0f);
			internodeInfo.m_globalRotation = internodeInfo.m_regulatedGlobalRotation = internodeData.m_desiredGlobalRotation = glm::vec3(glm::radians(90.0f), 0.0f, 0.0f);
			internodeInfo.m_globalDirection = glm::normalize(internodeInfo.m_globalRotation * glm::vec3(0, 0, -1));
		}
		else {
			auto& parentInternode = m_shootSkeleton.RefNode(internode.GetParentHandle());
			internodeData.m_sagging = shootGrowthController.m_sagging(internode);
			auto parentGlobalRotation = parentInternode.m_info.m_globalRotation;
			internodeInfo.m_globalRotation = parentGlobalRotation * internodeData.m_desiredLocalRotation;
			auto front = glm::normalize(internodeInfo.m_globalRotation * glm::vec3(0, 0, -1));
			auto up = glm::normalize(internodeInfo.m_globalRotation * glm::vec3(0, 1, 0));
			if (sagging) {
				float dotP = glm::abs(glm::dot(front, m_currentGravityDirection));
				ApplyTropism(m_currentGravityDirection, internodeData.m_sagging * (1.0f - dotP), front, up);
				internodeInfo.m_globalRotation = glm::quatLookAt(front, up);
			}
			auto parentRegulatedUp = parentInternode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 1, 0);
			auto regulatedUp = glm::normalize(glm::cross(glm::cross(front, parentRegulatedUp), front));
			internodeInfo.m_regulatedGlobalRotation = glm::quatLookAt(front, regulatedUp);

			internodeInfo.m_globalDirection = glm::normalize(internodeInfo.m_globalRotation * glm::vec3(0, 0, -1));
			internodeInfo.m_globalPosition =
				parentInternode.m_info.m_globalPosition
				+ parentInternode.m_info.m_length * parentInternode.m_info.m_globalDirection;

			if (!internode.IsApical())
			{
				const auto relativeFront = glm::inverse(parentInternode.m_info.m_globalRotation) * internodeInfo.m_globalRotation * glm::vec3(0, 0, -1);
				auto parentUp = glm::normalize(parentInternode.m_info.m_globalRotation * glm::vec3(0, 1, 0));
				auto parentLeft = glm::normalize(parentInternode.m_info.m_globalRotation * glm::vec3(1, 0, 0));
				auto parentFront = glm::normalize(parentInternode.m_info.m_globalRotation * glm::vec3(0, 0, -1));
				const auto sinValue = glm::sin(glm::acos(glm::dot(parentFront, front)));
				const auto offset = glm::normalize(glm::vec2(relativeFront.x, relativeFront.y)) * sinValue;
				internodeInfo.m_globalPosition += parentLeft * parentInternode.m_info.m_thickness * offset.x;
				internodeInfo.m_globalPosition += parentUp * parentInternode.m_info.m_thickness * offset.y;
				internodeInfo.m_globalPosition += parentFront * parentInternode.m_info.m_thickness * sinValue;
			}

			internodeData.m_desiredGlobalRotation = parentInternode.m_data.m_desiredGlobalRotation * internodeData.m_desiredLocalRotation;
			auto parentDesiredFront = parentInternode.m_data.m_desiredGlobalRotation * glm::vec3(0, 0, -1);
			internodeData.m_desiredGlobalPosition = parentInternode.m_data.m_desiredGlobalPosition +
				parentInternode.m_info.m_length * parentDesiredFront;
		}

		m_shootSkeleton.m_min = glm::min(m_shootSkeleton.m_min, internodeInfo.m_globalPosition);
		m_shootSkeleton.m_max = glm::max(m_shootSkeleton.m_max, internodeInfo.m_globalPosition);
		const auto endPosition = internodeInfo.m_globalPosition
			+ internodeInfo.m_length * internodeInfo.m_globalDirection;
		m_shootSkeleton.m_min = glm::min(m_shootSkeleton.m_min, endPosition);
		m_shootSkeleton.m_max = glm::max(m_shootSkeleton.m_max, endPosition);

		m_shootSkeleton.m_data.m_desiredMin = glm::min(m_shootSkeleton.m_data.m_desiredMin, internodeData.m_desiredGlobalPosition);
		m_shootSkeleton.m_data.m_desiredMax = glm::max(m_shootSkeleton.m_data.m_desiredMax, internodeData.m_desiredGlobalPosition);
		const auto desiredGlobalDirection = internodeData.m_desiredGlobalRotation * glm::vec3(0, 0, -1);
		const auto desiredEndPosition = internodeData.m_desiredGlobalPosition
			+ internodeInfo.m_length * desiredGlobalDirection;
		m_shootSkeleton.m_data.m_desiredMin = glm::min(m_shootSkeleton.m_data.m_desiredMin, desiredEndPosition);
		m_shootSkeleton.m_data.m_desiredMax = glm::max(m_shootSkeleton.m_data.m_desiredMax, desiredEndPosition);
	}
}

bool TreeModel::ElongateInternode(float extendLength, NodeHandle internodeHandle,
	const ShootGrowthController& shootGrowthController, float& collectedInhibitor) {
	bool graphChanged = false;
	auto& internode = m_shootSkeleton.RefNode(internodeHandle);
	const auto internodeLength = shootGrowthController.m_internodeLength;
	auto& internodeData = internode.m_data;
	const auto& internodeInfo = internode.m_info;
	internodeData.m_internodeLength += extendLength;
	const float extraLength = internodeData.m_internodeLength - internodeLength;
	auto& apicalBud = internodeData.m_buds.front();
	//If we need to add a new end node
	if (extraLength >= 0) {
		graphChanged = true;
		apicalBud.m_status = BudStatus::Removed;
		internodeData.m_internodeLength = internodeLength;
		const auto desiredGlobalRotation = internodeInfo.m_globalRotation * apicalBud.m_localRotation;
		auto desiredGlobalFront = desiredGlobalRotation * glm::vec3(0, 0, -1);
		auto desiredGlobalUp = desiredGlobalRotation * glm::vec3(0, 1, 0);
		if (internodeHandle != 0)
		{
			ApplyTropism(-m_currentGravityDirection, shootGrowthController.m_gravitropism(internode), desiredGlobalFront,
				desiredGlobalUp);
			ApplyTropism(internodeData.m_lightDirection, shootGrowthController.m_phototropism(internode),
				desiredGlobalFront, desiredGlobalUp);
		}

		//Allocate Lateral bud for current internode
		{
			const auto lateralBudCount = shootGrowthController.m_lateralBudCount;
			const float turnAngle = glm::radians(360.0f / lateralBudCount);
			for (int i = 0; i < lateralBudCount; i++) {
				internodeData.m_buds.emplace_back();
				auto& newLateralBud = internodeData.m_buds.back();
				newLateralBud.m_type = BudType::Lateral;
				newLateralBud.m_status = BudStatus::Dormant;
				newLateralBud.m_localRotation = glm::vec3(glm::radians(shootGrowthController.m_branchingAngle(internode)), 0.0f,
					i * turnAngle);
				shootGrowthController.m_budExtinctionRate(internode, newLateralBud);
				if (internodeHandle != 0 && newLateralBud.m_extinctionRate > glm::linearRand(0.0f, 1.0f))
				{
					newLateralBud.m_status = BudStatus::Removed;
				}
			}
		}
		//Allocate Fruit bud for current internode
		{
			const auto fruitBudCount = shootGrowthController.m_fruitBudCount;
			for (int i = 0; i < fruitBudCount; i++) {
				internodeData.m_buds.emplace_back();
				auto& newFruitBud = internodeData.m_buds.back();
				newFruitBud.m_type = BudType::Fruit;
				newFruitBud.m_status = BudStatus::Dormant;
				newFruitBud.m_localRotation = glm::vec3(
					glm::radians(shootGrowthController.m_branchingAngle(internode)), 0.0f,
					glm::radians(glm::linearRand(0.0f, 360.0f)));
				shootGrowthController.m_budExtinctionRate(internode, newFruitBud);
				if (internodeHandle != 0 && newFruitBud.m_extinctionRate > glm::linearRand(0.0f, 1.0f))
				{
					newFruitBud.m_status = BudStatus::Removed;
				}
			}
		}
		//Allocate Leaf bud for current internode
		{
			const auto leafBudCount = shootGrowthController.m_leafBudCount;
			for (int i = 0; i < leafBudCount; i++) {
				internodeData.m_buds.emplace_back();
				auto& newLeafBud = internodeData.m_buds.back();
				//Hack: Leaf bud will be given vigor for the first time.
				newLeafBud.m_type = BudType::Leaf;
				newLeafBud.m_status = BudStatus::Dormant;
				newLeafBud.m_localRotation = glm::vec3(
					glm::radians(shootGrowthController.m_branchingAngle(internode)), 0.0f,
					glm::radians(glm::linearRand(0.0f, 360.0f)));
				shootGrowthController.m_budExtinctionRate(internode, newLeafBud);
				if (internodeHandle != 0 && newLeafBud.m_extinctionRate > glm::linearRand(0.0f, 1.0f))
				{
					newLeafBud.m_status = BudStatus::Removed;
				}
			}
		}
		//Create new internode
		const auto newInternodeHandle = m_shootSkeleton.Extend(internodeHandle, false);
		auto& oldInternode = m_shootSkeleton.RefNode(internodeHandle);
		auto& newInternode = m_shootSkeleton.RefNode(newInternodeHandle);
		newInternode.m_data = {};
		newInternode.m_data.m_frontProfile = {};
		newInternode.m_data.m_frontProfile.Reset(0.001f);
		newInternode.m_data.m_backProfile = {};
		newInternode.m_data.m_backProfile.Reset(0.001f);
		newInternode.m_data.m_frontParticleMap = {};
		newInternode.m_data.m_backParticleMap = {};
		newInternode.m_data.m_profileConstraints = {};
		newInternode.m_data.m_indexOfParentBud = 0;
		newInternode.m_data.m_pipeSize = 0;
		newInternode.m_data.m_lightIntensity = oldInternode.m_data.m_lightIntensity;
		newInternode.m_data.m_lightDirection = oldInternode.m_data.m_lightDirection;
		oldInternode.m_data.m_finishAge = newInternode.m_data.m_startAge = m_age;
		newInternode.m_data.m_finishAge = 0.0f;
		newInternode.m_data.m_order = oldInternode.m_data.m_order;
		newInternode.m_data.m_lateral = false;
		newInternode.m_data.m_inhibitorSink = 0.0f;
		newInternode.m_data.m_internodeLength = glm::clamp(extendLength, 0.0f, internodeLength);
		newInternode.m_info.m_thickness = shootGrowthController.m_endNodeThickness;
		newInternode.m_info.m_globalRotation = glm::quatLookAt(desiredGlobalFront, desiredGlobalUp);
		newInternode.m_data.m_desiredLocalRotation =
			glm::inverse(oldInternode.m_info.m_globalRotation) *
			newInternode.m_info.m_globalRotation;
		//Allocate apical bud for new internode
		newInternode.m_data.m_buds.emplace_back();
		auto& newApicalBud = newInternode.m_data.m_buds.back();
		newApicalBud.m_type = BudType::Apical;
		newApicalBud.m_status = BudStatus::Dormant;
		newApicalBud.m_localRotation = glm::vec3(
			glm::radians(shootGrowthController.m_apicalAngle(newInternode)), 0.0f,
			glm::radians(shootGrowthController.m_rollAngle(newInternode)));

		shootGrowthController.m_budExtinctionRate(newInternode, newApicalBud);
		if (internodeHandle != 0 && newApicalBud.m_extinctionRate > glm::linearRand(0.0f, 1.0f))
		{
			newApicalBud.m_status = BudStatus::Removed;
		}
		if (newApicalBud.m_status != BudStatus::Removed && extraLength > internodeLength) {
			float childInhibitor = 0.0f;
			ElongateInternode(extraLength - internodeLength, newInternodeHandle, shootGrowthController, childInhibitor);
			auto& currentNewInternode = m_shootSkeleton.RefNode(newInternodeHandle);
			currentNewInternode.m_data.m_inhibitorSink += glm::max(0.0f, childInhibitor * glm::clamp(1.0f - shootGrowthController.m_apicalDominanceLoss, 0.0f, 1.0f));
			collectedInhibitor += currentNewInternode.m_data.m_inhibitorSink + shootGrowthController.m_apicalDominance(currentNewInternode);
		}
		else {
			collectedInhibitor += shootGrowthController.m_apicalDominance(newInternode);
		}
	}
	return graphChanged;
}

bool TreeModel::GrowInternode(ClimateModel& climateModel, NodeHandle internodeHandle, const ShootGrowthController& shootGrowthController) {
	bool graphChanged = false;
	{
		auto& internode = m_shootSkeleton.RefNode(internodeHandle);
		auto& internodeData = internode.m_data;
		internodeData.m_inhibitorSink = 0;
		for (const auto& childHandle : internode.RefChildHandles()) {
			auto& childNode = m_shootSkeleton.RefNode(childHandle);
			internodeData.m_inhibitorSink += glm::max(0.0f, (shootGrowthController.m_apicalDominance(childNode) + childNode.m_data.m_inhibitorSink) *
				glm::clamp(1.0f - shootGrowthController.m_apicalDominanceLoss, 0.0f, 1.0f));
		}
	}
	const auto budSize = m_shootSkeleton.RefNode(internodeHandle).m_data.m_buds.size();
	for (int budIndex = 0; budIndex < budSize; budIndex++) {
		auto& internode = m_shootSkeleton.RefNode(internodeHandle);
		auto& bud = internode.m_data.m_buds[budIndex];
		auto& internodeData = internode.m_data;
		auto& internodeInfo = internode.m_info;
		shootGrowthController.m_budFlushingRate(internode, bud);
		shootGrowthController.m_budExtinctionRate(internode, bud);
		if (bud.m_status == BudStatus::Removed) continue;
		if (bud.m_extinctionRate >= glm::linearRand(0.0f, 1.0f))
		{
			bud.m_status = BudStatus::Removed;
			continue;
		}
		//Calculate vigor used for maintenance and development.
		if (bud.m_type == BudType::Apical) {
			float elongateLength = 0.0f;
			if (m_treeGrowthSettings.m_useSpaceColonization) {
				if (m_shootSkeleton.m_data.m_maxMarkerCount > 0) elongateLength = static_cast<float>(bud.m_markerCount) / m_shootSkeleton.m_data.m_maxMarkerCount * shootGrowthController.m_internodeLength;
			}
			else
			{
				elongateLength = internodeData.m_growthRate * m_currentDeltaTime * shootGrowthController.m_internodeLength * shootGrowthController.m_internodeGrowthRate;
			}
			//Use up the vigor stored in this bud.
			float collectedInhibitor = 0.0f;
			graphChanged = ElongateInternode(elongateLength, internodeHandle, shootGrowthController, collectedInhibitor) || graphChanged;
			m_shootSkeleton.RefNode(internodeHandle).m_data.m_inhibitorSink += glm::max(0.0f, collectedInhibitor * glm::clamp(1.0f - shootGrowthController.m_apicalDominanceLoss, 0.0f, 1.0f));
			break;
		}
		if (bud.m_type == BudType::Lateral && bud.m_status == BudStatus::Dormant) {
			float flushProbability = bud.m_flushingRate;
			if (m_treeGrowthSettings.m_useSpaceColonization) {
				if (m_shootSkeleton.m_data.m_maxMarkerCount > 0) flushProbability *= static_cast<float>(bud.m_markerCount) / m_shootSkeleton.m_data.m_maxMarkerCount;
			}
			else
			{
				flushProbability *= internodeData.m_growthRate * m_currentDeltaTime * shootGrowthController.m_internodeGrowthRate;
			}
			if (flushProbability >= glm::linearRand(0.0f, 1.0f)) {
				graphChanged = true;
				bud.m_status = BudStatus::Removed;
				//Prepare information for new internode
				auto desiredGlobalRotation = internodeInfo.m_globalRotation * bud.m_localRotation;
				auto desiredGlobalFront = desiredGlobalRotation * glm::vec3(0, 0, -1);
				auto desiredGlobalUp = desiredGlobalRotation * glm::vec3(0, 1, 0);
				ApplyTropism(-m_currentGravityDirection, shootGrowthController.m_gravitropism(internode), desiredGlobalFront,
					desiredGlobalUp);
				ApplyTropism(internodeData.m_lightDirection, shootGrowthController.m_phototropism(internode),
					desiredGlobalFront, desiredGlobalUp);
				//Create new internode
				const auto newInternodeHandle = m_shootSkeleton.Extend(internodeHandle, true);
				auto& oldInternode = m_shootSkeleton.RefNode(internodeHandle);
				auto& newInternode = m_shootSkeleton.RefNode(newInternodeHandle);
				newInternode.m_data = {};
				newInternode.m_data.m_frontProfile = {};
				newInternode.m_data.m_frontProfile.Reset(0.001f);
				newInternode.m_data.m_backProfile = {};
				newInternode.m_data.m_backProfile.Reset(0.001f);
				newInternode.m_data.m_frontParticleMap = {};
				newInternode.m_data.m_backParticleMap = {};
				newInternode.m_data.m_profileConstraints = {};
				newInternode.m_data.m_indexOfParentBud = 0;
				newInternode.m_data.m_pipeSize = 0;

				newInternode.m_data.m_startAge = m_age;
				newInternode.m_data.m_finishAge = 0.0f;
				newInternode.m_data.m_order = oldInternode.m_data.m_order + 1;
				newInternode.m_data.m_lateral = true;
				newInternode.m_data.m_internodeLength = 0.0f;
				newInternode.m_info.m_thickness = shootGrowthController.m_endNodeThickness;
				newInternode.m_data.m_desiredLocalRotation =
					glm::inverse(oldInternode.m_info.m_globalRotation) *
					glm::quatLookAt(desiredGlobalFront, desiredGlobalUp);
				//Allocate apical bud
				newInternode.m_data.m_buds.emplace_back();
				auto& apicalBud = newInternode.m_data.m_buds.back();
				apicalBud.m_type = BudType::Apical;
				apicalBud.m_status = BudStatus::Dormant;
				apicalBud.m_localRotation = glm::vec3(
					glm::radians(shootGrowthController.m_apicalAngle(newInternode)), 0.0f,
					glm::radians(shootGrowthController.m_rollAngle(newInternode)));
			}
		}
		else if (bud.m_type == BudType::Fruit)
		{
			/*
			if (!seasonality)
			{
				const float maxMaturityIncrease = availableDevelopmentVigor / shootGrowthController.m_fruitVigorRequirement;
				const auto developmentVigor = bud.m_vigorSink.SubtractVigor(maxMaturityIncrease * shootGrowthController.m_fruitVigorRequirement);
			}
			else if (bud.m_status == BudStatus::Dormant) {
				const float flushProbability = m_currentDeltaTime * shootGrowthController.m_fruitBudFlushingProbability(internode);
				if (flushProbability >= glm::linearRand(0.0f, 1.0f))
				{
					bud.m_status = BudStatus::Flushed;
				}
			}
			else if (bud.m_status == BudStatus::Flushed)
			{
				//Make the fruit larger;
				const float maxMaturityIncrease = availableDevelopmentVigor / shootGrowthController.m_fruitVigorRequirement;
				const float maturityIncrease = glm::min(maxMaturityIncrease, glm::min(m_currentDeltaTime * shootGrowthController.m_fruitGrowthRate, 1.0f - bud.m_reproductiveModule.m_maturity));
				bud.m_reproductiveModule.m_maturity += maturityIncrease;
				const auto developmentVigor = bud.m_vigorSink.SubtractVigor(maturityIncrease * shootGrowthController.m_fruitVigorRequirement);
				auto fruitSize = shootGrowthController.m_maxFruitSize * glm::sqrt(bud.m_reproductiveModule.m_maturity);
				float angle = glm::radians(glm::linearRand(0.0f, 360.0f));
				glm::quat rotation = internodeData.m_desiredLocalRotation * bud.m_localRotation;
				auto up = rotation * glm::vec3(0, 1, 0);
				auto front = rotation * glm::vec3(0, 0, -1);
				ApplyTropism(internodeData.m_lightDirection, 0.3f, up, front);
				rotation = glm::quatLookAt(front, up);
				auto fruitPosition = internodeInfo.m_globalPosition + front * (fruitSize.z * 1.f);
				bud.m_reproductiveModule.m_transform = glm::translate(fruitPosition) * glm::mat4_cast(glm::quat(glm::vec3(0.0f))) * glm::scale(fruitSize);

				bud.m_reproductiveModule.m_health -= m_currentDeltaTime * shootGrowthController.m_fruitDamage(internode);
				bud.m_reproductiveModule.m_health = glm::clamp(bud.m_reproductiveModule.m_health, 0.0f, 1.0f);

				//Handle fruit drop here.
				if (bud.m_reproductiveModule.m_maturity >= 0.95f || bud.m_reproductiveModule.m_health <= 0.05f)
				{
					auto dropProbability = m_currentDeltaTime * shootGrowthController.m_fruitFallProbability(internode);
					if (dropProbability >= glm::linearRand(0.0f, 1.0f))
					{
						bud.m_status = BudStatus::Died;
						m_shootSkeleton.m_data.m_droppedFruits.emplace_back(bud.m_reproductiveModule);
						bud.m_reproductiveModule.Reset();
					}
				}

			}*/
		}
		else if (bud.m_type == BudType::Leaf)
		{
			if (bud.m_status == BudStatus::Dormant) {
				const float flushProbability = m_currentDeltaTime * shootGrowthController.m_leafBudFlushingProbability(internode);
				if (flushProbability >= glm::linearRand(0.0f, 1.0f))
				{
					bud.m_status = BudStatus::Died;
				}
			}
			else if (bud.m_status == BudStatus::Died)
			{
				//Make the leaf larger
				//const float maxMaturityIncrease = availableDevelopmentVigor / shootGrowthController.m_leafVigorRequirement;
				//const float maturityIncrease = glm::min(maxMaturityIncrease, glm::min(m_currentDeltaTime * shootGrowthController.m_leafGrowthRate, 1.0f - bud.m_reproductiveModule.m_maturity));
				//bud.m_reproductiveModule.m_maturity += maturityIncrease;
				//const auto developmentVigor = bud.m_vigorSink.SubtractVigor(maturityIncrease * shootGrowthController.m_leafVigorRequirement);
				auto leafSize = shootGrowthController.m_maxLeafSize * glm::sqrt(bud.m_reproductiveModule.m_maturity);
				glm::quat rotation = internodeData.m_desiredLocalRotation * bud.m_localRotation;
				auto up = rotation * glm::vec3(0, 1, 0);
				auto front = rotation * glm::vec3(0, 0, -1);
				ApplyTropism(internodeData.m_lightDirection, 0.3f, up, front);
				rotation = glm::quatLookAt(front, up);
				auto foliagePosition = internodeInfo.m_globalPosition + front * (leafSize.z);
				bud.m_reproductiveModule.m_transform = glm::translate(foliagePosition) * glm::mat4_cast(rotation) * glm::scale(leafSize);

				bud.m_reproductiveModule.m_health -= m_currentDeltaTime * shootGrowthController.m_leafDamage(internode);
				bud.m_reproductiveModule.m_health = glm::clamp(bud.m_reproductiveModule.m_health, 0.0f, 1.0f);

				//Handle leaf drop here.
				if (bud.m_reproductiveModule.m_health <= 0.05f)
				{
					auto dropProbability = m_currentDeltaTime * shootGrowthController.m_leafFallProbability(internode);
					if (dropProbability >= glm::linearRand(0.0f, 1.0f))
					{
						bud.m_status = BudStatus::Died;
						m_shootSkeleton.m_data.m_droppedLeaves.emplace_back(bud.m_reproductiveModule);
						bud.m_reproductiveModule.Reset();
					}
				}
			}
		}
	}
	return graphChanged;
}

void TreeModel::CalculateLevel()
{
	auto& sortedInternodeList = m_shootSkeleton.RefSortedNodeList();
	for (const auto& internodeHandle : sortedInternodeList)
	{
		auto& node = m_shootSkeleton.RefNode(internodeHandle);
		node.m_data.m_growthPotential = 0.0f;
		if (node.GetParentHandle() == -1)
		{
			node.m_data.m_level = 0;
		}
		else
		{
			float maxBiomass = 0.0f;
			NodeHandle maxChild = -1;
			for (const auto& childHandle : node.RefChildHandles())
			{
				auto& childNode = m_shootSkeleton.PeekNode(childHandle);
				const auto childBiomass = childNode.m_data.m_descendentTotalBiomass + childNode.m_data.m_biomass;
				if (childBiomass > maxBiomass)
				{
					maxBiomass = childBiomass;
					maxChild = childHandle;
				}
			}
			for (const auto& childHandle : node.RefChildHandles())
			{
				auto& childNode = m_shootSkeleton.RefNode(childHandle);
				if (childHandle == maxChild)
				{
					childNode.m_data.m_level = node.m_data.m_level;
					childNode.m_data.m_maxChild = true;
				}
				else
				{
					childNode.m_data.m_level = node.m_data.m_level + 1;
					childNode.m_data.m_maxChild = false;
				}
			}
		}
		m_shootSkeleton.m_data.m_maxLevel = glm::max(m_shootSkeleton.m_data.m_maxLevel, node.m_data.m_level);
	}
}


void TreeModel::AdjustGrowthRate(const std::vector<NodeHandle>& sortedInternodeList, float factor)
{
	const float clampedFactor = glm::clamp(factor, 0.0f, 1.0f);
	for (const auto& internodeHandle : sortedInternodeList)
	{
		auto& node = m_shootSkeleton.RefNode(internodeHandle);
		//You cannot give more than enough resources.
		node.m_data.m_growthRate = clampedFactor * node.m_data.m_desiredGrowthRate;
	}
}

float TreeModel::CalculateDesiredGrowthRate(const std::vector<NodeHandle>& sortedInternodeList, const ShootGrowthController& shootGrowthController)
{
	const float apicalControl = shootGrowthController.m_apicalControl;
	const float pipeResistanceFactor = shootGrowthController.m_pipeResistance;
	float minDistance = FLT_MAX;
	for (const auto& internodeHandle : sortedInternodeList)
	{
		const auto& node = m_shootSkeleton.PeekNode(internodeHandle);
		minDistance = glm::min(minDistance, node.m_info.m_rootDistance);
	}
	if (minDistance == 0.0f) minDistance = 1.0f;
	float maximumDesiredGrowthPotential = 0.0f;
	float maximumApicalControl = 0.0f;
	std::vector<float> apicalControlValues{};
	apicalControlValues.resize(m_shootSkeleton.m_data.m_maxLevel + 1);
	apicalControlValues[0] = 1.0f;

	for (int i = 1; i < m_shootSkeleton.m_data.m_maxLevel + 1; i++)
	{
		apicalControlValues[i] = apicalControlValues[i - 1] * apicalControl;
	}

	for (const auto& internodeHandle : sortedInternodeList)
	{
		auto& node = m_shootSkeleton.RefNode(internodeHandle);
		node.m_data.m_pipeResistance = glm::pow(glm::max(1.0f, node.m_info.m_rootDistance / minDistance), pipeResistanceFactor);
		node.m_data.m_growthPotential = node.m_data.m_lightIntensity / node.m_data.m_pipeResistance;
		if (apicalControl > 1.0f)
		{
			node.m_data.m_apicalControl = 1.0f / apicalControlValues[node.m_data.m_level];
		}
		else if (apicalControl < 1.0f)
		{
			node.m_data.m_apicalControl = apicalControlValues[m_shootSkeleton.m_data.m_maxLevel - node.m_data.m_level];
		}
		else
		{
			node.m_data.m_apicalControl = 1.0f;
		}
		maximumApicalControl = glm::max(maximumApicalControl, node.m_data.m_apicalControl);
		maximumDesiredGrowthPotential = glm::max(maximumDesiredGrowthPotential, node.m_data.m_growthPotential);
	}
	float maximumGrowthRate = 0.0f;
	for (const auto& internodeHandle : sortedInternodeList)
	{
		auto& node = m_shootSkeleton.RefNode(internodeHandle);
		node.m_data.m_growthPotential /= maximumDesiredGrowthPotential;
		node.m_data.m_apicalControl /= maximumApicalControl;
		node.m_data.m_desiredGrowthRate = node.m_data.m_growthPotential * node.m_data.m_apicalControl;
		maximumGrowthRate = glm::max(maximumGrowthRate, node.m_data.m_desiredGrowthRate);
	}

	float totalDesiredGrowthRate = 1.0f;
	for (const auto& internodeHandle : sortedInternodeList)
	{
		auto& node = m_shootSkeleton.RefNode(internodeHandle);
		node.m_data.m_desiredGrowthRate /= maximumGrowthRate;
		totalDesiredGrowthRate += node.m_data.m_desiredGrowthRate;
	}
	return totalDesiredGrowthRate;

}

void TreeModel::CalculateThickness(const ShootGrowthController& shootGrowthController) {
	auto& sortedInternodeList = m_shootSkeleton.RefSortedNodeList();
	for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); ++it) {
		const auto internodeHandle = *it;
		auto& internode = m_shootSkeleton.RefNode(internodeHandle);
		const auto& internodeData = internode.m_data;
		auto& internodeInfo = internode.m_info;
		float childThicknessCollection = 0.0f;
		for (const auto& i : internode.RefChildHandles()) {
			const auto& childInternode = m_shootSkeleton.PeekNode(i);
			childThicknessCollection += glm::pow(childInternode.m_info.m_thickness,
				1.0f / shootGrowthController.m_thicknessAccumulationFactor);
		}
		childThicknessCollection += shootGrowthController.m_thicknessAccumulateAgeFactor * shootGrowthController.m_endNodeThickness * shootGrowthController.m_internodeGrowthRate * (m_age - internodeData.m_startAge);
		if (childThicknessCollection != 0.0f) {
			internodeInfo.m_thickness = glm::pow(childThicknessCollection, shootGrowthController.m_thicknessAccumulationFactor);
		}
		else
		{
			internodeInfo.m_thickness = glm::max(internodeInfo.m_thickness, shootGrowthController.m_endNodeThickness);
		}
	}
}
void TreeModel::CalculateBiomass(const ShootGrowthController& shootGrowthController)
{
	auto& sortedInternodeList = m_shootSkeleton.RefSortedNodeList();
	for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); ++it) {
		const auto internodeHandle = *it;
		auto& internode = m_shootSkeleton.RefNode(internodeHandle);
		auto& internodeData = internode.m_data;
		auto& internodeInfo = internode.m_info;
		internodeData.m_descendentTotalBiomass = internodeData.m_biomass = 0.0f;
		internodeData.m_biomass =
			internodeInfo.m_thickness / shootGrowthController.m_endNodeThickness * internodeData.m_internodeLength /
			shootGrowthController.m_internodeLength;
		for (const auto& i : internode.RefChildHandles()) {
			const auto& childInternode = m_shootSkeleton.RefNode(i);
			internodeData.m_descendentTotalBiomass +=
				childInternode.m_data.m_descendentTotalBiomass +
				childInternode.m_data.m_biomass;
		}
	}
}
void TreeModel::Clear() {
	m_shootSkeleton = {};
	m_history = {};
	m_initialized = false;

	if (m_treeGrowthSettings.m_useSpaceColonization && !m_treeGrowthSettings.m_spaceColonizationAutoResize)
	{
		m_treeOccupancyGrid.ResetMarkers();
	}
	else
	{
		m_treeOccupancyGrid = {};
	}

	m_age = 0;
	m_iteration = 0;
}

int TreeModel::GetLeafCount() const
{
	return m_leafCount;
}

int TreeModel::GetFruitCount() const
{
	return m_fruitCount;
}

bool TreeModel::PruneInternodes(const glm::mat4& globalTransform, ClimateModel& climateModel, const ShootGrowthController& shootGrowthController) {

	const auto maxDistance = m_shootSkeleton.PeekNode(0).m_info.m_endDistance;
	const auto& sortedInternodeList = m_shootSkeleton.RefSortedNodeList();
	bool anyInternodePruned = false;

	for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); ++it) {
		const auto internodeHandle = *it;
		if (m_shootSkeleton.PeekNode(internodeHandle).IsRecycled()) continue;
		if (internodeHandle == 0) continue;
		const auto& internode = m_shootSkeleton.PeekNode(*it);
		if (internode.m_info.m_globalPosition.y <= 0.05f && internode.m_data.m_order != 0)
		{
			auto handleWalker = internodeHandle;
			int i = 0;
			while (i < 4 && handleWalker != -1 && m_shootSkeleton.PeekNode(handleWalker).IsApical())
			{
				handleWalker = m_shootSkeleton.PeekNode(handleWalker).GetParentHandle();
				i++;
			}
			if (handleWalker != -1) {
				auto& targetInternode = m_shootSkeleton.PeekNode(handleWalker);
				if (targetInternode.m_data.m_order != 0)
				{
					PruneInternode(handleWalker);
					anyInternodePruned = true;
				}
			}
		}
	}

	if (anyInternodePruned) m_shootSkeleton.SortLists();

	for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); ++it) {
		const auto internodeHandle = *it;
		if (m_shootSkeleton.PeekNode(internodeHandle).IsRecycled()) continue;
		if (internodeHandle == 0) continue;
		const auto& internode = m_shootSkeleton.PeekNode(internodeHandle);
		//Pruning here.
		bool pruning = false;
		const float pruningProbability = shootGrowthController.m_pruningFactor(m_currentDeltaTime, internode);
		if (pruningProbability > glm::linearRand(0.0f, 1.0f)) pruning = true;
		bool lowBranchPruning = false;
		if (!pruning && maxDistance > 5.0f * shootGrowthController.m_internodeLength && internode.m_data.m_order == 1 &&
			(internode.m_info.m_rootDistance / maxDistance) < shootGrowthController.m_lowBranchPruning) {
			const auto parentHandle = internode.GetParentHandle();
			if (parentHandle != -1) {
				const auto& parent = m_shootSkeleton.PeekNode(parentHandle);
				if (shootGrowthController.m_lowBranchPruningThicknessFactor == 0.0f || internode.m_info.m_thickness / parent.m_info.m_thickness < shootGrowthController.m_lowBranchPruningThicknessFactor) lowBranchPruning = true;
			}

		}
		bool pruneByCrownShyness = false;
		if (!pruning && m_crownShynessDistance > 0.f && internode.IsEndNode()) {
			//const glm::vec3 startPosition = globalTransform * glm::vec4(internode.m_info.m_globalPosition, 1.0f);
			const glm::vec3 endPosition = globalTransform * glm::vec4(internode.m_info.GetGlobalEndPosition(), 1.0f);
			climateModel.m_environmentGrid.m_voxel.ForEach(endPosition, m_crownShynessDistance * 2.0f, [&](EnvironmentVoxel& data)
				{
					if (pruneByCrownShyness) return;
					for (const auto& i : data.m_internodeVoxelRegistrations)
					{
						if (i.m_treeModelIndex == m_index) continue;
						if (glm::distance(endPosition, i.m_position) < m_crownShynessDistance) pruneByCrownShyness = true;
					}
				}
			);
		}
		if (pruning || pruneByCrownShyness || lowBranchPruning)
		{
			PruneInternode(internodeHandle);
			anyInternodePruned = true;
		}
	}
	m_shootSkeleton.CalculateDistance();
	CalculateLevel();
	return anyInternodePruned;
}




void TreeModel::SampleTemperature(const glm::mat4& globalTransform, ClimateModel& climateModel)
{
	const auto& sortedInternodeList = m_shootSkeleton.RefSortedNodeList();
	for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); ++it) {
		auto& internode = m_shootSkeleton.RefNode(*it);
		auto& internodeData = internode.m_data;
		auto& internodeInfo = internode.m_info;
		internodeData.m_temperature = climateModel.GetTemperature(globalTransform * glm::translate(internodeInfo.m_globalPosition)[3]);
	}
}

ShootSkeleton& TreeModel::RefShootSkeleton() {
	return m_shootSkeleton;
}

const ShootSkeleton&
TreeModel::PeekShootSkeleton(const int iteration) const {
	assert(iteration < 0 || iteration <= m_history.size());
	if (iteration == m_history.size() || iteration < 0) return m_shootSkeleton;
	return m_history.at(iteration);
}

void TreeModel::ClearHistory() {
	m_history.clear();
}

void TreeModel::Step() {
	m_history.emplace_back(m_shootSkeleton);
	if (m_historyLimit > 0) {
		while (m_history.size() > m_historyLimit) {
			m_history.pop_front();
		}
	}
}

void TreeModel::Pop() {
	m_history.pop_back();
}

int TreeModel::CurrentIteration() const {
	return m_history.size();
}

void TreeModel::Reverse(int iteration) {
	assert(iteration >= 0 && iteration < m_history.size());
	m_shootSkeleton = m_history[iteration];
	m_history.erase((m_history.begin() + iteration), m_history.end());
}

void TreeModel::ExportTreeIOSkeleton(treeio::ArrayTree& arrayTree) const
{
	using namespace treeio;
	const auto& sortedInternodeList = m_shootSkeleton.RefSortedNodeList();
	if (sortedInternodeList.empty()) return;
	const auto& rootNode = m_shootSkeleton.PeekNode(0);
	TreeNodeData rootNodeData;
	//rootNodeData.direction = rootNode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
	rootNodeData.thickness = rootNode.m_info.m_thickness;
	rootNodeData.pos = rootNode.m_info.m_globalPosition;

	auto rootId = arrayTree.addRoot(rootNodeData);
	std::unordered_map<NodeHandle, size_t> nodeMap;
	nodeMap[0] = rootId;
	for (const auto& nodeHandle : sortedInternodeList)
	{
		if (nodeHandle == 0) continue;
		const auto& node = m_shootSkeleton.PeekNode(nodeHandle);
		TreeNodeData nodeData;
		//nodeData.direction = node.m_info.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
		nodeData.thickness = node.m_info.m_thickness;
		nodeData.pos = node.m_info.m_globalPosition;

		auto currentId = arrayTree.addNodeChild(nodeMap[node.GetParentHandle()], nodeData);
		nodeMap[nodeHandle] = currentId;
	}
}

void TreeModel::InitializeProfiles(const PipeModelParameters& pipeModelParameters)
{
	m_shootSkeleton.m_data.m_pipeGroup = {};
	auto& pipeGroup = m_shootSkeleton.m_data.m_pipeGroup;
	const auto& sortedInternodeList = m_shootSkeleton.RefSortedNodeList();
	for (const auto& internodeHandle : sortedInternodeList)
	{
		auto& internode = m_shootSkeleton.RefNode(internodeHandle);
		auto& frontPhysics2D = internode.m_data.m_frontProfile;
		auto& backPhysics2D = internode.m_data.m_backProfile;
		frontPhysics2D.m_settings = pipeModelParameters.m_profilePhysicsSettings;
		backPhysics2D.m_settings = pipeModelParameters.m_profilePhysicsSettings;
		if (internode.IsEndNode()) continue;
		frontPhysics2D.Reset(0.001f);
		backPhysics2D.Reset(0.001f);
	}


	for (const auto& internodeHandle : sortedInternodeList)
	{
		auto& internode = m_shootSkeleton.RefNode(internodeHandle);
		auto& frontPhysics2D = internode.m_data.m_frontProfile;
		auto& backPhysics2D = internode.m_data.m_backProfile;
		if (!internode.IsEndNode()) continue;
		std::vector<NodeHandle> parentNodeToRootChain;
		NodeHandle walker = internode.GetParentHandle();
		while (walker >= 0)
		{
			parentNodeToRootChain.emplace_back(walker);
			walker = m_shootSkeleton.PeekNode(walker).GetParentHandle();
		}
		if (frontPhysics2D.RefParticles().empty()) {
			for (int i = 0; i < pipeModelParameters.m_endNodeStrands; i++) {
				const auto pipeHandle = pipeGroup.AllocatePipe();
				for (auto it = parentNodeToRootChain.rbegin(); it != parentNodeToRootChain.rend(); ++it) {
					const auto newPipeSegmentHandle = pipeGroup.Extend(pipeHandle);
					auto& nodeOnChain = m_shootSkeleton.RefNode(*it);
					const auto newStartParticleHandle = nodeOnChain.m_data.m_frontProfile.AllocateParticle();
					auto& newStartParticle = nodeOnChain.m_data.m_frontProfile.RefParticle(newStartParticleHandle);
					newStartParticle.m_data.m_pipeHandle = pipeHandle;
					newStartParticle.m_data.m_pipeSegmentHandle = newPipeSegmentHandle;
					newStartParticle.m_data.m_base = false;
					const auto newEndParticleHandle = nodeOnChain.m_data.m_backProfile.AllocateParticle();
					auto& newEndParticle = nodeOnChain.m_data.m_backProfile.RefParticle(newEndParticleHandle);
					newEndParticle.m_data.m_pipeHandle = pipeHandle;
					newEndParticle.m_data.m_pipeSegmentHandle = newPipeSegmentHandle;
					newEndParticle.m_data.m_base = false;

					auto& newSegment = pipeGroup.RefPipeSegment(newPipeSegmentHandle);
					newSegment.m_data.m_nodeHandle = *it;
					newSegment.m_data.m_frontProfileParticleHandle = newStartParticleHandle;
					newSegment.m_data.m_backProfileParticleHandle = newEndParticleHandle;
				}
				const auto position = glm::diskRand(glm::sqrt(static_cast<float>(pipeModelParameters.m_endNodeStrands)));
				const auto newPipeSegmentHandle = pipeGroup.Extend(pipeHandle);
				const auto newStartParticleHandle = frontPhysics2D.AllocateParticle();
				auto& newStartParticle = frontPhysics2D.RefParticle(newStartParticleHandle);
				newStartParticle.m_data.m_pipeHandle = pipeHandle;
				newStartParticle.m_data.m_pipeSegmentHandle = newPipeSegmentHandle;
				newStartParticle.m_data.m_base = true;
				newStartParticle.SetPosition(position);
				const auto newEndParticleHandle = backPhysics2D.AllocateParticle();
				auto& newEndParticle = backPhysics2D.RefParticle(newEndParticleHandle);
				newEndParticle.m_data.m_pipeHandle = pipeHandle;
				newEndParticle.m_data.m_pipeSegmentHandle = newPipeSegmentHandle;
				newEndParticle.m_data.m_base = true;
				newStartParticle.SetPosition(position);
				auto& newSegment = pipeGroup.RefPipeSegment(newPipeSegmentHandle);
				newSegment.m_data.m_nodeHandle = internodeHandle;
				newSegment.m_data.m_frontProfileParticleHandle = newStartParticleHandle;
				newSegment.m_data.m_backProfileParticleHandle = newEndParticleHandle;
			}
		}
		else
		{
			for (ParticleHandle particleHandle = 0; particleHandle < frontPhysics2D.RefParticles().size(); particleHandle++)
			{
				const auto pipeHandle = pipeGroup.AllocatePipe();
				for (auto it = parentNodeToRootChain.rbegin(); it != parentNodeToRootChain.rend(); ++it) {
					const auto newPipeSegmentHandle = pipeGroup.Extend(pipeHandle);
					auto& nodeOnChain = m_shootSkeleton.RefNode(*it);
					const auto newStartParticleHandle = nodeOnChain.m_data.m_frontProfile.AllocateParticle();
					auto& newStartParticle = nodeOnChain.m_data.m_frontProfile.RefParticle(newStartParticleHandle);
					newStartParticle.m_data.m_pipeHandle = pipeHandle;
					newStartParticle.m_data.m_pipeSegmentHandle = newPipeSegmentHandle;
					newStartParticle.m_data.m_base = false;
					const auto newEndParticleHandle = nodeOnChain.m_data.m_backProfile.AllocateParticle();
					auto& newEndParticle = nodeOnChain.m_data.m_backProfile.RefParticle(newEndParticleHandle);
					newEndParticle.m_data.m_pipeHandle = pipeHandle;
					newEndParticle.m_data.m_pipeSegmentHandle = newPipeSegmentHandle;
					newEndParticle.m_data.m_base = false;

					auto& newSegment = pipeGroup.RefPipeSegment(newPipeSegmentHandle);
					newSegment.m_data.m_nodeHandle = *it;
					newSegment.m_data.m_frontProfileParticleHandle = newStartParticleHandle;
					newSegment.m_data.m_backProfileParticleHandle = newEndParticleHandle;
				}
				const auto newPipeSegmentHandle = pipeGroup.Extend(pipeHandle);
				auto& frontParticle = frontPhysics2D.RefParticle(particleHandle);
				frontParticle.m_data.m_pipeHandle = pipeHandle;
				frontParticle.m_data.m_pipeSegmentHandle = newPipeSegmentHandle;
				frontParticle.m_data.m_base = true;
				auto& backParticle = backPhysics2D.RefParticle(particleHandle);
				backParticle.m_data.m_pipeHandle = pipeHandle;
				backParticle.m_data.m_pipeSegmentHandle = newPipeSegmentHandle;
				backParticle.m_data.m_base = true;
				frontParticle.SetPosition(backParticle.GetPosition());
				auto& newSegment = pipeGroup.RefPipeSegment(newPipeSegmentHandle);
				newSegment.m_data.m_nodeHandle = internodeHandle;
				newSegment.m_data.m_frontProfileParticleHandle = particleHandle;
				newSegment.m_data.m_backProfileParticleHandle = particleHandle;
			}
		}
	}

	for (const auto& internodeHandle : sortedInternodeList)
	{
		auto& internode = m_shootSkeleton.RefNode(internodeHandle);
		internode.m_data.m_frontParticleMap.clear();
		internode.m_data.m_backParticleMap.clear();
		auto& startPhysics2D = internode.m_data.m_frontProfile;

		for (auto& particle : startPhysics2D.RefParticles())
		{
			internode.m_data.m_frontParticleMap.insert({ particle.m_data.m_pipeHandle, particle.GetHandle() });
		}
		auto& endPhysics2D = internode.m_data.m_backProfile;

		for (auto& particle : endPhysics2D.RefParticles())
		{
			internode.m_data.m_backParticleMap.insert({ particle.m_data.m_pipeHandle, particle.GetHandle() });
		}
		internode.m_data.m_pipeSize = startPhysics2D.RefParticles().size();
	}
	m_shootSkeleton.m_data.m_numOfParticles = 0;
	for (const auto& internodeHandle : sortedInternodeList)
	{
		auto& internode = m_shootSkeleton.RefNode(internodeHandle);
		int maxChildSize = 0;
		for (const auto& childHandle : internode.RefChildHandles())
		{
			auto& childInternode = m_shootSkeleton.RefNode(childHandle);
			const auto childSize = static_cast<float>(childInternode.m_data.m_frontParticleMap.size());
			if (childSize > maxChildSize)
			{
				maxChildSize = childSize;
			}
		}
		for (const auto& childHandle : internode.RefChildHandles())
		{
			auto& childInternode = m_shootSkeleton.RefNode(childHandle);
			if (childInternode.m_data.m_maxChild) childInternode.m_data.m_split = false;
			const auto childSize = static_cast<float>(childInternode.m_data.m_frontParticleMap.size());
			if (childSize > maxChildSize * pipeModelParameters.m_overlapThreshold)
			{
				childInternode.m_data.m_split = true;
			}
			else
			{
				childInternode.m_data.m_split = false;
			}
		}
		m_shootSkeleton.m_data.m_numOfParticles += internode.m_data.m_frontProfile.PeekParticles().size();
	}
}

void TreeModel::CalculateProfiles(const PipeModelParameters& pipeModelParameters)
{
	const float time = Times::Now();
	const auto& sortedInternodeList = m_shootSkeleton.RefSortedNodeList();

	Jobs::ParallelFor(sortedInternodeList.size(), [&](unsigned i)
		{
			auto& internode = m_shootSkeleton.RefNode(sortedInternodeList[i]);
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

	if (m_shootSkeleton.m_data.m_parallelScheduling) {
		for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); ++it)
		{
			CalculateProfile(*it, pipeModelParameters, true);
			m_shootSkeleton.m_data.m_numOfParticles += m_shootSkeleton.RefNode(*it).m_data.m_frontProfile.PeekParticles().size();
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
			CalculateProfile(*it, pipeModelParameters, false);
			Wait(*it);
			m_shootSkeleton.m_data.m_numOfParticles += m_shootSkeleton.RefNode(*it).m_data.m_frontProfile.PeekParticles().size();
		}
	}
	m_shootSkeleton.m_data.m_profileCalculationTime = Times::Now() - time;
}

void TreeModel::CalculateProfile(const NodeHandle nodeHandle, const PipeModelParameters& pipeModelParameters, bool scheduling)
{
	if (scheduling) {
		m_shootSkeleton.RefNode(nodeHandle).m_data.m_tasks.emplace_back(Jobs::AddTask([&, nodeHandle, scheduling](unsigned threadIndex) {
			MergeTask(nodeHandle, pipeModelParameters);
			auto& internode = m_shootSkeleton.RefNode(nodeHandle);
			if (internode.m_data.m_frontProfile.PeekParticles().size() > 1) {
				PackTask(nodeHandle, pipeModelParameters, !scheduling);
				if (internode.RefChildHandles().empty()) CopyFrontToBackTask(nodeHandle);
				CalculateShiftTask(nodeHandle, pipeModelParameters);
			}
			internode.m_data.m_frontProfile.CalculateBoundaries(pipeModelParameters.m_boundaryPointDistance);
			internode.m_data.m_backProfile.CalculateBoundaries(pipeModelParameters.m_boundaryPointDistance);
			}
		)
		);
	}
	else
	{
		MergeTask(nodeHandle, pipeModelParameters);
		auto& internode = m_shootSkeleton.RefNode(nodeHandle);
		if (internode.m_data.m_frontProfile.PeekParticles().size() > 1) {
			PackTask(nodeHandle, pipeModelParameters, !scheduling);
			if (internode.RefChildHandles().empty()) CopyFrontToBackTask(nodeHandle);
			CalculateShiftTask(nodeHandle, pipeModelParameters);
		}
		internode.m_data.m_frontProfile.CalculateBoundaries(pipeModelParameters.m_boundaryPointDistance);
		internode.m_data.m_backProfile.CalculateBoundaries(pipeModelParameters.m_boundaryPointDistance);
	}
}

void TreeModel::Wait(const NodeHandle nodeHandle)
{
	auto& internode = m_shootSkeleton.RefNode(nodeHandle);
	if (internode.m_data.m_tasks.empty()) return;
	for (const auto& i : internode.m_data.m_tasks)
	{
		i.wait();
	}
	internode.m_data.m_tasks.clear();
}

void TreeModel::PackTask(NodeHandle nodeHandle, const PipeModelParameters& pipeModelParameters, bool parallel)
{
	auto& internode = m_shootSkeleton.RefNode(nodeHandle);
	auto& internodeData = internode.m_data;
	internodeData.m_frontProfile.m_parallel = parallel;

	const auto iterations = internodeData.m_packingIteration;

	int timeout = pipeModelParameters.m_junctionProfilePackingMaxIteration;
	if (!internodeData.m_profileConstraints.m_boundaries.empty()) timeout = pipeModelParameters.m_modifiedProfilePackingMaxIteration;
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
						acceleration += pipeModelParameters.m_centerAttractionStrength * glm::normalize(cell.m_target);
					}
				}
				particle.SetAcceleration(acceleration);
			}
		);
		if (timeout > 0 && i > timeout) break;
	}
}

void TreeModel::MergeTask(NodeHandle nodeHandle, const PipeModelParameters& pipeModelParameters)
{
	auto& internode = m_shootSkeleton.RefNode(nodeHandle);
	auto& internodeData = internode.m_data;
	const auto& childHandles = internode.RefChildHandles();
	NodeHandle mainChildHandle = -1;
	for (const auto& childHandle : childHandles) {
		Wait(childHandle);
		if (m_shootSkeleton.PeekNode(childHandle).m_data.m_maxChild) mainChildHandle = childHandle;
	}
	internodeData.m_centerDirectionRadius = 0.0f;

	if (childHandles.empty())
	{
		internode.m_data.m_packingIteration = glm::min(pipeModelParameters.m_junctionProfilePackingMaxIteration, static_cast<int>(internodeData.m_frontProfile.RefParticles().size()) * pipeModelParameters.m_maxSimulationIterationCellFactor);
		int particleIndex = 0;
		for (const auto& particle : internodeData.m_frontProfile.RefParticles())
		{
			const auto nodeStartParticleHandle = internodeData.m_frontParticleMap.at(particle.m_data.m_pipeHandle);
			const auto nodeEndParticleHandle = internodeData.m_backParticleMap.at(particle.m_data.m_pipeHandle);
			auto& nodeStartParticle = internodeData.m_frontProfile.RefParticle(nodeStartParticleHandle);
			auto& nodeEndParticle = internodeData.m_backProfile.RefParticle(nodeEndParticleHandle);
			particleIndex++;
			nodeStartParticle.SetColor(particle.GetColor());
			nodeEndParticle.SetColor(particle.GetColor());
			nodeStartParticle.m_data.m_mainChild = nodeEndParticle.m_data.m_mainChild = true;
		}
		return;
	}
	if (!internode.m_data.m_profileConstraints.m_boundaries.empty())
	{
		internode.m_data.m_packingIteration = glm::min(pipeModelParameters.m_modifiedProfilePackingMaxIteration, static_cast<int>(internodeData.m_frontProfile.RefParticles().size()) * pipeModelParameters.m_maxSimulationIterationCellFactor);
	}
	if (mainChildHandle == -1) mainChildHandle = childHandles.front();
	auto& mainChildNode = m_shootSkeleton.RefNode(mainChildHandle);
	if (childHandles.size() == 1)
	{
		//Copy from child flow start to self flow start
		const auto& childNode = m_shootSkeleton.RefNode(childHandles.front());
		const auto& childPhysics2D = childNode.m_data.m_frontProfile;
		for (const auto& childParticle : childPhysics2D.PeekParticles())
		{
			const auto nodeStartParticleHandle = internodeData.m_frontParticleMap.at(childParticle.m_data.m_pipeHandle);
			const auto nodeEndParticleHandle = internodeData.m_backParticleMap.at(childParticle.m_data.m_pipeHandle);
			auto& nodeStartParticle = internodeData.m_frontProfile.RefParticle(nodeStartParticleHandle);
			auto& nodeEndParticle = internodeData.m_backProfile.RefParticle(nodeEndParticleHandle);
			nodeStartParticle.SetColor(childParticle.GetColor());
			nodeEndParticle.SetColor(childParticle.GetColor());
			auto polarPosition = childParticle.GetPolarPosition();
			polarPosition.y += glm::radians(pipeModelParameters.m_branchTwistAngle);
			nodeStartParticle.SetPolarPosition(polarPosition);
			nodeEndParticle.SetPolarPosition(polarPosition);

			nodeStartParticle.m_data.m_mainChild = nodeEndParticle.m_data.m_mainChild = true;
		}
		internode.m_data.m_packingIteration = glm::min(pipeModelParameters.m_branchProfilePackingMaxIteration, static_cast<int>(internodeData.m_frontProfile.RefParticles().size()) * pipeModelParameters.m_maxSimulationIterationCellFactor);
		return;
	}
	internode.m_data.m_packingIteration = glm::min(pipeModelParameters.m_junctionProfilePackingMaxIteration, static_cast<int>(internodeData.m_frontProfile.RefParticles().size()) * pipeModelParameters.m_maxSimulationIterationCellFactor);
	const auto& mainChildPhysics2D = mainChildNode.m_data.m_frontProfile;
	for (const auto& mainChildParticle : mainChildPhysics2D.PeekParticles())
	{
		const auto nodeStartParticleHandle = internodeData.m_frontParticleMap.at(mainChildParticle.m_data.m_pipeHandle);
		const auto nodeEndParticleHandle = internodeData.m_backParticleMap.at(mainChildParticle.m_data.m_pipeHandle);
		auto& nodeStartParticle = internodeData.m_frontProfile.RefParticle(nodeStartParticleHandle);
		auto& nodeEndParticle = internodeData.m_backProfile.RefParticle(nodeEndParticleHandle);
		nodeStartParticle.SetColor(mainChildParticle.GetColor());
		nodeEndParticle.SetColor(mainChildParticle.GetColor());
		auto polarPosition = mainChildParticle.GetPolarPosition();
		polarPosition.y += glm::radians(pipeModelParameters.m_junctionTwistAngle);
		nodeStartParticle.SetPolarPosition(polarPosition);
		nodeEndParticle.SetPolarPosition(polarPosition);

		nodeStartParticle.m_data.m_mainChild = nodeEndParticle.m_data.m_mainChild = true;
	}
	//const auto nodeGlobalRotation = scene->GetDataComponent<GlobalTransform>(owner);
	if (pipeModelParameters.m_preMerge) {
		bool needSimulation = false;
		for (const auto& childHandle : childHandles)
		{
			if (childHandle == mainChildHandle) continue;
			auto& childNode = m_shootSkeleton.RefNode(childHandle);
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

					const auto nodeStartParticleHandle = internodeData.m_frontParticleMap.at(childParticle.m_data.m_pipeHandle);
					const auto nodeEndParticleHandle = internodeData.m_backParticleMap.at(childParticle.m_data.m_pipeHandle);
					auto& nodeStartParticle = internodeData.m_frontProfile.RefParticle(nodeStartParticleHandle);
					auto& nodeEndParticle = internodeData.m_backProfile.RefParticle(nodeEndParticleHandle);
					nodeStartParticle.SetColor(childParticle.GetColor());
					nodeEndParticle.SetColor(childParticle.GetColor());
					auto polarPosition = childParticle.GetPolarPosition();
					polarPosition.y += glm::radians(pipeModelParameters.m_junctionTwistAngle);
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
					const auto nodeStartParticleHandle = internodeData.m_frontParticleMap.at(childParticle.m_data.m_pipeHandle);
					const auto nodeEndParticleHandle = internodeData.m_backParticleMap.at(childParticle.m_data.m_pipeHandle);
					auto& nodeStartParticle = internodeData.m_frontProfile.RefParticle(nodeStartParticleHandle);
					auto& nodeEndParticle = internodeData.m_backProfile.RefParticle(nodeEndParticleHandle);
					nodeStartParticle.SetColor(childParticle.GetColor());
					nodeEndParticle.SetColor(childParticle.GetColor());
					auto polarPosition = childParticle.GetPolarPosition();
					polarPosition.y += glm::radians(pipeModelParameters.m_junctionTwistAngle);
					nodeStartParticle.SetPolarPosition(polarPosition);
					nodeEndParticle.SetPolarPosition(polarPosition);
					nodeStartParticle.m_enable = false;
					nodeStartParticle.m_data.m_mainChild = nodeEndParticle.m_data.m_mainChild = false;
				}
			}
		}
		if (needSimulation) PackTask(nodeHandle, pipeModelParameters, false);
		for (const auto& childHandle : childHandles)
		{
			if (childHandle == mainChildHandle) continue;
			auto& childNode = m_shootSkeleton.RefNode(childHandle);
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
					polarPosition.y += glm::radians(pipeModelParameters.m_junctionTwistAngle);
					childParticle.SetPolarPosition(polarPosition);
					childParticle.SetPosition(childParticle.GetPosition() + offset);

					const auto nodeStartParticleHandle = internodeData.m_frontParticleMap.at(childParticle.m_data.m_pipeHandle);
					const auto nodeEndParticleHandle = internodeData.m_backParticleMap.at(childParticle.m_data.m_pipeHandle);
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
			if (childHandle == mainChildHandle) continue;
			auto& childNode = m_shootSkeleton.RefNode(childHandle);
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

				const auto nodeStartParticleHandle = internodeData.m_frontParticleMap.at(childParticle.m_data.m_pipeHandle);
				const auto nodeEndParticleHandle = internodeData.m_backParticleMap.at(childParticle.m_data.m_pipeHandle);
				auto& nodeStartParticle = internodeData.m_frontProfile.RefParticle(nodeStartParticleHandle);
				auto& nodeEndParticle = internodeData.m_backProfile.RefParticle(nodeEndParticleHandle);
				nodeStartParticle.SetColor(childParticle.GetColor());
				nodeEndParticle.SetColor(childParticle.GetColor());
				auto polarPosition = childParticle.GetPolarPosition();
				polarPosition.y += glm::radians(pipeModelParameters.m_junctionTwistAngle);
				nodeStartParticle.SetPolarPosition(polarPosition);
				nodeEndParticle.SetPolarPosition(polarPosition);

				nodeStartParticle.m_data.m_mainChild = nodeEndParticle.m_data.m_mainChild = false;
			}
		}
	}
}

void TreeModel::CopyFrontToBackTask(NodeHandle nodeHandle)
{
	auto& internode = m_shootSkeleton.RefNode(nodeHandle);
	auto& internodeData = internode.m_data;
	for (int i = 0; i < internodeData.m_frontProfile.RefParticles().size(); i++)
	{
		internodeData.m_backProfile.RefParticle(i).SetPosition(internodeData.m_frontProfile.RefParticle(i).GetPosition());
	}
}

void TreeModel::CalculateShiftTask(NodeHandle nodeHandle, const PipeModelParameters& pipeModelParameters)
{
	auto& internode = m_shootSkeleton.RefNode(nodeHandle);
	auto& internodeData = internode.m_data;
	const auto& childHandles = internode.RefChildHandles();
	NodeHandle mainChildHandle = -1;
	for (const auto& childHandle : childHandles) {
		Wait(childHandle);
		if (m_shootSkeleton.PeekNode(childHandle).m_data.m_maxChild) mainChildHandle = childHandle;
	}
	internodeData.m_centerDirectionRadius = 0.0f;
	if (childHandles.empty())
	{
		internode.m_data.m_packingIteration = glm::min(pipeModelParameters.m_junctionProfilePackingMaxIteration, static_cast<int>(internodeData.m_frontProfile.RefParticles().size()) * pipeModelParameters.m_maxSimulationIterationCellFactor);
		return;
	}
	if (mainChildHandle == -1) mainChildHandle = childHandles.front();
	const auto& mainChildNode = m_shootSkeleton.RefNode(mainChildHandle);
	const auto& mainChildPhysics2D = mainChildNode.m_data.m_frontProfile;
	auto sum = glm::vec2(0.0f);
	for (const auto& mainChildParticle : mainChildPhysics2D.PeekParticles())
	{
		const auto nodeStartParticleHandle = internodeData.m_frontParticleMap.at(mainChildParticle.m_data.m_pipeHandle);
		auto& nodeStartParticle = internodeData.m_frontProfile.RefParticle(nodeStartParticleHandle);
		sum += nodeStartParticle.GetPosition();
	}
	internodeData.m_shift = sum / static_cast<float>(mainChildPhysics2D.PeekParticles().size());

}

void TreeModel::ApplyProfile(const PipeModelParameters& pipeModelParameters,
	const glm::vec3& globalPosition, const glm::quat& globalRotation,
	const PipeProfile<CellParticlePhysicsData>& profile,
	const std::unordered_map<PipeHandle, ParticleHandle>& map, float pipeRadius)
{
	const auto currentUp = globalRotation * glm::vec3(0, 1, 0);
	const auto currentLeft = globalRotation * glm::vec3(1, 0, 0);
	const auto& parameters = pipeModelParameters;
	for (const auto& [pipeHandle, particleHandle] : map)
	{
		const auto& particle = profile.PeekParticle(particleHandle);
		auto& newPipeSegment = m_shootSkeleton.m_data.m_pipeGroup.RefPipeSegment(particle.m_data.m_pipeSegmentHandle);
		newPipeSegment.m_info.m_thickness = pipeRadius;
		newPipeSegment.m_info.m_globalPosition = globalPosition
			+ pipeRadius * particle.GetPosition().x * currentLeft
			+ pipeRadius * particle.GetPosition().y * currentUp;
		newPipeSegment.m_info.m_globalRotation = globalRotation;
		newPipeSegment.m_info.m_color = particle.IsBoundary() ? parameters.m_boundaryPointColor : parameters.m_contentPointColor;
		newPipeSegment.m_info.m_isBoundary = particle.IsBoundary();
	}
}

void TreeModel::ApplyProfiles(const PipeModelParameters& pipeModelParameters)
{
	auto& pipeGroup = m_shootSkeleton.m_data.m_pipeGroup;
	const auto& sortedInternodeList = m_shootSkeleton.RefSortedNodeList();
	for (const auto& nodeHandle : sortedInternodeList)
	{
		const auto& node = m_shootSkeleton.RefNode(nodeHandle);
		glm::quat parentGlobalRotation;
		glm::vec3 parentGlobalPosition;
		if (node.GetParentHandle() == -1)
		{
			parentGlobalRotation = node.m_data.m_adjustedGlobalRotation;
			parentGlobalPosition = glm::vec3(0.0f);
		}
		else {
			const auto& parent = m_shootSkeleton.RefNode(node.GetParentHandle());
			parentGlobalRotation = parent.m_data.m_adjustedGlobalRotation;
			parentGlobalPosition = parent.m_data.m_adjustedGlobalPosition;
		}
		if (node.GetParentHandle() == -1)
		{
			const auto currentUp = parentGlobalRotation * glm::vec3(0, 1, 0);
			const auto currentLeft = parentGlobalRotation * glm::vec3(1, 0, 0);
			const auto baseRadius = pipeModelParameters.m_pipeRadiusDistribution.GetValue(0.0f);
			for (const auto& [pipeHandle, particleHandle] : node.m_data.m_frontParticleMap)
			{
				const auto& particle = node.m_data.m_frontProfile.PeekParticle(particleHandle);
				auto& pipe = pipeGroup.RefPipe(pipeHandle);
				pipe.m_info.m_baseInfo.m_thickness = baseRadius;
				pipe.m_info.m_baseInfo.m_globalPosition = parentGlobalPosition
					+ pipe.m_info.m_baseInfo.m_thickness * particle.GetPosition().x * currentLeft
					+ pipe.m_info.m_baseInfo.m_thickness * particle.GetPosition().y * currentUp;
				pipe.m_info.m_baseInfo.m_globalRotation = parentGlobalRotation;
				pipe.m_info.m_baseInfo.m_isBoundary = particle.IsBoundary();
				pipe.m_info.m_baseInfo.m_color = particle.IsBoundary() ? pipeModelParameters.m_boundaryPointColor : pipeModelParameters.m_contentPointColor;
			}
		}
		ApplyProfile(pipeModelParameters, node.m_data.m_adjustedGlobalPosition, node.m_data.m_adjustedGlobalRotation, node.m_data.m_backProfile, node.m_data.m_backParticleMap, node.m_data.m_pipeCellRadius);
	}
}

void TreeModel::CalculatePipeProfileAdjustedTransforms(const PipeModelParameters& pipeModelParameters)
{
	const auto& sortedInternodeList = m_shootSkeleton.RefSortedNodeList();
	float maxRootDistance = 0.0f;
	for (const auto& nodeHandle : sortedInternodeList)
	{
		auto& node = m_shootSkeleton.RefNode(nodeHandle);
		const auto parentHandle = node.GetParentHandle();
		if (parentHandle == -1)
		{
			node.m_data.m_adjustedGlobalPosition = node.m_info.GetGlobalEndPosition();
			node.m_data.m_adjustedGlobalRotation = node.m_info.m_regulatedGlobalRotation;
			maxRootDistance = node.m_info.m_endDistance + node.m_info.m_length;

			node.m_data.m_pipeCellRadius = pipeModelParameters.m_pipeRadiusDistribution.GetValue(node.m_info.m_rootDistance / maxRootDistance);
			continue;
		}
		const auto& parentNode = m_shootSkeleton.PeekNode(parentHandle);
		glm::vec3 parentGlobalPosition = parentNode.m_data.m_adjustedGlobalPosition;
		glm::quat parentGlobalRotation = parentNode.m_data.m_adjustedGlobalRotation;
		node.m_data.m_pipeCellRadius = pipeModelParameters.m_pipeRadiusDistribution.GetValue(node.m_info.m_rootDistance / maxRootDistance);
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
			const auto newOffset = (offsetLength + innerRadius + (outerRadius - outerRadius * cosFront) * pipeModelParameters.m_rotationPushRatio) * offsetDirection;
			node.m_data.m_adjustedGlobalPosition += parentUp * newOffset.y * pipeModelParameters.m_sidePushRatio * node.m_data.m_pipeCellRadius;
			node.m_data.m_adjustedGlobalPosition += parentLeft * newOffset.x * pipeModelParameters.m_sidePushRatio * node.m_data.m_pipeCellRadius;
			node.m_data.m_adjustedGlobalPosition += parentFront * (sinFront * outerRadius * pipeModelParameters.m_rotationPushRatio) * node.m_data.m_pipeCellRadius;
		}
		node.m_data.m_adjustedGlobalPosition += parentUp * node.m_data.m_shift.y * pipeModelParameters.m_shiftPushRatio * node.m_data.m_pipeCellRadius;
		node.m_data.m_adjustedGlobalPosition += parentLeft * node.m_data.m_shift.x * pipeModelParameters.m_shiftPushRatio * node.m_data.m_pipeCellRadius;
		node.m_data.m_adjustedGlobalPosition += parentFront * sinFront * maxDistanceToCenter * pipeModelParameters.m_frontPushRatio * node.m_data.m_pipeCellRadius;
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

glm::vec3 TreeModel::InterpolatePipeSegmentPosition(const PipeSegmentHandle pipeSegmentHandle, const float a) const
{
	assert(pipeSegmentHandle >= 0);
	assert(a >= 0.f && a <= 1.f);
	const auto& pipeGroup = m_shootSkeleton.m_data.m_pipeGroup;
	assert(pipeGroup.PeekPipeSegments().size() > pipeSegmentHandle);
	const auto& pipeSegment = pipeGroup.PeekPipeSegment(pipeSegmentHandle);
	const auto& pipe = pipeGroup.PeekPipe(pipeSegment.GetPipeHandle());
	auto& baseInfo = pipe.m_info.m_baseInfo;

	const auto& pipeSegmentHandles = pipe.PeekPipeSegmentHandles();

	glm::vec3 p[4];

	p[2] = pipeSegment.m_info.m_globalPosition;
	if (pipeSegmentHandle == pipeSegmentHandles.front())
	{
		p[1] = baseInfo.m_globalPosition;
		p[0] = p[1] * 2.0f - p[2];
	}
	else if (pipeSegmentHandle == pipeSegmentHandles.at(1))
	{
		p[0] = baseInfo.m_globalPosition;
		p[1] = pipeGroup.PeekPipeSegment(pipeSegmentHandles.front()).m_info.m_globalPosition;
	}
	else
	{
		const auto& prevSegment = pipeGroup.PeekPipeSegment(pipeSegment.GetPrevHandle());
		p[1] = prevSegment.m_info.m_globalPosition;
		const auto& prevPrevSegment = pipeGroup.PeekPipeSegment(prevSegment.GetPrevHandle());
		p[0] = prevPrevSegment.m_info.m_globalPosition;
	}
	if (pipeSegmentHandle == pipeSegmentHandles.back())
	{
		p[3] = p[2] * 2.0f - p[1];
	}
	else
	{
		const auto& nextSegment = pipeGroup.PeekPipeSegment(pipeSegment.GetNextHandle());
		p[3] = nextSegment.m_info.m_globalPosition;
	}
	glm::vec3 position, tangent;
	Strands::CubicInterpolation(p[0], p[1], p[2], p[3], position, tangent, a);
	return position;
}

glm::vec3 TreeModel::InterpolatePipeSegmentAxis(PipeSegmentHandle pipeSegmentHandle, float a) const
{
	assert(pipeSegmentHandle >= 0);
	assert(a >= 0.f && a <= 1.f);
	const auto& pipeGroup = m_shootSkeleton.m_data.m_pipeGroup;
	assert(pipeGroup.PeekPipeSegments().size() > pipeSegmentHandle);
	const auto& pipeSegment = pipeGroup.PeekPipeSegment(pipeSegmentHandle);
	const auto& pipe = pipeGroup.PeekPipe(pipeSegment.GetPipeHandle());
	auto& baseInfo = pipe.m_info.m_baseInfo;

	const auto& pipeSegmentHandles = pipe.PeekPipeSegmentHandles();

	glm::vec3 p[4];

	p[2] = pipeSegment.m_info.m_globalPosition;
	if (pipeSegmentHandle == pipeSegmentHandles.front())
	{
		p[1] = baseInfo.m_globalPosition;
		p[0] = p[1] * 2.0f - p[2];
	}
	else if (pipeSegmentHandle == pipeSegmentHandles.at(1))
	{
		p[0] = baseInfo.m_globalPosition;
		p[1] = pipeGroup.PeekPipeSegment(pipeSegmentHandles.front()).m_info.m_globalPosition;
	}
	else
	{
		const auto& prevSegment = pipeGroup.PeekPipeSegment(pipeSegment.GetPrevHandle());
		p[1] = prevSegment.m_info.m_globalPosition;
		const auto& prevPrevSegment = pipeGroup.PeekPipeSegment(prevSegment.GetPrevHandle());
		p[0] = prevPrevSegment.m_info.m_globalPosition;
	}
	if (pipeSegmentHandle == pipeSegmentHandles.back())
	{
		p[3] = p[2] * 2.0f - p[1];
	}
	else
	{
		const auto& nextSegment = pipeGroup.PeekPipeSegment(pipeSegment.GetNextHandle());
		p[3] = nextSegment.m_info.m_globalPosition;
	}
	glm::vec3 position, tangent;
	Strands::CubicInterpolation(p[0], p[1], p[2], p[3], position, tangent, a);
	return tangent;
}

float TreeModel::InterpolatePipeSegmentRadius(PipeSegmentHandle pipeSegmentHandle, float a) const
{
	assert(pipeSegmentHandle >= 0);
	assert(a >= 0.f && a <= 1.f);
	const auto& pipeGroup = m_shootSkeleton.m_data.m_pipeGroup;
	assert(pipeGroup.PeekPipeSegments().size() > pipeSegmentHandle);
	const auto& pipeSegment = pipeGroup.PeekPipeSegment(pipeSegmentHandle);
	const auto& pipe = pipeGroup.PeekPipe(pipeSegment.GetPipeHandle());
	auto& baseInfo = pipe.m_info.m_baseInfo;

	const auto& pipeSegmentHandles = pipe.PeekPipeSegmentHandles();

	float p[4];

	p[2] = pipeSegment.m_info.m_thickness;
	if (pipeSegmentHandle == pipeSegmentHandles.front())
	{
		p[1] = baseInfo.m_thickness;
		p[0] = p[1] * 2.0f - p[2];
	}
	else if (pipeSegmentHandle == pipeSegmentHandles.at(1))
	{
		p[0] = baseInfo.m_thickness;
		p[1] = pipeGroup.PeekPipeSegment(pipeSegmentHandles.front()).m_info.m_thickness;
	}
	else
	{
		const auto& prevSegment = pipeGroup.PeekPipeSegment(pipeSegment.GetPrevHandle());
		p[1] = prevSegment.m_info.m_thickness;
		const auto& prevPrevSegment = pipeGroup.PeekPipeSegment(prevSegment.GetPrevHandle());
		p[0] = prevPrevSegment.m_info.m_thickness;
	}
	if (pipeSegmentHandle == pipeSegmentHandles.back())
	{
		p[3] = p[2] * 2.0f - p[1];
	}
	else
	{
		const auto& nextSegment = pipeGroup.PeekPipeSegment(pipeSegment.GetNextHandle());
		p[3] = nextSegment.m_info.m_thickness;
	}
	float radius, tangent;
	Strands::CubicInterpolation(p[0], p[1], p[2], p[3], radius, tangent, a);
	return radius;
}
#pragma endregion
