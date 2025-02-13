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
	const auto& sortedInternodeList = m_shootSkeleton.PeekSortedNodeList();
	for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); ++it) {
		auto& internode = m_shootSkeleton.RefNode(*it);
		auto& internodeData = internode.m_data;
		auto& buds = internodeData.m_buds;
		for (auto& bud : buds)
		{
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
	const auto& sortedInternodeList = m_shootSkeleton.PeekSortedNodeList();
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
			registration.m_treeSkeletonIndex = m_shootSkeleton.m_data.m_index;
			registration.m_thickness = internodeInfo.m_thickness;
			environmentGrid.AddNode(registration);
		}
	}
}

void TreeModel::PruneInternode(const SkeletonNodeHandle internodeHandle)
{
	const auto& internode = m_shootSkeleton.RefNode(internodeHandle);

	if (!internode.IsRecycled() && !internode.m_info.m_locked) {
		auto& parentNode = m_shootSkeleton.RefNode(internode.GetParentHandle());
		parentNode.m_info.m_wounds.emplace_back();
		auto& wound = parentNode.m_info.m_wounds.back();
		wound.m_apical = internode.IsApical();
		wound.m_thickness = internode.m_info.m_thickness;
		wound.m_healing = 0.f;
		wound.m_localRotation = glm::inverse(parentNode.m_info.m_globalRotation) * internode.m_info.m_globalRotation;

		m_shootSkeleton.RecycleNode(internodeHandle,
			[&](SkeletonFlowHandle flowHandle) {},
			[&](SkeletonNodeHandle nodeHandle)
			{

			});
	}
}

void TreeModel::HarvestFruits(const std::function<bool(const ReproductiveModule& fruit)>& harvestFunction)
{
	const auto& sortedInternodeList = m_shootSkeleton.PeekSortedNodeList();
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

bool TreeModel::Grow(float deltaTime, const glm::mat4& globalTransform, ClimateModel& climateModel,
	const ShootGrowthController& shootGrowthController, const bool pruning)
{
	m_currentDeltaTime = deltaTime;
	m_age += m_currentDeltaTime;
	bool treeStructureChanged = false;
	if (!m_initialized) {
		Initialize(shootGrowthController);
		treeStructureChanged = true;
	}
	m_shootSkeleton.SortLists();
	CalculateShootFlux(globalTransform, climateModel, shootGrowthController);
	SampleTemperature(globalTransform, climateModel);

	if (pruning) {
		const bool anyBranchPruned = PruneInternodes(globalTransform, climateModel, shootGrowthController);
		if (anyBranchPruned) m_shootSkeleton.SortLists();
		treeStructureChanged = treeStructureChanged || anyBranchPruned;
	}
	bool anyBranchGrown = false;
	{
		const auto& sortedNodeList = m_shootSkeleton.PeekSortedNodeList();
		if (!m_treeGrowthSettings.m_useSpaceColonization) {
			const auto totalShootFlux = CollectShootFlux(sortedNodeList);
			RootFlux totalRootFlux;
			totalRootFlux.m_value = totalShootFlux.m_value;
			const auto totalFlux = glm::min(totalShootFlux.m_value, totalRootFlux.m_value);
			CalculateInternodeStrength(sortedNodeList, shootGrowthController);
			const float requiredVigor = CalculateGrowthPotential(sortedNodeList, shootGrowthController);
			CalculateGrowthRate(sortedNodeList, totalFlux / requiredVigor);
		}
		for (auto it = sortedNodeList.rbegin(); it != sortedNodeList.rend(); ++it) {
			const bool graphChanged = GrowInternode(climateModel, *it, shootGrowthController);
			anyBranchGrown = anyBranchGrown || graphChanged;
		}
	}

	if (anyBranchGrown) {
		m_shootSkeleton.SortLists();
	}
	{
		const auto& sortedNodeList = m_shootSkeleton.PeekSortedNodeList();
		for (auto it = sortedNodeList.rbegin(); it != sortedNodeList.rend(); ++it) {
			const bool reproductiveModuleChanged = GrowReproductiveModules(climateModel, *it, shootGrowthController);
			anyBranchGrown = anyBranchGrown || reproductiveModuleChanged;
		}
	}
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

bool TreeModel::Grow(const float deltaTime, const SkeletonNodeHandle baseInternodeHandle, const glm::mat4& globalTransform, ClimateModel& climateModel,
	const ShootGrowthController& shootGrowthController,
	const bool pruning)
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
	auto sortedSubTreeInternodeList = m_shootSkeleton.GetSubTree(baseInternodeHandle);

	if (!m_treeGrowthSettings.m_useSpaceColonization) {
		const auto totalShootFlux = CollectShootFlux(sortedSubTreeInternodeList);
		RootFlux totalRootFlux;
		totalRootFlux.m_value = totalShootFlux.m_value;
		const auto totalFlux = glm::min(totalShootFlux.m_value, totalRootFlux.m_value);
		const float requiredVigor = CalculateGrowthPotential(sortedSubTreeInternodeList, shootGrowthController);
		CalculateGrowthRate(sortedSubTreeInternodeList, totalFlux / requiredVigor);
	}
	for (auto it = sortedSubTreeInternodeList.rbegin(); it != sortedSubTreeInternodeList.rend(); ++it) {
		const bool graphChanged = GrowInternode(climateModel, *it, shootGrowthController);
		anyBranchGrown = anyBranchGrown || graphChanged;
	}
	if (anyBranchGrown) {
		m_shootSkeleton.SortLists();
		sortedSubTreeInternodeList = m_shootSkeleton.GetSubTree(baseInternodeHandle);
	}
	for (auto it = sortedSubTreeInternodeList.rbegin(); it != sortedSubTreeInternodeList.rend(); ++it) {
		const bool reproductiveModuleChanged = GrowReproductiveModules(climateModel, *it, shootGrowthController);
		anyBranchGrown = anyBranchGrown || reproductiveModuleChanged;
	}
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
		m_shootSkeleton = ShootSkeleton(shootGrowthController.m_baseInternodeCount);
		m_shootSkeleton.SortLists();
		for (const auto& nodeHandle : m_shootSkeleton.PeekSortedNodeList())
		{
			auto& node = m_shootSkeleton.RefNode(nodeHandle);
			node.m_info.m_thickness = shootGrowthController.m_endNodeThickness;
			node.m_data.m_internodeLength = 0.0f;
			node.m_data.m_buds.emplace_back();
			auto& apicalBud = node.m_data.m_buds.back();
			apicalBud.m_type = BudType::Apical;
			apicalBud.m_status = BudStatus::Dormant;
			apicalBud.m_localRotation = glm::vec3(
				glm::radians(shootGrowthController.m_baseNodeApicalAngle(node)), 0.0f,
				glm::radians(glm::linearRand(0.f, 360.f)));
		}
	}

	if (m_treeGrowthSettings.m_useSpaceColonization && m_treeGrowthSettings.m_spaceColonizationAutoResize) {
		const auto gridRadius = m_treeGrowthSettings.m_spaceColonizationDetectionDistanceFactor * shootGrowthController.m_internodeLength;
		m_treeOccupancyGrid.Initialize(glm::vec3(-gridRadius, 0.0f, -gridRadius), glm::vec3(gridRadius), shootGrowthController.m_internodeLength,
			m_treeGrowthSettings.m_spaceColonizationRemovalDistanceFactor, m_treeGrowthSettings.m_spaceColonizationTheta, m_treeGrowthSettings.m_spaceColonizationDetectionDistanceFactor);
	}

	m_currentSeedValue = m_seed;
	m_initialized = true;
}

void TreeModel::CalculateShootFlux(const glm::mat4& globalTransform, const ClimateModel& climateModel, const ShootGrowthController& shootGrowthController)
{
	auto& shootData = m_shootSkeleton.m_data;
	shootData.m_maxMarkerCount = 0;
	const auto& sortedInternodeList = m_shootSkeleton.PeekSortedNodeList();
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
		internodeData.m_lightDirection = -m_currentGravityDirection;
		bool sampleLightIntensity = false;

		for (const auto& bud : internodeData.m_buds)
		{
			sampleLightIntensity = true;
			if (m_treeGrowthSettings.m_useSpaceColonization) {
				shootData.m_maxMarkerCount = glm::max(shootData.m_maxMarkerCount, bud.m_markerCount);
			}
		}
		const glm::vec3 position = globalTransform * glm::vec4(internodeInfo.m_globalPosition, 1.0f);
		if (sampleLightIntensity) {

			internodeData.m_lightIntensity = glm::clamp(climateModel.m_environmentGrid.Sample(position, internodeData.m_lightDirection), 0.f, 1.f);
			if (internodeData.m_lightIntensity <= glm::epsilon<float>())
			{
				internodeData.m_lightDirection = glm::normalize(internodeInfo.GetGlobalDirection());
			}
		}
		internodeData.m_spaceOccupancy = climateModel.m_environmentGrid.m_voxel.Peek(position).m_totalBiomass;
	}
}
ShootFlux TreeModel::CollectShootFlux(const std::vector<SkeletonNodeHandle>& sortedInternodeList)
{
	ShootFlux totalShootFlux;
	totalShootFlux.m_value = 0.0f;
	for (const auto& internodeHandle : sortedInternodeList) {
		auto& internode = m_shootSkeleton.RefNode(internodeHandle);
		const auto& internodeData = internode.m_data;
		totalShootFlux.m_value += internodeData.m_lightIntensity;
		
	}

	for(auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); ++it)
	{
		auto& internode = m_shootSkeleton.RefNode(*it);
		auto& internodeData = internode.m_data;
		internodeData.m_maxDescendantLightIntensity = glm::clamp(internodeData.m_lightIntensity, 0.f, 1.f);
		for (const auto& childHandle : internode.PeekChildHandles())
		{
			internodeData.m_maxDescendantLightIntensity = glm::max(internodeData.m_maxDescendantLightIntensity, m_shootSkeleton.RefNode(childHandle).m_data.m_maxDescendantLightIntensity);
		}
	}
	return totalShootFlux;
}

void TreeModel::CalculateInternodeStrength(const std::vector<SkeletonNodeHandle>& sortedInternodeList,
	const ShootGrowthController& shootGrowthController)
{
	for (const auto& internodeHandle : sortedInternodeList) {
		auto& internode = m_shootSkeleton.RefNode(internodeHandle);
		auto& internodeData = internode.m_data;
		internodeData.m_strength = shootGrowthController.m_internodeStrength(internode);
	}
}

void TreeModel::ShootGrowthPostProcess(const ShootGrowthController& shootGrowthController)
{
	{
		m_shootSkeleton.m_min = glm::vec3(FLT_MAX);
		m_shootSkeleton.m_max = glm::vec3(-FLT_MAX);
		m_shootSkeleton.m_data.m_desiredMin = glm::vec3(FLT_MAX);
		m_shootSkeleton.m_data.m_desiredMax = glm::vec3(-FLT_MAX);

		m_shootSkeleton.CalculateDistance();
		CalculateThickness(shootGrowthController);
		const auto& sortedInternodeList = m_shootSkeleton.PeekSortedNodeList();
		for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); ++it) {
			CalculateBiomass(*it, shootGrowthController);
			CalculateSaggingStress(*it, shootGrowthController);
		}
		CalculateLevel();
		CalculateTransform(shootGrowthController, true);

	};

	m_internodeOrderCounts.clear();
	m_fruitCount = m_leafCount = 0;
	{
		int maxOrder = 0;
		const auto& sortedFlowList = m_shootSkeleton.PeekSortedFlowList();
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
		const auto& sortedInternodeList = m_shootSkeleton.PeekSortedNodeList();
		for (const auto& internodeHandle : sortedInternodeList)
		{
			auto& internode = m_shootSkeleton.RefNode(internodeHandle);
			internode.m_info.m_leaves = shootGrowthController.m_leaf(internode);
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

float TreeModel::GetSubTreeMaxAge(const SkeletonNodeHandle baseInternodeHandle) const
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

bool TreeModel::Reduce(const ShootGrowthController& shootGrowthController, const SkeletonNodeHandle baseInternodeHandle, float targetAge)
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
	const auto& sortedInternodeList = m_shootSkeleton.PeekSortedNodeList();
	for (const auto& internodeHandle : sortedInternodeList) {
		auto& internode = m_shootSkeleton.RefNode(internodeHandle);
		auto& internodeData = internode.m_data;
		auto& internodeInfo = internode.m_info;

		internodeInfo.m_length = internodeData.m_internodeLength * glm::pow(internodeInfo.m_thickness / shootGrowthController.m_endNodeThickness, shootGrowthController.m_internodeLengthThicknessFactor);

		if (internode.GetParentHandle() == -1) {
			internodeInfo.m_globalPosition = internodeData.m_desiredGlobalPosition = glm::vec3(0.0f);
			internodeData.m_desiredLocalRotation = glm::vec3(0.0f);
			internodeInfo.m_globalRotation = internodeInfo.m_regulatedGlobalRotation = internodeData.m_desiredGlobalRotation = glm::vec3(glm::radians(90.0f), 0.0f, 0.0f);
			internodeInfo.GetGlobalDirection() = glm::normalize(internodeInfo.m_globalRotation * glm::vec3(0, 0, -1));
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

			internodeInfo.GetGlobalDirection() = glm::normalize(internodeInfo.m_globalRotation * glm::vec3(0, 0, -1));
			internodeInfo.m_globalPosition =
				parentInternode.m_info.m_globalPosition
				+ parentInternode.m_info.m_length * parentInternode.m_info.GetGlobalDirection();

			if (shootGrowthController.m_branchPush && !internode.IsApical())
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
			+ internodeInfo.m_length * internodeInfo.GetGlobalDirection();
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

bool TreeModel::ElongateInternode(float extendLength, SkeletonNodeHandle internodeHandle,
	const ShootGrowthController& shootGrowthController, float& collectedInhibitor) {
	bool graphChanged = false;
	auto& internode = m_shootSkeleton.RefNode(internodeHandle);
	const auto internodeLength = shootGrowthController.m_internodeLength;
	auto& internodeData = internode.m_data;
	const auto& internodeInfo = internode.m_info;
	internodeData.m_internodeLength += extendLength;
	const float extraLength = internodeData.m_internodeLength - internodeLength;
	//If we need to add a new end node
	assert(internodeData.m_buds.size() == 1);
	if (extraLength >= 0) {
		graphChanged = true;
		internodeData.m_internodeLength = internodeLength;
		const auto desiredGlobalRotation = internodeInfo.m_globalRotation * internodeData.m_buds.front().m_localRotation;
		auto desiredGlobalFront = desiredGlobalRotation * glm::vec3(0, 0, -1);
		auto desiredGlobalUp = desiredGlobalRotation * glm::vec3(0, 1, 0);
		if (internodeHandle != 0)
		{
			ApplyTropism(-m_currentGravityDirection, shootGrowthController.m_gravitropism(internode), desiredGlobalFront,
				desiredGlobalUp);
			ApplyTropism(internodeData.m_lightDirection, shootGrowthController.m_phototropism(internode),
				desiredGlobalFront, desiredGlobalUp);
		}
		//First, remove only apical bud.
		internode.m_data.m_buds.clear();
		//Allocate Lateral bud for current internode

		const auto lateralBudCount = shootGrowthController.m_lateralBudCount(internode);
		for (int i = 0; i < lateralBudCount; i++) {
			internodeData.m_buds.emplace_back();
			auto& newLateralBud = internodeData.m_buds.back();
			newLateralBud.m_type = BudType::Lateral;
			newLateralBud.m_status = BudStatus::Dormant;
			newLateralBud.m_localRotation = glm::vec3(0.f, glm::radians(shootGrowthController.m_branchingAngle(internode)),
				glm::linearRand(0.0f, 360.0f));
		}

		//Allocate Fruit bud for current internode
		{
			const auto fruitBudCount = 1;
			for (int i = 0; i < fruitBudCount; i++) {
				internodeData.m_buds.emplace_back();
				auto& newFruitBud = internodeData.m_buds.back();
				newFruitBud.m_type = BudType::Fruit;
				newFruitBud.m_status = BudStatus::Dormant;
				newFruitBud.m_localRotation = glm::vec3(
					glm::radians(shootGrowthController.m_branchingAngle(internode)), 0.0f,
					glm::radians(glm::linearRand(0.0f, 360.0f)));
			}
		}
		//Allocate Leaf bud for current internode
		{
			const auto leafBudCount = 1;
			for (int i = 0; i < leafBudCount; i++) {
				internodeData.m_buds.emplace_back();
				auto& newLeafBud = internodeData.m_buds.back();
				//Hack: Leaf bud will be given vigor for the first time.
				newLeafBud.m_type = BudType::Leaf;
				newLeafBud.m_status = BudStatus::Dormant;
				newLeafBud.m_localRotation = glm::vec3(
					glm::radians(shootGrowthController.m_branchingAngle(internode)), 0.0f,
					glm::radians(glm::linearRand(0.0f, 360.0f)));
			}
		}

		//Create new internode
		const auto newInternodeHandle = m_shootSkeleton.Extend(internodeHandle, false);
		auto& oldInternode = m_shootSkeleton.RefNode(internodeHandle);
		auto& newInternode = m_shootSkeleton.RefNode(newInternodeHandle);

		newInternode.m_data = {};
		newInternode.m_data.m_indexOfParentBud = 0;
		newInternode.m_data.m_lightIntensity = oldInternode.m_data.m_lightIntensity;
		newInternode.m_data.m_lightDirection = oldInternode.m_data.m_lightDirection;
		oldInternode.m_data.m_finishAge = newInternode.m_data.m_startAge = m_age;
		newInternode.m_data.m_finishAge = 0.0f;
		newInternode.m_data.m_order = oldInternode.m_data.m_order;
		newInternode.m_data.m_inhibitorSink = 0.0f;
		newInternode.m_data.m_internodeLength = glm::clamp(extendLength, 0.0f, internodeLength);
		newInternode.m_info.m_rootDistance = oldInternode.m_info.m_rootDistance + newInternode.m_data.m_internodeLength;
		newInternode.m_info.m_thickness = shootGrowthController.m_endNodeThickness;
		newInternode.m_info.m_globalRotation = glm::quatLookAt(desiredGlobalFront, desiredGlobalUp);
		newInternode.m_data.m_desiredLocalRotation =
			glm::inverse(oldInternode.m_info.m_globalRotation) *
			newInternode.m_info.m_globalRotation;
		if (shootGrowthController.m_apicalBudExtinctionRate(oldInternode) < glm::linearRand(0.0f, 1.0f)) {
			//Allocate apical bud for new internode
			newInternode.m_data.m_buds.emplace_back();
			auto& newApicalBud = newInternode.m_data.m_buds.back();
			newApicalBud.m_type = BudType::Apical;
			newApicalBud.m_status = BudStatus::Dormant;
			newApicalBud.m_localRotation = glm::vec3(
				glm::radians(shootGrowthController.m_apicalAngle(newInternode)), 0.0f,
				glm::radians(shootGrowthController.m_rollAngle(newInternode)));
			if (extraLength > internodeLength) {
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
		else
		{
			newInternode.m_info.m_thickness = oldInternode.m_info.m_thickness;
		}
	}
	return graphChanged;
}

bool TreeModel::GrowInternode(ClimateModel& climateModel, const SkeletonNodeHandle internodeHandle, const ShootGrowthController& shootGrowthController) {
	bool graphChanged = false;
	{
		auto& internode = m_shootSkeleton.RefNode(internodeHandle);
		auto& internodeData = internode.m_data;
		internodeData.m_inhibitorSink = 0;
		for (const auto& childHandle : internode.PeekChildHandles()) {
			auto& childNode = m_shootSkeleton.RefNode(childHandle);
			float childNodeInhibitor = 0.f;
			if (!childNode.m_data.m_buds.empty() && childNode.m_data.m_buds[0].m_type == BudType::Apical)
			{
				childNodeInhibitor = shootGrowthController.m_apicalDominance(childNode);
			}
			internodeData.m_inhibitorSink += glm::max(0.0f, (childNodeInhibitor + childNode.m_data.m_inhibitorSink) *
				glm::clamp(1.0f - shootGrowthController.m_apicalDominanceLoss, 0.0f, 1.0f));
		}
		if (!internode.m_data.m_buds.empty()) {
			const auto& bud = internode.m_data.m_buds[0];
			if (bud.m_type == BudType::Apical) {
				assert(internode.m_data.m_buds.size() == 1);
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
			}
		}
	}

	auto budSize = m_shootSkeleton.RefNode(internodeHandle).m_data.m_buds.size();
	for (int budIndex = 0; budIndex < budSize; budIndex++) {
		auto& internode = m_shootSkeleton.RefNode(internodeHandle);
		auto& bud = internode.m_data.m_buds[budIndex];
		const auto& internodeData = internode.m_data;
		const auto& internodeInfo = internode.m_info;
		if (bud.m_type == BudType::Lateral && bud.m_status == BudStatus::Dormant) {
			float flushProbability = bud.m_flushingRate = shootGrowthController.m_lateralBudFlushingRate(internode);
			if (m_treeGrowthSettings.m_useSpaceColonization) {
				if (m_shootSkeleton.m_data.m_maxMarkerCount > 0) flushProbability *= static_cast<float>(bud.m_markerCount) / m_shootSkeleton.m_data.m_maxMarkerCount;
			}
			else
			{
				flushProbability *= internodeData.m_growthRate * m_currentDeltaTime * shootGrowthController.m_internodeGrowthRate;
			}
			if (flushProbability >= glm::linearRand(0.0f, 1.0f)) {
				graphChanged = true;
				//Prepare information for new internode
				const auto desiredGlobalRotation = internodeInfo.m_globalRotation * bud.m_localRotation;
				auto desiredGlobalFront = desiredGlobalRotation * glm::vec3(0, 0, -1);
				auto desiredGlobalUp = desiredGlobalRotation * glm::vec3(0, 1, 0);
				ApplyTropism(-m_currentGravityDirection, shootGrowthController.m_gravitropism(internode), desiredGlobalFront,
					desiredGlobalUp);
				ApplyTropism(internodeData.m_lightDirection, shootGrowthController.m_phototropism(internode),
					desiredGlobalFront, desiredGlobalUp);
				const auto horizontalDirection = glm::vec3(desiredGlobalFront.x, 0.0f, desiredGlobalFront.z);
				if (glm::length(horizontalDirection) > glm::epsilon<float>()) {
					ApplyTropism(glm::normalize(horizontalDirection), shootGrowthController.m_horizontalTropism(internode),
						desiredGlobalFront, desiredGlobalUp);
				}
				//Remove current lateral bud.
				internode.m_data.m_buds[budIndex] = internode.m_data.m_buds.back();
				internode.m_data.m_buds.pop_back();
				budIndex--;
				budSize--;

				//Create new internode
				const auto newInternodeHandle = m_shootSkeleton.Extend(internodeHandle, true);
				const auto& oldInternode = m_shootSkeleton.PeekNode(internodeHandle);
				auto& newInternode = m_shootSkeleton.RefNode(newInternodeHandle);
				newInternode.m_data = {};
				newInternode.m_data.m_indexOfParentBud = 0;
				newInternode.m_data.m_startAge = m_age;
				newInternode.m_data.m_finishAge = 0.0f;
				newInternode.m_data.m_order = oldInternode.m_data.m_order + 1;
				newInternode.m_data.m_internodeLength = 0.0f;
				newInternode.m_info.m_rootDistance = oldInternode.m_info.m_rootDistance;
				newInternode.m_info.m_thickness = shootGrowthController.m_endNodeThickness;
				newInternode.m_data.m_desiredLocalRotation =
					glm::inverse(oldInternode.m_info.m_globalRotation) *
					glm::quatLookAt(desiredGlobalFront, desiredGlobalUp);
				//Allocate apical bud
				newInternode.m_data.m_buds.emplace_back();
				auto& apicalBud = newInternode.m_data.m_buds.back();
				apicalBud.m_type = BudType::Apical;
				apicalBud.m_status = BudStatus::Dormant;
				apicalBud.m_localRotation = glm::vec3(0.f);
			}
		}
	}
	return graphChanged;
}

bool TreeModel::GrowReproductiveModules(ClimateModel& climateModel, const SkeletonNodeHandle internodeHandle,
	const ShootGrowthController& shootGrowthController)
{
	bool statusChanged = false;
	const auto budSize = m_shootSkeleton.RefNode(internodeHandle).m_data.m_buds.size();
	for (int budIndex = 0; budIndex < budSize; budIndex++) {
		auto& internode = m_shootSkeleton.RefNode(internodeHandle);
		auto& bud = internode.m_data.m_buds[budIndex];
		auto& internodeData = internode.m_data;
		auto& internodeInfo = internode.m_info;
		//Calculate vigor used for maintenance and development.
		if (bud.m_type == BudType::Fruit)
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
				const float flushProbability = m_currentDeltaTime * 1.;
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
				/*
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
					const auto dropProbability = m_currentDeltaTime * shootGrowthController.m_leafFallProbability(internode);
					if (dropProbability >= glm::linearRand(0.0f, 1.0f))
					{
						bud.m_status = BudStatus::Died;
						m_shootSkeleton.m_data.m_droppedLeaves.emplace_back(bud.m_reproductiveModule);
						bud.m_reproductiveModule.Reset();
					}
				}*/
			}
		}
	}
	return statusChanged;
}

void TreeModel::CalculateLevel()
{
	auto& sortedInternodeList = m_shootSkeleton.PeekSortedNodeList();
	for (const auto& internodeHandle : sortedInternodeList)
	{
		auto& node = m_shootSkeleton.RefNode(internodeHandle);
		if (node.GetParentHandle() == -1)
		{
			node.m_data.m_level = 0;
		}
		else
		{
			float maxBiomass = 0.0f;
			SkeletonNodeHandle maxChild = -1;
			for (const auto& childHandle : node.PeekChildHandles())
			{
				auto& childNode = m_shootSkeleton.PeekNode(childHandle);
				const auto childBiomass = childNode.m_data.m_descendantTotalBiomass + childNode.m_data.m_biomass;
				if (childBiomass > maxBiomass)
				{
					maxBiomass = childBiomass;
					maxChild = childHandle;
				}
			}
			for (const auto& childHandle : node.PeekChildHandles())
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
		m_shootSkeleton.m_data.m_maxOrder = glm::max(m_shootSkeleton.m_data.m_maxOrder, node.m_data.m_order);
	}
}


void TreeModel::CalculateGrowthRate(const std::vector<SkeletonNodeHandle>& sortedInternodeList, const float factor)
{
	const float clampedFactor = glm::clamp(factor, 0.0f, 1.0f);
	for (const auto& internodeHandle : sortedInternodeList)
	{
		auto& node = m_shootSkeleton.RefNode(internodeHandle);
		//You cannot give more than enough resources.
		node.m_data.m_growthRate = clampedFactor * node.m_data.m_desiredGrowthRate;
	}
}

float TreeModel::CalculateGrowthPotential(const std::vector<SkeletonNodeHandle>& sortedInternodeList, const ShootGrowthController& shootGrowthController)
{
	const float apicalControl = shootGrowthController.m_apicalControl;
	const float rootDistanceControl = shootGrowthController.m_rootDistanceControl;
	const float heightControl = shootGrowthController.m_heightControl;

	float maxGrowPotential = 0.0f;

	std::vector<float> apicalControlValues{};

	const auto maxVal = shootGrowthController.m_useLevelForApicalControl ? m_shootSkeleton.m_data.m_maxLevel : m_shootSkeleton.m_data.m_maxOrder;
	apicalControlValues.resize(maxVal + 1);
	apicalControlValues[0] = 1.0f;
	for (int i = 1; i < (maxVal + 1); i++)
	{
		apicalControlValues[i] = apicalControlValues[i - 1] * apicalControl;
	}

	for (const auto& internodeHandle : sortedInternodeList)
	{
		auto& node = m_shootSkeleton.RefNode(internodeHandle);

		float localApicalControl;
		if (apicalControl > 1.0f)
		{
			localApicalControl = 1.0f / apicalControlValues[(shootGrowthController.m_useLevelForApicalControl ? node.m_data.m_level : node.m_data.m_order)];
		}
		else if (apicalControl < 1.0f)
		{
			localApicalControl = apicalControlValues[maxVal - (shootGrowthController.m_useLevelForApicalControl ? node.m_data.m_level : node.m_data.m_order)];
		}
		else
		{
			localApicalControl = 1.0f;
		}
		float localRootDistanceControl;
		if (rootDistanceControl != 0.f) {
			float distance = node.m_info.m_rootDistance + node.m_info.m_length;
			if (distance == 0.f) distance = 1.f;
			localRootDistanceControl = glm::pow(1.f / distance, rootDistanceControl);
		}
		else {
			localRootDistanceControl = 1.f;
		}
		float localHeightControl;
		if (heightControl != 0.f)
		{
			float y = node.m_info.GetGlobalEndPosition().y;
			if (y == 0.f) y = 1.f;
			localHeightControl = glm::pow(1.f / y, heightControl);
		}
		else
		{
			localHeightControl = 1.f;
		}
		node.m_data.m_growthPotential = localApicalControl * localRootDistanceControl * localHeightControl;
		maxGrowPotential = glm::max(maxGrowPotential, node.m_data.m_growthPotential);
	}
	float totalDesiredGrowthRate = 1.0f;
	for (const auto& internodeHandle : sortedInternodeList)
	{
		auto& node = m_shootSkeleton.RefNode(internodeHandle);
		if (maxGrowPotential > 0.0f) node.m_data.m_growthPotential /= maxGrowPotential;
		node.m_data.m_desiredGrowthRate = node.m_data.m_lightIntensity * node.m_data.m_growthPotential;
		totalDesiredGrowthRate += node.m_data.m_desiredGrowthRate;
	}
	return totalDesiredGrowthRate;

}

void TreeModel::CalculateThickness(const ShootGrowthController& shootGrowthController) {
	auto& sortedInternodeList = m_shootSkeleton.PeekSortedNodeList();
	for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); ++it) {
		const auto internodeHandle = *it;
		auto& internode = m_shootSkeleton.RefNode(internodeHandle);
		const auto& internodeData = internode.m_data;
		auto& internodeInfo = internode.m_info;
		float childThicknessCollection = 0.0f;
		for (const auto& i : internode.PeekChildHandles()) {
			const auto& childInternode = m_shootSkeleton.PeekNode(i);
			childThicknessCollection += glm::pow(childInternode.m_info.m_thickness,
				1.0f / shootGrowthController.m_thicknessAccumulationFactor);
		}
		childThicknessCollection += shootGrowthController.m_thicknessAgeFactor * shootGrowthController.m_endNodeThickness * shootGrowthController.m_internodeGrowthRate * (m_age - internodeData.m_startAge);
		if (childThicknessCollection != 0.0f) {
			internodeInfo.m_thickness = glm::max(internodeInfo.m_thickness, glm::pow(childThicknessCollection, shootGrowthController.m_thicknessAccumulationFactor));
		}
		else
		{
			internodeInfo.m_thickness = glm::max(internodeInfo.m_thickness, shootGrowthController.m_endNodeThickness);
		}
	}
}
void TreeModel::CalculateBiomass(SkeletonNodeHandle internodeHandle, const ShootGrowthController& shootGrowthController)
{
	auto& internode = m_shootSkeleton.RefNode(internodeHandle);
	auto& internodeData = internode.m_data;
	const auto& internodeInfo = internode.m_info;
	internodeData.m_descendantTotalBiomass = internodeData.m_biomass = 0.0f;
	const auto relativeThickness = internodeInfo.m_thickness / shootGrowthController.m_endNodeThickness;
	internodeData.m_biomass =
		internodeData.m_density * (relativeThickness * relativeThickness) * internodeData.m_internodeLength /
		shootGrowthController.m_internodeLength;
	glm::vec3 positionedSum = glm::vec3(0.f);
	glm::vec3 desiredPositionSum = glm::vec3(0.f);
	for (const auto& i : internode.PeekChildHandles()) {
		const auto& childInternode = m_shootSkeleton.RefNode(i);
		internodeData.m_descendantTotalBiomass +=
			childInternode.m_data.m_descendantTotalBiomass +
			childInternode.m_data.m_biomass;
		positionedSum += childInternode.m_data.m_biomass * (childInternode.m_info.m_globalPosition + childInternode.m_info.GetGlobalEndPosition()) * .5f;
		positionedSum += childInternode.m_data.m_descendantWeightCenter * childInternode.m_data.m_descendantTotalBiomass;

		glm::vec3 childDesiredGlobalEndPosition = childInternode.m_data.m_desiredGlobalPosition;
		childDesiredGlobalEndPosition += childInternode.m_info.m_length * (childInternode.m_data.m_desiredGlobalRotation * glm::vec3(0, 0, -1));
		desiredPositionSum += childInternode.m_data.m_biomass * (childInternode.m_data.m_desiredGlobalPosition + childDesiredGlobalEndPosition) * .5f;
		desiredPositionSum += childInternode.m_data.m_desiredDescendantWeightCenter * childInternode.m_data.m_descendantTotalBiomass;
	}
	if (!internode.PeekChildHandles().empty() && internodeData.m_descendantTotalBiomass != 0.f) {
		internodeData.m_descendantWeightCenter = positionedSum / internodeData.m_descendantTotalBiomass;
		internodeData.m_desiredDescendantWeightCenter = desiredPositionSum / internodeData.m_descendantTotalBiomass;
	}
	else
	{
		internodeData.m_descendantWeightCenter = internode.m_info.GetGlobalEndPosition();

		glm::vec3 desiredGlobalEndPosition = internode.m_data.m_desiredGlobalPosition;
		desiredGlobalEndPosition += internode.m_info.m_length * (internode.m_data.m_desiredGlobalRotation * glm::vec3(0, 0, -1));
		internodeData.m_desiredDescendantWeightCenter = desiredGlobalEndPosition;
	}
}

void TreeModel::CalculateSaggingStress(const SkeletonNodeHandle internodeHandle, const ShootGrowthController& shootGrowthController)
{
	auto& internode = m_shootSkeleton.RefNode(internodeHandle);
	if(internode.IsEndNode() || internode.m_info.m_thickness == 0.f || internode.m_info.m_length == 0.f)
	{
		internode.m_data.m_saggingForce = 0.f;
		internode.m_data.m_saggingStress = 0.f;
		return;
	}
	const auto weightCenterRelativePosition = internode.m_info.m_globalPosition - internode.m_data.m_descendantWeightCenter;
	//const auto horizontalDistanceToEnd = glm::length(glm::vec2(weightCenterRelativePosition.x, weightCenterRelativePosition.z));
	//const auto front = glm::normalize(internode.m_info.m_globalRotation * glm::vec3(0, 0, -1));
	//const auto frontVector = internode.m_info.m_length * front;
	//const auto baseVector = glm::vec2(glm::length(glm::vec2(frontVector.x, frontVector.z)), glm::abs(frontVector.y));
	//const auto combinedVector = glm::vec2(horizontalDistanceToEnd, glm::abs(weightCenterRelativePosition.y)) + baseVector;
	//const auto projectedVector = baseVector * glm::dot(combinedVector, baseVector);
	//const auto forceArm = glm::length(projectedVector) / shootGrowthController.m_endNodeThickness;

	//const auto normalizedCombinedVector = glm::normalize(combinedVector);
	//const float cosTheta = glm::abs(glm::dot(normalizedCombinedVector, glm::normalize(glm::vec2(baseVector.y, -baseVector.x))));
	//float sinTheta = 1.0f;
	//if(cosTheta != 1.f) sinTheta = glm::sqrt(1 - cosTheta * cosTheta);
	//const float tangentForce = (internode.m_data.m_biomass + internode.m_data.m_descendantTotalBiomass) * sinTheta * glm::length(glm::vec2(0, -1) * glm::dot(normalizedCombinedVector, glm::vec2(0, -1)));
	const float cosTheta = glm::abs(glm::dot(glm::normalize(weightCenterRelativePosition), glm::vec3(0, -1, 0)));
	float sinTheta = 0.0f;
	if(cosTheta != 1.f) sinTheta = glm::sqrt(1 - cosTheta * cosTheta);
	const float tangentForce = (internode.m_data.m_biomass + internode.m_data.m_descendantTotalBiomass) * sinTheta * glm::length(weightCenterRelativePosition);
	internode.m_data.m_saggingForce = tangentForce;
	if(glm::isnan(internode.m_data.m_saggingForce))
	{
		internode.m_data.m_saggingForce = 0.f;
	}
	const auto breakingForce = shootGrowthController.m_breakingForce(internode);
	internode.m_data.m_saggingStress = internode.m_data.m_saggingForce / breakingForce;
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

	
	const auto& sortedInternodeList = m_shootSkeleton.PeekSortedNodeList();

	bool rootToEndPruned = false;
	for(const auto& internodeHandle : sortedInternodeList)
	{
		if (m_shootSkeleton.PeekNode(internodeHandle).IsRecycled()) continue;
		auto& internode = m_shootSkeleton.RefNode(internodeHandle);
		if (internodeHandle == 0) continue;
		if (internode.m_info.m_locked) continue;
		//Pruning here.
		bool pruning = false;
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
					pruning = true;
				}
			}
		}
		const float pruningProbability = shootGrowthController.m_rootToEndPruningFactor(globalTransform, climateModel, m_shootSkeleton, internode) * m_currentDeltaTime;
		if (!pruning && pruningProbability > glm::linearRand(0.0f, 1.0f)) pruning = true;

		if (pruning)
		{
			PruneInternode(internodeHandle);
			rootToEndPruned = true;
		}
	}
	bool endToRootPruned = false;
	for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); ++it) {
		const auto internodeHandle = *it;
		if (m_shootSkeleton.PeekNode(internodeHandle).IsRecycled()) continue;
		CalculateBiomass(internodeHandle, shootGrowthController);
		CalculateSaggingStress(internodeHandle, shootGrowthController);
		auto& internode = m_shootSkeleton.RefNode(internodeHandle);
		if (internodeHandle == 0) continue;
		if (internode.m_info.m_locked) continue;
		//Pruning here.
		bool pruning = false;
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
					pruning = true;
				}
			}
		}

		const float pruningProbability = shootGrowthController.m_endToRootPruningFactor(globalTransform, climateModel, m_shootSkeleton, internode) * m_currentDeltaTime;
		if (!pruning && pruningProbability > glm::linearRand(0.0f, 1.0f)) pruning = true;
		if (pruning)
		{
			PruneInternode(internodeHandle);
			endToRootPruned = true;
		}
	}
	m_shootSkeleton.CalculateDistance();
	CalculateLevel();
	return rootToEndPruned || endToRootPruned;
}




void TreeModel::SampleTemperature(const glm::mat4& globalTransform, ClimateModel& climateModel)
{
	const auto& sortedInternodeList = m_shootSkeleton.PeekSortedNodeList();
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
