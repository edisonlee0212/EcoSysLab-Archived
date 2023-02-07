//
// Created by lllll on 10/21/2022.
//

#include "TreeModel.hpp"

using namespace EcoSysLab;

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

bool TreeModel::Grow(const glm::mat4& globalTransform, SoilModel& soilModel, ClimateModel& climateModel,
	const RootGrowthParameters& rootGrowthParameters, const ShootGrowthParameters& shootGrowthParameters)
{
	m_fruitCount = 0;
	m_leafCount = 0;

	bool treeStructureChanged = false;
	bool rootStructureChanged = false;
	if (!m_initialized) {
		Initialize(shootGrowthParameters, rootGrowthParameters);
		treeStructureChanged = true;
		rootStructureChanged = true;
	}
	//Collect water from roots.
	CollectRootFlux(globalTransform, soilModel, rootGrowthParameters);
	//Collect light from branches.
	CollectShootFlux(globalTransform, climateModel, shootGrowthParameters);
	//Perform photosynthesis.
	PlantVigorAllocation();
	//Grow roots and set up nutrient requirements for next iteration.
	PlantGrowthRequirement newShootGrowthRequirement;
	PlantGrowthRequirement newRootGrowthRequirement;
	if (rootGrowthParameters.m_growthRate != 0.0f
		&& GrowRoots(globalTransform, soilModel, rootGrowthParameters, newRootGrowthRequirement)) {
		rootStructureChanged = true;
	}
	//Grow branches and set up nutrient requirements for next iteration.
	if (shootGrowthParameters.m_growthRate != 0.0f
		&& GrowShoots(globalTransform, climateModel, shootGrowthParameters, newShootGrowthRequirement)) {
		treeStructureChanged = true;
	}
	//Set new growth nutrients requirement for next iteration.
	m_shootGrowthRequirement = newShootGrowthRequirement;
	m_rootGrowthRequirement = newRootGrowthRequirement;

	m_age++;
	return treeStructureChanged || rootStructureChanged;
}

void TreeModel::Initialize(const ShootGrowthParameters& shootGrowthParameters, const RootGrowthParameters& rootGrowthParameters) {
	{
		auto& firstInternode = m_shootSkeleton.RefNode(0);
		firstInternode.m_info.m_thickness = shootGrowthParameters.m_endNodeThickness;
		firstInternode.m_data.m_buds.emplace_back();
		auto& apicalBud = firstInternode.m_data.m_buds.back();
		apicalBud.m_type = BudType::Apical;
		apicalBud.m_status = BudStatus::Dormant;
		apicalBud.m_localRotation = glm::vec3(glm::radians(shootGrowthParameters.GetDesiredApicalAngle(firstInternode)),
			0.0f,
			shootGrowthParameters.GetDesiredRollAngle(firstInternode));
	}
	{
		auto& firstRootNode = m_rootSkeleton.RefNode(0);
		firstRootNode.m_info.m_thickness = 0.003f;
		firstRootNode.m_info.m_length = 0.0f;
		firstRootNode.m_info.m_localRotation = glm::vec3(glm::radians(rootGrowthParameters.GetRootApicalAngle(firstRootNode)),
			0.0f,
			rootGrowthParameters.GetRootRollAngle(firstRootNode));
		firstRootNode.m_data.m_verticalTropism = rootGrowthParameters.m_tropismIntensity;
		firstRootNode.m_data.m_horizontalTropism = 0;
	}
	m_initialized = true;
}

void TreeModel::CollectRootFlux(const glm::mat4& globalTransform, SoilModel& soilModel, const RootGrowthParameters& rootGrowthParameters)
{
	m_rootFlux.m_water = 0.0f;
	const auto& sortedRootNodeList = m_rootSkeleton.RefSortedNodeList();
	for (const auto& rootNodeHandle : sortedRootNodeList) {
		auto& rootNode = m_rootSkeleton.RefNode(rootNodeHandle);
		rootNode.m_data.m_age++;
		auto& rootNodeInfo = rootNode.m_info;
		auto worldSpacePosition = globalTransform * glm::translate(rootNodeInfo.m_globalPosition)[3];
		if (m_collectWater) {
			rootNode.m_data.m_water = soilModel.IntegrateWater(worldSpacePosition, 0.2);
			m_rootFlux.m_water += rootNode.m_data.m_water;
		}
	}
	if (!m_collectWater) {
		m_rootFlux.m_water = m_shootGrowthRequirement.m_maintenanceVigor + m_rootGrowthRequirement.m_maintenanceVigor
			+ m_shootGrowthRequirement.m_developmentalVigor + m_rootGrowthRequirement.m_developmentalVigor;
	}
}

void TreeModel::CollectShootFlux(const glm::mat4& globalTransform, ClimateModel& climateModel,
	const ShootGrowthParameters& shootGrowthParameters)
{
	m_shootFlux.m_lightEnergy = 0.0f;
	const auto& sortedInternodeList = m_shootSkeleton.RefSortedNodeList();
	for (const auto& internodeHandle : sortedInternodeList) {
		auto& internode = m_shootSkeleton.RefNode(internodeHandle);
		auto& internodeData = internode.m_data;
		auto& internodeInfo = internode.m_info;
		internodeData.m_age++;
		internodeData.m_lightIntensity =
			m_treeVolume.IlluminationEstimation(internodeInfo.m_globalPosition, m_illuminationEstimationSettings, internodeData.m_lightDirection);
		internodeInfo.m_color = glm::mix(glm::vec4(0.0f, 0.0f, 0.0f, 1.0f), glm::vec4(1.0f), internodeData.m_lightIntensity);
		for (const auto& bud : internode.m_data.m_buds)
		{
			if (bud.m_status == BudStatus::Flushed && bud.m_type == BudType::Leaf)
			{
				if (m_collectLight) {
					internodeData.m_lightEnergy = internodeData.m_lightIntensity * glm::pow(bud.m_maturity, 2.0f) * bud.m_drought;
					m_shootFlux.m_lightEnergy += internodeData.m_lightEnergy;
				}
			}
		}
	}
	if (!m_collectLight) {
		m_shootFlux.m_lightEnergy =
			m_shootGrowthRequirement.m_maintenanceVigor + m_rootGrowthRequirement.m_maintenanceVigor
			+ m_shootGrowthRequirement.m_developmentalVigor + m_rootGrowthRequirement.m_developmentalVigor;
	}
}

void TreeModel::PlantVigorAllocation()
{
	const float totalVigor = glm::min(m_rootFlux.m_water, m_shootFlux.m_lightEnergy);
	const float totalMaintenanceVigorRequirement = m_shootGrowthRequirement.m_maintenanceVigor + m_rootGrowthRequirement.m_maintenanceVigor;
	const float totalDevelopmentalVigorRequirement = m_shootGrowthRequirement.m_developmentalVigor + m_rootGrowthRequirement.m_developmentalVigor;
	const float maintenanceVigor = glm::min(totalVigor, totalMaintenanceVigorRequirement);
	const float developmentVigor = totalVigor - maintenanceVigor;
	m_plantVigor.m_rootVigor = m_plantVigor.m_shootVigor = 0.0f;
	if (totalMaintenanceVigorRequirement != 0.0f) {
		m_plantVigor.m_rootVigor += maintenanceVigor * m_rootGrowthRequirement.m_maintenanceVigor
			/ totalMaintenanceVigorRequirement;
		m_plantVigor.m_shootVigor += maintenanceVigor * m_shootGrowthRequirement.m_maintenanceVigor
			/ totalMaintenanceVigorRequirement;
	}
	m_plantVigor.m_rootVigor += developmentVigor * m_rootGrowthRequirement.m_developmentalVigor / totalDevelopmentalVigorRequirement;
	m_plantVigor.m_shootVigor += developmentVigor * m_shootGrowthRequirement.m_developmentalVigor / totalDevelopmentalVigorRequirement;
	/*
	if (m_vigorRatio.m_shootVigorWeight + m_vigorRatio.m_rootVigorWeight != 0.0f) {
		m_plantVigor.m_rootVigor += developmentVigor * m_vigorRatio.m_rootVigorWeight / (m_vigorRatio.m_shootVigorWeight + m_vigorRatio.m_rootVigorWeight);
		m_plantVigor.m_shootVigor += developmentVigor * m_vigorRatio.m_shootVigorWeight / (m_vigorRatio.m_shootVigorWeight + m_vigorRatio.m_rootVigorWeight);
	}
	*/
}

bool TreeModel::GrowRoots(const glm::mat4& globalTransform, SoilModel& soilModel, const RootGrowthParameters& rootGrowthParameters, PlantGrowthRequirement& newRootGrowthRequirement)
{
	bool rootStructureChanged = false;

#pragma region Root Growth
	{
#pragma region Pruning
		bool anyRootPruned = false;
		m_rootSkeleton.SortLists();
		{

		};
#pragma endregion



#pragma region Grow
		if (anyRootPruned) m_rootSkeleton.SortLists();
		rootStructureChanged = rootStructureChanged || anyRootPruned;
		bool anyRootGrown = false;
		{
			const auto& sortedRootNodeList = m_rootSkeleton.RefSortedNodeList();
			AggregateRootVigorRequirement();
			AllocateRootVigor(rootGrowthParameters);
			for (auto it = sortedRootNodeList.rbegin(); it != sortedRootNodeList.rend(); it++) {
				const bool graphChanged = GrowRootNode(soilModel, *it, rootGrowthParameters);
				anyRootGrown = anyRootGrown || graphChanged;
			}
		};
#pragma endregion
#pragma region Postprocess
		if (anyRootGrown) m_rootSkeleton.SortLists();

		rootStructureChanged = rootStructureChanged || anyRootGrown;
		{
			m_rootSkeleton.m_min = glm::vec3(FLT_MAX);
			m_rootSkeleton.m_max = glm::vec3(FLT_MIN);
			const auto& sortedRootNodeList = m_rootSkeleton.RefSortedNodeList();
			for (auto it = sortedRootNodeList.rbegin(); it != sortedRootNodeList.rend(); it++) {
				CalculateThickness(*it, rootGrowthParameters);
			}
			for (const auto& rootNodeHandle : sortedRootNodeList) {
				auto& rootNode = m_rootSkeleton.RefNode(rootNodeHandle);
				auto& rootNodeData = rootNode.m_data;
				auto& rootNodeInfo = rootNode.m_info;

				if (rootNode.GetParentHandle() == -1) {
					rootNodeInfo.m_globalPosition = glm::vec3(0.0f);
					rootNodeInfo.m_localRotation = glm::vec3(0.0f);
					rootNodeInfo.m_globalRotation = glm::vec3(glm::radians(-90.0f), 0.0f, 0.0f);

					rootNodeData.m_rootDistance = rootNodeInfo.m_length / rootGrowthParameters.m_rootNodeLength;
				}
				else {
					auto& parentRootNode = m_rootSkeleton.RefNode(rootNode.GetParentHandle());
					rootNodeData.m_rootDistance = parentRootNode.m_data.m_rootDistance + rootNodeInfo.m_length / rootGrowthParameters.m_rootNodeLength;
					rootNodeInfo.m_globalRotation =
						parentRootNode.m_info.m_globalRotation * rootNodeInfo.m_localRotation;
					rootNodeInfo.m_globalPosition =
						parentRootNode.m_info.m_globalPosition + parentRootNode.m_info.m_length *
						(parentRootNode.m_info.m_globalRotation *
							glm::vec3(0, 0, -1));



				}
				m_rootSkeleton.m_min = glm::min(m_rootSkeleton.m_min, rootNodeInfo.m_globalPosition);
				m_rootSkeleton.m_max = glm::max(m_rootSkeleton.m_max, rootNodeInfo.m_globalPosition);
				const auto endPosition = rootNodeInfo.m_globalPosition + rootNodeInfo.m_length *
					(rootNodeInfo.m_globalRotation *
						glm::vec3(0, 0, -1));
				m_rootSkeleton.m_min = glm::min(m_rootSkeleton.m_min, endPosition);
				m_rootSkeleton.m_max = glm::max(m_rootSkeleton.m_max, endPosition);
			}
		};
		SampleSoilDensity(globalTransform, soilModel);
		SampleNitrite(globalTransform, soilModel);
		CalculateVigorRequirement(rootGrowthParameters, newRootGrowthRequirement);
		if (m_enableRootCollisionDetection)
		{
			const float minRadius = rootGrowthParameters.m_endNodeThickness * 4.0f;
			CollisionDetection(minRadius, m_rootOctree, m_rootSkeleton);
		}
		m_rootNodeOrderCounts.clear();
		{
			int maxOrder = 0;
			const auto& sortedFlowList = m_rootSkeleton.RefSortedFlowList();
			for (const auto& flowHandle : sortedFlowList) {
				auto& flow = m_rootSkeleton.RefFlow(flowHandle);
				auto& flowData = flow.m_data;
				if (flow.GetParentHandle() == -1) {
					flowData.m_order = 0;
				}
				else {
					auto& parentFlow = m_rootSkeleton.RefFlow(flow.GetParentHandle());
					if (flow.IsApical()) flowData.m_order = parentFlow.m_data.m_order;
					else flowData.m_order = parentFlow.m_data.m_order + 1;
				}
				maxOrder = glm::max(maxOrder, flowData.m_order);
			}
			m_rootNodeOrderCounts.resize(maxOrder + 1);
			std::fill(m_rootNodeOrderCounts.begin(), m_rootNodeOrderCounts.end(), 0);
			const auto& sortedRootNodeList = m_rootSkeleton.RefSortedNodeList();
			m_fineRootCount = 0;
			for (const auto& rootNodeHandle : sortedRootNodeList)
			{
				auto& rootNode = m_rootSkeleton.RefNode(rootNodeHandle);
				const auto order = m_rootSkeleton.RefFlow(rootNode.GetFlowHandle()).m_data.m_order;
				rootNode.m_data.m_order = order;
				m_rootNodeOrderCounts[order]++;

				//Generate fine root here
				if (rootNode.m_info.m_thickness < rootGrowthParameters.m_fineRootMinNodeThickness && rootNodeHandle % rootGrowthParameters.m_fineRootNodeCount == 0)
				{
					m_fineRootCount++;
					if (rootNode.m_data.m_fineRootAnchors.empty())
					{
						rootNode.m_data.m_fineRootAnchors.resize(5);
						auto desiredGlobalRotation = rootNode.m_info.m_globalRotation * glm::quat(glm::vec3(
							glm::radians(rootGrowthParameters.m_fineRootBranchingAngle), 0.0f,
							glm::radians(rootGrowthParameters.GetRootRollAngle(rootNode))));

						glm::vec3 positionWalker = rootNode.m_info.m_globalPosition;
						for (int i = 0; i < 5; i++)
						{
							auto front = desiredGlobalRotation * glm::vec3(0, 0, -1);
							positionWalker = positionWalker + front * rootGrowthParameters.m_fineRootSegmentLength;
							rootNode.m_data.m_fineRootAnchors[i] = glm::vec4(positionWalker, rootGrowthParameters.m_fineRootThickness);
							desiredGlobalRotation = rootNode.m_info.m_globalRotation * glm::quat(glm::vec3(
								glm::radians(glm::gaussRand(0.f, rootGrowthParameters.m_fineRootApicalAngleVariance) + rootGrowthParameters.m_fineRootBranchingAngle), 0.0f,
								glm::radians(glm::linearRand(0.0f, 360.0f))));
						}
					}
				}
				else
				{
					rootNode.m_data.m_fineRootAnchors.clear();
				}
			}
			m_rootSkeleton.CalculateFlows();
		};
#pragma endregion
	};
#pragma endregion

	return rootStructureChanged;
}

bool TreeModel::GrowShoots(const glm::mat4& globalTransform, ClimateModel& climateModel, const ShootGrowthParameters& shootGrowthParameters, PlantGrowthRequirement& newShootGrowthRequirement) {
	bool treeStructureChanged = false;

#pragma region Tree Growth

#pragma region Pruning
	bool anyBranchPruned = false;
	m_shootSkeleton.SortLists();
	{
		const auto maxDistance = m_shootSkeleton.RefNode(0).m_data.m_maxDistanceToAnyBranchEnd;
		const auto& sortedInternodeList = m_shootSkeleton.RefSortedNodeList();
		for (const auto& internodeHandle : sortedInternodeList) {
			if (m_shootSkeleton.RefNode(internodeHandle).IsRecycled()) continue;
			if (PruneInternodes(maxDistance, internodeHandle, shootGrowthParameters)) {
				anyBranchPruned = true;
			}
		}
	};
#pragma endregion
#pragma region Grow
	if (anyBranchPruned) m_shootSkeleton.SortLists();
	treeStructureChanged = treeStructureChanged || anyBranchPruned;
	bool anyBranchGrown = false;
	{
		AggregateInternodeVigorRequirement();
		AllocateShootVigor(shootGrowthParameters);
		const auto& sortedInternodeList = m_shootSkeleton.RefSortedNodeList();
		for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); it++) {
			const bool graphChanged = GrowInternode(climateModel, *it, shootGrowthParameters);
			anyBranchGrown = anyBranchGrown || graphChanged;
		}
	};
#pragma endregion
#pragma region Postprocess
	if (anyBranchGrown) m_shootSkeleton.SortLists();
	treeStructureChanged = treeStructureChanged || anyBranchGrown;
	{
		m_shootSkeleton.m_min = glm::vec3(FLT_MAX);
		m_shootSkeleton.m_max = glm::vec3(FLT_MIN);
		const auto& sortedInternodeList = m_shootSkeleton.RefSortedNodeList();
		for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); it++) {
			auto internodeHandle = *it;
			CalculateThicknessAndSagging(internodeHandle, shootGrowthParameters);
		}

		for (const auto& internodeHandle : sortedInternodeList) {
			auto& internode = m_shootSkeleton.RefNode(internodeHandle);
			auto& internodeData = internode.m_data;
			auto& internodeInfo = internode.m_info;

			if (internode.GetParentHandle() == -1) {
				internodeInfo.m_globalPosition = glm::vec3(0.0f);
				internodeInfo.m_localRotation = glm::vec3(0.0f);
				internodeInfo.m_globalRotation = glm::vec3(glm::radians(90.0f), 0.0f, 0.0f);

				internodeData.m_rootDistance =
					internodeInfo.m_length / shootGrowthParameters.m_internodeLength;
			}
			else {
				auto& parentInternode = m_shootSkeleton.RefNode(internode.GetParentHandle());
				internodeData.m_rootDistance = parentInternode.m_data.m_rootDistance + internodeInfo.m_length /
					shootGrowthParameters.m_internodeLength;
				internodeInfo.m_globalRotation =
					parentInternode.m_info.m_globalRotation * internodeInfo.m_localRotation;
#pragma region Apply Sagging
				auto parentGlobalRotation = m_shootSkeleton.RefNode(
					internode.GetParentHandle()).m_info.m_globalRotation;
				internodeInfo.m_globalRotation = parentGlobalRotation * internodeData.m_desiredLocalRotation;
				auto front = internodeInfo.m_globalRotation * glm::vec3(0, 0, -1);
				auto up = internodeInfo.m_globalRotation * glm::vec3(0, 1, 0);
				float dotP = glm::abs(glm::dot(front, m_currentGravityDirection));
				ApplyTropism(m_currentGravityDirection, internodeData.m_sagging * (1.0f - dotP), front, up);
				internodeInfo.m_globalRotation = glm::quatLookAt(front, up);
				internodeInfo.m_localRotation = glm::inverse(parentGlobalRotation) * internodeInfo.m_globalRotation;
#pragma endregion

				internodeInfo.m_globalPosition =
					parentInternode.m_info.m_globalPosition + parentInternode.m_info.m_length *
					(parentInternode.m_info.m_globalRotation *
						glm::vec3(0, 0, -1));

			}

			m_shootSkeleton.m_min = glm::min(m_shootSkeleton.m_min, internodeInfo.m_globalPosition);
			m_shootSkeleton.m_max = glm::max(m_shootSkeleton.m_max, internodeInfo.m_globalPosition);
			const auto endPosition = internodeInfo.m_globalPosition + internodeInfo.m_length *
				(internodeInfo.m_globalRotation *
					glm::vec3(0, 0, -1));
			m_shootSkeleton.m_min = glm::min(m_shootSkeleton.m_min, endPosition);
			m_shootSkeleton.m_max = glm::max(m_shootSkeleton.m_max, endPosition);
		}
		SampleTemperature(globalTransform, climateModel);
		CalculateVigorRequirement(shootGrowthParameters, newShootGrowthRequirement);
		m_treeVolume.Clear();
		if (!sortedInternodeList.empty())
		{
			m_treeVolume.m_center = (m_shootSkeleton.m_max + m_shootSkeleton.m_min) * 0.5f;
			for (const auto& internodeHandle : sortedInternodeList)
			{
				auto& internode = m_shootSkeleton.RefNode(internodeHandle);
				if (!internode.IsEndNode()) continue;
				auto& internodeInfo = internode.m_info;
				const auto& point1 = internodeInfo.m_globalPosition;
				{
					const auto sectorIndex = m_treeVolume.GetSectorIndex(point1);
					const float currentDistance = glm::length(point1 - m_treeVolume.m_center);
					auto& distance = m_treeVolume.m_distances[sectorIndex];
					if (distance <
						currentDistance + m_treeVolume.m_offset)
						distance = currentDistance + m_treeVolume.m_offset;
				}
				/*{
					auto point2 = point1 + internodeInfo.m_length * (internodeInfo.m_globalRotation * glm::vec3(0, 0, -1));
					const auto sectorIndex = m_treeVolume.GetSectorIndex(point2);
					const float currentDistance = glm::length(point2 - m_treeVolume.m_center);
					auto& distance = m_treeVolume.m_distances[sectorIndex];
					if (distance <
						currentDistance + m_treeVolume.m_offset)
						distance = currentDistance + m_treeVolume.m_offset;
				}*/
			}
			m_treeVolume.Smooth();
		}
		m_treeVolume.m_hasData = true;
	};

	if (m_enableBranchCollisionDetection)
	{
		const float minRadius = shootGrowthParameters.m_endNodeThickness * 4.0f;
		CollisionDetection(minRadius, m_branchOctree, m_shootSkeleton);
	}
	m_internodeOrderCounts.clear();
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
		}
		m_shootSkeleton.CalculateFlows();
	};
#pragma endregion
#pragma endregion
	return treeStructureChanged;
}

bool TreeModel::ElongateRoot(SoilModel& soilModel, const float extendLength, NodeHandle rootNodeHandle, const RootGrowthParameters& rootGrowthParameters,
	float& collectedAuxin) {
	bool graphChanged = false;
	auto& rootNode = m_rootSkeleton.RefNode(rootNodeHandle);
	const auto rootNodeLength = rootGrowthParameters.m_rootNodeLength;
	auto& rootNodeData = rootNode.m_data;
	auto& rootNodeInfo = rootNode.m_info;
	rootNodeInfo.m_length += extendLength;
	float extraLength = rootNodeInfo.m_length - rootNodeLength;
	//If we need to add a new end node
	if (extraLength > 0) {
		graphChanged = true;
		rootNodeInfo.m_length = rootNodeLength;
		auto desiredGlobalRotation = rootNodeInfo.m_globalRotation * glm::quat(glm::vec3(
			glm::radians(rootGrowthParameters.GetRootApicalAngle(rootNode)), 0.0f,
			glm::radians(rootGrowthParameters.GetRootRollAngle(rootNode))));
		//Create new internode
		auto newRootNodeHandle = m_rootSkeleton.Extend(rootNodeHandle, false, m_rootSkeleton.RefFlow(rootNode.GetFlowHandle()).RefNodeHandles().size() > m_flowNodeLimit);
		auto& oldRootNode = m_rootSkeleton.RefNode(rootNodeHandle);
		auto& newRootNode = m_rootSkeleton.RefNode(newRootNodeHandle);
		newRootNode.m_data = {};
		newRootNode.m_data.m_order = oldRootNode.m_data.m_order;
		newRootNode.m_data.m_lateral = false;
		//Set and apply tropisms
		auto desiredGlobalFront = desiredGlobalRotation * glm::vec3(0, 0, -1);
		auto desiredGlobalUp = desiredGlobalRotation * glm::vec3(0, 1, 0);
		newRootNode.m_data.m_verticalTropism = oldRootNode.m_data.m_verticalTropism;
		newRootNode.m_data.m_horizontalTropism = oldRootNode.m_data.m_horizontalTropism;
		auto horizontalDirection = desiredGlobalFront;
		horizontalDirection.y = 0;
		if (glm::length(horizontalDirection) == 0) {
			auto x = glm::linearRand(0.0f, 1.0f);
			horizontalDirection = glm::vec3(x, 0, 1.0f - x);
		}
		horizontalDirection = glm::normalize(horizontalDirection);
		ApplyTropism(horizontalDirection, newRootNode.m_data.m_horizontalTropism, desiredGlobalFront,
			desiredGlobalUp);
		ApplyTropism(m_currentGravityDirection, newRootNode.m_data.m_verticalTropism,
			desiredGlobalFront, desiredGlobalUp);
		if (oldRootNode.m_data.m_soilDensity == 0.0f) {
			ApplyTropism(m_currentGravityDirection, 0.1f, desiredGlobalFront,
				desiredGlobalUp);
		}
		newRootNode.m_data.m_vigorFlow.m_developmentalVigorRequirementWeight = oldRootNode.m_data.m_vigorFlow.m_developmentalVigorRequirementWeight;

		newRootNode.m_data.m_inhibitorTarget = newRootNode.m_data.m_inhibitor = 0.0f;
		newRootNode.m_info.m_length = glm::clamp(extendLength, 0.0f, rootNodeLength);
		newRootNode.m_data.m_rootDistance = oldRootNode.m_data.m_rootDistance + newRootNode.m_info.m_length / rootGrowthParameters.m_rootNodeLength;
		newRootNode.m_info.m_thickness = rootGrowthParameters.m_endNodeThickness;
		newRootNode.m_info.m_globalRotation = glm::quatLookAt(desiredGlobalFront, desiredGlobalUp);
		newRootNode.m_info.m_localRotation =
			glm::inverse(oldRootNode.m_info.m_globalRotation) *
			newRootNode.m_info.m_globalRotation;
		if (extraLength > rootNodeLength) {
			float childAuxin = 0.0f;
			ElongateRoot(soilModel, extraLength - rootNodeLength, newRootNodeHandle, rootGrowthParameters, childAuxin);
			childAuxin *= rootGrowthParameters.m_apicalDominanceDistanceFactor;
			collectedAuxin += childAuxin;
			m_rootSkeleton.RefNode(newRootNodeHandle).m_data.m_inhibitorTarget = childAuxin;
		}
		else {
			newRootNode.m_data.m_inhibitorTarget = newRootNode.m_data.m_inhibitor = rootGrowthParameters.m_apicalDominance * glm::exp(-rootGrowthParameters.m_apicalDominanceAgeFactor * rootGrowthParameters.m_growthRate * m_age);
			collectedAuxin += newRootNode.m_data.m_inhibitor *= rootGrowthParameters.m_apicalDominanceDistanceFactor;
		}
	}
	else {
		//Otherwise, we add the inhibitor.
		collectedAuxin += rootGrowthParameters.m_apicalDominance * glm::exp(-rootGrowthParameters.m_apicalDominanceAgeFactor * rootGrowthParameters.m_growthRate * m_age);
	}
	return graphChanged;
}

bool TreeModel::ElongateInternode(float extendLength, NodeHandle internodeHandle,
	const ShootGrowthParameters& shootGrowthParameters, float& collectedInhibitor) {
	bool graphChanged = false;
	auto& internode = m_shootSkeleton.RefNode(internodeHandle);
	const auto internodeLength = shootGrowthParameters.m_internodeLength;
	auto& internodeData = internode.m_data;
	auto& internodeInfo = internode.m_info;
	internodeInfo.m_length += extendLength;
	const float extraLength = internodeInfo.m_length - internodeLength;
	auto& apicalBud = internodeData.m_buds.front();
	//If we need to add a new end node
	if (extraLength > 0) {
		graphChanged = true;
		apicalBud.m_status = BudStatus::Died;
		internodeInfo.m_length = internodeLength;
		auto desiredGlobalRotation = internodeInfo.m_globalRotation * apicalBud.m_localRotation;
		auto desiredGlobalFront = desiredGlobalRotation * glm::vec3(0, 0, -1);
		auto desiredGlobalUp = desiredGlobalRotation * glm::vec3(0, 1, 0);
		ApplyTropism(-m_currentGravityDirection, shootGrowthParameters.m_gravitropism, desiredGlobalFront,
			desiredGlobalUp);
		ApplyTropism(internodeData.m_lightDirection, shootGrowthParameters.m_phototropism,
			desiredGlobalFront, desiredGlobalUp);
		//Allocate Lateral bud for current internode
		{
			const auto lateralBudCount = shootGrowthParameters.m_lateralBudCount;
			const float turnAngle = glm::radians(360.0f / lateralBudCount);
			for (int i = 0; i < lateralBudCount; i++) {
				internodeData.m_buds.emplace_back();
				auto& lateralBud = internodeData.m_buds.back();
				lateralBud.m_type = BudType::Lateral;
				lateralBud.m_status = BudStatus::Dormant;
				lateralBud.m_localRotation = glm::vec3(
					glm::radians(shootGrowthParameters.GetDesiredBranchingAngle(internode)), 0.0f,
					i * turnAngle);
			}
		}
		//Allocate Fruit bud for current internode
		{
			const auto fruitBudCount = shootGrowthParameters.m_fruitBudCount;
			for (int i = 0; i < fruitBudCount; i++) {
				internodeData.m_buds.emplace_back();
				auto& fruitBud = internodeData.m_buds.back();
				fruitBud.m_type = BudType::Fruit;
				fruitBud.m_status = BudStatus::Dormant;
				fruitBud.m_localRotation = glm::vec3(
					glm::radians(shootGrowthParameters.GetDesiredBranchingAngle(internode)), 0.0f,
					glm::radians(glm::linearRand(0.0f, 360.0f)));
			}
		}
		//Allocate Leaf bud for current internode
		{
			const auto leafBudCount = shootGrowthParameters.m_leafBudCount;
			for (int i = 0; i < leafBudCount; i++) {
				internodeData.m_buds.emplace_back();
				auto& leafBud = internodeData.m_buds.back();
				leafBud.m_type = BudType::Leaf;
				leafBud.m_status = BudStatus::Dormant;
				leafBud.m_localRotation = glm::vec3(
					glm::radians(shootGrowthParameters.GetDesiredBranchingAngle(internode)), 0.0f,
					glm::radians(glm::linearRand(0.0f, 360.0f)));
			}
		}
		//Create new internode
		const auto newInternodeHandle = m_shootSkeleton.Extend(internodeHandle, false, m_shootSkeleton.RefFlow(internode.GetFlowHandle()).RefNodeHandles().size() > m_flowNodeLimit);
		const auto& oldInternode = m_shootSkeleton.RefNode(internodeHandle);
		auto& newInternode = m_shootSkeleton.RefNode(newInternodeHandle);
		newInternode.m_data = {};
		newInternode.m_data.m_order = oldInternode.m_data.m_order;
		newInternode.m_data.m_lateral = false;
		newInternode.m_data.m_inhibitor = 0.0f;
		newInternode.m_info.m_length = glm::clamp(extendLength, 0.0f, internodeLength);
		newInternode.m_info.m_thickness = shootGrowthParameters.m_endNodeThickness;
		newInternode.m_info.m_globalRotation = glm::quatLookAt(desiredGlobalFront, desiredGlobalUp);
		newInternode.m_info.m_localRotation = newInternode.m_data.m_desiredLocalRotation =
			glm::inverse(oldInternode.m_info.m_globalRotation) *
			newInternode.m_info.m_globalRotation;
		//Allocate apical bud for new internode
		newInternode.m_data.m_buds.emplace_back();
		auto& newApicalBud = newInternode.m_data.m_buds.back();
		newApicalBud.m_type = BudType::Apical;
		newApicalBud.m_status = BudStatus::Dormant;
		newApicalBud.m_localRotation = glm::vec3(
			glm::radians(shootGrowthParameters.GetDesiredApicalAngle(newInternode)), 0.0f,
			shootGrowthParameters.GetDesiredRollAngle(newInternode));
		if (extraLength > internodeLength) {
			float childInhibitor = 0.0f;
			ElongateInternode(extraLength - internodeLength, newInternodeHandle, shootGrowthParameters, childInhibitor);
			childInhibitor *= shootGrowthParameters.m_apicalDominanceDistanceFactor;
			collectedInhibitor += childInhibitor;
			m_shootSkeleton.RefNode(newInternodeHandle).m_data.m_inhibitor = childInhibitor;
		}
		else {
			newInternode.m_data.m_inhibitor = shootGrowthParameters.m_apicalDominance * glm::exp(-shootGrowthParameters.m_apicalDominanceAgeFactor * shootGrowthParameters.m_growthRate * m_age);
			collectedInhibitor += newInternode.m_data.m_inhibitor *= shootGrowthParameters.m_apicalDominanceDistanceFactor;
		}
	}
	else {
		//Otherwise, we add the inhibitor.
		collectedInhibitor += shootGrowthParameters.m_apicalDominance * glm::exp(-shootGrowthParameters.m_apicalDominanceAgeFactor * shootGrowthParameters.m_growthRate * m_age);
	}
	return graphChanged;
}

inline bool TreeModel::GrowRootNode(SoilModel& soilModel, NodeHandle rootNodeHandle, const RootGrowthParameters& rootGrowthParameters)
{
	bool graphChanged = false;
	{
		auto& rootNode = m_rootSkeleton.RefNode(rootNodeHandle);
		auto& rootNodeData = rootNode.m_data;
		rootNodeData.m_inhibitorTarget = 0;
		for (const auto& childHandle : rootNode.RefChildHandles()) {
			rootNodeData.m_inhibitorTarget += m_rootSkeleton.RefNode(childHandle).m_data.m_inhibitor *
				rootGrowthParameters.m_apicalDominanceDistanceFactor;
		}
	}

	{

		auto& rootNode = m_rootSkeleton.RefNode(rootNodeHandle);
		//1. Elongate current node.
		const float availableMaintenanceVigor = rootNode.m_data.m_vigorSink.GetAvailableMaintenanceVigor();
		float availableDevelopmentalVigor = rootNode.m_data.m_vigorSink.GetAvailableDevelopmentalVigor();
		const float developmentVigor = rootNode.m_data.m_vigorSink.SubtractAllDevelopmentalVigor();
		
		if (rootNode.RefChildHandles().empty())
		{
			const float extendLength = developmentVigor / rootGrowthParameters.m_rootNodeVigorRequirement * rootGrowthParameters.m_rootNodeLength;
			//Remove development vigor from sink since it's used for elongation
			float collectedAuxin = 0.0f;
			const auto dd = rootGrowthParameters.m_apicalDominanceDistanceFactor;
			graphChanged = ElongateRoot(soilModel, extendLength, rootNodeHandle, rootGrowthParameters, collectedAuxin) || graphChanged;
			m_rootSkeleton.RefNode(rootNodeHandle).m_data.m_inhibitorTarget += collectedAuxin * dd;

			const float maintenanceVigor = m_rootSkeleton.RefNode(rootNodeHandle).m_data.m_vigorSink.SubtractVigor(availableMaintenanceVigor);
		}
		else
		{
			//2. Form new shoot if necessary
			float branchingProb = m_rootNodeDevelopmentRate * rootGrowthParameters.m_growthRate * rootGrowthParameters.m_branchingProbability;
			if (rootNode.m_data.m_inhibitor > 0.0f) branchingProb *= glm::exp(-rootNode.m_data.m_inhibitor);
			//More nitrite, more likely to form new shoot.
			if (branchingProb >= glm::linearRand(0.0f, 1.0f)) {
				const auto newRootNodeHandle = m_rootSkeleton.Extend(rootNodeHandle, true);
				auto& oldRootNode = m_rootSkeleton.RefNode(rootNodeHandle);
				auto& newRootNode = m_rootSkeleton.RefNode(newRootNodeHandle);
				newRootNode.m_data = {};
				newRootNode.m_data.m_order = oldRootNode.m_data.m_order + 1;
				newRootNode.m_data.m_lateral = true;
				//Assign new tropism for new shoot based on parent node. The tropism switching happens here.
				rootGrowthParameters.SetTropisms(oldRootNode, newRootNode);
				newRootNode.m_info.m_length = 0.0f;
				newRootNode.m_info.m_thickness = rootGrowthParameters.m_endNodeThickness;
				newRootNode.m_info.m_localRotation =
					glm::quat(glm::vec3(
						glm::radians(rootGrowthParameters.GetRootBranchingAngle(newRootNode)),
						glm::radians(glm::linearRand(0.0f, 360.0f)), 0.0f));
				auto globalRotation = oldRootNode.m_info.m_globalRotation * newRootNode.m_info.m_localRotation;
				auto front = globalRotation * glm::vec3(0, 0, -1);
				auto up = globalRotation * glm::vec3(0, 1, 0);
				auto angleTowardsUp = glm::degrees(glm::acos(glm::dot(front, glm::vec3(0, 1, 0))));
				const float maxAngle = 60.0f;
				if (angleTowardsUp < maxAngle) {
					const glm::vec3 left = glm::cross(front, glm::vec3(0, -1, 0));
					front = glm::rotate(front, glm::radians(maxAngle - angleTowardsUp), left);

					up = glm::normalize(glm::cross(glm::cross(front, up), front));
					globalRotation = glm::quatLookAt(front, up);
					newRootNode.m_info.m_localRotation = glm::inverse(oldRootNode.m_info.m_globalRotation) * globalRotation;
					front = globalRotation * glm::vec3(0, 0, -1);
				}
				const float maintenanceVigor = m_rootSkeleton.RefNode(rootNodeHandle).m_data.m_vigorSink.SubtractVigor(availableMaintenanceVigor);
			}
		}
		
		
	}


	{
		auto& rootNode = m_rootSkeleton.RefNode(rootNodeHandle);
		auto& rootNodeData = rootNode.m_data;
		rootNodeData.m_inhibitor = (rootNodeData.m_inhibitor + rootNodeData.m_inhibitorTarget) / 2.0f;
	}
	return graphChanged;
}

bool TreeModel::GrowInternode(ClimateModel& climateModel, NodeHandle internodeHandle, const ShootGrowthParameters& shootGrowthParameters) {
	bool graphChanged = false;
	{
		auto& internode = m_shootSkeleton.RefNode(internodeHandle);
		auto& internodeData = internode.m_data;
		internodeData.m_inhibitor = 0;
		for (const auto& childHandle : internode.RefChildHandles()) {
			internodeData.m_inhibitor += m_shootSkeleton.RefNode(childHandle).m_data.m_inhibitor *
				shootGrowthParameters.m_apicalDominanceDistanceFactor;
		}
	}
	auto& buds = m_shootSkeleton.RefNode(internodeHandle).m_data.m_buds;
	for (auto& bud : buds) {
		auto& internode = m_shootSkeleton.RefNode(internodeHandle);
		auto& internodeData = internode.m_data;
		auto& internodeInfo = internode.m_info;

		//auto killProbability = shootGrowthParameters.m_growthRate * shootGrowthParameters.m_budKillProbability;
		//if (internodeData.m_rootDistance < 1.0f) killProbability = 0.0f;
		//if (bud.m_status == BudStatus::Dormant && killProbability > glm::linearRand(0.0f, 1.0f)) {
		//	bud.m_status = BudStatus::Removed;
		//}
		//if (bud.m_status == BudStatus::Removed) continue;

		//Calculate vigor used for maintenance and development.
		const float availableMaintenanceVigor = bud.m_vigorSink.GetAvailableMaintenanceVigor();
		if (bud.m_vigorSink.GetDesiredMaintenanceVigorRequirement() != 0.0f) bud.m_drought = glm::clamp(1.0f - (1.0f - bud.m_drought) * availableMaintenanceVigor / bud.m_vigorSink.GetDesiredMaintenanceVigorRequirement(), 0.0f, 1.0f);
		//Subtract maintenance vigor and the rest is for development.
		const float developmentalVigor = bud.m_vigorSink.SubtractAllDevelopmentalVigor();
		const float maintenanceVigor = bud.m_vigorSink.SubtractAllVigor();
		if (bud.m_type == BudType::Apical && bud.m_status == BudStatus::Dormant) {
			const float elongateLength = developmentalVigor / shootGrowthParameters.m_internodeVigorRequirement * shootGrowthParameters.m_internodeLength;
			//Use up the vigor stored in this bud.
			float collectedInhibitor = 0.0f;
			const auto dd = shootGrowthParameters.m_apicalDominanceDistanceFactor;
			graphChanged = ElongateInternode(elongateLength, internodeHandle, shootGrowthParameters, collectedInhibitor) || graphChanged;
			m_shootSkeleton.RefNode(internodeHandle).m_data.m_inhibitor += collectedInhibitor * dd;
		}
		if (bud.m_type == BudType::Lateral && bud.m_status == BudStatus::Dormant) {
			const auto& probabilityRange = shootGrowthParameters.m_lateralBudFlushingProbabilityTemperatureRange;
			float flushProbability = m_internodeDevelopmentRate * shootGrowthParameters.m_growthRate * glm::mix(probabilityRange.x, probabilityRange.y,
				glm::clamp((internodeData.m_temperature - probabilityRange.z) / (probabilityRange.w - probabilityRange.z), 0.0f, 1.0f));
			if (internodeData.m_inhibitor > 0.0f) flushProbability *= glm::exp(-internodeData.m_inhibitor);
			flushProbability *= glm::pow(internodeData.m_lightIntensity, shootGrowthParameters.m_lateralBudLightingFactor);
			if (flushProbability >= glm::linearRand(0.0f, 1.0f)) {
				graphChanged = true;
				bud.m_status = BudStatus::Flushed;
				//Prepare information for new internode
				auto desiredGlobalRotation = internodeInfo.m_globalRotation * bud.m_localRotation;
				auto desiredGlobalFront = desiredGlobalRotation * glm::vec3(0, 0, -1);
				auto desiredGlobalUp = desiredGlobalRotation * glm::vec3(0, 1, 0);
				ApplyTropism(-m_currentGravityDirection, shootGrowthParameters.m_gravitropism, desiredGlobalFront,
					desiredGlobalUp);
				ApplyTropism(internodeData.m_lightDirection, shootGrowthParameters.m_phototropism,
					desiredGlobalFront, desiredGlobalUp);
				//Create new internode
				const auto newInternodeHandle = m_shootSkeleton.Extend(internodeHandle, true);
				auto& oldInternode = m_shootSkeleton.RefNode(internodeHandle);
				auto& newInternode = m_shootSkeleton.RefNode(newInternodeHandle);
				newInternode.m_data = {};
				newInternode.m_data.m_order = oldInternode.m_data.m_order + 1;
				newInternode.m_data.m_lateral = true;
				newInternode.m_info.m_length = 0.0f;
				newInternode.m_info.m_thickness = shootGrowthParameters.m_endNodeThickness;
				newInternode.m_info.m_localRotation = newInternode.m_data.m_desiredLocalRotation =
					glm::inverse(oldInternode.m_info.m_globalRotation) *
					glm::quatLookAt(desiredGlobalFront, desiredGlobalUp);
				//Allocate apical bud
				newInternode.m_data.m_buds.emplace_back();
				auto& apicalBud = newInternode.m_data.m_buds.back();
				apicalBud.m_type = BudType::Apical;
				apicalBud.m_status = BudStatus::Dormant;
				apicalBud.m_localRotation = glm::vec3(
					glm::radians(shootGrowthParameters.GetDesiredApicalAngle(newInternode)), 0.0f,
					shootGrowthParameters.GetDesiredRollAngle(newInternode));
			}
		}
		else if (climateModel.m_days % 360 == 0)
		{
			if (bud.m_type == BudType::Fruit || bud.m_type == BudType::Leaf)
			{
				bud.m_status = BudStatus::Dormant;
				bud.m_maturity = 0.0f;
				bud.m_drought = 0.0f;
				bud.m_chlorophyll = 1.0f;
				bud.m_reproductiveModuleTransform = glm::mat4(0.0f);
			}
		}
		else if (bud.m_type == BudType::Fruit)
		{
			if (bud.m_status == BudStatus::Dormant) {

				const auto& probabilityRange = shootGrowthParameters.m_fruitBudFlushingProbabilityTemperatureRange;
				float flushProbability = m_internodeDevelopmentRate * shootGrowthParameters.m_growthRate * glm::mix(probabilityRange.x, probabilityRange.y,
					glm::clamp((internodeData.m_temperature - probabilityRange.z) / (probabilityRange.w - probabilityRange.z), 0.0f, 1.0f));
				flushProbability *= glm::pow(internodeData.m_lightIntensity, shootGrowthParameters.m_fruitBudLightingFactor);
				if (flushProbability >= glm::linearRand(0.0f, 1.0f))
				{
					bud.m_status = BudStatus::Flushed;
					bud.m_maturity = 0.0f;
					bud.m_drought = 0.0f;
					bud.m_chlorophyll = 1.0f;
					bud.m_reproductiveModuleTransform = glm::mat4(0.0f);
				}
			}
			else if (bud.m_status == BudStatus::Flushed)
			{
				m_fruitCount++;
				//Make the fruit larger;
				bud.m_maturity += 0.05f;
				bud.m_maturity = glm::clamp(bud.m_maturity, 0.0f, 1.0f);
				auto fruitSize = shootGrowthParameters.m_maxFruitSize * bud.m_maturity;
				glm::quat rotation = internodeInfo.m_globalRotation * bud.m_localRotation;
				auto front = rotation * glm::vec3(0, 0, -1);
				ApplyTropism(glm::vec3(0, -1, 0), 0.25f, rotation);
				auto fruitPosition = front * (fruitSize.z * 1.5f);
				bud.m_reproductiveModuleTransform = glm::translate(fruitPosition) * glm::mat4_cast(glm::quat(glm::vec3(0.0f))) * glm::scale(fruitSize);
			}
		}
		else if (bud.m_type == BudType::Leaf)
		{
			if (bud.m_status == BudStatus::Dormant) {
				const auto& probabilityRange = shootGrowthParameters.m_fruitBudFlushingProbabilityTemperatureRange;
				float flushProbability = m_internodeDevelopmentRate * shootGrowthParameters.m_growthRate * glm::mix(probabilityRange.x, probabilityRange.y,
					glm::clamp((internodeData.m_temperature - probabilityRange.z) / (probabilityRange.w - probabilityRange.z), 0.0f, 1.0f));
				flushProbability *= glm::pow(internodeData.m_lightIntensity, shootGrowthParameters.m_leafBudLightingFactor);
				if (internodeData.m_maxDistanceToAnyBranchEnd < shootGrowthParameters.m_leafDistanceToBranchEndLimit && flushProbability >= glm::linearRand(0.0f, 1.0f))
				{
					bud.m_status = BudStatus::Flushed;
					bud.m_maturity = 0.0f;
					bud.m_drought = 0.0f;
					bud.m_chlorophyll = 1.0f;
					bud.m_reproductiveModuleTransform = glm::mat4(0.0f);
				}
			}
			else if (bud.m_status == BudStatus::Flushed)
			{
				m_leafCount++;
				//Make the leaf larger
				bud.m_maturity += 0.05f;
				bud.m_maturity = glm::clamp(bud.m_maturity, 0.0f, 1.0f);
				auto leafSize = shootGrowthParameters.m_maxLeafSize * bud.m_maturity;
				glm::quat rotation = internodeInfo.m_globalRotation * bud.m_localRotation;
				auto front = rotation * glm::vec3(0, 0, -1);
				ApplyTropism(glm::vec3(0, -1, 0), 0.9f, rotation);
				auto foliagePosition = front * (leafSize.z * 1.5f);
				bud.m_reproductiveModuleTransform = glm::translate(foliagePosition) * glm::mat4_cast(rotation) * glm::scale(leafSize);

				if (climateModel.m_days % 360 > 180 && internodeData.m_temperature < shootGrowthParameters.m_leafChlorophyllSynthesisFactorTemperature)
				{
					bud.m_chlorophyll -= shootGrowthParameters.m_growthRate * shootGrowthParameters.m_leafChlorophyllLoss;
					bud.m_chlorophyll = glm::clamp(bud.m_chlorophyll, 0.0f, 1.0f);
				}
				if (bud.m_chlorophyll == 0.0f)
				{
					auto dropProbability = shootGrowthParameters.m_growthRate * shootGrowthParameters.m_leafFallProbability;
					if (dropProbability >= glm::linearRand(0.0f, 1.0f))
					{
						bud.m_status = BudStatus::Died;
					}
				}
			}
		}
	}
	return graphChanged;
}

void TreeModel::CalculateThickness(NodeHandle rootNodeHandle, const RootGrowthParameters& rootGrowthParameters)
{
	auto& rootNode = m_rootSkeleton.RefNode(rootNodeHandle);
	auto& rootNodeData = rootNode.m_data;
	auto& rootNodeInfo = rootNode.m_info;
	rootNodeData.m_descendentTotalBiomass = 0;
	float maxDistanceToAnyBranchEnd = 0;
	float childThicknessCollection = 0.0f;
	//std::set<float> thicknessCollection;
	int maxChildHandle = -1;
	//float maxChildBiomass = 999.f;
	int minChildOrder = 999;
	for (const auto& i : rootNode.RefChildHandles()) {
		const auto& childRootNode = m_rootSkeleton.RefNode(i);
		const float childMaxDistanceToAnyBranchEnd =
			childRootNode.m_data.m_maxDistanceToAnyBranchEnd +
			childRootNode.m_info.m_length / rootGrowthParameters.m_rootNodeLength;
		maxDistanceToAnyBranchEnd = glm::max(maxDistanceToAnyBranchEnd, childMaxDistanceToAnyBranchEnd);
		childThicknessCollection += glm::pow(childRootNode.m_info.m_thickness,
			1.0f / rootGrowthParameters.m_thicknessAccumulationFactor);
		//thicknessCollection.emplace();
		if (childRootNode.m_data.m_order > minChildOrder)
		{
			minChildOrder = childRootNode.m_data.m_order;
			maxChildHandle = i;
		}
	}

	//int addedIndex = 0;
	//for (auto i = thicknessCollection.begin(); i != thicknessCollection.end(); ++i)
	//{
	//	childThicknessCollection += *i;
	//	addedIndex++;
		//if (addedIndex > 1) break;
	//}

	childThicknessCollection += rootGrowthParameters.m_thicknessAccumulateAgeFactor * rootGrowthParameters.m_endNodeThickness * rootGrowthParameters.m_growthRate * rootNodeData.m_age;

	rootNodeData.m_maxDistanceToAnyBranchEnd = maxDistanceToAnyBranchEnd;
	if (childThicknessCollection != 0.0f) {
		const auto newThickness = glm::pow(childThicknessCollection,
			rootGrowthParameters.m_thicknessAccumulationFactor);
		rootNodeInfo.m_thickness = glm::max(rootNodeInfo.m_thickness, newThickness);
	}
	else
	{
		rootNodeInfo.m_thickness = glm::max(rootNodeInfo.m_thickness, rootGrowthParameters.m_endNodeThickness);
	}

	rootNodeData.m_biomass =
		rootNodeInfo.m_thickness / rootGrowthParameters.m_endNodeThickness * rootNodeInfo.m_length /
		rootGrowthParameters.m_rootNodeLength;
	for (const auto& i : rootNode.RefChildHandles()) {
		auto& childRootNode = m_rootSkeleton.RefNode(i);
		rootNodeData.m_descendentTotalBiomass +=
			childRootNode.m_data.m_descendentTotalBiomass +
			childRootNode.m_data.m_biomass;
		childRootNode.m_data.m_isMaxChild = i == maxChildHandle;
	}
}

void TreeModel::AggregateInternodeVigorRequirement()
{
	const auto& sortedInternodeList = m_shootSkeleton.RefSortedNodeList();
	for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); it++) {
		auto& internode = m_shootSkeleton.RefNode(*it);
		auto& internodeData = internode.m_data;
		if (!internode.IsEndNode()) {
			//If current node is not end node
			for (const auto& i : internode.RefChildHandles()) {
				auto& childInternode = m_shootSkeleton.RefNode(i);
				internodeData.m_vigorFlow.m_subtreeDevelopmentalVigorRequirementWeight +=
					childInternode.m_data.m_vigorFlow.m_developmentalVigorRequirementWeight
					+ childInternode.m_data.m_vigorFlow.m_subtreeDevelopmentalVigorRequirementWeight;
				internodeData.m_vigorFlow.m_subtreeMaintenanceVigorRequirementWeight +=
					childInternode.m_data.m_vigorFlow.m_maintenanceVigorRequirementWeight
					+ childInternode.m_data.m_vigorFlow.m_subtreeMaintenanceVigorRequirementWeight;
			}
		}
	}
}

void TreeModel::AggregateRootVigorRequirement()
{
	const auto& sortedRootNodeList = m_rootSkeleton.RefSortedNodeList();

	for (auto it = sortedRootNodeList.rbegin(); it != sortedRootNodeList.rend(); ++it) {
		auto& rootNode = m_rootSkeleton.RefNode(*it);
		auto& rootNodeData = rootNode.m_data;
		rootNodeData.m_vigorFlow.m_subtreeDevelopmentalVigorRequirementWeight = 0.0f;
		rootNodeData.m_vigorFlow.m_subtreeMaintenanceVigorRequirementWeight = 0.0f;
		if (!rootNode.IsEndNode()) {
			//If current node is not end node
			for (const auto& i : rootNode.RefChildHandles()) {
				const auto& childInternode = m_rootSkeleton.RefNode(i);
				rootNodeData.m_vigorFlow.m_subtreeDevelopmentalVigorRequirementWeight +=
					childInternode.m_data.m_vigorFlow.m_developmentalVigorRequirementWeight + childInternode.m_data.m_vigorFlow.m_subtreeDevelopmentalVigorRequirementWeight;
				rootNodeData.m_vigorFlow.m_subtreeMaintenanceVigorRequirementWeight +=
					childInternode.m_data.m_vigorFlow.m_maintenanceVigorRequirementWeight + childInternode.m_data.m_vigorFlow.m_subtreeMaintenanceVigorRequirementWeight;
			}
		}
	}
}

inline void TreeModel::AllocateShootVigor(const ShootGrowthParameters& shootGrowthParameters)
{
	const auto& sortedInternodeList = m_shootSkeleton.RefSortedNodeList();
	//Go from rooting point to all end nodes
	const float apicalControl =
		1.0f + shootGrowthParameters.m_apicalControl
		* glm::exp(-shootGrowthParameters.m_apicalControlAgeFactor * m_age * shootGrowthParameters.m_growthRate);
	float actualAllocatedDevelopmentVigor = 0.0f;



	for (const auto& internodeHandle : sortedInternodeList) {
		auto& internode = m_shootSkeleton.RefNode(internodeHandle);
		auto& internodeData = internode.m_data;
		auto& internodeVigorFlow = internodeData.m_vigorFlow;
		//If this is the first node (node at the rooting point)
		if (internode.GetParentHandle() == -1) {
			internodeVigorFlow.m_allocatedVigor = 0.0f;
			internodeVigorFlow.m_subTreeAllocatedVigor = 0.0f;
			if (m_shootGrowthRequirement.m_maintenanceVigor + m_shootGrowthRequirement.m_developmentalVigor != 0.0f) {
				//We firstly determine how much resource being allocated to shoot system on system graph level.
				const auto shootVigor = m_plantVigor.m_shootVigor;
				const float totalRequirement = internodeVigorFlow.m_maintenanceVigorRequirementWeight + internodeVigorFlow.m_developmentalVigorRequirementWeight +
					internodeVigorFlow.m_subtreeDevelopmentalVigorRequirementWeight + internodeVigorFlow.m_subtreeMaintenanceVigorRequirementWeight;

				if (totalRequirement != 0.0f) {
					//The root internode firstly extract it's own resources needed for itself.
					internodeVigorFlow.m_allocatedVigor = shootVigor *
						(internodeVigorFlow.m_maintenanceVigorRequirementWeight + internodeVigorFlow.m_developmentalVigorRequirementWeight)
						/ totalRequirement;
					if (shootGrowthParameters.m_maintenanceVigorRequirementPriority)
					{
						internodeVigorFlow.m_allocatedVigor = glm::min(shootVigor, glm::max(internodeVigorFlow.m_allocatedVigor, internodeVigorFlow.m_maintenanceVigorRequirementWeight));
					}
				}
				//The rest resource will be distributed to the descendants. 
				internodeVigorFlow.m_subTreeAllocatedVigor = shootVigor - internodeVigorFlow.m_allocatedVigor;
			}
			//The buds will get its own resources
			for (auto& bud : internodeData.m_buds) {
				if (internodeVigorFlow.m_maintenanceVigorRequirementWeight + internodeVigorFlow.m_developmentalVigorRequirementWeight != 0.0f) {
					//The vigor gets allocated and stored eventually into the buds
					bud.m_vigorSink.AddVigor(internodeVigorFlow.m_allocatedVigor *
						(bud.m_vigorSink.GetMaintenanceVigorRequirement() + bud.m_vigorSink.GetDevelopmentalVigorRequirement())
						/ (internodeVigorFlow.m_maintenanceVigorRequirementWeight + internodeVigorFlow.m_developmentalVigorRequirementWeight));
				}
			}
		}
		if (internodeVigorFlow.m_subTreeAllocatedVigor != 0.0f) {
			float childDevelopmentalVigorRequirementWeightSum = 0.0f;
			float childMaintenanceVigorRequirementWeightSum = 0.0f;
			for (const auto& i : internode.RefChildHandles()) {
				const auto& childInternode = m_shootSkeleton.RefNode(i);
				const auto& childInternodeData = childInternode.m_data;
				auto& childInternodeVigorFlow = childInternodeData.m_vigorFlow;
				float childDevelopmentalVigorRequirementWeight = 0.0f;
				if (internodeVigorFlow.m_subtreeDevelopmentalVigorRequirementWeight != 0.0f)childDevelopmentalVigorRequirementWeight = (childInternodeVigorFlow.m_developmentalVigorRequirementWeight + childInternodeVigorFlow.m_subtreeDevelopmentalVigorRequirementWeight)
					/ internodeVigorFlow.m_subtreeDevelopmentalVigorRequirementWeight;
				float childMaintenanceVigorRequirementWeight = 0.0f;
				if (internodeVigorFlow.m_subtreeMaintenanceVigorRequirementWeight != 0.0f) childMaintenanceVigorRequirementWeight = (childInternodeVigorFlow.m_maintenanceVigorRequirementWeight + childInternodeVigorFlow.m_subtreeMaintenanceVigorRequirementWeight)
					/ internodeVigorFlow.m_subtreeMaintenanceVigorRequirementWeight;
				//Perform Apical control here.
				if (childInternodeData.m_isMaxChild) childDevelopmentalVigorRequirementWeight *= apicalControl;
				childDevelopmentalVigorRequirementWeightSum += childDevelopmentalVigorRequirementWeight;
				childMaintenanceVigorRequirementWeightSum += childMaintenanceVigorRequirementWeight;
			}
			float internodeSubtreeAllocatedMaintenanceVigor = 0.0f;
			if (!shootGrowthParameters.m_maintenanceVigorRequirementPriority) {
				const float totalRequirement = internodeVigorFlow.m_subtreeDevelopmentalVigorRequirementWeight + internodeVigorFlow.m_subtreeMaintenanceVigorRequirementWeight;
				if (totalRequirement != 0.0f) {
					internodeSubtreeAllocatedMaintenanceVigor =
						internodeVigorFlow.m_subTreeAllocatedVigor *
						internodeVigorFlow.m_subtreeMaintenanceVigorRequirementWeight
						/ totalRequirement;
				}
			}
			else
			{
				internodeSubtreeAllocatedMaintenanceVigor = glm::min(internodeVigorFlow.m_subTreeAllocatedVigor,
					internodeVigorFlow.m_subtreeMaintenanceVigorRequirementWeight);
			}
			const float internodeSubtreeAllocatedDevelopmentVigor = internodeVigorFlow.m_subTreeAllocatedVigor - internodeSubtreeAllocatedMaintenanceVigor;
			for (const auto& i : internode.RefChildHandles()) {
				auto& childInternode = m_shootSkeleton.RefNode(i);
				auto& childInternodeData = childInternode.m_data;
				auto& childInternodeVigorFlow = childInternodeData.m_vigorFlow;
				float childDevelopmentalVigorRequirementWeight = 0.0f;
				if (internodeVigorFlow.m_subtreeDevelopmentalVigorRequirementWeight != 0.0f)
					childDevelopmentalVigorRequirementWeight = (childInternodeVigorFlow.m_developmentalVigorRequirementWeight + childInternodeVigorFlow.m_subtreeDevelopmentalVigorRequirementWeight)
					/ internodeVigorFlow.m_subtreeDevelopmentalVigorRequirementWeight;

				float childMaintenanceVigorRequirementWeight = 0.0f;
				if (internodeVigorFlow.m_subtreeMaintenanceVigorRequirementWeight != 0.0f)
					childMaintenanceVigorRequirementWeight = (childInternodeVigorFlow.m_maintenanceVigorRequirementWeight + childInternodeVigorFlow.m_subtreeMaintenanceVigorRequirementWeight)
					/ internodeVigorFlow.m_subtreeMaintenanceVigorRequirementWeight;

				//Re-perform apical control.
				if (childInternodeData.m_isMaxChild) childDevelopmentalVigorRequirementWeight *= apicalControl;
				
				//Firstly calculate total amount of maintenance vigor belongs to this child from internode.
				float childTotalAllocatedMaintenanceVigor = 0.0f;
				if (childMaintenanceVigorRequirementWeightSum != 0.0f) childTotalAllocatedMaintenanceVigor = internodeSubtreeAllocatedMaintenanceVigor *
					childMaintenanceVigorRequirementWeight / childMaintenanceVigorRequirementWeightSum;

				//Then calculate total amount of development vigor belongs to this child from internode received vigor for its children.
				float childTotalAllocatedDevelopmentVigor = 0.0f;
				if (childDevelopmentalVigorRequirementWeightSum != 0.0f) childTotalAllocatedDevelopmentVigor = internodeSubtreeAllocatedDevelopmentVigor *
					childDevelopmentalVigorRequirementWeight / childDevelopmentalVigorRequirementWeightSum;

				//Combine maintenance and development allocated to this child.
				childInternodeVigorFlow.m_allocatedVigor = 0.0f;
				if (childInternodeVigorFlow.m_maintenanceVigorRequirementWeight + childInternodeVigorFlow.m_subtreeMaintenanceVigorRequirementWeight != 0.0f) {
					childInternodeVigorFlow.m_allocatedVigor += childTotalAllocatedMaintenanceVigor *
						childInternodeVigorFlow.m_maintenanceVigorRequirementWeight
						/ (childInternodeVigorFlow.m_maintenanceVigorRequirementWeight + childInternodeVigorFlow.m_subtreeMaintenanceVigorRequirementWeight);
				}
				if (childInternodeVigorFlow.m_developmentalVigorRequirementWeight + childInternodeVigorFlow.m_subtreeDevelopmentalVigorRequirementWeight != 0.0f) {
					childInternodeVigorFlow.m_allocatedVigor += childTotalAllocatedDevelopmentVigor *
						childInternodeVigorFlow.m_developmentalVigorRequirementWeight
						/ (childInternodeVigorFlow.m_developmentalVigorRequirementWeight + childInternodeVigorFlow.m_subtreeDevelopmentalVigorRequirementWeight);
				}
				childInternodeVigorFlow.m_subTreeAllocatedVigor =
					childTotalAllocatedMaintenanceVigor + childTotalAllocatedDevelopmentVigor - childInternodeVigorFlow.m_allocatedVigor;

				for (auto& bud : childInternodeData.m_buds) {
					//The vigor gets allocated and stored eventually into the buds
					bud.m_vigorSink.AddVigor(childInternodeVigorFlow.m_allocatedVigor*
						(bud.m_vigorSink.GetMaintenanceVigorRequirement() + bud.m_vigorSink.GetDevelopmentalVigorRequirement())
						/ (childInternodeVigorFlow.m_maintenanceVigorRequirementWeight + childInternodeVigorFlow.m_developmentalVigorRequirementWeight));
				}
			}
		}
		else
		{
			for (const auto& i : internode.RefChildHandles())
			{
				auto& childInternode = m_shootSkeleton.RefNode(i);
				auto& childInternodeData = childInternode.m_data;
				auto& childInternodeVigorFlow = childInternodeData.m_vigorFlow;
				childInternodeVigorFlow.m_allocatedVigor = childInternodeVigorFlow.m_subTreeAllocatedVigor = 0.0f;
			}
		}
		for (auto& bud : internodeData.m_buds) {
			if (bud.m_type == BudType::Apical && bud.m_status == BudStatus::Dormant) {
				actualAllocatedDevelopmentVigor += bud.m_vigorSink.GetAvailableDevelopmentalVigor();
			}
		}
	}

	m_internodeDevelopmentRate = actualAllocatedDevelopmentVigor / m_shootGrowthRequirement.m_developmentalVigor;
}

void TreeModel::CalculateThicknessAndSagging(NodeHandle internodeHandle,
	const ShootGrowthParameters& shootGrowthParameters) {
	auto& internode = m_shootSkeleton.RefNode(internodeHandle);
	auto& internodeData = internode.m_data;
	auto& internodeInfo = internode.m_info;
	internodeData.m_descendentTotalBiomass = internodeData.m_biomass = 0.0f;
	float maxDistanceToAnyBranchEnd = 0;
	float childThicknessCollection = 0.0f;

	int maxChildHandle = -1;
	//float maxChildBiomass = 999.f;
	int minChildOrder = 999;
	for (const auto& i : internode.RefChildHandles()) {
		auto& childInternode = m_shootSkeleton.RefNode(i);
		const float childMaxDistanceToAnyBranchEnd =
			childInternode.m_data.m_maxDistanceToAnyBranchEnd +
			childInternode.m_info.m_length / shootGrowthParameters.m_internodeLength;
		maxDistanceToAnyBranchEnd = glm::max(maxDistanceToAnyBranchEnd, childMaxDistanceToAnyBranchEnd);

		childThicknessCollection += glm::pow(childInternode.m_info.m_thickness,
			1.0f / shootGrowthParameters.m_thicknessAccumulationFactor);

		if (childInternode.m_data.m_order < minChildOrder)
		{
			minChildOrder = childInternode.m_data.m_order;
			maxChildHandle = i;
		}
	}
	childThicknessCollection += shootGrowthParameters.m_thicknessAccumulateAgeFactor * shootGrowthParameters.m_endNodeThickness * shootGrowthParameters.m_growthRate * internodeData.m_age;


	internodeData.m_maxDistanceToAnyBranchEnd = maxDistanceToAnyBranchEnd;
	if (childThicknessCollection != 0.0f) {
		internodeInfo.m_thickness = glm::max(internodeInfo.m_thickness, glm::pow(childThicknessCollection,
			shootGrowthParameters.m_thicknessAccumulationFactor));
	}
	else
	{
		internodeInfo.m_thickness = glm::max(internodeInfo.m_thickness, shootGrowthParameters.m_endNodeThickness);
	}

	internodeData.m_biomass =
		internodeInfo.m_thickness / shootGrowthParameters.m_endNodeThickness * internodeInfo.m_length /
		shootGrowthParameters.m_internodeLength;
	for (const auto& i : internode.RefChildHandles()) {
		auto& childInternode = m_shootSkeleton.RefNode(i);
		internodeData.m_descendentTotalBiomass +=
			childInternode.m_data.m_descendentTotalBiomass +
			childInternode.m_data.m_biomass;
		childInternode.m_data.m_isMaxChild = i == maxChildHandle;
	}
	internodeData.m_sagging = shootGrowthParameters.GetSagging(internode);
}

void TreeModel::CalculateVigorRequirement(const ShootGrowthParameters& shootGrowthParameters, PlantGrowthRequirement& newTreeGrowthNutrientsRequirement) {

	const auto& sortedInternodeList = m_shootSkeleton.RefSortedNodeList();
	for (const auto& internodeHandle : sortedInternodeList) {
		auto& internode = m_shootSkeleton.RefNode(internodeHandle);
		auto& internodeData = internode.m_data;
		auto& internodeVigorFlow = internodeData.m_vigorFlow;
		internodeVigorFlow.m_maintenanceVigorRequirementWeight = 0.0f;
		internodeVigorFlow.m_developmentalVigorRequirementWeight = 0.0f;
		internodeVigorFlow.m_subtreeDevelopmentalVigorRequirementWeight = 0.0f;
		internodeVigorFlow.m_subtreeMaintenanceVigorRequirementWeight = 0.0f;
		const auto growthRate = shootGrowthParameters.m_growthRate;


		for (auto& bud : internodeData.m_buds) {
			bud.m_vigorSink.SetDesiredDevelopmentalVigorRequirement(0.0f);
			bud.m_vigorSink.SetDesiredMaintenanceVigorRequirement(0.0f);
			if (bud.m_status == BudStatus::Died || bud.m_status == BudStatus::Removed) {
				continue;
			}
			switch (bud.m_type) {
			case BudType::Apical: {
				if (bud.m_status == BudStatus::Dormant) {
					//Elongation
					bud.m_vigorSink.SetDesiredDevelopmentalVigorRequirement(growthRate * shootGrowthParameters.m_internodeElongationRate * shootGrowthParameters.m_internodeVigorRequirement);
				}
			}break;
			case BudType::Leaf: {
				if (bud.m_status == BudStatus::Dormant)
				{
					//No requirement since the lateral bud only gets activated and turned into new shoot.
					//We can make use of the development vigor for bud flushing probability here in future.
					bud.m_vigorSink.SetDesiredMaintenanceVigorRequirement(shootGrowthParameters.m_leafVigorRequirement);
				}
				else if (bud.m_status == BudStatus::Flushed)
				{
					//The maintenance vigor requirement is related to the size and the drought factor of the leaf.
					bud.m_vigorSink.SetDesiredMaintenanceVigorRequirement(shootGrowthParameters.m_leafVigorRequirement * (1.0f - bud.m_drought));
				}
			}break;
			case BudType::Fruit: {
				if (bud.m_status == BudStatus::Dormant)
				{
					//No requirement since the lateral bud only gets activated and turned into new shoot.
					//We can make use of the development vigor for bud flushing probability here in future.
					bud.m_vigorSink.SetDesiredMaintenanceVigorRequirement(shootGrowthParameters.m_fruitVigorRequirement);
				}
				else if (bud.m_status == BudStatus::Flushed)
				{
					//The maintenance vigor requirement is related to the volume and the drought factor of the fruit.
					bud.m_vigorSink.SetDesiredMaintenanceVigorRequirement(shootGrowthParameters.m_fruitVigorRequirement * (1.0f - bud.m_drought));
				}
			}break;
			default: break;
			}
			//Collect requirement for internode. The internode doesn't has it's own requirement for now since we consider it as simple pipes
			//that only perform transportation. However this can be change in future.
			internodeVigorFlow.m_maintenanceVigorRequirementWeight += bud.m_vigorSink.GetMaintenanceVigorRequirement() - bud.m_vigorSink.GetAvailableMaintenanceVigor();
			internodeVigorFlow.m_developmentalVigorRequirementWeight += bud.m_vigorSink.GetDevelopmentalVigorRequirement() - bud.m_vigorSink.GetAvailableDevelopmentalVigor();

			//Collect vigor requirement to system graph.
			newTreeGrowthNutrientsRequirement.m_maintenanceVigor += internodeVigorFlow.m_maintenanceVigorRequirementWeight;
			newTreeGrowthNutrientsRequirement.m_developmentalVigor += internodeVigorFlow.m_developmentalVigorRequirementWeight;
		}
	}
}

void TreeModel::Clear() {
	m_shootSkeleton = {};
	m_rootSkeleton = {};
	m_history = {};
	m_initialized = false;
	m_treeVolume.Clear();

	m_age = 0;

	m_shootGrowthRequirement = {};
	m_rootGrowthRequirement = {};
	m_plantVigor = {};
	m_shootFlux = {};
	m_rootFlux = {};
}

int TreeModel::GetLeafCount() const
{
	return m_leafCount;
}

int TreeModel::GetFruitCount() const
{
	return m_fruitCount;
}

int TreeModel::GetFineRootCount() const
{
	return m_fineRootCount;
}

bool TreeModel::PruneInternodes(float maxDistance, NodeHandle internodeHandle,
	const ShootGrowthParameters& shootGrowthParameters) {
	auto& internode = m_shootSkeleton.RefNode(internodeHandle);
	//Pruning here.
	bool pruning = false;

	if (maxDistance > 5 && internode.m_data.m_order != 0 &&
		internode.m_data.m_rootDistance / maxDistance < shootGrowthParameters.m_lowBranchPruning) {
		pruning = true;
	}
	if (internode.IsEndNode())
	{
		float pruningProbability = shootGrowthParameters.m_growthRate * (1.0f - internode.m_data.m_lightIntensity) * shootGrowthParameters.m_endNodePruningLightFactor;

		if (pruningProbability >= glm::linearRand(0.0f, 1.0f)) pruning = true;
	}
	if (internode.m_info.m_globalPosition.y <= 0.5f && internode.m_data.m_order != 0 && glm::linearRand(0.0f, 1.0f) < shootGrowthParameters.m_growthRate * 0.1f) pruning = true;
	if (pruning)
	{
		m_shootSkeleton.RecycleNode(internodeHandle);
	}
	return pruning;
}

void TreeModel::SampleNitrite(const glm::mat4& globalTransform, SoilModel& soilModel)
{
	m_rootFlux.m_nitrite = 0.0f;
	const auto& sortedRootNodeList = m_rootSkeleton.RefSortedNodeList();
	for (const auto& rootNodeHandle : sortedRootNodeList) {
		auto& rootNode = m_rootSkeleton.RefNode(rootNodeHandle);
		auto& rootNodeInfo = rootNode.m_info;
		auto worldSpacePosition = globalTransform * glm::translate(rootNodeInfo.m_globalPosition)[3];
		if (m_collectNitrite) {
			rootNode.m_data.m_nitrite = soilModel.IntegrateNutrient(worldSpacePosition, 0.2);
			m_rootFlux.m_nitrite += rootNode.m_data.m_nitrite;
		}else
		{
			m_rootFlux.m_nitrite += 1.0f;
		}
	}
}

void TreeModel::AllocateRootVigor(const RootGrowthParameters& rootGrowthParameters)
{
	//For how this works, refer to AllocateShootVigor().
	const auto& sortedRootNodeList = m_rootSkeleton.RefSortedNodeList();
	const float apicalControl =
		1.0f + rootGrowthParameters.m_apicalControl
		* glm::exp(-rootGrowthParameters.m_apicalControlAgeFactor * m_age * rootGrowthParameters.m_growthRate);

	float actualAllocatedDevelopmentVigor = 0.0f;

	for (const auto& rootNodeHandle : sortedRootNodeList) {
		auto& rootNode = m_rootSkeleton.RefNode(rootNodeHandle);
		auto& rootNodeData = rootNode.m_data;
		auto& rootNodeVigorFlow = rootNodeData.m_vigorFlow;
		if (rootNode.GetParentHandle() == -1) {
			if (m_rootGrowthRequirement.m_maintenanceVigor + m_rootGrowthRequirement.m_developmentalVigor != 0.0f) {
				const auto rootVigor = m_plantVigor.m_rootVigor;
				const float totalRequirement = rootNodeVigorFlow.m_developmentalVigorRequirementWeight + rootNodeVigorFlow.m_maintenanceVigorRequirementWeight
					+ rootNodeVigorFlow.m_subtreeDevelopmentalVigorRequirementWeight + rootNodeVigorFlow.m_subtreeMaintenanceVigorRequirementWeight;
				if (totalRequirement != 0.0f) {
					rootNodeVigorFlow.m_allocatedVigor = rootVigor *
						(rootNodeVigorFlow.m_developmentalVigorRequirementWeight + rootNodeVigorFlow.m_maintenanceVigorRequirementWeight)
						/ totalRequirement;
					if (rootGrowthParameters.m_maintenanceVigorRequirementPriority)
					{
						rootNodeVigorFlow.m_allocatedVigor = glm::min(rootVigor, glm::max(rootNodeVigorFlow.m_allocatedVigor, rootNodeVigorFlow.m_maintenanceVigorRequirementWeight));
					}
				}
				rootNodeData.m_vigorSink.AddVigor(rootNodeVigorFlow.m_allocatedVigor);
				rootNodeVigorFlow.m_subTreeAllocatedVigor = rootVigor - rootNodeVigorFlow.m_allocatedVigor;
			}
		}
		if (rootNodeVigorFlow.m_subTreeAllocatedVigor != 0.0f) {
			float childDevelopmentalVigorRequirementWeightSum = 0.0f;
			float childMaintenanceVigorRequirementWeightSum = 0.0f;
			for (const auto& i : rootNode.RefChildHandles()) {
				const auto& childRootNode = m_rootSkeleton.RefNode(i);
				const auto& childRootNodeData = childRootNode.m_data;
				const auto& childRootNodeVigorFlow = childRootNodeData.m_vigorFlow;
				float childDevelopmentalVigorRequirementWeight = 0.0f;
				if (rootNodeVigorFlow.m_subtreeDevelopmentalVigorRequirementWeight != 0.0f) {
					childDevelopmentalVigorRequirementWeight =
						(childRootNodeVigorFlow.m_developmentalVigorRequirementWeight + childRootNodeVigorFlow.m_subtreeDevelopmentalVigorRequirementWeight)
						/ rootNodeVigorFlow.m_subtreeDevelopmentalVigorRequirementWeight;
				}
				float childMaintenanceVigorRequirementWeight = 0.0f;
				if (rootNodeVigorFlow.m_subtreeMaintenanceVigorRequirementWeight != 0.0f) {
					childMaintenanceVigorRequirementWeight = (childRootNodeVigorFlow.m_maintenanceVigorRequirementWeight + childRootNodeVigorFlow.m_subtreeMaintenanceVigorRequirementWeight)
						/ rootNodeVigorFlow.m_subtreeMaintenanceVigorRequirementWeight;
				}
				//Perform Apical control here.
				if (rootNodeData.m_isMaxChild) childDevelopmentalVigorRequirementWeight *= apicalControl;

				childDevelopmentalVigorRequirementWeightSum += childDevelopmentalVigorRequirementWeight;
				childMaintenanceVigorRequirementWeightSum += childMaintenanceVigorRequirementWeight;
			}
			//Calculate total subtree vigor used for maintenance and development here.
			float rootNodeSubtreeAllocatedMaintenanceVigor = 0.0f;
			if (!rootGrowthParameters.m_maintenanceVigorRequirementPriority) {
				//Vigor divided by maintenance/development ratio
				const float totalRequirement = rootNodeVigorFlow.m_subtreeDevelopmentalVigorRequirementWeight + rootNodeVigorFlow.m_subtreeMaintenanceVigorRequirementWeight;
				if (totalRequirement != 0.0f) {
					rootNodeSubtreeAllocatedMaintenanceVigor =
						rootNodeVigorFlow.m_subTreeAllocatedVigor *
						rootNodeVigorFlow.m_subtreeMaintenanceVigorRequirementWeight
						/ totalRequirement;
				}
			}
			else
			{
				//Vigor firstly try to suffice maintenance requirement
				rootNodeSubtreeAllocatedMaintenanceVigor = glm::min(rootNodeVigorFlow.m_subTreeAllocatedVigor,
					rootNodeVigorFlow.m_subtreeMaintenanceVigorRequirementWeight);
			}
			const float rootNodeSubtreeAllocatedDevelopmentVigor = rootNodeVigorFlow.m_subTreeAllocatedVigor - rootNodeSubtreeAllocatedMaintenanceVigor;
			for (const auto& i : rootNode.RefChildHandles()) {
				auto& childRootNode = m_rootSkeleton.RefNode(i);
				auto& childRootNodeData = childRootNode.m_data;
				auto& childRootNodeVigorFlow = childRootNodeData.m_vigorFlow;
				float childDevelopmentalVigorRequirementWeight = 0.0f;
				if (rootNodeVigorFlow.m_subtreeDevelopmentalVigorRequirementWeight != 0.0f)
					childDevelopmentalVigorRequirementWeight = (childRootNodeVigorFlow.m_developmentalVigorRequirementWeight + childRootNodeVigorFlow.m_subtreeDevelopmentalVigorRequirementWeight)
					/ rootNodeVigorFlow.m_subtreeDevelopmentalVigorRequirementWeight;


				float childMaintenanceVigorRequirementWeight = 0.0f;
				if (rootNodeVigorFlow.m_subtreeMaintenanceVigorRequirementWeight != 0.0f)
					childMaintenanceVigorRequirementWeight = (childRootNodeVigorFlow.m_maintenanceVigorRequirementWeight + childRootNodeVigorFlow.m_subtreeMaintenanceVigorRequirementWeight)
					/ rootNodeVigorFlow.m_subtreeMaintenanceVigorRequirementWeight;

				//Perform Apical control here.
				if (rootNodeData.m_isMaxChild) childDevelopmentalVigorRequirementWeight *= apicalControl;

				//Firstly calculate total amount of maintenance vigor belongs to this child from internode.
				float childTotalAllocatedMaintenanceVigor = 0.0f;
				if (childMaintenanceVigorRequirementWeightSum != 0.0f) 
					childTotalAllocatedMaintenanceVigor = rootNodeSubtreeAllocatedMaintenanceVigor *
					childMaintenanceVigorRequirementWeight / childMaintenanceVigorRequirementWeightSum;

				//Then calculate total amount of development vigor belongs to this child from internode received vigor for its children.
				float childTotalAllocatedDevelopmentVigor = 0.0f;
				if (childDevelopmentalVigorRequirementWeightSum != 0.0f) childTotalAllocatedDevelopmentVigor = rootNodeSubtreeAllocatedDevelopmentVigor *
					childDevelopmentalVigorRequirementWeight / childDevelopmentalVigorRequirementWeightSum;

				childRootNodeVigorFlow.m_allocatedVigor = 0.0f;
				//Combine maintenance and development allocated to this child.
				if (childRootNodeVigorFlow.m_maintenanceVigorRequirementWeight + childRootNodeVigorFlow.m_subtreeMaintenanceVigorRequirementWeight != 0.0f) {
					childRootNodeVigorFlow.m_allocatedVigor += childTotalAllocatedMaintenanceVigor *
						childRootNodeVigorFlow.m_maintenanceVigorRequirementWeight
						/ (childRootNodeVigorFlow.m_maintenanceVigorRequirementWeight + childRootNodeVigorFlow.m_subtreeMaintenanceVigorRequirementWeight);
				}
				if (childRootNodeVigorFlow.m_developmentalVigorRequirementWeight + childRootNodeVigorFlow.m_subtreeDevelopmentalVigorRequirementWeight != 0.0f) {
					childRootNodeVigorFlow.m_allocatedVigor += childTotalAllocatedDevelopmentVigor *
						childRootNodeVigorFlow.m_developmentalVigorRequirementWeight
						/ (childRootNodeVigorFlow.m_developmentalVigorRequirementWeight + childRootNodeVigorFlow.m_subtreeDevelopmentalVigorRequirementWeight);
				}

				childRootNodeData.m_vigorSink.AddVigor(childRootNodeVigorFlow.m_allocatedVigor);

				childRootNodeVigorFlow.m_subTreeAllocatedVigor =
					childTotalAllocatedMaintenanceVigor + childTotalAllocatedDevelopmentVigor - childRootNodeVigorFlow.m_allocatedVigor;
			}
		}
		else
		{
			for (const auto& i : rootNode.RefChildHandles())
			{
				auto& childRootNode = m_rootSkeleton.RefNode(i);
				auto& childRootNodeData = childRootNode.m_data;
				auto& childRootNodeVigorFlow = childRootNodeData.m_vigorFlow;
				childRootNodeVigorFlow.m_allocatedVigor = childRootNodeVigorFlow.m_subTreeAllocatedVigor = 0.0f;
			}
		}

		actualAllocatedDevelopmentVigor += rootNodeData.m_vigorSink.GetAvailableDevelopmentalVigor();
	}
	m_rootNodeDevelopmentRate = actualAllocatedDevelopmentVigor / m_rootGrowthRequirement.m_developmentalVigor;
}

void TreeModel::CalculateVigorRequirement(const RootGrowthParameters& rootGrowthParameters,
	PlantGrowthRequirement& newRootGrowthNutrientsRequirement)
{
	const auto& sortedRootNodeList = m_rootSkeleton.RefSortedNodeList();
	const auto growthRate = rootGrowthParameters.m_growthRate;
	for (const auto& rootNodeHandle : sortedRootNodeList) {
		auto& rootNode = m_rootSkeleton.RefNode(rootNodeHandle);
		auto& rootNodeData = rootNode.m_data;
		auto& rootNodeVigorFlow = rootNodeData.m_vigorFlow;
		//This one has 0 always but we will put value in it in future.
		rootNodeVigorFlow.m_maintenanceVigorRequirementWeight = 0.0f;
		rootNodeVigorFlow.m_developmentalVigorRequirementWeight = 0.0f;
		float growthPotential = 0.0f;
		if (rootNode.RefChildHandles().empty())
		{
			growthPotential = growthRate * rootGrowthParameters.m_rootNodeElongationRate * rootGrowthParameters.m_rootNodeVigorRequirement;
			rootNodeVigorFlow.m_developmentalVigorRequirementWeight =
				rootNodeData.m_nitrite * growthPotential *
				glm::pow(1.0f / glm::max(rootNodeData.m_soilDensity * rootGrowthParameters.m_environmentalFriction, 1.0f), rootGrowthParameters.m_environmentalFrictionFactor);
		}
		rootNodeData.m_vigorSink.SetDesiredMaintenanceVigorRequirement(rootNodeVigorFlow.m_maintenanceVigorRequirementWeight);
		rootNodeData.m_vigorSink.SetDesiredDevelopmentalVigorRequirement(rootNodeVigorFlow.m_developmentalVigorRequirementWeight);
		//We sum the vigor requirement with the developmentalVigorRequirement,
		//so the overall nitrite will not affect the root growth. Thus we will have same growth rate for low/high nitrite density.
		newRootGrowthNutrientsRequirement.m_maintenanceVigor += rootNodeVigorFlow.m_maintenanceVigorRequirementWeight - rootNodeData.m_vigorSink.GetAvailableMaintenanceVigor();
		newRootGrowthNutrientsRequirement.m_developmentalVigor += growthPotential - rootNodeData.m_vigorSink.GetAvailableDevelopmentalVigor();
	}
}

void TreeModel::SampleTemperature(const glm::mat4& globalTransform, ClimateModel& climateModel)
{
	const auto& sortedInternodeList = m_shootSkeleton.RefSortedNodeList();
	for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); it++) {
		auto& internode = m_shootSkeleton.RefNode(*it);
		auto& internodeData = internode.m_data;
		auto& internodeInfo = internode.m_info;
		internodeData.m_temperature = climateModel.GetTemperature(globalTransform * glm::translate(internodeInfo.m_globalPosition)[3]);
	}
}

void TreeModel::SampleSoilDensity(const glm::mat4& globalTransform, SoilModel& soilModel)
{
	const auto& sortedRootNodeList = m_rootSkeleton.RefSortedNodeList();
	for (auto it = sortedRootNodeList.rbegin(); it != sortedRootNodeList.rend(); it++) {
		auto& rootNode = m_rootSkeleton.RefNode(*it);
		auto& rootNodeData = rootNode.m_data;
		auto& rootNodeInfo = rootNode.m_info;
		rootNodeData.m_soilDensity = soilModel.GetDensity(globalTransform * glm::translate(rootNodeInfo.m_globalPosition)[3]);
	}
}

ShootSkeleton& TreeModel::RefShootSkeleton() {
	return m_shootSkeleton;
}

const ShootSkeleton&
TreeModel::PeekShootSkeleton(const int iteration) const {
	assert(iteration >= 0 && iteration <= m_history.size());
	if (iteration == m_history.size()) return m_shootSkeleton;
	return m_history.at(iteration).first;
}

RootSkeleton& TreeModel::RefRootSkeleton() {
	return m_rootSkeleton;
}

const RootSkeleton&
TreeModel::PeekRootSkeleton(const int iteration) const {
	assert(iteration >= 0 && iteration <= m_history.size());
	if (iteration == m_history.size()) return m_rootSkeleton;
	return m_history.at(iteration).second;
}

void TreeModel::ClearHistory() {
	m_history.clear();
}

void TreeModel::Step() {
	m_history.emplace_back(m_shootSkeleton, m_rootSkeleton);
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
	m_shootSkeleton = m_history[iteration].first;
	m_rootSkeleton = m_history[iteration].second;
	m_history.erase((m_history.begin() + iteration), m_history.end());
}


#pragma region TreeGrowthParameters
ShootGrowthParameters::ShootGrowthParameters() {
	m_growthRate = 0.03f;

	//Structure
	m_lateralBudCount = 2;
	m_fruitBudCount = 0;
	m_leafBudCount = 1;
	m_internodeLength = 0.03f;
	m_endNodeThickness = 0.002f;
	m_thicknessAccumulationFactor = 0.5f;

	//Bud
	m_branchingAngleMeanVariance = glm::vec2(45, 3);
	m_rollAngleMeanVariance = glm::vec2(120, 2);
	m_apicalAngleMeanVariance = glm::vec2(0, 3);
	m_gravitropism = 0.03f;
	m_phototropism = 0.0f;

	m_lateralBudFlushingProbabilityTemperatureRange = glm::vec4(0.05f, 0.05f, 0.0f, 100.0f);
	m_leafBudFlushingProbabilityTemperatureRange = glm::vec4(0.0f, 1.0f, 45.0f, 60.0f);
	m_fruitBudFlushingProbabilityTemperatureRange = glm::vec4(0.0f, 1.0f, 50.0f, 70.0f);

	m_lateralBudLightingFactor = 1.0f;
	m_leafBudLightingFactor = 1.0f;
	m_fruitBudLightingFactor = 1.0f;

	//m_apicalControlBaseDistFactor = { 1.05f, 0.95f };
	m_apicalDominance = 1.5;
	m_apicalDominanceDistanceFactor = 0.95f;
	//m_budKillProbability = 0.00f;

	m_leafVigorRequirement = 1.0f;
	m_fruitVigorRequirement = 1.0f;

	//Internode
	m_lowBranchPruning = 0.15f;
	m_saggingFactorThicknessReductionMax = glm::vec3(0.0003, 2, 0.5);

	//Foliage
	m_maxLeafSize = glm::vec3(0.05f, 1.0f, 0.05f) / 2.0f;
	m_leafPositionVariance = 0.5f;
	m_leafRandomRotation = 10.0f;
	m_leafChlorophyllLoss = 4.0f;
	m_leafChlorophyllSynthesisFactorTemperature = 65.f;
	m_leafFallProbability = 3.0f;
	m_leafDistanceToBranchEndLimit = 10.f;


	//Fruit
	m_maxFruitSize = glm::vec3(0.07f, 0.07f, 0.07f) / 2.0f;
	m_fruitPositionVariance = 0.5f;
	m_fruitRandomRotation = 10.0f;
}


float ShootGrowthParameters::GetDesiredBranchingAngle(const Node<InternodeGrowthData>& internode) const {
	return glm::gaussRand(
		m_branchingAngleMeanVariance.x,
		m_branchingAngleMeanVariance.y);
}

float ShootGrowthParameters::GetDesiredRollAngle(const Node<InternodeGrowthData>& internode) const {
	return glm::gaussRand(
		m_rollAngleMeanVariance.x,
		m_rollAngleMeanVariance.y);
}

float ShootGrowthParameters::GetDesiredApicalAngle(const Node<InternodeGrowthData>& internode) const {
	return glm::gaussRand(
		m_apicalAngleMeanVariance.x,
		m_apicalAngleMeanVariance.y);
}

float ShootGrowthParameters::GetSagging(const Node<InternodeGrowthData>& internode) const {
	const auto newSagging = glm::min(
		m_saggingFactorThicknessReductionMax.z,
		m_saggingFactorThicknessReductionMax.x *
		(internode.m_data.m_descendentTotalBiomass + internode.m_data.m_extraMass) /
		glm::pow(
			internode.m_info.m_thickness /
			m_endNodeThickness,
			m_saggingFactorThicknessReductionMax.y));
	return glm::max(internode.m_data.m_sagging, newSagging);
}

#pragma endregion
#pragma region RootGrowthParameters
RootGrowthParameters::RootGrowthParameters()
{
	m_growthRate = 0.04f;
	m_rootNodeLength = 0.03f;
	m_endNodeThickness = 0.002f;
	m_thicknessAccumulationFactor = 0.5f;
	m_thicknessAccumulateAgeFactor = 0.0002f;
	m_branchingAngleMeanVariance = glm::vec2(60, 3);
	m_rollAngleMeanVariance = glm::vec2(120, 2);
	m_apicalAngleMeanVariance = glm::vec2(0, 3);
	m_apicalDominanceDistanceFactor = 1.0f;
	m_tropismSwitchingProbability = 1.0f;
	m_tropismSwitchingProbabilityDistanceFactor = 0.99f;
	m_tropismIntensity = 0.1f;
}

float RootGrowthParameters::GetRootApicalAngle(const Node<RootNodeGrowthData>& rootNode) const
{
	return glm::gaussRand(
		m_apicalAngleMeanVariance.x,
		m_apicalAngleMeanVariance.y);
}

float RootGrowthParameters::GetRootRollAngle(const Node<RootNodeGrowthData>& rootNode) const
{
	return glm::gaussRand(
		m_rollAngleMeanVariance.x,
		m_rollAngleMeanVariance.y);
}

float RootGrowthParameters::GetRootBranchingAngle(const Node<RootNodeGrowthData>& rootNode) const
{
	return glm::gaussRand(
		m_branchingAngleMeanVariance.x,
		m_branchingAngleMeanVariance.y);
}

void RootGrowthParameters::SetTropisms(Node<RootNodeGrowthData>& oldNode, Node<RootNodeGrowthData>& newNode) const
{
	float probability = m_tropismSwitchingProbability *
		glm::exp(-m_tropismSwitchingProbabilityDistanceFactor * oldNode.m_data.m_rootDistance);

	const bool needSwitch = probability >= glm::linearRand(0.0f, 1.0f);
	newNode.m_data.m_horizontalTropism = needSwitch ? oldNode.m_data.m_verticalTropism : oldNode.m_data.m_horizontalTropism;
	newNode.m_data.m_verticalTropism = needSwitch ? oldNode.m_data.m_horizontalTropism : oldNode.m_data.m_verticalTropism;
}
#pragma endregion
