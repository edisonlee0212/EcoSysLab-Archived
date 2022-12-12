//
// Created by lllll on 10/21/2022.
//

#include "TreeModel.hpp"

using namespace EcoSysLab;

void ApplyTropism(const glm::vec3& targetDir, float tropism, glm::vec3& front, glm::vec3& up) {
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

void ApplyTropism(const glm::vec3& targetDir, float tropism, glm::quat& rotation) {
	auto front = rotation * glm::vec3(0, 0, -1);
	auto up = rotation * glm::vec3(0, 1, 0);
	ApplyTropism(targetDir, tropism, front, up);
	rotation = glm::quatLookAt(front, up);
}

bool TreeModel::ElongateRoot(const float extendLength, NodeHandle rootNodeHandle, const RootGrowthParameters& rootGrowthParameters,
	float& collectedAuxin) {
	bool graphChanged = false;
	auto& rootNode = m_rootSkeleton.RefNode(rootNodeHandle);
	const auto rootNodeLength = rootGrowthParameters.GetRootNodeLength(rootNode);
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
		auto newRootNodeHandle = m_rootSkeleton.Extend(rootNodeHandle, false);
		auto& oldRootNode = m_rootSkeleton.RefNode(rootNodeHandle);
		auto& newRootNode = m_rootSkeleton.RefNode(newRootNodeHandle);
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


		//TODO: Nitrite packets here.
		newRootNode.m_data.m_rootUnitDistance = oldRootNode.m_data.m_rootUnitDistance + 1;
		newRootNode.m_data.m_auxinTarget = newRootNode.m_data.m_auxin = 0.0f;
		newRootNode.m_info.m_length = glm::clamp(extendLength, 0.0f, rootNodeLength);
		newRootNode.m_info.m_thickness = rootGrowthParameters.GetEndNodeThickness(newRootNode);
		newRootNode.m_info.m_globalRotation = glm::quatLookAt(desiredGlobalFront, desiredGlobalUp);
		newRootNode.m_info.m_localRotation =
			glm::inverse(oldRootNode.m_info.m_globalRotation) *
			newRootNode.m_info.m_globalRotation;
		if (extraLength > rootNodeLength) {
			float childAuxin = 0.0f;
			ElongateRoot(extraLength - rootNodeLength, newRootNodeHandle, rootGrowthParameters, childAuxin);
			childAuxin *= rootGrowthParameters.GetAuxinTransportLoss(newRootNode);
			collectedAuxin += childAuxin;
			m_rootSkeleton.RefNode(newRootNodeHandle).m_data.m_auxinTarget = childAuxin;
		}
		else {
			collectedAuxin += rootGrowthParameters.GetAuxinTransportLoss(newRootNode);
		}
	}
	else {
		//Otherwise, we add the inhibitor.
		collectedAuxin += rootGrowthParameters.GetAuxinTransportLoss(rootNode);
	}
	return graphChanged;
}

inline bool TreeModel::GrowRootNode(SoilModel& soilModel, NodeHandle rootNodeHandle, const RootGrowthParameters& rootGrowthParameters)
{
	bool graphChanged = false;
	{
		auto& rootNode = m_rootSkeleton.RefNode(rootNodeHandle);
		auto& rootNodeData = rootNode.m_data;
		rootNodeData.m_auxinTarget = 0;
		for (const auto& childHandle : rootNode.RefChildHandles()) {
			rootNodeData.m_auxinTarget += m_rootSkeleton.RefNode(childHandle).m_data.m_auxin *
				rootGrowthParameters.GetAuxinTransportLoss(rootNode);
		}
	}

	{
		auto& rootNode = m_rootSkeleton.RefNode(rootNodeHandle);
		if (rootNode.RefChildHandles().empty())
		{
			//1. Elongate current node.
			float collectedAuxin = 0.0f;

			float extendLength = 0.0f;
			if (m_rootGrowthNutrients.m_totalNitrate != 0.0f) { extendLength = rootNode.m_data.m_nitrateLevels / m_rootGrowthNutrients.m_totalNitrate * m_rootGrowthNutrients.m_carbon; }
			const auto dd = rootGrowthParameters.GetAuxinTransportLoss(rootNode);
			ElongateRoot(extendLength, rootNodeHandle, rootGrowthParameters, collectedAuxin);
			m_rootSkeleton.RefNode(rootNodeHandle).m_data.m_auxinTarget += collectedAuxin * dd;
		}
		else
		{
			//2. Form new shoot if necessary
			float branchingProb = rootGrowthParameters.GetBranchingProbability(rootNode);
			bool formNewShoot = glm::linearRand(0.0f, 1.0f) < branchingProb;
			if (formNewShoot) {
				const auto newRootNodeHandle = m_rootSkeleton.Extend(rootNodeHandle, true);
				auto& oldRootNode = m_rootSkeleton.RefNode(rootNodeHandle);
				auto& newRootNode = m_rootSkeleton.RefNode(newRootNodeHandle);
				rootGrowthParameters.SetTropisms(oldRootNode, newRootNode);
				newRootNode.m_info.m_length = 0.0f;
				newRootNode.m_info.m_thickness = rootGrowthParameters.GetEndNodeThickness(newRootNode);
				newRootNode.m_info.m_localRotation = glm::quat(glm::vec3(0, glm::radians(glm::linearRand(0.0f, 360.0f)), rootGrowthParameters.GetRootBranchingAngle(newRootNode)));
				newRootNode.m_data.m_rootUnitDistance = oldRootNode.m_data.m_rootUnitDistance + 1;
			}
		}
	}


	{
		auto& rootNode = m_rootSkeleton.RefNode(rootNodeHandle);
		auto& rootNodeData = rootNode.m_data;
		rootNodeData.m_auxin = (rootNodeData.m_auxin + rootNodeData.m_auxinTarget) / 2.0f;
	}
	return graphChanged;
}

void TreeModel::CalculateResourceRequirement(NodeHandle rootNodeHandle, const RootGrowthParameters& rootGrowthParameters)
{
}

void TreeModel::CalculateThickness(NodeHandle rootNodeHandle, const RootGrowthParameters& rootGrowthParameters)
{
	auto& rootNode = m_rootSkeleton.RefNode(rootNodeHandle);
	auto& rootNodeData = rootNode.m_data;
	auto& rootNodeInfo = rootNode.m_info;
	rootNodeData.m_descendentTotalBiomass = 0;
	float maxDistanceToAnyBranchEnd = 0;
	float childThicknessCollection = 0.0f;
	NodeHandle maxChildHandle = -1;
	float maxSubTreeBiomass = -1.0f;
	for (const auto& i : rootNode.RefChildHandles()) {
		auto& childInternode = m_rootSkeleton.RefNode(i);
		const float childMaxDistanceToAnyBranchEnd =
			childInternode.m_data.m_maxDistanceToAnyBranchEnd +
			childInternode.m_info.m_length / rootGrowthParameters.GetRootNodeLength(childInternode);
		maxDistanceToAnyBranchEnd = glm::max(maxDistanceToAnyBranchEnd, childMaxDistanceToAnyBranchEnd);

		childThicknessCollection += glm::pow(childInternode.m_info.m_thickness,
			1.0f / rootGrowthParameters.GetThicknessControlFactor(rootNode));
	}
	rootNodeData.m_maxDistanceToAnyBranchEnd = maxDistanceToAnyBranchEnd;
	if (childThicknessCollection != 0.0f) {
		rootNodeInfo.m_thickness = glm::max(rootNodeInfo.m_thickness, glm::pow(childThicknessCollection + rootGrowthParameters.GetThicknessAccumulateFactor(rootNode),
			rootGrowthParameters.GetThicknessControlFactor(
				rootNode)));
	}
	else
	{
		rootNodeInfo.m_thickness = glm::max(rootNodeInfo.m_thickness, rootGrowthParameters.GetEndNodeThickness(rootNode));
	}

	rootNodeData.m_biomass =
		rootNodeInfo.m_thickness / rootGrowthParameters.GetEndNodeThickness(rootNode) * rootNodeInfo.m_length /
		rootGrowthParameters.GetRootNodeLength(rootNode);
	for (const auto& i : rootNode.RefChildHandles()) {
		const auto& childInternode = m_rootSkeleton.RefNode(i);
		rootNodeData.m_descendentTotalBiomass +=
			childInternode.m_data.m_descendentTotalBiomass +
			childInternode.m_data.m_biomass;
	}
}


bool TreeModel::ElongateInternode(float extendLength, NodeHandle internodeHandle,
                                  const TreeGrowthParameters& parameters, float& collectedInhibitor) {
	bool graphChanged = false;
	auto& internode = m_branchSkeleton.RefNode(internodeHandle);
	auto internodeLength = parameters.GetInternodeLength(internode);
	auto& internodeData = internode.m_data;
	auto& internodeInfo = internode.m_info;
	internodeInfo.m_length += extendLength;
	float extraLength = internodeInfo.m_length - internodeLength;
	auto& apicalBud = internodeData.m_buds.front();
	//If we need to add a new end node
	if (extraLength > 0) {
		graphChanged = true;
		apicalBud.m_status = BudStatus::Died;
		internodeInfo.m_length = internodeLength;
		auto desiredGlobalRotation = internodeInfo.m_globalRotation * apicalBud.m_localRotation;
		auto desiredGlobalFront = desiredGlobalRotation * glm::vec3(0, 0, -1);
		auto desiredGlobalUp = desiredGlobalRotation * glm::vec3(0, 1, 0);
		ApplyTropism(-m_currentGravityDirection, parameters.GetGravitropism(internode), desiredGlobalFront,
			desiredGlobalUp);
		ApplyTropism(internodeData.m_lightDirection, parameters.GetPhototropism(internode),
			desiredGlobalFront, desiredGlobalUp);
		auto lateralBudCount = parameters.GetLateralBudCount(internode);
		//Allocate Lateral bud for current internode
		float turnAngle = glm::radians(360.0f / lateralBudCount);
		for (int i = 0; i < lateralBudCount; i++) {
			internodeData.m_buds.emplace_back();
			auto& lateralBud = internodeData.m_buds.back();
			lateralBud.m_type = BudType::Lateral;
			lateralBud.m_status = BudStatus::Dormant;
			lateralBud.m_localRotation = glm::vec3(
				glm::radians(parameters.GetDesiredBranchingAngle(internode)), 0.0f,
				i * turnAngle);
		}

		//Create new internode
		auto newInternodeHandle = m_branchSkeleton.Extend(internodeHandle, false);
		auto& oldInternode = m_branchSkeleton.RefNode(internodeHandle);
		auto& newInternode = m_branchSkeleton.RefNode(newInternodeHandle);
		newInternode.m_data.Clear();
		newInternode.m_data.m_inhibitorTarget = newInternode.m_data.m_inhibitor = parameters.GetApicalDominanceBase(
			newInternode);
		newInternode.m_info.m_length = glm::clamp(extendLength, 0.0f, internodeLength);
		newInternode.m_info.m_thickness = parameters.GetEndNodeThickness(newInternode);
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
			glm::radians(parameters.GetDesiredApicalAngle(newInternode)), 0.0f,
			parameters.GetDesiredRollAngle(newInternode));
		if (extraLength > internodeLength) {
			float childInhibitor = 0.0f;
			ElongateInternode(extraLength - internodeLength, newInternodeHandle, parameters, childInhibitor);
			childInhibitor *= parameters.GetApicalDominanceDecrease(newInternode);
			collectedInhibitor += childInhibitor;
			m_branchSkeleton.RefNode(newInternodeHandle).m_data.m_inhibitorTarget = childInhibitor;
		}
		else {
			collectedInhibitor += parameters.GetApicalDominanceBase(newInternode);
		}
	}
	else {
		//Otherwise, we add the inhibitor.
		collectedInhibitor += parameters.GetApicalDominanceBase(internode);
	}
	return graphChanged;
}

bool TreeModel::GrowInternode(ClimateModel& climateModel, NodeHandle internodeHandle, const TreeGrowthParameters& treeGrowthParameters) {
	bool graphChanged = false;
	{
		auto& internode = m_branchSkeleton.RefNode(internodeHandle);
		auto& internodeData = internode.m_data;
		internodeData.m_inhibitorTarget = 0;
		for (const auto& childHandle : internode.RefChildHandles()) {
			internodeData.m_inhibitorTarget += m_branchSkeleton.RefNode(childHandle).m_data.m_inhibitor *
				treeGrowthParameters.GetApicalDominanceDecrease(internode);
		}
	}
	auto& buds = m_branchSkeleton.RefNode(internodeHandle).m_data.m_buds;
	for (auto& bud : buds) {
		auto& internode = m_branchSkeleton.RefNode(internodeHandle);
		auto& internodeData = internode.m_data;
		auto& internodeInfo = internode.m_info;
		if (bud.m_type == BudType::Apical && bud.m_status == BudStatus::Dormant) {
			if (treeGrowthParameters.GetApicalBudKillProbability(internode) > glm::linearRand(0.0f, 1.0f)) {
				bud.m_status = BudStatus::Died;
			}
			else {
				const float waterReceived = bud.m_adjustedProductiveResourceRequirement;
				const auto growthRate = treeGrowthParameters.GetGrowthRate(internode);
				const float elongateLength = waterReceived * treeGrowthParameters.GetInternodeLength(internode) * growthRate;
				float collectedInhibitor = 0.0f;
				const auto dd = treeGrowthParameters.GetApicalDominanceDecrease(internode);
				graphChanged = ElongateInternode(elongateLength, internodeHandle, treeGrowthParameters, collectedInhibitor);
				m_branchSkeleton.RefNode(internodeHandle).m_data.m_inhibitorTarget += collectedInhibitor * dd;
			}
			//If apical bud is dormant, then there's no lateral bud at this stage. We should quit anyway.
			break;
		}
		else if (bud.m_type == BudType::Lateral) {
			if (bud.m_status == BudStatus::Dormant) {
				if (treeGrowthParameters.GetLateralBudKillProbability(internode) > glm::linearRand(0.0f, 1.0f)) {
					bud.m_status = BudStatus::Died;
				}
				else {
					float flushProbability = treeGrowthParameters.GetLateralBudFlushingProbability(internode) *
						treeGrowthParameters.GetGrowthRate(internode);
					flushProbability /= (1.0f + internodeData.m_inhibitor);
					if (flushProbability >= glm::linearRand(0.0f, 1.0f)) {
						graphChanged = true;
						bud.m_status = BudStatus::Flushed;
						//Prepare information for new internode
						auto desiredGlobalRotation = internodeInfo.m_globalRotation * bud.m_localRotation;
						auto desiredGlobalFront = desiredGlobalRotation * glm::vec3(0, 0, -1);
						auto desiredGlobalUp = desiredGlobalRotation * glm::vec3(0, 1, 0);
						ApplyTropism(-m_currentGravityDirection, treeGrowthParameters.GetGravitropism(internode), desiredGlobalFront,
							desiredGlobalUp);
						ApplyTropism(internodeData.m_lightDirection, treeGrowthParameters.GetPhototropism(internode),
							desiredGlobalFront, desiredGlobalUp);
						//Create new internode
						const auto newInternodeHandle = m_branchSkeleton.Extend(internodeHandle, true);
						auto& oldInternode = m_branchSkeleton.RefNode(internodeHandle);
						auto& newInternode = m_branchSkeleton.RefNode(newInternodeHandle);
						newInternode.m_data.Clear();
						newInternode.m_info.m_length = 0.0f;
						newInternode.m_info.m_thickness = treeGrowthParameters.GetEndNodeThickness(newInternode);
						newInternode.m_info.m_localRotation = newInternode.m_data.m_desiredLocalRotation =
							glm::inverse(oldInternode.m_info.m_globalRotation) *
							glm::quatLookAt(desiredGlobalFront, desiredGlobalUp);
						//Allocate apical bud
						newInternode.m_data.m_buds.emplace_back();
						auto& apicalBud = newInternode.m_data.m_buds.back();
						apicalBud.m_type = BudType::Apical;
						apicalBud.m_status = BudStatus::Dormant;
						apicalBud.m_localRotation = glm::vec3(
							glm::radians(treeGrowthParameters.GetDesiredApicalAngle(newInternode)), 0.0f,
							treeGrowthParameters.GetDesiredRollAngle(newInternode));
					}
				}
			}
		}
	}
	{
		auto& internode = m_branchSkeleton.RefNode(internodeHandle);
		auto& internodeData = internode.m_data;
		internodeData.m_inhibitor = (internodeData.m_inhibitor + internodeData.m_inhibitorTarget) / 2.0f;
	}
	return graphChanged;
}

inline void TreeModel::AdjustResourceRequirement(NodeHandle internodeHandle,
	const TreeGrowthParameters& treeGrowthParameters)
{
	auto& internode = m_branchSkeleton.RefNode(internodeHandle);
	auto& internodeData = internode.m_data;
	auto& internodeInfo = internode.m_info;
	if (internode.GetParentHandle() == -1) {
		internodeData.m_adjustedTotalProductiveWaterRequirement =
			internodeData.m_productiveResourceRequirement +
			internodeData.m_descendentProductiveResourceRequirement;
		internodeData.m_adjustedDescendentProductiveResourceRequirement = internodeData.m_descendentProductiveResourceRequirement;
		internodeData.m_adjustedProductiveResourceRequirement = internodeData.m_productiveResourceRequirement;
		for (auto& bud : internodeData.m_buds) {
			bud.m_adjustedProductiveResourceRequirement = bud.m_productiveResourceRequirement;
		}
	}
	const float apicalControl = treeGrowthParameters.GetApicalControl(internode);
	float totalChildResourceRequirement = 0.0f;

	for (const auto& i : internode.RefChildHandles()) {
		auto& childInternode = m_branchSkeleton.RefNode(i);
		auto& childInternodeData = childInternode.m_data;
		childInternodeData.m_adjustedTotalProductiveWaterRequirement = 0;
		if (internodeData.m_adjustedDescendentProductiveResourceRequirement != 0) {
			childInternodeData.m_adjustedTotalProductiveWaterRequirement =
				(childInternodeData.m_descendentProductiveResourceRequirement +
					childInternodeData.m_productiveResourceRequirement) /
				internodeData.m_adjustedDescendentProductiveResourceRequirement;
			childInternodeData.m_adjustedTotalProductiveWaterRequirement = glm::pow(
				childInternodeData.m_adjustedTotalProductiveWaterRequirement, apicalControl);
			totalChildResourceRequirement += childInternodeData.m_adjustedTotalProductiveWaterRequirement;
		}
	}
	for (const auto& i : internode.RefChildHandles()) {
		auto& childInternode = m_branchSkeleton.RefNode(i);
		auto& childInternodeData = childInternode.m_data;
		if (internodeData.m_adjustedDescendentProductiveResourceRequirement != 0) {
			childInternodeData.m_adjustedTotalProductiveWaterRequirement *=
				internodeData.m_adjustedDescendentProductiveResourceRequirement /
				totalChildResourceRequirement;
			const float resourceFactor = childInternodeData.m_adjustedTotalProductiveWaterRequirement /
				(childInternodeData.m_descendentProductiveResourceRequirement +
					childInternodeData.m_productiveResourceRequirement);
			childInternodeData.m_adjustedProductiveResourceRequirement =
				childInternodeData.m_productiveResourceRequirement * resourceFactor;
			childInternodeData.m_adjustedDescendentProductiveResourceRequirement =
				childInternodeData.m_descendentProductiveResourceRequirement * resourceFactor;
			for (auto& bud : childInternodeData.m_buds) {
				bud.m_adjustedProductiveResourceRequirement =
					bud.m_productiveResourceRequirement * resourceFactor;
			}
		}
	}
}

void TreeModel::CalculateThicknessAndSagging(NodeHandle internodeHandle,
	const TreeGrowthParameters& treeGrowthParameters) {
	auto& internode = m_branchSkeleton.RefNode(internodeHandle);
	auto& internodeData = internode.m_data;
	auto& internodeInfo = internode.m_info;
	internodeData.m_descendentTotalBiomass = 0;
	float maxDistanceToAnyBranchEnd = 0;
	float childThicknessCollection = 0.0f;
	NodeHandle maxChildHandle = -1;
	float maxSubTreeBiomass = -1.0f;
	for (const auto& i : internode.RefChildHandles()) {
		auto& childInternode = m_branchSkeleton.RefNode(i);
		const float childMaxDistanceToAnyBranchEnd =
			childInternode.m_data.m_maxDistanceToAnyBranchEnd +
			childInternode.m_info.m_length / treeGrowthParameters.GetInternodeLength(childInternode);
		maxDistanceToAnyBranchEnd = glm::max(maxDistanceToAnyBranchEnd, childMaxDistanceToAnyBranchEnd);

		childThicknessCollection += glm::pow(childInternode.m_info.m_thickness,
			1.0f / treeGrowthParameters.GetThicknessControlFactor(internode));
	}
	internodeData.m_maxDistanceToAnyBranchEnd = maxDistanceToAnyBranchEnd;
	if (childThicknessCollection != 0.0f) {
		internodeInfo.m_thickness = glm::max(internodeInfo.m_thickness, glm::pow(childThicknessCollection,
			treeGrowthParameters.GetThicknessControlFactor(
				internode)));
	}
	else
	{
		internodeInfo.m_thickness = glm::max(internodeInfo.m_thickness, treeGrowthParameters.GetEndNodeThickness(internode));
	}

	internodeData.m_biomass =
		internodeInfo.m_thickness / treeGrowthParameters.GetEndNodeThickness(internode) * internodeInfo.m_length /
		treeGrowthParameters.GetInternodeLength(internode);
	for (const auto& i : internode.RefChildHandles()) {
		const auto& childInternode = m_branchSkeleton.RefNode(i);
		internodeData.m_descendentTotalBiomass +=
			childInternode.m_data.m_descendentTotalBiomass +
			childInternode.m_data.m_biomass;
	}
	internodeData.m_sagging = treeGrowthParameters.GetSagging(internode);
}

void TreeModel::CalculateResourceRequirement(NodeHandle internodeHandle,
	const TreeGrowthParameters& treeGrowthParameters) {
	auto& internode = m_branchSkeleton.RefNode(internodeHandle);
	auto& internodeData = internode.m_data;
	auto& internodeInfo = internode.m_info;
	internodeData.m_productiveResourceRequirement = 0.0f;
	internodeData.m_descendentProductiveResourceRequirement = 0.0f;
	for (auto& bud : internodeData.m_buds) {
		if (bud.m_status == BudStatus::Died) {
			bud.m_baseResourceRequirement = 0.0f;
			bud.m_productiveResourceRequirement = 0.0f;
			continue;
		}
		switch (bud.m_type) {
		case BudType::Apical: {
			if (bud.m_status == BudStatus::Dormant) {
				bud.m_baseResourceRequirement = treeGrowthParameters.GetShootBaseResourceRequirementFactor(
					internode);
				bud.m_productiveResourceRequirement = treeGrowthParameters.GetShootProductiveResourceRequirementFactor(
					internode);
			}
		}
							break;
		case BudType::Leaf: {
			bud.m_baseResourceRequirement = treeGrowthParameters.GetLeafBaseResourceRequirementFactor(internode);
			bud.m_productiveResourceRequirement = treeGrowthParameters.GetLeafProductiveResourceRequirementFactor(
				internode);
		}
						  break;
		case BudType::Fruit: {
			bud.m_baseResourceRequirement = treeGrowthParameters.GetFruitBaseResourceRequirementFactor(internode);
			bud.m_productiveResourceRequirement = treeGrowthParameters.GetFruitProductiveResourceRequirementFactor(
				internode);
		}
						   break;
		case BudType::Lateral: {
			bud.m_baseResourceRequirement = 0.0f;
			bud.m_productiveResourceRequirement = 0.0f;
		}
							 break;
		}

		internodeData.m_productiveResourceRequirement += bud.m_productiveResourceRequirement;
	}

	if (!internode.IsEndNode()) {
		//If current node is not end node
		for (const auto& i : internode.RefChildHandles()) {
			auto& childInternode = m_branchSkeleton.RefNode(i);
			internodeData.m_descendentProductiveResourceRequirement += childInternode.m_data.
				m_productiveResourceRequirement + childInternode.m_data.m_descendentProductiveResourceRequirement;
		}
	}
}

bool TreeModel::GrowBranches(ClimateModel& climateModel, const TreeGrowthParameters& treeGrowthParameters) {
	bool treeStructureChanged = false;

#pragma region Tree Growth

#pragma region Pruning
	bool anyBranchPruned = false;
	m_branchSkeleton.SortLists();
	{
		const auto maxDistance = m_branchSkeleton.RefNode(0).m_data.m_maxDistanceToAnyBranchEnd;
		const auto& sortedInternodeList = m_branchSkeleton.RefSortedNodeList();
		for (const auto& internodeHandle : sortedInternodeList) {
			if (m_branchSkeleton.RefNode(internodeHandle).IsRecycled()) continue;
			if (LowBranchPruning(maxDistance, internodeHandle, treeGrowthParameters)) {
				anyBranchPruned = true;
			}
		}
	};
#pragma endregion
#pragma region Grow
	if (anyBranchPruned) m_branchSkeleton.SortLists();
	treeStructureChanged = treeStructureChanged || anyBranchPruned;
	bool anyBranchGrown = false;
	{
		const auto& sortedInternodeList = m_branchSkeleton.RefSortedNodeList();
		for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); it++) {
			auto internodeHandle = *it;
			CalculateResourceRequirement(internodeHandle, treeGrowthParameters);
		}
		for (const auto& internodeHandle : sortedInternodeList) {
			AdjustResourceRequirement(internodeHandle, treeGrowthParameters);
		}
		for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); it++) {
			const bool graphChanged = GrowInternode(climateModel, *it, treeGrowthParameters);
			anyBranchGrown = anyBranchGrown || graphChanged;
		}
	};
#pragma endregion
#pragma region Postprocess
	if (anyBranchGrown) m_branchSkeleton.SortLists();
	treeStructureChanged = treeStructureChanged || anyBranchGrown;
	{
		m_branchSkeleton.m_min = glm::vec3(FLT_MAX);
		m_branchSkeleton.m_max = glm::vec3(FLT_MIN);

		const auto& sortedInternodeList = m_branchSkeleton.RefSortedNodeList();
		for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); it++) {
			auto internodeHandle = *it;
			CalculateThicknessAndSagging(internodeHandle, treeGrowthParameters);
		}
		for (const auto& internodeHandle : sortedInternodeList) {
			auto& internode = m_branchSkeleton.RefNode(internodeHandle);
			auto& internodeData = internode.m_data;
			auto& internodeInfo = internode.m_info;
			if (internode.GetParentHandle() == -1) {
				internodeInfo.m_globalPosition = glm::vec3(0.0f);
				internodeInfo.m_localRotation = glm::vec3(0.0f);
				internodeInfo.m_globalRotation = glm::vec3(glm::radians(90.0f), 0.0f, 0.0f);

				internodeData.m_rootDistance =
					internodeInfo.m_length / treeGrowthParameters.GetInternodeLength(internode);
			}
			else {
				auto& parentInternode = m_branchSkeleton.RefNode(internode.GetParentHandle());
				internodeData.m_rootDistance = parentInternode.m_data.m_rootDistance + internodeInfo.m_length /
					treeGrowthParameters.GetInternodeLength(
						internode);
				internodeInfo.m_globalRotation =
					parentInternode.m_info.m_globalRotation * internodeInfo.m_localRotation;
#pragma region Apply Sagging
				auto parentGlobalRotation = m_branchSkeleton.RefNode(
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
			m_branchSkeleton.m_min = glm::min(m_branchSkeleton.m_min, internodeInfo.m_globalPosition);
			m_branchSkeleton.m_max = glm::max(m_branchSkeleton.m_max, internodeInfo.m_globalPosition);
			const auto endPosition = internodeInfo.m_globalPosition + internodeInfo.m_length *
				(internodeInfo.m_globalRotation *
					glm::vec3(0, 0, -1));
			m_branchSkeleton.m_min = glm::min(m_branchSkeleton.m_min, endPosition);
			m_branchSkeleton.m_max = glm::max(m_branchSkeleton.m_max, endPosition);
		}
	};
	{
		const auto& sortedFlowList = m_branchSkeleton.RefSortedFlowList();
		for (const auto& flowHandle : sortedFlowList) {
			auto& flow = m_branchSkeleton.RefFlow(flowHandle);
			auto& flowData = flow.m_data;
			if (flow.GetParentHandle() == -1) {
				flowData.m_order = 0;
			}
			else {
				auto& parentFlow = m_branchSkeleton.RefFlow(flow.GetParentHandle());
				if (flow.IsApical()) flowData.m_order = parentFlow.m_data.m_order;
				else flowData.m_order = parentFlow.m_data.m_order + 1;
			}
			for (const auto& internodeHandle : flow.RefNodeHandles()) {
				m_branchSkeleton.RefNode(internodeHandle).m_data.m_order = flowData.m_order;
			}
		}
		m_branchSkeleton.CalculateFlows();
	};
#pragma endregion
#pragma endregion
	return treeStructureChanged;
}

void
TreeModel::Initialize(const TreeGrowthParameters& treeGrowthParameters, const RootGrowthParameters& rootGrowthParameters) {
	{
		auto& firstInternode = m_branchSkeleton.RefNode(0);
		firstInternode.m_info.m_thickness = treeGrowthParameters.GetEndNodeThickness(firstInternode);
		firstInternode.m_data.m_buds.emplace_back();
		auto& apicalBud = firstInternode.m_data.m_buds.back();
		apicalBud.m_type = BudType::Apical;
		apicalBud.m_status = BudStatus::Dormant;
		apicalBud.m_localRotation = glm::vec3(glm::radians(treeGrowthParameters.GetDesiredApicalAngle(firstInternode)),
			0.0f,
			treeGrowthParameters.GetDesiredRollAngle(firstInternode));
	}
	{
		auto& firstRootNode = m_rootSkeleton.RefNode(0);
		firstRootNode.m_info.m_thickness = 0.003f;
		firstRootNode.m_info.m_length = 0.0f;
		firstRootNode.m_info.m_localRotation = glm::vec3(glm::radians(rootGrowthParameters.GetRootApicalAngle(firstRootNode)),
			0.0f,
			rootGrowthParameters.GetRootRollAngle(firstRootNode));
		firstRootNode.m_data.m_verticalTropism = rootGrowthParameters.GetTropismIntensity(firstRootNode);
		firstRootNode.m_data.m_horizontalTropism = 0;
	}
	m_initialized = true;
}

void TreeModel::Clear() {
	m_globalTransform = glm::translate(glm::vec3(0, 0, 0)) * glm::mat4_cast(glm::quat(glm::vec3(0.0f))) * glm::scale(glm::vec3(1.0f));
	m_branchSkeleton = {};
	m_rootSkeleton = {};
	m_history = {};
	m_initialized = false;
}


bool TreeModel::LowBranchPruning(float maxDistance, NodeHandle internodeHandle,
	const TreeGrowthParameters& treeGrowthParameters) {
	auto& internode = m_branchSkeleton.RefNode(internodeHandle);
	//Pruning here.
	if (maxDistance > 5 && internode.m_data.m_order != 0 &&
		internode.m_data.m_rootDistance / maxDistance < treeGrowthParameters.GetLowBranchPruning(internode)) {
		m_branchSkeleton.RecycleNode(internodeHandle);
		return true;
	}

	return false;
}

bool TreeModel::GrowRoots(SoilModel& soilModel, const RootGrowthParameters& rootGrowthParameters)
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
			m_rootGrowthNutrients.m_carbonCapacity = 0;
			m_rootGrowthNutrients.m_totalNitrate = 0;
			for (const auto& rootNodeHandle : sortedRootNodeList) {
				auto& rootNode = m_rootSkeleton.RefNode(rootNodeHandle);
				auto& rootNodeData = rootNode.m_data;
				auto& rootNodeInfo = rootNode.m_info;
				auto pos = m_globalTransform * glm::translate(rootNode.m_info.m_globalPosition);
				rootNode.m_data.m_nitrateLevels = soilModel.GetNutrient(pos[3]);
				m_rootGrowthNutrients.m_totalNitrate += rootNodeData.m_nitrateLevels;
				if (rootNode.GetParentHandle() == -1) {
					rootNodeInfo.m_globalPosition = glm::vec3(0.0f);
					rootNodeInfo.m_localRotation = glm::vec3(0.0f);
					rootNodeInfo.m_globalRotation = glm::vec3(glm::radians(-90.0f), 0.0f, 0.0f);

					rootNodeData.m_rootDistance = rootNodeInfo.m_length;
					rootNodeData.m_rootUnitDistance = 1;
				}
				else {
					auto& parentRootNode = m_rootSkeleton.RefNode(rootNode.GetParentHandle());
					rootNodeData.m_rootDistance = parentRootNode.m_data.m_rootDistance + rootNodeInfo.m_length;
					rootNodeData.m_rootUnitDistance = parentRootNode.m_data.m_rootUnitDistance + 1;
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
			m_rootGrowthNutrients.m_carbonCapacity = m_rootGrowthNutrients.m_totalNitrate * rootGrowthParameters.GetGrowthRate();
		};
		{
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
				for (const auto& rootNodeHandle : flow.RefNodeHandles()) {
					m_rootSkeleton.RefNode(rootNodeHandle).m_data.m_order = flowData.m_order;
				}
			}
			m_rootSkeleton.CalculateFlows();
		};
#pragma endregion
	};
#pragma endregion

	return rootStructureChanged;
}

bool TreeModel::Grow(SoilModel& soilModel, ClimateModel& climateModel, const RootGrowthParameters& rootGrowthParameters, const TreeGrowthParameters& treeGrowthParameters)
{
	bool treeStructureChanged = false;
	bool rootStructureChanged = false;
	if (!m_initialized) {
		Initialize(treeGrowthParameters, rootGrowthParameters);
		treeStructureChanged = true;
		rootStructureChanged = true;
	}
	if (GrowRoots(soilModel, rootGrowthParameters)) {
		rootStructureChanged = true;
	}
	if (GrowBranches(climateModel, treeGrowthParameters)) {
		treeStructureChanged = true;
	}

	m_rootGrowthNutrients.m_carbon = m_rootGrowthNutrients.m_carbonCapacity;
	return treeStructureChanged || rootStructureChanged;
}

Skeleton<SkeletonGrowthData, BranchGrowthData, InternodeGrowthData>& TreeModel::RefBranchSkeleton() {
	return m_branchSkeleton;
}

const Skeleton<SkeletonGrowthData, BranchGrowthData, InternodeGrowthData>&
TreeModel::PeekBranchSkeleton(const int iteration) const {
	assert(iteration >= 0 && iteration <= m_history.size());
	if (iteration == m_history.size()) return m_branchSkeleton;
	return m_history.at(iteration).first;
}

Skeleton<RootSkeletonGrowthData, RootBranchGrowthData, RootInternodeGrowthData>& TreeModel::RefRootSkeleton() {
	return m_rootSkeleton;
}

const Skeleton<RootSkeletonGrowthData, RootBranchGrowthData, RootInternodeGrowthData>&
TreeModel::PeekRootSkeleton(const int iteration) const {
	assert(iteration >= 0 && iteration <= m_history.size());
	if (iteration == m_history.size()) return m_rootSkeleton;
	return m_history.at(iteration).second;
}

void TreeModel::ClearHistory() {
	m_history.clear();
}

void TreeModel::Step() {
	m_history.emplace_back(m_branchSkeleton, m_rootSkeleton);
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
	m_branchSkeleton = m_history[iteration].first;
	m_rootSkeleton = m_history[iteration].second;
	m_history.erase((m_history.begin() + iteration), m_history.end());
}

void InternodeGrowthData::Clear() {
	m_age = 0;
	m_inhibitor = 0;
	m_desiredLocalRotation = glm::vec3(0.0f);
	m_sagging = 0;

	m_maxDistanceToAnyBranchEnd = 0;
	m_level = 0;
	m_descendentTotalBiomass = 0;

	m_rootDistance = 0;

	m_productiveResourceRequirement = 0.0f;
	m_descendentProductiveResourceRequirement = 0.0f;
	m_adjustedTotalProductiveWaterRequirement = 0.0f;
	m_lightDirection = glm::vec3(0, 1, 0);
	m_lightIntensity = 1.0f;
	m_buds.clear();
}
RootGrowthParameters::RootGrowthParameters()
{
	m_growthRate = 0.6f;
	m_rootNodeLength = 0.03f;
	m_endNodeThicknessAndControl = glm::vec2(0.002, 0.5);
	m_thicknessLengthAccumulate = 0.000002f;
	m_branchingAngleMeanVariance = glm::vec2(60, 3);
	m_rollAngleMeanVariance = glm::vec2(120, 2);
	m_apicalAngleMeanVariance = glm::vec2(0, 3);
	m_auxinTransportLoss = 1.0f;
	m_tropismSwitchingProbability = 1.0f;
	m_tropismSwitchingProbabilityDistanceFactor = 0.99f;
	m_tropismIntensity = 0.1f;

	m_baseBranchingProbability = 0.1f;
	m_branchingProbabilityChildrenDecrease = 0.01f;
	m_branchingProbabilityDistanceDecrease = 0.99f;
}
#pragma region TreeGrowthParameters
TreeGrowthParameters::TreeGrowthParameters() {
	m_lateralBudCount = 2;
	m_branchingAngleMeanVariance = glm::vec2(60, 3);
	m_rollAngleMeanVariance = glm::vec2(120, 2);
	m_apicalAngleMeanVariance = glm::vec2(0, 2.5);
	m_gravitropism = 0.03f;
	m_phototropism = 0.0f;
	m_internodeLength = 0.03f;
	m_growthRate = 1.0f;
	m_endNodeThicknessAndControl = glm::vec2(0.002, 0.5);
	m_lateralBudFlushingProbability = 0.5f;
	m_apicalControlBaseDistFactor = { 1.1f, 0.99f };
	m_apicalDominanceBaseAgeDist = glm::vec3(300, 1, 0.97);
	m_budKillProbabilityApicalLateral = glm::vec2(0.0, 0.03);
	m_lowBranchPruning = 0.2f;
	m_saggingFactorThicknessReductionMax = glm::vec3(0.0001, 2, 0.5);

	m_baseResourceRequirementFactor = glm::vec3(1.0f);
	m_productiveResourceRequirementFactor = glm::vec3(1.0f);
}

int TreeGrowthParameters::GetLateralBudCount(const Node<InternodeGrowthData>& internode) const {
	return m_lateralBudCount;
}

float TreeGrowthParameters::GetDesiredBranchingAngle(const Node<InternodeGrowthData>& internode) const {
	return glm::gaussRand(
		m_branchingAngleMeanVariance.x,
		m_branchingAngleMeanVariance.y);
}

float TreeGrowthParameters::GetDesiredRollAngle(const Node<InternodeGrowthData>& internode) const {
	return glm::gaussRand(
		m_rollAngleMeanVariance.x,
		m_rollAngleMeanVariance.y);
}

float TreeGrowthParameters::GetDesiredApicalAngle(const Node<InternodeGrowthData>& internode) const {
	return glm::gaussRand(
		m_apicalAngleMeanVariance.x,
		m_apicalAngleMeanVariance.y);
}

float TreeGrowthParameters::GetGravitropism(const Node<InternodeGrowthData>& internode) const {
	return m_gravitropism;
}

float TreeGrowthParameters::GetPhototropism(const Node<InternodeGrowthData>& internode) const {
	return m_phototropism;
}

float TreeGrowthParameters::GetInternodeLength(const Node<InternodeGrowthData>& internode) const {
	return m_internodeLength;
}

float TreeGrowthParameters::GetGrowthRate(const Node<InternodeGrowthData>& internode) const {
	return m_growthRate;
}

float TreeGrowthParameters::GetEndNodeThickness(const Node<InternodeGrowthData>& internode) const {
	return m_endNodeThicknessAndControl.x;
}

float TreeGrowthParameters::GetThicknessControlFactor(const Node<InternodeGrowthData>& internode) const {
	return m_endNodeThicknessAndControl.y;
}

float TreeGrowthParameters::GetLateralBudFlushingProbability(
	const Node<InternodeGrowthData>& internode) const {
	return m_lateralBudFlushingProbability;
}

float TreeGrowthParameters::GetApicalControl(const Node<InternodeGrowthData>& internode) const {
	return glm::pow(m_apicalControlBaseDistFactor.x, glm::max(1.0f, 1.0f /
		internode.m_data.m_rootDistance *
		m_apicalControlBaseDistFactor.y));
}

float TreeGrowthParameters::GetApicalDominanceBase(const Node<InternodeGrowthData>& internode) const {
	return m_apicalDominanceBaseAgeDist.x *
		glm::pow(
			m_apicalDominanceBaseAgeDist.y,
			internode.m_data.m_age);
}

float TreeGrowthParameters::GetApicalDominanceDecrease(
	const Node<InternodeGrowthData>& internode) const {
	return m_apicalDominanceBaseAgeDist.z;
}

float
TreeGrowthParameters::GetApicalBudKillProbability(const Node<InternodeGrowthData>& internode) const {
	return m_budKillProbabilityApicalLateral.x;
}

float
TreeGrowthParameters::GetLateralBudKillProbability(const Node<InternodeGrowthData>& internode) const {
	return m_budKillProbabilityApicalLateral.y;
}

float TreeGrowthParameters::GetLowBranchPruning(const Node<InternodeGrowthData>& internode) const {
	return m_lowBranchPruning;
}

bool TreeGrowthParameters::GetPruning(const Node<InternodeGrowthData>& internode) const {
	return false;
}

float TreeGrowthParameters::GetSagging(const Node<InternodeGrowthData>& internode) const {
	auto newSagging = glm::min(
		m_saggingFactorThicknessReductionMax.z,
		m_saggingFactorThicknessReductionMax.x *
		(internode.m_data.m_descendentTotalBiomass + internode.m_data.m_extraMass) /
		glm::pow(
			internode.m_info.m_thickness /
			m_endNodeThicknessAndControl.x,
			m_saggingFactorThicknessReductionMax.y));
	return glm::max(internode.m_data.m_sagging, newSagging);
}

float TreeGrowthParameters::GetShootBaseResourceRequirementFactor(
	const Node<InternodeGrowthData>& internode) const {
	return m_baseResourceRequirementFactor.x;
}

float TreeGrowthParameters::GetLeafBaseResourceRequirementFactor(
	const Node<InternodeGrowthData>& internode) const {
	return m_baseResourceRequirementFactor.y;
}

float TreeGrowthParameters::GetFruitBaseResourceRequirementFactor(
	const Node<InternodeGrowthData>& internode) const {
	return m_baseResourceRequirementFactor.z;
}

float TreeGrowthParameters::GetShootProductiveResourceRequirementFactor(
	const Node<InternodeGrowthData>& internode) const {
	return m_productiveResourceRequirementFactor.x;
}

float TreeGrowthParameters::GetLeafProductiveResourceRequirementFactor(
	const Node<InternodeGrowthData>& internode) const {
	return m_productiveResourceRequirementFactor.y;
}

float TreeGrowthParameters::GetFruitProductiveResourceRequirementFactor(
	const Node<InternodeGrowthData>& internode) const {
	return m_productiveResourceRequirementFactor.z;
}
#pragma endregion

#pragma region RootGrowthParameters
float RootGrowthParameters::GetGrowthRate() const
{
	return m_growthRate;
}

float RootGrowthParameters::GetAuxinTransportLoss(const Node<RootInternodeGrowthData>& rootNode) const
{
	return m_auxinTransportLoss;
}

float RootGrowthParameters::GetRootNodeLength(const Node<RootInternodeGrowthData>& rootNode) const
{
	return m_rootNodeLength;
}

float RootGrowthParameters::GetRootApicalAngle(const Node<RootInternodeGrowthData>& rootNode) const
{
	return glm::gaussRand(
		m_apicalAngleMeanVariance.x,
		m_apicalAngleMeanVariance.y);
}

float RootGrowthParameters::GetRootRollAngle(const Node<RootInternodeGrowthData>& rootNode) const
{
	return glm::gaussRand(
		m_rollAngleMeanVariance.x,
		m_rollAngleMeanVariance.y);
}

float RootGrowthParameters::GetRootBranchingAngle(const Node<RootInternodeGrowthData>& rootNode) const
{
	return glm::gaussRand(
		m_branchingAngleMeanVariance.x,
		m_branchingAngleMeanVariance.y);
}

float RootGrowthParameters::GetEndNodeThickness(const Node<RootInternodeGrowthData>& rootNode) const
{
	return m_endNodeThicknessAndControl.x;
}

float RootGrowthParameters::GetThicknessControlFactor(const Node<RootInternodeGrowthData>& rootNode) const
{
	return m_endNodeThicknessAndControl.y;
}

float RootGrowthParameters::GetThicknessAccumulateFactor(const Node<RootInternodeGrowthData>& rootNode) const
{
	return m_thicknessLengthAccumulate;
}

float RootGrowthParameters::GetBranchingProbability(const Node<RootInternodeGrowthData>& rootNode) const
{
	return m_baseBranchingProbability * 
		glm::pow(m_branchingProbabilityChildrenDecrease, rootNode.RefChildHandles().size())
	* glm::pow(m_branchingProbabilityDistanceDecrease, glm::max(0, rootNode.m_data.m_rootUnitDistance - 1));
}

float RootGrowthParameters::GetTropismIntensity(const Node<RootInternodeGrowthData>& rootNode) const
{
	return m_tropismIntensity;
}

void RootGrowthParameters::SetTropisms(Node<RootInternodeGrowthData>& oldNode, Node<RootInternodeGrowthData>& newNode) const
{
	const bool needSwitch = glm::linearRand(0.0f, 1.0f) < m_tropismSwitchingProbability * glm::pow(m_branchingProbabilityDistanceDecrease, glm::max(0, oldNode.m_data.m_rootUnitDistance - 1));
	newNode.m_data.m_horizontalTropism = needSwitch ? oldNode.m_data.m_verticalTropism : oldNode.m_data.m_horizontalTropism;
	newNode.m_data.m_verticalTropism = needSwitch ? oldNode.m_data.m_horizontalTropism : oldNode.m_data.m_verticalTropism;
}
#pragma endregion
