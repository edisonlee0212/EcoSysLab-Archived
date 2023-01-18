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

int TreeVolume::GetSectorIndex(const glm::vec3& position) const
{
	const float layerAngle = 180.0f / m_layerAmount;
	const float sectorAngle = 360.0f / m_sectorAmount;

	const auto v = position - m_center;
	if (glm::abs(v.x + v.z) < 0.000001f) return 0;
	const auto dot = glm::dot(glm::normalize(v), glm::vec3(0, 1, 0));
	const auto rd = glm::degrees(glm::acos(dot));
	const int layerIndex = rd / layerAngle;
	const int sectorIndex = (glm::degrees(glm::atan(v.x, v.z)) + 180.0f) / sectorAngle;
	return glm::clamp(layerIndex * m_sectorAmount + sectorIndex, 0, m_layerAmount * m_sectorAmount - 1);
}

void TreeVolume::Clear()
{
	m_center = glm::vec3(0.0f);
	m_distances.resize(m_layerAmount * m_sectorAmount);
	for (auto& distance : m_distances)
	{
		distance = -1;
	}
	m_hasData = false;
}

void TreeVolume::TipPosition(int layerIndex, int sectorIndex, glm::vec3& position) const
{
	position = m_center;
	const float layerAngle = glm::radians(180.0f / m_layerAmount * (layerIndex + 0.5f));
	const float sectorAngle = glm::radians(360.0f / m_sectorAmount * (sectorIndex + 0.5f));

	const auto& distance = m_distances[layerIndex * m_sectorAmount + sectorIndex];
	if (distance < 0.0f) return;
	position.y += glm::cos(layerAngle) * distance;
	position.x -= glm::sin(layerAngle) * distance * glm::cos(sectorAngle + glm::radians(270.0f));
	position.z += glm::sin(layerAngle) * distance * glm::sin(sectorAngle + glm::radians(270.0f));
}

void TreeVolume::Smooth()
{
	auto copy = m_distances;
	for (int i = 0; i < m_layerAmount; i++)
	{
		for (int j = 0; j < m_sectorAmount; j++)
		{
			int count = 0;
			float sum = 0.0f;
			if (m_distances[i * m_sectorAmount + j] != -1) continue;
			if (i > 0) {
				if (m_distances[(i - 1) * m_sectorAmount + (j + m_sectorAmount - 1) % m_sectorAmount] != -1)
				{
					count++;
					sum += m_distances[(i - 1) * m_sectorAmount + (j + m_sectorAmount - 1) % m_sectorAmount];
				}
				if (m_distances[(i - 1) * m_sectorAmount + j] != -1)
				{
					count++;
					sum += m_distances[(i - 1) * m_sectorAmount + j];
				}
				if (m_distances[(i - 1) * m_sectorAmount + (j + 1) % m_sectorAmount] != -1)
				{
					count++;
					sum += m_distances[(i - 1) * m_sectorAmount + (j + 1) % m_sectorAmount];
				}
			}
			if (m_distances[i * m_sectorAmount + (j + m_sectorAmount - 1) % m_sectorAmount] != -1)
			{
				count++;
				sum += m_distances[i * m_sectorAmount + (j + m_sectorAmount - 1) % m_sectorAmount];
			}
			if (m_distances[i * m_sectorAmount + (j + 1) % m_sectorAmount] != -1)
			{
				count++;
				sum += m_distances[i * m_sectorAmount + (j + 1) % m_sectorAmount];
			}
			if (i < m_layerAmount - 1) {
				if (m_distances[(i + 1) * m_sectorAmount + (j + m_sectorAmount - 1) % m_sectorAmount] != -1)
				{
					count++;
					sum += m_distances[(i + 1) * m_sectorAmount + (j + m_sectorAmount - 1) % m_sectorAmount];
				}
				if (m_distances[(i + 1) * m_sectorAmount + j] != -1)
				{
					count++;
					sum += m_distances[(i + 1) * m_sectorAmount + j];
				}
				if (m_distances[(i + 1) * m_sectorAmount + (j + 1) % m_sectorAmount] != -1)
				{
					count++;
					sum += m_distances[(i + 1) * m_sectorAmount + (j + 1) % m_sectorAmount];
				}
			}
			if (count != 0) copy[i * m_sectorAmount + j] = sum / count;
			else
			{
				copy[i * m_sectorAmount + j] = 0;
			}
		}
	}
	m_distances = copy;
}

float TreeVolume::IlluminationEstimation(const glm::vec3& position,
	const IlluminationEstimationSettings& settings, glm::vec3& lightDirection)
{
	if (!m_hasData || m_distances.empty()) return 1.0f;
	const float layerAngle = 180.0f / settings.m_probeLayerAmount;
	const float sectorAngle = 360.0f / settings.m_probeSectorAmount;
	const auto maxSectorIndex = settings.m_probeLayerAmount * settings.m_probeSectorAmount - 1;
	m_probe.resize(settings.m_probeLayerAmount * settings.m_probeSectorAmount);
	for (auto& sector : m_probe) sector = { 0.0f, 0 };
	for (int i = 0; i < m_layerAmount; i++)
	{
		for (int j = 0; j < m_sectorAmount; j++)
		{
			glm::vec3 tipPosition;
			TipPosition(i, j, tipPosition);
			const float distance = glm::distance(position, tipPosition);
			auto diff = tipPosition - position;
			if (glm::abs(diff.x + diff.z) < 0.000001f) continue;
			const auto dot = glm::dot(glm::normalize(diff), glm::vec3(0, 1, 0));
			const auto rd = glm::degrees(glm::acos(dot));
			const int probeLayerIndex = rd / layerAngle;
			const int probeSectorIndex = (glm::degrees(glm::atan(diff.x, diff.z)) + 180.0f) / sectorAngle;
			auto& probeSector = m_probe[glm::clamp(probeLayerIndex * settings.m_probeSectorAmount + probeSectorIndex, 0, maxSectorIndex)];
			probeSector.first += distance;
			probeSector.second += 1;
		}
	}

	std::vector<float> probeAvg;
	probeAvg.resize(settings.m_probeLayerAmount * settings.m_probeSectorAmount);
	for (int i = 0; i < settings.m_probeLayerAmount; i++)
	{
		for (int j = 0; j < settings.m_probeSectorAmount; j++)
		{
			const auto index = i * settings.m_probeSectorAmount + j;
			const auto& probeSector = m_probe[index];
			if (probeSector.second == 0)
			{
				probeAvg[index] = -1;
			}
			else {
				probeAvg[index] = probeSector.first / probeSector.second;
			}
		}
	}
	{
		for (int i = 0; i < settings.m_probeLayerAmount; i++)
		{
			for (int j = 0; j < settings.m_probeSectorAmount; j++)
			{

				if (probeAvg[i * settings.m_probeSectorAmount + j] == -1) {
					int count = 0;
					float sum = 0.0f;

					if (i > 1) {
						/*
						if (probeAvg[i - 1][(j + settings.m_probeSectorAmount - 1) % settings.m_probeSectorAmount] != -1)
						{
							count++;
							sum += probeAvg[i - 1][(j + settings.m_probeSectorAmount - 1) % settings.m_probeSectorAmount];
						}
						*/
						if (probeAvg[(i - 1) *settings.m_probeSectorAmount + j] != -1)
						{
							count++;
							sum += probeAvg[(i - 1) * settings.m_probeSectorAmount + j];
						}
						/*
						if (probeAvg[i - 1][(j + 1) % settings.m_probeSectorAmount] != -1)
						{
							count++;
							sum += probeAvg[i - 1][(j + 1) % settings.m_probeSectorAmount];
						}
						*/
					}

					if (probeAvg[i * settings.m_probeSectorAmount + (j + settings.m_probeSectorAmount - 1) % settings.m_probeSectorAmount] != -1)
					{
						count++;
						sum += probeAvg[i * settings.m_probeSectorAmount + (j + settings.m_probeSectorAmount - 1) % settings.m_probeSectorAmount];
					}

					if (probeAvg[i * settings.m_probeSectorAmount + (j + 1) % settings.m_probeSectorAmount] != -1)
					{
						count++;
						sum += probeAvg[i * settings.m_probeSectorAmount + (j + 1) % settings.m_probeSectorAmount];
					}

					if (i < settings.m_probeLayerAmount - 1) {
						/*
						if (probeAvg[i + 1][(j + settings.m_probeSectorAmount - 1) % settings.m_probeSectorAmount] != -1)
						{
							count++;
							sum += probeAvg[i + 1][(j + settings.m_probeSectorAmount - 1) % settings.m_probeSectorAmount];
						}
						*/
						if (probeAvg[(i + 1) * settings.m_probeSectorAmount + j] != -1)
						{
							count++;
							sum += probeAvg[(i + 1) * settings.m_probeSectorAmount + j];
						}
						/*
						if (probeAvg[i + 1][(j + 1) % settings.m_probeSectorAmount] != -1)
						{
							count++;
							sum += probeAvg[i + 1][(j + 1) % settings.m_probeSectorAmount];
						}
						*/
					}
					if (count != 0) probeAvg[i * settings.m_probeSectorAmount + j] = sum / count;
					else probeAvg[i * settings.m_probeSectorAmount + j] = 0;
				}
			}
		}
	}

	float lightIntensity = 0.0f;
	float ratio = 1.0f;
	float ratioSum = 0.0f;
	auto direction = glm::vec3(0.0f);
	for (int i = 0; i < settings.m_probeLayerAmount; i++)
	{
		for (int j = 0; j < settings.m_probeSectorAmount; j++)
		{
			ratioSum += ratio;

			glm::vec3 lightDir = glm::vec3(0.0f);
			lightDir.y += glm::cos(glm::radians(i * layerAngle));
			lightDir.x -= glm::sin(glm::radians(i * layerAngle)) * glm::cos(glm::radians(j * sectorAngle + 270.0f));
			lightDir.z += glm::sin(glm::radians(i * layerAngle)) * glm::sin(glm::radians(j * sectorAngle + 270.0f));
			
			float sectorLightIntensity = ratio * (1.0f - glm::clamp(settings.m_occlusion * glm::pow(probeAvg[i * settings.m_probeSectorAmount + j], settings.m_occlusionDistanceFactor), 0.0f, 1.0f));
			lightIntensity += sectorLightIntensity;
			direction += sectorLightIntensity * lightDir;
		}
		ratio *= settings.m_layerAngleFactor;
	}
	lightDirection = glm::normalize(direction);
	lightIntensity = glm::clamp(lightIntensity / ratioSum * settings.m_overallIntensity, 0.0f, 1.0f);
	return lightIntensity;
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
		newRootNode.m_data.m_growthPotential = oldRootNode.m_data.m_growthPotential;
		newRootNode.m_data.m_rootUnitDistance = oldRootNode.m_data.m_rootUnitDistance + 1;
		newRootNode.m_data.m_inhibitorTarget = newRootNode.m_data.m_inhibitor = rootGrowthParameters.m_apicalDominance;
		newRootNode.m_info.m_length = glm::clamp(extendLength, 0.0f, rootNodeLength);
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
			collectedAuxin += rootGrowthParameters.m_apicalDominance;
		}
	}
	else {
		//Otherwise, we add the inhibitor.
		collectedAuxin += rootGrowthParameters.m_apicalDominance;
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
		if (rootNode.RefChildHandles().empty())
		{
			//1. Elongate current node.
			const float developmentVigor = glm::max(0.0f, rootNode.m_data.m_allocatedVigor - rootNode.m_data.m_maintenanceVigorRequirement);
			//const float reproductiveContent = reproductiveWater * m_globalGrowthRate;
			const float extendLength = developmentVigor * rootGrowthParameters.m_rootNodeLength;
			float collectedAuxin = 0.0f;
			const auto dd = rootGrowthParameters.m_apicalDominanceDistanceFactor;
			graphChanged = ElongateRoot(soilModel, extendLength, rootNodeHandle, rootGrowthParameters, collectedAuxin) || graphChanged;
			m_rootSkeleton.RefNode(rootNodeHandle).m_data.m_inhibitorTarget += collectedAuxin * dd;
		}
		else
		{
			//2. Form new shoot if necessary
			float branchingProb = rootGrowthParameters.m_growthRate * rootGrowthParameters.GetBranchingProbability(rootNode);
			branchingProb *= glm::clamp(1.0f - rootNode.m_data.m_inhibitor, 0.0f, 1.0f);
			if (branchingProb >= glm::linearRand(0.0f, 1.0f)) {
				const auto newRootNodeHandle = m_rootSkeleton.Extend(rootNodeHandle, true);
				auto& oldRootNode = m_rootSkeleton.RefNode(rootNodeHandle);
				auto& newRootNode = m_rootSkeleton.RefNode(newRootNodeHandle);
				newRootNode.m_data = {};
				newRootNode.m_data.m_lateral = true;
				rootGrowthParameters.SetTropisms(oldRootNode, newRootNode);
				newRootNode.m_info.m_length = 0.0f;
				newRootNode.m_info.m_thickness = rootGrowthParameters.m_endNodeThickness;
				newRootNode.m_info.m_localRotation = glm::quat(glm::vec3(glm::radians(rootGrowthParameters.GetRootBranchingAngle(newRootNode)), glm::radians(glm::linearRand(0.0f, 360.0f)), 0.0f));
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
				newRootNode.m_data.m_rootUnitDistance = oldRootNode.m_data.m_rootUnitDistance + 1;
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

void TreeModel::CalculateThickness(NodeHandle rootNodeHandle, const RootGrowthParameters& rootGrowthParameters)
{
	auto& rootNode = m_rootSkeleton.RefNode(rootNodeHandle);
	auto& rootNodeData = rootNode.m_data;
	auto& rootNodeInfo = rootNode.m_info;
	rootNodeData.m_descendentTotalBiomass = 0;
	float maxDistanceToAnyBranchEnd = 0;
	float childThicknessCollection = 0.0f;
	//std::set<float> thicknessCollection;
	for (const auto& i : rootNode.RefChildHandles()) {
		auto& childInternode = m_rootSkeleton.RefNode(i);
		const float childMaxDistanceToAnyBranchEnd =
			childInternode.m_data.m_maxDistanceToAnyBranchEnd +
			childInternode.m_info.m_length / rootGrowthParameters.m_rootNodeLength;
		maxDistanceToAnyBranchEnd = glm::max(maxDistanceToAnyBranchEnd, childMaxDistanceToAnyBranchEnd);
		childThicknessCollection += glm::pow(childInternode.m_info.m_thickness,
			1.0f / rootGrowthParameters.m_thicknessAccumulationFactor);
		//thicknessCollection.emplace();
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
	const auto internodeLength = parameters.m_internodeLength;
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
		ApplyTropism(-m_currentGravityDirection, parameters.m_gravitropism, desiredGlobalFront,
			desiredGlobalUp);
		ApplyTropism(internodeData.m_lightDirection, parameters.m_phototropism,
			desiredGlobalFront, desiredGlobalUp);
		//Allocate Lateral bud for current internode
		{
			const auto lateralBudCount = parameters.m_lateralBudCount;
			const float turnAngle = glm::radians(360.0f / lateralBudCount);
			for (int i = 0; i < lateralBudCount; i++) {
				internodeData.m_buds.emplace_back();
				auto& lateralBud = internodeData.m_buds.back();
				lateralBud.m_type = BudType::Lateral;
				lateralBud.m_status = BudStatus::Dormant;
				lateralBud.m_localRotation = glm::vec3(
					glm::radians(parameters.GetDesiredBranchingAngle(internode)), 0.0f,
					i * turnAngle);
			}
		}
		//Allocate Fruit bud for current internode
		{
			const auto fruitBudCount = parameters.m_fruitBudCount;
			for (int i = 0; i < fruitBudCount; i++) {
				internodeData.m_buds.emplace_back();
				auto& fruitBud = internodeData.m_buds.back();
				fruitBud.m_type = BudType::Fruit;
				fruitBud.m_status = BudStatus::Dormant;
				fruitBud.m_localRotation = glm::vec3(
					glm::radians(parameters.GetDesiredBranchingAngle(internode)), 0.0f,
					glm::radians(glm::linearRand(0.0f, 360.0f)));
			}
		}
		//Allocate Leaf bud for current internode
		{
			const auto leafBudCount = parameters.m_leafBudCount;
			for (int i = 0; i < leafBudCount; i++) {
				internodeData.m_buds.emplace_back();
				auto& leafBud = internodeData.m_buds.back();
				leafBud.m_type = BudType::Leaf;
				leafBud.m_status = BudStatus::Dormant;
				leafBud.m_localRotation = glm::vec3(
					glm::radians(parameters.GetDesiredBranchingAngle(internode)), 0.0f,
					glm::radians(glm::linearRand(0.0f, 360.0f)));
			}
		}
		//Create new internode
		const auto newInternodeHandle = m_branchSkeleton.Extend(internodeHandle, false, m_branchSkeleton.RefFlow(internode.GetFlowHandle()).RefNodeHandles().size() > m_flowNodeLimit);
		const auto& oldInternode = m_branchSkeleton.RefNode(internodeHandle);
		auto& newInternode = m_branchSkeleton.RefNode(newInternodeHandle);
		newInternode.m_data = {};
		newInternode.m_data.m_lateral = false;
		newInternode.m_data.m_inhibitorTarget = newInternode.m_data.m_inhibitor = 0.0f;
		newInternode.m_info.m_length = glm::clamp(extendLength, 0.0f, internodeLength);
		newInternode.m_info.m_thickness = parameters.m_endNodeThickness;
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
			childInhibitor *= parameters.m_apicalDominanceDistanceFactor;
			collectedInhibitor += childInhibitor;
			m_branchSkeleton.RefNode(newInternodeHandle).m_data.m_inhibitorTarget = childInhibitor;
		}
		else {
			newInternode.m_data.m_inhibitorTarget = newInternode.m_data.m_inhibitor = parameters.m_apicalDominance;
			collectedInhibitor += newInternode.m_data.m_inhibitor *= parameters.m_apicalDominanceDistanceFactor;
		}
	}
	else {
		//Otherwise, we add the inhibitor.
		collectedInhibitor += parameters.m_apicalDominance * glm::pow(parameters.m_apicalDominanceAgeFactor, parameters.m_growthRate * internodeData.m_age);
	}
	return graphChanged;
}

void TreeModel::CollectShootFlux(ClimateModel& climateModel,
	const TreeGrowthParameters& treeGrowthParameters)
{
	m_plantGrowthNutrients.m_shootFlux = 0.0f;
	const auto& sortedInternodeList = m_branchSkeleton.RefSortedNodeList();
	for (const auto& internodeHandle : sortedInternodeList) {
		auto& internode = m_branchSkeleton.RefNode(internodeHandle);
		auto& internodeData = internode.m_data;
		auto& internodeInfo = internode.m_info;
		internodeData.m_age++;
		internodeData.m_lightIntensity = m_treeVolume.IlluminationEstimation(internodeInfo.m_globalPosition, m_illuminationEstimationSettings, internodeData.m_lightDirection);
		internodeInfo.m_color = glm::mix(glm::vec4(0.0f, 0.0f, 0.0f, 1.0f), glm::vec4(1.0f), internodeData.m_lightIntensity);
		for (const auto& bud : internode.m_data.m_buds)
		{
			if (bud.m_status == BudStatus::Flushed && bud.m_type == BudType::Leaf)
			{
				if (m_collectLight) {
					internodeData.m_shootFlux = internodeData.m_lightIntensity * glm::pow(bud.m_maturity, 2.0f) * bud.m_drought;
					m_plantGrowthNutrients.m_shootFlux += internodeData.m_shootFlux;
				}
			}
		}
	}
	if (!m_collectLight) m_plantGrowthNutrients.m_shootFlux = m_shootGrowthRequirement.m_vigor + m_rootGrowthRequirement.m_vigor;
}

bool TreeModel::GrowInternode(ClimateModel& climateModel, NodeHandle internodeHandle, const TreeGrowthParameters& treeGrowthParameters) {
	bool graphChanged = false;
	{
		auto& internode = m_branchSkeleton.RefNode(internodeHandle);
		auto& internodeData = internode.m_data;
		internodeData.m_inhibitorTarget = 0;
		for (const auto& childHandle : internode.RefChildHandles()) {
			internodeData.m_inhibitorTarget += m_branchSkeleton.RefNode(childHandle).m_data.m_inhibitor *
				treeGrowthParameters.m_apicalDominanceDistanceFactor;
		}
	}
	auto& buds = m_branchSkeleton.RefNode(internodeHandle).m_data.m_buds;
	for (auto& bud : buds) {
		auto& internode = m_branchSkeleton.RefNode(internodeHandle);
		auto& internodeData = internode.m_data;
		auto& internodeInfo = internode.m_info;

		//auto killProbability = treeGrowthParameters.m_growthRate * treeGrowthParameters.m_budKillProbability;
		//if (internodeData.m_rootDistance < 1.0f) killProbability = 0.0f;
		//if (bud.m_status == BudStatus::Dormant && killProbability > glm::linearRand(0.0f, 1.0f)) {
		//	bud.m_status = BudStatus::Removed;
		//}
		//if (bud.m_status == BudStatus::Removed) continue;

		const float baseWater = glm::clamp(bud.m_allocatedVigor, 0.0f, bud.m_maintenanceVigorRequirement);
		bud.m_drought = glm::clamp(1.0f - (1.0f - bud.m_drought) * baseWater / bud.m_maintenanceVigorRequirement, 0.0f, 1.0f);
		const float developmentResource = glm::max(0.0f, bud.m_allocatedVigor - bud.m_maintenanceVigorRequirement);

		if (bud.m_type == BudType::Apical && bud.m_status == BudStatus::Dormant) {
			const float elongateLength = developmentResource * treeGrowthParameters.m_internodeLength;
			float collectedInhibitor = 0.0f;
			const auto dd = treeGrowthParameters.m_apicalDominanceDistanceFactor;
			graphChanged = ElongateInternode(elongateLength, internodeHandle, treeGrowthParameters, collectedInhibitor) || graphChanged;
			m_branchSkeleton.RefNode(internodeHandle).m_data.m_inhibitorTarget += collectedInhibitor * dd;
			//If apical bud is dormant, then there's no lateral bud at this stage. We should quit anyway.
			break;
		}
		if (bud.m_type == BudType::Lateral && bud.m_status == BudStatus::Dormant) {
			const auto& probabilityRange = treeGrowthParameters.m_lateralBudFlushingProbabilityTemperatureRange;
			float flushProbability = treeGrowthParameters.m_growthRate * glm::mix(probabilityRange.x, probabilityRange.y,
				glm::clamp((internodeData.m_temperature - probabilityRange.z) / (probabilityRange.w - probabilityRange.z), 0.0f, 1.0f));
			flushProbability *= glm::clamp(1.0f - internodeData.m_inhibitor, 0.0f, 1.0f);
			flushProbability *= glm::pow(internodeData.m_lightIntensity, treeGrowthParameters.m_lateralBudLightingFactor);
			if (flushProbability >= glm::linearRand(0.0f, 1.0f)) {
				graphChanged = true;
				bud.m_status = BudStatus::Flushed;
				//Prepare information for new internode
				auto desiredGlobalRotation = internodeInfo.m_globalRotation * bud.m_localRotation;
				auto desiredGlobalFront = desiredGlobalRotation * glm::vec3(0, 0, -1);
				auto desiredGlobalUp = desiredGlobalRotation * glm::vec3(0, 1, 0);
				ApplyTropism(-m_currentGravityDirection, treeGrowthParameters.m_gravitropism, desiredGlobalFront,
					desiredGlobalUp);
				ApplyTropism(internodeData.m_lightDirection, treeGrowthParameters.m_phototropism,
					desiredGlobalFront, desiredGlobalUp);
				//Create new internode
				const auto newInternodeHandle = m_branchSkeleton.Extend(internodeHandle, true);
				auto& oldInternode = m_branchSkeleton.RefNode(internodeHandle);
				auto& newInternode = m_branchSkeleton.RefNode(newInternodeHandle);
				newInternode.m_data = {};
				newInternode.m_data.m_lateral = true;
				newInternode.m_info.m_length = 0.0f;
				newInternode.m_info.m_thickness = treeGrowthParameters.m_endNodeThickness;
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

				const auto& probabilityRange = treeGrowthParameters.m_fruitBudFlushingProbabilityTemperatureRange;
				float flushProbability = treeGrowthParameters.m_growthRate * glm::mix(probabilityRange.x, probabilityRange.y,
					glm::clamp((internodeData.m_temperature - probabilityRange.z) / (probabilityRange.w - probabilityRange.z), 0.0f, 1.0f));
				flushProbability *= glm::pow(internodeData.m_lightIntensity, treeGrowthParameters.m_fruitBudLightingFactor);
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
				auto fruitSize = treeGrowthParameters.m_maxFruitSize * bud.m_maturity;
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
				const auto& probabilityRange = treeGrowthParameters.m_fruitBudFlushingProbabilityTemperatureRange;
				float flushProbability = treeGrowthParameters.m_growthRate * glm::mix(probabilityRange.x, probabilityRange.y,
					glm::clamp((internodeData.m_temperature - probabilityRange.z) / (probabilityRange.w - probabilityRange.z), 0.0f, 1.0f));
				flushProbability *= glm::pow(internodeData.m_lightIntensity, treeGrowthParameters.m_leafBudLightingFactor);
				if (internodeData.m_maxDistanceToAnyBranchEnd < treeGrowthParameters.m_leafDistanceToBranchEndLimit && flushProbability >= glm::linearRand(0.0f, 1.0f))
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
				auto leafSize = treeGrowthParameters.m_maxLeafSize * bud.m_maturity;
				glm::quat rotation = internodeInfo.m_globalRotation * bud.m_localRotation;
				auto front = rotation * glm::vec3(0, 0, -1);
				ApplyTropism(glm::vec3(0, -1, 0), 0.9f, rotation);
				auto foliagePosition = front * (leafSize.z * 1.5f);
				bud.m_reproductiveModuleTransform = glm::translate(foliagePosition) * glm::mat4_cast(rotation) * glm::scale(leafSize);

				if (climateModel.m_days % 360 > 180 && internodeData.m_temperature < treeGrowthParameters.m_leafChlorophyllSynthesisFactorTemperature)
				{
					bud.m_chlorophyll -= treeGrowthParameters.m_growthRate * treeGrowthParameters.m_leafChlorophyllLoss;
					bud.m_chlorophyll = glm::clamp(bud.m_chlorophyll, 0.0f, 1.0f);
				}
				if (bud.m_chlorophyll == 0.0f)
				{
					auto dropProbability = treeGrowthParameters.m_growthRate * treeGrowthParameters.m_leafFallProbability;
					if (dropProbability >= glm::linearRand(0.0f, 1.0f))
					{
						bud.m_status = BudStatus::Died;
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

void TreeModel::AggregateInternodeVigorRequirement()
{
	const auto& sortedInternodeList = m_branchSkeleton.RefSortedNodeList();
	for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); it++) {
		auto& internode = m_branchSkeleton.RefNode(*it);
		auto& internodeData = internode.m_data;
		if (!internode.IsEndNode()) {
			//If current node is not end node
			for (const auto& i : internode.RefChildHandles()) {
				auto& childInternode = m_branchSkeleton.RefNode(i);
				internodeData.m_subtreeDevelopmentalVigorRequirement +=
					childInternode.m_data.m_developmentalVigorRequirement
					+ childInternode.m_data.m_subtreeDevelopmentalVigorRequirement;
				internodeData.m_subtreeMaintenanceVigorRequirement +=
					childInternode.m_data.m_maintenanceVigorRequirement
					+ childInternode.m_data.m_subtreeMaintenanceVigorRequirement;
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
		rootNodeData.m_subtreeGrowthPotential = 0.0f;
		rootNodeData.m_subtreeDevelopmentalVigorRequirement = 0.0f;
		if (!rootNode.IsEndNode()) {
			//If current node is not end node
			for (const auto& i : rootNode.RefChildHandles()) {
				const auto& childInternode = m_rootSkeleton.RefNode(i);
				rootNodeData.m_subtreeGrowthPotential += childInternode.m_data.m_growthPotential + childInternode.m_data.m_subtreeGrowthPotential;

				rootNodeData.m_subtreeDevelopmentalVigorRequirement += childInternode.m_data.m_developmentalVigorRequirement + childInternode.m_data.m_subtreeDevelopmentalVigorRequirement;
			}
		}
	}
}

inline void TreeModel::ShootVigorAllocation(const TreeGrowthParameters& treeGrowthParameters)
{
	const auto& sortedInternodeList = m_branchSkeleton.RefSortedNodeList();
	for (const auto& internodeHandle : sortedInternodeList) {
		auto& internode = m_branchSkeleton.RefNode(internodeHandle);
		auto& internodeData = internode.m_data;
		if (internode.GetParentHandle() == -1) {
			if (m_shootGrowthRequirement.m_vigor + m_rootGrowthRequirement.m_vigor != 0.0f) {
				//We firstly determine how much resource being allocated to shoot system on system graph level.
				const auto totalResourceForTree = m_plantGrowthNutrients.m_vigor *
					m_shootGrowthRequirement.m_vigor / (m_shootGrowthRequirement.m_vigor + m_rootGrowthRequirement.m_vigor);
				//The root internode firstly extract it's own resources needed for itself.
				internodeData.m_allocatedVigor = totalResourceForTree *
					(internodeData.m_maintenanceVigorRequirement + internodeData.m_developmentalVigorRequirement)
					/ (internodeData.m_maintenanceVigorRequirement + internodeData.m_developmentalVigorRequirement + 
						internodeData.m_subtreeDevelopmentalVigorRequirement + internodeData.m_subtreeMaintenanceVigorRequirement);
				//The rest resource will be distributed to the descendants. 
				internodeData.m_subTreeAllocatedVigor = totalResourceForTree - internodeData.m_allocatedVigor;
			}
			else
			{
				internodeData.m_allocatedVigor = internodeData.m_subTreeAllocatedVigor = 0.0f;
			}

			//The buds will get its own resources
			for (auto& bud : internodeData.m_buds) {
				if (internodeData.m_maintenanceVigorRequirement + internodeData.m_developmentalVigorRequirement != 0.0f) {
					bud.m_allocatedVigor = internodeData.m_allocatedVigor *
						(bud.m_developmentVigorRequirement + bud.m_maintenanceVigorRequirement) / (internodeData.m_maintenanceVigorRequirement + internodeData.m_developmentalVigorRequirement);
				}
				else
				{
					bud.m_allocatedVigor = 0.0f;
				}
			}
		}
		const float apicalControl = treeGrowthParameters.GetApicalControlFactor(internode);
		if (internodeData.m_subTreeAllocatedVigor != 0.0f) {
			float childDevelopmentalVigorRequirementWeightSum = 0.0f;
			float childMaintenanceVigorRequirementWeightSum = 0.0f;
			for (const auto& i : internode.RefChildHandles()) {
				auto& childInternode = m_branchSkeleton.RefNode(i);
				const auto& childInternodeData = childInternode.m_data;

				float childDevelopmentalVigorRequirementWeight =
					(childInternodeData.m_developmentalVigorRequirement + childInternodeData.m_subtreeDevelopmentalVigorRequirement) / internodeData.m_subtreeDevelopmentalVigorRequirement;

				childDevelopmentalVigorRequirementWeight = glm::pow(childDevelopmentalVigorRequirementWeight, apicalControl);

				childDevelopmentalVigorRequirementWeightSum += childDevelopmentalVigorRequirementWeight;
			}
			for (const auto& i : internode.RefChildHandles()) {
				auto& childInternode = m_branchSkeleton.RefNode(i);
				auto& childInternodeData = childInternode.m_data;

				float childDevelopmentalVigorRequirementWeight =
					(childInternodeData.m_developmentalVigorRequirement + childInternodeData.m_subtreeDevelopmentalVigorRequirement) / internodeData.m_subtreeDevelopmentalVigorRequirement;

				childDevelopmentalVigorRequirementWeight = glm::pow(childDevelopmentalVigorRequirementWeight, apicalControl);


				const float childTotalAllocatedResource = childResourceWeight / childDevelopmentalVigorRequirementWeightSum * internodeData.m_subTreeAllocatedVigor;

				childInternodeData.m_allocatedVigor = childTotalAllocatedResource *
					(childInternodeData.m_maintenanceVigorRequirement + childInternodeData.m_developmentalVigorRequirement)
					/ (childInternodeData.m_maintenanceVigorRequirement + childInternodeData.m_developmentalVigorRequirement + childInternodeData.m_subtreeDevelopmentalVigorRequirement);
				childInternodeData.m_subTreeAllocatedVigor = childTotalAllocatedResource - childInternodeData.m_allocatedVigor;

				for (auto& bud : childInternodeData.m_buds) {
					bud.m_allocatedVigor = childInternodeData.m_allocatedVigor *
						(bud.m_developmentVigorRequirement + bud.m_maintenanceVigorRequirement)
						/ (childInternodeData.m_maintenanceVigorRequirement + childInternodeData.m_developmentalVigorRequirement);
				}
			}
		}else
		{
			for (const auto& i : internode.RefChildHandles()) {
				auto& childInternode = m_branchSkeleton.RefNode(i);
				auto& childInternodeData = childInternode.m_data;
				childInternodeData.m_allocatedVigor = childInternodeData.m_subTreeAllocatedVigor = 0.0f;
				for (auto& bud : childInternodeData.m_buds) {
					bud.m_allocatedVigor = 0.0f;
				}
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
			childInternode.m_info.m_length / treeGrowthParameters.m_internodeLength;
		maxDistanceToAnyBranchEnd = glm::max(maxDistanceToAnyBranchEnd, childMaxDistanceToAnyBranchEnd);

		childThicknessCollection += glm::pow(childInternode.m_info.m_thickness,
			1.0f / treeGrowthParameters.m_thicknessAccumulationFactor);
	}
	childThicknessCollection += treeGrowthParameters.m_thicknessAccumulateAgeFactor * treeGrowthParameters.m_endNodeThickness * treeGrowthParameters.m_growthRate * internodeData.m_age;


	internodeData.m_maxDistanceToAnyBranchEnd = maxDistanceToAnyBranchEnd;
	if (childThicknessCollection != 0.0f) {
		internodeInfo.m_thickness = glm::max(internodeInfo.m_thickness, glm::pow(childThicknessCollection,
			treeGrowthParameters.m_thicknessAccumulationFactor));
	}
	else
	{
		internodeInfo.m_thickness = glm::max(internodeInfo.m_thickness, treeGrowthParameters.m_endNodeThickness);
	}

	internodeData.m_biomass =
		internodeInfo.m_thickness / treeGrowthParameters.m_endNodeThickness * internodeInfo.m_length /
		treeGrowthParameters.m_internodeLength;
	for (const auto& i : internode.RefChildHandles()) {
		const auto& childInternode = m_branchSkeleton.RefNode(i);
		internodeData.m_descendentTotalBiomass +=
			childInternode.m_data.m_descendentTotalBiomass +
			childInternode.m_data.m_biomass;
	}
	internodeData.m_sagging = treeGrowthParameters.GetSagging(internode);
}

void TreeModel::CalculateVigorRequirement(const TreeGrowthParameters& treeGrowthParameters, TreeGrowthRequirement& newTreeGrowthNutrientsRequirement) {

	const auto& sortedInternodeList = m_branchSkeleton.RefSortedNodeList();
	for (const auto& internodeHandle : sortedInternodeList) {
		auto& internode = m_branchSkeleton.RefNode(internodeHandle);
		auto& internodeData = internode.m_data;
		internodeData.m_maintenanceVigorRequirement = 0.0f;
		internodeData.m_developmentalVigorRequirement = 0.0f;
		internodeData.m_subtreeDevelopmentalVigorRequirement = 0.0f;
		const auto growthRate = treeGrowthParameters.m_growthRate;
		for (auto& bud : internodeData.m_buds) {
			if (bud.m_status == BudStatus::Died || bud.m_status == BudStatus::Removed) {
				bud.m_maintenanceVigorRequirement = 0.0f;
				bud.m_developmentVigorRequirement = 0.0f;
				continue;
			}
			switch (bud.m_type) {
			case BudType::Apical: {
				bud.m_maintenanceVigorRequirement = treeGrowthParameters.m_shootMaintenanceVigorRequirement;
				if (bud.m_status == BudStatus::Dormant) {
					//Elongation
					bud.m_developmentVigorRequirement = treeGrowthParameters.m_shootProductiveWaterRequirement;
				}
			}break;
			case BudType::Leaf: {
				bud.m_maintenanceVigorRequirement = treeGrowthParameters.m_leafMaintenanceVigorRequirement;
				bud.m_developmentVigorRequirement = treeGrowthParameters.m_leafProductiveWaterRequirement;
			}break;
			case BudType::Fruit: {
				bud.m_maintenanceVigorRequirement = treeGrowthParameters.m_fruitBaseWaterRequirement;
				bud.m_developmentVigorRequirement = treeGrowthParameters.m_fruitProductiveWaterRequirement;
			}break;
			case BudType::Lateral: {
				bud.m_maintenanceVigorRequirement = 0.0f;
				bud.m_developmentVigorRequirement = 0.0f;
			}break;
			}
			bud.m_developmentVigorRequirement = growthRate * bud.m_developmentVigorRequirement;
			internodeData.m_developmentalVigorRequirement += bud.m_developmentVigorRequirement;
			internodeData.m_maintenanceVigorRequirement += bud.m_maintenanceVigorRequirement;
			newTreeGrowthNutrientsRequirement.m_vigor += bud.m_developmentVigorRequirement + bud.m_maintenanceVigorRequirement;
		}
	}
}

bool TreeModel::GrowShoots(const glm::mat4& globalTransform, ClimateModel& climateModel, const TreeGrowthParameters& treeGrowthParameters, TreeGrowthRequirement& newTreeGrowthRequirement) {
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
			if (InternodePruning(maxDistance, internodeHandle, treeGrowthParameters)) {
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
		AggregateInternodeVigorRequirement();
		ShootVigorAllocation(treeGrowthParameters);
		const auto& sortedInternodeList = m_branchSkeleton.RefSortedNodeList();
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
					internodeInfo.m_length / treeGrowthParameters.m_internodeLength;
			}
			else {
				auto& parentInternode = m_branchSkeleton.RefNode(internode.GetParentHandle());
				internodeData.m_rootDistance = parentInternode.m_data.m_rootDistance + internodeInfo.m_length /
					treeGrowthParameters.m_internodeLength;
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
		SampleTemperature(globalTransform, climateModel);
		CalculateVigorRequirement(treeGrowthParameters, newTreeGrowthRequirement);
		m_treeVolume.Clear();
		if (!sortedInternodeList.empty())
		{
			m_treeVolume.m_center = (m_branchSkeleton.m_max + m_branchSkeleton.m_min) * 0.5f;
			for (const auto& internodeHandle : sortedInternodeList)
			{
				auto& internode = m_branchSkeleton.RefNode(internodeHandle);
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
		const float minRadius = treeGrowthParameters.m_endNodeThickness * 4.0f;
		CollisionDetection(minRadius, m_branchOctree, m_branchSkeleton);
	}
	m_internodeOrderCounts.clear();
	{
		int maxOrder = 0;
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
			maxOrder = glm::max(maxOrder, flowData.m_order);
		}
		m_internodeOrderCounts.resize(maxOrder + 1);
		std::fill(m_internodeOrderCounts.begin(), m_internodeOrderCounts.end(), 0);
		const auto& sortedInternodeList = m_branchSkeleton.RefSortedNodeList();
		for (const auto& internodeHandle : sortedInternodeList)
		{
			auto& internode = m_branchSkeleton.RefNode(internodeHandle);
			const auto order = m_branchSkeleton.RefFlow(internode.GetFlowHandle()).m_data.m_order;
			internode.m_data.m_order = order;
			m_internodeOrderCounts[order]++;
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
		firstInternode.m_info.m_thickness = treeGrowthParameters.m_endNodeThickness;
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
		firstRootNode.m_data.m_verticalTropism = rootGrowthParameters.m_tropismIntensity;
		firstRootNode.m_data.m_horizontalTropism = 0;
	}
	m_initialized = true;
}

void TreeModel::Clear() {
	m_branchSkeleton = {};
	m_rootSkeleton = {};
	m_history = {};
	m_initialized = false;
	m_treeVolume.Clear();
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

bool TreeModel::InternodePruning(float maxDistance, NodeHandle internodeHandle,
	const TreeGrowthParameters& treeGrowthParameters) {
	auto& internode = m_branchSkeleton.RefNode(internodeHandle);
	//Pruning here.
	bool pruning = false;

	if (maxDistance > 5 && internode.m_data.m_order != 0 &&
		internode.m_data.m_rootDistance / maxDistance < treeGrowthParameters.m_lowBranchPruning) {
		pruning = true;
	}
	if (internode.IsEndNode())
	{
		float pruningProbability = treeGrowthParameters.m_growthRate * (1.0f - internode.m_data.m_lightIntensity) * treeGrowthParameters.m_endNodePruningLightFactor;

		if (pruningProbability >= glm::linearRand(0.0f, 1.0f)) pruning = true;
	}

	if (pruning)
	{
		m_branchSkeleton.RecycleNode(internodeHandle);
	}
	return pruning;
}

bool TreeModel::GrowRoots(const glm::mat4& globalTransform, SoilModel& soilModel, const RootGrowthParameters& rootGrowthParameters, TreeGrowthRequirement& newTreeGrowthRequirement)
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
			RootVigorAllocation(rootGrowthParameters);
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
		};
		SampleSoilDensity(globalTransform, soilModel);
		SampleNitrite(globalTransform, soilModel);
		CalculateVigorRequirement(rootGrowthParameters, newTreeGrowthRequirement);
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
					if(rootNode.m_data.m_fineRootAnchors.empty())
					{
						rootNode.m_data.m_fineRootAnchors.resize(5);
						auto desiredGlobalRotation = rootNode.m_info.m_globalRotation * glm::quat(glm::vec3(
							glm::radians(rootGrowthParameters.m_fineRootBranchingAngle), 0.0f,
							glm::radians(rootGrowthParameters.GetRootRollAngle(rootNode))));

						glm::vec3 positionWalker = rootNode.m_info.m_globalPosition;
						for(int i = 0; i < 5; i++)
						{
							auto front = desiredGlobalRotation * glm::vec3(0, 0, -1);
							positionWalker = positionWalker + front * rootGrowthParameters.m_fineRootSegmentLength;
							rootNode.m_data.m_fineRootAnchors[i] = glm::vec4(positionWalker, rootGrowthParameters.m_fineRootThickness);
							desiredGlobalRotation = rootNode.m_info.m_globalRotation * glm::quat(glm::vec3(
								glm::radians(glm::gaussRand(0.f, rootGrowthParameters.m_fineRootApicalAngleVariance) + rootGrowthParameters.m_fineRootBranchingAngle), 0.0f,
								glm::radians(glm::linearRand(0.0f, 360.0f))));
						}
					}
				}else
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

void TreeModel::CollectRootFlux(const glm::mat4& globalTransform, SoilModel& soilModel, const RootGrowthParameters& rootGrowthParameters)
{
	m_plantGrowthNutrients.m_rootFlux = 0.0f;
	const auto& sortedRootNodeList = m_rootSkeleton.RefSortedNodeList();
	for (const auto& rootNodeHandle : sortedRootNodeList) {
		auto& rootNode = m_rootSkeleton.RefNode(rootNodeHandle);
		rootNode.m_data.m_age++;
		auto& rootNodeInfo = rootNode.m_info;
		auto worldSpacePosition = globalTransform * glm::translate(rootNodeInfo.m_globalPosition)[3];
		if (m_collectWater) {
			rootNode.m_data.m_rootFlux = soilModel.GetWater(worldSpacePosition);
			m_plantGrowthNutrients.m_rootFlux += rootNode.m_data.m_rootFlux;
		}
	}
	if (!m_collectWater) m_plantGrowthNutrients.m_rootFlux = m_shootGrowthRequirement.m_vigor + m_rootGrowthRequirement.m_vigor;
}

void TreeModel::SampleNitrite(const glm::mat4& globalTransform, SoilModel& soilModel)
{
	const auto& sortedRootNodeList = m_rootSkeleton.RefSortedNodeList();
	for (const auto& rootNodeHandle : sortedRootNodeList) {
		auto& rootNode = m_rootSkeleton.RefNode(rootNodeHandle);
		auto& rootNodeInfo = rootNode.m_info;
		auto worldSpacePosition = globalTransform * glm::translate(rootNodeInfo.m_globalPosition)[3];
		if (m_collectNitrite) {
			rootNode.m_data.m_nitrite = soilModel.GetNutrient(worldSpacePosition);
			m_plantGrowthNutrients.m_nitrite += rootNode.m_data.m_nitrite;
		}
	}
}

void TreeModel::RootVigorAllocation(const RootGrowthParameters& rootGrowthParameters)
{
	const auto& sortedRootNodeList = m_rootSkeleton.RefSortedNodeList();
	for (const auto& rootNodeHandle : sortedRootNodeList) {
		auto& rootNode = m_rootSkeleton.RefNode(rootNodeHandle);
		auto& rootNodeData = rootNode.m_data;
		if (rootNode.GetParentHandle() == -1) {
			if (m_shootGrowthRequirement.m_vigor + m_rootGrowthRequirement.m_vigor != 0.0f) {
				const auto totalResourceForRoot = m_plantGrowthNutrients.m_vigor *
					m_rootGrowthRequirement.m_vigor / (m_shootGrowthRequirement.m_vigor + m_rootGrowthRequirement.m_vigor);
				rootNodeData.m_allocatedVigor = totalResourceForRoot * 
					rootNodeData.m_growthPotential / (rootNodeData.m_growthPotential + rootNodeData.m_subtreeGrowthPotential);
				rootNodeData.m_subTreeAllocatedVigor = totalResourceForRoot - rootNodeData.m_allocatedVigor;
			}
			else
			{
				rootNodeData.m_allocatedVigor = rootNodeData.m_subTreeAllocatedVigor = 0.0f;
			}
		}
		//const float apicalControl = rootGrowthParameters.GetApicalControlFactor(rootNode);
		if (rootNodeData.m_subTreeAllocatedVigor != 0.0f) {
			float childResourceWeightSum = 0.0f;
			for (const auto& i : rootNode.RefChildHandles()) {
				const auto& childRootNode = m_rootSkeleton.RefNode(i);
				const auto& childRootNodeData = childRootNode.m_data;

				float childResourceWeight =
					(childRootNodeData.m_growthPotential + childRootNodeData.m_subtreeGrowthPotential) / rootNodeData.m_subtreeGrowthPotential;

				//childResourceWeight = glm::pow(childResourceWeight, apicalControl);

				childResourceWeightSum += childResourceWeight;
			}
			for (const auto& i : rootNode.RefChildHandles()) {
				auto& childRootNode = m_rootSkeleton.RefNode(i);
				auto& childRootNodeData = childRootNode.m_data;

				float childResourceWeight =
					(childRootNodeData.m_growthPotential + childRootNodeData.m_subtreeGrowthPotential) / rootNodeData.m_subtreeGrowthPotential;

				//childResourceWeight = glm::pow(childResourceWeight, apicalControl);

				const float childTotalAllocatedResource = childResourceWeight / childResourceWeightSum * rootNodeData.m_subTreeAllocatedVigor;

				childRootNodeData.m_allocatedVigor = childTotalAllocatedResource *
					childRootNodeData.m_growthPotential
					/ (childRootNodeData.m_growthPotential + childRootNodeData.m_subtreeGrowthPotential);
				childRootNodeData.m_subTreeAllocatedVigor = childTotalAllocatedResource - childRootNodeData.m_allocatedVigor;
			}
		}else
		{
			for (const auto& i : rootNode.RefChildHandles())
			{
				auto& childRootNode = m_rootSkeleton.RefNode(i);
				auto& childRootNodeData = childRootNode.m_data;
				childRootNodeData.m_allocatedVigor = childRootNodeData.m_subTreeAllocatedVigor = 0.0f;
			}
		}
	}
}

void TreeModel::CalculateVigorRequirement(const RootGrowthParameters& rootGrowthParameters,
	TreeGrowthRequirement& newRootGrowthNutrientsRequirement)
{
	const auto& sortedRootNodeList = m_rootSkeleton.RefSortedNodeList();
	const auto growthRate = rootGrowthParameters.m_growthRate;
	for (const auto& rootNodeHandle : sortedRootNodeList) {
		auto& rootNode = m_rootSkeleton.RefNode(rootNodeHandle);
		auto& rootNodeData = rootNode.m_data;
		rootNodeData.m_developmentalVigorRequirement = 0.0f;
		rootNodeData.m_maintenanceVigorRequirement = 0.0f;
		rootNodeData.m_developmentalVigorRequirement = growthRate * glm::pow(1.0f / glm::max(rootNodeData.m_soilDensity * rootGrowthParameters.m_environmentalFriction, 1.0f), rootGrowthParameters.m_environmentalFrictionFactor);
		rootNodeData.m_growthPotential = rootNodeData.m_nitrite * rootNodeData.m_developmentalVigorRequirement + rootNodeData.m_maintenanceVigorRequirement;
		newRootGrowthNutrientsRequirement.m_vigor += rootNodeData.m_developmentalVigorRequirement + rootNodeData.m_maintenanceVigorRequirement;
	}
}

bool TreeModel::Grow(const glm::mat4& globalTransform, SoilModel& soilModel, ClimateModel& climateModel,
	const RootGrowthParameters& rootGrowthParameters, const TreeGrowthParameters& treeGrowthParameters)
{
	m_fruitCount = 0;
	m_leafCount = 0;

	bool treeStructureChanged = false;
	bool rootStructureChanged = false;
	if (!m_initialized) {
		Initialize(treeGrowthParameters, rootGrowthParameters);
		treeStructureChanged = true;
		rootStructureChanged = true;
	}
	//Collect water from roots.
	CollectRootFlux(globalTransform, soilModel, rootGrowthParameters);
	//Collect light from branches.
	CollectShootFlux(climateModel, treeGrowthParameters);
	//Perform photosynthesis.
	m_plantGrowthNutrients.m_vigor = glm::min(m_plantGrowthNutrients.m_rootFlux, m_plantGrowthNutrients.m_shootFlux);
	//Grow roots and set up nutrient requirements for next iteration.
	TreeGrowthRequirement newShootGrowthRequirement;
	TreeGrowthRequirement newRootGrowthRequirement;
	if (rootGrowthParameters.m_growthRate != 0 && GrowRoots(globalTransform, soilModel, rootGrowthParameters, newRootGrowthRequirement)) {
		rootStructureChanged = true;
	}
	//Grow branches and set up nutrient requirements for next iteration.
	if (treeGrowthParameters.m_growthRate != 0 && GrowShoots(globalTransform, climateModel, treeGrowthParameters, newShootGrowthRequirement)) {
		treeStructureChanged = true;
	}
	//Set new growth nutrients requirement for next iteration.
	m_shootGrowthRequirement = newShootGrowthRequirement;
	m_rootGrowthRequirement = newRootGrowthRequirement;
	return treeStructureChanged || rootStructureChanged;
}

void TreeModel::SampleTemperature(const glm::mat4& globalTransform, ClimateModel& climateModel)
{
	const auto& sortedInternodeList = m_branchSkeleton.RefSortedNodeList();
	for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); it++) {
		auto& internode = m_branchSkeleton.RefNode(*it);
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

	m_baseBranchingProbability = 0.4f;
	m_branchingProbabilityChildrenDecrease = 0.98f;
	m_branchingProbabilityDistanceDecrease = 0.98f;
	m_branchingProbabilityOrderDecrease = 1.0f;
}
#pragma region TreeGrowthParameters
TreeGrowthParameters::TreeGrowthParameters() {
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

	m_shootMaintenanceVigorRequirement = 1.0f;
	m_leafMaintenanceVigorRequirement = 1.0f;
	m_fruitBaseWaterRequirement = 1.0f;
	m_shootProductiveWaterRequirement = 1.0f;
	m_leafProductiveWaterRequirement = 1.0f;
	m_fruitProductiveWaterRequirement = 1.0f;



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

float TreeGrowthParameters::GetApicalControlFactor(const Node<InternodeGrowthData>& internode) const {
	const float ageDecrease = glm::pow(m_apicalControlAgeFactor, internode.m_data.m_age * m_growthRate);
	const float distDecrease = glm::pow(m_apicalControlDistanceFactor, internode.m_data.m_rootDistance);
	return m_apicalControl * ageDecrease * distDecrease;
}

float TreeGrowthParameters::GetSagging(const Node<InternodeGrowthData>& internode) const {
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


float RootGrowthParameters::GetBranchingProbability(const Node<RootInternodeGrowthData>& rootNode) const
{
	return m_baseBranchingProbability
		* glm::pow(m_branchingProbabilityChildrenDecrease, rootNode.RefChildHandles().size())
		* glm::pow(m_branchingProbabilityDistanceDecrease, glm::max(0, rootNode.m_data.m_rootUnitDistance - 1));
}

float RootGrowthParameters::GetApicalControlFactor(const Node<RootInternodeGrowthData>& rootNode) const
{
	return glm::pow(m_apicalControlBaseDistFactor.x, glm::max(1.0f, 1.0f /
		rootNode.m_data.m_rootDistance *
		m_apicalControlBaseDistFactor.y));
}

void RootGrowthParameters::SetTropisms(Node<RootInternodeGrowthData>& oldNode, Node<RootInternodeGrowthData>& newNode) const
{
	float probability = m_tropismSwitchingProbability * glm::pow(m_tropismSwitchingProbabilityDistanceFactor, glm::max(0, oldNode.m_data.m_rootUnitDistance - 1));
	const bool needSwitch = probability >= glm::linearRand(0.0f, 1.0f);
	newNode.m_data.m_horizontalTropism = needSwitch ? oldNode.m_data.m_verticalTropism : oldNode.m_data.m_horizontalTropism;
	newNode.m_data.m_verticalTropism = needSwitch ? oldNode.m_data.m_horizontalTropism : oldNode.m_data.m_verticalTropism;
}
#pragma endregion
