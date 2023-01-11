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

glm::ivec2 TreeVolume::SelectSlice(const glm::vec3& position) const
{
	glm::ivec2 retVal;
	const float heightLevel = m_maxHeight / m_layerAmount;
	const float sliceAngle = 360.0f / m_sectorAmount;
	auto x = static_cast<int>(position.y / heightLevel);
	if (x < 0)
		x = 0;
	retVal.x = x;
	if (retVal.x >= m_layerAmount)
		retVal.x = m_layerAmount - 1;
	if (position.x == 0.0f && position.z == 0.0f)
		retVal.y = 0;
	else
		retVal.y = static_cast<int>(
			(glm::degrees(glm::atan(position.x, position.z)) + 180.0f) /
			sliceAngle);
	if (retVal.y >= m_sectorAmount)
		retVal.y = m_sectorAmount - 1;
	return retVal;
}

void TreeVolume::Clear()
{
	m_layers.resize(m_layerAmount);
	for (auto& layer : m_layers)
	{
		layer.resize(m_sectorAmount);
		for (auto& distance : layer)
		{
			distance = -1;
		}
	}
	m_maxHeight = 0.0f;
	m_hasData = false;

}

glm::vec3 TreeVolume::TipPosition(int layer, int slice) const
{
	const float heightLevel = m_maxHeight / m_layerAmount;
	glm::vec3 retVal;
	retVal.y = heightLevel * (0.5f + layer);
	const float sliceAngle = 360.0f / m_sectorAmount;
	const auto& target = m_layers[layer][slice];
	retVal.x = target * glm::sin(glm::radians(sliceAngle * slice + 180.0f));
	retVal.z = target * glm::cos(glm::radians(sliceAngle * slice + 180.0f));
	return retVal;
}

void TreeVolume::Smooth()
{
	auto copy = m_layers;
	for (int i = 0; i < m_layerAmount; i++)
	{
		for (int j = 0; j < m_sectorAmount; j++)
		{
			int count = 0;
			float sum = 0.0f;
			if (m_layers[i][j] != -1) continue;
			if (i > 0) {
				if (m_layers[i - 1][(j + m_sectorAmount - 1) % m_sectorAmount] != -1)
				{
					count++;
					sum += m_layers[i - 1][(j + m_sectorAmount - 1) % m_sectorAmount];
				}
				if (m_layers[i - 1][j] != -1)
				{
					count++;
					sum += m_layers[i - 1][j];
				}
				if (m_layers[i - 1][(j + 1) % m_sectorAmount] != -1)
				{
					count++;
					sum += m_layers[i - 1][(j + 1) % m_sectorAmount];
				}
			}
			if (m_layers[i][(j + m_sectorAmount - 1) % m_sectorAmount] != -1)
			{
				count++;
				sum += m_layers[i][(j + m_sectorAmount - 1) % m_sectorAmount];
			}
			if (m_layers[i][(j + 1) % m_sectorAmount] != -1)
			{
				count++;
				sum += m_layers[i][(j + 1) % m_sectorAmount];
			}
			if (i < m_layerAmount - 1) {
				if (m_layers[i + 1][(j + m_sectorAmount - 1) % m_sectorAmount] != -1)
				{
					count++;
					sum += m_layers[i + 1][(j + m_sectorAmount - 1) % m_sectorAmount];
				}
				if (m_layers[i + 1][j] != -1)
				{
					count++;
					sum += m_layers[i + 1][j];
				}
				if (m_layers[i + 1][(j + 1) % m_sectorAmount] != -1)
				{
					count++;
					sum += m_layers[i + 1][(j + 1) % m_sectorAmount];
				}
			}
			if(count != 0) copy[i][j] = sum / count;
			else
			{
				copy[i][j] = 0;
			}
		}
	}
	m_layers = copy;
}

float TreeVolume::IlluminationEstimation(const glm::vec3& position,
	const IlluminationEstimationSettings& settings) const
{
	if (!m_hasData || m_layers.empty()) return 1.0f;
	std::vector<std::vector<std::pair<float, int>>> probes;
	const float layerAngle = 180.0f / settings.m_probeLayerAmount;
	const float sectorAngle = 360.0f / settings.m_probeCountPerLayer;
	probes.resize(settings.m_probeLayerAmount);
	for (auto& probeLayer : probes)
	{
		probeLayer.resize(settings.m_probeCountPerLayer);
		for (auto& i : probeLayer) i = { 0.0, 0 };
	}
	for (int i = 0; i < m_layerAmount; i++)
	{
		for (int j = 0; j < m_sectorAmount; j++)
		{
			auto tipPosition = TipPosition(i, j);
			float distance = glm::distance(position, tipPosition);
			auto diff = tipPosition - position;
			if(diff.x == 0 && diff.z == 0) continue;
			auto dot = glm::clamp(glm::dot(glm::normalize(diff), glm::normalize(glm::vec3(diff.x, 0.0f, diff.z))), 0.0f, 1.0f);
			auto rd = glm::acos(dot);
			const int layerOffset = glm::degrees(rd) / layerAngle;
			int probeLayerIndex = diff.y > 0 ? settings.m_probeLayerAmount / 2 - layerOffset : layerOffset + settings.m_probeLayerAmount / 2;
			int probeSectorIndex = glm::degrees(glm::atan(diff.x, diff.z)) / sectorAngle;
			if (probeLayerIndex < 0) probeLayerIndex += settings.m_probeLayerAmount;
			if (probeSectorIndex < 0) probeSectorIndex += settings.m_probeCountPerLayer;
			probeLayerIndex = probeLayerIndex % settings.m_probeLayerAmount;
			probeSectorIndex = probeSectorIndex % settings.m_probeCountPerLayer;
			probes[probeLayerIndex][probeSectorIndex].first += distance;
			probes[probeLayerIndex][probeSectorIndex].second += 1;
		}
	}

	std::vector<std::vector<float>> probeAvg;
	probeAvg.resize(settings.m_probeLayerAmount);
	for (int i = 0; i < settings.m_probeLayerAmount; i++)
	{
		probeAvg[i].resize(settings.m_probeCountPerLayer);
		for (int j = 0; j < settings.m_probeCountPerLayer; j++)
		{
			const auto& probe = probes[i][j];
			if (probe.second == 0)
			{
				probeAvg[i][j] = -1;
			}
			else {
				probeAvg[i][j] = probe.first / probe.second;
			}
		}
	}
	{
		for (int i = 0; i < settings.m_probeLayerAmount; i++)
		{
			for (int j = 0; j < settings.m_probeCountPerLayer; j++)
			{
				if (probeAvg[i][j] == -1) {
					int count = 0;
					float sum = 0.0f;

					if (i > 1) {
						/*
						if (probeAvg[i - 1][(j + settings.m_probeCountPerLayer - 1) % settings.m_probeCountPerLayer] != -1)
						{
							count++;
							sum += probeAvg[i - 1][(j + settings.m_probeCountPerLayer - 1) % settings.m_probeCountPerLayer];
						}
						*/
						if (probeAvg[i - 1][j] != -1)
						{
							count++;
							sum += probeAvg[i - 1][j];
						}
						/*
						if (probeAvg[i - 1][(j + 1) % settings.m_probeCountPerLayer] != -1)
						{
							count++;
							sum += probeAvg[i - 1][(j + 1) % settings.m_probeCountPerLayer];
						}
						*/
					}

					if (probeAvg[i][(j + settings.m_probeCountPerLayer - 1) % settings.m_probeCountPerLayer] != -1)
					{
						count++;
						sum += probeAvg[i][(j + settings.m_probeCountPerLayer - 1) % settings.m_probeCountPerLayer];
					}

					if (probeAvg[i][(j + 1) % settings.m_probeCountPerLayer] != -1)
					{
						count++;
						sum += probeAvg[i][(j + 1) % settings.m_probeCountPerLayer];
					}

					if (i < settings.m_probeLayerAmount - 1) {
						/*
						if (probeAvg[i + 1][(j + settings.m_probeCountPerLayer - 1) % settings.m_probeCountPerLayer] != -1)
						{
							count++;
							sum += probeAvg[i + 1][(j + settings.m_probeCountPerLayer - 1) % settings.m_probeCountPerLayer];
						}
						*/
						if (probeAvg[i + 1][j] != -1)
						{
							count++;
							sum += probeAvg[i + 1][j];
						}
						/*
						if (probeAvg[i + 1][(j + 1) % settings.m_probeCountPerLayer] != -1)
						{
							count++;
							sum += probeAvg[i + 1][(j + 1) % settings.m_probeCountPerLayer];
						}
						*/
					}
					if (count != 0) probeAvg[i][j] = sum / count;
					else probeAvg[i][j] = 0;
				}
			}
		}
	}

	float loss = 0.0f;
	float ratio = 1.0f;
	float ratioSum = 1.0f;
	for (int i = 0; i < settings.m_probeLayerAmount; i++)
	{
		ratioSum += ratio;
		for (int j = 0; j < settings.m_probeCountPerLayer; j++)
		{
			loss += ratio * glm::clamp(settings.m_distanceLossMagnitude * glm::pow(probeAvg[i][j], settings.m_distanceLossFactor), 0.0f, 1.0f);
		}
		ratio *= settings.m_probeMinMaxRatio;
	}
	loss = loss / ratioSum / settings.m_probeCountPerLayer;
	return glm::clamp(1.0f - loss, 0.0f, 1.0f);
}

bool TreeModel::ElongateRoot(const glm::mat4& globalTransform, SoilModel& soilModel, const float extendLength, NodeHandle rootNodeHandle, const RootGrowthParameters& rootGrowthParameters,
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
		if (soilModel.GetDensity((globalTransform * glm::translate(oldRootNode.m_info.m_globalPosition))[3]) == 0.0f) {
			ApplyTropism(m_currentGravityDirection, 0.1f, desiredGlobalFront,
				desiredGlobalUp);
		}

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
			ElongateRoot(globalTransform, soilModel, extraLength - rootNodeLength, newRootNodeHandle, rootGrowthParameters, childAuxin);
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

inline bool TreeModel::GrowRootNode(const glm::mat4& globalTransform, SoilModel& soilModel, NodeHandle rootNodeHandle, const RootGrowthParameters& rootGrowthParameters)
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
			const float extendLength = rootNode.m_data.m_reproductiveWaterRequirement * rootGrowthParameters.m_rootNodeLength * m_globalGrowthRate;
			const auto dd = rootGrowthParameters.GetAuxinTransportLoss(rootNode);
			ElongateRoot(globalTransform, soilModel, extendLength, rootNodeHandle, rootGrowthParameters, collectedAuxin);
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

void TreeModel::CollectResourceRequirement(NodeHandle internodeHandle)
{
	auto& internode = m_branchSkeleton.RefNode(internodeHandle);
	auto& internodeData = internode.m_data;
	auto& internodeInfo = internode.m_info;
	internodeData.m_lightIntensity = m_treeVolume.IlluminationEstimation(internodeInfo.m_globalPosition, m_illuminationEstimationSettings);
	internodeInfo.m_color = glm::mix(glm::vec4(0.0, 0.0, 0.0, 1.0), glm::vec4(1.0f), internodeData.m_lightIntensity);
	if (!internode.IsEndNode()) {
		//If current node is not end node
		for (const auto& i : internode.RefChildHandles()) {
			auto& childInternode = m_branchSkeleton.RefNode(i);
			internodeData.m_descendentReproductionWaterRequirement += childInternode.m_data.
				m_reproductionWaterRequirement + childInternode.m_data.m_descendentReproductionWaterRequirement;
		}
	}
}


bool TreeModel::ElongateInternode(const glm::mat4& globalTransform, float extendLength, NodeHandle internodeHandle,
	const TreeGrowthParameters& parameters, float& collectedInhibitor) {
	bool graphChanged = false;
	auto& internode = m_branchSkeleton.RefNode(internodeHandle);
	auto internodeLength = parameters.m_internodeLength;
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
		auto newInternodeHandle = m_branchSkeleton.Extend(internodeHandle, false);
		auto& oldInternode = m_branchSkeleton.RefNode(internodeHandle);
		auto& newInternode = m_branchSkeleton.RefNode(newInternodeHandle);
		newInternode.m_data.Clear();
		newInternode.m_data.m_inhibitorTarget = newInternode.m_data.m_inhibitor = parameters.m_apicalDominance;
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
			ElongateInternode(globalTransform, extraLength - internodeLength, newInternodeHandle, parameters, childInhibitor);
			childInhibitor *= parameters.m_apicalDominanceDistanceFactor;
			collectedInhibitor += childInhibitor;
			m_branchSkeleton.RefNode(newInternodeHandle).m_data.m_inhibitorTarget = childInhibitor;
		}
		else {
			collectedInhibitor += parameters.m_apicalDominance;
		}
	}
	else {
		//Otherwise, we add the inhibitor.
		collectedInhibitor += parameters.m_apicalDominance;
	}
	return graphChanged;
}

void TreeModel::CollectLuminousFluxFromLeaves(ClimateModel& climateModel,
	const TreeGrowthParameters& treeGrowthParameters)
{
	m_treeGrowthNutrients.m_luminousFlux = 0.0f;
	const auto& sortedInternodeList = m_branchSkeleton.RefSortedNodeList();
	for (const auto& internodeHandle : sortedInternodeList) {
		auto& internode = m_branchSkeleton.RefNode(internodeHandle);
		for (const auto& bud : internode.m_data.m_buds)
		{
			if (bud.m_status == BudStatus::Flushed && bud.m_type == BudType::Leaf)
			{
				m_treeGrowthNutrients.m_luminousFlux += bud.m_luminousFlux;
			}
		}
	}
	m_treeGrowthNutrients.m_luminousFlux = m_treeGrowthNutrientsRequirement.m_luminousFlux;
}

bool TreeModel::GrowInternode(const glm::mat4& globalTransform, ClimateModel& climateModel, NodeHandle internodeHandle, const TreeGrowthParameters& treeGrowthParameters) {
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

		auto killProbability = treeGrowthParameters.m_growthRate * treeGrowthParameters.m_budKillProbability;
		if (internodeData.m_rootDistance < 1.0f) killProbability = 0.0f;
		if (bud.m_status == BudStatus::Dormant && killProbability > glm::linearRand(0.0f, 1.0f)) {
			bud.m_status = BudStatus::Removed;
		}
		if ((bud.m_type == BudType::Leaf || bud.m_type == BudType::Fruit) && internodeInfo.m_thickness > treeGrowthParameters.m_trunkRadius)
		{
			bud.m_status = BudStatus::Removed;
		}
		if (bud.m_status == BudStatus::Removed) continue;

		const float baseWater = glm::clamp(bud.m_waterGain, 0.0f, bud.m_baseWaterRequirement);
		bud.m_drought = glm::clamp(1.0f - (1.0f - bud.m_drought) * baseWater / bud.m_baseWaterRequirement, 0.0f, 1.0f);
		const float reproductiveWater = glm::max(0.0f, bud.m_waterGain - bud.m_baseWaterRequirement);
		const float reproductiveContent = reproductiveWater * m_globalGrowthRate;

		if (bud.m_type == BudType::Apical && bud.m_status == BudStatus::Dormant) {
			const float elongateLength = reproductiveContent * treeGrowthParameters.m_internodeLength;
			float collectedInhibitor = 0.0f;
			const auto dd = treeGrowthParameters.m_apicalDominanceDistanceFactor;
			graphChanged = ElongateInternode(globalTransform, elongateLength, internodeHandle, treeGrowthParameters, collectedInhibitor);
			m_branchSkeleton.RefNode(internodeHandle).m_data.m_inhibitorTarget += collectedInhibitor * dd;
			//If apical bud is dormant, then there's no lateral bud at this stage. We should quit anyway.
			break;
		}
		auto temperature = climateModel.GetTemperature(globalTransform * glm::translate(internodeInfo.m_globalPosition)[3]);
		if (bud.m_type == BudType::Lateral && bud.m_status == BudStatus::Dormant) {
			const auto& probabilityRange = treeGrowthParameters.m_lateralBudFlushingProbabilityTemperatureRange;
			float flushProbability = treeGrowthParameters.m_growthRate * glm::mix(probabilityRange.x, probabilityRange.y,
				glm::clamp((temperature - probabilityRange.z) / (probabilityRange.w - probabilityRange.z), 0.0f, 1.0f));
			flushProbability /= 1.0f + internodeData.m_inhibitor;
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
				newInternode.m_data.Clear();
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
					glm::clamp((temperature - probabilityRange.z) / (probabilityRange.w - probabilityRange.z), 0.0f, 1.0f));
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
					glm::clamp((temperature - probabilityRange.z) / (probabilityRange.w - probabilityRange.z), 0.0f, 1.0f));
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

				if (climateModel.m_days % 360 > 180 && temperature < treeGrowthParameters.m_leafChlorophyllSynthesisFactorTemperature)
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

inline void TreeModel::AdjustProductiveResourceRequirement(NodeHandle internodeHandle,
	const TreeGrowthParameters& treeGrowthParameters)
{
	auto& internode = m_branchSkeleton.RefNode(internodeHandle);
	auto& internodeData = internode.m_data;
	if (internode.GetParentHandle() == -1) {
		//Total water collected from root applied to here.
		internodeData.m_adjustedTotalReproductionWaterRequirement =
			internodeData.m_reproductionWaterRequirement +
			internodeData.m_descendentReproductionWaterRequirement;
		internodeData.m_adjustedDescendentReproductionWaterRequirement = internodeData.m_descendentReproductionWaterRequirement;
		for (auto& bud : internodeData.m_buds) {
			bud.m_waterGain = (bud.m_reproductionWaterRequirement + bud.m_baseWaterRequirement) * m_treeGrowthNutrients.m_water / m_treeGrowthNutrientsRequirement.m_water;
		}
	}
	const float apicalControl = treeGrowthParameters.GetApicalControl(internode);
	float totalChildResourceRequirement = 0.0f;

	for (const auto& i : internode.RefChildHandles()) {
		auto& childInternode = m_branchSkeleton.RefNode(i);
		auto& childInternodeData = childInternode.m_data;

		childInternodeData.m_adjustedTotalReproductionWaterRequirement = 0;
		//If current internode has resources to distribute to child
		if (internodeData.m_adjustedDescendentReproductionWaterRequirement != 0) {
			childInternodeData.m_adjustedTotalReproductionWaterRequirement =
				(childInternodeData.m_descendentReproductionWaterRequirement +
					childInternodeData.m_reproductionWaterRequirement) /
				internodeData.m_adjustedDescendentReproductionWaterRequirement;
			childInternodeData.m_adjustedTotalReproductionWaterRequirement = glm::pow(
				childInternodeData.m_adjustedTotalReproductionWaterRequirement, apicalControl);
			totalChildResourceRequirement += childInternodeData.m_adjustedTotalReproductionWaterRequirement;
		}
	}
	for (const auto& i : internode.RefChildHandles()) {
		auto& childInternode = m_branchSkeleton.RefNode(i);
		auto& childInternodeData = childInternode.m_data;
		if (internodeData.m_adjustedDescendentReproductionWaterRequirement != 0) {
			childInternodeData.m_adjustedTotalReproductionWaterRequirement *=
				internodeData.m_adjustedDescendentReproductionWaterRequirement /
				totalChildResourceRequirement;
			const float resourceFactor = childInternodeData.m_adjustedTotalReproductionWaterRequirement /
				(childInternodeData.m_descendentReproductionWaterRequirement +
					childInternodeData.m_reproductionWaterRequirement);
			childInternodeData.m_adjustedDescendentReproductionWaterRequirement =
				childInternodeData.m_descendentReproductionWaterRequirement * resourceFactor;
			for (auto& bud : childInternodeData.m_buds) {
				const float adjustedReproductionWaterRequirement =
					bud.m_reproductionWaterRequirement * resourceFactor;
				bud.m_waterGain = (adjustedReproductionWaterRequirement + bud.m_baseWaterRequirement) * m_treeGrowthNutrients.m_water / m_treeGrowthNutrientsRequirement.m_water;
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
			1.0f / treeGrowthParameters.m_thicknessAccumulateFactor);
	}
	internodeData.m_maxDistanceToAnyBranchEnd = maxDistanceToAnyBranchEnd;
	if (childThicknessCollection != 0.0f) {
		internodeInfo.m_thickness = glm::max(internodeInfo.m_thickness, glm::pow(childThicknessCollection,
			treeGrowthParameters.m_thicknessAccumulateFactor));
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

void TreeModel::CalculateResourceRequirement(NodeHandle internodeHandle,
	const TreeGrowthParameters& treeGrowthParameters, TreeGrowthNutrients& newTreeGrowthNutrientsRequirement) {
	auto& internode = m_branchSkeleton.RefNode(internodeHandle);
	auto& internodeData = internode.m_data;
	internodeData.m_reproductionWaterRequirement = 0.0f;
	internodeData.m_descendentReproductionWaterRequirement = 0.0f;
	const auto growthRate = treeGrowthParameters.m_growthRate;
	for (auto& bud : internodeData.m_buds) {
		if (bud.m_status == BudStatus::Died || bud.m_status == BudStatus::Removed) {
			bud.m_baseWaterRequirement = 0.0f;
			bud.m_reproductionWaterRequirement = 0.0f;
			continue;
		}
		switch (bud.m_type) {
		case BudType::Apical: {
			bud.m_baseWaterRequirement = treeGrowthParameters.m_shootBaseWaterRequirement;
			if (bud.m_status == BudStatus::Dormant) {
				//Elongation
				bud.m_reproductionWaterRequirement = treeGrowthParameters.m_shootProductiveWaterRequirement;
			}
		}break;
		case BudType::Leaf: {
			bud.m_baseWaterRequirement = treeGrowthParameters.m_leafBaseWaterRequirement;
			bud.m_reproductionWaterRequirement = treeGrowthParameters.m_leafProductiveWaterRequirement;
		}break;
		case BudType::Fruit: {
			bud.m_baseWaterRequirement = treeGrowthParameters.m_fruitBaseWaterRequirement;
			bud.m_reproductionWaterRequirement = treeGrowthParameters.m_fruitProductiveWaterRequirement;
		}break;
		case BudType::Lateral: {
			bud.m_baseWaterRequirement = 0.0f;
			bud.m_reproductionWaterRequirement = 0.0f;
		}break;
		}
		bud.m_reproductionWaterRequirement = growthRate * bud.m_reproductionWaterRequirement;
		internodeData.m_reproductionWaterRequirement += bud.m_reproductionWaterRequirement;
		newTreeGrowthNutrientsRequirement.m_water += bud.m_reproductionWaterRequirement + bud.m_baseWaterRequirement;
		newTreeGrowthNutrientsRequirement.m_luminousFlux += bud.m_reproductionWaterRequirement;
	}
}

bool TreeModel::GrowBranches(const glm::mat4& globalTransform, ClimateModel& climateModel, const TreeGrowthParameters& treeGrowthParameters, TreeGrowthNutrients& newTreeGrowthNutrientsRequirement) {
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
			CollectResourceRequirement(*it);
		}
		for (const auto& internodeHandle : sortedInternodeList) {
			AdjustProductiveResourceRequirement(internodeHandle, treeGrowthParameters);
		}
		for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); it++) {
			const bool graphChanged = GrowInternode(globalTransform, climateModel, *it, treeGrowthParameters);
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

			CalculateResourceRequirement(internodeHandle, treeGrowthParameters, newTreeGrowthNutrientsRequirement);
		}

		m_treeVolume.Clear();
		m_treeVolume.m_maxHeight = m_branchSkeleton.m_max.y - m_branchSkeleton.m_min.y;
		for (const auto& internodeHandle : sortedInternodeList)
		{
			auto& internode = m_branchSkeleton.RefNode(internodeHandle);
			auto& internodeInfo = internode.m_info;
			const auto& point1 = internodeInfo.m_globalPosition;
			{
				const auto sliceIndex = m_treeVolume.SelectSlice(point1);
				const float currentDistance =
					glm::length(glm::vec2(point1.x, point1.z));
				auto& slice = m_treeVolume.m_layers[sliceIndex.x][sliceIndex.y];
				if (slice <
					currentDistance + m_treeVolume.m_offset)
					slice = currentDistance + m_treeVolume.m_offset;
			}
			{
				auto point2 = point1 + internodeInfo.m_length * (internodeInfo.m_globalRotation * glm::vec3(0, 0, -1));
				const auto sliceIndex = m_treeVolume.SelectSlice(point2);
				const float currentDistance =
					glm::length(glm::vec2(point2.x, point2.z));
				auto& slice = m_treeVolume.m_layers[sliceIndex.x][sliceIndex.y];
				if (slice <
					currentDistance + m_treeVolume.m_offset)
					slice = currentDistance + m_treeVolume.m_offset;
			}
		}
		m_treeVolume.Smooth();
		m_treeVolume.m_hasData = true;
	};

	if (m_enableBranchCollisionDetection)
	{
		const float minRadius = treeGrowthParameters.m_endNodeThickness * 4.0f;
		const auto boxSize = m_branchSkeleton.m_max - m_branchSkeleton.m_min;
		const float maxRadius = glm::max(glm::max(boxSize.x, boxSize.y), boxSize.z) + 2.0f * minRadius;
		int subdivisionLevel = 0;
		float testRadius = minRadius;
		while (testRadius <= maxRadius)
		{
			subdivisionLevel++;
			testRadius *= 2.f;
		}
		m_branchOctree.Reset(maxRadius, subdivisionLevel, (m_branchSkeleton.m_min + m_branchSkeleton.m_max) * 0.5f);
		const auto& sortedBranchNodeList = m_branchSkeleton.RefSortedNodeList();
		const auto& sortedBranchFlowList = m_branchSkeleton.RefSortedFlowList();
		int collisionCount = 0;
		int flowCollisionCount = 0;
		std::unordered_map<int, int> nodeCollisionCollection;
		std::unordered_map<int, int> flowCollisionCollection;
		for (const auto& nodeHandle : sortedBranchNodeList)
		{
			const auto& node = m_branchSkeleton.RefNode(nodeHandle);
			const auto& info = node.m_info;
			m_branchOctree.Occupy(info.m_globalPosition, info.m_globalRotation, info.m_length, info.m_thickness, [&](OctreeNode<TreeVoxelData>& octreeNode)
				{
					if (octreeNode.m_data.m_nodeHandle == nodeHandle) return;
			if (octreeNode.m_data.m_nodeHandle == node.GetParentHandle()) return;
			for (const auto& i : node.RefChildHandles()) if (octreeNode.m_data.m_nodeHandle == i) return;
			auto flowHandle = node.GetFlowHandle();
			if (octreeNode.m_data.m_referenceCount != 0)
			{
				if (octreeNode.m_data.m_nodeHandle > nodeHandle)
				{
					nodeCollisionCollection[octreeNode.m_data.m_nodeHandle] = nodeHandle;
				}
				else
				{
					nodeCollisionCollection[nodeHandle] = octreeNode.m_data.m_nodeHandle;
				}
				if (octreeNode.m_data.m_flowHandle != flowHandle) {
					if (octreeNode.m_data.m_flowHandle > flowHandle)
					{
						flowCollisionCollection[octreeNode.m_data.m_flowHandle] = flowHandle;
					}
					else
					{
						flowCollisionCollection[flowHandle] = octreeNode.m_data.m_flowHandle;
					}
				}
			}
			else {
				octreeNode.m_data.m_flowHandle = flowHandle;
				octreeNode.m_data.m_nodeHandle = nodeHandle;
			}
			octreeNode.m_data.m_referenceCount++;
				});
		}
		collisionCount = nodeCollisionCollection.size();
		flowCollisionCount = flowCollisionCollection.size();
		std::vector<int> collisionStat;
		collisionStat.resize(200);
		for (auto& i : collisionStat) i = 0;
		int totalVoxel = 0;
		m_branchOctree.IterateLeaves([&](const OctreeNode<TreeVoxelData>& leaf)
			{
				collisionStat[leaf.m_data.m_referenceCount]++;
		totalVoxel++;
			});

		std::string report = "Branch collision: [" + std::to_string(collisionCount) + "/" + std::to_string(sortedBranchNodeList.size()) + "], [" + std::to_string(flowCollisionCount) + "/" + std::to_string(sortedBranchFlowList.size()) + "], ";
		report += "total occupied: " + std::to_string(totalVoxel) + ", collision stat: ";

		std::string appendStat = "";
		for (int i = 199; i > 0; i--)
		{
			if (collisionStat[i] != 0)
			{
				appendStat += "[" + std::to_string(i) + "]->" + std::to_string(collisionStat[i]) + "; ";
			}
		}
		if (appendStat == "") appendStat = "No collision";

		UNIENGINE_LOG(report + appendStat);
	}

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
		firstRootNode.m_data.m_verticalTropism = rootGrowthParameters.GetTropismIntensity(firstRootNode);
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


bool TreeModel::LowBranchPruning(float maxDistance, NodeHandle internodeHandle,
	const TreeGrowthParameters& treeGrowthParameters) {
	auto& internode = m_branchSkeleton.RefNode(internodeHandle);
	//Pruning here.
	if (maxDistance > 5 && internode.m_data.m_order != 0 &&
		internode.m_data.m_rootDistance / maxDistance < treeGrowthParameters.m_lowBranchPruning) {
		m_branchSkeleton.RecycleNode(internodeHandle);
		return true;
	}

	return false;
}

bool TreeModel::GrowRoots(const glm::mat4& globalTransform, SoilModel& soilModel, const RootGrowthParameters& rootGrowthParameters, TreeGrowthNutrients& newTreeGrowthNutrientsRequirement)
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
				const bool graphChanged = GrowRootNode(globalTransform, soilModel, *it, rootGrowthParameters);
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
			const float growthRate = rootGrowthParameters.GetExpectedGrowthRate();
			const auto& sortedRootNodeList = m_rootSkeleton.RefSortedNodeList();
			for (auto it = sortedRootNodeList.rbegin(); it != sortedRootNodeList.rend(); it++) {
				CalculateThickness(*it, rootGrowthParameters);
			}
			float totalWaterRequirement = 0;
			float totalLuminousFluxRequirement = 0;
			for (const auto& rootNodeHandle : sortedRootNodeList) {
				auto& rootNode = m_rootSkeleton.RefNode(rootNodeHandle);
				auto& rootNodeData = rootNode.m_data;
				auto& rootNodeInfo = rootNode.m_info;
				rootNode.m_data.m_reproductiveWaterRequirement = /*soilModel.GetNutrient(pos[3])*/1.0f * growthRate;
				totalWaterRequirement += rootNodeData.m_reproductiveWaterRequirement;
				totalLuminousFluxRequirement += rootNodeData.m_reproductiveWaterRequirement;
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
			newTreeGrowthNutrientsRequirement.m_water += totalWaterRequirement * totalWaterRequirement;
			newTreeGrowthNutrientsRequirement.m_luminousFlux += totalWaterRequirement * totalLuminousFluxRequirement;
		};

		if (m_enableRootCollisionDetection)
		{
			const float minRadius = rootGrowthParameters.m_endNodeThicknessAndControl.x * 4.0f;
			const auto boxSize = m_rootSkeleton.m_max - m_rootSkeleton.m_min;
			const float maxRadius = glm::max(glm::max(boxSize.x, boxSize.y), boxSize.z) + 2.0f * minRadius;
			int subdivisionLevel = 0;
			float testRadius = minRadius;
			while (testRadius <= maxRadius)
			{
				subdivisionLevel++;
				testRadius *= 2.f;
			}
			m_rootOctree.Reset(maxRadius, subdivisionLevel, (m_rootSkeleton.m_min + m_rootSkeleton.m_max) * 0.5f);
			const auto& sortedRootNodeList = m_rootSkeleton.RefSortedNodeList();
			const auto& sortedRootFlowList = m_rootSkeleton.RefSortedFlowList();
			int collisionCount = 0;
			int flowCollisionCount = 0;
			std::unordered_map<int, int> nodeCollisionCollection;
			std::unordered_map<int, int> flowCollisionCollection;
			for (const auto& nodeHandle : sortedRootNodeList)
			{
				const auto& node = m_rootSkeleton.RefNode(nodeHandle);
				const auto& info = node.m_info;
				m_rootOctree.Occupy(info.m_globalPosition, info.m_globalRotation, info.m_length * 0.9f, info.m_thickness, [&](OctreeNode<TreeVoxelData>& octreeNode)
					{
						if (octreeNode.m_data.m_nodeHandle == nodeHandle) return;
				if (octreeNode.m_data.m_nodeHandle == node.GetParentHandle()) return;
				for (const auto& i : node.RefChildHandles()) if (octreeNode.m_data.m_nodeHandle == i) return;
				auto flowHandle = node.GetFlowHandle();
				if (octreeNode.m_data.m_referenceCount != 0)
				{
					if (octreeNode.m_data.m_nodeHandle > nodeHandle)
					{
						nodeCollisionCollection[octreeNode.m_data.m_nodeHandle] = nodeHandle;
					}
					else
					{
						nodeCollisionCollection[nodeHandle] = octreeNode.m_data.m_nodeHandle;
					}
					if (octreeNode.m_data.m_flowHandle != flowHandle) {
						if (octreeNode.m_data.m_flowHandle > flowHandle)
						{
							flowCollisionCollection[octreeNode.m_data.m_flowHandle] = flowHandle;
						}
						else
						{
							flowCollisionCollection[flowHandle] = octreeNode.m_data.m_flowHandle;
						}
					}
				}
				else {
					octreeNode.m_data.m_flowHandle = flowHandle;
					octreeNode.m_data.m_nodeHandle = nodeHandle;
				}
				octreeNode.m_data.m_referenceCount++;
					});

			}
			collisionCount = nodeCollisionCollection.size();
			flowCollisionCount = flowCollisionCollection.size();
			std::vector<int> collisionStat;
			collisionStat.resize(200);
			for (auto& i : collisionStat) i = 0;
			int totalVoxel = 0;
			m_rootOctree.IterateLeaves([&](const OctreeNode<TreeVoxelData>& leaf)
				{
					collisionStat[leaf.m_data.m_referenceCount]++;
			totalVoxel++;
				});

			std::string report = "Root collision: [" + std::to_string(collisionCount) + "/" + std::to_string(sortedRootNodeList.size()) + "], [" + std::to_string(flowCollisionCount) + "/" + std::to_string(sortedRootFlowList.size()) + "], ";
			report += "total occupied: " + std::to_string(totalVoxel) + ", collision stat: ";

			std::string appendStat;
			for (int i = 199; i > 0; i--)
			{
				if (collisionStat[i] != 0)
				{
					appendStat += "[" + std::to_string(i) + "]->" + std::to_string(collisionStat[i]) + "; ";
				}
			}
			if (appendStat.empty()) appendStat = "No collision";

			UNIENGINE_LOG(report + appendStat);
		}

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

void TreeModel::CollectWaterFromRoots(const glm::mat4& globalTransform, SoilModel& soilModel, const RootGrowthParameters& rootGrowthParameters)
{
	m_treeGrowthNutrients.m_water = 0.0f;
	const auto& sortedRootNodeList = m_rootSkeleton.RefSortedNodeList();
	for (const auto& rootNodeHandle : sortedRootNodeList) {
		auto& rootNode = m_rootSkeleton.RefNode(rootNodeHandle);
		m_treeGrowthNutrients.m_water += 1.0f;//soilModel.GetWater(rootNode.m_info.m_globalPosition);
	}
	m_treeGrowthNutrients.m_water = m_treeGrowthNutrientsRequirement.m_water;
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
	//Set target carbohydrate.
	m_treeGrowthNutrientsRequirement.m_carbohydrate = m_treeGrowthNutrientsRequirement.m_luminousFlux;
	//Collect water from roots.
	CollectWaterFromRoots(globalTransform, soilModel, rootGrowthParameters);
	//Collect light from branches.
	CollectLuminousFluxFromLeaves(climateModel, treeGrowthParameters);
	//Perform photosynthesis.
	m_treeGrowthNutrients.m_carbohydrate =
		glm::min(
			glm::max(m_treeGrowthNutrients.m_water, m_treeGrowthNutrientsRequirement.m_water),
			glm::max(m_treeGrowthNutrients.m_luminousFlux, m_treeGrowthNutrientsRequirement.m_luminousFlux));
	//Calculate global growth rate
	if (m_treeGrowthNutrientsRequirement.m_carbohydrate != 0.0f) {
		m_globalGrowthRate = m_treeGrowthNutrients.m_carbohydrate / m_treeGrowthNutrientsRequirement.m_carbohydrate;
	}
	else { m_globalGrowthRate = 0.0f; }
	m_globalGrowthRate = glm::clamp(m_globalGrowthRate, 0.0f, 1.0f);

	//Grow roots and set up nutrient requirements for next iteration.
	TreeGrowthNutrients newTreeNutrientsRequirement;
	if (GrowRoots(globalTransform, soilModel, rootGrowthParameters, newTreeNutrientsRequirement)) {
		rootStructureChanged = true;
	}
	//Grow branches and set up nutrient requirements for next iteration.
	if (GrowBranches(globalTransform, climateModel, treeGrowthParameters, newTreeNutrientsRequirement)) {
		treeStructureChanged = true;
	}
	//Set new growth nutrients requirement for next iteration.
	m_treeGrowthNutrientsRequirement = newTreeNutrientsRequirement;
	return treeStructureChanged || rootStructureChanged;
}

void TreeModel::CalculateIllumination()
{
	const auto& sortedInternodeList = m_branchSkeleton.RefSortedNodeList();
	for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); it++) {
		auto& internode = m_branchSkeleton.RefNode(*it);
		auto& internodeData = internode.m_data;
		auto& internodeInfo = internode.m_info;
		internodeData.m_lightIntensity = m_treeVolume.IlluminationEstimation(internodeInfo.m_globalPosition, m_illuminationEstimationSettings);
		internodeInfo.m_color = glm::mix(glm::vec4(0.0, 0.0, 0.0, 1.0), glm::vec4(1.0f), internodeData.m_lightIntensity);
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

void InternodeGrowthData::Clear() {
	m_age = 0;
	m_inhibitor = 0;
	m_desiredLocalRotation = glm::vec3(0.0f);
	m_sagging = 0;

	m_maxDistanceToAnyBranchEnd = 0;
	m_level = 0;
	m_descendentTotalBiomass = 0;

	m_rootDistance = 0;

	m_reproductionWaterRequirement = 0.0f;
	m_descendentReproductionWaterRequirement = 0.0f;
	m_adjustedTotalReproductionWaterRequirement = 0.0f;
	m_lightDirection = glm::vec3(0, 1, 0);
	m_lightIntensity = 1.0f;
	m_buds.clear();
}
RootGrowthParameters::RootGrowthParameters()
{
	m_growthRate = 0.04f;
	m_rootNodeLength = 0.03f;
	m_endNodeThicknessAndControl = glm::vec2(0.002, 0.5);
	m_thicknessLengthAccumulate = 0.0002f;
	m_branchingAngleMeanVariance = glm::vec2(60, 3);
	m_rollAngleMeanVariance = glm::vec2(120, 2);
	m_apicalAngleMeanVariance = glm::vec2(0, 3);
	m_auxinTransportLoss = 1.0f;
	m_tropismSwitchingProbability = 1.0f;
	m_tropismSwitchingProbabilityDistanceFactor = 0.99f;
	m_tropismIntensity = 0.1f;

	m_baseBranchingProbability = 0.4f;
	m_branchingProbabilityChildrenDecrease = 0.01f;
	m_branchingProbabilityDistanceDecrease = 0.98f;
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
	m_thicknessAccumulateFactor = 0.5f;
	m_trunkRadius = 0.02f;

	//Bud
	m_branchingAngleMeanVariance = glm::vec2(60, 3);
	m_rollAngleMeanVariance = glm::vec2(120, 2);
	m_apicalAngleMeanVariance = glm::vec2(0, 2.5);
	m_gravitropism = 0.03f;
	m_phototropism = 0.0f;

	m_lateralBudFlushingProbabilityTemperatureRange = glm::vec4(1.0f, 1.0f, 0.0f, 100.0f);
	m_leafBudFlushingProbabilityTemperatureRange = glm::vec4(0.0f, 1.0f, 45.0f, 60.0f);
	m_fruitBudFlushingProbabilityTemperatureRange = glm::vec4(0.0f, 1.0f, 50.0f, 70.0f);

	m_apicalControlBaseDistFactor = { 1.1f, 0.99f };
	m_apicalDominance = 300;
	m_apicalDominanceDistanceFactor = 0.97f;
	m_budKillProbability = 0.00f;

	m_shootBaseWaterRequirement = 1.0f;
	m_leafBaseWaterRequirement = 1.0f;
	m_fruitBaseWaterRequirement = 1.0f;
	m_shootProductiveWaterRequirement = 1.0f;
	m_leafProductiveWaterRequirement = 1.0f;
	m_fruitProductiveWaterRequirement = 1.0f;



	//Internode
	m_lowBranchPruning = 0.2f;
	m_saggingFactorThicknessReductionMax = glm::vec3(0.0001, 2, 0.5);

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

float TreeGrowthParameters::GetApicalControl(const Node<InternodeGrowthData>& internode) const {
	return glm::pow(m_apicalControlBaseDistFactor.x, glm::max(1.0f, 1.0f /
		internode.m_data.m_rootDistance *
		m_apicalControlBaseDistFactor.y));
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
float RootGrowthParameters::GetExpectedGrowthRate() const
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
	return m_thicknessLengthAccumulate * m_endNodeThicknessAndControl.x;
}

float RootGrowthParameters::GetBranchingProbability(const Node<RootInternodeGrowthData>& rootNode) const
{
	return m_baseBranchingProbability * m_growthRate *
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
