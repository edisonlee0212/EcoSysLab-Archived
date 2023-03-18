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
	for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); it++) {
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

bool TreeModel::IsPipeEnabled() const
{
	return m_enablePipe;
}

void TreeModel::PruneInternode(NodeHandle internodeHandle)
{
	if (m_enablePipe) {
		auto& pipeGroup = m_shootSkeleton.m_data.m_shootPipeGroup;
		auto& gridGroup = m_shootSkeleton.m_data.m_hexagonGridGroup;
		auto& node = m_shootSkeleton.RefNode(internodeHandle);
		auto& nodeData = node.m_data;
		for (const auto& i : nodeData.m_pipeNodeHandles)
		{
			pipeGroup.RecyclePipeNode(i);
		}
		nodeData.m_pipeNodeHandles.clear();
		const auto baseGridHandle = m_shootSkeleton.PeekNode(node.GetParentHandle()).m_data.m_gridHandle;
		m_shootSkeleton.RecycleNode(internodeHandle,
			[&](FlowHandle flowHandle) {},
			[&](NodeHandle nodeHandle)
			{
				if (nodeHandle == internodeHandle) return;
		const auto& currentNode = m_shootSkeleton.PeekNode(nodeHandle);
		if (currentNode.m_data.m_gridHandle == baseGridHandle) return;
		if (!gridGroup.PeekGrid(currentNode.m_data.m_gridHandle).IsRecycled()) gridGroup.RecycleGrid(currentNode.m_data.m_gridHandle);
			});
	}
	else {
		m_shootSkeleton.RecycleNode(internodeHandle,
			[&](FlowHandle flowHandle) {},
			[&](NodeHandle nodeHandle)
			{});
	}
}

void TreeModel::PruneRootNode(NodeHandle rootNodeHandle)
{
	if (m_enablePipe) {
		auto& pipeGroup = m_rootSkeleton.m_data.m_rootPipeGroup;
		auto& gridGroup = m_rootSkeleton.m_data.m_hexagonGridGroup;
		auto& node = m_rootSkeleton.RefNode(rootNodeHandle);
		auto& nodeData = node.m_data;
		for (const auto& i : nodeData.m_pipeNodeHandles)
		{
			pipeGroup.RecyclePipeNode(i);
		}
		nodeData.m_pipeNodeHandles.clear();
		const auto baseGridHandle = m_rootSkeleton.PeekNode(node.GetParentHandle()).m_data.m_gridHandle;
		m_rootSkeleton.RecycleNode(rootNodeHandle,
			[&](FlowHandle flowHandle) {},
			[&](NodeHandle nodeHandle)
			{
				if (nodeHandle == rootNodeHandle) return;
		const auto& currentNode = m_rootSkeleton.PeekNode(nodeHandle);
		if (currentNode.m_data.m_gridHandle == baseGridHandle) return;
		if (!gridGroup.PeekGrid(currentNode.m_data.m_gridHandle).IsRecycled()) gridGroup.RecycleGrid(currentNode.m_data.m_gridHandle);
			});
	}
	else {
		m_rootSkeleton.RecycleNode(rootNodeHandle,
			[&](FlowHandle flowHandle) {},
			[&](NodeHandle nodeHandle)
			{});
	}
}

void TreeModel::HarvestFruits(const std::function<bool(const ReproductiveModule& fruit)>& harvestFunction)
{
	const auto& sortedInternodeList = m_shootSkeleton.RefSortedNodeList();
	m_fruitCount = 0;

	for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); it++) {
		auto& internode = m_shootSkeleton.RefNode(*it);
		auto& internodeData = internode.m_data;
		auto& buds = internodeData.m_buds;
		for (auto& bud : buds)
		{
			if (bud.m_type != BudType::Fruit || bud.m_status != BudStatus::Flushed) continue;

			if (harvestFunction(bud.m_reproductiveModule)) {
				bud.m_reproductiveModule.Reset();
				bud.m_status = BudStatus::Died;
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

bool TreeModel::Grow(float deltaTime, const glm::mat4& globalTransform, SoilModel& soilModel, ClimateModel& climateModel,
	const RootGrowthParameters& rootGrowthParameters, const ShootGrowthParameters& shootGrowthParameters)
{

	m_currentDeltaTime = deltaTime;

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
	if (m_currentDeltaTime != 0.0f
		&& GrowRoots(globalTransform, soilModel, rootGrowthParameters, newRootGrowthRequirement)) {
		rootStructureChanged = true;
	}
	//Grow branches and set up nutrient requirements for next iteration.
	if (m_currentDeltaTime != 0.0f
		&& GrowShoots(globalTransform, climateModel, shootGrowthParameters, newShootGrowthRequirement)) {
		treeStructureChanged = true;
	}
	const int year = climateModel.m_time;
	if (year != m_ageInYear)
	{
		ResetReproductiveModule();
		m_ageInYear = year;
	}
	//Set new growth nutrients requirement for next iteration.
	m_shootSkeleton.m_data.m_vigorRequirement = newShootGrowthRequirement;
	m_rootSkeleton.m_data.m_vigorRequirement = newRootGrowthRequirement;
	m_iteration++;
	m_age += m_currentDeltaTime;
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
		apicalBud.m_vigorSink.AddVigor(shootGrowthParameters.m_internodeVigorRequirement);
		apicalBud.m_localRotation = glm::vec3(glm::radians(shootGrowthParameters.GetDesiredApicalAngle(firstInternode)),
			0.0f,
			shootGrowthParameters.GetDesiredRollAngle(firstInternode));

		if (m_enablePipe)
		{
			auto& pipeGroup = m_shootSkeleton.m_data.m_shootPipeGroup;
			pipeGroup = {};
			const auto& newPipeHandle = pipeGroup.AllocatePipe();
			const auto newPipeNodeHandle = pipeGroup.Extend(newPipeHandle);
			firstInternode.m_data.m_pipeNodeHandles.emplace_back(newPipeNodeHandle);
			auto& newPipeNode = pipeGroup.RefPipeNode(newPipeNodeHandle);
			newPipeNode.m_data.m_nodeHandle = firstInternode.GetHandle();

			auto& gridGroup = m_shootSkeleton.m_data.m_hexagonGridGroup;
			const auto newGridHandle = gridGroup.Allocate();
			auto& newGrid = gridGroup.RefGrid(newGridHandle);
			newGrid.m_data.m_nodeHandle = firstInternode.GetHandle();
			const auto newCellHandle = newGrid.Allocate(glm::ivec2(0, 0));
			auto& newCell = newGrid.RefCell(newCellHandle);
			newCell.m_data.m_pipeNodeHandle = newPipeNodeHandle;
			newPipeNode.m_data.m_cellHandle = newCellHandle;
			firstInternode.m_data.m_gridHandle = newGridHandle;
		}
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
		firstRootNode.m_data.m_vigorSink.AddVigor(rootGrowthParameters.m_rootNodeVigorRequirement);

		if (m_enablePipe)
		{
			auto& pipeGroup = m_rootSkeleton.m_data.m_rootPipeGroup;
			pipeGroup = {};
			const auto& newPipeHandle = pipeGroup.AllocatePipe();
			const auto newPipeNodeHandle = pipeGroup.Extend(newPipeHandle);
			firstRootNode.m_data.m_pipeNodeHandles.emplace_back(newPipeNodeHandle);
			auto& newPipeNode = pipeGroup.RefPipeNode(newPipeNodeHandle);
			newPipeNode.m_data.m_nodeHandle = firstRootNode.GetHandle();

			auto& gridGroup = m_rootSkeleton.m_data.m_hexagonGridGroup;
			const auto newGridHandle = gridGroup.Allocate();
			auto& newGrid = gridGroup.RefGrid(newGridHandle);
			newGrid.m_data.m_nodeHandle = firstRootNode.GetHandle();
			const auto newCellHandle = newGrid.Allocate(glm::ivec2(0, 0));
			auto& newCell = newGrid.RefCell(newCellHandle);
			newCell.m_data.m_pipeNodeHandle = newPipeNodeHandle;
			newPipeNode.m_data.m_cellHandle = newCellHandle;
			firstRootNode.m_data.m_gridHandle = newGridHandle;
		}

	}
	m_initialized = true;
}

void TreeModel::CollectRootFlux(const glm::mat4& globalTransform, SoilModel& soilModel, const RootGrowthParameters& rootGrowthParameters)
{
	m_rootSkeleton.m_data.m_rootFlux.m_water = 0.0f;
	const auto& sortedRootNodeList = m_rootSkeleton.RefSortedNodeList();
	for (const auto& rootNodeHandle : sortedRootNodeList) {
		auto& rootNode = m_rootSkeleton.RefNode(rootNodeHandle);
		rootNode.m_data.m_age += m_currentDeltaTime;
		auto& rootNodeInfo = rootNode.m_info;
		auto worldSpacePosition = globalTransform * glm::translate(rootNodeInfo.m_globalPosition)[3];
		if (m_treeGrowthSettings.m_collectWater) {
			rootNode.m_data.m_water = soilModel.IntegrateWater(worldSpacePosition, 0.2);
			m_rootSkeleton.m_data.m_rootFlux.m_water += rootNode.m_data.m_water;
		}
	}

}

void TreeModel::CollectShootFlux(const glm::mat4& globalTransform, ClimateModel& climateModel,
	const ShootGrowthParameters& shootGrowthParameters)
{
	auto& shootData = m_shootSkeleton.m_data;
	shootData.m_shootFlux.m_lightEnergy = 0.0f;
	const auto& sortedInternodeList = m_shootSkeleton.RefSortedNodeList();
	shootData.m_treeVoxelVolume.m_voxel.Initialize(m_shootSkeleton.m_data.m_treeVoxelVolume.m_settings.m_voxelSize, m_shootSkeleton.m_min, m_shootSkeleton.m_max);

	const float maxLeafSize = glm::pow((shootGrowthParameters.m_maxLeafSize.x + shootGrowthParameters.m_maxLeafSize.z) / 2.0f, 2.0f);
	const float maxFruitSize = glm::pow((shootGrowthParameters.m_maxFruitSize.x + shootGrowthParameters.m_maxFruitSize.y + shootGrowthParameters.m_maxFruitSize.z) / 3.0f, 2.0f);
	for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); it++) {
		const auto& internode = m_shootSkeleton.RefNode(*it);
		const auto& internodeData = internode.m_data;
		const auto& internodeInfo = internode.m_info;
		float shadowSize = internodeInfo.m_length * internodeInfo.m_thickness * 2.0f;
		for (const auto& i : internodeData.m_buds)
		{
			if (i.m_type == BudType::Leaf && i.m_reproductiveModule.m_maturity > 0.0f)
			{
				shadowSize += maxLeafSize * glm::pow(i.m_reproductiveModule.m_maturity, 0.5f);
			}
			else if (i.m_type == BudType::Fruit && i.m_reproductiveModule.m_maturity > 0.0f)
			{
				shadowSize += maxFruitSize * glm::pow(i.m_reproductiveModule.m_maturity, 1.0f / 3.0f);
			}
		}
		shootData.m_treeVoxelVolume.AddShadowVolume({ internodeInfo.m_globalPosition, shadowSize });
	}

	for (const auto& internodeHandle : sortedInternodeList) {
		auto& internode = m_shootSkeleton.RefNode(internodeHandle);
		auto& internodeData = internode.m_data;
		auto& internodeInfo = internode.m_info;
		internodeData.m_age += m_currentDeltaTime;
		internodeData.m_lightIntensity =
			m_shootSkeleton.m_data.m_treeVoxelVolume.IlluminationEstimation(internodeInfo.m_globalPosition, internodeData.m_lightDirection);
		for (const auto& bud : internode.m_data.m_buds)
		{
			if (bud.m_status == BudStatus::Flushed && bud.m_type == BudType::Leaf)
			{
				if (m_treeGrowthSettings.m_collectLight) {
					internodeData.m_lightEnergy = internodeData.m_lightIntensity * glm::pow(bud.m_reproductiveModule.m_maturity, 2.0f) * bud.m_reproductiveModule.m_health;
					m_shootSkeleton.m_data.m_shootFlux.m_lightEnergy += internodeData.m_lightEnergy;
				}
			}
		}
	}


}

void TreeModel::PlantVigorAllocation()
{
	if (!m_treeGrowthSettings.m_collectWater) {
		if (!m_treeGrowthSettings.m_collectLight) {
			m_shootSkeleton.m_data.m_shootFlux.m_lightEnergy = m_rootSkeleton.m_data.m_rootFlux.m_water =
				m_shootSkeleton.m_data.m_vigorRequirement.m_leafMaintenanceVigor * m_treeGrowthSettings.m_leafMaintenanceVigorFillingRate
				+ m_shootSkeleton.m_data.m_vigorRequirement.m_leafDevelopmentalVigor * m_treeGrowthSettings.m_leafDevelopmentalVigorFillingRate
				+ m_shootSkeleton.m_data.m_vigorRequirement.m_fruitMaintenanceVigor * m_treeGrowthSettings.m_fruitMaintenanceVigorFillingRate
				+ m_shootSkeleton.m_data.m_vigorRequirement.m_fruitDevelopmentalVigor * m_treeGrowthSettings.m_fruitDevelopmentalVigorFillingRate
				+ m_shootSkeleton.m_data.m_vigorRequirement.m_nodeDevelopmentalVigor * m_treeGrowthSettings.m_nodeDevelopmentalVigorFillingRate

				//+ m_rootSkeleton.m_data.m_vigorRequirement.m_leafMaintenanceVigor * m_treeGrowthSettings.m_leafMaintenanceVigorFillingRate
				//+ m_rootSkeleton.m_data.m_vigorRequirement.m_leafDevelopmentalVigor * m_treeGrowthSettings.m_leafDevelopmentalVigorFillingRate
				//+ m_rootSkeleton.m_data.m_vigorRequirement.m_fruitMaintenanceVigor * m_treeGrowthSettings.m_fruitMaintenanceVigorFillingRate
				//+ m_rootSkeleton.m_data.m_vigorRequirement.m_fruitDevelopmentalVigor * m_treeGrowthSettings.m_fruitDevelopmentalVigorFillingRate
				+m_rootSkeleton.m_data.m_vigorRequirement.m_nodeDevelopmentalVigor * m_treeGrowthSettings.m_nodeDevelopmentalVigorFillingRate;
		}
		else
		{
			m_rootSkeleton.m_data.m_rootFlux.m_water =
				m_shootSkeleton.m_data.m_vigorRequirement.m_leafMaintenanceVigor
				+ m_shootSkeleton.m_data.m_vigorRequirement.m_leafDevelopmentalVigor
				+ m_shootSkeleton.m_data.m_vigorRequirement.m_fruitMaintenanceVigor
				+ m_shootSkeleton.m_data.m_vigorRequirement.m_fruitDevelopmentalVigor
				+ m_shootSkeleton.m_data.m_vigorRequirement.m_nodeDevelopmentalVigor

				//+ m_rootSkeleton.m_data.m_vigorRequirement.m_leafMaintenanceVigor
				//+ m_rootSkeleton.m_data.m_vigorRequirement.m_leafDevelopmentalVigor
				//+ m_rootSkeleton.m_data.m_vigorRequirement.m_fruitMaintenanceVigor
				//+ m_rootSkeleton.m_data.m_vigorRequirement.m_fruitDevelopmentalVigor
				+m_rootSkeleton.m_data.m_vigorRequirement.m_nodeDevelopmentalVigor;
		}
	}
	else if (!m_treeGrowthSettings.m_collectLight)
	{
		m_shootSkeleton.m_data.m_shootFlux.m_lightEnergy =
			m_shootSkeleton.m_data.m_vigorRequirement.m_leafMaintenanceVigor
			+ m_shootSkeleton.m_data.m_vigorRequirement.m_leafDevelopmentalVigor
			+ m_shootSkeleton.m_data.m_vigorRequirement.m_fruitMaintenanceVigor
			+ m_shootSkeleton.m_data.m_vigorRequirement.m_fruitDevelopmentalVigor
			+ m_shootSkeleton.m_data.m_vigorRequirement.m_nodeDevelopmentalVigor

			//+ m_rootSkeleton.m_data.m_vigorRequirement.m_leafMaintenanceVigor
			//+ m_rootSkeleton.m_data.m_vigorRequirement.m_leafDevelopmentalVigor
			//+ m_rootSkeleton.m_data.m_vigorRequirement.m_fruitMaintenanceVigor
			//+ m_rootSkeleton.m_data.m_vigorRequirement.m_fruitDevelopmentalVigor
			+m_rootSkeleton.m_data.m_vigorRequirement.m_nodeDevelopmentalVigor;
	}

	const float totalVigor = glm::min(m_rootSkeleton.m_data.m_rootFlux.m_water, m_shootSkeleton.m_data.m_shootFlux.m_lightEnergy);
	const float totalLeafMaintenanceVigorRequirement = m_shootSkeleton.m_data.m_vigorRequirement.m_leafMaintenanceVigor + m_rootSkeleton.m_data.m_vigorRequirement.m_leafMaintenanceVigor;
	const float totalLeafDevelopmentVigorRequirement = m_shootSkeleton.m_data.m_vigorRequirement.m_leafDevelopmentalVigor + m_rootSkeleton.m_data.m_vigorRequirement.m_leafDevelopmentalVigor;
	const float totalFruitMaintenanceVigorRequirement = m_shootSkeleton.m_data.m_vigorRequirement.m_fruitMaintenanceVigor + m_rootSkeleton.m_data.m_vigorRequirement.m_fruitMaintenanceVigor;
	const float totalFruitDevelopmentVigorRequirement = m_shootSkeleton.m_data.m_vigorRequirement.m_fruitDevelopmentalVigor + m_rootSkeleton.m_data.m_vigorRequirement.m_fruitDevelopmentalVigor;
	const float totalNodeDevelopmentalVigorRequirement = m_shootSkeleton.m_data.m_vigorRequirement.m_nodeDevelopmentalVigor + m_rootSkeleton.m_data.m_vigorRequirement.m_nodeDevelopmentalVigor;

	const float leafMaintenanceVigor = glm::min(totalVigor, totalLeafMaintenanceVigorRequirement);
	const float leafDevelopmentVigor = glm::min(totalVigor - totalLeafMaintenanceVigorRequirement, totalLeafDevelopmentVigorRequirement);
	const float fruitMaintenanceVigor = glm::min(totalVigor - totalLeafMaintenanceVigorRequirement - leafDevelopmentVigor, totalFruitMaintenanceVigorRequirement);
	const float fruitDevelopmentVigor = glm::min(totalVigor - totalLeafMaintenanceVigorRequirement - leafDevelopmentVigor - fruitMaintenanceVigor, totalFruitDevelopmentVigorRequirement);
	const float nodeDevelopmentVigor = glm::min(totalVigor - totalLeafMaintenanceVigorRequirement - leafDevelopmentVigor - fruitMaintenanceVigor - fruitDevelopmentVigor, totalNodeDevelopmentalVigorRequirement);
	m_rootSkeleton.m_data.m_vigor = m_shootSkeleton.m_data.m_vigor = 0.0f;
	if (totalLeafMaintenanceVigorRequirement != 0.0f) {
		m_rootSkeleton.m_data.m_vigor += leafMaintenanceVigor * m_rootSkeleton.m_data.m_vigorRequirement.m_leafMaintenanceVigor
			/ totalLeafMaintenanceVigorRequirement;
		m_shootSkeleton.m_data.m_vigor += leafMaintenanceVigor * m_shootSkeleton.m_data.m_vigorRequirement.m_leafMaintenanceVigor
			/ totalLeafMaintenanceVigorRequirement;
	}
	if (totalLeafDevelopmentVigorRequirement != 0.0f) {
		m_rootSkeleton.m_data.m_vigor += leafDevelopmentVigor * m_rootSkeleton.m_data.m_vigorRequirement.m_leafDevelopmentalVigor
			/ totalLeafDevelopmentVigorRequirement;
		m_shootSkeleton.m_data.m_vigor += leafDevelopmentVigor * m_shootSkeleton.m_data.m_vigorRequirement.m_leafDevelopmentalVigor
			/ totalLeafDevelopmentVigorRequirement;
	}
	if (totalFruitMaintenanceVigorRequirement != 0.0f) {
		m_rootSkeleton.m_data.m_vigor += fruitMaintenanceVigor * m_rootSkeleton.m_data.m_vigorRequirement.m_fruitMaintenanceVigor
			/ totalFruitMaintenanceVigorRequirement;
		m_shootSkeleton.m_data.m_vigor += fruitMaintenanceVigor * m_shootSkeleton.m_data.m_vigorRequirement.m_fruitMaintenanceVigor
			/ totalFruitMaintenanceVigorRequirement;
	}
	if (totalFruitDevelopmentVigorRequirement != 0.0f) {
		m_rootSkeleton.m_data.m_vigor += fruitDevelopmentVigor * m_rootSkeleton.m_data.m_vigorRequirement.m_fruitDevelopmentalVigor
			/ totalFruitDevelopmentVigorRequirement;
		m_shootSkeleton.m_data.m_vigor += fruitDevelopmentVigor * m_shootSkeleton.m_data.m_vigorRequirement.m_fruitDevelopmentalVigor
			/ totalFruitDevelopmentVigorRequirement;
	}


	if (m_treeGrowthSettings.m_autoBalance) {
		m_vigorRatio.m_shootVigorWeight = m_rootSkeleton.RefSortedNodeList().size() * m_shootSkeleton.m_data.m_vigorRequirement.m_nodeDevelopmentalVigor;
		m_vigorRatio.m_rootVigorWeight = m_shootSkeleton.RefSortedNodeList().size() * m_rootSkeleton.m_data.m_vigorRequirement.m_nodeDevelopmentalVigor;
	}
	else {
		m_vigorRatio.m_rootVigorWeight = m_rootSkeleton.m_data.m_vigorRequirement.m_nodeDevelopmentalVigor / totalNodeDevelopmentalVigorRequirement;
		m_vigorRatio.m_shootVigorWeight = m_shootSkeleton.m_data.m_vigorRequirement.m_nodeDevelopmentalVigor / totalNodeDevelopmentalVigorRequirement;
	}

	if (m_vigorRatio.m_shootVigorWeight + m_vigorRatio.m_rootVigorWeight != 0.0f) {
		m_rootSkeleton.m_data.m_vigor += nodeDevelopmentVigor * m_vigorRatio.m_rootVigorWeight / (m_vigorRatio.m_shootVigorWeight + m_vigorRatio.m_rootVigorWeight);
		m_shootSkeleton.m_data.m_vigor += nodeDevelopmentVigor * m_vigorRatio.m_shootVigorWeight / (m_vigorRatio.m_shootVigorWeight + m_vigorRatio.m_rootVigorWeight);
	}
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
			AggregateRootVigorRequirement(rootGrowthParameters);
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
					rootNodeInfo.m_globalRotation = rootNodeInfo.m_regulatedGlobalRotation = glm::vec3(glm::radians(-90.0f), 0.0f, 0.0f);

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


					auto front = rootNodeInfo.m_globalRotation * glm::vec3(0, 0, -1);
					auto parentRegulatedUp = parentRootNode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 1, 0);
					auto regulatedUp = glm::normalize(glm::cross(glm::cross(front, parentRegulatedUp), front));
					rootNodeInfo.m_regulatedGlobalRotation = glm::quatLookAt(front, regulatedUp);

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
		if (m_treeGrowthSettings.m_enableRootCollisionDetection)
		{
			const float minRadius = rootGrowthParameters.m_endNodeThickness * 4.0f;
			CollisionDetection(minRadius, m_rootSkeleton.m_data.m_octree, m_rootSkeleton);
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

			if (m_enablePipe)
			{
				auto& rootPipeGroup = m_rootSkeleton.m_data.m_rootPipeGroup;
				auto& gridGroup = m_rootSkeleton.m_data.m_hexagonGridGroup;
				for (auto& pipeNode : rootPipeGroup.RefPipeNodes())
				{
					if (pipeNode.IsRecycled()) continue;
					const auto& node = m_rootSkeleton.PeekNode(pipeNode.m_data.m_nodeHandle);
					const auto& nodeInfo = node.m_info;
					const glm::vec3 left = nodeInfo.m_regulatedGlobalRotation * glm::vec3(1, 0, 0);
					const glm::vec3 up = nodeInfo.m_regulatedGlobalRotation * glm::vec3(0, 1, 0);
					const glm::vec3 front = nodeInfo.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
					auto& pipeInfo = pipeNode.m_info;
					const auto& grid = gridGroup.PeekGrid(node.m_data.m_gridHandle);
					const auto& cell = grid.PeekCell(pipeNode.m_data.m_cellHandle);

					pipeInfo.m_localPosition = rootGrowthParameters.m_endNodeThickness * 2.0f * grid.GetPosition(cell.GetCoordinate());
					pipeInfo.m_startThickness = pipeInfo.m_endThickness = rootGrowthParameters.m_endNodeThickness;
					pipeInfo.m_globalStartPosition = nodeInfo.m_globalPosition + left * pipeInfo.m_localPosition.x + up * pipeInfo.m_localPosition.y;
					pipeInfo.m_globalStartRotation = nodeInfo.m_regulatedGlobalRotation;
					pipeInfo.m_globalEndPosition = nodeInfo.m_globalPosition + nodeInfo.m_length * front + left * pipeInfo.m_localPosition.x + up * pipeInfo.m_localPosition.y;
					pipeInfo.m_globalEndRotation = nodeInfo.m_regulatedGlobalRotation;

				}
			}
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
		AggregateInternodeVigorRequirement(shootGrowthParameters);
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
				internodeInfo.m_globalRotation = internodeInfo.m_regulatedGlobalRotation = glm::vec3(glm::radians(90.0f), 0.0f, 0.0f);

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
				const auto& parentNode = m_shootSkeleton.RefNode(
					internode.GetParentHandle());
				auto parentGlobalRotation = parentNode.m_info.m_globalRotation;
				internodeInfo.m_globalRotation = parentGlobalRotation * internodeData.m_desiredLocalRotation;
				auto front = internodeInfo.m_globalRotation * glm::vec3(0, 0, -1);
				auto up = internodeInfo.m_globalRotation * glm::vec3(0, 1, 0);
				float dotP = glm::abs(glm::dot(front, m_currentGravityDirection));
				ApplyTropism(m_currentGravityDirection, internodeData.m_sagging * (1.0f - dotP), front, up);
				internodeInfo.m_globalRotation = glm::quatLookAt(front, up);
				internodeInfo.m_localRotation = glm::inverse(parentGlobalRotation) * internodeInfo.m_globalRotation;

				auto parentRegulatedUp = parentNode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 1, 0);
				auto regulatedUp = glm::normalize(glm::cross(glm::cross(front, parentRegulatedUp), front));
				internodeInfo.m_regulatedGlobalRotation = glm::quatLookAt(front, regulatedUp);
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
	};

	if (m_treeGrowthSettings.m_enableBranchCollisionDetection)
	{
		const float minRadius = shootGrowthParameters.m_endNodeThickness * 4.0f;
		CollisionDetection(minRadius, m_shootSkeleton.m_data.m_octree, m_shootSkeleton);
	}
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
				if (bud.m_status != BudStatus::Flushed || bud.m_reproductiveModule.m_maturity <= 0) continue;
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
		if (m_enablePipe)
		{
			auto& shootPipeGroup = m_shootSkeleton.m_data.m_shootPipeGroup;
			auto& gridGroup = m_shootSkeleton.m_data.m_hexagonGridGroup;
			for (auto& pipeNode : shootPipeGroup.RefPipeNodes())
			{
				if (pipeNode.IsRecycled()) continue;
				const auto& node = m_shootSkeleton.PeekNode(pipeNode.m_data.m_nodeHandle);
				const auto& nodeInfo = node.m_info;
				const glm::vec3 left = nodeInfo.m_regulatedGlobalRotation * glm::vec3(1, 0, 0);
				const glm::vec3 up = nodeInfo.m_regulatedGlobalRotation * glm::vec3(0, 1, 0);
				const glm::vec3 front = nodeInfo.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
				auto& pipeInfo = pipeNode.m_info;
				const auto& grid = gridGroup.PeekGrid(node.m_data.m_gridHandle);
				const auto& cell = grid.PeekCell(pipeNode.m_data.m_cellHandle);

				pipeInfo.m_localPosition = shootGrowthParameters.m_endNodeThickness * 2.0f * grid.GetPosition(cell.GetCoordinate());
				pipeInfo.m_startThickness = pipeInfo.m_endThickness = shootGrowthParameters.m_endNodeThickness;
				pipeInfo.m_globalStartPosition = nodeInfo.m_globalPosition + left * pipeInfo.m_localPosition.x + up * pipeInfo.m_localPosition.y;
				pipeInfo.m_globalStartRotation = nodeInfo.m_regulatedGlobalRotation;
				pipeInfo.m_globalEndPosition = nodeInfo.m_globalPosition + nodeInfo.m_length * front + left * pipeInfo.m_localPosition.x + up * pipeInfo.m_localPosition.y;
				pipeInfo.m_globalEndRotation = nodeInfo.m_regulatedGlobalRotation;

			}
		}

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
		auto newRootNodeHandle = m_rootSkeleton.Extend(rootNodeHandle, false, m_rootSkeleton.RefFlow(rootNode.GetFlowHandle()).RefNodeHandles().size() > m_treeGrowthSettings.m_flowNodeLimit);
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
		newRootNode.m_data.m_vigorFlow.m_vigorRequirementWeight = oldRootNode.m_data.m_vigorFlow.m_vigorRequirementWeight;

		newRootNode.m_data.m_inhibitor = 0.0f;
		newRootNode.m_info.m_length = glm::clamp(extendLength, 0.0f, rootNodeLength);
		newRootNode.m_data.m_rootDistance = oldRootNode.m_data.m_rootDistance + newRootNode.m_info.m_length / rootGrowthParameters.m_rootNodeLength;
		newRootNode.m_info.m_thickness = rootGrowthParameters.m_endNodeThickness;
		newRootNode.m_info.m_globalRotation = glm::quatLookAt(desiredGlobalFront, desiredGlobalUp);
		newRootNode.m_info.m_localRotation =
			glm::inverse(oldRootNode.m_info.m_globalRotation) *
			newRootNode.m_info.m_globalRotation;

		if (m_enablePipe)
		{
			auto& pipeGroup = m_rootSkeleton.m_data.m_rootPipeGroup;
			const auto oldPipeNodeHandle = oldRootNode.m_data.m_pipeNodeHandles[0];
			const auto pipeHandle = pipeGroup.RefPipeNode(oldPipeNodeHandle).GetPipeHandle();
			const auto newPipeNodeHandle = pipeGroup.Extend(pipeHandle);
			const auto& oldPipeNode = pipeGroup.RefPipeNode(oldPipeNodeHandle);
			auto& newPipeNode = pipeGroup.RefPipeNode(newPipeNodeHandle);
			newRootNode.m_data.m_pipeNodeHandles.emplace_back(newPipeNodeHandle);
			newPipeNode.m_data.m_nodeHandle = newRootNode.GetHandle();

			const auto parentInternodeHandle = oldRootNode.GetParentHandle();
			bool needReplacement = false;
			if (parentInternodeHandle > 0)
			{
				if (m_rootSkeleton.RefNode(parentInternodeHandle).RefChildHandles().size() > 1) needReplacement = true;
			}
			if (needReplacement)
			{
				auto newGridHandle = m_rootSkeleton.m_data.m_hexagonGridGroup.Allocate();
				auto& newGrid = m_rootSkeleton.m_data.m_hexagonGridGroup.RefGrid(newGridHandle);
				newGrid.m_data.m_nodeHandle = newRootNodeHandle;
				newPipeNode.m_data.m_cellHandle = newGrid.Allocate(glm::vec2(0, 0));
				newRootNode.m_data.m_gridHandle = newGridHandle;
			}
			else {
				newRootNode.m_data.m_gridHandle = oldRootNode.m_data.m_gridHandle;
				newPipeNode.m_data.m_cellHandle = oldPipeNode.m_data.m_cellHandle;
			}
		}

		if (extraLength > rootNodeLength) {
			float childAuxin = 0.0f;
			ElongateRoot(soilModel, extraLength - rootNodeLength, newRootNodeHandle, rootGrowthParameters, childAuxin);
			childAuxin *= rootGrowthParameters.m_apicalDominanceDistanceFactor;
			collectedAuxin += childAuxin;
			m_rootSkeleton.RefNode(newRootNodeHandle).m_data.m_inhibitor = childAuxin;
		}
		else {
			newRootNode.m_data.m_inhibitor = rootGrowthParameters.m_apicalDominance * glm::exp(-rootGrowthParameters.m_apicalDominanceAgeFactor * m_age);
			collectedAuxin += newRootNode.m_data.m_inhibitor *= rootGrowthParameters.m_apicalDominanceDistanceFactor;
		}
	}
	else {
		//Otherwise, we add the inhibitor.
		collectedAuxin += rootGrowthParameters.m_apicalDominance * glm::exp(-rootGrowthParameters.m_apicalDominanceAgeFactor * m_age);
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
	if (extraLength >= 0) {
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
				//Hack: Leaf bud will be given vigor for the first time.
				leafBud.m_vigorSink.AddVigor(shootGrowthParameters.m_leafVigorRequirement);
				leafBud.m_type = BudType::Leaf;
				leafBud.m_status = BudStatus::Dormant;
				leafBud.m_localRotation = glm::vec3(
					glm::radians(shootGrowthParameters.GetDesiredBranchingAngle(internode)), 0.0f,
					glm::radians(glm::linearRand(0.0f, 360.0f)));
			}
		}
		//Create new internode
		const auto newInternodeHandle = m_shootSkeleton.Extend(internodeHandle, false, m_shootSkeleton.RefFlow(internode.GetFlowHandle()).RefNodeHandles().size() > m_treeGrowthSettings.m_flowNodeLimit);
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
		if (m_enablePipe)
		{
			auto& pipeGroup = m_shootSkeleton.m_data.m_shootPipeGroup;
			const auto oldPipeNodeHandle = oldInternode.m_data.m_pipeNodeHandles[0];
			const auto pipeHandle = pipeGroup.RefPipeNode(oldPipeNodeHandle).GetPipeHandle();
			const auto newPipeNodeHandle = pipeGroup.Extend(pipeHandle);
			const auto& oldPipeNode = pipeGroup.RefPipeNode(oldPipeNodeHandle);
			auto& newPipeNode = pipeGroup.RefPipeNode(newPipeNodeHandle);
			newInternode.m_data.m_pipeNodeHandles.emplace_back(newPipeNodeHandle);
			newPipeNode.m_data.m_nodeHandle = newInternode.GetHandle();

			const auto parentInternodeHandle = oldInternode.GetParentHandle();
			bool needReplacement = false;
			if(parentInternodeHandle > 0)
			{
				if (m_shootSkeleton.RefNode(parentInternodeHandle).RefChildHandles().size() > 1) needReplacement = true;
			}
			if(needReplacement)
			{
				auto newGridHandle = m_shootSkeleton.m_data.m_hexagonGridGroup.Allocate();
				auto& newGrid = m_shootSkeleton.m_data.m_hexagonGridGroup.RefGrid(newGridHandle);
				newGrid.m_data.m_nodeHandle = newInternodeHandle;
				newPipeNode.m_data.m_cellHandle = newGrid.Allocate(glm::vec2(0, 0));
				newInternode.m_data.m_gridHandle = newGridHandle;
			}
			else {
				newInternode.m_data.m_gridHandle = oldInternode.m_data.m_gridHandle;
				newPipeNode.m_data.m_cellHandle = oldPipeNode.m_data.m_cellHandle;
			}
		}

		if (extraLength > internodeLength) {
			float childInhibitor = 0.0f;
			ElongateInternode(extraLength - internodeLength, newInternodeHandle, shootGrowthParameters, childInhibitor);
			childInhibitor *= shootGrowthParameters.m_apicalDominanceDistanceFactor;
			collectedInhibitor += childInhibitor;
			m_shootSkeleton.RefNode(newInternodeHandle).m_data.m_inhibitor = childInhibitor;
		}
		else {
			newInternode.m_data.m_inhibitor = shootGrowthParameters.m_apicalDominance * glm::exp(-shootGrowthParameters.m_apicalDominanceAgeFactor * m_age);
			collectedInhibitor += newInternode.m_data.m_inhibitor *= shootGrowthParameters.m_apicalDominanceDistanceFactor;
		}


	}
	else {
		//Otherwise, we add the inhibitor.
		collectedInhibitor += shootGrowthParameters.m_apicalDominance * glm::exp(-shootGrowthParameters.m_apicalDominanceAgeFactor * m_age);
	}
	return graphChanged;
}

inline bool TreeModel::GrowRootNode(SoilModel& soilModel, NodeHandle rootNodeHandle, const RootGrowthParameters& rootGrowthParameters)
{
	bool graphChanged = false;
	{
		auto& rootNode = m_rootSkeleton.RefNode(rootNodeHandle);
		auto& rootNodeData = rootNode.m_data;
		rootNodeData.m_inhibitor = 0;
		for (const auto& childHandle : rootNode.RefChildHandles()) {
			rootNodeData.m_inhibitor += m_rootSkeleton.RefNode(childHandle).m_data.m_inhibitor *
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
			m_rootSkeleton.RefNode(rootNodeHandle).m_data.m_inhibitor += collectedAuxin * dd;

			const float maintenanceVigor = m_rootSkeleton.RefNode(rootNodeHandle).m_data.m_vigorSink.SubtractVigor(availableMaintenanceVigor);
		}
		else
		{
			//2. Form new shoot if necessary
			float branchingProb = m_rootNodeDevelopmentRate * m_currentDeltaTime * rootGrowthParameters.m_rootNodeGrowthRate * rootGrowthParameters.m_branchingProbability;
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

				if (m_enablePipe)
				{
					auto& pipeGroup = m_rootSkeleton.m_data.m_rootPipeGroup;
					auto& gridGroup = m_rootSkeleton.m_data.m_hexagonGridGroup;
					const auto newPipeHandle = pipeGroup.AllocatePipe();
					PipeNodeHandle newPipeNodeHandle = -1;

					//Here, we want to build a new pipe that is close to the original pipe that this node branches from.
					const auto& prevPipeNodeEnd = pipeGroup.PeekPipeNode(oldRootNode.m_data.m_pipeNodeHandles[0]);
					const auto& prevPipe = pipeGroup.PeekPipe(prevPipeNodeEnd.GetPipeHandle());
					//To be inherited when the flow stays the same
					//To be used to track flow changes.
					//Walk along the old pipe, create new pipe nodes one by one until the branching point.
					glm::vec2 principleDirection = glm::diskRand(1.0f);
					HexagonGridHandle currentGridHandle = -1;
					HexagonCellHandle currentCellHandle = -1;
					bool branchingPoint = false;
					HexagonGridHandle replacementGridHandle = -1;
					for (const auto& prevPipeNodeHandle : prevPipe.PeekPipeNodeHandles())
					{
						if (!branchingPoint) {
							//First, create a new pipe node from new pipe.
							newPipeNodeHandle = pipeGroup.Extend(newPipeHandle);
							auto& newPipeNode = pipeGroup.RefPipeNode(newPipeNodeHandle);
							//Find the node that includes the current old pipe node.
							auto& prevPipeNode = pipeGroup.PeekPipeNode(prevPipeNodeHandle).m_data;
							auto prevNodeHandle = prevPipeNode.m_nodeHandle;
							auto& currentVisitingNode = m_rootSkeleton.RefNode(prevNodeHandle);
							newPipeNode.m_data.m_nodeHandle = prevNodeHandle;
							currentVisitingNode.m_data.m_pipeNodeHandles.emplace_back(newPipeNodeHandle);


							if (currentVisitingNode.m_data.m_gridHandle != currentGridHandle) {
								//If we are meeting a new grid, we need to create a new cell for current pipe.
								currentGridHandle = currentVisitingNode.m_data.m_gridHandle;
								auto& grid = gridGroup.RefGrid(currentGridHandle);
								auto newCoordinate = grid.FindClosestEmptyCoordinate(prevPipeNode.m_cellHandle, principleDirection);
								currentCellHandle = grid.Allocate(newCoordinate);
								auto& cell = grid.RefCell(currentCellHandle);
								cell.m_data.m_pipeNodeHandle = newPipeNodeHandle;
							}
							newPipeNode.m_data.m_cellHandle = currentCellHandle;

							if (prevNodeHandle == rootNodeHandle) {
								branchingPoint = true;
							}
						}
						else
						{
							auto& prevPipeNode = pipeGroup.PeekPipeNode(prevPipeNodeHandle).m_data;
							auto prevNodeHandle = prevPipeNode.m_nodeHandle;
							auto& currentVisitingNode = m_rootSkeleton.RefNode(prevNodeHandle);
							if (replacementGridHandle == -1)
							{
								replacementGridHandle = gridGroup.Allocate();
								auto& replacementGrid = gridGroup.RefGrid(replacementGridHandle);
								replacementGrid.Copy(gridGroup.RefGrid(currentVisitingNode.m_data.m_gridHandle));
								replacementGrid.m_data.m_nodeHandle = prevNodeHandle;
							}
							if (currentVisitingNode.m_data.m_gridHandle == currentGridHandle) {
								currentVisitingNode.m_data.m_gridHandle = replacementGridHandle;
							}
							else
							{
								break;
							}
						}
					}

					newPipeNodeHandle = pipeGroup.Extend(newPipeHandle);
					auto& newPipeNode = pipeGroup.RefPipeNode(newPipeNodeHandle);
					newPipeNode.m_data.m_nodeHandle = newRootNode.GetHandle();

					const auto newGridHandle = gridGroup.Allocate();
					auto& newGrid = gridGroup.RefGrid(newGridHandle);
					newGrid.m_data.m_nodeHandle = newRootNodeHandle;
					const auto newCellHandle = newGrid.Allocate(glm::ivec2(0, 0));
					newPipeNode.m_data.m_cellHandle = newCellHandle;
					newRootNode.m_data.m_gridHandle = newGridHandle;
					newRootNode.m_data.m_pipeNodeHandles.emplace_back(newPipeNodeHandle);

					auto& cell = newGrid.RefCell(newCellHandle);
					cell.m_data.m_pipeNodeHandle = newPipeNodeHandle;
				}

			}
		}


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

		/*
		auto killProbability = shootGrowthParameters.m_growthRate * shootGrowthParameters.m_pro;
		if (internodeData.m_rootDistance < 1.0f) killProbability = 0.0f;
		if (bud.m_status == BudStatus::Dormant && killProbability > glm::linearRand(0.0f, 1.0f)) {
			bud.m_status = BudStatus::Died;
		}
		if (bud.m_status == BudStatus::Died) continue;
		*/

		//Calculate vigor used for maintenance and development.
		const float desiredMaintenanceVigor = bud.m_vigorSink.GetDesiredMaintenanceVigorRequirement();
		const float availableMaintenanceVigor = bud.m_vigorSink.GetAvailableMaintenanceVigor();
		const float availableDevelopmentVigor = bud.m_vigorSink.GetAvailableDevelopmentalVigor();
		const float maintenanceVigor = bud.m_vigorSink.SubtractVigor(availableMaintenanceVigor);
		if (desiredMaintenanceVigor != 0.0f && availableMaintenanceVigor < desiredMaintenanceVigor) {
			bud.m_reproductiveModule.m_health = glm::clamp(bud.m_reproductiveModule.m_health * availableMaintenanceVigor / desiredMaintenanceVigor, 0.0f, 1.0f);
		}
		if (bud.m_type == BudType::Apical && bud.m_status == BudStatus::Dormant) {
			const float developmentalVigor = bud.m_vigorSink.SubtractVigor(availableDevelopmentVigor);
			const float elongateLength = developmentalVigor / shootGrowthParameters.m_internodeVigorRequirement * shootGrowthParameters.m_internodeLength;
			//Use up the vigor stored in this bud.
			float collectedInhibitor = 0.0f;
			const auto dd = shootGrowthParameters.m_apicalDominanceDistanceFactor;
			graphChanged = ElongateInternode(elongateLength, internodeHandle, shootGrowthParameters, collectedInhibitor) || graphChanged;
			m_shootSkeleton.RefNode(internodeHandle).m_data.m_inhibitor += collectedInhibitor * dd;
		}
		if (bud.m_type == BudType::Lateral && bud.m_status == BudStatus::Dormant) {
			const auto& probabilityRange = shootGrowthParameters.m_lateralBudFlushingProbabilityTemperatureRange;
			float flushProbability = m_internodeDevelopmentRate * m_currentDeltaTime * shootGrowthParameters.m_internodeGrowthRate * glm::mix(probabilityRange.x, probabilityRange.y,
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

				if (m_enablePipe)
				{
					auto& pipeGroup = m_shootSkeleton.m_data.m_shootPipeGroup;
					auto& gridGroup = m_shootSkeleton.m_data.m_hexagonGridGroup;
					const auto newPipeHandle = pipeGroup.AllocatePipe();
					PipeNodeHandle newPipeNodeHandle = -1;

					//Here, we want to build a new pipe that is close to the original pipe that this node branches from.
					const auto& prevPipeNodeEnd = pipeGroup.PeekPipeNode(oldInternode.m_data.m_pipeNodeHandles[0]);
					const auto& prevPipe = pipeGroup.PeekPipe(prevPipeNodeEnd.GetPipeHandle());
					//To be inherited when the flow stays the same
					//To be used to track flow changes.
					//Walk along the old pipe, create new pipe nodes one by one until the branching point.
					glm::vec2 principleDirection = glm::diskRand(1.0f);
					HexagonGridHandle currentGridHandle = -1;
					HexagonCellHandle currentCellHandle = -1;
					bool branchingPoint = false;
					HexagonGridHandle replacementGridHandle = -1;
					for (const auto& prevPipeNodeHandle : prevPipe.PeekPipeNodeHandles())
					{
						if (!branchingPoint) {
							//First, create a new pipe node from new pipe.
							newPipeNodeHandle = pipeGroup.Extend(newPipeHandle);
							auto& newPipeNode = pipeGroup.RefPipeNode(newPipeNodeHandle);
							//Find the node that includes the current old pipe node.
							auto& prevPipeNode = pipeGroup.PeekPipeNode(prevPipeNodeHandle).m_data;
							auto prevNodeHandle = prevPipeNode.m_nodeHandle;
							auto& currentVisitingNode = m_shootSkeleton.RefNode(prevNodeHandle);
							newPipeNode.m_data.m_nodeHandle = prevNodeHandle;
							currentVisitingNode.m_data.m_pipeNodeHandles.emplace_back(newPipeNodeHandle);


							if (currentVisitingNode.m_data.m_gridHandle != currentGridHandle) {
								//If we are meeting a new grid, we need to create a new cell for current pipe.
								currentGridHandle = currentVisitingNode.m_data.m_gridHandle;
								auto& grid = gridGroup.RefGrid(currentGridHandle);
								auto newCoordinate = grid.FindClosestEmptyCoordinate(prevPipeNode.m_cellHandle, principleDirection);
								currentCellHandle = grid.Allocate(newCoordinate);
								auto& cell = grid.RefCell(currentCellHandle);
								cell.m_data.m_pipeNodeHandle = newPipeNodeHandle;
							}
							newPipeNode.m_data.m_cellHandle = currentCellHandle;

							if (prevNodeHandle == internodeHandle) {
								branchingPoint = true;
							}
						}
						else
						{
							auto& prevPipeNode = pipeGroup.PeekPipeNode(prevPipeNodeHandle).m_data;
							auto prevNodeHandle = prevPipeNode.m_nodeHandle;
							auto& currentVisitingNode = m_shootSkeleton.RefNode(prevNodeHandle);
							if (replacementGridHandle == -1)
							{
								replacementGridHandle = gridGroup.Allocate();
								auto& replacementGrid = gridGroup.RefGrid(replacementGridHandle);
								replacementGrid.Copy(gridGroup.RefGrid(currentVisitingNode.m_data.m_gridHandle));
								replacementGrid.m_data.m_nodeHandle = prevNodeHandle;
							}
							if (currentVisitingNode.m_data.m_gridHandle == currentGridHandle) {
								currentVisitingNode.m_data.m_gridHandle = replacementGridHandle;
							}
							else
							{
								break;
							}
						}
					}

					newPipeNodeHandle = pipeGroup.Extend(newPipeHandle);
					auto& newPipeNode = pipeGroup.RefPipeNode(newPipeNodeHandle);
					newPipeNode.m_data.m_nodeHandle = newInternode.GetHandle();

					const auto newGridHandle = gridGroup.Allocate();
					auto& newGrid = gridGroup.RefGrid(newGridHandle);
					newGrid.m_data.m_nodeHandle = newInternodeHandle;
					const auto newCellHandle = newGrid.Allocate(glm::ivec2(0, 0));
					newPipeNode.m_data.m_cellHandle = newCellHandle;
					newInternode.m_data.m_gridHandle = newGridHandle;
					newInternode.m_data.m_pipeNodeHandles.emplace_back(newPipeNodeHandle);

					auto& cell = newGrid.RefCell(newCellHandle);
					cell.m_data.m_pipeNodeHandle = newPipeNodeHandle;
				}
			}
		}
		else if (bud.m_type == BudType::Fruit)
		{
			if (bud.m_status == BudStatus::Dormant) {

				const auto& probabilityRange = shootGrowthParameters.m_fruitBudFlushingProbabilityTemperatureRange;
				float flushProbability = m_currentDeltaTime * glm::mix(probabilityRange.x, probabilityRange.y,
					glm::clamp((internodeData.m_temperature - probabilityRange.z) / (probabilityRange.w - probabilityRange.z), 0.0f, 1.0f));
				flushProbability *= glm::pow(internodeData.m_lightIntensity, shootGrowthParameters.m_fruitBudLightingFactor);
				if (flushProbability >= glm::linearRand(0.0f, 1.0f))
				{
					bud.m_status = BudStatus::Flushed;
				}
			}
			else if (bud.m_status == BudStatus::Flushed)
			{
				//Make the fruit larger;
				const float maxMaturityIncrease = availableDevelopmentVigor / shootGrowthParameters.m_fruitVigorRequirement;
				const float maturityIncrease = glm::min(maxMaturityIncrease, glm::min(m_currentDeltaTime * shootGrowthParameters.m_fruitGrowthRate, 1.0f - bud.m_reproductiveModule.m_maturity));
				bud.m_reproductiveModule.m_maturity += maturityIncrease;
				const auto developmentVigor = bud.m_vigorSink.SubtractVigor(maturityIncrease * shootGrowthParameters.m_fruitVigorRequirement);
				auto fruitSize = shootGrowthParameters.m_maxFruitSize * glm::pow(bud.m_reproductiveModule.m_maturity, 1.0f / 3.0f);
				float angle = glm::radians(glm::linearRand(0.0f, 360.0f));
				glm::quat rotation = internodeData.m_desiredLocalRotation * bud.m_localRotation;
				auto up = rotation * glm::vec3(0, 1, 0);
				auto front = rotation * glm::vec3(0, 0, -1);
				ApplyTropism(internodeData.m_lightDirection, 0.3f, up, front);
				rotation = glm::quatLookAt(front, up);
				auto fruitPosition = internodeInfo.m_globalPosition + front * (fruitSize.z * 1.5f);
				bud.m_reproductiveModule.m_transform = glm::translate(fruitPosition) * glm::mat4_cast(glm::quat(glm::vec3(0.0f))) * glm::scale(fruitSize);

				//Handle fruit drop here.
				if (bud.m_reproductiveModule.m_maturity >= 0.95f || bud.m_reproductiveModule.m_health <= 0.05f)
				{
					auto dropProbability = m_currentDeltaTime * shootGrowthParameters.m_fruitFallProbability;
					if (dropProbability >= glm::linearRand(0.0f, 1.0f))
					{
						bud.m_status = BudStatus::Died;
						m_shootSkeleton.m_data.m_droppedFruits.emplace_back(bud.m_reproductiveModule);
						bud.m_reproductiveModule.Reset();
					}
				}

			}
		}
		else if (bud.m_type == BudType::Leaf)
		{
			if (bud.m_status == BudStatus::Dormant) {
				const auto& probabilityRange = shootGrowthParameters.m_leafBudFlushingProbabilityTemperatureRange;
				float flushProbability = m_currentDeltaTime * glm::mix(probabilityRange.x, probabilityRange.y,
					glm::clamp((internodeData.m_temperature - probabilityRange.z) / (probabilityRange.w - probabilityRange.z), 0.0f, 1.0f));
				flushProbability *= glm::pow(internodeData.m_lightIntensity, shootGrowthParameters.m_leafBudLightingFactor);
				if (internodeData.m_maxDistanceToAnyBranchEnd < shootGrowthParameters.m_leafDistanceToBranchEndLimit && flushProbability >= glm::linearRand(0.0f, 1.0f))
				{
					bud.m_status = BudStatus::Flushed;
				}


			}
			else if (bud.m_status == BudStatus::Flushed)
			{
				//Make the leaf larger
				const float maxMaturityIncrease = availableDevelopmentVigor / shootGrowthParameters.m_leafVigorRequirement;
				const float maturityIncrease = glm::min(maxMaturityIncrease, glm::min(m_currentDeltaTime * shootGrowthParameters.m_leafGrowthRate, 1.0f - bud.m_reproductiveModule.m_maturity));
				bud.m_reproductiveModule.m_maturity += maturityIncrease;
				const auto developmentVigor = bud.m_vigorSink.SubtractVigor(maturityIncrease * shootGrowthParameters.m_leafVigorRequirement);
				auto leafSize = shootGrowthParameters.m_maxLeafSize * glm::pow(bud.m_reproductiveModule.m_maturity, 1.0f / 2.0f);
				glm::quat rotation = internodeData.m_desiredLocalRotation * bud.m_localRotation;
				auto up = rotation * glm::vec3(0, 1, 0);
				auto front = rotation * glm::vec3(0, 0, -1);
				ApplyTropism(internodeData.m_lightDirection, 0.3f, up, front);
				rotation = glm::quatLookAt(front, up);
				auto foliagePosition = internodeInfo.m_globalPosition + front * (leafSize.z * 1.5f);
				bud.m_reproductiveModule.m_transform = glm::translate(foliagePosition) * glm::mat4_cast(rotation) * glm::scale(leafSize);
				//Handle leaf drop here.
				if (climateModel.m_time - glm::floor(climateModel.m_time) > 0.5f && internodeData.m_temperature < shootGrowthParameters.m_leafChlorophyllSynthesisFactorTemperature)
				{
					bud.m_reproductiveModule.m_health -= m_currentDeltaTime * shootGrowthParameters.m_leafChlorophyllLoss;
					bud.m_reproductiveModule.m_health = glm::clamp(bud.m_reproductiveModule.m_health, 0.0f, 1.0f);
				}
				if (bud.m_reproductiveModule.m_health <= 0.05f)
				{
					auto dropProbability = m_currentDeltaTime * shootGrowthParameters.m_leafFallProbability;
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

	childThicknessCollection += rootGrowthParameters.m_thicknessAccumulateAgeFactor * rootGrowthParameters.m_endNodeThickness * rootGrowthParameters.m_rootNodeGrowthRate * rootNodeData.m_age;

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

void TreeModel::AggregateInternodeVigorRequirement(const ShootGrowthParameters& shootGrowthParameters)
{
	const auto& sortedInternodeList = m_shootSkeleton.RefSortedNodeList();
	for (auto it = sortedInternodeList.rbegin(); it != sortedInternodeList.rend(); ++it) {
		auto& internode = m_shootSkeleton.RefNode(*it);
		auto& internodeData = internode.m_data;
		if (!internode.IsEndNode()) {
			//If current node is not end node
			for (const auto& i : internode.RefChildHandles()) {
				auto& childInternode = m_shootSkeleton.RefNode(i);
				internodeData.m_vigorFlow.m_subtreeVigorRequirementWeight +=
					shootGrowthParameters.m_vigorRequirementAggregateLoss *
					(childInternode.m_data.m_vigorFlow.m_vigorRequirementWeight
						+ childInternode.m_data.m_vigorFlow.m_subtreeVigorRequirementWeight);
			}
		}
	}
}

void TreeModel::AggregateRootVigorRequirement(const RootGrowthParameters& rootGrowthParameters)
{
	const auto& sortedRootNodeList = m_rootSkeleton.RefSortedNodeList();

	for (auto it = sortedRootNodeList.rbegin(); it != sortedRootNodeList.rend(); ++it) {
		auto& rootNode = m_rootSkeleton.RefNode(*it);
		auto& rootNodeData = rootNode.m_data;
		rootNodeData.m_vigorFlow.m_subtreeVigorRequirementWeight = 0.0f;
		if (!rootNode.IsEndNode()) {
			//If current node is not end node
			for (const auto& i : rootNode.RefChildHandles()) {
				const auto& childInternode = m_rootSkeleton.RefNode(i);
				rootNodeData.m_vigorFlow.m_subtreeVigorRequirementWeight +=
					rootGrowthParameters.m_vigorRequirementAggregateLoss *
					(childInternode.m_data.m_vigorFlow.m_vigorRequirementWeight + childInternode.m_data.m_vigorFlow.m_subtreeVigorRequirementWeight);
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
		* glm::exp(-shootGrowthParameters.m_apicalControlAgeFactor * m_age);
	float remainingVigor = m_shootSkeleton.m_data.m_vigor;

	const float leafMaintenanceVigor = glm::min(remainingVigor, m_shootSkeleton.m_data.m_vigorRequirement.m_leafMaintenanceVigor);
	remainingVigor -= leafMaintenanceVigor;
	float leafMaintenanceVigorFillingRate = 0.0f;
	if (m_shootSkeleton.m_data.m_vigorRequirement.m_leafMaintenanceVigor != 0.0f)
		leafMaintenanceVigorFillingRate = leafMaintenanceVigor / m_shootSkeleton.m_data.m_vigorRequirement.m_leafMaintenanceVigor;

	const float leafDevelopmentVigor = glm::min(remainingVigor, m_shootSkeleton.m_data.m_vigorRequirement.m_leafDevelopmentalVigor);
	remainingVigor -= leafDevelopmentVigor;
	float leafDevelopmentVigorFillingRate = 0.0f;
	if (m_shootSkeleton.m_data.m_vigorRequirement.m_leafDevelopmentalVigor != 0.0f)
		leafDevelopmentVigorFillingRate = leafDevelopmentVigor / m_shootSkeleton.m_data.m_vigorRequirement.m_leafDevelopmentalVigor;

	const float fruitMaintenanceVigor = glm::min(remainingVigor, m_shootSkeleton.m_data.m_vigorRequirement.m_fruitMaintenanceVigor);
	remainingVigor -= fruitMaintenanceVigor;
	float fruitMaintenanceVigorFillingRate = 0.0f;
	if (m_shootSkeleton.m_data.m_vigorRequirement.m_fruitMaintenanceVigor != 0.0f)
		fruitMaintenanceVigorFillingRate = fruitMaintenanceVigor / m_shootSkeleton.m_data.m_vigorRequirement.m_fruitMaintenanceVigor;

	const float fruitDevelopmentVigor = glm::min(remainingVigor, m_shootSkeleton.m_data.m_vigorRequirement.m_fruitDevelopmentalVigor);
	remainingVigor -= fruitDevelopmentVigor;
	float fruitDevelopmentVigorFillingRate = 0.0f;
	if (m_shootSkeleton.m_data.m_vigorRequirement.m_fruitDevelopmentalVigor != 0.0f)
		fruitDevelopmentVigorFillingRate = fruitDevelopmentVigor / m_shootSkeleton.m_data.m_vigorRequirement.m_fruitDevelopmentalVigor;

	const float nodeDevelopmentVigor = glm::min(remainingVigor, m_shootSkeleton.m_data.m_vigorRequirement.m_nodeDevelopmentalVigor);
	if (m_shootSkeleton.m_data.m_vigorRequirement.m_nodeDevelopmentalVigor != 0.0f) {
		m_internodeDevelopmentRate = nodeDevelopmentVigor / m_shootSkeleton.m_data.m_vigorRequirement.m_nodeDevelopmentalVigor;
	}

	for (const auto& internodeHandle : sortedInternodeList) {
		auto& internode = m_shootSkeleton.RefNode(internodeHandle);
		auto& internodeData = internode.m_data;
		auto& internodeVigorFlow = internodeData.m_vigorFlow;
		//1. Allocate maintenance vigor first
		for (auto& bud : internodeData.m_buds) {
			switch (bud.m_type)
			{
			case BudType::Leaf:
			{
				bud.m_vigorSink.AddVigor(leafMaintenanceVigorFillingRate * bud.m_vigorSink.GetMaintenanceVigorRequirement());
				bud.m_vigorSink.AddVigor(leafDevelopmentVigorFillingRate * bud.m_vigorSink.GetMaxVigorRequirement());
			}break;
			case BudType::Fruit:
			{
				bud.m_vigorSink.AddVigor(fruitMaintenanceVigorFillingRate * bud.m_vigorSink.GetMaintenanceVigorRequirement());
				bud.m_vigorSink.AddVigor(fruitDevelopmentVigorFillingRate * bud.m_vigorSink.GetMaxVigorRequirement());
			}break;
			default:break;
			}

		}
		//2. Allocate development vigor for structural growth
		//If this is the first node (node at the rooting point)
		if (internode.GetParentHandle() == -1) {
			internodeVigorFlow.m_allocatedVigor = 0.0f;
			internodeVigorFlow.m_subTreeAllocatedVigor = 0.0f;
			if (m_shootSkeleton.m_data.m_vigorRequirement.m_nodeDevelopmentalVigor != 0.0f) {
				const float totalRequirement = internodeVigorFlow.m_vigorRequirementWeight + internodeVigorFlow.m_subtreeVigorRequirementWeight;
				if (totalRequirement != 0.0f) {
					//The root internode firstly extract it's own resources needed for itself.
					internodeVigorFlow.m_allocatedVigor = nodeDevelopmentVigor * internodeVigorFlow.m_vigorRequirementWeight / totalRequirement;
				}
				//The rest resource will be distributed to the descendants. 
				internodeVigorFlow.m_subTreeAllocatedVigor = nodeDevelopmentVigor - internodeVigorFlow.m_allocatedVigor;
			}

		}
		//The buds will get its own resources
		for (auto& bud : internodeData.m_buds) {
			if (bud.m_type == BudType::Apical && internodeVigorFlow.m_vigorRequirementWeight != 0.0f) {
				//The vigor gets allocated and stored eventually into the buds
				const float budAllocatedVigor = internodeVigorFlow.m_allocatedVigor *
					bud.m_vigorSink.GetMaxVigorRequirement() / internodeVigorFlow.m_vigorRequirementWeight;
				bud.m_vigorSink.AddVigor(budAllocatedVigor);
			}
		}

		if (internodeVigorFlow.m_subTreeAllocatedVigor != 0.0f) {
			float childDevelopmentalVigorRequirementWeightSum = 0.0f;
			for (const auto& i : internode.RefChildHandles()) {
				const auto& childInternode = m_shootSkeleton.RefNode(i);
				const auto& childInternodeData = childInternode.m_data;
				auto& childInternodeVigorFlow = childInternodeData.m_vigorFlow;
				float childDevelopmentalVigorRequirementWeight = 0.0f;
				if (internodeVigorFlow.m_subtreeVigorRequirementWeight != 0.0f)childDevelopmentalVigorRequirementWeight = (childInternodeVigorFlow.m_vigorRequirementWeight + childInternodeVigorFlow.m_subtreeVigorRequirementWeight)
					/ internodeVigorFlow.m_subtreeVigorRequirementWeight;
				//Perform Apical control here.
				if (childInternodeData.m_isMaxChild) childDevelopmentalVigorRequirementWeight *= apicalControl;
				childDevelopmentalVigorRequirementWeightSum += childDevelopmentalVigorRequirementWeight;
			}
			for (const auto& i : internode.RefChildHandles()) {
				auto& childInternode = m_shootSkeleton.RefNode(i);
				auto& childInternodeData = childInternode.m_data;
				auto& childInternodeVigorFlow = childInternodeData.m_vigorFlow;
				float childDevelopmentalVigorRequirementWeight = 0.0f;
				if (internodeVigorFlow.m_subtreeVigorRequirementWeight != 0.0f)
					childDevelopmentalVigorRequirementWeight = (childInternodeVigorFlow.m_vigorRequirementWeight + childInternodeVigorFlow.m_subtreeVigorRequirementWeight)
					/ internodeVigorFlow.m_subtreeVigorRequirementWeight;

				//Re-perform apical control.
				if (childInternodeData.m_isMaxChild) childDevelopmentalVigorRequirementWeight *= apicalControl;

				//Calculate total amount of development vigor belongs to this child from internode received vigor for its children.
				float childTotalAllocatedDevelopmentVigor = 0.0f;
				if (childDevelopmentalVigorRequirementWeightSum != 0.0f) childTotalAllocatedDevelopmentVigor = internodeVigorFlow.m_subTreeAllocatedVigor *
					childDevelopmentalVigorRequirementWeight / childDevelopmentalVigorRequirementWeightSum;

				//Calculate allocated vigor.
				childInternodeVigorFlow.m_allocatedVigor = 0.0f;
				if (childInternodeVigorFlow.m_vigorRequirementWeight + childInternodeVigorFlow.m_subtreeVigorRequirementWeight != 0.0f) {
					childInternodeVigorFlow.m_allocatedVigor += childTotalAllocatedDevelopmentVigor *
						childInternodeVigorFlow.m_vigorRequirementWeight
						/ (childInternodeVigorFlow.m_vigorRequirementWeight + childInternodeVigorFlow.m_subtreeVigorRequirementWeight);
				}
				childInternodeVigorFlow.m_subTreeAllocatedVigor = childTotalAllocatedDevelopmentVigor - childInternodeVigorFlow.m_allocatedVigor;
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
	}
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
	childThicknessCollection += shootGrowthParameters.m_thicknessAccumulateAgeFactor * shootGrowthParameters.m_endNodeThickness * shootGrowthParameters.m_internodeGrowthRate * internodeData.m_age;


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
		internodeVigorFlow.m_vigorRequirementWeight = 0.0f;
		internodeVigorFlow.m_subtreeVigorRequirementWeight = 0.0f;
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
					bud.m_vigorSink.SetDesiredDevelopmentalVigorRequirement(m_currentDeltaTime * shootGrowthParameters.m_internodeGrowthRate * shootGrowthParameters.m_internodeVigorRequirement);
					newTreeGrowthNutrientsRequirement.m_nodeDevelopmentalVigor += bud.m_vigorSink.GetMaxVigorRequirement();
					//Collect requirement for internode. The internode doesn't has it's own requirement for now since we consider it as simple pipes
					//that only perform transportation. However this can be change in future.
					internodeVigorFlow.m_vigorRequirementWeight += bud.m_vigorSink.GetDesiredDevelopmentalVigorRequirement();
				}
			}break;
			case BudType::Leaf: {
				if (bud.m_status == BudStatus::Dormant)
				{
					//No requirement since the lateral bud only gets activated and turned into new shoot.
					//We can make use of the development vigor for bud flushing probability here in future.
					bud.m_vigorSink.SetDesiredMaintenanceVigorRequirement(0.0f);
					newTreeGrowthNutrientsRequirement.m_leafMaintenanceVigor += bud.m_vigorSink.GetMaintenanceVigorRequirement();

				}
				else if (bud.m_status == BudStatus::Flushed)
				{
					//The maintenance vigor requirement is related to the size and the drought factor of the leaf.
					bud.m_vigorSink.SetDesiredDevelopmentalVigorRequirement(shootGrowthParameters.m_leafVigorRequirement * bud.m_reproductiveModule.m_health);
					newTreeGrowthNutrientsRequirement.m_leafDevelopmentalVigor += bud.m_vigorSink.GetMaxVigorRequirement();
				}
			}break;
			case BudType::Fruit: {
				if (bud.m_status == BudStatus::Dormant)
				{
					//No requirement since the lateral bud only gets activated and turned into new shoot.
					//We can make use of the development vigor for bud flushing probability here in future.
					bud.m_vigorSink.SetDesiredMaintenanceVigorRequirement(0.0f);
					newTreeGrowthNutrientsRequirement.m_fruitDevelopmentalVigor += bud.m_vigorSink.GetMaintenanceVigorRequirement();
				}
				else if (bud.m_status == BudStatus::Flushed)
				{
					//The maintenance vigor requirement is related to the volume and the drought factor of the fruit.
					bud.m_vigorSink.SetDesiredDevelopmentalVigorRequirement(shootGrowthParameters.m_fruitVigorRequirement * bud.m_reproductiveModule.m_health);
					newTreeGrowthNutrientsRequirement.m_fruitDevelopmentalVigor += bud.m_vigorSink.GetMaxVigorRequirement();
				}
			}break;
			default: break;
			}

		}
	}
}

void TreeModel::Clear() {
	m_shootSkeleton = {};
	m_rootSkeleton = {};
	m_history = {};
	m_initialized = false;
	//m_shootVolume.Clear();

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
		float pruningProbability = m_currentDeltaTime * (1.0f - internode.m_data.m_lightIntensity) * shootGrowthParameters.m_endNodePruningLightFactor;

		if (pruningProbability >= glm::linearRand(0.0f, 1.0f)) pruning = true;
	}
	if (internode.m_info.m_globalPosition.y <= 0.5f && internode.m_data.m_order != 0 && glm::linearRand(0.0f, 1.0f) < m_currentDeltaTime * 0.1f) pruning = true;
	if (pruning)
	{
		PruneInternode(internodeHandle);
	}
	return pruning;
}

void TreeModel::SampleNitrite(const glm::mat4& globalTransform, SoilModel& soilModel)
{
	m_rootSkeleton.m_data.m_rootFlux.m_nitrite = 0.0f;
	const auto& sortedRootNodeList = m_rootSkeleton.RefSortedNodeList();
	for (const auto& rootNodeHandle : sortedRootNodeList) {
		auto& rootNode = m_rootSkeleton.RefNode(rootNodeHandle);
		auto& rootNodeInfo = rootNode.m_info;
		auto worldSpacePosition = globalTransform * glm::translate(rootNodeInfo.m_globalPosition)[3];
		if (m_treeGrowthSettings.m_collectNitrite) {
			rootNode.m_data.m_nitrite = soilModel.IntegrateNutrient(worldSpacePosition, 0.2);
			m_rootSkeleton.m_data.m_rootFlux.m_nitrite += rootNode.m_data.m_nitrite;
		}
		else
		{
			m_rootSkeleton.m_data.m_rootFlux.m_nitrite += 1.0f;
		}
	}
}

void TreeModel::AllocateRootVigor(const RootGrowthParameters& rootGrowthParameters)
{
	//For how this works, refer to AllocateShootVigor().
	const auto& sortedRootNodeList = m_rootSkeleton.RefSortedNodeList();
	const float apicalControl =
		1.0f + rootGrowthParameters.m_apicalControl
		* glm::exp(-rootGrowthParameters.m_apicalControlAgeFactor * m_age);

	const float rootMaintenanceVigor = glm::min(m_rootSkeleton.m_data.m_vigor, m_rootSkeleton.m_data.m_vigorRequirement.m_leafDevelopmentalVigor);
	float rootMaintenanceVigorFillingRate = 0.0f;
	if (m_rootSkeleton.m_data.m_vigorRequirement.m_leafDevelopmentalVigor != 0.0f) rootMaintenanceVigorFillingRate = rootMaintenanceVigor / m_rootSkeleton.m_data.m_vigorRequirement.m_leafDevelopmentalVigor;
	const float rootDevelopmentVigor = m_rootSkeleton.m_data.m_vigor - rootMaintenanceVigor;
	if (m_rootSkeleton.m_data.m_vigorRequirement.m_nodeDevelopmentalVigor != 0.0f) {
		m_rootNodeDevelopmentRate = rootDevelopmentVigor / m_rootSkeleton.m_data.m_vigorRequirement.m_nodeDevelopmentalVigor;
	}

	for (const auto& rootNodeHandle : sortedRootNodeList) {
		auto& rootNode = m_rootSkeleton.RefNode(rootNodeHandle);
		auto& rootNodeData = rootNode.m_data;
		auto& rootNodeVigorFlow = rootNodeData.m_vigorFlow;

		rootNodeData.m_vigorSink.AddVigor(rootMaintenanceVigorFillingRate * rootNodeData.m_vigorSink.GetMaintenanceVigorRequirement());

		if (rootNode.GetParentHandle() == -1) {
			if (m_rootSkeleton.m_data.m_vigorRequirement.m_nodeDevelopmentalVigor != 0.0f) {
				const float totalRequirement = rootNodeVigorFlow.m_vigorRequirementWeight + rootNodeVigorFlow.m_subtreeVigorRequirementWeight;
				if (totalRequirement != 0.0f) {
					rootNodeVigorFlow.m_allocatedVigor = rootDevelopmentVigor * rootNodeVigorFlow.m_vigorRequirementWeight / totalRequirement;
				}
				rootNodeData.m_vigorSink.AddVigor(rootNodeVigorFlow.m_allocatedVigor);
				rootNodeVigorFlow.m_subTreeAllocatedVigor = rootDevelopmentVigor - rootNodeVigorFlow.m_allocatedVigor;
			}
		}
		rootNodeData.m_vigorSink.AddVigor(rootNodeVigorFlow.m_allocatedVigor);

		if (rootNodeVigorFlow.m_subTreeAllocatedVigor != 0.0f) {
			float childDevelopmentalVigorRequirementWeightSum = 0.0f;
			for (const auto& i : rootNode.RefChildHandles()) {
				const auto& childRootNode = m_rootSkeleton.RefNode(i);
				const auto& childRootNodeData = childRootNode.m_data;
				const auto& childRootNodeVigorFlow = childRootNodeData.m_vigorFlow;
				float childDevelopmentalVigorRequirementWeight = 0.0f;
				if (rootNodeVigorFlow.m_subtreeVigorRequirementWeight != 0.0f) {
					childDevelopmentalVigorRequirementWeight =
						(childRootNodeVigorFlow.m_vigorRequirementWeight + childRootNodeVigorFlow.m_subtreeVigorRequirementWeight)
						/ rootNodeVigorFlow.m_subtreeVigorRequirementWeight;
				}
				//Perform Apical control here.
				if (rootNodeData.m_isMaxChild) childDevelopmentalVigorRequirementWeight *= apicalControl;
				childDevelopmentalVigorRequirementWeightSum += childDevelopmentalVigorRequirementWeight;
			}
			for (const auto& i : rootNode.RefChildHandles()) {
				auto& childRootNode = m_rootSkeleton.RefNode(i);
				auto& childRootNodeData = childRootNode.m_data;
				auto& childRootNodeVigorFlow = childRootNodeData.m_vigorFlow;
				float childDevelopmentalVigorRequirementWeight = 0.0f;
				if (rootNodeVigorFlow.m_subtreeVigorRequirementWeight != 0.0f)
					childDevelopmentalVigorRequirementWeight = (childRootNodeVigorFlow.m_vigorRequirementWeight + childRootNodeVigorFlow.m_subtreeVigorRequirementWeight)
					/ rootNodeVigorFlow.m_subtreeVigorRequirementWeight;

				//Perform Apical control here.
				if (rootNodeData.m_isMaxChild) childDevelopmentalVigorRequirementWeight *= apicalControl;

				//Then calculate total amount of development vigor belongs to this child from internode received vigor for its children.
				float childTotalAllocatedDevelopmentVigor = 0.0f;
				if (childDevelopmentalVigorRequirementWeightSum != 0.0f) childTotalAllocatedDevelopmentVigor = rootNodeVigorFlow.m_subTreeAllocatedVigor *
					childDevelopmentalVigorRequirementWeight / childDevelopmentalVigorRequirementWeightSum;

				childRootNodeVigorFlow.m_allocatedVigor = 0.0f;
				if (childRootNodeVigorFlow.m_vigorRequirementWeight + childRootNodeVigorFlow.m_subtreeVigorRequirementWeight != 0.0f) {
					childRootNodeVigorFlow.m_allocatedVigor += childTotalAllocatedDevelopmentVigor *
						childRootNodeVigorFlow.m_vigorRequirementWeight
						/ (childRootNodeVigorFlow.m_vigorRequirementWeight + childRootNodeVigorFlow.m_subtreeVigorRequirementWeight);
				}
				childRootNodeVigorFlow.m_subTreeAllocatedVigor = childTotalAllocatedDevelopmentVigor - childRootNodeVigorFlow.m_allocatedVigor;
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
	}
}

void TreeModel::CalculateVigorRequirement(const RootGrowthParameters& rootGrowthParameters,
	PlantGrowthRequirement& newRootGrowthNutrientsRequirement)
{
	const auto& sortedRootNodeList = m_rootSkeleton.RefSortedNodeList();
	for (const auto& rootNodeHandle : sortedRootNodeList) {
		auto& rootNode = m_rootSkeleton.RefNode(rootNodeHandle);
		auto& rootNodeData = rootNode.m_data;
		auto& rootNodeVigorFlow = rootNodeData.m_vigorFlow;
		//This one has 0 always but we will put value in it in future.
		rootNodeVigorFlow.m_vigorRequirementWeight = 0.0f;
		float growthPotential = 0.0f;
		if (rootNode.RefChildHandles().empty())
		{
			growthPotential = m_currentDeltaTime * rootGrowthParameters.m_rootNodeGrowthRate * rootGrowthParameters.m_rootNodeVigorRequirement;
			rootNodeVigorFlow.m_vigorRequirementWeight =
				rootNodeData.m_nitrite * growthPotential *
				glm::pow(1.0f / glm::max(rootNodeData.m_soilDensity * rootGrowthParameters.m_environmentalFriction, 1.0f), rootGrowthParameters.m_environmentalFrictionFactor);
		}
		rootNodeData.m_vigorSink.SetDesiredMaintenanceVigorRequirement(0.0f);
		rootNodeData.m_vigorSink.SetDesiredDevelopmentalVigorRequirement(rootNodeVigorFlow.m_vigorRequirementWeight);
		//We sum the vigor requirement with the developmentalVigorRequirement,
		//so the overall nitrite will not affect the root growth. Thus we will have same growth rate for low/high nitrite density.
		newRootGrowthNutrientsRequirement.m_leafDevelopmentalVigor += 0.0f;
		newRootGrowthNutrientsRequirement.m_nodeDevelopmentalVigor += growthPotential;
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

	m_fruitFallProbability = 3.0f;
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
