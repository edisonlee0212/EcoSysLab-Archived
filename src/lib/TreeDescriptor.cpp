//
// Created by lllll on 10/24/2022.
//

#include "Tree.hpp"

#include <Material.hpp>
#include <Mesh.hpp>
#include "Strands.hpp"
#include "EditorLayer.hpp"
#include "Application.hpp"
#include "TreeMeshGenerator.hpp"
#include "Soil.hpp"
#include "Climate.hpp"
#include "Octree.hpp"
#include "EcoSysLabLayer.hpp"
#include "HeightField.hpp"
#include "StrandsRenderer.hpp"
#include "TreeDescriptor.hpp"

#include "BranchShape.hpp"
using namespace EcoSysLab;

void TreeDescriptor::OnCreate() {

}
bool TreeDescriptor::OnInspectFoliageParameters(FoliageParameters& foliageParameters) {
	bool changed = false;
	if (ImGui::TreeNodeEx("Foliage Parameters", ImGuiTreeNodeFlags_DefaultOpen))
	{
		if (ImGui::DragFloat2("Leaf size", &foliageParameters.m_leafSize.x, 0.01f, 0.0f, 1.0f)) changed = true;
		if (ImGui::ColorEdit3("Leaf color", &foliageParameters.m_leafColor.x)) changed = true;
		if (ImGui::DragInt("Leaf per node", &foliageParameters.m_leafCountPerInternode, 1, 0, 50)) changed = true;
		if (ImGui::DragFloat("Position variance", &foliageParameters.m_positionVariance, 0.01f, 0.0f, 1.0f)) changed = true;
		if (ImGui::DragFloat("Rotation variance", &foliageParameters.m_rotationVariance, 0.01f, 0.0f, 1.0f)) changed = true;
		if (ImGui::DragFloat("Branching angle", &foliageParameters.m_branchingAngle, 0.01f, 0.0f, 1.0f)) changed = true;
		if (ImGui::DragFloat("Max node thickness", &foliageParameters.m_maxNodeThickness, 0.001f, 0.0f, 5.0f)) changed = true;
		if (ImGui::DragFloat("Min root distance", &foliageParameters.m_minRootDistance, 0.001f, 0.0f, 1.0f)) changed = true;
		if (ImGui::DragFloat("Max end distance", &foliageParameters.m_maxEndDistance, 0.001f, 0.0f, 1.0f)) changed = true;
		ImGui::TreePop();
	}
	return changed;
}

bool TreeDescriptor::OnInspectShootGrowthParameters(ShootGrowthParameters& treeGrowthParameters) {
	bool changed = false;
	if (ImGui::TreeNodeEx("Shoot Parameters", ImGuiTreeNodeFlags_DefaultOpen)) {
		changed = ImGui::DragFloat("Internode growth rate", &treeGrowthParameters.m_internodeGrowthRate, 0.01f, 0.0f, 1.0f) || changed;
		changed = ImGui::DragFloat("Leaf growth rate", &treeGrowthParameters.m_leafGrowthRate, 0.01f, 0.0f, 1.0f) || changed;
		changed = ImGui::DragFloat("Fruit growth rate", &treeGrowthParameters.m_fruitGrowthRate, 0.01f, 0.0f, 1.0f) || changed;

		if (ImGui::TreeNodeEx("Bud", ImGuiTreeNodeFlags_DefaultOpen)) {
			changed = ImGui::DragInt3("Bud count lateral/fruit/leaf", &treeGrowthParameters.m_lateralBudCount, 1, 0, 3) || changed;
			changed = ImGui::DragFloat2("Branching Angle mean/var", &treeGrowthParameters.m_branchingAngleMeanVariance.x, 0.01f, 0.0f, 100.0f) || changed;
			changed = ImGui::DragFloat2("Roll Angle mean/var", &treeGrowthParameters.m_rollAngleMeanVariance.x, 0.01f, 0.0f, 100.0f) || changed;
			changed = ImGui::DragFloat2("Apical Angle mean/var", &treeGrowthParameters.m_apicalAngleMeanVariance.x, 0.01f, 0.0f, 100.0f) || changed;
			changed = ImGui::DragFloat("Gravitropism", &treeGrowthParameters.m_gravitropism, 0.01f) || changed;
			changed = ImGui::DragFloat("Phototropism", &treeGrowthParameters.m_phototropism, 0.01f) || changed;
			changed = ImGui::DragFloat("Apical bud kill prob", &treeGrowthParameters.m_apicalInternodeKillProbability, 0.00001f, 0.0f, 1.0f, "%.5f") || changed;
			changed = ImGui::DragFloat("Lateral bud flushing prob", &treeGrowthParameters.m_lateralBudFlushProbability, 0.00001f, 0.0f, 1.0f, "%.5f") || changed;
			changed = ImGui::DragFloat4("Leaf flushing prob/temp range", &treeGrowthParameters.m_leafBudFlushingProbabilityTemperatureRange.x, 0.00001f, 0.0f, 1.0f, "%.5f") || changed;
			changed = ImGui::DragFloat4("Fruit flushing prob/temp range", &treeGrowthParameters.m_fruitBudFlushingProbabilityTemperatureRange.x, 0.00001f, 0.0f, 1.0f, "%.5f") || changed;
			changed = ImGui::DragFloat2("Apical control base/age", &treeGrowthParameters.m_apicalControl, 0.01f) || changed;
			changed = ImGui::DragFloat3("Apical dominance base/age/dist", &treeGrowthParameters.m_apicalDominance, 0.01f) || changed;
			changed = ImGui::DragFloat3("Vigor requirement shoot/leaf/fruit", &treeGrowthParameters.m_internodeVigorRequirement, 0.01f) || changed;
			changed = ImGui::DragFloat("Vigor requirement aggregation loss", &treeGrowthParameters.m_vigorRequirementAggregateLoss, 0.001f, 0.0f, 1.0f) || changed;
			ImGui::TreePop();
		}
		if (ImGui::TreeNodeEx("Internode", ImGuiTreeNodeFlags_DefaultOpen))
		{
			changed = ImGui::DragFloat("Internode length", &treeGrowthParameters.m_internodeLength, 0.01f) || changed;
			changed = ImGui::DragFloat("Internode length thickness factor", &treeGrowthParameters.m_internodeLengthThicknessFactor, 0.01f, 0.0f, 1.0f) || changed;
			changed = ImGui::DragFloat3("Thickness min/factor/age", &treeGrowthParameters.m_endNodeThickness, 0.00001f, 0.0f, 1.0f, "%.6f") || changed;
			changed = ImGui::DragFloat3("Sagging thickness/reduction/max", &treeGrowthParameters.m_saggingFactorThicknessReductionMax.x, 0.01f, 0.0f, 1.0f, "%.5f") || changed;
			changed = ImGui::DragFloat("Low Branch Pruning", &treeGrowthParameters.m_lowBranchPruning, 0.01f) || changed;
			changed = ImGui::DragFloat("Low Branch Pruning Thickness factor", &treeGrowthParameters.m_lowBranchPruningThicknessFactor, 0.01f) || changed;
			changed = ImGui::DragFloat("Light Pruning", &treeGrowthParameters.m_lightPruningFactor, 0.01f) || changed;
			changed = ImGui::DragFloat("Thin branch Pruning", &treeGrowthParameters.m_thicknessPruningFactor, 0.01f, 0.0f) || changed;
			changed = ImGui::DragFloat("Max Space Occupancy", &treeGrowthParameters.m_maxSpaceOccupancy, 0.001f, 0.0f, 1.0f, "%.3f") || changed;

			ImGui::TreePop();
		}

		if (ImGui::TreeNodeEx("Leaf"))
		{
			changed = ImGui::DragFloat3("Size", &treeGrowthParameters.m_maxLeafSize.x, 0.01f) || changed;
			changed = ImGui::DragFloat("Position Variance", &treeGrowthParameters.m_leafPositionVariance, 0.01f) || changed;
			changed = ImGui::DragFloat("Random rotation", &treeGrowthParameters.m_leafRotationVariance, 0.01f) || changed;
			changed = ImGui::DragFloat("Chlorophyll Loss", &treeGrowthParameters.m_leafChlorophyllLoss, 0.01f) || changed;
			changed = ImGui::DragFloat("Chlorophyll temperature", &treeGrowthParameters.m_leafChlorophyllSynthesisFactorTemperature, 0.01f) || changed;
			changed = ImGui::DragFloat("Drop prob", &treeGrowthParameters.m_leafFallProbability, 0.01f) || changed;
			changed = ImGui::DragFloat("Distance To End Limit", &treeGrowthParameters.m_leafDistanceToBranchEndLimit, 0.01f) || changed;
			ImGui::TreePop();
		}
		if (ImGui::TreeNodeEx("Fruit"))
		{
			changed = ImGui::DragFloat3("Size", &treeGrowthParameters.m_maxFruitSize.x, 0.01f) || changed;
			changed = ImGui::DragFloat("Position Variance", &treeGrowthParameters.m_fruitPositionVariance, 0.01f) || changed;
			changed = ImGui::DragFloat("Random rotation", &treeGrowthParameters.m_fruitRotationVariance, 0.01f) || changed;

			changed = ImGui::DragFloat("Drop prob", &treeGrowthParameters.m_fruitFallProbability, 0.01f) || changed;
			ImGui::TreePop();
		}
		ImGui::TreePop();
	}

	return changed;
}

bool TreeDescriptor::OnInspectRootGrowthParameters(RootGrowthParameters& rootGrowthParameters) {
	bool changed = false;
	if (ImGui::TreeNodeEx("Root Growth Parameters")) {
		if (ImGui::TreeNodeEx("Structure", ImGuiTreeNodeFlags_DefaultOpen)) {
			changed = ImGui::DragFloat("Root node length", &rootGrowthParameters.m_rootNodeLength, 0.01f) || changed;
			changed = ImGui::DragFloat("Root node elongation rate", &rootGrowthParameters.m_rootNodeGrowthRate, 0.01f) || changed;

			changed = ImGui::DragFloat3("Thickness min/factor/age", &rootGrowthParameters.m_endNodeThickness, 0.00001f, 0.0f, 1.0f, "%.6f") || changed;

			ImGui::TreePop();
		}
		if (ImGui::TreeNodeEx("Growth", ImGuiTreeNodeFlags_DefaultOpen))
		{
			changed = ImGui::DragFloat("Root node elongation vigor requirement", &rootGrowthParameters.m_rootNodeVigorRequirement, 0.01f) || changed;
			changed = ImGui::DragFloat("Vigor requirement aggregation loss", &rootGrowthParameters.m_vigorRequirementAggregateLoss, 0.001f, 0.0f, 1.0f) || changed;

			changed = ImGui::DragFloat2("Branching Angle mean/var", &rootGrowthParameters.m_branchingAngleMeanVariance.x, 0.01f) || changed;
			changed = ImGui::DragFloat2("Roll Angle mean/var", &rootGrowthParameters.m_rollAngleMeanVariance.x, 0.01f) || changed;
			changed = ImGui::DragFloat2("Apical Angle mean/var", &rootGrowthParameters.m_apicalAngleMeanVariance.x, 0.01f) || changed;
			changed = ImGui::DragFloat2("Environmental friction base/factor", &rootGrowthParameters.m_environmentalFriction, 0.01f) || changed;
			changed = ImGui::DragFloat2("Apical control base/age", &rootGrowthParameters.m_apicalControl, 0.01f) || changed;
			changed = ImGui::DragFloat3("Apical dominance base/age/dist", &rootGrowthParameters.m_apicalDominance, 0.01f) || changed;
			changed = ImGui::DragFloat("Tropism switching prob", &rootGrowthParameters.m_tropismSwitchingProbability, 0.01f) || changed;
			changed = ImGui::DragFloat("Tropism switching prob dist factor", &rootGrowthParameters.m_tropismSwitchingProbabilityDistanceFactor, 0.01f) || changed;
			changed = ImGui::DragFloat("Tropism intensity", &rootGrowthParameters.m_tropismIntensity, 0.01f) || changed;
			changed = ImGui::DragFloat("Branching probability", &rootGrowthParameters.m_branchingProbability, 0.00001f, 0.0f, 1.0f, "%.5f") || changed;
			ImGui::TreePop();
		}
		ImGui::TreePop();
	}
	return changed;
}

void TreeDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
	bool changed = false;
	const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
	const auto soil = ecoSysLabLayer->m_soilHolder.Get<Soil>();
	const auto climate = ecoSysLabLayer->m_climateHolder.Get<Climate>();
	if (soil && climate) {
		if (ImGui::Button("Instantiate")) {
			const auto scene = Application::GetActiveScene();
			const auto treeEntity = scene->CreateEntity(GetTitle());
			const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
			float height = 0;
			const auto soilDescriptor = soil->m_soilDescriptor.Get<SoilDescriptor>();
			if (soilDescriptor)
			{
				const auto heightField = soilDescriptor->m_heightField.Get<HeightField>();
				if (heightField) height = heightField->GetValue({ 0.0f, 0.0f }) - 0.05f;
			}
			GlobalTransform globalTransform;
			globalTransform.SetPosition(glm::vec3(0, height, 0));
			scene->SetDataComponent(treeEntity, globalTransform);
			tree->m_treeDescriptor = ProjectManager::GetAsset(GetHandle());
		}
	}
	else
	{
		ImGui::Text("Attach soil and climate entity to instantiate!");
	}
	if (OnInspectShootGrowthParameters(m_shootGrowthParameters)) { changed = true; }
	if (OnInspectRootGrowthParameters(m_rootGrowthParameters)) { changed = true; }
	if (OnInspectFoliageParameters(m_foliageParameters)) { changed = true; }
	if (ImGui::TreeNode("Foliage Parameters")) {
		editorLayer->DragAndDropButton<Texture2D>(m_foliageAlbedoTexture, "Foliage Albedo Texture");
		editorLayer->DragAndDropButton<Texture2D>(m_foliageNormalTexture, "Foliage Normal Texture");
		editorLayer->DragAndDropButton<Texture2D>(m_foliageRoughnessTexture, "Foliage Roughness Texture");
		editorLayer->DragAndDropButton<Texture2D>(m_foliageMetallicTexture, "Foliage Metallic Texture");
	}
	if (ImGui::TreeNode("Branch Parameters")) {
		editorLayer->DragAndDropButton<Texture2D>(m_branchAlbedoTexture, "Branch Albedo Texture");
		editorLayer->DragAndDropButton<Texture2D>(m_branchNormalTexture, "Branch Normal Texture");
		editorLayer->DragAndDropButton<Texture2D>(m_branchRoughnessTexture, "Branch Roughness Texture");
		editorLayer->DragAndDropButton<Texture2D>(m_branchMetallicTexture, "Branch Metallic Texture");
	}

	editorLayer->DragAndDropButton<BranchShape>(m_shootBranchShape, "Shoot Branch Shape");
	editorLayer->DragAndDropButton<BranchShape>(m_rootBranchShape, "Root Branch Shape");
	if (changed) m_saved = false;
}

void TreeDescriptor::CollectAssetRef(std::vector<AssetRef>& list) {
	if (m_foliageAlbedoTexture.Get<Texture2D>()) list.push_back(m_foliageAlbedoTexture);
	if (m_foliageNormalTexture.Get<Texture2D>()) list.push_back(m_foliageNormalTexture);
	if (m_foliageRoughnessTexture.Get<Texture2D>()) list.push_back(m_foliageRoughnessTexture);
	if (m_foliageMetallicTexture.Get<Texture2D>()) list.push_back(m_foliageMetallicTexture);

	if (m_branchAlbedoTexture.Get<Texture2D>()) list.push_back(m_branchAlbedoTexture);
	if (m_branchNormalTexture.Get<Texture2D>()) list.push_back(m_branchNormalTexture);
	if (m_branchRoughnessTexture.Get<Texture2D>()) list.push_back(m_branchRoughnessTexture);
	if (m_branchMetallicTexture.Get<Texture2D>()) list.push_back(m_branchMetallicTexture);

	if (m_shootBranchShape.Get<BranchShape>()) list.push_back(m_shootBranchShape);
	if (m_rootBranchShape.Get<BranchShape>()) list.push_back(m_rootBranchShape);
}
void TreeDescriptor::SerializeFoliageParameters(const std::string& name, const FoliageParameters& foliageParameters, YAML::Emitter& out)
{
	out << YAML::Key << name << YAML::BeginMap;
	out << YAML::Key << "m_leafSize" << YAML::Value << foliageParameters.m_leafSize;
	out << YAML::Key << "m_leafColor" << YAML::Value << foliageParameters.m_leafColor;
	out << YAML::Key << "m_leafCountPerInternode" << YAML::Value << foliageParameters.m_leafCountPerInternode;
	out << YAML::Key << "m_positionVariance" << YAML::Value << foliageParameters.m_positionVariance;
	out << YAML::Key << "m_rotationVariance" << YAML::Value << foliageParameters.m_rotationVariance;
	out << YAML::Key << "m_branchingAngle" << YAML::Value << foliageParameters.m_branchingAngle;
	out << YAML::Key << "m_maxNodeThickness" << YAML::Value << foliageParameters.m_maxNodeThickness;
	out << YAML::Key << "m_minRootDistance" << YAML::Value << foliageParameters.m_minRootDistance;
	out << YAML::Key << "m_maxEndDistance" << YAML::Value << foliageParameters.m_maxEndDistance;
	out << YAML::EndMap;
}
void TreeDescriptor::SerializeShootGrowthParameters(const std::string& name, const ShootGrowthParameters& treeGrowthParameters, YAML::Emitter& out) {
	out << YAML::Key << name << YAML::BeginMap;
	out << YAML::Key << "m_internodeGrowthRate" << YAML::Value << treeGrowthParameters.m_internodeGrowthRate;
	out << YAML::Key << "m_leafGrowthRate" << YAML::Value << treeGrowthParameters.m_leafGrowthRate;
	out << YAML::Key << "m_fruitGrowthRate" << YAML::Value << treeGrowthParameters.m_fruitGrowthRate;
	//Structure
	out << YAML::Key << "m_lateralBudCount" << YAML::Value << treeGrowthParameters.m_lateralBudCount;
	out << YAML::Key << "m_fruitBudCount" << YAML::Value << treeGrowthParameters.m_fruitBudCount;
	out << YAML::Key << "m_leafBudCount" << YAML::Value << treeGrowthParameters.m_leafBudCount;
	out << YAML::Key << "m_branchingAngleMeanVariance" << YAML::Value << treeGrowthParameters.m_branchingAngleMeanVariance;
	out << YAML::Key << "m_rollAngleMeanVariance" << YAML::Value << treeGrowthParameters.m_rollAngleMeanVariance;
	out << YAML::Key << "m_apicalAngleMeanVariance" << YAML::Value << treeGrowthParameters.m_apicalAngleMeanVariance;
	out << YAML::Key << "m_gravitropism" << YAML::Value << treeGrowthParameters.m_gravitropism;
	out << YAML::Key << "m_phototropism" << YAML::Value << treeGrowthParameters.m_phototropism;
	out << YAML::Key << "m_internodeLength" << YAML::Value << treeGrowthParameters.m_internodeLength;

	out << YAML::Key << "m_endNodeThickness" << YAML::Value << treeGrowthParameters.m_endNodeThickness;
	out << YAML::Key << "m_thicknessAccumulationFactor" << YAML::Value << treeGrowthParameters.m_thicknessAccumulationFactor;
	out << YAML::Key << "m_thicknessAccumulateAgeFactor" << YAML::Value << treeGrowthParameters.m_thicknessAccumulateAgeFactor;

	//Bud
	out << YAML::Key << "m_apicalInternodeKillProbability" << YAML::Value << treeGrowthParameters.m_apicalInternodeKillProbability;
	out << YAML::Key << "m_lateralBudFlushProbability" << YAML::Value << treeGrowthParameters.m_lateralBudFlushProbability;
	out << YAML::Key << "m_leafBudFlushingProbabilityTemperatureRange" << YAML::Value << treeGrowthParameters.m_leafBudFlushingProbabilityTemperatureRange;
	out << YAML::Key << "m_fruitBudFlushingProbabilityTemperatureRange" << YAML::Value << treeGrowthParameters.m_fruitBudFlushingProbabilityTemperatureRange;

	out << YAML::Key << "m_apicalControl" << YAML::Value << treeGrowthParameters.m_apicalControl;
	out << YAML::Key << "m_apicalControlAgeFactor" << YAML::Value << treeGrowthParameters.m_apicalControlAgeFactor;

	out << YAML::Key << "m_apicalDominance" << YAML::Value << treeGrowthParameters.m_apicalDominance;
	out << YAML::Key << "m_apicalDominanceAgeFactor" << YAML::Value << treeGrowthParameters.m_apicalDominanceAgeFactor;
	out << YAML::Key << "m_apicalDominanceDistanceFactor" << YAML::Value << treeGrowthParameters.m_apicalDominanceDistanceFactor;

	out << YAML::Key << "m_leafVigorRequirement" << YAML::Value << treeGrowthParameters.m_leafVigorRequirement;
	out << YAML::Key << "m_fruitVigorRequirement" << YAML::Value << treeGrowthParameters.m_fruitVigorRequirement;
	out << YAML::Key << "m_internodeVigorRequirement" << YAML::Value << treeGrowthParameters.m_internodeVigorRequirement;
	out << YAML::Key << "m_vigorRequirementAggregateLoss" << YAML::Value << treeGrowthParameters.m_vigorRequirementAggregateLoss;
	//Internode
	out << YAML::Key << "m_lowBranchPruning" << YAML::Value << treeGrowthParameters.m_lowBranchPruning;
	out << YAML::Key << "m_lowBranchPruningThicknessFactor" << YAML::Value << treeGrowthParameters.m_lowBranchPruningThicknessFactor;
	out << YAML::Key << "m_lightPruningFactor" << YAML::Value << treeGrowthParameters.m_lightPruningFactor;
	out << YAML::Key << "m_thicknessPruningFactor" << YAML::Value << treeGrowthParameters.m_thicknessPruningFactor;
	out << YAML::Key << "m_saggingFactorThicknessReductionMax" << YAML::Value << treeGrowthParameters.m_saggingFactorThicknessReductionMax;
	out << YAML::Key << "m_maxSpaceOccupancy" << YAML::Value << treeGrowthParameters.m_maxSpaceOccupancy;

	//Foliage
	out << YAML::Key << "m_maxLeafSize" << YAML::Value << treeGrowthParameters.m_maxLeafSize;
	out << YAML::Key << "m_leafPositionVariance" << YAML::Value << treeGrowthParameters.m_leafPositionVariance;
	out << YAML::Key << "m_leafRotationVariance" << YAML::Value << treeGrowthParameters.m_leafRotationVariance;
	out << YAML::Key << "m_leafChlorophyllLoss" << YAML::Value << treeGrowthParameters.m_leafChlorophyllLoss;
	out << YAML::Key << "m_leafChlorophyllSynthesisFactorTemperature" << YAML::Value << treeGrowthParameters.m_leafChlorophyllSynthesisFactorTemperature;
	out << YAML::Key << "m_leafFallProbability" << YAML::Value << treeGrowthParameters.m_leafFallProbability;
	out << YAML::Key << "m_leafDistanceToBranchEndLimit" << YAML::Value << treeGrowthParameters.m_leafDistanceToBranchEndLimit;

	out << YAML::Key << "m_maxFruitSize" << YAML::Value << treeGrowthParameters.m_maxFruitSize;
	out << YAML::Key << "m_fruitPositionVariance" << YAML::Value << treeGrowthParameters.m_fruitPositionVariance;
	out << YAML::Key << "m_fruitRotationVariance" << YAML::Value << treeGrowthParameters.m_fruitRotationVariance;
	out << YAML::Key << "m_fruitFallProbability" << YAML::Value << treeGrowthParameters.m_fruitFallProbability;
	out << YAML::EndMap;
}
void TreeDescriptor::SerializeRootGrowthParameters(const std::string& name, const RootGrowthParameters& rootGrowthParameters, YAML::Emitter& out) {
	out << YAML::Key << name << YAML::BeginMap;

	out << YAML::Key << "m_branchingAngleMeanVariance" << YAML::Value << rootGrowthParameters.m_branchingAngleMeanVariance;
	out << YAML::Key << "m_rollAngleMeanVariance" << YAML::Value << rootGrowthParameters.m_rollAngleMeanVariance;
	out << YAML::Key << "m_apicalAngleMeanVariance" << YAML::Value << rootGrowthParameters.m_apicalAngleMeanVariance;
	out << YAML::Key << "m_rootNodeLength" << YAML::Value << rootGrowthParameters.m_rootNodeLength;
	out << YAML::Key << "m_rootNodeGrowthRate" << YAML::Value << rootGrowthParameters.m_rootNodeGrowthRate;
	out << YAML::Key << "m_endNodeThickness" << YAML::Value << rootGrowthParameters.m_endNodeThickness;
	out << YAML::Key << "m_thicknessAccumulationFactor" << YAML::Value << rootGrowthParameters.m_thicknessAccumulationFactor;
	out << YAML::Key << "m_thicknessAccumulateAgeFactor" << YAML::Value << rootGrowthParameters.m_thicknessAccumulateAgeFactor;


	out << YAML::Key << "m_rootNodeVigorRequirement" << YAML::Value << rootGrowthParameters.m_rootNodeVigorRequirement;
	out << YAML::Key << "m_vigorRequirementAggregateLoss" << YAML::Value << rootGrowthParameters.m_vigorRequirementAggregateLoss;
	out << YAML::Key << "m_environmentalFriction" << YAML::Value << rootGrowthParameters.m_environmentalFriction;
	out << YAML::Key << "m_environmentalFrictionFactor" << YAML::Value << rootGrowthParameters.m_environmentalFrictionFactor;


	out << YAML::Key << "m_apicalControl" << YAML::Value << rootGrowthParameters.m_apicalControl;
	out << YAML::Key << "m_apicalControlAgeFactor" << YAML::Value << rootGrowthParameters.m_apicalControlAgeFactor;
	out << YAML::Key << "m_apicalDominance" << YAML::Value << rootGrowthParameters.m_apicalDominance;
	out << YAML::Key << "m_apicalDominanceAgeFactor" << YAML::Value << rootGrowthParameters.m_apicalDominanceAgeFactor;
	out << YAML::Key << "m_apicalDominanceDistanceFactor" << YAML::Value << rootGrowthParameters.m_apicalDominanceDistanceFactor;
	out << YAML::Key << "m_tropismSwitchingProbability" << YAML::Value << rootGrowthParameters.m_tropismSwitchingProbability;
	out << YAML::Key << "m_tropismSwitchingProbabilityDistanceFactor" << YAML::Value << rootGrowthParameters.m_tropismSwitchingProbabilityDistanceFactor;
	out << YAML::Key << "m_tropismIntensity" << YAML::Value << rootGrowthParameters.m_tropismIntensity;
	out << YAML::Key << "m_branchingProbability" << YAML::Value << rootGrowthParameters.m_branchingProbability;

	out << YAML::EndMap;
}
void TreeDescriptor::Serialize(YAML::Emitter& out) {
	SerializeShootGrowthParameters("m_shootGrowthParameters", m_shootGrowthParameters, out);
	SerializeRootGrowthParameters("m_rootGrowthParameters", m_rootGrowthParameters, out);
	SerializeFoliageParameters("m_foliageParameters", m_foliageParameters, out);

	m_foliageAlbedoTexture.Save("m_foliageAlbedoTexture", out);
	m_foliageNormalTexture.Save("m_foliageNormalTexture", out);
	m_foliageRoughnessTexture.Save("m_foliageRoughnessTexture", out);
	m_foliageMetallicTexture.Save("m_foliageMetallicTexture", out);

	m_branchAlbedoTexture.Save("m_branchAlbedoTexture", out);
	m_branchNormalTexture.Save("m_branchNormalTexture", out);
	m_branchRoughnessTexture.Save("m_branchRoughnessTexture", out);
	m_branchMetallicTexture.Save("m_branchMetallicTexture", out);

	m_shootBranchShape.Save("m_shootBranchShape", out);
	m_rootBranchShape.Save("m_rootBranchShape", out);
}
void TreeDescriptor::DeserializeFoliageParameters(const std::string& name, FoliageParameters& foliageParameters, const YAML::Node& in) {
	if (in[name]) {
		auto& param = in[name];
		if (param["m_leafSize"]) foliageParameters.m_leafSize = param["m_leafSize"].as<glm::vec2>();
		if (param["m_leafColor"]) foliageParameters.m_leafColor = param["m_leafColor"].as<glm::vec3>();
		if (param["m_leafCountPerInternode"]) foliageParameters.m_leafCountPerInternode = param["m_leafCountPerInternode"].as<int>();
		if (param["m_positionVariance"]) foliageParameters.m_positionVariance = param["m_positionVariance"].as<float>();
		if (param["m_rotationVariance"]) foliageParameters.m_rotationVariance = param["m_rotationVariance"].as<float>();
		if (param["m_branchingAngle"]) foliageParameters.m_branchingAngle = param["m_branchingAngle"].as<float>();
		if (param["m_maxNodeThickness"]) foliageParameters.m_maxNodeThickness = param["m_maxNodeThickness"].as<float>();
		if (param["m_minRootDistance"]) foliageParameters.m_minRootDistance = param["m_minRootDistance"].as<float>();
		if (param["m_maxEndDistance"]) foliageParameters.m_maxEndDistance = param["m_maxEndDistance"].as<float>();
	}
}
void TreeDescriptor::DeserializeShootGrowthParameters(const std::string& name, ShootGrowthParameters& treeGrowthParameters, const YAML::Node& in) {
	if (in[name]) {
		auto& param = in[name];

		if (param["m_internodeGrowthRate"]) treeGrowthParameters.m_internodeGrowthRate = param["m_internodeGrowthRate"].as<float>();
		if (param["m_leafGrowthRate"]) treeGrowthParameters.m_leafGrowthRate = param["m_leafGrowthRate"].as<float>();
		if (param["m_fruitGrowthRate"]) treeGrowthParameters.m_fruitGrowthRate = param["m_fruitGrowthRate"].as<float>();
		//Structure
		if (param["m_lateralBudCount"]) treeGrowthParameters.m_lateralBudCount = param["m_lateralBudCount"].as<int>();
		if (param["m_fruitBudCount"]) treeGrowthParameters.m_fruitBudCount = param["m_fruitBudCount"].as<int>();
		if (param["m_leafBudCount"]) treeGrowthParameters.m_leafBudCount = param["m_leafBudCount"].as<int>();

		if (param["m_branchingAngleMeanVariance"]) treeGrowthParameters.m_branchingAngleMeanVariance = param["m_branchingAngleMeanVariance"].as<glm::vec2>();
		if (param["m_rollAngleMeanVariance"]) treeGrowthParameters.m_rollAngleMeanVariance = param["m_rollAngleMeanVariance"].as<glm::vec2>();
		if (param["m_apicalAngleMeanVariance"]) treeGrowthParameters.m_apicalAngleMeanVariance = param["m_apicalAngleMeanVariance"].as<glm::vec2>();
		if (param["m_gravitropism"]) treeGrowthParameters.m_gravitropism = param["m_gravitropism"].as<float>();
		if (param["m_phototropism"]) treeGrowthParameters.m_phototropism = param["m_phototropism"].as<float>();

		if (param["m_internodeLength"]) treeGrowthParameters.m_internodeLength = param["m_internodeLength"].as<float>();
		if (param["m_internodeLengthThicknessFactor"]) treeGrowthParameters.m_internodeLengthThicknessFactor = param["m_internodeLengthThicknessFactor"].as<float>();
		if (param["m_endNodeThickness"]) treeGrowthParameters.m_endNodeThickness = param["m_endNodeThickness"].as<float>();
		if (param["m_thicknessAccumulationFactor"]) treeGrowthParameters.m_thicknessAccumulationFactor = param["m_thicknessAccumulationFactor"].as<float>();
		if (param["m_thicknessAccumulateAgeFactor"]) treeGrowthParameters.m_thicknessAccumulateAgeFactor = param["m_thicknessAccumulateAgeFactor"].as<float>();

		if (param["m_lowBranchPruning"]) treeGrowthParameters.m_lowBranchPruning = param["m_lowBranchPruning"].as<float>();
		if (param["m_lowBranchPruningThicknessFactor"]) treeGrowthParameters.m_lowBranchPruningThicknessFactor = param["m_lowBranchPruningThicknessFactor"].as<float>();
		if (param["m_lightPruningFactor"]) treeGrowthParameters.m_lightPruningFactor = param["m_lightPruningFactor"].as<float>();
		if (param["m_thicknessPruningFactor"]) treeGrowthParameters.m_thicknessPruningFactor = param["m_thicknessPruningFactor"].as<float>();
		if (param["m_saggingFactorThicknessReductionMax"]) treeGrowthParameters.m_saggingFactorThicknessReductionMax = param["m_saggingFactorThicknessReductionMax"].as<glm::vec3>();
		if (param["m_maxSpaceOccupancy"]) treeGrowthParameters.m_maxSpaceOccupancy = param["m_maxSpaceOccupancy"].as<float>();

		//Bud fate

		if (param["m_lateralBudFlushingProbabilityTemperatureRange"]) treeGrowthParameters.m_lateralBudFlushProbability = param["m_lateralBudFlushingProbabilityTemperatureRange"].as<glm::vec4>().x;
		if (param["m_apicalInternodeKillProbability"]) treeGrowthParameters.m_apicalInternodeKillProbability = param["m_apicalInternodeKillProbability"].as<float>();
		if (param["m_lateralBudFlushProbability"]) treeGrowthParameters.m_lateralBudFlushProbability = param["m_lateralBudFlushProbability"].as<float>();
		if (param["m_leafBudFlushingProbabilityTemperatureRange"]) treeGrowthParameters.m_leafBudFlushingProbabilityTemperatureRange = param["m_leafBudFlushingProbabilityTemperatureRange"].as< glm::vec4>();
		if (param["m_fruitBudFlushingProbabilityTemperatureRange"]) treeGrowthParameters.m_fruitBudFlushingProbabilityTemperatureRange = param["m_fruitBudFlushingProbabilityTemperatureRange"].as<glm::vec4>();

		if (param["m_apicalControl"]) treeGrowthParameters.m_apicalControl = param["m_apicalControl"].as<float>();
		if (param["m_apicalControlAgeFactor"]) treeGrowthParameters.m_apicalControlAgeFactor = param["m_apicalControlAgeFactor"].as<float>();
		if (param["m_apicalDominance"]) treeGrowthParameters.m_apicalDominance = param["m_apicalDominance"].as<float>();
		if (param["m_apicalDominanceAgeFactor"]) treeGrowthParameters.m_apicalDominanceAgeFactor = param["m_apicalDominanceAgeFactor"].as<float>();
		if (param["m_apicalDominanceDistanceFactor"]) treeGrowthParameters.m_apicalDominanceDistanceFactor = param["m_apicalDominanceDistanceFactor"].as<float>();

		if (param["m_leafVigorRequirement"]) treeGrowthParameters.m_leafVigorRequirement = param["m_leafVigorRequirement"].as<float>();
		if (param["m_fruitVigorRequirement"]) treeGrowthParameters.m_fruitVigorRequirement = param["m_fruitVigorRequirement"].as<float>();
		if (param["m_internodeVigorRequirement"]) treeGrowthParameters.m_internodeVigorRequirement = param["m_internodeVigorRequirement"].as<float>();
		if (param["m_vigorRequirementAggregateLoss"]) treeGrowthParameters.m_vigorRequirementAggregateLoss = param["m_vigorRequirementAggregateLoss"].as<float>();

		//Foliage
		if (param["m_maxLeafSize"]) treeGrowthParameters.m_maxLeafSize = param["m_maxLeafSize"].as<glm::vec3>();
		if (param["m_leafPositionVariance"]) treeGrowthParameters.m_leafPositionVariance = param["m_leafPositionVariance"].as<float>();
		if (param["m_leafRotationVariance"]) treeGrowthParameters.m_leafRotationVariance = param["m_leafRotationVariance"].as<float>();
		if (param["m_leafChlorophyllLoss"]) treeGrowthParameters.m_leafChlorophyllLoss = param["m_leafChlorophyllLoss"].as<float>();
		if (param["m_leafChlorophyllSynthesisFactorTemperature"]) treeGrowthParameters.m_leafChlorophyllSynthesisFactorTemperature = param["m_leafChlorophyllSynthesisFactorTemperature"].as<float>();
		if (param["m_leafFallProbability"]) treeGrowthParameters.m_leafFallProbability = param["m_leafFallProbability"].as<float>();
		if (param["m_leafDistanceToBranchEndLimit"]) treeGrowthParameters.m_leafDistanceToBranchEndLimit = param["m_leafDistanceToBranchEndLimit"].as<float>();

		//Fruit
		if (param["m_maxFruitSize"]) treeGrowthParameters.m_maxFruitSize = param["m_maxFruitSize"].as<glm::vec3>();
		if (param["m_fruitPositionVariance"]) treeGrowthParameters.m_fruitPositionVariance = param["m_fruitPositionVariance"].as<float>();
		if (param["m_fruitRotationVariance"]) treeGrowthParameters.m_fruitRotationVariance = param["m_fruitRotationVariance"].as<float>();
		if (param["m_fruitFallProbability"]) treeGrowthParameters.m_fruitFallProbability = param["m_fruitFallProbability"].as<float>();
	}
}
void TreeDescriptor::DeserializeRootGrowthParameters(const std::string& name, RootGrowthParameters& rootGrowthParameters, const YAML::Node& in) {
	if (in[name]) {
		auto& param = in[name];
		if (param["m_rootNodeLength"]) rootGrowthParameters.m_rootNodeLength = param["m_rootNodeLength"].as<float>();
		if (param["m_rootNodeGrowthRate"]) rootGrowthParameters.m_rootNodeGrowthRate = param["m_rootNodeGrowthRate"].as<float>();
		if (param["m_endNodeThickness"]) rootGrowthParameters.m_endNodeThickness = param["m_endNodeThickness"].as<float>();
		if (param["m_thicknessAccumulationFactor"]) rootGrowthParameters.m_thicknessAccumulationFactor = param["m_thicknessAccumulationFactor"].as<float>();
		if (param["m_thicknessAccumulateAgeFactor"]) rootGrowthParameters.m_thicknessAccumulateAgeFactor = param["m_thicknessAccumulateAgeFactor"].as<float>();

		if (param["m_branchingAngleMeanVariance"]) rootGrowthParameters.m_branchingAngleMeanVariance = param["m_branchingAngleMeanVariance"].as<glm::vec2>();
		if (param["m_rollAngleMeanVariance"]) rootGrowthParameters.m_rollAngleMeanVariance = param["m_rollAngleMeanVariance"].as<glm::vec2>();
		if (param["m_apicalAngleMeanVariance"]) rootGrowthParameters.m_apicalAngleMeanVariance = param["m_apicalAngleMeanVariance"].as<glm::vec2>();

		if (param["m_rootNodeVigorRequirement"]) rootGrowthParameters.m_rootNodeVigorRequirement = param["m_rootNodeVigorRequirement"].as<float>();
		if (param["m_vigorRequirementAggregateLoss"]) rootGrowthParameters.m_vigorRequirementAggregateLoss = param["m_vigorRequirementAggregateLoss"].as<float>();

		if (param["m_environmentalFriction"]) rootGrowthParameters.m_environmentalFriction = param["m_environmentalFriction"].as<float>();
		if (param["m_environmentalFrictionFactor"]) rootGrowthParameters.m_environmentalFrictionFactor = param["m_environmentalFrictionFactor"].as<float>();

		if (param["m_apicalControl"]) rootGrowthParameters.m_apicalControl = param["m_apicalControl"].as<float>();
		if (param["m_apicalControlAgeFactor"]) rootGrowthParameters.m_apicalControlAgeFactor = param["m_apicalControlAgeFactor"].as<float>();
		if (param["m_apicalDominance"]) rootGrowthParameters.m_apicalDominance = param["m_apicalDominance"].as<float>();
		if (param["m_apicalDominanceAgeFactor"]) rootGrowthParameters.m_apicalDominanceAgeFactor = param["m_apicalDominanceAgeFactor"].as<float>();
		if (param["m_apicalDominanceDistanceFactor"]) rootGrowthParameters.m_apicalDominanceDistanceFactor = param["m_apicalDominanceDistanceFactor"].as<float>();

		if (param["m_tropismSwitchingProbability"]) rootGrowthParameters.m_tropismSwitchingProbability = param["m_tropismSwitchingProbability"].as<float>();
		if (param["m_tropismSwitchingProbabilityDistanceFactor"]) rootGrowthParameters.m_tropismSwitchingProbabilityDistanceFactor = param["m_tropismSwitchingProbabilityDistanceFactor"].as<float>();
		if (param["m_tropismIntensity"]) rootGrowthParameters.m_tropismIntensity = param["m_tropismIntensity"].as<float>();

		if (param["m_branchingProbability"]) rootGrowthParameters.m_branchingProbability = param["m_branchingProbability"].as<float>();
	}
}
void TreeDescriptor::Deserialize(const YAML::Node& in) {
	DeserializeShootGrowthParameters("m_shootGrowthParameters", m_shootGrowthParameters, in);
	DeserializeRootGrowthParameters("m_rootGrowthParameters", m_rootGrowthParameters, in);

	DeserializeFoliageParameters("m_foliageParameters", m_foliageParameters, in);

	m_foliageAlbedoTexture.Load("m_foliageAlbedoTexture", in);
	m_foliageNormalTexture.Load("m_foliageNormalTexture", in);
	m_foliageRoughnessTexture.Load("m_foliageRoughnessTexture", in);
	m_foliageMetallicTexture.Load("m_foliageMetallicTexture", in);

	m_branchAlbedoTexture.Load("m_branchAlbedoTexture", in);
	m_branchNormalTexture.Load("m_branchNormalTexture", in);
	m_branchRoughnessTexture.Load("m_branchRoughnessTexture", in);
	m_branchMetallicTexture.Load("m_branchMetallicTexture", in);

	m_shootBranchShape.Load("m_shootBranchShape", in);
	m_rootBranchShape.Load("m_rootBranchShape", in);
}