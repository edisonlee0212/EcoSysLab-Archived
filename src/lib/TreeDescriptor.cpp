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
bool TreeDescriptor::OnInspectFoliageDescriptor(FoliageDescriptor& foliageDescriptor) {
	bool changed = false;
	if (ImGui::TreeNodeEx("Foliage Parameters"))
	{
		if (ImGui::DragFloat2("Leaf size", &foliageDescriptor.m_leafSize.x, 0.01f, 0.0f, 1.0f)) changed = true;
		if (ImGui::ColorEdit3("Leaf color", &foliageDescriptor.m_leafColor.x)) changed = true;
		if (ImGui::DragInt("Leaf per node", &foliageDescriptor.m_leafCountPerInternode, 1, 0, 50)) changed = true;
		if (ImGui::DragFloat("Position variance", &foliageDescriptor.m_positionVariance, 0.01f, 0.0f, 1.0f)) changed = true;
		if (ImGui::DragFloat("Rotation variance", &foliageDescriptor.m_rotationVariance, 0.01f, 0.0f, 1.0f)) changed = true;
		if (ImGui::DragFloat("Branching angle", &foliageDescriptor.m_branchingAngle, 0.01f, 0.0f, 1.0f)) changed = true;
		if (ImGui::DragFloat("Max node thickness", &foliageDescriptor.m_maxNodeThickness, 0.001f, 0.0f, 5.0f)) changed = true;
		if (ImGui::DragFloat("Min root distance", &foliageDescriptor.m_minRootDistance, 0.01f, 0.0f, 10.0f)) changed = true;
		if (ImGui::DragFloat("Max end distance", &foliageDescriptor.m_maxEndDistance, 0.01f, 0.0f, 10.0f)) changed = true;
		ImGui::TreePop();
	}
	return changed;
}

bool TreeDescriptor::OnInspectShootDescriptor(ShootDescriptor& shootDescriptor) {
	bool changed = false;
	if (ImGui::TreeNodeEx("Shoot Parameters", ImGuiTreeNodeFlags_DefaultOpen)) {
		changed = ImGui::DragFloat("Growth rate", &shootDescriptor.m_growthRate, 0.01f, 0.0f, 10.0f) || changed;
		if (ImGui::TreeNodeEx("Internode", ImGuiTreeNodeFlags_DefaultOpen))
		{
			changed = ImGui::DragInt("Lateral bud count", &shootDescriptor.m_lateralBudCount, 1, 0, 3) || changed;
			changed = ImGui::DragFloat2("Branching angle mean/var", &shootDescriptor.m_branchingAngleMeanVariance.x, 0.1f, 0.0f, 100.0f) || changed;
			changed = ImGui::DragFloat2("Roll angle mean/var", &shootDescriptor.m_rollAngleMeanVariance.x, 0.1f, 0.0f, 100.0f) || changed;
			changed = ImGui::DragFloat2("Apical angle variance", &shootDescriptor.m_apicalAngleMeanVariance.x, 0.1f, 0.0f, 100.0f) || changed;
			changed = ImGui::DragFloat("Internode length", &shootDescriptor.m_internodeLength, 0.001f) || changed;
			changed = ImGui::DragFloat("Internode length thickness factor", &shootDescriptor.m_internodeLengthThicknessFactor, 0.0001f, 0.0f, 1.0f) || changed;
			changed = ImGui::DragFloat3("Thickness min/factor/age", &shootDescriptor.m_endNodeThickness, 0.0001f, 0.0f, 1.0f, "%.6f") || changed;

			changed = ImGui::DragFloat("Sagging strength", &shootDescriptor.m_saggingFactorThicknessReductionMax.x, 0.0001f, 0.0f, 10.0f, "%.5f") || changed;
			changed = ImGui::DragFloat("Sagging thickness factor", &shootDescriptor.m_saggingFactorThicknessReductionMax.y, 0.01f, 0.0f, 10.0f, "%.5f") || changed;
			changed = ImGui::DragFloat("Sagging max", &shootDescriptor.m_saggingFactorThicknessReductionMax.z, 0.001f, 0.0f, 1.0f, "%.5f") || changed;

			
			changed = ImGui::DragFloat("Internode shadow factor", &shootDescriptor.m_internodeShadowFactor, 0.001f, 0.0f, 1.0f) || changed;
			ImGui::TreePop();
		}
		if (ImGui::TreeNodeEx("Bud fate", ImGuiTreeNodeFlags_DefaultOpen)) {
			changed = ImGui::DragFloat("Gravitropism", &shootDescriptor.m_gravitropism, 0.01f) || changed;
			changed = ImGui::DragFloat("Phototropism", &shootDescriptor.m_phototropism, 0.01f) || changed;
			changed = ImGui::DragFloat("Horizontal Tropism", &shootDescriptor.m_horizontalTropism, 0.01f) || changed;

			changed = ImGui::DragFloat("Apical bud extinction rate", &shootDescriptor.m_apicalBudExtinctionRate, 0.00001f, 0.0f, 1.0f, "%.5f") || changed;
			changed = ImGui::DragFloat("Lateral bud flushing rate", &shootDescriptor.m_lateralBudFlushingRate, 0.00001f, 0.0f, 1.0f, "%.5f") || changed;
			changed = ImGui::DragFloat("Apical control", &shootDescriptor.m_apicalControl, 0.01f) || changed;
			changed = ImGui::DragFloat2("Inhibitor val/loss", &shootDescriptor.m_apicalDominance, 0.01f) || changed;
			ImGui::TreePop();
		}
		if (ImGui::TreeNodeEx("Pruning", ImGuiTreeNodeFlags_DefaultOpen))
		{
			changed = ImGui::DragFloat("Low branch pruning", &shootDescriptor.m_lowBranchPruning, 0.01f) || changed;
			if(shootDescriptor.m_lowBranchPruning > 0.0f) changed = ImGui::DragFloat("Low branch pruning thickness factor", &shootDescriptor.m_lowBranchPruningThicknessFactor, 0.01f) || changed;
			changed = ImGui::DragFloat("Light pruning", &shootDescriptor.m_lightPruningFactor, 0.01f) || changed;
			changed = ImGui::DragFloat("Thin branch pruning", &shootDescriptor.m_thicknessPruningFactor, 0.01f, 0.0f) || changed;
			ImGui::TreePop();
		}
		changed = ImGui::DragInt("Leaf bud count", &shootDescriptor.m_leafBudCount, 1, 0, 3) || changed;
		if (shootDescriptor.m_leafBudCount > 0 && ImGui::TreeNodeEx("Leaf"))
		{
			changed = ImGui::DragFloat("Leaf growth rate", &shootDescriptor.m_leafGrowthRate, 0.01f, 0.0f, 1.0f) || changed;
			changed = ImGui::DragFloat4("Leaf flushing prob/temp range", &shootDescriptor.m_leafBudFlushingProbabilityTemperatureRange.x, 0.00001f, 0.0f, 1.0f, "%.5f") || changed;
			changed = ImGui::DragFloat3("Leaf vigor requirement", &shootDescriptor.m_leafVigorRequirement, 0.01f) || changed;
			changed = ImGui::DragFloat3("Size", &shootDescriptor.m_maxLeafSize.x, 0.01f) || changed;
			changed = ImGui::DragFloat("Position Variance", &shootDescriptor.m_leafPositionVariance, 0.01f) || changed;
			changed = ImGui::DragFloat("Random rotation", &shootDescriptor.m_leafRotationVariance, 0.01f) || changed;
			changed = ImGui::DragFloat("Chlorophyll Loss", &shootDescriptor.m_leafChlorophyllLoss, 0.01f) || changed;
			changed = ImGui::DragFloat("Chlorophyll temperature", &shootDescriptor.m_leafChlorophyllSynthesisFactorTemperature, 0.01f) || changed;
			changed = ImGui::DragFloat("Drop prob", &shootDescriptor.m_leafFallProbability, 0.01f) || changed;
			changed = ImGui::DragFloat("Distance To End Limit", &shootDescriptor.m_leafDistanceToBranchEndLimit, 0.01f) || changed;
			ImGui::TreePop();
		}
		changed = ImGui::DragInt("Fruit bud count", &shootDescriptor.m_fruitBudCount, 1, 0, 3) || changed;
		if (shootDescriptor.m_fruitBudCount > 0 && ImGui::TreeNodeEx("Fruit"))
		{
			changed = ImGui::DragFloat("Fruit growth rate", &shootDescriptor.m_fruitGrowthRate, 0.01f, 0.0f, 1.0f) || changed;
			changed = ImGui::DragFloat4("Fruit flushing prob/temp range", &shootDescriptor.m_fruitBudFlushingProbabilityTemperatureRange.x, 0.00001f, 0.0f, 1.0f, "%.5f") || changed;
			changed = ImGui::DragFloat3("Fruit vigor requirement", &shootDescriptor.m_fruitVigorRequirement, 0.01f) || changed;
			changed = ImGui::DragFloat3("Size", &shootDescriptor.m_maxFruitSize.x, 0.01f) || changed;
			changed = ImGui::DragFloat("Position Variance", &shootDescriptor.m_fruitPositionVariance, 0.01f) || changed;
			changed = ImGui::DragFloat("Random rotation", &shootDescriptor.m_fruitRotationVariance, 0.01f) || changed;

			changed = ImGui::DragFloat("Drop prob", &shootDescriptor.m_fruitFallProbability, 0.01f) || changed;
			ImGui::TreePop();
		}
		ImGui::TreePop();
	}
	return changed;
}


void TreeDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
	bool changed = false;
	const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
	std::shared_ptr<Climate> climate;
	std::shared_ptr<Soil> soil;
	const auto climateCandidate = EcoSysLabLayer::FindClimate();
	if (!climateCandidate.expired()) climate = climateCandidate.lock();
	const auto soilCandidate = EcoSysLabLayer::FindSoil();
	if (!soilCandidate.expired()) soil = soilCandidate.lock();
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
			editorLayer->SetSelectedEntity(treeEntity);
		}
	}
	else
	{
		ImGui::Text("Create soil and climate entity to instantiate!");
	}
	if (OnInspectShootDescriptor(m_shootDescriptor)) { changed = true; }
	if (OnInspectFoliageDescriptor(m_foliageDescriptor)) { changed = true; }
	if (ImGui::TreeNode("Foliage Parameters")) {
		editorLayer->DragAndDropButton<Texture2D>(m_foliageAlbedoTexture, "Foliage Albedo Texture##FFT");
		editorLayer->DragAndDropButton<Texture2D>(m_foliageNormalTexture, "Foliage Normal Texture##FNT");
		editorLayer->DragAndDropButton<Texture2D>(m_foliageRoughnessTexture, "Foliage Roughness Texture##FRT");
		editorLayer->DragAndDropButton<Texture2D>(m_foliageMetallicTexture, "Foliage Metallic Texture##FMT");
		ImGui::TreePop();
	}
	if (ImGui::TreeNode("Branch Parameters")) {
		editorLayer->DragAndDropButton<Texture2D>(m_branchAlbedoTexture, "Branch Albedo Texture##BFT");
		editorLayer->DragAndDropButton<Texture2D>(m_branchNormalTexture, "Branch Normal Texture##BNT");
		editorLayer->DragAndDropButton<Texture2D>(m_branchRoughnessTexture, "Branch Roughness Texture##BRT");
		editorLayer->DragAndDropButton<Texture2D>(m_branchMetallicTexture, "Branch Metallic Texture##BMT");
		ImGui::TreePop();
	}

	editorLayer->DragAndDropButton<BranchShape>(m_shootBranchShape, "Shoot Branch Shape##SBS");
	editorLayer->DragAndDropButton<BranchShape>(m_rootBranchShape, "Root Branch Shape##RBS");
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
void TreeDescriptor::SerializeFoliageDescriptor(const std::string& name, const FoliageDescriptor& foliageDescriptor, YAML::Emitter& out)
{
	out << YAML::Key << name << YAML::BeginMap;
	out << YAML::Key << "m_leafSize" << YAML::Value << foliageDescriptor.m_leafSize;
	out << YAML::Key << "m_leafColor" << YAML::Value << foliageDescriptor.m_leafColor;
	out << YAML::Key << "m_leafCountPerInternode" << YAML::Value << foliageDescriptor.m_leafCountPerInternode;
	out << YAML::Key << "m_positionVariance" << YAML::Value << foliageDescriptor.m_positionVariance;
	out << YAML::Key << "m_rotationVariance" << YAML::Value << foliageDescriptor.m_rotationVariance;
	out << YAML::Key << "m_branchingAngle" << YAML::Value << foliageDescriptor.m_branchingAngle;
	out << YAML::Key << "m_maxNodeThickness" << YAML::Value << foliageDescriptor.m_maxNodeThickness;
	out << YAML::Key << "m_minRootDistance" << YAML::Value << foliageDescriptor.m_minRootDistance;
	out << YAML::Key << "m_maxEndDistance" << YAML::Value << foliageDescriptor.m_maxEndDistance;
	out << YAML::EndMap;
}
void TreeDescriptor::SerializeShootDescriptor(const std::string& name, const ShootDescriptor& shootDescriptor, YAML::Emitter& out) {
	out << YAML::Key << name << YAML::BeginMap;
	out << YAML::Key << "m_growthRate" << YAML::Value << shootDescriptor.m_growthRate;
	out << YAML::Key << "m_branchingAngleMeanVariance" << YAML::Value << shootDescriptor.m_branchingAngleMeanVariance;
	out << YAML::Key << "m_rollAngleMeanVariance" << YAML::Value << shootDescriptor.m_rollAngleMeanVariance;
	out << YAML::Key << "m_apicalAngleMeanVariance" << YAML::Value << shootDescriptor.m_apicalAngleMeanVariance;
	out << YAML::Key << "m_gravitropism" << YAML::Value << shootDescriptor.m_gravitropism;
	out << YAML::Key << "m_phototropism" << YAML::Value << shootDescriptor.m_phototropism;
	out << YAML::Key << "m_horizontalTropism" << YAML::Value << shootDescriptor.m_horizontalTropism;
	out << YAML::Key << "m_saggingFactorThicknessReductionMax" << YAML::Value << shootDescriptor.m_saggingFactorThicknessReductionMax;
	out << YAML::Key << "m_internodeLength" << YAML::Value << shootDescriptor.m_internodeLength;
	out << YAML::Key << "m_internodeLengthThicknessFactor" << YAML::Value << shootDescriptor.m_internodeLengthThicknessFactor;
	out << YAML::Key << "m_endNodeThickness" << YAML::Value << shootDescriptor.m_endNodeThickness;
	out << YAML::Key << "m_thicknessAccumulationFactor" << YAML::Value << shootDescriptor.m_thicknessAccumulationFactor;
	out << YAML::Key << "m_thicknessAgeFactor" << YAML::Value << shootDescriptor.m_thicknessAgeFactor;
	out << YAML::Key << "m_internodeShadowFactor" << YAML::Value << shootDescriptor.m_internodeShadowFactor;

	out << YAML::Key << "m_lateralBudCount" << YAML::Value << shootDescriptor.m_lateralBudCount;
	out << YAML::Key << "m_apicalBudExtinctionRate" << YAML::Value << shootDescriptor.m_apicalBudExtinctionRate;
	out << YAML::Key << "m_lateralBudFlushingRate" << YAML::Value << shootDescriptor.m_lateralBudFlushingRate;
	out << YAML::Key << "m_apicalControl" << YAML::Value << shootDescriptor.m_apicalControl;
	out << YAML::Key << "m_apicalDominance" << YAML::Value << shootDescriptor.m_apicalDominance;
	out << YAML::Key << "m_apicalDominanceLoss" << YAML::Value << shootDescriptor.m_apicalDominanceLoss;

	out << YAML::Key << "m_lowBranchPruning" << YAML::Value << shootDescriptor.m_lowBranchPruning;
	out << YAML::Key << "m_lowBranchPruningThicknessFactor" << YAML::Value << shootDescriptor.m_lowBranchPruningThicknessFactor;
	out << YAML::Key << "m_lightPruningFactor" << YAML::Value << shootDescriptor.m_lightPruningFactor;
	out << YAML::Key << "m_thicknessPruningFactor" << YAML::Value << shootDescriptor.m_thicknessPruningFactor;

	out << YAML::Key << "m_leafBudCount" << YAML::Value << shootDescriptor.m_leafBudCount;
	out << YAML::Key << "m_leafGrowthRate" << YAML::Value << shootDescriptor.m_leafGrowthRate;
	out << YAML::Key << "m_leafBudFlushingProbabilityTemperatureRange" << YAML::Value << shootDescriptor.m_leafBudFlushingProbabilityTemperatureRange;
	out << YAML::Key << "m_leafVigorRequirement" << YAML::Value << shootDescriptor.m_leafVigorRequirement;
	out << YAML::Key << "m_maxLeafSize" << YAML::Value << shootDescriptor.m_maxLeafSize;
	out << YAML::Key << "m_leafPositionVariance" << YAML::Value << shootDescriptor.m_leafPositionVariance;
	out << YAML::Key << "m_leafRotationVariance" << YAML::Value << shootDescriptor.m_leafRotationVariance;
	out << YAML::Key << "m_leafChlorophyllLoss" << YAML::Value << shootDescriptor.m_leafChlorophyllLoss;
	out << YAML::Key << "m_leafChlorophyllSynthesisFactorTemperature" << YAML::Value << shootDescriptor.m_leafChlorophyllSynthesisFactorTemperature;
	out << YAML::Key << "m_leafFallProbability" << YAML::Value << shootDescriptor.m_leafFallProbability;
	out << YAML::Key << "m_leafDistanceToBranchEndLimit" << YAML::Value << shootDescriptor.m_leafDistanceToBranchEndLimit;

	out << YAML::Key << "m_fruitBudCount" << YAML::Value << shootDescriptor.m_fruitBudCount;
	out << YAML::Key << "m_fruitGrowthRate" << YAML::Value << shootDescriptor.m_fruitGrowthRate;
	out << YAML::Key << "m_fruitBudFlushingProbabilityTemperatureRange" << YAML::Value << shootDescriptor.m_fruitBudFlushingProbabilityTemperatureRange;
	out << YAML::Key << "m_fruitVigorRequirement" << YAML::Value << shootDescriptor.m_fruitVigorRequirement;
	out << YAML::Key << "m_maxFruitSize" << YAML::Value << shootDescriptor.m_maxFruitSize;
	out << YAML::Key << "m_fruitPositionVariance" << YAML::Value << shootDescriptor.m_fruitPositionVariance;
	out << YAML::Key << "m_fruitRotationVariance" << YAML::Value << shootDescriptor.m_fruitRotationVariance;
	out << YAML::Key << "m_fruitFallProbability" << YAML::Value << shootDescriptor.m_fruitFallProbability;
	
	out << YAML::EndMap;
}
void TreeDescriptor::Serialize(YAML::Emitter& out) {
	SerializeShootDescriptor("m_shootDescriptor", m_shootDescriptor, out);
	SerializeFoliageDescriptor("m_foliageDescriptor", m_foliageDescriptor, out);

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
void TreeDescriptor::DeserializeFoliageDescriptor(const std::string& name, FoliageDescriptor& foliageDescriptor, const YAML::Node& in) {
	if (in[name]) {
		auto& param = in[name];
		if (param["m_leafSize"]) foliageDescriptor.m_leafSize = param["m_leafSize"].as<glm::vec2>();
		if (param["m_leafColor"]) foliageDescriptor.m_leafColor = param["m_leafColor"].as<glm::vec3>();
		if (param["m_leafCountPerInternode"]) foliageDescriptor.m_leafCountPerInternode = param["m_leafCountPerInternode"].as<int>();
		if (param["m_positionVariance"]) foliageDescriptor.m_positionVariance = param["m_positionVariance"].as<float>();
		if (param["m_rotationVariance"]) foliageDescriptor.m_rotationVariance = param["m_rotationVariance"].as<float>();
		if (param["m_branchingAngle"]) foliageDescriptor.m_branchingAngle = param["m_branchingAngle"].as<float>();
		if (param["m_maxNodeThickness"]) foliageDescriptor.m_maxNodeThickness = param["m_maxNodeThickness"].as<float>();
		if (param["m_minRootDistance"]) foliageDescriptor.m_minRootDistance = param["m_minRootDistance"].as<float>();
		if (param["m_maxEndDistance"]) foliageDescriptor.m_maxEndDistance = param["m_maxEndDistance"].as<float>();
	}
}
void TreeDescriptor::DeserializeShootDescriptor(const std::string& name, ShootDescriptor& shootDescriptor, const YAML::Node& in) {
	if (in[name]) {
		auto& param = in[name];

		if (param["m_growthRate"]) shootDescriptor.m_growthRate = param["m_growthRate"].as<float>();
		if (param["m_branchingAngleMeanVariance"]) shootDescriptor.m_branchingAngleMeanVariance = param["m_branchingAngleMeanVariance"].as<glm::vec2>();
		if (param["m_rollAngleMeanVariance"]) shootDescriptor.m_rollAngleMeanVariance = param["m_rollAngleMeanVariance"].as<glm::vec2>();
		if (param["m_apicalAngleMeanVariance"]) shootDescriptor.m_apicalAngleMeanVariance = param["m_apicalAngleMeanVariance"].as<glm::vec2>();
		if (param["m_gravitropism"]) shootDescriptor.m_gravitropism = param["m_gravitropism"].as<float>();
		if (param["m_phototropism"]) shootDescriptor.m_phototropism = param["m_phototropism"].as<float>();
		if (param["m_horizontalTropism"]) shootDescriptor.m_horizontalTropism = param["m_horizontalTropism"].as<float>();
		if (param["m_saggingFactorThicknessReductionMax"]) shootDescriptor.m_saggingFactorThicknessReductionMax = param["m_saggingFactorThicknessReductionMax"].as<glm::vec3>();
		if (param["m_internodeLength"]) shootDescriptor.m_internodeLength = param["m_internodeLength"].as<float>();
		if (param["m_internodeLengthThicknessFactor"]) shootDescriptor.m_internodeLengthThicknessFactor = param["m_internodeLengthThicknessFactor"].as<float>();
		if (param["m_endNodeThickness"]) shootDescriptor.m_endNodeThickness = param["m_endNodeThickness"].as<float>();
		if (param["m_thicknessAccumulationFactor"]) shootDescriptor.m_thicknessAccumulationFactor = param["m_thicknessAccumulationFactor"].as<float>();
		if (param["m_thicknessAgeFactor"]) shootDescriptor.m_thicknessAgeFactor = param["m_thicknessAgeFactor"].as<float>();
		if (param["m_internodeShadowFactor"]) shootDescriptor.m_internodeShadowFactor = param["m_internodeShadowFactor"].as<float>();


		if (param["m_lateralBudCount"]) shootDescriptor.m_lateralBudCount = param["m_lateralBudCount"].as<int>();
		if (param["m_apicalBudExtinctionRate"]) shootDescriptor.m_apicalBudExtinctionRate = param["m_apicalBudExtinctionRate"].as<float>();
		if (param["m_lateralBudFlushingRate"]) shootDescriptor.m_lateralBudFlushingRate = param["m_lateralBudFlushingRate"].as<float>();
		if (param["m_apicalControl"]) shootDescriptor.m_apicalControl = param["m_apicalControl"].as<float>();
		if (param["m_apicalDominance"]) shootDescriptor.m_apicalDominance = param["m_apicalDominance"].as<float>();
		if (param["m_apicalDominanceLoss"]) shootDescriptor.m_apicalDominanceLoss = param["m_apicalDominanceLoss"].as<float>();

		if (param["m_lowBranchPruning"]) shootDescriptor.m_lowBranchPruning = param["m_lowBranchPruning"].as<float>();
		if (param["m_lowBranchPruningThicknessFactor"]) shootDescriptor.m_lowBranchPruningThicknessFactor = param["m_lowBranchPruningThicknessFactor"].as<float>();
		if (param["m_lightPruningFactor"]) shootDescriptor.m_lightPruningFactor = param["m_lightPruningFactor"].as<float>();
		if (param["m_thicknessPruningFactor"]) shootDescriptor.m_thicknessPruningFactor = param["m_thicknessPruningFactor"].as<float>();

		if (param["m_leafBudCount"]) shootDescriptor.m_leafBudCount = param["m_leafBudCount"].as<int>();
		if (param["m_leafGrowthRate"]) shootDescriptor.m_leafGrowthRate = param["m_leafGrowthRate"].as<float>();
		if (param["m_leafBudFlushingProbabilityTemperatureRange"]) shootDescriptor.m_leafBudFlushingProbabilityTemperatureRange = param["m_leafBudFlushingProbabilityTemperatureRange"].as< glm::vec4>();
		if (param["m_leafVigorRequirement"]) shootDescriptor.m_leafVigorRequirement = param["m_leafVigorRequirement"].as<float>();
		if (param["m_maxLeafSize"]) shootDescriptor.m_maxLeafSize = param["m_maxLeafSize"].as<glm::vec3>();
		if (param["m_leafPositionVariance"]) shootDescriptor.m_leafPositionVariance = param["m_leafPositionVariance"].as<float>();
		if (param["m_leafRotationVariance"]) shootDescriptor.m_leafRotationVariance = param["m_leafRotationVariance"].as<float>();
		if (param["m_leafChlorophyllLoss"]) shootDescriptor.m_leafChlorophyllLoss = param["m_leafChlorophyllLoss"].as<float>();
		if (param["m_leafChlorophyllSynthesisFactorTemperature"]) shootDescriptor.m_leafChlorophyllSynthesisFactorTemperature = param["m_leafChlorophyllSynthesisFactorTemperature"].as<float>();
		if (param["m_leafFallProbability"]) shootDescriptor.m_leafFallProbability = param["m_leafFallProbability"].as<float>();
		if (param["m_leafDistanceToBranchEndLimit"]) shootDescriptor.m_leafDistanceToBranchEndLimit = param["m_leafDistanceToBranchEndLimit"].as<float>();

		//Structure
		if (param["m_fruitBudCount"]) shootDescriptor.m_fruitBudCount = param["m_fruitBudCount"].as<int>();
		if (param["m_fruitGrowthRate"]) shootDescriptor.m_fruitGrowthRate = param["m_fruitGrowthRate"].as<float>();
		if (param["m_fruitBudFlushingProbabilityTemperatureRange"]) shootDescriptor.m_fruitBudFlushingProbabilityTemperatureRange = param["m_fruitBudFlushingProbabilityTemperatureRange"].as<glm::vec4>();
		if (param["m_fruitVigorRequirement"]) shootDescriptor.m_fruitVigorRequirement = param["m_fruitVigorRequirement"].as<float>();
		if (param["m_maxFruitSize"]) shootDescriptor.m_maxFruitSize = param["m_maxFruitSize"].as<glm::vec3>();
		if (param["m_fruitPositionVariance"]) shootDescriptor.m_fruitPositionVariance = param["m_fruitPositionVariance"].as<float>();
		if (param["m_fruitRotationVariance"]) shootDescriptor.m_fruitRotationVariance = param["m_fruitRotationVariance"].as<float>();
		if (param["m_fruitFallProbability"]) shootDescriptor.m_fruitFallProbability = param["m_fruitFallProbability"].as<float>();
	}
}
void TreeDescriptor::Deserialize(const YAML::Node& in) {
	DeserializeShootDescriptor("m_shootDescriptor", m_shootDescriptor, in);
	DeserializeFoliageDescriptor("m_foliageDescriptor", m_foliageDescriptor, in);

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