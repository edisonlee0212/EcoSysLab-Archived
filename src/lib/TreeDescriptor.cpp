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
		if (ImGui::DragFloat("Min root distance", &foliageParameters.m_minRootDistance, 0.01f, 0.0f, 10.0f)) changed = true;
		if (ImGui::DragFloat("Max end distance", &foliageParameters.m_maxEndDistance, 0.01f, 0.0f, 10.0f)) changed = true;
		ImGui::TreePop();
	}
	return changed;
}

bool TreeDescriptor::OnInspectShootGrowthParameters(ShootGrowthParameters& treeGrowthParameters) {
	bool changed = false;
	if (ImGui::TreeNodeEx("Shoot Parameters", ImGuiTreeNodeFlags_DefaultOpen)) {
		changed = ImGui::DragFloat("Growth rate", &treeGrowthParameters.m_growthRate, 0.01f, 0.0f, 10.0f) || changed;
		if (ImGui::TreeNodeEx("Internode", ImGuiTreeNodeFlags_DefaultOpen))
		{
			changed = ImGui::DragFloat2("Branching angle mean/var", &treeGrowthParameters.m_branchingAngleMeanVariance.x, 0.1f, 0.0f, 100.0f) || changed;
			changed = ImGui::DragFloat2("Roll angle mean/var", &treeGrowthParameters.m_rollAngleMeanVariance.x, 0.1f, 0.0f, 100.0f) || changed;
			changed = ImGui::DragFloat2("Apical angle variance", &treeGrowthParameters.m_apicalAngleMeanVariance.x, 0.1f, 0.0f, 100.0f) || changed;

			changed = ImGui::DragFloat("Gravitropism", &treeGrowthParameters.m_gravitropism, 0.01f) || changed;
			changed = ImGui::DragFloat("Phototropism", &treeGrowthParameters.m_phototropism, 0.01f) || changed;
			changed = ImGui::DragFloat("Horizontal Tropism", &treeGrowthParameters.m_horizontalTropism, 0.01f) || changed;
			changed = ImGui::DragFloat("Sagging strength", &treeGrowthParameters.m_saggingFactorThicknessReductionMax.x, 0.0001f, 0.0f, 10.0f, "%.5f") || changed;
			changed = ImGui::DragFloat("Sagging thickness factor", &treeGrowthParameters.m_saggingFactorThicknessReductionMax.y, 0.01f, 0.0f, 10.0f, "%.5f") || changed;
			changed = ImGui::DragFloat("Sagging max", &treeGrowthParameters.m_saggingFactorThicknessReductionMax.z, 0.001f, 0.0f, 1.0f, "%.5f") || changed;

			changed = ImGui::DragFloat("Internode length", &treeGrowthParameters.m_internodeLength, 0.001f) || changed;
			changed = ImGui::DragFloat("Internode length thickness factor", &treeGrowthParameters.m_internodeLengthThicknessFactor, 0.0001f, 0.0f, 1.0f) || changed;
			changed = ImGui::DragFloat3("Thickness min/factor/age", &treeGrowthParameters.m_endNodeThickness, 0.0001f, 0.0f, 1.0f, "%.6f") || changed;
			changed = ImGui::DragFloat("Internode shadow factor", &treeGrowthParameters.m_internodeShadowFactor, 0.001f, 0.0f, 1.0f) || changed;
			ImGui::TreePop();
		}
		if (ImGui::TreeNodeEx("Bud fate", ImGuiTreeNodeFlags_DefaultOpen)) {
			changed = ImGui::DragInt("Lateral bud count", &treeGrowthParameters.m_lateralBudCount, 1, 0, 3) || changed;
			changed = ImGui::DragFloat("Apical bud extinction rate", &treeGrowthParameters.m_apicalBudExtinctionRate, 0.00001f, 0.0f, 1.0f, "%.5f") || changed;
			changed = ImGui::DragFloat("Lateral bud flushing rate", &treeGrowthParameters.m_lateralBudFlushingRate, 0.00001f, 0.0f, 1.0f, "%.5f") || changed;
			changed = ImGui::DragFloat("Apical control", &treeGrowthParameters.m_apicalControl, 0.01f) || changed;
			changed = ImGui::DragFloat2("Inhibitor val/loss", &treeGrowthParameters.m_apicalDominance, 0.01f) || changed;
			ImGui::TreePop();
		}
		if (ImGui::TreeNodeEx("Pruning", ImGuiTreeNodeFlags_DefaultOpen))
		{
			changed = ImGui::DragFloat("Low branch pruning", &treeGrowthParameters.m_lowBranchPruning, 0.01f) || changed;
			if(treeGrowthParameters.m_lowBranchPruning > 0.0f) changed = ImGui::DragFloat("Low branch pruning thickness factor", &treeGrowthParameters.m_lowBranchPruningThicknessFactor, 0.01f) || changed;
			changed = ImGui::DragFloat("Light pruning", &treeGrowthParameters.m_lightPruningFactor, 0.01f) || changed;
			changed = ImGui::DragFloat("Thin branch pruning", &treeGrowthParameters.m_thicknessPruningFactor, 0.01f, 0.0f) || changed;
			ImGui::TreePop();
		}
		changed = ImGui::DragInt("Leaf bud count", &treeGrowthParameters.m_leafBudCount, 1, 0, 3) || changed;
		if (treeGrowthParameters.m_leafBudCount > 0 && ImGui::TreeNodeEx("Leaf"))
		{
			changed = ImGui::DragFloat("Leaf growth rate", &treeGrowthParameters.m_leafGrowthRate, 0.01f, 0.0f, 1.0f) || changed;
			changed = ImGui::DragFloat4("Leaf flushing prob/temp range", &treeGrowthParameters.m_leafBudFlushingProbabilityTemperatureRange.x, 0.00001f, 0.0f, 1.0f, "%.5f") || changed;
			changed = ImGui::DragFloat3("Leaf vigor requirement", &treeGrowthParameters.m_leafVigorRequirement, 0.01f) || changed;
			changed = ImGui::DragFloat3("Size", &treeGrowthParameters.m_maxLeafSize.x, 0.01f) || changed;
			changed = ImGui::DragFloat("Position Variance", &treeGrowthParameters.m_leafPositionVariance, 0.01f) || changed;
			changed = ImGui::DragFloat("Random rotation", &treeGrowthParameters.m_leafRotationVariance, 0.01f) || changed;
			changed = ImGui::DragFloat("Chlorophyll Loss", &treeGrowthParameters.m_leafChlorophyllLoss, 0.01f) || changed;
			changed = ImGui::DragFloat("Chlorophyll temperature", &treeGrowthParameters.m_leafChlorophyllSynthesisFactorTemperature, 0.01f) || changed;
			changed = ImGui::DragFloat("Drop prob", &treeGrowthParameters.m_leafFallProbability, 0.01f) || changed;
			changed = ImGui::DragFloat("Distance To End Limit", &treeGrowthParameters.m_leafDistanceToBranchEndLimit, 0.01f) || changed;
			ImGui::TreePop();
		}
		changed = ImGui::DragInt("Fruit bud count", &treeGrowthParameters.m_fruitBudCount, 1, 0, 3) || changed;
		if (treeGrowthParameters.m_fruitBudCount > 0 && ImGui::TreeNodeEx("Fruit"))
		{
			changed = ImGui::DragFloat("Fruit growth rate", &treeGrowthParameters.m_fruitGrowthRate, 0.01f, 0.0f, 1.0f) || changed;
			changed = ImGui::DragFloat4("Fruit flushing prob/temp range", &treeGrowthParameters.m_fruitBudFlushingProbabilityTemperatureRange.x, 0.00001f, 0.0f, 1.0f, "%.5f") || changed;
			changed = ImGui::DragFloat3("Fruit vigor requirement", &treeGrowthParameters.m_fruitVigorRequirement, 0.01f) || changed;
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

void TreeDescriptor::ApplyOffsets(ShootGrowthParameters& treeGrowthParameters, const std::vector<ShootGrowthParameterOffset>& offsets)
{
	for (const auto& i : offsets)
	{
		float a = i.m_offset.GetValue(glm::linearRand(0.0f, 1.0f));

		switch (i.m_type)
		{
		case static_cast<unsigned>(ShootGrowthParameterType::GrowthRate):
		{
			treeGrowthParameters.m_growthRate += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::BranchingAngleMean):
		{
			treeGrowthParameters.m_branchingAngleMeanVariance.x += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::BranchingAngleVariance):
		{
			treeGrowthParameters.m_branchingAngleMeanVariance.y += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::RollAngleMean):
		{
			treeGrowthParameters.m_rollAngleMeanVariance.x += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::RollAngleVariance):
		{
			treeGrowthParameters.m_rollAngleMeanVariance.y += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::ApicalAngleMean):
		{
			treeGrowthParameters.m_apicalAngleMeanVariance.x += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::ApicalAngleVariance):
		{
			treeGrowthParameters.m_apicalAngleMeanVariance.y += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::Gravitropism):
		{
			treeGrowthParameters.m_gravitropism += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::Phototropism):
		{
			treeGrowthParameters.m_phototropism += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::Sagging):
		{
			treeGrowthParameters.m_saggingFactorThicknessReductionMax.x += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::SaggingThicknessFactor):
		{
			treeGrowthParameters.m_saggingFactorThicknessReductionMax.y += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::MaxSagging):
		{
			treeGrowthParameters.m_saggingFactorThicknessReductionMax.z += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::InternodeLength):
		{
			treeGrowthParameters.m_internodeLength += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::InternodeLengthThicknessFactor):
		{
			treeGrowthParameters.m_internodeLengthThicknessFactor += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::EndNodeThickness):
		{
			treeGrowthParameters.m_endNodeThickness += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::ThicknessAccumulationFactor):
		{
			treeGrowthParameters.m_thicknessAccumulationFactor += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::ThicknessAgeFactor):
		{
			treeGrowthParameters.m_thicknessAgeFactor += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::ShadowFactor):
		{
			treeGrowthParameters.m_internodeShadowFactor += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::ApicalBudExtinctionRate):
		{
			treeGrowthParameters.m_apicalBudExtinctionRate += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::ApicalControl):
		{
			treeGrowthParameters.m_apicalControl += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::ApicalDominance):
		{
			treeGrowthParameters.m_apicalDominance += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::ApicalDominanceLoss):
		{
			treeGrowthParameters.m_apicalDominanceLoss += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::LowBranchPruning):
		{
			treeGrowthParameters.m_lowBranchPruning += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::LowBranchPruningThicknessFactor):
		{
			treeGrowthParameters.m_lowBranchPruningThicknessFactor += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::LightPruningFactor):
		{
			treeGrowthParameters.m_lightPruningFactor += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::ThicknessPruningFactor):
		{
			treeGrowthParameters.m_thicknessPruningFactor += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		default:break;
		}
	}
}

bool TreeDescriptor::OnInspectShootGrowthParametersOffset(
	std::vector<ShootGrowthParameterOffset>& shootGrowthParameterOffsets)
{
	bool changed = false;
	if (ImGui::TreeNodeEx("Shoot Parameter Offsets", ImGuiTreeNodeFlags_DefaultOpen))
	{
		if(ImGui::Button("New..."))
		{
			shootGrowthParameterOffsets.emplace_back();
			shootGrowthParameterOffsets.back().m_offset.SetStart(0.0f);
			shootGrowthParameterOffsets.back().m_offset.SetEnd(1.0f);
			changed = true;
		}
		for (int offsetIndex = 0; offsetIndex < shootGrowthParameterOffsets.size(); offsetIndex++)
		{
			auto& offset = shootGrowthParameterOffsets.at(offsetIndex);
			std::string tag = "Offset " + std::to_string(offsetIndex + 1);
			if (ImGui::TreeNode(tag.c_str())) {
				if (ImGui::Button("Remove"))
				{
					shootGrowthParameterOffsets.erase(shootGrowthParameterOffsets.begin() + offsetIndex);
					offsetIndex--;
					ImGui::TreePop();
					continue;
				}
				ImGui::SameLine();
				if (ImGui::Combo("Type", { "GrowthRate", "BranchingAngleMean", "BranchingAngleVariance",
					"RollAngleMean", "RollAngleVariance", "ApicalAngleMean", "ApicalAngleVariance", "Gravitropism", "Phototropism", "Sagging", "SaggingThicknessFactor",
					"MaxSagging", "InternodeLength", "InternodeLengthThicknessFactor", "EndNodeThickness", "ThicknessAccumulationFactor", "ThicknessAgeFactor", "ShadowFactor",
					"ApicalBudExtinctionRate", "LateralBudExtinctionRate", "ApicalBudLightingFactor",
					"LateralBudLightingFactor", "ApicalControl", "ApicalDominance", "ApicalDominanceLoss",
					"LowBranchPruning", "LowBranchPruningThicknessFactor", "LightPruningFactor", "ThicknessPruningFactor"}, offset.m_type)) {
					changed = true;
				}
				ImGui::DragFloat2("Range", &offset.m_range.x, 0.01f);
				if (offset.m_offset.OnInspect(tag, ImVec2(-1, -1), static_cast<unsigned>(CurveEditorFlags::ALLOW_RESIZE) | static_cast<unsigned>(CurveEditorFlags::SHOW_GRID) | static_cast<unsigned>(CurveEditorFlags::DISABLE_START_END_Y))) changed = true;
				ImGui::TreePop();
			}
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
	if (OnInspectShootGrowthParameters(m_shootGrowthParameters)) { changed = true; }
	if (OnInspectShootGrowthParametersOffset(m_shootGrowthParameterOffsets)) { changed = true; }
	if (OnInspectFoliageParameters(m_foliageParameters)) { changed = true; }
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
	out << YAML::Key << "m_growthRate" << YAML::Value << treeGrowthParameters.m_growthRate;
	out << YAML::Key << "m_branchingAngleMeanVariance" << YAML::Value << treeGrowthParameters.m_branchingAngleMeanVariance;
	out << YAML::Key << "m_rollAngleMeanVariance" << YAML::Value << treeGrowthParameters.m_rollAngleMeanVariance;
	out << YAML::Key << "m_apicalAngleMeanVariance" << YAML::Value << treeGrowthParameters.m_apicalAngleMeanVariance;
	out << YAML::Key << "m_gravitropism" << YAML::Value << treeGrowthParameters.m_gravitropism;
	out << YAML::Key << "m_phototropism" << YAML::Value << treeGrowthParameters.m_phototropism;
	out << YAML::Key << "m_horizontalTropism" << YAML::Value << treeGrowthParameters.m_horizontalTropism;
	out << YAML::Key << "m_saggingFactorThicknessReductionMax" << YAML::Value << treeGrowthParameters.m_saggingFactorThicknessReductionMax;
	out << YAML::Key << "m_internodeLength" << YAML::Value << treeGrowthParameters.m_internodeLength;
	out << YAML::Key << "m_internodeLengthThicknessFactor" << YAML::Value << treeGrowthParameters.m_internodeLengthThicknessFactor;
	out << YAML::Key << "m_endNodeThickness" << YAML::Value << treeGrowthParameters.m_endNodeThickness;
	out << YAML::Key << "m_thicknessAccumulationFactor" << YAML::Value << treeGrowthParameters.m_thicknessAccumulationFactor;
	out << YAML::Key << "m_thicknessAgeFactor" << YAML::Value << treeGrowthParameters.m_thicknessAgeFactor;
	out << YAML::Key << "m_internodeShadowFactor" << YAML::Value << treeGrowthParameters.m_internodeShadowFactor;

	out << YAML::Key << "m_lateralBudCount" << YAML::Value << treeGrowthParameters.m_lateralBudCount;
	out << YAML::Key << "m_apicalBudExtinctionRate" << YAML::Value << treeGrowthParameters.m_apicalBudExtinctionRate;
	out << YAML::Key << "m_lateralBudFlushingRate" << YAML::Value << treeGrowthParameters.m_lateralBudFlushingRate;
	out << YAML::Key << "m_apicalControl" << YAML::Value << treeGrowthParameters.m_apicalControl;
	out << YAML::Key << "m_apicalDominance" << YAML::Value << treeGrowthParameters.m_apicalDominance;
	out << YAML::Key << "m_apicalDominanceLoss" << YAML::Value << treeGrowthParameters.m_apicalDominanceLoss;

	out << YAML::Key << "m_lowBranchPruning" << YAML::Value << treeGrowthParameters.m_lowBranchPruning;
	out << YAML::Key << "m_lowBranchPruningThicknessFactor" << YAML::Value << treeGrowthParameters.m_lowBranchPruningThicknessFactor;
	out << YAML::Key << "m_lightPruningFactor" << YAML::Value << treeGrowthParameters.m_lightPruningFactor;
	out << YAML::Key << "m_thicknessPruningFactor" << YAML::Value << treeGrowthParameters.m_thicknessPruningFactor;

	out << YAML::Key << "m_leafBudCount" << YAML::Value << treeGrowthParameters.m_leafBudCount;
	out << YAML::Key << "m_leafGrowthRate" << YAML::Value << treeGrowthParameters.m_leafGrowthRate;
	out << YAML::Key << "m_leafBudFlushingProbabilityTemperatureRange" << YAML::Value << treeGrowthParameters.m_leafBudFlushingProbabilityTemperatureRange;
	out << YAML::Key << "m_leafVigorRequirement" << YAML::Value << treeGrowthParameters.m_leafVigorRequirement;
	out << YAML::Key << "m_maxLeafSize" << YAML::Value << treeGrowthParameters.m_maxLeafSize;
	out << YAML::Key << "m_leafPositionVariance" << YAML::Value << treeGrowthParameters.m_leafPositionVariance;
	out << YAML::Key << "m_leafRotationVariance" << YAML::Value << treeGrowthParameters.m_leafRotationVariance;
	out << YAML::Key << "m_leafChlorophyllLoss" << YAML::Value << treeGrowthParameters.m_leafChlorophyllLoss;
	out << YAML::Key << "m_leafChlorophyllSynthesisFactorTemperature" << YAML::Value << treeGrowthParameters.m_leafChlorophyllSynthesisFactorTemperature;
	out << YAML::Key << "m_leafFallProbability" << YAML::Value << treeGrowthParameters.m_leafFallProbability;
	out << YAML::Key << "m_leafDistanceToBranchEndLimit" << YAML::Value << treeGrowthParameters.m_leafDistanceToBranchEndLimit;

	out << YAML::Key << "m_fruitBudCount" << YAML::Value << treeGrowthParameters.m_fruitBudCount;
	out << YAML::Key << "m_fruitGrowthRate" << YAML::Value << treeGrowthParameters.m_fruitGrowthRate;
	out << YAML::Key << "m_fruitBudFlushingProbabilityTemperatureRange" << YAML::Value << treeGrowthParameters.m_fruitBudFlushingProbabilityTemperatureRange;
	out << YAML::Key << "m_fruitVigorRequirement" << YAML::Value << treeGrowthParameters.m_fruitVigorRequirement;
	out << YAML::Key << "m_maxFruitSize" << YAML::Value << treeGrowthParameters.m_maxFruitSize;
	out << YAML::Key << "m_fruitPositionVariance" << YAML::Value << treeGrowthParameters.m_fruitPositionVariance;
	out << YAML::Key << "m_fruitRotationVariance" << YAML::Value << treeGrowthParameters.m_fruitRotationVariance;
	out << YAML::Key << "m_fruitFallProbability" << YAML::Value << treeGrowthParameters.m_fruitFallProbability;
	
	out << YAML::EndMap;
}
void TreeDescriptor::Serialize(YAML::Emitter& out) {
	SerializeShootGrowthParameters("m_shootGrowthParameters", m_shootGrowthParameters, out);
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

	if(!m_shootGrowthParameterOffsets.empty())
	{
		out << YAML::Key << "m_shootGrowthParameterOffsets" << YAML::BeginSeq;
		for(const auto& i : m_shootGrowthParameterOffsets)
		{
			out << YAML::BeginMap;
			out << YAML::Key << "m_type" << YAML::Value << i.m_type;
			out << YAML::Key << "m_range" << YAML::Value << i.m_range;
			i.m_offset.Save("m_offset", out);
			out << YAML::EndMap;
		}
		out << YAML::EndSeq;
	}
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

		if (param["m_growthRate"]) treeGrowthParameters.m_growthRate = param["m_growthRate"].as<float>();
		if (param["m_branchingAngleMeanVariance"]) treeGrowthParameters.m_branchingAngleMeanVariance = param["m_branchingAngleMeanVariance"].as<glm::vec2>();
		if (param["m_rollAngleMeanVariance"]) treeGrowthParameters.m_rollAngleMeanVariance = param["m_rollAngleMeanVariance"].as<glm::vec2>();
		if (param["m_apicalAngleMeanVariance"]) treeGrowthParameters.m_apicalAngleMeanVariance = param["m_apicalAngleMeanVariance"].as<glm::vec2>();
		if (param["m_gravitropism"]) treeGrowthParameters.m_gravitropism = param["m_gravitropism"].as<float>();
		if (param["m_phototropism"]) treeGrowthParameters.m_phototropism = param["m_phototropism"].as<float>();
		if (param["m_horizontalTropism"]) treeGrowthParameters.m_horizontalTropism = param["m_horizontalTropism"].as<float>();
		if (param["m_saggingFactorThicknessReductionMax"]) treeGrowthParameters.m_saggingFactorThicknessReductionMax = param["m_saggingFactorThicknessReductionMax"].as<glm::vec3>();
		if (param["m_internodeLength"]) treeGrowthParameters.m_internodeLength = param["m_internodeLength"].as<float>();
		if (param["m_internodeLengthThicknessFactor"]) treeGrowthParameters.m_internodeLengthThicknessFactor = param["m_internodeLengthThicknessFactor"].as<float>();
		if (param["m_endNodeThickness"]) treeGrowthParameters.m_endNodeThickness = param["m_endNodeThickness"].as<float>();
		if (param["m_thicknessAccumulationFactor"]) treeGrowthParameters.m_thicknessAccumulationFactor = param["m_thicknessAccumulationFactor"].as<float>();
		if (param["m_thicknessAgeFactor"]) treeGrowthParameters.m_thicknessAgeFactor = param["m_thicknessAgeFactor"].as<float>();
		if (param["m_internodeShadowFactor"]) treeGrowthParameters.m_internodeShadowFactor = param["m_internodeShadowFactor"].as<float>();


		if (param["m_lateralBudCount"]) treeGrowthParameters.m_lateralBudCount = param["m_lateralBudCount"].as<int>();
		if (param["m_apicalBudExtinctionRate"]) treeGrowthParameters.m_apicalBudExtinctionRate = param["m_apicalBudExtinctionRate"].as<float>();
		if (param["m_lateralBudFlushingRate"]) treeGrowthParameters.m_lateralBudFlushingRate = param["m_lateralBudFlushingRate"].as<float>();
		if (param["m_apicalControl"]) treeGrowthParameters.m_apicalControl = param["m_apicalControl"].as<float>();
		if (param["m_apicalDominance"]) treeGrowthParameters.m_apicalDominance = param["m_apicalDominance"].as<float>();
		if (param["m_apicalDominanceLoss"]) treeGrowthParameters.m_apicalDominanceLoss = param["m_apicalDominanceLoss"].as<float>();

		if (param["m_lowBranchPruning"]) treeGrowthParameters.m_lowBranchPruning = param["m_lowBranchPruning"].as<float>();
		if (param["m_lowBranchPruningThicknessFactor"]) treeGrowthParameters.m_lowBranchPruningThicknessFactor = param["m_lowBranchPruningThicknessFactor"].as<float>();
		if (param["m_lightPruningFactor"]) treeGrowthParameters.m_lightPruningFactor = param["m_lightPruningFactor"].as<float>();
		if (param["m_thicknessPruningFactor"]) treeGrowthParameters.m_thicknessPruningFactor = param["m_thicknessPruningFactor"].as<float>();

		if (param["m_leafBudCount"]) treeGrowthParameters.m_leafBudCount = param["m_leafBudCount"].as<int>();
		if (param["m_leafGrowthRate"]) treeGrowthParameters.m_leafGrowthRate = param["m_leafGrowthRate"].as<float>();
		if (param["m_leafBudFlushingProbabilityTemperatureRange"]) treeGrowthParameters.m_leafBudFlushingProbabilityTemperatureRange = param["m_leafBudFlushingProbabilityTemperatureRange"].as< glm::vec4>();
		if (param["m_leafVigorRequirement"]) treeGrowthParameters.m_leafVigorRequirement = param["m_leafVigorRequirement"].as<float>();
		if (param["m_maxLeafSize"]) treeGrowthParameters.m_maxLeafSize = param["m_maxLeafSize"].as<glm::vec3>();
		if (param["m_leafPositionVariance"]) treeGrowthParameters.m_leafPositionVariance = param["m_leafPositionVariance"].as<float>();
		if (param["m_leafRotationVariance"]) treeGrowthParameters.m_leafRotationVariance = param["m_leafRotationVariance"].as<float>();
		if (param["m_leafChlorophyllLoss"]) treeGrowthParameters.m_leafChlorophyllLoss = param["m_leafChlorophyllLoss"].as<float>();
		if (param["m_leafChlorophyllSynthesisFactorTemperature"]) treeGrowthParameters.m_leafChlorophyllSynthesisFactorTemperature = param["m_leafChlorophyllSynthesisFactorTemperature"].as<float>();
		if (param["m_leafFallProbability"]) treeGrowthParameters.m_leafFallProbability = param["m_leafFallProbability"].as<float>();
		if (param["m_leafDistanceToBranchEndLimit"]) treeGrowthParameters.m_leafDistanceToBranchEndLimit = param["m_leafDistanceToBranchEndLimit"].as<float>();

		//Structure
		if (param["m_fruitBudCount"]) treeGrowthParameters.m_fruitBudCount = param["m_fruitBudCount"].as<int>();
		if (param["m_fruitGrowthRate"]) treeGrowthParameters.m_fruitGrowthRate = param["m_fruitGrowthRate"].as<float>();
		if (param["m_fruitBudFlushingProbabilityTemperatureRange"]) treeGrowthParameters.m_fruitBudFlushingProbabilityTemperatureRange = param["m_fruitBudFlushingProbabilityTemperatureRange"].as<glm::vec4>();
		if (param["m_fruitVigorRequirement"]) treeGrowthParameters.m_fruitVigorRequirement = param["m_fruitVigorRequirement"].as<float>();
		if (param["m_maxFruitSize"]) treeGrowthParameters.m_maxFruitSize = param["m_maxFruitSize"].as<glm::vec3>();
		if (param["m_fruitPositionVariance"]) treeGrowthParameters.m_fruitPositionVariance = param["m_fruitPositionVariance"].as<float>();
		if (param["m_fruitRotationVariance"]) treeGrowthParameters.m_fruitRotationVariance = param["m_fruitRotationVariance"].as<float>();
		if (param["m_fruitFallProbability"]) treeGrowthParameters.m_fruitFallProbability = param["m_fruitFallProbability"].as<float>();
	}
}
void TreeDescriptor::Deserialize(const YAML::Node& in) {
	DeserializeShootGrowthParameters("m_shootGrowthParameters", m_shootGrowthParameters, in);
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

	
	if(in["m_shootGrowthParameterOffsets"])
	{
		m_shootGrowthParameterOffsets.clear();
		const auto& inShootGrowthParametersOffsets = in["m_shootGrowthParameterOffsets"];
		for(const auto& inShootGrowthParametersOffset : inShootGrowthParametersOffsets)
		{
			m_shootGrowthParameterOffsets.emplace_back();
			auto& shootGrowthParameterOffsets = m_shootGrowthParameterOffsets.back();
			shootGrowthParameterOffsets.m_type = inShootGrowthParametersOffset["m_type"].as<unsigned>();
			shootGrowthParameterOffsets.m_range = inShootGrowthParametersOffset["m_range"].as<glm::vec2>();
			shootGrowthParameterOffsets.m_offset.Load("m_offset", inShootGrowthParametersOffset);
		}
	}
}