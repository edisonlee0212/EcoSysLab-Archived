//
// Created by lllll on 10/24/2022.
//

#include "Tree.hpp"
#include "Graphics.hpp"
#include "EditorLayer.hpp"
#include "Application.hpp"
#include "TreeMeshGenerator.hpp"
#include "Soil.hpp"
#include "Climate.hpp"
#include "Octree.hpp"
#include "EcoSysLabLayer.hpp"
#include "HeightField.hpp"
using namespace EcoSysLab;

void Tree::OnInspect() {
	bool modelChanged = false;
	if (Editor::DragAndDropButton<TreeDescriptor>(m_treeDescriptor, "TreeDescriptor", true)) {
		m_treeModel.Clear();
		modelChanged = true;
	}
	if (m_treeDescriptor.Get<TreeDescriptor>()) {
		ImGui::Checkbox("Enable History", &m_enableHistory);
		ImGui::Checkbox("Enable Branch collision detection", &m_treeModel.m_enableBranchCollisionDetection);
		ImGui::Checkbox("Enable Root collision detection", &m_treeModel.m_enableRootCollisionDetection);
		if (ImGui::Button("Grow")) {
			TryGrow();
			modelChanged = true;
		}
		static int iterations = 5;
		ImGui::DragInt("Iterations", &iterations, 1, 1, 100);
		if (ImGui::TreeNode("Mesh generation")) {
			static TreeMeshGeneratorSettings meshGeneratorSettings;
			meshGeneratorSettings.OnInspect();
			if (ImGui::Button("Generate Mesh")) {
				GenerateMesh(meshGeneratorSettings);
			}
			ImGui::TreePop();
		}
		if (ImGui::Button("Clear")) {
			m_treeModel.Clear();
			modelChanged = true;
		}
	}
	if (modelChanged) {
		const auto treeVisualizationLayer = Application::GetLayer<EcoSysLabLayer>();
		if (treeVisualizationLayer && treeVisualizationLayer->m_selectedTree == GetOwner()) {
			treeVisualizationLayer->m_treeVisualizer.Reset(m_treeModel);
		}
	}
}

void Tree::OnCreate() {
}

void Tree::OnDestroy() {
	m_treeModel.Clear();
	m_treeDescriptor.Clear();
	m_soil.Clear();
	m_climate.Clear();
	m_enableHistory = false;
}

bool Tree::TryGrow() {
	const auto scene = GetScene();
	const auto treeDescriptor = m_treeDescriptor.Get<TreeDescriptor>();
	if (!treeDescriptor) {
		UNIENGINE_ERROR("No tree descriptor!");
		return false;
	}
	if (!m_soil.Get<Soil>()) {
		UNIENGINE_ERROR("No soil model!");
		return false;
	}
	if (!m_climate.Get<Climate>()) {
		UNIENGINE_ERROR("No climate model!");
		return false;
	}
	const auto soil = m_soil.Get<Soil>();
	const auto climate = m_climate.Get<Climate>();
	if (m_enableHistory) m_treeModel.Step();

	const auto owner = GetOwner();
	return m_treeModel.Grow(scene->GetDataComponent<GlobalTransform>(owner).m_value, soil->m_soilModel, climate->m_climateModel,
		treeDescriptor->m_rootGrowthParameters, treeDescriptor->m_treeGrowthParameters);
}

void Tree::GenerateMesh(const TreeMeshGeneratorSettings& meshGeneratorSettings) {
	const auto scene = GetScene();
	const auto self = GetOwner();
	const auto children = scene->GetChildren(self);

	Entity branchEntity, rootEntity, foliageEntity, fruitEntity;

	for (const auto& child : children) {
		auto name = scene->GetEntityName(child);
		if (name == "Branch Mesh") {
			branchEntity = child;
		}
		else if (name == "Root Mesh") {
			rootEntity = child;
		}
		else if (name == "Foliage Mesh") {
			foliageEntity = child;
		}
		else if (name == "Fruit Mesh") {
			fruitEntity = child;
		}
	}
	if (branchEntity.GetIndex() == 0)
	{
		branchEntity = scene->CreateEntity("Branch Mesh");
		scene->SetParent(branchEntity, self);
	}
	if (rootEntity.GetIndex() == 0)
	{
		rootEntity = scene->CreateEntity("Root Mesh");
		scene->SetParent(rootEntity, self);
	}
	if (foliageEntity.GetIndex() == 0)
	{
		foliageEntity = scene->CreateEntity("Foliage Mesh");
		scene->SetParent(foliageEntity, self);
	}
	if (fruitEntity.GetIndex() == 0)
	{
		fruitEntity = scene->CreateEntity("Fruit Mesh");
		scene->SetParent(fruitEntity, self);
	}
	{
		std::vector<Vertex> vertices;
		std::vector<unsigned int> indices;
		CylindricalMeshGenerator<SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> meshGenerator;
		meshGenerator.Generate(m_treeModel.RefBranchSkeleton(), vertices, indices,
			meshGeneratorSettings);
		auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
		auto material = ProjectManager::CreateTemporaryAsset<Material>();
		material->SetProgram(DefaultResources::GLPrograms::StandardProgram);
		mesh->SetVertices(17, vertices, indices);
		auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(branchEntity).lock();
		meshRenderer->m_mesh = mesh;
		meshRenderer->m_material = material;
	}
	{
		std::vector<Vertex> vertices;
		std::vector<unsigned int> indices;
		const auto treeDescriptor = m_treeDescriptor.Get<TreeDescriptor>();
		float minRadius = 0.01f;
		if(treeDescriptor)
		{
			minRadius = treeDescriptor->m_rootGrowthParameters.m_endNodeThicknessAndControl.x;
		}
		VoxelMeshGenerator<RootSkeletonGrowthData, RootBranchGrowthData, RootInternodeGrowthData> meshGenerator;
		meshGenerator.Generate(m_treeModel.RefRootSkeleton(), vertices, indices,
			meshGeneratorSettings, minRadius);
		auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
		auto material = ProjectManager::CreateTemporaryAsset<Material>();
		material->SetProgram(DefaultResources::GLPrograms::StandardProgram);
		mesh->SetVertices(17, vertices, indices);
		auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(rootEntity).lock();
		meshRenderer->m_mesh = mesh;
		meshRenderer->m_material = material;
	}
}

void TreeDescriptor::OnCreate() {

}


bool OnInspectTreeGrowthParameters(TreeGrowthParameters& treeGrowthParameters) {
	bool changed = false;
	if (ImGui::TreeNodeEx("Tree Parameters", ImGuiTreeNodeFlags_DefaultOpen)) {
		if (ImGui::TreeNodeEx("Structure", ImGuiTreeNodeFlags_DefaultOpen)) {
			if (ImGui::DragInt("Lateral bud count", &treeGrowthParameters.m_lateralBudCount))
			{
				changed = true;
			}
			if (ImGui::DragInt("Fruit bud count", &treeGrowthParameters.m_fruitBudCount))
			{
				changed = true;
			}
			if (ImGui::DragInt("Leaf bud count", &treeGrowthParameters.m_leafBudCount))
			{
				changed = true;
			}
			if (ImGui::DragFloat2("Branching Angle mean/var", &treeGrowthParameters.m_branchingAngleMeanVariance.x,
				0.01f))
			{
				changed = true;
			}
			if (ImGui::DragFloat2("Roll Angle mean/var", &treeGrowthParameters.m_rollAngleMeanVariance.x, 0.01f))
			{
				changed = true;
			}
			if (ImGui::DragFloat2("Apical Angle mean/var", &treeGrowthParameters.m_apicalAngleMeanVariance.x,
				0.01f))
			{
				changed = true;
			}
			if (ImGui::DragFloat("Gravitropism", &treeGrowthParameters.m_gravitropism, 0.01f))
			{
				changed = true;
			}
			if (ImGui::DragFloat("Phototropism", &treeGrowthParameters.m_phototropism, 0.01f))
			{
				changed = true;
			}
			if (ImGui::DragFloat("Internode length", &treeGrowthParameters.m_internodeLength, 0.01f))
			{
				changed = true;
			}
			if (ImGui::DragFloat("Growth rate", &treeGrowthParameters.m_growthRate, 0.01f))
			{
				changed = true;
			}
			if (ImGui::DragFloat2("Thickness min/factor", &treeGrowthParameters.m_endNodeThickness,
				0.01f))
			{
				changed = true;
			}
			ImGui::TreePop();
		}
		if (ImGui::TreeNodeEx("Bud", ImGuiTreeNodeFlags_DefaultOpen)) {
			if (ImGui::DragFloat("Lateral bud flushing probability",
				&treeGrowthParameters.m_lateralBudFlushingProbability, 0.01f))
			{
				changed = true;
			}
			if (ImGui::DragFloat2("Apical control base/dist", &treeGrowthParameters.m_apicalControlBaseDistFactor.x,
				0.01f))
			{
				changed = true;
			}
			if (ImGui::DragFloat2("Apical dominance base/dist",
				&treeGrowthParameters.m_apicalDominance, 0.01f))
			{
				changed = true;
			}
			float maxDistance = treeGrowthParameters.m_apicalDominance /
				treeGrowthParameters.m_apicalDominanceDistanceFactor;
			ImGui::Text("Max age / distance: [%.3f]", maxDistance);

			if (ImGui::DragFloat("Kill probability",
				&treeGrowthParameters.m_budKillProbability, 0.01f))
			{
				changed = true;
			}

			if (ImGui::DragFloat3("Base resource shoot/leaf/fruit",
				&treeGrowthParameters.m_shootBaseWaterRequirement, 0.01f))
			{
				changed = true;
			}

			if (ImGui::DragFloat3("Productive resource shoot/leaf/fruit",
				&treeGrowthParameters.m_shootProductiveWaterRequirement, 0.01f))
			{
				changed = true;
			}

			ImGui::TreePop();
		}
		if (ImGui::TreeNodeEx("Internode")) {
			if (ImGui::DragFloat("Low Branch Pruning", &treeGrowthParameters.m_lowBranchPruning, 0.01f);
				ImGui::DragFloat3("Sagging thickness/reduction/max",
					&treeGrowthParameters.m_saggingFactorThicknessReductionMax.x, 0.01f, 0.0f, 1.0f, "%.5f"))
			{
				changed = true;
			}
			ImGui::TreePop();
		}
		ImGui::TreePop();
	}

	return changed;
}

bool OnInspectRootGrowthParameters(RootGrowthParameters& rootGrowthParameters) {
	bool changed = false;
	if (ImGui::TreeNodeEx("Root Parameters", ImGuiTreeNodeFlags_DefaultOpen)) {
		if (ImGui::TreeNodeEx("Structure", ImGuiTreeNodeFlags_DefaultOpen)) {
			if (ImGui::DragFloat("Root node length", &rootGrowthParameters.m_rootNodeLength, 0.01f))
			{
				changed = true;
			}
			if (ImGui::DragFloat2("Thickness min/factor", &rootGrowthParameters.m_endNodeThicknessAndControl.x,
				0.01f))
			{
				changed = true;
			}
			if (ImGui::DragFloat("Thickness accmu", &rootGrowthParameters.m_thicknessLengthAccumulate, 0.000001f, 0.0f, 1.0f, "%.6f"))
			{
				changed = true;
			}
			if (ImGui::DragFloat2("Branching Angle mean/var", &rootGrowthParameters.m_branchingAngleMeanVariance.x,
				0.01f))
			{
				changed = true;
			}
			if (ImGui::DragFloat2("Roll Angle mean/var", &rootGrowthParameters.m_rollAngleMeanVariance.x, 0.01f))
			{
				changed = true;
			}
			if (ImGui::DragFloat2("Apical Angle mean/var", &rootGrowthParameters.m_apicalAngleMeanVariance.x,
				0.01f))
			{
				changed = true;
			}
			ImGui::TreePop();
		}
		if (ImGui::TreeNodeEx("Growth", ImGuiTreeNodeFlags_DefaultOpen))
		{
			if (ImGui::DragFloat("Growth rate", &rootGrowthParameters.m_growthRate, 0.01f))
			{
				changed = true;
			}
			if (ImGui::DragFloat("Auxin loss", &rootGrowthParameters.m_auxinTransportLoss, 0.01f))
			{
				changed = true;
			}
			if (ImGui::DragFloat("Tropism switching prob", &rootGrowthParameters.m_tropismSwitchingProbability, 0.01f))
			{
				changed = true;
			}
			if (ImGui::DragFloat("Tropism switching prob dist factor", &rootGrowthParameters.m_tropismSwitchingProbabilityDistanceFactor, 0.01f))
			{
				changed = true;
			}
			if (ImGui::DragFloat("Tropism intensity", &rootGrowthParameters.m_tropismIntensity, 0.01f))
			{
				changed = true;
			}
			if (ImGui::DragFloat("Branching prob base", &rootGrowthParameters.m_baseBranchingProbability, 0.01f))
			{
				changed = true;
			}
			if (ImGui::DragFloat("Branching prob child decrease", &rootGrowthParameters.m_branchingProbabilityChildrenDecrease, 0.01f))
			{
				changed = true;
			}
			if (ImGui::DragFloat("Branching prob dist decrease", &rootGrowthParameters.m_branchingProbabilityDistanceDecrease, 0.01f))
			{
				changed = true;
			}
			ImGui::TreePop();
		}
		ImGui::TreePop();
	}
	return changed;
}

void TreeDescriptor::OnInspect() {
	bool changed = false;
	auto environmentLayer = Application::GetLayer<EcoSysLabLayer>();
	const auto soil = environmentLayer->m_soilHolder.Get<Soil>();
	const auto climate = environmentLayer->m_climateHolder.Get<Climate>();
	if (soil && climate) {
		if (ImGui::Button("Instantiate")) {
			auto scene = Application::GetActiveScene();
			auto treeEntity = scene->CreateEntity(GetTitle());
			auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
			float height = 0;
			auto soilDescriptor = soil->m_soilDescriptor.Get<SoilDescriptor>();
			if(soilDescriptor)
			{
				auto heightField = soilDescriptor->m_heightField.Get<HeightField>();
				if (heightField) height = heightField->GetValue({ 0.0f, 0.0f }) - 0.05f;
			}
			GlobalTransform globalTransform;
			globalTransform.SetPosition(glm::vec3(0, height, 0));
			scene->SetDataComponent(treeEntity, globalTransform);
			tree->m_treeDescriptor = ProjectManager::GetAsset(GetHandle());
		}
	}else
	{
		ImGui::Text("Attach soil and climate entity to instantiate!");
	}
	if (OnInspectTreeGrowthParameters(m_treeGrowthParameters)) { changed = true; }
	if (OnInspectRootGrowthParameters(m_rootGrowthParameters)) { changed = true; }
	if (changed) m_saved = false;
}

void TreeDescriptor::CollectAssetRef(std::vector<AssetRef>& list) {

}

void SerializeTreeGrowthParameters(const std::string& name, const TreeGrowthParameters& treeGrowthParameters, YAML::Emitter& out) {
	out << YAML::Key << name << YAML::BeginMap;
	out << YAML::Key << "m_lateralBudCount" << YAML::Value << treeGrowthParameters.m_lateralBudCount;
	out << YAML::Key << "m_fruitBudCount" << YAML::Value << treeGrowthParameters.m_fruitBudCount;
	out << YAML::Key << "m_leafBudCount" << YAML::Value << treeGrowthParameters.m_leafBudCount;
	out << YAML::Key << "m_branchingAngleMeanVariance" << YAML::Value
		<< treeGrowthParameters.m_branchingAngleMeanVariance;
	out << YAML::Key << "m_rollAngleMeanVariance" << YAML::Value
		<< treeGrowthParameters.m_rollAngleMeanVariance;
	out << YAML::Key << "m_apicalAngleMeanVariance" << YAML::Value
		<< treeGrowthParameters.m_apicalAngleMeanVariance;
	out << YAML::Key << "m_gravitropism" << YAML::Value << treeGrowthParameters.m_gravitropism;
	out << YAML::Key << "m_phototropism" << YAML::Value << treeGrowthParameters.m_phototropism;
	out << YAML::Key << "m_internodeLength" << YAML::Value << treeGrowthParameters.m_internodeLength;
	out << YAML::Key << "m_growthRate" << YAML::Value << treeGrowthParameters.m_growthRate;
	out << YAML::Key << "m_endNodeThickness" << YAML::Value
		<< treeGrowthParameters.m_endNodeThickness;
	out << YAML::Key << "m_thicknessAccumulateFactor" << YAML::Value
		<< treeGrowthParameters.m_thicknessAccumulateFactor;
	out << YAML::Key << "m_lateralBudFlushingProbability" << YAML::Value
		<< treeGrowthParameters.m_lateralBudFlushingProbability;
	out << YAML::Key << "m_apicalControlBaseDistFactor" << YAML::Value
		<< treeGrowthParameters.m_apicalControlBaseDistFactor;
	out << YAML::Key << "m_apicalDominance" << YAML::Value
		<< treeGrowthParameters.m_apicalDominance;
	out << YAML::Key << "m_apicalDominanceDistanceFactor" << YAML::Value
		<< treeGrowthParameters.m_apicalDominanceDistanceFactor;

	out << YAML::Key << "m_budKillProbability" << YAML::Value
		<< treeGrowthParameters.m_budKillProbability;
	out << YAML::Key << "m_lowBranchPruning" << YAML::Value << treeGrowthParameters.m_lowBranchPruning;
	out << YAML::Key << "m_saggingFactorThicknessReductionMax" << YAML::Value
		<< treeGrowthParameters.m_saggingFactorThicknessReductionMax;

	out << YAML::Key << "m_shootBaseWaterRequirement" << YAML::Value
		<< treeGrowthParameters.m_shootBaseWaterRequirement;
	out << YAML::Key << "m_leafBaseWaterRequirement" << YAML::Value
		<< treeGrowthParameters.m_leafBaseWaterRequirement;
	out << YAML::Key << "m_fruitBaseWaterRequirement" << YAML::Value
		<< treeGrowthParameters.m_fruitBaseWaterRequirement;


	out << YAML::Key << "m_shootProductiveWaterRequirement" << YAML::Value
		<< treeGrowthParameters.m_shootProductiveWaterRequirement;
	out << YAML::Key << "m_leafProductiveWaterRequirement" << YAML::Value
		<< treeGrowthParameters.m_leafProductiveWaterRequirement;
	out << YAML::Key << "m_fruitProductiveWaterRequirement" << YAML::Value
		<< treeGrowthParameters.m_fruitProductiveWaterRequirement;
	out << YAML::EndMap;
}
void SerializeRootGrowthParameters(const std::string& name, const RootGrowthParameters& rootGrowthParameters, YAML::Emitter& out) {
	out << YAML::Key << name << YAML::BeginMap;
	out << YAML::Key << "m_rootNodeLength" << YAML::Value << rootGrowthParameters.m_rootNodeLength;
	out << YAML::Key << "m_growthRate" << YAML::Value << rootGrowthParameters.m_growthRate;
	out << YAML::Key << "m_endNodeThicknessAndControl" << YAML::Value
		<< rootGrowthParameters.m_endNodeThicknessAndControl;
	out << YAML::Key << "m_thicknessLengthAccumulate" << YAML::Value
		<< rootGrowthParameters.m_thicknessLengthAccumulate;

	out << YAML::Key << "m_branchingAngleMeanVariance" << YAML::Value
		<< rootGrowthParameters.m_branchingAngleMeanVariance;
	out << YAML::Key << "m_rollAngleMeanVariance" << YAML::Value
		<< rootGrowthParameters.m_rollAngleMeanVariance;
	out << YAML::Key << "m_apicalAngleMeanVariance" << YAML::Value
		<< rootGrowthParameters.m_apicalAngleMeanVariance;
	out << YAML::Key << "m_auxinTransportLoss" << YAML::Value << rootGrowthParameters.m_auxinTransportLoss;
	out << YAML::Key << "m_tropismSwitchingProbability" << YAML::Value << rootGrowthParameters.m_tropismSwitchingProbability;
	out << YAML::Key << "m_tropismSwitchingProbabilityDistanceFactor" << YAML::Value << rootGrowthParameters.m_tropismSwitchingProbabilityDistanceFactor;
	out << YAML::Key << "m_tropismIntensity" << YAML::Value << rootGrowthParameters.m_tropismIntensity;
	out << YAML::Key << "m_baseBranchingProbability" << YAML::Value << rootGrowthParameters.m_baseBranchingProbability;
	out << YAML::Key << "m_branchingProbabilityChildrenDecrease" << YAML::Value << rootGrowthParameters.m_branchingProbabilityChildrenDecrease;
	out << YAML::Key << "m_branchingProbabilityDistanceDecrease" << YAML::Value << rootGrowthParameters.m_branchingProbabilityDistanceDecrease;
	out << YAML::EndMap;
}
void TreeDescriptor::Serialize(YAML::Emitter& out) {
	SerializeTreeGrowthParameters("m_treeGrowthParameters", m_treeGrowthParameters, out);
	SerializeRootGrowthParameters("m_rootGrowthParameters", m_rootGrowthParameters, out);
}

void DeserializeTreeGrowthParameters(const std::string& name, TreeGrowthParameters& treeGrowthParameters, const YAML::Node& in) {
	if (in[name]) {
		auto& param = in[name];
		if (param["m_lateralBudCount"]) treeGrowthParameters.m_lateralBudCount = param["m_lateralBudCount"].as<int>();
		if (param["m_fruitBudCount"]) treeGrowthParameters.m_fruitBudCount = param["m_fruitBudCount"].as<int>();
		if (param["m_leafBudCount"]) treeGrowthParameters.m_leafBudCount = param["m_leafBudCount"].as<int>();
		if (param["m_branchingAngleMeanVariance"]) treeGrowthParameters.m_branchingAngleMeanVariance = param["m_branchingAngleMeanVariance"].as<glm::vec2>();
		if (param["m_rollAngleMeanVariance"]) treeGrowthParameters.m_rollAngleMeanVariance = param["m_rollAngleMeanVariance"].as<glm::vec2>();
		if (param["m_apicalAngleMeanVariance"]) treeGrowthParameters.m_apicalAngleMeanVariance = param["m_apicalAngleMeanVariance"].as<glm::vec2>();
		if (param["m_gravitropism"]) treeGrowthParameters.m_gravitropism = param["m_gravitropism"].as<float>();
		if (param["m_phototropism"]) treeGrowthParameters.m_phototropism = param["m_phototropism"].as<float>();
		if (param["m_internodeLength"]) treeGrowthParameters.m_internodeLength = param["m_internodeLength"].as<float>();
		if (param["m_growthRate"]) treeGrowthParameters.m_growthRate = param["m_growthRate"].as<float>();
		if (param["m_endNodeThickness"]) treeGrowthParameters.m_endNodeThickness = param["m_endNodeThickness"].as<float>();
		if (param["m_thicknessAccumulateFactor"]) treeGrowthParameters.m_thicknessAccumulateFactor = param["m_thicknessAccumulateFactor"].as<float>();

		if (param["m_lateralBudFlushingProbability"]) treeGrowthParameters.m_lateralBudFlushingProbability = param["m_lateralBudFlushingProbability"].as<float>();
		if (param["m_apicalControlBaseDistFactor"]) treeGrowthParameters.m_apicalControlBaseDistFactor = param["m_apicalControlBaseDistFactor"].as<glm::vec2>();
		if (param["m_apicalDominance"]) treeGrowthParameters.m_apicalDominance = param["m_apicalDominance"].as<float>();
		if (param["m_apicalDominanceDistanceFactor"]) treeGrowthParameters.m_apicalDominanceDistanceFactor = param["m_apicalDominanceDistanceFactor"].as<float>();

		if (param["m_budKillProbability"]) treeGrowthParameters.m_budKillProbability = param["m_budKillProbability"].as<float>();
		if (param["m_lowBranchPruning"]) treeGrowthParameters.m_lowBranchPruning = param["m_lowBranchPruning"].as<float>();
		if (param["m_saggingFactorThicknessReductionMax"]) treeGrowthParameters.m_saggingFactorThicknessReductionMax = param["m_saggingFactorThicknessReductionMax"].as<glm::vec3>();

		if (param["m_shootBaseWaterRequirement"]) treeGrowthParameters.m_shootBaseWaterRequirement = param["m_shootBaseWaterRequirement"].as<float>();
		if (param["m_leafBaseWaterRequirement"]) treeGrowthParameters.m_leafBaseWaterRequirement = param["m_leafBaseWaterRequirement"].as<float>();
		if (param["m_fruitBaseWaterRequirement"]) treeGrowthParameters.m_fruitBaseWaterRequirement = param["m_fruitBaseWaterRequirement"].as<float>();

		if (param["m_shootProductiveWaterRequirement"]) treeGrowthParameters.m_shootProductiveWaterRequirement = param["m_shootProductiveWaterRequirement"].as<float>();
		if (param["m_leafProductiveWaterRequirement"]) treeGrowthParameters.m_leafProductiveWaterRequirement = param["m_leafProductiveWaterRequirement"].as<float>();
		if (param["m_fruitProductiveWaterRequirement"]) treeGrowthParameters.m_fruitProductiveWaterRequirement = param["m_fruitProductiveWaterRequirement"].as<float>();

	}
}
void DeserializeRootGrowthParameters(const std::string& name, RootGrowthParameters& rootGrowthParameters, const YAML::Node& in) {
	if (in[name]) {
		auto& param = in[name];
		if (param["m_rootNodeLength"]) rootGrowthParameters.m_rootNodeLength = param["m_rootNodeLength"].as<float>();
		if (param["m_growthRate"]) rootGrowthParameters.m_growthRate = param["m_growthRate"].as<float>();
		if (param["m_endNodeThicknessAndControl"]) rootGrowthParameters.m_endNodeThicknessAndControl = param["m_endNodeThicknessAndControl"].as<glm::vec2>();
		if (param["m_thicknessLengthAccumulate"]) rootGrowthParameters.m_thicknessLengthAccumulate = param["m_thicknessLengthAccumulate"].as<float>();

		if (param["m_branchingAngleMeanVariance"]) rootGrowthParameters.m_branchingAngleMeanVariance = param["m_branchingAngleMeanVariance"].as<glm::vec2>();
		if (param["m_rollAngleMeanVariance"]) rootGrowthParameters.m_rollAngleMeanVariance = param["m_rollAngleMeanVariance"].as<glm::vec2>();
		if (param["m_apicalAngleMeanVariance"]) rootGrowthParameters.m_apicalAngleMeanVariance = param["m_apicalAngleMeanVariance"].as<glm::vec2>();

		if (param["m_auxinTransportLoss"]) rootGrowthParameters.m_auxinTransportLoss = param["m_auxinTransportLoss"].as<float>();

		if (param["m_tropismSwitchingProbability"]) rootGrowthParameters.m_tropismSwitchingProbability = param["m_tropismSwitchingProbability"].as<float>();
		if (param["m_tropismSwitchingProbabilityDistanceFactor"]) rootGrowthParameters.m_tropismSwitchingProbabilityDistanceFactor = param["m_tropismSwitchingProbabilityDistanceFactor"].as<float>();
		if (param["m_tropismIntensity"]) rootGrowthParameters.m_tropismIntensity = param["m_tropismIntensity"].as<float>();

		if (param["m_baseBranchingProbability"]) rootGrowthParameters.m_baseBranchingProbability = param["m_baseBranchingProbability"].as<float>();
		if (param["m_branchingProbabilityChildrenDecrease"]) rootGrowthParameters.m_branchingProbabilityChildrenDecrease = param["m_branchingProbabilityChildrenDecrease"].as<float>();
		if (param["m_branchingProbabilityDistanceDecrease"]) rootGrowthParameters.m_branchingProbabilityDistanceDecrease = param["m_branchingProbabilityDistanceDecrease"].as<float>();

	}
}
void TreeDescriptor::Deserialize(const YAML::Node& in) {
	DeserializeTreeGrowthParameters("m_treeGrowthParameters", m_treeGrowthParameters, in);
	DeserializeRootGrowthParameters("m_rootGrowthParameters", m_rootGrowthParameters, in);
}