//
// Created by lllll on 10/24/2022.
//

#include "Tree.hpp"
#include "Graphics.hpp"
#include "EditorLayer.hpp"
#include "Application.hpp"
#include "BranchMeshGenerator.hpp"
#include "Soil.hpp"
#include "Climate.hpp"
#include "TreeVisualizationLayer.hpp"

using namespace EcoSysLab;

void Tree::OnInspect() {
	static MeshGeneratorSettings meshGeneratorSettings;
	bool modelChanged = false;
	if (Editor::DragAndDropButton<TreeDescriptor>(m_treeDescriptor, "TreeDescriptor", true)) {
		m_treeModel.Clear();
		modelChanged = true;
	}
	if (m_treeDescriptor.Get<TreeDescriptor>()) {
		auto& parameters = m_treeDescriptor.Get<TreeDescriptor>()->m_treeGrowthParameters;
		ImGui::Checkbox("Enable History", &m_enableHistory);
		if (ImGui::Button("Grow")) {
			TryGrow();
			modelChanged = true;
		}
		static int iterations = 5;
		ImGui::DragInt("Iterations", &iterations, 1, 1, 100);
		if (ImGui::Button(("Grow " + std::to_string(iterations) + " iterations").c_str())) {
			for (int i = 0; i < iterations; i++) TryGrow();
			modelChanged = true;
		}
		if (ImGui::TreeNode("Mesh generation")) {
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
		const auto treeVisualizationLayer = Application::GetLayer<TreeVisualizationLayer>();
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
	m_enableHistory = true;
}

bool Tree::TryGrow() {
	const auto scene = GetScene();
	const auto owner = GetOwner();
	const auto treeDescriptor = m_treeDescriptor.Get<TreeDescriptor>();
	if (!treeDescriptor) return false;
	std::shared_ptr<Soil> soil;
	std::shared_ptr<Climate> climate;
	if (!m_soil.Get<Soil>()) {
		UNIENGINE_ERROR("No soil model!");
		soil = scene->GetOrSetPrivateComponent<Soil>(owner).lock();
	}else
	{
		soil = m_soil.Get<Soil>();
	}
	if (!m_climate.Get<Climate>()) {
		UNIENGINE_ERROR("No climate model!");
		climate = scene->GetOrSetPrivateComponent<Climate>(owner).lock();
	}
	else
	{
		climate = m_climate.Get<Climate>();
	}
	if (m_enableHistory) m_treeModel.Step();
	return m_treeModel.Grow(soil->m_soilModel, climate->m_climateModel,
		treeDescriptor->m_rootGrowthParameters, treeDescriptor->m_treeGrowthParameters);
}

void Tree::GenerateMesh(const MeshGeneratorSettings& meshGeneratorSettings) {
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
	if(branchEntity.GetIndex() == 0)
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
	std::vector<Vertex> vertices;
	std::vector<unsigned int> indices;
	{
		BranchMeshGenerator<SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> meshGenerator;
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
	vertices.clear();
	indices.clear();
	{
		BranchMeshGenerator<RootSkeletonGrowthData, RootBranchGrowthData, RootInternodeGrowthData> meshGenerator;
		meshGenerator.Generate(m_treeModel.RefRootSkeleton(), vertices, indices,
			meshGeneratorSettings);
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


void OnInspectTreeGrowthParameters(TreeGrowthParameters& treeGrowthParameters) {
	if (ImGui::TreeNodeEx("Tree Parameters", ImGuiTreeNodeFlags_DefaultOpen)) {
		if (ImGui::TreeNodeEx("Structure", ImGuiTreeNodeFlags_DefaultOpen)) {
			ImGui::DragInt("Lateral bud count", &treeGrowthParameters.m_lateralBudCount);
			ImGui::DragInt("Fruit bud count", &treeGrowthParameters.m_fruitBudCount);
			ImGui::DragInt("Leaf bud count", &treeGrowthParameters.m_leafBudCount);
			ImGui::DragFloat2("Branching Angle mean/var", &treeGrowthParameters.m_branchingAngleMeanVariance.x,
				0.01f);
			ImGui::DragFloat2("Roll Angle mean/var", &treeGrowthParameters.m_rollAngleMeanVariance.x, 0.01f);
			ImGui::DragFloat2("Apical Angle mean/var", &treeGrowthParameters.m_apicalAngleMeanVariance.x,
				0.01f);
			ImGui::DragFloat("Gravitropism", &treeGrowthParameters.m_gravitropism, 0.01f);
			ImGui::DragFloat("Phototropism", &treeGrowthParameters.m_phototropism, 0.01f);
			ImGui::DragFloat("Internode length", &treeGrowthParameters.m_internodeLength, 0.01f);
			ImGui::DragFloat("Growth rate", &treeGrowthParameters.m_growthRate, 0.01f);
			ImGui::DragFloat2("Thickness min/factor", &treeGrowthParameters.m_endNodeThicknessAndControl.x,
				0.01f);
			ImGui::TreePop();
		}
		if (ImGui::TreeNodeEx("Bud", ImGuiTreeNodeFlags_DefaultOpen)) {
			ImGui::DragFloat("Lateral bud flushing probability",
				&treeGrowthParameters.m_lateralBudFlushingProbability, 0.01f);
			ImGui::DragFloat2("Apical control base/dist", &treeGrowthParameters.m_apicalControlBaseDistFactor.x,
				0.01f);
			ImGui::DragFloat3("Apical dominance base/age/dist",
				&treeGrowthParameters.m_apicalDominanceBaseAgeDist.x, 0.01f);
			int maxAgeBeforeInhibitorEnds = treeGrowthParameters.m_apicalDominanceBaseAgeDist.x /
				treeGrowthParameters.m_apicalDominanceBaseAgeDist.y;
			float maxDistance = treeGrowthParameters.m_apicalDominanceBaseAgeDist.x /
				treeGrowthParameters.m_apicalDominanceBaseAgeDist.z;
			ImGui::Text("Max age / distance: [%i, %.3f]", maxAgeBeforeInhibitorEnds, maxDistance);

			ImGui::DragFloat("Kill probability",
				&treeGrowthParameters.m_budKillProbability, 0.01f);

			ImGui::DragFloat3("Base resource shoot/leaf/fruit",
				&treeGrowthParameters.m_baseResourceRequirementFactor.x, 0.01f);

			ImGui::DragFloat3("Productive resource shoot/leaf/fruit",
				&treeGrowthParameters.m_productiveResourceRequirementFactor.x, 0.01f);

			ImGui::TreePop();
		}
		if (ImGui::TreeNodeEx("Internode")) {
			ImGui::DragFloat("Low Branch Pruning", &treeGrowthParameters.m_lowBranchPruning, 0.01f);
			ImGui::DragFloat3("Sagging thickness/reduction/max",
				&treeGrowthParameters.m_saggingFactorThicknessReductionMax.x, 0.01f, 0.0f, 1.0f, "%.5f");
			ImGui::TreePop();
		}
		ImGui::TreePop();
	}
}

void OnInspectRootGrowthParameters(RootGrowthParameters& rootGrowthParameters) {
	if (ImGui::TreeNodeEx("Root Parameters", ImGuiTreeNodeFlags_DefaultOpen)) {
		if (ImGui::TreeNodeEx("Structure", ImGuiTreeNodeFlags_DefaultOpen)) {
			ImGui::DragFloat("Root node length", &rootGrowthParameters.m_rootNodeLength, 0.01f);
			ImGui::DragFloat2("Thickness min/factor", &rootGrowthParameters.m_endNodeThicknessAndControl.x,
				0.01f);
			ImGui::DragFloat("Thickness accmu", &rootGrowthParameters.m_thicknessLengthAccumulate, 0.000001f, 0.0f, 1.0f, "%.6f");
			ImGui::DragFloat2("Branching Angle mean/var", &rootGrowthParameters.m_branchingAngleMeanVariance.x,
				0.01f);
			ImGui::DragFloat2("Roll Angle mean/var", &rootGrowthParameters.m_rollAngleMeanVariance.x, 0.01f);
			ImGui::DragFloat2("Apical Angle mean/var", &rootGrowthParameters.m_apicalAngleMeanVariance.x,
				0.01f);
			ImGui::TreePop();
		}
		if (ImGui::TreeNodeEx("Growth", ImGuiTreeNodeFlags_DefaultOpen))
		{
			ImGui::DragFloat("Growth rate", &rootGrowthParameters.m_growthRate, 0.01f);
			ImGui::DragFloat("Auxin loss", &rootGrowthParameters.m_auxinTransportLoss, 0.01f);
			ImGui::DragFloat("Tropism switching prob", &rootGrowthParameters.m_tropismSwitchingProbability, 0.01f);
			ImGui::DragFloat("Tropism switching prob dist factor", &rootGrowthParameters.m_tropismSwitchingProbabilityDistanceFactor, 0.01f);
			ImGui::DragFloat("Tropism intensity", &rootGrowthParameters.m_tropismIntensity, 0.01f);
			ImGui::DragFloat("Branching prob base", &rootGrowthParameters.m_baseBranchingProbability, 0.01f);
			ImGui::DragFloat("Branching prob child decrease", &rootGrowthParameters.m_branchingProbabilityChildrenDecrease, 0.01f);
			ImGui::DragFloat("Branching prob dist decrease", &rootGrowthParameters.m_branchingProbabilityDistanceDecrease, 0.01f);
			ImGui::TreePop();
		}
		ImGui::TreePop();
	}
}

void TreeDescriptor::OnInspect() {
	if (ImGui::Button("Instantiate")) {
		auto scene = Application::GetActiveScene();
		auto treeEntity = scene->CreateEntity(GetTitle());
		auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
		tree->m_treeDescriptor = ProjectManager::GetAsset(GetHandle());
	}
	OnInspectTreeGrowthParameters(m_treeGrowthParameters);
	OnInspectRootGrowthParameters(m_rootGrowthParameters);
}

void TreeDescriptor::CollectAssetRef(std::vector<AssetRef>& list) {

}

void SerializeTreeGrowthParamaters(const std::string& name, const TreeGrowthParameters& treeGrowthParameters, YAML::Emitter& out) {
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
	out << YAML::Key << "m_endNodeThicknessAndControl" << YAML::Value
		<< treeGrowthParameters.m_endNodeThicknessAndControl;
	out << YAML::Key << "m_lateralBudFlushingProbability" << YAML::Value
		<< treeGrowthParameters.m_lateralBudFlushingProbability;
	out << YAML::Key << "m_apicalControlBaseDistFactor" << YAML::Value
		<< treeGrowthParameters.m_apicalControlBaseDistFactor;
	out << YAML::Key << "m_apicalDominanceBaseAgeDist" << YAML::Value
		<< treeGrowthParameters.m_apicalDominanceBaseAgeDist;
	out << YAML::Key << "m_budKillProbability" << YAML::Value
		<< treeGrowthParameters.m_budKillProbability;
	out << YAML::Key << "m_lowBranchPruning" << YAML::Value << treeGrowthParameters.m_lowBranchPruning;
	out << YAML::Key << "m_saggingFactorThicknessReductionMax" << YAML::Value
		<< treeGrowthParameters.m_saggingFactorThicknessReductionMax;

	out << YAML::Key << "m_baseResourceRequirementFactor" << YAML::Value
		<< treeGrowthParameters.m_baseResourceRequirementFactor;
	out << YAML::Key << "m_productiveResourceRequirementFactor" << YAML::Value
		<< treeGrowthParameters.m_productiveResourceRequirementFactor;

	out << YAML::EndMap;
}
void SerializeRootGrowthParamaters(const std::string& name, const RootGrowthParameters& rootGrowthParameters, YAML::Emitter& out) {
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
	SerializeTreeGrowthParamaters("m_treeGrowthParameters", m_treeGrowthParameters, out);
	SerializeRootGrowthParamaters("m_rootGrowthParameters", m_rootGrowthParameters, out);
}

void DeserializeTreeGrowthParamaters(const std::string& name, TreeGrowthParameters& treeGrowthParameters, const YAML::Node& in) {
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
		if (param["m_endNodeThicknessAndControl"]) treeGrowthParameters.m_endNodeThicknessAndControl = param["m_endNodeThicknessAndControl"].as<glm::vec2>();

		if (param["m_lateralBudFlushingProbability"]) treeGrowthParameters.m_lateralBudFlushingProbability = param["m_lateralBudFlushingProbability"].as<float>();
		if (param["m_apicalControlBaseDistFactor"]) treeGrowthParameters.m_apicalControlBaseDistFactor = param["m_apicalControlBaseDistFactor"].as<glm::vec2>();
		if (param["m_apicalDominanceBaseAgeDist"]) treeGrowthParameters.m_apicalDominanceBaseAgeDist = param["m_apicalDominanceBaseAgeDist"].as<glm::vec3>();
		if (param["m_budKillProbability"]) treeGrowthParameters.m_budKillProbability = param["m_budKillProbability"].as<float>();
		if (param["m_lowBranchPruning"]) treeGrowthParameters.m_lowBranchPruning = param["m_lowBranchPruning"].as<float>();
		if (param["m_saggingFactorThicknessReductionMax"]) treeGrowthParameters.m_saggingFactorThicknessReductionMax = param["m_saggingFactorThicknessReductionMax"].as<glm::vec3>();

		if (param["m_baseResourceRequirementFactor"]) treeGrowthParameters.m_baseResourceRequirementFactor = param["m_baseResourceRequirementFactor"].as<glm::vec3>();
		if (param["m_productiveResourceRequirementFactor"]) treeGrowthParameters.m_productiveResourceRequirementFactor = param["m_productiveResourceRequirementFactor"].as<glm::vec3>();
	}
}
void DeserializeRootGrowthParamaters(const std::string& name, RootGrowthParameters& rootGrowthParameters, const YAML::Node& in) {
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
	DeserializeTreeGrowthParamaters("m_treeGrowthParameters", m_treeGrowthParameters, in);
	DeserializeRootGrowthParamaters("m_rootGrowthParameters", m_rootGrowthParameters, in);
}