//
// Created by lllll on 10/24/2022.
//

#include "Tree.hpp"
#include "Graphics.hpp"
#include "EditorLayer.hpp"
#include "Application.hpp"
#include "BranchMeshGenerator.hpp"
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
		auto treeVisualizationLayer = Application::GetLayer<TreeVisualizationLayer>();
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
	auto treeDescriptor = m_treeDescriptor.Get<TreeDescriptor>();
	if (!treeDescriptor) return false;
	if (m_enableHistory) m_treeModel.Step();
	static SoilModel soilModel;
	static ClimateModel climateModel;

	return m_treeModel.Grow(soilModel, climateModel, treeDescriptor->m_treeGrowthParameters,
		treeDescriptor->m_rootGrowthParameters);
}

void Tree::GenerateMesh(const MeshGeneratorSettings& meshGeneratorSettings) {
	auto scene = GetScene();
	auto self = GetOwner();
	auto children = scene->GetChildren(self);
	for (const auto& child : children) {
		auto name = scene->GetEntityName(child);
		if (name == "Tree Branch") {
			scene->DeleteEntity(child);
		}
		else if (name == "Root Branch") {
			scene->DeleteEntity(child);
		}
		else if (name == "Tree Foliage") {
			scene->DeleteEntity(child);
		}
		else if (name == "Tree Fruit") {
			scene->DeleteEntity(child);
		}
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
		auto branchEntity = scene->CreateEntity("Tree Branch");
		scene->SetParent(branchEntity, self);
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
		auto branchEntity = scene->CreateEntity("Root Branch");
		scene->SetParent(branchEntity, self);
		auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(branchEntity).lock();
		meshRenderer->m_mesh = mesh;
		meshRenderer->m_material = material;
	}
}

void TreeDescriptor::OnCreate() {

}


void OnInspectTreeGrowthParameters(TreeGrowthParameters& treeGrowthParameters) {
	if (ImGui::TreeNodeEx("Tree Parameters", ImGuiTreeNodeFlags_DefaultOpen)) {
		if (ImGui::TreeNodeEx("Structure", ImGuiTreeNodeFlags_DefaultOpen)) {
			ImGui::DragInt("Lateral bud per node", &treeGrowthParameters.m_lateralBudCount);
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

			ImGui::DragFloat2("Kill probability apical/lateral",
				&treeGrowthParameters.m_budKillProbabilityApicalLateral.x, 0.01f);

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
	out << YAML::Key << "m_budKillProbabilityApicalLateral" << YAML::Value
		<< treeGrowthParameters.m_budKillProbabilityApicalLateral;
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
	out << YAML::Key << "m_branchingAngleMeanVariance" << YAML::Value
		<< rootGrowthParameters.m_branchingAngleMeanVariance;
	out << YAML::Key << "m_rollAngleMeanVariance" << YAML::Value
		<< rootGrowthParameters.m_rollAngleMeanVariance;
	out << YAML::Key << "m_apicalAngleMeanVariance" << YAML::Value
		<< rootGrowthParameters.m_apicalAngleMeanVariance;
	out << YAML::Key << "m_gravitropism" << YAML::Value << rootGrowthParameters.m_gravitropism;
	out << YAML::Key << "m_rootNodeLength" << YAML::Value << rootGrowthParameters.m_rootNodeLength;
	out << YAML::Key << "m_growthRate" << YAML::Value << rootGrowthParameters.m_growthRate;
	out << YAML::Key << "m_endNodeThicknessAndControl" << YAML::Value
		<< rootGrowthParameters.m_endNodeThicknessAndControl;
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
		if (param["m_budKillProbabilityApicalLateral"]) treeGrowthParameters.m_budKillProbabilityApicalLateral = param["m_budKillProbabilityApicalLateral"].as<glm::vec2>();
		if (param["m_lowBranchPruning"]) treeGrowthParameters.m_lowBranchPruning = param["m_lowBranchPruning"].as<float>();
		if (param["m_saggingFactorThicknessReductionMax"]) treeGrowthParameters.m_saggingFactorThicknessReductionMax = param["m_saggingFactorThicknessReductionMax"].as<glm::vec3>();

		if (param["m_baseResourceRequirementFactor"]) treeGrowthParameters.m_baseResourceRequirementFactor = param["m_baseResourceRequirementFactor"].as<glm::vec3>();
		if (param["m_productiveResourceRequirementFactor"]) treeGrowthParameters.m_productiveResourceRequirementFactor = param["m_productiveResourceRequirementFactor"].as<glm::vec3>();
	}
}
void DeserializeRootGrowthParamaters(const std::string& name, RootGrowthParameters& rootGrowthParameters, const YAML::Node& in) {
	if (in[name]) {
		auto& param = in[name];
		if (param["m_branchingAngleMeanVariance"]) rootGrowthParameters.m_branchingAngleMeanVariance = param["m_branchingAngleMeanVariance"].as<glm::vec2>();
		if (param["m_rollAngleMeanVariance"]) rootGrowthParameters.m_rollAngleMeanVariance = param["m_rollAngleMeanVariance"].as<glm::vec2>();
		if (param["m_apicalAngleMeanVariance"]) rootGrowthParameters.m_apicalAngleMeanVariance = param["m_apicalAngleMeanVariance"].as<glm::vec2>();
		if (param["m_gravitropism"]) rootGrowthParameters.m_gravitropism = param["m_gravitropism"].as<float>();
		if (param["m_rootNodeLength"]) rootGrowthParameters.m_rootNodeLength = param["m_rootNodeLength"].as<float>();
		if (param["m_growthRate"]) rootGrowthParameters.m_growthRate = param["m_growthRate"].as<float>();
		if (param["m_endNodeThicknessAndControl"]) rootGrowthParameters.m_endNodeThicknessAndControl = param["m_endNodeThicknessAndControl"].as<glm::vec2>();
	}
}
void TreeDescriptor::Deserialize(const YAML::Node& in) {
	DeserializeTreeGrowthParamaters("m_treeGrowthParameters", m_treeGrowthParameters, in);
	DeserializeRootGrowthParamaters("m_rootGrowthParameters", m_rootGrowthParameters, in);
}