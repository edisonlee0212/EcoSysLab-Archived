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
        auto &parameters = m_treeDescriptor.Get<TreeDescriptor>()->m_treeStructuralGrowthParameters;
        ImGui::Checkbox("Enable History", &m_enableHistory);
        ImGui::DragFloat("Water", &m_growthNutrients.m_water, 1.0f, 0.0f, 99999.0f);

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
    m_growthNutrients = {999};
    m_treeDescriptor.Clear();
    m_enableHistory = true;
}

bool Tree::TryGrow() {
    auto treeDescriptor = m_treeDescriptor.Get<TreeDescriptor>();
    if (!treeDescriptor) return false;
    if (m_enableHistory) m_treeModel.Step();
    return m_treeModel.Grow(m_growthNutrients, treeDescriptor->m_treeStructuralGrowthParameters,
                            treeDescriptor->m_rootGrowthParameters);
}

void Tree::GenerateMesh(const MeshGeneratorSettings &meshGeneratorSettings) {
    auto scene = GetScene();
    auto self = GetOwner();
    auto children = scene->GetChildren(self);
    for(const auto& child : children){
        auto name = scene->GetEntityName(child);
        if(name == "Tree Branch"){
            scene->DeleteEntity(child);
        }else if(name == "Root Branch"){
            scene->DeleteEntity(child);
        }else if(name == "Tree Foliage"){
            scene->DeleteEntity(child);
        }else if(name == "Tree Fruit"){
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

void TreeDescriptor::OnInspect() {
    if (ImGui::Button("Instantiate")) {
        auto scene = Application::GetActiveScene();
        auto treeEntity = scene->CreateEntity(GetTitle());
        auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
        tree->m_treeDescriptor = ProjectManager::GetAsset(GetHandle());
    }
    if (ImGui::TreeNodeEx("Structure", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::DragInt("Lateral bud per node", &m_treeStructuralGrowthParameters.m_lateralBudCount);
        ImGui::DragFloat2("Branching Angle mean/var", &m_treeStructuralGrowthParameters.m_branchingAngleMeanVariance.x,
                          0.01f);
        ImGui::DragFloat2("Roll Angle mean/var", &m_treeStructuralGrowthParameters.m_rollAngleMeanVariance.x, 0.01f);
        ImGui::DragFloat2("Apical Angle mean/var", &m_treeStructuralGrowthParameters.m_apicalAngleMeanVariance.x,
                          0.01f);
        ImGui::DragFloat("Gravitropism", &m_treeStructuralGrowthParameters.m_gravitropism, 0.01f);
        ImGui::DragFloat("Phototropism", &m_treeStructuralGrowthParameters.m_phototropism, 0.01f);
        ImGui::DragFloat("Internode length", &m_treeStructuralGrowthParameters.m_internodeLength, 0.01f);
        ImGui::DragFloat("Growth rate", &m_treeStructuralGrowthParameters.m_growthRate, 0.01f);
        ImGui::DragFloat2("Thickness min/factor", &m_treeStructuralGrowthParameters.m_endNodeThicknessAndControl.x,
                          0.01f);
        ImGui::TreePop();
    }
    if (ImGui::TreeNodeEx("Bud", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::DragFloat("Lateral bud flushing probability",
                         &m_treeStructuralGrowthParameters.m_lateralBudFlushingProbability, 0.01f);
        ImGui::DragFloat2("Apical control base/dist", &m_treeStructuralGrowthParameters.m_apicalControlBaseDistFactor.x,
                          0.01f);
        ImGui::DragFloat3("Apical dominance base/age/dist",
                          &m_treeStructuralGrowthParameters.m_apicalDominanceBaseAgeDist.x, 0.01f);
        int maxAgeBeforeInhibitorEnds = m_treeStructuralGrowthParameters.m_apicalDominanceBaseAgeDist.x /
                                        m_treeStructuralGrowthParameters.m_apicalDominanceBaseAgeDist.y;
        float maxDistance = m_treeStructuralGrowthParameters.m_apicalDominanceBaseAgeDist.x /
                            m_treeStructuralGrowthParameters.m_apicalDominanceBaseAgeDist.z;
        ImGui::Text("Max age / distance: [%i, %.3f]", maxAgeBeforeInhibitorEnds, maxDistance);

        ImGui::DragFloat2("Kill probability apical/lateral",
                          &m_treeStructuralGrowthParameters.m_budKillProbabilityApicalLateral.x, 0.01f);

        ImGui::DragFloat3("Base resource shoot/leaf/fruit",
                          &m_treeStructuralGrowthParameters.m_baseResourceRequirementFactor.x, 0.01f);

        ImGui::DragFloat3("Productive resource shoot/leaf/fruit",
                          &m_treeStructuralGrowthParameters.m_productiveResourceRequirementFactor.x, 0.01f);

        ImGui::TreePop();
    }
    if (ImGui::TreeNodeEx("Internode")) {
        ImGui::DragFloat("Low Branch Pruning", &m_treeStructuralGrowthParameters.m_lowBranchPruning, 0.01f);
        ImGui::DragFloat3("Sagging thickness/reduction/max",
                          &m_treeStructuralGrowthParameters.m_saggingFactorThicknessReductionMax.x, 0.01f, 0.0f, 1.0f, "%.5f");


        ImGui::TreePop();
    }
}

void TreeDescriptor::CollectAssetRef(std::vector<AssetRef> &list) {

}

void TreeDescriptor::Serialize(YAML::Emitter &out) {
    out << YAML::Key << "m_lateralBudCount" << YAML::Value << m_treeStructuralGrowthParameters.m_lateralBudCount;
    out << YAML::Key << "m_branchingAngleMeanVariance" << YAML::Value
        << m_treeStructuralGrowthParameters.m_branchingAngleMeanVariance;
    out << YAML::Key << "m_rollAngleMeanVariance" << YAML::Value
        << m_treeStructuralGrowthParameters.m_rollAngleMeanVariance;
    out << YAML::Key << "m_apicalAngleMeanVariance" << YAML::Value
        << m_treeStructuralGrowthParameters.m_apicalAngleMeanVariance;
    out << YAML::Key << "m_gravitropism" << YAML::Value << m_treeStructuralGrowthParameters.m_gravitropism;
    out << YAML::Key << "m_phototropism" << YAML::Value << m_treeStructuralGrowthParameters.m_phototropism;
    out << YAML::Key << "m_internodeLength" << YAML::Value << m_treeStructuralGrowthParameters.m_internodeLength;
    out << YAML::Key << "m_growthRate" << YAML::Value << m_treeStructuralGrowthParameters.m_growthRate;
    out << YAML::Key << "m_endNodeThicknessAndControl" << YAML::Value
        << m_treeStructuralGrowthParameters.m_endNodeThicknessAndControl;
    out << YAML::Key << "m_lateralBudFlushingProbability" << YAML::Value
        << m_treeStructuralGrowthParameters.m_lateralBudFlushingProbability;
    out << YAML::Key << "m_apicalControlBaseDistFactor" << YAML::Value
        << m_treeStructuralGrowthParameters.m_apicalControlBaseDistFactor;
    out << YAML::Key << "m_apicalDominanceBaseAgeDist" << YAML::Value
        << m_treeStructuralGrowthParameters.m_apicalDominanceBaseAgeDist;
    out << YAML::Key << "m_budKillProbabilityApicalLateral" << YAML::Value
        << m_treeStructuralGrowthParameters.m_budKillProbabilityApicalLateral;
    out << YAML::Key << "m_lowBranchPruning" << YAML::Value << m_treeStructuralGrowthParameters.m_lowBranchPruning;
    out << YAML::Key << "m_saggingFactorThicknessReductionMax" << YAML::Value
        << m_treeStructuralGrowthParameters.m_saggingFactorThicknessReductionMax;

    out << YAML::Key << "m_baseResourceRequirementFactor" << YAML::Value
        << m_treeStructuralGrowthParameters.m_baseResourceRequirementFactor;
    out << YAML::Key << "m_productiveResourceRequirementFactor" << YAML::Value
        << m_treeStructuralGrowthParameters.m_productiveResourceRequirementFactor;
}

void TreeDescriptor::Deserialize(const YAML::Node &in) {
    if (in["m_lateralBudCount"]) m_treeStructuralGrowthParameters.m_lateralBudCount = in["m_lateralBudCount"].as<int>();
    if (in["m_branchingAngleMeanVariance"]) m_treeStructuralGrowthParameters.m_branchingAngleMeanVariance = in["m_branchingAngleMeanVariance"].as<glm::vec2>();
    if (in["m_rollAngleMeanVariance"]) m_treeStructuralGrowthParameters.m_rollAngleMeanVariance = in["m_rollAngleMeanVariance"].as<glm::vec2>();
    if (in["m_apicalAngleMeanVariance"]) m_treeStructuralGrowthParameters.m_apicalAngleMeanVariance = in["m_apicalAngleMeanVariance"].as<glm::vec2>();
    if (in["m_gravitropism"]) m_treeStructuralGrowthParameters.m_gravitropism = in["m_gravitropism"].as<float>();
    if (in["m_phototropism"]) m_treeStructuralGrowthParameters.m_phototropism = in["m_phototropism"].as<float>();
    if (in["m_internodeLength"]) m_treeStructuralGrowthParameters.m_internodeLength = in["m_internodeLength"].as<float>();
    if (in["m_growthRate"]) m_treeStructuralGrowthParameters.m_growthRate = in["m_growthRate"].as<float>();
    if (in["m_endNodeThicknessAndControl"]) m_treeStructuralGrowthParameters.m_endNodeThicknessAndControl = in["m_endNodeThicknessAndControl"].as<glm::vec2>();
    if (in["m_lateralBudFlushingProbability"]) m_treeStructuralGrowthParameters.m_lateralBudFlushingProbability = in["m_lateralBudFlushingProbability"].as<float>();
    if (in["m_apicalControlBaseDistFactor"]) m_treeStructuralGrowthParameters.m_apicalControlBaseDistFactor = in["m_apicalControlBaseDistFactor"].as<glm::vec2>();
    if (in["m_apicalDominanceBaseAgeDist"]) m_treeStructuralGrowthParameters.m_apicalDominanceBaseAgeDist = in["m_apicalDominanceBaseAgeDist"].as<glm::vec3>();
    if (in["m_budKillProbabilityApicalLateral"]) m_treeStructuralGrowthParameters.m_budKillProbabilityApicalLateral = in["m_budKillProbabilityApicalLateral"].as<glm::vec2>();
    if (in["m_lowBranchPruning"]) m_treeStructuralGrowthParameters.m_lowBranchPruning = in["m_lowBranchPruning"].as<float>();
    if (in["m_saggingFactorThicknessReductionMax"]) m_treeStructuralGrowthParameters.m_saggingFactorThicknessReductionMax = in["m_saggingFactorThicknessReductionMax"].as<glm::vec3>();

    if (in["m_baseResourceRequirementFactor"]) m_treeStructuralGrowthParameters.m_baseResourceRequirementFactor = in["m_baseResourceRequirementFactor"].as<glm::vec3>();
    if (in["m_productiveResourceRequirementFactor"]) m_treeStructuralGrowthParameters.m_productiveResourceRequirementFactor = in["m_productiveResourceRequirementFactor"].as<glm::vec3>();
}





