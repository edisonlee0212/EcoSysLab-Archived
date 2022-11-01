//
// Created by lllll on 10/24/2022.
//

#include "Tree.hpp"
#include "Graphics.hpp"
#include "EditorLayer.hpp"
#include "Application.hpp"
#include "BranchMeshGenerator.hpp"

using namespace EcoSysLab;

void Tree::OnInspect() {
    static Handle handle;
    static TreeVisualizer<BranchGrowthData, InternodeGrowthData> treeVisualizer;
    static MeshGeneratorSettings meshGeneratorSettings;
    static GlobalTransform globalTransform;
    if (Editor::DragAndDropButton<TreeDescriptor>(m_treeDescriptor, "TreeDescriptor", true)) {
        m_treeModel.Clear();
        treeVisualizer.Reset();
    }
    if (GetHandle() != handle) {
        treeVisualizer.Reset();
        handle = GetHandle();
    }
    auto tempGlobalTransform = GetScene()->GetDataComponent<GlobalTransform>(GetOwner());
    if (tempGlobalTransform.m_value != globalTransform.m_value) {
        globalTransform = tempGlobalTransform;
        treeVisualizer.SyncMatrices(m_treeModel.m_treeStructure->Peek(treeVisualizer.m_iteration), globalTransform);
    }
    if (m_treeDescriptor.Get<TreeDescriptor>()) {
        auto &parameters = m_treeDescriptor.Get<TreeDescriptor>()->m_treeStructuralGrowthParameters;
        if (!m_treeModel.IsInitialized()) m_treeModel.Initialize(parameters);
        ImGui::Checkbox("Enable History", &m_enableHistory);
        if (ImGui::Button("Grow")) {
            if (m_enableHistory) m_treeModel.m_treeStructure->Step();
            m_treeModel.Grow({999}, parameters);
            treeVisualizer.Reset();
            treeVisualizer.m_iteration = m_treeModel.m_treeStructure->CurrentIteration();
        }
        treeVisualizer.OnInspect(*m_treeModel.m_treeStructure, globalTransform);
        if (ImGui::Button("Generate Mesh")) {
            std::vector<Vertex> vertices;
            std::vector<unsigned int> indices;
            BranchMeshGenerator::Generate(m_treeModel.m_treeStructure->Skeleton(), vertices, indices, meshGeneratorSettings);
            auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
            auto material = ProjectManager::CreateTemporaryAsset<Material>();
            material->SetProgram(DefaultResources::GLPrograms::StandardProgram);
            mesh->SetVertices(17, vertices, indices);
            auto meshRenderer = GetScene()->GetOrSetPrivateComponent<MeshRenderer>(GetOwner()).lock();
            meshRenderer->m_mesh = mesh;
            meshRenderer->m_material = material;
        }
        if (ImGui::Button("Clear")) {
            m_treeModel.Clear();
            treeVisualizer.Reset();
        }
    }
}

void Tree::OnCreate() {
}

void Tree::OnDestroy() {
    m_treeModel.Clear();
}

void TreeDescriptor::OnCreate() {

}

void TreeDescriptor::OnInspect() {
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

        ImGui::DragFloat("Lateral bud lighting factor",
                         &m_treeStructuralGrowthParameters.m_lateralBudFlushingLightingFactor, 0.01f);
        ImGui::DragFloat2("Kill probability apical/lateral",
                          &m_treeStructuralGrowthParameters.m_budKillProbabilityApicalLateral.x, 0.01f);
        ImGui::TreePop();
    }
    if (ImGui::TreeNodeEx("Internode")) {
        ImGui::DragInt("Random pruning Order Protection",
                       &m_treeStructuralGrowthParameters.m_randomPruningOrderProtection);
        ImGui::DragFloat3("Random pruning base/age/max", &m_treeStructuralGrowthParameters.m_randomPruningBaseAgeMax.x,
                          0.0001f, -1.0f, 1.0f, "%.5f");
        const float maxAgeBeforeMaxCutOff =
                (m_treeStructuralGrowthParameters.m_randomPruningBaseAgeMax.z -
                 m_treeStructuralGrowthParameters.m_randomPruningBaseAgeMax.x) /
                m_treeStructuralGrowthParameters.m_randomPruningBaseAgeMax.y;
        ImGui::Text("Max age before reaching max: %.3f", maxAgeBeforeMaxCutOff);
        ImGui::DragFloat("Low Branch Pruning", &m_treeStructuralGrowthParameters.m_lowBranchPruning, 0.01f);
        ImGui::DragFloat3("Sagging thickness/reduction/max",
                          &m_treeStructuralGrowthParameters.m_saggingFactorThicknessReductionMax.x, 0.01f);
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
    out << YAML::Key << "m_lateralBudFlushingLightingFactor" << YAML::Value
        << m_treeStructuralGrowthParameters.m_lateralBudFlushingLightingFactor;
    out << YAML::Key << "m_budKillProbabilityApicalLateral" << YAML::Value
        << m_treeStructuralGrowthParameters.m_budKillProbabilityApicalLateral;
    out << YAML::Key << "m_randomPruningOrderProtection" << YAML::Value
        << m_treeStructuralGrowthParameters.m_randomPruningOrderProtection;
    out << YAML::Key << "m_randomPruningBaseAgeMax" << YAML::Value
        << m_treeStructuralGrowthParameters.m_randomPruningBaseAgeMax;
    out << YAML::Key << "m_lowBranchPruning" << YAML::Value << m_treeStructuralGrowthParameters.m_lowBranchPruning;
    out << YAML::Key << "m_saggingFactorThicknessReductionMax" << YAML::Value
        << m_treeStructuralGrowthParameters.m_saggingFactorThicknessReductionMax;
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
    if (in["m_lateralBudFlushingLightingFactor"]) m_treeStructuralGrowthParameters.m_lateralBudFlushingLightingFactor = in["m_lateralBudFlushingLightingFactor"].as<float>();
    if (in["m_budKillProbabilityApicalLateral"]) m_treeStructuralGrowthParameters.m_budKillProbabilityApicalLateral = in["m_budKillProbabilityApicalLateral"].as<glm::vec2>();
    if (in["m_randomPruningOrderProtection"]) m_treeStructuralGrowthParameters.m_randomPruningOrderProtection = in["m_randomPruningOrderProtection"].as<int>();
    if (in["m_randomPruningBaseAgeMax"]) m_treeStructuralGrowthParameters.m_randomPruningBaseAgeMax = in["m_randomPruningBaseAgeMax"].as<glm::vec3>();
    if (in["m_lowBranchPruning"]) m_treeStructuralGrowthParameters.m_lowBranchPruning = in["m_lowBranchPruning"].as<float>();
    if (in["m_saggingFactorThicknessReductionMax"]) m_treeStructuralGrowthParameters.m_saggingFactorThicknessReductionMax = in["m_saggingFactorThicknessReductionMax"].as<glm::vec3>();
}





