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
    static TreeVisualizer internodeSelectionData;
    static MeshGeneratorSettings meshGeneratorSettings;
    static GlobalTransform globalTransform;
    if (Editor::DragAndDropButton<TreeDescriptor>(m_treeDescriptor, "TreeDescriptor", true)) {
        m_treeModel.Clear();
        internodeSelectionData.Reset();
    }
    if (GetHandle() != handle) {
        internodeSelectionData.Reset();
        handle = GetHandle();
    }
    auto tempGlobalTransform = GetScene()->GetDataComponent<GlobalTransform>(GetOwner());
    if (tempGlobalTransform.m_value != globalTransform.m_value) {
        globalTransform = tempGlobalTransform;
        internodeSelectionData.Reset();
    }
    if (m_treeDescriptor.Get<TreeDescriptor>()) {
        auto &parameters = m_treeDescriptor.Get<TreeDescriptor>()->m_treeStructuralGrowthParameters;
        if (!m_treeModel.IsInitialized()) m_treeModel.Initialize(parameters);
        if (ImGui::Button("Grow")) {
            m_treeModel.Grow({999}, parameters);
        }
        if (ImGui::Button("Generate Mesh")) {
            std::vector<Vertex> vertices;
            std::vector<unsigned int> indices;
            BranchMeshGenerator::Generate(*m_treeModel.m_tree, vertices, indices, meshGeneratorSettings);
            auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
            auto material = ProjectManager::CreateTemporaryAsset<Material>();
            material->SetProgram(DefaultResources::GLPrograms::StandardProgram);
            mesh->SetVertices(17, vertices, indices);
            auto meshRenderer = GetScene()->GetOrSetPrivateComponent<MeshRenderer>(GetOwner()).lock();
            meshRenderer->m_mesh = mesh;
            meshRenderer->m_material = material;
        }
        internodeSelectionData.OnInspect(m_treeModel, globalTransform);
    }

    if (ImGui::Button("Clear")) {
        m_treeModel.Clear();
        internodeSelectionData.Reset();
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

bool
TreeVisualizer::DrawInternodeInspectionGui(TreeModel &treeModel, InternodeHandle internodeHandle, bool &deleted,
                                           const unsigned int &hierarchyLevel) {
    const int index = m_selectedInternodeHierarchyList.size() - hierarchyLevel - 1;
    if (!m_selectedInternodeHierarchyList.empty() && index >= 0 && index < m_selectedInternodeHierarchyList.size() &&
        m_selectedInternodeHierarchyList[index] == internodeHandle) {
        ImGui::SetNextItemOpen(true);
    }
    const bool opened = ImGui::TreeNodeEx(("Handle: " + std::to_string(internodeHandle)).c_str(),
                                          ImGuiTreeNodeFlags_NoTreePushOnOpen | ImGuiTreeNodeFlags_OpenOnArrow |
                                          ImGuiTreeNodeFlags_NoAutoOpenOnLog |
                                          (m_selectedInternodeHandle == internodeHandle ? ImGuiTreeNodeFlags_Framed
                                                                                        : ImGuiTreeNodeFlags_FramePadding));
    if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
        SetSelectedInternode(treeModel, internodeHandle);
    }

    bool modified = deleted = DrawInternodeMenu(treeModel, internodeHandle);
    if (opened && !deleted) {
        ImGui::TreePush();
        auto &internodeChildren = treeModel.m_tree->RefInternode(internodeHandle).m_children;
        for (int &child: internodeChildren) {
            bool childDeleted = false;
            DrawInternodeInspectionGui(treeModel, child, childDeleted, hierarchyLevel + 1);
            if (childDeleted) {
                treeModel.m_tree->PruneInternode(child);
                modified = true;
                break;
            }
        }
        ImGui::TreePop();
    }
    return modified;
}

bool TreeVisualizer::OnInspect(TreeModel &treeModel, const GlobalTransform &globalTransform) {
    bool needUpdate = false;

    ImGui::Checkbox("Visualization", &m_visualization);
    ImGui::Checkbox("Tree Hierarchy", &m_treeHierarchyGui);
    if (m_treeHierarchyGui) {
        if (ImGui::Begin("Tree Hierarchy")) {
            bool deleted = false;
            needUpdate = DrawInternodeInspectionGui(treeModel, 0, deleted, 0);
            m_selectedInternodeHierarchyList.clear();
        }
        ImGui::End();
        if (m_selectedInternodeHandle >= 0) {
            InspectInternode(treeModel, m_selectedInternodeHandle);
        }
    }
    if (m_visualization) {
        static std::vector<glm::vec4> randomColors;
        if (randomColors.empty()) {
            for (int i = 0; i < 100; i++) {
                randomColors.emplace_back(glm::ballRand(1.0f), 1.0f);
            }
        }
        ImGui::Text("Internode count: %d", m_sortedInternodeList.size());
        ImGui::Text("Branch count: %d", m_sortedBranchList.size());
        if (ImGui::Button("Update")) needUpdate = true;
        if (treeModel.m_tree->GetVersion() != m_version) needUpdate = true;
        if (RayCastSelection(treeModel, globalTransform)) needUpdate = true;
        if (needUpdate) {
            m_version = treeModel.m_tree->GetVersion();
            m_sortedBranchList = treeModel.m_tree->GetSortedBranchList();
            m_sortedInternodeList = treeModel.m_tree->GetSortedInternodeList();
            m_matrices.resize(m_sortedInternodeList.size());
            m_colors.resize(m_sortedInternodeList.size());
            std::vector<std::shared_future<void>> results;
            Jobs::ParallelFor(m_sortedInternodeList.size(), [&](unsigned i) {
                auto internodeHandle = m_sortedInternodeList[i];
                auto &internode = treeModel.m_tree->RefInternode(internodeHandle);
                auto rotation = globalTransform.GetRotation() * internode.m_globalRotation;
                glm::vec3 translation = (globalTransform.m_value * glm::translate(internode.m_globalPosition))[3];
                const auto direction = glm::normalize(rotation * glm::vec3(0, 0, -1));
                const glm::vec3 position2 =
                        translation + internode.m_length * direction;
                rotation = glm::quatLookAt(
                        direction, glm::vec3(direction.y, direction.z, direction.x));
                rotation *= glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f));
                const glm::mat4 rotationTransform = glm::mat4_cast(rotation);
                m_matrices[i] =
                        glm::translate((translation + position2) / 2.0f) *
                        rotationTransform *
                        glm::scale(glm::vec3(
                                internode.m_thickness,
                                glm::distance(translation, position2) / 2.0f,
                                internode.m_thickness));
                if (internodeHandle == m_selectedInternodeHandle) {
                    m_colors[i] = glm::vec4(1, 0, 0, 1);
                } else {
                    m_colors[i] = randomColors[treeModel.m_tree->RefBranch(internode.m_branchHandle).m_data.m_order];
                    if(m_selectedInternodeHandle != -1) m_colors[i].w = 0.5f;
                }
            }, results);
            for (auto &i: results) i.wait();
        }

        if (!m_matrices.empty()) {
            auto editorLayer = Application::GetLayer<EditorLayer>();
            Gizmos::DrawGizmoMeshInstancedColored(
                    DefaultResources::Primitives::Cylinder, editorLayer->m_sceneCamera,
                    editorLayer->m_sceneCameraPosition,
                    editorLayer->m_sceneCameraRotation,
                    *reinterpret_cast<std::vector<glm::vec4> *>(&m_colors),
                    *reinterpret_cast<std::vector<glm::mat4> *>(&m_matrices),
                    glm::mat4(1.0f), 1.0f);

        }
    }

    return needUpdate;
}

void TreeVisualizer::InspectInternode(TreeModel &treeModel, InternodeHandle internodeHandle) {
    if(internodeHandle < 0) return;
}

void TreeVisualizer::Reset() {
    m_version = -1;
    m_selectedInternodeHandle = -1;
    m_selectedInternodeHierarchyList.clear();
    m_sortedInternodeList.clear();
    m_sortedBranchList.clear();
    m_matrices.clear();
}

bool TreeVisualizer::DrawInternodeMenu(TreeModel &treeModel, InternodeHandle internodeHandle) {
    bool deleted = false;
    if (ImGui::BeginPopupContextItem(std::to_string(internodeHandle).c_str())) {
        ImGui::Text(("Handle: " + std::to_string(internodeHandle)).c_str());
        if (ImGui::Button("Delete")) {
            deleted = true;
        }
        ImGui::EndPopup();
    }
    return deleted;
}

void
TreeVisualizer::SetSelectedInternode(TreeModel &treeModel, InternodeHandle internodeHandle) {
    if (internodeHandle == m_selectedInternodeHandle)
        return;
    m_selectedInternodeHierarchyList.clear();
    if (internodeHandle < 0) {
        m_selectedInternodeHandle = -1;
        return;
    }
    m_selectedInternodeHandle = internodeHandle;
    auto walker = internodeHandle;
    while (walker != -1) {
        m_selectedInternodeHierarchyList.push_back(walker);
        auto &internode = treeModel.m_tree->RefInternode(walker);
        walker = internode.m_parent;
    }
}

bool TreeVisualizer::RayCastSelection(TreeModel &treeModel, const GlobalTransform &globalTransform) {
    auto editorLayer = Application::GetLayer<EditorLayer>();
    bool changed = false;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{0, 0});
    if (ImGui::Begin("Scene")) {
        // Using a Child allow to fill all the space of the window.
        // It also allows customization
        if (ImGui::BeginChild("SceneCameraRenderer", ImVec2(0, 0), false, ImGuiWindowFlags_MenuBar)) {
            auto mousePosition = glm::vec2(FLT_MAX, FLT_MIN);
            if (editorLayer->SceneCameraWindowFocused()) {
#pragma region Ray selection
                InternodeHandle currentFocusingInternodeHandle = -1;
                std::mutex writeMutex;
                float minDistance = FLT_MAX;
                GlobalTransform cameraLtw;
                cameraLtw.m_value =
                        glm::translate(
                                editorLayer->m_sceneCameraPosition) *
                        glm::mat4_cast(
                                editorLayer->m_sceneCameraRotation);
                auto mp = ImGui::GetMousePos();
                auto wp = ImGui::GetWindowPos();
                mousePosition = glm::vec2(mp.x - wp.x, mp.y - wp.y - 20);
                const Ray cameraRay = editorLayer->m_sceneCamera->ScreenPointToRay(
                        cameraLtw, mousePosition);

                std::vector<std::shared_future<void>> results;
                Jobs::ParallelFor(m_sortedInternodeList.size(), [&](unsigned i) {
                    auto &internode = treeModel.m_tree->RefInternode(m_sortedInternodeList[i]);
                    auto rotation = globalTransform.GetRotation() * internode.m_globalRotation;
                    glm::vec3 position = (globalTransform.m_value *
                                          glm::translate(internode.m_globalPosition))[3];
                    const auto direction = glm::normalize(rotation * glm::vec3(0, 0, -1));
                    const glm::vec3 position2 =
                            position + internode.m_length * direction;
                    const auto center =
                            (position + position2) / 2.0f;
                    auto dir = cameraRay.m_direction;
                    auto pos = cameraRay.m_start;
                    const auto radius = internode.m_thickness;
                    const auto height = glm::distance(position2,
                                                      position);
                    if (!cameraRay.Intersect(center,
                                             height / 2.0f))
                        return;

#pragma region Line Line intersection
                    /*
 * http://geomalgorithms.com/a07-_distance.html
 */
                    glm::vec3 u = pos - (pos + dir);
                    glm::vec3 v = position - position2;
                    glm::vec3 w = (pos + dir) - position2;
                    const auto a = dot(u,
                                       u); // always >= 0
                    const auto b = dot(u, v);
                    const auto c = dot(v,
                                       v); // always >= 0
                    const auto d = dot(u, w);
                    const auto e = dot(v, w);
                    const auto dotP =
                            a * c - b * b; // always >= 0
                    float sc, tc;
                    // compute the line parameters of the two closest points
                    if (dotP <
                        0.001f) { // the lines are almost parallel
                        sc = 0.0f;
                        tc = (b > c ? d / b
                                    : e /
                                      c); // use the largest denominator
                    } else {
                        sc = (b * e - c * d) / dotP;
                        tc = (a * e - b * d) / dotP;
                    }
                    // get the difference of the two closest points
                    glm::vec3 dP = w + sc * u -
                                   tc * v; // =  L1(sc) - L2(tc)
                    if (glm::length(dP) > radius)
                        return;
#pragma endregion

                    const auto distance = glm::distance(
                            glm::vec3(cameraLtw.m_value[3]),
                            glm::vec3(center));
                    std::lock_guard<std::mutex> lock(writeMutex);
                    if (distance < minDistance) {
                        minDistance = distance;
                        currentFocusingInternodeHandle = m_sortedInternodeList[i];
                    }
                }, results);
                for (auto &i: results) i.wait();

                if (Inputs::GetMouseInternal(GLFW_MOUSE_BUTTON_LEFT,
                                             Windows::GetWindow())) {
                    if (currentFocusingInternodeHandle != -1) {
                        SetSelectedInternode(treeModel, currentFocusingInternodeHandle);
                        changed = true;
                    }
                }
#pragma endregion
            }
        }
        ImGui::EndChild();
    }
    ImGui::End();
    ImGui::PopStyleVar();

    return changed;
}


