//
// Created by lllll on 10/25/2022.
//

#include "Trees.hpp"
#include "Graphics.hpp"
#include "EditorLayer.hpp"
#include "Application.hpp"

using namespace EcoSysLab;

void Trees::OnInspect() {
    static bool displayInternodes = true;
    static bool displayBoundingBox = true;
    static bool visualization = true;
    static std::vector<int> versions;
    static std::vector<glm::vec4> randomColors;
    static std::vector<glm::mat4> matrices;
    static std::vector<glm::vec4> colors;
    static std::vector<glm::mat4> boundingBoxMatrices;
    static std::vector<glm::vec4> boundingBoxColors;
    static bool needUpdate = false;
    if (randomColors.empty()) {
        for (int i = 0; i < 10000; i++) {
            randomColors.emplace_back(glm::ballRand(1.0f), 1.0f);
        }
    }
    static glm::ivec2 gridSize = {32, 32};
    static glm::vec2 gridDistance = {10, 10};
    static float totalTime = 0.0f;
    static int internodeSize = 0;
    static int branchSize = 0;
    //static int iteration = 0;
    if (Editor::DragAndDropButton<TreeDescriptor>(m_treeDescriptor, "TreeDescriptor", true)) {
        internodeSize = 0;
        branchSize = 0;
        m_iteration = 0;
        m_treeModelGroup.m_treeModels.clear();
        totalTime = 0.0f;
        versions.clear();
    }
    if (m_treeDescriptor.Get<TreeDescriptor>()) {
        auto &parameters = m_treeDescriptor.Get<TreeDescriptor>()->m_treeStructuralGrowthParameters;
        if (m_treeModelGroup.m_treeModels.empty()) {
            ImGui::DragInt2("Grid size", &gridSize.x, 1, 0, 100);
            ImGui::DragFloat2("Grid distance", &gridDistance.x, 0.1f, 0.0f, 100.0f);
            if (ImGui::Button("Create trees")) {
                matrices.clear();
                colors.clear();
                boundingBoxColors.clear();
                boundingBoxMatrices.clear();

                internodeSize = 0;
                branchSize = 0;
                m_iteration = 0;
                m_treeModelGroup.m_treeModels.clear();
                totalTime = 0.0f;
                versions.clear();
                for (int i = 0; i < gridSize.x; i++) {
                    for (int j = 0; j < gridSize.y; j++) {
                        m_treeModelGroup.m_treeModels.emplace_back();
                        versions.emplace_back(-1);
                        auto &tree = m_treeModelGroup.m_treeModels.back();
                        tree.Initialize(parameters);
                        Transform transform;
                        transform.SetPosition(glm::vec3(i * gridDistance.x, 0.0f, j * gridDistance.y));
                        tree.m_globalTransform = transform.m_value;
                    }
                }
            }
        } else {
            static float lastUsedTime = 0.0f;
            if (ImGui::Button("Grow all")) {
                float time = Application::Time().CurrentTime();
                std::vector<std::shared_future<void>> results;
                Jobs::ParallelFor(m_treeModelGroup.m_treeModels.size(), [&](unsigned i) {
                    if (m_enableHistory) m_treeModelGroup.m_treeModels[i].m_treeStructure.Step();
                    m_treeModelGroup.m_treeModels[i].Grow({999}, parameters);
                }, results);
                for (auto &i: results) i.wait();
                lastUsedTime = Application::Time().CurrentTime() - time;
                totalTime += lastUsedTime;

                m_iteration = m_treeModelGroup.m_treeModels[0].m_treeStructure.CurrentIteration();
            }
            if (ImGui::Button("Grow all 5 iterations")) {
                float time = Application::Time().CurrentTime();
                for(int j = 0; j < 5; j++) {
                    std::vector<std::shared_future<void>> results;
                    Jobs::ParallelFor(m_treeModelGroup.m_treeModels.size(), [&](unsigned i) {
                        if (m_enableHistory) m_treeModelGroup.m_treeModels[i].m_treeStructure.Step();
                        m_treeModelGroup.m_treeModels[i].Grow({999}, parameters);
                    }, results);
                    for (auto &i: results) i.wait();
                }
                lastUsedTime = Application::Time().CurrentTime() - time;
                totalTime += lastUsedTime;
                m_iteration = m_treeModelGroup.m_treeModels[0].m_treeStructure.CurrentIteration();
            }
            ImGui::Checkbox("Enable History", &m_enableHistory);
            if (m_enableHistory && !m_treeModelGroup.m_treeModels.empty()) {
                auto &treeStructure = m_treeModelGroup.m_treeModels[0].m_treeStructure;
                if (treeStructure.CurrentIteration() > 0) {
                    if (ImGui::TreeNodeEx("History", ImGuiTreeNodeFlags_DefaultOpen)) {
                        if (ImGui::SliderInt("Iteration", &m_iteration, 0, treeStructure.CurrentIteration())) {
                            m_iteration = glm::clamp(m_iteration, 0, treeStructure.CurrentIteration());
                            needUpdate = true;
                        }
                        if (ImGui::Button("Reverse")) {
                            for (auto &treeModel: m_treeModelGroup.m_treeModels) {
                                treeModel.m_treeStructure.Reverse(m_iteration);
                            }
                            needUpdate = true;
                        }
                        ImGui::TreePop();
                    }
                }
            }

            ImGui::Text("Growth time: %.4f", lastUsedTime);
            ImGui::Text("Total time: %.4f", totalTime);
            ImGui::Checkbox("Visualization", &visualization);
            if (visualization) {
                ImGui::Checkbox("Display Internodes", &displayInternodes);
                ImGui::Checkbox("Display Bounding Box", &displayBoundingBox);
            }
            ImGui::Text("Internode count: %d", internodeSize);
            ImGui::Text("Branch count: %d", branchSize);
            ImGui::Text("Tree count: %d", m_treeModelGroup.m_treeModels.size());
            if (visualization && !m_treeModelGroup.m_treeModels.empty()) {
                auto editorLayer = Application::GetLayer<EditorLayer>();
                static Handle handle;
                needUpdate = handle == GetHandle();
                if (ImGui::Button("Update")) needUpdate = true;
                int totalInternodeSize = 0;
                int totalBranchSize = 0;
                for (int i = 0; i < m_treeModelGroup.m_treeModels.size(); i++) {
                    auto &treeModel = m_treeModelGroup.m_treeModels[i];
                    if (versions[i] != treeModel.m_treeStructure.Peek(m_iteration).GetVersion()) {
                        versions[i] = treeModel.m_treeStructure.Peek(m_iteration).GetVersion();
                        needUpdate = true;
                    }
                    totalInternodeSize += treeModel.m_treeStructure.Peek(m_iteration).RefSortedInternodeList().size();
                    totalBranchSize += treeModel.m_treeStructure.Peek(m_iteration).RefSortedBranchList().size();
                }
                internodeSize = totalInternodeSize;
                branchSize = totalBranchSize;
                if (needUpdate) {
                    boundingBoxMatrices.resize(m_treeModelGroup.m_treeModels.size());
                    boundingBoxColors.resize(m_treeModelGroup.m_treeModels.size());
                    int startIndex = 0;
                    auto entityGlobalTransform = GetScene()->GetDataComponent<GlobalTransform>(GetOwner());

                    std::map<float, int> sortedModels;
                    for (int listIndex = 0; listIndex < m_treeModelGroup.m_treeModels.size(); listIndex++) {
                        auto &treeModel = m_treeModelGroup.m_treeModels[listIndex];
                        const auto &skeleton = treeModel.m_treeStructure.Peek(m_iteration);
                        boundingBoxMatrices[listIndex] = entityGlobalTransform.m_value * treeModel.m_globalTransform *
                                                         (glm::translate(
                                                                 (skeleton.m_max + skeleton.m_min) / 2.0f) *
                                                          glm::scale((skeleton.m_max - skeleton.m_min) / 2.0f));
                        boundingBoxColors[listIndex] = randomColors[listIndex];
                        boundingBoxColors[listIndex].a = 0.1f;
                        auto distance = glm::distance(editorLayer->m_sceneCameraPosition,
                                                      glm::vec3(treeModel.m_globalTransform[3]));
                        sortedModels[distance] = listIndex;
                    }
                    for (const auto &modelPair: sortedModels) {
                        const auto &treeModel = m_treeModelGroup.m_treeModels[modelPair.second];
                        const auto &list = treeModel.m_treeStructure.Peek(m_iteration).RefSortedInternodeList();
                        if (startIndex + list.size() > 500000) break;

                        std::vector<std::shared_future<void>> results;
                        GlobalTransform globalTransform;
                        globalTransform.m_value =
                                entityGlobalTransform.m_value * treeModel.m_globalTransform;
                        matrices.resize(startIndex + list.size());
                        colors.resize(startIndex + list.size());
                        Jobs::ParallelFor(list.size(), [&](unsigned i) {
                            const auto &skeleton = treeModel.m_treeStructure.Peek(m_iteration);
                            auto &internode = skeleton.PeekInternode(list[i]);
                            auto rotation = globalTransform.GetRotation() * internode.m_info.m_globalRotation;
                            glm::vec3 translation = (globalTransform.m_value *
                                                     glm::translate(internode.m_info.m_globalPosition))[3];
                            const auto direction = glm::normalize(rotation * glm::vec3(0, 0, -1));
                            auto localEndPosition =
                                    internode.m_info.m_globalPosition + internode.m_info.m_length * direction;
                            const glm::vec3 position2 = (globalTransform.m_value * glm::translate(localEndPosition))[3];
                            rotation = glm::quatLookAt(
                                    direction, glm::vec3(direction.y, direction.z, direction.x));
                            rotation *= glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f));
                            const glm::mat4 rotationTransform = glm::mat4_cast(rotation);
                            matrices[i + startIndex] =
                                    glm::translate((translation + position2) / 2.0f) *
                                    rotationTransform *
                                    glm::scale(glm::vec3(
                                            internode.m_info.m_thickness,
                                            glm::distance(translation, position2) / 2.0f,
                                            internode.m_info.m_thickness));
                            colors[i + startIndex] = randomColors[skeleton.PeekBranch(
                                    internode.m_branchHandle).m_data.m_order];
                        }, results);
                        for (auto &i: results) i.wait();
                        startIndex += list.size();


                    }

                }
                if (!matrices.empty()) {

                    GizmoSettings m_gizmoSettings;
                    m_gizmoSettings.m_drawSettings.m_blending = true;
                    if (displayInternodes) {
                        Gizmos::DrawGizmoMeshInstancedColored(
                                DefaultResources::Primitives::Cube, editorLayer->m_sceneCamera,
                                editorLayer->m_sceneCameraPosition,
                                editorLayer->m_sceneCameraRotation,
                                *reinterpret_cast<std::vector<glm::vec4> *>(&colors),
                                *reinterpret_cast<std::vector<glm::mat4> *>(&matrices),
                                glm::mat4(1.0f), 1.0f, m_gizmoSettings);
                    }
                    if (displayBoundingBox) {
                        Gizmos::DrawGizmoMeshInstancedColored(
                                DefaultResources::Primitives::Cube, editorLayer->m_sceneCamera,
                                editorLayer->m_sceneCameraPosition,
                                editorLayer->m_sceneCameraRotation,
                                *reinterpret_cast<std::vector<glm::vec4> *>(&boundingBoxColors),
                                *reinterpret_cast<std::vector<glm::mat4> *>(&boundingBoxMatrices),
                                glm::mat4(1.0f), 1.0f, m_gizmoSettings);
                    }

                }
            }
        }
    }
    if (ImGui::Button("Clear")) {
        internodeSize = 0;
        branchSize = 0;
        m_iteration = 0;
        m_treeModelGroup.m_treeModels.clear();
        totalTime = 0.0f;
        versions.clear();
    }
}

void Trees::OnCreate() {

}

void Trees::OnDestroy() {

}
