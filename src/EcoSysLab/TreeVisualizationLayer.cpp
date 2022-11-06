//
// Created by lllll on 11/1/2022.
//

#include "TreeVisualizationLayer.hpp"
#include "Tree.hpp"

using namespace EcoSysLab;

void TreeVisualizationLayer::OnCreate() {
    if (m_randomColors.empty()) {
        for (int i = 0; i < 10000; i++) {
            m_randomColors.emplace_back(glm::ballRand(1.0f), 1.0f);
        }
    }
}

void TreeVisualizationLayer::OnDestroy() {

}

void TreeVisualizationLayer::LateUpdate() {
    auto scene = GetScene();
    const std::vector<Entity> *treeEntities =
            scene->UnsafeGetPrivateComponentOwnersList<Tree>();
    if (m_visualization && treeEntities && !treeEntities->empty()) {
        auto editorLayer = Application::GetLayer<EditorLayer>();
        if (m_versions.size() != treeEntities->size()) {
            m_matrices.clear();
            m_colors.clear();
            m_boundingBoxColors.clear();
            m_boundingBoxMatrices.clear();

            m_internodeSize = 0;
            m_iteration = 0;
            m_totalTime = 0.0f;
            m_versions.clear();
            for (int i = 0; i < treeEntities->size(); i++) {
                m_versions.emplace_back(-1);
            }
        }

        int totalInternodeSize = 0;
        int totalBranchSize = 0;
        for (int i = 0; i < treeEntities->size(); i++) {
            auto treeEntity = treeEntities->at(i);
            auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
            auto &treeModel = tree->m_treeModel;
            if (m_versions[i] != treeModel.m_treeStructure.Peek(m_iteration).GetVersion()) {
                m_versions[i] = treeModel.m_treeStructure.Peek(m_iteration).GetVersion();
                m_needUpdate = true;
            }
            totalInternodeSize += treeModel.m_treeStructure.Peek(m_iteration).RefSortedInternodeList().size();
            totalBranchSize += treeModel.m_treeStructure.Peek(m_iteration).RefSortedFlowList().size();
        }
        m_internodeSize = totalInternodeSize;
        if (m_needUpdate) {
            m_boundingBoxMatrices.resize(treeEntities->size());
            m_boundingBoxColors.resize(treeEntities->size());
            int startIndex = 0;


            std::map<float, Entity> sortedModels;
            for (int listIndex = 0; listIndex < treeEntities->size(); listIndex++) {
                auto treeEntity = treeEntities->at(listIndex);
                auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
                auto &treeModel = tree->m_treeModel;
                const auto &skeleton = treeModel.m_treeStructure.Peek(m_iteration);
                auto entityGlobalTransform = scene->GetDataComponent<GlobalTransform>(treeEntity);
                m_boundingBoxMatrices[listIndex] = entityGlobalTransform.m_value * treeModel.m_globalTransform *
                                                   (glm::translate(
                                                         (skeleton.m_max + skeleton.m_min) / 2.0f) *
                                                  glm::scale((skeleton.m_max - skeleton.m_min) / 2.0f));
                m_boundingBoxColors[listIndex] = m_randomColors[listIndex];
                m_boundingBoxColors[listIndex].a = 0.1f;
                auto distance = glm::distance(editorLayer->m_sceneCameraPosition,
                                              glm::vec3(treeModel.m_globalTransform[3]));
                sortedModels[distance] = treeEntity;
            }
            for (const auto &modelPair: sortedModels) {
                auto tree = scene->GetOrSetPrivateComponent<Tree>(modelPair.second).lock();
                const auto &treeModel = tree->m_treeModel;
                const auto &list = treeModel.m_treeStructure.Peek(m_iteration).RefSortedInternodeList();
                if (startIndex + list.size() > 500000) break;
                auto entityGlobalTransform = scene->GetDataComponent<GlobalTransform>(modelPair.second);
                std::vector<std::shared_future<void>> results;
                GlobalTransform globalTransform;
                globalTransform.m_value =
                        entityGlobalTransform.m_value * treeModel.m_globalTransform;
                m_matrices.resize(startIndex + list.size());
                m_colors.resize(startIndex + list.size());
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
                    m_matrices[i + startIndex] =
                            glm::translate((translation + position2) / 2.0f) *
                            rotationTransform *
                            glm::scale(glm::vec3(
                                    internode.m_info.m_thickness,
                                    glm::distance(translation, position2) / 2.0f,
                                    internode.m_info.m_thickness));
                    m_colors[i + startIndex] = m_randomColors[skeleton.PeekFlow(
                            internode.GetFlowHandle()).m_data.m_order];
                }, results);
                for (auto &i: results) i.wait();
                startIndex += list.size();
            }

        }
        if (!m_matrices.empty()) {

            GizmoSettings m_gizmoSettings;
            m_gizmoSettings.m_drawSettings.m_blending = true;
            if (m_displayInternodes) {
                Gizmos::DrawGizmoMeshInstancedColored(
                        DefaultResources::Primitives::Cube, editorLayer->m_sceneCamera,
                        editorLayer->m_sceneCameraPosition,
                        editorLayer->m_sceneCameraRotation,
                        *reinterpret_cast<std::vector<glm::vec4> *>(&m_colors),
                        *reinterpret_cast<std::vector<glm::mat4> *>(&m_matrices),
                        glm::mat4(1.0f), 1.0f, m_gizmoSettings);
            }
            if (m_displayBoundingBox) {
                Gizmos::DrawGizmoMeshInstancedColored(
                        DefaultResources::Primitives::Cube, editorLayer->m_sceneCamera,
                        editorLayer->m_sceneCameraPosition,
                        editorLayer->m_sceneCameraRotation,
                        *reinterpret_cast<std::vector<glm::vec4> *>(&m_boundingBoxColors),
                        *reinterpret_cast<std::vector<glm::mat4> *>(&m_boundingBoxMatrices),
                        glm::mat4(1.0f), 1.0f, m_gizmoSettings);
            }

        }
    }
}

void TreeVisualizationLayer::OnInspect() {
    if(ImGui::Begin("TreeVisualizationLayer")) {

        m_needUpdate = false;
        auto scene = GetScene();
        const std::vector<Entity> *treeEntities =
                scene->UnsafeGetPrivateComponentOwnersList<Tree>();
        if(treeEntities && !treeEntities->empty()) {
            if (ImGui::Button("Grow all")) {
                float time = Application::Time().CurrentTime();
                std::vector<std::shared_future<void>> results;
                Jobs::ParallelFor(treeEntities->size(), [&](unsigned i) {
                    auto treeEntity = treeEntities->at(i);
                    auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
                    auto treeDescriptor = tree->m_treeDescriptor.Get<TreeStructuralGrowthParameters>();
                    if(!treeDescriptor) return;
                    auto& treeModel = tree->m_treeModel;
                    if (m_enableHistory) treeModel.m_treeStructure.Step();
                    treeModel.Grow({999}, *treeDescriptor);
                }, results);
                for (auto &i: results) i.wait();
                m_lastUsedTime = Application::Time().CurrentTime() - time;
                m_totalTime += m_lastUsedTime;

                m_iteration++;
            }
            if (ImGui::Button("Grow all 5 iterations")) {
                float time = Application::Time().CurrentTime();
                for (int j = 0; j < 5; j++) {
                    std::vector<std::shared_future<void>> results;
                    Jobs::ParallelFor(treeEntities->size(), [&](unsigned i) {
                        auto treeEntity = treeEntities->at(i);
                        auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
                        auto treeDescriptor = tree->m_treeDescriptor.Get<TreeStructuralGrowthParameters>();
                        if(!treeDescriptor) return;
                        auto& treeModel = tree->m_treeModel;
                        if (m_enableHistory) treeModel.m_treeStructure.Step();
                        treeModel.Grow({999}, *treeDescriptor);
                    }, results);
                    for (auto &i: results) i.wait();
                }
                m_lastUsedTime = Application::Time().CurrentTime() - time;
                m_totalTime += m_lastUsedTime;

                m_iteration++;
            }
            ImGui::Checkbox("Enable History", &m_enableHistory);
            /*
            if (m_enableHistory && !m_treeModelGroup.m_treeModels.empty()) {
                auto &treeStructure = m_treeModelGroup.m_treeModels[0].m_treeStructure;
                if (treeStructure.CurrentIteration() > 0) {
                    if (ImGui::TreeNodeEx("History", ImGuiTreeNodeFlags_DefaultOpen)) {
                        if (ImGui::SliderInt("Iteration", &m_iteration, 0, treeStructure.CurrentIteration())) {
                            m_iteration = glm::clamp(m_iteration, 0, treeStructure.CurrentIteration());
                            m_needUpdate = true;
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
            */
            ImGui::Text("Growth time: %.4f", m_lastUsedTime);
            ImGui::Text("Total time: %.4f", m_totalTime);
            ImGui::Checkbox("Visualization", &m_visualization);
            if (m_visualization) {
                ImGui::Checkbox("Display Internodes", &m_displayInternodes);
                ImGui::Checkbox("Display Bounding Box", &m_displayBoundingBox);
            }
            ImGui::Text("Internode count: %d", m_internodeSize);
            if (treeEntities) ImGui::Text("Tree count: %d", treeEntities->size());
            if (ImGui::Button("Update")) m_needUpdate = true;
        }
    }
    ImGui::End();
}
