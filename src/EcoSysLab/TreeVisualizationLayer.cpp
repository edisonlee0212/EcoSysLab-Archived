//
// Created by lllll on 11/1/2022.
//

#include "TreeVisualizationLayer.hpp"
#include "Tree.hpp"

using namespace EcoSysLab;

void TreeVisualizationLayer::OnCreate() {
    if (m_randomColors.empty()) {
        for (int i = 0; i < 10000; i++) {
            m_randomColors.emplace_back(glm::linearRand(glm::vec4(0.1f), glm::vec4(1.0f)));
        }
    }
}

void TreeVisualizationLayer::OnDestroy() {

}

void TreeVisualizationLayer::LateUpdate() {
    auto scene = GetScene();
    auto editorLayer = Application::GetLayer<EditorLayer>();
    auto selectedEntity = editorLayer->GetSelectedEntity();
    if (selectedEntity != m_selectedTree) {
        m_needFlowUpdate = true;
        m_selectedTree = selectedEntity;
    }
    const std::vector<Entity> *treeEntities =
            scene->UnsafeGetPrivateComponentOwnersList<Tree>();
    if(treeEntities && !treeEntities->empty()) {
        //Tree selection
        if (m_visualization) {
            if (m_versions.size() != treeEntities->size()) {
                m_matrices.clear();
                m_colors.clear();
                m_boundingBoxColors.clear();
                m_boundingBoxMatrices.clear();

                m_internodeSize = 0;
                m_totalTime = 0.0f;
                m_versions.clear();
                for (int i = 0; i < treeEntities->size(); i++) {
                    m_versions.emplace_back(-1);
                }
                m_needFlowUpdate = true;
            }
            int totalInternodeSize = 0;
            int totalFlowSize = 0;
            for (int i = 0; i < treeEntities->size(); i++) {
                auto treeEntity = treeEntities->at(i);
                auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
                auto &treeModel = tree->m_treeModel;
                totalInternodeSize += treeModel.m_treeStructure.Skeleton().RefSortedInternodeList().size();
                totalFlowSize += treeModel.m_treeStructure.Skeleton().RefSortedFlowList().size();
                if (selectedEntity == treeEntity) continue;
                if (m_versions[i] != treeModel.m_treeStructure.Skeleton().GetVersion()) {
                    m_versions[i] = treeModel.m_treeStructure.Skeleton().GetVersion();
                    m_needFlowUpdate = true;
                }
            }
            m_internodeSize = totalInternodeSize;
            m_flowSize = totalFlowSize;
            if (m_needFlowUpdate) {
                m_needFlowUpdate = false;
                m_boundingBoxMatrices.clear();
                m_boundingBoxColors.clear();
                int startIndex = 0;
                std::map<float, Entity> sortedModels;
                for (int listIndex = 0; listIndex < treeEntities->size(); listIndex++) {
                    auto treeEntity = treeEntities->at(listIndex);
                    if (selectedEntity == treeEntity) continue;

                    auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
                    auto &treeModel = tree->m_treeModel;
                    const auto &skeleton = treeModel.m_treeStructure.Skeleton();
                    auto entityGlobalTransform = scene->GetDataComponent<GlobalTransform>(treeEntity);
                    m_boundingBoxMatrices.emplace_back();
                    m_boundingBoxMatrices.back() = entityGlobalTransform.m_value *
                                                   (glm::translate(
                                                           (skeleton.m_max + skeleton.m_min) / 2.0f) *
                                                    glm::scale((skeleton.m_max - skeleton.m_min) / 2.0f));
                    m_boundingBoxColors.emplace_back();
                    m_boundingBoxColors.back() = m_randomColors[listIndex];
                    m_boundingBoxColors.back().a = 0.05f;
                    auto distance = glm::distance(editorLayer->m_sceneCameraPosition,
                                                  glm::vec3(entityGlobalTransform.GetPosition()));
                    sortedModels[distance] = treeEntity;
                }
                for (const auto &modelPair: sortedModels) {
                    auto tree = scene->GetOrSetPrivateComponent<Tree>(modelPair.second).lock();
                    auto &treeModel = tree->m_treeModel;
                    const auto &list = treeModel.m_treeStructure.Skeleton().RefSortedFlowList();
                    if (startIndex + list.size() > 10000000) break;
                    auto entityGlobalTransform = scene->GetDataComponent<GlobalTransform>(modelPair.second);
                    std::vector<std::shared_future<void>> results;
                    m_matrices.resize(startIndex + list.size());
                    m_colors.resize(startIndex + list.size());
                    Jobs::ParallelFor(list.size(), [&](unsigned i) {
                        const auto &skeleton = treeModel.m_treeStructure.Skeleton();
                        auto &flow = skeleton.PeekFlow(list[i]);
                        glm::vec3 translation = (entityGlobalTransform.m_value *
                                                 glm::translate(flow.m_info.m_globalStartPosition))[3];
                        const auto direction = glm::normalize(
                                flow.m_info.m_globalEndPosition - flow.m_info.m_globalStartPosition);
                        const glm::vec3 position2 = (entityGlobalTransform.m_value *
                                                     glm::translate(flow.m_info.m_globalEndPosition))[3];
                        auto rotation = glm::quatLookAt(
                                direction, glm::vec3(direction.y, direction.z, direction.x));
                        rotation *= glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f));
                        const glm::mat4 rotationTransform = glm::mat4_cast(rotation);
                        m_matrices[i + startIndex] =
                                glm::translate((translation + position2) / 2.0f) *
                                rotationTransform *
                                glm::scale(glm::vec3(
                                        flow.m_info.m_startThickness,
                                        glm::distance(translation, position2) / 2.0f,
                                        flow.m_info.m_startThickness));
                        m_colors[i + startIndex] = m_randomColors[flow.m_data.m_order];
                        m_colors[i + startIndex].a = m_selectedTree.GetIndex() != 0 ? 0.05f : 1.0f;
                    }, results);
                    for (auto &i: results) i.wait();
                    startIndex += list.size();
                }
            }
            GizmoSettings gizmoSettings;
            gizmoSettings.m_drawSettings.m_blending = true;
            if (m_displayTrees && !m_matrices.empty()) {
                Gizmos::DrawGizmoMeshInstancedColored(
                        DefaultResources::Primitives::Cube, editorLayer->m_sceneCamera,
                        editorLayer->m_sceneCameraPosition,
                        editorLayer->m_sceneCameraRotation,
                        *reinterpret_cast<std::vector<glm::vec4> *>(&m_colors),
                        *reinterpret_cast<std::vector<glm::mat4> *>(&m_matrices),
                        glm::mat4(1.0f), 1.0f, gizmoSettings);
            }
        }
    }
}

void TreeVisualizationLayer::OnInspect() {
    if (ImGui::Begin("Tree Visualization Layer")) {
        auto scene = GetScene();
        const std::vector<Entity> *treeEntities =
                scene->UnsafeGetPrivateComponentOwnersList<Tree>();
        if (treeEntities && !treeEntities->empty()) {
            if (ImGui::Button("Grow all")) {
                float time = Application::Time().CurrentTime();
                std::vector<std::shared_future<void>> results;
                Jobs::ParallelFor(treeEntities->size(), [&](unsigned i) {
                    auto treeEntity = treeEntities->at(i);
                    auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
                    auto treeDescriptor = tree->m_treeDescriptor.Get<TreeDescriptor>();
                    if (!treeDescriptor) return;
                    auto &treeModel = tree->m_treeModel;
                    if (m_enableHistory) treeModel.m_treeStructure.Step();
                    treeModel.Grow({999}, treeDescriptor->m_treeStructuralGrowthParameters);
                }, results);
                for (auto &i: results) i.wait();
                m_lastUsedTime = Application::Time().CurrentTime() - time;
                m_totalTime += m_lastUsedTime;
            }
            if (ImGui::Button("Grow all 5 iterations")) {
                float time = Application::Time().CurrentTime();
                for (int j = 0; j < 5; j++) {
                    std::vector<std::shared_future<void>> results;
                    Jobs::ParallelFor(treeEntities->size(), [&](unsigned i) {
                        auto treeEntity = treeEntities->at(i);
                        auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
                        auto treeDescriptor = tree->m_treeDescriptor.Get<TreeDescriptor>();
                        if (!treeDescriptor) return;
                        auto &treeModel = tree->m_treeModel;
                        if (m_enableHistory) treeModel.m_treeStructure.Step();
                        treeModel.Grow({999}, treeDescriptor->m_treeStructuralGrowthParameters);
                    }, results);
                    for (auto &i: results) i.wait();
                }
                m_lastUsedTime = Application::Time().CurrentTime() - time;
                m_totalTime += m_lastUsedTime;
            }
            ImGui::Text("Growth time: %.4f", m_lastUsedTime);
            ImGui::Text("Total time: %.4f", m_totalTime);
            ImGui::Checkbox("Visualization", &m_visualization);
            if (m_visualization) {
                ImGui::Checkbox("Display Flows", &m_displayTrees);
                ImGui::Checkbox("Display Bounding Box", &m_displayBoundingBox);
            }
            ImGui::Text("Internode size: %d", m_internodeSize);
            ImGui::Text("Flow size: %d", m_flowSize);
            if (treeEntities) ImGui::Text("Tree count: %d", treeEntities->size());

        }
    }
    ImGui::End();
}
