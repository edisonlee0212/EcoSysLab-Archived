//
// Created by lllll on 11/1/2022.
//

#include "TreeVisualizationLayer.hpp"
#include "Tree.hpp"

using namespace EcoSysLab;

void TreeVisualizationLayer::OnCreate() {
    if (m_randomColors.empty()) {
        for (int i = 0; i < 10000; i++) {
            m_randomColors.emplace_back(glm::linearRand(glm::vec3(0.0f), glm::vec3(1.0f)));
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
        if(scene->IsEntityValid(m_selectedTree)) m_treeVisualizer.Reset(scene->GetOrSetPrivateComponent<Tree>(m_selectedTree).lock()->m_treeModel.m_treeStructure);
    }
    const std::vector<Entity> *treeEntities =
            scene->UnsafeGetPrivateComponentOwnersList<Tree>();
    if (treeEntities && !treeEntities->empty()) {
        //Tree selection
        if (!m_lockTreeSelection && editorLayer->GetLockEntitySelection() && Inputs::GetMouseInternal(GLFW_MOUSE_BUTTON_LEFT,
                                                                               Windows::GetWindow())) {
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{0, 0});
            if (ImGui::Begin("Scene")) {
                // Using a Child allow to fill all the space of the window.
                // It also allows customization
                if (ImGui::BeginChild("SceneCameraRenderer", ImVec2(0, 0), false, ImGuiWindowFlags_MenuBar)) {
                    auto mousePosition = glm::vec2(FLT_MAX, FLT_MIN);
                    if (editorLayer->SceneCameraWindowFocused()) {
#pragma region Ray selection
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
                        bool detected = false;
                        Entity currentFocusingTree;
                        Jobs::ParallelFor(treeEntities->size(), [&](unsigned i) {
                            const auto treeEntity = treeEntities->at(i);
                            auto globalTransform = scene->GetDataComponent<GlobalTransform>(treeEntity);
                            auto skeleton = scene->GetOrSetPrivateComponent<Tree>(
                                    treeEntity).lock()->m_treeModel.m_treeStructure.RefSkeleton();
                            Bound bound;
                            bound.m_min = skeleton.m_min;
                            bound.m_max = skeleton.m_max;
                            if (!cameraRay.Intersect(globalTransform.m_value, bound)) return;
                            auto distance = glm::distance(globalTransform.GetPosition(),
                                                          glm::vec3(cameraLtw.m_value[3]));
                            std::lock_guard<std::mutex> lock(writeMutex);
                            if (distance < minDistance) {
                                minDistance = distance;
                                currentFocusingTree = treeEntity;
                                detected = true;
                            }
                        }, results);
                        for (auto &i: results) i.wait();
                        if (detected && currentFocusingTree != m_selectedTree && scene->IsEntityValid(currentFocusingTree)) {
                            editorLayer->SetSelectedEntity(currentFocusingTree);
                            m_selectedTree = currentFocusingTree;
                            if(scene->IsEntityValid(m_selectedTree)) m_treeVisualizer.Reset(scene->GetOrSetPrivateComponent<Tree>(m_selectedTree).lock()->m_treeModel.m_treeStructure);
                            m_needFlowUpdate = true;
                        }
#pragma endregion
                    }
                }
                ImGui::EndChild();
            }
            ImGui::End();
            ImGui::PopStyleVar();
        }

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
                totalInternodeSize += treeModel.m_treeStructure.RefSkeleton().RefSortedNodeList().size();
                totalFlowSize += treeModel.m_treeStructure.RefSkeleton().RefSortedFlowList().size();
                if (m_selectedTree == treeEntity) continue;
                if (m_versions[i] != treeModel.m_treeStructure.RefSkeleton().GetVersion()) {
                    m_versions[i] = treeModel.m_treeStructure.RefSkeleton().GetVersion();
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
                    if (m_selectedTree == treeEntity) continue;

                    auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
                    auto &treeModel = tree->m_treeModel;
                    const auto &skeleton = treeModel.m_treeStructure.RefSkeleton();
                    auto entityGlobalTransform = scene->GetDataComponent<GlobalTransform>(treeEntity);
                    m_boundingBoxMatrices.emplace_back();
                    m_boundingBoxMatrices.back() = entityGlobalTransform.m_value *
                                                   (glm::translate(
                                                           (skeleton.m_max + skeleton.m_min) / 2.0f) *
                                                    glm::scale((skeleton.m_max - skeleton.m_min) / 2.0f));
                    m_boundingBoxColors.emplace_back();
                    m_boundingBoxColors.back() = glm::vec4(m_randomColors[listIndex], 0.05f);
                    auto distance = glm::distance(editorLayer->m_sceneCameraPosition,
                                                  glm::vec3(entityGlobalTransform.GetPosition()));
                    sortedModels[distance] = treeEntity;
                }
                m_matrices.clear();
                m_colors.clear();
                for (const auto &modelPair: sortedModels) {
                    auto tree = scene->GetOrSetPrivateComponent<Tree>(modelPair.second).lock();
                    auto &treeModel = tree->m_treeModel;
                    const auto &list = treeModel.m_treeStructure.RefSkeleton().RefSortedFlowList();
                    if (startIndex + list.size() > 50000000) break;
                    auto entityGlobalTransform = scene->GetDataComponent<GlobalTransform>(modelPair.second);
                    std::vector<std::shared_future<void>> results;
                    m_matrices.resize(startIndex + list.size());
                    m_colors.resize(startIndex + list.size());
                    Jobs::ParallelFor(list.size(), [&](unsigned i) {
                        const auto &skeleton = treeModel.m_treeStructure.RefSkeleton();
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
                        m_colors[i + startIndex] = glm::vec4(m_randomColors[flow.m_data.m_order], m_selectedTree.GetIndex() != 0 ? 0.05f : 1.0f);
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
            if (m_displayBoundingBox && !m_boundingBoxMatrices.empty()) {
                Gizmos::DrawGizmoMeshInstancedColored(
                        DefaultResources::Primitives::Cube, editorLayer->m_sceneCamera,
                        editorLayer->m_sceneCameraPosition,
                        editorLayer->m_sceneCameraRotation,
                        *reinterpret_cast<std::vector<glm::vec4> *>(&m_boundingBoxColors),
                        *reinterpret_cast<std::vector<glm::mat4> *>(&m_boundingBoxMatrices),
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
        ImGui::Checkbox("Lock tree selection", &m_lockTreeSelection);
        ImGui::Checkbox("Visualization", &m_visualization);
        if (m_visualization) {
            ImGui::Checkbox("Display Flows", &m_displayTrees);
            ImGui::Checkbox("Display Bounding Box", &m_displayBoundingBox);
        }
        if (treeEntities && !treeEntities->empty()) {
            ImGui::Checkbox("Auto grow", &m_autoGrow);
            if(!m_autoGrow) {
                bool changed = false;
                if (ImGui::Button("Grow all")) {
                    GrowAllTrees();
                    changed = true;
                }
                static int iterations = 5;
                ImGui::DragInt("Iterations", &iterations, 1, 1, 100);
                if (ImGui::Button(("Grow all " + std::to_string(iterations) + " iterations").c_str())) {
                    for (int i = 0; i < iterations; i++) GrowAllTrees();
                    changed = true;
                }
                if(changed){
                    if(scene->IsEntityValid(m_selectedTree)) {
                        m_treeVisualizer.m_iteration = scene->GetOrSetPrivateComponent<Tree>(m_selectedTree).lock()->m_treeModel.m_treeStructure.CurrentIteration();
                        m_treeVisualizer.m_needUpdate = true;
                    }
                }
            }
            ImGui::Text("Growth time: %.4f", m_lastUsedTime);
            ImGui::Text("Total time: %.4f", m_totalTime);
            ImGui::Text("Tree count: %d", treeEntities->size());
            ImGui::Text("Total Internode size: %d", m_internodeSize);
            ImGui::Text("Total Flow size: %d", m_flowSize);

            if(ImGui::TreeNode("Mesh generation")) {
                m_meshGeneratorSettings.OnInspect();
                if (ImGui::Button("Generate Meshes")) {
                    GenerateMeshes(m_meshGeneratorSettings);
                }
                ImGui::TreePop();
            }
        }

        if(m_visualization && scene->IsEntityValid(m_selectedTree)) {
            m_treeVisualizer.OnInspect(
                    scene->GetOrSetPrivateComponent<Tree>(m_selectedTree).lock()->m_treeModel.m_treeStructure, scene->GetDataComponent<GlobalTransform>(m_selectedTree));
        }
    }
    ImGui::End();
}

void TreeVisualizationLayer::FixedUpdate() {
    if(m_autoGrow){
        GrowAllTrees();
    }
}

void TreeVisualizationLayer::GrowAllTrees() {
    auto scene = GetScene();
    const std::vector<Entity> *treeEntities =
            scene->UnsafeGetPrivateComponentOwnersList<Tree>();
    if (treeEntities && !treeEntities->empty()) {
        float time = Application::Time().CurrentTime();
        std::vector<std::shared_future<void>> results;
        Jobs::ParallelFor(treeEntities->size(), [&](unsigned i) {
            auto treeEntity = treeEntities->at(i);
            auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
            tree->TryGrow();
        }, results);
        for (auto &i: results) i.wait();
        m_lastUsedTime = Application::Time().CurrentTime() - time;
        m_totalTime += m_lastUsedTime;

        if(scene->IsEntityValid(m_selectedTree)){
            m_treeVisualizer.Reset(scene->GetOrSetPrivateComponent<Tree>(m_selectedTree).lock()->m_treeModel.m_treeStructure);
        }
    }
}

void TreeVisualizationLayer::GenerateMeshes(const MeshGeneratorSettings& meshGeneratorSettings) {
    auto scene = GetScene();
    const std::vector<Entity> *treeEntities =
            scene->UnsafeGetPrivateComponentOwnersList<Tree>();
    if (treeEntities && !treeEntities->empty()) {
        auto copiedEntities = *treeEntities;
        for(auto treeEntity : copiedEntities){
            auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
            tree->GenerateMesh(meshGeneratorSettings);
        }
    }
}
