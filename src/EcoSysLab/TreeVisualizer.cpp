//
// Created by lllll on 11/20/2022.
//

#include "TreeVisualizer.hpp"

using namespace EcoSysLab;

bool TreeVisualizer::DrawInternodeInspectionGui(
        PlantStructure<SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeStructure,
        NodeHandle internodeHandle,
        bool &deleted,
        const unsigned int &hierarchyLevel) {
    auto &treeSkeleton = treeStructure.RefSkeleton();
    const int index = m_selectedInternodeHierarchyList.size() - hierarchyLevel - 1;
    if (!m_selectedInternodeHierarchyList.empty() && index >= 0 &&
        index < m_selectedInternodeHierarchyList.size() &&
        m_selectedInternodeHierarchyList[index] == internodeHandle) {
        ImGui::SetNextItemOpen(true);
    }
    const bool opened = ImGui::TreeNodeEx(("Handle: " + std::to_string(internodeHandle)).c_str(),
                                          ImGuiTreeNodeFlags_NoTreePushOnOpen | ImGuiTreeNodeFlags_OpenOnArrow |
                                          ImGuiTreeNodeFlags_NoAutoOpenOnLog |
                                          (m_selectedInternodeHandle == internodeHandle ? ImGuiTreeNodeFlags_Framed
                                                                                        : ImGuiTreeNodeFlags_FramePadding));
    if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
        SetSelectedInternode(treeSkeleton, internodeHandle);
    }

    if (ImGui::BeginPopupContextItem(std::to_string(internodeHandle).c_str())) {
        ImGui::Text(("Handle: " + std::to_string(internodeHandle)).c_str());
        if (ImGui::Button("Delete")) {
            deleted = true;
        }
        ImGui::EndPopup();
    }
    bool modified = deleted;
    if (opened && !deleted) {
        ImGui::TreePush();
        const auto &internodeChildren = treeSkeleton.RefNode(internodeHandle).RefChildHandles();
        for (const auto &child: internodeChildren) {
            bool childDeleted = false;
            DrawInternodeInspectionGui(treeStructure, child, childDeleted, hierarchyLevel + 1);
            if (childDeleted) {
                treeStructure.Step();
                treeSkeleton.RecycleNode(child);
                treeSkeleton.SortLists();
                m_iteration = treeStructure.CurrentIteration();
                modified = true;
                break;
            }
        }
        ImGui::TreePop();
    }
    return modified;
}

void
TreeVisualizer::PeekInternodeInspectionGui(
        const Skeleton<SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeSkeleton,
        NodeHandle internodeHandle,
        const unsigned int &hierarchyLevel) {
    const int index = m_selectedInternodeHierarchyList.size() - hierarchyLevel - 1;
    if (!m_selectedInternodeHierarchyList.empty() && index >= 0 &&
        index < m_selectedInternodeHierarchyList.size() &&
        m_selectedInternodeHierarchyList[index] == internodeHandle) {
        ImGui::SetNextItemOpen(true);
    }
    const bool opened = ImGui::TreeNodeEx(("Handle: " + std::to_string(internodeHandle)).c_str(),
                                          ImGuiTreeNodeFlags_NoTreePushOnOpen | ImGuiTreeNodeFlags_OpenOnArrow |
                                          ImGuiTreeNodeFlags_NoAutoOpenOnLog |
                                          (m_selectedInternodeHandle == internodeHandle ? ImGuiTreeNodeFlags_Framed
                                                                                        : ImGuiTreeNodeFlags_FramePadding));
    if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
        SetSelectedInternode(treeSkeleton, internodeHandle);
    }
    if (opened) {
        ImGui::TreePush();
        const auto &internode = treeSkeleton.PeekNode(internodeHandle);
        const auto &internodeChildren = internode.RefChildHandles();
        for (const auto &child: internodeChildren) {
            PeekInternodeInspectionGui(treeSkeleton, child, hierarchyLevel + 1);
        }
        ImGui::TreePop();
    }
}

bool
TreeVisualizer::OnInspect(
        PlantStructure<SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeStructure,
        const GlobalTransform &globalTransform) {
    bool updated = false;
    if (ImGui::TreeNodeEx("Current selected tree", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (treeStructure.CurrentIteration() > 0) {
            if (ImGui::TreeNodeEx("History", ImGuiTreeNodeFlags_DefaultOpen)) {
                if (ImGui::SliderInt("Iteration", &m_iteration, 0, treeStructure.CurrentIteration())) {
                    m_iteration = glm::clamp(m_iteration, 0, treeStructure.CurrentIteration());
                    m_selectedInternodeHandle = -1;
                    m_selectedInternodeHierarchyList.clear();
                    m_needUpdate = true;
                }
                if (m_iteration != treeStructure.CurrentIteration() && ImGui::Button("Reverse")) {
                    treeStructure.Reverse(m_iteration);
                    m_needUpdate = true;
                }
                ImGui::TreePop();
            }
        }

        if (ImGui::TreeNodeEx("Settings")) {
            ImGui::Checkbox("Visualization", &m_visualization);
            ImGui::Checkbox("Tree Hierarchy", &m_treeHierarchyGui);
            ImGui::TreePop();
        }

        if (m_treeHierarchyGui) {
            if (ImGui::TreeNodeEx("Tree Hierarchy")) {

                bool deleted = false;
                if (m_iteration == treeStructure.CurrentIteration()) {
                    if (DrawInternodeInspectionGui(treeStructure, 0, deleted, 0)) {
                        m_needUpdate = true;
                        updated = true;
                    }
                } else PeekInternodeInspectionGui(treeStructure.Peek(m_iteration), 0, 0);
                m_selectedInternodeHierarchyList.clear();

                ImGui::TreePop();
            }
            if (m_selectedInternodeHandle >= 0) {
                if (m_iteration == treeStructure.CurrentIteration()) {
                    InspectInternode(treeStructure.RefSkeleton(), m_selectedInternodeHandle);
                } else {
                    PeekInternode(treeStructure.Peek(m_iteration), m_selectedInternodeHandle);
                }
            }
        }

        ImGui::TreePop();
    }
    const auto &treeSkeleton = treeStructure.Peek(m_iteration);

    if (m_visualization) {
        auto editorLayer = Application::GetLayer<EditorLayer>();
        const auto &sortedBranchList = treeSkeleton.RefSortedFlowList();
        const auto &sortedInternodeList = treeSkeleton.RefSortedNodeList();
        ImGui::Text("Internode count: %d", sortedInternodeList.size());
        ImGui::Text("Flow count: %d", sortedBranchList.size());
        static bool enableStroke = false;
        if (ImGui::Checkbox("Enable stroke", &enableStroke)) {
            if (enableStroke) {
                m_mode = PruningMode::Stroke;
            } else {
                m_mode = PruningMode::None;
            }
            m_storedMousePositions.clear();
        }
        switch (m_mode) {
            case PruningMode::None: {
                if (Inputs::GetMouseInternal(GLFW_MOUSE_BUTTON_LEFT, Windows::GetWindow())) {
                    if (RayCastSelection(treeSkeleton, globalTransform)) {
                        m_needUpdate = true;
                        updated = true;
                    }
                }
                if (m_iteration == treeStructure.CurrentIteration() && m_selectedInternodeHandle > 0 &&
                    Inputs::GetKeyInternal(GLFW_KEY_DELETE,
                                           Windows::GetWindow())) {
                    treeStructure.Step();
                    auto &skeleton = treeStructure.RefSkeleton();
                    auto &pruningInternode = skeleton.RefNode(m_selectedInternodeHandle);
                    auto childHandles = pruningInternode.RefChildHandles();
                    for (const auto &childHandle: childHandles) {
                        skeleton.RecycleNode(childHandle);
                    }
                    pruningInternode.m_info.m_length *= m_selectedInternodeLengthFactor;
                    m_selectedInternodeLengthFactor = 1.0f;
                    for (auto &bud: pruningInternode.m_data.m_buds) {
                        bud.m_status = BudStatus::Died;
                    }
                    skeleton.SortLists();
                    m_iteration = treeStructure.CurrentIteration();
                    m_needUpdate = true;
                    updated = true;
                }
            }
                break;
            case PruningMode::Stroke: {
                if (m_iteration == treeStructure.CurrentIteration()) {
                    if (Inputs::GetMouseInternal(GLFW_MOUSE_BUTTON_LEFT, Windows::GetWindow())) {
                        glm::vec2 mousePosition = editorLayer->GetMouseScreenPosition();
                        const float halfX = static_cast<float>(editorLayer->m_sceneCamera->GetResolution().x) / 2.0f;
                        const float halfY = static_cast<float>(editorLayer->m_sceneCamera->GetResolution().y) / 2.0f;
                        mousePosition = {-1.0f * (mousePosition.x - halfX) / halfX,
                                         -1.0f * (mousePosition.y - halfY) / halfY};
                        if (mousePosition.x > -1.0f && mousePosition.x < 1.0f && mousePosition.y > -1.0f &&
                            mousePosition.y < 1.0f) {
                            m_storedMousePositions.emplace_back(mousePosition);
                        }
                    } else {
                        //Once released, check if empty.
                        if (!m_storedMousePositions.empty()) {
                            treeStructure.Step();
                            auto &skeleton = treeStructure.RefSkeleton();
                            bool changed = ScreenCurvePruning(skeleton, globalTransform);
                            if (changed) {
                                skeleton.SortLists();
                                m_iteration = treeStructure.CurrentIteration();
                                m_needUpdate = true;
                                updated = true;
                            } else {
                                treeStructure.Pop();
                            }
                            m_storedMousePositions.clear();
                        }
                    }
                }
            }
                break;
        }


        if (m_needUpdate) {
            SyncMatrices(treeSkeleton);
            m_needUpdate = false;
        }
        if (!m_matrices.empty()) {
            GizmoSettings m_gizmoSettings;
            m_gizmoSettings.m_drawSettings.m_blending = true;
            if (m_selectedInternodeHandle == -1) {
                m_matrices[0] = glm::translate(glm::vec3(1.0f)) * glm::scale(glm::vec3(0.0f));
                m_colors[0] = glm::vec4(0.0f);
            }
            Gizmos::DrawGizmoMeshInstancedColored(
                    DefaultResources::Primitives::Cylinder, editorLayer->m_sceneCamera,
                    editorLayer->m_sceneCameraPosition,
                    editorLayer->m_sceneCameraRotation,
                    *reinterpret_cast<std::vector<glm::vec4> *>(&m_colors),
                    *reinterpret_cast<std::vector<glm::mat4> *>(&m_matrices),
                    globalTransform.m_value, 1.0f, m_gizmoSettings);
        }
    }

    return updated;
}

bool
TreeVisualizer::InspectInternode(
        Skeleton<SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeSkeleton,
        NodeHandle internodeHandle) {
    bool changed = false;
    if (ImGui::Begin("Internode Inspector")) {
        const auto &internode = treeSkeleton.PeekNode(internodeHandle);
        if (ImGui::TreeNode("Internode info")) {
            ImGui::Text("Thickness: %.3f", internode.m_info.m_thickness);
            ImGui::Text("Length: %.3f", internode.m_info.m_length);
            ImGui::InputFloat3("Position", (float *) &internode.m_info.m_globalPosition.x, "%.3f",
                               ImGuiInputTextFlags_ReadOnly);
            auto globalRotationAngle = glm::eulerAngles(internode.m_info.m_globalRotation);
            ImGui::InputFloat3("Global rotation", (float *) &globalRotationAngle.x, "%.3f",
                               ImGuiInputTextFlags_ReadOnly);
            auto localRotationAngle = glm::eulerAngles(internode.m_info.m_localRotation);
            ImGui::InputFloat3("Local rotation", (float *) &localRotationAngle.x, "%.3f",
                               ImGuiInputTextFlags_ReadOnly);
            auto &internodeData = internode.m_data;
            ImGui::InputInt("Age", (int *) &internodeData.m_age, 1, 100, ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Distance to end", (float *) &internodeData.m_maxDistanceToAnyBranchEnd, 1, 100,
                              "%.3f",
                              ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Level", (float *) &internodeData.m_level, 1, 100, "%.3f",
                              ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Child biomass", (float *) &internodeData.m_childTotalBiomass, 1, 100, "%.3f",
                              ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Root distance", (float *) &internodeData.m_rootDistance, 1, 100, "%.3f",
                              ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat3("Light dir", (float *) &internodeData.m_lightDirection.x, "%.3f",
                               ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Light intensity", (float *) &internodeData.m_lightIntensity, 1, 100, "%.3f",
                              ImGuiInputTextFlags_ReadOnly);

            if (ImGui::DragFloat("Inhibitor", (float *) &internodeData.m_inhibitor)) {
                changed = true;
            }
            if (ImGui::DragFloat("Sagging", (float *) &internodeData.m_sagging)) {
                changed = true;
            }
            if (ImGui::DragFloat("Extra mass", (float *) &internodeData.m_extraMass)) {
                changed = true;
            }


            if (ImGui::TreeNodeEx("Buds", ImGuiTreeNodeFlags_DefaultOpen)) {
                int index = 1;
                for (auto &bud: internodeData.m_buds) {
                    if (ImGui::TreeNode(("Bud " + std::to_string(index)).c_str())) {
                        switch (bud.m_type) {
                            case BudType::Apical:
                                ImGui::Text("Apical");
                                break;
                            case BudType::Lateral:
                                ImGui::Text("Lateral");
                                break;
                            case BudType::Leaf:
                                ImGui::Text("Leaf");
                                break;
                            case BudType::Fruit:
                                ImGui::Text("Fruit");
                                break;
                        }
                        switch (bud.m_status) {
                            case BudStatus::Dormant:
                                ImGui::Text("Dormant");
                                break;
                            case BudStatus::Flushed:
                                ImGui::Text("Flushed");
                                break;
                            case BudStatus::Died:
                                ImGui::Text("Died");
                                break;
                        }

                        auto budRotationAngle = glm::eulerAngles(bud.m_localRotation);
                        ImGui::InputFloat3("Rotation", &budRotationAngle.x, "%.3f", ImGuiInputTextFlags_ReadOnly);
                        ImGui::InputFloat("Base resource requirement", (float *) &bud.m_baseResourceRequirement, 1, 100,
                                          "%.3f", ImGuiInputTextFlags_ReadOnly);
                        ImGui::InputFloat("Productive resource requirement",
                                          (float *) &bud.m_productiveResourceRequirement,
                                          1, 100, "%.3f", ImGuiInputTextFlags_ReadOnly);
                        ImGui::TreePop();
                    }
                }
                ImGui::TreePop();
            }
            ImGui::TreePop();
        }
        if (ImGui::TreeNodeEx("Flow info", ImGuiTreeNodeFlags_DefaultOpen)) {
            const auto &flow = treeSkeleton.PeekFlow(internode.GetFlowHandle());
            ImGui::Text("Child flow size: %d", flow.RefChildHandles().size());
            ImGui::Text("Internode size: %d", flow.RefNodeHandles().size());
            if (ImGui::TreeNode("Internodes")) {
                int i = 0;
                for (const auto &chainedInternodeHandle: flow.RefNodeHandles()) {
                    ImGui::Text("No.%d: Handle: %d", i, chainedInternodeHandle);
                    i++;
                }
                ImGui::TreePop();
            }
            ImGui::TreePop();
        }
    }
    ImGui::End();
    return changed;
}

void
TreeVisualizer::PeekInternode(
        const Skeleton<SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeSkeleton,
        NodeHandle internodeHandle) {
    if (ImGui::Begin("Internode Inspector")) {
        const auto &internode = treeSkeleton.PeekNode(internodeHandle);
        if (ImGui::TreeNode("Internode info")) {
            ImGui::Text("Thickness: %.3f", internode.m_info.m_thickness);
            ImGui::Text("Length: %.3f", internode.m_info.m_length);
            ImGui::InputFloat3("Position", (float *) &internode.m_info.m_globalPosition.x, "%.3f",
                               ImGuiInputTextFlags_ReadOnly);
            auto globalRotationAngle = glm::eulerAngles(internode.m_info.m_globalRotation);
            ImGui::InputFloat3("Global rotation", (float *) &globalRotationAngle.x, "%.3f",
                               ImGuiInputTextFlags_ReadOnly);
            auto localRotationAngle = glm::eulerAngles(internode.m_info.m_localRotation);
            ImGui::InputFloat3("Local rotation", (float *) &localRotationAngle.x, "%.3f",
                               ImGuiInputTextFlags_ReadOnly);
            auto &internodeData = internode.m_data;
            ImGui::InputInt("Age", (int *) &internodeData.m_age, 1, 100, ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Inhibitor", (float *) &internodeData.m_inhibitor, 1, 100, "%.3f",
                              ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Sagging", (float *) &internodeData.m_sagging, 1, 100, "%.3f",
                              ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Distance to end", (float *) &internodeData.m_maxDistanceToAnyBranchEnd, 1, 100,
                              "%.3f",
                              ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Level", (float *) &internodeData.m_level, 1, 100, "%.3f",
                              ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Child biomass", (float *) &internodeData.m_childTotalBiomass, 1, 100, "%.3f",
                              ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Root distance", (float *) &internodeData.m_rootDistance, 1, 100, "%.3f",
                              ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat3("Light dir", (float *) &internodeData.m_lightDirection.x, "%.3f",
                               ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Light intensity", (float *) &internodeData.m_lightIntensity, 1, 100, "%.3f",
                              ImGuiInputTextFlags_ReadOnly);

            if (ImGui::TreeNodeEx("Buds")) {
                for (auto &bud: internodeData.m_buds) {
                    switch (bud.m_type) {
                        case BudType::Apical:
                            ImGui::Text("Apical");
                            break;
                        case BudType::Lateral:
                            ImGui::Text("Lateral");
                            break;
                        case BudType::Leaf:
                            ImGui::Text("Leaf");
                            break;
                        case BudType::Fruit:
                            ImGui::Text("Fruit");
                            break;
                    }
                    switch (bud.m_status) {
                        case BudStatus::Dormant:
                            ImGui::Text("Dormant");
                            break;
                        case BudStatus::Flushed:
                            ImGui::Text("Flushed");
                            break;
                        case BudStatus::Died:
                            ImGui::Text("Died");
                            break;
                    }

                    auto budRotationAngle = glm::eulerAngles(bud.m_localRotation);
                    ImGui::InputFloat3("Rotation", &budRotationAngle.x, "%.3f", ImGuiInputTextFlags_ReadOnly);
                    ImGui::InputFloat("Base resource requirement", (float *) &bud.m_baseResourceRequirement, 1, 100,
                                      "%.3f", ImGuiInputTextFlags_ReadOnly);
                    ImGui::InputFloat("Productive resource requirement",
                                      (float *) &bud.m_productiveResourceRequirement,
                                      1, 100, "%.3f", ImGuiInputTextFlags_ReadOnly);
                }
                ImGui::TreePop();
            }
            ImGui::TreePop();
        }
        if (ImGui::TreeNodeEx("Flow info", ImGuiTreeNodeFlags_DefaultOpen)) {
            const auto &flow = treeSkeleton.PeekFlow(internode.GetFlowHandle());
            ImGui::Text("Child flow size: %d", flow.RefChildHandles().size());
            ImGui::Text("Internode size: %d", flow.RefNodeHandles().size());
            if (ImGui::TreeNode("Internodes")) {
                int i = 0;
                for (const auto &chainedInternodeHandle: flow.RefNodeHandles()) {
                    ImGui::Text("No.%d: Handle: %d", i, chainedInternodeHandle);
                    i++;
                }
                ImGui::TreePop();
            }
            ImGui::TreePop();
        }
    }
    ImGui::End();
}

void TreeVisualizer::Reset(
        PlantStructure<SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeStructure) {
    m_selectedInternodeHandle = -1;
    m_selectedInternodeHierarchyList.clear();
    m_iteration = treeStructure.CurrentIteration();
    m_matrices.clear();
    m_needUpdate = true;
}

void
TreeVisualizer::SetSelectedInternode(
        const Skeleton<SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeSkeleton,
        NodeHandle internodeHandle) {
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
        const auto &internode = treeSkeleton.PeekNode(walker);
        walker = internode.GetParentHandle();
    }
}

bool TreeVisualizer::RayCastSelection(
        const Skeleton<SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeSkeleton,
        const GlobalTransform &globalTransform) {
    auto editorLayer = Application::GetLayer<EditorLayer>();
    bool changed = false;

    if (editorLayer->SceneCameraWindowFocused()) {
#pragma region Ray selection
        NodeHandle currentFocusingInternodeHandle = -1;
        std::mutex writeMutex;
        float minDistance = FLT_MAX;
        GlobalTransform cameraLtw;
        cameraLtw.m_value =
                glm::translate(
                        editorLayer->m_sceneCameraPosition) *
                glm::mat4_cast(
                        editorLayer->m_sceneCameraRotation);
        const Ray cameraRay = editorLayer->m_sceneCamera->ScreenPointToRay(
                cameraLtw, editorLayer->GetMouseScreenPosition());
        const auto &sortedBranchList = treeSkeleton.RefSortedFlowList();
        const auto &sortedInternodeList = treeSkeleton.RefSortedNodeList();
        std::vector<std::shared_future<void>> results;
        Jobs::ParallelFor(sortedInternodeList.size(), [&](unsigned i) {
            const auto &internode = treeSkeleton.PeekNode(sortedInternodeList[i]);
            auto rotation = globalTransform.GetRotation() * internode.m_info.m_globalRotation;
            glm::vec3 position = (globalTransform.m_value *
                                  glm::translate(internode.m_info.m_globalPosition))[3];
            const auto direction = glm::normalize(rotation * glm::vec3(0, 0, -1));
            const glm::vec3 position2 =
                    position + internode.m_info.m_length * direction;
            const auto center =
                    (position + position2) / 2.0f;
            auto radius = internode.m_info.m_thickness;
            const auto height = glm::distance(position2,
                                              position);
            radius *= height / internode.m_info.m_length;
            if (!cameraRay.Intersect(center,
                                     height / 2.0f) && !cameraRay.Intersect(center,
                                                                            radius)) {
                return;
            }
            const auto &dir = -cameraRay.m_direction;
#pragma region Line Line intersection
            /*
* http://geomalgorithms.com/a07-_distance.html
*/
            glm::vec3 v = position - position2;
            glm::vec3 w = (cameraRay.m_start + dir) - position2;
            const auto a = glm::dot(dir, dir); // always >= 0
            const auto b = glm::dot(dir, v);
            const auto c = glm::dot(v, v); // always >= 0
            const auto d = glm::dot(dir, w);
            const auto e = glm::dot(v, w);
            const auto dotP = a * c - b * b; // always >= 0
            float sc, tc;
            // compute the line parameters of the two closest points
            if (dotP < 0.00001f) { // the lines are almost parallel
                sc = 0.0f;
                tc = (b > c ? d / b : e / c); // use the largest denominator
            } else {
                sc = (b * e - c * d) / dotP;
                tc = (a * e - b * d) / dotP;
            }
            // get the difference of the two closest points
            glm::vec3 dP = w + sc * dir - tc * v; // =  L1(sc) - L2(tc)
            if (glm::length(dP) > radius)
                return;
#pragma endregion

            const auto distance = glm::distance(
                    glm::vec3(cameraLtw.m_value[3]),
                    glm::vec3(center));
            std::lock_guard<std::mutex> lock(writeMutex);
            if (distance < minDistance) {
                minDistance = distance;
                m_selectedInternodeLengthFactor = glm::clamp(1.0f - tc, 0.0f, 1.0f);
                currentFocusingInternodeHandle = sortedInternodeList[i];
            }
        }, results);
        for (auto &i: results) i.wait();


        if (currentFocusingInternodeHandle != -1) {
            SetSelectedInternode(treeSkeleton, currentFocusingInternodeHandle);
            changed = true;
        }

#pragma endregion
    }

    return changed;
}

void
TreeVisualizer::SyncMatrices(
        const Skeleton<SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeSkeleton) {
    static std::vector<glm::vec4> randomColors;
    if (randomColors.empty()) {
        for (int i = 0; i < 100; i++) {
            randomColors.emplace_back(glm::ballRand(1.0f), 1.0f);
        }
    }
    const auto &sortedBranchList = treeSkeleton.RefSortedFlowList();
    const auto &sortedInternodeList = treeSkeleton.RefSortedNodeList();
    m_matrices.resize(sortedInternodeList.size() + 1);
    m_colors.resize(sortedInternodeList.size() + 1);
    std::vector<std::shared_future<void>> results;
    Jobs::ParallelFor(sortedInternodeList.size(), [&](unsigned i) {
        auto internodeHandle = sortedInternodeList[i];
        const auto &internode = treeSkeleton.PeekNode(internodeHandle);
        glm::vec3 position = internode.m_info.m_globalPosition;
        const auto direction = glm::normalize(internode.m_info.m_globalRotation * glm::vec3(0, 0, -1));
        auto rotation = glm::quatLookAt(
                direction, glm::vec3(direction.y, direction.z, direction.x));
        rotation *= glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f));
        const glm::mat4 rotationTransform = glm::mat4_cast(rotation);
        m_matrices[i + 1] =
                glm::translate(position + (internode.m_info.m_length / 2.0f) * direction) *
                rotationTransform *
                glm::scale(glm::vec3(
                        internode.m_info.m_thickness,
                        internode.m_info.m_length / 2.0f,
                        internode.m_info.m_thickness));
        if (internodeHandle == m_selectedInternodeHandle) {
            m_colors[i + 1] = glm::vec4(1, 0, 0, 1);

            const glm::vec3 selectedCenter =
                    position + (internode.m_info.m_length * m_selectedInternodeLengthFactor) * direction;
            m_matrices[0] = glm::translate(selectedCenter) *
                            rotationTransform *
                            glm::scale(glm::vec3(
                                    internode.m_info.m_thickness + 0.001f,
                                    internode.m_info.m_length / 10.0f,
                                    internode.m_info.m_thickness + 0.001f));
            m_colors[0] = glm::vec4(1.0f);
        } else {
            m_colors[i + 1] = randomColors[treeSkeleton.PeekFlow(internode.GetFlowHandle()).m_data.m_order];
            if (m_selectedInternodeHandle != -1) m_colors[i + 1].a = 0.3f;
        }
    }, results);
    for (auto &i: results) i.wait();
}

bool TreeVisualizer::ScreenCurvePruning(
        Skeleton<SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeSkeleton,
        const GlobalTransform &globalTransform) {

    auto editorLayer = Application::GetLayer<EditorLayer>();
    const auto cameraRotation = editorLayer->m_sceneCameraRotation;
    const auto cameraPosition = editorLayer->m_sceneCameraPosition;
    const glm::vec3 cameraFront = cameraRotation * glm::vec3(0, 0, -1);
    const glm::vec3 cameraUp = cameraRotation * glm::vec3(0, 1, 0);
    glm::mat4 projectionView = editorLayer->m_sceneCamera->GetProjection() *
                               glm::lookAt(cameraPosition, cameraPosition + cameraFront, cameraUp);

    const auto &sortedInternodeList = treeSkeleton.RefSortedNodeList();
    bool changed = false;
    for (const auto &internodeHandle: sortedInternodeList) {
        if (internodeHandle == 0) continue;
        auto &internode = treeSkeleton.RefNode(internodeHandle);
        if (internode.IsRecycled()) continue;
        glm::vec3 position = internode.m_info.m_globalPosition;
        auto rotation = internode.m_info.m_globalRotation;
        const auto direction = glm::normalize(rotation * glm::vec3(0, 0, -1));
        auto position2 =
                position + internode.m_info.m_length * direction;

        position = (globalTransform.m_value *
                    glm::translate(position))[3];
        position2 = (globalTransform.m_value *
                     glm::translate(position2))[3];
        const glm::vec4 internodeScreenStart4 = projectionView * glm::vec4(position, 1.0f);
        const glm::vec4 internodeScreenEnd4 = projectionView * glm::vec4(position2, 1.0f);
        glm::vec3 internodeScreenStart = internodeScreenStart4 / internodeScreenStart4.w;
        glm::vec3 internodeScreenEnd = internodeScreenEnd4 / internodeScreenEnd4.w;
        internodeScreenStart.x *= -1.0f;
        internodeScreenEnd.x *= -1.0f;
        if (internodeScreenStart.x < -1.0f || internodeScreenStart.x > 1.0f || internodeScreenStart.y < -1.0f ||
            internodeScreenStart.y > 1.0f || internodeScreenStart.z < 0.0f)
            continue;
        if (internodeScreenEnd.x < -1.0f || internodeScreenEnd.x > 1.0f || internodeScreenEnd.y < -1.0f ||
            internodeScreenEnd.y > 1.0f || internodeScreenEnd.z < 0.0f)
            continue;
        bool intersect = false;
        for (int i = 0; i < m_storedMousePositions.size() - 1; i++) {
            auto &lineStart = m_storedMousePositions[i];
            auto &lineEnd = m_storedMousePositions[i + 1];
            float a1 = internodeScreenEnd.y - internodeScreenStart.y;
            float b1 = internodeScreenStart.x - internodeScreenEnd.x;
            float c1 = a1 * (internodeScreenStart.x) + b1 * (internodeScreenStart.y);

            // Line CD represented as a2x + b2y = c2
            float a2 = lineEnd.y - lineStart.y;
            float b2 = lineStart.x - lineEnd.x;
            float c2 = a2 * (lineStart.x) + b2 * (lineStart.y);

            float determinant = a1 * b2 - a2 * b1;
            if (determinant == 0.0f) continue;
            float x = (b2 * c1 - b1 * c2) / determinant;
            float y = (a1 * c2 - a2 * c1) / determinant;
            if (x <= glm::max(internodeScreenStart.x, internodeScreenEnd.x) &&
                x >= glm::min(internodeScreenStart.x, internodeScreenEnd.x) &&
                y <= glm::max(internodeScreenStart.y, internodeScreenEnd.y) &&
                y >= glm::min(internodeScreenStart.y, internodeScreenEnd.y) &&
                x <= glm::max(lineStart.x, lineEnd.x) &&
                x >= glm::min(lineStart.x, lineEnd.x) &&
                y <= glm::max(lineStart.y, lineEnd.y) &&
                y >= glm::min(lineStart.y, lineEnd.y)) {
                intersect = true;
                break;
            }
        }
        if (intersect) {
            treeSkeleton.RecycleNode(internodeHandle);
            changed = true;
        }

    }
    if (changed) {
        m_selectedInternodeHandle = -1;
        m_selectedInternodeHierarchyList.clear();
    }
    return changed;
}
