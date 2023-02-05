//
// Created by lllll on 11/20/2022.
//

#include "TreeVisualizer.hpp"

using namespace EcoSysLab;

bool TreeVisualizer::DrawInternodeInspectionGui(
        TreeModel &treeModel,
        NodeHandle internodeHandle,
        bool &deleted,
        const unsigned &hierarchyLevel) {
    auto &treeSkeleton = treeModel.RefShootSkeleton();
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
        SetSelectedNode(treeSkeleton, internodeHandle, m_selectedInternodeHandle, m_selectedInternodeHierarchyList);
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
            DrawInternodeInspectionGui(treeModel, child, childDeleted, hierarchyLevel + 1);
            if (childDeleted) {
                treeModel.Step();
                treeSkeleton.RecycleNode(child);
                treeSkeleton.SortLists();
                m_iteration = treeModel.CurrentIteration();
                modified = true;
                break;
            }
        }
        ImGui::TreePop();
    }
    return modified;
}

bool
TreeVisualizer::OnInspect(
        TreeModel &treeModel,
        const GlobalTransform &globalTransform) {
    bool updated = false;
    if (ImGui::TreeNodeEx("Selected Tree Visualizer", ImGuiTreeNodeFlags_DefaultOpen)) {
        if(ImGui::Checkbox("Use node color", &m_useNodeColor))
        {
            m_needUpdate = true;
        }

        if (treeModel.CurrentIteration() > 0) {
            if (ImGui::TreeNodeEx("History", ImGuiTreeNodeFlags_DefaultOpen)) {
                ImGui::DragInt("History Limit", &treeModel.m_historyLimit, 1, -1, 1024);
                if (ImGui::SliderInt("Iteration", &m_iteration, 0, treeModel.CurrentIteration())) {
                    m_iteration = glm::clamp(m_iteration, 0, treeModel.CurrentIteration());
                    m_selectedInternodeHandle = -1;
                    m_selectedInternodeHierarchyList.clear();
                    m_selectedRootNodeHandle = -1;
                    m_selectedRootNodeHierarchyList.clear();
                    m_needUpdate = true;
                }
                if (m_iteration != treeModel.CurrentIteration() && ImGui::Button("Reverse")) {
                    treeModel.Reverse(m_iteration);
                    m_needUpdate = true;
                }
                if (ImGui::Button("Clear history")) {
                    m_iteration = 0;
                    treeModel.ClearHistory();
                }
                ImGui::TreePop();
            }
        }

        if (ImGui::TreeNodeEx("Settings")) {
            ImGui::Checkbox("Visualization", &m_visualization);
            ImGui::Checkbox("Tree Hierarchy", &m_treeHierarchyGui);
            ImGui::Checkbox("Root Hierarchy", &m_rootHierarchyGui);
            ImGui::TreePop();
        }

        if (m_treeHierarchyGui) {
            if (ImGui::TreeNodeEx("Tree Hierarchy")) {
                bool deleted = false;
                auto tempSelection = m_selectedInternodeHandle;
                if (m_iteration == treeModel.CurrentIteration()) {
                    if (DrawInternodeInspectionGui(treeModel, 0, deleted, 0)) {
                        m_needUpdate = true;
                        updated = true;
                    }
                } else
                    PeekNodeInspectionGui(treeModel.PeekShootSkeleton(m_iteration), 0, m_selectedInternodeHandle,
                                          m_selectedInternodeHierarchyList, 0);
                m_selectedInternodeHierarchyList.clear();
                if (tempSelection != m_selectedInternodeHandle) {
                    m_selectedRootNodeHandle = -1;
                    m_selectedRootNodeHierarchyList.clear();
                }
                ImGui::TreePop();
            }
            if (m_selectedInternodeHandle >= 0) {
                if (m_iteration == treeModel.CurrentIteration()) {
                    InspectInternode(treeModel.RefShootSkeleton(), m_selectedInternodeHandle);
                } else {
                    PeekInternode(treeModel.PeekShootSkeleton(m_iteration), m_selectedInternodeHandle);
                }
                
            }
        }
        if (m_rootHierarchyGui) {
            if (ImGui::TreeNodeEx("Root Hierarchy")) {
                bool deleted = false;
                const auto tempSelection = m_selectedRootNodeHandle;
                if (m_iteration == treeModel.CurrentIteration()) {
                    if (DrawRootNodeInspectionGui(treeModel, 0, deleted, 0)) {
                        m_needUpdate = true;
                        updated = true;
                    }
                }
                else
                    PeekNodeInspectionGui(treeModel.PeekRootSkeleton(m_iteration), 0, m_selectedRootNodeHandle,
                        m_selectedRootNodeHierarchyList, 0);
                m_selectedRootNodeHierarchyList.clear();
                if (tempSelection != m_selectedRootNodeHandle) {
                    m_selectedInternodeHandle = -1;
                    m_selectedInternodeHierarchyList.clear();
                }
                ImGui::TreePop();
            }
            if (m_selectedRootNodeHandle >= 0) {
                if (m_iteration == treeModel.CurrentIteration()) {
                    InspectRootNode(treeModel.RefRootSkeleton(), m_selectedRootNodeHandle);
                }
                else {
                    PeekRootNode(treeModel.PeekRootSkeleton(m_iteration), m_selectedRootNodeHandle);
                }
            }
        }

        if (m_visualization) {
            const auto& treeSkeleton = treeModel.PeekShootSkeleton(m_iteration);
            const auto& rootSkeleton = treeModel.PeekRootSkeleton(m_iteration);
            const auto editorLayer = Application::GetLayer<EditorLayer>();
            const auto& sortedBranchList = treeSkeleton.RefSortedFlowList();
            const auto& sortedInternodeList = treeSkeleton.RefSortedNodeList();
            ImGui::Text("Internode count: %d", sortedInternodeList.size());
            ImGui::Text("Shoot stem count: %d", sortedBranchList.size());

            const auto& sortedRootFlowList = rootSkeleton.RefSortedFlowList();
            const auto& sortedRootNodeList = rootSkeleton.RefSortedNodeList();
            ImGui::Text("Root node count: %d", sortedRootNodeList.size());
            ImGui::Text("Root stem count: %d", sortedRootFlowList.size());

            static bool enableStroke = false;
            if (ImGui::Checkbox("Enable stroke", &enableStroke)) {
                if (enableStroke) {
                    m_mode = PruningMode::Stroke;
                }
                else {
                    m_mode = PruningMode::None;
                }
                m_storedMousePositions.clear();
            }
        }
        ImGui::TreePop();
    }

    return updated;
}

bool TreeVisualizer::Visualize(TreeModel& treeModel,
    const GlobalTransform& globalTransform)
{
    bool updated = false;
    const auto& treeSkeleton = treeModel.PeekShootSkeleton(m_iteration);
    const auto& rootSkeleton = treeModel.PeekRootSkeleton(m_iteration);
    if (m_visualization) {
        const auto editorLayer = Application::GetLayer<EditorLayer>();
        switch (m_mode) {
        case PruningMode::None: {
            if (Inputs::GetMouseInternal(GLFW_MOUSE_BUTTON_LEFT, Windows::GetWindow())) {
                if (RayCastSelection(treeSkeleton, globalTransform, m_selectedInternodeHandle, m_selectedInternodeHierarchyList, m_selectedInternodeLengthFactor)) {
                    m_needUpdate = true;
                    updated = true;
                    m_selectedRootNodeHandle = -1;
                    m_selectedRootNodeHierarchyList.clear();
                }
                else if (RayCastSelection(rootSkeleton, globalTransform, m_selectedRootNodeHandle, m_selectedRootNodeHierarchyList, m_selectedRootNodeLengthFactor)) {
                    m_needUpdate = true;
                    updated = true;
                    m_selectedInternodeHandle = -1;
                    m_selectedInternodeHierarchyList.clear();
                }
            }
            if (m_iteration == treeModel.CurrentIteration() &&
                Inputs::GetKeyInternal(GLFW_KEY_DELETE,
                    Windows::GetWindow())) {
                if (m_selectedInternodeHandle > 0) {
                    treeModel.Step();
                    auto& skeleton = treeModel.RefShootSkeleton();
                    auto& pruningInternode = skeleton.RefNode(m_selectedInternodeHandle);
                    auto childHandles = pruningInternode.RefChildHandles();
                    for (const auto& childHandle : childHandles) {
                        skeleton.RecycleNode(childHandle);
                    }
                    pruningInternode.m_info.m_length *= m_selectedInternodeLengthFactor;
                    m_selectedInternodeLengthFactor = 1.0f;
                    for (auto& bud : pruningInternode.m_data.m_buds) {
                        bud.m_status = BudStatus::Died;
                    }
                    skeleton.SortLists();
                    m_iteration = treeModel.CurrentIteration();
                    m_needUpdate = true;
                    updated = true;
                }
                if (m_selectedRootNodeHandle > 0) {
                    treeModel.Step();
                    auto& skeleton = treeModel.RefRootSkeleton();
                    auto& pruningRootNode = skeleton.RefNode(m_selectedRootNodeHandle);
                    auto childHandles = pruningRootNode.RefChildHandles();
                    for (const auto& childHandle : childHandles) {
                        skeleton.RecycleNode(childHandle);
                    }
                    pruningRootNode.m_info.m_length *= m_selectedRootNodeLengthFactor;
                    m_selectedRootNodeLengthFactor = 1.0f;
                    skeleton.SortLists();
                    m_iteration = treeModel.CurrentIteration();
                    m_needUpdate = true;
                    updated = true;
                }
            }
        }
                              break;
        case PruningMode::Stroke: {
            if (m_iteration == treeModel.CurrentIteration()) {
                if (Inputs::GetMouseInternal(GLFW_MOUSE_BUTTON_LEFT, Windows::GetWindow())) {
                    glm::vec2 mousePosition = editorLayer->GetMouseScreenPosition();
                    const float halfX = editorLayer->m_sceneCamera->GetResolution().x / 2.0f;
                    const float halfY = editorLayer->m_sceneCamera->GetResolution().y / 2.0f;
                    mousePosition = { -1.0f * (mousePosition.x - halfX) / halfX,
                                     -1.0f * (mousePosition.y - halfY) / halfY };
                    if (mousePosition.x > -1.0f && mousePosition.x < 1.0f && mousePosition.y > -1.0f &&
                        mousePosition.y < 1.0f &&
                        (m_storedMousePositions.empty() || mousePosition != m_storedMousePositions.back())) {
                        m_storedMousePositions.emplace_back(mousePosition);
                    }
                }
                else {
                    //Once released, check if empty.
                    if (!m_storedMousePositions.empty()) {
                        treeModel.Step();
                        auto& skeleton = treeModel.RefShootSkeleton();
                        bool changed = ScreenCurvePruning(skeleton, globalTransform, m_selectedInternodeHandle, m_selectedInternodeHierarchyList);
                        if (changed) {
                            skeleton.SortLists();
                            m_iteration = treeModel.CurrentIteration();
                            m_needUpdate = true;
                            updated = true;
                        }
                        else {
                            treeModel.Pop();
                        }
                        m_storedMousePositions.clear();
                    }
                }
            }
        }
                                break;
        }


        if (m_needUpdate) {
            SyncMatrices(treeSkeleton, m_internodeMatrices, m_internodeColors, m_selectedInternodeHandle, m_selectedInternodeLengthFactor);
            SyncMatrices(rootSkeleton, m_rootNodeMatrices, m_rootNodeColors, m_selectedRootNodeHandle, m_selectedRootNodeLengthFactor);
            m_needUpdate = false;
        }
        if (!m_internodeMatrices.empty()) {
            GizmoSettings gizmoSettings;
            gizmoSettings.m_drawSettings.m_blending = true;
            if (m_selectedInternodeHandle == -1) {
                m_internodeMatrices[0] = glm::translate(glm::vec3(1.0f)) * glm::scale(glm::vec3(0.0f));
                m_internodeColors[0] = glm::vec4(0.0f);
            }
            Gizmos::DrawGizmoMeshInstancedColored(
                DefaultResources::Primitives::Cylinder, editorLayer->m_sceneCamera,
                editorLayer->m_sceneCameraPosition,
                editorLayer->m_sceneCameraRotation,
                m_internodeColors,
                m_internodeMatrices,
                globalTransform.m_value, 1.0f, gizmoSettings);

        }
        if (!m_rootNodeMatrices.empty()) {
            GizmoSettings gizmoSettings;
            gizmoSettings.m_drawSettings.m_blending = true;
            if (m_selectedRootNodeHandle == -1) {
                m_rootNodeMatrices[0] = glm::translate(glm::vec3(1.0f)) * glm::scale(glm::vec3(0.0f));
                m_rootNodeColors[0] = glm::vec4(0.0f);
            }
            Gizmos::DrawGizmoMeshInstancedColored(
                DefaultResources::Primitives::Cylinder, editorLayer->m_sceneCamera,
                editorLayer->m_sceneCameraPosition,
                editorLayer->m_sceneCameraRotation,
                m_rootNodeColors,
                m_rootNodeMatrices,
                globalTransform.m_value, 1.0f, gizmoSettings);
        }
    }
    return updated;
}

bool
TreeVisualizer::InspectInternode(
        ShootSkeleton &shootSkeleton,
        NodeHandle internodeHandle) {
    bool changed = false;
    if (ImGui::Begin("Internode Inspector")) {
        const auto &internode = shootSkeleton.RefNode(internodeHandle);
        if (ImGui::TreeNode("Internode info")) {
            ImGui::Checkbox("Is max child", (bool*)&internode.m_data.m_isMaxChild);
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
            ImGui::InputFloat("Descendent biomass", (float *) &internodeData.m_descendentTotalBiomass, 1, 100, "%.3f",
                              ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Biomass", (float *) &internodeData.m_biomass, 1, 100, "%.3f",
                              ImGuiInputTextFlags_ReadOnly);

            ImGui::InputFloat("Root distance", (float *) &internodeData.m_rootDistance, 1, 100, "%.3f",
                              ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat3("Light dir", (float *) &internodeData.m_lightDirection.x, "%.3f",
                               ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Light intensity", (float *) &internodeData.m_lightIntensity, 1, 100, "%.3f",
                              ImGuiInputTextFlags_ReadOnly);

            if (ImGui::DragFloat("Sagging", (float *) &internodeData.m_sagging)) {
                changed = true;
            }
            if (ImGui::DragFloat("Extra mass", (float *) &internodeData.m_extraMass)) {
                changed = true;
            }

            ImGui::InputFloat("Productive req", (float *) &internodeData.m_developmentalVigorRequirementWeight, 1, 100,
                              "%.3f",
                              ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Descendent req", (float *) &internodeData.m_subtreeDevelopmentalVigorRequirementWeight, 1,
                              100, "%.3f",
                              ImGuiInputTextFlags_ReadOnly);
            

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
                        /*
                        ImGui::InputFloat("Base resource requirement", (float *) &bud.m_maintenanceVigorRequirementWeight, 1, 100,
                                          "%.3f", ImGuiInputTextFlags_ReadOnly);
                                          */
                        ImGui::TreePop();
                    }
                    index++;
                }
                ImGui::TreePop();
            }
            ImGui::TreePop();
        }
        if (ImGui::TreeNodeEx("Flow info", ImGuiTreeNodeFlags_DefaultOpen)) {
            const auto &flow = shootSkeleton.PeekFlow(internode.GetFlowHandle());
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

bool TreeVisualizer::DrawRootNodeInspectionGui(TreeModel& treeModel, NodeHandle rootNodeHandle, bool& deleted, const unsigned& hierarchyLevel)
{
    auto& rootSkeleton = treeModel.RefRootSkeleton();
    const int index = m_selectedRootNodeHierarchyList.size() - hierarchyLevel - 1;
    if (!m_selectedRootNodeHierarchyList.empty() && index >= 0 &&
        index < m_selectedRootNodeHierarchyList.size() &&
        m_selectedRootNodeHierarchyList[index] == rootNodeHandle) {
        ImGui::SetNextItemOpen(true);
    }
    const bool opened = ImGui::TreeNodeEx(("Handle: " + std::to_string(rootNodeHandle)).c_str(),
        ImGuiTreeNodeFlags_NoTreePushOnOpen | ImGuiTreeNodeFlags_OpenOnArrow |
        ImGuiTreeNodeFlags_NoAutoOpenOnLog |
        (m_selectedRootNodeHandle == rootNodeHandle ? ImGuiTreeNodeFlags_Framed
            : ImGuiTreeNodeFlags_FramePadding));
    if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
        SetSelectedNode(rootSkeleton, rootNodeHandle, m_selectedRootNodeHandle, m_selectedRootNodeHierarchyList);
    }

    if (ImGui::BeginPopupContextItem(std::to_string(rootNodeHandle).c_str())) {
        ImGui::Text(("Handle: " + std::to_string(rootNodeHandle)).c_str());
        if (ImGui::Button("Delete")) {
            deleted = true;
        }
        ImGui::EndPopup();
    }
    bool modified = deleted;
    if (opened && !deleted) {
        ImGui::TreePush();
        const auto& rootNodeChildren = rootSkeleton.RefNode(rootNodeHandle).RefChildHandles();
        for (const auto& child : rootNodeChildren) {
            bool childDeleted = false;
            DrawRootNodeInspectionGui(treeModel, child, childDeleted, hierarchyLevel + 1);
            if (childDeleted) {
                treeModel.Step();
                rootSkeleton.RecycleNode(child);
                rootSkeleton.SortLists();
                m_iteration = treeModel.CurrentIteration();
                modified = true;
                break;
            }
        }
        ImGui::TreePop();
    }
    return modified;
}

void
TreeVisualizer::PeekInternode(const ShootSkeleton &shootSkeleton, NodeHandle internodeHandle) const
{
    if (ImGui::Begin("Internode Inspector")) {
        const auto &internode = shootSkeleton.PeekNode(internodeHandle);
        if (ImGui::TreeNode("Internode info")) {
            ImGui::Checkbox("Is max child", (bool*)&internode.m_data.m_isMaxChild);
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
            ImGui::InputFloat("Sagging", (float *) &internodeData.m_sagging, 1, 100, "%.3f",
                              ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Distance to end", (float *) &internodeData.m_maxDistanceToAnyBranchEnd, 1, 100,
                              "%.3f",
                              ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Descendent biomass", (float *) &internodeData.m_descendentTotalBiomass, 1, 100, "%.3f",
                              ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Biomass", (float *) &internodeData.m_biomass, 1, 100, "%.3f",
                              ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Root distance", (float *) &internodeData.m_rootDistance, 1, 100, "%.3f",
                              ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat3("Light dir", (float *) &internodeData.m_lightDirection.x, "%.3f",
                               ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Light intensity", (float *) &internodeData.m_lightIntensity, 1, 100, "%.3f",
                              ImGuiInputTextFlags_ReadOnly);

            ImGui::InputFloat("Productive req", (float *) &internodeData.m_developmentalVigorRequirementWeight, 1, 100,
                              "%.3f",
                              ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Descendent req", (float *) &internodeData.m_subtreeDevelopmentalVigorRequirementWeight, 1,
                              100, "%.3f",
                              ImGuiInputTextFlags_ReadOnly);
            
            
            if (ImGui::TreeNodeEx("Buds")) {
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
                        /*
                        ImGui::InputFloat("Base resource requirement", (float *) &bud.m_maintenanceVigorRequirementWeight, 1, 100,
                                          "%.3f", ImGuiInputTextFlags_ReadOnly);
                                          */
                        ImGui::TreePop();
                    }
                    index++;
                }
                ImGui::TreePop();
            }
            ImGui::TreePop();
        }
        if (ImGui::TreeNodeEx("Stem info", ImGuiTreeNodeFlags_DefaultOpen)) {
            const auto &flow = shootSkeleton.PeekFlow(internode.GetFlowHandle());
            ImGui::Text("Child stem size: %d", flow.RefChildHandles().size());
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
        TreeModel &treeModel) {
    m_selectedInternodeHandle = -1;
    m_selectedInternodeHierarchyList.clear();
    m_selectedRootNodeHandle = -1;
    m_selectedRootNodeHierarchyList.clear();
    m_iteration = treeModel.CurrentIteration();
    m_internodeMatrices.clear();
    m_rootNodeMatrices.clear();
    m_needUpdate = true;
}

void TreeVisualizer::Clear()
{
    m_selectedInternodeHandle = -1;
    m_selectedInternodeHierarchyList.clear();
    m_selectedRootNodeHandle = -1;
    m_selectedRootNodeHierarchyList.clear();
    m_iteration = 0;
    m_internodeMatrices.clear();
    m_rootNodeMatrices.clear();
}


void TreeVisualizer::PeekRootNode(
        const RootSkeleton &rootSkeleton,
        NodeHandle rootNodeHandle) const
{
    if (ImGui::Begin("Root Node Inspector")) {
        const auto& rootNode = rootSkeleton.PeekNode(rootNodeHandle);
        if (ImGui::TreeNode("Root node info")) {
            ImGui::Checkbox("Is max child", (bool*)&rootNode.m_data.m_isMaxChild);
            ImGui::Text("Thickness: %.3f", rootNode.m_info.m_thickness);
            ImGui::Text("Length: %.3f", rootNode.m_info.m_length);
            ImGui::InputFloat3("Position", (float*)&rootNode.m_info.m_globalPosition.x, "%.3f",
                ImGuiInputTextFlags_ReadOnly);
            auto globalRotationAngle = glm::eulerAngles(rootNode.m_info.m_globalRotation);
            ImGui::InputFloat3("Global rotation", (float*)&globalRotationAngle.x, "%.3f",
                ImGuiInputTextFlags_ReadOnly);
            auto localRotationAngle = glm::eulerAngles(rootNode.m_info.m_localRotation);
            ImGui::InputFloat3("Local rotation", (float*)&localRotationAngle.x, "%.3f",
                ImGuiInputTextFlags_ReadOnly);
            auto& rootNodeData = rootNode.m_data;
            ImGui::InputFloat("Nitrite", (float*)&rootNodeData.m_nitrite, 1, 100, "%.3f",
                ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Soil Density", (float*)&rootNodeData.m_soilDensity, 1, 100, "%.3f",
                ImGuiInputTextFlags_ReadOnly);

            ImGui::InputFloat("Root flux", (float*)&rootNodeData.m_water, 1, 100, "%.3f",
                ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Growth potential", (float*)&rootNodeData.m_developmentalVigorRequirementWeight, 1, 100, "%.3f",
                ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Subtree Growth potential", (float*)&rootNodeData.m_subtreeDevelopmentalVigorRequirementWeight, 1, 100, "%.3f",
                ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Vigor", (float*)&rootNodeData.m_vigorSink, 1, 100, "%.3f",
                ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Subtree vigor", (float*)&rootNodeData.m_subTreeAllocatedVigor, 1, 100, "%.3f",
                ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Auxin", (float*)&rootNodeData.m_inhibitor, 1, 100, "%.3f",
                ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Auxin target", (float*)&rootNodeData.m_inhibitorTarget, 1, 100,
                "%.3f",
                ImGuiInputTextFlags_ReadOnly);

            ImGui::InputFloat("Horizontal tropism", (float*)&rootNodeData.m_horizontalTropism, 1, 100,
                "%.3f",
                ImGuiInputTextFlags_ReadOnly);

            ImGui::InputFloat("Vertical tropism", (float*)&rootNodeData.m_verticalTropism, 1, 100,
                "%.3f",
                ImGuiInputTextFlags_ReadOnly);
            ImGui::TreePop();
        }
        if (ImGui::TreeNodeEx("Stem info", ImGuiTreeNodeFlags_DefaultOpen)) {
            const auto& flow = rootSkeleton.PeekFlow(rootNode.GetFlowHandle());
            ImGui::Text("Child stem size: %d", flow.RefChildHandles().size());
            ImGui::Text("Root node size: %d", flow.RefNodeHandles().size());
            if (ImGui::TreeNode("Root nodes")) {
                int i = 0;
                for (const auto& chainedInternodeHandle : flow.RefNodeHandles()) {
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

bool TreeVisualizer::InspectRootNode(
        RootSkeleton &rootSkeleton,
        NodeHandle rootNodeHandle) {
    bool changed = false;
    if (ImGui::Begin("Root Node Inspector")) {
        const auto& rootNode = rootSkeleton.RefNode(rootNodeHandle);
        if (ImGui::TreeNode("Root node info")) {
            ImGui::Checkbox("Is max child", (bool*) & rootNode.m_data.m_isMaxChild);
            ImGui::Text("Thickness: %.3f", rootNode.m_info.m_thickness);
            ImGui::Text("Length: %.3f", rootNode.m_info.m_length);
            ImGui::InputFloat3("Position", (float*)&rootNode.m_info.m_globalPosition.x, "%.3f",
                ImGuiInputTextFlags_ReadOnly);
            auto globalRotationAngle = glm::eulerAngles(rootNode.m_info.m_globalRotation);
            ImGui::InputFloat3("Global rotation", (float*)&globalRotationAngle.x, "%.3f",
                ImGuiInputTextFlags_ReadOnly);
            auto localRotationAngle = glm::eulerAngles(rootNode.m_info.m_localRotation);
            ImGui::InputFloat3("Local rotation", (float*)&localRotationAngle.x, "%.3f",
                ImGuiInputTextFlags_ReadOnly);
            auto& rootNodeData = rootNode.m_data;
            
            ImGui::InputFloat("Root distance", (float*)&rootNodeData.m_rootDistance, 1, 100, "%.3f",
                ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Soil density", (float*)&rootNodeData.m_soilDensity, 1, 100, "%.3f",
                ImGuiInputTextFlags_ReadOnly);
            
            
            ImGui::InputFloat("Nitrite", (float*)&rootNodeData.m_nitrite, 1, 100, "%.3f",
                ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Root flux", (float*)&rootNodeData.m_water, 1, 100, "%.3f",
                ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Growth potential", (float*)&rootNodeData.m_developmentalVigorRequirementWeight, 1, 100, "%.3f",
                ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Subtree Growth potential", (float*)&rootNodeData.m_subtreeDevelopmentalVigorRequirementWeight, 1, 100, "%.3f",
                ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Vigor", (float*)&rootNodeData.m_vigorSink, 1, 100, "%.3f",
                ImGuiInputTextFlags_ReadOnly);
            ImGui::InputFloat("Subtree vigor", (float*)&rootNodeData.m_subTreeAllocatedVigor, 1, 100, "%.3f",
                ImGuiInputTextFlags_ReadOnly);
            if (ImGui::DragFloat("Inhibitor", (float*)&rootNodeData.m_inhibitor)) {
                changed = true;
            }
            if (ImGui::DragFloat("Inhibitor target", (float*)&rootNodeData.m_inhibitorTarget)) {
                changed = true;
            }
            ImGui::InputFloat("Horizontal tropism", (float*)&rootNodeData.m_horizontalTropism, 1, 100,
                "%.3f",
                ImGuiInputTextFlags_ReadOnly);

            ImGui::InputFloat("Vertical tropism", (float*)&rootNodeData.m_verticalTropism, 1, 100,
                "%.3f",
                ImGuiInputTextFlags_ReadOnly);
            ImGui::TreePop();
        }
        if (ImGui::TreeNodeEx("Flow info", ImGuiTreeNodeFlags_DefaultOpen)) {
            const auto& flow = rootSkeleton.PeekFlow(rootNode.GetFlowHandle());
            ImGui::Text("Child flow size: %d", flow.RefChildHandles().size());
            ImGui::Text("Root node size: %d", flow.RefNodeHandles().size());
            if (ImGui::TreeNode("Root nodes")) {
                int i = 0;
                for (const auto& chainedInternodeHandle : flow.RefNodeHandles()) {
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










