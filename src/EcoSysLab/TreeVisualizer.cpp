//
// Created by lllll on 11/20/2022.
//

#include "TreeVisualizer.hpp"

using namespace EcoSysLab;

bool TreeVisualizer::DrawInternodeInspectionGui(
        TreeStructure<SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeStructure,
        InternodeHandle internodeHandle,
        bool &deleted,
        const unsigned int &hierarchyLevel) {
    auto &treeSkeleton = treeStructure.Skeleton();
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
        const auto &internodeChildren = treeSkeleton.RefInternode(internodeHandle).RefChildHandles();
        for (const auto &child: internodeChildren) {
            bool childDeleted = false;
            DrawInternodeInspectionGui(treeStructure, child, childDeleted, hierarchyLevel + 1);
            if (childDeleted) {
                treeStructure.Step();
                treeSkeleton.RecycleInternode(child);
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
        const TreeSkeleton<SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeSkeleton,
        InternodeHandle internodeHandle,
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
        const auto &internode = treeSkeleton.PeekInternode(internodeHandle);
        const auto &internodeChildren = internode.RefChildHandles();
        for (const auto &child: internodeChildren) {
            PeekInternodeInspectionGui(treeSkeleton, child, hierarchyLevel + 1);
        }
        ImGui::TreePop();
    }
}

bool
TreeVisualizer::OnInspect(
        TreeStructure<SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeStructure,
        const GlobalTransform &globalTransform) {
    bool needUpdate = false;
    const auto &treeSkeleton = treeStructure.Peek(m_iteration);
    if (ImGui::TreeNodeEx("Current selected tree", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (treeStructure.CurrentIteration() > 0) {
            if (ImGui::TreeNodeEx("History", ImGuiTreeNodeFlags_DefaultOpen)) {
                if (ImGui::SliderInt("Iteration", &m_iteration, 0, treeStructure.CurrentIteration())) {
                    m_iteration = glm::clamp(m_iteration, 0, treeStructure.CurrentIteration());
                    m_selectedInternodeHandle = -1;
                    m_selectedInternodeHierarchyList.clear();
                }
                if (m_iteration != treeStructure.CurrentIteration() && ImGui::Button("Reverse")) {
                    treeStructure.Reverse(m_iteration);
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
                if (m_iteration == treeStructure.CurrentIteration())
                    needUpdate = DrawInternodeInspectionGui(treeStructure, 0, deleted, 0);
                else PeekInternodeInspectionGui(treeStructure.Peek(m_iteration), 0, 0);
                m_selectedInternodeHierarchyList.clear();
                ImGui::TreePop();
            }
        }
        if (m_treeHierarchyGui) {
            if (m_selectedInternodeHandle >= 0) {
                InspectInternode(treeSkeleton, m_selectedInternodeHandle);
            }
        }

        ImGui::TreePop();
    }
    if (m_visualization) {
        const auto &sortedBranchList = treeSkeleton.RefSortedFlowList();
        const auto &sortedInternodeList = treeSkeleton.RefSortedInternodeList();
        ImGui::Text("Internode count: %d", sortedInternodeList.size());
        ImGui::Text("Flow count: %d", sortedBranchList.size());
        if (treeSkeleton.GetVersion() != m_version) {
            needUpdate = true;
            m_iteration = treeStructure.CurrentIteration();
        }
        if (Inputs::GetMouseInternal(GLFW_MOUSE_BUTTON_LEFT, Windows::GetWindow())) {
            if (RayCastSelection(treeSkeleton, globalTransform)) needUpdate = true;
        }
        if (m_iteration == treeStructure.CurrentIteration() && m_selectedInternodeHandle > 0 &&
            Inputs::GetKeyInternal(GLFW_KEY_DELETE,
                                   Windows::GetWindow())) {
            treeStructure.Step();
            auto& skeleton = treeStructure.Skeleton();
            auto& pruningInternode = skeleton.RefInternode(m_selectedInternodeHandle);
            auto childHandles = pruningInternode.RefChildHandles();
            for(const auto& childHandle : childHandles){
                skeleton.RecycleInternode(childHandle);
            }
            pruningInternode.m_info.m_length *= m_selectedInternodeLengthFactor;
            m_selectedInternodeLengthFactor = 1.0f;
            for(auto& bud : pruningInternode.m_data.m_buds){
                bud.m_status = BudStatus::Died;
            }
            skeleton.SortLists();
            m_iteration = treeStructure.CurrentIteration();
            needUpdate = true;
        }
        if (needUpdate) {
            SyncMatrices(treeSkeleton);
        }
        if (!m_matrices.empty()) {
            auto editorLayer = Application::GetLayer<EditorLayer>();
            GizmoSettings m_gizmoSettings;
            m_gizmoSettings.m_drawSettings.m_blending = true;
            if(m_selectedInternodeHandle == -1){
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

    return needUpdate;
}

void
TreeVisualizer::InspectInternode(
        const TreeSkeleton<SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeSkeleton,
        InternodeHandle internodeHandle) {
    if (ImGui::Begin("Internode Inspector")) {
        const auto &internode = treeSkeleton.PeekInternode(internodeHandle);
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
            ImGui::Text("Internode size: %d", flow.RefInternodes().size());
            if (ImGui::TreeNode("Internodes")) {
                int i = 0;
                for (const auto &chainedInternodeHandle: flow.RefInternodes()) {
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
        TreeStructure<SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeStructure) {
    m_version = -1;
    m_selectedInternodeHandle = -1;
    m_selectedInternodeHierarchyList.clear();
    m_iteration = treeStructure.CurrentIteration();
    m_matrices.clear();
}

void
TreeVisualizer::SetSelectedInternode(
        const TreeSkeleton<SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeSkeleton,
        InternodeHandle internodeHandle) {
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
        const auto &internode = treeSkeleton.PeekInternode(walker);
        walker = internode.GetParentHandle();
    }
}

bool TreeVisualizer::RayCastSelection(
        const TreeSkeleton<SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeSkeleton,
        const GlobalTransform &globalTransform) {
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
                const auto &sortedBranchList = treeSkeleton.RefSortedFlowList();
                const auto &sortedInternodeList = treeSkeleton.RefSortedInternodeList();
                std::vector<std::shared_future<void>> results;
                Jobs::ParallelFor(sortedInternodeList.size(), [&](unsigned i) {
                    const auto &internode = treeSkeleton.PeekInternode(sortedInternodeList[i]);
                    auto rotation = globalTransform.GetRotation() * internode.m_info.m_globalRotation;
                    glm::vec3 position = (globalTransform.m_value *
                                          glm::translate(internode.m_info.m_globalPosition))[3];
                    const auto direction = glm::normalize(rotation * glm::vec3(0, 0, -1));
                    const glm::vec3 position2 =
                            position + internode.m_info.m_length * direction;
                    const auto center =
                            (position + position2) / 2.0f;
                    auto dir = cameraRay.m_direction;
                    auto pos = cameraRay.m_start;
                    const auto radius = internode.m_info.m_thickness;
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
                    const auto a = dot(u, u); // always >= 0
                    const auto b = dot(u, v);
                    const auto c = dot(v, v); // always >= 0
                    const auto d = dot(u, w);
                    const auto e = dot(v, w);
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
                    glm::vec3 dP = w + sc * u - tc * v; // =  L1(sc) - L2(tc)
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
        }
        ImGui::EndChild();
    }
    ImGui::End();
    ImGui::PopStyleVar();

    return changed;
}

void
TreeVisualizer::SyncMatrices(
        const TreeSkeleton<SkeletonGrowthData, BranchGrowthData, InternodeGrowthData> &treeSkeleton) {
    static std::vector<glm::vec4> randomColors;
    if (randomColors.empty()) {
        for (int i = 0; i < 100; i++) {
            randomColors.emplace_back(glm::ballRand(1.0f), 1.0f);
        }
    }

    m_version = treeSkeleton.GetVersion();
    const auto &sortedBranchList = treeSkeleton.RefSortedFlowList();
    const auto &sortedInternodeList = treeSkeleton.RefSortedInternodeList();
    m_matrices.resize(sortedInternodeList.size() + 1);
    m_colors.resize(sortedInternodeList.size() + 1);
    std::vector<std::shared_future<void>> results;
    Jobs::ParallelFor(sortedInternodeList.size(), [&](unsigned i) {
        auto internodeHandle = sortedInternodeList[i];
        const auto &internode = treeSkeleton.PeekInternode(internodeHandle);
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

            const glm::vec3 selectedCenter = position + (internode.m_info.m_length * m_selectedInternodeLengthFactor) * direction;
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