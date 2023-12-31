//
// Created by lllll on 11/20/2022.
//

#include "TreeVisualizer.hpp"
#include "Utilities.hpp"
#include "Application.hpp"
#include "EcoSysLabLayer.hpp"
using namespace EcoSysLab;

bool TreeVisualizer::DrawInternodeInspectionGui(
	TreeModel& treeModel,
	NodeHandle internodeHandle,
	bool& deleted,
	const unsigned& hierarchyLevel) {
	auto& treeSkeleton = treeModel.RefShootSkeleton();
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
		ImGui::TreePush(std::to_string(internodeHandle).c_str());
		const auto& internodeChildren = treeSkeleton.RefNode(internodeHandle).RefChildHandles();
		for (const auto& child : internodeChildren) {
			bool childDeleted = false;
			DrawInternodeInspectionGui(treeModel, child, childDeleted, hierarchyLevel + 1);
			if (childDeleted) {
				treeModel.Step();
				treeModel.PruneInternode(child);

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
void TreeVisualizer::ClearSelections()
{
	m_selectedInternodeHandle = -1;
}
bool
TreeVisualizer::OnInspect(
	TreeModel& treeModel) {
	bool updated = false;
	if (ImGui::Combo("Shoot Color mode",
		{ "Default", "GrowthPotential", "LightDirection", "IsMaxChild", "AllocatedVigor" },
		m_settings.m_shootVisualizationMode)) {
		m_needShootColorUpdate = true;
	}

	if (ImGui::TreeNode("Shoot Color settings")) {
		switch (static_cast<ShootVisualizerMode>(m_settings.m_shootVisualizationMode)) {
		case ShootVisualizerMode::GrowthPotential:
			ImGui::DragFloat("Light intensity multiplier", &m_settings.m_shootColorMultiplier, 0.001f);
			m_needShootColorUpdate = true;
			break;
		case ShootVisualizerMode::AllocatedVigor:
			ImGui::DragFloat("Vigor multiplier", &m_settings.m_shootColorMultiplier, 0.001f);
			m_needShootColorUpdate = true;
			break;
		default:
			break;
		}
		ImGui::TreePop();
	}
	
	if (treeModel.CurrentIteration() > 0) {
		if (ImGui::TreeNodeEx("History", ImGuiTreeNodeFlags_DefaultOpen)) {
			ImGui::DragInt("History Limit", &treeModel.m_historyLimit, 1, -1, 1024);
			if (ImGui::SliderInt("Iteration", &m_iteration, 0, treeModel.CurrentIteration())) {
				m_iteration = glm::clamp(m_iteration, 0, treeModel.CurrentIteration());
				m_selectedInternodeHandle = -1;
				m_selectedInternodeHierarchyList.clear();
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
		ImGui::Checkbox("Hexagon profile", &m_hexagonProfileGui);
		ImGui::Checkbox("Tree Hierarchy", &m_treeHierarchyGui);

		if (m_visualization) {
			const auto& treeSkeleton = treeModel.PeekShootSkeleton(m_iteration);
			const auto editorLayer = Application::GetLayer<EditorLayer>();
			const auto& sortedBranchList = treeSkeleton.RefSortedFlowList();
			const auto& sortedInternodeList = treeSkeleton.RefSortedNodeList();
			ImGui::Text("Internode count: %d", sortedInternodeList.size());
			ImGui::Text("Shoot stem count: %d", sortedBranchList.size());

			static bool enableStroke = false;
			if (ImGui::Checkbox("Enable stroke", &enableStroke)) {
				if (enableStroke) {
					m_mode = PruningMode::Stroke;
				}
				else {
					m_mode = PruningMode::Empty;
				}
				m_storedMousePositions.clear();
			}
		}

		ImGui::TreePop();
	}
	if (m_selectedInternodeHandle >= 0) {
		if (m_iteration == treeModel.CurrentIteration()) {
			InspectInternode(treeModel.RefShootSkeleton(), m_selectedInternodeHandle);
		}
		else {
			PeekInternode(treeModel.PeekShootSkeleton(m_iteration), m_selectedInternodeHandle);
		}
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
			}
			else
				PeekNodeInspectionGui(treeModel.PeekShootSkeleton(m_iteration), 0, m_selectedInternodeHandle,
					m_selectedInternodeHierarchyList, 0);
			m_selectedInternodeHierarchyList.clear();
			ImGui::TreePop();
		}

	}
	
	return updated;
}

bool TreeVisualizer::Visualize(TreeModel& treeModel, const GlobalTransform& globalTransform) {
	bool updated = false;
	const auto& treeSkeleton = treeModel.PeekShootSkeleton(m_iteration);
	if (m_visualization) {
		const auto editorLayer = Application::GetLayer<EditorLayer>();
		if (editorLayer->SceneCameraWindowFocused()) {
			switch (m_mode) {
			case PruningMode::Empty: {
				if (editorLayer->GetKey(GLFW_MOUSE_BUTTON_LEFT) == KeyActionType::Press) {
					if (RayCastSelection(editorLayer->GetSceneCamera(), editorLayer->GetMouseSceneCameraPosition(), treeSkeleton, globalTransform, m_selectedInternodeHandle,
						m_selectedInternodeHierarchyList, m_selectedInternodeLengthFactor)) {
						m_needUpdate = true;
						updated = true;
					}
				}
				if (m_iteration == treeModel.CurrentIteration() &&
					editorLayer->GetKey(GLFW_KEY_DELETE) == KeyActionType::Press) {
					if (m_selectedInternodeHandle > 0) {
						treeModel.Step();
						auto& skeleton = treeModel.RefShootSkeleton();
						auto& pruningInternode = skeleton.RefNode(m_selectedInternodeHandle);
						auto childHandles = pruningInternode.RefChildHandles();
						for (const auto& childHandle : childHandles) {
							treeModel.PruneInternode(childHandle);
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
				}
			}
								   break;
			case PruningMode::Stroke: {
				if (m_iteration == treeModel.CurrentIteration()) {
					if (editorLayer->GetKey(GLFW_MOUSE_BUTTON_LEFT) == KeyActionType::Hold) {
						glm::vec2 mousePosition = editorLayer->GetMouseSceneCameraPosition();
						const float halfX = editorLayer->GetSceneCamera()->GetSize().x / 2.0f;
						const float halfY = editorLayer->GetSceneCamera()->GetSize().y / 2.0f;
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
							bool changed = ScreenCurvePruning(
								[&](NodeHandle nodeHandle) { treeModel.PruneInternode(nodeHandle); }, skeleton,
								globalTransform, m_selectedInternodeHandle, m_selectedInternodeHierarchyList);
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
		}

		if (m_needUpdate) {
			SyncMatrices(treeSkeleton, m_internodeMatrices);
			SyncColors(treeSkeleton, m_selectedInternodeHandle);
			m_needUpdate = false;
		}
		else {
			if (m_needShootColorUpdate) {
				SyncColors(treeSkeleton, m_selectedInternodeHandle);
				m_needShootColorUpdate = false;
			}
		}
		GizmoSettings gizmoSettings;
		gizmoSettings.m_drawSettings.m_blending = true;
		gizmoSettings.m_depthTest = true;
		gizmoSettings.m_depthWrite = true;
		if (!m_internodeMatrices->m_particleInfos.empty()) {

			editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER"), editorLayer->GetSceneCamera(),
				m_internodeMatrices,
				globalTransform.m_value, 1.0f, gizmoSettings);
			if (m_selectedInternodeHandle != -1)
			{
				const auto& node = treeSkeleton.PeekNode(m_selectedInternodeHandle);
				glm::vec3 position = node.m_info.m_globalPosition;
				const auto direction = node.m_info.m_globalDirection;
				auto rotation = glm::quatLookAt(
					direction, glm::vec3(direction.y, direction.z, direction.x));
				rotation *= glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f));
				const glm::mat4 rotationTransform = glm::mat4_cast(rotation);
				const glm::vec3 selectedCenter =
					position + node.m_info.m_length * m_selectedInternodeLengthFactor * direction;
				auto matrix = globalTransform.m_value * glm::translate(selectedCenter) *
					rotationTransform *
					glm::scale(glm::vec3(
						2.0f * node.m_info.m_thickness + 0.01f,
						node.m_info.m_length / 5.0f,
						2.0f * node.m_info.m_thickness + 0.01f));
				auto color = glm::vec4(1.0f);
				editorLayer->DrawGizmoCylinder(color, matrix, 1, gizmoSettings);
			}
		}
	}
	return updated;
}

bool
TreeVisualizer::InspectInternode(
	ShootSkeleton& shootSkeleton,
	NodeHandle internodeHandle) {
	bool changed = false;

	const auto& internode = shootSkeleton.RefNode(internodeHandle);
	if (ImGui::TreeNode("Internode info")) {
		ImGui::Checkbox("Is max child", (bool*)&internode.m_data.m_isMaxChild);
		ImGui::Text("Thickness: %.3f", internode.m_info.m_thickness);
		ImGui::Text("Length: %.3f", internode.m_info.m_length);
		ImGui::InputFloat3("Position", (float*)&internode.m_info.m_globalPosition.x, "%.3f",
			ImGuiInputTextFlags_ReadOnly);
		auto globalRotationAngle = glm::eulerAngles(internode.m_info.m_globalRotation);
		ImGui::InputFloat3("Global rotation", (float*)&globalRotationAngle.x, "%.3f",
			ImGuiInputTextFlags_ReadOnly);
		auto localRotationAngle = glm::eulerAngles(internode.m_data.m_localRotation);
		ImGui::InputFloat3("Local rotation", (float*)&localRotationAngle.x, "%.3f",
			ImGuiInputTextFlags_ReadOnly);
		auto& internodeData = internode.m_data;
		ImGui::InputFloat("Start Age", (float*)&internodeData.m_startAge, 1, 100, "%.3f",
			ImGuiInputTextFlags_ReadOnly);
		ImGui::InputFloat("Distance to end", (float*)&internode.m_info.m_endDistance, 1, 100,
			"%.3f",
			ImGuiInputTextFlags_ReadOnly);
		ImGui::InputFloat("Descendent biomass", (float*)&internodeData.m_descendentTotalBiomass, 1, 100, "%.3f",
			ImGuiInputTextFlags_ReadOnly);
		ImGui::InputFloat("Biomass", (float*)&internodeData.m_biomass, 1, 100, "%.3f",
			ImGuiInputTextFlags_ReadOnly);

		ImGui::InputFloat("Root distance", (float*)&internode.m_info.m_rootDistance, 1, 100, "%.3f",
			ImGuiInputTextFlags_ReadOnly);
		ImGui::InputFloat3("Light dir", (float*)&internodeData.m_lightDirection.x, "%.3f",
			ImGuiInputTextFlags_ReadOnly);
		ImGui::InputFloat("Growth Potential", (float*)&internodeData.m_lightIntensity, 1, 100, "%.3f",
			ImGuiInputTextFlags_ReadOnly);

		if (ImGui::DragFloat("Sagging", (float*)&internodeData.m_sagging)) {
			changed = true;
		}
		if (ImGui::DragFloat("Extra mass", (float*)&internodeData.m_extraMass)) {
			changed = true;
		}


		if (ImGui::TreeNodeEx("Buds", ImGuiTreeNodeFlags_DefaultOpen)) {
			int index = 1;
			for (auto& bud : internodeData.m_buds) {
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
		const auto& flow = shootSkeleton.PeekFlow(internode.GetFlowHandle());
		ImGui::Text("Child flow size: %d", flow.RefChildHandles().size());
		ImGui::Text("Internode size: %d", flow.RefNodeHandles().size());
		if (ImGui::TreeNode("Internodes")) {
			int i = 0;
			for (const auto& chainedInternodeHandle : flow.RefNodeHandles()) {
				ImGui::Text("No.%d: Handle: %d", i, chainedInternodeHandle);
				i++;
			}
			ImGui::TreePop();
		}
		ImGui::TreePop();
	}
	return changed;
}

void
TreeVisualizer::PeekInternode(const ShootSkeleton& shootSkeleton, NodeHandle internodeHandle) const {
	const auto& internode = shootSkeleton.PeekNode(internodeHandle);
	if (ImGui::TreeNode("Internode info")) {
		ImGui::Checkbox("Is max child", (bool*)&internode.m_data.m_isMaxChild);
		ImGui::Text("Thickness: %.3f", internode.m_info.m_thickness);
		ImGui::Text("Length: %.3f", internode.m_info.m_length);
		ImGui::InputFloat3("Position", (float*)&internode.m_info.m_globalPosition.x, "%.3f",
			ImGuiInputTextFlags_ReadOnly);
		auto globalRotationAngle = glm::eulerAngles(internode.m_info.m_globalRotation);
		ImGui::InputFloat3("Global rotation", (float*)&globalRotationAngle.x, "%.3f",
			ImGuiInputTextFlags_ReadOnly);
		auto localRotationAngle = glm::eulerAngles(internode.m_data.m_localRotation);
		ImGui::InputFloat3("Local rotation", (float*)&localRotationAngle.x, "%.3f",
			ImGuiInputTextFlags_ReadOnly);
		auto& internodeData = internode.m_data;
		ImGui::InputInt("Start Age", (int*)&internodeData.m_startAge, 1, 100, ImGuiInputTextFlags_ReadOnly);
		ImGui::InputFloat("Sagging", (float*)&internodeData.m_sagging, 1, 100, "%.3f",
			ImGuiInputTextFlags_ReadOnly);
		ImGui::InputFloat("Distance to end", (float*)&internode.m_info.m_endDistance, 1, 100,
			"%.3f",
			ImGuiInputTextFlags_ReadOnly);
		ImGui::InputFloat("Descendent biomass", (float*)&internodeData.m_descendentTotalBiomass, 1, 100, "%.3f",
			ImGuiInputTextFlags_ReadOnly);
		ImGui::InputFloat("Biomass", (float*)&internodeData.m_biomass, 1, 100, "%.3f",
			ImGuiInputTextFlags_ReadOnly);
		ImGui::InputFloat("Root distance", (float*)&internode.m_info.m_rootDistance, 1, 100, "%.3f",
			ImGuiInputTextFlags_ReadOnly);
		ImGui::InputFloat3("Light dir", (float*)&internodeData.m_lightDirection.x, "%.3f",
			ImGuiInputTextFlags_ReadOnly);
		ImGui::InputFloat("Growth Potential", (float*)&internodeData.m_lightIntensity, 1, 100, "%.3f",
			ImGuiInputTextFlags_ReadOnly);

		if (ImGui::TreeNodeEx("Buds")) {
			int index = 1;
			for (auto& bud : internodeData.m_buds) {
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
		const auto& flow = shootSkeleton.PeekFlow(internode.GetFlowHandle());
		ImGui::Text("Child stem size: %d", flow.RefChildHandles().size());
		ImGui::Text("Internode size: %d", flow.RefNodeHandles().size());
		if (ImGui::TreeNode("Internodes")) {
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

void TreeVisualizer::Reset(
	TreeModel& treeModel) {
	m_selectedInternodeHandle = -1;
	m_selectedInternodeHierarchyList.clear();
	m_iteration = treeModel.CurrentIteration();
	m_internodeMatrices->m_particleInfos.clear();
	m_internodeMatrices->SetPendingUpdate();
	m_needUpdate = true;
}

void TreeVisualizer::Clear() {
	m_selectedInternodeHandle = -1;
	m_selectedInternodeHierarchyList.clear();
	m_iteration = 0;
	m_internodeMatrices->m_particleInfos.clear();
	m_internodeMatrices->SetPendingUpdate();
}

bool TreeVisualizer::Initialized() const
{
	return m_initialized;
}

NodeHandle TreeVisualizer::GetSelectedInternodeHandle() const
{
	return m_selectedInternodeHandle;
}

void TreeVisualizer::Initialize()
{
	m_internodeMatrices = std::make_shared<ParticleInfoList>();
}

void TreeVisualizer::SyncColors(const ShootSkeleton& shootSkeleton, const NodeHandle selectedNodeHandle) {
	if (m_randomColors.empty()) {
		for (int i = 0; i < 1000; i++) {
			m_randomColors.emplace_back(glm::abs(glm::ballRand(1.0f)), 1.0f);
		}
	}

	const auto& sortedNodeList = shootSkeleton.RefSortedNodeList();
	auto& matrices = m_internodeMatrices->m_particleInfos;
	m_internodeMatrices->SetPendingUpdate();
	matrices.resize(sortedNodeList.size());
	Jobs::ParallelFor(sortedNodeList.size(), [&](unsigned i) {
		const auto nodeHandle = sortedNodeList[i];
		const auto& node = shootSkeleton.PeekNode(nodeHandle);
		switch (static_cast<ShootVisualizerMode>(m_settings.m_shootVisualizationMode)) {
		case ShootVisualizerMode::GrowthPotential:
			matrices[i].m_instanceColor = glm::vec4(
				glm::clamp(node.m_data.m_lightIntensity * m_settings.m_shootColorMultiplier, 0.0f, 1.f));
			break;
		case ShootVisualizerMode::LightDirection:
			matrices[i].m_instanceColor = glm::vec4(glm::vec3(glm::clamp(node.m_data.m_lightDirection, 0.0f, 1.f)),
				1.0f);
			break;
		case ShootVisualizerMode::IsMaxChild:
			matrices[i].m_instanceColor = glm::vec4(glm::vec3(node.m_data.m_isMaxChild ? 1.0f : 0.0f), 1.0f);
			break;
		case ShootVisualizerMode::AllocatedVigor:
			break;
		default:
			matrices[i].m_instanceColor = m_randomColors[node.m_data.m_order];
			break;
		}
		matrices[i].m_instanceColor.a = 1.0f;
		if (selectedNodeHandle != -1) matrices[i].m_instanceColor.a = 0.75f;
		}
	);
}
