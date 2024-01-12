//
// Created by lllll on 11/20/2022.
//

#include "TreeVisualizer.hpp"
#include "Utilities.hpp"
#include "Application.hpp"
#include "EcoSysLabLayer.hpp"
#include "ProfileConstraints.hpp"
using namespace EcoSysLab;

bool TreeVisualizer::ScreenCurvePruning(const std::function<void(NodeHandle)>& handler, std::vector<glm::vec2>& mousePositions,
	ShootSkeleton& skeleton, const GlobalTransform& globalTransform) {
	auto editorLayer = Application::GetLayer<EditorLayer>();
	auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
	const auto cameraRotation = editorLayer->GetSceneCameraRotation();
	const auto cameraPosition = editorLayer->GetSceneCameraPosition();
	const glm::vec3 cameraFront = cameraRotation * glm::vec3(0, 0, -1);
	const glm::vec3 cameraUp = cameraRotation * glm::vec3(0, 1, 0);
	glm::mat4 projectionView = ecoSysLabLayer->m_visualizationCamera->GetProjection() *
		glm::lookAt(cameraPosition, cameraPosition + cameraFront, cameraUp);

	const auto& sortedInternodeList = skeleton.RefSortedNodeList();
	bool changed = false;
	for (const auto& internodeHandle : sortedInternodeList) {
		if (internodeHandle == 0) continue;
		auto& internode = skeleton.RefNode(internodeHandle);
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
		for (int i = 0; i < mousePositions.size() - 1; i++) {
			auto& lineStart = mousePositions[i];
			auto& lineEnd = mousePositions[i + 1];
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
			handler(internodeHandle);
			changed = true;
		}

	}
	if (changed) {
		m_selectedInternodeHandle = -1;
		m_selectedInternodeHierarchyList.clear();
	}
	return changed;
}

bool TreeVisualizer::RayCastSelection(const std::shared_ptr<Camera>& cameraComponent,
	const glm::vec2& mousePosition,
	const ShootSkeleton& skeleton,
	const GlobalTransform& globalTransform) {
	const auto editorLayer = Application::GetLayer<EditorLayer>();
	bool changed = false;
#pragma region Ray selection
	NodeHandle currentFocusingNodeHandle = -1;
	std::mutex writeMutex;
	float minDistance = FLT_MAX;
	GlobalTransform cameraLtw;
	cameraLtw.m_value =
		glm::translate(
			editorLayer->GetSceneCameraPosition()) *
		glm::mat4_cast(
			editorLayer->GetSceneCameraRotation());
	const Ray cameraRay = cameraComponent->ScreenPointToRay(
		cameraLtw, mousePosition);
	const auto& sortedNodeList = skeleton.RefSortedNodeList();
	std::vector<std::shared_future<void>> results;
	Jobs::ParallelFor(sortedNodeList.size(), [&](unsigned i) {
		const auto& node = skeleton.PeekNode(sortedNodeList[i]);
		auto rotation = globalTransform.GetRotation() * node.m_info.m_globalRotation;
		glm::vec3 position = (globalTransform.m_value *
			glm::translate(node.m_info.m_globalPosition))[3];
		const auto direction = glm::normalize(rotation * glm::vec3(0, 0, -1));
		const glm::vec3 position2 =
			position + node.m_info.m_length * direction;
		const auto center =
			(position + position2) / 2.0f;
		auto radius = node.m_info.m_thickness;
		const auto height = glm::distance(position2,
			position);
		radius *= height / node.m_info.m_length;
		if (!cameraRay.Intersect(center,
			height / 2.0f) && !cameraRay.Intersect(center,
				radius)) {
			return;
		}
		const auto& dir = -cameraRay.m_direction;
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
		}
		else {
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
			currentFocusingNodeHandle = sortedNodeList[i];
		}
		}, results);
	for (auto& i : results) i.wait();
	if (currentFocusingNodeHandle != -1) {
		SetSelectedNode(skeleton, currentFocusingNodeHandle);
		changed = true;
#pragma endregion
	}
	return changed;
}

void TreeVisualizer::PeekNodeInspectionGui(
	const ShootSkeleton& skeleton,
	NodeHandle nodeHandle,
	const unsigned& hierarchyLevel) {
	const int index = m_selectedInternodeHierarchyList.size() - hierarchyLevel - 1;
	if (!m_selectedInternodeHierarchyList.empty() && index >= 0 &&
		index < m_selectedInternodeHierarchyList.size() &&
		m_selectedInternodeHierarchyList[index] == nodeHandle) {
		ImGui::SetNextItemOpen(true);
	}
	const bool opened = ImGui::TreeNodeEx(("Handle: " + std::to_string(nodeHandle)).c_str(),
		ImGuiTreeNodeFlags_NoTreePushOnOpen | ImGuiTreeNodeFlags_OpenOnArrow |
		ImGuiTreeNodeFlags_NoAutoOpenOnLog |
		(m_selectedInternodeHandle == nodeHandle ? ImGuiTreeNodeFlags_Framed
			: ImGuiTreeNodeFlags_FramePadding));
	if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
		SetSelectedNode(skeleton, nodeHandle);
	}
	if (opened) {
		ImGui::TreePush(std::to_string(nodeHandle).c_str());
		const auto& internode = skeleton.PeekNode(nodeHandle);
		const auto& internodeChildren = internode.RefChildHandles();
		for (const auto& child : internodeChildren) {
			PeekNodeInspectionGui(skeleton, child, hierarchyLevel + 1);
		}
		ImGui::TreePop();
	}
}


void TreeVisualizer::SetSelectedNode(const ShootSkeleton& skeleton, const NodeHandle nodeHandle) {
	if (nodeHandle != m_selectedInternodeHandle) {
		m_selectedInternodeHierarchyList.clear();
		if (nodeHandle < 0) {
			m_selectedInternodeHandle = -1;
		}
		else {
			m_selectedInternodeHandle = nodeHandle;
			auto walker = nodeHandle;
			while (walker != -1) {
				m_selectedInternodeHierarchyList.push_back(walker);
				const auto& internode = skeleton.PeekNode(walker);
				walker = internode.GetParentHandle();
			}
		}
	}
}

void TreeVisualizer::SyncMatrices(const ShootSkeleton& skeleton, const std::shared_ptr<ParticleInfoList>& particleInfoList) {

	const auto& sortedNodeList = skeleton.RefSortedNodeList();
	auto& matrices = particleInfoList->m_particleInfos;
	particleInfoList->SetPendingUpdate();
	matrices.resize(sortedNodeList.size());
	Jobs::ParallelFor(sortedNodeList.size(), [&](unsigned i) {
		auto nodeHandle = sortedNodeList[i];
		const auto& node = skeleton.PeekNode(nodeHandle);
		glm::vec3 position = node.m_info.m_globalPosition;
		const auto direction = node.m_info.m_globalDirection;
		auto rotation = glm::quatLookAt(
			direction, glm::vec3(direction.y, direction.z, direction.x));
		rotation *= glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f));
		const glm::mat4 rotationTransform = glm::mat4_cast(rotation);
		matrices[i].m_instanceMatrix.m_value =
			glm::translate(position + (node.m_info.m_length / 2.0f) * direction) *
			rotationTransform *
			glm::scale(glm::vec3(
				node.m_info.m_thickness * 2.0f,
				node.m_info.m_length,
				node.m_info.m_thickness * 2.0f));
		});
}

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
		SetSelectedNode(treeSkeleton, internodeHandle);
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
				m_checkpointIteration = treeModel.CurrentIteration();
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
	if (ImGui::TreeNodeEx("Checkpoints")) {
		if (ImGui::Button("Push Checkpoint"))
		{
			treeModel.Step();
			m_checkpointIteration = treeModel.CurrentIteration();
		}
		if (ImGui::SliderInt("Current checkpoint", &m_checkpointIteration, 0, treeModel.CurrentIteration())) {
			m_checkpointIteration = glm::clamp(m_checkpointIteration, 0, treeModel.CurrentIteration());
			m_selectedInternodeHandle = -1;
			m_selectedInternodeHierarchyList.clear();
			m_needUpdate = true;
		}
		if (m_checkpointIteration != treeModel.CurrentIteration() && ImGui::Button("Reverse")) {
			treeModel.Reverse(m_checkpointIteration);
			m_needUpdate = true;
		}
		if (ImGui::Button("Clear checkpoints")) {
			m_checkpointIteration = 0;
			treeModel.ClearHistory();
		}
		ImGui::TreePop();
	}
	if (ImGui::TreeNodeEx("Visualizer Settings")) {
		if (ImGui::Combo("Shoot Color mode",
			{ "Default", "Order", "Level", "Light Intensity", "Light Direction", "Growth Potential", "Apical control", "Desired growth rate", "IsMaxChild", "AllocatedVigor" },
			m_settings.m_shootVisualizationMode)) {
			m_needShootColorUpdate = true;
		}
		ImGui::DragInt("History Limit", &treeModel.m_historyLimit, 1, -1, 1024);

		if (ImGui::TreeNode("Shoot Color settings")) {
			switch (static_cast<ShootVisualizerMode>(m_settings.m_shootVisualizationMode)) {
			case ShootVisualizerMode::LightIntensity:
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

		ImGui::Checkbox("Visualization", &m_visualization);
		ImGui::Checkbox("Front Profile", &m_frontProfileGui);
		ImGui::Checkbox("Back Profile", &m_backProfileGui);
		ImGui::Checkbox("Tree Hierarchy", &m_treeHierarchyGui);

		if (m_visualization) {
			const auto& treeSkeleton = treeModel.PeekShootSkeleton(m_checkpointIteration);
			const auto editorLayer = Application::GetLayer<EditorLayer>();
			const auto& sortedBranchList = treeSkeleton.RefSortedFlowList();
			const auto& sortedInternodeList = treeSkeleton.RefSortedNodeList();
			ImGui::Text("Internode count: %d", sortedInternodeList.size());
			ImGui::Text("Shoot stem count: %d", sortedBranchList.size());
		}

		ImGui::TreePop();
	}
	if (ImGui::TreeNodeEx("Inspection")) {
		if (m_selectedInternodeHandle >= 0) {
			if (m_checkpointIteration == treeModel.CurrentIteration()) {
				InspectInternode(treeModel.RefShootSkeleton(), m_selectedInternodeHandle);
			}
			else {
				PeekInternode(treeModel.PeekShootSkeleton(m_checkpointIteration), m_selectedInternodeHandle);
			}
		}

		if (m_treeHierarchyGui) {
			if (ImGui::TreeNodeEx("Tree Hierarchy")) {
				bool deleted = false;
				auto tempSelection = m_selectedInternodeHandle;
				if (m_checkpointIteration == treeModel.CurrentIteration()) {
					if (DrawInternodeInspectionGui(treeModel, 0, deleted, 0)) {
						m_needUpdate = true;
						updated = true;
					}
				}
				else
					PeekNodeInspectionGui(treeModel.PeekShootSkeleton(m_checkpointIteration), 0, 0);
				m_selectedInternodeHierarchyList.clear();
				ImGui::TreePop();
			}
		}
		ImGui::TreePop();
	}
	return updated;
}

void TreeVisualizer::Visualize(TreeModel& treeModel, const GlobalTransform& globalTransform) {
	const auto& treeSkeleton = treeModel.PeekShootSkeleton(m_checkpointIteration);
	if (m_visualization) {
		const auto editorLayer = Application::GetLayer<EditorLayer>();
		const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
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

			editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER"), ecoSysLabLayer->m_visualizationCamera,
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
				const auto matrix = globalTransform.m_value * glm::translate(selectedCenter) *
					rotationTransform *
					glm::scale(glm::vec3(
						2.0f * node.m_info.m_thickness + 0.01f,
						node.m_info.m_length / 5.0f,
						2.0f * node.m_info.m_thickness + 0.01f));
				const auto color = glm::vec4(1.0f);
				editorLayer->DrawGizmoMesh(Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER"), ecoSysLabLayer->m_visualizationCamera, color, matrix, 1, gizmoSettings);
			}
		}

		if (m_checkpointIteration == treeModel.CurrentIteration())
		{
			static bool showGrid = false;
			if (m_frontProfileGui) {
				const std::string frontTag = "Front Profile";
				if (ImGui::Begin(frontTag.c_str()))
				{
					if (m_selectedInternodeHandle != -1) {
						auto& node = treeModel.RefShootSkeleton().RefNode(m_selectedInternodeHandle);

						glm::vec2 mousePosition{};
						static bool lastFrameClicked = false;
						bool mouseDown = false;
						static bool addAttractor = false;
						ImGui::Checkbox("Attractor", &addAttractor);
						if (ImGui::Button("Clear boundaries"))
						{
							node.m_data.m_profileConstraints.m_boundaries.clear();
							node.m_data.m_boundariesUpdated = true;
						}
						ImGui::SameLine();
						if (ImGui::Button("Clear attractors"))
						{
							node.m_data.m_profileConstraints.m_attractors.clear();
							node.m_data.m_boundariesUpdated = true;
						}
						ImGui::SameLine();
						ImGui::Checkbox("Show Grid", &showGrid);
						if (node.GetParentHandle() != -1)
						{
							const auto& parentNode = treeModel.RefShootSkeleton().RefNode(node.GetParentHandle());
							if (!parentNode.m_data.m_profileConstraints.m_boundaries.empty())
							{
								ImGui::SameLine();
								if (ImGui::Button("Copy parent boundaries"))
								{
									node.m_data.m_profileConstraints = parentNode.m_data.m_profileConstraints;
									node.m_data.m_boundariesUpdated = true;
								}
							}
						}
						node.m_data.m_frontProfile.OnInspect([&](const glm::vec2 position)
							{
								mouseDown = true;
								mousePosition = position;
							},
							[&](const ImVec2 origin, const float zoomFactor, ImDrawList* drawList) {
								node.m_data.m_frontProfile.RenderEdges(origin, zoomFactor, drawList, IM_COL32(0.0f, 0.0f, 128.0f, 128.0f), 1.0f);
								node.m_data.m_frontProfile.RenderBoundary(origin, zoomFactor, drawList, IM_COL32(255.f, 255.f, 255.0f, 255.0f), 4.0f);

								if (node.GetParentHandle() != -1)
								{
									const auto& parentNode = treeModel.RefShootSkeleton().RefNode(node.GetParentHandle());
									if (!parentNode.m_data.m_profileConstraints.m_boundaries.empty())
									{
										for (const auto& parentBoundary : parentNode.m_data.m_profileConstraints.m_boundaries)
										{
											parentBoundary.RenderBoundary(origin, zoomFactor, drawList, IM_COL32(128.0f, 0.0f, 0, 128.0f), 4.0f);
										}
										for (const auto& parentAttractor : parentNode.m_data.m_profileConstraints.m_attractors)
										{
											parentAttractor.RenderAttractor(origin, zoomFactor, drawList, IM_COL32(0.0f, 128.0f, 0, 128.0f), 4.0f);
										}
									}
									
								}
								for (const auto& boundary : node.m_data.m_profileConstraints.m_boundaries)
								{
									boundary.RenderBoundary(origin, zoomFactor, drawList, IM_COL32(255.0f, 0.0f, 0, 255.0f), 2.0f);
								}

								for (const auto& attractor : node.m_data.m_profileConstraints.m_attractors)
								{
									attractor.RenderAttractor(origin, zoomFactor, drawList, IM_COL32(0.0f, 255.0f, 0, 255.0f), 2.0f);
								}
						},
							showGrid);
						auto& profileBoundaries = node.m_data.m_profileConstraints;
						static glm::vec2 attractorStartMousePosition;
						if (lastFrameClicked)
						{
							if (mouseDown)
							{
								if (!addAttractor) {
									//Continue recording.
									if (glm::distance(mousePosition, profileBoundaries.m_boundaries.back().m_points.back()) > 1.0f) profileBoundaries.m_boundaries.back().m_points.emplace_back(mousePosition);
								}
								else
								{
									auto& attractorPoints = profileBoundaries.m_attractors.back().m_attractorPoints;
									if(attractorPoints.empty())
									{
										if(glm::distance(attractorStartMousePosition, mousePosition) > 1.0f)
										{
											attractorPoints.emplace_back(attractorStartMousePosition, mousePosition);
										}
									}else if(glm::distance(mousePosition, attractorPoints.back().second) > 1.0f)
									{
										attractorPoints.emplace_back(attractorPoints.back().second, mousePosition);
									}
								}
							}
							else if (!profileBoundaries.m_boundaries.empty())
							{
								if (!addAttractor)
								{
									//Stop and check boundary.
									if (!profileBoundaries.Valid(profileBoundaries.m_boundaries.size() - 1))
									{
										profileBoundaries.m_boundaries.pop_back();
									}
									else
									{
										profileBoundaries.m_boundaries.back().CalculateCenter();
										node.m_data.m_boundariesUpdated = true;
									}
								}
								else
								{
									//Stop and check attractors.
									node.m_data.m_boundariesUpdated = true;
								}
							}
						}
						else if (mouseDown) {
							//Start recording.
							if (!addAttractor) {
								node.m_data.m_profileConstraints.m_boundaries.emplace_back();
								node.m_data.m_profileConstraints.m_boundaries.back().m_points.push_back(mousePosition);
							}
							else
							{
								node.m_data.m_profileConstraints.m_attractors.emplace_back();
								attractorStartMousePosition = mousePosition;
							}
						}
						lastFrameClicked = mouseDown;
					}
					else
					{
						ImGui::Text("Select an internode to show its front profile!");
					}
				}
				ImGui::End();
			}
			if (m_backProfileGui) {
				const std::string backTag = "Back Profile";
				if (ImGui::Begin(backTag.c_str()))
				{
					if (m_selectedInternodeHandle != -1) {
						auto& node = treeModel.RefShootSkeleton().RefNode(m_selectedInternodeHandle);
						ImGui::Checkbox("Show Grid", &showGrid);
						if (node.GetParentHandle() != -1)
						{
							const auto& parentNode = treeModel.RefShootSkeleton().RefNode(node.GetParentHandle());
							if (!parentNode.m_data.m_profileConstraints.m_boundaries.empty())
							{
								ImGui::SameLine();
								if (ImGui::Button("Copy parent boundaries"))
								{
									node.m_data.m_profileConstraints = parentNode.m_data.m_profileConstraints;
									node.m_data.m_boundariesUpdated = true;
								}
							}
						}
						node.m_data.m_backProfile.OnInspect([&](const glm::vec2 position)
							{},
							[&](const ImVec2 origin, const float zoomFactor, ImDrawList* drawList)
							{
								node.m_data.m_backProfile.RenderEdges(origin, zoomFactor, drawList, IM_COL32(0.0f, 0.0f, 128.0f, 128.0f), 1.0f);
								node.m_data.m_backProfile.RenderBoundary(origin, zoomFactor, drawList, IM_COL32(255.f, 255.f, 255.0f, 255.0f), 4.0f);


								if (node.GetParentHandle() != -1)
								{
									const auto& parentNode = treeModel.RefShootSkeleton().RefNode(node.GetParentHandle());
									if (!parentNode.m_data.m_profileConstraints.m_boundaries.empty())
									{
										for (const auto& parentBoundary : parentNode.m_data.m_profileConstraints.m_boundaries)
										{
											parentBoundary.RenderBoundary(origin, zoomFactor, drawList, IM_COL32(128.0f, 0.0f, 0, 128.0f), 4.0f);
										}
										for (const auto& parentAttractor : parentNode.m_data.m_profileConstraints.m_attractors)
										{
											parentAttractor.RenderAttractor(origin, zoomFactor, drawList, IM_COL32(0.0f, 128.0f, 0, 128.0f), 4.0f);
										}
									}

								}
								for (const auto& boundary : node.m_data.m_profileConstraints.m_boundaries)
								{
									boundary.RenderBoundary(origin, zoomFactor, drawList, IM_COL32(255.0f, 0.0f, 0, 255.0f), 2.0f);
								}

								for (const auto& attractor : node.m_data.m_profileConstraints.m_attractors)
								{
									attractor.RenderAttractor(origin, zoomFactor, drawList, IM_COL32(0.0f, 255.0f, 0, 255.0f), 2.0f);
								}
							},
							showGrid);
					}
					else
					{
						ImGui::Text("Select an internode to show its back profile!");
					}
				}
				ImGui::End();
			}
		}
	}
}

bool
TreeVisualizer::InspectInternode(
	ShootSkeleton& shootSkeleton,
	NodeHandle internodeHandle) {
	bool changed = false;

	const auto& internode = shootSkeleton.RefNode(internodeHandle);
	if (ImGui::TreeNode("Internode info")) {
		ImGui::Checkbox("Is max child", (bool*)&internode.m_data.m_maxChild);
		ImGui::Text("Thickness: %.3f", internode.m_info.m_thickness);
		ImGui::Text("Length: %.3f", internode.m_info.m_length);
		ImGui::InputFloat3("Position", (float*)&internode.m_info.m_globalPosition.x, "%.3f",
			ImGuiInputTextFlags_ReadOnly);
		auto globalRotationAngle = glm::eulerAngles(internode.m_info.m_globalRotation);
		ImGui::InputFloat3("Global rotation", (float*)&globalRotationAngle.x, "%.3f",
			ImGuiInputTextFlags_ReadOnly);
		auto localRotationAngle = glm::eulerAngles(internode.m_data.m_desiredLocalRotation);
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

		ImGui::InputFloat("Light Intensity", (float*)&internodeData.m_lightIntensity, 1, 100, "%.3f",
			ImGuiInputTextFlags_ReadOnly);
		ImGui::InputFloat3("Light direction", (float*)&internodeData.m_lightDirection.x, "%.3f",
			ImGuiInputTextFlags_ReadOnly);
		ImGui::InputFloat("Pipe resistance", (float*)&internodeData.m_pipeResistance, 1, 100, "%.3f",
			ImGuiInputTextFlags_ReadOnly);

		ImGui::InputFloat("Growth potential", (float*)&internodeData.m_growthPotential, 1, 100, "%.3f",
			ImGuiInputTextFlags_ReadOnly);
		ImGui::InputFloat("Apical control", (float*)&internodeData.m_apicalControl, 1, 100, "%.3f",
			ImGuiInputTextFlags_ReadOnly);
		ImGui::InputFloat("Desired growth rate", (float*)&internodeData.m_desiredGrowthRate, 1, 100, "%.3f",
			ImGuiInputTextFlags_ReadOnly);
		ImGui::InputFloat("Growth rate", (float*)&internodeData.m_growthRate, 1, 100, "%.3f",
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
		ImGui::Checkbox("Is max child", (bool*)&internode.m_data.m_maxChild);
		ImGui::Text("Thickness: %.3f", internode.m_info.m_thickness);
		ImGui::Text("Length: %.3f", internode.m_info.m_length);
		ImGui::InputFloat3("Position", (float*)&internode.m_info.m_globalPosition.x, "%.3f",
			ImGuiInputTextFlags_ReadOnly);
		auto globalRotationAngle = glm::eulerAngles(internode.m_info.m_globalRotation);
		ImGui::InputFloat3("Global rotation", (float*)&globalRotationAngle.x, "%.3f",
			ImGuiInputTextFlags_ReadOnly);
		auto localRotationAngle = glm::eulerAngles(internode.m_data.m_desiredLocalRotation);
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

void TreeVisualizer::Reset(TreeModel& treeModel) {
	m_selectedInternodeHandle = -1;
	m_selectedInternodeHierarchyList.clear();
	m_checkpointIteration = treeModel.CurrentIteration();
	m_internodeMatrices->m_particleInfos.clear();
	m_internodeMatrices->SetPendingUpdate();
	m_needUpdate = true;
}

void TreeVisualizer::Clear() {
	m_selectedInternodeHandle = -1;
	m_selectedInternodeHierarchyList.clear();
	m_checkpointIteration = 0;
	m_internodeMatrices->m_particleInfos.clear();
	m_internodeMatrices->SetPendingUpdate();
}



bool TreeVisualizer::Initialized() const
{
	return m_initialized;
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
		case ShootVisualizerMode::Default:
			matrices[i].m_instanceColor = m_randomColors[nodeHandle % m_randomColors.size()];
			break;
		case ShootVisualizerMode::Order:
			matrices[i].m_instanceColor = m_randomColors[node.m_data.m_order];
			break;
		case ShootVisualizerMode::Level:
			matrices[i].m_instanceColor = m_randomColors[node.m_data.m_level];
			break;
		case ShootVisualizerMode::LightIntensity:
			matrices[i].m_instanceColor = glm::vec4(
				glm::clamp(glm::pow(node.m_data.m_lightIntensity, m_settings.m_shootColorMultiplier), 0.0f, 1.f));
			break;
		case ShootVisualizerMode::LightDirection:
			matrices[i].m_instanceColor = glm::vec4(glm::vec3(glm::clamp(node.m_data.m_lightDirection, 0.0f, 1.f)),
				1.0f);
			break;
		case ShootVisualizerMode::IsMaxChild:
			matrices[i].m_instanceColor = glm::vec4(glm::vec3(node.m_data.m_maxChild ? 1.0f : 0.0f), 1.0f);
			break;
		case ShootVisualizerMode::GrowthPotential:
			matrices[i].m_instanceColor = glm::vec4(
				glm::clamp(glm::pow(node.m_data.m_growthPotential, m_settings.m_shootColorMultiplier), 0.0f, 1.f));
			break;
		case ShootVisualizerMode::ApicalControl:
			matrices[i].m_instanceColor = glm::vec4(
				glm::clamp(glm::pow(node.m_data.m_apicalControl, m_settings.m_shootColorMultiplier), 0.0f, 1.f));
			break;
		case ShootVisualizerMode::DesiredGrowthRate:
			matrices[i].m_instanceColor = glm::vec4(
				glm::clamp(glm::pow(node.m_data.m_desiredGrowthRate, m_settings.m_shootColorMultiplier), 0.0f, 1.f));
			break;
		default:
			matrices[i].m_instanceColor = m_randomColors[node.m_data.m_order];
			break;
		}
		matrices[i].m_instanceColor.a = 1.0f;
		if (selectedNodeHandle != -1) matrices[i].m_instanceColor.a = 1.0f;
		}
	);
}
