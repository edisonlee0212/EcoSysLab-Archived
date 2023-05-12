#include <unordered_set>
#include "TreePointCloud.hpp"
#include "Graphics.hpp"
#include "EcoSysLabLayer.hpp"
#include "rapidcsv.h"

using namespace EcoSysLab;

void TreePointCloud::FindPoints(const glm::vec3 &position, VoxelGrid<std::vector<PointCloudVoxel>> &pointVoxelGrid,
																float radius,
																const std::function<void(const PointCloudVoxel &voxel)> &func) const {
		pointVoxelGrid.ForEach(position, radius * 2.0f, [&](const std::vector<PointCloudVoxel> &voxels) {
			for (const auto &voxel: voxels) {
					if (glm::distance(position, voxel.m_position) > radius) continue;
					func(voxel);
			}
		});
}

void TreePointCloud::ImportGraph(const std::filesystem::path &path, float scaleFactor) {
		if (!std::filesystem::exists(path)) {
				UNIENGINE_ERROR("Not exist!");
				return;
		}
		try {
				std::ifstream stream(path.string());
				std::stringstream stringStream;
				stringStream << stream.rdbuf();
				YAML::Node in = YAML::Load(stringStream.str());

				const auto &tree = in["Tree"];
				const auto &branches = tree["Branches"]["Content"];
				const auto &scatterPoints = tree["Scatter Points"]["Content"];

				m_min = glm::vec3(FLT_MAX);
				m_max = glm::vec3(FLT_MIN);

				m_points.resize(scatterPoints.size());

				for (int i = 0; i < scatterPoints.size(); i++) {
						auto &point = m_points[i];
						point.m_position = scatterPoints[i].as<glm::vec3>();
						m_min = glm::min(m_min, point.m_position);
						m_max = glm::max(m_max, point.m_position);
						point.m_handle = i;
						point.m_neighbors.clear();
				}
				m_branches.resize(branches.size());
				for (int i = 0; i < branches.size(); i++) {
						const auto &inBranch = branches[i];
						auto &branch = m_branches[i];
						branch.m_bezierCurve.m_p0 = inBranch["Start Pos"].as<glm::vec3>();
						branch.m_bezierCurve.m_p3 = inBranch["End Pos"].as<glm::vec3>();
						auto cPLength = glm::distance(branch.m_bezierCurve.m_p0, branch.m_bezierCurve.m_p3) * 0.3f;
						branch.m_bezierCurve.m_p1 =
										glm::normalize(inBranch["Start Dir"].as<glm::vec3>()) * cPLength + branch.m_bezierCurve.m_p0;
						branch.m_bezierCurve.m_p2 =
										branch.m_bezierCurve.m_p3 - glm::normalize(inBranch["End Dir"].as<glm::vec3>()) * cPLength;
						branch.m_startThickness = inBranch["Start Radius"].as<float>();
						branch.m_endThickness = inBranch["End Radius"].as<float>();
						branch.m_handle = i;
						branch.m_parentHandle = -1;
						branch.m_endJunction = false;
						branch.m_childHandles.clear();

						m_min = glm::min(m_min, branch.m_bezierCurve.m_p0);
						m_max = glm::max(m_max, branch.m_bezierCurve.m_p0);
						m_min = glm::min(m_min, branch.m_bezierCurve.m_p3);
						m_max = glm::max(m_max, branch.m_bezierCurve.m_p3);
				}
		}
		catch (std::exception e) {
				UNIENGINE_ERROR("Failed to load!");
				return;
		}
}

void TreePointCloud::OnInspect() {
		static Handle previousHandle = 0;
		static std::vector<glm::mat4> scatterPointMatrices;
		static std::vector<glm::mat4> nodeMatrices;
		static std::vector<glm::vec3> scatterConnectionStarts;
		static std::vector<glm::vec3> scatterConnectionEnds;
		static std::vector<glm::vec3> junctionConnectionStarts;
		static std::vector<glm::vec3> junctionConnectionEnds;
		static std::vector<glm::vec3> filteredJunctionConnectionStarts;
		static std::vector<glm::vec3> filteredJunctionConnectionEnds;

		static std::vector<glm::vec3> scannedBranchStarts;
		static std::vector<glm::vec3> scannedBranchEnds;
		static bool enableDebugRendering = true;

		static bool drawBranches = true;
		static bool drawScatterPointsConnections = false;
		static bool drawJunctionConnections = false;
		static bool drawFilteredConnections = true;
		static bool drawNode = true;
		static float branchWidth = 0.01f;
		static float connectionWidth = 0.001f;
		static float pointSize = 0.02f;

		static float nodeSize = 1.0f;

		static glm::vec4 scatterPointColor = glm::vec4(0, 1, 0, 1);
		static glm::vec4 scatterPointCollectionColor = glm::vec4(1, 1, 1, 1);
		static glm::vec4 junctionCollectionColor = glm::vec4(1, 0, 0, 1);
		static glm::vec4 filteredJunctionCollectionColor = glm::vec4(1, 1, 0, 1);
		static glm::vec4 branchColor = glm::vec4(0, 1, 1, 1);
		static glm::vec4 nodeColor = glm::vec4(1, 1, 1, 1);
		static ConnectivityGraphSettings connectivityGraphSettings;
		ImGui::DragFloat("Branch width", &branchWidth, 0.001f, 0.001f, 1.0f);
		ImGui::DragFloat("Connection width", &connectionWidth, 0.001f, 0.001f, 1.0f);
		ImGui::DragFloat("Point size", &pointSize, 0.001f, 0.001f, 1.0f);
		ImGui::DragFloat("Node size", &nodeSize, 0.01f, 0.01f, 10.0f);
		ImGui::Checkbox("Debug Rendering", &enableDebugRendering);
		if (enableDebugRendering) {
				ImGui::Checkbox("Render scatter point connections", &drawScatterPointsConnections);
				ImGui::Checkbox("Render branch", &drawBranches);
				ImGui::Checkbox("Render all junction connections", &drawJunctionConnections);
				ImGui::Checkbox("Render filtered junction connections", &drawFilteredConnections);
				ImGui::Checkbox("Render nodes", &drawNode);
				ImGui::ColorEdit4("Scatter Point Color", &scatterPointColor.x);
				if (drawBranches) ImGui::ColorEdit4("Branch Color", &branchColor.x);
				if (drawScatterPointsConnections) ImGui::ColorEdit4("Scatter Connection Color", &scatterPointCollectionColor.x);
				if (drawJunctionConnections) ImGui::ColorEdit4("Junction Connection Color", &junctionCollectionColor.x);
				if (drawFilteredConnections)
						ImGui::ColorEdit4("Filtered Scatter Connection Color", &filteredJunctionCollectionColor.x);
				if (drawNode)
						ImGui::ColorEdit4("Node Color", &nodeColor.x);

				if (ImGui::Button("Refresh Data")) {
						previousHandle = GetHandle();
						scatterPointMatrices.resize(m_points.size());
						const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
						for (int i = 0; i < m_points.size(); i++) {
								scatterPointMatrices[i] = glm::translate(m_points[i].m_position) * glm::scale(glm::vec3(1.0f));
						}
						scatterConnectionStarts.resize(m_scatterPointsConnections.size());
						scatterConnectionEnds.resize(m_scatterPointsConnections.size());
						for (int i = 0; i < m_scatterPointsConnections.size(); i++) {
								scatterConnectionStarts[i] = m_scatterPointsConnections[i].first;
								scatterConnectionEnds[i] = m_scatterPointsConnections[i].second;
						}

						junctionConnectionStarts.resize(m_branchConnections.size());
						junctionConnectionEnds.resize(m_branchConnections.size());
						for (int i = 0; i < m_branchConnections.size(); i++) {
								junctionConnectionStarts[i] = m_branchConnections[i].first;
								junctionConnectionEnds[i] = m_branchConnections[i].second;
						}

						filteredJunctionConnectionStarts.resize(m_filteredJunctionConnections.size());
						filteredJunctionConnectionEnds.resize(m_filteredJunctionConnections.size());
						for (int i = 0; i < m_filteredJunctionConnections.size(); i++) {
								filteredJunctionConnectionStarts[i] = m_filteredJunctionConnections[i].first;
								filteredJunctionConnectionEnds[i] = m_filteredJunctionConnections[i].second;
						}

						scannedBranchStarts.resize(m_branches.size());
						scannedBranchEnds.resize(m_branches.size());
						for (int i = 0; i < m_branches.size(); i++) {
								scannedBranchStarts[i] = m_branches[i].m_bezierCurve.m_p0;
								scannedBranchEnds[i] = m_branches[i].m_bezierCurve.m_p3;
						}
						nodeMatrices.clear();
						for (const auto &skeleton: m_skeletons) {
								const auto &nodeList = skeleton.RefSortedNodeList();
								auto startIndex = nodeMatrices.size();
								nodeMatrices.resize(startIndex + nodeList.size());
								for (int i = 0; i < nodeList.size(); i++) {
										const auto &node = skeleton.PeekNode(nodeList[i]);
										nodeMatrices[startIndex + i] = glm::translate(node.m_info.m_globalPosition) *
																									 glm::scale(glm::vec3(node.m_info.m_thickness * nodeSize));
								}
						}
				}
				static ReconstructionSettings reconstructionSettings;
				if (ImGui::Button("Build Skeleton")) {
						BuildTreeStructure(reconstructionSettings);
				}

				static TreeMeshGeneratorSettings meshGeneratorSettings;
				meshGeneratorSettings.OnInspect();
				if (ImGui::Button("Form tree mesh")) {
						if (m_filteredJunctionConnections.empty()) {
								m_skeletons.clear();
								EstablishConnectivityGraph(connectivityGraphSettings);
						}
						if (m_skeletons.empty()) BuildTreeStructure(reconstructionSettings);
						GenerateMeshes(meshGeneratorSettings);
				}

				if (enableDebugRendering && !scatterPointMatrices.empty()) {
						Gizmos::DrawGizmoMeshInstanced(DefaultResources::Primitives::Sphere, scatterPointColor,
																					 scatterPointMatrices,
																					 glm::mat4(1.0f),
																					 pointSize);
						if (drawBranches)
								Gizmos::DrawGizmoRays(branchColor, scannedBranchStarts, scannedBranchEnds,
																			branchWidth);
						if (drawScatterPointsConnections)
								Gizmos::DrawGizmoRays(scatterPointCollectionColor, scatterConnectionStarts, scatterConnectionEnds,
																			connectionWidth);
						if (drawJunctionConnections)
								Gizmos::DrawGizmoRays(junctionCollectionColor, junctionConnectionStarts, junctionConnectionEnds,
																			connectionWidth * 1.1f);
						if (drawFilteredConnections)
								Gizmos::DrawGizmoRays(filteredJunctionCollectionColor, filteredJunctionConnectionStarts,
																			filteredJunctionConnectionEnds,
																			connectionWidth * 1.2f);

						if (drawNode) {
								Gizmos::DrawGizmoMeshInstanced(DefaultResources::Primitives::Sphere, nodeColor, nodeMatrices);
						}
				}
		}
		FileUtils::OpenFile("Load YAML", "YAML", {".yml"}, [&](const std::filesystem::path &path) {
			ImportGraph(path);
		}, false);

		if (!m_points.empty()) {
				if (ImGui::TreeNodeEx("Connectivity Graph Settings", ImGuiTreeNodeFlags_DefaultOpen)) {
						connectivityGraphSettings.OnInspect();

						ImGui::TreePop();
				}
				if (ImGui::Button("Establish Connectivity Graph")) EstablishConnectivityGraph(connectivityGraphSettings);
		}
}

void TreePointCloud::EstablishConnectivityGraph(const ConnectivityGraphSettings &settings) {
		VoxelGrid<std::vector<PointCloudVoxel>> pointVoxelGrid;
		pointVoxelGrid.Initialize(3.0f * settings.m_edgeLength, m_min, m_max);
		for (auto &point: m_points) {
				point.m_neighbors.clear();
				point.m_neighborBranchStarts.clear();
				point.m_neighborBranchEnds.clear();
				PointCloudVoxel voxel;
				voxel.m_handle = point.m_handle;
				voxel.m_position = point.m_position;
				voxel.m_type = PointCloudVoxelType::ScatteredPoint;
				pointVoxelGrid.Ref(point.m_position).emplace_back(voxel);
		}

		for (auto &branch: m_branches) {
				branch.m_childHandles.clear();
				branch.m_startNeighbors.clear();
				branch.m_endNeighbors.clear();

				branch.m_neighborBranchStarts.clear();
				branch.m_neighborBranchEnds.clear();

				branch.m_parentHandle = -1;
				PointCloudVoxel voxel;
				voxel.m_handle = branch.m_handle;
				voxel.m_position = branch.m_bezierCurve.m_p0;
				voxel.m_type = PointCloudVoxelType::BranchStart;
				pointVoxelGrid.Ref(branch.m_bezierCurve.m_p0).emplace_back(voxel);
				voxel.m_position = branch.m_bezierCurve.m_p3;
				voxel.m_type = PointCloudVoxelType::BranchEnd;
				pointVoxelGrid.Ref(branch.m_bezierCurve.m_p3).emplace_back(voxel);
		}

		m_scatterPointsConnections.clear();

		for (auto &point: m_points) {
				if (m_scatterPointsConnections.size() > 1000000) {
						UNIENGINE_ERROR("Too much connections!");
						return;
				}
				FindPoints(point.m_position, pointVoxelGrid, settings.m_edgeLength,
									 [&](const PointCloudVoxel &voxel) {
										 if (voxel.m_type != PointCloudVoxelType::ScatteredPoint) return;
										 if (voxel.m_handle == point.m_handle) return;
										 for (const auto &neighbor: point.m_neighbors) {
												 if (voxel.m_handle == neighbor) return;
										 }
										 auto &otherPoint = m_points[voxel.m_handle];
										 point.m_neighbors.emplace_back(voxel.m_handle);
										 otherPoint.m_neighbors.emplace_back(point.m_handle);
										 m_scatterPointsConnections.emplace_back(point.m_position, otherPoint.m_position);
									 });
		}
		m_branchConnections.clear();
		for (auto &branch: m_branches) {
				float currentEdgeLength = settings.m_edgeLength;
				int timeout = 0;
				bool findScatterPoint = false;
				while (!findScatterPoint && timeout < settings.m_maxTimeout) {
						FindPoints(branch.m_bezierCurve.m_p0, pointVoxelGrid, currentEdgeLength,
											 [&](const PointCloudVoxel &voxel) {
												 if (voxel.m_type == PointCloudVoxelType::BranchStart) return;
												 if (voxel.m_type == PointCloudVoxelType::ScatteredPoint) {
														 findScatterPoint = true;
														 auto &otherPoint = m_points[voxel.m_handle];
														 for (const auto &i: branch.m_startNeighbors) {
																 if (i == voxel.m_handle) return;
														 }
														 branch.m_startNeighbors.emplace_back(voxel.m_handle);
														 otherPoint.m_neighborBranchStarts.emplace_back(branch.m_handle);
												 } else {
														 if (glm::distance(voxel.m_position, branch.m_bezierCurve.m_p0) >
																 settings.m_forceConnectionLength &&
																 glm::dot(glm::normalize(voxel.m_position - branch.m_bezierCurve.m_p0),
																					glm::normalize(branch.m_bezierCurve.m_p0 - branch.m_bezierCurve.m_p1)) <
																 glm::cos(glm::radians(settings.m_junctionLimit)))
																 return;
														 for (const auto &i: branch.m_neighborBranchEnds) {
																 if (i == voxel.m_handle) return;
														 }
														 auto &otherBranch = m_branches[voxel.m_handle];
														 branch.m_neighborBranchEnds.emplace_back(voxel.m_handle);
														 otherBranch.m_neighborBranchStarts.emplace_back(branch.m_handle);
												 }
												 m_branchConnections.emplace_back(branch.m_bezierCurve.m_p0, voxel.m_position);
											 });
						currentEdgeLength += settings.m_finderStep;
						timeout++;
				}

				currentEdgeLength = settings.m_edgeLength;
				timeout = 0;
				findScatterPoint = false;
				while (!findScatterPoint && timeout < settings.m_maxTimeout) {
						FindPoints(branch.m_bezierCurve.m_p3, pointVoxelGrid, currentEdgeLength,
											 [&](const PointCloudVoxel &voxel) {
												 if (voxel.m_type == PointCloudVoxelType::BranchEnd) return;
												 if (voxel.m_type == PointCloudVoxelType::ScatteredPoint) {
														 findScatterPoint = true;
														 auto &otherPoint = m_points[voxel.m_handle];
														 for (const auto &i: branch.m_endNeighbors) {
																 if (i == voxel.m_handle) return;
														 }
														 branch.m_endNeighbors.emplace_back(voxel.m_handle);
														 otherPoint.m_neighborBranchEnds.emplace_back(branch.m_handle);
												 } else {
														 if (glm::distance(voxel.m_position, branch.m_bezierCurve.m_p3) >
																 settings.m_forceConnectionLength &&
																 glm::dot(glm::normalize(voxel.m_position - branch.m_bezierCurve.m_p3),
																					glm::normalize(branch.m_bezierCurve.m_p3 - branch.m_bezierCurve.m_p2)) <
																 glm::cos(glm::radians(settings.m_junctionLimit)))
																 return;
														 for (const auto &i: branch.m_neighborBranchStarts) {
																 if (i == voxel.m_handle) return;
														 }
														 auto &otherBranch = m_branches[voxel.m_handle];
														 branch.m_neighborBranchStarts.emplace_back(voxel.m_handle);
														 otherBranch.m_neighborBranchEnds.emplace_back(branch.m_handle);
												 }
												 m_branchConnections.emplace_back(branch.m_bezierCurve.m_p3, voxel.m_position);
											 });
						currentEdgeLength += settings.m_finderStep;
						timeout++;
				}
		}

		int endJunctionCount = 0;
		int startJunctionCount = 0;
		for (auto &branches: m_branches) {
				if (branches.m_endNeighbors.empty() && branches.m_neighborBranchStarts.empty()) {
						branches.m_endJunction = true;
						endJunctionCount++;
				} else {
						branches.m_endJunction = false;
				}
				if (branches.m_startNeighbors.empty() && branches.m_neighborBranchEnds.empty()) {
						startJunctionCount++;
				}
		}
		UNIENGINE_LOG("End junction: " + std::to_string(endJunctionCount));
		UNIENGINE_LOG("Start junction: " + std::to_string(startJunctionCount));

		m_filteredJunctionConnections.clear();
		for (auto &branch: m_branches) {
				//Detect connection to branch start.
				std::unordered_set<BranchHandle> availableCandidates;
				for (const auto &branchEndHandle: branch.m_neighborBranchEnds) {
						availableCandidates.emplace(branchEndHandle);
				}
				std::unordered_set<PointHandle> visitedPoints;
				std::vector<PointHandle> processingPoints = branch.m_startNeighbors;
				for (const auto &i: processingPoints) {
						visitedPoints.emplace(i);
				}
				while (!processingPoints.empty()) {
						auto currentPointHandle = processingPoints.back();
						visitedPoints.emplace(currentPointHandle);
						processingPoints.pop_back();
						auto &currentPoint = m_points[currentPointHandle];
						for (const auto &neighborHandle: currentPoint.m_neighbors) {
								if (visitedPoints.find(neighborHandle) != visitedPoints.end()) continue;
								auto &neighbor = m_points[neighborHandle];
								//We stop search if the point is junction point.
								for (const auto &branchEndHandle: neighbor.m_neighborBranchEnds) {
										availableCandidates.emplace(branchEndHandle);
								}
								processingPoints.emplace_back(neighborHandle);
						}
				}
				if (availableCandidates.empty()) continue;
				float minDistance = 999.0f;
				BranchHandle bestCandidate;
				for (const auto &candidateHandle: availableCandidates) {
						auto &candidate = m_branches[candidateHandle];
						auto distance = glm::distance(branch.m_bezierCurve.m_p0, candidate.m_bezierCurve.m_p3);
						if (distance < minDistance) {
								minDistance = distance;
								bestCandidate = candidateHandle;
						}
				}
				m_branches[bestCandidate].m_childHandles.emplace_back(branch.m_handle);
				branch.m_parentHandle = bestCandidate;
				m_filteredJunctionConnections.emplace_back(branch.m_bezierCurve.m_p0,
																									 m_branches[bestCandidate].m_bezierCurve.m_p3);
		}

}

void TreePointCloud::BuildTreeStructure(const ReconstructionSettings &reconstructionSettings) {
		m_skeletons.clear();
		auto copiedBranches = m_branches;
		std::vector<BranchHandle> rootBranchHandles;
		for (auto &branch: copiedBranches) {
				branch.m_chainNodeHandles.clear();
				const auto branchStart = branch.m_bezierCurve.m_p0;
				if (branchStart.y < reconstructionSettings.m_minHeight) {
						bool replaced = false;
						for (int i = 0; i < rootBranchHandles.size(); i++) {
								const auto &rootBranchStart = copiedBranches[rootBranchHandles[i]].m_bezierCurve.m_p0;
								if (glm::distance(glm::vec2(rootBranchStart.x, rootBranchStart.z),
																	glm::vec2(branchStart.x, branchStart.z)) < reconstructionSettings.m_maxTreeDistance
										&& rootBranchStart.y > branchStart.y) {
										replaced = true;
										rootBranchHandles[i] = branch.m_handle;
								}
						}
						if (!replaced) rootBranchHandles.emplace_back(branch.m_handle);
				}
		}
		for(const auto& rootBranchHandle : rootBranchHandles) {
				auto &skeleton = m_skeletons.emplace_back();
				std::queue<BranchHandle> processingBranchHandles;
				processingBranchHandles.emplace(rootBranchHandle);
				while (!processingBranchHandles.empty()) {
						auto processingBranchHandle = processingBranchHandles.front();
						processingBranchHandles.pop();
						auto &processingBranch = copiedBranches[processingBranchHandle];
						bool onlyChild = true;
						NodeHandle prevNodeHandle = -1;
						float chainLength = glm::distance(processingBranch.m_bezierCurve.m_p0, processingBranch.m_bezierCurve.m_p3);
						int chainAmount = glm::max(2, (int) (chainLength /
																								 reconstructionSettings.m_internodeLength));
						if (processingBranch.m_handle == rootBranchHandle) {
								prevNodeHandle = 0;
								processingBranch.m_chainNodeHandles.emplace_back(0);
						} else {
								auto &parentBranch = copiedBranches[processingBranch.m_parentHandle];
								if (parentBranch.m_childHandles.size() > 1) onlyChild = false;
								auto chainFirstNodeHandle = skeleton.Extend(parentBranch.m_chainNodeHandles.back(), !onlyChild);
								processingBranch.m_chainNodeHandles.emplace_back(chainFirstNodeHandle);
								prevNodeHandle = chainFirstNodeHandle;
						}
						for (int i = 1; i < chainAmount; i++) {
								auto newNodeHandle = skeleton.Extend(prevNodeHandle, false);
								processingBranch.m_chainNodeHandles.emplace_back(newNodeHandle);
								prevNodeHandle = newNodeHandle;
						}
						for (int i = 0; i < chainAmount; i++) {
								auto &node = skeleton.RefNode(processingBranch.m_chainNodeHandles[i]);
								node.m_info.m_globalPosition = processingBranch.m_bezierCurve.GetPoint(
												static_cast<float>(i) / chainAmount);
								node.m_info.m_length = glm::distance(
												processingBranch.m_bezierCurve.GetPoint(static_cast<float>(i) / chainAmount),
												processingBranch.m_bezierCurve.GetPoint(static_cast<float>(i + 1) / chainAmount));
								node.m_info.m_localPosition = glm::normalize(
												processingBranch.m_bezierCurve.GetAxis(static_cast<float>(i) / chainAmount));
								node.m_info.m_thickness = glm::mix(processingBranch.m_startThickness, processingBranch.m_endThickness,
																									 static_cast<float>(i) / chainAmount);
						}
						for (const auto &childBranchHandle: processingBranch.m_childHandles) {
								processingBranchHandles.emplace(childBranchHandle);
						}
				}
				skeleton.SortLists();
				auto &sortedNodeList = skeleton.RefSortedNodeList();
				auto &rootNode = skeleton.RefNode(0);
				rootNode.m_info.m_globalPosition = glm::vec3(0.0f);
				rootNode.m_info.m_localRotation = glm::vec3(0.0f);
				rootNode.m_info.m_globalRotation = rootNode.m_info.m_regulatedGlobalRotation = glm::vec3(glm::radians(90.0f),
																																																 0.0f,
																																																 0.0f);

				for (const auto &nodeHandle: sortedNodeList) {
						auto &node = skeleton.RefNode(nodeHandle);
						if (node.GetParentHandle() <= 0) continue;
						auto &nodeInfo = node.m_info;
						auto &parentNode = skeleton.RefNode(node.GetParentHandle());
						auto front = nodeInfo.m_localPosition;
						nodeInfo.m_localPosition = nodeInfo.m_globalPosition - parentNode.m_info.m_localPosition;
						auto parentUp = parentNode.m_info.m_globalRotation * glm::vec3(0, 1, 0);
						auto regulatedUp = glm::normalize(glm::cross(glm::cross(front, parentUp), front));
						nodeInfo.m_globalRotation = glm::quatLookAt(front, regulatedUp);
						nodeInfo.m_localRotation = glm::inverse(parentNode.m_info.m_globalRotation) * nodeInfo.m_globalRotation;
						nodeInfo.m_regulatedGlobalRotation = nodeInfo.m_globalRotation;
						m_min = glm::min(m_min, nodeInfo.m_globalPosition);
						m_max = glm::max(m_max, nodeInfo.m_globalPosition);
						const auto endPosition = nodeInfo.m_globalPosition + nodeInfo.m_length *
																																 (nodeInfo.m_globalRotation *
																																	glm::vec3(0, 0, -1));
						m_min = glm::min(m_min, endPosition);
						m_max = glm::max(m_max, endPosition);
				}
				//m_skeleton.CalculateTransforms();
				skeleton.CalculateFlows();
		}
}

void TreePointCloud::ClearMeshes() const {
		const auto scene = GetScene();
		const auto self = GetOwner();
		const auto children = scene->GetChildren(self);
		for (const auto &child: children) {
				auto name = scene->GetEntityName(child);
				if(name.substr(0, 4) == "Tree"){
						scene->DeleteEntity(child);
				}
		}
}

void TreePointCloud::GenerateMeshes(const TreeMeshGeneratorSettings &meshGeneratorSettings) {
		const auto scene = GetScene();
		const auto self = GetOwner();
		const auto children = scene->GetChildren(self);

		ClearMeshes();
		for(int i = 0; i < m_skeletons.size(); i++) {
				Entity treeEntity;
				treeEntity = scene->CreateEntity("Tree " + std::to_string(i));
				scene->SetParent(treeEntity, self);
				if (meshGeneratorSettings.m_enableBranch) {
						Entity branchEntity;
						branchEntity = scene->CreateEntity("Branch Mesh");
						scene->SetParent(branchEntity, treeEntity);

						std::vector<Vertex> vertices;
						std::vector<unsigned int> indices;
						CylindricalMeshGenerator<BaseSkeletonData, BaseFlowData, BaseNodeData> meshGenerator;
						meshGenerator.Generate(m_skeletons[i], vertices, indices, meshGeneratorSettings, 999.0f);

						auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
						auto material = ProjectManager::CreateTemporaryAsset<Material>();
						material->SetProgram(DefaultResources::GLPrograms::StandardProgram);
						mesh->SetVertices(17, vertices, indices);
						auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(branchEntity).lock();
						if (meshGeneratorSettings.m_overridePresentation) {
								material->m_materialProperties.m_albedoColor = meshGeneratorSettings.m_presentationOverrideSettings.m_branchOverrideColor;
						} else {
								material->m_materialProperties.m_albedoColor = glm::vec3(109, 79, 75) / 255.0f;
						}
						material->m_materialProperties.m_roughness = 1.0f;
						material->m_materialProperties.m_metallic = 0.0f;
						meshRenderer->m_mesh = mesh;
						meshRenderer->m_material = material;
				}
		}
}

void ConnectivityGraphSettings::OnInspect() {
		ImGui::DragFloat("Finder step", &m_finderStep, 0.01f, 0.01f, 1.0f);
		ImGui::DragFloat("Edge width", &m_edgeLength, 0.01f, 0.01f, 1.0f);
		ImGui::DragFloat("Force connection length", &m_forceConnectionLength, 0.01f, 0.01f, 1.0f);

		ImGui::DragFloat("Junction finder angle", &m_junctionLimit, 0.01f, 0.01f, 90.0f);
		ImGui::DragInt("Junction finder timeout", &m_maxTimeout, 1, 1, 30);
}
