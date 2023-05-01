#include <unordered_set>
#include "TreePointCloud.hpp"
#include "Graphics.hpp"
#include "EcoSysLabLayer.hpp"
#include "rapidcsv.h"

using namespace EcoSysLab;

void TreePointCloud::FindPoints(PointHandle targetPoint, VoxelGrid<std::vector<PointHandle>> &pointVoxelGrid,
																const float radius,
																const std::function<void(PointHandle handle)> &func) const {
		auto &point = m_points[targetPoint];
		pointVoxelGrid.ForEach(point.m_position, radius * 2.0f, [&](const std::vector<PointHandle> &pointHandles) {
			for (const auto pointHandle: pointHandles) {
					auto &currentPoint = m_points[pointHandle];
					if (glm::distance(point.m_position, currentPoint.m_position) > radius) continue;
					func(pointHandle);
			}
		});
}

void TreePointCloud::ImportCsv(const std::filesystem::path &path) {
		rapidcsv::Document doc(path.string(), rapidcsv::LabelParams(-1, -1));
		auto pointSize = doc.GetColumn<float>(0).size();
		m_points.resize(pointSize);
		m_scatterPointsConnections.clear();
		m_min = glm::vec3(FLT_MAX);
		m_max = glm::vec3(FLT_MIN);
		int maxJunctionSize = 0;
		for (int i = 0; i < pointSize; i++) {
				auto &point = m_points[i];
				point.m_handle = i;
				point.m_position = glm::vec3(doc.GetCell<float>(0, i), doc.GetCell<float>(1, i), doc.GetCell<float>(2, i));
				point.m_junctionHandle = doc.GetCell<int>(3, i);
				point.m_prevHandle = doc.GetCell<int>(5, i);
				point.m_nextHandle = -1;
				point.m_thickness = doc.GetCell<float>(4, i);
				maxJunctionSize = glm::max(point.m_junctionHandle, maxJunctionSize);
				m_min = glm::min(point.m_position, m_min);
				m_max = glm::max(point.m_position, m_max);
		}
		m_junctions.resize(maxJunctionSize + 1);
		for (int i = 0; i < pointSize; i++) {
				auto &point = m_points[i];
				if (point.m_junctionHandle == -1) continue;
				if (point.m_prevHandle != -1) {
						m_points[point.m_prevHandle].m_nextHandle = i;
				} else {
						m_junctions[point.m_junctionHandle].m_start = point.m_position;
						m_junctions[point.m_junctionHandle].m_startHandle = i;
				}
				if (point.m_nextHandle == -1) {
						m_junctions[point.m_junctionHandle].m_end = point.m_position;
						m_junctions[point.m_junctionHandle].m_endHandle = i;
				}
		}
		for (auto &junction: m_junctions) {
				junction.m_center = (junction.m_start + junction.m_end) / 2.0f;
				if (glm::distance(junction.m_start, junction.m_end) == 0.0f) {
						UNIENGINE_ERROR("Junction with zero length!");
						continue;
				}
				junction.m_direction = glm::normalize(junction.m_end - junction.m_start);
		}
}

void TreePointCloud::OnInspect() {
		static Handle previousHandle = 0;
		static std::vector<glm::mat4> matrices;
		static std::vector<glm::vec4> colors;
		static std::vector<glm::vec3> scatterConnectionStarts;
		static std::vector<glm::vec3> scatterConnectionEnds;
		static std::vector<glm::vec3> junctionConnectionStarts;
		static std::vector<glm::vec3> junctionConnectionEnds;
		static std::vector<glm::vec3> filteredJunctionConnectionStarts;
		static std::vector<glm::vec3> filteredJunctionConnectionEnds;
		static bool enableDebugRendering = true;

		static bool drawScatterPointsConnections = false;
		static bool drawJunctionConnections = false;
		static bool drawFilteredConnections = true;

		static float connectionWidth = 0.001f;
		static float pointSize = 0.02f;

		static glm::vec4 scatterPointColor = glm::vec4(0, 1, 0, 1);
		static glm::vec4 scatterPointCollectionColor = glm::vec4(1, 1, 1, 1);
		static glm::vec4 junctionCollectionColor = glm::vec4(1, 0, 0, 1);
		static glm::vec4 filteredJunctionCollectionColor = glm::vec4(1, 1, 0, 1);
		static ConnectivityGraphSettings settings;
		ImGui::DragFloat("Connection width", &connectionWidth, 0.001f, 0.001f, 1.0f);
		ImGui::DragFloat("Point size", &pointSize, 0.001f, 0.001f, 1.0f);
		ImGui::Checkbox("Debug Rendering", &enableDebugRendering);
		if (enableDebugRendering) {
				ImGui::Checkbox("Render scatter point connections", &drawScatterPointsConnections);
				ImGui::Checkbox("Render all junction connections", &drawJunctionConnections);
				ImGui::Checkbox("Render filtered junction connections", &drawFilteredConnections);

				ImGui::ColorEdit4("Scatter Point Color", &scatterPointColor.x);

				if (drawScatterPointsConnections) ImGui::ColorEdit4("Scatter Connection Color", &scatterPointCollectionColor.x);
				if (drawJunctionConnections) ImGui::ColorEdit4("Junction Connection Color", &junctionCollectionColor.x);
				if (drawFilteredConnections)
						ImGui::ColorEdit4("Filtered Scatter Connection Color", &filteredJunctionCollectionColor.x);

				if (ImGui::Button("Refresh Data")) {
						previousHandle = GetHandle();
						matrices.resize(m_points.size());
						colors.resize(m_points.size());
						const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
						for (int i = 0; i < m_points.size(); i++) {
								matrices[i] = glm::translate(m_points[i].m_position) * glm::scale(glm::vec3(1.0f));
								if (m_points[i].m_junctionHandle == -1) {
										colors[i] = scatterPointColor;
								} else {
										colors[i] = glm::vec4(ecoSysLabLayer->RandomColors()[m_points[i].m_junctionHandle], 1.0f);
								}
						}
						scatterConnectionStarts.resize(m_scatterPointsConnections.size());
						scatterConnectionEnds.resize(m_scatterPointsConnections.size());
						for (int i = 0; i < m_scatterPointsConnections.size(); i++) {
								scatterConnectionStarts[i] = m_points[m_scatterPointsConnections[i].first].m_position;
								scatterConnectionEnds[i] = m_points[m_scatterPointsConnections[i].second].m_position;
						}

						junctionConnectionStarts.resize(m_junctionConnections.size());
						junctionConnectionEnds.resize(m_junctionConnections.size());
						for (int i = 0; i < m_junctionConnections.size(); i++) {
								junctionConnectionStarts[i] = m_points[m_junctionConnections[i].first].m_position;
								junctionConnectionEnds[i] = m_points[m_junctionConnections[i].second].m_position;
						}

						filteredJunctionConnectionStarts.resize(m_filteredJunctionConnections.size());
						filteredJunctionConnectionEnds.resize(m_filteredJunctionConnections.size());
						for (int i = 0; i < m_filteredJunctionConnections.size(); i++) {
								filteredJunctionConnectionStarts[i] = m_points[m_filteredJunctionConnections[i].first].m_position;
								filteredJunctionConnectionEnds[i] = m_points[m_filteredJunctionConnections[i].second].m_position;
						}
				}
				TreeMeshGeneratorSettings meshGeneratorSettings;
				meshGeneratorSettings.m_subdivision = 1.0f;
				meshGeneratorSettings.m_resolution = 1.0f;
				if (ImGui::Button("Form tree mesh")) {
						GenerateMeshes(meshGeneratorSettings);
				}

				if (enableDebugRendering && !matrices.empty()) {
						Gizmos::DrawGizmoMeshInstancedColored(DefaultResources::Primitives::Sphere, colors, matrices,
																									glm::mat4(1.0f),
																									pointSize);
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
				}
		}
		FileUtils::OpenFile("Load CSV", "CSV", {".csv"}, [&](const std::filesystem::path &path) {
			ImportCsv(path);
		}, false);

		if (!m_points.empty()) {
				if (ImGui::TreeNodeEx("Connectivity settings", ImGuiTreeNodeFlags_DefaultOpen)) {

						ImGui::DragFloat("Finder step", &settings.m_finderStep, 0.01f, 0.01f, 1.0f);
						ImGui::DragFloat("Edge width", &settings.m_edgeLength, 0.01f, 0.01f, 1.0f);

						ImGui::DragFloat("Junction finder angle", &settings.m_junctionLimit, 0.01f, 0.01f, 90.0f);
						ImGui::DragInt("Junction finder timeout", &settings.m_maxTimeout, 1, 1, 30);
						ImGui::TreePop();
				}
				if (ImGui::Button("Establish Connectivity Graph")) EstablishConnectivityGraph(settings);
		}
}

void TreePointCloud::EstablishConnectivityGraph(const ConnectivityGraphSettings &settings) {
		std::vector<PointHandle> processingPoints;
		VoxelGrid<std::vector<PointHandle>> pointVoxelGrid;
		pointVoxelGrid.Initialize(3.0f * settings.m_edgeLength, m_min, m_max);
		for (auto &point: m_points) {
				point.m_neighbors.clear();
				if (point.m_junctionHandle != -1 && point.m_nextHandle >= 0 && point.m_prevHandle >= 0) continue;
				processingPoints.emplace_back(point.m_handle);
				pointVoxelGrid.Ref(point.m_position).emplace_back(point.m_handle);
		}
		m_scatterPointsConnections.clear();
		m_junctionConnections.clear();

		for (const auto &currentProcessingPointHandle: processingPoints) {
				if (m_scatterPointsConnections.size() > 1000000) {
						UNIENGINE_ERROR("Too much connections!");
						return;
				}
				auto &currentProcessingPoint = m_points[currentProcessingPointHandle];
				if (currentProcessingPoint.m_junctionHandle == -1) {
						FindPoints(currentProcessingPointHandle, pointVoxelGrid, settings.m_edgeLength,
											 [&](PointHandle pointHandle) {
												 if (pointHandle == currentProcessingPointHandle) return;
												 for (const auto &neighbor: currentProcessingPoint.m_neighbors) {
														 if (pointHandle == neighbor) return;
												 }
												 auto &otherPoint = m_points[pointHandle];
												 if (otherPoint.m_junctionHandle != -1) return;
												 currentProcessingPoint.m_neighbors.emplace_back(pointHandle);
												 otherPoint.m_neighbors.emplace_back(currentProcessingPointHandle);
												 m_scatterPointsConnections.emplace_back(currentProcessingPointHandle, pointHandle);
											 });
				} else {
						float currentEdgeLength = settings.m_edgeLength;
						int timeout = 0;
						bool isJunctionStart = currentProcessingPoint.m_prevHandle == -1;
						bool isJunctionEnd = currentProcessingPoint.m_nextHandle == -1;
						const auto &junction = m_junctions[currentProcessingPoint.m_junctionHandle];
						bool findScatterPoint = false;
						while (!findScatterPoint && timeout < settings.m_maxTimeout) {
								FindPoints(currentProcessingPointHandle, pointVoxelGrid, currentEdgeLength,
													 [&](PointHandle pointHandle) {
														 auto &otherPoint = m_points[pointHandle];
														 if (pointHandle == currentProcessingPointHandle ||
																 otherPoint.m_junctionHandle == currentProcessingPoint.m_junctionHandle)
																 return;
														 if (otherPoint.m_junctionHandle != -1 && otherPoint.m_nextHandle != -1 &&
																 otherPoint.m_prevHandle != -1)
																 return;
														 if (isJunctionStart) {
																 if (glm::dot(glm::normalize(otherPoint.m_position - junction.m_center),
																							-junction.m_direction) <
																		 glm::cos(glm::radians(settings.m_junctionLimit)))
																		 return;
																 if (otherPoint.m_prevHandle == -1) return;
														 }

														 if (isJunctionEnd) {
																 if (glm::dot(glm::normalize(otherPoint.m_position - junction.m_center),
																							junction.m_direction) <
																		 glm::cos(glm::radians(settings.m_junctionLimit))) {
																		 return;
																 }
																 if (otherPoint.m_nextHandle == -1) return;
														 }
														 if (otherPoint.m_junctionHandle == -1) findScatterPoint = true;
														 currentProcessingPoint.m_neighbors.emplace_back(pointHandle);
														 otherPoint.m_neighbors.emplace_back(currentProcessingPointHandle);
														 m_junctionConnections.emplace_back(currentProcessingPointHandle, pointHandle);
													 });
								currentEdgeLength += settings.m_finderStep;
								timeout++;
						}
				}
		}
		int endJunctionCount = 0;
		int startJunctionCount = 0;
		for (auto &junction: m_junctions) {
				if (m_points[junction.m_endHandle].m_neighbors.empty()) {
						junction.m_endJunction = true;
						endJunctionCount++;
				} else {
						junction.m_endJunction = false;
				}
				if (m_points[junction.m_startHandle].m_neighbors.empty()) {
						startJunctionCount++;
				}
				junction.m_parentHandle = -1;
				junction.m_childHandles.clear();
		}
		UNIENGINE_LOG("End junction: " + std::to_string(endJunctionCount));
		UNIENGINE_LOG("Start junction: " + std::to_string(startJunctionCount));
		m_filteredJunctionConnections.clear();
		for (JunctionHandle i = 0; i < m_junctions.size(); i++) {
				auto &junction = m_junctions[i];
				std::vector<JunctionHandle> availableCandidates;
				std::unordered_set<PointHandle> visitedPoints;
				visitedPoints.emplace(junction.m_startHandle);
				std::vector<PointHandle> processingPoints = {junction.m_startHandle};
				while (!processingPoints.empty()) {
						auto currentPointHandle = processingPoints.back();
						visitedPoints.emplace(currentPointHandle);
						processingPoints.pop_back();
						auto &currentPoint = m_points[currentPointHandle];
						for (const auto &neighborHandle: currentPoint.m_neighbors) {
								if (visitedPoints.find(neighborHandle) != visitedPoints.end()) continue;
								auto &neighbor = m_points[neighborHandle];
								//We stop search if the point is junction point.
								if (neighbor.m_junctionHandle != -1) {
										availableCandidates.emplace_back(neighbor.m_junctionHandle);
								} else {
										processingPoints.emplace_back(neighborHandle);
								}
						}
				}
				if (availableCandidates.empty()) continue;
				float minDistance = 999.0f;
				JunctionHandle bestCandidate;
				for (const auto &candidateHandle: availableCandidates) {
						auto &candidate = m_junctions[candidateHandle];
						auto distance = glm::distance(junction.m_start, candidate.m_end);
						if (distance < minDistance) {
								minDistance = distance;
								bestCandidate = candidateHandle;
						}
				}
				m_junctions[bestCandidate].m_childHandles.emplace_back(i);
				junction.m_parentHandle = bestCandidate;
				m_filteredJunctionConnections.emplace_back(junction.m_startHandle, m_junctions[bestCandidate].m_endHandle);
		}
}

BaseSkeleton TreePointCloud::BuildTreeStructure() {
		BaseSkeleton retVal;

		JunctionHandle rootJunctionHandle;
		float minHeight = 999.0f;
		for (auto &point: m_points) {
				point.m_parentNodeHandle = point.m_nodeHandle = -1;
				if (point.m_junctionHandle != -1 && point.m_prevHandle == -1
						&& point.m_position.y < minHeight) {
						rootJunctionHandle = point.m_junctionHandle;
						minHeight = point.m_position.y;
				}
		}

		std::queue<JunctionHandle> processingJunctionHandles;
		processingJunctionHandles.emplace(rootJunctionHandle);
		while (!processingJunctionHandles.empty()) {
				auto processingJunctionHandle = processingJunctionHandles.front();
				processingJunctionHandles.pop();
				auto &processingJunction = m_junctions[processingJunctionHandle];
				bool onlyChild = true;
				NodeHandle prevNodeHandle = -1;
				if(processingJunction.m_parentHandle != -1) {
						auto& parentJunction = m_junctions[processingJunction.m_parentHandle];
						if (parentJunction.m_childHandles.size() > 1) onlyChild = false;
						prevNodeHandle = m_points[parentJunction.m_endHandle].m_nodeHandle;
				}
				auto pointWalkerHandle = processingJunction.m_startHandle;
				while (pointWalkerHandle != -1) {
						auto &currentPoint = m_points[pointWalkerHandle];
						currentPoint.m_parentNodeHandle = prevNodeHandle;
						if (currentPoint.m_parentNodeHandle == -1) {
								currentPoint.m_parentNodeHandle = 0;
								currentPoint.m_nodeHandle = retVal.Extend(currentPoint.m_parentNodeHandle, false);
						} else if (pointWalkerHandle == processingJunction.m_startHandle) {
								if (onlyChild) {
										currentPoint.m_nodeHandle = retVal.Extend(currentPoint.m_parentNodeHandle, false);
								} else {
										currentPoint.m_nodeHandle = retVal.Extend(currentPoint.m_parentNodeHandle, true);
								}
						} else {
								currentPoint.m_nodeHandle = retVal.Extend(currentPoint.m_parentNodeHandle, false);
						}

						prevNodeHandle = currentPoint.m_nodeHandle;
						auto perspectivePosition = currentPoint.m_position;
						if (currentPoint.m_parentNodeHandle == 0) {
								auto& rootNode = retVal.RefNode(0);
								rootNode.m_info.m_localPosition = rootNode.m_info.m_globalPosition = glm::vec3(0.0f);
								rootNode.m_info.m_localRotation = glm::vec3(0.0f);
								rootNode.m_info.m_globalRotation = rootNode.m_info.m_regulatedGlobalRotation = glm::vec3(glm::radians(90.0f), 0.0f, 0.0f);
								rootNode.m_info.m_thickness = currentPoint.m_thickness;
						}
						auto &newNode = retVal.RefNode(currentPoint.m_nodeHandle);
						newNode.m_info.m_thickness = currentPoint.m_thickness;
						newNode.m_info.m_localPosition = currentPoint.m_position;
						pointWalkerHandle = currentPoint.m_nextHandle;
				}
				for (const auto &childJunctionHandle: processingJunction.m_childHandles) {
						processingJunctionHandles.emplace(childJunctionHandle);
				}
		}
		retVal.SortLists();
		auto& sortedNodeList = retVal.RefSortedNodeList();
		for(const auto& nodeHandle : sortedNodeList){
				auto& node = retVal.RefNode(nodeHandle);
				if(node.GetParentHandle() < 0) continue;
				auto& parentNode = retVal.RefNode(node.GetParentHandle());
				node.m_info.m_globalPosition = parentNode.m_info.m_localPosition;
		}
		for(const auto& nodeHandle : sortedNodeList){
				auto& node = retVal.RefNode(nodeHandle);
				if(node.GetParentHandle() < 0) continue;
				auto& parentNode = retVal.RefNode(node.GetParentHandle());
				parentNode.m_info.m_length = glm::distance(node.m_info.m_globalPosition, parentNode.m_info.m_globalPosition);
				parentNode.m_info.m_localPosition = node.m_info.m_globalPosition - parentNode.m_info.m_globalPosition;
				auto front = glm::normalize(parentNode.m_info.m_localPosition);
				parentNode.m_info.m_globalRotation = glm::quatLookAt(front, glm::vec3(front.y, front.z, front.x));
		}
		for(const auto& nodeHandle : sortedNodeList){
				auto& node = retVal.RefNode(nodeHandle);
				if(node.GetParentHandle() < 0) continue;
				auto& parentNode = retVal.RefNode(node.GetParentHandle());
				node.m_info.m_localRotation = glm::inverse(parentNode.m_info.m_globalRotation) * node.m_info.m_globalRotation;
		}
		//retVal.CalculateTransforms();
		retVal.CalculateFlows();

		return retVal;
}

void TreePointCloud::ClearMeshes() const {
		const auto scene = GetScene();
		const auto self = GetOwner();
		const auto children = scene->GetChildren(self);
		for (const auto &child: children) {
				auto name = scene->GetEntityName(child);
				if (name == "Branch Mesh") {
						scene->DeleteEntity(child);
				} else if (name == "Root Mesh") {
						scene->DeleteEntity(child);
				} else if (name == "Foliage Mesh") {
						scene->DeleteEntity(child);
				} else if (name == "Fruit Mesh") {
						scene->DeleteEntity(child);
				} else if (name == "Fine Root Mesh") {
						scene->DeleteEntity(child);
				}
		}
}

void TreePointCloud::GenerateMeshes(const TreeMeshGeneratorSettings &meshGeneratorSettings) {
		const auto scene = GetScene();
		const auto self = GetOwner();
		const auto children = scene->GetChildren(self);

		ClearMeshes();
		if (meshGeneratorSettings.m_enableBranch) {
				Entity branchEntity;
				branchEntity = scene->CreateEntity("Branch Mesh");
				scene->SetParent(branchEntity, self);

				std::vector<Vertex> vertices;
				std::vector<unsigned int> indices;
				auto skeleton = BuildTreeStructure();
				CylindricalMeshGenerator<BaseSkeletonData, BaseFlowData, BaseNodeData> meshGenerator;
				meshGenerator.Generate(skeleton, vertices, indices, meshGeneratorSettings, 999.0f);

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
