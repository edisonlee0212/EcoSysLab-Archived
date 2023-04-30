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
					auto& currentPoint = m_points[pointHandle];
					if(glm::distance(point.m_position, currentPoint.m_position) > radius) continue;
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

		for (int i = 0; i < pointSize; i++) {
				auto &point = m_points[i];
				point.m_handle = i;
				point.m_position = glm::vec3(doc.GetCell<float>(0, i), doc.GetCell<float>(1, i), doc.GetCell<float>(2, i));
				point.m_junctionIndex = doc.GetCell<int>(3, i);
				point.m_prevHandle = doc.GetCell<int>(5, i);
				point.m_nextHandle = -1;
				point.m_width = doc.GetCell<float>(4, i);

				m_min = glm::min(point.m_position, m_min);
				m_max = glm::max(point.m_position, m_max);
		}
		for (int i = 0; i < pointSize; i++) {
				auto &point = m_points[i];
				if (point.m_prevHandle != -1) {
						m_points[point.m_prevHandle].m_nextHandle = i;
				}
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
		static bool enableDebugRendering = true;
		static float connectionWidth = 0.001f;
		static float pointSize = 0.02f;

		static glm::vec4 scatterPointColor = glm::vec4(0, 1, 0, 1);
		static glm::vec4 scatterPointCollectionColor = glm::vec4(1, 1, 1, 1);
		static glm::vec4 junctionCollectionColor = glm::vec4(1, 0, 0, 1);
		static ConnectivityGraphSettings settings;
		ImGui::DragFloat("Connection width", &connectionWidth, 0.001f, 0.001f, 1.0f);
		ImGui::DragFloat("Point size", &pointSize, 0.001f, 0.001f, 1.0f);
		ImGui::Checkbox("Debug Rendering", &enableDebugRendering);

		ImGui::ColorEdit4("Scatter Point Color", &scatterPointColor.x);
		ImGui::ColorEdit4("Junction Connection Color", &junctionCollectionColor.x);
		ImGui::ColorEdit4("Scatter Connection Color", &scatterPointCollectionColor.x);
		if (enableDebugRendering && ImGui::Button("Refresh Data")) {
				previousHandle = GetHandle();
				matrices.resize(m_points.size());
				colors.resize(m_points.size());
				const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
				for (int i = 0; i < m_points.size(); i++) {
						matrices[i] = glm::translate(m_points[i].m_position) * glm::scale(glm::vec3(1.0f));
						if (m_points[i].m_junctionIndex == -1) {
								colors[i] = scatterPointColor;
						} else {
								colors[i] = glm::vec4(ecoSysLabLayer->RandomColors()[m_points[i].m_junctionIndex], 1.0f);
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
		}

		if (enableDebugRendering && !matrices.empty()) {
				Gizmos::DrawGizmoMeshInstancedColored(DefaultResources::Primitives::Cube, colors, matrices, glm::mat4(1.0f),
																							pointSize);
				Gizmos::DrawGizmoRays(scatterPointCollectionColor, scatterConnectionStarts, scatterConnectionEnds, connectionWidth);
				Gizmos::DrawGizmoRays(junctionCollectionColor, junctionConnectionStarts, junctionConnectionEnds, connectionWidth);
		}

		FileUtils::OpenFile("Load CSV", "CSV", {".csv"}, [&](const std::filesystem::path &path) {
			ImportCsv(path);
		}, false);

		if (!m_points.empty()) {
				if (ImGui::TreeNodeEx("Connectivity settings", ImGuiTreeNodeFlags_DefaultOpen)) {

						ImGui::DragFloat("Finder step", &settings.m_finderStep, 0.01f, 0.01f, 1.0f);
						ImGui::DragFloat("Edge width", &settings.m_edgeLength, 0.01f, 0.01f, 1.0f);
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
				if (point.m_junctionIndex != -1 && point.m_nextHandle >= 0 && point.m_prevHandle >= 0) continue;
				processingPoints.emplace_back(point.m_handle);
				pointVoxelGrid.Ref(point.m_position).emplace_back(point.m_handle);
		}
		m_scatterPointsConnections.clear();


		for(const auto& currentProcessingPointHandle : processingPoints) {
				if (m_scatterPointsConnections.size() > 1000000) {
						UNIENGINE_ERROR("Too much connections!");
						return;
				}
				auto &currentProcessingPoint = m_points[currentProcessingPointHandle];
				if (currentProcessingPoint.m_junctionIndex == -1) {
						FindPoints(currentProcessingPointHandle, pointVoxelGrid, settings.m_edgeLength,
											 [&](PointHandle pointHandle) {
												 if (pointHandle == currentProcessingPointHandle) return;
												 for (const auto &neighbor: currentProcessingPoint.m_neighbors) {
														 if (pointHandle == neighbor) return;
												 }
												 auto &otherPoint = m_points[pointHandle];
												 if(otherPoint.m_junctionIndex != -1) return;
												 currentProcessingPoint.m_neighbors.emplace_back(pointHandle);
												 otherPoint.m_neighbors.emplace_back(currentProcessingPointHandle);
												 m_scatterPointsConnections.emplace_back(currentProcessingPointHandle, pointHandle);
											 });
				} else {
						float currentEdgeLength = settings.m_edgeLength;
						int timeout = 0;
						while (currentProcessingPoint.m_neighbors.empty() && timeout < settings.m_maxTimeout) {
								FindPoints(currentProcessingPointHandle, pointVoxelGrid, currentEdgeLength,
													 [&](PointHandle pointHandle) {
														 auto &otherPoint = m_points[pointHandle];
														 if (pointHandle == currentProcessingPointHandle ||
																 otherPoint.m_junctionIndex == currentProcessingPoint.m_junctionIndex) return;
														 currentProcessingPoint.m_neighbors.emplace_back(pointHandle);
														 otherPoint.m_neighbors.emplace_back(currentProcessingPointHandle);
														 m_junctionConnections.emplace_back(currentProcessingPointHandle, pointHandle);
													 });
								currentEdgeLength += settings.m_finderStep;
								timeout++;
						}
				}
		}

}
