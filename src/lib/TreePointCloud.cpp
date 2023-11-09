#include "TreePointCloud.hpp"
#include <unordered_set>
#include "Graphics.hpp"
#include "EcoSysLabLayer.hpp"
#include "rapidcsv.h"
using namespace EcoSysLab;

void TreePointCloud::FindPoints(const glm::vec3& position, VoxelGrid<std::vector<ScatterPointData>>& pointVoxelGrid,
	float radius,
	const std::function<void(const ScatterPointData& voxel)>& func) const {
	pointVoxelGrid.ForEach(position, radius, [&](const std::vector<ScatterPointData>& voxels) {
		for (const auto& voxel : voxels) {
			if (glm::distance(position, voxel.m_position) > radius) continue;
			func(voxel);
		}
		});
}

void
TreePointCloud::ForEachBranchEnd(const glm::vec3& position, VoxelGrid<std::vector<BranchEndData>>& branchEndsVoxelGrid,
	float radius,
	const std::function<void(const BranchEndData& voxel)>& func) const {
	branchEndsVoxelGrid.ForEach(position, radius, [&](const std::vector<BranchEndData>& branchEnds) {
		for (const auto& branchEnd : branchEnds) {
			if (glm::distance(position, branchEnd.m_position) > radius) continue;
			func(branchEnd);
		}
		});
}

void TreePointCloud::CalculateNodeTransforms(ReconstructionSkeleton& skeleton)
{
	skeleton.m_min = glm::vec3(FLT_MAX);
	skeleton.m_max = glm::vec3(FLT_MIN);
	for (const auto& nodeHandle : skeleton.RefSortedNodeList()) {
		auto& node = skeleton.RefNode(nodeHandle);
		auto& nodeInfo = node.m_info;
		auto& nodeData = node.m_data;
		if (node.GetParentHandle() != -1) {
			auto& parentInfo = skeleton.RefNode(node.GetParentHandle()).m_info;
			nodeInfo.m_globalRotation =
				parentInfo.m_globalRotation * nodeData.m_localRotation;
			nodeInfo.m_globalDirection = glm::normalize(nodeInfo.m_globalRotation * glm::vec3(0, 0, -1));
			nodeInfo.m_globalPosition =
				parentInfo.m_globalPosition
				+ parentInfo.m_length * parentInfo.m_globalDirection;
			auto parentRegulatedUp = parentInfo.m_regulatedGlobalRotation * glm::vec3(0, 1, 0);
			auto regulatedUp = glm::normalize(glm::cross(glm::cross(nodeInfo.m_globalDirection, parentRegulatedUp), nodeInfo.m_globalDirection));
			nodeInfo.m_regulatedGlobalRotation = glm::quatLookAt(nodeInfo.m_globalDirection, regulatedUp);
		}
		skeleton.m_min = glm::min(skeleton.m_min, nodeInfo.m_globalPosition);
		skeleton.m_max = glm::max(skeleton.m_max, nodeInfo.m_globalPosition);
		const auto endPosition = nodeInfo.m_globalPosition
			+ nodeInfo.m_length * nodeInfo.m_globalDirection;
		skeleton.m_min = glm::min(skeleton.m_min, endPosition);
		skeleton.m_max = glm::max(skeleton.m_max, endPosition);
	}
}

void TreePointCloud::ImportGraph(const std::filesystem::path& path, float scaleFactor) {
	if (!std::filesystem::exists(path)) {
		EVOENGINE_ERROR("Not exist!");
		return;
	}
	try {
		std::ifstream stream(path.string());
		std::stringstream stringStream;
		stringStream << stream.rdbuf();
		YAML::Node in = YAML::Load(stringStream.str());

		const auto& tree = in["Tree"];
		const auto& scatterPoints = tree["Scatter Points"];
		const auto& treeParts = tree["Tree Parts"];

		m_scatteredPoints.resize(scatterPoints.size());

		for (int i = 0; i < scatterPoints.size(); i++) {
			auto& point = m_scatteredPoints[i];
			point.m_position = scatterPoints[i].as<glm::vec3>() * scaleFactor;

			point.m_handle = i;
			point.m_neighborScatterPoints.clear();
		}

		m_scannedBranches.clear();
		m_operatingBranches.clear();
		m_treeParts.clear();
		m_allocatedPoints.clear();
		m_skeletons.clear();
		m_scatterPointToBranchEndConnections.clear();
		m_scatterPointToBranchStartConnections.clear();
		m_scatterPointsConnections.clear();
		m_branchConnections.clear();
		m_filteredBranchConnections.clear();
		m_min = glm::vec3(FLT_MAX);
		m_max = glm::vec3(FLT_MIN);
		float minHeight = 999.0f;
		for (int i = 0; i < treeParts.size(); i++) {
			const auto& inTreeParts = treeParts[i];
			auto& treePart = m_treeParts.emplace_back();
			treePart.m_handle = m_treeParts.size() - 1;
			try {
				if (inTreeParts["Color"]) treePart.m_color = inTreeParts["Color"].as<glm::vec3>() / 255.0f;
			}
			catch (const std::exception& e)
			{
				EVOENGINE_ERROR("Color is wrong at node " + std::to_string(i) + ": " + std::string(e.what()));
			}
			for (const auto& inBranch : inTreeParts["Branches"]) {
				auto& branch = m_scannedBranches.emplace_back();
				branch.m_bezierCurve.m_p0 = inBranch["Start Pos"].as<glm::vec3>() * scaleFactor;
				branch.m_bezierCurve.m_p3 = inBranch["End Pos"].as<glm::vec3>() * scaleFactor;
				branch.m_color = treePart.m_color;
				auto cPLength = glm::distance(branch.m_bezierCurve.m_p0, branch.m_bezierCurve.m_p3) * 0.3f;
				branch.m_bezierCurve.m_p1 =
					glm::normalize(inBranch["Start Dir"].as<glm::vec3>()) * cPLength + branch.m_bezierCurve.m_p0;
				branch.m_bezierCurve.m_p2 =
					branch.m_bezierCurve.m_p3 - glm::normalize(inBranch["End Dir"].as<glm::vec3>()) * cPLength;
				if (glm::any(glm::isnan(branch.m_bezierCurve.m_p1)))
				{
					branch.m_bezierCurve.m_p1 = glm::mix(branch.m_bezierCurve.m_p0, branch.m_bezierCurve.m_p3, 0.25f);
				}
				if (glm::any(glm::isnan(branch.m_bezierCurve.m_p2)))
				{
					branch.m_bezierCurve.m_p2 = glm::mix(branch.m_bezierCurve.m_p0, branch.m_bezierCurve.m_p3, 0.75f);
				}
				/*
				branch.m_bezierCurve.m_p1 = glm::mix(branch.m_bezierCurve.m_p0, branch.m_bezierCurve.m_p3, 0.3f);
				branch.m_bezierCurve.m_p2 = glm::mix(branch.m_bezierCurve.m_p0, branch.m_bezierCurve.m_p3, 0.7f);
				*/
				branch.m_startThickness = inBranch["Start Radius"].as<float>() * scaleFactor;
				branch.m_endThickness = inBranch["End Radius"].as<float>() * scaleFactor;
				branch.m_handle = m_scannedBranches.size() - 1;
				treePart.m_branchHandles.emplace_back(branch.m_handle);
				branch.m_treePartHandle = treePart.m_handle;
				if (branch.m_bezierCurve.m_p0.y >= branch.m_bezierCurve.m_p3.y) {
					auto p0 = branch.m_bezierCurve.m_p3;
					branch.m_bezierCurve.m_p3 = branch.m_bezierCurve.m_p0;
					branch.m_bezierCurve.m_p0 = p0;
					auto p1 = branch.m_bezierCurve.m_p2;
					branch.m_bezierCurve.m_p2 = branch.m_bezierCurve.m_p1;
					branch.m_bezierCurve.m_p1 = p1;
					auto startT = branch.m_startThickness;
					branch.m_startThickness = branch.m_endThickness;
					branch.m_endThickness = startT;
				}
				minHeight = glm::min(minHeight, branch.m_bezierCurve.m_p0.y);
				minHeight = glm::min(minHeight, branch.m_bezierCurve.m_p3.y);
			}
			for (const auto& inAllocatedPoint : inTreeParts["Allocated Points"]) {
				auto& allocatedPoint = m_allocatedPoints.emplace_back();
				allocatedPoint.m_color = treePart.m_color;
				allocatedPoint.m_position = inAllocatedPoint.as<glm::vec3>() * scaleFactor;
				allocatedPoint.m_handle = m_allocatedPoints.size() - 1;
				allocatedPoint.m_treePartHandle = treePart.m_handle;
				allocatedPoint.m_branchHandle = -1;
				treePart.m_allocatedPoints.emplace_back(allocatedPoint.m_handle);
			}
		}
		for (auto& scatterPoint : m_scatteredPoints) {
			scatterPoint.m_position.y -= minHeight;

			m_min = glm::min(m_min, scatterPoint.m_position);
			m_max = glm::max(m_max, scatterPoint.m_position);
		}
		for (auto& allocatedPoint : m_allocatedPoints) {
			allocatedPoint.m_position.y -= minHeight;

			m_min = glm::min(m_min, allocatedPoint.m_position);
			m_max = glm::max(m_max, allocatedPoint.m_position);
		}
		for (auto& scannedBranch : m_scannedBranches) {
			scannedBranch.m_bezierCurve.m_p0.y -= minHeight;
			scannedBranch.m_bezierCurve.m_p1.y -= minHeight;
			scannedBranch.m_bezierCurve.m_p2.y -= minHeight;
			scannedBranch.m_bezierCurve.m_p3.y -= minHeight;

			m_min = glm::min(m_min, scannedBranch.m_bezierCurve.m_p0);
			m_max = glm::max(m_max, scannedBranch.m_bezierCurve.m_p0);
			m_min = glm::min(m_min, scannedBranch.m_bezierCurve.m_p3);
			m_max = glm::max(m_max, scannedBranch.m_bezierCurve.m_p3);
		}

		auto center = (m_min + m_max) / 2.0f;
		auto newMin = center + (m_min - center) * 1.25f;
		auto newMax = center + (m_max - center) * 1.25f;
		m_min = newMin;
		m_max = newMax;
	}
	catch (std::exception e) {
		EVOENGINE_ERROR("Failed to load!");
	}
}

void TreePointCloud::ExportForestOBJ(const std::filesystem::path& path, const TreeMeshGeneratorSettings& meshGeneratorSettings)
{
	if (path.extension() == ".obj") {
		std::ofstream of;
		of.open(path.string(), std::ofstream::out | std::ofstream::trunc);
		if (of.is_open()) {
			std::string start = "#Forest OBJ exporter, by Bosheng Li";
			start += "\n";
			of.write(start.c_str(), start.size());
			of.flush();
			unsigned startIndex = 1;
			const auto branchMeshes = GenerateBranchMeshes(meshGeneratorSettings);
			if(!branchMeshes.empty()){
				
				unsigned treeIndex = 0;
				for (auto& mesh : branchMeshes) {
					auto& vertices = mesh->UnsafeGetVertices();
					auto& triangles = mesh->UnsafeGetTriangles();
					if (!vertices.empty() && !triangles.empty()) {
						std::string header =
							"#Vertices: " + std::to_string(vertices.size()) +
							", tris: " + std::to_string(triangles.size());
						header += "\n";
						of.write(header.c_str(), header.size());
						of.flush();
						std::stringstream data;
						data << "o tree " + std::to_string(treeIndex) + "\n";
#pragma region Data collection
						for (auto i = 0; i < vertices.size(); i++) {
							auto& vertexPosition = vertices.at(i).m_position;
							auto& color = vertices.at(i).m_color;
							data << "v " + std::to_string(vertexPosition.x) + " " +
								std::to_string(vertexPosition.y) + " " +
								std::to_string(vertexPosition.z) + " " +
								std::to_string(color.x) + " " + std::to_string(color.y) + " " +
								std::to_string(color.z) + "\n";
						}
						for (const auto& vertex : vertices) {
							data << "vt " + std::to_string(vertex.m_texCoord.x) + " " +
								std::to_string(vertex.m_texCoord.y) + "\n";
						}
						// data += "s off\n";
						data << "# List of indices for faces vertices, with (x, y, z).\n";
						for (auto i = 0; i < triangles.size(); i++) {
							const auto triangle = triangles[i];
							const auto f1 = triangle.x + startIndex;
							const auto f2 = triangle.y + startIndex;
							const auto f3 = triangle.z + startIndex;
							data << "f " + std::to_string(f1) + "/" + std::to_string(f1) + "/" +
								std::to_string(f1) + " " + std::to_string(f2) + "/" +
								std::to_string(f2) + "/" + std::to_string(f2) + " " +
								std::to_string(f3) + "/" + std::to_string(f3) + "/" +
								std::to_string(f3) + "\n";
						}
#pragma endregion
						const auto result = data.str();
						of.write(result.c_str(), result.size());
						of.flush();
						startIndex += vertices.size();
						treeIndex++;
					}
				}
			}
			const auto foliageMeshes = GenerateFoliageMeshes(meshGeneratorSettings);
			if(!foliageMeshes.empty()) {
				
				unsigned treeIndex = 0;
				for (auto& mesh : foliageMeshes) {
					auto& vertices = mesh->UnsafeGetVertices();
					auto& triangles = mesh->UnsafeGetTriangles();
					if (!vertices.empty() && !triangles.empty()) {
						std::string header =
							"#Vertices: " + std::to_string(vertices.size()) +
							", tris: " + std::to_string(triangles.size());
						header += "\n";
						of.write(header.c_str(), header.size());
						of.flush();
						std::stringstream data;
						data << "o tree " + std::to_string(treeIndex) + "\n";
#pragma region Data collection
						for (auto i = 0; i < vertices.size(); i++) {
							auto& vertexPosition = vertices.at(i).m_position;
							auto& color = vertices.at(i).m_color;
							data << "v " + std::to_string(vertexPosition.x) + " " +
								std::to_string(vertexPosition.y) + " " +
								std::to_string(vertexPosition.z) + " " +
								std::to_string(color.x) + " " + std::to_string(color.y) + " " +
								std::to_string(color.z) + "\n";
						}
						for (const auto& vertex : vertices) {
							data << "vt " + std::to_string(vertex.m_texCoord.x) + " " +
								std::to_string(vertex.m_texCoord.y) + "\n";
						}
						// data += "s off\n";
						data << "# List of indices for faces vertices, with (x, y, z).\n";
						for (auto i = 0; i < triangles.size(); i++) {
							const auto triangle = triangles[i];
							const auto f1 = triangle.x + startIndex;
							const auto f2 = triangle.y + startIndex;
							const auto f3 = triangle.z + startIndex;
							data << "f " + std::to_string(f1) + "/" + std::to_string(f1) + "/" +
								std::to_string(f1) + " " + std::to_string(f2) + "/" +
								std::to_string(f2) + "/" + std::to_string(f2) + " " +
								std::to_string(f3) + "/" + std::to_string(f3) + "/" +
								std::to_string(f3) + "\n";
						}
#pragma endregion
						const auto result = data.str();
						of.write(result.c_str(), result.size());
						of.flush();
						startIndex += vertices.size();
						treeIndex++;
					}
				}
			}
			of.close();
		}
	}
}

void TreePointCloud::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
	static Handle previousHandle = 0;
	static std::shared_ptr<ParticleInfoList> allocatedPointInfoList;
	static std::shared_ptr<ParticleInfoList> scatterPointInfoList;
	static std::shared_ptr<ParticleInfoList> nodeInfoList;

	static std::shared_ptr<ParticleInfoList> scatteredPointConnectionInfoList;
	static std::vector<glm::vec3> scatteredPointConnectionsStarts;
	static std::vector<glm::vec3> scatteredPointConnectionsEnds;
	static std::vector<glm::vec4> scatteredPointConnectionColors;

	static std::shared_ptr<ParticleInfoList> branchConnectionInfoList;
	static std::vector<glm::vec3> branchConnectionStarts;
	static std::vector<glm::vec3> branchConnectionEnds;
	static std::vector<glm::vec4> branchConnectionColors;

	static std::shared_ptr<ParticleInfoList> filteredBranchConnectionInfoList;
	static std::vector<glm::vec3> filteredBranchConnectionStarts;
	static std::vector<glm::vec3> filteredBranchConnectionEnds;
	static std::vector<glm::vec4> filteredBranchConnectionColors;

	static std::shared_ptr<ParticleInfoList> scatterPointToBranchConnectionInfoList;
	static std::vector<glm::vec3> scatterPointToBranchConnectionStarts;
	static std::vector<glm::vec3> scatterPointToBranchConnectionEnds;
	static std::vector<glm::vec4> scatterPointToBranchConnectionColors;

	static std::shared_ptr<ParticleInfoList> scannedBranchConnectionInfoList;
	static std::vector<glm::vec3> scannedBranchConnectionStarts;
	static std::vector<glm::vec3> scannedBranchConnectionEnds;
	static std::vector<glm::vec4> scannedBranchConnectionColors;

	if (!branchConnectionInfoList) branchConnectionInfoList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
	if (!allocatedPointInfoList) allocatedPointInfoList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
	if (!scatterPointInfoList) scatterPointInfoList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
	if (!nodeInfoList) nodeInfoList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
	if (!scatteredPointConnectionInfoList) scatteredPointConnectionInfoList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
	if (!filteredBranchConnectionInfoList) filteredBranchConnectionInfoList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
	if (!scatterPointToBranchConnectionInfoList) scatterPointToBranchConnectionInfoList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
	if (!scannedBranchConnectionInfoList) scannedBranchConnectionInfoList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();

	auto& allocatedPointMatrices = allocatedPointInfoList->m_particleInfos;
	auto& scatterPointMatrices = scatterPointInfoList->m_particleInfos;
	auto& nodeMatrices = nodeInfoList->m_particleInfos;

	static bool enableDebugRendering = true;

	static bool drawAllocatedPoints = true;
	static bool drawScannedBranches = true;
	static bool drawScatteredPoints = true;
	static bool drawScatteredPointConnections = false;
	static bool drawBranchConnections = false;
	static bool drawFilteredConnections = true;
	static bool drawScatterPointToBranchConnections = false;
	static bool drawNode = true;
	static float scannedBranchWidth = 0.005f;
	static float connectionWidth = 0.001f;
	static float pointSize = 0.002f;

	static float nodeSize = 1.0f;
	bool refreshData = false;

	static int colorMode = 0;


	static glm::vec4 scatterPointToBranchConnectionColor = glm::vec4(1, 0, 1, 1);
	static glm::vec4 scatterPointColor = glm::vec4(0, 1, 0, 1);
	static glm::vec4 scatteredPointConnectionColor = glm::vec4(1, 1, 1, 1);
	static glm::vec4 branchConnectionColor = glm::vec4(1, 0, 0, 1);
	static glm::vec4 filteredBranchConnectionColor = glm::vec4(1, 1, 0, 1);

	static float importScale = 0.1f;
	ImGui::DragFloat("Import scale", &importScale, 0.01f, 0.01f, 10.0f);
	FileUtils::OpenFile("Load YAML", "YAML", { ".yml" }, [&](const std::filesystem::path& path) {
		ImportGraph(path, importScale);
		refreshData = true;
		}, false);

	if (!m_scatteredPoints.empty()) {
		static ConnectivityGraphSettings connectivityGraphSettings;
		if (ImGui::TreeNodeEx("Graph Settings")) {
			connectivityGraphSettings.OnInspect();
			ImGui::TreePop();
		}
		static ReconstructionSettings reconstructionSettings;
		if (ImGui::TreeNodeEx("Reconstruction Settings")) {
			reconstructionSettings.OnInspect();
			ImGui::TreePop();
		}
		if (ImGui::Button("Build Skeleton")) {
			EstablishConnectivityGraph(connectivityGraphSettings);
			BuildSkeletons(reconstructionSettings);
			refreshData = true;
		}
		static TreeMeshGeneratorSettings meshGeneratorSettings;
		meshGeneratorSettings.OnInspect(editorLayer);
		if (ImGui::Button("Form tree mesh")) {
			if (m_filteredBranchConnections.empty()) {
				m_skeletons.clear();
				EstablishConnectivityGraph(connectivityGraphSettings);
			}
			if (m_skeletons.empty()) BuildSkeletons(reconstructionSettings);
			FormGeometryEntity(meshGeneratorSettings);
		}
		FileUtils::SaveFile("Export all trees as OBJ", "OBJ", { ".obj" }, [&](const std::filesystem::path& path) {
			ExportForestOBJ(path, meshGeneratorSettings);
			}, false);
		if (ImGui::Button("Refresh Data")) {
			refreshData = true;
		}
	}

	ImGui::Checkbox("Debug Rendering", &enableDebugRendering);
	if (enableDebugRendering) {
		GizmoSettings gizmoSettings;
		if (ImGui::Combo("Color mode", { "TreePart", "Branch", "Node" }, colorMode)) refreshData = true;
		if (ImGui::TreeNode("Render settings")) {
			ImGui::DragFloat("Branch width", &scannedBranchWidth, 0.0001f, 0.0001f, 1.0f, "%.4f");
			ImGui::DragFloat("Connection width", &connectionWidth, 0.0001f, 0.0001f, 1.0f, "%.4f");
			ImGui::DragFloat("Point size", &pointSize, 0.0001f, 0.0001f, 1.0f, "%.4f");
			ImGui::DragFloat("Node size", &nodeSize, 0.01f, 0.1f, 10.0f);

			ImGui::Checkbox("Render allocated point", &drawAllocatedPoints);
			ImGui::Checkbox("Render scattered point", &drawScatteredPoints);
			ImGui::Checkbox("Render scatter point connections", &drawScatteredPointConnections);
			ImGui::Checkbox("Render branch", &drawScannedBranches);
			ImGui::Checkbox("Render all junction connections", &drawBranchConnections);
			ImGui::Checkbox("Render filtered junction connections", &drawFilteredConnections);
			ImGui::Checkbox("Render scatter point to branch connections", &drawScatterPointToBranchConnections);
			ImGui::Checkbox("Render nodes", &drawNode);
			ImGui::ColorEdit4("Scatter Point Color", &scatterPointColor.x);
			if (drawScatteredPoints) ImGui::ColorEdit4("Scatter Connection Color", &scatteredPointConnectionColor.x);
			if (drawBranchConnections) ImGui::ColorEdit4("Junction Connection Color", &branchConnectionColor.x);
			if (drawFilteredConnections)
				ImGui::ColorEdit4("Filtered Scatter Connection Color", &filteredBranchConnectionColor.x);
			if (drawScatterPointToBranchConnections)
				ImGui::ColorEdit4("Scattered point to branch connection color", &scatterPointToBranchConnectionColor.x);

			gizmoSettings.m_drawSettings.OnInspect();

			ImGui::TreePop();
		}
		if (GetHandle() != previousHandle) refreshData = true;

		if (refreshData) {
			previousHandle = GetHandle();
			const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();

			allocatedPointMatrices.resize(m_allocatedPoints.size());

			scannedBranchConnectionStarts.resize(m_scannedBranches.size());
			scannedBranchConnectionEnds.resize(m_scannedBranches.size());
			scannedBranchConnectionColors.resize(m_scannedBranches.size());

			allocatedPointInfoList->SetPendingUpdate();
			scatterPointInfoList->SetPendingUpdate();
			nodeInfoList->SetPendingUpdate();

			nodeMatrices.clear();
			switch (colorMode) {
			case 0: {
				//TreePart
				for (int i = 0; i < m_allocatedPoints.size(); i++) {
					allocatedPointMatrices[i].m_instanceMatrix.m_value =
						glm::translate(m_allocatedPoints[i].m_position) * glm::scale(glm::vec3(1.0f));
					allocatedPointMatrices[i].m_instanceColor = glm::vec4(m_allocatedPoints[i].m_color, 1.0f);
				}

				for (int i = 0; i < m_scannedBranches.size(); i++) {
					scannedBranchConnectionStarts[i] = m_scannedBranches[i].m_bezierCurve.m_p0;
					scannedBranchConnectionEnds[i] = m_scannedBranches[i].m_bezierCurve.m_p3;
					scannedBranchConnectionColors[i] = glm::vec4(m_scannedBranches[i].m_color, 1.0f);
				}
				scannedBranchConnectionInfoList->ApplyConnections(scannedBranchConnectionStarts, scannedBranchConnectionEnds, scannedBranchConnectionColors, scannedBranchWidth);
				for (const auto& skeleton : m_skeletons) {
					const auto& nodeList = skeleton.RefSortedNodeList();
					auto startIndex = nodeMatrices.size();
					nodeMatrices.resize(startIndex + nodeList.size());
					for (int i = 0; i < nodeList.size(); i++) {
						const auto& node = skeleton.PeekNode(nodeList[i]);
						nodeMatrices[startIndex + i].m_instanceMatrix.m_value = glm::translate(node.m_info.m_globalPosition) *
							glm::scale(glm::vec3(node.m_info.m_thickness));
						nodeMatrices[startIndex + i].m_instanceColor = glm::vec4(1.0f);
					}
				}
			}
				  break;
			case 1: {
				//Branch
				for (int i = 0; i < m_allocatedPoints.size(); i++) {
					allocatedPointMatrices[i].m_instanceMatrix.m_value =
						glm::translate(m_allocatedPoints[i].m_position) * glm::scale(glm::vec3(1.0f));
					if (m_allocatedPoints[i].m_branchHandle >= 0) {
						allocatedPointMatrices[i].m_instanceColor = glm::vec4(
							ecoSysLabLayer->RandomColors()[m_allocatedPoints[i].m_branchHandle], 1.0f);
					}
					else {
						allocatedPointMatrices[i].m_instanceColor = glm::vec4(
							ecoSysLabLayer->RandomColors()[m_allocatedPoints[i].m_treePartHandle], 1.0f);
					}
				}

				for (int i = 0; i < m_scannedBranches.size(); i++) {
					scannedBranchConnectionStarts[i] = m_scannedBranches[i].m_bezierCurve.m_p0;
					scannedBranchConnectionEnds[i] = m_scannedBranches[i].m_bezierCurve.m_p3;
					scannedBranchConnectionColors[i] = glm::vec4(
						ecoSysLabLayer->RandomColors()[m_scannedBranches[i].m_handle], 1.0f);
				}
				scannedBranchConnectionInfoList->ApplyConnections(scannedBranchConnectionStarts, scannedBranchConnectionEnds, scannedBranchConnectionColors, scannedBranchWidth);

				for (const auto& skeleton : m_skeletons) {
					const auto& nodeList = skeleton.RefSortedNodeList();
					auto startIndex = nodeMatrices.size();
					nodeMatrices.resize(startIndex + nodeList.size());
					for (int i = 0; i < nodeList.size(); i++) {
						const auto& node = skeleton.PeekNode(nodeList[i]);
						nodeMatrices[startIndex + i].m_instanceMatrix.m_value = glm::translate(node.m_info.m_globalPosition) *
							glm::scale(glm::vec3(node.m_info.m_thickness));
						nodeMatrices[startIndex + i].m_instanceColor = glm::vec4(
							ecoSysLabLayer->RandomColors()[node.m_data.m_branchHandle],
							1.0f);
					}
				}
			}
				  break;
			case 2: {
				//Node
				for (int i = 0; i < m_allocatedPoints.size(); i++) {
					allocatedPointMatrices[i].m_instanceMatrix.m_value =
						glm::translate(m_allocatedPoints[i].m_position) * glm::scale(glm::vec3(1.0f));
					if (m_allocatedPoints[i].m_nodeHandle >= 0) {
						allocatedPointMatrices[i].m_instanceColor = glm::vec4(
							ecoSysLabLayer->RandomColors()[m_allocatedPoints[i].m_nodeHandle], 1.0f);
					}
					else {
						allocatedPointMatrices[i].m_instanceColor = glm::vec4(
							ecoSysLabLayer->RandomColors()[m_allocatedPoints[i].m_treePartHandle], 1.0f);
					}
				}

				for (int i = 0; i < m_scannedBranches.size(); i++) {
					scannedBranchConnectionStarts[i] = m_scannedBranches[i].m_bezierCurve.m_p0;
					scannedBranchConnectionEnds[i] = m_scannedBranches[i].m_bezierCurve.m_p3;
					scannedBranchConnectionColors[i] = glm::vec4(1.0f);
				}
				scannedBranchConnectionInfoList->ApplyConnections(scannedBranchConnectionStarts, scannedBranchConnectionEnds, scannedBranchConnectionColors, scannedBranchWidth);
				for (const auto& skeleton : m_skeletons) {
					const auto& nodeList = skeleton.RefSortedNodeList();
					auto startIndex = nodeMatrices.size();
					nodeMatrices.resize(startIndex + nodeList.size());
					for (int i = 0; i < nodeList.size(); i++) {
						const auto& node = skeleton.PeekNode(nodeList[i]);
						nodeMatrices[startIndex + i].m_instanceMatrix.m_value = glm::translate(node.m_info.m_globalPosition) *
							glm::scale(glm::vec3(node.m_info.m_thickness));
						nodeMatrices[startIndex + i].m_instanceColor = glm::vec4(ecoSysLabLayer->RandomColors()[node.GetHandle()],
							1.0f);
					}
				}
			}
				  break;
			}

			scatterPointMatrices.resize(m_scatteredPoints.size());
			for (int i = 0; i < m_scatteredPoints.size(); i++) {
				scatterPointMatrices[i].m_instanceMatrix.m_value = glm::translate(m_scatteredPoints[i].m_position) * glm::scale(glm::vec3(1.0f));
				scatterPointMatrices[i].m_instanceColor = scatterPointColor;
			}

			scatteredPointConnectionsStarts.resize(m_scatterPointsConnections.size());
			scatteredPointConnectionsEnds.resize(m_scatterPointsConnections.size());
			scatteredPointConnectionColors.resize(m_scatterPointsConnections.size());
			for (int i = 0; i < m_scatterPointsConnections.size(); i++) {
				scatteredPointConnectionsStarts[i] = m_scatterPointsConnections[i].first;
				scatteredPointConnectionsEnds[i] = m_scatterPointsConnections[i].second;
				scatteredPointConnectionColors[i] = scatterPointToBranchConnectionColor;
			}
			scatteredPointConnectionInfoList->ApplyConnections(
				scatteredPointConnectionsStarts,
				scatteredPointConnectionsEnds,
				scatteredPointConnectionColors, connectionWidth
			);
			branchConnectionStarts.resize(m_branchConnections.size());
			branchConnectionEnds.resize(m_branchConnections.size());
			branchConnectionColors.resize(m_branchConnections.size());
			for (int i = 0; i < m_branchConnections.size(); i++) {
				branchConnectionStarts[i] = m_branchConnections[i].first;
				branchConnectionEnds[i] = m_branchConnections[i].second;
				branchConnectionColors[i] = branchConnectionColor;
			}
			branchConnectionInfoList->ApplyConnections(
				branchConnectionStarts,
				branchConnectionEnds,
				branchConnectionColors, connectionWidth * 1.5f
			);

			filteredBranchConnectionStarts.resize(m_filteredBranchConnections.size());
			filteredBranchConnectionEnds.resize(m_filteredBranchConnections.size());
			filteredBranchConnectionColors.resize(m_filteredBranchConnections.size());
			for (int i = 0; i < m_filteredBranchConnections.size(); i++) {
				filteredBranchConnectionStarts[i] = m_filteredBranchConnections[i].first;
				filteredBranchConnectionEnds[i] = m_filteredBranchConnections[i].second;
				filteredBranchConnectionColors[i] = filteredBranchConnectionColor;
			}
			filteredBranchConnectionInfoList->ApplyConnections(
				filteredBranchConnectionStarts,
				filteredBranchConnectionEnds,
				filteredBranchConnectionColors, connectionWidth * 2.0f
			);

			scatterPointToBranchConnectionStarts.resize(
				m_scatterPointToBranchStartConnections.size() + m_scatterPointToBranchEndConnections.size());
			scatterPointToBranchConnectionEnds.resize(
				m_scatterPointToBranchStartConnections.size() + m_scatterPointToBranchEndConnections.size());
			scatterPointToBranchConnectionColors.resize(
				m_scatterPointToBranchStartConnections.size() + m_scatterPointToBranchEndConnections.size());
			for (int i = 0; i < m_scatterPointToBranchStartConnections.size(); i++) {
				scatterPointToBranchConnectionStarts[i] = m_scatterPointToBranchStartConnections[i].first;
				scatterPointToBranchConnectionEnds[i] = m_scatterPointToBranchStartConnections[i].second;
				scatterPointToBranchConnectionColors[i] = scatterPointToBranchConnectionColor;
			}
			for (int i = m_scatterPointToBranchStartConnections.size();
				i < m_scatterPointToBranchStartConnections.size() + m_scatterPointToBranchEndConnections.size(); i++) {
				scatterPointToBranchConnectionStarts[i] = m_scatterPointToBranchEndConnections[i -
					m_scatterPointToBranchStartConnections.size()].first;
				scatterPointToBranchConnectionEnds[i] = m_scatterPointToBranchEndConnections[i -
					m_scatterPointToBranchStartConnections.size()].second;
			}
			scatterPointToBranchConnectionInfoList->ApplyConnections(
				scatterPointToBranchConnectionStarts,
				scatterPointToBranchConnectionEnds,
				scatterPointToBranchConnectionColors, connectionWidth
			);
		}
		if (!scatterPointMatrices.empty()) {
			if (drawScatteredPoints) {
				editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CUBE"),
					scatterPointInfoList,
					glm::mat4(1.0f),
					pointSize, gizmoSettings);
			}
			if (drawAllocatedPoints) {
				editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CUBE"),
					allocatedPointInfoList,
					glm::mat4(1.0f),
					pointSize, gizmoSettings);
			}
			if (drawScannedBranches)
				editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CONE"), scannedBranchConnectionInfoList, glm::mat4(1.0f), 1.0f, gizmoSettings);
			if (drawScatteredPointConnections)
				editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER"), scatteredPointConnectionInfoList, glm::mat4(1.0f), 1.0f, gizmoSettings);
			if (drawBranchConnections)
				editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER"), branchConnectionInfoList, glm::mat4(1.0f), 1.0f, gizmoSettings);
			if (drawFilteredConnections)
				editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER"), filteredBranchConnectionInfoList, glm::mat4(1.0f), 1.0f, gizmoSettings);
			if (drawScatterPointToBranchConnections)
				editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER"), scatterPointToBranchConnectionInfoList, glm::mat4(1.0f), 1.0f, gizmoSettings);
			if (drawNode) {
				editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CUBE"), nodeInfoList,
					glm::mat4(1.0f), nodeSize, gizmoSettings);
			}
		}
	}
}

void TreePointCloud::EstablishConnectivityGraph(const ConnectivityGraphSettings& settings) {
	VoxelGrid<std::vector<ScatterPointData>> scatterPointsVoxelGrid;
	VoxelGrid<std::vector<BranchEndData>> branchEndsVoxelGrid;
	scatterPointsVoxelGrid.Initialize(2.0f * settings.m_edgeLength, m_min, m_max);
	branchEndsVoxelGrid.Initialize(2.0f * settings.m_edgeLength, m_min, m_max);
	for (auto& point : m_allocatedPoints) {
		point.m_branchHandle = point.m_nodeHandle = point.m_skeletonIndex = -1;
	}
	for (auto& point : m_scatteredPoints) {
		point.m_neighborScatterPoints.clear();
		point.m_neighborBranchP3s.clear();

		ScatterPointData voxel;
		voxel.m_pointHandle = point.m_handle;
		voxel.m_position = point.m_position;
		scatterPointsVoxelGrid.Ref(point.m_position).emplace_back(voxel);
	}

	for (auto& scannedBranch : m_scannedBranches) {
		scannedBranch.m_neighborScatterPointP0.clear();
		scannedBranch.m_neighborBranchP3.clear();
		scannedBranch.m_parentHandle = -1;
		scannedBranch.m_childHandles.clear();


		BranchEndData voxel;
		voxel.m_branchHandle = scannedBranch.m_handle;
		auto shortenedP0 = scannedBranch.m_bezierCurve.GetPoint(settings.m_branchShortening);
		auto shortenedP3 = scannedBranch.m_bezierCurve.GetPoint(1.0f - settings.m_branchShortening);
		voxel.m_position = shortenedP0;
		voxel.m_isP0 = true;
		branchEndsVoxelGrid.Ref(shortenedP0).emplace_back(voxel);
		voxel.m_position = shortenedP3;
		voxel.m_isP0 = false;
		branchEndsVoxelGrid.Ref(shortenedP3).emplace_back(voxel);
	}
	m_scatterPointsConnections.clear();
	m_filteredBranchConnections.clear();
	m_branchConnections.clear();
	m_scatterPointToBranchStartConnections.clear();
	m_scatterPointToBranchEndConnections.clear();
	//We establish connection between any 2 scatter points.
	for (auto& point : m_scatteredPoints) {
		if (m_scatterPointsConnections.size() > 1000000) {
			EVOENGINE_ERROR("Too much connections!");
			return;
		}
		FindPoints(point.m_position, scatterPointsVoxelGrid, settings.m_scatterPointsConnectionMaxLength,
			[&](const ScatterPointData& voxel) {
				if (voxel.m_pointHandle == point.m_handle) return;
				for (const auto& neighbor : point.m_neighborScatterPoints) {
					if (voxel.m_pointHandle == neighbor) return;
				}
				auto& otherPoint = m_scatteredPoints[voxel.m_pointHandle];
				point.m_neighborScatterPoints.emplace_back(voxel.m_pointHandle);
				otherPoint.m_neighborScatterPoints.emplace_back(point.m_handle);
				m_scatterPointsConnections.emplace_back(point.m_position, otherPoint.m_position);
			});
	}

	for (auto& branch : m_scannedBranches) {
		float currentEdgeLength = settings.m_edgeLength;
		auto shortenedP0 = branch.m_bezierCurve.GetPoint(settings.m_branchShortening);
		auto shortenedP3 = branch.m_bezierCurve.GetPoint(1.0f - settings.m_branchShortening);
		float branchLength = glm::distance(shortenedP0, shortenedP3);
		//We find scatter points close to the branch p0.
		FindPoints(shortenedP0, scatterPointsVoxelGrid, settings.m_scatterPointBranchConnectionMaxLength,
			[&](const ScatterPointData& voxel) {
				for (const auto& i : branch.m_neighborScatterPointP0) {
					if (i.second == voxel.m_pointHandle) return;
				}
				branch.m_neighborScatterPointP0.emplace_back(glm::distance(branch.m_bezierCurve.m_p0, voxel.m_position), voxel.m_pointHandle);
				m_scatterPointToBranchStartConnections.emplace_back(branch.m_bezierCurve.m_p0,
					voxel.m_position);
			});
		//We find branch p3 close to the scatter point.
		FindPoints(shortenedP3, scatterPointsVoxelGrid, settings.m_scatterPointBranchConnectionMaxLength,
			[&](const ScatterPointData& voxel) {
				auto& otherPoint = m_scatteredPoints[voxel.m_pointHandle];
				for (const auto& i : otherPoint.m_neighborBranchP3s) {
					if (i.second == branch.m_handle) return;
				}
				otherPoint.m_neighborBranchP3s.emplace_back(glm::distance(branch.m_bezierCurve.m_p3, voxel.m_position), branch.m_handle);
				m_scatterPointToBranchEndConnections.emplace_back(branch.m_bezierCurve.m_p3,
					voxel.m_position);
			});
		//Connect P3 from other branch to this branch's P0
		bool findBranchP3 = false;
		int timeout = 0;
		while (!findBranchP3 && timeout < settings.m_maxTimeout) {
			ForEachBranchEnd(shortenedP0, branchEndsVoxelGrid, currentEdgeLength,
				[&](const BranchEndData& voxel) {
					if (voxel.m_isP0) return;
					const auto dotP = glm::dot(glm::normalize(voxel.m_position - shortenedP0),
						glm::normalize(shortenedP0 - shortenedP3));
					if (dotP < glm::cos(glm::radians(settings.m_angleLimit))) return;
					const auto distance = glm::distance(voxel.m_position, shortenedP0);
					if (distance >
						settings.m_forceConnectionRatio * branchLength &&
						dotP < glm::cos(glm::radians(settings.m_forceConnectionAngleLimit)))
						return;
					for (const auto& i : branch.m_neighborBranchP3) {
						if (i.first == voxel.m_branchHandle) return;
					}
					findBranchP3 = true;
					auto& otherBranch = m_scannedBranches[voxel.m_branchHandle];
					const auto search = branch.m_neighborBranchP3.find(voxel.m_branchHandle);
					if (search != branch.m_neighborBranchP3.end() && search->second > distance)
					{
						branch.m_neighborBranchP3.at(voxel.m_branchHandle) = distance;
					}
					else
					{
						branch.m_neighborBranchP3[voxel.m_branchHandle] = distance;
					}
					m_branchConnections.emplace_back(branch.m_bezierCurve.m_p0,
						otherBranch.m_bezierCurve.m_p3);

				});
			currentEdgeLength += settings.m_edgeExtendStep;
			timeout++;
		}
		//Connect P0 from other branch to this branch's P3
		bool findBranchP0 = false;
		currentEdgeLength = settings.m_edgeLength;
		timeout = 0;
		while (!findBranchP0 && timeout < settings.m_maxTimeout) {
			ForEachBranchEnd(shortenedP3, branchEndsVoxelGrid, currentEdgeLength,
				[&](const BranchEndData& voxel) {
					if (!voxel.m_isP0) return;
					const auto dotP = glm::dot(glm::normalize(voxel.m_position - shortenedP3),
						glm::normalize(shortenedP3 - shortenedP0));
					if (dotP < glm::cos(glm::radians(settings.m_angleLimit)))
						return;
					const auto distance = glm::distance(voxel.m_position, shortenedP3);
					if (glm::distance(voxel.m_position, shortenedP3) >
						settings.m_forceConnectionRatio * branchLength &&
						dotP < glm::cos(glm::radians(settings.m_forceConnectionAngleLimit)))
						return;
					auto& otherBranch = m_scannedBranches[voxel.m_branchHandle];
					for (const auto& i : otherBranch.m_neighborBranchP3) {
						if (i.first == branch.m_handle) return;
					}
					findBranchP0 = true;
					const auto search = otherBranch.m_neighborBranchP3.find(branch.m_handle);
					if (search != otherBranch.m_neighborBranchP3.end() && search->second > distance)
					{
						otherBranch.m_neighborBranchP3.at(branch.m_handle) = distance;
					}
					else
					{
						otherBranch.m_neighborBranchP3[branch.m_handle] = distance;
					}
					m_branchConnections.emplace_back(branch.m_bezierCurve.m_p3,
						otherBranch.m_bezierCurve.m_p0);

				});
			currentEdgeLength += settings.m_edgeExtendStep;
			timeout++;
		}
	}
	//We search branch connections via scatter points.
	for (auto& scannedBranch : m_scannedBranches) {
		std::unordered_set<PointHandle> visitedPoints;
		std::vector<PointHandle> processingPoints;
		float distanceL = FLT_MAX;
		for (const auto& i : scannedBranch.m_neighborScatterPointP0)
		{
			processingPoints.emplace_back(i.second);
			auto distance = glm::distance(scannedBranch.m_bezierCurve.GetPoint(settings.m_branchShortening), m_scatteredPoints[i.second].m_position);
			if (distance < distanceL) distanceL = distance;
		}
		for (const auto& i : processingPoints) {
			visitedPoints.emplace(i);
		}
		while (!processingPoints.empty()) {
			auto currentPointHandle = processingPoints.back();
			visitedPoints.emplace(currentPointHandle);
			processingPoints.pop_back();
			auto& currentPoint = m_scatteredPoints[currentPointHandle];
			for (const auto& neighborHandle : currentPoint.m_neighborScatterPoints) {
				if (visitedPoints.find(neighborHandle) != visitedPoints.end()) continue;
				auto& neighbor = m_scatteredPoints[neighborHandle];
				//We stop search if the point is junction point.
				for (const auto& branchInfo : neighbor.m_neighborBranchP3s) {
					bool skip = false;
					for (const auto& i : scannedBranch.m_neighborBranchP3) if (branchInfo.second == i.first) skip = true;
					auto& otherBranch = m_scannedBranches[branchInfo.second];
					auto shortenedP0 = scannedBranch.m_bezierCurve.GetPoint(settings.m_branchShortening);
					auto shortenedP3 = scannedBranch.m_bezierCurve.GetPoint(1.0f - settings.m_branchShortening);
					float branchLength = glm::distance(shortenedP0, shortenedP3);
					auto otherBranchShortenedP3 = otherBranch.m_bezierCurve.GetPoint(
						1.0f - settings.m_branchShortening);
					auto dotP = glm::dot(glm::normalize(otherBranchShortenedP3 - shortenedP0),
						glm::normalize(shortenedP0 - shortenedP3));
					if (dotP < glm::cos(glm::radians(settings.m_angleLimit))) skip = true;
					if (!skip) {
						float distance = distanceL + branchInfo.first;
						const auto search = scannedBranch.m_neighborBranchP3.find(branchInfo.second);
						if (search != scannedBranch.m_neighborBranchP3.end() && search->second > distance)
						{
							scannedBranch.m_neighborBranchP3.at(branchInfo.second) = distance;
						}
						else
						{
							scannedBranch.m_neighborBranchP3[branchInfo.second] = distance;
						}
						m_branchConnections.emplace_back(scannedBranch.m_bezierCurve.m_p0, otherBranch.m_bezierCurve.m_p3);
					}
				}
				processingPoints.emplace_back(neighborHandle);
			}
		}
	}
	for (auto& branch : m_scannedBranches) {
		//Detect connection to branch start.
		if (branch.m_neighborBranchP3.empty()) {
			continue;
		}
		std::map<float, BranchHandle> availableCandidates;
		for (const auto& branchEndHandle : branch.m_neighborBranchP3) {
			availableCandidates.emplace(branchEndHandle.second, branchEndHandle.first);
		}
		BranchHandle bestCandidate = availableCandidates.begin()->second;
		auto& candidate = m_scannedBranches[bestCandidate];
		candidate.m_childHandles.emplace_back(branch.m_handle);
		branch.m_parentHandle = bestCandidate;
		m_filteredBranchConnections.emplace_back(branch.m_bezierCurve.m_p0,
			candidate.m_bezierCurve.m_p3);
		candidate.m_neighborBranchP3.erase(branch.m_handle);
	}

}

void ApplyCurve(ReconstructionSkeleton& skeleton, OperatingBranch& branch) {
	const auto chainAmount = branch.m_chainNodeHandles.size();
	for (int i = 0; i < chainAmount; i++) {
		auto& node = skeleton.RefNode(branch.m_chainNodeHandles[i]);
		node.m_info.m_globalPosition = branch.m_bezierCurve.GetPoint(
			static_cast<float>(i) / chainAmount);
		node.m_info.m_length = glm::distance(
			branch.m_bezierCurve.GetPoint(static_cast<float>(i) / chainAmount),
			branch.m_bezierCurve.GetPoint(static_cast<float>(i + 1) / chainAmount));
		node.m_data.m_localPosition = glm::normalize(
			branch.m_bezierCurve.GetPoint(static_cast<float>(i + 1) / chainAmount) -
			branch.m_bezierCurve.GetPoint(static_cast<float>(i) / chainAmount));
		node.m_info.m_thickness = glm::mix(branch.m_startThickness, branch.m_endThickness,
			static_cast<float>(i) / chainAmount);
		node.m_data.m_branchHandle = branch.m_handle;
		node.m_info.m_color = glm::vec4(branch.m_color, 1.0f);
	}
}

void TreePointCloud::BuildSkeletons(const ReconstructionSettings& reconstructionSettings) {
	m_skeletons.clear();
	std::unordered_set<BranchHandle> allocatedBranchHandles;

	m_operatingBranches.resize(m_scannedBranches.size());
	for (int i = 0; i < m_scannedBranches.size(); i++) {
		m_operatingBranches[i].Apply(m_scannedBranches[i]);
	}

	auto& operatingBranches = m_operatingBranches;
	std::vector<BranchHandle> rootBranchHandles;
	for (auto& branch : operatingBranches) {
		branch.m_chainNodeHandles.clear();
		const auto branchStart = branch.m_bezierCurve.m_p0;
		auto shortenedP0 = branch.m_bezierCurve.GetPoint(reconstructionSettings.m_branchShortening);
		auto shortenedP3 = branch.m_bezierCurve.GetPoint(1.0f - reconstructionSettings.m_branchShortening);
		auto shortenedLength = glm::distance(shortenedP0, shortenedP3);
		if (branchStart.y <= reconstructionSettings.m_minHeight) {
			bool add = true;
			for (auto& rootBranchHandle : rootBranchHandles)
			{
				const auto& rootBranchStart = operatingBranches[rootBranchHandle].m_bezierCurve.m_p0;
				if (glm::distance(glm::vec2(rootBranchStart.x, rootBranchStart.z),
					glm::vec2(branchStart.x, branchStart.z)) < reconstructionSettings.m_maxTreeDistance) {
					add = false;
					if (rootBranchStart.y > branchStart.y)
					{
						rootBranchHandle = branch.m_handle;
						break;
					}
				}
			}
			if (add) rootBranchHandles.emplace_back(branch.m_handle);
		}
		else {
			branch.m_bezierCurve.m_p0 = shortenedP0;
		}
		branch.m_bezierCurve.m_p1 = shortenedP0 +
			branch.m_bezierCurve.GetAxis(reconstructionSettings.m_branchShortening) *
			shortenedLength * 0.25f;
		branch.m_bezierCurve.m_p2 = shortenedP3 -
			branch.m_bezierCurve.GetAxis(1.0f - reconstructionSettings.m_branchShortening) *
			shortenedLength * 0.25f;
		branch.m_bezierCurve.m_p3 = shortenedP3;
	}
	for (const auto& rootBranchHandle : rootBranchHandles) {
		allocatedBranchHandles.emplace(rootBranchHandle);
	}
	for (const auto& rootBranchHandle : rootBranchHandles) {
		auto& skeleton = m_skeletons.emplace_back();
		std::queue<BranchHandle> processingBranchHandles;
		processingBranchHandles.emplace(rootBranchHandle);
		while (!processingBranchHandles.empty()) {
			auto processingBranchHandle = processingBranchHandles.front();
			processingBranchHandles.pop();
			for (const auto& childBranchHandle : operatingBranches[processingBranchHandle].m_childHandles) {
				if (allocatedBranchHandles.find(childBranchHandle) != allocatedBranchHandles.end()) continue;
				processingBranchHandles.emplace(childBranchHandle);
			}
			bool onlyChild = true;
			NodeHandle prevNodeHandle = -1;
			if (operatingBranches[processingBranchHandle].m_handle == rootBranchHandle) {
				prevNodeHandle = 0;
				operatingBranches[processingBranchHandle].m_chainNodeHandles.emplace_back(0);
			}
			else {
				operatingBranches.emplace_back();
				auto& processingBranch = operatingBranches[processingBranchHandle];
				auto& connectionBranch = operatingBranches.back();
				auto& parentBranch = operatingBranches[processingBranch.m_parentHandle];
				connectionBranch.m_color = parentBranch.m_color;
				connectionBranch.m_handle = operatingBranches.size() - 1;
				connectionBranch.m_bezierCurve.m_p0 = parentBranch.m_bezierCurve.m_p3;
				connectionBranch.m_bezierCurve.m_p3 = processingBranch.m_bezierCurve.m_p0;
				auto connectionLength = glm::distance(connectionBranch.m_bezierCurve.m_p0,
					connectionBranch.m_bezierCurve.m_p3);
				/*
				connectionBranch.m_bezierCurve.m_p1 = parentBranch.m_bezierCurve.m_p3 +
																							(parentBranch.m_bezierCurve.m_p3 -
																							 parentBranch.m_bezierCurve.m_p2) * 0.1f * connectionLength;
				connectionBranch.m_bezierCurve.m_p2 = processingBranch.m_bezierCurve.m_p0 +
																							(processingBranch.m_bezierCurve.m_p0 -
																							 processingBranch.m_bezierCurve.m_p1) * 0.1f * connectionLength;
				*/
				connectionBranch.m_bezierCurve.m_p1 = glm::mix(connectionBranch.m_bezierCurve.m_p0,
					connectionBranch.m_bezierCurve.m_p3, 0.25f);
				connectionBranch.m_bezierCurve.m_p2 = glm::mix(connectionBranch.m_bezierCurve.m_p0,
					connectionBranch.m_bezierCurve.m_p3, 0.75f);
				connectionBranch.m_startThickness = parentBranch.m_endThickness;
				connectionBranch.m_endThickness = processingBranch.m_startThickness;
				connectionBranch.m_childHandles.emplace_back(processingBranch.m_handle);
				connectionBranch.m_parentHandle = processingBranch.m_parentHandle;
				processingBranch.m_parentHandle = connectionBranch.m_handle;
				for (int i = 0; i < parentBranch.m_childHandles.size(); i++) {
					if (parentBranch.m_childHandles[i] == processingBranch.m_handle) {
						parentBranch.m_childHandles[i] = connectionBranch.m_handle;
						break;
					}
				}
				if (parentBranch.m_childHandles.size() > 1) onlyChild = false;
				prevNodeHandle = parentBranch.m_chainNodeHandles.back();
				auto connectionFirstNodeHandle = skeleton.Extend(prevNodeHandle, !onlyChild);
				connectionBranch.m_chainNodeHandles.emplace_back(connectionFirstNodeHandle);
				prevNodeHandle = connectionFirstNodeHandle;

				float connectionChainLength = connectionBranch.m_bezierCurve.GetLength();
				int connectionChainAmount = glm::max(2, (int)(connectionChainLength /
					reconstructionSettings.m_internodeLength));
				for (int i = 1; i < connectionChainAmount; i++) {
					auto newNodeHandle = skeleton.Extend(prevNodeHandle, false);
					connectionBranch.m_chainNodeHandles.emplace_back(newNodeHandle);
					prevNodeHandle = newNodeHandle;
				}
				ApplyCurve(skeleton, connectionBranch);
				auto chainFirstNodeHandle = skeleton.Extend(prevNodeHandle, false);
				processingBranch.m_chainNodeHandles.emplace_back(chainFirstNodeHandle);
				prevNodeHandle = chainFirstNodeHandle;
			}
			auto& processingBranch = operatingBranches[processingBranchHandle];
			processingBranch.m_skeletonIndex = m_skeletons.size() - 1;
			float chainLength = processingBranch.m_bezierCurve.GetLength();
			int chainAmount = glm::max(2, (int)(chainLength /
				reconstructionSettings.m_internodeLength));
			for (int i = 1; i < chainAmount; i++) {
				auto newNodeHandle = skeleton.Extend(prevNodeHandle, false);
				processingBranch.m_chainNodeHandles.emplace_back(newNodeHandle);
				prevNodeHandle = newNodeHandle;
			}
			//assert(glm::isnan(processingBranch.m_bezierCurve.m_p0.x))
			ApplyCurve(skeleton, processingBranch);

		}
		skeleton.SortLists();
		auto& sortedNodeList = skeleton.RefSortedNodeList();
		auto& rootNode = skeleton.RefNode(0);
		rootNode.m_data.m_localRotation = glm::vec3(0.0f);
		rootNode.m_info.m_globalRotation = rootNode.m_info.m_regulatedGlobalRotation = glm::quatLookAt(
			rootNode.m_data.m_localPosition, glm::vec3(0, 0, -1));
		rootNode.m_data.m_localPosition = glm::vec3(0.0f);

		for (const auto& nodeHandle : sortedNodeList) {
			auto& node = skeleton.RefNode(nodeHandle);
			auto& nodeInfo = node.m_info;
			auto& nodeData = node.m_data;
			if (nodeHandle == 0) continue;

			auto& parentNode = skeleton.RefNode(node.GetParentHandle());
			auto front = glm::normalize(nodeData.m_localPosition);
			nodeData.m_localPosition = glm::vec3(0.0f);
			auto parentUp = parentNode.m_info.m_globalRotation * glm::vec3(0, 1, 0);
			auto regulatedUp = glm::normalize(glm::cross(glm::cross(front, parentUp), front));
			nodeInfo.m_globalRotation = glm::quatLookAt(front, regulatedUp);
			nodeData.m_localRotation = glm::inverse(parentNode.m_info.m_globalRotation) * nodeInfo.m_globalRotation;

		}
		if (reconstructionSettings.m_overrideThickness) {
			for (auto i = sortedNodeList.rbegin(); i != sortedNodeList.rend(); i++) {
				auto& node = skeleton.RefNode(*i);
				auto& nodeInfo = node.m_info;
				auto& childHandles = node.RefChildHandles();
				if (childHandles.empty())
				{
					nodeInfo.m_thickness = reconstructionSettings.m_endNodeThickness;
				}
				else {
					float childThicknessCollection = 0.0f;
					for (const auto& childHandle : childHandles) {
						const auto& childNode = skeleton.RefNode(childHandle);
						childThicknessCollection += glm::pow(childNode.m_info.m_thickness + reconstructionSettings.m_thicknessAccumulationFactor,
							1.0f / reconstructionSettings.m_thicknessSumFactor);
					}
					nodeInfo.m_thickness = glm::pow(childThicknessCollection,
						reconstructionSettings.m_thicknessSumFactor);
				}
			}
		}
		else if (reconstructionSettings.m_limitParentThickness)
		{
			for (auto i = sortedNodeList.rbegin(); i != sortedNodeList.rend(); i++) {
				auto& node = skeleton.RefNode(*i);
				auto& nodeInfo = node.m_info;
				for (const auto& childHandle : node.RefChildHandles()) {
					const auto& childNode = skeleton.RefNode(childHandle);
					nodeInfo.m_thickness = glm::max(childNode.m_info.m_thickness, nodeInfo.m_thickness);
				}
			}
		}
		CalculateNodeTransforms(skeleton);
		skeleton.CalculateFlows();
		skeleton.CalculateDistance();
	}


	for (auto& allocatedPoint : m_allocatedPoints) {
		const auto& treePart = m_treeParts[allocatedPoint.m_treePartHandle];
		float minDistance = 999.f;
		NodeHandle closestNodeHandle = -1;
		BranchHandle cloeseBranchHandle = -1;
		int closestSkeletonIndex = -1;
		for (const auto& branchHandle : treePart.m_branchHandles) {
			auto& branch = operatingBranches[branchHandle];
			for (const auto& nodeHandle : branch.m_chainNodeHandles) {
				auto& node = m_skeletons[branch.m_skeletonIndex].RefNode(nodeHandle);
				auto distance = glm::distance(node.m_info.m_globalPosition, allocatedPoint.m_position);
				if (distance < minDistance) {
					minDistance = distance;
					closestNodeHandle = nodeHandle;
					closestSkeletonIndex = branch.m_skeletonIndex;
					cloeseBranchHandle = branchHandle;
				}
			}
		}
		allocatedPoint.m_nodeHandle = closestNodeHandle;
		allocatedPoint.m_branchHandle = cloeseBranchHandle;
		allocatedPoint.m_skeletonIndex = closestSkeletonIndex;
		if (allocatedPoint.m_skeletonIndex != -1)
			m_skeletons[allocatedPoint.m_skeletonIndex].RefNode(
				closestNodeHandle).m_data.m_allocatedPoints.emplace_back(allocatedPoint.m_handle);
	}
	if (m_skeletons.size() > 1) {
		for (int i = 0; i < m_skeletons.size(); i++)
		{
			auto& skeleton = m_skeletons[i];
			if (skeleton.RefSortedNodeList().size() < reconstructionSettings.m_minimumNodeCount)
			{
				m_skeletons.erase(m_skeletons.begin() + i);
				i--;
			}
		}
	}
}

void TreePointCloud::ClearMeshes() const {
	const auto scene = GetScene();
	const auto self = GetOwner();
	const auto children = scene->GetChildren(self);
	for (const auto& child : children) {
		auto name = scene->GetEntityName(child);
		if (name.substr(0, 4) == "Tree") {
			scene->DeleteEntity(child);
		}
	}
}

void TreePointCloud::FormGeometryEntity(const TreeMeshGeneratorSettings& meshGeneratorSettings) {
	const auto scene = GetScene();
	const auto self = GetOwner();
	const auto children = scene->GetChildren(self);

	ClearMeshes();
	for (int i = 0; i < m_skeletons.size(); i++) {
		Entity treeEntity;
		treeEntity = scene->CreateEntity("Tree " + std::to_string(i));
		scene->SetParent(treeEntity, self);
		if (meshGeneratorSettings.m_enableBranch) {
			Entity branchEntity;
			branchEntity = scene->CreateEntity("Branch Mesh");
			scene->SetParent(branchEntity, treeEntity);

			std::vector<Vertex> vertices;
			std::vector<unsigned int> indices;

			switch (meshGeneratorSettings.m_branchMeshType)
			{
			case 0:
			{
				CylindricalMeshGenerator<ReconstructionSkeletonData, ReconstructionFlowData, ReconstructionNodeData> meshGenerator;
				meshGenerator.Generate(m_skeletons[i], vertices, indices, meshGeneratorSettings, 999.0f);
			}
			break;
			case 1:
			{
				float minRadius = 0.003f;
				VoxelMeshGenerator<ReconstructionSkeletonData, ReconstructionFlowData, ReconstructionNodeData> meshGenerator;
				meshGenerator.Generate(m_skeletons[i], vertices, indices,
					meshGeneratorSettings, minRadius);
			}
			break;
			default: break;
			}

			auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
			auto material = ProjectManager::CreateTemporaryAsset<Material>();
			VertexAttributes attributes{};
			attributes.m_texCoord = true;
			mesh->SetVertices(attributes, vertices, indices);
			auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(branchEntity).lock();
			if (meshGeneratorSettings.m_foliageOverride) {
				material->m_materialProperties.m_albedoColor = meshGeneratorSettings.m_presentationOverrideSettings.m_branchOverrideColor;
			}
			else {
				material->m_materialProperties.m_albedoColor = glm::vec3(109, 79, 75) / 255.0f;
			}
			material->m_materialProperties.m_roughness = 1.0f;
			material->m_materialProperties.m_metallic = 0.0f;
			meshRenderer->m_mesh = mesh;
			meshRenderer->m_material = material;
		}
		if (meshGeneratorSettings.m_enableTwig)
		{
			Entity twigEntity;
			twigEntity = scene->CreateEntity("Twig Mesh");
			scene->SetParent(twigEntity, treeEntity);
			std::vector<glm::uint> twigSegments;
			std::vector<StrandPoint> twigPoints;
			const auto& shootSkeleton = m_skeletons[i];
			const auto& internodeList = shootSkeleton.RefSortedNodeList();
			for (int internodeHandle : internodeList)
			{
				const auto& internode = shootSkeleton.PeekNode(internodeHandle);
				const auto& internodeData = internode.m_data;
				const auto& internodeInfo = internode.m_info;
				std::vector<std::vector<glm::vec4>> twigs{};
				if (internodeInfo.m_thickness < meshGeneratorSettings.m_twigParameters.m_maxNodeThickness
					&& internodeInfo.m_rootDistance > meshGeneratorSettings.m_twigParameters.m_minRootDistance
					&& internodeInfo.m_endDistance < meshGeneratorSettings.m_twigParameters.m_maxEndDistance)
				{
					int twigCount = internodeInfo.m_length / meshGeneratorSettings.m_twigParameters.m_unitDistance;
					twigs.resize(twigCount);

					auto desiredGlobalRotation = internodeInfo.m_regulatedGlobalRotation * glm::quat(glm::vec3(
						glm::radians(meshGeneratorSettings.m_twigParameters.m_branchingAngle), 0.0f,
						glm::radians(glm::radians(
							glm::linearRand(0.0f,
								360.0f)))));

					glm::vec3 directionStart = internodeInfo.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
					glm::vec3 directionEnd = directionStart;

					glm::vec3 positionStart = internodeInfo.m_globalPosition;
					glm::vec3 positionEnd =
						positionStart + internodeInfo.m_length * meshGeneratorSettings.m_lineLengthFactor * internodeInfo.m_globalDirection;

					BezierCurve curve = BezierCurve(
						positionStart,
						positionStart +
						(meshGeneratorSettings.m_smoothness ? internodeInfo.m_length * meshGeneratorSettings.m_baseControlPointRatio : 0.0f) * directionStart,
						positionEnd -
						(meshGeneratorSettings.m_smoothness ? internodeInfo.m_length * meshGeneratorSettings.m_branchControlPointRatio : 0.0f) * directionEnd,
						positionEnd);

					for (int twigIndex = 0; twigIndex < twigCount; twigIndex++) {
						glm::vec3 positionWalker = curve.GetPoint(static_cast<float>(twigIndex) / twigCount);
						twigs[twigIndex].resize(meshGeneratorSettings.m_twigParameters.m_segmentSize);
						const float rollAngle = glm::radians(glm::linearRand(0.0f, 360.0f));
						for (int twigPointIndex = 0; twigPointIndex < meshGeneratorSettings.m_twigParameters.m_segmentSize; twigPointIndex++)
						{
							twigs[twigIndex][twigPointIndex] = glm::vec4(positionWalker, meshGeneratorSettings.m_twigParameters.m_thickness);
							desiredGlobalRotation = internodeInfo.m_regulatedGlobalRotation * glm::quat(glm::vec3(
								glm::radians(glm::gaussRand(0.f, meshGeneratorSettings.m_twigParameters.m_apicalAngleVariance) + meshGeneratorSettings.m_twigParameters.m_branchingAngle), 0.0f,
								rollAngle));

							auto twigFront = desiredGlobalRotation * glm::vec3(0, 0, -1);
							positionWalker = positionWalker + twigFront * meshGeneratorSettings.m_twigParameters.m_segmentLength;
						}
					}

				}

				for (const auto& twig : twigs) {
					const auto twigSegmentSize = twig.size();
					const auto twigControlPointSize = twig.size() + 3;
					const auto totalTwigPointSize = twigPoints.size();
					const auto totalTwigSegmentSize = twigSegments.size();
					twigPoints.resize(totalTwigPointSize + twigControlPointSize);
					twigSegments.resize(totalTwigSegmentSize + twigSegmentSize);

					for (int i = 0; i < twigControlPointSize; i++) {
						auto& p = twigPoints[totalTwigPointSize + i];
						p.m_position = glm::vec3(twig[glm::clamp(i - 2, 0, static_cast<int>(twigSegmentSize - 1))]);
						p.m_thickness = twig[glm::clamp(i - 2, 0, static_cast<int>(twigSegmentSize - 1))].w;
					}
					twigPoints[totalTwigPointSize].m_position = glm::vec3(twig[0]) * 2.0f - glm::vec3(twig[1]);
					twigPoints[totalTwigPointSize].m_thickness = twig[0].w * 2.0f - twig[1].w;

					twigPoints[totalTwigPointSize + twigControlPointSize - 1].m_position = glm::vec3(twig[twigSegmentSize - 1]) * 2.0f - glm::vec3(twig[twigSegmentSize - 2]);
					twigPoints[totalTwigPointSize + twigControlPointSize - 1].m_thickness = twig[twigSegmentSize - 1].w * 2.0f - twig[twigSegmentSize - 2].w;


					for (int i = 0; i < twigSegmentSize; i++) {
						twigSegments[totalTwigSegmentSize + i] = totalTwigPointSize + i;
					}
				}
			}

			auto strands = ProjectManager::CreateTemporaryAsset<Strands>();
			auto material = ProjectManager::CreateTemporaryAsset<Material>();
			if (meshGeneratorSettings.m_foliageOverride)
			{
				material->m_materialProperties.m_albedoColor = meshGeneratorSettings.m_presentationOverrideSettings.m_branchOverrideColor;
			}
			else {
				material->m_materialProperties.m_albedoColor = glm::vec3(80, 60, 50) / 255.0f;
			}
			material->m_materialProperties.m_roughness = 1.0f;
			material->m_materialProperties.m_metallic = 0.0f;
			StrandPointAttributes strandPointAttributes{};
			strands->SetSegments(strandPointAttributes, twigSegments, twigPoints);
			auto strandsRenderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(twigEntity).lock();
			strandsRenderer->m_strands = strands;
			strandsRenderer->m_material = material;
		}
		if (meshGeneratorSettings.m_enableFoliage)
		{
			Entity foliageEntity;
			foliageEntity = scene->CreateEntity("Foliage Mesh");
			scene->SetParent(foliageEntity, treeEntity);
			const auto& shootSkeleton = m_skeletons[i];
			std::vector<Vertex> vertices;
			std::vector<unsigned int> indices;
			auto quadMesh = Resources::GetResource<Mesh>("PRIMITIVE_QUAD");
			auto& quadTriangles = quadMesh->UnsafeGetTriangles();
			auto quadVerticesSize = quadMesh->GetVerticesAmount();
			size_t offset = 0;

			const auto& nodeList = shootSkeleton.RefSortedNodeList();
			for (const auto& internodeHandle : nodeList) {
				const auto& internode = shootSkeleton.PeekNode(internodeHandle);
				const auto& internodeInfo = internode.m_info;

				const auto& foliageOverrideSettings = meshGeneratorSettings.m_foliageOverrideSettings;
				if (internodeInfo.m_thickness < foliageOverrideSettings.m_maxNodeThickness
					&& internodeInfo.m_rootDistance > foliageOverrideSettings.m_minRootDistance
					&& internodeInfo.m_endDistance < foliageOverrideSettings.m_maxEndDistance) {
					for (int i = 0; i < foliageOverrideSettings.m_leafCountPerInternode; i++)
					{
						auto leafSize = foliageOverrideSettings.m_leafSize;
						glm::quat rotation = internodeInfo.m_globalDirection * glm::quat(glm::radians(glm::linearRand(glm::vec3(0.0f), glm::vec3(360.0f))));
						auto front = rotation * glm::vec3(0, 0, -1);
						auto foliagePosition = internodeInfo.m_globalPosition + front * (leafSize.z * 1.5f) + glm::sphericalRand(1.0f) * glm::linearRand(0.0f, foliageOverrideSettings.m_positionVariance);
						auto leafTransform = glm::translate(foliagePosition) * glm::mat4_cast(rotation) * glm::scale(leafSize);

						auto& matrix = leafTransform;
						Vertex archetype;
						for (auto i = 0; i < quadMesh->GetVerticesAmount(); i++) {
							archetype.m_position =
								matrix * glm::vec4(quadMesh->UnsafeGetVertices()[i].m_position, 1.0f);
							archetype.m_normal = glm::normalize(glm::vec3(
								matrix * glm::vec4(quadMesh->UnsafeGetVertices()[i].m_normal, 0.0f)));
							archetype.m_tangent = glm::normalize(glm::vec3(
								matrix *
								glm::vec4(quadMesh->UnsafeGetVertices()[i].m_tangent, 0.0f)));
							archetype.m_texCoord =
								quadMesh->UnsafeGetVertices()[i].m_texCoord;
							vertices.push_back(archetype);
						}
						for (auto triangle : quadTriangles) {
							triangle.x += offset;
							triangle.y += offset;
							triangle.z += offset;
							indices.push_back(triangle.x);
							indices.push_back(triangle.y);
							indices.push_back(triangle.z);
						}
						offset += quadVerticesSize;
					}
				}

			}

			auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
			auto material = ProjectManager::CreateTemporaryAsset<Material>();
			VertexAttributes vertexAttributes{};
			vertexAttributes.m_texCoord = true;
			mesh->SetVertices(vertexAttributes, vertices, indices);
			if (meshGeneratorSettings.m_foliageOverride)
			{
				material->m_materialProperties.m_albedoColor = meshGeneratorSettings.m_presentationOverrideSettings.m_foliageOverrideColor;
			}
			else {
				material->m_materialProperties.m_albedoColor = glm::vec3(152 / 255.0f, 203 / 255.0f, 0 / 255.0f);
			}
			material->m_materialProperties.m_roughness = 0.0f;
			auto texRef = meshGeneratorSettings.m_foliageTexture;
			if (texRef.Get<Texture2D>())
			{
				material->SetAlbedoTexture(texRef.Get<Texture2D>());

			}
			auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(foliageEntity).lock();
			meshRenderer->m_mesh = mesh;
			meshRenderer->m_material = material;

		}
	}
}

std::vector<std::shared_ptr<Mesh>> TreePointCloud::GenerateBranchMeshes(const TreeMeshGeneratorSettings& meshGeneratorSettings) const
{
	std::vector<std::shared_ptr<Mesh>> meshes{};
	for (int i = 0; i < m_skeletons.size(); i++) {
		std::vector<Vertex> vertices;
		std::vector<unsigned int> indices;
		CylindricalMeshGenerator<ReconstructionSkeletonData, ReconstructionFlowData, ReconstructionNodeData> meshGenerator;
		meshGenerator.Generate(m_skeletons[i], vertices, indices, meshGeneratorSettings, 999.0f);

		auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
		VertexAttributes attributes{};
		attributes.m_texCoord = true;
		mesh->SetVertices(attributes, vertices, indices);
		meshes.emplace_back(mesh);
	}
	return meshes;
}

std::vector<std::shared_ptr<Mesh>> TreePointCloud::GenerateFoliageMeshes(
	const TreeMeshGeneratorSettings& meshGeneratorSettings) const
{
	std::vector<std::shared_ptr<Mesh>> meshes{};
	for (int i = 0; i < m_skeletons.size(); i++) {
		std::vector<Vertex> vertices;
		std::vector<unsigned int> indices;
		
		auto quadMesh = Resources::GetResource<Mesh>("PRIMITIVE_QUAD");
		auto& quadTriangles = quadMesh->UnsafeGetTriangles();
		auto quadVerticesSize = quadMesh->GetVerticesAmount();
		size_t offset = 0;

		const auto& nodeList = m_skeletons[i].RefSortedNodeList();
		for (const auto& internodeHandle : nodeList) {
			const auto& internode = m_skeletons[i].PeekNode(internodeHandle);
			const auto& internodeInfo = internode.m_info;

			const auto& foliageOverrideSettings = meshGeneratorSettings.m_foliageOverrideSettings;
			if (internodeInfo.m_thickness < foliageOverrideSettings.m_maxNodeThickness
				&& internodeInfo.m_rootDistance > foliageOverrideSettings.m_minRootDistance
				&& internodeInfo.m_endDistance < foliageOverrideSettings.m_maxEndDistance) {
				for (int i = 0; i < foliageOverrideSettings.m_leafCountPerInternode; i++)
				{
					auto leafSize = foliageOverrideSettings.m_leafSize;
					glm::quat rotation = internodeInfo.m_globalDirection * glm::quat(glm::radians(glm::linearRand(glm::vec3(0.0f), glm::vec3(360.0f))));
					auto front = rotation * glm::vec3(0, 0, -1);
					auto foliagePosition = internodeInfo.m_globalPosition + front * (leafSize.z * 1.5f) + glm::sphericalRand(1.0f) * glm::linearRand(0.0f, foliageOverrideSettings.m_positionVariance);
					auto leafTransform = glm::translate(foliagePosition) * glm::mat4_cast(rotation) * glm::scale(leafSize);

					auto& matrix = leafTransform;
					Vertex archetype;
					for (auto i = 0; i < quadMesh->GetVerticesAmount(); i++) {
						archetype.m_position =
							matrix * glm::vec4(quadMesh->UnsafeGetVertices()[i].m_position, 1.0f);
						archetype.m_normal = glm::normalize(glm::vec3(
							matrix * glm::vec4(quadMesh->UnsafeGetVertices()[i].m_normal, 0.0f)));
						archetype.m_tangent = glm::normalize(glm::vec3(
							matrix *
							glm::vec4(quadMesh->UnsafeGetVertices()[i].m_tangent, 0.0f)));
						archetype.m_texCoord =
							quadMesh->UnsafeGetVertices()[i].m_texCoord;
						vertices.push_back(archetype);
					}
					for (auto triangle : quadTriangles) {
						triangle.x += offset;
						triangle.y += offset;
						triangle.z += offset;
						indices.push_back(triangle.x);
						indices.push_back(triangle.y);
						indices.push_back(triangle.z);
					}
					offset += quadVerticesSize;
				}
			}

		}

		auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
		VertexAttributes attributes{};
		attributes.m_texCoord = true;
		mesh->SetVertices(attributes, vertices, indices);
		meshes.emplace_back(mesh);
	}
	return meshes;
}


void ConnectivityGraphSettings::OnInspect() {
	ImGui::DragFloat("Point-point max length", &m_scatterPointsConnectionMaxLength, 0.01f, 0.01f, 1.0f);
	ImGui::DragFloat("Point-branch max length", &m_scatterPointBranchConnectionMaxLength, 0.01f, 0.01f, 1.0f);
	ImGui::DragInt("Branch-branch search timeout", &m_maxTimeout, 1, 1, 30);
	ImGui::DragFloat("Branch-branch search length", &m_edgeLength, 0.01f, 0.01f, 1.0f);
	ImGui::DragFloat("Branch-branch search step", &m_edgeExtendStep, 0.01f, 0.01f, 1.0f);
	ImGui::DragFloat("Angle limit", &m_angleLimit, 0.01f, 0.0f, 180.0f);
	ImGui::Checkbox("Check reverse", &m_checkReverse);
	ImGui::DragFloat("Branch shortening", &m_branchShortening, 0.01f, 0.00f, 0.5f);
	ImGui::DragFloat("Force connection ratio", &m_forceConnectionRatio, 0.01f, 0.01f, 1.0f);
	if (m_forceConnectionRatio > 0.0f) {
		ImGui::DragFloat("Force connection angle limit", &m_forceConnectionAngleLimit, 0.01f, 0.01f, 90.0f);
	}
}

void OperatingBranch::Apply(const ScannedBranch& target) {
	m_color = target.m_color;
	m_treePartHandle = target.m_treePartHandle;
	m_handle = target.m_handle;
	m_bezierCurve = target.m_bezierCurve;
	m_startThickness = target.m_startThickness;
	m_endThickness = target.m_endThickness;
	m_parentHandle = target.m_parentHandle;
	m_childHandles = target.m_childHandles;
	m_skeletonIndex = -1;
	m_chainNodeHandles.clear();
}

void ReconstructionSettings::OnInspect() {
	ImGui::DragFloat("Internode length", &m_internodeLength, 0.01f, 0.01f, 1.0f);
	ImGui::DragFloat("Root node max height", &m_minHeight, 0.01f, 0.01f, 1.0f);
	ImGui::DragFloat("Tree distance limit", &m_maxTreeDistance, 0.01f, 0.01f, 1.0f);
	ImGui::DragFloat("Branch shortening", &m_branchShortening, 0.01f, 0.01f, 0.5f);

	ImGui::Checkbox("Override thickness", &m_overrideThickness);
	if (m_overrideThickness) {
		ImGui::DragFloat("End node thickness", &m_endNodeThickness, 0.001f, 0.001f, 1.0f);
		ImGui::DragFloat("Thickness sum factor", &m_thicknessSumFactor, 0.01f, 0.0f, 2.0f);
		ImGui::DragFloat("Thickness accumulation factor", &m_thicknessAccumulationFactor, 0.00001f, 0.0f, 1.0f, "%.5f");
	}
	else
	{
		ImGui::Checkbox("Limit parent thickness", &m_limitParentThickness);
	}
	ImGui::DragInt("Minimum node count", &m_minimumNodeCount, 1, 0, 100);
}
