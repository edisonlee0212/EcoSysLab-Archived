//
// Created by lllll on 10/24/2022.
//

#include "Tree.hpp"

#include <Material.hpp>
#include <Mesh.hpp>
#include "Strands.hpp"
#include "EditorLayer.hpp"
#include "Application.hpp"
#include "TreeMeshGenerator.hpp"
#include "Soil.hpp"
#include "Climate.hpp"
#include "Octree.hpp"
#include "EcoSysLabLayer.hpp"
#include "HeightField.hpp"
#include "StrandsRenderer.hpp"
using namespace EcoSysLab;
void Tree::SerializeTreeGrowthSettings(const TreeGrowthSettings& treeGrowthSettings, YAML::Emitter& out)
{
	out << YAML::Key << "m_enableRoot" << YAML::Value << treeGrowthSettings.m_enableRoot;
	out << YAML::Key << "m_enableShoot" << YAML::Value << treeGrowthSettings.m_enableShoot;

	out << YAML::Key << "m_autoBalance" << YAML::Value << treeGrowthSettings.m_autoBalance;
	out << YAML::Key << "m_collectRootFlux" << YAML::Value << treeGrowthSettings.m_collectRootFlux;

	out << YAML::Key << "m_collectNitrite" << YAML::Value << treeGrowthSettings.m_collectNitrite;
	out << YAML::Key << "m_nodeDevelopmentalVigorFillingRate" << YAML::Value << treeGrowthSettings.m_nodeDevelopmentalVigorFillingRate;

	out << YAML::Key << "m_enableRootCollisionDetection" << YAML::Value << treeGrowthSettings.m_enableRootCollisionDetection;
	out << YAML::Key << "m_enableBranchCollisionDetection" << YAML::Value << treeGrowthSettings.m_enableBranchCollisionDetection;

	out << YAML::Key << "m_useSpaceColonization" << YAML::Value << treeGrowthSettings.m_useSpaceColonization;
	out << YAML::Key << "m_spaceColonizationAutoResize" << YAML::Value << treeGrowthSettings.m_spaceColonizationAutoResize;
	out << YAML::Key << "m_spaceColonizationRemovalDistanceFactor" << YAML::Value << treeGrowthSettings.m_spaceColonizationRemovalDistanceFactor;
	out << YAML::Key << "m_spaceColonizationDetectionDistanceFactor" << YAML::Value << treeGrowthSettings.m_spaceColonizationDetectionDistanceFactor;
	out << YAML::Key << "m_spaceColonizationTheta" << YAML::Value << treeGrowthSettings.m_spaceColonizationTheta;
}
void Tree::DeserializeTreeGrowthSettings(TreeGrowthSettings& treeGrowthSettings, const YAML::Node& param) {
	if (param["m_enableRoot"]) treeGrowthSettings.m_enableRoot = param["m_enableRoot"].as<bool>();
	if (param["m_enableShoot"]) treeGrowthSettings.m_enableShoot = param["m_enableShoot"].as<bool>();

	if (param["m_autoBalance"]) treeGrowthSettings.m_autoBalance = param["m_autoBalance"].as<bool>();
	if (param["m_collectRootFlux"]) treeGrowthSettings.m_collectRootFlux = param["m_collectRootFlux"].as<bool>();
	if (param["m_collectNitrite"]) treeGrowthSettings.m_collectNitrite = param["m_collectNitrite"].as<bool>();

	if (param["m_nodeDevelopmentalVigorFillingRate"]) treeGrowthSettings.m_nodeDevelopmentalVigorFillingRate = param["m_nodeDevelopmentalVigorFillingRate"].as<float>();

	if (param["m_enableRootCollisionDetection"]) treeGrowthSettings.m_enableRootCollisionDetection = param["m_enableRootCollisionDetection"].as<bool>();
	if (param["m_enableBranchCollisionDetection"]) treeGrowthSettings.m_enableBranchCollisionDetection = param["m_enableBranchCollisionDetection"].as<bool>();

	if (param["m_useSpaceColonization"]) treeGrowthSettings.m_useSpaceColonization = param["m_useSpaceColonization"].as<bool>();
	if (param["m_spaceColonizationAutoResize"]) treeGrowthSettings.m_spaceColonizationAutoResize = param["m_spaceColonizationAutoResize"].as<bool>();
	if (param["m_spaceColonizationRemovalDistanceFactor"]) treeGrowthSettings.m_spaceColonizationRemovalDistanceFactor = param["m_spaceColonizationRemovalDistanceFactor"].as<float>();
	if (param["m_spaceColonizationDetectionDistanceFactor"]) treeGrowthSettings.m_spaceColonizationDetectionDistanceFactor = param["m_spaceColonizationDetectionDistanceFactor"].as<float>();
	if (param["m_spaceColonizationTheta"]) treeGrowthSettings.m_spaceColonizationTheta = param["m_spaceColonizationTheta"].as<float>();
}

bool Tree::ParseBinvox(const std::filesystem::path& filePath, VoxelGrid<TreeOccupancyGridBasicData>& voxelGrid, float voxelSize)
{
	std::ifstream input(filePath, std::ios::in | std::ios::binary);
	if (!input.is_open()) {
		std::cout << "Error: could not open file " << filePath << std::endl;
		return false;
	}

	// Read header
	std::string line;
	input >> line;  // #binvox
	if (line.compare("#binvox") != 0) {
		std::cout << "Error: first line reads [" << line << "] instead of [#binvox]" << std::endl;
		return false;
	}
	int version;
	input >> version;
#ifndef NDEBUG
	std::cout << "reading binvox version " << version << std::endl;
#endif
	int depth, height, width;
	depth = -1;
	bool done = false;
	while (input.good() && !done) {
		input >> line;
		if (line.compare("data") == 0) done = true;
		else if (line.compare("dim") == 0) {
			input >> depth >> height >> width;
		}
		else {
#ifndef NDEBUG
			std::cout << "  unrecognized keyword [" << line << "], skipping" << std::endl;
#endif
			char c;
			do {  // skip until end of line
				c = input.get();
			} while (input.good() && (c != '\n'));
		}
	}

	if (!done) {
		std::cout << "  error reading header" << std::endl;
		return false;
	}
	if (depth == -1) {
		std::cout << "  missing dimensions in header" << std::endl;
		return false;
	}

	// Initialize the voxel grid based on the dimensions read
	glm::vec3 minBound(0, 0, 0);  // Assuming starting from origin
	glm::ivec3 resolution(width, height, depth);
	voxelGrid.Initialize(voxelSize, resolution, minBound, {});  // Assuming voxelSize is globally defined or passed as an argument

	// Read voxel data
	unsigned char value;
	unsigned char count;
	int index = 0;
	int end_index = 0;
	int nr_voxels = 0;

	input.unsetf(std::ios::skipws);  // need to read every byte now (!)
	input >> value;  // read the linefeed char
	glm::vec3 lowSum = glm::ivec3(0.0f);
	size_t lowSumCount = 0;
	while (end_index < width * height * depth && input.good()) {
		input >> value >> count;

		if (input.good()) {
			end_index = index + count;
			if (end_index > (width * height * depth)) return false;

			for (int i = index; i < end_index; i++) {
				// Convert 1D index to 3D coordinates
				const int x = (i / width) % height;
				const int y = i % width;
				const int z = i / (width * height);

				if (value) {
					voxelGrid.Ref(glm::ivec3(x, y, z)).m_occupied = true;
					nr_voxels++;

					if (y < (height * 0.2f))
					{
						lowSum += voxelGrid.GetPosition(glm::ivec3(x, y, z));
						lowSumCount++;
					}
				}
			}

			index = end_index;
		}
	}
	lowSum /= lowSumCount;
	voxelGrid.ShiftMinBound(-glm::vec3(lowSum.x, 0, lowSum.z));

	input.close();
#ifndef NDEBUG
	std::cout << "  read " << nr_voxels << " voxels" << std::endl;
#endif
	return true;
}

void Tree::Reset()
{
	m_treeModel.Clear();
	m_treeModel.m_index = GetOwner().GetIndex();
	m_treeVisualizer.Reset(m_treeModel);
}
void Tree::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
	bool modelChanged = false;
	const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
	const auto scene = GetScene();
	if (editorLayer->DragAndDropButton<TreeDescriptor>(m_treeDescriptor, "TreeDescriptor", true)) {
		m_treeModel.Clear();
		modelChanged = true;
	}
	static bool showSpaceColonizationGrid = false;

	static std::shared_ptr<ParticleInfoList> spaceColonizationGridParticleInfoList;
	if (!spaceColonizationGridParticleInfoList)
	{
		spaceColonizationGridParticleInfoList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
	}


	if (m_treeDescriptor.Get<TreeDescriptor>()) {
		if (ImGui::Button("Reset")) {
			Reset();
			modelChanged = true;
		}
		if (ImGui::TreeNode("Tree settings")) {
			ImGui::Checkbox("Enable History", &m_enableHistory);
			if (m_enableHistory)
			{
				ImGui::DragInt("History per iteration", &m_historyIteration, 1, 1, 1000);
			}
			OnInspectTreeGrowthSettings(m_treeModel.m_treeGrowthSettings);

			if (m_treeModel.m_treeGrowthSettings.m_useSpaceColonization && !m_treeModel.m_treeGrowthSettings.m_spaceColonizationAutoResize)
			{
				static float radius = 1.5f;
				static int markersPerVoxel = 5;
				ImGui::DragFloat("Import radius", &radius, 0.01f, 0.01f, 10.0f);
				ImGui::DragInt("Markers per voxel", &markersPerVoxel);
				FileUtils::OpenFile("Load Voxel Data", "Binvox", { ".binvox" }, [&](const std::filesystem::path& path) {
					auto& occupancyGrid = m_treeModel.m_treeOccupancyGrid;
					VoxelGrid<TreeOccupancyGridBasicData> inputGrid{};
					if (ParseBinvox(path, inputGrid, 1.f))
					{
						const auto treeDescriptor = m_treeDescriptor.Get<TreeDescriptor>();
						occupancyGrid.Initialize(inputGrid,
							glm::vec3(-radius, 0, -radius),
							glm::vec3(radius, 2.0f * radius, radius),
							treeDescriptor->m_shootGrowthParameters.m_internodeLength,
							m_treeModel.m_treeGrowthSettings.m_spaceColonizationRemovalDistanceFactor,
							m_treeModel.m_treeGrowthSettings.m_spaceColonizationTheta,
							m_treeModel.m_treeGrowthSettings.m_spaceColonizationDetectionDistanceFactor, markersPerVoxel);
					}

					}, false);

				static PrivateComponentRef privateComponentRef{};

				if (editorLayer->DragAndDropButton<MeshRenderer>(privateComponentRef, "Add Obstacle"))
				{
					if (const auto mmr = privateComponentRef.Get<MeshRenderer>())
					{
						const auto cubeVolume = ProjectManager::CreateTemporaryAsset<CubeVolume>();
						cubeVolume->ApplyMeshBounds(mmr->m_mesh.Get<Mesh>());
						const auto globalTransform = scene->GetDataComponent<GlobalTransform>(mmr->GetOwner());
						m_treeModel.m_treeOccupancyGrid.InsertObstacle(globalTransform, cubeVolume);
						privateComponentRef.Clear();
					}

				}
			}

			ImGui::TreePop();
		}
		if (ImGui::TreeNode("Mesh generation settings")) {
			static int iterations = 5;
			ImGui::DragInt("Iterations", &iterations, 1, 0, m_treeModel.CurrentIteration());
			iterations = glm::clamp(iterations, 0, m_treeModel.CurrentIteration());
			m_meshGeneratorSettings.OnInspect(editorLayer);
			if (ImGui::Button("Generate Mesh")) {
				GenerateGeometry(m_meshGeneratorSettings, iterations);
			}
			if (ImGui::Button("Clear Mesh"))
			{
				ClearMeshes();
			}
			ImGui::TreePop();
		}

		ImGui::Checkbox("Enable Visualization", &m_enableVisualization);
		if (m_enableVisualization) {
			bool needGridUpdate = false;
			if (m_treeVisualizer.m_needUpdate)
			{
				needGridUpdate = true;
			}
			if (ImGui::Button("Update grids")) needGridUpdate = true;
			ImGui::Checkbox("Show Space Colonization Grid", &showSpaceColonizationGrid);
			if (showSpaceColonizationGrid && needGridUpdate) {
				auto& occupancyGrid = m_treeModel.m_treeOccupancyGrid;
				auto& voxelGrid = occupancyGrid.RefGrid();
				const auto numVoxels = voxelGrid.GetVoxelCount();
				auto& scalarMatrices = spaceColonizationGridParticleInfoList->m_particleInfos;

				if (scalarMatrices.size() != numVoxels) {
					scalarMatrices.resize(numVoxels);
				}

				if (scalarMatrices.size() != numVoxels) {
					scalarMatrices.reserve(occupancyGrid.GetMarkersPerVoxel() * numVoxels);
				}
				int i = 0;
				for (const auto& voxel : voxelGrid.RefData())
				{
					for (const auto& marker : voxel.m_markers) {
						scalarMatrices.resize(i + 1);
						scalarMatrices[i].m_instanceMatrix.m_value =
							glm::translate(marker.m_position)
							* glm::mat4_cast(glm::quat(glm::vec3(0.0f)))
							* glm::scale(glm::vec3(voxelGrid.GetVoxelSize() * 0.2f));
						if (marker.m_nodeHandle == -1) scalarMatrices[i].m_instanceColor = glm::vec4(1.0f, 1.0f, 1.0f, 0.75f);
						else
						{
							scalarMatrices[i].m_instanceColor = glm::vec4(ecoSysLabLayer->RandomColors()[marker.m_nodeHandle], 1.0f);
						}
						i++;
					}
				}

				spaceColonizationGridParticleInfoList->SetPendingUpdate();
			}

		}
		if (m_enableVisualization && ImGui::TreeNodeEx("Tree Inspector", ImGuiTreeNodeFlags_DefaultOpen))
		{
			modelChanged = m_treeVisualizer.OnInspect(m_treeModel) || modelChanged;
			if (m_treeVisualizer.GetSelectedInternodeHandle() >= 0)
			{
				static float deltaTime = 0.01918f;
				static bool autoGrowSubTree = false;
				ImGui::DragFloat("Delta time", &deltaTime, 0.00001f, 0, 1, "%.5f");
				if (ImGui::Button("Day")) deltaTime = 0.00274f;
				ImGui::SameLine();
				if (ImGui::Button("Week")) deltaTime = 0.01918f;
				ImGui::SameLine();
				if (ImGui::Button("Month")) deltaTime = 0.0822f;
				ImGui::Checkbox("Auto grow subtree", &autoGrowSubTree);
				if (!autoGrowSubTree) {
					bool changed = false;
					if (ImGui::Button("Grow subtree")) {
						TryGrowSubTree(m_treeVisualizer.GetSelectedInternodeHandle(), deltaTime);
						changed = true;
					}
					static int iterations = 5;
					ImGui::DragInt("Iterations", &iterations, 1, 1, 100);
					if (ImGui::Button(("Grow subtree with " + std::to_string(iterations) + " iterations").c_str())) {
						for (int i = 0; i < iterations; i++) TryGrowSubTree(m_treeVisualizer.GetSelectedInternodeHandle(), deltaTime);
						changed = true;
					}
				}
				else
				{
					TryGrowSubTree(m_treeVisualizer.GetSelectedInternodeHandle(), deltaTime);
				}
			}
			ImGui::TreePop();
		}
		if (m_enableHistory)
		{
			if (ImGui::Button("Temporal Progression"))
			{
				m_temporalProgression = true;
				m_temporalProgressionIteration = 0;
			}
		}
	}

	/*
	ImGui::Checkbox("Split root test", &m_splitRootTest);
	ImGui::Checkbox("Biomass history", &m_recordBiomassHistory);

	if (m_splitRootTest) ImGui::Text(("Left/Right side biomass: [" + std::to_string(m_leftSideBiomass) + ", " + std::to_string(m_rightSideBiomass) + "]").c_str());
	*/


	if (m_enableVisualization)
	{

		m_treeVisualizer.Visualize(m_treeModel,
			scene->GetDataComponent<GlobalTransform>(GetOwner()));
		GizmoSettings gizmoSettings{};
		gizmoSettings.m_drawSettings.m_blending = true;
		if (showSpaceColonizationGrid)
		{
			editorLayer->DrawGizmoMeshInstancedColored(
				Resources::GetResource<Mesh>("PRIMITIVE_CUBE"), spaceColonizationGridParticleInfoList,
				glm::mat4(1.0f), 1.0f, gizmoSettings);
		}

	}
}
void Tree::Update()
{
	if (m_temporalProgression) {
		if (m_temporalProgressionIteration <= m_treeModel.CurrentIteration()) {
			GenerateGeometry(m_meshGeneratorSettings, m_temporalProgressionIteration);
			m_temporalProgressionIteration++;
		}
		else
		{
			m_temporalProgressionIteration = 0;
			m_temporalProgression = false;
		}
	}
}

void Tree::OnCreate() {
	m_treeVisualizer.Initialize();
}

void Tree::OnDestroy() {
	m_treeModel.Clear();
	m_treeDescriptor.Clear();
	m_soil.Clear();
	m_climate.Clear();
	m_enableHistory = false;

	m_leftSideBiomass = m_rightSideBiomass = 0.0f;
	m_rootBiomassHistory.clear();
	m_shootBiomassHistory.clear();
}

bool Tree::OnInspectTreeGrowthSettings(TreeGrowthSettings& treeGrowthSettings)
{
	bool changed = false;
	if (ImGui::Checkbox("Enable Root", &treeGrowthSettings.m_enableRoot)) changed = true;
	if (ImGui::Checkbox("Enable Shoot", &treeGrowthSettings.m_enableShoot)) changed = true;
	if (ImGui::Checkbox("Auto balance vigor", &treeGrowthSettings.m_autoBalance)) changed = true;
	if (ImGui::Checkbox("Receive water", &treeGrowthSettings.m_collectRootFlux)) changed = true;
	if (ImGui::Checkbox("Receive nitrite", &treeGrowthSettings.m_collectNitrite)) changed = true;
	if (ImGui::Checkbox("Enable Branch collision detection", &treeGrowthSettings.m_enableBranchCollisionDetection)) changed = true;
	if (ImGui::Checkbox("Enable Root collision detection", &treeGrowthSettings.m_enableRootCollisionDetection)) changed = true;

	if (!treeGrowthSettings.m_collectRootFlux)
	{
		if (ImGui::TreeNode("Vigor filling rates"))
		{
			if (ImGui::SliderFloat("Node development", &treeGrowthSettings.m_nodeDevelopmentalVigorFillingRate, 0.0f, 1.0f)) changed = true;
			if (ImGui::Button("Reset"))
			{
				treeGrowthSettings.m_nodeDevelopmentalVigorFillingRate = 1.0f;
				changed = true;
			}
			ImGui::TreePop();
		}
	}
	if (ImGui::Checkbox("Enable space colonization", &treeGrowthSettings.m_useSpaceColonization))changed = true;
	if (treeGrowthSettings.m_useSpaceColonization)
	{
		if (ImGui::Checkbox("Space colonization auto resize", &treeGrowthSettings.m_spaceColonizationAutoResize))changed = true;
	}

	
	return changed;
}

std::shared_ptr<Mesh> Tree::GenerateBranchMesh(const TreeMeshGeneratorSettings& meshGeneratorSettings)
{
	std::vector<Vertex> vertices;
	std::vector<unsigned int> indices;
	const CylindricalMeshGenerator<ShootGrowthData, ShootStemGrowthData, InternodeGrowthData> meshGenerator{};
	meshGenerator.Generate(m_treeModel.PeekShootSkeleton(), vertices, indices, meshGeneratorSettings, 999.0f);

	auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
	VertexAttributes attributes{};
	attributes.m_texCoord = true;
	mesh->SetVertices(attributes, vertices, indices);
	return mesh;
}

std::shared_ptr<Mesh> Tree::GenerateFoliageMesh(const TreeMeshGeneratorSettings& meshGeneratorSettings)
{
	std::vector<Vertex> vertices;
	std::vector<unsigned int> indices;

	auto quadMesh = Resources::GetResource<Mesh>("PRIMITIVE_QUAD");
	auto& quadTriangles = quadMesh->UnsafeGetTriangles();
	auto quadVerticesSize = quadMesh->GetVerticesAmount();
	size_t offset = 0;
	auto treeDescriptor = m_treeDescriptor.Get<TreeDescriptor>();
	const auto& foliageOverrideSettings = (!meshGeneratorSettings.m_foliageOverride && treeDescriptor) ? treeDescriptor->m_foliageParameters : meshGeneratorSettings.m_foliageOverrideSettings;
	const auto& nodeList = m_treeModel.PeekShootSkeleton().RefSortedNodeList();
	for (const auto& internodeHandle : nodeList) {
		const auto& internode = m_treeModel.PeekShootSkeleton().PeekNode(internodeHandle);
		const auto& internodeInfo = internode.m_info;

		
		if (internodeInfo.m_thickness < foliageOverrideSettings.m_maxNodeThickness
			&& internodeInfo.m_rootDistance > foliageOverrideSettings.m_minRootDistance
			&& internodeInfo.m_endDistance < foliageOverrideSettings.m_maxEndDistance) {
			for (int i = 0; i < foliageOverrideSettings.m_leafCountPerInternode; i++)
			{
				auto leafSize = foliageOverrideSettings.m_leafSize * internode.m_data.m_lightIntensity;
				glm::quat rotation = internodeInfo.m_globalDirection * glm::quat(glm::radians(glm::linearRand(glm::vec3(0.0f), glm::vec3(360.0f))));
				auto front = rotation * glm::vec3(0, 0, -1);
				auto foliagePosition = internodeInfo.m_globalPosition + front * (leafSize.y) + glm::sphericalRand(1.0f) * glm::linearRand(0.0f, foliageOverrideSettings.m_positionVariance);
				auto leafTransform = glm::translate(foliagePosition) * glm::mat4_cast(rotation) * glm::scale(glm::vec3(leafSize.x, 1.0f, leafSize.y));

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
	return mesh;
}

void Tree::ExportOBJ(const std::filesystem::path& path, const TreeMeshGeneratorSettings& meshGeneratorSettings)
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
			if (meshGeneratorSettings.m_enableBranch) {
				const auto branchMesh = GenerateBranchMesh(meshGeneratorSettings);
				if (branchMesh) {
					auto& vertices = branchMesh->UnsafeGetVertices();
					auto& triangles = branchMesh->UnsafeGetTriangles();
					if (!vertices.empty() && !triangles.empty()) {
						std::string header =
							"#Vertices: " + std::to_string(vertices.size()) +
							", tris: " + std::to_string(triangles.size());
						header += "\n";
						of.write(header.c_str(), header.size());
						of.flush();
						std::stringstream data;
						data << "o tree " + std::to_string(0) + "\n";
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
					}
				}
			}
			if (meshGeneratorSettings.m_enableFoliage) {
				const auto foliageMesh = GenerateFoliageMesh(meshGeneratorSettings);
				if (foliageMesh) {

					auto& vertices = foliageMesh->UnsafeGetVertices();
					auto& triangles = foliageMesh->UnsafeGetTriangles();
					if (!vertices.empty() && !triangles.empty()) {
						std::string header =
							"#Vertices: " + std::to_string(vertices.size()) +
							", tris: " + std::to_string(triangles.size());
						header += "\n";
						of.write(header.c_str(), header.size());
						of.flush();
						std::stringstream data;
						data << "o tree " + std::to_string(0) + "\n";
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
					}
				}
			}
			of.close();
		}
	}
}

bool Tree::TryGrow(float deltaTime) {
	const auto scene = GetScene();
	const auto treeDescriptor = m_treeDescriptor.Get<TreeDescriptor>();
	const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
	if (!treeDescriptor) {
		EVOENGINE_ERROR("No tree descriptor!");
		return false;
	}
	if (!m_soil.Get<Soil>()) {
		EVOENGINE_ERROR("No soil model!");
		return false;
	}
	if (!m_climate.Get<Climate>()) {
		EVOENGINE_ERROR("No climate model!");
		return false;
	}
	const auto soil = m_soil.Get<Soil>();
	const auto climate = m_climate.Get<Climate>();
	const auto owner = GetOwner();
	PrepareControllers(treeDescriptor);
	const bool grown = m_treeModel.Grow(deltaTime, scene->GetDataComponent<GlobalTransform>(owner).m_value, soil->m_soilModel, climate->m_climateModel,
		m_rootGrowthController, m_shootGrowthController);
	if (grown)
	{
		m_treeVisualizer.ClearSelections();
		m_treeVisualizer.m_needUpdate = true;
	}
	if (m_enableHistory && m_treeModel.m_iteration % m_historyIteration == 0) m_treeModel.Step();

	if (m_recordBiomassHistory)
	{
		const auto& baseRootNode = m_treeModel.RefRootSkeleton().RefNode(0);
		const auto& baseShootNode = m_treeModel.RefShootSkeleton().RefNode(0);
		m_rootBiomassHistory.emplace_back(baseRootNode.m_data.m_biomass + baseRootNode.m_data.m_descendentTotalBiomass);
		m_shootBiomassHistory.emplace_back(baseShootNode.m_data.m_biomass + baseShootNode.m_data.m_descendentTotalBiomass);
	}

	if (m_splitRootTest)
	{
		const auto& rootNodeList = m_treeModel.RefRootSkeleton().RefSortedNodeList();
		m_leftSideBiomass = m_rightSideBiomass = 0.0f;
		for (const auto& rootNodeHandle : rootNodeList)
		{
			const auto& rootNode = m_treeModel.RefRootSkeleton().RefNode(rootNodeHandle);
			if (rootNode.m_info.m_globalPosition.x < 0.0f)
			{
				m_leftSideBiomass += rootNode.m_data.m_biomass;
			}
			else
			{
				m_rightSideBiomass += rootNode.m_data.m_biomass;
			}
		}
	}

	return grown;
}

bool Tree::TryGrowSubTree(const NodeHandle internodeHandle, const float deltaTime)
{
	const auto scene = GetScene();
	const auto treeDescriptor = m_treeDescriptor.Get<TreeDescriptor>();
	const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
	if (!ecoSysLabLayer) return false;

	if (!m_climate.Get<Climate>()) m_climate = ecoSysLabLayer->m_climateHolder;
	if (!m_soil.Get<Soil>()) m_soil = ecoSysLabLayer->m_soilHolder;

	if (!treeDescriptor) {
		EVOENGINE_ERROR("No tree descriptor!");
		return false;
	}
	if (!m_soil.Get<Soil>()) {
		EVOENGINE_ERROR("No soil model!");
		return false;
	}
	if (!m_climate.Get<Climate>()) {
		EVOENGINE_ERROR("No climate model!");
		return false;
	}
	const auto soil = m_soil.Get<Soil>();
	const auto climate = m_climate.Get<Climate>();
	const auto owner = GetOwner();

	PrepareControllers(treeDescriptor);
	const bool grown = m_treeModel.GrowSubTree(deltaTime, internodeHandle, scene->GetDataComponent<GlobalTransform>(owner).m_value, climate->m_climateModel, m_shootGrowthController);
	if (grown)
	{
		m_treeVisualizer.ClearSelections();
		m_treeVisualizer.m_needUpdate = true;
	}
	if (m_enableHistory && m_treeModel.m_iteration % m_historyIteration == 0) m_treeModel.Step();

	m_treeVisualizer.m_needUpdate = true;
	m_treeVisualizer.m_iteration = m_treeModel.CurrentIteration();
	ecoSysLabLayer->m_needFullFlowUpdate = true;

}


void Tree::Serialize(YAML::Emitter& out)
{
	m_treeDescriptor.Save("m_treeDescriptor", out);
}



void Tree::Deserialize(const YAML::Node& in)
{
	m_treeDescriptor.Load("m_treeDescriptor", in);
}

void Tree::ClearMeshes() const
{
	const auto scene = GetScene();
	const auto self = GetOwner();
	const auto children = scene->GetChildren(self);
	for (const auto& child : children) {
		auto name = scene->GetEntityName(child);
		if (name == "Branch Mesh") {
			scene->DeleteEntity(child);
		}
		else if (name == "Root Mesh") {
			scene->DeleteEntity(child);
		}
		else if (name == "Foliage Mesh") {
			scene->DeleteEntity(child);
		}
		else if (name == "Fruit Mesh") {
			scene->DeleteEntity(child);
		}
	}
}

void Tree::ClearStrands() const
{
	const auto scene = GetScene();
	const auto self = GetOwner();
	const auto children = scene->GetChildren(self);
	for (const auto& child : children) {
		auto name = scene->GetEntityName(child);
		if (name == "Twig Strands") {
			scene->DeleteEntity(child);
		}
		else if (name == "Fine Root Strands") {
			scene->DeleteEntity(child);
		}
	}
}

void Tree::GenerateGeometry(const TreeMeshGeneratorSettings& meshGeneratorSettings, int iteration) {
	const auto scene = GetScene();
	const auto self = GetOwner();
	const auto children = scene->GetChildren(self);

	ClearMeshes();
	ClearStrands();
	auto actualIteration = iteration;
	if (actualIteration < 0 || actualIteration > m_treeModel.CurrentIteration())
	{
		actualIteration = m_treeModel.CurrentIteration();
	}
	if (meshGeneratorSettings.m_enableBranch)
	{
		Entity branchEntity;
		branchEntity = scene->CreateEntity("Branch Mesh");
		scene->SetParent(branchEntity, self);
		
		auto mesh = GenerateBranchMesh(meshGeneratorSettings);
		auto material = ProjectManager::CreateTemporaryAsset<Material>();
		auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(branchEntity).lock();
		if (meshGeneratorSettings.m_presentationOverride)
		{
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
	if (meshGeneratorSettings.m_enableRoot)
	{
		Entity rootEntity;
		rootEntity = scene->CreateEntity("Root Mesh");
		scene->SetParent(rootEntity, self);
		std::vector<Vertex> vertices;
		std::vector<unsigned int> indices;
		switch (meshGeneratorSettings.m_rootMeshType)
		{
		case 0:
		{
			CylindricalMeshGenerator<RootGrowthData, RootStemGrowthData, RootNodeGrowthData> meshGenerator;
			meshGenerator.Generate(m_treeModel.PeekRootSkeleton(actualIteration), vertices, indices,
				meshGeneratorSettings, glm::min(m_treeModel.PeekShootSkeleton(actualIteration).PeekNode(0).m_info.m_thickness,
					m_treeModel.PeekRootSkeleton(actualIteration).PeekNode(0).m_info.m_thickness));
		}
		break;
		case 1:
		{
			const auto treeDescriptor = m_treeDescriptor.Get<TreeDescriptor>();
			float minRadius = 0.01f;
			if (treeDescriptor)
			{
				minRadius = treeDescriptor->m_rootGrowthParameters.m_endNodeThickness;
			}
			VoxelMeshGenerator<RootGrowthData, RootStemGrowthData, RootNodeGrowthData> meshGenerator;
			meshGenerator.Generate(m_treeModel.PeekRootSkeleton(actualIteration), vertices, indices,
				meshGeneratorSettings, minRadius);
		}
		break;
		default: break;
		}
		auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
		auto material = ProjectManager::CreateTemporaryAsset<Material>();
		if (meshGeneratorSettings.m_presentationOverride)
		{
			material->m_materialProperties.m_albedoColor = meshGeneratorSettings.m_presentationOverrideSettings.m_rootOverrideColor;
		}
		else {
			material->m_materialProperties.m_albedoColor = glm::vec3(80, 60, 50) / 255.0f;
		}
		material->m_materialProperties.m_roughness = 1.0f;
		material->m_materialProperties.m_metallic = 0.0f;
		VertexAttributes vertexAttributes{};
		vertexAttributes.m_texCoord = true;
		mesh->SetVertices(vertexAttributes, vertices, indices);
		auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(rootEntity).lock();
		meshRenderer->m_mesh = mesh;
		meshRenderer->m_material = material;
	}
	if (meshGeneratorSettings.m_enableFineRoot)
	{
		Entity fineRootEntity;
		fineRootEntity = scene->CreateEntity("Fine Root Strands");
		scene->SetParent(fineRootEntity, self);
		std::vector<glm::uint> fineRootSegments;
		std::vector<StrandPoint> fineRootPoints;
		const auto& rootSkeleton = m_treeModel.PeekRootSkeleton(actualIteration);
		const auto& rootNodeList = rootSkeleton.RefSortedNodeList();
		for (int rootNodeHandle : rootNodeList)
		{
			const auto& rootNode = rootSkeleton.PeekNode(rootNodeHandle);
			const auto& rootNodeData = rootNode.m_data;
			const auto& rootNodeInfo = rootNode.m_info;
			std::vector<std::vector<glm::vec4>> fineRoots{};
			if (rootNodeInfo.m_thickness < meshGeneratorSettings.m_fineRootParameters.m_maxNodeThickness && rootNodeData.m_rootDistance > meshGeneratorSettings.m_fineRootParameters.m_minRootDistance)
			{
				int fineRootCount = rootNodeInfo.m_length / meshGeneratorSettings.m_fineRootParameters.m_unitDistance;
				fineRoots.resize(fineRootCount);

				auto desiredGlobalRotation = rootNodeInfo.m_regulatedGlobalRotation * glm::quat(glm::vec3(
					glm::radians(meshGeneratorSettings.m_fineRootParameters.m_branchingAngle), 0.0f,
					glm::radians(glm::radians(
						glm::linearRand(0.0f,
							360.0f)))));

				glm::vec3 directionStart = rootNodeInfo.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
				glm::vec3 directionEnd = directionStart;

				glm::vec3 positionStart = rootNodeInfo.m_globalPosition;
				glm::vec3 positionEnd =
					positionStart + rootNodeInfo.m_length * meshGeneratorSettings.m_lineLengthFactor * rootNodeInfo.m_globalDirection;

				BezierCurve curve = BezierCurve(
					positionStart,
					positionStart +
					(meshGeneratorSettings.m_smoothness ? rootNodeInfo.m_length * meshGeneratorSettings.m_baseControlPointRatio : 0.0f) * directionStart,
					positionEnd -
					(meshGeneratorSettings.m_smoothness ? rootNodeInfo.m_length * meshGeneratorSettings.m_branchControlPointRatio : 0.0f) * directionEnd,
					positionEnd);

				for (int fineRootIndex = 0; fineRootIndex < fineRootCount; fineRootIndex++) {
					glm::vec3 positionWalker = curve.GetPoint(static_cast<float>(fineRootIndex) / fineRootCount);
					fineRoots[fineRootIndex].resize(meshGeneratorSettings.m_fineRootParameters.m_segmentSize);
					const float rollAngle = glm::radians(glm::linearRand(0.0f, 360.0f));
					for (int fineRootPointIndex = 0; fineRootPointIndex < meshGeneratorSettings.m_fineRootParameters.m_segmentSize; fineRootPointIndex++)
					{
						fineRoots[fineRootIndex][fineRootPointIndex] = glm::vec4(positionWalker, meshGeneratorSettings.m_fineRootParameters.m_thickness);
						desiredGlobalRotation = rootNodeInfo.m_regulatedGlobalRotation * glm::quat(glm::vec3(
							glm::radians(glm::gaussRand(0.f, meshGeneratorSettings.m_fineRootParameters.m_apicalAngleVariance) + meshGeneratorSettings.m_fineRootParameters.m_branchingAngle), 0.0f,
							rollAngle));

						auto fineRootFront = desiredGlobalRotation * glm::vec3(0, 0, -1);
						positionWalker = positionWalker + fineRootFront * meshGeneratorSettings.m_fineRootParameters.m_segmentLength;
					}
				}

			}

			for (const auto& fineRoot : fineRoots) {
				const auto fineRootSegmentSize = fineRoot.size();
				const auto fineRootControlPointSize = fineRoot.size() + 3;
				const auto totalTwigPointSize = fineRootPoints.size();
				const auto totalTwigSegmentSize = fineRootSegments.size();
				fineRootPoints.resize(totalTwigPointSize + fineRootControlPointSize);
				fineRootSegments.resize(totalTwigSegmentSize + fineRootSegmentSize);

				for (int i = 0; i < fineRootControlPointSize; i++) {
					auto& p = fineRootPoints[totalTwigPointSize + i];
					p.m_position = glm::vec3(fineRoot[glm::clamp(i - 2, 0, static_cast<int>(fineRootSegmentSize - 1))]);
					p.m_thickness = fineRoot[glm::clamp(i - 2, 0, static_cast<int>(fineRootSegmentSize - 1))].w;
				}
				fineRootPoints[totalTwigPointSize].m_position = glm::vec3(fineRoot[0]) * 2.0f - glm::vec3(fineRoot[1]);
				fineRootPoints[totalTwigPointSize].m_thickness = fineRoot[0].w * 2.0f - fineRoot[1].w;

				fineRootPoints[totalTwigPointSize + fineRootControlPointSize - 1].m_position = glm::vec3(fineRoot[fineRootSegmentSize - 1]) * 2.0f - glm::vec3(fineRoot[fineRootSegmentSize - 2]);
				fineRootPoints[totalTwigPointSize + fineRootControlPointSize - 1].m_thickness = fineRoot[fineRootSegmentSize - 1].w * 2.0f - fineRoot[fineRootSegmentSize - 2].w;


				for (int i = 0; i < fineRootSegmentSize; i++) {
					fineRootSegments[totalTwigSegmentSize + i] = totalTwigPointSize + i;
				}
			}
		}

		auto strands = ProjectManager::CreateTemporaryAsset<Strands>();
		auto material = ProjectManager::CreateTemporaryAsset<Material>();
		if (meshGeneratorSettings.m_presentationOverride)
		{
			material->m_materialProperties.m_albedoColor = meshGeneratorSettings.m_presentationOverrideSettings.m_rootOverrideColor;
		}
		else {
			material->m_materialProperties.m_albedoColor = glm::vec3(80, 60, 50) / 255.0f;
		}
		material->m_materialProperties.m_roughness = 1.0f;
		material->m_materialProperties.m_metallic = 0.0f;
		StrandPointAttributes strandPointAttributes{};
		strands->SetSegments(strandPointAttributes, fineRootSegments, fineRootPoints);
		auto strandsRenderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(fineRootEntity).lock();
		strandsRenderer->m_strands = strands;
		strandsRenderer->m_material = material;
	}

	if (meshGeneratorSettings.m_enableTwig)
	{
		Entity twigEntity;
		twigEntity = scene->CreateEntity("Twig Strands");
		scene->SetParent(twigEntity, self);
		std::vector<glm::uint> twigSegments;
		std::vector<StrandPoint> twigPoints;
		const auto& shootSkeleton = m_treeModel.PeekShootSkeleton(actualIteration);
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
		if (meshGeneratorSettings.m_presentationOverride)
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
		scene->SetParent(foliageEntity, self);

		std::vector<Vertex> vertices;
		std::vector<unsigned int> indices;
		auto mesh = GenerateFoliageMesh(meshGeneratorSettings);
		auto material = ProjectManager::CreateTemporaryAsset<Material>();
		VertexAttributes vertexAttributes{};
		vertexAttributes.m_texCoord = true;
		mesh->SetVertices(vertexAttributes, vertices, indices);
		if (meshGeneratorSettings.m_foliageOverride)
		{
			material->m_materialProperties.m_albedoColor = meshGeneratorSettings.m_presentationOverrideSettings.m_foliageOverrideColor;
			auto texRef = meshGeneratorSettings.m_foliageTexture;
			if (texRef.Get<Texture2D>())
			{
				material->SetAlbedoTexture(texRef.Get<Texture2D>());

			}
		}
		else {
			material->m_materialProperties.m_albedoColor = glm::vec3(152 / 255.0f, 203 / 255.0f, 0 / 255.0f);
		}
		material->m_materialProperties.m_roughness = 0.0f;

		auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(foliageEntity).lock();
		meshRenderer->m_mesh = mesh;
		meshRenderer->m_material = material;

	}
	if (meshGeneratorSettings.m_enableFruit)
	{
		Entity fruitEntity;
		fruitEntity = scene->CreateEntity("Fruit Mesh");
		scene->SetParent(fruitEntity, self);
		std::vector<Vertex> vertices;
		std::vector<unsigned int> indices;
		auto fruitMesh = Resources::GetResource<Mesh>("PRIMITIVE_SPHERE");
		auto& fruitTriangles = fruitMesh->UnsafeGetTriangles();
		auto fruitVerticesSize = fruitMesh->GetVerticesAmount();
		size_t offset = 0;

		const auto& nodeList = m_treeModel.PeekShootSkeleton(actualIteration).RefSortedNodeList();
		for (const auto& internodeHandle : nodeList) {
			const auto& internode = m_treeModel.PeekShootSkeleton(actualIteration).PeekNode(internodeHandle);
			const auto& internodeInfo = internode.m_info;
			const auto& internodeData = internode.m_data;
			if (!meshGeneratorSettings.m_foliageOverride) {
				for (const auto& bud : internodeData.m_buds) {
					if (bud.m_status != BudStatus::Flushed) continue;
					if (bud.m_reproductiveModule.m_maturity <= 0.0f) continue;
					if (bud.m_type == BudType::Fruit)
					{
						auto matrix = bud.m_reproductiveModule.m_transform;
						Vertex archetype;
						for (auto i = 0; i < fruitMesh->GetVerticesAmount(); i++) {
							archetype.m_position =
								matrix * glm::vec4(fruitMesh->UnsafeGetVertices()[i].m_position, 1.0f);
							archetype.m_normal = glm::normalize(glm::vec3(
								matrix * glm::vec4(fruitMesh->UnsafeGetVertices()[i].m_normal, 0.0f)));
							archetype.m_tangent = glm::normalize(glm::vec3(
								matrix *
								glm::vec4(fruitMesh->UnsafeGetVertices()[i].m_tangent, 0.0f)));
							archetype.m_texCoord =
								fruitMesh->UnsafeGetVertices()[i].m_texCoord;
							vertices.push_back(archetype);
						}
						for (auto triangle : fruitTriangles) {
							triangle.x += offset;
							triangle.y += offset;
							triangle.z += offset;
							indices.push_back(triangle.x);
							indices.push_back(triangle.y);
							indices.push_back(triangle.z);
						}
						offset += fruitVerticesSize;
					}
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
		auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(fruitEntity).lock();
		meshRenderer->m_mesh = mesh;
		meshRenderer->m_material = material;
	}
}

void Tree::RegisterVoxel()
{
	const auto scene = GetScene();
	const auto owner = GetOwner();
	const auto globalTransform = scene->GetDataComponent<GlobalTransform>(owner).m_value;
	m_treeModel.m_index = owner.GetIndex();
	m_treeModel.RegisterVoxel(globalTransform, m_climate.Get<Climate>()->m_climateModel, m_shootGrowthController);
}

void Tree::FromLSystemString(const std::shared_ptr<LSystemString>& lSystemString)
{
}

void Tree::FromTreeGraph(const std::shared_ptr<TreeGraph>& treeGraph)
{
}

void Tree::FromTreeGraphV2(const std::shared_ptr<TreeGraphV2>& treeGraphV2)
{
}

bool Tree::ExportIOTree(const std::filesystem::path& path) const
{
	treeio::ArrayTree tree{};
	m_treeModel.ExportTreeIOSkeleton(tree);
	return tree.saveTree(path.string());
}

void Tree::ExportRadialBoundingVolume(const std::shared_ptr<RadialBoundingVolume>& rbv) const
{
	const auto& sortedInternodeList = m_treeModel.m_shootSkeleton.RefSortedNodeList();
	const auto& skeleton = m_treeModel.m_shootSkeleton;
	std::vector<glm::vec3> points;
	for (const auto& nodeHandle : sortedInternodeList)
	{
		const auto& node = skeleton.PeekNode(nodeHandle);
		points.emplace_back(node.m_info.m_globalPosition);
		points.emplace_back(node.m_info.GetGlobalEndPosition());
	}
	rbv->CalculateVolume(points);
}

void TreeDescriptor::OnCreate() {

}
bool TreeDescriptor::OnInspectFoliageParameters(FoliageParameters& foliageParameters) {
	bool changed = false;
	if (ImGui::TreeNodeEx("Foliage Parameters", ImGuiTreeNodeFlags_DefaultOpen))
	{
		if (ImGui::DragFloat3("Leaf size", &foliageParameters.m_leafSize.x, 0.01f, 0.0f, 1.0f)) changed = true;
		if (ImGui::DragInt("Leaf per node", &foliageParameters.m_leafCountPerInternode, 1, 0, 50)) changed = true;
		if (ImGui::DragFloat("Position variance", &foliageParameters.m_positionVariance, 0.01f, 0.0f, 1.0f)) changed = true;

		if (ImGui::DragFloat("Max node thickness", &foliageParameters.m_maxNodeThickness, 0.001f, 0.0f, 5.0f)) changed = true;
		if (ImGui::DragFloat("Min root distance", &foliageParameters.m_minRootDistance, 0.001f, 0.0f, 1.0f)) changed = true;
		if (ImGui::DragFloat("Max end distance", &foliageParameters.m_maxEndDistance, 0.001f, 0.0f, 1.0f)) changed = true;
		ImGui::TreePop();
	}
	return changed;
}

bool TreeDescriptor::OnInspectShootGrowthParameters(ShootGrowthParameters& treeGrowthParameters) {
	bool changed = false;
	if (ImGui::TreeNodeEx("Shoot Parameters", ImGuiTreeNodeFlags_DefaultOpen)) {
		changed = ImGui::DragFloat("Internode growth rate", &treeGrowthParameters.m_internodeGrowthRate, 0.01f, 0.0f, 1.0f) || changed;
		changed = ImGui::DragFloat("Leaf growth rate", &treeGrowthParameters.m_leafGrowthRate, 0.01f, 0.0f, 1.0f) || changed;
		changed = ImGui::DragFloat("Fruit growth rate", &treeGrowthParameters.m_fruitGrowthRate, 0.01f, 0.0f, 1.0f) || changed;

		if (ImGui::TreeNodeEx("Bud", ImGuiTreeNodeFlags_DefaultOpen)) {
			changed = ImGui::DragInt3("Bud count lateral/fruit/leaf", &treeGrowthParameters.m_lateralBudCount, 1, 0, 3) || changed;
			changed = ImGui::DragFloat2("Branching Angle mean/var", &treeGrowthParameters.m_branchingAngleMeanVariance.x, 0.01f, 0.0f, 100.0f) || changed;
			changed = ImGui::DragFloat2("Roll Angle mean/var", &treeGrowthParameters.m_rollAngleMeanVariance.x, 0.01f, 0.0f, 100.0f) || changed;
			changed = ImGui::DragFloat2("Apical Angle mean/var", &treeGrowthParameters.m_apicalAngleMeanVariance.x, 0.01f, 0.0f, 100.0f) || changed;
			changed = ImGui::DragFloat("Gravitropism", &treeGrowthParameters.m_gravitropism, 0.01f) || changed;
			changed = ImGui::DragFloat("Phototropism", &treeGrowthParameters.m_phototropism, 0.01f) || changed;
			changed = ImGui::DragFloat("Apical bud kill prob", &treeGrowthParameters.m_apicalInternodeKillProbability, 0.00001f, 0.0f, 1.0f, "%.5f") || changed;
			changed = ImGui::DragFloat("Lateral bud flushing prob", &treeGrowthParameters.m_lateralBudFlushProbability, 0.00001f, 0.0f, 1.0f, "%.5f") || changed;
			changed = ImGui::DragFloat4("Leaf flushing prob/temp range", &treeGrowthParameters.m_leafBudFlushingProbabilityTemperatureRange.x, 0.00001f, 0.0f, 1.0f, "%.5f") || changed;
			changed = ImGui::DragFloat4("Fruit flushing prob/temp range", &treeGrowthParameters.m_fruitBudFlushingProbabilityTemperatureRange.x, 0.00001f, 0.0f, 1.0f, "%.5f") || changed;
			changed = ImGui::DragFloat2("Apical control base/age", &treeGrowthParameters.m_apicalControl, 0.01f) || changed;
			changed = ImGui::DragFloat3("Apical dominance base/age/dist", &treeGrowthParameters.m_apicalDominance, 0.01f) || changed;
			changed = ImGui::DragFloat3("Vigor requirement shoot/leaf/fruit", &treeGrowthParameters.m_internodeVigorRequirement, 0.01f) || changed;
			changed = ImGui::DragFloat("Vigor requirement aggregation loss", &treeGrowthParameters.m_vigorRequirementAggregateLoss, 0.001f, 0.0f, 1.0f) || changed;
			ImGui::TreePop();
		}
		if (ImGui::TreeNodeEx("Internode", ImGuiTreeNodeFlags_DefaultOpen))
		{
			changed = ImGui::DragFloat("Internode length", &treeGrowthParameters.m_internodeLength, 0.01f) || changed;
			changed = ImGui::DragFloat("Internode length thickness factor", &treeGrowthParameters.m_internodeLengthThicknessFactor, 0.01f, 0.0f, 1.0f) || changed;
			changed = ImGui::DragFloat3("Thickness min/factor/age", &treeGrowthParameters.m_endNodeThickness, 0.00001f, 0.0f, 1.0f, "%.6f") || changed;
			changed = ImGui::DragFloat3("Sagging thickness/reduction/max", &treeGrowthParameters.m_saggingFactorThicknessReductionMax.x, 0.01f, 0.0f, 1.0f, "%.5f") || changed;
			changed = ImGui::DragFloat("Low Branch Pruning", &treeGrowthParameters.m_lowBranchPruning, 0.01f) || changed;
			changed = ImGui::DragFloat("Low Branch Pruning Thickness factor", &treeGrowthParameters.m_lowBranchPruningThicknessFactor, 0.01f) || changed;
			changed = ImGui::DragFloat("Light Pruning", &treeGrowthParameters.m_lightPruningFactor, 0.01f) || changed;
			changed = ImGui::DragFloat("Max Space Occupancy", &treeGrowthParameters.m_maxSpaceOccupancy, 0.001f, 0.0f, 1.0f, "%.3f") || changed;

			ImGui::TreePop();
		}

		if (ImGui::TreeNodeEx("Leaf"))
		{
			changed = ImGui::DragFloat3("Size", &treeGrowthParameters.m_maxLeafSize.x, 0.01f) || changed;
			changed = ImGui::DragFloat("Position Variance", &treeGrowthParameters.m_leafPositionVariance, 0.01f) || changed;
			changed = ImGui::DragFloat("Random rotation", &treeGrowthParameters.m_leafRotationVariance, 0.01f) || changed;
			changed = ImGui::DragFloat("Chlorophyll Loss", &treeGrowthParameters.m_leafChlorophyllLoss, 0.01f) || changed;
			changed = ImGui::DragFloat("Chlorophyll temperature", &treeGrowthParameters.m_leafChlorophyllSynthesisFactorTemperature, 0.01f) || changed;
			changed = ImGui::DragFloat("Drop prob", &treeGrowthParameters.m_leafFallProbability, 0.01f) || changed;
			changed = ImGui::DragFloat("Distance To End Limit", &treeGrowthParameters.m_leafDistanceToBranchEndLimit, 0.01f) || changed;
			ImGui::TreePop();
		}
		if (ImGui::TreeNodeEx("Fruit"))
		{
			changed = ImGui::DragFloat3("Size", &treeGrowthParameters.m_maxFruitSize.x, 0.01f) || changed;
			changed = ImGui::DragFloat("Position Variance", &treeGrowthParameters.m_fruitPositionVariance, 0.01f) || changed;
			changed = ImGui::DragFloat("Random rotation", &treeGrowthParameters.m_fruitRotationVariance, 0.01f) || changed;

			changed = ImGui::DragFloat("Drop prob", &treeGrowthParameters.m_fruitFallProbability, 0.01f) || changed;
			ImGui::TreePop();
		}
		ImGui::TreePop();
	}

	return changed;
}

bool TreeDescriptor::OnInspectRootGrowthParameters(RootGrowthParameters& rootGrowthParameters) {
	bool changed = false;
	if (ImGui::TreeNodeEx("Root Growth Parameters")) {
		if (ImGui::TreeNodeEx("Structure", ImGuiTreeNodeFlags_DefaultOpen)) {
			changed = ImGui::DragFloat("Root node length", &rootGrowthParameters.m_rootNodeLength, 0.01f) || changed;
			changed = ImGui::DragFloat("Root node elongation rate", &rootGrowthParameters.m_rootNodeGrowthRate, 0.01f) || changed;

			changed = ImGui::DragFloat3("Thickness min/factor/age", &rootGrowthParameters.m_endNodeThickness, 0.00001f, 0.0f, 1.0f, "%.6f") || changed;

			ImGui::TreePop();
		}
		if (ImGui::TreeNodeEx("Growth", ImGuiTreeNodeFlags_DefaultOpen))
		{
			changed = ImGui::DragFloat("Root node elongation vigor requirement", &rootGrowthParameters.m_rootNodeVigorRequirement, 0.01f) || changed;
			changed = ImGui::DragFloat("Vigor requirement aggregation loss", &rootGrowthParameters.m_vigorRequirementAggregateLoss, 0.001f, 0.0f, 1.0f) || changed;

			changed = ImGui::DragFloat2("Branching Angle mean/var", &rootGrowthParameters.m_branchingAngleMeanVariance.x, 0.01f) || changed;
			changed = ImGui::DragFloat2("Roll Angle mean/var", &rootGrowthParameters.m_rollAngleMeanVariance.x, 0.01f) || changed;
			changed = ImGui::DragFloat2("Apical Angle mean/var", &rootGrowthParameters.m_apicalAngleMeanVariance.x, 0.01f) || changed;
			changed = ImGui::DragFloat2("Environmental friction base/factor", &rootGrowthParameters.m_environmentalFriction, 0.01f) || changed;
			changed = ImGui::DragFloat2("Apical control base/age", &rootGrowthParameters.m_apicalControl, 0.01f) || changed;
			changed = ImGui::DragFloat3("Apical dominance base/age/dist", &rootGrowthParameters.m_apicalDominance, 0.01f) || changed;
			changed = ImGui::DragFloat("Tropism switching prob", &rootGrowthParameters.m_tropismSwitchingProbability, 0.01f) || changed;
			changed = ImGui::DragFloat("Tropism switching prob dist factor", &rootGrowthParameters.m_tropismSwitchingProbabilityDistanceFactor, 0.01f) || changed;
			changed = ImGui::DragFloat("Tropism intensity", &rootGrowthParameters.m_tropismIntensity, 0.01f) || changed;
			changed = ImGui::DragFloat("Branching probability", &rootGrowthParameters.m_branchingProbability, 0.00001f, 0.0f, 1.0f, "%.5f") || changed;
			ImGui::TreePop();
		}
		ImGui::TreePop();
	}
	return changed;
}

void TreeDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
	bool changed = false;
	const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
	const auto soil = ecoSysLabLayer->m_soilHolder.Get<Soil>();
	const auto climate = ecoSysLabLayer->m_climateHolder.Get<Climate>();
	if (soil && climate) {
		if (ImGui::Button("Instantiate")) {
			const auto scene = Application::GetActiveScene();
			const auto treeEntity = scene->CreateEntity(GetTitle());
			const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
			float height = 0;
			const auto soilDescriptor = soil->m_soilDescriptor.Get<SoilDescriptor>();
			if (soilDescriptor)
			{
				const auto heightField = soilDescriptor->m_heightField.Get<HeightField>();
				if (heightField) height = heightField->GetValue({ 0.0f, 0.0f }) - 0.05f;
			}
			GlobalTransform globalTransform;
			globalTransform.SetPosition(glm::vec3(0, height, 0));
			scene->SetDataComponent(treeEntity, globalTransform);
			tree->m_treeDescriptor = ProjectManager::GetAsset(GetHandle());
		}
	}
	else
	{
		ImGui::Text("Attach soil and climate entity to instantiate!");
	}
	if (OnInspectShootGrowthParameters(m_shootGrowthParameters)) { changed = true; }
	if (OnInspectRootGrowthParameters(m_rootGrowthParameters)) { changed = true; }
	if (OnInspectFoliageParameters(m_foliageParameters)) { changed = true; }
	if (changed) m_saved = false;
}

void TreeDescriptor::CollectAssetRef(std::vector<AssetRef>& list) {

}
void TreeDescriptor::SerializeFoliageParameters(const std::string& name, const FoliageParameters& foliageParameters, YAML::Emitter& out)
{
	out << YAML::Key << name << YAML::BeginMap;
	out << YAML::Key << "m_leafSize" << YAML::Value << foliageParameters.m_leafSize;
	out << YAML::Key << "m_leafCountPerInternode" << YAML::Value << foliageParameters.m_leafCountPerInternode;
	out << YAML::Key << "m_positionVariance" << YAML::Value << foliageParameters.m_positionVariance;
	out << YAML::Key << "m_maxNodeThickness" << YAML::Value << foliageParameters.m_maxNodeThickness;
	out << YAML::Key << "m_minRootDistance" << YAML::Value << foliageParameters.m_minRootDistance;
	out << YAML::Key << "m_maxEndDistance" << YAML::Value << foliageParameters.m_maxEndDistance;
	out << YAML::EndMap;
}
void TreeDescriptor::SerializeShootGrowthParameters(const std::string& name, const ShootGrowthParameters& treeGrowthParameters, YAML::Emitter& out) {
	out << YAML::Key << name << YAML::BeginMap;
	out << YAML::Key << "m_internodeGrowthRate" << YAML::Value << treeGrowthParameters.m_internodeGrowthRate;
	out << YAML::Key << "m_leafGrowthRate" << YAML::Value << treeGrowthParameters.m_leafGrowthRate;
	out << YAML::Key << "m_fruitGrowthRate" << YAML::Value << treeGrowthParameters.m_fruitGrowthRate;
	//Structure
	out << YAML::Key << "m_lateralBudCount" << YAML::Value << treeGrowthParameters.m_lateralBudCount;
	out << YAML::Key << "m_fruitBudCount" << YAML::Value << treeGrowthParameters.m_fruitBudCount;
	out << YAML::Key << "m_leafBudCount" << YAML::Value << treeGrowthParameters.m_leafBudCount;
	out << YAML::Key << "m_branchingAngleMeanVariance" << YAML::Value << treeGrowthParameters.m_branchingAngleMeanVariance;
	out << YAML::Key << "m_rollAngleMeanVariance" << YAML::Value << treeGrowthParameters.m_rollAngleMeanVariance;
	out << YAML::Key << "m_apicalAngleMeanVariance" << YAML::Value << treeGrowthParameters.m_apicalAngleMeanVariance;
	out << YAML::Key << "m_gravitropism" << YAML::Value << treeGrowthParameters.m_gravitropism;
	out << YAML::Key << "m_phototropism" << YAML::Value << treeGrowthParameters.m_phototropism;
	out << YAML::Key << "m_internodeLength" << YAML::Value << treeGrowthParameters.m_internodeLength;

	out << YAML::Key << "m_endNodeThickness" << YAML::Value << treeGrowthParameters.m_endNodeThickness;
	out << YAML::Key << "m_thicknessAccumulationFactor" << YAML::Value << treeGrowthParameters.m_thicknessAccumulationFactor;
	out << YAML::Key << "m_thicknessAccumulateAgeFactor" << YAML::Value << treeGrowthParameters.m_thicknessAccumulateAgeFactor;

	//Bud
	out << YAML::Key << "m_apicalInternodeKillProbability" << YAML::Value << treeGrowthParameters.m_apicalInternodeKillProbability;
	out << YAML::Key << "m_lateralBudFlushProbability" << YAML::Value << treeGrowthParameters.m_lateralBudFlushProbability;
	out << YAML::Key << "m_leafBudFlushingProbabilityTemperatureRange" << YAML::Value << treeGrowthParameters.m_leafBudFlushingProbabilityTemperatureRange;
	out << YAML::Key << "m_fruitBudFlushingProbabilityTemperatureRange" << YAML::Value << treeGrowthParameters.m_fruitBudFlushingProbabilityTemperatureRange;

	out << YAML::Key << "m_apicalControl" << YAML::Value << treeGrowthParameters.m_apicalControl;
	out << YAML::Key << "m_apicalControlAgeFactor" << YAML::Value << treeGrowthParameters.m_apicalControlAgeFactor;

	out << YAML::Key << "m_apicalDominance" << YAML::Value << treeGrowthParameters.m_apicalDominance;
	out << YAML::Key << "m_apicalDominanceAgeFactor" << YAML::Value << treeGrowthParameters.m_apicalDominanceAgeFactor;
	out << YAML::Key << "m_apicalDominanceDistanceFactor" << YAML::Value << treeGrowthParameters.m_apicalDominanceDistanceFactor;

	out << YAML::Key << "m_leafVigorRequirement" << YAML::Value << treeGrowthParameters.m_leafVigorRequirement;
	out << YAML::Key << "m_fruitVigorRequirement" << YAML::Value << treeGrowthParameters.m_fruitVigorRequirement;
	out << YAML::Key << "m_internodeVigorRequirement" << YAML::Value << treeGrowthParameters.m_internodeVigorRequirement;
	out << YAML::Key << "m_vigorRequirementAggregateLoss" << YAML::Value << treeGrowthParameters.m_vigorRequirementAggregateLoss;
	//Internode
	out << YAML::Key << "m_lowBranchPruning" << YAML::Value << treeGrowthParameters.m_lowBranchPruning;
	out << YAML::Key << "m_lowBranchPruningThicknessFactor" << YAML::Value << treeGrowthParameters.m_lowBranchPruningThicknessFactor;
	out << YAML::Key << "m_lightPruningFactor" << YAML::Value << treeGrowthParameters.m_lightPruningFactor;
	out << YAML::Key << "m_saggingFactorThicknessReductionMax" << YAML::Value << treeGrowthParameters.m_saggingFactorThicknessReductionMax;
	out << YAML::Key << "m_maxSpaceOccupancy" << YAML::Value << treeGrowthParameters.m_maxSpaceOccupancy;

	//Foliage
	out << YAML::Key << "m_maxLeafSize" << YAML::Value << treeGrowthParameters.m_maxLeafSize;
	out << YAML::Key << "m_leafPositionVariance" << YAML::Value << treeGrowthParameters.m_leafPositionVariance;
	out << YAML::Key << "m_leafRotationVariance" << YAML::Value << treeGrowthParameters.m_leafRotationVariance;
	out << YAML::Key << "m_leafChlorophyllLoss" << YAML::Value << treeGrowthParameters.m_leafChlorophyllLoss;
	out << YAML::Key << "m_leafChlorophyllSynthesisFactorTemperature" << YAML::Value << treeGrowthParameters.m_leafChlorophyllSynthesisFactorTemperature;
	out << YAML::Key << "m_leafFallProbability" << YAML::Value << treeGrowthParameters.m_leafFallProbability;
	out << YAML::Key << "m_leafDistanceToBranchEndLimit" << YAML::Value << treeGrowthParameters.m_leafDistanceToBranchEndLimit;

	out << YAML::Key << "m_maxFruitSize" << YAML::Value << treeGrowthParameters.m_maxFruitSize;
	out << YAML::Key << "m_fruitPositionVariance" << YAML::Value << treeGrowthParameters.m_fruitPositionVariance;
	out << YAML::Key << "m_fruitRotationVariance" << YAML::Value << treeGrowthParameters.m_fruitRotationVariance;
	out << YAML::Key << "m_fruitFallProbability" << YAML::Value << treeGrowthParameters.m_fruitFallProbability;
	out << YAML::EndMap;
}
void TreeDescriptor::SerializeRootGrowthParameters(const std::string& name, const RootGrowthParameters& rootGrowthParameters, YAML::Emitter& out) {
	out << YAML::Key << name << YAML::BeginMap;

	out << YAML::Key << "m_branchingAngleMeanVariance" << YAML::Value << rootGrowthParameters.m_branchingAngleMeanVariance;
	out << YAML::Key << "m_rollAngleMeanVariance" << YAML::Value << rootGrowthParameters.m_rollAngleMeanVariance;
	out << YAML::Key << "m_apicalAngleMeanVariance" << YAML::Value << rootGrowthParameters.m_apicalAngleMeanVariance;
	out << YAML::Key << "m_rootNodeLength" << YAML::Value << rootGrowthParameters.m_rootNodeLength;
	out << YAML::Key << "m_rootNodeGrowthRate" << YAML::Value << rootGrowthParameters.m_rootNodeGrowthRate;
	out << YAML::Key << "m_endNodeThickness" << YAML::Value << rootGrowthParameters.m_endNodeThickness;
	out << YAML::Key << "m_thicknessAccumulationFactor" << YAML::Value << rootGrowthParameters.m_thicknessAccumulationFactor;
	out << YAML::Key << "m_thicknessAccumulateAgeFactor" << YAML::Value << rootGrowthParameters.m_thicknessAccumulateAgeFactor;


	out << YAML::Key << "m_rootNodeVigorRequirement" << YAML::Value << rootGrowthParameters.m_rootNodeVigorRequirement;
	out << YAML::Key << "m_vigorRequirementAggregateLoss" << YAML::Value << rootGrowthParameters.m_vigorRequirementAggregateLoss;
	out << YAML::Key << "m_environmentalFriction" << YAML::Value << rootGrowthParameters.m_environmentalFriction;
	out << YAML::Key << "m_environmentalFrictionFactor" << YAML::Value << rootGrowthParameters.m_environmentalFrictionFactor;


	out << YAML::Key << "m_apicalControl" << YAML::Value << rootGrowthParameters.m_apicalControl;
	out << YAML::Key << "m_apicalControlAgeFactor" << YAML::Value << rootGrowthParameters.m_apicalControlAgeFactor;
	out << YAML::Key << "m_apicalDominance" << YAML::Value << rootGrowthParameters.m_apicalDominance;
	out << YAML::Key << "m_apicalDominanceAgeFactor" << YAML::Value << rootGrowthParameters.m_apicalDominanceAgeFactor;
	out << YAML::Key << "m_apicalDominanceDistanceFactor" << YAML::Value << rootGrowthParameters.m_apicalDominanceDistanceFactor;
	out << YAML::Key << "m_tropismSwitchingProbability" << YAML::Value << rootGrowthParameters.m_tropismSwitchingProbability;
	out << YAML::Key << "m_tropismSwitchingProbabilityDistanceFactor" << YAML::Value << rootGrowthParameters.m_tropismSwitchingProbabilityDistanceFactor;
	out << YAML::Key << "m_tropismIntensity" << YAML::Value << rootGrowthParameters.m_tropismIntensity;
	out << YAML::Key << "m_branchingProbability" << YAML::Value << rootGrowthParameters.m_branchingProbability;

	out << YAML::EndMap;
}
void TreeDescriptor::Serialize(YAML::Emitter& out) {
	SerializeShootGrowthParameters("m_shootGrowthParameters", m_shootGrowthParameters, out);
	SerializeRootGrowthParameters("m_rootGrowthParameters", m_rootGrowthParameters, out);
	SerializeFoliageParameters("m_foliageParameters", m_foliageParameters, out);


}
void TreeDescriptor::DeserializeFoliageParameters(const std::string& name, FoliageParameters& foliageParameters, const YAML::Node& in) {
	if (in[name]) {
		auto& param = in[name];
		if (param["m_leafSize"]) foliageParameters.m_leafSize = param["m_leafSize"].as<glm::vec2>();
		if (param["m_leafCountPerInternode"]) foliageParameters.m_leafCountPerInternode = param["m_leafCountPerInternode"].as<int>();
		if (param["m_positionVariance"]) foliageParameters.m_positionVariance = param["m_positionVariance"].as<float>();
		if (param["m_maxNodeThickness"]) foliageParameters.m_maxNodeThickness = param["m_maxNodeThickness"].as<float>();
		if (param["m_minRootDistance"]) foliageParameters.m_minRootDistance = param["m_minRootDistance"].as<float>();
		if (param["m_maxEndDistance"]) foliageParameters.m_maxEndDistance = param["m_maxEndDistance"].as<float>();
	}
}
void TreeDescriptor::DeserializeShootGrowthParameters(const std::string& name, ShootGrowthParameters& treeGrowthParameters, const YAML::Node& in) {
	if (in[name]) {
		auto& param = in[name];

		if (param["m_internodeGrowthRate"]) treeGrowthParameters.m_internodeGrowthRate = param["m_internodeGrowthRate"].as<float>();
		if (param["m_leafGrowthRate"]) treeGrowthParameters.m_leafGrowthRate = param["m_leafGrowthRate"].as<float>();
		if (param["m_fruitGrowthRate"]) treeGrowthParameters.m_fruitGrowthRate = param["m_fruitGrowthRate"].as<float>();
		//Structure
		if (param["m_lateralBudCount"]) treeGrowthParameters.m_lateralBudCount = param["m_lateralBudCount"].as<int>();
		if (param["m_fruitBudCount"]) treeGrowthParameters.m_fruitBudCount = param["m_fruitBudCount"].as<int>();
		if (param["m_leafBudCount"]) treeGrowthParameters.m_leafBudCount = param["m_leafBudCount"].as<int>();

		if (param["m_branchingAngleMeanVariance"]) treeGrowthParameters.m_branchingAngleMeanVariance = param["m_branchingAngleMeanVariance"].as<glm::vec2>();
		if (param["m_rollAngleMeanVariance"]) treeGrowthParameters.m_rollAngleMeanVariance = param["m_rollAngleMeanVariance"].as<glm::vec2>();
		if (param["m_apicalAngleMeanVariance"]) treeGrowthParameters.m_apicalAngleMeanVariance = param["m_apicalAngleMeanVariance"].as<glm::vec2>();
		if (param["m_gravitropism"]) treeGrowthParameters.m_gravitropism = param["m_gravitropism"].as<float>();
		if (param["m_phototropism"]) treeGrowthParameters.m_phototropism = param["m_phototropism"].as<float>();

		if (param["m_internodeLength"]) treeGrowthParameters.m_internodeLength = param["m_internodeLength"].as<float>();
		if (param["m_internodeLengthThicknessFactor"]) treeGrowthParameters.m_internodeLengthThicknessFactor = param["m_internodeLengthThicknessFactor"].as<float>();
		if (param["m_endNodeThickness"]) treeGrowthParameters.m_endNodeThickness = param["m_endNodeThickness"].as<float>();
		if (param["m_thicknessAccumulationFactor"]) treeGrowthParameters.m_thicknessAccumulationFactor = param["m_thicknessAccumulationFactor"].as<float>();
		if (param["m_thicknessAccumulateAgeFactor"]) treeGrowthParameters.m_thicknessAccumulateAgeFactor = param["m_thicknessAccumulateAgeFactor"].as<float>();

		if (param["m_lowBranchPruning"]) treeGrowthParameters.m_lowBranchPruning = param["m_lowBranchPruning"].as<float>();
		if (param["m_lowBranchPruningThicknessFactor"]) treeGrowthParameters.m_lowBranchPruningThicknessFactor = param["m_lowBranchPruningThicknessFactor"].as<float>();
		if (param["m_lightPruningFactor"]) treeGrowthParameters.m_lightPruningFactor = param["m_lightPruningFactor"].as<float>();
		if (param["m_saggingFactorThicknessReductionMax"]) treeGrowthParameters.m_saggingFactorThicknessReductionMax = param["m_saggingFactorThicknessReductionMax"].as<glm::vec3>();
		if (param["m_maxSpaceOccupancy"]) treeGrowthParameters.m_maxSpaceOccupancy = param["m_maxSpaceOccupancy"].as<float>();

		//Bud fate

		if (param["m_lateralBudFlushingProbabilityTemperatureRange"]) treeGrowthParameters.m_lateralBudFlushProbability = param["m_lateralBudFlushingProbabilityTemperatureRange"].as<glm::vec4>().x;
		if (param["m_apicalInternodeKillProbability"]) treeGrowthParameters.m_apicalInternodeKillProbability = param["m_apicalInternodeKillProbability"].as<float>();
		if (param["m_lateralBudFlushProbability"]) treeGrowthParameters.m_lateralBudFlushProbability = param["m_lateralBudFlushProbability"].as<float>();
		if (param["m_leafBudFlushingProbabilityTemperatureRange"]) treeGrowthParameters.m_leafBudFlushingProbabilityTemperatureRange = param["m_leafBudFlushingProbabilityTemperatureRange"].as< glm::vec4>();
		if (param["m_fruitBudFlushingProbabilityTemperatureRange"]) treeGrowthParameters.m_fruitBudFlushingProbabilityTemperatureRange = param["m_fruitBudFlushingProbabilityTemperatureRange"].as<glm::vec4>();

		if (param["m_apicalControl"]) treeGrowthParameters.m_apicalControl = param["m_apicalControl"].as<float>();
		if (param["m_apicalControlAgeFactor"]) treeGrowthParameters.m_apicalControlAgeFactor = param["m_apicalControlAgeFactor"].as<float>();
		if (param["m_apicalDominance"]) treeGrowthParameters.m_apicalDominance = param["m_apicalDominance"].as<float>();
		if (param["m_apicalDominanceAgeFactor"]) treeGrowthParameters.m_apicalDominanceAgeFactor = param["m_apicalDominanceAgeFactor"].as<float>();
		if (param["m_apicalDominanceDistanceFactor"]) treeGrowthParameters.m_apicalDominanceDistanceFactor = param["m_apicalDominanceDistanceFactor"].as<float>();

		if (param["m_leafVigorRequirement"]) treeGrowthParameters.m_leafVigorRequirement = param["m_leafVigorRequirement"].as<float>();
		if (param["m_fruitVigorRequirement"]) treeGrowthParameters.m_fruitVigorRequirement = param["m_fruitVigorRequirement"].as<float>();
		if (param["m_internodeVigorRequirement"]) treeGrowthParameters.m_internodeVigorRequirement = param["m_internodeVigorRequirement"].as<float>();
		if (param["m_vigorRequirementAggregateLoss"]) treeGrowthParameters.m_vigorRequirementAggregateLoss = param["m_vigorRequirementAggregateLoss"].as<float>();

		//Foliage
		if (param["m_maxLeafSize"]) treeGrowthParameters.m_maxLeafSize = param["m_maxLeafSize"].as<glm::vec3>();
		if (param["m_leafPositionVariance"]) treeGrowthParameters.m_leafPositionVariance = param["m_leafPositionVariance"].as<float>();
		if (param["m_leafRotationVariance"]) treeGrowthParameters.m_leafRotationVariance = param["m_leafRotationVariance"].as<float>();
		if (param["m_leafChlorophyllLoss"]) treeGrowthParameters.m_leafChlorophyllLoss = param["m_leafChlorophyllLoss"].as<float>();
		if (param["m_leafChlorophyllSynthesisFactorTemperature"]) treeGrowthParameters.m_leafChlorophyllSynthesisFactorTemperature = param["m_leafChlorophyllSynthesisFactorTemperature"].as<float>();
		if (param["m_leafFallProbability"]) treeGrowthParameters.m_leafFallProbability = param["m_leafFallProbability"].as<float>();
		if (param["m_leafDistanceToBranchEndLimit"]) treeGrowthParameters.m_leafDistanceToBranchEndLimit = param["m_leafDistanceToBranchEndLimit"].as<float>();

		//Fruit
		if (param["m_maxFruitSize"]) treeGrowthParameters.m_maxFruitSize = param["m_maxFruitSize"].as<glm::vec3>();
		if (param["m_fruitPositionVariance"]) treeGrowthParameters.m_fruitPositionVariance = param["m_fruitPositionVariance"].as<float>();
		if (param["m_fruitRotationVariance"]) treeGrowthParameters.m_fruitRotationVariance = param["m_fruitRotationVariance"].as<float>();
		if (param["m_fruitFallProbability"]) treeGrowthParameters.m_fruitFallProbability = param["m_fruitFallProbability"].as<float>();
	}
}
void TreeDescriptor::DeserializeRootGrowthParameters(const std::string& name, RootGrowthParameters& rootGrowthParameters, const YAML::Node& in) {
	if (in[name]) {
		auto& param = in[name];
		if (param["m_rootNodeLength"]) rootGrowthParameters.m_rootNodeLength = param["m_rootNodeLength"].as<float>();
		if (param["m_rootNodeGrowthRate"]) rootGrowthParameters.m_rootNodeGrowthRate = param["m_rootNodeGrowthRate"].as<float>();
		if (param["m_endNodeThickness"]) rootGrowthParameters.m_endNodeThickness = param["m_endNodeThickness"].as<float>();
		if (param["m_thicknessAccumulationFactor"]) rootGrowthParameters.m_thicknessAccumulationFactor = param["m_thicknessAccumulationFactor"].as<float>();
		if (param["m_thicknessAccumulateAgeFactor"]) rootGrowthParameters.m_thicknessAccumulateAgeFactor = param["m_thicknessAccumulateAgeFactor"].as<float>();

		if (param["m_branchingAngleMeanVariance"]) rootGrowthParameters.m_branchingAngleMeanVariance = param["m_branchingAngleMeanVariance"].as<glm::vec2>();
		if (param["m_rollAngleMeanVariance"]) rootGrowthParameters.m_rollAngleMeanVariance = param["m_rollAngleMeanVariance"].as<glm::vec2>();
		if (param["m_apicalAngleMeanVariance"]) rootGrowthParameters.m_apicalAngleMeanVariance = param["m_apicalAngleMeanVariance"].as<glm::vec2>();

		if (param["m_rootNodeVigorRequirement"]) rootGrowthParameters.m_rootNodeVigorRequirement = param["m_rootNodeVigorRequirement"].as<float>();
		if (param["m_vigorRequirementAggregateLoss"]) rootGrowthParameters.m_vigorRequirementAggregateLoss = param["m_vigorRequirementAggregateLoss"].as<float>();

		if (param["m_environmentalFriction"]) rootGrowthParameters.m_environmentalFriction = param["m_environmentalFriction"].as<float>();
		if (param["m_environmentalFrictionFactor"]) rootGrowthParameters.m_environmentalFrictionFactor = param["m_environmentalFrictionFactor"].as<float>();

		if (param["m_apicalControl"]) rootGrowthParameters.m_apicalControl = param["m_apicalControl"].as<float>();
		if (param["m_apicalControlAgeFactor"]) rootGrowthParameters.m_apicalControlAgeFactor = param["m_apicalControlAgeFactor"].as<float>();
		if (param["m_apicalDominance"]) rootGrowthParameters.m_apicalDominance = param["m_apicalDominance"].as<float>();
		if (param["m_apicalDominanceAgeFactor"]) rootGrowthParameters.m_apicalDominanceAgeFactor = param["m_apicalDominanceAgeFactor"].as<float>();
		if (param["m_apicalDominanceDistanceFactor"]) rootGrowthParameters.m_apicalDominanceDistanceFactor = param["m_apicalDominanceDistanceFactor"].as<float>();

		if (param["m_tropismSwitchingProbability"]) rootGrowthParameters.m_tropismSwitchingProbability = param["m_tropismSwitchingProbability"].as<float>();
		if (param["m_tropismSwitchingProbabilityDistanceFactor"]) rootGrowthParameters.m_tropismSwitchingProbabilityDistanceFactor = param["m_tropismSwitchingProbabilityDistanceFactor"].as<float>();
		if (param["m_tropismIntensity"]) rootGrowthParameters.m_tropismIntensity = param["m_tropismIntensity"].as<float>();

		if (param["m_branchingProbability"]) rootGrowthParameters.m_branchingProbability = param["m_branchingProbability"].as<float>();
	}
}
void TreeDescriptor::Deserialize(const YAML::Node& in) {
	DeserializeShootGrowthParameters("m_shootGrowthParameters", m_shootGrowthParameters, in);
	DeserializeRootGrowthParameters("m_rootGrowthParameters", m_rootGrowthParameters, in);

	DeserializeFoliageParameters("m_foliageParameters", m_foliageParameters, in);
}

void Tree::PrepareControllers(const std::shared_ptr<TreeDescriptor>& treeDescriptor)
{
	const auto soil = m_soil.Get<Soil>();
	const auto climate = m_climate.Get<Climate>();
	{
		m_shootGrowthController.m_internodeGrowthRate = treeDescriptor->m_shootGrowthParameters.m_internodeGrowthRate;
		m_shootGrowthController.m_leafGrowthRate = treeDescriptor->m_shootGrowthParameters.m_leafGrowthRate;
		m_shootGrowthController.m_fruitGrowthRate = treeDescriptor->m_shootGrowthParameters.m_fruitGrowthRate;
		m_shootGrowthController.m_lateralBudCount = treeDescriptor->m_shootGrowthParameters.m_lateralBudCount;
		m_shootGrowthController.m_fruitBudCount = treeDescriptor->m_shootGrowthParameters.m_fruitBudCount;
		m_shootGrowthController.m_leafBudCount = treeDescriptor->m_shootGrowthParameters.m_leafBudCount;
		m_shootGrowthController.m_branchingAngle = [=](const Node<InternodeGrowthData>& internode)
			{
				return glm::gaussRand(treeDescriptor->m_shootGrowthParameters.m_branchingAngleMeanVariance.x, treeDescriptor->m_shootGrowthParameters.m_branchingAngleMeanVariance.y);
			};
		m_shootGrowthController.m_rollAngle = [=](const Node<InternodeGrowthData>& internode)
			{
				return glm::gaussRand(treeDescriptor->m_shootGrowthParameters.m_rollAngleMeanVariance.x, treeDescriptor->m_shootGrowthParameters.m_rollAngleMeanVariance.y);
			};
		m_shootGrowthController.m_apicalAngle = [=](const Node<InternodeGrowthData>& internode)
			{
				return glm::gaussRand(treeDescriptor->m_shootGrowthParameters.m_apicalAngleMeanVariance.x, treeDescriptor->m_shootGrowthParameters.m_apicalAngleMeanVariance.y);
			};
		m_shootGrowthController.m_gravitropism = [=](const Node<InternodeGrowthData>& internode)
			{
				return treeDescriptor->m_shootGrowthParameters.m_gravitropism;
			};
		m_shootGrowthController.m_phototropism = [=](const Node<InternodeGrowthData>& internode)
			{
				return treeDescriptor->m_shootGrowthParameters.m_phototropism;
			};
		m_shootGrowthController.m_maxSpaceOccupancy = treeDescriptor->m_shootGrowthParameters.m_maxSpaceOccupancy;
		m_shootGrowthController.m_sagging = [=](const Node<InternodeGrowthData>& internode)
			{
				const auto& shootGrowthParameters = treeDescriptor->m_shootGrowthParameters;
				const auto newSagging = glm::min(
					shootGrowthParameters.m_saggingFactorThicknessReductionMax.z,
					shootGrowthParameters.m_saggingFactorThicknessReductionMax.x *
					(internode.m_data.m_descendentTotalBiomass + internode.m_data.m_extraMass) /
					glm::pow(
						internode.m_info.m_thickness /
						shootGrowthParameters.m_endNodeThickness,
						shootGrowthParameters.m_saggingFactorThicknessReductionMax.y));
				return glm::max(internode.m_data.m_sagging, newSagging);
			};
		m_shootGrowthController.m_apicalInternodeKillProbability = [=](const Node<InternodeGrowthData>& internode)
			{
				const auto& shootGrowthParameters = treeDescriptor->m_shootGrowthParameters;
				float killProbability = shootGrowthParameters.m_apicalInternodeKillProbability;
				return killProbability;
			};

		m_shootGrowthController.m_lateralBudFlushProbability = [=](const Node<InternodeGrowthData>& internode)
			{
				const auto& shootGrowthParameters = treeDescriptor->m_shootGrowthParameters;
				const auto& internodeData = internode.m_data;
				float flushProbability = shootGrowthParameters.m_lateralBudFlushProbability * internodeData.m_growthPotential;
				if (internodeData.m_inhibitor > 0.0f) flushProbability *= glm::exp(-internodeData.m_inhibitor);
				return flushProbability;
			};
		m_shootGrowthController.m_leafBudFlushingProbability = [=](const Node<InternodeGrowthData>& internode)
			{
				const auto& shootGrowthParameters = treeDescriptor->m_shootGrowthParameters;
				const auto& internodeData = internode.m_data;
				const auto& probabilityRange = shootGrowthParameters.m_leafBudFlushingProbabilityTemperatureRange;
				float flushProbability = glm::mix(probabilityRange.x, probabilityRange.y,
					glm::clamp((internodeData.m_temperature - probabilityRange.z) / (probabilityRange.w - probabilityRange.z), 0.0f, 1.0f));
				flushProbability *= internodeData.m_growthPotential;
				return flushProbability;
			};
		m_shootGrowthController.m_fruitBudFlushingProbability = [=](const Node<InternodeGrowthData>& internode)
			{
				const auto& shootGrowthParameters = treeDescriptor->m_shootGrowthParameters;
				const auto& internodeData = internode.m_data;
				const auto& probabilityRange = shootGrowthParameters.m_fruitBudFlushingProbabilityTemperatureRange;
				float flushProbability = glm::mix(probabilityRange.x, probabilityRange.y,
					glm::clamp((internodeData.m_temperature - probabilityRange.z) / (probabilityRange.w - probabilityRange.z), 0.0f, 1.0f));
				flushProbability *= internodeData.m_growthPotential;
				return flushProbability;
			};
		m_shootGrowthController.m_apicalControl =
			1.0f + treeDescriptor->m_shootGrowthParameters.m_apicalControl * glm::exp(-treeDescriptor->m_shootGrowthParameters.m_apicalControlAgeFactor * m_treeModel.m_age);
		m_shootGrowthController.m_apicalDominance = [=](const Node<InternodeGrowthData>& internode)
			{
				return treeDescriptor->m_shootGrowthParameters.m_apicalDominance * glm::exp(-treeDescriptor->m_shootGrowthParameters.m_apicalDominanceAgeFactor * m_treeModel.m_age);
			};
		m_shootGrowthController.m_apicalDominanceDistanceFactor = treeDescriptor->m_shootGrowthParameters.m_apicalDominanceDistanceFactor;
		m_shootGrowthController.m_internodeVigorRequirement = treeDescriptor->m_shootGrowthParameters.m_internodeVigorRequirement;
		m_shootGrowthController.m_leafVigorRequirement = treeDescriptor->m_shootGrowthParameters.m_leafVigorRequirement;
		m_shootGrowthController.m_fruitVigorRequirement = treeDescriptor->m_shootGrowthParameters.m_fruitVigorRequirement;
		m_shootGrowthController.m_vigorRequirementAggregateLoss = treeDescriptor->m_shootGrowthParameters.m_vigorRequirementAggregateLoss;
		m_shootGrowthController.m_internodeLength = treeDescriptor->m_shootGrowthParameters.m_internodeLength;
		m_shootGrowthController.m_internodeLengthThicknessFactor = treeDescriptor->m_shootGrowthParameters.m_internodeLengthThicknessFactor;
		m_shootGrowthController.m_endNodeThickness = treeDescriptor->m_shootGrowthParameters.m_endNodeThickness;
		m_shootGrowthController.m_lowBranchPruningThicknessFactor = treeDescriptor->m_shootGrowthParameters.m_lowBranchPruningThicknessFactor;
		m_shootGrowthController.m_thicknessAccumulationFactor = treeDescriptor->m_shootGrowthParameters.m_thicknessAccumulationFactor;
		m_shootGrowthController.m_thicknessAccumulateAgeFactor = treeDescriptor->m_shootGrowthParameters.m_thicknessAccumulateAgeFactor;
		m_shootGrowthController.m_lowBranchPruning = treeDescriptor->m_shootGrowthParameters.m_lowBranchPruning;
		m_shootGrowthController.m_pruningFactor = [=](const float deltaTime, const Node<InternodeGrowthData>& internode)
			{
				float pruningProbability = 0.0f;
				if (internode.IsEndNode() && internode.m_data.m_lightIntensity == 0.0f)
				{
					pruningProbability = treeDescriptor->m_shootGrowthParameters.m_lightPruningFactor;
				}
				return pruningProbability;
			};
		m_shootGrowthController.m_maxLeafSize = treeDescriptor->m_shootGrowthParameters.m_maxLeafSize;
		m_shootGrowthController.m_leafPositionVariance = treeDescriptor->m_shootGrowthParameters.m_leafPositionVariance;
		m_shootGrowthController.m_leafRotationVariance = treeDescriptor->m_shootGrowthParameters.m_leafRotationVariance;
		m_shootGrowthController.m_leafDamage = [=](const Node<InternodeGrowthData>& internode)
			{
				const auto& shootGrowthParameters = treeDescriptor->m_shootGrowthParameters;
				const auto& internodeData = internode.m_data;
				float leafDamage = 0.0f;
				if (climate->m_climateModel.m_time - glm::floor(climate->m_climateModel.m_time) > 0.5f && internodeData.m_temperature < shootGrowthParameters.m_leafChlorophyllSynthesisFactorTemperature)
				{
					leafDamage += shootGrowthParameters.m_leafChlorophyllLoss;
				}
				return leafDamage;
			};
		m_shootGrowthController.m_leafFallProbability = [=](const Node<InternodeGrowthData>& internode)
			{
				return treeDescriptor->m_shootGrowthParameters.m_leafFallProbability;
			};
		m_shootGrowthController.m_maxFruitSize = treeDescriptor->m_shootGrowthParameters.m_maxFruitSize;
		m_shootGrowthController.m_fruitPositionVariance = treeDescriptor->m_shootGrowthParameters.m_fruitPositionVariance;
		m_shootGrowthController.m_fruitRotationVariance = treeDescriptor->m_shootGrowthParameters.m_fruitRotationVariance;
		m_shootGrowthController.m_fruitDamage = [=](const Node<InternodeGrowthData>& internode)
			{
				const auto& shootGrowthParameters = treeDescriptor->m_shootGrowthParameters;
				const auto& internodeData = internode.m_data;
				float fruitDamage = 0.0f;
				return fruitDamage;
			};
		m_shootGrowthController.m_fruitFallProbability = [=](const Node<InternodeGrowthData>& internode)
			{
				return treeDescriptor->m_shootGrowthParameters.m_fruitFallProbability;
			};

	}
	{
		m_rootGrowthController.m_rootNodeGrowthRate = treeDescriptor->m_rootGrowthParameters.m_rootNodeGrowthRate;
		m_rootGrowthController.m_rootNodeLength = treeDescriptor->m_rootGrowthParameters.m_rootNodeLength;
		m_rootGrowthController.m_endNodeThickness = treeDescriptor->m_rootGrowthParameters.m_endNodeThickness;
		m_rootGrowthController.m_thicknessAccumulationFactor = treeDescriptor->m_rootGrowthParameters.m_thicknessAccumulationFactor;
		m_rootGrowthController.m_thicknessAccumulateAgeFactor = treeDescriptor->m_rootGrowthParameters.m_thicknessAccumulateAgeFactor;
		m_rootGrowthController.m_branchingAngle = [=](const Node<RootNodeGrowthData>& rootNode)
			{
				return glm::gaussRand(treeDescriptor->m_rootGrowthParameters.m_branchingAngleMeanVariance.x, treeDescriptor->m_rootGrowthParameters.m_branchingAngleMeanVariance.y);
			};
		m_rootGrowthController.m_rollAngle = [=](const Node<RootNodeGrowthData>& rootNode)
			{
				return glm::gaussRand(treeDescriptor->m_rootGrowthParameters.m_rollAngleMeanVariance.x, treeDescriptor->m_rootGrowthParameters.m_rollAngleMeanVariance.y);
			};
		m_rootGrowthController.m_apicalAngle = [=](const Node<RootNodeGrowthData>& rootNode)
			{
				return glm::gaussRand(treeDescriptor->m_rootGrowthParameters.m_apicalAngleMeanVariance.x, treeDescriptor->m_rootGrowthParameters.m_apicalAngleMeanVariance.y);
			};
		m_rootGrowthController.m_environmentalFriction = [=](const Node<RootNodeGrowthData>& rootNode)
			{
				const auto& rootNodeData = rootNode.m_data;
				return 1.0f - glm::pow(1.0f / glm::max(rootNodeData.m_soilDensity * treeDescriptor->m_rootGrowthParameters.m_environmentalFriction, 1.0f), treeDescriptor->m_rootGrowthParameters.m_environmentalFrictionFactor);
			};

		m_rootGrowthController.m_apicalControl = 1.0f + treeDescriptor->m_rootGrowthParameters.m_apicalControl * glm::exp(-treeDescriptor->m_rootGrowthParameters.m_apicalControlAgeFactor * m_treeModel.m_age);
		m_rootGrowthController.m_apicalDominance = [=](const Node<RootNodeGrowthData>& rootNode)
			{
				return treeDescriptor->m_rootGrowthParameters.m_apicalDominance * glm::exp(-treeDescriptor->m_rootGrowthParameters.m_apicalDominanceAgeFactor * m_treeModel.m_age);
			};
		m_rootGrowthController.m_apicalDominanceDistanceFactor = treeDescriptor->m_rootGrowthParameters.m_apicalDominanceDistanceFactor;
		m_rootGrowthController.m_tropismSwitchingProbability = treeDescriptor->m_rootGrowthParameters.m_tropismSwitchingProbability;
		m_rootGrowthController.m_tropismSwitchingProbabilityDistanceFactor = treeDescriptor->m_rootGrowthParameters.m_tropismSwitchingProbabilityDistanceFactor;
		m_rootGrowthController.m_tropismIntensity = treeDescriptor->m_rootGrowthParameters.m_tropismIntensity;
		m_rootGrowthController.m_rootNodeVigorRequirement = treeDescriptor->m_rootGrowthParameters.m_rootNodeVigorRequirement;
		m_rootGrowthController.m_vigorRequirementAggregateLoss = treeDescriptor->m_rootGrowthParameters.m_vigorRequirementAggregateLoss;

		m_rootGrowthController.m_branchingProbability = [=](const Node<RootNodeGrowthData>& rootNode)
			{
				return treeDescriptor->m_rootGrowthParameters.m_branchingProbability;
			};
	}

}
