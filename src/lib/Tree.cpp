//
// Created by lllll on 10/24/2022.
//

#include "Tree.hpp"

#include <Material.hpp>
#include <Mesh.hpp>
#include "Strands.hpp"
#include "EditorLayer.hpp"
#include "Application.hpp"
#include "BranchShape.hpp"
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
	out << YAML::Key << "m_nodeDevelopmentalVigorFillingRate" << YAML::Value << treeGrowthSettings.m_nodeDevelopmentalVigorFillingRate;

	out << YAML::Key << "m_useSpaceColonization" << YAML::Value << treeGrowthSettings.m_useSpaceColonization;
	out << YAML::Key << "m_spaceColonizationAutoResize" << YAML::Value << treeGrowthSettings.m_spaceColonizationAutoResize;
	out << YAML::Key << "m_spaceColonizationRemovalDistanceFactor" << YAML::Value << treeGrowthSettings.m_spaceColonizationRemovalDistanceFactor;
	out << YAML::Key << "m_spaceColonizationDetectionDistanceFactor" << YAML::Value << treeGrowthSettings.m_spaceColonizationDetectionDistanceFactor;
	out << YAML::Key << "m_spaceColonizationTheta" << YAML::Value << treeGrowthSettings.m_spaceColonizationTheta;
}
void Tree::DeserializeTreeGrowthSettings(TreeGrowthSettings& treeGrowthSettings, const YAML::Node& param) {
	if (param["m_nodeDevelopmentalVigorFillingRate"]) treeGrowthSettings.m_nodeDevelopmentalVigorFillingRate = param["m_nodeDevelopmentalVigorFillingRate"].as<float>();
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
		ImGui::DragInt("Seed", &m_treeModel.m_seed, 1, 0);
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
				InitializeMeshRenderer(m_meshGeneratorSettings, iterations);
			}
			if (ImGui::Button("Clear Mesh"))
			{
				ClearMeshRenderer();
			}
			ImGui::TreePop();
		}

		/*
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
		}*/
		if (ImGui::Button("Add Checkpoint"))
		{
			m_treeModel.Step();
			m_treeVisualizer.m_checkpointIteration = m_treeModel.CurrentIteration();
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
	/*
	if (m_enableVisualization)
	{
		if (showSpaceColonizationGrid)
		{
			GizmoSettings gizmoSettings{};
			gizmoSettings.m_drawSettings.m_blending = true;
			editorLayer->DrawGizmoMeshInstancedColored(
				Resources::GetResource<Mesh>("PRIMITIVE_CUBE"), spaceColonizationGridParticleInfoList,
				glm::mat4(1.0f), 1.0f, gizmoSettings);
		}
	}
	*/


	auto& pipeModelParameters = m_treeModel.RefShootSkeleton().m_data.m_pipeModelParameters;
	auto& physicsSettings = m_treeModel.RefShootSkeleton().m_data.m_particlePhysicsSettings;
	ImGui::DragFloat("Default profile cell radius", &pipeModelParameters.m_profileDefaultCellRadius, 0.001f, 0.001f, 1.0f);
	ImGui::DragFloat("Physics damping", &physicsSettings.m_damping, 0.01f, 0.0f, 1.0f);
	ImGui::DragFloat("Physics max speed", &physicsSettings.m_maxSpeed, 0.01f, 0.0f, 100.0f);
	ImGui::DragFloat("Physics particle softness", &physicsSettings.m_particleSoftness, 0.01f, 0.0f, 1.0f);
	ImGui::DragFloat("Boundary strength", &pipeModelParameters.m_boundaryStrength, 1000.f, 0.0f, 100000.0f);
	ImGui::DragFloat("Center attraction strength", &pipeModelParameters.m_centerAttractionStrength, 100.f, 0.0f, 10000.0f);
	ImGui::DragInt("Max iteration cell factor", &pipeModelParameters.m_maxSimulationIterationCellFactor, 1, 0, 500);
	ImGui::DragInt("Timeout", &pipeModelParameters.m_timeout, 100, 200, 10000);
	ImGui::DragInt("Timeout with boundaries)", &pipeModelParameters.m_timeoutWithBoundaries, 100, 200, 10000);
	ImGui::DragFloat("Split limit", &pipeModelParameters.m_splitRatioLimit, 0.01f, 0.0f, 1.0f);
	ImGui::DragInt("End node strand count", &pipeModelParameters.m_endNodeStrands, 1, 1, 50);
	ImGui::Checkbox("Pre-merge", &pipeModelParameters.m_preMerge);

	ImGui::Text(("Last calculation time: " + std::to_string(m_treeModel.PeekShootSkeleton().m_data.m_profileCalculationTime)).c_str());
	ImGui::Text(("Strand count: " + std::to_string(m_treeModel.PeekShootSkeleton().m_data.m_pipeGroup.PeekPipes().size())).c_str());
	ImGui::Text(("Total particle count: " + std::to_string(m_treeModel.PeekShootSkeleton().m_data.m_numOfParticles)).c_str());

	ImGui::DragFloat("Front Control Point Ratio", &pipeModelParameters.m_frontControlPointRatio, 0.01f, 0.01f, 0.5f);
	ImGui::DragFloat("Back Control Point Ratio", &pipeModelParameters.m_backControlPointRatio, 0.01f, 0.01f, 0.5f);

	if (ImGui::TreeNodeEx("Graph Adjustment settings"))
	{
		auto& adjustmentSettings = m_treeModel.RefShootSkeleton().m_data.m_graphAdjustmentSettings;
		ImGui::DragFloat("Shift push ratio", &adjustmentSettings.m_shiftPushRatio, 0.01f, 0.0f, 2.0f);
		ImGui::DragFloat("Side push ratio", &adjustmentSettings.m_sidePushRatio, 0.01f, 0.0f, 2.0f);
		ImGui::DragFloat("Front push ratio", &adjustmentSettings.m_frontPushRatio, 0.01f, 0.0f, 2.0f);
		ImGui::DragFloat("Rotation push ratio", &adjustmentSettings.m_rotationPushRatio, 0.01f, 0.0f, 2.0f);
		ImGui::TreePop();
	}
	ImGui::Checkbox("Triple points", &pipeModelParameters.m_triplePoints);

	if (ImGui::Button("Build Strands"))
	{
		m_treeModel.InitializeProfiles();
		m_treeModel.CalculateProfiles();
		InitializeStrandRenderer();
	}

	if (ImGui::Button("Clear Strands"))
	{
		ClearStrandRenderer();
	}

	if (ImGui::TreeNodeEx("Tree Pipe Mesh Generator Settings", ImGuiTreeNodeFlags_DefaultOpen)) {
		m_treePipeMeshGeneratorSettings.OnInspect(editorLayer);
		ImGui::TreePop();
	}
	if (ImGui::Button("Build Pipe Mesh"))
	{
		GenerateStrands();
		InitializeMeshRendererPipe(m_treePipeMeshGeneratorSettings);
	}

	if (ImGui::Button("Clear Pipe Mesh"))
	{

		ClearMeshRendererPipe();
	}
}
void Tree::Update()
{
	if (m_temporalProgression) {
		if (m_temporalProgressionIteration <= m_treeModel.CurrentIteration()) {
			InitializeMeshRenderer(m_meshGeneratorSettings, m_temporalProgressionIteration);
			m_temporalProgressionIteration++;
		}
		else
		{
			m_temporalProgressionIteration = 0;
			m_temporalProgression = false;
		}
	}
	const auto editorLayer = Application::GetLayer<EditorLayer>();
	const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();

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

	m_treeVisualizer.Clear();

	m_leftSideBiomass = m_rightSideBiomass = 0.0f;
	m_rootBiomassHistory.clear();
	m_shootBiomassHistory.clear();
}

bool Tree::OnInspectTreeGrowthSettings(TreeGrowthSettings& treeGrowthSettings)
{
	bool changed = false;
	if (ImGui::Checkbox("Enable space colonization", &treeGrowthSettings.m_useSpaceColonization))changed = true;
	if (treeGrowthSettings.m_useSpaceColonization)
	{
		if (ImGui::Checkbox("Space colonization auto resize", &treeGrowthSettings.m_spaceColonizationAutoResize))changed = true;
	}


	return changed;
}

std::shared_ptr<Strands> Tree::GenerateStrands()
{
	m_treeModel.InitializeProfiles();
	m_treeModel.CalculateProfiles();
	const auto strandsAsset = ProjectManager::CreateTemporaryAsset<Strands>();

	m_treeModel.CalculatePipeProfileAdjustedTransforms();
	m_treeModel.ApplyProfiles();
	const auto& parameters = m_treeModel.RefShootSkeleton().m_data.m_pipeModelParameters;
	std::vector<glm::uint> strandsList;
	std::vector<StrandPoint> points;
	m_treeModel.RefShootSkeleton().m_data.m_pipeGroup.BuildStrands(parameters.m_frontControlPointRatio, parameters.m_backControlPointRatio, strandsList, points, parameters.m_triplePoints, parameters.m_nodeMaxCount);
	if (!points.empty()) strandsList.emplace_back(points.size());
	StrandPointAttributes strandPointAttributes{};
	strandPointAttributes.m_color = true;
	strandsAsset->SetStrands(strandPointAttributes, strandsList, points);
	return strandsAsset;
}

std::shared_ptr<Mesh> Tree::GenerateBranchMesh(const TreeMeshGeneratorSettings& meshGeneratorSettings)
{
	std::vector<Vertex> vertices;
	std::vector<unsigned int> indices;
	if (meshGeneratorSettings.m_branchMeshType == 0) {
		const CylindricalMeshGenerator<ShootGrowthData, ShootStemGrowthData, InternodeGrowthData> meshGenerator{};

		const auto treeDescriptor = m_treeDescriptor.Get<TreeDescriptor>();
		std::shared_ptr<BranchShape> branchShape{};
		if (treeDescriptor)
		{
			branchShape = treeDescriptor->m_shootBranchShape.Get<BranchShape>();
		}
		meshGenerator.Generate(m_treeModel.PeekShootSkeleton(), vertices, indices, meshGeneratorSettings, [&](float xFactor, float distanceToRoot)
			{
				if (branchShape)
				{
					return branchShape->GetValue(xFactor, distanceToRoot);
				}
				return 1.0f;
			});
	}
	else
	{
		const VoxelMeshGenerator<ShootGrowthData, ShootStemGrowthData, InternodeGrowthData> meshGenerator{};

		const auto treeDescriptor = m_treeDescriptor.Get<TreeDescriptor>();
		std::shared_ptr<BranchShape> branchShape{};
		if (treeDescriptor)
		{
			branchShape = treeDescriptor->m_shootBranchShape.Get<BranchShape>();
		}
		meshGenerator.Generate(m_treeModel.PeekShootSkeleton(), vertices, indices, meshGeneratorSettings);
	}
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
	const auto& foliageParameters = (!meshGeneratorSettings.m_foliageOverride && treeDescriptor) ? treeDescriptor->m_foliageParameters : meshGeneratorSettings.m_foliageOverrideSettings;
	const auto& nodeList = m_treeModel.PeekShootSkeleton().RefSortedNodeList();
	for (const auto& internodeHandle : nodeList) {
		const auto& internode = m_treeModel.PeekShootSkeleton().PeekNode(internodeHandle);
		const auto& internodeInfo = internode.m_info;
		if (internodeInfo.m_thickness < foliageParameters.m_maxNodeThickness
			&& internodeInfo.m_rootDistance > foliageParameters.m_minRootDistance
			&& internodeInfo.m_endDistance < foliageParameters.m_maxEndDistance) {
			for (int i = 0; i < foliageParameters.m_leafCountPerInternode; i++)
			{
				auto leafSize = foliageParameters.m_leafSize * internode.m_data.m_lightIntensity;
				glm::quat rotation = internodeInfo.m_globalRotation * glm::quat(glm::radians(glm::vec3(glm::gaussRand(0.0f, foliageParameters.m_rotationVariance), foliageParameters.m_branchingAngle, glm::linearRand(0.0f, 360.0f))));
				auto front = rotation * glm::vec3(0, 0, -1);
				auto foliagePosition = internodeInfo.m_globalPosition + front * (leafSize.y + glm::gaussRand(0.0f, foliageParameters.m_positionVariance));
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
					indices.push_back(triangle.z);
					indices.push_back(triangle.y);
					indices.push_back(triangle.x);
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

std::shared_ptr<Mesh> Tree::GeneratePipeMesh(const TreePipeMeshGeneratorSettings& treePipeMeshGeneratorSettings)
{
	std::vector<Vertex> vertices;
	std::vector<unsigned int> indices;
	const TreePipeMeshGenerator meshGenerator{};

	meshGenerator.Generate(m_treeModel.m_shootSkeleton.m_data.m_pipeGroup, vertices, indices, treePipeMeshGeneratorSettings);

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

bool Tree::TryGrow(const float deltaTime, const NodeHandle baseInternodeHandle, const bool pruning, const float overrideGrowthRate) {
	const auto scene = GetScene();
	const auto treeDescriptor = m_treeDescriptor.Get<TreeDescriptor>();
	const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();

	const auto climateCandidate = EcoSysLabLayer::FindClimate();
	if (!climateCandidate.expired()) m_climate = climateCandidate.lock();
	const auto soilCandidate = EcoSysLabLayer::FindSoil();
	if (!soilCandidate.expired()) m_soil = soilCandidate.lock();

	const auto soil = m_soil.Get<Soil>();
	const auto climate = m_climate.Get<Climate>();

	if (!soil) {
		return false;
	}
	if (!climate) {
		return false;
	}

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

	const auto owner = GetOwner();
	PrepareControllers(treeDescriptor);
	const bool grown = m_treeModel.Grow(deltaTime, baseInternodeHandle, scene->GetDataComponent<GlobalTransform>(owner).m_value, climate->m_climateModel, m_shootGrowthController, pruning, overrideGrowthRate);
	if (grown)
	{
		if (pruning) m_treeVisualizer.ClearSelections();
		m_treeVisualizer.m_needUpdate = true;
	}
	if (m_enableHistory && m_treeModel.m_iteration % m_historyIteration == 0) m_treeModel.Step();
	if (m_recordBiomassHistory)
	{
		const auto& baseShootNode = m_treeModel.RefShootSkeleton().RefNode(0);
		m_shootBiomassHistory.emplace_back(baseShootNode.m_data.m_biomass + baseShootNode.m_data.m_descendentTotalBiomass);
	}
	return grown;
}


void Tree::Serialize(YAML::Emitter& out)
{
	m_treeDescriptor.Save("m_treeDescriptor", out);
}



void Tree::Deserialize(const YAML::Node& in)
{
	m_treeDescriptor.Load("m_treeDescriptor", in);
}

void Tree::ClearMeshRenderer() const
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

void Tree::ClearTwigsStrandRenderer() const
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

void Tree::ClearStrandRenderer() const
{
	const auto scene = GetScene();
	const auto self = GetOwner();
	const auto children = scene->GetChildren(self);
	for (const auto& child : children) {
		auto name = scene->GetEntityName(child);
		if (name == "Branch Strands") {
			scene->DeleteEntity(child);
		}
	}
}

void Tree::InitializeMeshRenderer(const TreeMeshGeneratorSettings& meshGeneratorSettings, int iteration) {
	const auto scene = GetScene();
	const auto self = GetOwner();
	const auto children = scene->GetChildren(self);
	auto treeDescriptor = m_treeDescriptor.Get<TreeDescriptor>();
	ClearMeshRenderer();
	ClearTwigsStrandRenderer();
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

		if (treeDescriptor) {
			auto texRef = treeDescriptor->m_branchAlbedoTexture;
			if (texRef.Get<Texture2D>())
			{
				material->SetAlbedoTexture(texRef.Get<Texture2D>());

			}
			texRef = treeDescriptor->m_branchNormalTexture;
			if (texRef.Get<Texture2D>())
			{
				material->SetNormalTexture(texRef.Get<Texture2D>());

			}
			texRef = treeDescriptor->m_branchRoughnessTexture;
			if (texRef.Get<Texture2D>())
			{
				material->SetRoughnessTexture(texRef.Get<Texture2D>());

			}
			texRef = treeDescriptor->m_branchMetallicTexture;
			if (texRef.Get<Texture2D>())
			{
				material->SetMetallicTexture(texRef.Get<Texture2D>());
			}
		}
		if (meshGeneratorSettings.m_presentationOverride)
		{
			material->m_materialProperties.m_albedoColor = meshGeneratorSettings.m_presentationOverrideSettings.m_branchOverrideColor;
			auto texRef = meshGeneratorSettings.m_branchAlbedoTexture;
			if (texRef.Get<Texture2D>())
			{
				material->SetAlbedoTexture(texRef.Get<Texture2D>());

			}
			texRef = meshGeneratorSettings.m_branchNormalTexture;
			if (texRef.Get<Texture2D>())
			{
				material->SetNormalTexture(texRef.Get<Texture2D>());

			}
			texRef = meshGeneratorSettings.m_branchRoughnessTexture;
			if (texRef.Get<Texture2D>())
			{
				material->SetRoughnessTexture(texRef.Get<Texture2D>());
			}
			texRef = meshGeneratorSettings.m_branchMetallicTexture;
			if (texRef.Get<Texture2D>())
			{
				material->SetMetallicTexture(texRef.Get<Texture2D>());
			}
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
					positionStart + internodeInfo.m_length * (meshGeneratorSettings.m_smoothness ? 1.0f - meshGeneratorSettings.m_baseControlPointRatio * 0.5f : 1.0f) * internodeInfo.m_globalDirection;

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
		if (treeDescriptor) {
			auto texRef = treeDescriptor->m_foliageAlbedoTexture;
			if (texRef.Get<Texture2D>())
			{
				material->SetAlbedoTexture(texRef.Get<Texture2D>());

			}
			texRef = treeDescriptor->m_foliageNormalTexture;
			if (texRef.Get<Texture2D>())
			{
				material->SetNormalTexture(texRef.Get<Texture2D>());

			}
			texRef = treeDescriptor->m_foliageRoughnessTexture;
			if (texRef.Get<Texture2D>())
			{
				material->SetRoughnessTexture(texRef.Get<Texture2D>());

			}
			texRef = treeDescriptor->m_foliageMetallicTexture;
			if (texRef.Get<Texture2D>())
			{
				material->SetMetallicTexture(texRef.Get<Texture2D>());
			}
		}

		if (meshGeneratorSettings.m_foliageOverride)
		{
			material->m_materialProperties.m_albedoColor = meshGeneratorSettings.m_foliageOverrideSettings.m_leafColor;
			auto texRef = meshGeneratorSettings.m_foliageAlbedoTexture;
			if (texRef.Get<Texture2D>())
			{
				material->SetAlbedoTexture(texRef.Get<Texture2D>());

			}
			texRef = meshGeneratorSettings.m_foliageNormalTexture;
			if (texRef.Get<Texture2D>())
			{
				material->SetNormalTexture(texRef.Get<Texture2D>());

			}
			texRef = meshGeneratorSettings.m_foliageRoughnessTexture;
			if (texRef.Get<Texture2D>())
			{
				material->SetRoughnessTexture(texRef.Get<Texture2D>());

			}
			texRef = meshGeneratorSettings.m_foliageMetallicTexture;
			if (texRef.Get<Texture2D>())
			{
				material->SetMetallicTexture(texRef.Get<Texture2D>());
			}
		}
		else {
			material->m_materialProperties.m_albedoColor = treeDescriptor->m_foliageParameters.m_leafColor;
		}
		material->m_materialProperties.m_roughness = 1.0f;
		material->m_materialProperties.m_metallic = 0.0f;
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
					if (bud.m_status != BudStatus::Died) continue;
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
			material->m_materialProperties.m_albedoColor = meshGeneratorSettings.m_foliageOverrideSettings.m_leafColor;
		}
		else {
			material->m_materialProperties.m_albedoColor = glm::vec3(152 / 255.0f, 203 / 255.0f, 0 / 255.0f);
		}
		material->m_materialProperties.m_roughness = 0.0f;

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

struct JunctionLine {
	int m_lineIndex = -1;
	glm::vec3 m_startPosition;
	glm::vec3 m_endPosition;
	float m_startRadius;
	float m_endRadius;

	glm::vec3 m_startDirection;
	glm::vec3 m_endDirection;
};

struct TreePart {
	int m_treePartIndex;
	bool m_isJunction = false;
	JunctionLine m_baseLine;
	std::vector<JunctionLine> m_childrenLines;
	std::vector<NodeHandle> m_nodeHandles;
	std::vector<bool> m_isEnd;
	std::vector<int> m_lineIndex;
};

void Tree::ExportJunction(const TreeMeshGeneratorSettings& meshGeneratorSettings, const std::filesystem::path& path)
{
	try {
		auto directory = path;
		directory.remove_filename();
		std::filesystem::create_directories(directory);
		YAML::Emitter out;
		out << YAML::BeginMap;
		const auto& skeleton = m_treeModel.RefShootSkeleton();
		const auto& sortedInternodeList = skeleton.RefSortedNodeList();
		std::vector<TreePart> treeParts{};
		std::unordered_map<NodeHandle, TreePartInfo> treePartInfos{};
		int nextLineIndex = 0;
		for (int internodeHandle : sortedInternodeList)
		{
			const auto& internode = skeleton.PeekNode(internodeHandle);
			const auto& internodeInfo = internode.m_info;
			auto parentInternodeHandle = internode.GetParentHandle();
			const auto flowHandle = internode.GetFlowHandle();
			const auto& flow = skeleton.PeekFlow(flowHandle);
			const auto& chainHandles = flow.RefNodeHandles();
			const bool hasMultipleChildren = flow.RefChildHandles().size() > 1;
			bool onlyChild = true;
			const auto parentFlowHandle = flow.GetParentHandle();
			float distanceToChainStart = 0;
			float distanceToChainEnd = 0;
			const auto chainSize = chainHandles.size();
			for (int i = 0; i < chainSize; i++)
			{
				if (chainHandles[i] == internodeHandle) break;
				distanceToChainStart += skeleton.PeekNode(chainHandles[i]).m_info.m_length;

			}
			distanceToChainEnd = flow.m_info.m_flowLength - distanceToChainStart - internode.m_info.m_length;
			float compareRadius = internode.m_info.m_thickness;
			if (parentFlowHandle != -1)
			{
				const auto& parentFlow = skeleton.PeekFlow(parentFlowHandle);
				onlyChild = parentFlow.RefChildHandles().size() <= 1;
				compareRadius = parentFlow.m_info.m_endThickness;
			}
			int treePartType = 0;
			if (hasMultipleChildren && distanceToChainEnd <= meshGeneratorSettings.m_treePartBaseDistance * compareRadius) {
				treePartType = 1;
			}
			else if (!onlyChild && distanceToChainStart <= meshGeneratorSettings.m_treePartEndDistance * compareRadius)
			{
				treePartType = 2;
			}
			int currentTreePartIndex = -1;
			int currentLineIndex = -1;
			if (treePartType == 0)
			{
				//IShape
				//If root or parent is Y Shape or length exceeds limit, create a new IShape from this node.
				bool restartIShape = parentInternodeHandle == -1 || treePartInfos[parentInternodeHandle].m_treePartType != 0;
				if (!restartIShape)
				{
					const auto& parentJunctionInfo = treePartInfos[parentInternodeHandle];
					if (parentJunctionInfo.m_distanceToStart / internodeInfo.m_thickness > meshGeneratorSettings.m_treePartBreakRatio) restartIShape = true;
				}
				if (restartIShape)
				{
					TreePartInfo treePartInfo;
					treePartInfo.m_treePartType = 0;
					treePartInfo.m_treePartIndex = treeParts.size();
					treePartInfo.m_lineIndex = nextLineIndex;
					treePartInfo.m_distanceToStart = 0.0f;
					treePartInfos[internodeHandle] = treePartInfo;
					treeParts.emplace_back();
					auto& treePart = treeParts.back();
					treePart.m_isJunction = false;
					currentTreePartIndex = treePart.m_treePartIndex = treePartInfo.m_treePartIndex;

					currentLineIndex = nextLineIndex;
					nextLineIndex++;
				}
				else
				{
					auto& currentTreePartInfo = treePartInfos[internodeHandle];
					currentTreePartInfo = treePartInfos[parentInternodeHandle];
					currentTreePartInfo.m_distanceToStart += internodeInfo.m_length;
					currentTreePartInfo.m_treePartType = 0;
					currentTreePartIndex = currentTreePartInfo.m_treePartIndex;

					currentLineIndex = currentTreePartInfo.m_lineIndex;
				}
			}
			else if (treePartType == 1)
			{
				//Base of Y Shape
				if (parentInternodeHandle == -1 || treePartInfos[parentInternodeHandle].m_treePartType != 1
					|| treePartInfos[parentInternodeHandle].m_baseFlowHandle != flowHandle)
				{
					TreePartInfo treePartInfo;
					treePartInfo.m_treePartType = 1;
					treePartInfo.m_treePartIndex = treeParts.size();
					treePartInfo.m_lineIndex = nextLineIndex;
					treePartInfo.m_distanceToStart = 0.0f;
					treePartInfo.m_baseFlowHandle = flowHandle;
					treePartInfos[internodeHandle] = treePartInfo;
					treeParts.emplace_back();
					auto& treePart = treeParts.back();
					treePart.m_isJunction = true;
					currentTreePartIndex = treePart.m_treePartIndex = treePartInfo.m_treePartIndex;

					currentLineIndex = nextLineIndex;
					nextLineIndex++;
				}
				else
				{
					auto& currentTreePartInfo = treePartInfos[internodeHandle];
					currentTreePartInfo = treePartInfos[parentInternodeHandle];
					currentTreePartInfo.m_treePartType = 1;
					currentTreePartIndex = currentTreePartInfo.m_treePartIndex;

					currentLineIndex = currentTreePartInfo.m_lineIndex;
				}
			}
			else if (treePartType == 2)
			{
				//Branch of Y Shape
				if (parentInternodeHandle == -1 || treePartInfos[parentInternodeHandle].m_treePartType == 0
					|| treePartInfos[parentInternodeHandle].m_baseFlowHandle != parentFlowHandle)
				{
					EVOENGINE_ERROR("Error!");
				}
				else
				{
					auto& currentTreePartInfo = treePartInfos[internodeHandle];
					currentTreePartInfo = treePartInfos[parentInternodeHandle];
					if (currentTreePartInfo.m_treePartType != 2)
					{
						currentTreePartInfo.m_lineIndex = nextLineIndex;
						nextLineIndex++;
					}
					currentTreePartInfo.m_treePartType = 2;
					currentTreePartIndex = currentTreePartInfo.m_treePartIndex;

					currentLineIndex = currentTreePartInfo.m_lineIndex;
				}
			}
			auto& treePart = treeParts[currentTreePartIndex];
			treePart.m_nodeHandles.emplace_back(internodeHandle);
			treePart.m_isEnd.emplace_back(true);
			treePart.m_lineIndex.emplace_back(currentLineIndex);
			for (int i = 0; i < treePart.m_nodeHandles.size(); i++)
			{
				if (treePart.m_nodeHandles[i] == parentInternodeHandle)
				{
					treePart.m_isEnd[i] = false;
					break;
				}
			}
		}
		for (auto& treePart : treeParts)
		{
			const auto& startInternode = skeleton.PeekNode(treePart.m_nodeHandles.front());
			if (treePart.m_isJunction)
			{
				const auto& baseNode = skeleton.PeekNode(treePart.m_nodeHandles.front());
				const auto& flow = skeleton.PeekFlow(baseNode.GetFlowHandle());
				const auto& chainHandles = flow.RefNodeHandles();
				const auto centerInternodeHandle = chainHandles.back();
				const auto& centerInternode = skeleton.PeekNode(centerInternodeHandle);
				treePart.m_baseLine.m_startPosition = startInternode.m_info.m_globalPosition;
				treePart.m_baseLine.m_startRadius = startInternode.m_info.m_thickness;
				treePart.m_baseLine.m_endPosition = centerInternode.m_info.GetGlobalEndPosition();
				treePart.m_baseLine.m_endRadius = centerInternode.m_info.m_thickness;

				treePart.m_baseLine.m_startDirection = startInternode.m_info.m_globalDirection;
				treePart.m_baseLine.m_endDirection = centerInternode.m_info.m_globalDirection;

				treePart.m_baseLine.m_lineIndex = treePart.m_lineIndex.front();
				for (int i = 1; i < treePart.m_nodeHandles.size(); i++)
				{
					if (treePart.m_isEnd[i])
					{
						const auto& endInternode = skeleton.PeekNode(treePart.m_nodeHandles[i]);
						treePart.m_childrenLines.emplace_back();
						auto& newLine = treePart.m_childrenLines.back();
						newLine.m_startPosition = centerInternode.m_info.GetGlobalEndPosition();
						newLine.m_startRadius = centerInternode.m_info.m_thickness;
						newLine.m_endPosition = endInternode.m_info.GetGlobalEndPosition();
						newLine.m_endRadius = endInternode.m_info.m_thickness;

						newLine.m_startDirection = centerInternode.m_info.m_globalDirection;
						newLine.m_endDirection = endInternode.m_info.m_globalDirection;

						newLine.m_lineIndex = treePart.m_lineIndex[i];
					}
				}
			}
			else
			{
				const auto& endInternode = skeleton.PeekNode(treePart.m_nodeHandles.back());
				treePart.m_baseLine.m_startPosition = startInternode.m_info.m_globalPosition;
				treePart.m_baseLine.m_startRadius = startInternode.m_info.m_thickness;
				treePart.m_baseLine.m_endPosition = endInternode.m_info.GetGlobalEndPosition();
				treePart.m_baseLine.m_endRadius = endInternode.m_info.m_thickness;

				treePart.m_baseLine.m_startDirection = startInternode.m_info.m_globalDirection;
				treePart.m_baseLine.m_endDirection = endInternode.m_info.m_globalDirection;

				treePart.m_baseLine.m_lineIndex = treePart.m_lineIndex.front();
			}
		}
		std::unordered_set<int> lineIndexCheck{};
		out << YAML::Key << "TreeParts" << YAML::Value << YAML::BeginSeq;
		for (const auto& treePart : treeParts)
		{
			out << YAML::BeginMap;
			out << YAML::Key << "J" << YAML::Value << (treePart.m_isJunction ? 1 : 0);
			out << YAML::Key << "I" << YAML::Value << treePart.m_treePartIndex + 1;
			out << YAML::Key << "LI" << YAML::Value << treePart.m_baseLine.m_lineIndex + 1;
			if (lineIndexCheck.find(treePart.m_baseLine.m_lineIndex) != lineIndexCheck.end())
			{
				EVOENGINE_ERROR("Duplicate!");
			}
			lineIndexCheck.emplace(treePart.m_baseLine.m_lineIndex);
			out << YAML::Key << "BSP" << YAML::Value << treePart.m_baseLine.m_startPosition;
			out << YAML::Key << "BEP" << YAML::Value << treePart.m_baseLine.m_endPosition;
			out << YAML::Key << "BSR" << YAML::Value << treePart.m_baseLine.m_startRadius;
			out << YAML::Key << "BER" << YAML::Value << treePart.m_baseLine.m_endRadius;
			out << YAML::Key << "BSD" << YAML::Value << treePart.m_baseLine.m_startDirection;
			out << YAML::Key << "BED" << YAML::Value << treePart.m_baseLine.m_endDirection;
			out << YAML::Key << "C" << YAML::Value << YAML::BeginSeq;
			if (treePart.m_childrenLines.size() > 3)
			{
				EVOENGINE_ERROR("Too many child!");
			}
			for (const auto& childLine : treePart.m_childrenLines) {
				out << YAML::BeginMap;
				out << YAML::Key << "LI" << YAML::Value << childLine.m_lineIndex + 1;
				if (lineIndexCheck.find(childLine.m_lineIndex) != lineIndexCheck.end())
				{
					EVOENGINE_ERROR("Duplicate!");
				}
				lineIndexCheck.emplace(childLine.m_lineIndex);
				out << YAML::Key << "SP" << YAML::Value << childLine.m_startPosition;
				out << YAML::Key << "EP" << YAML::Value << childLine.m_endPosition;
				out << YAML::Key << "SR" << YAML::Value << childLine.m_startRadius;
				out << YAML::Key << "ER" << YAML::Value << childLine.m_endRadius;
				out << YAML::Key << "SD" << YAML::Value << childLine.m_startDirection;
				out << YAML::Key << "ED" << YAML::Value << childLine.m_endDirection;
				out << YAML::EndMap;
			}
			out << YAML::EndSeq;
			out << YAML::EndMap;
		}
		out << YAML::EndSeq;
		out << YAML::EndMap;
		std::ofstream outputFile(path.string());
		outputFile << out.c_str();
		outputFile.flush();
	}
	catch (const std::exception& e) {
		EVOENGINE_ERROR("Failed to save!");
	}
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



void Tree::PrepareControllers(const std::shared_ptr<TreeDescriptor>& treeDescriptor)
{
	const auto soil = m_soil.Get<Soil>();
	const auto climate = m_climate.Get<Climate>();
	{
		m_shootGrowthController.m_internodeGrowthRate = treeDescriptor->m_shootGrowthParameters.m_growthRate / treeDescriptor->m_shootGrowthParameters.m_internodeLength;

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
				return glm::gaussRand(0.0f, treeDescriptor->m_shootGrowthParameters.m_apicalAngleVariance);
			};
		m_shootGrowthController.m_gravitropism = [=](const Node<InternodeGrowthData>& internode)
			{
				return treeDescriptor->m_shootGrowthParameters.m_gravitropism;
			};
		m_shootGrowthController.m_phototropism = [=](const Node<InternodeGrowthData>& internode)
			{
				return treeDescriptor->m_shootGrowthParameters.m_phototropism;
			};
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
		m_shootGrowthController.m_internodeLength = treeDescriptor->m_shootGrowthParameters.m_internodeLength;
		m_shootGrowthController.m_internodeLengthThicknessFactor = treeDescriptor->m_shootGrowthParameters.m_internodeLengthThicknessFactor;
		m_shootGrowthController.m_endNodeThickness = treeDescriptor->m_shootGrowthParameters.m_endNodeThickness;
		m_shootGrowthController.m_thicknessAccumulationFactor = treeDescriptor->m_shootGrowthParameters.m_thicknessAccumulationFactor;
		m_shootGrowthController.m_thicknessAccumulateAgeFactor = treeDescriptor->m_shootGrowthParameters.m_thicknessAccumulateAgeFactor;
		m_shootGrowthController.m_internodeShadowFactor = treeDescriptor->m_shootGrowthParameters.m_internodeShadowFactor;

		m_shootGrowthController.m_lateralBudCount = treeDescriptor->m_shootGrowthParameters.m_lateralBudCount;
		m_shootGrowthController.m_budExtinctionRate = [=](const Node<InternodeGrowthData>& internode, Bud& bud)
			{
				const auto& shootGrowthParameters = treeDescriptor->m_shootGrowthParameters;
				bud.m_extinctionRate = 0.0f;
				if (bud.m_type == BudType::Apical) {
					bud.m_extinctionRate = shootGrowthParameters.m_apicalBudExtinctionRate;
				}
				else
				{
					bud.m_extinctionRate = 0.0f;
				}
			};
		m_shootGrowthController.m_budFlushingRate = [=](const Node<InternodeGrowthData>& internode, Bud& bud)
			{
				const auto& shootGrowthParameters = treeDescriptor->m_shootGrowthParameters;
				bud.m_flushingRate = 1.0f;
				if (bud.m_type == BudType::Apical) {
					bud.m_flushingRate = 1.0f;
					bud.m_flushingRate *= glm::pow(internode.m_data.m_lightIntensity, shootGrowthParameters.m_apicalBudLightingFactor);

				}
				else
				{
					bud.m_flushingRate = shootGrowthParameters.m_lateralBudFlushingRate;
					bud.m_flushingRate *= glm::pow(internode.m_data.m_lightIntensity, shootGrowthParameters.m_lateralBudLightingFactor);
					if (internode.m_data.m_inhibitorSink > 0.0f) bud.m_flushingRate *= glm::exp(-internode.m_data.m_inhibitorSink);
				}
			};
		m_shootGrowthController.m_pipeResistance = treeDescriptor->m_shootGrowthParameters.m_pipeResistance;
		m_shootGrowthController.m_apicalControl = treeDescriptor->m_shootGrowthParameters.m_apicalControl;
		m_shootGrowthController.m_apicalDominance = [=](const Node<InternodeGrowthData>& internode)
			{
				return treeDescriptor->m_shootGrowthParameters.m_apicalDominance * internode.m_data.m_lightIntensity;
			};
		m_shootGrowthController.m_apicalDominanceLoss = treeDescriptor->m_shootGrowthParameters.m_apicalDominanceLoss;

		m_shootGrowthController.m_lowBranchPruning = treeDescriptor->m_shootGrowthParameters.m_lowBranchPruning;
		m_shootGrowthController.m_lowBranchPruningThicknessFactor = treeDescriptor->m_shootGrowthParameters.m_lowBranchPruningThicknessFactor;
		m_shootGrowthController.m_pruningFactor = [=](const float deltaTime, const Node<InternodeGrowthData>& internode)
			{
				float pruningProbability = 0.0f;
				if (internode.IsEndNode() && internode.m_data.m_lightIntensity == 0.0f)
				{
					pruningProbability = treeDescriptor->m_shootGrowthParameters.m_lightPruningFactor;
				}
				if (!internode.IsApical() && treeDescriptor->m_shootGrowthParameters.m_thicknessPruningFactor != 0.0f
					&& internode.m_info.m_thickness / internode.m_info.m_endDistance < treeDescriptor->m_shootGrowthParameters.m_thicknessPruningFactor)
				{
					pruningProbability += 1.0f;
				}
				return pruningProbability * deltaTime;
			};


		m_shootGrowthController.m_leafGrowthRate = treeDescriptor->m_shootGrowthParameters.m_leafGrowthRate;
		m_shootGrowthController.m_fruitGrowthRate = treeDescriptor->m_shootGrowthParameters.m_fruitGrowthRate;

		m_shootGrowthController.m_fruitBudCount = treeDescriptor->m_shootGrowthParameters.m_fruitBudCount;
		m_shootGrowthController.m_leafBudCount = treeDescriptor->m_shootGrowthParameters.m_leafBudCount;

		m_shootGrowthController.m_leafBudFlushingProbability = [=](const Node<InternodeGrowthData>& internode)
			{
				const auto& shootGrowthParameters = treeDescriptor->m_shootGrowthParameters;
				const auto& internodeData = internode.m_data;
				const auto& probabilityRange = shootGrowthParameters.m_leafBudFlushingProbabilityTemperatureRange;
				float flushProbability = glm::mix(probabilityRange.x, probabilityRange.y,
					glm::clamp((internodeData.m_temperature - probabilityRange.z) / (probabilityRange.w - probabilityRange.z), 0.0f, 1.0f));
				flushProbability *= internodeData.m_lightIntensity;
				return flushProbability;
			};
		m_shootGrowthController.m_fruitBudFlushingProbability = [=](const Node<InternodeGrowthData>& internode)
			{
				const auto& shootGrowthParameters = treeDescriptor->m_shootGrowthParameters;
				const auto& internodeData = internode.m_data;
				const auto& probabilityRange = shootGrowthParameters.m_fruitBudFlushingProbabilityTemperatureRange;
				float flushProbability = glm::mix(probabilityRange.x, probabilityRange.y,
					glm::clamp((internodeData.m_temperature - probabilityRange.z) / (probabilityRange.w - probabilityRange.z), 0.0f, 1.0f));
				flushProbability *= internodeData.m_lightIntensity;
				return flushProbability;
			};

		m_shootGrowthController.m_leafVigorRequirement = treeDescriptor->m_shootGrowthParameters.m_leafVigorRequirement;
		m_shootGrowthController.m_fruitVigorRequirement = treeDescriptor->m_shootGrowthParameters.m_fruitVigorRequirement;



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
}

void Tree::InitializeStrandRenderer()
{
	const auto scene = GetScene();
	const auto owner = GetOwner();

	ClearStrandRenderer();
	const auto strandsEntity = scene->CreateEntity("Branch Strands");
	scene->SetParent(strandsEntity, owner);

	const auto renderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(strandsEntity).lock();

	renderer->m_strands = GenerateStrands();

	const auto material = ProjectManager::CreateTemporaryAsset<Material>();

	renderer->m_material = material;
	material->m_vertexColorOnly = true;
	material->m_materialProperties.m_albedoColor = glm::vec3(0.6f, 0.3f, 0.0f);
}

void Tree::InitializeStrandRenderer(const std::shared_ptr<Strands>& strands) const
{
	const auto scene = GetScene();
	const auto owner = GetOwner();

	ClearStrandRenderer();
	const auto strandsEntity = scene->CreateEntity("Branch Strands");
	scene->SetParent(strandsEntity, owner);

	const auto renderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(strandsEntity).lock();

	renderer->m_strands = strands;

	const auto material = ProjectManager::CreateTemporaryAsset<Material>();

	renderer->m_material = material;
	material->m_vertexColorOnly = true;
	material->m_materialProperties.m_albedoColor = glm::vec3(0.6f, 0.3f, 0.0f);
}

void Tree::InitializeMeshRendererPipe(const TreePipeMeshGeneratorSettings& treePipeMeshGeneratorSettings)
{
	ClearMeshRendererPipe();
	const auto scene = GetScene();
	const auto self = GetOwner();
	Entity branchEntity;
	branchEntity = scene->CreateEntity("Pipe Mesh");
	scene->SetParent(branchEntity, self);

	auto mesh = GeneratePipeMesh(treePipeMeshGeneratorSettings);
	auto material = ProjectManager::CreateTemporaryAsset<Material>();
	auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(branchEntity).lock();

	material->m_materialProperties.m_albedoColor = glm::vec3(109, 79, 75) / 255.0f;

	material->m_materialProperties.m_roughness = 1.0f;
	material->m_materialProperties.m_metallic = 0.0f;
	meshRenderer->m_mesh = mesh;
	meshRenderer->m_material = material;
}

void Tree::ClearMeshRendererPipe()
{
	const auto scene = GetScene();
	const auto self = GetOwner();
	const auto children = scene->GetChildren(self);
	for (const auto& child : children) {
		auto name = scene->GetEntityName(child);
		if (name == "Pipe Mesh") {
			scene->DeleteEntity(child);
		}
	}
}
