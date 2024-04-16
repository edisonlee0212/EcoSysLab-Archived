//
// Created by lllll on 10/24/2022.
//
#include "ShootDescriptor.hpp"
#include "Tree.hpp"
#include "SkeletonSerializer.hpp"
#include "StrandGroupSerializer.hpp"
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
#include "FoliageDescriptor.hpp"
#include "HeightField.hpp"
#include "StrandsRenderer.hpp"
#include "StrandModelProfileSerializer.hpp"
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
	ClearSkeletalGraph();
	ClearGeometryEntities();
	ClearStrandModelMeshRenderer();
	ClearStrandRenderer();
	ClearTwigsStrandRenderer();
	m_treeModel.Clear();
	m_strandModel = {};
	m_treeModel.m_index = GetOwner().GetIndex();
	m_treeVisualizer.Reset(m_treeModel);
}

void Tree::ClearSkeletalGraph() const
{
	const auto scene = GetScene();
	const auto self = GetOwner();
	const auto children = scene->GetChildren(self);
	for (const auto& child : children) {
		auto name = scene->GetEntityName(child);
		if (name == "Skeletal Graph Lines") {
			scene->DeleteEntity(child);
		}
		else if (name == "Skeletal Graph Points") {
			scene->DeleteEntity(child);
		}
	}

}

void Tree::GenerateSkeletalGraph(
	const SkeletalGraphSettings& skeletalGraphSettings, 
	SkeletonNodeHandle baseNodeHandle,
	const std::shared_ptr<Mesh>& pointMeshSample,
	const std::shared_ptr<Mesh>& lineMeshSample) const
{
	const auto scene = GetScene();
	const auto self = GetOwner();
	ClearSkeletalGraph();

	const auto lineEntity = scene->CreateEntity("Skeletal Graph Lines");
	scene->SetParent(lineEntity, self);

	const auto pointEntity = scene->CreateEntity("Skeletal Graph Points");
	scene->SetParent(pointEntity, self);


	bool strandReady = false;
	if (m_strandModel.m_strandModelSkeleton.PeekSortedNodeList().size() > 1)
	{
		strandReady = true;
	}

	const auto lineList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
	const auto lineMaterial = ProjectManager::CreateTemporaryAsset<Material>();
	const auto lineParticles = scene->GetOrSetPrivateComponent<Particles>(lineEntity).lock();
	lineParticles->m_mesh = lineMeshSample;
	lineParticles->m_material = lineMaterial;
	lineParticles->m_particleInfoList = lineList;
	lineMaterial->m_vertexColorOnly = true;
	const auto pointList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
	const auto pointMaterial = ProjectManager::CreateTemporaryAsset<Material>();
	const auto pointParticles = scene->GetOrSetPrivateComponent<Particles>(pointEntity).lock();
	pointParticles->m_mesh = pointMeshSample;
	pointParticles->m_material = pointMaterial;
	pointParticles->m_particleInfoList = pointList;
	pointMaterial->m_vertexColorOnly = true;

	std::vector<ParticleInfo> lineParticleInfos;
	std::vector<ParticleInfo> pointParticleInfos;
	const int nodeSize = strandReady ? m_strandModel.m_strandModelSkeleton.PeekSortedNodeList().size() : m_treeModel.PeekShootSkeleton().PeekSortedNodeList().size();
	if (strandReady) {
		lineParticleInfos.resize(nodeSize);
		pointParticleInfos.resize(nodeSize);
	}
	else
	{
		lineParticleInfos.resize(nodeSize);
		pointParticleInfos.resize(nodeSize);
	}
	Jobs::ParallelFor(nodeSize, [&](unsigned internodeIndex)
		{
			if (strandReady)
			{
				const auto& sortedInternodeList = m_strandModel.m_strandModelSkeleton.PeekSortedNodeList();
				const auto internodeHandle = sortedInternodeList[internodeIndex];
				SkeletonNodeHandle walker = internodeHandle;
				bool subTree = false;
				const auto& skeleton = m_strandModel.m_strandModelSkeleton;
				const auto& node = skeleton.PeekNode(internodeHandle);

				while (walker != -1)
				{
					if (walker == baseNodeHandle)
					{
						subTree = true;
						break;
					}
					walker = skeleton.PeekNode(walker).GetParentHandle();
				}
				const glm::vec3 position = node.m_info.m_globalPosition;
				auto rotation = node.m_info.m_globalRotation;
				{

					rotation *= glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f));
					const glm::mat4 rotationTransform = glm::mat4_cast(rotation);
					lineParticleInfos[internodeIndex].m_instanceMatrix.m_value =
						glm::translate(position + (node.m_info.m_length / 2.0f) * node.m_info.GetGlobalDirection()) *
						rotationTransform *
						glm::scale(glm::vec3(
							skeletalGraphSettings.m_fixedLineThickness * (subTree ? 1.25f : 1.0f),
							node.m_info.m_length,
							skeletalGraphSettings.m_fixedLineThickness * (subTree ? 1.25f : 1.0f)));

					if (subTree)
					{
						lineParticleInfos[internodeIndex].m_instanceColor = skeletalGraphSettings.m_lineFocusColor;
					}
					else {
						lineParticleInfos[internodeIndex].m_instanceColor = skeletalGraphSettings.m_lineColor;
					}
				}
				{
					rotation *= glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f));
					const glm::mat4 rotationTransform = glm::mat4_cast(rotation);
					float thicknessFactor = node.m_info.m_thickness;
					if (skeletalGraphSettings.m_fixedPointSize) thicknessFactor = skeletalGraphSettings.m_fixedPointSizeFactor;
					auto scale = glm::vec3(skeletalGraphSettings.m_branchPointSize * thicknessFactor);
					pointParticleInfos[internodeIndex].m_instanceColor = skeletalGraphSettings.m_branchPointColor;
					if (internodeIndex == 0 || node.PeekChildHandles().size() > 1)
					{
						scale = glm::vec3(skeletalGraphSettings.m_junctionPointSize * thicknessFactor);
						pointParticleInfos[internodeIndex].m_instanceColor = skeletalGraphSettings.m_junctionPointColor;
					}
					pointParticleInfos[internodeIndex].m_instanceMatrix.m_value =
						glm::translate(position) *
						rotationTransform *
						glm::scale(scale * (subTree ? 1.25f : 1.0f));
					if (subTree)
					{
						pointParticleInfos[internodeIndex].m_instanceColor = skeletalGraphSettings.m_branchFocusColor;
					}
				}
			}
			else {
				const auto& sortedInternodeList = m_treeModel.PeekShootSkeleton().PeekSortedNodeList();
				const auto internodeHandle = sortedInternodeList[internodeIndex];
				SkeletonNodeHandle walker = internodeHandle;
				bool subTree = false;
				const auto& skeleton = m_treeModel.PeekShootSkeleton();
				const auto& node = skeleton.PeekNode(internodeHandle);

				while (walker != -1)
				{
					if (walker == baseNodeHandle)
					{
						subTree = true;
						break;
					}
					walker = skeleton.PeekNode(walker).GetParentHandle();
				}
				const glm::vec3 position = node.m_info.m_globalPosition;
				auto rotation = node.m_info.m_globalRotation;
				{

					rotation *= glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f));
					const glm::mat4 rotationTransform = glm::mat4_cast(rotation);
					lineParticleInfos[internodeIndex].m_instanceMatrix.m_value =
						glm::translate(position + (node.m_info.m_length / 2.0f) * node.m_info.GetGlobalDirection()) *
						rotationTransform *
						glm::scale(glm::vec3(
							skeletalGraphSettings.m_fixedLineThickness * (subTree ? 1.25f : 1.0f),
							node.m_info.m_length,
							skeletalGraphSettings.m_fixedLineThickness * (subTree ? 1.25f : 1.0f)));

					if (subTree)
					{
						lineParticleInfos[internodeIndex].m_instanceColor = skeletalGraphSettings.m_lineFocusColor;
					}
					else {
						lineParticleInfos[internodeIndex].m_instanceColor = skeletalGraphSettings.m_lineColor;
					}
				}
				{
					rotation *= glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f));
					const glm::mat4 rotationTransform = glm::mat4_cast(rotation);
					float thicknessFactor = node.m_info.m_thickness;
					if (skeletalGraphSettings.m_fixedPointSize) thicknessFactor = skeletalGraphSettings.m_fixedPointSizeFactor;
					auto scale = glm::vec3(skeletalGraphSettings.m_branchPointSize * thicknessFactor);
					pointParticleInfos[internodeIndex].m_instanceColor = skeletalGraphSettings.m_branchPointColor;
					if (internodeIndex == 0 || node.PeekChildHandles().size() > 1)
					{
						scale = glm::vec3(skeletalGraphSettings.m_junctionPointSize * thicknessFactor);
						pointParticleInfos[internodeIndex].m_instanceColor = skeletalGraphSettings.m_junctionPointColor;
					}
					pointParticleInfos[internodeIndex].m_instanceMatrix.m_value =
						glm::translate(position) *
						rotationTransform *
						glm::scale(scale * (subTree ? 1.25f : 1.0f));
					if (subTree)
					{
						pointParticleInfos[internodeIndex].m_instanceColor = skeletalGraphSettings.m_branchFocusColor;
					}
				}
			}

		});
	lineList->SetParticleInfos(lineParticleInfos);
	pointList->SetParticleInfos(pointParticleInfos);
}

void Tree::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
	const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
	const auto scene = GetScene();
	editorLayer->DragAndDropButton<TreeDescriptor>(m_treeDescriptor, "TreeDescriptor", true);
	static bool showSpaceColonizationGrid = false;

	static std::shared_ptr<ParticleInfoList> spaceColonizationGridParticleInfoList;
	if (!spaceColonizationGridParticleInfoList)
	{
		spaceColonizationGridParticleInfoList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
	}

	const auto treeDescriptor = m_treeDescriptor.Get<TreeDescriptor>();
	if (treeDescriptor) {
		const auto shootDescriptor = treeDescriptor->m_shootDescriptor.Get<ShootDescriptor>();
		if (shootDescriptor) {
			/*
			ImGui::DragInt("Seed", &m_treeModel.m_seed, 1, 0);
			if (ImGui::Button("Reset")) {
				Reset();
				modelChanged = true;
			}*/
			if (ImGui::TreeNode("Tree settings")) {

				ImGui::Checkbox("Enable History", &m_enableHistory);
				if (m_enableHistory)
				{
					ImGui::DragInt("History per iteration", &m_historyIteration, 1, 1, 1000);
				}
				if (ImGui::TreeNode("Sagging"))
				{
					bool changed = false;
					changed = ImGui::DragFloat("Sagging strength", &shootDescriptor->m_saggingFactorThicknessReductionMax.x, 0.0001f, 0.0f, 10.0f, "%.5f") || changed;
					changed = ImGui::DragFloat("Sagging thickness factor", &shootDescriptor->m_saggingFactorThicknessReductionMax.y, 0.01f, 0.0f, 10.0f, "%.5f") || changed;
					changed = ImGui::DragFloat("Sagging max", &shootDescriptor->m_saggingFactorThicknessReductionMax.z, 0.001f, 0.0f, 1.0f, "%.5f") || changed;
					if (changed)
					{
						m_shootGrowthController.m_sagging = [=](const SkeletonNode<InternodeGrowthData>& internode)
							{
								const auto newSagging = glm::min(
									shootDescriptor->m_saggingFactorThicknessReductionMax.z,
									shootDescriptor->m_saggingFactorThicknessReductionMax.x *
									(internode.m_data.m_descendantTotalBiomass + internode.m_data.m_extraMass) /
									glm::pow(
										internode.m_info.m_thickness /
										shootDescriptor->m_endNodeThickness,
										shootDescriptor->m_saggingFactorThicknessReductionMax.y));
								return newSagging;
							};
						m_treeModel.CalculateTransform(m_shootGrowthController, true);
						m_shootGrowthController.m_sagging = [=](const SkeletonNode<InternodeGrowthData>& internode)
							{
								const auto& shootGrowthParameters = treeDescriptor->m_shootDescriptor;
								const auto newSagging = glm::min(
									shootDescriptor->m_saggingFactorThicknessReductionMax.z,
									shootDescriptor->m_saggingFactorThicknessReductionMax.x *
									(internode.m_data.m_descendantTotalBiomass + internode.m_data.m_extraMass) /
									glm::pow(
										internode.m_info.m_thickness /
										shootDescriptor->m_endNodeThickness,
										shootDescriptor->m_saggingFactorThicknessReductionMax.y));
								return glm::max(internode.m_data.m_sagging, newSagging);
							};
						m_treeVisualizer.m_needUpdate = true;
					}
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
								shootDescriptor->m_internodeLength,
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
			static int meshGenerateIterations = 0;
			if (ImGui::TreeNode("Cylindrical Mesh generation settings")) {

				ImGui::DragInt("Iterations", &meshGenerateIterations, 1, 0, m_treeModel.CurrentIteration());
				meshGenerateIterations = glm::clamp(meshGenerateIterations, 0, m_treeModel.CurrentIteration());
				m_meshGeneratorSettings.OnInspect(editorLayer);

				ImGui::TreePop();
			}
			if (ImGui::Button("Generate Cylindrical Mesh")) {
				GenerateGeometryEntities(m_meshGeneratorSettings, meshGenerateIterations);
			}
			ImGui::SameLine();
			if (ImGui::Button("Clear Cylindrical Mesh"))
			{
				ClearGeometryEntities();
			}

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
	if (ImGui::TreeNode("Strand Model")) {

		auto& strandModelParameters = m_strandModelParameters;
		if (ImGui::TreeNodeEx("Profile settings", ImGuiTreeNodeFlags_DefaultOpen))
		{
			if (ImGui::TreeNode("Physics settings"))
			{
				ImGui::DragFloat("Physics damping", &strandModelParameters.m_profilePhysicsSettings.m_damping, 0.01f, 0.0f, 1.0f);
				ImGui::DragFloat("Physics max speed", &strandModelParameters.m_profilePhysicsSettings.m_maxSpeed, 0.01f, 0.0f, 100.0f);
				ImGui::DragFloat("Physics particle softness", &strandModelParameters.m_profilePhysicsSettings.m_particleSoftness, 0.01f, 0.0f, 1.0f);
				ImGui::TreePop();
			}
			ImGui::DragFloat("Center attraction strength", &strandModelParameters.m_centerAttractionStrength, 100.f, 0.0f, 10000.0f);
			ImGui::DragInt("Max iteration cell factor", &strandModelParameters.m_maxSimulationIterationCellFactor, 1, 0, 500);
			ImGui::DragInt("Branch Packing Timeout", &strandModelParameters.m_branchProfilePackingMaxIteration, 1, 0, 10000);
			ImGui::DragInt("Junction Packing Timeout", &strandModelParameters.m_junctionProfilePackingMaxIteration, 1, 20, 10000);
			ImGui::DragInt("Modified Packing Timeout", &strandModelParameters.m_modifiedProfilePackingMaxIteration, 1, 20, 10000);
			ImGui::DragInt("Timeout with boundaries)", &strandModelParameters.m_modifiedProfilePackingMaxIteration, 1, 20, 10000);
			ImGui::TreePop();
		}
		ImGui::DragFloat("Overlap threshold", &strandModelParameters.m_overlapThreshold, 0.01f, 0.0f, 1.0f);
		ImGui::DragInt("Initial branch strand count", &strandModelParameters.m_strandsAlongBranch, 1, 1, 50);
		ImGui::DragInt("Initial end node strand count", &strandModelParameters.m_endNodeStrands, 1, 1, 50);

		ImGui::Checkbox("Pre-merge", &strandModelParameters.m_preMerge);

		static PlottedDistributionSettings plottedDistributionSettings = { 0.001f,
														{0.001f, true, true, ""},
														{0.001f, true, true, ""},
														"" };
		m_strandModelParameters.m_branchTwistDistribution.OnInspect("Branch Twist", plottedDistributionSettings);
		m_strandModelParameters.m_junctionTwistDistribution.OnInspect("Junction Twist", plottedDistributionSettings);
		m_strandModelParameters.m_strandRadiusDistribution.OnInspect("Strand Thickness", plottedDistributionSettings);

		ImGui::DragFloat("Cladoptosis Range", &strandModelParameters.m_cladoptosisRange, 0.01f, 0.0f, 50.f);
		m_strandModelParameters.m_cladoptosisDistribution.OnInspect("Cladoptosis", plottedDistributionSettings);
		ImGui::Text(("Strand count: " + std::to_string(m_strandModel.m_strandModelSkeleton.m_data.m_strandGroup.PeekStrands().size())).c_str());
		ImGui::Text(("Total particle count: " + std::to_string(m_strandModel.m_strandModelSkeleton.m_data.m_numOfParticles)).c_str());


		if (ImGui::TreeNodeEx("Graph Adjustment settings", ImGuiTreeNodeFlags_DefaultOpen))
		{
			ImGui::DragFloat("Side factor", &strandModelParameters.m_sidePushFactor, 0.01f, 0.0f, 2.0f);
			ImGui::DragFloat("Apical Side factor", &strandModelParameters.m_apicalSidePushFactor, 0.01f, 0.0f, 2.0f);
			ImGui::DragFloat("Rotation factor", &strandModelParameters.m_rotationPushFactor, 0.01f, 0.0f, 2.0f);
			ImGui::DragFloat("Apical Rotation factor", &strandModelParameters.m_apicalBranchRotationPushFactor, 0.01f, 0.0f, 2.0f);
			ImGui::TreePop();
		}
		ImGui::DragInt("Max node count", &strandModelParameters.m_nodeMaxCount, 1, -1, 999);
		ImGui::DragInt("Boundary point distance", &strandModelParameters.m_boundaryPointDistance, 1, 3, 30);
		ImGui::ColorEdit4("Boundary color", &strandModelParameters.m_boundaryPointColor.x);
		ImGui::ColorEdit4("Content color", &strandModelParameters.m_contentPointColor.x);

		if (ImGui::Button("Rebuild Strand Model"))
		{
			m_strandModel = {};
			BuildStrandModel();
		}

		ImGui::SameLine();
		if (ImGui::Button("Clear Strand Model"))
		{
			m_strandModel = {};
		}

		if (ImGui::TreeNodeEx("Strand Model Mesh Generator Settings", ImGuiTreeNodeFlags_DefaultOpen)) {
			m_strandModelMeshGeneratorSettings.OnInspect(editorLayer);
			ImGui::TreePop();
		}

		ImGui::TreePop();
	}

	if (ImGui::Button("Build StrandRenderer"))
	{
		InitializeStrandRenderer();
	}
	ImGui::SameLine();
	if (ImGui::Button("Clear StrandRenderer"))
	{
		ClearStrandRenderer();
	}
	if (ImGui::Button("Build Strand Mesh"))
	{
		InitializeStrandModelMeshRenderer(m_strandModelMeshGeneratorSettings);
	}
	ImGui::SameLine();
	if (ImGui::Button("Clear Strand Mesh"))
	{
		ClearStrandModelMeshRenderer();
	}
	if (ImGui::TreeNode("Tree Visualizer"))
	{
		m_treeVisualizer.OnInspect(m_treeModel);
		ImGui::TreePop();
	}
	m_treeVisualizer.Visualize(m_strandModel);
	if (ImGui::TreeNode("Skeletal graph settings"))
	{
		m_skeletalGraphSettings.OnInspect();
	}
	if (ImGui::Button("Build skeletal graph"))
	{
		GenerateSkeletalGraph(m_skeletalGraphSettings, -1,
			Resources::GetResource<Mesh>("PRIMITIVE_SPHERE"),
			Resources::GetResource<Mesh>("PRIMITIVE_CUBE"));
	}
	ImGui::SameLine();
	if (ImGui::Button("Clear skeletal graph"))
	{
		ClearSkeletalGraph();
	}

	FileUtils::SaveFile("Export Cylindrical Mesh", "OBJ", { ".obj" }, [&](const std::filesystem::path& path) {
		ExportOBJ(path, m_meshGeneratorSettings);
		}, false);
	ImGui::SameLine();
	FileUtils::SaveFile("Export Strand Mesh", "OBJ", { ".obj" }, [&](const std::filesystem::path& path) {
		ExportStrandModelOBJ(path, m_strandModelMeshGeneratorSettings);
		}, false);
}
void Tree::Update()
{
	if (m_temporalProgression) {
		if (m_temporalProgressionIteration <= m_treeModel.CurrentIteration()) {
			GenerateGeometryEntities(m_meshGeneratorSettings, m_temporalProgressionIteration);
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
	m_treeModel = {};
	m_strandModel = {};

	m_treeVisualizer.Initialize();

	m_strandModelParameters.m_branchTwistDistribution.m_mean = { -60.0f, 60.0f };
	m_strandModelParameters.m_branchTwistDistribution.m_deviation = { 0.0f, 1.0f, {0, 0} };

	m_strandModelParameters.m_junctionTwistDistribution.m_mean = { -60.0f, 60.0f };
	m_strandModelParameters.m_junctionTwistDistribution.m_deviation = { 0.0f, 1.0f, {0, 0} };

	m_strandModelParameters.m_strandRadiusDistribution.m_mean = { 0.0f, 0.002f };
	m_strandModelParameters.m_strandRadiusDistribution.m_deviation = { 0.0f, 1.0f, {0, 0} };

	m_strandModelParameters.m_cladoptosisDistribution.m_mean = { 0.0f, 0.02f };
	m_strandModelParameters.m_cladoptosisDistribution.m_deviation = { 0.0f, 1.0f, {0, 0} };
}

void Tree::OnDestroy() {
	m_treeModel = {};
	m_strandModel = {};

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

void Tree::CalculateProfiles()
{
	const float time = Times::Now();
	m_strandModel.m_strandModelSkeleton.Clone(m_treeModel.RefShootSkeleton());

	m_strandModel.InitializeProfiles(m_strandModelParameters);
	m_strandModel.CalculateProfiles(m_strandModelParameters);
	const float profileCalculationTime = Times::Now() - time;
	std::string output;
	output += "\nProfile count: [" + std::to_string(m_strandModel.m_strandModelSkeleton.PeekSortedNodeList().size());
	output += "], Strand count: [" + std::to_string(m_strandModel.m_strandModelSkeleton.m_data.m_strandGroup.PeekStrands().size());
	output += "], Particle count: [" + std::to_string(m_strandModel.m_strandModelSkeleton.m_data.m_numOfParticles);
	output += "]\nCalculate Profile Used time: " + std::to_string(profileCalculationTime) + "\n";
	EVOENGINE_LOG(output);
}

void Tree::BuildStrandModel()
{
	std::string output;
	if (m_strandModel.m_strandModelSkeleton.RefRawNodes().size() != m_treeModel.PeekShootSkeleton().RefRawNodes().size())
	{
		CalculateProfiles();
	}
	else
	{
		for (const auto& nodeHandle : m_treeModel.PeekShootSkeleton().PeekSortedNodeList())
		{
			m_strandModel.m_strandModelSkeleton.RefNode(nodeHandle).m_info = m_treeModel.PeekShootSkeleton().PeekNode(nodeHandle).m_info;
		}
	}
	const float time = Times::Now();
	m_strandModel.CalculateStrandProfileAdjustedTransforms(m_strandModelParameters);
	m_strandModel.ApplyProfiles(m_strandModelParameters);
	const float strandModelingTime = Times::Now() - time;
	output += "\nBuild Strand Model Used time: " + std::to_string(strandModelingTime) + "\n";
	EVOENGINE_LOG(output);
}

std::shared_ptr<Strands> Tree::GenerateStrands() const
{
	const auto strandsAsset = ProjectManager::CreateTemporaryAsset<Strands>();
	const auto& parameters = m_strandModelParameters;
	std::vector<glm::uint> strandsList;
	std::vector<StrandPoint> points;
	m_strandModel.m_strandModelSkeleton.m_data.m_strandGroup.BuildStrands(strandsList, points, parameters.m_nodeMaxCount);
	if (!points.empty()) strandsList.emplace_back(points.size());
	StrandPointAttributes strandPointAttributes{};
	strandPointAttributes.m_color = true;
	strandsAsset->SetStrands(strandPointAttributes, strandsList, points);
	return strandsAsset;
}

void Tree::GenerateTrunkMeshes(const std::shared_ptr<Mesh>& trunkMesh, const TreeMeshGeneratorSettings& meshGeneratorSettings)
{
	const auto& sortedInternodeList = m_treeModel.RefShootSkeleton().PeekSortedNodeList();
	std::unordered_set<SkeletonNodeHandle> trunkHandles{};
	for (const auto& nodeHandle : sortedInternodeList)
	{
		const auto& node = m_treeModel.RefShootSkeleton().PeekNode(nodeHandle);
		trunkHandles.insert(nodeHandle);
		if (node.PeekChildHandles().size() > 1)break;
	}
	{
		std::vector<Vertex> vertices;
		std::vector<unsigned int> indices;
		const auto treeDescriptor = m_treeDescriptor.Get<TreeDescriptor>();
		std::shared_ptr<BranchShape> branchShape{};
		if (treeDescriptor)
		{
			branchShape = treeDescriptor->m_shootBranchShape.Get<BranchShape>();
		}
		CylindricalMeshGenerator<ShootGrowthData, ShootStemGrowthData, InternodeGrowthData>::GeneratePartially(trunkHandles, m_treeModel.PeekShootSkeleton(), vertices, indices, meshGeneratorSettings,
			[&](glm::vec3& vertexPosition, const glm::vec3& direction, const float xFactor, const float yFactor)
			{
				if (branchShape)
				{
					const float pushValue = branchShape->GetValue(xFactor, yFactor);
					vertexPosition += pushValue * direction;
				}
			},
			[&](glm::vec2& texCoords, float xFactor, float distanceToRoot)
			{});
		VertexAttributes attributes{};
		attributes.m_texCoord = true;
		trunkMesh->SetVertices(attributes, vertices, indices);
	}
}

std::shared_ptr<Mesh> Tree::GenerateBranchMesh(const TreeMeshGeneratorSettings& meshGeneratorSettings)
{
	std::vector<Vertex> vertices;
	std::vector<unsigned int> indices;
	if (meshGeneratorSettings.m_branchMeshType == 0) {
		auto treeDescriptor = m_treeDescriptor.Get<TreeDescriptor>();
		if (!treeDescriptor)
		{
			EVOENGINE_WARNING("TreeDescriptor missing!");
			treeDescriptor = ProjectManager::CreateTemporaryAsset<TreeDescriptor>();
			treeDescriptor->m_foliageDescriptor = ProjectManager::CreateTemporaryAsset<FoliageDescriptor>();
		}
		std::shared_ptr<BranchShape> branchShape{};
		branchShape = treeDescriptor->m_shootBranchShape.Get<BranchShape>();
		if (m_strandModel.m_strandModelSkeleton.RefRawNodes().size() == m_treeModel.m_shootSkeleton.RefRawNodes().size())
		{
			CylindricalMeshGenerator<StrandModelSkeletonData, StrandModelFlowData, StrandModelNodeData>::Generate(m_strandModel.m_strandModelSkeleton, vertices, indices, meshGeneratorSettings,
				[&](glm::vec3& vertexPosition, const glm::vec3& direction, const float xFactor, const float yFactor)
				{
					if (branchShape)
					{
						const float pushValue = branchShape->GetValue(xFactor, yFactor);
						vertexPosition += pushValue * direction;
					}
				},
				[&](glm::vec2& texCoords, float xFactor, float distanceToRoot)
				{});
		}
		else {
			CylindricalMeshGenerator<ShootGrowthData, ShootStemGrowthData, InternodeGrowthData>::Generate(m_treeModel.PeekShootSkeleton(), vertices, indices, meshGeneratorSettings,
				[&](glm::vec3& vertexPosition, const glm::vec3& direction, const float xFactor, const float yFactor)
				{},
				[&](glm::vec2& texCoords, float xFactor, float distanceToRoot)
				{}
			);
		}
	}
	else
	{
		auto treeDescriptor = m_treeDescriptor.Get<TreeDescriptor>();
		if (!treeDescriptor)
		{
			EVOENGINE_WARNING("TreeDescriptor missing!");
			treeDescriptor = ProjectManager::CreateTemporaryAsset<TreeDescriptor>();
			treeDescriptor->m_foliageDescriptor = ProjectManager::CreateTemporaryAsset<FoliageDescriptor>();
		}
		VoxelMeshGenerator<ShootGrowthData, ShootStemGrowthData, InternodeGrowthData>::Generate(m_treeModel.PeekShootSkeleton(), vertices, indices, meshGeneratorSettings);
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
	if (!treeDescriptor)
	{
		EVOENGINE_WARNING("TreeDescriptor missing!");
		treeDescriptor = ProjectManager::CreateTemporaryAsset<TreeDescriptor>();
		treeDescriptor->m_foliageDescriptor = ProjectManager::CreateTemporaryAsset<FoliageDescriptor>();
	}
	auto foliageDescriptor = treeDescriptor->m_foliageDescriptor.Get<FoliageDescriptor>();
	if (!foliageDescriptor) foliageDescriptor = ProjectManager::CreateTemporaryAsset<FoliageDescriptor>();
	const auto& nodeList = m_treeModel.PeekShootSkeleton().PeekSortedNodeList();
	for (const auto& internodeHandle : nodeList) {
		const auto& internode = m_treeModel.PeekShootSkeleton().PeekNode(internodeHandle);
		const auto& internodeInfo = internode.m_info;
		std::vector<glm::mat4> leafMatrices;
		foliageDescriptor->GenerateFoliageMatrices(leafMatrices, internodeInfo);
		Vertex archetype;
		for (const auto& matrix : leafMatrices)
		{
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

	auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
	VertexAttributes attributes{};
	attributes.m_texCoord = true;
	mesh->SetVertices(attributes, vertices, indices);
	return mesh;
}

std::shared_ptr<Strands> Tree::GenerateTwigStrands(const TreeMeshGeneratorSettings& meshGeneratorSettings)
{
	const auto treeDescriptor = m_treeDescriptor.Get<TreeDescriptor>();
	if (!treeDescriptor) return nullptr;
	const auto foliageDescriptor = treeDescriptor->m_foliageDescriptor.Get<FoliageDescriptor>();
	if (!foliageDescriptor) return nullptr;
	auto strands = ProjectManager::CreateTemporaryAsset<Strands>();

	std::vector<glm::uint> twigSegments;
	std::vector<StrandPoint> twigPoints;
	const auto& shootSkeleton = m_treeModel.PeekShootSkeleton();
	const auto& internodeList = shootSkeleton.PeekSortedNodeList();

	for (int internodeHandle : internodeList)
	{
		const auto& internode = shootSkeleton.PeekNode(internodeHandle);
		const auto& internodeData = internode.m_data;
		const auto& internodeInfo = internode.m_info;
		std::vector<std::vector<glm::vec4>> twigs{};
		if (internodeInfo.m_thickness < foliageDescriptor->m_twigParameters.m_maxNodeThickness
			&& internodeInfo.m_rootDistance > foliageDescriptor->m_twigParameters.m_minRootDistance
			&& internodeInfo.m_endDistance < foliageDescriptor->m_twigParameters.m_maxEndDistance)
		{
			int twigCount = internodeInfo.m_length / foliageDescriptor->m_twigParameters.m_unitDistance;
			twigs.resize(twigCount);

			auto desiredGlobalRotation = internodeInfo.m_regulatedGlobalRotation * glm::quat(glm::vec3(
				glm::radians(foliageDescriptor->m_twigParameters.m_branchingAngle), 0.0f,
				glm::radians(glm::radians(
					glm::linearRand(0.0f,
						360.0f)))));

			glm::vec3 directionStart = internodeInfo.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
			glm::vec3 directionEnd = directionStart;

			glm::vec3 positionStart = internodeInfo.m_globalPosition;
			glm::vec3 positionEnd =
				positionStart + internodeInfo.m_length * (meshGeneratorSettings.m_smoothness ? 1.0f - meshGeneratorSettings.m_baseControlPointRatio * 0.5f : 1.0f) * internodeInfo.GetGlobalDirection();

			BezierCurve curve = BezierCurve(
				positionStart,
				positionStart +
				(meshGeneratorSettings.m_smoothness ? internodeInfo.m_length * meshGeneratorSettings.m_baseControlPointRatio : 0.0f) * directionStart,
				positionEnd -
				(meshGeneratorSettings.m_smoothness ? internodeInfo.m_length * meshGeneratorSettings.m_branchControlPointRatio : 0.0f) * directionEnd,
				positionEnd);

			for (int twigIndex = 0; twigIndex < twigCount; twigIndex++) {
				glm::vec3 positionWalker = curve.GetPoint(static_cast<float>(twigIndex) / twigCount);
				twigs[twigIndex].resize(foliageDescriptor->m_twigParameters.m_segmentSize);
				const float rollAngle = glm::radians(glm::linearRand(0.0f, 360.0f));
				for (int twigPointIndex = 0; twigPointIndex < foliageDescriptor->m_twigParameters.m_segmentSize; twigPointIndex++)
				{
					twigs[twigIndex][twigPointIndex] = glm::vec4(positionWalker, foliageDescriptor->m_twigParameters.m_thickness);
					desiredGlobalRotation = internodeInfo.m_regulatedGlobalRotation * glm::quat(glm::vec3(
						glm::radians(glm::gaussRand(0.f, foliageDescriptor->m_twigParameters.m_apicalAngleVariance) + foliageDescriptor->m_twigParameters.m_branchingAngle), 0.0f,
						rollAngle));

					auto twigFront = desiredGlobalRotation * glm::vec3(0, 0, -1);
					positionWalker = positionWalker + twigFront * foliageDescriptor->m_twigParameters.m_segmentLength;
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

	StrandPointAttributes strandPointAttributes{};
	strands->SetSegments(strandPointAttributes, twigSegments, twigPoints);
	return strands;
}

std::shared_ptr<ParticleInfoList> Tree::GenerateFoliageParticleInfoList(
	const TreeMeshGeneratorSettings& meshGeneratorSettings)
{
	auto treeDescriptor = m_treeDescriptor.Get<TreeDescriptor>();
	if (!treeDescriptor)
	{
		EVOENGINE_WARNING("TreeDescriptor missing!");
		treeDescriptor = ProjectManager::CreateTemporaryAsset<TreeDescriptor>();
		treeDescriptor->m_foliageDescriptor = ProjectManager::CreateTemporaryAsset<FoliageDescriptor>();
	}
	const auto retVal = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
	auto foliageDescriptor = treeDescriptor->m_foliageDescriptor.Get<FoliageDescriptor>();
	if (!foliageDescriptor) foliageDescriptor = ProjectManager::CreateTemporaryAsset<FoliageDescriptor>();
	std::vector<ParticleInfo> particleInfos;
	const auto& nodeList = m_treeModel.PeekShootSkeleton().PeekSortedNodeList();
	for (const auto& internodeHandle : nodeList) {
		const auto& internode = m_treeModel.PeekShootSkeleton().PeekNode(internodeHandle);
		const auto& internodeInfo = internode.m_info;
		std::vector<glm::mat4> leafMatrices{};
		foliageDescriptor->GenerateFoliageMatrices(leafMatrices, internodeInfo);
		const auto startIndex = particleInfos.size();
		particleInfos.resize(startIndex + leafMatrices.size());
		for (int i = 0; i < leafMatrices.size(); i++)
		{
			auto& particleInfo = particleInfos.at(startIndex + i);
			particleInfo.m_instanceMatrix.m_value = leafMatrices.at(i);
			particleInfo.m_instanceColor = glm::vec4(0, 1, 0, 1);
		}
	}
	retVal->SetParticleInfos(particleInfos);
	return retVal;
}

std::shared_ptr<Mesh> Tree::GenerateStrandModelFoliageMesh(
	const StrandModelMeshGeneratorSettings& strandModelMeshGeneratorSettings)
{
	std::vector<Vertex> vertices;
	std::vector<unsigned int> indices;

	auto quadMesh = Resources::GetResource<Mesh>("PRIMITIVE_QUAD");
	auto& quadTriangles = quadMesh->UnsafeGetTriangles();
	auto quadVerticesSize = quadMesh->GetVerticesAmount();
	size_t offset = 0;
	auto treeDescriptor = m_treeDescriptor.Get<TreeDescriptor>();
	if (!treeDescriptor) return nullptr;
	auto foliageDescriptor = treeDescriptor->m_foliageDescriptor.Get<FoliageDescriptor>();
	if (!foliageDescriptor) foliageDescriptor = ProjectManager::CreateTemporaryAsset<FoliageDescriptor>();
	const auto& nodeList = m_strandModel.m_strandModelSkeleton.PeekSortedNodeList();
	for (const auto& internodeHandle : nodeList) {
		const auto& strandModelNode = m_strandModel.m_strandModelSkeleton.PeekNode(internodeHandle);
		std::vector<glm::mat4> leafMatrices;
		foliageDescriptor->GenerateFoliageMatrices(leafMatrices, strandModelNode.m_info);
		Vertex archetype;
		for (const auto& matrix : leafMatrices)
		{

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

	auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
	VertexAttributes attributes{};
	attributes.m_texCoord = true;
	mesh->SetVertices(attributes, vertices, indices);
	return mesh;
}

std::shared_ptr<Mesh> Tree::GenerateStrandModelBranchMesh(const StrandModelMeshGeneratorSettings& strandModelMeshGeneratorSettings)
{
	std::vector<Vertex> vertices;
	std::vector<unsigned int> indices;
	StrandModelMeshGenerator::Generate(m_strandModel, vertices, indices, strandModelMeshGeneratorSettings);

	auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
	VertexAttributes attributes{};
	attributes.m_texCoord = true;
	mesh->SetVertices(attributes, vertices, indices);
	return mesh;
}



void Tree::ExportOBJ(const std::filesystem::path& path, const TreeMeshGeneratorSettings& meshGeneratorSettings)
{
	if (path.extension() == ".obj") {
		try
		{
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
		catch (std::exception e)
		{
			EVOENGINE_ERROR("Export failed: " + std::string(e.what()));
		}

	}
}

void Tree::ExportStrandModelOBJ(const std::filesystem::path& path,
	const StrandModelMeshGeneratorSettings& meshGeneratorSettings)
{
	if (path.extension() == ".obj") {
		BuildStrandModel();
		try
		{
			std::ofstream of;
			of.open(path.string(), std::ofstream::out | std::ofstream::trunc);
			if (of.is_open()) {
				std::string start = "#Forest OBJ exporter, by Bosheng Li";
				start += "\n";
				of.write(start.c_str(), start.size());
				of.flush();
				unsigned vertexStartIndex = 1;
				unsigned texCoordsStartIndex = 1;
				if (meshGeneratorSettings.m_enableBranch) {
					std::vector<Vertex> vertices;
					std::vector<glm::vec2> texCoords;
					std::vector<std::pair<unsigned int, unsigned int>> indices;
					StrandModelMeshGenerator::Generate(m_strandModel, vertices, texCoords, indices, meshGeneratorSettings);
					if (!vertices.empty() && !indices.empty()) {
						std::string header =
							"#Vertices: " + std::to_string(vertices.size()) +
							", tris: " + std::to_string(indices.size());
						header += "\n";
						of.write(header.c_str(), header.size());
						of.flush();
						std::stringstream data;
						data << "o tree " + std::to_string(0) + "\n";
#pragma region Data collection
						for (auto& vertex : vertices)
						{
							auto& vertexPosition = vertex.m_position;
							auto& color = vertex.m_color;
							data << "v " + std::to_string(vertexPosition.x) + " " +
								std::to_string(vertexPosition.y) + " " +
								std::to_string(vertexPosition.z) + " " +
								std::to_string(color.x) + " " + std::to_string(color.y) + " " +
								std::to_string(color.z) + "\n";
						}
						for (const auto& texCoord : texCoords) {
							data << "vt " + std::to_string(texCoord.x) + " " +
								std::to_string(texCoord.y) + "\n";
						}
						data << "# List of indices for faces vertices, with (x, y, z).\n";
						for (auto i = 0; i < indices.size() / 3; i++) {
							const auto f1 = indices.at(i * 3).first + vertexStartIndex;
							const auto f2 = indices.at(i * 3 + 1).first + vertexStartIndex;
							const auto f3 = indices.at(i * 3 + 2).first + vertexStartIndex;
							const auto t1 = indices.at(i * 3).second + texCoordsStartIndex;
							const auto t2 = indices.at(i * 3 + 1).second + texCoordsStartIndex;
							const auto t3 = indices.at(i * 3 + 2).second + texCoordsStartIndex;
							data << "f " + std::to_string(f1) + "/" + std::to_string(t1) + "/" + std::to_string(f1) + " "
								+ std::to_string(f2) + "/" + std::to_string(t2) + "/" + std::to_string(f2) + " "
								+ std::to_string(f3) + "/" + std::to_string(t3) + "/" + std::to_string(f3) + "\n";
						}
#pragma endregion
						const auto result = data.str();
						of.write(result.c_str(), result.size());
						of.flush();
						vertexStartIndex += vertices.size();
						texCoordsStartIndex += texCoords.size();
					}
				}
				if (meshGeneratorSettings.m_enableFoliage) {
					if (const auto foliageMesh = GenerateStrandModelFoliageMesh(meshGeneratorSettings)) {
						const auto& vertices = foliageMesh->UnsafeGetVertices();
						const auto& triangles = foliageMesh->UnsafeGetTriangles();
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
							for (auto& vertex : vertices)
							{
								auto& vertexPosition = vertex.m_position;
								auto& color = vertex.m_color;
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
							for (auto triangle : triangles)
							{
								const auto f1 = triangle.x + vertexStartIndex;
								const auto f2 = triangle.y + vertexStartIndex;
								const auto f3 = triangle.z + vertexStartIndex;
								const auto t1 = triangle.x + texCoordsStartIndex;
								const auto t2 = triangle.y + texCoordsStartIndex;
								const auto t3 = triangle.z + texCoordsStartIndex;
								data << "f " + std::to_string(f1) + "/" + std::to_string(t1) + "/" + std::to_string(f1) + " "
									+ std::to_string(f2) + "/" + std::to_string(t2) + "/" + std::to_string(f2) + " "
									+ std::to_string(f3) + "/" + std::to_string(t3) + "/" + std::to_string(f3) + "\n";
							}
#pragma endregion
							const auto result = data.str();
							of.write(result.c_str(), result.size());
							of.flush();
							vertexStartIndex += vertices.size();
							texCoordsStartIndex += vertices.size();
						}
					}
				}
				of.close();
			}
		}
		catch (std::exception e)
		{
			EVOENGINE_ERROR("Export failed: " + std::string(e.what()));
		}
	}
}

void Tree::ExportTrunkOBJ(const std::filesystem::path& path,
	const TreeMeshGeneratorSettings& meshGeneratorSettings)
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
				std::shared_ptr<Mesh> trunkMesh = ProjectManager::CreateTemporaryAsset<Mesh>();
				GenerateTrunkMeshes(trunkMesh, meshGeneratorSettings);
				if (trunkMesh) {
					auto& vertices = trunkMesh->UnsafeGetVertices();
					auto& triangles = trunkMesh->UnsafeGetTriangles();
					if (!vertices.empty() && !triangles.empty()) {
						std::string header =
							"#Vertices: " + std::to_string(vertices.size()) +
							", tris: " + std::to_string(triangles.size());
						header += "\n";
						of.write(header.c_str(), header.size());
						of.flush();
						std::stringstream data;
						data << std::string("o trunk") + "\n";
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

bool Tree::TryGrow(float deltaTime, bool pruning, float overrideGrowthRate)
{
	const auto scene = GetScene();
	auto treeDescriptor = m_treeDescriptor.Get<TreeDescriptor>();
	if (!treeDescriptor) {
		EVOENGINE_WARNING("Growing tree without tree descriptor!");
		treeDescriptor = ProjectManager::CreateTemporaryAsset<TreeDescriptor>();
		m_treeDescriptor = treeDescriptor;
		const auto shootDescriptor = ProjectManager::CreateTemporaryAsset<ShootDescriptor>();
		treeDescriptor->m_shootDescriptor = shootDescriptor;
		const auto foliageDescriptor = ProjectManager::CreateTemporaryAsset<FoliageDescriptor>();
		treeDescriptor->m_foliageDescriptor = foliageDescriptor;
	}
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

	if (!m_soil.Get<Soil>()) {
		EVOENGINE_ERROR("No soil model!");
		return false;
	}
	if (!m_climate.Get<Climate>()) {
		EVOENGINE_ERROR("No climate model!");
		return false;
	}

	const auto owner = GetOwner();

	auto shootDescriptor = treeDescriptor->m_shootDescriptor.Get<ShootDescriptor>();
	if (!shootDescriptor)
	{
		shootDescriptor = ProjectManager::CreateTemporaryAsset<ShootDescriptor>();
		treeDescriptor->m_shootDescriptor = shootDescriptor;
		EVOENGINE_WARNING("Shoot Descriptor Missing!");
	}

	PrepareController(shootDescriptor, soil, climate);
	const bool grown = m_treeModel.Grow(deltaTime, scene->GetDataComponent<GlobalTransform>(owner).m_value, climate->m_climateModel, m_shootGrowthController, pruning, overrideGrowthRate);
	if (grown)
	{
		if (pruning) m_treeVisualizer.ClearSelections();
		m_treeVisualizer.m_needUpdate = true;
	}
	if (m_enableHistory && m_treeModel.m_iteration % m_historyIteration == 0) m_treeModel.Step();
	if (m_recordBiomassHistory)
	{
		const auto& baseShootNode = m_treeModel.RefShootSkeleton().RefNode(0);
		m_shootBiomassHistory.emplace_back(baseShootNode.m_data.m_biomass + baseShootNode.m_data.m_descendantTotalBiomass);
	}
	return grown;
}

bool Tree::TryGrowSubTree(const float deltaTime, const SkeletonNodeHandle baseInternodeHandle, const bool pruning, const float overrideGrowthRate) {
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

	auto shootDescriptor = treeDescriptor->m_shootDescriptor.Get<ShootDescriptor>();
	if (!shootDescriptor)
	{
		shootDescriptor = ProjectManager::CreateTemporaryAsset<ShootDescriptor>();
		treeDescriptor->m_shootDescriptor = shootDescriptor;
		EVOENGINE_WARNING("Shoot Descriptor Missing!");
	}

	PrepareController(shootDescriptor, soil, climate);
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
		m_shootBiomassHistory.emplace_back(baseShootNode.m_data.m_biomass + baseShootNode.m_data.m_descendantTotalBiomass);
	}
	return grown;
}


void Tree::Serialize(YAML::Emitter& out)
{
	m_treeDescriptor.Save("m_treeDescriptor", out);

	out << YAML::Key << "m_strandModel" << YAML::Value << YAML::BeginMap;
	{
		out << YAML::Key << "m_strandModelSkeleton" << YAML::Value << YAML::BeginMap;
		{
			SkeletonSerializer<StrandModelSkeletonData, StrandModelFlowData, StrandModelNodeData>::Serialize(out, m_strandModel.m_strandModelSkeleton,
				[&](YAML::Emitter& nodeOut, const StrandModelNodeData& nodeData)
				{
					nodeOut << YAML::Key << "m_profile" << YAML::Value << YAML::BeginMap;
					{
						StrandModelProfileSerializer<CellParticlePhysicsData>::Serialize(nodeOut, nodeData.m_profile,
							[&](YAML::Emitter& particleOut, const CellParticlePhysicsData& particleData){});
					}
					nodeOut << YAML::EndMap;
				},
				[&](YAML::Emitter& flowOut, const StrandModelFlowData& flowData){},
				[&](YAML::Emitter& skeletonOut, const StrandModelSkeletonData& skeletonData)
				{
					skeletonOut << YAML::Key << "m_strandGroup" << YAML::Value << YAML::BeginMap;
					{
						StrandGroupSerializer<StrandModelStrandGroupData, StrandModelStrandData, StrandModelStrandSegmentData>::Serialize(skeletonOut, skeletonData.m_strandGroup,
							[&](YAML::Emitter& segmentOut, const StrandModelStrandSegmentData& segmentData)
							{},
							[&](YAML::Emitter& strandOut, const StrandModelStrandData& strandData) {},
							[&](YAML::Emitter& groupOut, const StrandModelStrandGroupData& groupData)
							{
								const auto strandSegmentSize = skeletonData.m_strandGroup.PeekStrandSegments().size();
								auto nodeHandle = std::vector<SkeletonNodeHandle>(strandSegmentSize);
								auto profileParticleHandles = std::vector<ParticleHandle>(strandSegmentSize);
								for (int strandSegmentIndex = 0; strandSegmentIndex < strandSegmentSize; strandSegmentIndex++)
								{
									const auto& strandSegment = skeletonData.m_strandGroup.PeekStrandSegment(strandSegmentIndex);
									nodeHandle.at(strandSegmentIndex) = strandSegment.m_data.m_nodeHandle;
									profileParticleHandles.at(strandSegmentIndex) = strandSegment.m_data.m_profileParticleHandle;
								}
								groupOut << YAML::Key << "m_strandSegments.m_data.m_nodeHandle" << YAML::Value << YAML::Binary(
									reinterpret_cast<const unsigned char*>(nodeHandle.data()), nodeHandle.size() * sizeof(SkeletonNodeHandle));
								groupOut << YAML::Key << "m_strandSegments.m_data.m_profileParticleHandle" << YAML::Value << YAML::Binary(
									reinterpret_cast<const unsigned char*>(profileParticleHandles.data()), profileParticleHandles.size() * sizeof(ParticleHandle));
							}
						);
					}
					skeletonOut << YAML::EndMap;

					const auto nodeSize = m_strandModel.m_strandModelSkeleton.RefRawNodes().size();
					auto offset = std::vector<glm::vec2>(nodeSize);
					auto twistAngle = std::vector<float>(nodeSize);
					auto split = std::vector<int>(nodeSize);
					auto strandRadius = std::vector<float>(nodeSize);
					auto strandCount = std::vector<int>(nodeSize);

					for (int nodeIndex = 0; nodeIndex < nodeSize; nodeIndex++)
					{
						const auto& node = m_strandModel.m_strandModelSkeleton.RefRawNodes().at(nodeIndex);
						offset.at(nodeIndex) = node.m_data.m_offset;
						twistAngle.at(nodeIndex) = node.m_data.m_twistAngle;
						split.at(nodeIndex) = node.m_data.m_split == 1;
						strandRadius.at(nodeIndex) = node.m_data.m_strandRadius;
						strandCount.at(nodeIndex) = node.m_data.m_strandCount;
					}

					skeletonOut << YAML::Key << "m_node.m_data.m_offset" << YAML::Value << YAML::Binary(
						reinterpret_cast<const unsigned char*>(offset.data()), offset.size() * sizeof(glm::vec2));
					skeletonOut << YAML::Key << "m_node.m_data.m_twistAngle" << YAML::Value << YAML::Binary(
						reinterpret_cast<const unsigned char*>(twistAngle.data()), twistAngle.size() * sizeof(float));
					skeletonOut << YAML::Key << "m_node.m_data.m_split" << YAML::Value << YAML::Binary(
						reinterpret_cast<const unsigned char*>(split.data()), split.size() * sizeof(int));
					skeletonOut << YAML::Key << "m_node.m_data.m_strandRadius" << YAML::Value << YAML::Binary(
						reinterpret_cast<const unsigned char*>(strandRadius.data()), strandRadius.size() * sizeof(float));
					skeletonOut << YAML::Key << "m_node.m_data.m_strandCount" << YAML::Value << YAML::Binary(
						reinterpret_cast<const unsigned char*>(strandCount.data()), strandCount.size() * sizeof(float));
				}
			);
		}
		out << YAML::EndMap;
	}
	out << YAML::EndMap;

	out << YAML::Key << "m_treeModel" << YAML::Value << YAML::BeginMap;
	{
		out << YAML::Key << "m_shootSkeleton" << YAML::Value << YAML::BeginMap;
		{
			SkeletonSerializer<ShootGrowthData, ShootStemGrowthData, InternodeGrowthData>::Serialize(out, m_treeModel.RefShootSkeleton(),
				[&](YAML::Emitter& nodeOut, const InternodeGrowthData& nodeData)
				{
					nodeOut << YAML::Key << "m_buds" << YAML::Value << YAML::BeginSeq;
					for (const auto& bud : nodeData.m_buds)
					{
						nodeOut << YAML::BeginMap;
						{
							nodeOut << YAML::Key << "T" << YAML::Value << static_cast<unsigned>(bud.m_type);
							nodeOut << YAML::Key << "S" << YAML::Value << static_cast<unsigned>(bud.m_status);
							nodeOut << YAML::Key << "LR" << YAML::Value << bud.m_localRotation;
							nodeOut << YAML::Key << "RM" << YAML::Value << YAML::BeginMap;
							{
								nodeOut << YAML::Key << "M" << YAML::Value << bud.m_reproductiveModule.m_maturity;
								nodeOut << YAML::Key << "H" << YAML::Value << bud.m_reproductiveModule.m_health;
								nodeOut << YAML::Key << "T" << YAML::Value << bud.m_reproductiveModule.m_transform;
							}
							nodeOut << YAML::EndMap;
						}
						nodeOut << YAML::EndMap;
					}
					nodeOut << YAML::EndSeq;
				},
				[&](YAML::Emitter& flowOut, const ShootStemGrowthData& flowData)
				{
					flowOut << YAML::Key << "m_order" << YAML::Value << flowData.m_order;
				},
				[&](YAML::Emitter& skeletonOut, const ShootGrowthData& skeletonData)
				{
					skeletonOut << YAML::Key << "m_desiredMin" << YAML::Value << skeletonData.m_desiredMin;
					skeletonOut << YAML::Key << "m_desiredMax" << YAML::Value << skeletonData.m_desiredMax;

					const auto nodeSize = m_treeModel.RefShootSkeleton().RefRawNodes().size();
					auto internodeLength = std::vector<float>(nodeSize);
					auto indexOfParentBud = std::vector<int>(nodeSize);
					auto startAge = std::vector<float>(nodeSize);
					auto finishAge = std::vector<float>(nodeSize);
					auto desiredLocalRotation = std::vector<glm::quat>(nodeSize);
					auto desiredGlobalRotation = std::vector<glm::quat>(nodeSize);
					auto desiredGlobalPosition = std::vector<glm::vec3>(nodeSize);
					auto sagging = std::vector<float>(nodeSize);
					auto order = std::vector<int>(nodeSize);
					auto extraMass = std::vector<float>(nodeSize);

					for (int nodeIndex = 0; nodeIndex < nodeSize; nodeIndex++)
					{
						const auto& node = m_treeModel.RefShootSkeleton().RefRawNodes().at(nodeIndex);
						internodeLength.at(nodeIndex) = node.m_data.m_internodeLength;
						indexOfParentBud.at(nodeIndex) = node.m_data.m_indexOfParentBud;
						startAge.at(nodeIndex) = node.m_data.m_startAge;
						finishAge.at(nodeIndex) = node.m_data.m_finishAge;
						desiredLocalRotation.at(nodeIndex) = node.m_data.m_desiredLocalRotation;
						desiredGlobalRotation.at(nodeIndex) = node.m_data.m_desiredGlobalRotation;
						desiredGlobalPosition.at(nodeIndex) = node.m_data.m_desiredGlobalPosition;
						sagging.at(nodeIndex) = node.m_data.m_sagging;
						order.at(nodeIndex) = node.m_data.m_order;
						extraMass.at(nodeIndex) = node.m_data.m_extraMass;

					}

					skeletonOut << YAML::Key << "m_node.m_data.m_internodeLength" << YAML::Value << YAML::Binary(
						reinterpret_cast<const unsigned char*>(internodeLength.data()), internodeLength.size() * sizeof(float));
					skeletonOut << YAML::Key << "m_node.m_data.m_indexOfParentBud" << YAML::Value << YAML::Binary(
						reinterpret_cast<const unsigned char*>(indexOfParentBud.data()), indexOfParentBud.size() * sizeof(int));
					skeletonOut << YAML::Key << "m_node.m_data.m_startAge" << YAML::Value << YAML::Binary(
						reinterpret_cast<const unsigned char*>(startAge.data()), startAge.size() * sizeof(float));
					skeletonOut << YAML::Key << "m_node.m_data.m_finishAge" << YAML::Value << YAML::Binary(
						reinterpret_cast<const unsigned char*>(finishAge.data()), finishAge.size() * sizeof(float));
					skeletonOut << YAML::Key << "m_node.m_data.m_desiredLocalRotation" << YAML::Value << YAML::Binary(
						reinterpret_cast<const unsigned char*>(desiredLocalRotation.data()), desiredLocalRotation.size() * sizeof(glm::quat));
					skeletonOut << YAML::Key << "m_node.m_data.m_desiredGlobalRotation" << YAML::Value << YAML::Binary(
						reinterpret_cast<const unsigned char*>(desiredGlobalRotation.data()), desiredGlobalRotation.size() * sizeof(glm::quat));
					skeletonOut << YAML::Key << "m_node.m_data.m_desiredGlobalPosition" << YAML::Value << YAML::Binary(
						reinterpret_cast<const unsigned char*>(desiredGlobalPosition.data()), desiredGlobalPosition.size() * sizeof(glm::vec3));
					skeletonOut << YAML::Key << "m_node.m_data.m_sagging" << YAML::Value << YAML::Binary(
						reinterpret_cast<const unsigned char*>(sagging.data()), sagging.size() * sizeof(float));
					skeletonOut << YAML::Key << "m_node.m_data.m_order" << YAML::Value << YAML::Binary(
						reinterpret_cast<const unsigned char*>(order.data()), order.size() * sizeof(int));
					skeletonOut << YAML::Key << "m_node.m_data.m_extraMass" << YAML::Value << YAML::Binary(
						reinterpret_cast<const unsigned char*>(extraMass.data()), extraMass.size() * sizeof(float));
				}
			);
		}
		out << YAML::EndMap;
	}
	out << YAML::EndMap;
}



void Tree::Deserialize(const YAML::Node& in)
{
	m_treeDescriptor.Load("m_treeDescriptor", in);
	if (in["m_strandModel"])
	{
		const auto& inStrandModel = in["m_strandModel"];
		if (inStrandModel["m_strandModelSkeleton"])
		{
			const auto& inStrandModelSkeleton = inStrandModel["m_strandModelSkeleton"];
			SkeletonSerializer<StrandModelSkeletonData, StrandModelFlowData, StrandModelNodeData>::Deserialize(inStrandModelSkeleton, m_strandModel.m_strandModelSkeleton,
				[&](const YAML::Node& nodeIn, StrandModelNodeData& nodeData)
				{
					nodeData = {};
					if (nodeIn["m_profile"])
					{
						const auto& inStrandGroup = nodeIn["m_profile"];
						StrandModelProfileSerializer<CellParticlePhysicsData>::Deserialize(inStrandGroup, nodeData.m_profile,
							[&](const YAML::Node& segmentIn, CellParticlePhysicsData& particleData){});
					}
				},
				[&](const YAML::Node& flowIn, StrandModelFlowData& flowData) {},
				[&](const YAML::Node& skeletonIn, StrandModelSkeletonData& skeletonData)
				{
					if (skeletonIn["m_strandGroup"])
					{
						const auto& inStrandGroup = skeletonIn["m_strandGroup"];
						StrandGroupSerializer<StrandModelStrandGroupData, StrandModelStrandData, StrandModelStrandSegmentData>::Deserialize(inStrandGroup, m_strandModel.m_strandModelSkeleton.m_data.m_strandGroup,
							[&](const YAML::Node& segmentIn, StrandModelStrandSegmentData& segmentData)
							{},
							[&](const YAML::Node& strandIn, StrandModelStrandData& strandData) {},
							[&](const YAML::Node& groupIn, StrandModelStrandGroupData& groupData)
							{
								if (groupIn["m_strandSegments.m_data.m_nodeHandle"])
								{
									auto list = std::vector<SkeletonNodeHandle>();
									const auto data = groupIn["m_strandSegments.m_data.m_nodeHandle"].as<YAML::Binary>();
									list.resize(data.size() / sizeof(SkeletonNodeHandle));
									std::memcpy(list.data(), data.data(), data.size());
									for (size_t i = 0; i < list.size(); i++)
									{
										auto& strandSegment = skeletonData.m_strandGroup.RefStrandSegment(i);
										strandSegment.m_data.m_nodeHandle = list[i];
									}
								}

								if (groupIn["m_strandSegments.m_data.m_profileParticleHandle"])
								{
									auto list = std::vector<ParticleHandle>();
									const auto data = groupIn["m_strandSegments.m_data.m_profileParticleHandle"].as<YAML::Binary>();
									list.resize(data.size() / sizeof(ParticleHandle));
									std::memcpy(list.data(), data.data(), data.size());
									for (size_t i = 0; i < list.size(); i++)
									{
										auto& strandSegment = skeletonData.m_strandGroup.RefStrandSegment(i);
										strandSegment.m_data.m_profileParticleHandle = list[i];
									}
								}
							}
						);
					}

					if (skeletonIn["m_node.m_data.m_offset"])
					{
						auto list = std::vector<glm::vec2>();
						const auto data = skeletonIn["m_node.m_data.m_offset"].as<YAML::Binary>();
						list.resize(data.size() / sizeof(glm::vec2));
						std::memcpy(list.data(), data.data(), data.size());
						for (size_t i = 0; i < list.size(); i++)
						{
							auto& node = m_strandModel.m_strandModelSkeleton.RefNode(i);
							node.m_data.m_offset = list[i];
						}
					}

					if (skeletonIn["m_node.m_data.m_twistAngle"])
					{
						auto list = std::vector<float>();
						const auto data = skeletonIn["m_node.m_data.m_twistAngle"].as<YAML::Binary>();
						list.resize(data.size() / sizeof(float));
						std::memcpy(list.data(), data.data(), data.size());
						for (size_t i = 0; i < list.size(); i++)
						{
							auto& node = m_strandModel.m_strandModelSkeleton.RefNode(i);
							node.m_data.m_twistAngle = list[i];
						}
					}

					if (skeletonIn["m_node.m_data.m_packingIteration"])
					{
						auto list = std::vector<int>();
						const auto data = skeletonIn["m_node.m_data.m_packingIteration"].as<YAML::Binary>();
						list.resize(data.size() / sizeof(int));
						std::memcpy(list.data(), data.data(), data.size());
						for (size_t i = 0; i < list.size(); i++)
						{
							auto& node = m_strandModel.m_strandModelSkeleton.RefNode(i);
							node.m_data.m_packingIteration = list[i];
						}
					}

					if (skeletonIn["m_node.m_data.m_split"])
					{
						auto list = std::vector<int>();
						const auto data = skeletonIn["m_node.m_data.m_split"].as<YAML::Binary>();
						list.resize(data.size() / sizeof(int));
						std::memcpy(list.data(), data.data(), data.size());
						for (size_t i = 0; i < list.size(); i++)
						{
							auto& node = m_strandModel.m_strandModelSkeleton.RefNode(i);
							node.m_data.m_split = list[i] == 1;
						}
					}

					if (skeletonIn["m_node.m_data.m_strandRadius"])
					{
						auto list = std::vector<float>();
						const auto data = skeletonIn["m_node.m_data.m_strandRadius"].as<YAML::Binary>();
						list.resize(data.size() / sizeof(float));
						std::memcpy(list.data(), data.data(), data.size());
						for (size_t i = 0; i < list.size(); i++)
						{
							auto& node = m_strandModel.m_strandModelSkeleton.RefNode(i);
							node.m_data.m_strandRadius = list[i];
						}
					}

					if (skeletonIn["m_node.m_data.m_strandCount"])
					{
						auto list = std::vector<int>();
						const auto data = skeletonIn["m_node.m_data.m_strandCount"].as<YAML::Binary>();
						list.resize(data.size() / sizeof(int));
						std::memcpy(list.data(), data.data(), data.size());
						for (size_t i = 0; i < list.size(); i++)
						{
							auto& node = m_strandModel.m_strandModelSkeleton.RefNode(i);
							node.m_data.m_strandCount = list[i];
						}
					}
				}
			);
		}
	}
	if (in["m_treeModel"])
	{
		const auto& inTreeModel = in["m_treeModel"];
		if (inTreeModel["m_shootSkeleton"])
		{
			const auto& inShootSkeleton = inTreeModel["m_shootSkeleton"];
			SkeletonSerializer<ShootGrowthData, ShootStemGrowthData, InternodeGrowthData>::Deserialize(inShootSkeleton, m_treeModel.RefShootSkeleton(),
				[&](const YAML::Node& nodeIn, InternodeGrowthData& nodeData)
				{
					nodeData.m_buds.clear();
					if (nodeIn["m_buds"])
					{
						const auto& inBuds = nodeIn["m_buds"];
						for (const auto& inBud : inBuds)
						{
							nodeData.m_buds.emplace_back();
							auto& bud = nodeData.m_buds.back();
							if (inBud["T"]) bud.m_type = static_cast<BudType>(inBud["T"].as<unsigned>());
							if (inBud["S"]) bud.m_status = static_cast<BudStatus>(inBud["S"].as<unsigned>());
							if (inBud["LR"]) bud.m_localRotation = inBud["LR"].as<glm::quat>();
							if (inBud["RM"])
							{
								const auto& inReproductiveModule = inBud["RM"];
								if (inReproductiveModule["M"]) bud.m_reproductiveModule.m_maturity = inReproductiveModule["M"].as<float>();
								if (inReproductiveModule["H"]) bud.m_reproductiveModule.m_health = inReproductiveModule["H"].as<float>();
								if (inReproductiveModule["T"]) bud.m_reproductiveModule.m_transform = inReproductiveModule["T"].as<glm::mat4>();
							}
						}
					}
				},
				[&](const YAML::Node& flowIn, ShootStemGrowthData& flowData)
				{
					if (flowIn["m_data"]) flowData.m_order = flowIn["m_data"].as<int>();
				},
				[&](const YAML::Node& skeletonIn, ShootGrowthData& skeletonData)
				{
					if (skeletonIn["m_desiredMin"]) skeletonData.m_desiredMin = skeletonIn["m_desiredMin"].as<glm::vec3>();
					if (skeletonIn["m_desiredMax"]) skeletonData.m_desiredMax = skeletonIn["m_desiredMax"].as<glm::vec3>();

					if (skeletonIn["m_node.m_data.m_internodeLength"])
					{
						auto list = std::vector<float>();
						const auto data = skeletonIn["m_node.m_data.m_internodeLength"].as<YAML::Binary>();
						list.resize(data.size() / sizeof(float));
						std::memcpy(list.data(), data.data(), data.size());
						for (size_t i = 0; i < list.size(); i++)
						{
							auto& node = m_treeModel.RefShootSkeleton().RefNode(i);
							node.m_data.m_internodeLength = list[i];
						}
					}

					if (skeletonIn["m_node.m_data.m_indexOfParentBud"])
					{
						auto list = std::vector<int>();
						const auto data = skeletonIn["m_node.m_data.m_indexOfParentBud"].as<YAML::Binary>();
						list.resize(data.size() / sizeof(int));
						std::memcpy(list.data(), data.data(), data.size());
						for (size_t i = 0; i < list.size(); i++)
						{
							auto& node = m_treeModel.RefShootSkeleton().RefNode(i);
							node.m_data.m_indexOfParentBud = list[i];
						}
					}

					if (skeletonIn["m_node.m_data.m_startAge"])
					{
						auto list = std::vector<float>();
						const auto data = skeletonIn["m_node.m_data.m_startAge"].as<YAML::Binary>();
						list.resize(data.size() / sizeof(float));
						std::memcpy(list.data(), data.data(), data.size());
						for (size_t i = 0; i < list.size(); i++)
						{
							auto& node = m_treeModel.RefShootSkeleton().RefNode(i);
							node.m_data.m_startAge = list[i];
						}
					}

					if (skeletonIn["m_node.m_data.m_finishAge"])
					{
						auto list = std::vector<float>();
						const auto data = skeletonIn["m_node.m_data.m_finishAge"].as<YAML::Binary>();
						list.resize(data.size() / sizeof(float));
						std::memcpy(list.data(), data.data(), data.size());
						for (size_t i = 0; i < list.size(); i++)
						{
							auto& node = m_treeModel.RefShootSkeleton().RefNode(i);
							node.m_data.m_finishAge = list[i];
						}
					}

					if (skeletonIn["m_node.m_data.m_desiredLocalRotation"])
					{
						auto list = std::vector<glm::quat>();
						const auto data = skeletonIn["m_node.m_data.m_desiredLocalRotation"].as<YAML::Binary>();
						list.resize(data.size() / sizeof(glm::quat));
						std::memcpy(list.data(), data.data(), data.size());
						for (size_t i = 0; i < list.size(); i++)
						{
							auto& node = m_treeModel.RefShootSkeleton().RefNode(i);
							node.m_data.m_desiredLocalRotation = list[i];
						}
					}

					if (skeletonIn["m_node.m_data.m_desiredGlobalRotation"])
					{
						auto list = std::vector<glm::quat>();
						const auto data = skeletonIn["m_node.m_data.m_desiredGlobalRotation"].as<YAML::Binary>();
						list.resize(data.size() / sizeof(glm::quat));
						std::memcpy(list.data(), data.data(), data.size());
						for (size_t i = 0; i < list.size(); i++)
						{
							auto& node = m_treeModel.RefShootSkeleton().RefNode(i);
							node.m_data.m_desiredGlobalRotation = list[i];
						}
					}

					if (skeletonIn["m_node.m_data.m_desiredGlobalPosition"])
					{
						auto list = std::vector<glm::vec3>();
						const auto data = skeletonIn["m_node.m_data.m_desiredGlobalPosition"].as<YAML::Binary>();
						list.resize(data.size() / sizeof(glm::vec3));
						std::memcpy(list.data(), data.data(), data.size());
						for (size_t i = 0; i < list.size(); i++)
						{
							auto& node = m_treeModel.RefShootSkeleton().RefNode(i);
							node.m_data.m_desiredGlobalPosition = list[i];
						}
					}

					if (skeletonIn["m_node.m_data.m_sagging"])
					{
						auto list = std::vector<float>();
						const auto data = skeletonIn["m_node.m_data.m_sagging"].as<YAML::Binary>();
						list.resize(data.size() / sizeof(float));
						std::memcpy(list.data(), data.data(), data.size());
						for (size_t i = 0; i < list.size(); i++)
						{
							auto& node = m_treeModel.RefShootSkeleton().RefNode(i);
							node.m_data.m_sagging = list[i];
						}
					}

					if (skeletonIn["m_node.m_data.m_order"])
					{
						auto list = std::vector<int>();
						const auto data = skeletonIn["m_node.m_data.m_order"].as<YAML::Binary>();
						list.resize(data.size() / sizeof(int));
						std::memcpy(list.data(), data.data(), data.size());
						for (size_t i = 0; i < list.size(); i++)
						{
							auto& node = m_treeModel.RefShootSkeleton().RefNode(i);
							node.m_data.m_order = list[i];
						}
					}

					if (skeletonIn["m_node.m_data.m_extraMass"])
					{
						auto list = std::vector<float>();
						const auto data = skeletonIn["m_node.m_data.m_extraMass"].as<YAML::Binary>();
						list.resize(data.size() / sizeof(float));
						std::memcpy(list.data(), data.data(), data.size());
						for (size_t i = 0; i < list.size(); i++)
						{
							auto& node = m_treeModel.RefShootSkeleton().RefNode(i);
							node.m_data.m_extraMass = list[i];
						}
					}
				}
			);
			m_treeModel.m_initialized = true;
		}
	}
}

void Tree::ClearGeometryEntities() const
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

void Tree::GenerateGeometryEntities(const TreeMeshGeneratorSettings& meshGeneratorSettings, int iteration) {
	const auto scene = GetScene();
	const auto self = GetOwner();
	const auto children = scene->GetChildren(self);
	auto treeDescriptor = m_treeDescriptor.Get<TreeDescriptor>();
	ClearGeometryEntities();
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
		bool copiedMaterial = false;
		if (treeDescriptor) {
			if (const auto shootDescriptor = treeDescriptor->m_shootDescriptor.Get<ShootDescriptor>()) {
				if (const auto shootMaterial = shootDescriptor->m_barkMaterial.Get<Material>()) {
					material->SetAlbedoTexture(shootMaterial->GetAlbedoTexture());
					material->SetNormalTexture(shootMaterial->GetNormalTexture());
					material->SetRoughnessTexture(shootMaterial->GetRoughnessTexture());
					material->SetMetallicTexture(shootMaterial->GetMetallicTexture());
					material->m_materialProperties = shootMaterial->m_materialProperties;
				}
			}
		}
		if (!copiedMaterial)
		{
			material->m_materialProperties.m_albedoColor = glm::vec3(109, 79, 75) / 255.0f;
			material->m_materialProperties.m_roughness = 1.0f;
			material->m_materialProperties.m_metallic = 0.0f;
		}
		meshRenderer->m_mesh = mesh;
		meshRenderer->m_material = material;
	}

	if (meshGeneratorSettings.m_enableTwig)
	{
		const auto twigEntity = scene->CreateEntity("Twig Strands");
		scene->SetParent(twigEntity, self);
		const auto strands = GenerateTwigStrands(meshGeneratorSettings);
		const auto material = ProjectManager::CreateTemporaryAsset<Material>();
		bool copiedMaterial = false;
		if (treeDescriptor) {
			if (const auto foliageDescriptor = treeDescriptor->m_foliageDescriptor.Get<FoliageDescriptor>()) {
				if (const auto twigMaterial = foliageDescriptor->m_twigMaterial.Get<Material>()) {
					material->SetAlbedoTexture(twigMaterial->GetAlbedoTexture());
					material->SetNormalTexture(twigMaterial->GetNormalTexture());
					material->SetRoughnessTexture(twigMaterial->GetRoughnessTexture());
					material->SetMetallicTexture(twigMaterial->GetMetallicTexture());
					material->m_materialProperties = twigMaterial->m_materialProperties;
					copiedMaterial = true;
				}
			}
		}
		if (!copiedMaterial) {
			material->m_materialProperties.m_albedoColor = glm::vec3(109, 79, 75) / 255.0f;
			material->m_materialProperties.m_roughness = 1.0f;
			material->m_materialProperties.m_metallic = 0.0f;
		}

		const auto strandsRenderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(twigEntity).lock();
		strandsRenderer->m_strands = strands;
		strandsRenderer->m_material = material;
	}
	if (meshGeneratorSettings.m_enableFoliage)
	{
		const auto foliageEntity = scene->CreateEntity("Foliage Mesh");
		scene->SetParent(foliageEntity, self);

		const auto mesh = Resources::GetResource<Mesh>("PRIMITIVE_QUAD");
		const auto particleInfoList = GenerateFoliageParticleInfoList(meshGeneratorSettings);
		const auto material = ProjectManager::CreateTemporaryAsset<Material>();
		bool copiedMaterial = false;
		if (treeDescriptor) {
			if (const auto foliageDescriptor = treeDescriptor->m_foliageDescriptor.Get<FoliageDescriptor>()) {
				if (const auto leafMaterial = foliageDescriptor->m_leafMaterial.Get<Material>()) {
					material->SetAlbedoTexture(leafMaterial->GetAlbedoTexture());
					material->SetNormalTexture(leafMaterial->GetNormalTexture());
					material->SetRoughnessTexture(leafMaterial->GetRoughnessTexture());
					material->SetMetallicTexture(leafMaterial->GetMetallicTexture());
					material->m_materialProperties = leafMaterial->m_materialProperties;
					copiedMaterial = true;
				}
			}
		}
		if (!copiedMaterial) {
			material->m_materialProperties.m_albedoColor = glm::vec3(152 / 255.0f, 203 / 255.0f, 0 / 255.0f);
			material->m_materialProperties.m_roughness = 1.0f;
			material->m_materialProperties.m_metallic = 0.0f;
		}
		const auto particles = scene->GetOrSetPrivateComponent<Particles>(foliageEntity).lock();
		particles->m_mesh = mesh;
		particles->m_material = material;
		particles->m_particleInfoList = particleInfoList;
	}
	if (meshGeneratorSettings.m_enableFruit)
	{
		const auto fruitEntity = scene->CreateEntity("Fruit Mesh");
		scene->SetParent(fruitEntity, self);
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
	std::vector<SkeletonNodeHandle> m_nodeHandles;
	std::vector<bool> m_isEnd;
	std::vector<int> m_lineIndex;
};

void Tree::ExportJunction(const TreeMeshGeneratorSettings& meshGeneratorSettings, YAML::Emitter& out)
{
	out << YAML::Key << "Tree" << YAML::Value << YAML::BeginMap; {
		const auto& skeleton = m_treeModel.RefShootSkeleton();
		const auto& sortedInternodeList = skeleton.PeekSortedNodeList();
		std::vector<TreePart> treeParts{};
		std::unordered_map<SkeletonNodeHandle, TreePartInfo> treePartInfos{};
		int nextLineIndex = 0;
		for (int internodeHandle : sortedInternodeList)
		{
			const auto& internode = skeleton.PeekNode(internodeHandle);
			const auto& internodeInfo = internode.m_info;
			auto parentInternodeHandle = internode.GetParentHandle();
			const auto flowHandle = internode.GetFlowHandle();
			const auto& flow = skeleton.PeekFlow(flowHandle);
			const auto& chainHandles = flow.PeekNodeHandles();
			const bool hasMultipleChildren = flow.PeekChildHandles().size() > 1;
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
				onlyChild = parentFlow.PeekChildHandles().size() <= 1;
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
				const auto& chainHandles = flow.PeekNodeHandles();
				const auto centerInternodeHandle = chainHandles.back();
				const auto& centerInternode = skeleton.PeekNode(centerInternodeHandle);
				treePart.m_baseLine.m_startPosition = startInternode.m_info.m_globalPosition;
				treePart.m_baseLine.m_startRadius = startInternode.m_info.m_thickness;
				treePart.m_baseLine.m_endPosition = centerInternode.m_info.GetGlobalEndPosition();
				treePart.m_baseLine.m_endRadius = centerInternode.m_info.m_thickness;

				treePart.m_baseLine.m_startDirection = startInternode.m_info.GetGlobalDirection();
				treePart.m_baseLine.m_endDirection = centerInternode.m_info.GetGlobalDirection();

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

						newLine.m_startDirection = centerInternode.m_info.GetGlobalDirection();
						newLine.m_endDirection = endInternode.m_info.GetGlobalDirection();

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

				treePart.m_baseLine.m_startDirection = startInternode.m_info.GetGlobalDirection();
				treePart.m_baseLine.m_endDirection = endInternode.m_info.GetGlobalDirection();

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
	}
	out << YAML::EndMap;
}



void Tree::ExportJunction(const TreeMeshGeneratorSettings& meshGeneratorSettings, const std::filesystem::path& path)
{
	try {
		auto directory = path;
		directory.remove_filename();
		std::filesystem::create_directories(directory);
		YAML::Emitter out;
		ExportJunction(meshGeneratorSettings, out);
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
	using namespace treeio;
	const auto& shootSkeleton = m_treeModel.PeekShootSkeleton();
	const auto& sortedInternodeList = shootSkeleton.PeekSortedNodeList();
	if (sortedInternodeList.empty()) return false;
	const auto& rootNode = shootSkeleton.PeekNode(0);
	TreeNodeData rootNodeData;
	//rootNodeData.direction = rootNode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
	rootNodeData.thickness = rootNode.m_info.m_thickness;
	rootNodeData.pos = rootNode.m_info.m_globalPosition;

	auto rootId = tree.addRoot(rootNodeData);
	std::unordered_map<SkeletonNodeHandle, size_t> nodeMap;
	nodeMap[0] = rootId;
	for (const auto& nodeHandle : sortedInternodeList)
	{
		if (nodeHandle == 0) continue;
		const auto& node = shootSkeleton.PeekNode(nodeHandle);
		TreeNodeData nodeData;
		//nodeData.direction = node.m_info.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
		nodeData.thickness = node.m_info.m_thickness;
		nodeData.pos = node.m_info.m_globalPosition;

		auto currentId = tree.addNodeChild(nodeMap[node.GetParentHandle()], nodeData);
		nodeMap[nodeHandle] = currentId;
	}
	return tree.saveTree(path.string());
}

void Tree::ExportRadialBoundingVolume(const std::shared_ptr<RadialBoundingVolume>& rbv) const
{
	const auto& sortedInternodeList = m_treeModel.m_shootSkeleton.PeekSortedNodeList();
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

void Tree::CollectAssetRef(std::vector<AssetRef>& list)
{
	if (m_treeDescriptor.Get<TreeDescriptor>())
	{
		list.emplace_back(m_treeDescriptor);
	}
}


void SkeletalGraphSettings::OnInspect()
{
	ImGui::DragFloat("Line thickness", &m_lineThickness, 0.001f, 0.0f, 1.0f);
	ImGui::DragFloat("Fixed line thickness", &m_fixedLineThickness, 0.001f, 0.0f, 1.0f);
	ImGui::DragFloat("Branch point size", &m_branchPointSize, 0.01f, 0.0f, 1.0f);
	ImGui::DragFloat("Junction point size", &m_junctionPointSize, 0.01f, 0.0f, 1.0f);

	ImGui::Checkbox("Fixed point size", &m_fixedPointSize);
	if (m_fixedPointSize) {
		ImGui::DragFloat("Fixed point size multiplier", &m_fixedPointSizeFactor, 0.001f, 0.0f, 1.0f);
	}

	ImGui::ColorEdit4("Line color", &m_lineColor.x);
	ImGui::ColorEdit4("Branch point color", &m_branchPointColor.x);
	ImGui::ColorEdit4("Junction point color", &m_junctionPointColor.x);
}

void Tree::PrepareController(const std::shared_ptr<ShootDescriptor>& shootDescriptor, const std::shared_ptr<Soil>& soil, const std::shared_ptr<Climate>& climate)
{
	shootDescriptor->PrepareController(m_shootGrowthController);
	m_shootGrowthController.m_leafDamage = [=](const SkeletonNode<InternodeGrowthData>& internode)
		{
			const auto& internodeData = internode.m_data;
			float leafDamage = 0.0f;
			if (climate->m_climateModel.m_time - glm::floor(climate->m_climateModel.m_time) > 0.5f && internodeData.m_temperature < shootDescriptor->m_leafChlorophyllSynthesisFactorTemperature)
			{
				leafDamage += shootDescriptor->m_leafChlorophyllLoss;
			}
			return leafDamage;
		};
	m_shootGrowthController.m_pruningFactor = [&](const ShootSkeleton& shootSkeleton, const SkeletonNode<InternodeGrowthData>& internode)
		{
			if (shootDescriptor->m_trunkProtection && internode.m_data.m_order == 0)
			{
				return 0.f;
			}
			float pruningProbability = 0.0f;
			if (shootDescriptor->m_maxFlowLength != 0 && shootDescriptor->m_maxFlowLength < internode.m_info.m_chainIndex)
			{
				pruningProbability += 999.f;
			}
			if (internode.IsEndNode()) {
				if (internode.m_data.m_lightIntensity <= shootDescriptor->m_lightPruningFactor)
				{
					pruningProbability += shootDescriptor->m_lightPruningProbability;
				}
			}
			if (internode.m_data.m_level != 0 && shootDescriptor->m_thicknessPruningFactor != 0.0f
				&& internode.m_info.m_thickness / internode.m_info.m_endDistance < shootDescriptor->m_thicknessPruningFactor)
			{
				pruningProbability += shootDescriptor->m_thicknessPruningProbability;
			}
			return pruningProbability;
		};
}

void Tree::InitializeStrandRenderer()
{
	const auto scene = GetScene();
	const auto owner = GetOwner();

	ClearStrandRenderer();

	BuildStrandModel();

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

void Tree::InitializeStrandModelMeshRenderer(const StrandModelMeshGeneratorSettings& strandModelMeshGeneratorSettings)
{
	ClearStrandModelMeshRenderer();

	BuildStrandModel();

	const float time = Times::Now();
	const auto scene = GetScene();
	const auto self = GetOwner();
	const auto treeDescriptor = m_treeDescriptor.Get<TreeDescriptor>();
	if (strandModelMeshGeneratorSettings.m_enableBranch)
	{
		const auto foliageEntity = scene->CreateEntity("Strand Model Branch Mesh");
		scene->SetParent(foliageEntity, self);

		const auto mesh = GenerateStrandModelBranchMesh(strandModelMeshGeneratorSettings);
		const auto material = ProjectManager::CreateTemporaryAsset<Material>();
		const auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(foliageEntity).lock();

		bool copiedMaterial = false;
		if (treeDescriptor) {
			if (const auto shootDescriptor = treeDescriptor->m_shootDescriptor.Get<ShootDescriptor>()) {
				if (const auto shootMaterial = shootDescriptor->m_barkMaterial.Get<Material>()) {
					material->SetAlbedoTexture(shootMaterial->GetAlbedoTexture());
					material->SetNormalTexture(shootMaterial->GetNormalTexture());
					material->SetRoughnessTexture(shootMaterial->GetRoughnessTexture());
					material->SetMetallicTexture(shootMaterial->GetMetallicTexture());
					material->m_materialProperties = shootMaterial->m_materialProperties;
				}
			}
		}
		if (!copiedMaterial)
		{
			material->m_materialProperties.m_albedoColor = glm::vec3(109, 79, 75) / 255.0f;
			material->m_materialProperties.m_roughness = 1.0f;
			material->m_materialProperties.m_metallic = 0.0f;
		}

		meshRenderer->m_mesh = mesh;
		meshRenderer->m_material = material;
	}
	if (strandModelMeshGeneratorSettings.m_enableFoliage)
	{
		const Entity foliageEntity = scene->CreateEntity("Strand Model Foliage Mesh");
		scene->SetParent(foliageEntity, self);

		const auto mesh = GenerateStrandModelFoliageMesh(strandModelMeshGeneratorSettings);
		const auto material = ProjectManager::CreateTemporaryAsset<Material>();
		bool copiedMaterial = false;
		if (treeDescriptor) {
			if (const auto foliageDescriptor = treeDescriptor->m_foliageDescriptor.Get<FoliageDescriptor>()) {
				if (const auto leafMaterial = foliageDescriptor->m_leafMaterial.Get<Material>()) {
					material->SetAlbedoTexture(leafMaterial->GetAlbedoTexture());
					material->SetNormalTexture(leafMaterial->GetNormalTexture());
					material->SetRoughnessTexture(leafMaterial->GetRoughnessTexture());
					material->SetMetallicTexture(leafMaterial->GetMetallicTexture());
					material->m_materialProperties = leafMaterial->m_materialProperties;
					copiedMaterial = true;
				}
			}
		}
		if (!copiedMaterial) {
			material->m_materialProperties.m_albedoColor = glm::vec3(152 / 255.0f, 203 / 255.0f, 0 / 255.0f);
			material->m_materialProperties.m_roughness = 1.0f;
			material->m_materialProperties.m_metallic = 0.0f;
		}
		const auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(foliageEntity).lock();
		meshRenderer->m_mesh = mesh;
		meshRenderer->m_material = material;

	}
	std::string output;
	const float meshGenerationTime = Times::Now() - time;
	output += "\nMesh generation Used time: " + std::to_string(meshGenerationTime) + "\n";
	EVOENGINE_LOG(output);
}

void Tree::ClearStrandModelMeshRenderer() const
{
	const auto scene = GetScene();
	const auto self = GetOwner();
	const auto children = scene->GetChildren(self);
	for (const auto& child : children) {
		auto name = scene->GetEntityName(child);
		if (name == "Strand Model Branch Mesh") {
			scene->DeleteEntity(child);
		}
		else if (name == "Strand Model Foliage Mesh") {
			scene->DeleteEntity(child);
		}
	}
}
