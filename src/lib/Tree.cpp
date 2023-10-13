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
	std::cout << "reading binvox version " << version << std::endl;

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
			std::cout << "  unrecognized keyword [" << line << "], skipping" << std::endl;
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
	byte value;
	byte count;
	int index = 0;
	int end_index = 0;
	int nr_voxels = 0;

	input.unsetf(std::ios::skipws);  // need to read every byte now (!)
	input >> value;  // read the linefeed char

	while ((end_index < (width * height * depth)) && input.good()) {
		input >> value >> count;

		if (input.good()) {
			end_index = index + count;
			if (end_index > (width * height * depth)) return false;

			for (int i = index; i < end_index; i++) {
				// Convert 1D index to 3D coordinates
				int current_width = i % width;
				int current_height = (i / width) % height;
				int current_depth = i / (width * height);

				if (value) {
					voxelGrid.Ref(glm::ivec3(current_width, current_height, current_depth)).m_occupied = true;
					nr_voxels++;
				}
			}

			index = end_index;
		}
	}

	input.close();
	std::cout << "  read " << nr_voxels << " voxels" << std::endl;
	return true;
}

void Tree::Reset()
{
	m_treeModel.Clear();
	if (const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>()) m_treeModel.m_treeIlluminationEstimator.m_settings = ecoSysLabLayer->m_shadowEstimationSettings;
	m_treeVisualizer.Reset(m_treeModel);
}
void Tree::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
	bool modelChanged = false;
	const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
	if (editorLayer->DragAndDropButton<TreeDescriptor>(m_treeDescriptor, "TreeDescriptor", true)) {
		m_treeModel.Clear();
		modelChanged = true;
	}
	static bool showSpaceColonizationGrid = false;
	static bool showShadowGrid = false;
	static std::shared_ptr<ParticleInfoList> spaceColonizationGridParticleInfoList;
	if (!spaceColonizationGridParticleInfoList)
	{
		spaceColonizationGridParticleInfoList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
	}
	static std::shared_ptr<ParticleInfoList> shadowGridParticleInfoList;
	if (!shadowGridParticleInfoList)
	{
		shadowGridParticleInfoList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
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
			ImGui::Checkbox("Enable Root", &m_treeModel.m_treeGrowthSettings.m_enableRoot);
			ImGui::Checkbox("Enable Shoot", &m_treeModel.m_treeGrowthSettings.m_enableShoot);
			ImGui::Checkbox("Auto balance vigor", &m_treeModel.m_treeGrowthSettings.m_autoBalance);
			ImGui::Checkbox("Receive light", &m_treeModel.m_treeGrowthSettings.m_collectShootFlux);
			ImGui::Checkbox("Receive water", &m_treeModel.m_treeGrowthSettings.m_collectRootFlux);
			ImGui::Checkbox("Receive nitrite", &m_treeModel.m_treeGrowthSettings.m_collectNitrite);
			ImGui::Checkbox("Enable Branch collision detection", &m_treeModel.m_treeGrowthSettings.m_enableBranchCollisionDetection);
			ImGui::Checkbox("Enable Root collision detection", &m_treeModel.m_treeGrowthSettings.m_enableRootCollisionDetection);
			
			if (!m_treeModel.m_treeGrowthSettings.m_collectShootFlux && !m_treeModel.m_treeGrowthSettings.m_collectRootFlux)
			{
				if (ImGui::TreeNode("Vigor filling rates"))
				{
					if (ImGui::SliderFloat("Leaf maintenance", &m_treeModel.m_treeGrowthSettings.m_leafMaintenanceVigorFillingRate, 0.0f, 1.0f))
					{
						if (m_treeModel.m_treeGrowthSettings.m_leafMaintenanceVigorFillingRate != 1.0f)
						{
							m_treeModel.m_treeGrowthSettings.m_leafDevelopmentalVigorFillingRate
								= m_treeModel.m_treeGrowthSettings.m_fruitMaintenanceVigorFillingRate
								= m_treeModel.m_treeGrowthSettings.m_fruitDevelopmentalVigorFillingRate
								= m_treeModel.m_treeGrowthSettings.m_nodeDevelopmentalVigorFillingRate = 0.0f;
						}
					}
					if (m_treeModel.m_treeGrowthSettings.m_leafMaintenanceVigorFillingRate == 1.0f)
					{
						if (ImGui::SliderFloat("Leaf development", &m_treeModel.m_treeGrowthSettings.m_leafDevelopmentalVigorFillingRate, 0.0f, 1.0f))
						{
							if (m_treeModel.m_treeGrowthSettings.m_leafMaintenanceVigorFillingRate != 1.0f)
							{
								m_treeModel.m_treeGrowthSettings.m_fruitMaintenanceVigorFillingRate
									= m_treeModel.m_treeGrowthSettings.m_fruitDevelopmentalVigorFillingRate
									= m_treeModel.m_treeGrowthSettings.m_nodeDevelopmentalVigorFillingRate = 0.0f;
							}
						}
					}

					if (m_treeModel.m_treeGrowthSettings.m_leafDevelopmentalVigorFillingRate == 1.0f)
					{
						if (ImGui::SliderFloat("Fruit maintenance", &m_treeModel.m_treeGrowthSettings.m_fruitMaintenanceVigorFillingRate, 0.0f, 1.0f))
						{
							if (m_treeModel.m_treeGrowthSettings.m_leafMaintenanceVigorFillingRate != 1.0f)
							{
								m_treeModel.m_treeGrowthSettings.m_fruitDevelopmentalVigorFillingRate
									= m_treeModel.m_treeGrowthSettings.m_nodeDevelopmentalVigorFillingRate = 0.0f;
							}
						}
					}

					if (m_treeModel.m_treeGrowthSettings.m_fruitMaintenanceVigorFillingRate == 1.0f)
					{
						if (ImGui::SliderFloat("Fruit development", &m_treeModel.m_treeGrowthSettings.m_fruitDevelopmentalVigorFillingRate, 0.0f, 1.0f))
						{
							if (m_treeModel.m_treeGrowthSettings.m_leafMaintenanceVigorFillingRate != 1.0f)
							{
								m_treeModel.m_treeGrowthSettings.m_nodeDevelopmentalVigorFillingRate = 0.0f;
							}
						}
					}

					if (m_treeModel.m_treeGrowthSettings.m_fruitDevelopmentalVigorFillingRate == 1.0f)
					{
						ImGui::SliderFloat("Node development", &m_treeModel.m_treeGrowthSettings.m_nodeDevelopmentalVigorFillingRate, 0.0f, 1.0f);
					}

					if (ImGui::Button("Reset"))
					{
						m_treeModel.m_treeGrowthSettings.m_leafMaintenanceVigorFillingRate
							= m_treeModel.m_treeGrowthSettings.m_leafDevelopmentalVigorFillingRate
							= m_treeModel.m_treeGrowthSettings.m_fruitMaintenanceVigorFillingRate
							= m_treeModel.m_treeGrowthSettings.m_fruitDevelopmentalVigorFillingRate
							= m_treeModel.m_treeGrowthSettings.m_nodeDevelopmentalVigorFillingRate = 1.0f;
					}

					ImGui::TreePop();
				}
			}
			ImGui::Checkbox("Enable space colonization", &m_treeModel.m_treeGrowthSettings.m_useSpaceColonization);
			if(m_treeModel.m_treeGrowthSettings.m_useSpaceColonization)
			{
				ImGui::Checkbox("Space colonization auto resize", &m_treeModel.m_treeGrowthSettings.m_spaceColonizationAutoResize);
				if(!m_treeModel.m_treeGrowthSettings.m_spaceColonizationAutoResize)
				{
					static float radius = 2.0f;
					ImGui::DragFloat("Import radius", &radius, 0.01f, 0.01f, 10.0f);
					FileUtils::OpenFile("Load Voxel Data", "Binvox", { ".binvox" }, [&](const std::filesystem::path& path) {
						auto& occupancyGrid = m_treeModel.m_treeOccupancyGrid;
						VoxelGrid<TreeOccupancyGridBasicData> inputGrid {};
						if(ParseBinvox(path, inputGrid, 1.f))
						{
							const auto treeDescriptor = m_treeDescriptor.Get<TreeDescriptor>();
							occupancyGrid.Initialize(inputGrid, 
								glm::vec3(-radius, 0, -radius), 
								glm::vec3(radius, 2.0f * radius, radius),
								treeDescriptor->m_shootGrowthParameters.m_internodeLength, 
								m_treeModel.m_treeGrowthSettings.m_spaceColonizationRemovalDistanceFactor, 
								m_treeModel.m_treeGrowthSettings.m_spaceColonizationTheta,
								m_treeModel.m_treeGrowthSettings.m_spaceColonizationDetectionDistanceFactor);
						}

						}, false);
				}
			}
			ImGui::TreePop();
		}
		if (ImGui::TreeNode("Pipe settings")) {

			ImGui::DragFloat("Default profile cell radius", &m_pipeModelParameters.m_profileDefaultCellRadius, 0.001f, 0.001f, 1.0f);
			ImGui::DragFloat("Physics damping", &m_pipeModelParameters.m_damping, 0.01f, 0.0f, 1.0f);
			ImGui::DragFloat("Physics attraction strength", &m_pipeModelParameters.m_gravityStrength, 0.01f, 0.0f, 10.0f);
			ImGui::DragFloat("Physics simulation iteration chain factor", &m_pipeModelParameters.m_simulationIterationChainFactor, 0.1f, 0.0f, 50.0f);
			ImGui::DragFloat("Physics simulation iteration cell factor", &m_pipeModelParameters.m_simulationIterationCellFactor, 0.1f, 0.0f, 50.0f);
			if (ImGui::Button("Update pipes"))
			{
				m_treePipeModel.UpdatePipeModels(m_treeModel, m_pipeModelParameters);
			}
			
			if (ImGui::Button("Initialize strands"))
			{
				m_treePipeModel.ApplySimulationResults(m_pipeModelParameters);
				InitializeStrandRenderer();
			}
			static bool displayProfile = true;

			static bool showGrid = false;
			ImGui::Checkbox("Show Profile", &displayProfile);
			if (displayProfile) ImGui::Checkbox("Show Grid", &showGrid);
			if (displayProfile && m_treeVisualizer.GetSelectedInternodeHandle() >= 0)
			{
				auto& skeleton = m_treePipeModel.m_shootPipeModel.m_skeleton;
				if (skeleton.RefRawNodes().size() > m_treeVisualizer.GetSelectedInternodeHandle())
				{
					auto& profileGroup = m_treePipeModel.m_shootPipeModel.m_pipeProfileGroup;
					const auto targetProfileHandle = skeleton.RefNode(skeleton.m_data.m_nodeMap.at(m_treeVisualizer.GetSelectedInternodeHandle())).m_data.m_profileHandle;
					if (profileGroup.RefProfiles().size() > targetProfileHandle) {
						const std::string tag = "Profile [" + std::to_string(m_treeVisualizer.GetSelectedInternodeHandle()) + "]";
						if (ImGui::Begin(tag.c_str()))
						{
							profileGroup.RefProfile(targetProfileHandle).m_data.m_particlePhysics2D.OnInspect(
								[&](const glm::vec2 position) {},
								[&](const ImVec2 origin, const float zoomFactor, ImDrawList* drawList) {},
								showGrid);
						}
						ImGui::End();
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
				GenerateMeshes(m_meshGeneratorSettings, iterations);
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
				for(const auto& voxel : voxelGrid.RefData())
				{
					for (const auto& marker : voxel.m_markers) {
						scalarMatrices.resize(i + 1);
						scalarMatrices[i].m_instanceMatrix.m_value =
							glm::translate(marker.m_position)
							* glm::mat4_cast(glm::quat(glm::vec3(0.0f)))
							* glm::scale(glm::vec3(voxelGrid.GetVoxelSize() * 0.2f));
						if(marker.m_nodeHandle == -1) scalarMatrices[i].m_instanceColor = glm::vec4(1.0f, 1.0f, 1.0f, 0.75f);
						else
						{
							scalarMatrices[i].m_instanceColor = glm::vec4(ecoSysLabLayer->RandomColors()[marker.m_nodeHandle], 1.0f);
						}
						i++;
					}
				}
				
				spaceColonizationGridParticleInfoList->SetPendingUpdate();
			}
			ImGui::Checkbox("Show Shadow Grid", &showShadowGrid);
			if (showShadowGrid && needGridUpdate) {
				const auto& voxelGrid = m_treeModel.m_treeIlluminationEstimator.m_voxel;
				const auto numVoxels = voxelGrid.GetVoxelCount();
				auto& scalarMatrices = shadowGridParticleInfoList->m_particleInfos;
				if (scalarMatrices.size() != numVoxels) {
					scalarMatrices.resize(numVoxels);
				}
				Jobs::ParallelFor(numVoxels, [&](unsigned i) {
					const auto coordinate = voxelGrid.GetCoordinate(i);
					scalarMatrices[i].m_instanceMatrix.m_value =
						glm::translate(voxelGrid.GetPosition(coordinate))
						* glm::mat4_cast(glm::quat(glm::vec3(0.0f)))
						* glm::scale(glm::vec3(0.8f * voxelGrid.GetVoxelSize()));
					scalarMatrices[i].m_instanceColor = glm::vec4(0.0f, 0.0f, 0.0f, glm::clamp(voxelGrid.Peek(static_cast<int>(i)).m_shadowIntensity, 0.0f, 1.0f));
					}
				);
				shadowGridParticleInfoList->SetPendingUpdate();
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
					if (changed) {
						m_treeVisualizer.m_iteration = m_treeModel.CurrentIteration();
						m_treeVisualizer.m_needUpdate = true;
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
		const auto scene = GetScene();
		m_treeVisualizer.Visualize(m_treeModel,
			scene->GetDataComponent<GlobalTransform>(GetOwner()));
		GizmoSettings gizmoSettings {};
		gizmoSettings.m_drawSettings.m_blending = true;
		if (showSpaceColonizationGrid)
		{
			editorLayer->DrawGizmoMeshInstancedColored(
				Resources::GetResource<Mesh>("PRIMITIVE_CUBE"), spaceColonizationGridParticleInfoList,
				glm::mat4(1.0f), 1.0f, gizmoSettings);
		}
		if(showShadowGrid)
		{
			editorLayer->DrawGizmoMeshInstancedColored(
				Resources::GetResource<Mesh>("PRIMITIVE_CUBE"), shadowGridParticleInfoList,
				glm::mat4(1.0f), 1.0f, gizmoSettings);
		}
	}
}
void Tree::Update()
{
	if (m_temporalProgression) {
		if (m_temporalProgressionIteration <= m_treeModel.CurrentIteration()) {
			GenerateMeshes(m_meshGeneratorSettings, m_temporalProgressionIteration);
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

bool Tree::TryGrow(float deltaTime) {
	const auto scene = GetScene();
	const auto treeDescriptor = m_treeDescriptor.Get<TreeDescriptor>();
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
	ecoSysLabLayer->m_needFullFlowUpdate = true;
}

void Tree::InitializeStrandRenderer()
{
	const auto scene = GetScene();
	const auto owner = GetOwner();

	ClearStrands();
	const auto strandsEntity = scene->CreateEntity("Branch Strands");
	scene->SetParent(strandsEntity, owner);

	const auto renderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(strandsEntity).lock();
	const auto strandsAsset = ProjectManager::CreateTemporaryAsset<Strands>();

	m_treePipeModel.m_shootPipeModel.CalculatePipeSegmentInfos(m_pipeModelParameters);

	std::vector<glm::uint> strandsList;
	std::vector<StrandPoint> points;
	m_treePipeModel.m_shootPipeModel.m_pipeGroup.BuildStrands(strandsList, points);
	if (!points.empty()) strandsList.emplace_back(points.size());
	StrandPointAttributes strandPointAttributes{};
	strandPointAttributes.m_color = true;
	strandsAsset->SetStrands(strandPointAttributes, strandsList, points);
	renderer->m_strands = strandsAsset;

	const auto material = ProjectManager::CreateTemporaryAsset<Material>();
	
	renderer->m_material = material;
	material->m_vertexColorOnly = true;
	material->m_materialProperties.m_albedoColor = glm::vec3(0.6f, 0.3f, 0.0f);
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
		else if (name == "Fine Root Mesh") {
			scene->DeleteEntity(child);
		}
		else if (name == "Twig Mesh") {
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
		if (name == "Branch Strands") {
			scene->DeleteEntity(child);
		}
		else if (name == "Root Strands") {
			scene->DeleteEntity(child);
		}
	}
}

void Tree::GenerateMeshes(const TreeMeshGeneratorSettings& meshGeneratorSettings, int iteration) {
	const auto scene = GetScene();
	const auto self = GetOwner();
	const auto children = scene->GetChildren(self);

	ClearMeshes();

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
		std::vector<Vertex> vertices;
		std::vector<unsigned int> indices;
		switch (meshGeneratorSettings.m_branchMeshType)
		{
		case 0:
		{
			CylindricalMeshGenerator<ShootGrowthData, ShootStemGrowthData, InternodeGrowthData> meshGenerator;
			meshGenerator.Generate(m_treeModel.PeekShootSkeleton(actualIteration), vertices, indices,
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
				minRadius = treeDescriptor->m_shootGrowthParameters.m_endNodeThickness;
			}
			VoxelMeshGenerator<ShootGrowthData, ShootStemGrowthData, InternodeGrowthData> meshGenerator;
			meshGenerator.Generate(m_treeModel.PeekShootSkeleton(actualIteration), vertices, indices,
				meshGeneratorSettings, minRadius);
		}
		break;
		default: break;
		}


		auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
		auto material = ProjectManager::CreateTemporaryAsset<Material>();
		VertexAttributes vertexAttributes{};
		vertexAttributes.m_texCoord = true;
		mesh->SetVertices(vertexAttributes, vertices, indices);
		auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(branchEntity).lock();
		if (meshGeneratorSettings.m_overridePresentation)
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
		if (meshGeneratorSettings.m_overridePresentation)
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
		fineRootEntity = scene->CreateEntity("Fine Root Mesh");
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
			if (rootNodeInfo.m_thickness < meshGeneratorSettings.m_fineRootParameters.m_minNodeThicknessRequirement && rootNodeData.m_rootDistance > meshGeneratorSettings.m_fineRootParameters.m_distanceFromRoot)
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
		if (meshGeneratorSettings.m_overridePresentation)
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
		twigEntity = scene->CreateEntity("Twig Mesh");
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
			if (internodeInfo.m_thickness < meshGeneratorSettings.m_twigParameters.m_minNodeThicknessRequirement && internodeData.m_rootDistance > meshGeneratorSettings.m_twigParameters.m_distanceFromRoot)
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
		if (meshGeneratorSettings.m_overridePresentation)
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
		if (!meshGeneratorSettings.m_detailedFoliage) {
			std::vector<Vertex> vertices;
			std::vector<unsigned int> indices;
			auto quadMesh = Resources::GetResource<Mesh>("PRIMITIVE_QUAD");
			auto& quadTriangles = quadMesh->UnsafeGetTriangles();
			auto quadVerticesSize = quadMesh->GetVerticesAmount();
			size_t offset = 0;

			const auto& nodeList = m_treeModel.PeekShootSkeleton(actualIteration).RefSortedNodeList();
			for (const auto& internodeHandle : nodeList) {
				const auto& internode = m_treeModel.PeekShootSkeleton(actualIteration).PeekNode(internodeHandle);
				const auto& internodeInfo = internode.m_info;
				const auto& internodeData = internode.m_data;
				if (!meshGeneratorSettings.m_overridePresentation) {
					for (const auto& bud : internodeData.m_buds) {
						if (bud.m_status != BudStatus::Flushed) continue;
						if (bud.m_reproductiveModule.m_maturity <= 0.0f) continue;
						if (bud.m_type == BudType::Leaf)
						{
							auto& matrix = bud.m_reproductiveModule.m_transform;
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
				else
				{
					const auto& presentationSettings = meshGeneratorSettings.m_presentationOverrideSettings;
					if (internodeData.m_maxDistanceToAnyBranchEnd < presentationSettings.m_distanceToEndLimit) {
						for (int i = 0; i < presentationSettings.m_leafCountPerInternode; i++)
						{
							auto leafSize = presentationSettings.m_leafSize;
							glm::quat rotation = internodeInfo.m_globalDirection * glm::quat(glm::radians(glm::linearRand(glm::vec3(0.0f), glm::vec3(360.0f))));
							auto front = rotation * glm::vec3(0, 0, -1);
							TreeModel::ApplyTropism(internodeData.m_lightDirection, presentationSettings.m_phototropism, rotation);
							auto foliagePosition = internodeInfo.m_globalPosition + front * (leafSize.z * 1.5f);
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
			}

			auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
			auto material = ProjectManager::CreateTemporaryAsset<Material>();
			VertexAttributes vertexAttributes{};
			vertexAttributes.m_texCoord = true;
			mesh->SetVertices(vertexAttributes, vertices, indices);
			if (meshGeneratorSettings.m_overridePresentation)
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
		else
		{
			for (int i = 0; i < 8; i++)
			{
				float maxHealth = i / 8.0f;
				const auto& nodeList = m_treeModel.PeekShootSkeleton(actualIteration).RefSortedNodeList();
				for (const auto& internodeHandle : nodeList) {
					const auto& internode = m_treeModel.PeekShootSkeleton(actualIteration).PeekNode(internodeHandle);
					const auto& internodeInfo = internode.m_info;
					const auto& internodeData = internode.m_data;
					if (!meshGeneratorSettings.m_overridePresentation) {
						for (const auto& bud : internodeData.m_buds) {
							if (bud.m_status != BudStatus::Flushed) continue;
							if (bud.m_reproductiveModule.m_maturity <= 0.0f) continue;
							if (bud.m_type == BudType::Leaf)
							{
								auto matrix = bud.m_reproductiveModule.m_transform;
								Vertex archetype;

							}
						}
					}
				}
			}
		}
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
			if (!meshGeneratorSettings.m_overridePresentation) {
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
		if (meshGeneratorSettings.m_overridePresentation)
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

void Tree::FromLSystemString(const std::shared_ptr<LSystemString>& lSystemString)
{
}

void Tree::FromTreeGraph(const std::shared_ptr<TreeGraph>& treeGraph)
{
}

void Tree::FromTreeGraphV2(const std::shared_ptr<TreeGraphV2>& treeGraphV2)
{
}

void TreeDescriptor::OnCreate() {

}


bool OnInspectShootGrowthParameters(ShootGrowthParameters& treeGrowthParameters) {
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

			changed = ImGui::DragFloat4("Lateral bud flushing prob/temp range", &treeGrowthParameters.m_lateralBudFlushingProbabilityTemperatureRange.x, 0.00001f, 0.0f, 1.0f, "%.5f") || changed;
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
			changed = ImGui::DragFloat3("Thickness min/factor/age", &treeGrowthParameters.m_endNodeThickness, 0.00001f, 0.0f, 1.0f, "%.6f") || changed;
			changed = ImGui::DragFloat3("Sagging thickness/reduction/max", &treeGrowthParameters.m_saggingFactorThicknessReductionMax.x, 0.01f, 0.0f, 1.0f, "%.5f") || changed;
			changed = ImGui::DragFloat("Low Branch Pruning", &treeGrowthParameters.m_lowBranchPruning, 0.01f) || changed;
			changed = ImGui::DragFloat("Light pruning factor", &treeGrowthParameters.m_endNodePruningLightFactor, 0.01f) || changed;
			ImGui::TreePop();
		}

		if (ImGui::TreeNodeEx("Leaf", ImGuiTreeNodeFlags_DefaultOpen))
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
		if (ImGui::TreeNodeEx("Fruit", ImGuiTreeNodeFlags_DefaultOpen))
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

bool OnInspectRootGrowthParameters(RootGrowthParameters& rootGrowthParameters) {
	bool changed = false;
	if (ImGui::TreeNodeEx("Root Growth Parameters", ImGuiTreeNodeFlags_DefaultOpen)) {
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
			tree->m_treeModel.m_treeIlluminationEstimator.m_settings = ecoSysLabLayer->m_shadowEstimationSettings;
		}
	}
	else
	{
		ImGui::Text("Attach soil and climate entity to instantiate!");
	}
	if (OnInspectShootGrowthParameters(m_shootGrowthParameters)) { changed = true; }
	if (OnInspectRootGrowthParameters(m_rootGrowthParameters)) { changed = true; }
	if (changed) m_saved = false;
}

void TreeDescriptor::CollectAssetRef(std::vector<AssetRef>& list) {

}

void SerializeShootGrowthParameters(const std::string& name, const ShootGrowthParameters& treeGrowthParameters, YAML::Emitter& out) {
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
	out << YAML::Key << "m_lateralBudFlushingProbabilityTemperatureRange" << YAML::Value << treeGrowthParameters.m_lateralBudFlushingProbabilityTemperatureRange;
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
	out << YAML::Key << "m_saggingFactorThicknessReductionMax" << YAML::Value << treeGrowthParameters.m_saggingFactorThicknessReductionMax;

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
void SerializeRootGrowthParameters(const std::string& name, const RootGrowthParameters& rootGrowthParameters, YAML::Emitter& out) {
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

	
}

void DeserializeShootGrowthParameters(const std::string& name, ShootGrowthParameters& treeGrowthParameters, const YAML::Node& in) {
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

		if (param["m_endNodeThickness"]) treeGrowthParameters.m_endNodeThickness = param["m_endNodeThickness"].as<float>();
		if (param["m_thicknessAccumulationFactor"]) treeGrowthParameters.m_thicknessAccumulationFactor = param["m_thicknessAccumulationFactor"].as<float>();
		if (param["m_thicknessAccumulateAgeFactor"]) treeGrowthParameters.m_thicknessAccumulateAgeFactor = param["m_thicknessAccumulateAgeFactor"].as<float>();

		if (param["m_lowBranchPruning"]) treeGrowthParameters.m_lowBranchPruning = param["m_lowBranchPruning"].as<float>();
		if (param["m_saggingFactorThicknessReductionMax"]) treeGrowthParameters.m_saggingFactorThicknessReductionMax = param["m_saggingFactorThicknessReductionMax"].as<glm::vec3>();

		//Bud fate

		if (param["m_lateralBudFlushingProbabilityTemperatureRange"]) treeGrowthParameters.m_lateralBudFlushingProbabilityTemperatureRange = param["m_lateralBudFlushingProbabilityTemperatureRange"].as<glm::vec4>();
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
void DeserializeRootGrowthParameters(const std::string& name, RootGrowthParameters& rootGrowthParameters, const YAML::Node& in) {
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
		m_shootGrowthController.m_lateralBudFlushingProbability = [=](const Node<InternodeGrowthData>& internode)
			{
				const auto& shootGrowthParameters = treeDescriptor->m_shootGrowthParameters;
				const auto& probabilityRange = shootGrowthParameters.m_lateralBudFlushingProbabilityTemperatureRange;
				const auto& internodeData = internode.m_data;
				float flushProbability = glm::mix(probabilityRange.x, probabilityRange.y,
					glm::clamp((internodeData.m_temperature - probabilityRange.z) / (probabilityRange.w - probabilityRange.z), 0.0f, 1.0f));
				if (internodeData.m_inhibitor > 0.0f) flushProbability *= glm::exp(-internodeData.m_inhibitor);
				flushProbability *= internodeData.m_growthPotential;
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
		m_shootGrowthController.m_endNodeThickness = treeDescriptor->m_shootGrowthParameters.m_endNodeThickness;
		m_shootGrowthController.m_thicknessAccumulationFactor = treeDescriptor->m_shootGrowthParameters.m_thicknessAccumulationFactor;
		m_shootGrowthController.m_thicknessAccumulateAgeFactor = treeDescriptor->m_shootGrowthParameters.m_thicknessAccumulateAgeFactor;
		m_shootGrowthController.m_lowBranchPruning = treeDescriptor->m_shootGrowthParameters.m_lowBranchPruning;
		m_shootGrowthController.m_pruningFactor = [=](const Node<InternodeGrowthData>& internode)
			{
				float pruningProbability = 0.0f;
				const auto& shootGrowthParameters = treeDescriptor->m_shootGrowthParameters;
				const auto& internodeData = internode.m_data;
				if (internode.IsEndNode())
				{
					pruningProbability = (1.0f - internode.m_data.m_growthPotential) * shootGrowthParameters.m_endNodePruningLightFactor;
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
