//
// Created by lllll on 11/1/2022.
//

#include "EcoSysLabLayer.hpp"

#include "BranchShape.hpp"
#include "Times.hpp"

#include "StrandsRenderer.hpp"
#include "Climate.hpp"
#include "ForestPatch.hpp"
#include "Soil.hpp"
#include "Tree.hpp"
#include "RenderLayer.hpp"
#include "TreePipeBase.hpp"
#include "TreePipeNode.hpp"
#include "TreePointCloud.hpp"
#include "TreePointCloudScanner.hpp"
#include "ClassRegistry.hpp"
#include "CubeVolume.hpp"
using namespace EcoSysLab;

void EcoSysLabLayer::OnCreate() {
	ClassRegistry::RegisterPrivateComponent<Tree>("Tree");
	ClassRegistry::RegisterPrivateComponent<TreePointCloud>("TreePointCloud");
	ClassRegistry::RegisterPrivateComponent<Soil>("Soil");
	ClassRegistry::RegisterPrivateComponent<Climate>("Climate");

	ClassRegistry::RegisterAsset<BranchShape>("BranchShape", { ".bs" });
	ClassRegistry::RegisterAsset<ForestPatch>("ForestPatch", { ".fp" });
	ClassRegistry::RegisterAsset<TreeDescriptor>("TreeDescriptor", { ".td" });
	ClassRegistry::RegisterAsset<SoilDescriptor>("SoilDescriptor", { ".sd" });
	ClassRegistry::RegisterAsset<ClimateDescriptor>("ClimateDescriptor", { ".cd" });
	ClassRegistry::RegisterAsset<RadialBoundingVolume>("RadialBoundingVolume", { ".rbv" });
	ClassRegistry::RegisterAsset<CubeVolume>("CubeVolume", { ".cv" });
	ClassRegistry::RegisterAsset<HeightField>("HeightField", { ".hf" });
	ClassRegistry::RegisterAsset<SoilLayerDescriptor>("SoilLayerDescriptor", { ".nsld" });
	ClassRegistry::RegisterPrivateComponent<TreePipeBase>("TreePipeBase");
	ClassRegistry::RegisterPrivateComponent<TreePipeNode>("TreePipeNode");
	ClassRegistry::RegisterPrivateComponent<TreePointCloudScanner>("TreePointCloudScanner");

	if (m_randomColors.empty()) {
		for (int i = 0; i < 20000; i++) {
			m_randomColors.emplace_back(glm::linearRand(glm::vec3(0.0f), glm::vec3(1.0f)));
		}
	}

	if (m_soilLayerColors.empty()) {
		for (int i = 0; i < 10; i++) {
			glm::vec4 color = { glm::linearRand(glm::vec3(0.0f), glm::vec3(1.0f)), 1.0f };
			m_soilLayerColors.emplace_back(color);
		}
	}
	m_shootStemStrands = ProjectManager::CreateTemporaryAsset<Strands>();

	m_boundingBoxMatrices = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
	m_foliageMatrices = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
	m_fruitMatrices = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();

	m_groundFruitMatrices = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
	m_groundLeafMatrices = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
	m_vectorMatrices = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
	m_scalarMatrices = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
	m_shadowGridParticleInfoList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();

#pragma region Internode camera
	m_visualizationCamera =
		Serialization::ProduceSerializable<Camera>();

	m_visualizationCamera->OnCreate();
	m_visualizationCamera->m_useClearColor = true;
	m_visualizationCamera->m_clearColor = glm::vec3(0.5f, 0.5f, 0.5f);
#pragma endregion

	if (const auto editorLayer = Application::GetLayer<EditorLayer>())
	{
		editorLayer->RegisterEditorCamera(m_visualizationCamera);
	}
}

void EcoSysLabLayer::OnDestroy() {

}


void EcoSysLabLayer::Visualization() {
	auto scene = GetScene();
	auto editorLayer = Application::GetLayer<EditorLayer>();

	auto selectedEntity = editorLayer->GetSelectedEntity();
	if (selectedEntity != m_selectedTree) {
		if (scene->IsEntityValid(selectedEntity) && scene->HasPrivateComponent<Tree>(selectedEntity)) {
			m_selectedTree = selectedEntity;
			m_lastSelectedTreeIndex = m_selectedTree.GetIndex();
			m_needFlowUpdateForSelection = true;
		}
		else {
			m_selectedTree = Entity();
			m_needFlowUpdateForSelection = true;
		}
	}
	const std::vector<Entity>* treeEntities =
		scene->UnsafeGetPrivateComponentOwnersList<Tree>();

	const auto branchStrands = m_shootStemStrands.Get<Strands>();
	if (treeEntities && !treeEntities->empty()) {
		//Tree selection
		if (m_shootVersions.size() != treeEntities->size()) {
			m_internodeSize = 0;
			m_rootNodeSize = 0;
			m_totalTime = 0.0f;
			m_shootVersions.clear();
			for (int i = 0; i < treeEntities->size(); i++) {
				m_shootVersions.emplace_back(-1);
			}
			m_needFullFlowUpdate = true;
		}
		for (int i = 0; i < treeEntities->size(); i++) {
			auto treeEntity = treeEntities->at(i);
			auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
			auto& treeModel = tree->m_treeModel;
			if (m_shootVersions[i] != treeModel.RefShootSkeleton().GetVersion()) {
				m_shootVersions[i] = treeModel.RefShootSkeleton().GetVersion();
				m_needFullFlowUpdate = true;
			}
		}
		bool flowUpdated = false;
		if (m_debugVisualization) {

			if (m_needFullFlowUpdate) {
				UpdateFlows(treeEntities, branchStrands, -1);
				UpdateGroundFruitAndLeaves();
				m_needFullFlowUpdate = false;
				flowUpdated = true;
			}
			if (m_needFlowUpdateForSelection) {
				UpdateFlows(treeEntities, branchStrands, m_lastSelectedTreeIndex);
				m_needFlowUpdateForSelection = false;
				flowUpdated = true;
			}
		}
		if (flowUpdated) {
			const auto strandsHolder = m_shootStemStrandsHolder.Get();
			if (scene->IsEntityValid(strandsHolder)) {
				auto branchStrandsRenderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(strandsHolder).lock();
				branchStrandsRenderer->m_strands = m_shootStemStrands;
				auto material = branchStrandsRenderer->m_material.Get<Material>();
				if (!material) {
					material = ProjectManager::CreateTemporaryAsset<Material>();

					branchStrandsRenderer->m_material = material;
					material->m_materialProperties.m_albedoColor = glm::vec3(109, 79, 75) / 255.0f;
					if (m_meshGeneratorSettings.m_foliageOverride) {
						material->m_materialProperties.m_albedoColor = m_meshGeneratorSettings.m_presentationOverrideSettings.m_branchOverrideColor;
					}
					material->m_materialProperties.m_roughness = 1.0f;
					material->m_materialProperties.m_metallic = 0.0f;
				}
			}


			auto foliageHolder = m_foliageHolder.Get();
			if (scene->IsEntityValid(foliageHolder)) {
				auto foliageRenderer = scene->GetOrSetPrivateComponent<Particles>(foliageHolder).lock();
				auto mesh = foliageRenderer->m_mesh.Get<Mesh>();
				if (!mesh) foliageRenderer->m_mesh = Resources::GetResource<Mesh>("PRIMITIVE_QUAD");;
				foliageRenderer->m_particleInfoList = m_foliageMatrices;
				auto material = foliageRenderer->m_material.Get<Material>();
				if (!material) {
					material = ProjectManager::CreateTemporaryAsset<Material>();
					foliageRenderer->m_material = material;
					material->m_materialProperties.m_albedoColor = glm::vec3(80, 60, 50) / 255.0f;
					if (m_meshGeneratorSettings.m_foliageOverride) {
						material->m_materialProperties.m_albedoColor = m_meshGeneratorSettings.m_foliageOverrideSettings.m_leafColor;
					}
					material->m_materialProperties.m_roughness = 1.0f;
					material->m_materialProperties.m_metallic = 0.2f;
				}
				foliageRenderer->m_particleInfoList.Get<ParticleInfoList>()->SetPendingUpdate();
			}
			auto fruitHolder = m_fruitHolder.Get();
			if (scene->IsEntityValid(fruitHolder)) {
				auto fruitRenderer = scene->GetOrSetPrivateComponent<Particles>(fruitHolder).lock();
				auto mesh = fruitRenderer->m_mesh.Get<Mesh>();
				if (!mesh) fruitRenderer->m_mesh = Resources::GetResource<Mesh>("PRIMITIVE_SPHERE");;
				fruitRenderer->m_particleInfoList = m_fruitMatrices;
				auto material = fruitRenderer->m_material.Get<Material>();
				if (!material) {
					material = ProjectManager::CreateTemporaryAsset<Material>();
					fruitRenderer->m_material = material;
					material->m_materialProperties.m_albedoColor = glm::vec3(80, 60, 50) / 255.0f;
					if (m_meshGeneratorSettings.m_foliageOverride) {
						material->m_materialProperties.m_albedoColor = m_meshGeneratorSettings.m_foliageOverrideSettings.m_leafColor;
					}
					material->m_materialProperties.m_roughness = 1.0f;
					material->m_materialProperties.m_metallic = 0.2f;
				}
				fruitRenderer->m_particleInfoList.Get<ParticleInfoList>()->SetPendingUpdate();
			}
			if (auto climate = m_climateHolder.Get<Climate>()) {
				const auto& voxelGrid = climate->m_climateModel.m_environmentGrid.m_voxel;
				const auto numVoxels = voxelGrid.GetVoxelCount();
				auto& scalarMatrices = m_shadowGridParticleInfoList->m_particleInfos;
				if (scalarMatrices.size() != numVoxels) {
					scalarMatrices.resize(numVoxels);
				}
				Jobs::ParallelFor(numVoxels, [&](unsigned i) {
					const auto coordinate = voxelGrid.GetCoordinate(i);
					scalarMatrices[i].m_instanceMatrix.m_value =
						glm::translate(voxelGrid.GetPosition(coordinate) + glm::linearRand(-glm::vec3(0.5f * voxelGrid.GetVoxelSize()), glm::vec3(0.5f * voxelGrid.GetVoxelSize())))
						* glm::mat4_cast(glm::quat(glm::vec3(0.0f)))
						* glm::scale(glm::vec3(0.25f * voxelGrid.GetVoxelSize()));
					scalarMatrices[i].m_instanceColor = glm::vec4(0.5f, 0.5f, 0.5f, glm::clamp(voxelGrid.Peek(static_cast<int>(i)).m_shadowIntensity, 0.0f, 1.0f));
					}
				);
				m_shadowGridParticleInfoList->SetPendingUpdate();
			}
		}
	}
	if (m_debugVisualization) {
		GizmoSettings gizmoSettings;
		gizmoSettings.m_drawSettings.m_blending = true;
		
		gizmoSettings.m_drawSettings.m_blendingSrcFactor = VK_BLEND_FACTOR_SRC_ALPHA;
		gizmoSettings.m_drawSettings.m_blendingDstFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
		gizmoSettings.m_drawSettings.m_cullMode = VK_CULL_MODE_BACK_BIT;

		if (m_showShadowGrid)
		{
			editorLayer->DrawGizmoMeshInstancedColored(
				Resources::GetResource<Mesh>("PRIMITIVE_CUBE"), m_visualizationCamera,
				m_shadowGridParticleInfoList,
				glm::mat4(1.0f), 1.0f, gizmoSettings);
		}

		if (m_displayShootStem && !m_shootStemPoints.empty()) {
			gizmoSettings.m_colorMode = GizmoSettings::ColorMode::Default;
			editorLayer->DrawGizmoStrands(branchStrands, m_visualizationCamera, glm::vec4(1.0f, 1.0f, 1.0f, 0.75f), glm::mat4(1.0f), 1,
				gizmoSettings);
		}
		if (m_displayFruit && !m_fruitMatrices->m_particleInfos.empty()) {
			editorLayer->DrawGizmoMeshInstancedColored(
				Resources::GetResource<Mesh>("PRIMITIVE_CUBE"), m_visualizationCamera,
				m_fruitMatrices,
				glm::mat4(1.0f), 1.0f, gizmoSettings);
		}
		gizmoSettings.m_drawSettings.m_cullMode = VK_CULL_MODE_NONE;
		if (m_displayFoliage && !m_foliageMatrices->m_particleInfos.empty()) {
			editorLayer->DrawGizmoMeshInstancedColored(
				Resources::GetResource<Mesh>("PRIMITIVE_QUAD"), m_visualizationCamera,
				m_foliageMatrices,
				glm::mat4(1.0f), 1.0f, gizmoSettings);
		}
		if (m_displayGroundLeaves && !m_groundLeafMatrices->m_particleInfos.empty()) {
			editorLayer->DrawGizmoMeshInstancedColored(
				Resources::GetResource<Mesh>("PRIMITIVE_QUAD"), m_visualizationCamera,
				m_groundLeafMatrices,
				glm::mat4(1.0f), 1.0f, gizmoSettings);
		}
		gizmoSettings.m_drawSettings.m_cullMode = VK_CULL_MODE_BACK_BIT;

		if (m_displayGroundFruit && !m_groundFruitMatrices->m_particleInfos.empty()) {
			editorLayer->DrawGizmoMeshInstancedColored(
				Resources::GetResource<Mesh>("PRIMITIVE_CUBE"), m_visualizationCamera,
				m_groundFruitMatrices,
				glm::mat4(1.0f), 1.0f, gizmoSettings);
		}

		if (m_displayBoundingBox && !m_boundingBoxMatrices->m_particleInfos.empty()) {
			editorLayer->DrawGizmoMeshInstancedColored(
				Resources::GetResource<Mesh>("PRIMITIVE_CUBE"), m_visualizationCamera,
				m_boundingBoxMatrices,
				glm::mat4(1.0f), 1.0f, gizmoSettings);
		}

		gizmoSettings.m_colorMode = GizmoSettings::ColorMode::Default;
		if (m_displaySoil) {
			SoilVisualization();
		}
		
		
	}
}

void EcoSysLabLayer::ResetAllTrees(const std::vector<Entity>* treeEntities) {
	const auto scene = Application::GetActiveScene();
	m_time = 0;
	m_simulationSettings.m_iteration = 0;
	for (const auto& i : *treeEntities) {
		const auto tree = scene->GetOrSetPrivateComponent<Tree>(i).lock();
		tree->Reset();
	}
	m_needFullFlowUpdate = true;
	m_totalTime = 0;
	m_internodeSize = 0;
	m_leafSize = 0;
	m_fruitSize = 0;
	m_shootStemSize = 0;
	m_rootNodeSize = 0;
	m_rootStemSize = 0;

	m_shootStemSegments.clear();
	m_shootStemPoints.clear();
	
	m_shootStemStrands = ProjectManager::CreateTemporaryAsset<Strands>();
	
	m_boundingBoxMatrices->m_particleInfos.clear();
	m_boundingBoxMatrices->SetPendingUpdate();
	m_foliageMatrices->m_particleInfos.clear();
	m_foliageMatrices->SetPendingUpdate();
	m_fruitMatrices->m_particleInfos.clear();
	m_fruitMatrices->SetPendingUpdate();

	if (const auto climate = m_climateHolder.Get<Climate>()) {
		climate->m_climateModel.m_environmentGrid = {};
	}
}

const std::vector<glm::vec3>& EcoSysLabLayer::RandomColors() {
	return m_randomColors;
}

void EcoSysLabLayer::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
	auto scene = GetScene();
	bool simulate = false;
	static bool autoTimeGrow = false;
	static float targetTime = 0.f;
	if (ImGui::Begin("EcoSysLab Layer")) {
		const std::vector<Entity>* treeEntities =
			scene->UnsafeGetPrivateComponentOwnersList<Tree>();
		editorLayer->DragAndDropButton<Soil>(m_soilHolder, "Soil");
		ImGui::SameLine();
		editorLayer->DragAndDropButton<Climate>(m_climateHolder, "Climate");
		if (treeEntities && !treeEntities->empty()) {
			if (ImGui::TreeNode("Tree Growth Settings")){
				ImGui::DragFloat("Delta time", &m_simulationSettings.m_deltaTime, 0.00001f, 0, 1, "%.5f");
				ImGui::Checkbox("Auto clear fruit and leaves", &m_simulationSettings.m_autoClearFruitAndLeaves);
				ImGui::DragFloat("Crown shyness", &m_simulationSettings.m_crownShynessDistance, 0.01f, 0.0f, 1.0f);
				ImGui::Checkbox("Simulate soil", &m_simulationSettings.m_soilSimulation);
				if (ImGui::TreeNode("Shadow Estimation Settings")) {
					bool settingsChanged = false;
					settingsChanged =
						ImGui::DragFloat("Distance power factor", &m_simulationSettings.m_shadowEstimationSettings.m_distancePowerFactor, 0.01f,
							0.0f, 10.0f) || settingsChanged;

					settingsChanged =
						ImGui::DragFloat("Shadow intensity", &m_simulationSettings.m_shadowEstimationSettings.m_shadowIntensity, 0.001f,
							0.0f, 1.0f) || settingsChanged;

					settingsChanged =
						ImGui::DragFloat("Shadow propagate loss", &m_simulationSettings.m_shadowEstimationSettings.m_shadowPropagateLoss, 0.001f,
							0.0f, 1.0f) || settingsChanged;

					if (settingsChanged) {
						if (const auto climate = m_climateHolder.Get<Climate>()) {
							auto& estimator = climate->m_climateModel.m_environmentGrid;
							estimator.m_settings = m_simulationSettings.m_shadowEstimationSettings;
							auto minBound = estimator.m_voxel.GetMinBound();
							auto maxBound = estimator.m_voxel.GetMaxBound();
							bool boundChanged = false;
							for (const auto& treeEntity : *treeEntities)
							{
								if (!scene->IsEntityEnabled(treeEntity)) return;
								auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
								if (!tree->IsEnabled()) return;
								const auto globalTransform = scene->GetDataComponent<GlobalTransform>(treeEntity).m_value;
								const glm::vec3 currentMinBound = globalTransform * glm::vec4(tree->m_treeModel.RefShootSkeleton().m_min, 1.0f);
								const glm::vec3 currentMaxBound = globalTransform * glm::vec4(tree->m_treeModel.RefShootSkeleton().m_max, 1.0f);

								if (currentMinBound.x <= minBound.x || currentMinBound.y <= minBound.y || currentMinBound.z <= minBound.z
									|| currentMaxBound.x >= maxBound.x || currentMaxBound.y >= maxBound.y || currentMaxBound.z >= maxBound.z) {
									minBound = glm::min(currentMinBound - glm::vec3(1.0f, 0.1f, 1.0f), minBound);
									maxBound = glm::max(currentMaxBound + glm::vec3(1.0f), maxBound);
									boundChanged = true;
								}
								if (!tree->m_climate.Get<Climate>()) tree->m_climate = climate;
								tree->m_treeModel.m_crownShynessDistance = m_simulationSettings.m_crownShynessDistance;
							}
							if (boundChanged) estimator.m_voxel.Initialize(estimator.m_voxelSize, minBound, maxBound);
							estimator.m_voxel.Reset();
							for (const auto& treeEntity : *treeEntities)
							{
								if (!scene->IsEntityEnabled(treeEntity)) return;
								auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
								if (!tree->IsEnabled()) return;
								tree->RegisterVoxel();
							}

							estimator.ShadowPropagation();
							if (const auto climate = m_climateHolder.Get<Climate>()) {
								for (const auto& i : *treeEntities) {
									const auto tree = scene->GetOrSetPrivateComponent<Tree>(i).lock();
									tree->m_treeModel.CollectShootFlux(scene->GetDataComponent<GlobalTransform>(i).m_value,
										climate->m_climateModel, tree->m_treeModel.RefShootSkeleton().RefSortedNodeList(), tree->m_shootGrowthController);
									tree->m_treeVisualizer.m_needShootColorUpdate = true;
								}
							}
						}
					}
					ImGui::TreePop();
				}
				
				ImGui::TreePop();
			}
			
			if (ImGui::Button("Reset all trees")) {
				ResetAllTrees(treeEntities);
				ClearGeometries();
				ClearGroundFruitAndLeaf();
				targetTime = 0.0f;
			}
			ImGui::Text(("Simulated time: " + std::to_string(m_time) + " years").c_str());
			ImGui::Text(("Simulated iteration: " + std::to_string(m_simulationSettings.m_iteration)).c_str());
			if (ImGui::Button("Day")) m_simulationSettings.m_deltaTime = 0.00274f;
			ImGui::SameLine();
			if (ImGui::Button("Week")) m_simulationSettings.m_deltaTime = 0.01918f;
			ImGui::SameLine();
			if (ImGui::Button("Month")) m_simulationSettings.m_deltaTime = 0.0822f;
			ImGui::SameLine();
			
			
			if (targetTime < m_time) targetTime = m_time;
			ImGui::DragFloat("Target time", &targetTime, 0.1f, m_time, 999);
			
			if(m_time < targetTime) ImGui::Checkbox(("Grow until " + std::to_string(targetTime) + " years").c_str(), &autoTimeGrow);
			if (autoTimeGrow && m_time >= targetTime) autoTimeGrow = false;
			
			if (!autoTimeGrow && ImGui::Button("Grow 1 iteration")) {
				simulate = true;
			}
			ImGui::DragInt("Max node limit", &m_simulationSettings.m_maxNodeCount, 500, 0, INT_MAX);

			
			if (!m_simulationSettings.m_autoClearFruitAndLeaves && ImGui::Button("Clear ground leaves and fruits")) {
				ClearGroundFruitAndLeaf();
			}
			if (ImGui::TreeNodeEx("Mesh generation")) {
				m_meshGeneratorSettings.OnInspect(editorLayer);
				ImGui::TreePop();
			}
			if (ImGui::Button("Generate Meshes")) {
				GenerateMeshes(m_meshGeneratorSettings);
			}
			ImGui::SameLine();
			if (ImGui::Button("Clear Meshes")) {
				ClearGeometries();
			}
			ImGui::Text("Growth time: %.4f", m_lastUsedTime);
			ImGui::Text("Total time: %.4f", m_totalTime);
			ImGui::Text("Tree count: %d", treeEntities->size());
			ImGui::Text("Total internode size: %d", m_internodeSize);
			ImGui::Text("Total shoot branch size: %d", m_shootStemSize);
			ImGui::Text("Total fruit size: %d", m_fruitSize);
			ImGui::Text("Total leaf size: %d", m_leafSize);
			ImGui::Text("Total root node size: %d", m_rootNodeSize);
			ImGui::Text("Total root branch size: %d", m_rootStemSize);
			ImGui::Text("Total ground leaf size: %d", m_leaves.size());
			ImGui::Text("Total ground fruit size: %d", m_fruits.size());
			
			editorLayer->DragAndDropButton(m_shootStemStrandsHolder, "Shoot stem holder");
			editorLayer->DragAndDropButton(m_foliageHolder, "Foliage holder");
			editorLayer->DragAndDropButton(m_fruitHolder, "Fruit holder");
			editorLayer->DragAndDropButton(m_groundFruitsHolder, "Ground fruit holder");
			editorLayer->DragAndDropButton(m_groundLeavesHolder, "Ground leaves holder");
		}
		else {
			ImGui::Text("No trees in the scene!");
		}
		ImGui::Checkbox("Debug Visualization", &m_debugVisualization);
		if (m_debugVisualization && ImGui::TreeNodeEx("Debug visualization settings", ImGuiTreeNodeFlags_DefaultOpen)) {
			if (ImGui::Button("Update")) {
				m_needFullFlowUpdate = true;
			}
			ImGui::Checkbox("Display shoot stem", &m_displayShootStem);
			ImGui::Checkbox("Display fruits", &m_displayFruit);
			ImGui::Checkbox("Display foliage", &m_displayFoliage);

			ImGui::Checkbox("Display ground fruit", &m_displayGroundFruit);
			ImGui::Checkbox("Display ground leaves", &m_displayGroundLeaves);

			ImGui::Checkbox("Display Soil", &m_displaySoil);
			if (m_displaySoil && ImGui::TreeNodeEx("Soil visualization settings", ImGuiTreeNodeFlags_DefaultOpen)) {
				OnSoilVisualizationMenu();
				ImGui::TreePop();
			}
			ImGui::Checkbox("Display Bounding Box", &m_displayBoundingBox);
			ImGui::Checkbox("Show Shadow Grid", &m_showShadowGrid);
			ImGui::TreePop();
		}
	}
	ImGui::End();
	if (autoTimeGrow && m_time < targetTime) {
		simulate = true;
	}
	if(simulate)
	{
		Simulate(m_simulationSettings.m_deltaTime);
		if (m_simulationSettings.m_autoClearFruitAndLeaves) {
			ClearGroundFruitAndLeaf();
		}
		if (scene->IsEntityValid(m_selectedTree)) {
			auto tree = scene->GetOrSetPrivateComponent<Tree>(m_selectedTree).lock();
			tree->m_treeVisualizer.m_iteration = tree->m_treeModel.CurrentIteration();
			tree->m_treeVisualizer.m_needUpdate = true;
		}
	}

#pragma region Internode debugging camera
	if (m_debugVisualization) {
		ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{ 0, 0 });
		if (ImGui::Begin("Plant Visual")) {
			if (ImGui::BeginChild("InternodeCameraRenderer", ImVec2(0, 0), false)) {
				ImVec2 viewPortSize;
				viewPortSize = ImGui::GetWindowSize();
				m_visualizationCameraResolutionX = viewPortSize.x;
				m_visualizationCameraResolutionY = viewPortSize.y;
				ImGui::Image(m_visualizationCamera->GetRenderTexture()->GetColorImTextureId(),
					ImVec2(viewPortSize.x, viewPortSize.y), ImVec2(0, 1), ImVec2(1, 0));
				m_visualizationCameraMousePosition = glm::vec2(FLT_MAX, FLT_MIN);
				auto sceneCameraRotation = editorLayer->GetSceneCameraRotation();
				auto sceneCameraPosition = editorLayer->GetSceneCameraPosition();
				if (ImGui::IsWindowFocused()) {
					m_visualizationCameraWindowFocused = true;
					bool valid = true;
					auto mp = ImGui::GetMousePos();
					auto wp = ImGui::GetWindowPos();
					m_visualizationCameraMousePosition = glm::vec2(mp.x - wp.x, mp.y - wp.y);
					if (valid) {
						static bool isDraggingPreviously = false;
						bool mouseDrag = true;
						if (m_visualizationCameraMousePosition.x < 0 || m_visualizationCameraMousePosition.y < 0 ||
							m_visualizationCameraMousePosition.x > viewPortSize.x ||
							m_visualizationCameraMousePosition.y > viewPortSize.y ||
							editorLayer->GetKey(GLFW_MOUSE_BUTTON_RIGHT) != KeyActionType::Hold) {
							mouseDrag = false;
						}
						static float prevX = 0;
						static float prevY = 0;
						if (mouseDrag && !isDraggingPreviously) {
							prevX = m_visualizationCameraMousePosition.x;
							prevY = m_visualizationCameraMousePosition.y;
						}
						const float xOffset = m_visualizationCameraMousePosition.x - prevX;
						const float yOffset = m_visualizationCameraMousePosition.y - prevY;
						prevX = m_visualizationCameraMousePosition.x;
						prevY = m_visualizationCameraMousePosition.y;
						isDraggingPreviously = mouseDrag;
#pragma region Scene Camera Controller

						if (mouseDrag && !editorLayer->m_lockCamera) {
							glm::vec3 front = sceneCameraRotation *
								glm::vec3(0, 0, -1);
							glm::vec3 right = sceneCameraRotation *
								glm::vec3(1, 0, 0);
							if (editorLayer->GetKey(GLFW_KEY_W) == KeyActionType::Hold) {
								sceneCameraPosition +=
									front * static_cast<float>(Times::DeltaTime()) *
									editorLayer->m_velocity;
							}
							if (editorLayer->GetKey(GLFW_KEY_S) == KeyActionType::Hold) {
								sceneCameraPosition -=
									front * static_cast<float>(Times::DeltaTime()) *
									editorLayer->m_velocity;
							}
							if (editorLayer->GetKey(GLFW_KEY_A) == KeyActionType::Hold) {
								sceneCameraPosition -=
									right * static_cast<float>(Times::DeltaTime()) *
									editorLayer->m_velocity;
							}
							if (editorLayer->GetKey(GLFW_KEY_D) == KeyActionType::Hold) {
								sceneCameraPosition +=
									right * static_cast<float>(Times::DeltaTime()) *
									editorLayer->m_velocity;
							}
							if (editorLayer->GetKey(GLFW_KEY_LEFT_SHIFT) == KeyActionType::Hold) {
								sceneCameraPosition.y +=
									editorLayer->m_velocity *
									static_cast<float>(Times::DeltaTime());
							}
							if (editorLayer->GetKey(GLFW_KEY_LEFT_CONTROL) == KeyActionType::Hold) {
								sceneCameraPosition.y -=
									editorLayer->m_velocity *
									static_cast<float>(Times::DeltaTime());
							}
							if (xOffset != 0.0f || yOffset != 0.0f) {
								front = glm::rotate(front, glm::radians(-xOffset * editorLayer->m_sensitivity), glm::vec3(0, 1, 0));
								const glm::vec3 right = glm::normalize(glm::cross(front, glm::vec3(0.0f, 1.0f, 0.0f)));
								if ((front.y < 0.99f && yOffset < 0.0f) || (front.y > -0.99f && yOffset > 0.0f)) {
									front = glm::rotate(front, glm::radians(-yOffset * editorLayer->m_sensitivity), right);
								}
								const glm::vec3 up = glm::normalize(glm::cross(right, front));
								sceneCameraRotation = glm::quatLookAt(front, up);
							}
							editorLayer->SetCameraRotation(editorLayer->GetSceneCamera(), sceneCameraRotation);
							editorLayer->SetCameraPosition(editorLayer->GetSceneCamera(), sceneCameraPosition);
						}
#pragma endregion
					}
				}
				else {
					m_visualizationCameraWindowFocused = false;
				}
				editorLayer->SetCameraRotation(m_visualizationCamera, sceneCameraRotation);
				editorLayer->SetCameraPosition(m_visualizationCamera, sceneCameraPosition);
			}
			ImGui::EndChild();
			auto* window = ImGui::FindWindowByName("Plant Visual");
			m_visualizationCamera->SetEnabled(
				!(window->Hidden && !window->Collapsed));
		}
		ImGui::End();
		ImGui::PopStyleVar();
		Visualization();
	}
#pragma endregion
}

void EcoSysLabLayer::OnSoilVisualizationMenu() {
	static bool forceUpdate;
	ImGui::Checkbox("Force Update", &forceUpdate);

	if (ImGui::Checkbox("Vector Visualization", &m_vectorEnable)) {
		if (m_vectorEnable) m_updateVectorMatrices = true;
	}

	if (ImGui::Checkbox("Scalar Visualization", &m_scalarEnable)) {
		if (m_scalarEnable) m_updateScalarMatrices = true;
	}

	if (m_vectorEnable) {
		m_updateVectorMatrices = m_updateVectorMatrices || forceUpdate;

		if (ImGui::TreeNodeEx("Vector", ImGuiTreeNodeFlags_DefaultOpen)) {
			if (ImGui::Button("Reset")) {
				m_vectorMultiplier = 50.0f;
				m_vectorBaseColor = glm::vec4(1.0f, 1.0f, 1.0f, 0.8f);
				m_vectorSoilProperty = 4;
				m_vectorLineWidthFactor = 0.1f;
				m_vectorLineMaxWidth = 0.1f;
				m_updateVectorMatrices = true;
			}
			if (ImGui::ColorEdit4("Vector Base Color", &m_vectorBaseColor.x)) {
				m_updateVectorMatrices = true;
			}
			if (ImGui::DragFloat("Multiplier", &m_vectorMultiplier, 0.1f, 0.0f, 100.0f, "%.3f")) {
				m_updateVectorMatrices = true;
			}
			if (ImGui::DragFloat("Line Width Factor", &m_vectorLineWidthFactor, 0.01f, 0.0f, 5.0f)) {
				m_updateVectorMatrices = true;
			}
			if (ImGui::DragFloat("Max Line Width", &m_vectorLineMaxWidth, 0.01f, 0.0f, 5.0f)) {
				m_updateVectorMatrices = true;
			}
			if (ImGui::Combo("Vector Mode",
				{ "N/A", "N/A", "Water Density Gradient", "Flux", "Divergence", "N/A", "N/A", "N/A" },
				m_vectorSoilProperty)) {
				m_updateVectorMatrices = true;
			}
			ImGui::TreePop();
		}
	}
	if (m_scalarEnable) {
		m_updateScalarMatrices = m_updateScalarMatrices || forceUpdate;

		if (m_scalarEnable && ImGui::TreeNodeEx("Scalar", ImGuiTreeNodeFlags_DefaultOpen)) {
			if (ImGui::Button("Reset")) {
				m_scalarMultiplier = 1.0f;
				m_scalarBoxSize = 0.5f;
				m_scalarMinAlpha = 0.00f;
				m_scalarBaseColor = glm::vec3(0.0f, 0.0f, 1.0f);
				m_scalarSoilProperty = 1;
				m_updateScalarMatrices = true;
			}
			if (ImGui::SliderFloat("X Depth", &m_soilCutoutXDepth, 0.0f, 1.0f)) {
				m_updateScalarMatrices = true;
			}
			if (ImGui::SliderFloat("Z Depth", &m_soilCutoutZDepth, 0.0f, 1.0f)) {
				m_updateScalarMatrices = true;
			}

			if (ImGui::TreeNodeEx("Layer colors", ImGuiTreeNodeFlags_DefaultOpen)) {
				for (int i = 0; i < 10; i++) {
					ImGui::ColorEdit4(("Layer " + std::to_string(i)).c_str(), &m_soilLayerColors[i].x);
				}
				ImGui::TreePop();
			}

			if (ImGui::ColorEdit3("Scalar Base Color", &m_scalarBaseColor.x)) {
				m_updateScalarMatrices = true;
			}
			if (ImGui::SliderFloat("Multiplier", &m_scalarMultiplier, 0.001, 10000, "%.4f",
				ImGuiSliderFlags_Logarithmic)) {
				m_updateScalarMatrices = true;
			}
			if (ImGui::DragFloat("Min alpha", &m_scalarMinAlpha, 0.001f, 0.0f, 1.0f)) {
				m_updateScalarMatrices = true;
			}
			if (ImGui::DragFloat("Box size", &m_scalarBoxSize, 0.001f, 0.0f, 1.0f)) {
				m_updateScalarMatrices = true;
			}
			// disable less useful visualizations to avoid clutter in the gui
			if (ImGui::Combo("Scalar Mode",
				{ "Blank", "Water Density", "N/A", "N/A", "N/A", "Nutrient Density", "Soil Density",
				   "Soil Layer" }, m_scalarSoilProperty)) {
				m_updateScalarMatrices = true;
			}
			ImGui::TreePop();
		}
	}
}

void EcoSysLabLayer::UpdateFlows(const std::vector<Entity>* treeEntities, const std::shared_ptr<Strands>& branchStrands, int targetTreeIndex) {
		{
			const auto scene = Application::GetActiveScene();

			m_boundingBoxMatrices->m_particleInfos.clear();
			m_boundingBoxMatrices->SetPendingUpdate();

			std::vector<int> branchStartIndices;
			int branchLastStartIndex = 0;
			branchStartIndices.emplace_back(branchLastStartIndex);

			std::vector<int> fruitStartIndices;
			int fruitLastStartIndex = 0;
			fruitStartIndices.emplace_back(fruitLastStartIndex);

			std::vector<int> leafStartIndices;
			int leafLastStartIndex = 0;
			leafStartIndices.emplace_back(leafLastStartIndex);

			if (treeEntities->empty()) {
				m_shootStemSegments.clear();
				m_shootStemPoints.clear();

				m_foliageMatrices->m_particleInfos.clear();
				m_foliageMatrices->SetPendingUpdate();
				m_fruitMatrices->m_particleInfos.clear();
				m_fruitMatrices->SetPendingUpdate();
			}

			for (int listIndex = 0; listIndex < treeEntities->size(); listIndex++) {
				auto treeEntity = treeEntities->at(listIndex);
				auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
				auto& treeModel = tree->m_treeModel;
				const auto& branchSkeleton = treeModel.RefShootSkeleton();
				const auto& branchList = branchSkeleton.RefSortedFlowList();

				auto entityGlobalTransform = scene->GetDataComponent<GlobalTransform>(treeEntity);
				auto& [instanceMatrix, instanceColor] = m_boundingBoxMatrices->m_particleInfos.emplace_back();
				instanceMatrix.m_value = entityGlobalTransform.m_value *
					(glm::translate(
						(branchSkeleton.m_max + branchSkeleton.m_min) / 2.0f) *
						glm::scale(branchSkeleton.m_max - branchSkeleton.m_min));
				instanceColor = glm::vec4(m_randomColors[listIndex], 0.05f);
				branchLastStartIndex += branchList.size();
				branchStartIndices.emplace_back(branchLastStartIndex);

				fruitLastStartIndex += treeModel.GetFruitCount();
				fruitStartIndices.emplace_back(fruitLastStartIndex);

				leafLastStartIndex += treeModel.GetLeafCount();
				leafStartIndices.emplace_back(leafLastStartIndex);

			}
			m_shootStemSegments.resize(branchLastStartIndex * 3);
			m_shootStemPoints.resize(branchLastStartIndex * 6);

			m_foliageMatrices->m_particleInfos.resize(leafLastStartIndex);
			m_foliageMatrices->SetPendingUpdate();
			m_fruitMatrices->m_particleInfos.resize(fruitLastStartIndex);
			m_fruitMatrices->SetPendingUpdate();
			{
				auto& foliageMatrices = m_foliageMatrices->m_particleInfos;
				auto& fruitMatrices = m_fruitMatrices->m_particleInfos;
				Jobs::ParallelFor(treeEntities->size(), [&](unsigned treeIndex) {
					auto treeEntity = treeEntities->at(treeIndex);
					if (targetTreeIndex != -1 && treeEntity.GetIndex() != targetTreeIndex) return;
					auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
					auto& treeModel = tree->m_treeModel;
					const auto& branchSkeleton = treeModel.RefShootSkeleton();
					const auto& branchFlowList = branchSkeleton.RefSortedFlowList();
					const auto& internodeList = branchSkeleton.RefSortedNodeList();
					auto entityGlobalTransform = scene->GetDataComponent<GlobalTransform>(treeEntity);
					auto branchStartIndex = branchStartIndices[treeIndex];
					for (int i = 0; i < branchFlowList.size(); i++) {
						auto& flow = branchSkeleton.PeekFlow(branchFlowList[i]);
						auto cp1 = flow.m_info.m_globalStartPosition;
						auto cp4 = flow.m_info.m_globalEndPosition;
						float distance = glm::distance(cp1, cp4);
						glm::vec3 cp0, cp2;
						if (flow.GetParentHandle() > 0) {
							cp0 = cp1 + branchSkeleton.PeekFlow(flow.GetParentHandle()).m_info.m_globalEndRotation *
								glm::vec3(0, 0, 1) * distance / 3.0f;
							cp2 = cp1 + branchSkeleton.PeekFlow(flow.GetParentHandle()).m_info.m_globalEndRotation *
								glm::vec3(0, 0, -1) * distance / 3.0f;
						}
						else {
							cp0 = cp1 + flow.m_info.m_globalStartRotation * glm::vec3(0, 0, 1) * distance / 3.0f;
							cp2 = cp1 + flow.m_info.m_globalStartRotation * glm::vec3(0, 0, -1) * distance / 3.0f;
						}
						auto cp3 = cp4 + flow.m_info.m_globalEndRotation * glm::vec3(0, 0, 1) * distance / 3.0f;
						auto cp5 = cp4 + flow.m_info.m_globalEndRotation * glm::vec3(0, 0, -1) * distance / 3.0f;

						auto& p0 = m_shootStemPoints[branchStartIndex * 6 + i * 6];
						auto& p1 = m_shootStemPoints[branchStartIndex * 6 + i * 6 + 1];
						auto& p2 = m_shootStemPoints[branchStartIndex * 6 + i * 6 + 2];
						auto& p3 = m_shootStemPoints[branchStartIndex * 6 + i * 6 + 3];
						auto& p4 = m_shootStemPoints[branchStartIndex * 6 + i * 6 + 4];
						auto& p5 = m_shootStemPoints[branchStartIndex * 6 + i * 6 + 5];
						p0.m_position = (entityGlobalTransform.m_value *
							glm::translate(cp0))[3];
						p1.m_position = (entityGlobalTransform.m_value *
							glm::translate(cp1))[3];
						p2.m_position = (entityGlobalTransform.m_value *
							glm::translate(cp2))[3];
						p3.m_position = (entityGlobalTransform.m_value *
							glm::translate(cp3))[3];
						p4.m_position = (entityGlobalTransform.m_value *
							glm::translate(cp4))[3];
						p5.m_position = (entityGlobalTransform.m_value *
							glm::translate(cp5))[3];
						if (flow.GetParentHandle() > 0) {
							p1.m_thickness = branchSkeleton.PeekFlow(flow.GetParentHandle()).m_info.m_endThickness;
						}
						else {
							p1.m_thickness = flow.m_info.m_startThickness;
						}
						p4.m_thickness = flow.m_info.m_endThickness;


						p2.m_thickness = p3.m_thickness = (p1.m_thickness + p4.m_thickness) / 2.0f;
						p0.m_thickness = 2.0f * p1.m_thickness - p2.m_thickness;
						p5.m_thickness = 2.0f * p4.m_thickness - p3.m_thickness;


						p0.m_color = glm::vec4(m_randomColors[flow.m_data.m_order], 1.0f);
						p1.m_color = glm::vec4(m_randomColors[flow.m_data.m_order], 1.0f);
						p2.m_color = glm::vec4(m_randomColors[flow.m_data.m_order], 1.0f);
						p3.m_color = glm::vec4(m_randomColors[flow.m_data.m_order], 1.0f);
						p4.m_color = glm::vec4(m_randomColors[flow.m_data.m_order], 1.0f);
						p5.m_color = glm::vec4(m_randomColors[flow.m_data.m_order], 1.0f);

						m_shootStemSegments[branchStartIndex * 3 + i * 3] = branchStartIndex * 6 + i * 6;
						m_shootStemSegments[branchStartIndex * 3 + i * 3 + 1] = branchStartIndex * 6 + i * 6 + 1;
						m_shootStemSegments[branchStartIndex * 3 + i * 3 + 2] = branchStartIndex * 6 + i * 6 + 2;
					}
					auto leafStartIndex = leafStartIndices[treeIndex];
					auto fruitStartIndex = fruitStartIndices[treeIndex];
					
					int leafIndex = 0;
					int fruitIndex = 0;

					for (const auto& internodeHandle : internodeList) {
						const auto& internode = branchSkeleton.PeekNode(internodeHandle);
						const auto& internodeData = internode.m_data;

						for (const auto& bud : internodeData.m_buds) {

							if (bud.m_status != BudStatus::Died) continue;
							if (bud.m_reproductiveModule.m_maturity <= 0.0f) continue;


							if (bud.m_type == BudType::Leaf) {

								foliageMatrices[leafStartIndex + leafIndex].m_instanceMatrix.m_value =
									entityGlobalTransform.m_value * bud.m_reproductiveModule.m_transform;
								foliageMatrices[leafStartIndex + leafIndex].m_instanceColor = glm::vec4(
									glm::mix(glm::vec3(152 / 255.0f, 203 / 255.0f, 0 / 255.0f),
										glm::vec3(159 / 255.0f, 100 / 255.0f, 66 / 255.0f),
										1.0f - bud.m_reproductiveModule.m_health), 1.0f);

								leafIndex++;
							}
							else if (bud.m_type == BudType::Fruit) {

								fruitMatrices[fruitStartIndex + fruitIndex].m_instanceMatrix.m_value =
									entityGlobalTransform.m_value * bud.m_reproductiveModule.m_transform;
								fruitMatrices[fruitStartIndex + fruitIndex].m_instanceColor = glm::vec4(255 / 255.0f, 165 / 255.0f,
									0 / 255.0f, 1.0f);

								fruitIndex++;
							}
						}
					}
					});
				StrandPointAttributes strandPointAttributes{};
				strandPointAttributes.m_normal = false;
				branchStrands->SetSegments(strandPointAttributes, m_shootStemSegments, m_shootStemPoints);
				m_foliageMatrices->SetPendingUpdate();
				m_fruitMatrices->SetPendingUpdate();
			}
		}
}

void EcoSysLabLayer::ClearGroundFruitAndLeaf() {
	m_fruits.clear();
	m_leaves.clear();
	UpdateGroundFruitAndLeaves();
}

void EcoSysLabLayer::UpdateGroundFruitAndLeaves() const {
	auto& fruitMatrices = m_groundFruitMatrices->m_particleInfos;
	fruitMatrices.resize(m_fruits.size());
	for (int i = 0; i < m_fruits.size(); i++) {
		fruitMatrices[i].m_instanceMatrix.m_value = m_fruits[i].m_globalTransform.m_value;
		fruitMatrices[i].m_instanceColor = glm::vec4(255 / 255.0f, 165 / 255.0f, 0 / 255.0f, 1.0f);
	}

	auto& leafMatrices = m_groundLeafMatrices->m_particleInfos;
	leafMatrices.resize(m_leaves.size());
	for (int i = 0; i < m_leaves.size(); i++) {
		leafMatrices[i].m_instanceMatrix.m_value = m_leaves[i].m_globalTransform.m_value;
		leafMatrices[i].m_instanceColor = glm::vec4(glm::mix(glm::vec3(152 / 255.0f, 203 / 255.0f, 0 / 255.0f),
			glm::vec3(159 / 255.0f, 100 / 255.0f, 66 / 255.0f),
			1.0f - m_leaves[i].m_health), 1.0f);
	}
	m_groundFruitMatrices->SetPendingUpdate();
	m_groundLeafMatrices->SetPendingUpdate();
}


void EcoSysLabLayer::SoilVisualization() {
	const auto soil = m_soilHolder.Get<Soil>();
	if (!soil) {
		m_soilHolder.Clear();
		return;
	}

	auto& soilModel = soil->m_soilModel;
	if (m_soilVersion != soilModel.m_version) {
		m_updateVectorMatrices = true;
		m_updateScalarMatrices = true;
		m_soilVersion = soilModel.m_version;
	}

	if (m_vectorEnable) {
		SoilVisualizationVector(soilModel);
	}
	if (m_scalarEnable) {
		SoilVisualizationScalar(soilModel);

	}
}


void EcoSysLabLayer::SoilVisualizationScalar(VoxelSoilModel& soilModel) {
	const auto numVoxels = soilModel.m_resolution.x * soilModel.m_resolution.y * soilModel.m_resolution.z;
	auto& scalarMatrices = m_groundFruitMatrices->m_particleInfos;

	if (scalarMatrices.size() != numVoxels) {
		scalarMatrices.resize(numVoxels);
		m_updateScalarMatrices = true;
	}
	if (m_updateScalarMatrices) {
		m_groundFruitMatrices->SetPendingUpdate();
		Jobs::ParallelFor(numVoxels, [&](unsigned i) {
			const auto coordinate = soilModel.GetCoordinateFromIndex(i);
			if (static_cast<float>(coordinate.x) / soilModel.m_resolution.x<
				m_soilCutoutXDepth || static_cast<float>(coordinate.z) / soilModel.m_resolution.z>(
					1.0f - m_soilCutoutZDepth)) {
				scalarMatrices[i].m_instanceMatrix.m_value =
					glm::mat4(0.0f);
			}
			else {
				scalarMatrices[i].m_instanceMatrix.m_value =
					glm::translate(soilModel.GetPositionFromCoordinate(coordinate))
					* glm::mat4_cast(glm::quat(glm::vec3(0.0f)))
					* glm::scale(glm::vec3(soilModel.GetVoxelSize() * m_scalarBoxSize));
			}
			});
		auto visualize_vec3 = [&](const Field& x, const Field& y, const Field& z) {
			Jobs::ParallelFor(numVoxels, [&](unsigned i) {
				const auto value = glm::vec3(x[i], y[i], z[i]);
				scalarMatrices[i].m_instanceColor = { glm::normalize(value),
														 glm::clamp(glm::length(value) * m_scalarMultiplier, m_scalarMinAlpha, 1.0f) };
				});
			};

		auto visualize_float = [&](const Field& v) {
			Jobs::ParallelFor(numVoxels, [&](unsigned i) {
				const auto value = glm::vec3(v[i]);
				scalarMatrices[i].m_instanceColor = { m_scalarBaseColor,
														 glm::clamp(glm::length(value) * m_scalarMultiplier, m_scalarMinAlpha, 1.0f) };
				});
			};


		switch (static_cast<SoilProperty>(m_scalarSoilProperty)) {
		case SoilProperty::Blank: {
			Jobs::ParallelFor(numVoxels, [&](unsigned i) {
				scalarMatrices[i].m_instanceColor = { m_scalarBaseColor, 0.01f };
				});
		}
								break;
		case SoilProperty::WaterDensity: {
			visualize_float(soilModel.m_w);
		}
									   break;
		case SoilProperty::NutrientDensity: {
			visualize_float(soilModel.m_n);
		}
										  break;
		case SoilProperty::SoilDensity: {
			visualize_float(soilModel.m_d);
		}
									  break;
		case SoilProperty::SoilLayer: {
			Jobs::ParallelFor(numVoxels, [&](unsigned i) {
				const auto layerIndex = soilModel.m_material_id[i];
				if (layerIndex == 0) scalarMatrices[i].m_instanceColor = glm::vec4(0.0f);
				else {
					scalarMatrices[i].m_instanceColor = m_soilLayerColors[layerIndex - 1];
				}
				});
		}
									break;
									/*case SoilProperty::DiffusionDivergence:
									{
										visualize_vec3(soilModel.m_div_diff_x, soilModel.m_div_diff_y, soilModel.m_div_diff_z);
									}break;*/
		default: {
			Jobs::ParallelFor(numVoxels, [&](unsigned i) {
				scalarMatrices[i].m_instanceColor = { m_scalarBaseColor, 0.01f };
				}
			);
		}
			   break;
		}
	}
	m_updateScalarMatrices = false;
	const auto editorLayer = Application::GetLayer<EditorLayer>();
	GizmoSettings gizmoSettings;
	gizmoSettings.m_drawSettings.m_blending = true;
	gizmoSettings.m_drawSettings.m_blendingSrcFactor = VK_BLEND_FACTOR_SRC_ALPHA;
	gizmoSettings.m_drawSettings.m_blendingDstFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
	gizmoSettings.m_drawSettings.m_cullMode = VK_CULL_MODE_NONE;
	editorLayer->DrawGizmoMeshInstancedColored(
		Resources::GetResource<Mesh>("PRIMITIVE_CUBE"),
		m_scalarMatrices,
		glm::mat4(1.0f), 1.0f, gizmoSettings);
}


void EcoSysLab::EcoSysLabLayer::SoilVisualizationVector(VoxelSoilModel& soilModel) {
	const auto numVoxels = soilModel.m_resolution.x * soilModel.m_resolution.y * soilModel.m_resolution.z;
	auto& vectorMatrices = m_groundFruitMatrices->m_particleInfos;
	if (vectorMatrices.size() != numVoxels) {
		vectorMatrices.resize(numVoxels);
		m_updateVectorMatrices = true;
	}
	if (m_updateVectorMatrices) {
		m_groundFruitMatrices->SetPendingUpdate();
		const auto actualVectorMultiplier = m_vectorMultiplier * soilModel.m_dx;
		switch (static_cast<SoilProperty>(m_vectorSoilProperty)) {
			/*
			case SoilProperty::WaterDensityGradient:
			{
				Jobs::ParallelFor(numVoxels, [&](unsigned i)
					{
						const auto targetVector = glm::vec3(soilModel.m_w_grad_x[i], soilModel.m_w_grad_y[i], soilModel.m_w_grad_z[i]);
						const auto start = soilModel.GetPositionFromCoordinate(soilModel.GetCoordinateFromIndex(i));
						const auto end = start + targetVector * actualVectorMultiplier;
						const auto direction = glm::normalize(end - start);
						glm::quat rotation = glm::quatLookAt(direction, glm::vec3(direction.y, direction.z, direction.x));
						rotation *= glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f));
						const auto length = glm::distance(end, start) / 2.0f;
						const auto width = glm::min(m_vectorLineMaxWidth, length * m_vectorLineWidthFactor);
						const auto model = glm::translate((start + end) / 2.0f) * glm::mat4_cast(rotation) *
							glm::scale(glm::vec3(width, length, width));
						vectorMatrices[i] = model;
					}, results);
			}break;*/
			/*
			case SoilProperty::Divergence:
			{
				Jobs::ParallelFor(numVoxels, [&](unsigned i)
					{
						const auto targetVector = glm::vec3(soilModel.m_div_diff_x[i], soilModel.m_div_diff_y[i], soilModel.m_div_diff_z[i]);
						const auto start = soilModel.GetPositionFromCoordinate(soilModel.GetCoordinateFromIndex(i));
						const auto end = start + targetVector * actualVectorMultiplier;
						const auto direction = glm::normalize(end - start);
						glm::quat rotation = glm::quatLookAt(direction, glm::vec3(direction.y, direction.z, direction.x));
						rotation *= glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f));
						const auto length = glm::distance(end, start) / 2.0f;
						const auto width = glm::min(m_vectorLineMaxWidth, length * m_vectorLineWidthFactor);
						const auto model = glm::translate((start + end) / 2.0f) * glm::mat4_cast(rotation) *
							glm::scale(glm::vec3(width, length, width));
						vectorMatrices[i] = model;
					}, results);
			}break;
			*/
		default: {
			Jobs::ParallelFor(numVoxels, [&](unsigned i) {
				vectorMatrices[i].m_instanceMatrix.m_value =
					glm::translate(soilModel.GetPositionFromCoordinate(soilModel.GetCoordinateFromIndex(i)))
					* glm::mat4_cast(glm::quat(glm::vec3(0.0f)))
					* glm::scale(glm::vec3(0.0f));
				});
		}
			   break;
		}
		std::vector<std::shared_future<void>> results;

		Jobs::ParallelFor(numVoxels, [&](unsigned i) {
			vectorMatrices[i].m_instanceColor = m_vectorBaseColor;
			}, results);

		for (auto& i : results) i.wait();
	}
	m_updateVectorMatrices = false;
	const auto editorLayer = Application::GetLayer<EditorLayer>();
	GizmoSettings gizmoSettings;
	gizmoSettings.m_drawSettings.m_blending = true;
	gizmoSettings.m_drawSettings.m_blendingSrcFactor = VK_BLEND_FACTOR_SRC_ALPHA;
	gizmoSettings.m_drawSettings.m_blendingDstFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
	gizmoSettings.m_drawSettings.m_cullMode = VK_CULL_MODE_BACK_BIT;

	editorLayer->DrawGizmoMeshInstancedColored(Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER"), m_vectorMatrices, glm::mat4(1.0f), 1.0f, gizmoSettings);
}

void EcoSysLabLayer::Update() {

	const auto scene = Application::GetActiveScene();
	const auto soilEntity = scene->GetEntity(m_soilHolder.GetEntityHandle());
	if (!scene->IsEntityValid(soilEntity) || !scene->HasPrivateComponent<Soil>(soilEntity)) {
		m_soilHolder.Clear();
		const std::vector<Entity>* soilEntities =
			scene->UnsafeGetPrivateComponentOwnersList<Soil>();
		if (soilEntities && !soilEntities->empty()) {
			m_soilHolder = scene->GetOrSetPrivateComponent<Soil>(soilEntities->at(0)).lock();
		}
	}
	const auto climateEntity = scene->GetEntity(m_climateHolder.GetEntityHandle());
	if (!scene->IsEntityValid(climateEntity) || !scene->HasPrivateComponent<Climate>(climateEntity)) {
		m_climateHolder.Clear();
		const std::vector<Entity>* climateEntities =
			scene->UnsafeGetPrivateComponentOwnersList<Climate>();
		if (climateEntities && !climateEntities->empty()) {
			m_climateHolder = scene->GetOrSetPrivateComponent<Climate>(climateEntities->at(0)).lock();
		}
	}
}

void EcoSysLabLayer::Simulate(float deltaTime) {
	const auto scene = GetScene();
	const std::vector<Entity>* treeEntities =
		scene->UnsafeGetPrivateComponentOwnersList<Tree>();
	m_time += deltaTime;
	if (treeEntities && !treeEntities->empty()) {
		float time = Times::Now();
		const auto climate = m_climateHolder.Get<Climate>();
		const auto soil = m_soilHolder.Get<Soil>();

		climate->m_climateModel.m_time = m_time;

		if (m_simulationSettings.m_soilSimulation) {
			soil->m_soilModel.Irrigation();
			soil->m_soilModel.Step();
		}
		auto& estimator = climate->m_climateModel.m_environmentGrid;
		estimator.m_settings = m_simulationSettings.m_shadowEstimationSettings;
		auto minBound = estimator.m_voxel.GetMinBound();
		auto maxBound = estimator.m_voxel.GetMaxBound();
		bool boundChanged = false;
		for (const auto& treeEntity : *treeEntities)
		{
			if (!scene->IsEntityEnabled(treeEntity)) continue;
			auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
			if (!tree->IsEnabled()) continue;
			const auto globalTransform = scene->GetDataComponent<GlobalTransform>(treeEntity).m_value;
			const glm::vec3 currentMinBound = globalTransform * glm::vec4(tree->m_treeModel.RefShootSkeleton().m_min, 1.0f);
			const glm::vec3 currentMaxBound = globalTransform * glm::vec4(tree->m_treeModel.RefShootSkeleton().m_max, 1.0f);

			if (currentMinBound.x <= minBound.x || currentMinBound.y <= minBound.y || currentMinBound.z <= minBound.z
				|| currentMaxBound.x >= maxBound.x || currentMaxBound.y >= maxBound.y || currentMaxBound.z >= maxBound.z) {
				minBound = glm::min(currentMinBound - glm::vec3(1.0f, 0.1f, 1.0f), minBound);
				maxBound = glm::max(currentMaxBound + glm::vec3(1.0f), maxBound);
				boundChanged = true;
			}
			if (!tree->m_climate.Get<Climate>()) tree->m_climate = climate;
			if (!tree->m_soil.Get<Soil>()) tree->m_soil = soil;
			tree->m_treeModel.m_crownShynessDistance = m_simulationSettings.m_crownShynessDistance;
		}
		if (boundChanged) estimator.m_voxel.Initialize(estimator.m_voxelSize, minBound, maxBound);
		estimator.m_voxel.Reset();
		for (const auto& treeEntity : *treeEntities)
		{
			if (!scene->IsEntityEnabled(treeEntity)) continue;
			auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
			if (!tree->IsEnabled()) continue;
			tree->RegisterVoxel();
		}

		estimator.ShadowPropagation();

		std::vector<std::shared_future<void>> results;
		Jobs::ParallelFor(treeEntities->size(), [&](unsigned i) {
			const auto treeEntity = treeEntities->at(i);
			if (!scene->IsEntityEnabled(treeEntity)) return;
			const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
			if (!tree->IsEnabled()) return;
			if(m_simulationSettings.m_maxNodeCount > 0 && tree->m_treeModel.RefShootSkeleton().RefSortedNodeList().size() >= m_simulationSettings.m_maxNodeCount) return;
			tree->TryGrow(deltaTime);
			}, results);
		for (auto& i : results) i.wait();

		auto heightField = soil->m_soilDescriptor.Get<SoilDescriptor>()->m_heightField.Get<HeightField>();
		for (const auto& treeEntity : *treeEntities) {
			if (!scene->IsEntityEnabled(treeEntity)) return;
			auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
			auto treeGlobalTransform = scene->GetDataComponent<GlobalTransform>(treeEntity);
			if (!tree->IsEnabled()) return;
			//Collect fruit and leaves here.
			if (!m_simulationSettings.m_autoClearFruitAndLeaves) {
				for (const auto& fruit : tree->m_treeModel.RefShootSkeleton().m_data.m_droppedFruits) {
					Fruit newFruit;
					newFruit.m_globalTransform.m_value = treeGlobalTransform.m_value * fruit.m_transform;

					auto position = newFruit.m_globalTransform.GetPosition();
					const auto groundHeight = heightField->GetValue({ position.x, position.z });
					const auto height = position.y - groundHeight;
					position.x += glm::gaussRand(0.0f, height * 0.1f);
					position.z += glm::gaussRand(0.0f, height * 0.1f);
					position.y = groundHeight + 0.1f;
					newFruit.m_globalTransform.SetPosition(position);

					newFruit.m_maturity = fruit.m_maturity;
					newFruit.m_health = fruit.m_health;
					m_fruits.emplace_back(newFruit);
				}


				for (const auto& leaf : tree->m_treeModel.RefShootSkeleton().m_data.m_droppedLeaves) {
					Leaf newLeaf;
					newLeaf.m_globalTransform.m_value = treeGlobalTransform.m_value * leaf.m_transform;

					auto position = newLeaf.m_globalTransform.GetPosition();
					const auto groundHeight = heightField ? heightField->GetValue({ position.x, position.z }) : 0.0f;
					const auto height = position.y - groundHeight;
					position.x += glm::gaussRand(0.0f, height * 0.1f);
					position.z += glm::gaussRand(0.0f, height * 0.1f);
					position.y = groundHeight + 0.1f;
					newLeaf.m_globalTransform.SetPosition(position);

					newLeaf.m_maturity = leaf.m_maturity;
					newLeaf.m_health = leaf.m_health;
					m_leaves.emplace_back(newLeaf);
				}
				tree->m_treeVisualizer.m_needUpdate = true;
			}
			tree->m_treeModel.RefShootSkeleton().m_data.m_droppedFruits.clear();
			tree->m_treeModel.RefShootSkeleton().m_data.m_droppedLeaves.clear();

		}
		m_lastUsedTime = Times::Now() - time;
		m_totalTime += m_lastUsedTime;
		m_needFullFlowUpdate = true;

		int totalInternodeSize = 0;
		int totalFlowSize = 0;
		int totalRootNodeSize = 0;
		int totalRootFlowSize = 0;
		int totalLeafSize = 0;
		int totalFruitSize = 0;
		for (int i = 0; i < treeEntities->size(); i++) {
			auto treeEntity = treeEntities->at(i);
			auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
			auto& treeModel = tree->m_treeModel;
			totalInternodeSize += treeModel.RefShootSkeleton().RefSortedNodeList().size();
			totalFlowSize += treeModel.RefShootSkeleton().RefSortedFlowList().size();
			totalLeafSize += treeModel.GetLeafCount();
			totalFruitSize += treeModel.GetFruitCount();
		}
		m_internodeSize = totalInternodeSize;
		m_shootStemSize = totalFlowSize;
		m_rootNodeSize = totalRootNodeSize;
		m_rootStemSize = totalRootFlowSize;
		m_leafSize = totalLeafSize;
		m_fruitSize = totalFruitSize;
	}
}

void EcoSysLabLayer::GenerateMeshes(const TreeMeshGeneratorSettings& meshGeneratorSettings) const {
	const auto scene = GetScene();
	const std::vector<Entity>* treeEntities =
		scene->UnsafeGetPrivateComponentOwnersList<Tree>();
	if (treeEntities && !treeEntities->empty()) {
		const auto copiedEntities = *treeEntities;
		for (auto treeEntity : copiedEntities) {
			const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
			tree->GenerateGeometry(meshGeneratorSettings);
		}
	}
}

void EcoSysLabLayer::ClearGeometries() const {
	const auto scene = GetScene();
	const std::vector<Entity>* treeEntities =
		scene->UnsafeGetPrivateComponentOwnersList<Tree>();
	if (treeEntities && !treeEntities->empty()) {
		const auto copiedEntities = *treeEntities;
		for (auto treeEntity : copiedEntities) {
			const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
			tree->ClearMeshes();
			tree->ClearStrands();
		}
	}
}

void EcoSysLabLayer::UpdateVisualizationCamera() {
	if (const auto editorLayer = Application::GetLayer<EditorLayer>(); !editorLayer) return;
	m_visualizationCamera->Resize({
		m_visualizationCameraResolutionX,
		m_visualizationCameraResolutionY });
}

void EcoSysLabLayer::PreUpdate()
{
	UpdateVisualizationCamera();
}
