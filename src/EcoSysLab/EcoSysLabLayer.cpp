//
// Created by lllll on 11/1/2022.
//

#include "EcoSysLabLayer.hpp"

#include "Climate.hpp"
#include "Soil.hpp"
#include "Tree.hpp"

using namespace EcoSysLab;

void EcoSysLabLayer::OnCreate() {
	if (m_randomColors.empty()) {
		for (int i = 0; i < 10000; i++) {
			m_randomColors.emplace_back(glm::linearRand(glm::vec3(0.0f), glm::vec3(1.0f)));
		}
	}

	auto compShaderCode =
		std::string("#version 450 core\n") + *DefaultResources::ShaderIncludes::Uniform + "\n" +
		FileUtils::LoadFileAsString(std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/TreeBranch.comp");

	m_treeBranchComp = ProjectManager::CreateTemporaryAsset<OpenGLUtils::GLShader>();
	m_treeBranchComp->Set(OpenGLUtils::ShaderType::Compute, compShaderCode);

	m_treeBranchComputeProgram = ProjectManager::CreateTemporaryAsset<OpenGLUtils::GLProgram>();
	m_treeBranchComputeProgram->Attach(m_treeBranchComp);
	m_treeBranchComputeProgram->Link();

	m_treeBranchBuffer = std::make_unique<OpenGLUtils::GLBuffer>(OpenGLUtils::GLBufferTarget::ShaderStorage, 0);
	m_treeMesh = ProjectManager::CreateTemporaryAsset<Mesh>();

	m_branchStrands = ProjectManager::CreateTemporaryAsset<Strands>();
	m_rootStrands = ProjectManager::CreateTemporaryAsset<Strands>();
}

void EcoSysLabLayer::OnDestroy() {

}
struct Branch
{
	glm::vec3 m_startPosition;
	float m_startThickness;
	glm::vec3 m_endPosition;
	float m_endThickness;
};
void EcoSysLabLayer::LateUpdate() {
	auto scene = GetScene();
	auto editorLayer = Application::GetLayer<EditorLayer>();
	auto selectedEntity = editorLayer->GetSelectedEntity();
	if (scene->IsEntityValid(selectedEntity) && selectedEntity != m_selectedTree && scene->HasPrivateComponent<Tree>(selectedEntity)) {
		m_needFlowUpdate = true;
		m_selectedTree = selectedEntity;
		if (scene->IsEntityValid(m_selectedTree)) m_treeVisualizer.Reset(scene->GetOrSetPrivateComponent<Tree>(m_selectedTree).lock()->m_treeModel);
	}
	const std::vector<Entity>* treeEntities =
		scene->UnsafeGetPrivateComponentOwnersList<Tree>();

	auto branchStrands = m_branchStrands.Get<Strands>();
	auto rootStrands = m_rootStrands.Get<Strands>();
	if (treeEntities && !treeEntities->empty()) {
		//Tree selection
		if (editorLayer->SceneCameraWindowFocused() && !m_lockTreeSelection
			&& editorLayer->GetLockEntitySelection() && Inputs::GetMouseInternal(GLFW_MOUSE_BUTTON_LEFT, Windows::GetWindow())) {
			glm::vec2 mousePosition = editorLayer->GetMouseScreenPosition();
#pragma region Ray selection
			std::mutex writeMutex;
			float minDistance = FLT_MAX;
			GlobalTransform cameraLtw;
			cameraLtw.m_value =
				glm::translate(
					editorLayer->m_sceneCameraPosition) *
				glm::mat4_cast(
					editorLayer->m_sceneCameraRotation);
			const Ray cameraRay = editorLayer->m_sceneCamera->ScreenPointToRay(
				cameraLtw, mousePosition);
			std::vector<std::shared_future<void>> results;
			bool detected = false;
			Entity currentFocusingTree;
			Jobs::ParallelFor(treeEntities->size(), [&](unsigned i)
				{
					const auto treeEntity = treeEntities->at(i);
			auto globalTransform = scene->GetDataComponent<GlobalTransform>(treeEntity);
			auto tree = scene->GetOrSetPrivateComponent<Tree>(
				treeEntity).lock();
			auto branchSkeleton = tree->m_treeModel.RefBranchSkeleton();
			auto rootSkeleton = tree->m_treeModel.RefBranchSkeleton();
			Bound branchSkeletonBound;
			branchSkeletonBound.m_min = branchSkeleton.m_min;
			branchSkeletonBound.m_max = branchSkeleton.m_max;
			Bound rootSkeletonBound;
			rootSkeletonBound.m_min = rootSkeleton.m_min;
			rootSkeletonBound.m_max = rootSkeleton.m_max;
			if (!cameraRay.Intersect(globalTransform.m_value, branchSkeletonBound)
				&& !cameraRay.Intersect(globalTransform.m_value, rootSkeletonBound)) return;
			auto distance = glm::distance(globalTransform.GetPosition(),
				glm::vec3(cameraLtw.m_value[3]));
			std::lock_guard lock(writeMutex);
			if (distance < minDistance) {
				minDistance = distance;
				currentFocusingTree = treeEntity;
				detected = true;
			}
				}, results);
			for (auto& i : results) i.wait();
			if (detected && currentFocusingTree != m_selectedTree && scene->IsEntityValid(currentFocusingTree)) {
				editorLayer->SetSelectedEntity(currentFocusingTree);
				m_selectedTree = currentFocusingTree;
				if (scene->IsEntityValid(m_selectedTree)) m_treeVisualizer.Reset(scene->GetOrSetPrivateComponent<Tree>(m_selectedTree).lock()->m_treeModel);
				m_needFlowUpdate = true;
			}
#pragma endregion

		}
		if (m_rendering)
		{
			std::vector<Branch> branches;
			for (int i = 0; i < treeEntities->size(); i++) {
				auto treeEntity = treeEntities->at(i);
				auto treeTransform = scene->GetDataComponent<GlobalTransform>(treeEntity);
				auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
				auto& treeModel = tree->m_treeModel;
				auto& skeleton = treeModel.RefBranchSkeleton();
				for (auto& handle : skeleton.RefSortedFlowList())
				{
					auto& flow = skeleton.PeekFlow(handle);
					glm::vec3 start = (treeTransform.m_value * glm::translate(flow.m_info.m_globalStartPosition))[3];
					glm::vec3 end = (treeTransform.m_value * glm::translate(flow.m_info.m_globalEndPosition))[3];
					branches.push_back({
						start, flow.m_info.m_startThickness,
						end, flow.m_info.m_endThickness
						});
				}
			}
			m_treeBranchBuffer->SetData(branches.size() * sizeof(Branch), branches.data(), GL_DYNAMIC_READ);
			const auto res = 12;
			const auto numVertices = branches.size() * (res + 1) * 2;
			const auto numTriangles = branches.size() * (res * 2 * 2);
			auto vao = m_treeMesh->Vao();
			vao->Vbo().SetData((GLsizei)(numVertices * sizeof(Vertex)), nullptr, GL_DYNAMIC_DRAW);
			vao->Ebo().SetData((GLsizei)numTriangles * sizeof(glm::uvec3), nullptr, GL_DYNAMIC_DRAW);

			m_treeBranchBuffer->Bind();
			vao->Vbo().SetTargetBase(OpenGLUtils::GLBufferTarget::ShaderStorage, 1);
			vao->Vbo().Bind();
			vao->Ebo().SetTargetBase(OpenGLUtils::GLBufferTarget::ShaderStorage, 2);
			vao->Ebo().Bind();


			m_treeBranchComputeProgram->DispatchCompute(glm::uvec3(branches.size(), 1, 1));
			OpenGLUtils::InsertMemoryBarrier(GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT);
			OpenGLUtils::InsertMemoryBarrier(GL_ELEMENT_ARRAY_BARRIER_BIT);
			m_treeMesh->UnsafeSetTrianglesAmount(numTriangles);
			m_treeMesh->UnsafeSetVerticesAmount(numVertices);
			vao->Vbo().SetTarget(OpenGLUtils::GLBufferTarget::Array);
			vao->Ebo().SetTarget(OpenGLUtils::GLBufferTarget::ElementArray);
			vao->Bind();
			vao->Vbo().Bind();
			vao->Ebo().Bind();
#pragma region AttributePointer
			vao->EnableAttributeArray(0);
			vao->SetAttributePointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), 0);
			vao->EnableAttributeArray(1);
			vao->SetAttributePointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)(offsetof(Vertex, m_normal)));
			vao->EnableAttributeArray(2);
			vao->SetAttributePointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)(offsetof(Vertex, m_tangent)));
			vao->EnableAttributeArray(3);
			vao->SetAttributePointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)(offsetof(Vertex, m_color)));

			vao->EnableAttributeArray(4);
			vao->SetAttributePointer(4, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)(offsetof(Vertex, m_texCoord)));
#pragma endregion
			Gizmos::DrawGizmoMesh(m_treeMesh);
		}
		if (m_debugVisualization) {
			if (m_versions.size() != treeEntities->size()) {
				m_internodeSize = 0;
				m_totalTime = 0.0f;
				m_versions.clear();
				for (int i = 0; i < treeEntities->size(); i++) {
					m_versions.emplace_back(-1);
				}
				m_needFlowUpdate = true;
			}
			int totalInternodeSize = 0;
			int totalFlowSize = 0;
			for (int i = 0; i < treeEntities->size(); i++) {
				auto treeEntity = treeEntities->at(i);
				auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
				auto& treeModel = tree->m_treeModel;
				totalInternodeSize += treeModel.RefBranchSkeleton().RefSortedNodeList().size();
				totalFlowSize += treeModel.RefBranchSkeleton().RefSortedFlowList().size();
				if (m_selectedTree == treeEntity) continue;
				if (m_versions[i] != treeModel.RefBranchSkeleton().GetVersion()) {
					m_versions[i] = treeModel.RefBranchSkeleton().GetVersion();
					m_needFlowUpdate = true;
				}
			}
			m_internodeSize = totalInternodeSize;
			m_branchSize = totalFlowSize;
			if (m_needFlowUpdate) {
				m_needFlowUpdate = false;
				m_boundingBoxMatrices.clear();
				m_boundingBoxColors.clear();

				std::vector<int> branchStartIndices;
				int branchLastStartIndex = 0;
				branchStartIndices.emplace_back(branchLastStartIndex);

				std::vector<int> rootStartIndices;
				int rootLastStartIndex = 0;
				rootStartIndices.emplace_back(rootLastStartIndex);

				for (int listIndex = 0; listIndex < treeEntities->size(); listIndex++) {
					auto treeEntity = treeEntities->at(listIndex);
					auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
					auto& treeModel = tree->m_treeModel;
					const auto& branchSkeleton = treeModel.RefBranchSkeleton();
					const auto& rootSkeleton = treeModel.RefRootSkeleton();
					const auto& branchList = branchSkeleton.RefSortedFlowList();
					const auto& rootList = rootSkeleton.RefSortedFlowList();

					auto entityGlobalTransform = scene->GetDataComponent<GlobalTransform>(treeEntity);
					m_boundingBoxMatrices.emplace_back();
					m_boundingBoxMatrices.back() = entityGlobalTransform.m_value *
						(glm::translate(
							(branchSkeleton.m_max + branchSkeleton.m_min) / 2.0f) *
							glm::scale((branchSkeleton.m_max - branchSkeleton.m_min) / 2.0f));
					m_boundingBoxColors.emplace_back();
					m_boundingBoxColors.back() = glm::vec4(m_randomColors[listIndex], 0.05f);
					branchLastStartIndex += branchList.size();
					branchStartIndices.emplace_back(branchLastStartIndex);

					rootLastStartIndex += rootList.size();
					rootStartIndices.emplace_back(rootLastStartIndex);
				}
				m_branchSegments.resize(branchLastStartIndex * 3);
				m_branchPoints.resize(branchLastStartIndex * 6);

				m_rootSegments.resize(rootLastStartIndex * 3);
				m_rootPoints.resize(rootLastStartIndex * 6);
				{
					std::vector<std::shared_future<void>> results;
					Jobs::ParallelFor(treeEntities->size(), [&](unsigned treeIndex)
						{
							auto treeEntity = treeEntities->at(treeIndex);
					auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
					auto& treeModel = tree->m_treeModel;
					const auto& branchSkeleton = treeModel.RefBranchSkeleton();
					const auto& rootSkeleton = treeModel.RefRootSkeleton();
					const auto& branchList = branchSkeleton.RefSortedFlowList();
					const auto& rootList = rootSkeleton.RefSortedFlowList();

					auto entityGlobalTransform = scene->GetDataComponent<GlobalTransform>(treeEntity);
					auto branchStartIndex = branchStartIndices[treeIndex];
					for (int i = 0; i < branchList.size(); i++)
					{
						auto& flow = branchSkeleton.PeekFlow(branchList[i]);
						auto cp1 = flow.m_info.m_globalStartPosition;
						auto cp4 = flow.m_info.m_globalEndPosition;
						float distance = glm::distance(cp1, cp4);
						glm::vec3 cp0, cp2;
						if (flow.GetParentHandle() > 0)
						{
							cp0 = cp1 + branchSkeleton.PeekFlow(flow.GetParentHandle()).m_info.m_globalEndRotation * glm::vec3(0, 0, 1) * distance / 4.0f;
							cp2 = cp1 + branchSkeleton.PeekFlow(flow.GetParentHandle()).m_info.m_globalEndRotation * glm::vec3(0, 0, -1) * distance / 4.0f;
						}
						else
						{
							cp0 = cp1 + flow.m_info.m_globalStartRotation * glm::vec3(0, 0, 1) * distance / 4.0f;
							cp2 = cp1 + flow.m_info.m_globalStartRotation * glm::vec3(0, 0, -1) * distance / 4.0f;
						}
						auto cp3 = cp4 + flow.m_info.m_globalEndRotation * glm::vec3(0, 0, 1) * distance / 4.0f;
						auto cp5 = cp4 + flow.m_info.m_globalEndRotation * glm::vec3(0, 0, -1) * distance / 4.0f;

						auto& p0 = m_branchPoints[branchStartIndex * 6 + i * 6];
						auto& p1 = m_branchPoints[branchStartIndex * 6 + i * 6 + 1];
						auto& p2 = m_branchPoints[branchStartIndex * 6 + i * 6 + 2];
						auto& p3 = m_branchPoints[branchStartIndex * 6 + i * 6 + 3];
						auto& p4 = m_branchPoints[branchStartIndex * 6 + i * 6 + 4];
						auto& p5 = m_branchPoints[branchStartIndex * 6 + i * 6 + 5];
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

						if (flow.GetParentHandle() > 0)
						{
							p1.m_thickness = branchSkeleton.PeekFlow(flow.GetParentHandle()).m_info.m_endThickness;
						}
						else
						{
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

						m_branchSegments[branchStartIndex * 3 + i * 3] = branchStartIndex * 6 + i * 6;
						m_branchSegments[branchStartIndex * 3 + i * 3 + 1] = branchStartIndex * 6 + i * 6 + 1;
						m_branchSegments[branchStartIndex * 3 + i * 3 + 2] = branchStartIndex * 6 + i * 6 + 2;
					}
					auto rootStartIndex = rootStartIndices[treeIndex];
					for (int i = 0; i < rootList.size(); i++)
					{
						auto& flow = rootSkeleton.PeekFlow(rootList[i]);
						auto cp1 = flow.m_info.m_globalStartPosition;
						auto cp4 = flow.m_info.m_globalEndPosition;
						float distance = glm::distance(cp1, cp4);
						glm::vec3 cp0, cp2;
						if (flow.GetParentHandle() > 0)
						{
							cp0 = cp1 + rootSkeleton.PeekFlow(flow.GetParentHandle()).m_info.m_globalEndRotation * glm::vec3(0, 0, 1) * distance / 4.0f;
							cp2 = cp1 + rootSkeleton.PeekFlow(flow.GetParentHandle()).m_info.m_globalEndRotation * glm::vec3(0, 0, -1) * distance / 4.0f;
						}
						else
						{
							cp0 = cp1 + flow.m_info.m_globalStartRotation * glm::vec3(0, 0, 1) * distance / 4.0f;
							cp2 = cp1 + flow.m_info.m_globalStartRotation * glm::vec3(0, 0, -1) * distance / 4.0f;
						}
						auto cp3 = cp4 + flow.m_info.m_globalEndRotation * glm::vec3(0, 0, 1) * distance / 4.0f;
						auto cp5 = cp4 + flow.m_info.m_globalEndRotation * glm::vec3(0, 0, -1) * distance / 4.0f;

						auto& p0 = m_rootPoints[rootStartIndex * 6 + i * 6];
						auto& p1 = m_rootPoints[rootStartIndex * 6 + i * 6 + 1];
						auto& p2 = m_rootPoints[rootStartIndex * 6 + i * 6 + 2];
						auto& p3 = m_rootPoints[rootStartIndex * 6 + i * 6 + 3];
						auto& p4 = m_rootPoints[rootStartIndex * 6 + i * 6 + 4];
						auto& p5 = m_rootPoints[rootStartIndex * 6 + i * 6 + 5];
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

						if (flow.GetParentHandle() > 0)
						{
							p0.m_thickness = p1.m_thickness = p2.m_thickness = rootSkeleton.PeekFlow(flow.GetParentHandle()).m_info.m_endThickness;
						}
						else
						{
							p0.m_thickness = p1.m_thickness = p2.m_thickness = flow.m_info.m_startThickness;
						}
						p3.m_thickness = flow.m_info.m_endThickness;
						p4.m_thickness = flow.m_info.m_endThickness;
						p5.m_thickness = flow.m_info.m_endThickness;

						p0.m_color = glm::vec4(m_randomColors[flow.m_data.m_order], 1.0f);
						p1.m_color = glm::vec4(m_randomColors[flow.m_data.m_order], 1.0f);
						p2.m_color = glm::vec4(m_randomColors[flow.m_data.m_order], 1.0f);
						p3.m_color = glm::vec4(m_randomColors[flow.m_data.m_order], 1.0f);
						p4.m_color = glm::vec4(m_randomColors[flow.m_data.m_order], 1.0f);
						p5.m_color = glm::vec4(m_randomColors[flow.m_data.m_order], 1.0f);

						m_rootSegments[rootStartIndex * 3 + i * 3] = rootStartIndex * 6 + i * 6;
						m_rootSegments[rootStartIndex * 3 + i * 3 + 1] = rootStartIndex * 6 + i * 6 + 1;
						m_rootSegments[rootStartIndex * 3 + i * 3 + 2] = rootStartIndex * 6 + i * 6 + 2;
					}
						}, results);
					for (auto& i : results) i.wait();
					branchStrands->SetSegments(1, m_branchSegments, m_branchPoints);
					rootStrands->SetSegments(1, m_rootSegments, m_rootPoints);
				}


			}
		}

		auto strandsHolder = m_branchStrandsHolder.Get();
		if (scene->IsEntityValid(strandsHolder))
		{
			auto branchStrandsRenderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(strandsHolder).lock();
			branchStrandsRenderer->m_strands = m_branchStrands;
			auto material = branchStrandsRenderer->m_material.Get<Material>();
			if (!material)
			{
				material = ProjectManager::CreateTemporaryAsset<Material>();
				material->SetProgram(DefaultResources::GLPrograms::StandardStrandsProgram);
				branchStrandsRenderer->m_material = material;
				material->m_materialProperties.m_albedoColor = glm::vec4(188 / 255.0f, 143 / 255.0f, 143 / 255.0f, 1.0f);
			}
		}

		auto rootsHolder = m_rootStrandsHolder.Get();
		if (scene->IsEntityValid(rootsHolder))
		{
			auto rootStrandsRenderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(rootsHolder).lock();
			rootStrandsRenderer->m_strands = m_rootStrands;
			auto material = rootStrandsRenderer->m_material.Get<Material>();
			if (!material)
			{
				material = ProjectManager::CreateTemporaryAsset<Material>();
				material->SetProgram(DefaultResources::GLPrograms::StandardStrandsProgram);
				rootStrandsRenderer->m_material = material;
				material->m_materialProperties.m_albedoColor = glm::vec4(255 / 255.0f, 248 / 255.0f, 220 / 255.0f, 1.0f);
			}
		}
	}

	if (m_debugVisualization) {
		GizmoSettings gizmoSettings;
		gizmoSettings.m_drawSettings.m_blending = true;
		gizmoSettings.m_drawSettings.m_cullFace = false;
		if (m_displayBoundingBox && !m_boundingBoxMatrices.empty()) {
			Gizmos::DrawGizmoMeshInstancedColored(
				DefaultResources::Primitives::Cube, editorLayer->m_sceneCamera,
				editorLayer->m_sceneCameraPosition,
				editorLayer->m_sceneCameraRotation,
				m_boundingBoxColors,
				m_boundingBoxMatrices,
				glm::mat4(1.0f), 1.0f, gizmoSettings);
		}

		if (m_displayBranches && !m_branchPoints.empty()) {
			gizmoSettings.m_colorMode = GizmoSettings::ColorMode::VertexColor;
			Gizmos::DrawGizmoStrands(branchStrands, glm::vec4(1.0f), glm::mat4(1.0f), 1, gizmoSettings);
		}

		if (m_displayRootFlows && !m_rootPoints.empty())
		{
			gizmoSettings.m_colorMode = GizmoSettings::ColorMode::VertexColor;
			Gizmos::DrawGizmoStrands(rootStrands, glm::vec4(1.0f), glm::mat4(1.0f), 1, gizmoSettings);
		}
		auto soil = m_soilHolder.Get<Soil>();
		if (m_displaySoil && soil)
		{
			auto soilModel = soil->m_soilModel;
			if (m_soilVersion != soilModel.m_version)
			{
				m_updateVectorMatrices = m_updateVectorColors = true;
				m_updateScalarMatrices = m_updateScalarColors = true;
				m_soilVersion = soilModel.m_version;
			}

			const auto numVoxels = soilModel.m_resolution.x * soilModel.m_resolution.y * soilModel.m_resolution.z;
			if (m_vectorEnable) {
				if (m_vectorMatrices.size() != numVoxels || m_vectorColors.size() != numVoxels)
				{
					m_vectorMatrices.resize(numVoxels);
					m_vectorColors.resize(numVoxels);
					m_updateVectorMatrices = m_updateVectorColors = true;
				}
				if (m_updateVectorMatrices)
				{
					std::vector<std::shared_future<void>> results;
					const auto actualVectorMultiplier = m_vectorMultiplier * soilModel.m_dx;
					switch (static_cast<SoilProperty>(m_vectorSoilProperty))
					{
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
						m_vectorMatrices[i] = model;
							}, results);
					}break;
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
						m_vectorMatrices[i] = model;
							}, results);
					}break;
					default:
					{
						Jobs::ParallelFor(numVoxels, [&](unsigned i)
							{
								m_vectorMatrices[i] =
								glm::translate(soilModel.GetPositionFromCoordinate(soilModel.GetCoordinateFromIndex(i)))
							* glm::mat4_cast(glm::quat(glm::vec3(0.0f)))
							* glm::scale(glm::vec3(0.0f));
							}, results);
					}break;
					}
					for (auto& i : results) i.wait();
				}
				if (m_updateVectorColors) {
					std::vector<std::shared_future<void>> results;

					Jobs::ParallelFor(numVoxels, [&](unsigned i)
						{
							m_vectorColors[i] = m_vectorBaseColor;
						}, results);

					for (auto& i : results) i.wait();
				}
				m_updateVectorMatrices = false;
				m_updateVectorColors = false;
				auto editorLayer = Application::GetLayer<EditorLayer>();
				GizmoSettings gizmoSettings;
				gizmoSettings.m_drawSettings.m_blending = true;
				gizmoSettings.m_drawSettings.m_blendingSrcFactor = OpenGLBlendFactor::SrcAlpha;
				gizmoSettings.m_drawSettings.m_blendingDstFactor = OpenGLBlendFactor::OneMinusSrcAlpha;
				gizmoSettings.m_drawSettings.m_cullFace = true;

				Gizmos::DrawGizmoMeshInstancedColored(
					DefaultResources::Primitives::Cylinder, editorLayer->m_sceneCamera,
					editorLayer->m_sceneCameraPosition,
					editorLayer->m_sceneCameraRotation,
					m_vectorColors,
					m_vectorMatrices,
					glm::mat4(1.0f), 1.0f, gizmoSettings);
			}
			if (m_scalarEnable) {
				if (m_scalarMatrices.size() != numVoxels || m_scalarColors.size() != numVoxels)
				{
					m_scalarMatrices.resize(numVoxels);
					m_scalarColors.resize(numVoxels);
					m_updateScalarMatrices = m_updateScalarColors = true;
				}
				if (m_updateScalarMatrices)
				{
					std::vector<std::shared_future<void>> results;
					Jobs::ParallelFor(numVoxels, [&](unsigned i)
						{
							m_scalarMatrices[i] =
							glm::translate(soilModel.GetPositionFromCoordinate(soilModel.GetCoordinateFromIndex(i)))
						* glm::mat4_cast(glm::quat(glm::vec3(0.0f)))
						* glm::scale(glm::vec3(soilModel.GetVoxelSize() * m_scalarBoxSize));
						}, results);
					for (auto& i : results) i.wait();
				}
				if (m_updateScalarColors) {
					std::vector<std::shared_future<void>> results;
					switch (static_cast<SoilProperty>(m_scalarSoilProperty))
					{
					case SoilProperty::Blank:
					{
						Jobs::ParallelFor(numVoxels, [&](unsigned i)
							{
								m_scalarColors[i] = { m_scalarBaseColor, 0.01f };
							}, results);
					}break;
					case SoilProperty::SoilDensity:
					{
						Jobs::ParallelFor(numVoxels, [&](unsigned i)
							{
								auto value = glm::vec3(soilModel.m_soilDensity[i]);
						m_scalarColors[i] = { m_scalarBaseColor, glm::clamp(glm::length(value) * m_scalarMultiplier, m_scalarMinAlpha, 1.0f) };
							}, results);
					}break;
					case SoilProperty::WaterDensity:
					{
						Jobs::ParallelFor(numVoxels, [&](unsigned i)
							{
								auto value = glm::vec3(soilModel.m_w[i]);
						m_scalarColors[i] = { m_scalarBaseColor, glm::clamp(glm::length(value) * m_scalarMultiplier, m_scalarMinAlpha, 1.0f) };
							}, results);
					}break;
					case SoilProperty::WaterDensityGradient:
					{
						Jobs::ParallelFor(numVoxels, [&](unsigned i)
							{
								auto value = glm::vec3(soilModel.m_w_grad_x[i], soilModel.m_w_grad_y[i], soilModel.m_w_grad_z[i]);
						m_scalarColors[i] = { glm::normalize(value), glm::clamp(glm::length(value) * m_scalarMultiplier, m_scalarMinAlpha, 1.0f) };
							}, results);
					}break;
					case SoilProperty::Divergence:
					{
						Jobs::ParallelFor(numVoxels, [&](unsigned i)
							{
								auto value = glm::vec3(soilModel.m_div_diff_x[i], soilModel.m_div_diff_y[i], soilModel.m_div_diff_z[i]);
						m_scalarColors[i] = { glm::normalize(value), glm::clamp(glm::length(value) * m_scalarMultiplier, m_scalarMinAlpha, 1.0f) };
							}, results);
					}break;
					case SoilProperty::NutrientDensity:
					{
						Jobs::ParallelFor(numVoxels, [&](unsigned i)
							{
								auto value = glm::vec3(glm::vec3(soilModel.m_nutrientsDensity[i]));
						m_scalarColors[i] = { m_scalarBaseColor, glm::clamp(glm::length(value) * m_scalarMultiplier, m_scalarMinAlpha, 1.0f) };
							}, results);
					}break;
					default:
					{
						Jobs::ParallelFor(numVoxels, [&](unsigned i)
							{
								m_scalarColors[i] = glm::vec4(0.0f);
							}, results);
					}break;
					}
					for (auto& i : results) i.wait();
				}
				m_updateScalarMatrices = false;
				m_updateScalarColors = false;
				auto editorLayer = Application::GetLayer<EditorLayer>();
				GizmoSettings gizmoSettings;
				gizmoSettings.m_drawSettings.m_blending = true;
				gizmoSettings.m_drawSettings.m_blendingSrcFactor = OpenGLBlendFactor::SrcAlpha;
				gizmoSettings.m_drawSettings.m_blendingDstFactor = OpenGLBlendFactor::OneMinusSrcAlpha;
				gizmoSettings.m_drawSettings.m_cullFace = true;
				Gizmos::DrawGizmoMeshInstancedColored(
					DefaultResources::Primitives::Cube, editorLayer->m_sceneCamera,
					editorLayer->m_sceneCameraPosition,
					editorLayer->m_sceneCameraRotation,
					m_scalarColors,
					m_scalarMatrices,
					glm::mat4(1.0f), 1.0f, gizmoSettings);
			}
		}

	}
}

void EcoSysLabLayer::OnInspect() {
	if (ImGui::Begin("EcoSysLab Layer")) {
		auto scene = GetScene();
		ImGui::Checkbox("Lock tree selection", &m_lockTreeSelection);
		const std::vector<Entity>* treeEntities =
			scene->UnsafeGetPrivateComponentOwnersList<Tree>();
		if (ImGui::TreeNodeEx("Mesh generation")) {
			m_meshGeneratorSettings.OnInspect();
			if (ImGui::Button("Generate Meshes")) {
				GenerateMeshes(m_meshGeneratorSettings);
			}

			Editor::DragAndDropButton(m_branchStrandsHolder, "Branch strands holder");
			Editor::DragAndDropButton(m_rootStrandsHolder, "Root strands holder");
			ImGui::TreePop();
		}

		Editor::DragAndDropButton<Soil>(m_soilHolder, "Soil");
		Editor::DragAndDropButton<Climate>(m_climateHolder, "Climate");

		if (treeEntities && !treeEntities->empty()) {
			ImGui::Checkbox("Auto grow", &m_autoGrow);
			if (!m_autoGrow) {
				bool changed = false;
				if (ImGui::Button("Grow all")) {
					GrowAllTrees();
					changed = true;
				}
				static int iterations = 5;
				ImGui::DragInt("Iterations", &iterations, 1, 1, 100);
				if (ImGui::Button(("Grow all " + std::to_string(iterations) + " iterations").c_str())) {
					for (int i = 0; i < iterations; i++) GrowAllTrees();
					changed = true;
				}
				if (changed) {
					if (scene->IsEntityValid(m_selectedTree)) {
						m_treeVisualizer.m_iteration = scene->GetOrSetPrivateComponent<Tree>(m_selectedTree).lock()->m_treeModel.CurrentIteration();
						m_treeVisualizer.m_needUpdate = true;
					}
				}
			}
			ImGui::Text("Growth time: %.4f", m_lastUsedTime);
			ImGui::Text("Total time: %.4f", m_totalTime);
			ImGui::Text("Tree count: %d", treeEntities->size());
			ImGui::Text("Total Internode size: %d", m_internodeSize);
			ImGui::Text("Total Branch size: %d", m_branchSize);
			ImGui::Text("Total Root node size: %d", m_rootNodeSize);
			ImGui::Text("Total Root Flow size: %d", m_rootFlowSize);
		}
		ImGui::Checkbox("Debug Visualization", &m_debugVisualization);
		if (m_debugVisualization && ImGui::TreeNodeEx("Debug visualization settings", ImGuiTreeNodeFlags_DefaultOpen)) {
			ImGui::Checkbox("Display Branches", &m_displayBranches);
			ImGui::Checkbox("Display Root Flows", &m_displayRootFlows);
			ImGui::Checkbox("Display Soil", &m_displaySoil);
			if (m_displaySoil && ImGui::TreeNodeEx("Soil visualization settings", ImGuiTreeNodeFlags_DefaultOpen))
			{
				OnSoilVisualizationMenu();
				ImGui::TreePop();
			}
			ImGui::Checkbox("Display Bounding Box", &m_displayBoundingBox);
			ImGui::TreePop();
		}

		if (m_debugVisualization && scene->IsEntityValid(m_selectedTree)) {
			m_treeVisualizer.OnInspect(
				scene->GetOrSetPrivateComponent<Tree>(m_selectedTree).lock()->m_treeModel, scene->GetDataComponent<GlobalTransform>(m_selectedTree));
		}
	}
	ImGui::End();
}

void EcoSysLabLayer::OnSoilVisualizationMenu()
{
	static bool forceUpdate;
	ImGui::Checkbox("Force Update", &forceUpdate);

	if (ImGui::Checkbox("Vector Visualization", &m_vectorEnable))
	{
		if (m_vectorEnable) m_updateVectorMatrices = m_updateVectorColors = true;
	}

	if (ImGui::Checkbox("Scalar Visualization", &m_scalarEnable))
	{
		if (m_scalarEnable) m_updateScalarMatrices = m_updateScalarColors = true;
	}

	if (m_vectorEnable) {
		m_updateVectorMatrices = m_updateVectorMatrices || forceUpdate;
		m_updateVectorColors = m_updateVectorColors || forceUpdate;

		if (ImGui::TreeNodeEx("Vector", ImGuiTreeNodeFlags_DefaultOpen))
		{
			if (ImGui::Button("Reset"))
			{
				m_vectorMultiplier = 50.0f;
				m_vectorBaseColor = glm::vec4(1.0f, 1.0f, 1.0f, 0.8f);
				m_vectorSoilProperty = 4;
				m_vectorLineWidthFactor = 0.1f;
				m_vectorLineMaxWidth = 0.1f;
				m_updateVectorColors = m_updateVectorMatrices = true;
			}
			if (ImGui::ColorEdit4("Vector Base Color", &m_vectorBaseColor.x))
			{
				m_updateVectorColors = true;
			}
			if (ImGui::DragFloat("Multiplier", &m_vectorMultiplier, 0.1f, 0.0f, 100.0f, "%.3f"))
			{
				m_updateVectorMatrices = true;
			}
			if (ImGui::DragFloat("Line Width Factor", &m_vectorLineWidthFactor, 0.01f, 0.0f, 5.0f))
			{
				m_updateVectorMatrices = true;
			}
			if (ImGui::DragFloat("Max Line Width", &m_vectorLineMaxWidth, 0.01f, 0.0f, 5.0f))
			{
				m_updateVectorMatrices = true;
			}
			if (ImGui::Combo("Vector Mode", { "N/A", "N/A", "N/A", "N/A", "Water Density Gradient", "Flux", "Divergence", "N/A", "N/A" }, m_vectorSoilProperty))
			{
				m_updateVectorMatrices = true;
			}
			ImGui::TreePop();
		}
	}
	if (m_scalarEnable) {
		m_updateScalarMatrices = m_updateScalarMatrices || forceUpdate;
		m_updateScalarColors = m_updateScalarColors || forceUpdate;

		if (m_scalarEnable && ImGui::TreeNodeEx("Scalar", ImGuiTreeNodeFlags_DefaultOpen))
		{
			if (ImGui::Button("Reset"))
			{
				m_scalarMultiplier = 1.0f;
				m_scalarBoxSize = 0.5f;
				m_scalarMinAlpha = 0.00f;
				m_scalarBaseColor = glm::vec3(0.0f, 0.0f, 1.0f);
				m_scalarSoilProperty = 1;
				m_updateScalarColors = m_updateScalarMatrices = true;
			}
			if (ImGui::ColorEdit3("Scalar Base Color", &m_scalarBaseColor.x))
			{
				m_updateScalarColors = true;
			}
			if (ImGui::DragFloat("Multiplier", &m_scalarMultiplier, 0.001f, 0.0f, 10.0f, "%.5f"))
			{
				m_updateScalarColors = true;
			}
			if (ImGui::DragFloat("Min alpha", &m_scalarMinAlpha, 0.001f, 0.0f, 1.0f))
			{
				m_updateScalarColors = true;
			}
			if (ImGui::DragFloat("Box size", &m_scalarBoxSize, 0.001f, 0.0f, 1.0f))
			{
				m_updateScalarMatrices = true;
			}
			if (ImGui::Combo("Scalar Mode", { "Blank","Soil Density", "Water Density Blur", "Water Density", "Water Density Gradient", "Flux", "Divergence", "Scalar Divergence", "Nutrient Density" }, m_scalarSoilProperty))
			{
				m_updateScalarColors = true;
			}
			ImGui::TreePop();
		}
	}
}

void EcoSysLabLayer::FixedUpdate() {
	if (m_autoGrow) {
		GrowAllTrees();
	}
}

void EcoSysLabLayer::GrowAllTrees() {
	auto scene = GetScene();
	const std::vector<Entity>* treeEntities =
		scene->UnsafeGetPrivateComponentOwnersList<Tree>();
	if (treeEntities && !treeEntities->empty()) {
		float time = Application::Time().CurrentTime();
		std::vector<std::shared_future<void>> results;
		Jobs::ParallelFor(treeEntities->size(), [&](unsigned i) {
			auto treeEntity = treeEntities->at(i);
		auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
		if (!tree->m_climate.Get<Climate>()) tree->m_climate = m_climateHolder.Get<Climate>();
		if (!tree->m_soil.Get<Soil>()) tree->m_soil = m_soilHolder.Get<Soil>();
		tree->TryGrow();
			}, results);
		for (auto& i : results) i.wait();
		m_lastUsedTime = Application::Time().CurrentTime() - time;
		m_totalTime += m_lastUsedTime;

		if (scene->IsEntityValid(m_selectedTree)) {
			m_treeVisualizer.Reset(scene->GetOrSetPrivateComponent<Tree>(m_selectedTree).lock()->m_treeModel);
		}
	}
}

void EcoSysLabLayer::GenerateMeshes(const TreeMeshGeneratorSettings& meshGeneratorSettings) {
	auto scene = GetScene();
	const std::vector<Entity>* treeEntities =
		scene->UnsafeGetPrivateComponentOwnersList<Tree>();
	if (treeEntities && !treeEntities->empty()) {
		auto copiedEntities = *treeEntities;
		for (auto treeEntity : copiedEntities) {
			auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
			tree->GenerateMesh(meshGeneratorSettings);
		}
	}
}
