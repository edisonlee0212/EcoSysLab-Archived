//
// Created by lllll on 11/1/2022.
//

#include "TreeVisualizationLayer.hpp"
#include "Tree.hpp"

using namespace EcoSysLab;

void TreeVisualizationLayer::OnCreate() {
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

void TreeVisualizationLayer::OnDestroy() {

}
struct Branch
{
	glm::vec3 m_startPosition;
	float m_startThickness;
	glm::vec3 m_endPosition;
	float m_endThickness;
};
void TreeVisualizationLayer::LateUpdate() {
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

		if(m_rendering)
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
			auto branchStrands = m_branchStrands.Get<Strands>();
			auto rootStrands = m_rootStrands.Get<Strands>();

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
				m_branchSegments.resize(branchLastStartIndex);
				m_branchPoints.resize(branchLastStartIndex * 4);

				m_rootSegments.resize(rootLastStartIndex);
				m_rootPoints.resize(rootLastStartIndex * 4);


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
						for(int i = 0; i < branchList.size(); i++)
						{
							auto& flow = branchSkeleton.PeekFlow(branchList[i]);
							auto cp0 = flow.m_info.m_globalStartPosition;
							auto cp3 = flow.m_info.m_globalEndPosition;
							float distance = glm::distance(cp0, cp3);
							auto cp1 = cp0 + flow.m_info.m_globalStartRotation * glm::vec3(0, 0, -1) * distance / 3.0f;
							auto cp2 = cp3 + flow.m_info.m_globalEndRotation * glm::vec3(0, 0, 1) * distance / 3.0f;
							auto& p0 = m_branchPoints[branchStartIndex * 4 + i * 4];
							auto& p1 = m_branchPoints[branchStartIndex * 4 + i * 4 + 1];
							auto& p2 = m_branchPoints[branchStartIndex * 4 + i * 4 + 2];
							auto& p3 = m_branchPoints[branchStartIndex * 4 + i * 4 + 3];
							p0.m_position = (entityGlobalTransform.m_value *
								glm::translate(cp0))[3];
							p1.m_position = (entityGlobalTransform.m_value *
								glm::translate(cp1))[3];
							p2.m_position = (entityGlobalTransform.m_value *
								glm::translate(cp2))[3];
							p3.m_position = (entityGlobalTransform.m_value *
								glm::translate(cp3))[3];

							p0.m_thickness = flow.m_info.m_startThickness;
							p1.m_thickness = flow.m_info.m_startThickness;
							p2.m_thickness = flow.m_info.m_endThickness;
							p3.m_thickness = flow.m_info.m_endThickness;

							p0.m_color = glm::vec4(m_randomColors[flow.m_data.m_order], 1.0f);
							p1.m_color = glm::vec4(m_randomColors[flow.m_data.m_order], 1.0f);
							p2.m_color = glm::vec4(m_randomColors[flow.m_data.m_order], 1.0f);
							p3.m_color = glm::vec4(m_randomColors[flow.m_data.m_order], 1.0f);

							m_branchSegments[branchStartIndex + i] = branchStartIndex * 4 + i * 4;
						}
						auto rootStartIndex = rootStartIndices[treeIndex];
						for (int i = 0; i < rootList.size(); i++)
						{
							auto& flow = rootSkeleton.PeekFlow(rootList[i]);
							auto cp0 = flow.m_info.m_globalStartPosition;
							auto cp3 = flow.m_info.m_globalEndPosition;
							float distance = glm::distance(cp0, cp3);
							auto cp1 = cp0 + flow.m_info.m_globalStartRotation * glm::vec3(0, 0, -1) * distance / 3.0f;
							auto cp2 = cp3 + flow.m_info.m_globalEndRotation * glm::vec3(0, 0, 1) * distance / 3.0f;
							auto& p0 = m_rootPoints[rootStartIndex * 4 + i * 4];
							auto& p1 = m_rootPoints[rootStartIndex * 4 + i * 4 + 1];
							auto& p2 = m_rootPoints[rootStartIndex * 4 + i * 4 + 2];
							auto& p3 = m_rootPoints[rootStartIndex * 4 + i * 4 + 3];
							p0.m_position = (entityGlobalTransform.m_value *
								glm::translate(cp0))[3];
							p1.m_position = (entityGlobalTransform.m_value *
								glm::translate(cp1))[3];
							p2.m_position = (entityGlobalTransform.m_value *
								glm::translate(cp2))[3];
							p3.m_position = (entityGlobalTransform.m_value *
								glm::translate(cp3))[3];

							p0.m_thickness = flow.m_info.m_startThickness;
							p1.m_thickness = flow.m_info.m_startThickness;
							p2.m_thickness = flow.m_info.m_endThickness;
							p3.m_thickness = flow.m_info.m_endThickness;

							p0.m_color = glm::vec4(m_randomColors[flow.m_data.m_order], 1.0f);
							p1.m_color = glm::vec4(m_randomColors[flow.m_data.m_order], 1.0f);
							p2.m_color = glm::vec4(m_randomColors[flow.m_data.m_order], 1.0f);
							p3.m_color = glm::vec4(m_randomColors[flow.m_data.m_order], 1.0f);

							m_rootSegments[rootStartIndex + i] = rootStartIndex * 4 + i * 4;
						}
					}, results);
					for (auto& i : results) i.wait();
					branchStrands->SetSegments(1, m_branchSegments, m_branchPoints);
					rootStrands->SetSegments(1, m_rootSegments, m_rootPoints);
				}
			}
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
			if (m_displayRootFlows && !m_rootFlowMatrices.empty()) {
				Gizmos::DrawGizmoMeshInstancedColored(
					DefaultResources::Primitives::Cube, editorLayer->m_sceneCamera,
					editorLayer->m_sceneCameraPosition,
					editorLayer->m_sceneCameraRotation,
					m_rootFlowColors,
					m_rootFlowMatrices,
					glm::mat4(1.0f), 1.0f, gizmoSettings);
			}

			if (m_displayBranches) {
				gizmoSettings.m_colorMode = GizmoSettings::ColorMode::VertexColor;
				Gizmos::DrawGizmoStrands(branchStrands, glm::vec4(1.0f), glm::mat4(1.0f), 1, gizmoSettings);
				
			}

			if (m_displayRootFlows)
			{
				gizmoSettings.m_colorMode = GizmoSettings::ColorMode::VertexColor;
				Gizmos::DrawGizmoStrands(rootStrands, glm::vec4(1.0f), glm::mat4(1.0f), 1, gizmoSettings);
			}
		}
	}
}

void TreeVisualizationLayer::OnInspect() {
	if (ImGui::Begin("Tree Visualization Layer")) {
		auto scene = GetScene();
		const std::vector<Entity>* treeEntities =
			scene->UnsafeGetPrivateComponentOwnersList<Tree>();
		Editor::DragAndDropButton<Strands>(m_rootStrands, "Root strands");
		Editor::DragAndDropButton<Strands>(m_branchStrands, "Branch strands");

		ImGui::Checkbox("Lock tree selection", &m_lockTreeSelection);
		ImGui::Checkbox("Rendering", &m_rendering);
		ImGui::Checkbox("Debug Visualization", &m_debugVisualization);
		if (m_debugVisualization) {
			ImGui::Checkbox("Display Branches", &m_displayBranches);
			ImGui::Checkbox("Display Root Flows", &m_displayRootFlows);
			ImGui::Checkbox("Display Bounding Box", &m_displayBoundingBox);
		}
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

			if (ImGui::TreeNode("Mesh generation")) {
				m_meshGeneratorSettings.OnInspect();
				if (ImGui::Button("Generate Meshes")) {
					GenerateMeshes(m_meshGeneratorSettings);
				}
				ImGui::TreePop();
			}
		}

		if (m_debugVisualization && scene->IsEntityValid(m_selectedTree)) {
			m_treeVisualizer.OnInspect(
				scene->GetOrSetPrivateComponent<Tree>(m_selectedTree).lock()->m_treeModel, scene->GetDataComponent<GlobalTransform>(m_selectedTree));
		}
	}
	ImGui::End();
}

void TreeVisualizationLayer::FixedUpdate() {
	if (m_autoGrow) {
		GrowAllTrees();
	}
}

void TreeVisualizationLayer::GrowAllTrees() {
	auto scene = GetScene();
	const std::vector<Entity>* treeEntities =
		scene->UnsafeGetPrivateComponentOwnersList<Tree>();
	if (treeEntities && !treeEntities->empty()) {
		float time = Application::Time().CurrentTime();
		std::vector<std::shared_future<void>> results;
		Jobs::ParallelFor(treeEntities->size(), [&](unsigned i) {
			auto treeEntity = treeEntities->at(i);
		auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
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

void TreeVisualizationLayer::GenerateMeshes(const MeshGeneratorSettings& meshGeneratorSettings) {
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
