#include "LogGrader.hpp"

using namespace EcoSysLab;

bool ProceduralLogParameters::OnInspect()
{
	bool changed = false;
	if (ImGui::TreeNode("Procedural Log Parameters"))
	{
		if (ImGui::Checkbox("Butt only", &m_bottom)) changed = true;
		if (ImGui::DragFloat("Length without trim", &m_lengthWithoutTrim)) changed = true;
		if (ImGui::DragFloat("Large End Diameter (LED)", &m_largeEndDiameter)) changed = true;
		if (ImGui::DragFloat("Small End Diameter (LED)", &m_smallEndDiameter)) changed = true;
		static PlottedDistributionSettings plottedDistributionSettings = { 0.001f,
														{0.001f, true, false, ""},
														{0.001f, true, false, ""},
														"" };
		if (m_sweep.OnInspect("Sweep", plottedDistributionSettings)) changed = true;
		if (m_sweepDirectionAngle.OnInspect("Sweep Direction Angle", plottedDistributionSettings))changed = true;
		ImGui::TreePop();
	}
	return changed;
}

void LogGrader::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	m_proceduralLogParameters.OnInspect();
	editorLayer->DragAndDropButton<BranchShape>(m_branchShape, "Branch Shape", true);
	if (ImGui::Button("Initialize Log"))
	{
		auto branchShape = m_branchShape.Get<BranchShape>();
		if (!branchShape)
		{
			branchShape = ProjectManager::CreateTemporaryAsset<BranchShape>();
			m_branchShape = branchShape;
			branchShape->m_barkDepth = branchShape->m_baseDepth = 0.1f;
		}
		InitializeLogRandomly(m_proceduralLogParameters, branchShape);
	}
	if (ImGui::TreeNode("Log Mesh Generation Settings"))
	{
		ImGui::DragFloat("Y Subdivision", &m_logWoodMeshGenerationSettings.m_ySubdivision, 0.01f, 0.01f, 0.5f);
		ImGui::TreePop();
	}
	static int rotateDegrees = 10;
	ImGui::DragInt("Degrees", &rotateDegrees, 1, 1, 360);
	if (ImGui::Button(("Rotate " + std::to_string(rotateDegrees) + " degrees").c_str()))
	{
		m_logWood.Rotate(rotateDegrees);
		const auto gradeData = m_logWood.CalculateGradingData(0);
		m_logWood.ColorBasedOnGrading(gradeData);
		InitializeMeshRenderer(m_logWoodMeshGenerationSettings);
	}
	if (ImGui::Button("Initialize Mesh Renderer")) InitializeMeshRenderer(m_logWoodMeshGenerationSettings);
	static bool enableDefectSelection = true;
	static bool eraseMode = false;
	ImGui::Checkbox("Enable Defect Marker", &enableDefectSelection);
	ImGui::Checkbox("Erase mode", &eraseMode);
	if (enableDefectSelection) {
		static float defectHeightRange = 0.1f;
		static int defectAngleRange = 10.0f;
		ImGui::DragFloat("Defect Marker Y", &defectHeightRange, 0.01f, 0.03f, 1.0f);
		ImGui::DragInt("Defect Marker X", &defectAngleRange, 1, 3, 30);
		if (editorLayer->SceneCameraWindowFocused() && editorLayer->GetLockEntitySelection() && editorLayer->GetSelectedEntity() == GetOwner()) {
			static std::vector<glm::vec2> mousePositions{};
			if (editorLayer->GetKey(GLFW_MOUSE_BUTTON_RIGHT) == KeyActionType::Press)
			{
				mousePositions.clear();
			}else if (editorLayer->GetKey(GLFW_MOUSE_BUTTON_LEFT) == KeyActionType::Hold)
			{
				mousePositions.emplace_back(editorLayer->GetMouseSceneCameraPosition());
			}else if(editorLayer->GetKey(GLFW_MOUSE_BUTTON_LEFT) == KeyActionType::Release && !mousePositions.empty())
			{
				const auto scene = GetScene();
				GlobalTransform cameraLtw;
				cameraLtw.m_value =
					glm::translate(
						editorLayer->GetSceneCameraPosition()) *
					glm::mat4_cast(
						editorLayer->GetSceneCameraRotation());
				for (const auto& position : mousePositions) {
					const Ray cameraRay = editorLayer->GetSceneCamera()->ScreenPointToRay(
						cameraLtw, position);
					float height, angle;
					if (m_logWood.RayCastSelection(scene->GetDataComponent<GlobalTransform>(GetOwner()).m_value, 0.02f, cameraRay, height, angle))
					{
						if (!eraseMode) m_logWood.MarkDefectRegion(height, angle, defectHeightRange, defectAngleRange);
						else m_logWood.EraseDefectRegion(height, angle, defectHeightRange, defectAngleRange);
					}
				}
				mousePositions.clear();
				const auto gradeData = m_logWood.CalculateGradingData(0);
				m_logWood.ColorBasedOnGrading(gradeData);
				InitializeMeshRenderer(m_logWoodMeshGenerationSettings);
			}
		}
	}
	if (ImGui::Button("Clear Defects"))
	{
		m_logWood.ClearDefects();
		InitializeMeshRenderer(m_logWoodMeshGenerationSettings);
	}
}

void LogGrader::InitializeLogRandomly(const ProceduralLogParameters& proceduralLogParameters, const std::shared_ptr<BranchShape>& branchShape)
{
	m_logWood.m_intersections.clear();
	m_logWood.m_lengthWithoutTrim = proceduralLogParameters.m_lengthWithoutTrim;
	m_logWood.m_intersections.resize(glm::max(1.0f, proceduralLogParameters.m_lengthWithoutTrim / proceduralLogParameters.m_lengthStep));

	for (int intersectionIndex = 0; intersectionIndex < m_logWood.m_intersections.size(); intersectionIndex++)
	{
		const float a = static_cast<float>(intersectionIndex) / (m_logWood.m_intersections.size() - 1);
		const float radius = glm::mix(proceduralLogParameters.m_largeEndDiameter * 0.5f, proceduralLogParameters.m_smallEndDiameter * 0.5f, a);
		auto& intersection = m_logWood.m_intersections[intersectionIndex];
		glm::vec2 sweepDirection = glm::vec2(glm::cos(glm::radians(proceduralLogParameters.m_sweepDirectionAngle.GetValue(a))), glm::sin(glm::radians(proceduralLogParameters.m_sweepDirectionAngle.GetValue(a))));
		intersection.m_center = sweepDirection * proceduralLogParameters.m_sweep.GetValue(a);

		intersection.m_boundary.resize(360);
		for (int boundaryPointIndex = 0; boundaryPointIndex < 360; boundaryPointIndex++)
		{
			auto& boundaryPoint = intersection.m_boundary.at(boundaryPointIndex);
			boundaryPoint.m_centerDistance = radius * branchShape->GetValue(static_cast<float>(boundaryPointIndex) / 360.0f, intersectionIndex * proceduralLogParameters.m_lengthStep);
			boundaryPoint.m_defectStatus = 0.0f;
		}
	}
}

std::shared_ptr<Mesh> LogGrader::GenerateCylinderMesh(const LogWoodMeshGenerationSettings& meshGeneratorSettings) const
{
	if (m_logWood.m_intersections.size() < 2) return {};
	const float logLength = m_logWood.m_lengthWithoutTrim;
	const int yStepSize = logLength / meshGeneratorSettings.m_ySubdivision;
	const float yStep = logLength / yStepSize;

	std::vector<Vertex> vertices{};
	std::vector<unsigned int> indices{};

	vertices.resize((yStepSize + 1) * 360);
	indices.resize(yStepSize * 360 * 6);

	Jobs::ParallelFor(yStepSize + 1, [&](const unsigned yIndex)
		{
			Vertex archetype{};
			const float y = yStep * static_cast<float>(yIndex);
			for (int xIndex = 0; xIndex < 360; xIndex++)
			{
				const float x = static_cast<float>(xIndex);
				const glm::vec2 boundaryPoint = m_logWood.GetSurfacePoint(y, x);
				archetype.m_position = glm::vec3(boundaryPoint.x, y, boundaryPoint.y);
				archetype.m_color = m_logWood.GetColor(y, x);
				archetype.m_texCoord = { x, y };
				vertices[yIndex * 360 + xIndex] = archetype;
			}
		}
	);
	Jobs::ParallelFor(yStepSize, [&](const unsigned yIndex)
		{
			const auto vertexStartIndex = yIndex * 360;
			for (int xIndex = 0; xIndex < 360; xIndex++)
			{
				auto a = vertexStartIndex + xIndex;
				auto b = vertexStartIndex + (xIndex == 360 - 1 ? 0 : xIndex + 1);
				auto c = vertexStartIndex + 360 + xIndex;
				indices.at((yIndex * 360 + xIndex) * 6) = c;
				indices.at((yIndex * 360 + xIndex) * 6 + 1) = b;
				indices.at((yIndex * 360 + xIndex) * 6 + 2) = a;
				a = vertexStartIndex + 360 + (xIndex == 360 - 1 ? 0 : xIndex + 1);
				b = vertexStartIndex + 360 + xIndex;
				c = vertexStartIndex + (xIndex == 360 - 1 ? 0 : xIndex + 1);
				indices.at((yIndex * 360 + xIndex) * 6 + 3) = c;
				indices.at((yIndex * 360 + xIndex) * 6 + 4) = b;
				indices.at((yIndex * 360 + xIndex) * 6 + 5) = a;
			}
		}
	);

	const auto retVal = ProjectManager::CreateTemporaryAsset<Mesh>();
	VertexAttributes attributes{};
	attributes.m_texCoord = true;
	retVal->SetVertices(attributes, vertices, indices);
	return retVal;
}

std::shared_ptr<Mesh> LogGrader::GenerateFlatMesh(const LogWoodMeshGenerationSettings& meshGeneratorSettings, const int startX, const int endX) const
{
	if (m_logWood.m_intersections.size() < 2) return {};

	const float avgDistance = m_logWood.GetAverageDistance();
	const float circleLength = 2.0f * glm::pi<float>() * avgDistance;
	const float flatXStep = circleLength / 360;
	const float logLength = m_logWood.m_lengthWithoutTrim;
	const int yStepSize = logLength / meshGeneratorSettings.m_ySubdivision;
	const float yStep = logLength / yStepSize;


	std::vector<Vertex> vertices{};
	std::vector<unsigned int> indices{};

	const int span = endX - startX;

	vertices.resize((yStepSize + 1) * (span + 1));
	indices.resize(yStepSize * span * 6);

	Jobs::ParallelFor(yStepSize + 1, [&](const unsigned yIndex)
		{
			Vertex archetype{};
			const float y = yStep * static_cast<float>(yIndex);
			const float intersectionAvgDistance = m_logWood.GetAverageDistance(y);
			for (int xIndex = 0; xIndex <= span; xIndex++)
			{
				const float x = static_cast<float>(xIndex);
				const float centerDistance = m_logWood.GetCenterDistance(y, x);
				archetype.m_position = glm::vec3(flatXStep * static_cast<float>(xIndex - span), y, centerDistance - intersectionAvgDistance);
				archetype.m_color = m_logWood.GetColor(y, x + startX);
				archetype.m_texCoord = { x, y };
				vertices[yIndex * (span + 1) + xIndex] = archetype;
			}
		}
	);
	Jobs::ParallelFor(yStepSize, [&](const unsigned yIndex)
		{
			const auto vertexStartIndex = yIndex * (span + 1);
			for (int xIndex = 0; xIndex < span; xIndex++)
			{
				auto a = vertexStartIndex + xIndex;
				auto b = vertexStartIndex + xIndex + 1;
				auto c = vertexStartIndex + (span + 1) + xIndex;
				indices.at((yIndex * span + xIndex) * 6) = c;
				indices.at((yIndex * span + xIndex) * 6 + 1) = b;
				indices.at((yIndex * span + xIndex) * 6 + 2) = a;
				a = vertexStartIndex + (span + 1) + xIndex + 1;
				b = vertexStartIndex + (span + 1) + xIndex;
				c = vertexStartIndex + xIndex + 1;
				indices.at((yIndex * span + xIndex) * 6 + 3) = c;
				indices.at((yIndex * span + xIndex) * 6 + 4) = b;
				indices.at((yIndex * span + xIndex) * 6 + 5) = a;
			}
		}
	);

	const auto retVal = ProjectManager::CreateTemporaryAsset<Mesh>();
	VertexAttributes attributes{};
	attributes.m_texCoord = true;
	retVal->SetVertices(attributes, vertices, indices);
	return retVal;
}

void LogGrader::OnCreate()
{
	m_proceduralLogParameters.m_sweep.m_mean = { -1.0f, 1.0f };
	m_proceduralLogParameters.m_sweep.m_deviation = { 0.0f, 1.0f, {0, 0} };
	m_proceduralLogParameters.m_sweepDirectionAngle.m_mean = { -180, 180.0f };
	m_proceduralLogParameters.m_sweepDirectionAngle.m_deviation = { 0.0f, 1.0f, {0, 0} };
}

void LogGrader::InitializeMeshRenderer(const LogWoodMeshGenerationSettings& meshGeneratorSettings) const
{
	ClearMeshRenderer();
	const auto scene = GetScene();
	const auto self = GetOwner();
	const auto cylinderEntity = scene->CreateEntity("Log Wood Cylinder Mesh");
	if (scene->IsEntityValid(cylinderEntity)) {
		scene->SetParent(cylinderEntity, self);
		const auto mesh = GenerateCylinderMesh(meshGeneratorSettings);
		const auto material = ProjectManager::CreateTemporaryAsset<Material>();
		const auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(cylinderEntity).lock();
		material->m_materialProperties.m_roughness = 1.0f;
		material->m_materialProperties.m_metallic = 0.0f;
		material->m_vertexColorOnly = true;
		meshRenderer->m_mesh = mesh;
		meshRenderer->m_material = material;
	}

	const float avgDistance = m_logWood.GetMaxAverageDistance();
	const float circleLength = 2.0f * glm::pi<float>() * avgDistance;
	float xOffset = avgDistance * 1.5f;
	const auto flatEntity1 = scene->CreateEntity("Log Wood Flat Mesh 1");
	if (scene->IsEntityValid(flatEntity1)) {
		scene->SetParent(flatEntity1, self);
		Transform transform{};
		transform.SetPosition({ xOffset , 0, 0 });
		transform.SetEulerRotation(glm::radians(glm::vec3(0, 180, 0)));
		scene->SetDataComponent(flatEntity1, transform);
		const auto mesh = GenerateFlatMesh(meshGeneratorSettings, 90, 180);
		const auto material = ProjectManager::CreateTemporaryAsset<Material>();
		const auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(flatEntity1).lock();
		material->m_materialProperties.m_roughness = 1.0f;
		material->m_materialProperties.m_metallic = 0.0f;
		material->m_vertexColorOnly = true;
		meshRenderer->m_mesh = mesh;
		meshRenderer->m_material = material;
	}
	xOffset += circleLength / 4.0f + 0.2f;
	const auto flatEntity2 = scene->CreateEntity("Log Wood Flat Mesh 2");
	if (scene->IsEntityValid(flatEntity2)) {
		scene->SetParent(flatEntity2, self);
		Transform transform{};
		transform.SetPosition({ xOffset, 0, 0 });
		transform.SetEulerRotation(glm::radians(glm::vec3(0, 180, 0)));
		scene->SetDataComponent(flatEntity2, transform);
		const auto mesh = GenerateFlatMesh(meshGeneratorSettings, 0, 90);
		const auto material = ProjectManager::CreateTemporaryAsset<Material>();
		const auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(flatEntity2).lock();
		material->m_materialProperties.m_roughness = 1.0f;
		material->m_materialProperties.m_metallic = 0.0f;
		material->m_vertexColorOnly = true;
		meshRenderer->m_mesh = mesh;
		meshRenderer->m_material = material;
	}
	xOffset += circleLength / 4.0f + 0.2f;
	const auto flatEntity3 = scene->CreateEntity("Log Wood Flat Mesh 3");
	if (scene->IsEntityValid(flatEntity3)) {
		scene->SetParent(flatEntity3, self);
		Transform transform{};
		transform.SetPosition({ xOffset, 0, 0 });
		transform.SetEulerRotation(glm::radians(glm::vec3(0, 180, 0)));
		scene->SetDataComponent(flatEntity3, transform);
		const auto mesh = GenerateFlatMesh(meshGeneratorSettings, 270, 360);
		const auto material = ProjectManager::CreateTemporaryAsset<Material>();
		const auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(flatEntity3).lock();
		material->m_materialProperties.m_roughness = 1.0f;
		material->m_materialProperties.m_metallic = 0.0f;
		material->m_vertexColorOnly = true;
		meshRenderer->m_mesh = mesh;
		meshRenderer->m_material = material;
	}
	xOffset += circleLength / 4.0f + 0.2f;
	const auto flatEntity4 = scene->CreateEntity("Log Wood Flat Mesh 4");
	if (scene->IsEntityValid(flatEntity4)) {
		scene->SetParent(flatEntity4, self);
		Transform transform{};
		transform.SetPosition({ xOffset, 0, 0 });
		transform.SetEulerRotation(glm::radians(glm::vec3(0, 180, 0)));
		scene->SetDataComponent(flatEntity4, transform);
		const auto mesh = GenerateFlatMesh(meshGeneratorSettings, 180, 270);
		const auto material = ProjectManager::CreateTemporaryAsset<Material>();
		const auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(flatEntity4).lock();
		material->m_materialProperties.m_roughness = 1.0f;
		material->m_materialProperties.m_metallic = 0.0f;
		material->m_vertexColorOnly = true;
		meshRenderer->m_mesh = mesh;
		meshRenderer->m_material = material;
	}
}

void LogGrader::ClearMeshRenderer() const
{
	const auto scene = GetScene();
	const auto self = GetOwner();
	const auto children = scene->GetChildren(self);
	for (const auto& child : children) {
		auto name = scene->GetEntityName(child);
		if (name == "Log Wood Cylinder Mesh") {
			scene->DeleteEntity(child);
		}
		else if (name == "Log Wood Flat Mesh 1") {
			scene->DeleteEntity(child);
		}
		else if (name == "Log Wood Flat Mesh 2") {
			scene->DeleteEntity(child);
		}
		else if (name == "Log Wood Flat Mesh 3") {
			scene->DeleteEntity(child);
		}
		else if (name == "Log Wood Flat Mesh 4") {
			scene->DeleteEntity(child);
		}
	}
}
