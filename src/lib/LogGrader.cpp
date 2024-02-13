#include "LogGrader.hpp"

using namespace EcoSysLab;

void LogGrader::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	if(ImGui::TreeNode("Procedural Log Parameters"))
	{
		ImGui::DragFloat("Height", &m_proceduralLogParameters.m_height);
		ImGui::DragFloat("Start radius", &m_proceduralLogParameters.m_startRadius);
		ImGui::DragFloat("End radius", &m_proceduralLogParameters.m_endRadius);
		static PlottedDistributionSettings plottedDistributionSettings = { 0.001f,
														{0.001f, true, false, ""},
														{0.001f, true, false, ""},
														"" };
		m_proceduralLogParameters.m_sweep.OnInspect("Sweep", plottedDistributionSettings);
		m_proceduralLogParameters.m_sweepDirectionAngle.OnInspect("Sweep Direction Angle", plottedDistributionSettings);
		ImGui::TreePop();
	}
	editorLayer->DragAndDropButton<BranchShape>(m_branchShape, "Branch Shape", true);
	if (ImGui::Button("Initialize Log"))
	{
		auto branchShape = m_branchShape.Get<BranchShape>();
		if(!branchShape)
		{
			branchShape = ProjectManager::CreateTemporaryAsset<BranchShape>();
			m_branchShape = branchShape;
			branchShape->m_barkDepth = branchShape->m_baseDepth = 0.1f;
		}
		InitializeLogRandomly(m_proceduralLogParameters, branchShape);
	}
	if (ImGui::Button("Initialize Mesh Renderer")) InitializeMeshRenderer(m_logWoodMeshGenerationSettings);
}

void LogGrader::InitializeLogRandomly(const ProceduralLogParameters& proceduralLogParameters, const std::shared_ptr<BranchShape>& branchShape)
{
	m_logWood.m_intersections.clear();
	m_logWood.m_intersections.resize(glm::max(2.0f, proceduralLogParameters.m_height / m_logWood.m_heightStep));
	
	for(int intersectionIndex = 0; intersectionIndex < m_logWood.m_intersections.size(); intersectionIndex++)
	{
		const float a = static_cast<float>(intersectionIndex) / (m_logWood.m_intersections.size() - 1);
		const float radius = glm::mix(proceduralLogParameters.m_startRadius, proceduralLogParameters.m_endRadius, a);
		auto& intersection = m_logWood.m_intersections[intersectionIndex];
		glm::vec2 sweepDirection = glm::vec2(glm::cos(glm::radians(proceduralLogParameters.m_sweepDirectionAngle.GetValue(a))), glm::sin(glm::radians(proceduralLogParameters.m_sweepDirectionAngle.GetValue(a))));
		intersection.m_center = sweepDirection * proceduralLogParameters.m_sweep.GetValue(a);

		intersection.m_boundary.resize(360);
		for(int boundaryPointIndex = 0; boundaryPointIndex < 360; boundaryPointIndex++)
		{
			auto& boundaryPoint = intersection.m_boundary.at(boundaryPointIndex);
			boundaryPoint.m_centerDistance = radius * branchShape->GetValue(static_cast<float>(boundaryPointIndex) / 360.0f, intersectionIndex * m_logWood.m_heightStep);
			boundaryPoint.m_defectStatus = 0.0f;
		}
	}
}

std::shared_ptr<Mesh> LogGrader::GenerateSurfaceMesh(const LogWoodMeshGenerationSettings& meshGeneratorSettings) const
{
	if (m_logWood.m_intersections.size() < 2) return {};
	const int xStepSize = 360.0f / meshGeneratorSettings.m_xSubdivision;
	const float xStep = 360.0f / xStepSize;
	const float logLength = (m_logWood.m_intersections.size() - 1) * m_logWood.m_heightStep;
	const int yStepSize = logLength / meshGeneratorSettings.m_ySubdivision;
	const float yStep = logLength / yStepSize;
	Vertex archetype{};

	std::vector<Vertex> vertices{};
	std::vector<unsigned int> indices{};
	for(int yIndex = 0; yIndex <= yStepSize; yIndex++)
	{
		const float y = yStep * static_cast<float>(yIndex);
		
		if(yIndex < yStepSize)
		{
			const auto vertexIndex = vertices.size();
			for (int xIndex = 0; xIndex < xStepSize; xIndex++)
			{
				auto a = vertexIndex + xIndex;
				auto b = vertexIndex + (xIndex == xStepSize - 1 ? 0 : xIndex + 1);
				auto c = vertexIndex + xStepSize + xIndex;
				indices.push_back(c);
				indices.push_back(b);
				indices.push_back(a);
				a = vertexIndex + xStepSize + (xIndex == xStepSize - 1 ? 0 : xIndex + 1);
				b = vertexIndex + xStepSize + xIndex;
				c = vertexIndex + (xIndex == xStepSize - 1 ? 0 : xIndex + 1);
				indices.push_back(c);
				indices.push_back(b);
				indices.push_back(a);
			}
		}

		for (int xIndex = 0; xIndex < xStepSize; xIndex++)
		{
			const float x = xStep * static_cast<float>(xIndex);
			glm::vec2 boundaryPoint = m_logWood.GetSurfacePoint(y, x);
			archetype.m_position = glm::vec3(boundaryPoint.x, y, boundaryPoint.y);
			float defectStatus = m_logWood.GetDefectStatus(y, x);
			archetype.m_color = glm::mix(glm::vec4(0, 1, 0, 1), glm::vec4(1, 0, 0, 1), defectStatus);
			archetype.m_texCoord = { x, y };

			vertices.emplace_back(archetype);
		}
	}

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
	const auto branchEntity = scene->CreateEntity("Log Wood Mesh");
	scene->SetParent(branchEntity, self);

	const auto mesh = GenerateSurfaceMesh(meshGeneratorSettings);
	const auto material = ProjectManager::CreateTemporaryAsset<Material>();
	const auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(branchEntity).lock();
	material->m_materialProperties.m_roughness = 1.0f;
	material->m_materialProperties.m_metallic = 0.0f;
	meshRenderer->m_mesh = mesh;
	meshRenderer->m_material = material;
}

void LogGrader::ClearMeshRenderer() const
{
	const auto scene = GetScene();
	const auto self = GetOwner();
	const auto children = scene->GetChildren(self);
	for (const auto& child : children) {
		auto name = scene->GetEntityName(child);
		if (name == "Log Wood Mesh") {
			scene->DeleteEntity(child);
		}
	}
}
