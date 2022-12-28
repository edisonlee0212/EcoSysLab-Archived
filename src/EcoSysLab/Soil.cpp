#include "Soil.hpp"
#include "EditorLayer.hpp"
#include "Graphics.hpp"
using namespace EcoSysLab;
void OnInspectSoilParameters(SoilParameters& soilParameters)
{
	if (ImGui::TreeNodeEx("Soil Parameters", ImGuiTreeNodeFlags_DefaultOpen))
	{
		ImGui::DragFloat("Diffusion Factor", &soilParameters.m_diffusionFactor, 0.01f, 0.0f, 999.0f);
		ImGui::DragFloat("Gravity Factor", &soilParameters.m_gravityFactor, 0.01f, 0.0f, 999.0f);
		ImGui::DragFloat("Cap Factor", &soilParameters.m_capFactor, 0.01f, 0.0f, 999.0f);

		ImGui::DragFloat("Delta time", &soilParameters.m_deltaTime, 0.01f, 0.0f, 999.0f);
		ImGui::TreePop();
	}
}
void SoilDescriptor::OnInspect()
{
	glm::ivec3 resolution = m_voxelResolution;
	if(ImGui::DragInt3("Voxel Resolution", &resolution.x, 1, 1, 100))
	{
		m_voxelResolution = resolution;
	}
	ImGui::DragFloat("Voxel Size", &m_voxelSize, 0.01f, 0.01f, 1.0f);
	ImGui::DragFloat3("Voxel Start Position", &m_startPosition.x, 0.01f);

	if (ImGui::Button("Instantiate")) {
		auto scene = Application::GetActiveScene();
		auto soilEntity = scene->CreateEntity(GetTitle());
		auto soil = scene->GetOrSetPrivateComponent<Soil>(soilEntity).lock();
		soil->m_soilDescriptor = ProjectManager::GetAsset(GetHandle());
		soil->m_soilModel.Initialize(m_soilParameters, m_voxelResolution, m_voxelSize, m_startPosition);

		soil->m_soilModel.TestSetup();
	}


	OnInspectSoilParameters(m_soilParameters);
}



void Soil::OnInspect()
{
	static bool debugVisualization = true;
	ImGui::Checkbox("Debug Visualization", &debugVisualization);
	if (m_soilDescriptor.Get<SoilDescriptor>()) {
		auto soilDescriptor = m_soilDescriptor.Get<SoilDescriptor>();
		if (!m_soilModel.m_initialized) m_soilModel.Initialize(soilDescriptor->m_soilParameters);
		static bool autoStep = false;
		ImGui::Checkbox("Auto step", &autoStep);
		if(autoStep)
		{
			m_soilModel.Step(soilDescriptor->m_soilParameters);
		}else if (ImGui::Button("Step"))
		{
			m_soilModel.Step(soilDescriptor->m_soilParameters);
		}

		if (debugVisualization)
		{
			static float minAlpha = 0.01f;
			ImGui::DragFloat("Min alpha", &minAlpha, 0.001f, 0.0f, 1.0f);
			static bool forceUpdateMatrices = false;
			ImGui::Checkbox("Force Update Matrices", &forceUpdateMatrices);
			static unsigned soilProperty = 0;
			ImGui::Combo("Mode", { "Water Density Blur", "Water Density", "Water Density Gradient", "Flux", "Divergence", "Scalar Divergence", "NutrientDensity" }, soilProperty);
			//static SoilProperty soilProperty = SoilProperty::WaterDensity;

			{
				static std::vector<glm::mat4> matrices;
				static std::vector<glm::vec4> colors;
				const auto numVoxels = m_soilModel.m_voxelResolution.x * m_soilModel.m_voxelResolution.y * m_soilModel.m_voxelResolution.z;
				bool updateMatrices = forceUpdateMatrices;
				if (matrices.size() != numVoxels || colors.size() != numVoxels)
				{
					matrices.resize(numVoxels);
					colors.resize(numVoxels);

					updateMatrices = true;
				}
				if (updateMatrices)
				{
					std::vector<std::shared_future<void>> results;
					Jobs::ParallelFor(numVoxels, [&](unsigned i)
						{
							matrices[i] =
							glm::translate(m_soilModel.GetCenter(m_soilModel.GetCoordinate(i)))
						* glm::mat4_cast(glm::quat(glm::vec3(0.0f)))
						* glm::scale(glm::vec3(m_soilModel.GetVoxelSize() / 2.0f));
						}, results);
					for (auto& i : results) i.wait();
				}
				std::vector<std::shared_future<void>> results;
				switch (static_cast<SoilProperty>(soilProperty))
				{
				case SoilProperty::WaterDensityBlur:
				{
					Jobs::ParallelFor(numVoxels, [&](unsigned i)
						{
							auto value = glm::vec3(m_soilModel.m_waterDensityBlur[i]);
					colors[i] = { glm::vec3(1.0f, 0.0f, 0.0f), glm::clamp(glm::length(value), minAlpha, 1.0f) };
						}, results);
				}break;
				case SoilProperty::WaterDensity:
				{
					Jobs::ParallelFor(numVoxels, [&](unsigned i)
						{
							auto value = glm::vec3(m_soilModel.m_waterDensity[i]);
					colors[i] = { glm::vec3(0.0f, 0.0f, 1.0f) , glm::clamp(glm::length(value), minAlpha, 1.0f) };
						}, results);
				}break;
				case SoilProperty::WaterDensityGradient:
				{
					Jobs::ParallelFor(numVoxels, [&](unsigned i)
						{
							auto value = glm::vec3(m_soilModel.m_gradWaterDensityX1[i], m_soilModel.m_gradWaterDensityX2[i], m_soilModel.m_gradWaterDensityX3[i]);
					colors[i] = { value, glm::clamp(glm::length(value), minAlpha, 1.0f) };
						}, results);
				}break;
				case SoilProperty::Flux:
				{
					Jobs::ParallelFor(numVoxels, [&](unsigned i)
						{
							auto value = glm::vec3(m_soilModel.m_fluxX1[i], m_soilModel.m_fluxX2[i], m_soilModel.m_fluxX3[i]);
					colors[i] = { value, glm::clamp(glm::length(value), minAlpha, 1.0f) };
						}, results);
				}break;
				case SoilProperty::Divergence:
				{
					Jobs::ParallelFor(numVoxels, [&](unsigned i)
						{
							auto value = glm::vec3(m_soilModel.m_divergenceX1[i], m_soilModel.m_divergenceX2[i], m_soilModel.m_divergenceX3[i]);
					colors[i] = { value, glm::clamp(glm::length(value), minAlpha, 1.0f) };
						}, results);
				}break;
				case SoilProperty::ScalarDivergence:
				{
					Jobs::ParallelFor(numVoxels, [&](unsigned i)
						{
							auto value = glm::vec3(m_soilModel.m_divergence[i]);
					colors[i] = { glm::vec3(1.0f, 0.0f, 0.0f), glm::clamp(glm::length(value), minAlpha, 1.0f) };
						}, results);
				}break;
				case SoilProperty::NutrientDensity:
				{
					Jobs::ParallelFor(numVoxels, [&](unsigned i)
						{
							auto value = glm::vec3(glm::vec3(m_soilModel.m_nutrientsDensity[i]));
					colors[i] = { glm::vec3(1.0f, 0.0f, 0.0f ), glm::clamp(glm::length(value), minAlpha, 1.0f)};
						}, results);
				}break;
				}

				for (auto& i : results) i.wait();

				auto editorLayer = Application::GetLayer<EditorLayer>();

				GizmoSettings gizmoSettings;
				gizmoSettings.m_drawSettings.m_blending = true;
				Gizmos::DrawGizmoMeshInstancedColored(
					DefaultResources::Primitives::Cube, editorLayer->m_sceneCamera,
					editorLayer->m_sceneCameraPosition,
					editorLayer->m_sceneCameraRotation,
					colors,
					matrices,
					glm::mat4(1.0f), 1.0f, gizmoSettings);
			}
		}
	}
}

void SerializeSoilParameters(const std::string& name, const SoilParameters& soilParameters, YAML::Emitter& out) {
	out << YAML::Key << name << YAML::BeginMap;
	out << YAML::Key << "m_deltaTime" << YAML::Value << soilParameters.m_deltaTime;
	out << YAML::Key << "m_diffusionFactor" << YAML::Value << soilParameters.m_diffusionFactor;
	out << YAML::Key << "m_gravityFactor" << YAML::Value << soilParameters.m_gravityFactor;
	out << YAML::Key << "m_capFactor" << YAML::Value << soilParameters.m_capFactor;
	out << YAML::EndMap;
}

void DeserializeSoilParameters(const std::string& name, SoilParameters& soilParameters, const YAML::Node& in) {
	if (in[name]) {
		auto& param = in[name];
		if (param["m_deltaTime"]) soilParameters.m_deltaTime = param["m_deltaTime"].as<float>();
		if (param["m_diffusionFactor"]) soilParameters.m_diffusionFactor = param["m_diffusionFactor"].as<float>();
		if (param["m_gravityFactor"]) soilParameters.m_gravityFactor = param["m_gravityFactor"].as<float>();
		if (param["m_capFactor"]) soilParameters.m_capFactor = param["m_capFactor"].as<float>();
	}
}

void SoilDescriptor::Serialize(YAML::Emitter& out)
{
	out << YAML::Key << "m_voxelResolution" << YAML::Value << m_voxelResolution;
	out << YAML::Key << "m_voxelSize" << YAML::Value << m_voxelSize;
	out << YAML::Key << "m_startPosition" << YAML::Value << m_startPosition;

	SerializeSoilParameters("m_soilParameters", m_soilParameters, out);
}

void SoilDescriptor::Deserialize(const YAML::Node& in)
{
	if (in["m_voxelResolution"]) m_voxelResolution = in["m_voxelResolution"].as<glm::uvec3>();
	if (in["m_voxelSize"]) m_voxelSize = in["m_voxelSize"].as<float>();
	if (in["m_startPosition"]) m_startPosition = in["m_startPosition"].as<glm::vec3>();

	DeserializeSoilParameters("m_soilParameters", m_soilParameters, in);
}

