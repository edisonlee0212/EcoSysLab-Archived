#include "Soil.hpp"

#include <cassert>

#include "EditorLayer.hpp"
#include "Graphics.hpp"
#include "HeightField.hpp"
using namespace EcoSysLab;
bool OnInspectSoilParameters(SoilParameters& soilParameters)
{
	bool changed = false;
	if (ImGui::TreeNodeEx("Soil Parameters", ImGuiTreeNodeFlags_DefaultOpen))
	{
		if (ImGui::DragFloat("Density gradient factor", &soilParameters.m_densityGradientForceFactor, 0.01f, 0.0f, 999.0f))
		{
			changed = true;
		}
		if (ImGui::DragFloat("Gravity factor", &soilParameters.m_gravityForceFactor, 0.01f, 0.0f, 999.0f))
		{
			changed = true;
		}
		if (ImGui::DragFloat("Capacity gradient factor", &soilParameters.m_capacityGradientForceFactor, 0.01f, 0.0f, 999.0f))
		{
			changed = true;
		}

		if (ImGui::DragFloat("Delta time", &soilParameters.m_deltaTime, 0.01f, 0.0f, 999.0f))
		{
			changed = true;
		}
		ImGui::TreePop();
	}
	return changed;
}
void SoilDescriptor::OnInspect()
{
	bool changed = false;
	if (Editor::DragAndDropButton<HeightField>(m_heightField, "Height Field", true))
	{
		changed = true;
	}

	glm::ivec3 resolution = m_voxelResolution;
	if (ImGui::DragInt3("Voxel Resolution", &resolution.x, 1, 1, 100))
	{
		m_voxelResolution = resolution;
		changed = true;
	}
	if (ImGui::DragFloat("Voxel Size", &m_voxelSize, 0.01f, 0.01f, 1.0f))
	{
		changed = true;
	}
	if (ImGui::DragFloat3("Voxel Start Position", &m_startPosition.x, 0.01f))
	{
		changed = true;
	}

	if (ImGui::Button("Instantiate")) {
		auto scene = Application::GetActiveScene();
		auto soilEntity = scene->CreateEntity(GetTitle());
		auto soil = scene->GetOrSetPrivateComponent<Soil>(soilEntity).lock();
		soil->m_soilDescriptor = ProjectManager::GetAsset(GetHandle());
		soil->m_soilModel.Initialize(m_soilParameters, m_voxelResolution, m_voxelSize, m_startPosition);
	}


	if (OnInspectSoilParameters(m_soilParameters))
	{
		changed = true;
	}

	if (changed) m_saved = false;
}



void Soil::OnInspect()
{
	if (Editor::DragAndDropButton<SoilDescriptor>(m_soilDescriptor, "SoilDescriptor", true)) {
		auto soilDescriptor = m_soilDescriptor.Get<SoilDescriptor>();
		if (soilDescriptor)
		{
			m_soilModel.Initialize(soilDescriptor->m_soilParameters, soilDescriptor->m_voxelResolution, soilDescriptor->m_voxelSize, soilDescriptor->m_startPosition);
		}
	}
	auto soilDescriptor = m_soilDescriptor.Get<SoilDescriptor>();
	if (soilDescriptor)
	{
		if (ImGui::Button("Generate surface mesh")) {
			GenerateMesh();
		}
		//auto soilDescriptor = m_soilDescriptor.Get<SoilDescriptor>();
		//if (!m_soilModel.m_initialized) m_soilModel.Initialize(soilDescriptor->m_soilParameters);
		assert(m_soilModel.m_initialized);
		static bool autoStep = false;
		if (ImGui::Button("Reset"))
		{
			m_soilModel.Initialize(soilDescriptor->m_soilParameters, soilDescriptor->m_voxelResolution, soilDescriptor->m_voxelSize, soilDescriptor->m_startPosition);
		}
		bool updateVectorMatrices = false;
		bool updateVectorColors = false;
		bool updateScalarMatrices = false;
		bool updateScalarColors = false;

		ImGui::Checkbox("Auto step", &autoStep);
		if (autoStep)
		{
			m_soilModel.Step();
			m_soilModel.WaterLogic();
			updateVectorMatrices = updateScalarColors = true;
		}
		else if (ImGui::Button("Step"))
		{
			m_soilModel.Step();
			updateVectorMatrices = updateScalarColors = true;
		}

		static bool forceUpdate;
		ImGui::Checkbox("Force Update", &forceUpdate);
		static bool vectorEnable = true;
		if (ImGui::Checkbox("Vector Visualization", &vectorEnable))
		{
			if (vectorEnable) updateVectorMatrices = updateVectorColors = true;
		}
		static bool scalarEnable = true;
		if (ImGui::Checkbox("Scalar Visualization", &scalarEnable))
		{
			if (scalarEnable) updateScalarMatrices = updateScalarColors = true;
		}

		const auto numVoxels = m_soilModel.m_resolution.x * m_soilModel.m_resolution.y * m_soilModel.m_resolution.z;
		if (vectorEnable) {
			updateVectorMatrices = updateVectorMatrices || forceUpdate;
			updateVectorColors = updateVectorColors || forceUpdate;
			static float vectorMultiplier = 50.0f;
			static glm::vec4 vectorBaseColor = glm::vec4(1.0f, 1.0f, 1.0f, 0.8f);
			static unsigned vectorSoilProperty = 2;
			static float vectorLineWidthFactor = 0.1f;
			static float vectorLineMaxWidth = 0.1f;
			if (ImGui::TreeNodeEx("Vector", ImGuiTreeNodeFlags_DefaultOpen))
			{
				if (ImGui::Button("Reset"))
				{
					vectorMultiplier = 50.0f;
					vectorBaseColor = glm::vec4(1.0f, 1.0f, 1.0f, 0.8f);
					vectorSoilProperty = 2;
					vectorLineWidthFactor = 0.1f;
					vectorLineMaxWidth = 0.1f;
					updateVectorColors = updateVectorMatrices = true;
				}
				if (ImGui::ColorEdit4("Vector Base Color", &vectorBaseColor.x))
				{
					updateVectorColors = true;
				}
				if (ImGui::DragFloat("Multiplier", &vectorMultiplier, 0.1f, 0.0f, 100.0f, "%.3f"))
				{
					updateVectorMatrices = true;
				}
				if (ImGui::DragFloat("Line Width Factor", &vectorLineWidthFactor, 0.01f, 0.0f, 5.0f))
				{
					updateVectorMatrices = true;
				}
				if (ImGui::DragFloat("Max Line Width", &vectorLineMaxWidth, 0.01f, 0.0f, 5.0f))
				{
					updateVectorMatrices = true;
				}
				if (ImGui::Combo("Vector Mode", { "N/A", "N/A", "Water Density Gradient", "Flux", "Divergence", "N/A", "N/A" }, vectorSoilProperty))
				{
					updateVectorMatrices = true;
				}
				ImGui::TreePop();
			}

			static std::vector<glm::mat4> vectorMatrices;
			static std::vector<glm::vec4> vectorColors;

			if (vectorMatrices.size() != numVoxels || vectorColors.size() != numVoxels)
			{
				vectorMatrices.resize(numVoxels);
				vectorColors.resize(numVoxels);
				updateVectorMatrices = updateVectorColors = true;
			}
			if (updateVectorMatrices)
			{
				std::vector<std::shared_future<void>> results;
				const auto actualVectorMultiplier = vectorMultiplier * m_soilModel.m_voxelSize;
				switch (static_cast<SoilProperty>(vectorSoilProperty))
				{
				case SoilProperty::WaterDensityGradient:
				{
					Jobs::ParallelFor(numVoxels, [&](unsigned i)
						{
							const auto targetVector = glm::vec3(m_soilModel.m_gradWaterDensityX[i], m_soilModel.m_gradWaterDensityY[i], m_soilModel.m_gradWaterDensityZ[i]);
					const auto start = m_soilModel.GetCenter(m_soilModel.GetCoordinate(i));
					const auto end = start + targetVector * actualVectorMultiplier;
					const auto direction = glm::normalize(end - start);
					glm::quat rotation = glm::quatLookAt(direction, glm::vec3(direction.y, direction.z, direction.x));
					rotation *= glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f));
					const auto length = glm::distance(end, start) / 2.0f;
					const auto width = glm::min(vectorLineMaxWidth, length * vectorLineWidthFactor);
					const auto model = glm::translate((start + end) / 2.0f) * glm::mat4_cast(rotation) *
						glm::scale(glm::vec3(width, length, width));
					vectorMatrices[i] = model;
						}, results);
				}break;
				case SoilProperty::Flux:
				{
					Jobs::ParallelFor(numVoxels, [&](unsigned i)
						{
							const auto targetVector = glm::vec3(m_soilModel.m_fluxX[i], m_soilModel.m_fluxY[i], m_soilModel.m_fluxZ[i]);
					const auto start = m_soilModel.GetCenter(m_soilModel.GetCoordinate(i));
					const auto end = start + targetVector * actualVectorMultiplier;
					const auto direction = glm::normalize(end - start);
					glm::quat rotation = glm::quatLookAt(direction, glm::vec3(direction.y, direction.z, direction.x));
					rotation *= glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f));
					const auto length = glm::distance(end, start) / 2.0f;
					const auto width = glm::min(vectorLineMaxWidth, length * vectorLineWidthFactor);
					const auto model = glm::translate((start + end) / 2.0f) * glm::mat4_cast(rotation) *
						glm::scale(glm::vec3(width, length, width));
					vectorMatrices[i] = model;
						}, results);
				}break;
				case SoilProperty::Divergence:
				{
					Jobs::ParallelFor(numVoxels, [&](unsigned i)
						{
							const auto targetVector = glm::vec3(m_soilModel.m_divergenceX[i], m_soilModel.m_divergenceY[i], m_soilModel.m_divergenceZ[i]);
					const auto start = m_soilModel.GetCenter(m_soilModel.GetCoordinate(i));
					const auto end = start + targetVector * actualVectorMultiplier;
					const auto direction = glm::normalize(end - start);
					glm::quat rotation = glm::quatLookAt(direction, glm::vec3(direction.y, direction.z, direction.x));
					rotation *= glm::quat(glm::vec3(glm::radians(90.0f), 0.0f, 0.0f));
					const auto length = glm::distance(end, start) / 2.0f;
					const auto width = glm::min(vectorLineMaxWidth, length * vectorLineWidthFactor);
					const auto model = glm::translate((start + end) / 2.0f) * glm::mat4_cast(rotation) *
						glm::scale(glm::vec3(width, length, width));
					vectorMatrices[i] = model;
						}, results);
				}break;
				default:
				{
					Jobs::ParallelFor(numVoxels, [&](unsigned i)
						{
							vectorMatrices[i] =
							glm::translate(m_soilModel.GetCenter(m_soilModel.GetCoordinate(i)))
						* glm::mat4_cast(glm::quat(glm::vec3(0.0f)))
						* glm::scale(glm::vec3(0.0f));
						}, results);
				}break;
				}
				for (auto& i : results) i.wait();
			}
			if (updateVectorColors) {
				std::vector<std::shared_future<void>> results;

				Jobs::ParallelFor(numVoxels, [&](unsigned i)
					{
						vectorColors[i] = vectorBaseColor;
					}, results);

				for (auto& i : results) i.wait();
			}

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
				vectorColors,
				vectorMatrices,
				glm::mat4(1.0f), 1.0f, gizmoSettings);


		}
		if (scalarEnable) {
			updateScalarMatrices = updateScalarMatrices || forceUpdate;
			updateScalarColors = updateScalarColors || forceUpdate;
			static float scalarMultiplier = 1.0f;
			static float scalarBoxSize = 0.5f;
			static float scalarMinAlpha = 0.00f;
			static glm::vec3 scalarBaseColor = glm::vec3(0.0f, 0.0f, 1.0f);
			static unsigned scalarSoilProperty = 1;
			if (scalarEnable && ImGui::TreeNodeEx("Scalar", ImGuiTreeNodeFlags_DefaultOpen))
			{
				if (ImGui::Button("Reset"))
				{
					scalarMultiplier = 1.0f;
					scalarBoxSize = 0.5f;
					scalarMinAlpha = 0.00f;
					scalarBaseColor = glm::vec3(0.0f, 0.0f, 1.0f);
					scalarSoilProperty = 1;
					updateScalarColors = updateScalarMatrices = true;
				}
				if (ImGui::ColorEdit3("Scalar Base Color", &scalarBaseColor.x))
				{
					updateScalarColors = true;
				}
				if (ImGui::DragFloat("Multiplier", &scalarMultiplier, 0.001f, 0.0f, 10.0f, "%.5f"))
				{
					updateScalarColors = true;
				}
				if (ImGui::DragFloat("Min alpha", &scalarMinAlpha, 0.001f, 0.0f, 1.0f))
				{
					updateScalarColors = true;
				}
				if (ImGui::DragFloat("Box size", &scalarBoxSize, 0.001f, 0.0f, 1.0f))
				{
					updateScalarMatrices = true;
				}
				if (ImGui::Combo("Scalar Mode", { "Water Density Blur", "Water Density", "Water Density Gradient", "Flux", "Divergence", "Scalar Divergence", "NutrientDensity" }, scalarSoilProperty))
				{
					updateScalarColors = true;
				}
				ImGui::TreePop();
			}

			static std::vector<glm::mat4> scalarMatrices;
			static std::vector<glm::vec4> scalarColors;
			if (scalarMatrices.size() != numVoxels || scalarColors.size() != numVoxels)
			{
				scalarMatrices.resize(numVoxels);
				scalarColors.resize(numVoxels);
				updateScalarMatrices = updateScalarColors = true;
			}
			if (updateScalarMatrices)
			{
				std::vector<std::shared_future<void>> results;
				Jobs::ParallelFor(numVoxels, [&](unsigned i)
					{
						scalarMatrices[i] =
						glm::translate(m_soilModel.GetCenter(m_soilModel.GetCoordinate(i)))
					* glm::mat4_cast(glm::quat(glm::vec3(0.0f)))
					* glm::scale(glm::vec3(m_soilModel.GetVoxelSize() * scalarBoxSize));
					}, results);
				for (auto& i : results) i.wait();
			}
			if (updateScalarColors) {
				std::vector<std::shared_future<void>> results;
				switch (static_cast<SoilProperty>(scalarSoilProperty))
				{
				case SoilProperty::WaterDensityBlur:
				{
					Jobs::ParallelFor(numVoxels, [&](unsigned i)
						{
							auto value = glm::vec3(m_soilModel.m_waterDensityBlur[i]);
					scalarColors[i] = { scalarBaseColor, glm::clamp(glm::length(value) * scalarMultiplier, scalarMinAlpha, 1.0f) };
						}, results);
				}break;
				case SoilProperty::WaterDensity:
				{
					Jobs::ParallelFor(numVoxels, [&](unsigned i)
						{
							auto value = glm::vec3(m_soilModel.m_waterDensity[i]);
					scalarColors[i] = { scalarBaseColor, glm::clamp(glm::length(value) * scalarMultiplier, scalarMinAlpha, 1.0f) };
						}, results);
				}break;
				case SoilProperty::WaterDensityGradient:
				{
					Jobs::ParallelFor(numVoxels, [&](unsigned i)
						{
							auto value = glm::vec3(m_soilModel.m_gradWaterDensityX[i], m_soilModel.m_gradWaterDensityY[i], m_soilModel.m_gradWaterDensityZ[i]);
					scalarColors[i] = { glm::normalize(value), glm::clamp(glm::length(value) * scalarMultiplier, scalarMinAlpha, 1.0f) };
						}, results);
				}break;
				case SoilProperty::Flux:
				{
					Jobs::ParallelFor(numVoxels, [&](unsigned i)
						{
							auto value = glm::vec3(m_soilModel.m_fluxX[i], m_soilModel.m_fluxY[i], m_soilModel.m_fluxZ[i]);
					scalarColors[i] = { glm::normalize(value), glm::clamp(glm::length(value) * scalarMultiplier, scalarMinAlpha, 1.0f) };
						}, results);
				}break;
				case SoilProperty::Divergence:
				{
					Jobs::ParallelFor(numVoxels, [&](unsigned i)
						{
							auto value = glm::vec3(m_soilModel.m_divergenceX[i], m_soilModel.m_divergenceY[i], m_soilModel.m_divergenceZ[i]);
					scalarColors[i] = { glm::normalize(value), glm::clamp(glm::length(value) * scalarMultiplier, scalarMinAlpha, 1.0f) };
						}, results);
				}break;
				case SoilProperty::ScalarDivergence:
				{
					Jobs::ParallelFor(numVoxels, [&](unsigned i)
						{
							auto value = glm::vec3(m_soilModel.m_divergence[i]);
					scalarColors[i] = { scalarBaseColor, glm::clamp(glm::length(value) * scalarMultiplier, scalarMinAlpha, 1.0f) };
						}, results);
				}break;
				case SoilProperty::NutrientDensity:
				{
					Jobs::ParallelFor(numVoxels, [&](unsigned i)
						{
							auto value = glm::vec3(glm::vec3(m_soilModel.m_nutrientsDensity[i]));
					scalarColors[i] = { scalarBaseColor, glm::clamp(glm::length(value) * scalarMultiplier, scalarMinAlpha, 1.0f) };
						}, results);
				}break;
				default:
				{
					Jobs::ParallelFor(numVoxels, [&](unsigned i)
						{
							scalarColors[i] = glm::vec4(0.0f);
						}, results);
				}break;
				}
				for (auto& i : results) i.wait();
			}

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
				scalarColors,
				scalarMatrices,
				glm::mat4(1.0f), 1.0f, gizmoSettings);
		}
	}
}

void Soil::Serialize(YAML::Emitter& out)
{
	m_soilDescriptor.Save("m_soilDescriptor", out);
}

void Soil::Deserialize(const YAML::Node& in)
{
	m_soilDescriptor.Load("m_soilDescriptor", in);
	auto soilDescriptor = m_soilDescriptor.Get<SoilDescriptor>();
	if (soilDescriptor) {
		m_soilModel.Initialize(soilDescriptor->m_soilParameters, soilDescriptor->m_voxelResolution, soilDescriptor->m_voxelSize, soilDescriptor->m_startPosition);
	}
}

void Soil::CollectAssetRef(std::vector<AssetRef>& list)
{
	list.push_back(m_soilDescriptor);
}

void Soil::GenerateMesh()
{
	const auto soilDescriptor = m_soilDescriptor.Get<SoilDescriptor>();
	if (!soilDescriptor)
	{
		UNIENGINE_ERROR("No soil descriptor!");
		return;
	}
	const auto heightField = soilDescriptor->m_heightField.Get<HeightField>();
	if (!heightField)
	{
		UNIENGINE_ERROR("No height field!");
		return;
	}
	std::vector<Vertex> vertices;
	std::vector<glm::uvec3> triangles;
	heightField->GenerateMesh(glm::vec2(soilDescriptor->m_startPosition.x, soilDescriptor->m_startPosition.z), 
		glm::uvec2(soilDescriptor->m_voxelResolution.x, soilDescriptor->m_voxelResolution.z), soilDescriptor->m_voxelSize, vertices, triangles);

	const auto scene = Application::GetActiveScene();
	const auto self = GetOwner();
	Entity groundSurfaceEntity;
	const auto children = scene->GetChildren(self);

	for (const auto& child : children) {
		auto name = scene->GetEntityName(child);
		if (name == "Ground surface") {
			groundSurfaceEntity = child;
			break;
		}
	}
	if (groundSurfaceEntity.GetIndex() == 0)
	{
		groundSurfaceEntity = scene->CreateEntity("Ground surface");
		scene->SetParent(groundSurfaceEntity, self);
	}

	const auto meshRenderer =
		scene->GetOrSetPrivateComponent<MeshRenderer>(groundSurfaceEntity).lock();
	const auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
	const auto material = ProjectManager::CreateTemporaryAsset<Material>();
	mesh->SetVertices(17, vertices, triangles);
	meshRenderer->m_mesh = mesh;
	meshRenderer->m_material = material;
}

void SerializeSoilParameters(const std::string& name, const SoilParameters& soilParameters, YAML::Emitter& out) {
	out << YAML::Key << name << YAML::BeginMap;
	out << YAML::Key << "m_deltaTime" << YAML::Value << soilParameters.m_deltaTime;
	out << YAML::Key << "m_densityGradientForceFactor" << YAML::Value << soilParameters.m_densityGradientForceFactor;
	out << YAML::Key << "m_gravityForceFactor" << YAML::Value << soilParameters.m_gravityForceFactor;
	out << YAML::Key << "m_capacityGradientForceFactor" << YAML::Value << soilParameters.m_capacityGradientForceFactor;
	out << YAML::EndMap;
}

void DeserializeSoilParameters(const std::string& name, SoilParameters& soilParameters, const YAML::Node& in) {
	if (in[name]) {
		auto& param = in[name];
		if (param["m_deltaTime"]) soilParameters.m_deltaTime = param["m_deltaTime"].as<float>();
		if (param["m_densityGradientForceFactor"]) soilParameters.m_densityGradientForceFactor = param["m_densityGradientForceFactor"].as<float>();
		if (param["m_gravityForceFactor"]) soilParameters.m_gravityForceFactor = param["m_gravityForceFactor"].as<float>();
		if (param["m_capacityGradientForceFactor"]) soilParameters.m_capacityGradientForceFactor = param["m_capacityGradientForceFactor"].as<float>();
	}
}

void SoilDescriptor::Serialize(YAML::Emitter& out)
{
	out << YAML::Key << "m_voxelResolution" << YAML::Value << m_voxelResolution;
	out << YAML::Key << "m_voxelSize" << YAML::Value << m_voxelSize;
	out << YAML::Key << "m_startPosition" << YAML::Value << m_startPosition;
	m_heightField.Save("m_heightField", out);
	SerializeSoilParameters("m_soilParameters", m_soilParameters, out);
}

void SoilDescriptor::Deserialize(const YAML::Node& in)
{
	if (in["m_voxelResolution"]) m_voxelResolution = in["m_voxelResolution"].as<glm::uvec3>();
	if (in["m_voxelSize"]) m_voxelSize = in["m_voxelSize"].as<float>();
	if (in["m_startPosition"]) m_startPosition = in["m_startPosition"].as<glm::vec3>();
	m_heightField.Load("m_heightField", in);
	DeserializeSoilParameters("m_soilParameters", m_soilParameters, in);
}

void SoilDescriptor::CollectAssetRef(std::vector<AssetRef>& list)
{
	list.push_back(m_heightField);
}

