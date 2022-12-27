#include "Soil.hpp"
#include "EditorLayer.hpp"
#include "Graphics.hpp"
using namespace EcoSysLab;
void SoilDescriptor::OnInspect()
{
	if (ImGui::Button("Instantiate")) {
		auto scene = Application::GetActiveScene();
		auto soilEntity = scene->CreateEntity(GetTitle());
		auto soil = scene->GetOrSetPrivateComponent<Soil>(soilEntity).lock();
		soil->m_soilDescriptor = ProjectManager::GetAsset(GetHandle());
	}
}

void Soil::OnInspect()
{
	static bool debugVisualization = true;
	ImGui::Checkbox("Debug Visualization", &debugVisualization);
	if (m_soilDescriptor.Get<SoilDescriptor>()) {
		auto soilDescriptor = m_soilDescriptor.Get<SoilDescriptor>();
		if (!m_soilModel.m_initialized) m_soilModel.Initialize(soilDescriptor->m_soilParameters);

		if (ImGui::Button("Step"))
		{
			m_soilModel.Step(soilDescriptor->m_soilParameters);
		}

		if (debugVisualization)
		{
			static float alpha = 0.25f;
			ImGui::DragFloat("Alpha", &alpha, 0.01f, 0.0f, 1.0f);
			static bool forceUpdateMatrices = false;
			ImGui::Checkbox("Force Update Matrices", &forceUpdateMatrices);
			static SoilProperty soilProperty = SoilProperty::WaterDensity;

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
						* glm::scale(glm::vec3(m_soilModel.GetVoxelDistance() / 4.0f));
						}, results);
					for (auto& i : results) i.wait();
				}
				std::vector<std::shared_future<void>> results;
				switch (soilProperty)
				{
				case SoilProperty::WaterDensity:
				{
					Jobs::ParallelFor(numVoxels, [&](unsigned i)
						{
							colors[i] = { glm::vec3(m_soilModel.m_waterDensity[i]) , alpha };
						}, results);
				}break;
				case SoilProperty::WaterDensityGradient:
				{
					Jobs::ParallelFor(numVoxels, [&](unsigned i)
						{
							colors[i] = { m_soilModel.m_gradWaterDensityX1[i], m_soilModel.m_gradWaterDensityX1[i], m_soilModel.m_gradWaterDensityX1[i] , alpha };
						}, results);
				}break;
				case SoilProperty::NutrientDensity:
				{
					Jobs::ParallelFor(numVoxels, [&](unsigned i)
						{
							colors[i] = glm::vec4(glm::vec3(m_soilModel.m_nutrientsDensity[i]), alpha);
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
