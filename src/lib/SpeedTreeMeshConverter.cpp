#include "SpeedTreeMeshConverter.hpp"
#include "Prefab.hpp"
using namespace EcoSysLab;

bool SpeedTreeMeshConverter::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	bool changed = false;
	const auto scene = GetScene();
	if (EditorLayer::DragAndDropButton<Prefab>(m_originalModel, "SpeedTree Model")) changed = true;
	if (const auto prefab = m_originalModel.Get<Prefab>())
	{
		static BillboardCloud::ProjectSettings projectSettings {};
		ImGui::Checkbox("Albedo map", &projectSettings.m_transferAlbedoMap);
		ImGui::Checkbox("Normal map", &projectSettings.m_transferNormalMap);
		ImGui::Checkbox("Roughness map", &projectSettings.m_transferRoughnessMap);
		ImGui::Checkbox("Metallic map", &projectSettings.m_transferMetallicMap);
		ImGui::Checkbox("AO map", &projectSettings.m_transferAoMap);
		static BillboardCloud::ClusterizationSettings clusterizeSettings{};
		static bool combinePrefab = true;
		ImGui::Checkbox("Combine", &combinePrefab);
		ImGui::DragFloat("Size factor", &projectSettings.m_resolutionFactor, 1.f, 1, 1024);
		if (ImGui::Button("Build Billboard Cloud")) {
			BillboardCloud billboardCloud {};
			billboardCloud.BuildClusters(prefab, clusterizeSettings, combinePrefab);
			billboardCloud.ProjectClusters(projectSettings);
			const auto entity = billboardCloud.BuildEntity(scene);
			if(scene->IsEntityValid(entity)) scene->SetEntityName(entity, "SpeedTree billboard cloud");
			else
			{
				EVOENGINE_ERROR("Failed to build billboard cloud!");
			}
		}
	}
	EditorLayer::DragAndDropButton<Mesh>(m_foliageMesh, "Foliage mesh");
	EditorLayer::DragAndDropButton<Mesh>(m_branchMesh, "Branch mesh");
	EditorLayer::DragAndDropButton<Mesh>(m_foliageMaterial, "Foliage material");
	EditorLayer::DragAndDropButton<Mesh>(m_branchMaterial, "Branch material");

	return changed;
}
