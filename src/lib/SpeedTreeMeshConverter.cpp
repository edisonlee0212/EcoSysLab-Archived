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
		static BillboardCloud::JoinSettings joinSettings {};
		static BillboardCloud::ProjectSettings projectSettings {};
		static BillboardCloud::RasterizeSettings rasterizeSettings {};
		ImGui::Checkbox("Albedo map", &rasterizeSettings.m_transferAlbedoMap);
		ImGui::Checkbox("Normal map", &rasterizeSettings.m_transferNormalMap);
		ImGui::Checkbox("Roughness map", &rasterizeSettings.m_transferRoughnessMap);
		ImGui::Checkbox("Metallic map", &rasterizeSettings.m_transferMetallicMap);
		ImGui::Checkbox("AO map", &rasterizeSettings.m_transferAoMap);
		static BillboardCloud::ClusterizationSettings clusterizeSettings{};
		clusterizeSettings.m_clusterizeMode = BillboardCloud::ClusterizationMode::Default;
		static bool combinePrefab = true;
		ImGui::Checkbox("Combine", &combinePrefab);
		//ImGui::DragInt2("Size factor", &joinSettings.m_resolution, 1.f, 1, 1024);
		if (ImGui::Button("Build Billboard Cloud")) {
			BillboardCloud billboardCloud {};
			billboardCloud.ProcessPrefab(prefab, combinePrefab);
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
