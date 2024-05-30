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
		static BillboardCloud::GenerateSettings generateSettings {};
		generateSettings.OnInspect("Generate settings");
		if (ImGui::Button("Build Billboard Cloud")) {
			BillboardCloud billboardCloud {};
			billboardCloud.ProcessPrefab(prefab);
			billboardCloud.Generate(generateSettings);
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
