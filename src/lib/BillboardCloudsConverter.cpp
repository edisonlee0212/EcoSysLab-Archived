#include "BillboardCloudsConverter.hpp"
#include "Prefab.hpp"
using namespace EcoSysLab;

bool BillboardCloudsConverter::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	bool changed = false;
	const auto scene = GetScene();
	static BillboardCloud::GenerateSettings billboardCloudGenerateSettings{};
	billboardCloudGenerateSettings.OnInspect("Billboard clouds generation settings");

	if (ImGui::TreeNodeEx("For Prefab", ImGuiTreeNodeFlags_DefaultOpen)) {

		static AssetRef prefabRef;
		if (EditorLayer::DragAndDropButton<Prefab>(prefabRef, "Drop prefab here...")) {
			if (const auto prefab = prefabRef.Get<Prefab>())
			{
				BillboardCloud billboardCloud{};
				billboardCloud.ProcessPrefab(prefab);
				billboardCloud.Generate(billboardCloudGenerateSettings);
				const auto entity = billboardCloud.BuildEntity(scene);
				if (scene->IsEntityValid(entity)) scene->SetEntityName(entity, " Billboard cloud (" + prefab->GetTitle() + ")");
				else
				{
					EVOENGINE_ERROR("Failed to build billboard cloud!");
				}
			}
			prefabRef.Clear();
		}
		ImGui::TreePop();
	}

	if (ImGui::TreeNodeEx("For Entity", ImGuiTreeNodeFlags_DefaultOpen)) {
		static EntityRef entityRef;

		if (EditorLayer::DragAndDropButton(entityRef, "Drop Entity here..."))
		{
			const auto entity = entityRef.Get();
			if (scene->IsEntityValid(entity))
			{
				BillboardCloud billboardCloud{};
				billboardCloud.ProcessEntity(scene, entity);
				billboardCloud.Generate(billboardCloudGenerateSettings);
				const auto billboardEntity = billboardCloud.BuildEntity(scene);
				if (scene->IsEntityValid(billboardEntity)) scene->SetEntityName(billboardEntity,  " Billboard cloud (" + scene->GetEntityName(entity) + ")");
				else
				{
					EVOENGINE_ERROR("Failed to build billboard cloud!");
				}
			}
			entityRef.Clear();
		}
		ImGui::TreePop();
	}
	return changed;
}
