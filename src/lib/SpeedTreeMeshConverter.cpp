#include "SpeedTreeMeshConverter.hpp"
#include "Prefab.hpp"
using namespace EcoSysLab;

void SpeedTreeMeshConverter::Convert(const std::shared_ptr<Prefab>& currentPrefab, BillboardCloud::Cluster& cluster, const Transform& parentModelSpaceTransform)
{
	Transform transform{};
	for (const auto& dataComponent : currentPrefab->m_dataComponents)
	{
		if (dataComponent.m_type == Typeof<Transform>())
		{
			transform = *std::reinterpret_pointer_cast<Transform>(dataComponent.m_data);
		}
	}
	transform.m_value = parentModelSpaceTransform.m_value * transform.m_value;
	for (const auto& privateComponent : currentPrefab->m_privateComponents)
	{

		if (privateComponent.m_data->GetTypeName() == "MeshRenderer")
		{
			std::vector<AssetRef> assetRefs;
			privateComponent.m_data->CollectAssetRef(assetRefs);
			std::shared_ptr<Mesh> mesh{};
			std::shared_ptr<Material> material{};
			for (auto& assetRef : assetRefs)
			{
				if (const auto testMesh = assetRef.Get<Mesh>())
				{
					mesh = testMesh;
				}
				else if (const auto testMaterial = assetRef.Get<Material>())
				{
					material = testMaterial;
				}
			}
			if (mesh && material)
			{
				cluster.m_elements.emplace_back();
				auto& element = cluster.m_elements.back();
				element.m_content = std::make_shared<BillboardCloud::RenderContent>();
				element.m_content->m_mesh = mesh;
				element.m_content->m_material = material;
				element.m_content->m_triangles = mesh->UnsafeGetTriangles();
				element.m_modelSpaceTransform.m_value = transform.m_value;
			}
		}
	}
	for (const auto& childPrefab : currentPrefab->m_children)
	{
		Convert(childPrefab, cluster, transform);
	}
}

void SpeedTreeMeshConverter::Convert(const BillboardCloud::ProjectSettings& projectSettings)
{
	const auto originalModel = m_originalModel.Get<Prefab>();
	BillboardCloud::Cluster cluster;
	Convert(originalModel, cluster, Transform());

	const auto scene = GetScene();
	const auto owner = GetOwner();
	const auto projectedCluster = BillboardCloud::Project(cluster, projectSettings);
	const auto projectedClusterEntity = scene->CreateEntity("Projected Tree");
	scene->SetParent(projectedClusterEntity, owner);
	{
		const auto projectedElementEntity = scene->CreateEntity("Projected Billboard");
		const auto elementMeshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(projectedElementEntity).lock();
		elementMeshRenderer->m_mesh = projectedCluster.m_mesh;
		elementMeshRenderer->m_material = projectedCluster.m_material;
		scene->SetParent(projectedElementEntity, projectedClusterEntity);
	}
}

bool SpeedTreeMeshConverter::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	bool changed = false;
	if (EditorLayer::DragAndDropButton<Prefab>(m_originalModel, "SpeedTree Model")) changed = true;
	if (m_originalModel.Get<Prefab>())
	{
		static BillboardCloud::ProjectSettings settings;
		ImGui::Checkbox("Albedo map", &settings.m_transferAlbedoMap);
		ImGui::Checkbox("Normal map", &settings.m_transferNormalMap);
		ImGui::Checkbox("Roughness map", &settings.m_transferRoughnessMap);
		ImGui::Checkbox("Metallic map", &settings.m_transferMetallicMap);
		ImGui::Checkbox("AO map", &settings.m_transferAoMap);

		ImGui::DragFloat("Size factor", &settings.m_resolutionFactor, 1.f, 1, 1024);
		if (ImGui::Button("Convert")) {
			Convert(settings);
		}
	}
	EditorLayer::DragAndDropButton<Mesh>(m_foliageMesh, "Foliage mesh");
	EditorLayer::DragAndDropButton<Mesh>(m_branchMesh, "Branch mesh");
	EditorLayer::DragAndDropButton<Mesh>(m_foliageMaterial, "Foliage material");
	EditorLayer::DragAndDropButton<Mesh>(m_branchMaterial, "Branch material");

	return changed;
}
