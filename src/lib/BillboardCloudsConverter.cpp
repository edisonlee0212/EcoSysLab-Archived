#include "BillboardCloudsConverter.hpp"

#include <PointCloud.hpp>

#include "Prefab.hpp"
using namespace eco_sys_lab;

bool BillboardCloudsConverter::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  const auto scene = GetScene();
  static BillboardCloud::GenerateSettings billboardCloudGenerateSettings{};
  billboardCloudGenerateSettings.OnInspect("Billboard clouds generation settings");

  if (ImGui::TreeNodeEx("Mesh -> Billboard Clouds", ImGuiTreeNodeFlags_DefaultOpen)) {
    static AssetRef meshRef;
    if (EditorLayer::DragAndDropButton<Mesh>(meshRef, "Drop mesh here...")) {
      if (const auto mesh = meshRef.Get<Mesh>()) {
        BillboardCloud billboardCloud{};
        billboardCloud.Process(mesh, ProjectManager::CreateTemporaryAsset<Material>());
        billboardCloud.Generate(billboardCloudGenerateSettings);
        const auto entity = billboardCloud.BuildEntity(scene);
        if (scene->IsEntityValid(entity))
          scene->SetEntityName(entity, "Billboard cloud (" + mesh->GetTitle() + ")");
        else {
          EVOENGINE_ERROR("Failed to build billboard cloud!");
        }
      }
      meshRef.Clear();
    }
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Prefab -> Billboard Clouds", ImGuiTreeNodeFlags_DefaultOpen)) {
    static AssetRef prefabRef;
    if (EditorLayer::DragAndDropButton<Prefab>(prefabRef, "Drop prefab here...")) {
      if (const auto prefab = prefabRef.Get<Prefab>()) {
        BillboardCloud billboardCloud{};
        billboardCloud.Process(prefab);
        billboardCloud.Generate(billboardCloudGenerateSettings);
        const auto entity = billboardCloud.BuildEntity(scene);
        if (scene->IsEntityValid(entity))
          scene->SetEntityName(entity, "Billboard cloud (" + prefab->GetTitle() + ")");
        else {
          EVOENGINE_ERROR("Failed to build billboard cloud!");
        }
      }
      prefabRef.Clear();
    }
    ImGui::TreePop();
  }

  if (ImGui::TreeNodeEx("Entity -> Billboard Clouds", ImGuiTreeNodeFlags_DefaultOpen)) {
    static EntityRef entityRef;

    if (EditorLayer::DragAndDropButton(entityRef, "Drop Entity here...")) {
      const auto entity = entityRef.Get();
      if (scene->IsEntityValid(entity)) {
        BillboardCloud billboardCloud{};
        billboardCloud.Process(scene, entity);
        billboardCloud.Generate(billboardCloudGenerateSettings);
        const auto billboardEntity = billboardCloud.BuildEntity(scene);
        if (scene->IsEntityValid(billboardEntity))
          scene->SetEntityName(billboardEntity, "Billboard cloud (" + scene->GetEntityName(entity) + ")");
        else {
          EVOENGINE_ERROR("Failed to build billboard cloud!");
        }
      }
      entityRef.Clear();
    }
    ImGui::TreePop();
  }
  static std::vector<glm::vec3> points;
  if (ImGui::TreeNodeEx("Entity -> Point Clouds", ImGuiTreeNodeFlags_DefaultOpen)) {
    static EntityRef entityRef;

    if (EditorLayer::DragAndDropButton(entityRef, "Drop Entity here...")) {
      const auto entity = entityRef.Get();
      if (scene->IsEntityValid(entity)) {
        BillboardCloud billboardCloud{};
        billboardCloud.Process(scene, entity);
        points = billboardCloud.ExtractPointCloud(0.005f);
      }
      entityRef.Clear();
    }
    ImGui::TreePop();
  }
  if (!points.empty()) {
    FileUtils::SaveFile(
        "Save point cloud...", "Point Cloud", {".ply"},
        [&](const std::filesystem::path& path) {
          const auto pointCloud = ProjectManager::CreateTemporaryAsset<PointCloud>();
          pointCloud->positions.resize(points.size());
          Jobs::RunParallelFor(points.size(), [&](const unsigned pointIndex) {
            pointCloud->positions[pointIndex] = glm::dvec3(points[pointIndex]);
          });
          pointCloud->has_positions = true;
          PointCloud::PointCloudSaveSettings saveSettings{};
          saveSettings.binary = false;
          saveSettings.double_precision = false;
          if (pointCloud->Save(saveSettings, path)) {
            EVOENGINE_LOG("PointCloud Saved!")
          }
          points.clear();
        },
        false);
  }

  if (ImGui::TreeNodeEx("Entity -> Color by distance", ImGuiTreeNodeFlags_DefaultOpen)) {
    static EntityRef entityRef;

    if (EditorLayer::DragAndDropButton(entityRef, "Drop Entity here...")) {
      const auto entity = entityRef.Get();
      if (scene->IsEntityValid(entity)) {
        BillboardCloud billboardCloud{};
        billboardCloud.Process(scene, entity);
        for (auto& element : billboardCloud.elements) {
          const auto levelSet = element.CalculateLevelSets();
          Entity clone = scene->CreateEntity("Cloned");
          const auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(clone).lock();
          const auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
          VertexAttributes attributes{};
          attributes.color = true;
          attributes.normal = true;
          attributes.tangent = true;
          mesh->SetVertices(attributes, element.vertices, element.triangles);
          meshRenderer->mesh = mesh;

          const auto material = ProjectManager::CreateTemporaryAsset<Material>();
          material->vertex_color_only = true;
          meshRenderer->material = material;
        }
      }
      entityRef.Clear();
    }
    ImGui::TreePop();
  }

  return changed;
}
