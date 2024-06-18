//
// Created by lllll on 10/24/2022.
//

#include "Tree.hpp"

#include "Application.hpp"
#include "Climate.hpp"
#include "EcoSysLabLayer.hpp"
#include "EditorLayer.hpp"
#include "HeightField.hpp"
#include "Material.hpp"
#include "Octree.hpp"
#include "Soil.hpp"
#include "Strands.hpp"
#include "StrandsRenderer.hpp"
#include "TreeDescriptor.hpp"
#include "TreeMeshGenerator.hpp"

#include "BarkDescriptor.hpp"
#include "FlowerDescriptor.hpp"
#include "FoliageDescriptor.hpp"
#include "FruitDescriptor.hpp"
#include "ShootDescriptor.hpp"
using namespace eco_sys_lab;

void TreeDescriptor::OnCreate() {
}

bool TreeDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  const auto eco_sys_lab_layer = Application::GetLayer<EcoSysLabLayer>();
  std::shared_ptr<Climate> climate;
  std::shared_ptr<Soil> soil;
  if (const auto climate_candidate = EcoSysLabLayer::FindClimate(); !climate_candidate.expired())
    climate = climate_candidate.lock();
  if (const auto soil_candidate = EcoSysLabLayer::FindSoil(); !soil_candidate.expired())
    soil = soil_candidate.lock();
  if (soil && climate) {
    if (ImGui::Button("Instantiate")) {
      const auto scene = Application::GetActiveScene();
      const auto tree_entity = scene->CreateEntity(GetTitle());
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
      float height = 0;
      if (const auto soil_descriptor = soil->soil_descriptor.Get<SoilDescriptor>()) {
        if (const auto height_field = soil_descriptor->height_field.Get<HeightField>())
          height = height_field->GetValue({0.0f, 0.0f}) - 0.05f;
      }
      GlobalTransform global_transform;
      global_transform.SetPosition(glm::vec3(0, height, 0));
      scene->SetDataComponent(tree_entity, global_transform);
      tree->tree_descriptor = ProjectManager::GetAsset(GetHandle());
      editor_layer->SetSelectedEntity(tree_entity);
    }
  } else {
    ImGui::Text("Create soil and climate entity to instantiate!");
  }
  if (editor_layer->DragAndDropButton<ShootDescriptor>(m_shootDescriptor, "Shoot Descriptor"))
    changed = true;
  if (editor_layer->DragAndDropButton<FoliageDescriptor>(m_foliageDescriptor, "Foliage Descriptor"))
    changed = true;
  if (editor_layer->DragAndDropButton<FruitDescriptor>(m_fruitDescriptor, "Fruit Descriptor"))
    changed = true;
  if (editor_layer->DragAndDropButton<FlowerDescriptor>(m_flowerDescriptor, "Flower Descriptor"))
    changed = true;

  editor_layer->DragAndDropButton<BarkDescriptor>(m_barkDescriptor, "BarkDescriptor");
  return changed;
}

void TreeDescriptor::CollectAssetRef(std::vector<AssetRef>& list) {
  if (m_shootDescriptor.Get<ShootDescriptor>())
    list.push_back(m_shootDescriptor);
  if (m_foliageDescriptor.Get<FoliageDescriptor>())
    list.push_back(m_foliageDescriptor);
  if (m_fruitDescriptor.Get<FruitDescriptor>())
    list.push_back(m_fruitDescriptor);
  if (m_flowerDescriptor.Get<FlowerDescriptor>())
    list.push_back(m_flowerDescriptor);

  if (m_barkDescriptor.Get<BarkDescriptor>())
    list.push_back(m_barkDescriptor);
}

void TreeDescriptor::Serialize(YAML::Emitter& out) const {
  m_shootDescriptor.Save("m_shootDescriptor", out);
  m_foliageDescriptor.Save("m_foliageDescriptor", out);
  m_barkDescriptor.Save("m_barkDescriptor", out);

  m_fruitDescriptor.Save("m_fruitDescriptor", out);
  m_flowerDescriptor.Save("m_flowerDescriptor", out);
}

void TreeDescriptor::Deserialize(const YAML::Node& in) {
  m_shootDescriptor.Load("m_shootDescriptor", in);
  m_foliageDescriptor.Load("m_foliageDescriptor", in);
  m_barkDescriptor.Load("m_barkDescriptor", in);

  m_fruitDescriptor.Load("m_fruitDescriptor", in);
  m_flowerDescriptor.Load("m_flowerDescriptor", in);
}