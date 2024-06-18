//
// Created by lllll on 10/25/2022.
//

#include "ForestDescriptor.hpp"
#include "Application.hpp"
#include "Climate.hpp"
#include "EcoSysLabLayer.hpp"
#include "EditorLayer.hpp"
#include "Graphics.hpp"
#include "Tree.hpp"
using namespace eco_sys_lab;

Entity ForestPatch::InstantiatePatch(const glm::ivec2& gridSize, const bool setSimulationSettings) {
  const auto scene = Application::GetActiveScene();
  std::shared_ptr<Soil> soil;
  const auto soilCandidate = EcoSysLabLayer::FindSoil();
  if (!soilCandidate.expired())
    soil = soilCandidate.lock();
  std::shared_ptr<SoilDescriptor> soilDescriptor;
  if (soil) {
    soilDescriptor = soil->soil_descriptor.Get<SoilDescriptor>();
  }
  std::shared_ptr<HeightField> heightField{};
  if (soilDescriptor) {
    heightField = soilDescriptor->height_field.Get<HeightField>();
  }
  const glm::vec2 startPoint =
      glm::vec2((gridSize.x - 1) * m_gridDistance.x, (gridSize.y - 1) * m_gridDistance.y) * 0.5f;

  const auto retVal = scene->CreateEntity("Forest (" + std::to_string(gridSize.x * gridSize.y) + ") - " + GetTitle());
  const auto forest = scene->CreateEntity("Center");
  const auto boundary = scene->CreateEntity("Boundary");
  scene->SetParent(forest, retVal);
  scene->SetParent(boundary, retVal);

  int index = 0;

  const auto offset = glm::linearRand(glm::vec3(-10000), glm::vec3(10000));
  for (int i = 0; i < gridSize.x; i++) {
    for (int j = 0; j < gridSize.y; j++) {
      auto position = glm::vec3(-startPoint.x + i * m_gridDistance.x, 0.0f, -startPoint.y + j * m_gridDistance.y);
      position.x +=
          glm::linearRand(-m_gridDistance.x * m_positionOffsetMean.x, m_gridDistance.x * m_positionOffsetMean.x);
      position.z +=
          glm::linearRand(-m_gridDistance.y * m_positionOffsetMean.y, m_gridDistance.y * m_positionOffsetMean.y);
      position +=
          glm::gaussRand(glm::vec3(0.0f), glm::vec3(m_positionOffsetVariance.x, 0.0f, m_positionOffsetVariance.y));
      if (heightField)
        position.y = heightField->GetValue({position.x, position.z}) - 0.01f;
      GlobalTransform transform{};
      transform.SetPosition(position);
      auto rotation = glm::quat(glm::radians(glm::vec3(glm::gaussRand(glm::vec3(0.0f), m_rotationOffsetVariance))));
      transform.SetRotation(rotation);
      transform.SetScale(glm::vec3(1.f));

      auto treeEntity = scene->CreateEntity("Tree No." + std::to_string(index));
      index++;

      scene->SetDataComponent(treeEntity, transform);
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
      tree->tree_model.tree_growth_settings = m_treeGrowthSettings;
      tree->tree_descriptor = m_treeDescriptor.Get<TreeDescriptor>();
      if (i == 0 || j == 0 || i == gridSize.x - 1 || j == gridSize.y - 1) {
        scene->SetParent(treeEntity, boundary);
        tree->generate_mesh = false;
      } else {
        scene->SetParent(treeEntity, forest);
        tree->generate_mesh = true;
      }
      tree->start_time = glm::linearRand(0.0f, m_startTimeMax);
      tree->low_branch_pruning = glm::mix(m_minLowBranchPruning, m_maxLowBranchPruning,
                                          glm::abs(glm::perlin(offset + transform.GetPosition())));
    }
  }

  if (setSimulationSettings) {
    const auto lab = Application::GetLayer<EcoSysLabLayer>();
    lab->m_simulationSettings = m_simulationSettings;
  }

  return retVal;
}

Entity ForestPatch::InstantiatePatch(
    const std::vector<std::pair<TreeGrowthSettings, std::shared_ptr<TreeDescriptor>>>& candidates,
    const glm::ivec2& gridSize, bool setSimulationSettings) const {
  const auto scene = Application::GetActiveScene();
  std::shared_ptr<Soil> soil;
  const auto soilCandidate = EcoSysLabLayer::FindSoil();
  if (!soilCandidate.expired())
    soil = soilCandidate.lock();
  std::shared_ptr<SoilDescriptor> soilDescriptor;
  if (soil) {
    soilDescriptor = soil->soil_descriptor.Get<SoilDescriptor>();
  }
  std::shared_ptr<HeightField> heightField{};
  if (soilDescriptor) {
    heightField = soilDescriptor->height_field.Get<HeightField>();
  }
  const glm::vec2 startPoint =
      glm::vec2((gridSize.x - 1) * m_gridDistance.x, (gridSize.y - 1) * m_gridDistance.y) * 0.5f;

  const auto retVal = scene->CreateEntity("Forest (" + std::to_string(gridSize.x * gridSize.y) + ") - " + GetTitle());
  const auto forest = scene->CreateEntity("Center");
  const auto boundary = scene->CreateEntity("Boundary");
  scene->SetParent(forest, retVal);
  scene->SetParent(boundary, retVal);

  int index = 0;

  const auto offset = glm::linearRand(glm::vec3(-10000), glm::vec3(10000));
  for (int i = 0; i < gridSize.x; i++) {
    for (int j = 0; j < gridSize.y; j++) {
      auto position = glm::vec3(-startPoint.x + i * m_gridDistance.x, 0.0f, -startPoint.y + j * m_gridDistance.y);
      position.x +=
          glm::linearRand(-m_gridDistance.x * m_positionOffsetMean.x, m_gridDistance.x * m_positionOffsetMean.x);
      position.z +=
          glm::linearRand(-m_gridDistance.y * m_positionOffsetMean.y, m_gridDistance.y * m_positionOffsetMean.y);
      position +=
          glm::gaussRand(glm::vec3(0.0f), glm::vec3(m_positionOffsetVariance.x, 0.0f, m_positionOffsetVariance.y));
      if (heightField)
        position.y = heightField->GetValue({position.x, position.z}) - 0.01f;
      GlobalTransform transform{};
      transform.SetPosition(position);
      auto rotation = glm::quat(glm::radians(glm::vec3(glm::gaussRand(glm::vec3(0.0f), m_rotationOffsetVariance))));
      transform.SetRotation(rotation);
      transform.SetScale(glm::vec3(1.f));

      auto treeEntity = scene->CreateEntity("Tree No." + std::to_string(index));
      index++;

      scene->SetDataComponent(treeEntity, transform);
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();

      const auto candidateIndex = glm::linearRand(0, static_cast<int>(candidates.size() - 1));
      tree->tree_model.tree_growth_settings = candidates.at(candidateIndex).first;
      tree->tree_descriptor = candidates.at(candidateIndex).second;  // m_treeDescriptor.Get<TreeDescriptor>();
      if (i == 0 || j == 0 || i == gridSize.x - 1 || j == gridSize.y - 1) {
        scene->SetParent(treeEntity, boundary);
        tree->generate_mesh = false;
      } else {
        scene->SetParent(treeEntity, forest);
        tree->generate_mesh = true;
      }
      tree->start_time = glm::linearRand(0.0f, m_startTimeMax);
      tree->low_branch_pruning = glm::mix(m_minLowBranchPruning, m_maxLowBranchPruning,
                                          glm::abs(glm::perlin(offset + transform.GetPosition())));
    }
  }

  if (setSimulationSettings) {
    const auto lab = Application::GetLayer<EcoSysLabLayer>();
    lab->m_simulationSettings = m_simulationSettings;
  }

  return retVal;
}

void ForestPatch::CollectAssetRef(std::vector<AssetRef>& list) {
  if (m_treeDescriptor.Get<TreeDescriptor>())
    list.push_back(m_treeDescriptor);
}

void ForestPatch::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "m_gridDistance" << YAML::Value << m_gridDistance;
  out << YAML::Key << "m_positionOffsetMean" << YAML::Value << m_positionOffsetMean;
  out << YAML::Key << "m_positionOffsetVariance" << YAML::Value << m_positionOffsetVariance;
  out << YAML::Key << "m_rotationOffsetVariance" << YAML::Value << m_rotationOffsetVariance;

  out << YAML::Key << "m_minLowBranchPruning" << YAML::Value << m_minLowBranchPruning;
  out << YAML::Key << "m_maxLowBranchPruning" << YAML::Value << m_maxLowBranchPruning;
  out << YAML::Key << "m_simulationTime" << YAML::Value << m_simulationTime;
  out << YAML::Key << "m_startTimeMax" << YAML::Value << m_startTimeMax;

  m_treeDescriptor.Save("m_treeDescriptor", out);

  m_simulationSettings.Save("m_simulationSettings", out);
}

void ForestPatch::Deserialize(const YAML::Node& in) {
  if (in["m_gridDistance"])
    m_gridDistance = in["m_gridDistance"].as<glm::vec2>();
  if (in["m_positionOffsetMean"])
    m_positionOffsetMean = in["m_positionOffsetMean"].as<glm::vec2>();
  if (in["m_positionOffsetVariance"])
    m_positionOffsetVariance = in["m_positionOffsetVariance"].as<glm::vec2>();
  if (in["m_rotationOffsetVariance"])
    m_rotationOffsetVariance = in["m_rotationOffsetVariance"].as<glm::vec3>();

  if (in["m_minLowBranchPruning"])
    m_minLowBranchPruning = in["m_minLowBranchPruning"].as<float>();
  if (in["m_maxLowBranchPruning"])
    m_maxLowBranchPruning = in["m_maxLowBranchPruning"].as<float>();
  if (in["m_simulationTime"])
    m_simulationTime = in["m_simulationTime"].as<float>();
  if (in["m_startTimeMax"])
    m_startTimeMax = in["m_startTimeMax"].as<float>();
  m_treeDescriptor.Load("m_treeDescriptor", in);

  m_simulationSettings.Load("m_simulationSettings", in);
}

bool ForestPatch::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
  bool changed = false;
  editorLayer->DragAndDropButton<TreeDescriptor>(m_treeDescriptor, "TreeDescriptor");
  static glm::ivec2 gridSize = {8, 8};
  ImGui::DragInt2("Grid size", &gridSize.x, 1, 0, 100);
  if (ImGui::DragFloat2("Grid distance", &m_gridDistance.x, 0.1f, 0.0f, 100.0f))
    changed = true;
  ImGui::Separator();
  if (ImGui::DragFloat2("Position offset mean", &m_positionOffsetMean.x, 0.01f, 0.0f, 5.f))
    changed = true;
  if (ImGui::DragFloat2("Position offset variance", &m_positionOffsetVariance.x, 0.01f, 0.0f, 5.f))
    changed = true;
  if (ImGui::DragFloat2("Rotation offset variance", &m_rotationOffsetVariance.x, 0.01f, 0.0f, 5.f))
    changed = true;
  static bool setParent = true;
  ImGui::Checkbox("Set Parent", &setParent);
  static bool setSimulationSettings = true;
  ImGui::Checkbox("Set Simulation settings", &setSimulationSettings);
  if (ImGui::TreeNodeEx("Simulation Settings", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (m_simulationSettings.OnInspect(editorLayer))
      changed = true;
    ImGui::TreePop();
  }

  if (ImGui::DragFloat("Min low branch pruning", &m_minLowBranchPruning, 0.01f, 0.f, m_maxLowBranchPruning))
    changed = true;
  if (ImGui::DragFloat("Max low branch pruning", &m_maxLowBranchPruning, 0.01f, m_minLowBranchPruning, 1.f))
    changed = true;
  if (ImGui::DragFloat("Simulation time", &m_simulationTime, 0.1f, 0.0f, 100.f))
    changed = true;
  if (ImGui::DragFloat("Start time max", &m_startTimeMax, 0.01f, 0.0f, 10.f))
    changed = true;

  if (ImGui::Button("Instantiate")) {
    InstantiatePatch(gridSize, setParent);
  }
  FileUtils::OpenFolder("Create forest from folder...", [&](const std::filesystem::path& folderPath) {
    int index = 0;
    const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
    std::shared_ptr<Soil> soil;
    const auto soilCandidate = EcoSysLabLayer::FindSoil();
    if (!soilCandidate.expired())
      soil = soilCandidate.lock();
    std::shared_ptr<SoilDescriptor> soilDescriptor;
    if (soil) {
      soilDescriptor = soil->soil_descriptor.Get<SoilDescriptor>();
    }
    std::shared_ptr<HeightField> heightField{};
    if (soilDescriptor) {
      heightField = soilDescriptor->height_field.Get<HeightField>();
    }
    std::vector<std::pair<TreeGrowthSettings, std::shared_ptr<TreeDescriptor>>> treeDescriptors;
    for (const auto& i : std::filesystem::recursive_directory_iterator(folderPath)) {
      if (i.is_regular_file() && i.path().extension().string() == ".tree") {
        const auto treeDescriptor = std::dynamic_pointer_cast<TreeDescriptor>(
            ProjectManager::GetOrCreateAsset(ProjectManager::GetPathRelativeToProject(i.path())));
        if (treeDescriptor) {
          treeDescriptors.emplace_back(std::make_pair(m_treeGrowthSettings, treeDescriptor));
        }
        index++;
      }
    }
    if (!treeDescriptors.empty()) {
      const auto patch = InstantiatePatch(treeDescriptors, gridSize, setParent);
    }
  });
  return changed;
}

void TreeInfo::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "m_globalTransform" << YAML::Value << m_globalTransform.value;
  m_treeDescriptor.Save("m_treeDescriptor", out);
}

void TreeInfo::Deserialize(const YAML::Node& in) {
  if (in["m_globalTransform"])
    m_globalTransform.value = in["m_globalTransform"].as<glm::mat4>();
  m_treeDescriptor.Load("m_treeDescriptor", in);
}

void TreeInfo::CollectAssetRef(std::vector<AssetRef>& list) const {
  list.push_back(m_treeDescriptor);
}

void ForestDescriptor::ApplyTreeDescriptor(const std::shared_ptr<TreeDescriptor>& treeDescriptor) {
  if (treeDescriptor) {
    for (auto& i : m_treeInfos) {
      i.m_treeDescriptor = treeDescriptor;
    }
  }
}

void ForestDescriptor::ApplyTreeDescriptors(const std::vector<std::shared_ptr<TreeDescriptor>>& treeDescriptors) {
  if (treeDescriptors.empty())
    return;
  for (auto& i : m_treeInfos) {
    i.m_treeDescriptor = treeDescriptors.at(glm::linearRand(0, static_cast<int>(treeDescriptors.size()) - 1));
  }
}

void ForestDescriptor::ApplyTreeDescriptors(const std::filesystem::path& folderPath) {
  std::vector<std::shared_ptr<TreeDescriptor>> collectedTreeDescriptors{};
  for (const auto& i : std::filesystem::recursive_directory_iterator(folderPath)) {
    if (i.is_regular_file() && i.path().extension().string() == ".tree") {
      const auto treeDescriptor = std::dynamic_pointer_cast<TreeDescriptor>(
          ProjectManager::GetOrCreateAsset(ProjectManager::GetPathRelativeToProject(i.path())));
      collectedTreeDescriptors.emplace_back(treeDescriptor);
    }
  }
  ApplyTreeDescriptors(collectedTreeDescriptors);
}

void ForestDescriptor::ApplyTreeDescriptors(const std::vector<std::shared_ptr<TreeDescriptor>>& treeDescriptors,
                                            const std::vector<float>& ratios) {
  if (treeDescriptors.empty())
    return;
  for (auto& i : m_treeInfos) {
    i.m_treeDescriptor = treeDescriptors.at(glm::linearRand(0, static_cast<int>(treeDescriptors.size()) - 1));
  }

  std::random_device rd;
  std::mt19937 g(rd());
  auto copiedDescriptors = treeDescriptors;
  std::shuffle(copiedDescriptors.begin(), copiedDescriptors.end(), g);
  std::vector<std::shared_ptr<TreeDescriptor>> appliedTreeDescriptors;
  int count = 0;
  for (int i = 0; i < ratios.size(); i++) {
    if (count >= m_treeInfos.size())
      break;
    const int localSize = m_treeInfos.size() * ratios[i];
    for (int j = 0; j < localSize; j++) {
      if (count >= m_treeInfos.size())
        break;
      m_treeInfos[count].m_treeDescriptor = copiedDescriptors[i];
      count++;
    }
  }
}

void ForestDescriptor::ApplyTreeDescriptors(const std::filesystem::path& folderPath, const std::vector<float>& ratios) {
  std::vector<std::shared_ptr<TreeDescriptor>> collectedTreeDescriptors{};
  for (const auto& i : std::filesystem::recursive_directory_iterator(folderPath)) {
    if (i.is_regular_file() && i.path().extension().string() == ".tree") {
      const auto treeDescriptor = std::dynamic_pointer_cast<TreeDescriptor>(
          ProjectManager::GetOrCreateAsset(ProjectManager::GetPathRelativeToProject(i.path())));
      collectedTreeDescriptors.emplace_back(treeDescriptor);
    }
  }
  ApplyTreeDescriptors(collectedTreeDescriptors, ratios);
}

bool ForestDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
  bool changed = false;
  static glm::ivec2 gridSize = {4, 4};
  static float gridDistance = 1.5f;
  static float randomShift = 0.5f;
  static bool setParent = true;
  static bool enableHistory = false;
  static int historyIteration = 30;
  ImGui::Checkbox("Enable history", &enableHistory);
  if (enableHistory)
    ImGui::DragInt("History iteration", &historyIteration, 1, 1, 999);
  if (ImGui::TreeNodeEx("Grid...", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::DragInt2("Grid size", &gridSize.x, 1, 0, 100);
    ImGui::DragFloat("Grid distance", &gridDistance, 0.1f, 0.0f, 100.0f);
    ImGui::DragFloat("Random shift", &randomShift, 0.01f, 0.0f, 0.5f);
    if (ImGui::Button("Reset Grid")) {
      SetupGrid(gridSize, gridDistance, randomShift);
    }
    ImGui::TreePop();
  }

  FileUtils::OpenFolder(
      "Parameters sample",
      [&](const std::filesystem::path& path) {
        int index = 0;
        const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
        std::shared_ptr<Soil> soil;
        const auto soilCandidate = EcoSysLabLayer::FindSoil();
        if (!soilCandidate.expired())
          soil = soilCandidate.lock();
        std::shared_ptr<SoilDescriptor> soilDescriptor;
        if (soil) {
          soilDescriptor = soil->soil_descriptor.Get<SoilDescriptor>();
        }
        std::shared_ptr<HeightField> heightField{};
        if (soilDescriptor) {
          heightField = soilDescriptor->height_field.Get<HeightField>();
        }
        for (const auto& i : std::filesystem::recursive_directory_iterator(path)) {
          if (i.is_regular_file() && i.path().extension().string() == ".tree") {
            const auto treeDescriptor = std::dynamic_pointer_cast<TreeDescriptor>(
                ProjectManager::GetOrCreateAsset(ProjectManager::GetPathRelativeToProject(i.path())));
            m_treeInfos.emplace_back();
            glm::vec3 position = glm::vec3(5.f * index, 0.0f, 0.0f);
            if (heightField)
              position.y = heightField->GetValue({position.x, position.z}) - 0.05f;
            m_treeInfos.back().m_globalTransform.SetPosition(position);
            m_treeInfos.back().m_treeDescriptor = treeDescriptor;
            index++;
          }
        }
      },
      false);

  FileUtils::OpenFolder(
      "Randomly assign tree descriptors",
      [&](const std::filesystem::path& path) {
        ApplyTreeDescriptors(path);
      },
      false);
  static AssetRef treeDescriptorRef;
  if (editorLayer->DragAndDropButton<TreeDescriptor>(treeDescriptorRef, "Apply all with tree descriptor...", true)) {
    if (const auto treeDescriptor = treeDescriptorRef.Get<TreeDescriptor>()) {
      ApplyTreeDescriptor(treeDescriptor);
    }
    treeDescriptorRef.Clear();
  }

  if (ImGui::TreeNode("Tree Instances")) {
    int index = 1;
    for (auto& i : m_treeInfos) {
      editorLayer->DragAndDropButton<TreeDescriptor>(i.m_treeDescriptor, "Tree No." + std::to_string(index), true);
      index++;
    }
    ImGui::TreePop();
  }

  if (ImGui::Button("Instantiate patch")) {
    InstantiatePatch(setParent);
  }

  if (!m_treeInfos.empty() && ImGui::Button("Clear")) {
    m_treeInfos.clear();
  }

  return changed;
}

void ForestDescriptor::OnCreate() {
}

void ForestDescriptor::CollectAssetRef(std::vector<AssetRef>& list) {
  for (const auto& i : m_treeInfos) {
    i.CollectAssetRef(list);
  }
}

void ForestDescriptor::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "m_treeInfos" << YAML::BeginSeq;
  for (const auto& i : m_treeInfos) {
    i.Serialize(out);
  }
  out << YAML::EndSeq;

  out << YAML::Key << "m_treeGrowthSettings" << YAML::Value << YAML::BeginMap;
  Tree::SerializeTreeGrowthSettings(m_treeGrowthSettings, out);
  out << YAML::EndMap;
}

void ForestDescriptor::Deserialize(const YAML::Node& in) {
  if (in["m_treeInfos"]) {
    m_treeInfos.clear();
    for (const auto& i : in["m_treeInfos"]) {
      m_treeInfos.emplace_back();
      auto& back = m_treeInfos.back();
      back.Deserialize(i);
    }
  }

  if (in["m_treeGrowthSettings"]) {
    Tree::DeserializeTreeGrowthSettings(m_treeGrowthSettings, in["m_treeGrowthSettings"]);
  }
}

void ForestDescriptor::SetupGrid(const glm::ivec2& gridSize, float gridDistance, float randomShift) {
  m_treeInfos.clear();
  const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
  std::shared_ptr<Soil> soil;
  const auto soilCandidate = EcoSysLabLayer::FindSoil();
  if (!soilCandidate.expired())
    soil = soilCandidate.lock();
  std::shared_ptr<SoilDescriptor> soilDescriptor;
  if (soil) {
    soilDescriptor = soil->soil_descriptor.Get<SoilDescriptor>();
  }
  std::shared_ptr<HeightField> heightField{};
  if (soilDescriptor) {
    heightField = soilDescriptor->height_field.Get<HeightField>();
  }
  const glm::vec2 startPoint = glm::vec2((gridSize.x - 0.5f) * gridDistance, (gridSize.y - 0.5f) * gridDistance) * 0.5f;
  for (int i = 0; i < gridSize.x; i++) {
    for (int j = 0; j < gridSize.y; j++) {
      m_treeInfos.emplace_back();
      glm::vec3 position = glm::vec3(-startPoint.x + i * gridDistance, 0.0f, -startPoint.y + j * gridDistance);
      position.x += glm::linearRand(-gridDistance * randomShift, gridDistance * randomShift);
      position.z += glm::linearRand(-gridDistance * randomShift, gridDistance * randomShift);
      if (heightField)
        position.y = heightField->GetValue({position.x, position.z}) - 0.05f;
      m_treeInfos.back().m_globalTransform.SetPosition(position);
    }
  }
}

void ForestDescriptor::InstantiatePatch(const bool setParent) {
  const auto scene = Application::GetActiveScene();
  Entity parent;
  if (setParent) {
    parent = scene->CreateEntity("Forest (" + std::to_string(m_treeInfos.size()) + ") - " + GetTitle());
  }
  int i = 0;
  for (const auto& gt : m_treeInfos) {
    auto treeEntity = scene->CreateEntity("Tree No." + std::to_string(i));
    i++;
    scene->SetDataComponent(treeEntity, gt.m_globalTransform);
    const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
    tree->tree_model.tree_growth_settings = m_treeGrowthSettings;
    tree->tree_descriptor = gt.m_treeDescriptor;
    if (setParent)
      scene->SetParent(treeEntity, parent);
  }
}
