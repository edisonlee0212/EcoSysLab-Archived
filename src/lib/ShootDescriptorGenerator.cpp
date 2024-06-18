#include "ShootDescriptorGenerator.hpp"

#include "TreeDescriptor.hpp"

using namespace eco_sys_lab;

void ShootDescriptorGenerator::Serialize(YAML::Emitter& out) const {
  if (!m_shootDescriptorOffsets.empty()) {
    out << YAML::Key << "m_shootDescriptorOffsets" << YAML::BeginSeq;
    for (const auto& i : m_shootDescriptorOffsets) {
      out << YAML::BeginMap;
      out << YAML::Key << "m_type" << YAML::Value << i.m_type;
      out << YAML::Key << "m_range" << YAML::Value << i.m_range;
      i.m_offset.Save("m_offset", out);
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }
  m_baseShootDescriptor.Save("m_baseShootDescriptor", out);
}

void ShootDescriptorGenerator::Deserialize(const YAML::Node& in) {
  if (in["m_shootDescriptorOffsets"]) {
    m_shootDescriptorOffsets.clear();
    const auto& in_shoot_growth_parameters_offsets = in["m_shootDescriptorOffsets"];
    for (const auto& in_shoot_growth_parameters_offset : in_shoot_growth_parameters_offsets) {
      m_shootDescriptorOffsets.emplace_back();
      auto& shoot_growth_parameter_offsets = m_shootDescriptorOffsets.back();
      shoot_growth_parameter_offsets.m_type = in_shoot_growth_parameters_offset["m_type"].as<unsigned>();
      shoot_growth_parameter_offsets.m_range = in_shoot_growth_parameters_offset["m_range"].as<glm::vec2>();
      shoot_growth_parameter_offsets.m_offset.Load("m_offset", in_shoot_growth_parameters_offset);
    }
  }
  m_baseShootDescriptor.Load("m_baseShootDescriptor", in);
}

bool ShootDescriptorGenerator::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  editor_layer->DragAndDropButton<TreeDescriptor>(m_baseShootDescriptor, "Base");
  if (OnInspectShootGrowthParametersOffset(m_shootDescriptorOffsets)) {
    changed = true;
  }
  return changed;
}

std::shared_ptr<ShootDescriptor> ShootDescriptorGenerator::Generate() {
  const std::shared_ptr<ShootDescriptor> ret_val = ProjectManager::CreateTemporaryAsset<ShootDescriptor>();
  for (const auto& i : m_shootDescriptorOffsets) {
    float a = i.m_offset.GetValue(glm::linearRand(0.0f, 1.0f));

    switch (i.m_type) {
      case static_cast<unsigned>(ShootGrowthParameterType::GrowthRate): {
        ret_val->m_growthRate += glm::mix(i.m_range.x, i.m_range.y, a);
        break;
      }
      case static_cast<unsigned>(ShootGrowthParameterType::BranchingAngleMean): {
        ret_val->m_branchingAngleMeanVariance.x += glm::mix(i.m_range.x, i.m_range.y, a);
        break;
      }
      case static_cast<unsigned>(ShootGrowthParameterType::BranchingAngleVariance): {
        ret_val->m_branchingAngleMeanVariance.y += glm::mix(i.m_range.x, i.m_range.y, a);
        break;
      }
      case static_cast<unsigned>(ShootGrowthParameterType::RollAngleMean): {
        ret_val->m_rollAngleMeanVariance.x += glm::mix(i.m_range.x, i.m_range.y, a);
        break;
      }
      case static_cast<unsigned>(ShootGrowthParameterType::RollAngleVariance): {
        ret_val->m_rollAngleMeanVariance.y += glm::mix(i.m_range.x, i.m_range.y, a);
        break;
      }
      case static_cast<unsigned>(ShootGrowthParameterType::ApicalAngleMean): {
        ret_val->m_apicalAngleMeanVariance.x += glm::mix(i.m_range.x, i.m_range.y, a);
        break;
      }
      case static_cast<unsigned>(ShootGrowthParameterType::ApicalAngleVariance): {
        ret_val->m_apicalAngleMeanVariance.y += glm::mix(i.m_range.x, i.m_range.y, a);
        break;
      }
      case static_cast<unsigned>(ShootGrowthParameterType::Gravitropism): {
        ret_val->m_gravitropism += glm::mix(i.m_range.x, i.m_range.y, a);
        break;
      }
      case static_cast<unsigned>(ShootGrowthParameterType::Phototropism): {
        ret_val->m_phototropism += glm::mix(i.m_range.x, i.m_range.y, a);
        break;
      }
      case static_cast<unsigned>(ShootGrowthParameterType::InternodeLength): {
        ret_val->m_internodeLength += glm::mix(i.m_range.x, i.m_range.y, a);
        break;
      }
      case static_cast<unsigned>(ShootGrowthParameterType::InternodeLengthThicknessFactor): {
        ret_val->m_internodeLengthThicknessFactor += glm::mix(i.m_range.x, i.m_range.y, a);
        break;
      }
      case static_cast<unsigned>(ShootGrowthParameterType::EndNodeThickness): {
        ret_val->m_endNodeThickness += glm::mix(i.m_range.x, i.m_range.y, a);
        break;
      }
      case static_cast<unsigned>(ShootGrowthParameterType::ThicknessAccumulationFactor): {
        ret_val->m_thicknessAccumulationFactor += glm::mix(i.m_range.x, i.m_range.y, a);
        break;
      }
      case static_cast<unsigned>(ShootGrowthParameterType::ThicknessAgeFactor): {
        ret_val->m_thicknessAgeFactor += glm::mix(i.m_range.x, i.m_range.y, a);
        break;
      }
      case static_cast<unsigned>(ShootGrowthParameterType::ShadowFactor): {
        ret_val->m_internodeShadowFactor += glm::mix(i.m_range.x, i.m_range.y, a);
        break;
      }
      case static_cast<unsigned>(ShootGrowthParameterType::ApicalBudExtinctionRate): {
        ret_val->m_apicalBudExtinctionRate += glm::mix(i.m_range.x, i.m_range.y, a);
        break;
      }
      case static_cast<unsigned>(ShootGrowthParameterType::ApicalControl): {
        ret_val->m_apicalControl += glm::mix(i.m_range.x, i.m_range.y, a);
        break;
      }
      case static_cast<unsigned>(ShootGrowthParameterType::ApicalDominance): {
        ret_val->m_apicalDominance += glm::mix(i.m_range.x, i.m_range.y, a);
        break;
      }
      case static_cast<unsigned>(ShootGrowthParameterType::ApicalDominanceLoss): {
        ret_val->m_apicalDominanceLoss += glm::mix(i.m_range.x, i.m_range.y, a);
        break;
      }

      default:
        break;
    }
  }
  return ret_val;
}

bool ShootDescriptorGenerator::OnInspectShootGrowthParametersOffset(
    std::vector<ShootGrowthParameterOffset>& shoot_growth_parameter_offsets) {
  bool changed = false;
  if (ImGui::TreeNodeEx("Shoot Parameter Offsets", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::Button("New...")) {
      shoot_growth_parameter_offsets.emplace_back();
      shoot_growth_parameter_offsets.back().m_offset.SetStart(0.0f);
      shoot_growth_parameter_offsets.back().m_offset.SetEnd(1.0f);
      changed = true;
    }
    for (int offset_index = 0; offset_index < shoot_growth_parameter_offsets.size(); offset_index++) {
      auto& offset = shoot_growth_parameter_offsets.at(offset_index);
      std::string tag = "Offset " + std::to_string(offset_index + 1);
      if (ImGui::TreeNode(tag.c_str())) {
        if (ImGui::Button("Remove")) {
          shoot_growth_parameter_offsets.erase(shoot_growth_parameter_offsets.begin() + offset_index);
          offset_index--;
          ImGui::TreePop();
          continue;
        }
        ImGui::SameLine();
        if (ImGui::Combo("Type",
                         {"GrowthRate",
                          "BranchingAngleMean",
                          "BranchingAngleVariance",
                          "RollAngleMean",
                          "RollAngleVariance",
                          "ApicalAngleMean",
                          "ApicalAngleVariance",
                          "Gravitropism",
                          "Phototropism",
                          "Sagging",
                          "SaggingThicknessFactor",
                          "MaxSagging",
                          "InternodeLength",
                          "InternodeLengthThicknessFactor",
                          "EndNodeThickness",
                          "ThicknessAccumulationFactor",
                          "ThicknessAgeFactor",
                          "ShadowFactor",
                          "ApicalBudExtinctionRate",
                          "LateralBudExtinctionRate",
                          "ApicalBudLightingFactor",
                          "LateralBudLightingFactor",
                          "GrowthRateControl",
                          "ApicalDominance",
                          "ApicalDominanceLoss",
                          "LowBranchPruning",
                          "LowBranchPruningThicknessFactor",
                          "LightPruningFactor",
                          "ThicknessPruningFactor"},
                         offset.m_type)) {
          changed = true;
        }
        ImGui::DragFloat2("Range", &offset.m_range.x, 0.01f);
        if (offset.m_offset.OnInspect(tag, ImVec2(-1, -1),
                                      static_cast<unsigned>(CurveEditorFlags::AllowResize) |
                                          static_cast<unsigned>(CurveEditorFlags::ShowGrid) |
                                          static_cast<unsigned>(CurveEditorFlags::DisableStartEndY)))
          changed = true;
        ImGui::TreePop();
      }
    }
    ImGui::TreePop();
  }
  return changed;
}
