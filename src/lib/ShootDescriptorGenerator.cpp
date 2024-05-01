#include "ShootDescriptorGenerator.hpp"

#include "TreeDescriptor.hpp"

using namespace EcoSysLab;

void ShootDescriptorGenerator::Serialize(YAML::Emitter& out)
{
	if (!m_shootDescriptorOffsets.empty())
	{
		out << YAML::Key << "m_shootDescriptorOffsets" << YAML::BeginSeq;
		for (const auto& i : m_shootDescriptorOffsets)
		{
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

void ShootDescriptorGenerator::Deserialize(const YAML::Node& in)
{
	if (in["m_shootDescriptorOffsets"])
	{
		m_shootDescriptorOffsets.clear();
		const auto& inShootGrowthParametersOffsets = in["m_shootDescriptorOffsets"];
		for (const auto& inShootGrowthParametersOffset : inShootGrowthParametersOffsets)
		{
			m_shootDescriptorOffsets.emplace_back();
			auto& shootGrowthParameterOffsets = m_shootDescriptorOffsets.back();
			shootGrowthParameterOffsets.m_type = inShootGrowthParametersOffset["m_type"].as<unsigned>();
			shootGrowthParameterOffsets.m_range = inShootGrowthParametersOffset["m_range"].as<glm::vec2>();
			shootGrowthParameterOffsets.m_offset.Load("m_offset", inShootGrowthParametersOffset);
		}
	}
	m_baseShootDescriptor.Load("m_baseShootDescriptor", in);
}

void ShootDescriptorGenerator::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	bool changed = false;
	editorLayer->DragAndDropButton<TreeDescriptor>(m_baseShootDescriptor, "Base");
	if (OnInspectShootGrowthParametersOffset(m_shootDescriptorOffsets)) { changed = true; }
	if (changed) m_saved = false;
}

std::shared_ptr<ShootDescriptor> ShootDescriptorGenerator::Generate()
{
	const std::shared_ptr<ShootDescriptor> retVal = ProjectManager::CreateTemporaryAsset<ShootDescriptor>();
	for (const auto& i : m_shootDescriptorOffsets)
	{
		float a = i.m_offset.GetValue(glm::linearRand(0.0f, 1.0f));

		switch (i.m_type)
		{
		case static_cast<unsigned>(ShootGrowthParameterType::GrowthRate):
		{
			retVal->m_growthRate += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::BranchingAngleMean):
		{
			retVal->m_branchingAngleMeanVariance.x += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::BranchingAngleVariance):
		{
			retVal->m_branchingAngleMeanVariance.y += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::RollAngleMean):
		{
			retVal->m_rollAngleMeanVariance.x += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::RollAngleVariance):
		{
			retVal->m_rollAngleMeanVariance.y += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::ApicalAngleMean):
		{
			retVal->m_apicalAngleMeanVariance.x += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::ApicalAngleVariance):
		{
			retVal->m_apicalAngleMeanVariance.y += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::Gravitropism):
		{
			retVal->m_gravitropism += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::Phototropism):
		{
			retVal->m_phototropism += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::Sagging):
		{
			retVal->m_saggingFactorThicknessReductionMax.x += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::SaggingThicknessFactor):
		{
			retVal->m_saggingFactorThicknessReductionMax.y += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::MaxSagging):
		{
			retVal->m_saggingFactorThicknessReductionMax.z += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::InternodeLength):
		{
			retVal->m_internodeLength += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::InternodeLengthThicknessFactor):
		{
			retVal->m_internodeLengthThicknessFactor += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::EndNodeThickness):
		{
			retVal->m_endNodeThickness += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::ThicknessAccumulationFactor):
		{
			retVal->m_thicknessAccumulationFactor += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::ThicknessAgeFactor):
		{
			retVal->m_thicknessAgeFactor += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::ShadowFactor):
		{
			retVal->m_internodeShadowFactor += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::ApicalBudExtinctionRate):
		{
			retVal->m_apicalBudExtinctionRate += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::ApicalControl):
		{
			retVal->m_apicalControl += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::ApicalDominance):
		{
			retVal->m_apicalDominance += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::ApicalDominanceLoss):
		{
			retVal->m_apicalDominanceLoss += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::LowBranchPruning):
		{
			retVal->m_lowBranchPruning += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::LowBranchPruningThicknessFactor):
		{
			retVal->m_lowBranchPruningThicknessFactor += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::LightPruningFactor):
		{
			retVal->m_lightPruningFactor += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		case static_cast<unsigned>(ShootGrowthParameterType::ThicknessPruningFactor):
		{
			retVal->m_thicknessPruningFactor += glm::mix(i.m_range.x, i.m_range.y, a); break;
		}
		default:break;
		}
	}
	return retVal;
}

bool ShootDescriptorGenerator::OnInspectShootGrowthParametersOffset(
	std::vector<ShootGrowthParameterOffset>& shootGrowthParameterOffsets)
{
	bool changed = false;
	if (ImGui::TreeNodeEx("Shoot Parameter Offsets", ImGuiTreeNodeFlags_DefaultOpen))
	{
		if (ImGui::Button("New..."))
		{
			shootGrowthParameterOffsets.emplace_back();
			shootGrowthParameterOffsets.back().m_offset.SetStart(0.0f);
			shootGrowthParameterOffsets.back().m_offset.SetEnd(1.0f);
			changed = true;
		}
		for (int offsetIndex = 0; offsetIndex < shootGrowthParameterOffsets.size(); offsetIndex++)
		{
			auto& offset = shootGrowthParameterOffsets.at(offsetIndex);
			std::string tag = "Offset " + std::to_string(offsetIndex + 1);
			if (ImGui::TreeNode(tag.c_str())) {
				if (ImGui::Button("Remove"))
				{
					shootGrowthParameterOffsets.erase(shootGrowthParameterOffsets.begin() + offsetIndex);
					offsetIndex--;
					ImGui::TreePop();
					continue;
				}
				ImGui::SameLine();
				if (ImGui::Combo("Type", { "GrowthRate", "BranchingAngleMean", "BranchingAngleVariance",
					"RollAngleMean", "RollAngleVariance", "ApicalAngleMean", "ApicalAngleVariance", "Gravitropism", "Phototropism", "Sagging", "SaggingThicknessFactor",
					"MaxSagging", "InternodeLength", "InternodeLengthThicknessFactor", "EndNodeThickness", "ThicknessAccumulationFactor", "ThicknessAgeFactor", "ShadowFactor",
					"ApicalBudExtinctionRate", "LateralBudExtinctionRate", "ApicalBudLightingFactor",
					"LateralBudLightingFactor", "GrowthRateControl", "ApicalDominance", "ApicalDominanceLoss",
					"LowBranchPruning", "LowBranchPruningThicknessFactor", "LightPruningFactor", "ThicknessPruningFactor" }, offset.m_type)) {
					changed = true;
				}
				ImGui::DragFloat2("Range", &offset.m_range.x, 0.01f);
				if (offset.m_offset.OnInspect(tag, ImVec2(-1, -1), static_cast<unsigned>(CurveEditorFlags::ALLOW_RESIZE) | static_cast<unsigned>(CurveEditorFlags::SHOW_GRID) | static_cast<unsigned>(CurveEditorFlags::DISABLE_START_END_Y))) changed = true;
				ImGui::TreePop();
			}
		}
		ImGui::TreePop();
	}
	return changed;
}
