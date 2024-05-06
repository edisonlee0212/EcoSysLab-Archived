#include "FoliageDescriptor.hpp"
#include "TreeModel.hpp"
using namespace EcoSysLab;

void TwigParameters::Save(const std::string& name, YAML::Emitter& out)
{
	out << YAML::Key << name << YAML::BeginMap;
	out << YAML::Key << "m_segmentLength" << YAML::Value << m_segmentLength;
	out << YAML::Key << "m_apicalAngleVariance" << YAML::Value << m_apicalAngleVariance;
	out << YAML::Key << "m_branchingAngle" << YAML::Value << m_branchingAngle;
	out << YAML::Key << "m_thickness" << YAML::Value << m_thickness;
	out << YAML::Key << "m_maxNodeThickness" << YAML::Value << m_maxNodeThickness;
	out << YAML::Key << "m_minRootDistance" << YAML::Value << m_minRootDistance;
	out << YAML::Key << "m_maxEndDistance" << YAML::Value << m_maxEndDistance;
	out << YAML::Key << "m_segmentSize" << YAML::Value << m_segmentSize;
	out << YAML::Key << "m_unitDistance" << YAML::Value << m_unitDistance;
	
	out << YAML::EndMap;
}


void TwigParameters::Load(const std::string& name, const YAML::Node& in) {
	if (in[name]) {
		auto& ms = in[name];

		if (ms["m_segmentLength"]) m_segmentLength = ms["m_segmentLength"].as<float>();
		if (ms["m_apicalAngleVariance"]) m_apicalAngleVariance = ms["m_apicalAngleVariance"].as<float>();
		if (ms["m_branchingAngle"]) m_branchingAngle = ms["m_branchingAngle"].as<float>();
		if (ms["m_thickness"]) m_thickness = ms["m_thickness"].as<float>();
		if (ms["m_maxNodeThickness"]) m_maxNodeThickness = ms["m_maxNodeThickness"].as<float>();
		if (ms["m_maxEndDistance"]) m_maxEndDistance = ms["m_maxEndDistance"].as<float>();
		if (ms["m_segmentSize"]) m_segmentSize = ms["m_segmentSize"].as<int>();
		if (ms["m_unitDistance"]) m_unitDistance = ms["m_unitDistance"].as<float>();
		if (ms["m_minRootDistance"]) m_minRootDistance = ms["m_minRootDistance"].as<float>();
	}
}


bool TwigParameters::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	bool changed = false;
	if (ImGui::TreeNodeEx("Twig settings"))
	{
		changed = ImGui::DragFloat("Segment length", &m_segmentLength, 0.001f, 0.0f, 1.0f) || changed;
		changed = ImGui::DragFloat("Apical angle variance", &m_apicalAngleVariance, 0.01f, 0.0f, 10.0f) || changed;
		changed = ImGui::DragFloat("Branching angle", &m_branchingAngle, 0.01f, 0.0f, 180.0f) || changed;
		changed = ImGui::DragFloat("Twig thickness", &m_thickness, 0.001f, 0.0f, 1.0f) || changed;
		changed = ImGui::DragInt("Segment Size", &m_segmentSize, 1, 2, 10) || changed;
		changed = ImGui::DragFloat("Distance between twigs", &m_unitDistance, 0.001f, 0.0f, 1.0f) || changed;

		changed = ImGui::DragFloat("Max node thickness", &m_maxNodeThickness, 0.001f, 0.0f, 5.0f) || changed;
		changed = ImGui::DragFloat("Min root distance", &m_minRootDistance, 0.001f, 0.0f, 1.0f) || changed;
		changed = ImGui::DragFloat("Max end distance", &m_maxEndDistance, 0.001f, 0.0f, 1.0f) || changed;

		
		ImGui::TreePop();
	}
	return changed;
}

void FoliageDescriptor::Serialize(YAML::Emitter& out)
{
	out << YAML::Key << "m_leafSize" << YAML::Value << m_leafSize;
	out << YAML::Key << "m_leafCountPerInternode" << YAML::Value << m_leafCountPerInternode;
	out << YAML::Key << "m_positionVariance" << YAML::Value << m_positionVariance;
	out << YAML::Key << "m_rotationVariance" << YAML::Value << m_rotationVariance;
	out << YAML::Key << "m_branchingAngle" << YAML::Value << m_branchingAngle;
	out << YAML::Key << "m_maxNodeThickness" << YAML::Value << m_maxNodeThickness;
	out << YAML::Key << "m_minRootDistance" << YAML::Value << m_minRootDistance;
	out << YAML::Key << "m_maxEndDistance" << YAML::Value << m_maxEndDistance;
	out << YAML::Key << "m_horizontalTropism" << YAML::Value << m_horizontalTropism;
	out << YAML::Key << "m_gravitropism" << YAML::Value << m_gravitropism;

	m_twigParameters.Save("m_twigParameters", out);

	m_leafMaterial.Save("m_leafMaterial", out);
}

void FoliageDescriptor::Deserialize(const YAML::Node& in)
{
	if (in["m_leafSize"]) m_leafSize = in["m_leafSize"].as<glm::vec2>();
	if (in["m_leafCountPerInternode"]) m_leafCountPerInternode = in["m_leafCountPerInternode"].as<int>();
	if (in["m_positionVariance"]) m_positionVariance = in["m_positionVariance"].as<float>();
	if (in["m_rotationVariance"]) m_rotationVariance = in["m_rotationVariance"].as<float>();
	if (in["m_branchingAngle"]) m_branchingAngle = in["m_branchingAngle"].as<float>();
	if (in["m_maxNodeThickness"]) m_maxNodeThickness = in["m_maxNodeThickness"].as<float>();
	if (in["m_minRootDistance"]) m_minRootDistance = in["m_minRootDistance"].as<float>();
	if (in["m_maxEndDistance"]) m_maxEndDistance = in["m_maxEndDistance"].as<float>();
	if (in["m_horizontalTropism"]) m_horizontalTropism = in["m_horizontalTropism"].as<float>();
	if (in["m_gravitropism"]) m_gravitropism = in["m_gravitropism"].as<float>();
	m_twigParameters.Load("m_twigParameters", in);
	m_leafMaterial.Load("m_leafMaterial", in);
}

void FoliageDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	bool changed = false;

	if (ImGui::DragFloat2("Leaf size", &m_leafSize.x, 0.001f, 0.0f, 1.0f)) changed = true;
	if (ImGui::DragInt("Leaf per node", &m_leafCountPerInternode, 1, 0, 50)) changed = true;
	if (ImGui::DragFloat("Position variance", &m_positionVariance, 0.01f, 0.0f, 1.0f)) changed = true;
	if (ImGui::DragFloat("Rotation variance", &m_rotationVariance, 0.01f, 0.0f, 1.0f)) changed = true;
	if (ImGui::DragFloat("Branching angle", &m_branchingAngle, 0.01f, 0.0f, 1.0f)) changed = true;
	if (ImGui::DragFloat("Max node thickness", &m_maxNodeThickness, 0.001f, 0.0f, 5.0f)) changed = true;
	if (ImGui::DragFloat("Min root distance", &m_minRootDistance, 0.01f, 0.0f, 10.0f)) changed = true;
	if (ImGui::DragFloat("Max end distance", &m_maxEndDistance, 0.01f, 0.0f, 10.0f)) changed = true;

	changed = ImGui::DragFloat("Horizontal Tropism", &m_horizontalTropism, 0.001f, 0.0f, 1.0f) || changed;
	changed = ImGui::DragFloat("Gravitropism", &m_gravitropism, 0.001f, 0.0f, 1.0f) || changed;
	editorLayer->DragAndDropButton<Material>(m_leafMaterial, "Leaf Material");
	m_twigParameters.OnInspect(editorLayer);
	if (changed) m_saved = false;
}

void FoliageDescriptor::CollectAssetRef(std::vector<AssetRef>& list)
{
	if (m_leafMaterial.Get<Material>()) list.push_back(m_leafMaterial);
}

void FoliageDescriptor::GenerateFoliageMatrices(std::vector<glm::mat4>& matrices, const SkeletonNodeInfo& internodeInfo) const
{
	if (internodeInfo.m_thickness < m_maxNodeThickness
		&& internodeInfo.m_rootDistance > m_minRootDistance
		&& internodeInfo.m_endDistance < m_maxEndDistance) {
		for (int i = 0; i < m_leafCountPerInternode * internodeInfo.m_leaves; i++)
		{
			const auto leafSize = m_leafSize;
			glm::quat rotation = internodeInfo.m_globalRotation *
				glm::quat(glm::radians(glm::vec3(glm::gaussRand(0.0f, m_rotationVariance), m_branchingAngle, glm::linearRand(0.0f, 360.0f))));
			auto front = rotation * glm::vec3(0, 0, -1);
			auto up = rotation * glm::vec3(0, 1, 0);
			TreeModel::ApplyTropism(glm::vec3(0, -1, 0), m_gravitropism, front, up);
			const auto horizontalDirection = glm::vec3(front.x, 0.0f, front.z);
			if (glm::length(horizontalDirection) > glm::epsilon<float>())
			{
				TreeModel::ApplyTropism(glm::normalize(horizontalDirection), m_horizontalTropism,
					front, up);
			}
			auto foliagePosition = glm::mix(internodeInfo.m_globalPosition, internodeInfo.GetGlobalEndPosition(), glm::linearRand(0.f, 1.f)) + front * (leafSize.y + glm::gaussRand(0.0f, m_positionVariance));
			if(glm::any(glm::isnan(foliagePosition)) || glm::any(glm::isnan(front)) || glm::any(glm::isnan(up))) continue;
			const auto leafTransform = glm::translate(foliagePosition) * glm::mat4_cast(glm::quatLookAt(front, up)) * glm::scale(glm::vec3(leafSize.x, 1.0f, leafSize.y));
			matrices.emplace_back(leafTransform);
		}
	}

}
