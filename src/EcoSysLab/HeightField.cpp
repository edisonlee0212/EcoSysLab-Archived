#include "HeightField.hpp"

#include "glm/gtc/noise.hpp"

using namespace EcoSysLab;

void GroundSurface::OnInspect() {
	bool changed = false;
	if (ImGui::DragFloat2("Min/max", &m_minMax.x, 0, -1000, 1000)) { changed = true; }
	if (ImGui::Button("New start descriptor")) {
		changed = true;
		m_noiseDescriptors.emplace_back();
	}
	for (int i = 0; i < m_noiseDescriptors.size(); i++)
	{
		if (ImGui::TreeNodeEx(("No." + std::to_string(i)).c_str(), ImGuiTreeNodeFlags_DefaultOpen))
		{
			if (ImGui::Button("Remove"))
			{
				m_noiseDescriptors.erase(m_noiseDescriptors.begin() + i);
				ImGui::TreePop();
				continue;
			}
			ImGui::DragFloat("Scale", &m_noiseDescriptors[i].m_noiseScale, 0.01f);
			ImGui::DragFloat("Offset", &m_noiseDescriptors[i].m_offset, 0.01f);
			ImGui::DragFloat("Power factor", &m_noiseDescriptors[i].m_powerFactor, 0.01f);

			ImGui::DragFloat("Intensity", &m_noiseDescriptors[i].m_noiseIntensity, 0.01f);
			if (ImGui::DragFloat("Min", &m_noiseDescriptors[i].m_heightMin, 0.01f, -99999, m_noiseDescriptors[i].m_heightMax))
			{
				m_noiseDescriptors[i].m_heightMin = glm::min(m_noiseDescriptors[i].m_heightMin, m_noiseDescriptors[i].m_heightMax);
			}
			if (ImGui::DragFloat("Max", &m_noiseDescriptors[i].m_heightMax, 0.01f, m_noiseDescriptors[i].m_heightMin, 99999))
			{
				m_noiseDescriptors[i].m_heightMax = glm::max(m_noiseDescriptors[i].m_heightMin, m_noiseDescriptors[i].m_heightMax);
			}

			ImGui::TreePop();
		}
	}


}
void GroundSurface::Serialize(YAML::Emitter& out) {
	out << YAML::Key << "m_minMax" << YAML::Value << m_minMax;
	if (!m_noiseDescriptors.empty())
	{
		out << YAML::Key << "m_noiseDescriptors" << YAML::Value
			<< YAML::Binary((const unsigned char*)m_noiseDescriptors.data(), m_noiseDescriptors.size() * sizeof(NoiseDescriptor));
	}

}
void GroundSurface::Deserialize(const YAML::Node& in) {
	if (in["m_minMax"])
		m_minMax = in["m_minMax"].as<glm::vec2>();
	

	if (in["m_noiseDescriptors"])
	{
		const auto& ds = in["m_noiseDescriptors"].as<YAML::Binary>();
		m_noiseDescriptors.resize(ds.size() / sizeof(NoiseDescriptor));
		std::memcpy(m_noiseDescriptors.data(), ds.data(), ds.size());
	}

}

float GroundSurface::GetValue(const glm::vec2& position) const
{
	float retVal = 0;
	for (const auto& noiseDescriptor : m_noiseDescriptors)
	{
		float noise = noiseDescriptor.m_offset + glm::pow(glm::simplex(noiseDescriptor.m_noiseScale * position +
			m_positionOffset), noiseDescriptor.m_powerFactor) * noiseDescriptor.m_noiseIntensity;
		retVal += glm::clamp(noise, noiseDescriptor.m_heightMin, noiseDescriptor.m_heightMax);
	}
	return glm::clamp(retVal, m_minMax.x, m_minMax.y);
}

float HeightField::GetValue(const glm::vec2& position)
{
	float retVal = 0.0f;
	if(position.x < 0)
	{
		retVal += glm::max(position.x, -5.0f);
	}
	
	const auto groundSurface = m_groundSurface.Get<GroundSurface>();
	if (groundSurface)
	{
		retVal += groundSurface->GetValue(position);
	}

	return retVal;
}

void HeightField::OnInspect()
{
	ImGui::DragInt("Precision level", &m_precisionLevel);
	Editor::DragAndDropButton<GroundSurface>(m_groundSurface, "GroundSurface", true);
}

void HeightField::Serialize(YAML::Emitter& out)
{
	out << YAML::Key << "m_precisionLevel" << YAML::Value << m_precisionLevel;
	m_groundSurface.Save("m_groundSurface", out);
}

void HeightField::Deserialize(const YAML::Node& in)
{
	if (in["m_precisionLevel"])
		m_precisionLevel = in["m_precisionLevel"].as<int>();
	m_groundSurface.Load("m_groundSurface", in);
}

void HeightField::GenerateMesh(const glm::vec2& start, const glm::uvec2& resolution, float unitSize, std::vector<Vertex>& vertices, std::vector<glm::uvec3>& triangles)
{
	const auto groundSurface = m_groundSurface.Get<GroundSurface>();
	if (!groundSurface)
	{
		UNIENGINE_ERROR("No ground surface!");
		return;
	}
	for (unsigned i = 0; i < resolution.x * m_precisionLevel; i++) {
		for (unsigned j = 0; j < resolution.y * m_precisionLevel; j++) {
			Vertex archetype;
			archetype.m_position.x = start.x + unitSize * i / m_precisionLevel;
			archetype.m_position.z = start.y + unitSize * j / m_precisionLevel;
			archetype.m_position.y = GetValue({ archetype.m_position.x , archetype.m_position.z });
			archetype.m_texCoord = glm::vec2(static_cast<float>(i) / (resolution.x * m_precisionLevel),
				static_cast<float>(j) / (resolution.y * m_precisionLevel));
			vertices.push_back(archetype);
		}
	}

	for (int i = 0; i < resolution.x * m_precisionLevel - 1; i++) {
		for (int j = 0; j < resolution.y * m_precisionLevel - 1; j++) {
			int n = resolution.x * m_precisionLevel;
			triangles.emplace_back(i + j * n, i + 1 + j * n, i + (j + 1) * n);
			triangles.emplace_back(i + 1 + (j + 1) * n, i + (j + 1) * n,
				i + 1 + j * n);
		}
	}
}

void GroundSurface::OnCreate() {
	m_minMax = glm::vec2(-1000, 1000);
	m_noiseDescriptors.clear();
	m_noiseDescriptors.emplace_back();
}
