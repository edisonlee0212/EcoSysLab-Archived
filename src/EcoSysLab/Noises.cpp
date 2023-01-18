#include "Noises.hpp"
#include "glm/gtc/noise.hpp"
using namespace EcoSysLab;


Noises2D::Noises2D() {
	m_minMax = glm::vec2(-1000, 1000);
	m_noiseDescriptors.clear();
	m_noiseDescriptors.emplace_back();
}
Noises3D::Noises3D()
{
	m_minMax = glm::vec2(-1000, 1000);
	m_noiseDescriptors.clear();
	m_noiseDescriptors.emplace_back();
}

bool Noises2D::OnInspect() {
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
				changed = true;
				ImGui::TreePop();
				continue;
			}
			changed = ImGui::Combo("Type", { "Simplex", "Perlin", "Constant" }, m_noiseDescriptors[i].m_type) || changed;
			changed = ImGui::DragFloat("Scale", &m_noiseDescriptors[i].m_noiseScale, 0.01f) || changed;
			changed = ImGui::DragFloat("Base", &m_noiseDescriptors[i].m_base, 0.01f) || changed;
			changed = ImGui::DragFloat("Power factor", &m_noiseDescriptors[i].m_powerFactor, 0.01f) || changed;
			changed = ImGui::DragFloat("Offset", &m_noiseDescriptors[i].m_offset, 0.01f) || changed;
			changed = ImGui::DragFloat("Intensity", &m_noiseDescriptors[i].m_noiseIntensity, 0.01f) || changed;
			if (ImGui::DragFloat("Min", &m_noiseDescriptors[i].m_heightMin, 0.01f, -99999, m_noiseDescriptors[i].m_heightMax))
			{
				changed = true;
				m_noiseDescriptors[i].m_heightMin = glm::min(m_noiseDescriptors[i].m_heightMin, m_noiseDescriptors[i].m_heightMax);
			}
			if (ImGui::DragFloat("Max", &m_noiseDescriptors[i].m_heightMax, 0.01f, m_noiseDescriptors[i].m_heightMin, 99999))
			{
				changed = true;
				m_noiseDescriptors[i].m_heightMax = glm::max(m_noiseDescriptors[i].m_heightMin, m_noiseDescriptors[i].m_heightMax);
			}

			ImGui::TreePop();
		}
	}
	return changed;
}

void Noises2D::Save(const std::string& name, YAML::Emitter& out) const
{
	out << YAML::Key << name << YAML::Value << YAML::BeginMap;
	out << YAML::Key << "m_minMax" << YAML::Value << m_minMax;
	if (!m_noiseDescriptors.empty())
	{
		out << YAML::Key << "m_noiseDescriptors" << YAML::Value
			<< YAML::Binary((const unsigned char*)m_noiseDescriptors.data(), m_noiseDescriptors.size() * sizeof(NoiseDescriptor));
	}
	out << YAML::EndMap;
}

void Noises2D::Load(const std::string& name, const YAML::Node& in)
{
	if (in[name])
	{
		const auto& n = in[name];
		if (n["m_minMax"])
			m_minMax = n["m_minMax"].as<glm::vec2>();


		if (n["m_noiseDescriptors"])
		{
			const auto& ds = n["m_noiseDescriptors"].as<YAML::Binary>();
			m_noiseDescriptors.resize(ds.size() / sizeof(NoiseDescriptor));
			std::memcpy(m_noiseDescriptors.data(), ds.data(), ds.size());
		}
	}
}
void Noises3D::Save(const std::string& name, YAML::Emitter& out) const
{
	out << YAML::Key << name << YAML::Value << YAML::BeginMap;
	out << YAML::Key << "m_minMax" << YAML::Value << m_minMax;
	if (!m_noiseDescriptors.empty())
	{
		out << YAML::Key << "m_noiseDescriptors" << YAML::Value
			<< YAML::Binary((const unsigned char*)m_noiseDescriptors.data(), m_noiseDescriptors.size() * sizeof(NoiseDescriptor));
	}
	out << YAML::EndMap;
}

void Noises3D::Load(const std::string& name, const YAML::Node& in)
{
	if (in[name])
	{
		const auto& n = in[name];
		if (n["m_minMax"])
			m_minMax = n["m_minMax"].as<glm::vec2>();


		if (n["m_noiseDescriptors"])
		{
			const auto& ds = n["m_noiseDescriptors"].as<YAML::Binary>();
			m_noiseDescriptors.resize(ds.size() / sizeof(NoiseDescriptor));
			std::memcpy(m_noiseDescriptors.data(), ds.data(), ds.size());
		}
	}
}

float Noises2D::GetValue(const glm::vec2& position) const
{
	float retVal = 0;
	for (const auto& noiseDescriptor : m_noiseDescriptors)
	{
		float noise = 0;
		switch (static_cast<NoiseType>(noiseDescriptor.m_type))
		{
		case NoiseType::Perlin:
			noise = noiseDescriptor.m_base + glm::pow(glm::perlin(noiseDescriptor.m_noiseScale * position +
				glm::vec2(noiseDescriptor.m_offset)), noiseDescriptor.m_powerFactor) * noiseDescriptor.m_noiseIntensity;
			break;
		case NoiseType::Simplex:
			noise = noiseDescriptor.m_base + glm::pow(glm::simplex(noiseDescriptor.m_noiseScale * position +
				glm::vec2(noiseDescriptor.m_offset)), noiseDescriptor.m_powerFactor) * noiseDescriptor.m_noiseIntensity;
			break;
		case NoiseType::Constant:
			noise = noiseDescriptor.m_base;
			break;
		}
		retVal += glm::clamp(noise, noiseDescriptor.m_heightMin, noiseDescriptor.m_heightMax);
	}
	return glm::clamp(retVal, m_minMax.x, m_minMax.y);
}


bool Noises3D::OnInspect() {
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
				changed = true;
				ImGui::TreePop();
				continue;
			}
			changed = ImGui::Combo("Type", { "Simplex", "Perlin", "Constant" }, m_noiseDescriptors[i].m_type) || changed;
			changed = ImGui::DragFloat("Scale", &m_noiseDescriptors[i].m_noiseScale, 0.01f) || changed;
			changed = ImGui::DragFloat("Base", &m_noiseDescriptors[i].m_base, 0.01f) || changed;
			changed = ImGui::DragFloat("Power factor", &m_noiseDescriptors[i].m_powerFactor, 0.01f) || changed;
			changed = ImGui::DragFloat("Offset", &m_noiseDescriptors[i].m_offset, 0.01f) || changed;
			changed = ImGui::DragFloat("Intensity", &m_noiseDescriptors[i].m_noiseIntensity, 0.01f) || changed;
			if (ImGui::DragFloat("Min", &m_noiseDescriptors[i].m_heightMin, 0.01f, -99999, m_noiseDescriptors[i].m_heightMax))
			{
				changed = true;
				m_noiseDescriptors[i].m_heightMin = glm::min(m_noiseDescriptors[i].m_heightMin, m_noiseDescriptors[i].m_heightMax);
			}
			if (ImGui::DragFloat("Max", &m_noiseDescriptors[i].m_heightMax, 0.01f, m_noiseDescriptors[i].m_heightMin, 99999))
			{
				changed = true;
				m_noiseDescriptors[i].m_heightMax = glm::max(m_noiseDescriptors[i].m_heightMin, m_noiseDescriptors[i].m_heightMax);
			}

			ImGui::TreePop();
		}
	}
	return changed;
}


float Noises3D::GetValue(const glm::vec3& position) const
{
	float retVal = 0;
	for (const auto& noiseDescriptor : m_noiseDescriptors)
	{
		float noise = 0;
		switch (static_cast<NoiseType>(noiseDescriptor.m_type))
		{
		case NoiseType::Perlin:
			noise = noiseDescriptor.m_base + glm::pow(glm::perlin(noiseDescriptor.m_noiseScale * position +
				glm::vec3(noiseDescriptor.m_offset)), noiseDescriptor.m_powerFactor) * noiseDescriptor.m_noiseIntensity;
			break;
		case NoiseType::Simplex:
			noise = noiseDescriptor.m_base + glm::pow(glm::simplex(noiseDescriptor.m_noiseScale * position +
				glm::vec3(noiseDescriptor.m_offset)), noiseDescriptor.m_powerFactor) * noiseDescriptor.m_noiseIntensity;
			break;
		case NoiseType::Constant:
			noise = noiseDescriptor.m_base;
			break;
		}
		retVal += glm::clamp(noise, noiseDescriptor.m_heightMin, noiseDescriptor.m_heightMax);
	}
	return glm::clamp(retVal, m_minMax.x, m_minMax.y);
}