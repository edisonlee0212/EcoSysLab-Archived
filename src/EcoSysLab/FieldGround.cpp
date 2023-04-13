//
// Created by lllll on 3/3/2022.
//

#include "FieldGround.hpp"
#include "glm/gtc/noise.hpp"

using namespace EcoSysLab;
Entity FieldGround::GenerateMesh(float overrideDepth) {
	std::vector<Vertex> vertices;
	std::vector<glm::uvec3> triangles;
	Vertex archetype;
	glm::vec3 randomPositionOffset =
		glm::linearRand(glm::vec3(0.0f), glm::vec3(10000.0f));
	for (int i = -m_size.x; i <= m_size.x; i++) {
		for (int j = -m_size.y; j <= m_size.y; j++) {
			archetype.m_position.x = m_scale.x * i;
			archetype.m_position.z = m_scale.y * j;
			archetype.m_position.y =
				glm::min(0.0f, (overrideDepth < 0.0f ? m_alleyDepth : overrideDepth) *
					glm::cos(archetype.m_position.z * m_rowWidth));
			for (const auto& noiseDescriptor : m_noiseDescriptors)
			{
				float noise = glm::simplex(noiseDescriptor.m_noiseScale * archetype.m_position +
					randomPositionOffset) *
					noiseDescriptor.m_noiseIntensity;
				archetype.m_position.y += glm::clamp(noise, noiseDescriptor.m_heightMin, noiseDescriptor.m_heightMax);
			}
			archetype.m_texCoord = glm::vec2((float)i / (2 * m_size.x + 1),
				(float)j / (2 * m_size.y + 1));
			vertices.push_back(archetype);
		}
	}

	for (int i = 0; i < 2 * m_size.x; i++) {
		for (int j = 0; j < 2 * m_size.y; j++) {
			int n = 2 * m_size.x + 1;
			triangles.emplace_back(i + j * n, i + 1 + j * n, i + (j + 1) * n);
			triangles.emplace_back(i + 1 + (j + 1) * n, i + (j + 1) * n,
				i + 1 + j * n);
		}
	}
	
	auto scene = Application::GetActiveScene();
	auto owner = scene->CreateEntity("Ground");

	auto meshRenderer =
		scene->GetOrSetPrivateComponent<MeshRenderer>(owner).lock();
	auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
	auto material = ProjectManager::CreateTemporaryAsset<Material>();
	mesh->SetVertices(17, vertices, triangles);
	meshRenderer->m_mesh = mesh;
	meshRenderer->m_material = material;

	return owner;
}
void FieldGround::OnInspect() {
	static bool autoRefresh = false;
	ImGui::Checkbox("Auto refresh", &autoRefresh);
	bool changed = ImGui::DragFloat2("Scale", &m_scale.x);
	changed = changed || ImGui::DragInt2("Size", &m_size.x);
	changed = changed || ImGui::DragFloat("Row Width", &m_rowWidth);
	changed = changed || ImGui::DragFloat("Alley Depth", &m_alleyDepth);
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
			ImGui::DragFloat("Intensity", &m_noiseDescriptors[i].m_noiseIntensity, 0.01f);
			if(ImGui::DragFloat("Min", &m_noiseDescriptors[i].m_heightMin, 0.01f, -99999, m_noiseDescriptors[i].m_heightMax))
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

	if (ImGui::Button("Apply") || (changed && autoRefresh)) {
		GenerateMesh();
	}
}
void FieldGround::Serialize(YAML::Emitter& out) {
	out << YAML::Key << "m_scale" << YAML::Value << m_scale;
	out << YAML::Key << "m_size" << YAML::Value << m_size;
	out << YAML::Key << "m_rowWidth" << YAML::Value << m_rowWidth;
	out << YAML::Key << "m_alleyDepth" << YAML::Value << m_alleyDepth;

	if (!m_noiseDescriptors.empty())
	{
		out << YAML::Key << "m_noiseDescriptors" << YAML::Value
			<< YAML::Binary((const unsigned char*)m_noiseDescriptors.data(), m_noiseDescriptors.size() * sizeof(NoiseDescriptor));
	}
	
}
void FieldGround::Deserialize(const YAML::Node& in) {
	if (in["m_scale"])
		m_scale = in["m_scale"].as<glm::vec2>();
	if (in["m_size"])
		m_size = in["m_size"].as<glm::ivec2>();
	if (in["m_rowWidth"])
		m_rowWidth = in["m_rowWidth"].as<float>();
	if (in["m_alleyDepth"])
		m_alleyDepth = in["m_alleyDepth"].as<float>();


	if (in["m_noiseDescriptors"])
	{
		const auto &ds = in["m_noiseDescriptors"].as<YAML::Binary>();
		m_noiseDescriptors.resize(ds.size() / sizeof(NoiseDescriptor));
		std::memcpy(m_noiseDescriptors.data(), ds.data(), ds.size());
	}

}
void FieldGround::OnCreate() {
	m_scale = glm::vec2(0.02f);
	m_size = glm::ivec2(150);
	m_rowWidth = 0.0f;
	m_alleyDepth = 0.15f;
	m_noiseDescriptors.clear();
	m_noiseDescriptors.emplace_back();
}
