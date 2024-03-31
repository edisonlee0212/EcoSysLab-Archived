#include "SorghumState.hpp"

#include "IVolume.hpp"
#include "SorghumLayer.hpp"
using namespace EcoSysLab;

bool SorghumMeshGeneratorSettings::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	if(ImGui::TreeNode("Sorghum mesh generator settings"))
	{
		ImGui::Checkbox("Panicle", &m_enablePanicle);
		ImGui::Checkbox("Stem", &m_enableStem);
		ImGui::Checkbox("Leaves", &m_enableLeaves);
		ImGui::Checkbox("Bottom Face", &m_bottomFace);
		ImGui::Checkbox("Leaf separated", &m_leafSeparated);
		ImGui::DragFloat("Leaf thickness", &m_leafThickness, 0.0001f);
		ImGui::TreePop();
	}
	return false;
}

bool SorghumPanicleState::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	return false;
}

void SorghumPanicleState::Serialize(YAML::Emitter& out) const
{
	out << YAML::Key << "m_panicleSize" << YAML::Value << m_panicleSize;
	out << YAML::Key << "m_seedAmount" << YAML::Value << m_seedAmount;
	out << YAML::Key << "m_seedRadius" << YAML::Value << m_seedRadius;
}

void SorghumPanicleState::Deserialize(const YAML::Node& in)
{
	if (in["m_panicleSize"])
		m_panicleSize = in["m_panicleSize"].as<glm::vec3>();
	if (in["m_seedAmount"])
		m_seedAmount = in["m_seedAmount"].as<int>();
	if (in["m_seedRadius"])
		m_seedRadius = in["m_seedRadius"].as<float>();
}

void SorghumPanicleState::GenerateGeometry(const glm::vec3& stemTip, std::vector<Vertex>& vertices, std::vector<unsigned>& indices) const
{
	std::vector<glm::vec3> icosahedronVertices;
	std::vector<glm::uvec3> icosahedronTriangles;
	SphereMeshGenerator::Icosahedron(icosahedronVertices, icosahedronTriangles);
	int offset = 0;
	Vertex archetype = {};
	SphericalVolume volume;
	volume.m_radius = m_panicleSize;
	for (int seedIndex = 0;
		seedIndex < m_seedAmount;
		seedIndex++) {
		glm::vec3 positionOffset = volume.GetRandomPoint();
		for (const auto position : icosahedronVertices) {
			archetype.m_position =
				position * m_seedRadius + glm::vec3(0, m_panicleSize.y, 0) +
				positionOffset + stemTip;
			vertices.push_back(archetype);
		}
		for (const auto triangle : icosahedronTriangles) {
			glm::uvec3 actualTriangle = triangle + glm::uvec3(offset);
			indices.emplace_back(actualTriangle.x);
			indices.emplace_back(actualTriangle.y);
			indices.emplace_back(actualTriangle.z);
		}
		offset += icosahedronVertices.size();
	}
}

bool SorghumStemState::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	return false;
}

void SorghumStemState::Serialize(YAML::Emitter& out) const
{
	if (!m_nodes.empty()) {
		out << YAML::Key << "m_nodes" << YAML::Value
			<< YAML::Binary((const unsigned char*)m_nodes.data(),
				m_nodes.size() * sizeof(SorghumSplineNode));
	}
}

void SorghumStemState::Deserialize(const YAML::Node& in)
{
	if (in["m_nodes"]) {
		const auto& nodes = in["m_nodes"].as<YAML::Binary>();
		m_nodes.resize(nodes.size() / sizeof(SorghumSplineNode));
		std::memcpy(m_nodes.data(), nodes.data(), nodes.size());
	}
}

void SorghumStemState::GenerateGeometry(std::vector<Vertex>& vertices, std::vector<unsigned>& indices) const
{
	if (m_nodes.empty())
		return;
	auto sorghumLayer = Application::GetLayer<SorghumLayer>();
	if (!sorghumLayer)
		return;
	std::vector<SorghumLeafSegment> segments;
	for (int i = 1; i < m_nodes.size(); i++) {
		auto& prev = m_nodes.at(i - 1);
		auto& curr = m_nodes.at(i);
		float distance = glm::distance(prev.m_position, curr.m_position);
		BezierCurve curve = BezierCurve(
			prev.m_position, prev.m_position + distance / 5.0f * prev.m_axis,
			curr.m_position - distance / 5.0f * curr.m_axis, curr.m_position);
		for (float div = (i == 1 ? 0.0f : 0.5f); div <= 1.0f; div += 0.5f) {
			auto front = prev.m_axis * (1.0f - div) + curr.m_axis * div;
			auto left = prev.m_left * (1.0f - div) + curr.m_left * div;
			auto up = glm::normalize(glm::cross(left, front));
			segments.emplace_back(
				curve.GetPoint(div), up, front,
				prev.m_stemWidth * (1.0f - div) + curr.m_stemWidth * div,
				prev.m_leafWidth * (1.0f - div) + curr.m_leafWidth * div,
				prev.m_theta * (1.0f - div) + curr.m_theta * div, curr.m_type, 1.0f,
				1.0f);
		}
	}
	const int vertexIndex = vertices.size();
	Vertex archetype{};
	glm::vec4 m_vertexColor = glm::vec4(0, 0, 0, 1);
	archetype.m_color = m_vertexColor;
	const float xStep = 1.0f / sorghumLayer->m_horizontalSubdivisionStep / 2.0f;
	auto segmentSize = segments.size();
	const float yStemStep = 0.5f / segmentSize;
	for (int i = 0; i < segmentSize; i++) {
		auto& segment = segments.at(i);
		if (i <= segmentSize / 3) {
			archetype.m_color = glm::vec4(1, 0, 0, 1);
		}
		else if (i <= segmentSize * 2 / 3) {
			archetype.m_color = glm::vec4(0, 1, 0, 1);
		}
		else {
			archetype.m_color = glm::vec4(0, 0, 1, 1);
		}
		const float angleStep =
			segment.m_theta / sorghumLayer->m_horizontalSubdivisionStep;
		const int vertsCount = sorghumLayer->m_horizontalSubdivisionStep * 2 + 1;
		for (int j = 0; j < vertsCount; j++) {
			const auto position = segment.GetPoint(
				(j - sorghumLayer->m_horizontalSubdivisionStep) * angleStep);
			archetype.m_position = glm::vec3(position.x, position.y, position.z);
			float yPos = yStemStep * i;
			archetype.m_texCoord = glm::vec2(j * xStep, yPos);
			vertices.push_back(archetype);
		}
		if (i != 0) {
			for (int j = 0; j < vertsCount - 1; j++) {
				// Down triangle
				indices.emplace_back(vertexIndex + ((i - 1) + 1) * vertsCount + j);
				indices.emplace_back(vertexIndex + (i - 1) * vertsCount + j + 1);
				indices.emplace_back(vertexIndex + (i - 1) * vertsCount + j);
				// Up triangle
				indices.emplace_back(vertexIndex + (i - 1) * vertsCount + j + 1);
				indices.emplace_back(vertexIndex + ((i - 1) + 1) * vertsCount + j);
				indices.emplace_back(vertexIndex + ((i - 1) + 1) * vertsCount + j + 1);
			}
		}
	}
}

void SorghumLeafState::GenerateSegments(std::vector<SorghumLeafSegment>& segments, bool bottomFace) const
{
	for (int i = 1; i < m_nodes.size(); i++) {
		auto& prev = m_nodes.at(i - 1);
		auto& curr = m_nodes.at(i);
		if (bottomFace && prev.m_type != SorghumSplineType::Leaf) {
			continue;
		}
		float distance = glm::distance(prev.m_position, curr.m_position);
		BezierCurve curve = BezierCurve(
			prev.m_position, prev.m_position + distance / 5.0f * prev.m_axis,
			curr.m_position - distance / 5.0f * curr.m_axis, curr.m_position);

		for (float div = (i == 1 ? 0.0f : 0.5f); div <= 1.0f; div += 0.5f) {

			auto front = glm::normalize(prev.m_axis * (1.0f - div) + curr.m_axis * div);
			auto left = glm::normalize(prev.m_left * (1.0f - div) + curr.m_left * div);//?
			auto up = glm::normalize(glm::cross(left, front));
			auto waviness = glm::mix(prev.m_waviness, curr.m_waviness, div);
			float leftPeriod = 0.f;
			float rightPeriod = 0.f;
			segments.emplace_back(curve.GetPoint(div), up, front,
				glm::mix(prev.m_stemWidth, curr.m_stemWidth, div),
				glm::mix(prev.m_leafWidth, curr.m_leafWidth, div),
				glm::mix(prev.m_theta, curr.m_theta, div),
				curr.m_type, glm::sin(leftPeriod) * waviness,
				glm::sin(rightPeriod) * waviness);
		}
	}
}

bool SorghumLeafState::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	return false;
}

void SorghumLeafState::Serialize(YAML::Emitter& out) const
{
	out << YAML::Key << "m_wavinessPeriodStart" << YAML::Value << m_wavinessPeriodStart;
	out << YAML::Key << "m_wavinessFrequency" << YAML::Value << m_wavinessFrequency;
	out << YAML::Key << "m_index" << YAML::Value << m_index;
	if (!m_nodes.empty()) {
		out << YAML::Key << "m_nodes" << YAML::Value
			<< YAML::Binary((const unsigned char*)m_nodes.data(),
				m_nodes.size() * sizeof(SorghumSplineNode));
	}
}

void SorghumLeafState::Deserialize(const YAML::Node& in)
{
	if (in["m_index"]) m_index = in["m_index"].as<int>();
	if (in["m_wavinessPeriodStart"]) m_wavinessPeriodStart = in["m_wavinessPeriodStart"].as<glm::vec2>();
	if (in["m_wavinessFrequency"]) m_wavinessFrequency = in["m_wavinessFrequency"].as<glm::vec2>();

	if (in["m_nodes"]) {
		const auto& nodes = in["m_nodes"].as<YAML::Binary>();
		m_nodes.resize(nodes.size() / sizeof(SorghumSplineNode));
		std::memcpy(m_nodes.data(), nodes.data(), nodes.size());
	}
}

void SorghumLeafState::GenerateGeometry(std::vector<Vertex>& vertices, std::vector<unsigned>& indices, bool bottomFace, float thickness) const
{
	if (m_nodes.empty())
		return;
	auto sorghumLayer = Application::GetLayer<SorghumLayer>();
	if (!sorghumLayer)
		return;
	std::vector<SorghumLeafSegment> segments;
	GenerateSegments(segments);

	const int vertexIndex = vertices.size();
	Vertex archetype{};
#pragma region Semantic mask color
	auto index = m_index + 1;
	const auto vertexColor = glm::vec4(index % 3 * 0.5f, index / 3 % 3 * 0.5f,
		index / 9 % 3 * 0.5f, 1.0f);
#pragma endregion
	archetype.m_color = vertexColor;

	const float xStep = 1.0f / static_cast<float>(sorghumLayer->m_horizontalSubdivisionStep) / 2.0f;
	auto segmentSize = segments.size();
	const float yLeafStep = 0.5f / segmentSize;

	for (int i = 0; i < segmentSize; i++) {
		auto& segment = segments.at(i);
		/*
		if (i <= segmentSize / 3) {
			archetype.m_color = glm::vec4(1, 0, 0, 1);
		}
		else if (i <= segmentSize * 2 / 3) {
			archetype.m_color = glm::vec4(0, 1, 0, 1);
		}
		else {
			archetype.m_color = glm::vec4(0, 0, 1, 1);
		}*/
		const float angleStep =
			segment.m_theta / static_cast<float>(sorghumLayer->m_horizontalSubdivisionStep);
		const int vertsCount = sorghumLayer->m_horizontalSubdivisionStep * 2 + 1;
		for (int j = 0; j < vertsCount; j++) {
			auto position = segment.GetPoint(
				(j - sorghumLayer->m_horizontalSubdivisionStep) * angleStep);
			auto normal = segment.GetNormal(
				(j - sorghumLayer->m_horizontalSubdivisionStep) * angleStep);
			if (i != 0 && j != 0 && j != vertsCount - 1) {
				position -= normal * thickness;
			}
			archetype.m_position = glm::vec3(position.x, position.y, position.z);
			float yPos = 0.5f + yLeafStep * i;
			archetype.m_texCoord = glm::vec2(j * xStep, yPos);
			vertices.push_back(archetype);
		}
		if (i != 0) {
			for (int j = 0; j < vertsCount - 1; j++) {
				if (bottomFace) {
					// Down triangle
					indices.emplace_back(vertexIndex + i * vertsCount + j);
					indices.emplace_back(vertexIndex + (i - 1) * vertsCount + j + 1);
					indices.emplace_back(vertexIndex + (i - 1) * vertsCount + j);
					// Up triangle
					indices.emplace_back(vertexIndex + (i - 1) * vertsCount + j + 1);
					indices.emplace_back(vertexIndex + i * vertsCount + j);
					indices.emplace_back(vertexIndex + i * vertsCount + j + 1);
				}
				else
				{
					// Down triangle
					indices.emplace_back(vertexIndex + (i - 1) * vertsCount + j);
					indices.emplace_back(vertexIndex + (i - 1) * vertsCount + j + 1);
					indices.emplace_back(vertexIndex + i * vertsCount + j);
					// Up triangle
					indices.emplace_back(vertexIndex + i * vertsCount + j + 1);
					indices.emplace_back(vertexIndex + i * vertsCount + j);
					indices.emplace_back(vertexIndex + (i - 1) * vertsCount + j + 1);
				}
			}
		}
	}
}

void SorghumState::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	bool changed = false;
	if (ImGui::TreeNodeEx((std::string("Stem")).c_str())) {
		if (m_stem.OnInspect(editorLayer))
			changed = true;
		ImGui::TreePop();
	}

	if (ImGui::TreeNodeEx("Leaves")) {
		int leafSize = m_leaves.size();
		if (ImGui::InputInt("Number of leaves", &leafSize)) {
			changed = true;
			leafSize = glm::clamp(leafSize, 0, 999);
			const auto previousSize = m_leaves.size();
			m_leaves.resize(leafSize);
			for (int i = 0; i < leafSize; i++) {
				if (i >= previousSize) {
					if (i - 1 >= 0) {
						m_leaves[i] = m_leaves[i - 1];
						/*
						m_leaves[i].m_rollAngle =
							glm::mod(m_leaves[i - 1].m_rollAngle + 180.0f, 360.0f);
						m_leaves[i].m_startingPoint =
							m_leaves[i - 1].m_startingPoint + 0.1f;*/
					}
					else {
						m_leaves[i] = {};
						/*
						m_leaves[i].m_rollAngle = 0;
						m_leaves[i].m_startingPoint = 0.1f;*/
					}
				}
				m_leaves[i].m_index = i;
			}
		}
		for (auto& leaf : m_leaves) {
			if (ImGui::TreeNode(
				("Leaf No." + std::to_string(leaf.m_index + 1) +
					(leaf.m_nodes.empty() ? " (Dead)" : ""))
				.c_str())) {
				if (leaf.OnInspect(editorLayer))
					changed = true;
				ImGui::TreePop();
			}
		}
		ImGui::TreePop();
	}

	if (ImGui::TreeNodeEx((std::string("Panicle")).c_str())) {
		if (m_panicle.OnInspect(editorLayer))
			changed = true;
		ImGui::TreePop();
	}

	if (changed)
		m_saved = false;
}

void SorghumState::Serialize(YAML::Emitter& out)
{
	out << YAML::Key << "m_version" << YAML::Value << m_version;
	out << YAML::Key << "m_panicle" << YAML::Value << YAML::BeginMap;
	m_panicle.Serialize(out);
	out << YAML::EndMap;
	out << YAML::Key << "m_stem" << YAML::Value << YAML::BeginMap;
	m_stem.Serialize(out);
	out << YAML::EndMap;

	if (!m_leaves.empty()) {
		out << YAML::Key << "m_leaves" << YAML::Value << YAML::BeginSeq;
		for (auto& i : m_leaves) {
			out << YAML::BeginMap;
			i.Serialize(out);
			out << YAML::EndMap;
		}
		out << YAML::EndSeq;
	}

	m_saved = true;
}

void SorghumState::Deserialize(const YAML::Node& in)
{
	if (in["m_panicle"])
		m_panicle.Deserialize(in["m_panicle"]);

	if (in["m_stem"])
		m_stem.Deserialize(in["m_stem"]);

	if (in["m_leaves"]) {
		for (const auto& i : in["m_leaves"]) {
			SorghumLeafState leafState{};
			leafState.Deserialize(i);
			m_leaves.push_back(leafState);
		}
	}
	m_saved = true;
}
