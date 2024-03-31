//
// Created by lllll on 1/8/2022.
//
#include "SorghumGrowthDescriptor.hpp"
#include "SorghumLayer.hpp"
#include "SorghumDescriptor.hpp"
#include "Utilities.hpp"
#include "rapidcsv.h"
#include "EditorLayer.hpp"
#include "Application.hpp"
#include "Scene.hpp"
#include "Times.hpp"
using namespace EcoSysLab;

void SorghumGrowthDescriptor::Apply(const std::shared_ptr<SorghumGrowthStage>& targetState, float time) const
{
	if (m_sorghumStates.empty())
		return;
	auto actualTime = glm::clamp(time, 0.0f, 99999.0f);
	float previousTime = m_sorghumStates.begin()->first;
	
	if (actualTime < previousTime) {
		// Get from zero state to first state.
		*targetState = m_sorghumStates.begin()->second;
		return;
	}
	SorghumGrowthStagePair statePair;
	statePair.m_left = m_sorghumStates.begin()->second;
	statePair.m_right = statePair.m_left;
	float a = 0.0f;
	for (auto it = (++m_sorghumStates.begin()); it != m_sorghumStates.end();
		++it) {
		statePair.m_left = statePair.m_right;
		statePair.m_right = it->second;
		if(it->first > actualTime)
		{
			a = (actualTime - previousTime) / (it->first - previousTime);
			break;
		}
		previousTime = it->first;
	}
	statePair.Apply(*targetState, a);
}
void SorghumGrowthStagePair::Apply(SorghumGrowthStage& targetState, float a)
{

	auto leafSize = GetLeafSize(a);
	targetState.m_leaves.resize(leafSize);

}

int SorghumGrowthStagePair::GetLeafSize(float a) const {
	if (m_left.m_leaves.size() <= m_right.m_leaves.size()) {
		return m_left.m_leaves.size() +
			glm::ceil((m_right.m_leaves.size() - m_left.m_leaves.size()) * a);
	}
	return m_left.m_leaves.size();
}
float SorghumGrowthStagePair::GetStemLength(float a) const {
	float leftLength, rightLength;
	switch ((StateMode)m_mode) {
	case StateMode::Default:
		leftLength = m_left.m_stem.m_length;
		rightLength = m_right.m_stem.m_length;
		break;
	case StateMode::CubicBezier:
		if (!m_left.m_stem.m_spline.m_curves.empty()) {
			leftLength = glm::distance(m_left.m_stem.m_spline.m_curves.front().m_p0,
				m_left.m_stem.m_spline.m_curves.back().m_p3);
		}
		else {
			leftLength = 0.0f;
		}
		if (!m_right.m_stem.m_spline.m_curves.empty()) {
			rightLength = glm::distance(m_right.m_stem.m_spline.m_curves.front().m_p0,
				m_right.m_stem.m_spline.m_curves.back().m_p3);
		}
		else {
			rightLength = 0.0f;
		}
		break;
	}
	return glm::mix(leftLength, rightLength, a);
}
glm::vec3 SorghumGrowthStagePair::GetStemDirection(float a) const {
	glm::vec3 leftDir, rightDir;
	switch ((StateMode)m_mode) {
	case StateMode::Default:
		leftDir = glm::normalize(m_left.m_stem.m_direction);
		rightDir = glm::normalize(m_right.m_stem.m_direction);
		break;
	case StateMode::CubicBezier:
		if (!m_left.m_stem.m_spline.m_curves.empty()) {
			leftDir = glm::vec3(0.0f, 1.0f, 0.0f);
		}
		else {
			leftDir = glm::vec3(0.0f, 1.0f, 0.0f);
		}
		if (!m_right.m_stem.m_spline.m_curves.empty()) {
			rightDir = glm::vec3(0.0f, 1.0f, 0.0f);
		}
		else {
			rightDir = glm::vec3(0.0f, 1.0f, 0.0f);
		}
		break;
	}

	return glm::normalize(glm::mix(leftDir, rightDir, a));
}
glm::vec3 SorghumGrowthStagePair::GetStemPoint(float a, float point) const {
	glm::vec3 leftPoint, rightPoint;
	switch ((StateMode)m_mode) {
	case StateMode::Default:
		leftPoint = glm::normalize(m_left.m_stem.m_direction) * point * m_left.m_stem.m_length;
		rightPoint = glm::normalize(m_right.m_stem.m_direction) * point * m_right.m_stem.m_length;
		break;
	case StateMode::CubicBezier:
		if (!m_left.m_stem.m_spline.m_curves.empty()) {
			leftPoint = m_left.m_stem.m_spline.EvaluatePointFromCurves(point);
		}
		else {
			leftPoint = glm::vec3(0.0f, 0.0f, 0.0f);
		}
		if (!m_right.m_stem.m_spline.m_curves.empty()) {
			rightPoint = m_right.m_stem.m_spline.EvaluatePointFromCurves(point);
		}
		else {
			rightPoint = glm::vec3(0.0f, 0.0f, 0.0f);
		}
		break;
	}

	return glm::mix(leftPoint, rightPoint, a);
}

void SorghumGrowthStagePair::ApplyPanicle(SorghumGrowthStage& targetState, const float a) const
{
	targetState.m_panicle.m_panicleSize = glm::mix(m_left.m_panicle.m_panicleSize, m_right.m_panicle.m_panicleSize, a);
	targetState.m_panicle.m_seedAmount = glm::mix(m_left.m_panicle.m_seedAmount, m_right.m_panicle.m_seedAmount, a);
	targetState.m_panicle.m_seedRadius = glm::mix(m_left.m_panicle.m_seedRadius, m_right.m_panicle.m_seedRadius, a);
}

void SorghumGrowthStagePair::ApplyStem(SorghumGrowthStage& targetState, float a) const
{

}

bool SorghumPanicleGrowthStage::OnInspect() {
	bool changed = false;
	if (ImGui::DragFloat("Panicle width", &m_panicleSize.x, 0.001f)) {
		changed = true;
		m_panicleSize.z = m_panicleSize.x;
	}
	if (ImGui::DragFloat("Panicle height", &m_panicleSize.y, 0.001f))
		changed = true;
	if (ImGui::DragInt("Num of seeds", &m_seedAmount, 1.0f))
		changed = true;
	if (ImGui::DragFloat("Seed radius", &m_seedRadius, 0.0001f))
		changed = true;
	if (changed)
		m_saved = false;
	return changed;
}
void SorghumPanicleGrowthStage::Serialize(YAML::Emitter& out) {
	out << YAML::Key << "m_panicleSize" << YAML::Value << m_panicleSize;
	out << YAML::Key << "m_seedAmount" << YAML::Value << m_seedAmount;
	out << YAML::Key << "m_seedRadius" << YAML::Value << m_seedRadius;
	m_saved = true;
}
void SorghumPanicleGrowthStage::Deserialize(const YAML::Node& in) {
	if (in["m_panicleSize"])
		m_panicleSize = in["m_panicleSize"].as<glm::vec3>();
	if (in["m_seedAmount"])
		m_seedAmount = in["m_seedAmount"].as<int>();
	if (in["m_seedRadius"])
		m_seedRadius = in["m_seedRadius"].as<float>();
	m_saved = true;
}

SorghumPanicleGrowthStage::SorghumPanicleGrowthStage() {
	m_panicleSize = glm::vec3(0, 0, 0);
	m_seedAmount = 0;
	m_seedRadius = 0.002f;
	m_saved = false;
}
void SorghumStemGrowthStage::Serialize(YAML::Emitter& out) {
	out << YAML::Key << "m_direction" << YAML::Value << m_direction;
	m_widthAlongStem.Save("m_widthAlongStem", out);
	out << YAML::Key << "m_length" << YAML::Value << m_length;
	out << YAML::Key << "m_spline" << YAML::Value << YAML::BeginMap;
	m_spline.Serialize(out);
	out << YAML::EndMap;

	m_saved = true;
}
void SorghumStemGrowthStage::Deserialize(const YAML::Node& in) {
	if (in["m_spline"]) {
		m_spline.Deserialize(in["m_spline"]);
	}

	if (in["m_direction"])
		m_direction = in["m_direction"].as<glm::vec3>();
	if (in["m_length"])
		m_length = in["m_length"].as<float>();
	m_widthAlongStem.Load("m_widthAlongStem", in);

	m_saved = true;
}
bool SorghumStemGrowthStage::OnInspect(int mode) {
	bool changed = false;
	switch ((StateMode)mode) {
	case StateMode::Default:
		// ImGui::DragFloat3("Direction", &m_direction.x, 0.01f);
		if (ImGui::DragFloat("Length", &m_length, 0.01f))
			changed = true;
		break;
	case StateMode::CubicBezier:
		if (ImGui::TreeNode("Spline")) {
			m_spline.OnInspect();
			ImGui::TreePop();
		}
		break;
	}
	if (m_widthAlongStem.OnInspect("Width along stem"))
		changed = true;

	if (changed)
		m_saved = false;
	return changed;
}
bool SorghumLeafGrowthStage::OnInspect(int mode) {
	bool changed = false;
	if (ImGui::Checkbox("Dead", &m_dead)) {
		changed = true;
		if (!m_dead && m_length == 0.0f)
			m_length = 0.35f;
	}
	if (!m_dead) {
		if (ImGui::InputFloat("Starting point", &m_startingPoint)) {
			m_startingPoint = glm::clamp(m_startingPoint, 0.0f, 1.0f);
			changed = true;
		}
		switch ((StateMode)mode) {
		case StateMode::Default:
			if (ImGui::TreeNodeEx("Geometric", ImGuiTreeNodeFlags_DefaultOpen)) {
				if (ImGui::DragFloat("Length", &m_length, 0.01f, 0.0f, 999.0f))
					changed = true;
				if (ImGui::TreeNodeEx("Angles", ImGuiTreeNodeFlags_DefaultOpen)) {
					if (ImGui::DragFloat("Roll angle", &m_rollAngle, 1.0f, -999.0f,
						999.0f))
						changed = true;
					if (ImGui::InputFloat("Branching angle", &m_branchingAngle)) {
						m_branchingAngle = glm::clamp(m_branchingAngle, 0.0f, 180.0f);
						changed = true;
					}
					ImGui::TreePop();
				}
				ImGui::TreePop();
			}
			break;
		case StateMode::CubicBezier:
			if (ImGui::TreeNodeEx("Geometric", ImGuiTreeNodeFlags_DefaultOpen)) {
				m_spline.OnInspect();
				ImGui::TreePop();
			}
			break;
		}

		if (ImGui::TreeNodeEx("Others")) {
			if (m_widthAlongLeaf.OnInspect("Width"))
				changed = true;
			if (m_curlingAlongLeaf.OnInspect("Rolling"))
				changed = true;

			static CurveDescriptorSettings leafBending = {
				1.0f, false, true,
				"The bending of the leaf, controls how leaves bend because of "
				"gravity. Positive value results in leaf bending towards the "
				"ground, negative value results in leaf bend towards the sky" };

			if (m_bendingAlongLeaf.OnInspect("Bending along leaf", leafBending)) {
				changed = true;
				m_bendingAlongLeaf.m_curve.UnsafeGetValues()[1].y = 0.5f;
			}
			if (m_wavinessAlongLeaf.OnInspect("Waviness along leaf"))
				changed = true;

			if (ImGui::DragFloat2("Waviness frequency", &m_wavinessFrequency.x, 0.01f,
				0.0f, 999.0f))
				changed = true;
			if (ImGui::DragFloat2("Waviness start period", &m_wavinessPeriodStart.x,
				0.01f, 0.0f, 999.0f))
				changed = true;
			ImGui::TreePop();
		}
	}
	if (changed)
		m_saved = false;
	return changed;
}
void SorghumLeafGrowthStage::Serialize(YAML::Emitter& out) {

	out << YAML::Key << "m_dead" << YAML::Value << m_dead;
	out << YAML::Key << "m_index" << YAML::Value << m_index;
	if (!m_dead) {
		out << YAML::Key << "m_spline" << YAML::Value << YAML::BeginMap;
		m_spline.Serialize(out);
		out << YAML::EndMap;

		out << YAML::Key << "m_startingPoint" << YAML::Value << m_startingPoint;
		out << YAML::Key << "m_length" << YAML::Value << m_length;
		m_curlingAlongLeaf.Save("m_curlingAlongLeaf", out);
		m_widthAlongLeaf.Save("m_widthAlongLeaf", out);
		out << YAML::Key << "m_rollAngle" << YAML::Value << m_rollAngle;
		out << YAML::Key << "m_branchingAngle" << YAML::Value << m_branchingAngle;
		m_bendingAlongLeaf.Save("m_bendingAlongLeaf", out);
		m_wavinessAlongLeaf.Save("m_wavinessAlongLeaf", out);
		out << YAML::Key << "m_wavinessFrequency" << YAML::Value
			<< m_wavinessFrequency;
		out << YAML::Key << "m_wavinessPeriodStart" << YAML::Value
			<< m_wavinessPeriodStart;
	}

	m_saved = true;
}
void SorghumLeafGrowthStage::Deserialize(const YAML::Node& in) {
	if (in["m_index"])
		m_index = in["m_index"].as<int>();
	if (in["m_dead"])
		m_dead = in["m_dead"].as<bool>();
	if (!m_dead) {
		if (in["m_spline"]) {
			m_spline.Deserialize(in["m_spline"]);
		}

		if (in["m_startingPoint"])
			m_startingPoint = in["m_startingPoint"].as<float>();
		if (in["m_length"])
			m_length = in["m_length"].as<float>();
		if (in["m_rollAngle"])
			m_rollAngle = in["m_rollAngle"].as<float>();
		if (in["m_branchingAngle"])
			m_branchingAngle = in["m_branchingAngle"].as<float>();
		if (in["m_wavinessFrequency"])
			m_wavinessFrequency = in["m_wavinessFrequency"].as<glm::vec2>();
		if (in["m_wavinessPeriodStart"])
			m_wavinessPeriodStart = in["m_wavinessPeriodStart"].as<glm::vec2>();

		m_curlingAlongLeaf.Load("m_curlingAlongLeaf", in);
		m_bendingAlongLeaf.Load("m_bendingAlongLeaf", in);
		m_widthAlongLeaf.Load("m_widthAlongLeaf", in);
		m_wavinessAlongLeaf.Load("m_wavinessAlongLeaf", in);
	}

	m_saved = true;
}
SorghumStemGrowthStage::SorghumStemGrowthStage() {
	m_length = 0.35f;
	m_widthAlongStem = { 0.0f, 0.015f, {0.6f, 0.4f, {0, 0}, {1, 1}} };

	m_saved = false;
}

SorghumLeafGrowthStage::SorghumLeafGrowthStage() {
	m_dead = false;
	m_wavinessAlongLeaf = { 0.0f, 5.0f, {0.0f, 0.5f, {0, 0}, {1, 1}} };
	m_wavinessFrequency = { 30.0f, 30.0f };
	m_wavinessPeriodStart = { 0.0f, 0.0f };
	m_widthAlongLeaf = { 0.0f, 0.02f, {0.5f, 0.1f, {0, 0}, {1, 1}} };
	auto& pairs = m_widthAlongLeaf.m_curve.UnsafeGetValues();
	pairs.clear();
	pairs.emplace_back(-0.1, 0.0f);
	pairs.emplace_back(0, 0.5);
	pairs.emplace_back(0.11196319, 0.111996889);

	pairs.emplace_back(-0.0687116608, 0);
	pairs.emplace_back(0.268404901, 0.92331290);
	pairs.emplace_back(0.100000001, 0.0f);

	pairs.emplace_back(-0.100000001, 0);
	pairs.emplace_back(0.519368708, 1);
	pairs.emplace_back(0.100000001, 0);

	pairs.emplace_back(-0.100000001, 0.0f);
	pairs.emplace_back(1, 0.1);
	pairs.emplace_back(0.1, 0.0f);

	m_bendingAlongLeaf = { -180.0f, 180.0f, {0.5f, 0.5, {0, 0}, {1, 1}} };
	m_curlingAlongLeaf = { 0.0f, 90.0f, {0.3f, 0.3f, {0, 0}, {1, 1}} };
	m_length = 0.35f;
	m_branchingAngle = 30.0f;

	m_saved = false;
}
void SorghumLeafGrowthStage::CopyShape(const SorghumLeafGrowthStage& another) {
	m_spline = another.m_spline;
	m_widthAlongLeaf.m_curve = another.m_widthAlongLeaf.m_curve;
	m_curlingAlongLeaf = another.m_curlingAlongLeaf;
	m_bendingAlongLeaf = another.m_bendingAlongLeaf;
	m_wavinessAlongLeaf = another.m_wavinessAlongLeaf;
	m_wavinessPeriodStart = another.m_wavinessPeriodStart;
	m_wavinessFrequency = another.m_wavinessFrequency;

	m_saved = false;
}

bool SorghumGrowthStage::OnInspect(int mode) {
	bool changed = false;
	if (ImGui::TreeNodeEx((std::string("Stem")).c_str())) {
		if (m_stem.OnInspect(mode))
			changed = true;
		ImGui::TreePop();
	}

	if (ImGui::TreeNodeEx("Leaves")) {
		int leafSize = m_leaves.size();
		if (ImGui::InputInt("Number of leaves", &leafSize)) {
			changed = true;
			leafSize = glm::clamp(leafSize, 0, 999);
			auto previousSize = m_leaves.size();
			m_leaves.resize(leafSize);
			for (int i = 0; i < leafSize; i++) {
				if (i >= previousSize) {
					if (i - 1 >= 0) {
						m_leaves[i] = m_leaves[i - 1];
						m_leaves[i].m_rollAngle =
							glm::mod(m_leaves[i - 1].m_rollAngle + 180.0f, 360.0f);
						m_leaves[i].m_startingPoint =
							m_leaves[i - 1].m_startingPoint + 0.1f;
					}
					else {
						m_leaves[i] = SorghumLeafGrowthStage();
						m_leaves[i].m_rollAngle = 0;
						m_leaves[i].m_startingPoint = 0.1f;
					}
				}
				m_leaves[i].m_index = i;
			}
		}
		for (auto& leaf : m_leaves) {
			if (ImGui::TreeNode(
				("Leaf No." + std::to_string(leaf.m_index + 1) +
					(leaf.m_length == 0.0f || leaf.m_dead ? " (Dead)" : ""))
				.c_str())) {
				if (leaf.OnInspect(mode))
					changed = true;
				ImGui::TreePop();
			}
		}
		ImGui::TreePop();
	}

	if (ImGui::TreeNodeEx((std::string("Panicle")).c_str())) {
		if (m_panicle.OnInspect())
			changed = true;
		ImGui::TreePop();
	}
	if (mode == (int)StateMode::CubicBezier) {
		FileUtils::OpenFile(
			"Import...", "TXT", { ".txt" },
			[&](const std::filesystem::path& path) {
				std::ifstream file(path, std::fstream::in);
				if (!file.is_open()) {
					EVOENGINE_LOG("Failed to open file!");
					return;
				}
				changed = true;
				// Number of leaves in the file
				int leafCount;
				file >> leafCount;
				m_stem = SorghumStemGrowthStage();
				m_stem.m_spline.Import(file);
				/*
				// Recenter plant:
				glm::vec3 posSum = m_stem.m_spline.m_curves.front().m_p0;
				for (auto &curve : m_stem.m_spline.m_curves) {
				  curve.m_p0 -= posSum;
				  curve.m_p1 -= posSum;
				  curve.m_p2 -= posSum;
				  curve.m_p3 -= posSum;
				}
				*/
				m_leaves.resize(leafCount);
				for (int i = 0; i < leafCount; i++) {
					float startingPoint;
					file >> startingPoint;
					m_leaves[i] = SorghumLeafGrowthStage();
					m_leaves[i].m_startingPoint = startingPoint;
					m_leaves[i].m_spline.Import(file);
					m_leaves[i].m_spline.m_curves[0].m_p0 =
						m_stem.m_spline.EvaluatePointFromCurves(startingPoint);
				}

				for (int i = 0; i < leafCount; i++) {
					m_leaves[i].m_index = i;
				}
			},
			false);
	}
	if (changed)
		m_saved = false;
	return changed;
}

void SorghumGrowthStage::Serialize(YAML::Emitter& out) {

	out << YAML::Key << "m_version" << YAML::Value << m_version;
	out << YAML::Key << "m_name" << YAML::Value << m_name;
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
void SorghumGrowthStage::Deserialize(const YAML::Node& in) {
	if (in["m_version"])
		m_version = in["m_version"].as<unsigned>();
	if (in["m_name"])
		m_name = in["m_name"].as<std::string>();
	if (in["m_panicle"])
		m_panicle.Deserialize(in["m_panicle"]);

	if (in["m_stem"])
		m_stem.Deserialize(in["m_stem"]);

	if (in["m_leaves"]) {
		for (const auto& i : in["m_leaves"]) {
			SorghumLeafGrowthStage leaf;
			leaf.Deserialize(i);
			m_leaves.push_back(leaf);
		}
	}
	m_saved = true;
}
SorghumGrowthStage::SorghumGrowthStage() {
	m_saved = false;
	m_name = "Unnamed";
}


