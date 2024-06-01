//
// Created by lllll on 10/27/2022.
//

#include "TreeMeshGenerator.hpp"
#include "EditorLayer.hpp"
#include "Tree.hpp"

using namespace EcoSysLab;


RingSegment::RingSegment(const float startA, const float endA, glm::vec3 startPosition, glm::vec3 endPosition, glm::vec3 startAxis,
	glm::vec3 endAxis, float startRadius, float endRadius, float startDistanceToRoot, float endDistanceToRoot)
	:
	m_startA(startA),
	m_endA(endA),
	m_startPosition(startPosition),
	m_endPosition(endPosition),
	m_startAxis(startAxis),
	m_endAxis(endAxis),
	m_startRadius(startRadius),
	m_endRadius(endRadius),
	m_startDistanceToRoot(startDistanceToRoot),
	m_endDistanceToRoot(endDistanceToRoot) {
}

void RingSegment::AppendPoints(std::vector<Vertex>& vertices, glm::vec3& normalDir, int step) {
	std::vector<Vertex> startRing;
	std::vector<Vertex> endRing;

	float angleStep = 360.0f / (float)(step);
	Vertex archetype;
	for (int i = 0; i < step; i++) {
		archetype.m_position = GetPoint(normalDir, angleStep * i, true);
		startRing.push_back(archetype);
	}
	for (int i = 0; i < step; i++) {
		archetype.m_position = GetPoint(normalDir, angleStep * i, false);
		endRing.push_back(archetype);
	}
	float textureXstep = 1.0f / step * 4;
	for (int i = 0; i < step - 1; i++) {
		float x = (i % step) * textureXstep;
		startRing[i].m_texCoord = glm::vec2(x, 0.0f);
		startRing[i + 1].m_texCoord = glm::vec2(x + textureXstep, 0.0f);
		endRing[i].m_texCoord = glm::vec2(x, 1.0f);
		endRing[i + 1].m_texCoord = glm::vec2(x + textureXstep, 1.0f);
		vertices.push_back(startRing[i]);
		vertices.push_back(startRing[i + 1]);
		vertices.push_back(endRing[i]);
		vertices.push_back(endRing[i + 1]);
		vertices.push_back(endRing[i]);
		vertices.push_back(startRing[i + 1]);
	}
	startRing[step - 1].m_texCoord = glm::vec2(1.0f - textureXstep, 0.0f);
	startRing[0].m_texCoord = glm::vec2(1.0f, 0.0f);
	endRing[step - 1].m_texCoord = glm::vec2(1.0f - textureXstep, 1.0f);
	endRing[0].m_texCoord = glm::vec2(1.0f, 1.0f);
	vertices.push_back(startRing[step - 1]);
	vertices.push_back(startRing[0]);
	vertices.push_back(endRing[step - 1]);
	vertices.push_back(endRing[0]);
	vertices.push_back(endRing[step - 1]);
	vertices.push_back(startRing[0]);
}

glm::vec3 RingSegment::GetPoint(const glm::vec3& normalDir, const float angle, const bool isStart, const float multiplier) const
{
	const auto direction = GetDirection(normalDir, angle, isStart);
	const auto radius = isStart ? m_startRadius : m_endRadius;
	const glm::vec3 position = (isStart ? m_startPosition : m_endPosition) + 
		direction * multiplier * radius;
	return position;
}

glm::vec3 RingSegment::GetDirection(const glm::vec3& normalDir, float angle, const bool isStart) const
{
	glm::vec3 direction = glm::cross(normalDir, isStart ? this->m_startAxis : this->m_endAxis);
	direction = glm::rotate(direction, glm::radians(angle), isStart ? this->m_startAxis : this->m_endAxis);
	direction = glm::normalize(direction);
	return direction;
}

void TreeMeshGeneratorSettings::Save(const std::string& name, YAML::Emitter& out) {
	out << YAML::Key << name << YAML::Value << YAML::BeginMap;
	out << YAML::Key << "m_xSubdivision" << YAML::Value << m_xSubdivision;
	out << YAML::Key << "m_trunkYSubdivision" << YAML::Value << m_trunkYSubdivision;
	out << YAML::Key << "m_trunkThickness" << YAML::Value << m_trunkThickness;
	out << YAML::Key << "m_branchYSubdivision" << YAML::Value << m_branchYSubdivision;

	out << YAML::Key << "m_enableFoliage" << YAML::Value << m_enableFoliage;
	out << YAML::Key << "m_foliageInstancing" << YAML::Value << m_foliageInstancing;
	out << YAML::Key << "m_enableBranch" << YAML::Value << m_enableBranch;
	out << YAML::Key << "m_enableFruit" << YAML::Value << m_enableFruit;

	out << YAML::Key << "m_smoothness" << YAML::Value << m_smoothness;
	out << YAML::Key << "m_overrideRadius" << YAML::Value << m_overrideRadius;
	out << YAML::Key << "m_boundaryRadius" << YAML::Value << m_radius;
	out << YAML::Key << "m_baseControlPointRatio" << YAML::Value << m_baseControlPointRatio;
	out << YAML::Key << "m_branchControlPointRatio" << YAML::Value << m_branchControlPointRatio;
	out << YAML::Key << "m_treePartEndDistance" << YAML::Value << m_treePartEndDistance;
	out << YAML::Key << "m_treePartBaseDistance" << YAML::Value << m_treePartBaseDistance;
	
	out << YAML::Key << "m_autoLevel" << YAML::Value << m_autoLevel;
	out << YAML::Key << "m_voxelSubdivisionLevel" << YAML::Value << m_voxelSubdivisionLevel;
	out << YAML::Key << "m_voxelSmoothIteration" << YAML::Value << m_voxelSmoothIteration;
	out << YAML::Key << "m_removeDuplicate" << YAML::Value << m_removeDuplicate;

	out << YAML::Key << "m_branchMeshType" << YAML::Value << m_branchMeshType;

	

	out << YAML::EndMap;

	
}

void TreeMeshGeneratorSettings::Load(const std::string& name, const YAML::Node& in) {
	if (in[name]) {
		const auto& ms = in[name];
		if (ms["m_xSubdivision"]) m_xSubdivision = ms["m_xSubdivision"].as<float>();
		if (ms["m_trunkYSubdivision"]) m_trunkYSubdivision = ms["m_trunkYSubdivision"].as<float>();
		if (ms["m_trunkThickness"]) m_trunkThickness = ms["m_trunkThickness"].as<float>();
		if (ms["m_branchYSubdivision"]) m_branchYSubdivision = ms["m_branchYSubdivision"].as<float>();

		if (ms["m_enableFoliage"]) m_enableFoliage = ms["m_enableFoliage"].as<bool>();
		if (ms["m_foliageInstancing"]) m_foliageInstancing = ms["m_foliageInstancing"].as<bool>();
		if (ms["m_enableBranch"]) m_enableBranch = ms["m_enableBranch"].as<bool>();
		if (ms["m_enableFruit"]) m_enableFruit = ms["m_enableFruit"].as<bool>();

		if (ms["m_smoothness"]) m_smoothness = ms["m_smoothness"].as<bool>();
		if (ms["m_overrideRadius"]) m_overrideRadius = ms["m_overrideRadius"].as<bool>();
		if (ms["m_boundaryRadius"]) m_radius = ms["m_boundaryRadius"].as<float>();
		if (ms["m_baseControlPointRatio"]) m_baseControlPointRatio = ms["m_baseControlPointRatio"].as<float>();
		if (ms["m_branchControlPointRatio"]) m_branchControlPointRatio = ms["m_branchControlPointRatio"].as<float>();
		if (ms["m_treePartEndDistance"]) m_treePartEndDistance = ms["m_treePartEndDistance"].as<float>();
		if (ms["m_treePartBaseDistance"]) m_treePartBaseDistance = ms["m_treePartBaseDistance"].as<float>();

		if (ms["m_autoLevel"]) m_autoLevel = ms["m_autoLevel"].as<bool>();
		if (ms["m_voxelSubdivisionLevel"]) m_voxelSubdivisionLevel = ms["m_voxelSubdivisionLevel"].as<int>();
		if (ms["m_voxelSmoothIteration"]) m_voxelSmoothIteration = ms["m_voxelSmoothIteration"].as<int>();
		if (ms["m_removeDuplicate"]) m_removeDuplicate = ms["m_removeDuplicate"].as<bool>();

		if (ms["m_branchMeshType"]) m_branchMeshType = ms["m_branchMeshType"].as<unsigned>();
	}
}

void TreeMeshGeneratorSettings::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
	if (ImGui::TreeNodeEx("Mesh Generator settings")) {
		ImGui::Checkbox("Branch", &m_enableBranch);
		ImGui::Checkbox("Fruit", &m_enableFruit);
		ImGui::Checkbox("Foliage", &m_enableFoliage);
		ImGui::Checkbox("Foliage instancing", &m_foliageInstancing);
		ImGui::Combo("Branch mesh mode", { "Cylindrical", "Marching cubes" }, m_branchMeshType);

		ImGui::Combo("Branch color mode", { "Internode Color", "Junction" }, m_vertexColorMode);
		
		if(ImGui::TreeNode("Cylindrical mesh settings"))
		{
			ImGui::DragFloat("Trunk Thickness Threshold", &m_trunkThickness, 1.0f, 0.0f, 16.0f);
			ImGui::DragFloat("X Step", &m_xSubdivision, 0.00001f, 0.00001f, 1.0f, "%.5f");
			ImGui::DragFloat("Trunk Y Step", &m_trunkYSubdivision, 0.00001f, 0.00001f, 1.0f, "%.5f");
			ImGui::DragFloat("Branch Y Step", &m_branchYSubdivision, 0.00001f, 0.00001f, 1.0f, "%.5f");

			ImGui::Checkbox("Smoothness", &m_smoothness);
			if (m_smoothness) {
				ImGui::DragFloat("Base control point ratio", &m_baseControlPointRatio, 0.001f, 0.0f, 1.0f);
				ImGui::DragFloat("Branch control point ratio", &m_branchControlPointRatio, 0.001f, 0.0f, 1.0f);
			}
			ImGui::Checkbox("Override radius", &m_overrideRadius);
			if (m_overrideRadius) ImGui::DragFloat("Radius", &m_radius);
			ImGui::DragFloat("Tree Part Base Distance", &m_treePartBaseDistance, 1, 0, 10);
			ImGui::DragFloat("Tree Part End Distance", &m_treePartEndDistance, 1, 0, 10);
			ImGui::TreePop();
		}
		if(ImGui::TreeNode("Marching cubes settings"))
		{
			ImGui::Checkbox("Auto set level", &m_autoLevel);
			if (!m_autoLevel) ImGui::DragInt("Voxel subdivision level", &m_voxelSubdivisionLevel, 1, 5, 16);
			else ImGui::DragFloat("Min Cube size", &m_marchingCubeRadius, 0.0001, 0.001f, 1.0f);
			ImGui::DragInt("Smooth iteration", &m_voxelSmoothIteration, 0, 0, 10);
			if (m_voxelSmoothIteration == 0) ImGui::Checkbox("Remove duplicate", &m_removeDuplicate);
			ImGui::TreePop();
		}
		if (m_enableBranch && ImGui::TreeNode("Branch settings")) {
			ImGui::TreePop();
		}
		if (m_enableFoliage && ImGui::TreeNode("Foliage settings"))
		{

			ImGui::TreePop();
		}
		
		ImGui::Checkbox("Mesh Override", &m_presentationOverride);
		if (m_presentationOverride && ImGui::TreeNodeEx("Override settings"))
		{
			ImGui::DragFloat("Max thickness", &m_presentationOverrideSettings.m_maxThickness, 0.01f, 0.0f, 1.0f);
			
			ImGui::TreePop();
		}
		ImGui::TreePop();
	}
}


