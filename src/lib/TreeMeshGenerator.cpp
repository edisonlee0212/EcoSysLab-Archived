//
// Created by lllll on 10/27/2022.
//

#include "TreeMeshGenerator.hpp"
#include "EditorLayer.hpp"
#include "Tree.hpp"

using namespace EcoSysLab;


void SerializeTwigParameters(const std::string& name, const TwigParameters& twigParameters, YAML::Emitter& out)
{
	out << YAML::Key << name << YAML::BeginMap;
	out << YAML::Key << "m_segmentLength" << YAML::Value << twigParameters.m_segmentLength;
	out << YAML::Key << "m_apicalAngleVariance" << YAML::Value << twigParameters.m_apicalAngleVariance;
	out << YAML::Key << "m_branchingAngle" << YAML::Value << twigParameters.m_branchingAngle;
	out << YAML::Key << "m_thickness" << YAML::Value << twigParameters.m_thickness;
	out << YAML::Key << "m_maxNodeThickness" << YAML::Value << twigParameters.m_maxNodeThickness;
	out << YAML::Key << "m_minRootDistance" << YAML::Value << twigParameters.m_minRootDistance;
	out << YAML::Key << "m_maxEndDistance" << YAML::Value << twigParameters.m_maxEndDistance;
	out << YAML::Key << "m_segmentSize" << YAML::Value << twigParameters.m_segmentSize;
	out << YAML::Key << "m_unitDistance" << YAML::Value << twigParameters.m_unitDistance;
	out << YAML::EndMap;
}


void DeserializeTwigParameters(const std::string& name, TwigParameters& twigParameters, const YAML::Node& in) {
	if (in[name]) {
		auto& ms = in[name];

		if (ms["m_segmentLength"]) twigParameters.m_segmentLength = ms["m_segmentLength"].as<float>();
		if (ms["m_apicalAngleVariance"]) twigParameters.m_apicalAngleVariance = ms["m_apicalAngleVariance"].as<float>();
		if (ms["m_branchingAngle"]) twigParameters.m_branchingAngle = ms["m_branchingAngle"].as<float>();
		if (ms["m_thickness"]) twigParameters.m_thickness = ms["m_thickness"].as<float>();
		if (ms["m_maxNodeThickness"]) twigParameters.m_maxNodeThickness = ms["m_maxNodeThickness"].as<float>();
		if (ms["m_maxEndDistance"]) twigParameters.m_maxEndDistance = ms["m_maxEndDistance"].as<float>();
		if (ms["m_segmentSize"]) twigParameters.m_segmentSize = ms["m_segmentSize"].as<int>();
		if (ms["m_unitDistance"]) twigParameters.m_unitDistance = ms["m_unitDistance"].as<float>();
		if (ms["m_minRootDistance"]) twigParameters.m_minRootDistance = ms["m_minRootDistance"].as<float>();
	}
}


bool OnInspectTwigParameters(TwigParameters& twigParameters)
{
	bool changed = false;
	if (ImGui::TreeNodeEx("Twig settings"))
	{
		changed = ImGui::DragFloat("Segment length", &twigParameters.m_segmentLength, 0.001f, 0.0f, 1.0f) || changed;
		changed = ImGui::DragFloat("Apical angle variance", &twigParameters.m_apicalAngleVariance, 0.01f, 0.0f, 10.0f) || changed;
		changed = ImGui::DragFloat("Branching angle", &twigParameters.m_branchingAngle, 0.01f, 0.0f, 180.0f) || changed;
		changed = ImGui::DragFloat("Twig thickness", &twigParameters.m_thickness, 0.001f, 0.0f, 1.0f) || changed;
		changed = ImGui::DragInt("Segment Size", &twigParameters.m_segmentSize, 1, 2, 10) || changed;
		changed = ImGui::DragFloat("Distance between twigs", &twigParameters.m_unitDistance, 0.001f, 0.0f, 1.0f) || changed;

		changed = ImGui::DragFloat("Max node thickness", &twigParameters.m_maxNodeThickness, 0.001f, 0.0f, 5.0f) || changed;
		changed = ImGui::DragFloat("Min root distance", &twigParameters.m_minRootDistance, 0.001f, 0.0f, 1.0f) || changed;
		changed = ImGui::DragFloat("Max end distance", &twigParameters.m_maxEndDistance, 0.001f, 0.0f, 1.0f) || changed;
		ImGui::TreePop();
	}
	return changed;
}


RingSegment::RingSegment(glm::vec3 startPosition, glm::vec3 endPosition, glm::vec3 startAxis,
	glm::vec3 endAxis, float startRadius, float endRadius)
	: m_startPosition(startPosition),
	m_endPosition(endPosition),
	m_startAxis(startAxis),
	m_endAxis(endAxis),
	m_startRadius(startRadius),
	m_endRadius(endRadius) {
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

	out << YAML::Key << "m_vertexColorOnly" << YAML::Value << m_vertexColorOnly;
	out << YAML::Key << "m_enableFoliage" << YAML::Value << m_enableFoliage;
	out << YAML::Key << "m_enableBranch" << YAML::Value << m_enableBranch;
	out << YAML::Key << "m_enableFruit" << YAML::Value << m_enableFruit;
	out << YAML::Key << "m_enableTwig" << YAML::Value << m_enableTwig;

	out << YAML::Key << "m_smoothness" << YAML::Value << m_smoothness;
	out << YAML::Key << "m_overrideRadius" << YAML::Value << m_overrideRadius;
	out << YAML::Key << "m_boundaryRadius" << YAML::Value << m_radius;
	out << YAML::Key << "m_baseControlPointRatio" << YAML::Value << m_baseControlPointRatio;
	out << YAML::Key << "m_branchControlPointRatio" << YAML::Value << m_branchControlPointRatio;
	out << YAML::Key << "m_overrideVertexColor" << YAML::Value << m_overrideVertexColor;
	out << YAML::Key << "m_treePartEndDistance" << YAML::Value << m_treePartEndDistance;
	out << YAML::Key << "m_treePartBaseDistance" << YAML::Value << m_treePartBaseDistance;
	out << YAML::Key << "m_branchVertexColor" << YAML::Value << m_branchVertexColor;
	out << YAML::Key << "m_foliageVertexColor" << YAML::Value << m_foliageVertexColor;

	out << YAML::Key << "m_autoLevel" << YAML::Value << m_autoLevel;
	out << YAML::Key << "m_voxelSubdivisionLevel" << YAML::Value << m_voxelSubdivisionLevel;
	out << YAML::Key << "m_voxelSmoothIteration" << YAML::Value << m_voxelSmoothIteration;
	out << YAML::Key << "m_removeDuplicate" << YAML::Value << m_removeDuplicate;

	out << YAML::Key << "m_branchMeshType" << YAML::Value << m_branchMeshType;

	SerializeTwigParameters("m_twigParameters", m_twigParameters, out);

	out << YAML::EndMap;

	
}

void TreeMeshGeneratorSettings::Load(const std::string& name, const YAML::Node& in) {
	if (in[name]) {
		const auto& ms = in[name];
		if (ms["m_xSubdivision"]) m_xSubdivision = ms["m_xSubdivision"].as<float>();
		if (ms["m_trunkYSubdivision"]) m_trunkYSubdivision = ms["m_trunkYSubdivision"].as<float>();
		if (ms["m_trunkThickness"]) m_trunkThickness = ms["m_trunkThickness"].as<float>();
		if (ms["m_branchYSubdivision"]) m_branchYSubdivision = ms["m_branchYSubdivision"].as<float>();

		if (ms["m_vertexColorOnly"]) m_vertexColorOnly = ms["m_vertexColorOnly"].as<bool>();
		if (ms["m_enableFoliage"]) m_enableFoliage = ms["m_enableFoliage"].as<bool>();
		if (ms["m_enableBranch"]) m_enableBranch = ms["m_enableBranch"].as<bool>();
		if (ms["m_enableFruit"]) m_enableFruit = ms["m_enableFruit"].as<bool>();
		if (ms["m_enableTwig"]) m_enableTwig = ms["m_enableTwig"].as<bool>();

		if (ms["m_smoothness"]) m_smoothness = ms["m_smoothness"].as<bool>();
		if (ms["m_overrideRadius"]) m_overrideRadius = ms["m_overrideRadius"].as<bool>();
		if (ms["m_boundaryRadius"]) m_radius = ms["m_boundaryRadius"].as<float>();
		if (ms["m_baseControlPointRatio"]) m_baseControlPointRatio = ms["m_baseControlPointRatio"].as<float>();
		if (ms["m_branchControlPointRatio"]) m_branchControlPointRatio = ms["m_branchControlPointRatio"].as<float>();
		if (ms["m_overrideVertexColor"]) m_overrideVertexColor = ms["m_overrideVertexColor"].as<bool>();
		if (ms["m_treePartEndDistance"]) m_treePartEndDistance = ms["m_treePartEndDistance"].as<float>();
		if (ms["m_treePartBaseDistance"]) m_treePartBaseDistance = ms["m_treePartBaseDistance"].as<float>();
		if (ms["m_branchVertexColor"]) m_branchVertexColor = ms["m_branchVertexColor"].as<glm::vec3>();
		if (ms["m_foliageVertexColor"]) m_foliageVertexColor = ms["m_foliageVertexColor"].as<glm::vec3>();


		if (ms["m_autoLevel"]) m_autoLevel = ms["m_autoLevel"].as<bool>();
		if (ms["m_voxelSubdivisionLevel"]) m_voxelSubdivisionLevel = ms["m_voxelSubdivisionLevel"].as<int>();
		if (ms["m_voxelSmoothIteration"]) m_voxelSmoothIteration = ms["m_voxelSmoothIteration"].as<int>();
		if (ms["m_removeDuplicate"]) m_removeDuplicate = ms["m_removeDuplicate"].as<bool>();

		if (ms["m_branchMeshType"]) m_branchMeshType = ms["m_branchMeshType"].as<unsigned>();
		DeserializeTwigParameters("m_twigParameters", m_twigParameters, ms);
	}

	
}

void TreeMeshGeneratorSettings::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
	if (ImGui::TreeNodeEx("Mesh Generator settings")) {
		ImGui::Checkbox("Vertex color only", &m_vertexColorOnly);
		ImGui::Checkbox("Branch", &m_enableBranch);
		ImGui::Checkbox("Fruit", &m_enableFruit);
		ImGui::Checkbox("Foliage", &m_enableFoliage);
		ImGui::Checkbox("Twig", &m_enableTwig);
		ImGui::Combo("Branch mesh mode", { "Cylindrical", "Marching cubes" }, m_branchMeshType);
		if(m_enableTwig)
		{
			OnInspectTwigParameters(m_twigParameters);
		}
		
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
			ImGui::Checkbox("Override vertex color", &m_overrideVertexColor);
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
			
			if (m_overrideVertexColor) {
				ImGui::ColorEdit3("Branch vertex color", &m_branchVertexColor.x);
			}
			
			ImGui::TreePop();
		}
		if (m_enableFoliage && ImGui::TreeNode("Foliage settings"))
		{
			if (m_overrideVertexColor) {
				ImGui::ColorEdit3("Foliage vertex color", &m_foliageVertexColor.x);
			}
			ImGui::TreePop();
		}
		
		ImGui::Checkbox("Mesh Override", &m_presentationOverride);
		if (m_presentationOverride && ImGui::TreeNodeEx("Override settings"))
		{
			ImGui::DragFloat("Max thickness", &m_presentationOverrideSettings.m_maxThickness, 0.01f, 0.0f, 1.0f);
			ImGui::ColorEdit3("Branch color", &m_presentationOverrideSettings.m_branchOverrideColor.x);
			editorLayer->DragAndDropButton<Texture2D>(m_foliageAlbedoTexture, "Foliage Albedo Texture");
			editorLayer->DragAndDropButton<Texture2D>(m_foliageNormalTexture, "Foliage Normal Texture");
			editorLayer->DragAndDropButton<Texture2D>(m_foliageRoughnessTexture, "Foliage Roughness Texture");
			editorLayer->DragAndDropButton<Texture2D>(m_foliageMetallicTexture, "Foliage Metallic Texture");


			editorLayer->DragAndDropButton<Texture2D>(m_branchAlbedoTexture, "Branch Albedo Texture");
			editorLayer->DragAndDropButton<Texture2D>(m_branchNormalTexture, "Branch Normal Texture");
			editorLayer->DragAndDropButton<Texture2D>(m_branchRoughnessTexture, "Branch Roughness Texture");
			editorLayer->DragAndDropButton<Texture2D>(m_branchMetallicTexture, "Branch Metallic Texture");

			ImGui::TreePop();
		}
		ImGui::Checkbox("Foliage Override", &m_foliageOverride);
		if(m_foliageOverride && ImGui::TreeNodeEx("Foliage Override settings"))
		{
			TreeDescriptor::OnInspectFoliageParameters(m_foliageOverrideSettings);
			ImGui::TreePop();
		}
		ImGui::TreePop();
	}
}


