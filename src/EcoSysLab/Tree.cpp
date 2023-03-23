//
// Created by lllll on 10/24/2022.
//

#include "Tree.hpp"
#include "Graphics.hpp"
#include "EditorLayer.hpp"
#include "Application.hpp"
#include "TreeMeshGenerator.hpp"
#include "Soil.hpp"
#include "Climate.hpp"
#include "Octree.hpp"
#include "EcoSysLabLayer.hpp"
#include "HeightField.hpp"
using namespace EcoSysLab;

void Tree::OnInspect() {
	bool modelChanged = false;
	if (Editor::DragAndDropButton<TreeDescriptor>(m_treeDescriptor, "TreeDescriptor", true)) {
		m_treeModel.Clear();
		modelChanged = true;
	}
	if (m_treeDescriptor.Get<TreeDescriptor>()) {
		ImGui::Checkbox("Enable History", &m_enableHistory);
		if (m_enableHistory)
		{
			ImGui::DragInt("History per iteration", &m_historyIteration, 1, 1, 1000);
		}
		ImGui::DragInt("Flow max node size", &m_treeModel.m_treeGrowthSettings.m_flowNodeLimit, 1, 1, 100);
		ImGui::Checkbox("Auto balance vigor", &m_treeModel.m_treeGrowthSettings.m_autoBalance);
		ImGui::Checkbox("Receive light", &m_treeModel.m_treeGrowthSettings.m_collectLight);
		ImGui::Checkbox("Receive water", &m_treeModel.m_treeGrowthSettings.m_collectWater);
		ImGui::Checkbox("Receive nitrite", &m_treeModel.m_treeGrowthSettings.m_collectNitrite);
		ImGui::Checkbox("Enable Branch collision detection", &m_treeModel.m_treeGrowthSettings.m_enableBranchCollisionDetection);
		ImGui::Checkbox("Enable Root collision detection", &m_treeModel.m_treeGrowthSettings.m_enableRootCollisionDetection);
		if(!m_treeModel.m_initialized)
		{
			ImGui::Checkbox("Enable pipe", &m_treeModel.m_enablePipe);
		}

		if(m_treeModel.m_enablePipe && ImGui::Button("Build strands"))
		{
			InitializeStrandRenderer();
		}

		if(!m_treeModel.m_treeGrowthSettings.m_collectLight && !m_treeModel.m_treeGrowthSettings.m_collectWater)
		{
			if(ImGui::TreeNode("Vigor filling rates"))
			{
				if(ImGui::SliderFloat("Leaf maintenance", &m_treeModel.m_treeGrowthSettings.m_leafMaintenanceVigorFillingRate, 0.0f, 1.0f))
				{
					if (m_treeModel.m_treeGrowthSettings.m_leafMaintenanceVigorFillingRate != 1.0f)
					{
						m_treeModel.m_treeGrowthSettings.m_leafDevelopmentalVigorFillingRate
							= m_treeModel.m_treeGrowthSettings.m_fruitMaintenanceVigorFillingRate
							= m_treeModel.m_treeGrowthSettings.m_fruitDevelopmentalVigorFillingRate
							= m_treeModel.m_treeGrowthSettings.m_nodeDevelopmentalVigorFillingRate = 0.0f;
					}
				}
				if (m_treeModel.m_treeGrowthSettings.m_leafMaintenanceVigorFillingRate == 1.0f)
				{
					if(ImGui::SliderFloat("Leaf development", &m_treeModel.m_treeGrowthSettings.m_leafDevelopmentalVigorFillingRate, 0.0f, 1.0f))
					{
						if(m_treeModel.m_treeGrowthSettings.m_leafMaintenanceVigorFillingRate != 1.0f)
						{
							m_treeModel.m_treeGrowthSettings.m_fruitMaintenanceVigorFillingRate
								= m_treeModel.m_treeGrowthSettings.m_fruitDevelopmentalVigorFillingRate
								= m_treeModel.m_treeGrowthSettings.m_nodeDevelopmentalVigorFillingRate = 0.0f;
						}
					}
				}

				if (m_treeModel.m_treeGrowthSettings.m_leafDevelopmentalVigorFillingRate == 1.0f)
				{
					if (ImGui::SliderFloat("Fruit maintenance", &m_treeModel.m_treeGrowthSettings.m_fruitMaintenanceVigorFillingRate, 0.0f, 1.0f))
					{
						if (m_treeModel.m_treeGrowthSettings.m_leafMaintenanceVigorFillingRate != 1.0f)
						{
							m_treeModel.m_treeGrowthSettings.m_fruitDevelopmentalVigorFillingRate
								= m_treeModel.m_treeGrowthSettings.m_nodeDevelopmentalVigorFillingRate = 0.0f;
						}
					}
				}

				if (m_treeModel.m_treeGrowthSettings.m_fruitMaintenanceVigorFillingRate == 1.0f)
				{
					if (ImGui::SliderFloat("Fruit development", &m_treeModel.m_treeGrowthSettings.m_fruitDevelopmentalVigorFillingRate, 0.0f, 1.0f))
					{
						if (m_treeModel.m_treeGrowthSettings.m_leafMaintenanceVigorFillingRate != 1.0f)
						{
							m_treeModel.m_treeGrowthSettings.m_nodeDevelopmentalVigorFillingRate = 0.0f;
						}
					}
				}

				if (m_treeModel.m_treeGrowthSettings.m_fruitDevelopmentalVigorFillingRate == 1.0f)
				{
					ImGui::SliderFloat("Node development", &m_treeModel.m_treeGrowthSettings.m_nodeDevelopmentalVigorFillingRate, 0.0f, 1.0f);
				}

				if(ImGui::Button("Reset"))
				{
					m_treeModel.m_treeGrowthSettings.m_leafMaintenanceVigorFillingRate
						= m_treeModel.m_treeGrowthSettings.m_leafDevelopmentalVigorFillingRate
						= m_treeModel.m_treeGrowthSettings.m_fruitMaintenanceVigorFillingRate
						= m_treeModel.m_treeGrowthSettings.m_fruitDevelopmentalVigorFillingRate
						= m_treeModel.m_treeGrowthSettings.m_nodeDevelopmentalVigorFillingRate = 1.0f;
				}

				ImGui::TreePop();
			}
		}

		static int iterations = 5;
		ImGui::DragInt("Iterations", &iterations, 1, 0, m_treeModel.CurrentIteration());
		iterations = glm::clamp(iterations, 0, m_treeModel.CurrentIteration());

		if (ImGui::TreeNodeEx("Mesh generation", ImGuiTreeNodeFlags_DefaultOpen)) {
			m_meshGeneratorSettings.OnInspect();
			if (ImGui::Button("Generate Mesh")) {
				GenerateMeshes(m_meshGeneratorSettings, iterations);
			}
			if (ImGui::Button("Clear Mesh"))
			{
				ClearMeshes();
			}
			ImGui::TreePop();
		}
		if (ImGui::Button("Clear")) {
			m_treeModel.Clear();
			modelChanged = true;
		}

		if (m_enableHistory)
		{
			if (ImGui::Button("Temporal Progression"))
			{
				m_temporalProgression = true;
				m_temporalProgressionIteration = 0;
			}
		}
	}
	ImGui::Checkbox("Split root test", &m_splitRootTest);
	ImGui::Checkbox("Biomass history", &m_recordBiomassHistory);

	if (m_splitRootTest) ImGui::Text(("Left/Right side biomass: [" + std::to_string(m_leftSideBiomass) + ", " + std::to_string(m_rightSideBiomass) + "]").c_str());

}
void Tree::Update()
{
	if (m_temporalProgression) {
		if (m_temporalProgressionIteration <= m_treeModel.CurrentIteration()) {
			GenerateMeshes(m_meshGeneratorSettings, m_temporalProgressionIteration);
			m_temporalProgressionIteration++;
		}
		else
		{
			m_temporalProgressionIteration = 0;
			m_temporalProgression = false;
		}
	}
}
void Tree::OnCreate() {
}



void Tree::OnDestroy() {
	m_treeModel.Clear();
	m_treeDescriptor.Clear();
	m_soil.Clear();
	m_climate.Clear();
	m_enableHistory = false;

	m_leftSideBiomass = m_rightSideBiomass = 0.0f;
	m_rootBiomassHistory.clear();
	m_shootBiomassHistory.clear();
}

bool Tree::TryGrow(float deltaTime) {
	const auto scene = GetScene();
	const auto treeDescriptor = m_treeDescriptor.Get<TreeDescriptor>();
	if (!treeDescriptor) {
		UNIENGINE_ERROR("No tree descriptor!");
		return false;
	}
	if (!m_soil.Get<Soil>()) {
		UNIENGINE_ERROR("No soil model!");
		return false;
	}
	if (!m_climate.Get<Climate>()) {
		UNIENGINE_ERROR("No climate model!");
		return false;
	}
	const auto soil = m_soil.Get<Soil>();
	const auto climate = m_climate.Get<Climate>();
	const auto owner = GetOwner();
	bool grown = m_treeModel.Grow(deltaTime, scene->GetDataComponent<GlobalTransform>(owner).m_value, soil->m_soilModel, climate->m_climateModel,
		treeDescriptor->m_rootGrowthParameters, treeDescriptor->m_shootGrowthParameters);

	if (m_enableHistory && m_treeModel.m_iteration % m_historyIteration == 0) m_treeModel.Step();

	if (m_recordBiomassHistory)
	{
		const auto& baseRootNode = m_treeModel.RefRootSkeleton().RefNode(0);
		const auto& baseShootNode = m_treeModel.RefShootSkeleton().RefNode(0);
		m_rootBiomassHistory.emplace_back(baseRootNode.m_data.m_biomass + baseRootNode.m_data.m_descendentTotalBiomass);
		m_shootBiomassHistory.emplace_back(baseShootNode.m_data.m_biomass + baseShootNode.m_data.m_descendentTotalBiomass);
	}

	if (m_splitRootTest)
	{
		const auto& rootNodeList = m_treeModel.RefRootSkeleton().RefSortedNodeList();
		m_leftSideBiomass = m_rightSideBiomass = 0.0f;
		for (const auto& rootNodeHandle : rootNodeList)
		{
			const auto& rootNode = m_treeModel.RefRootSkeleton().RefNode(rootNodeHandle);
			if (rootNode.m_info.m_globalPosition.x < 0.0f)
			{
				m_leftSideBiomass += rootNode.m_data.m_biomass;
			}
			else
			{
				m_rightSideBiomass += rootNode.m_data.m_biomass;
			}
		}
	}

	return grown;
}

void Tree::InitializeStrandRenderer() const
{
	if(!m_treeModel.m_enablePipe)
	{
		UNIENGINE_ERROR("Pipe not enabled!");
		return;
	}
	const auto scene = GetScene();
	const auto owner = GetOwner();
	const auto renderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(owner).lock();
	const auto strandsAsset = ProjectManager::CreateTemporaryAsset<Strands>();
	std::vector<glm::uint> strandsList;
	std::vector<StrandPoint> points;
	BuildStrands(m_treeModel.m_shootSkeleton.m_data.m_pipeGroup, strandsList, points);
	BuildStrands(m_treeModel.m_rootSkeleton.m_data.m_pipeGroup, strandsList, points);
	if (!points.empty()) strandsList.emplace_back(points.size());
	strandsAsset->SetStrands(static_cast<unsigned>(StrandPointAttribute::Position) | static_cast<unsigned>(StrandPointAttribute::Thickness) | static_cast<unsigned>(StrandPointAttribute::Color), strandsList, points);
	renderer->m_strands = strandsAsset;

	auto material = ProjectManager::CreateTemporaryAsset<Material>();
	renderer->m_material = material;
	material->m_vertexColorOnly = true;

}

void Tree::Serialize(YAML::Emitter& out)
{
	m_treeDescriptor.Save("m_treeDescriptor", out);
}



void Tree::Deserialize(const YAML::Node& in)
{
	m_treeDescriptor.Load("m_treeDescriptor", in);
}

void Tree::ClearMeshes()
{
	const auto scene = GetScene();
	const auto self = GetOwner();
	const auto children = scene->GetChildren(self);
	for (const auto& child : children) {
		auto name = scene->GetEntityName(child);
		if (name == "Branch Mesh") {
			scene->DeleteEntity(child);
		}
		else if (name == "Root Mesh") {
			scene->DeleteEntity(child);
		}
		else if (name == "Foliage Mesh") {
			scene->DeleteEntity(child);
		}
		else if (name == "Fruit Mesh") {
			scene->DeleteEntity(child);
		}
		else if (name == "Fine Root Mesh") {
			scene->DeleteEntity(child);
		}
	}
}

void Tree::GenerateMeshes(const TreeMeshGeneratorSettings& meshGeneratorSettings, int iteration) {
	const auto scene = GetScene();
	const auto self = GetOwner();
	const auto children = scene->GetChildren(self);

	ClearMeshes();

	auto actualIteration = iteration;
	if (actualIteration < 0 || actualIteration > m_treeModel.CurrentIteration())
	{
		actualIteration = m_treeModel.CurrentIteration();
	}
	if (meshGeneratorSettings.m_enableBranch)
	{
		Entity branchEntity;
		branchEntity = scene->CreateEntity("Branch Mesh");
		scene->SetParent(branchEntity, self);
		std::vector<Vertex> vertices;
		std::vector<unsigned int> indices;
		switch (meshGeneratorSettings.m_branchMeshType)
		{
		case 0:
		{
			CylindricalMeshGenerator<ShootGrowthData, ShootStemGrowthData, InternodeGrowthData> meshGenerator;
			meshGenerator.Generate(m_treeModel.PeekShootSkeleton(actualIteration), vertices, indices,
				meshGeneratorSettings, glm::min(m_treeModel.PeekShootSkeleton(actualIteration).PeekNode(0).m_info.m_thickness,
					m_treeModel.PeekRootSkeleton(actualIteration).PeekNode(0).m_info.m_thickness));
		}
		break;
		case 1:
		{
			const auto treeDescriptor = m_treeDescriptor.Get<TreeDescriptor>();
			float minRadius = 0.01f;
			if (treeDescriptor)
			{
				minRadius = treeDescriptor->m_shootGrowthParameters.m_endNodeThickness;
			}
			VoxelMeshGenerator<ShootGrowthData, ShootStemGrowthData, InternodeGrowthData> meshGenerator;
			meshGenerator.Generate(m_treeModel.PeekShootSkeleton(actualIteration), vertices, indices,
				meshGeneratorSettings, minRadius);
		}
		break;
		default: break;
		}


		auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
		auto material = ProjectManager::CreateTemporaryAsset<Material>();
		material->SetProgram(DefaultResources::GLPrograms::StandardProgram);
		mesh->SetVertices(17, vertices, indices);
		auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(branchEntity).lock();
		if (meshGeneratorSettings.m_overridePresentation)
		{
			material->m_materialProperties.m_albedoColor = meshGeneratorSettings.m_presentationOverrideSettings.m_branchOverrideColor;
		}
		else {
			material->m_materialProperties.m_albedoColor = glm::vec3(109, 79, 75) / 255.0f;
		}
		material->m_materialProperties.m_roughness = 1.0f;
		material->m_materialProperties.m_metallic = 0.0f;
		meshRenderer->m_mesh = mesh;
		meshRenderer->m_material = material;
	}
	if (meshGeneratorSettings.m_enableRoot)
	{
		Entity rootEntity;
		rootEntity = scene->CreateEntity("Root Mesh");
		scene->SetParent(rootEntity, self);
		std::vector<Vertex> vertices;
		std::vector<unsigned int> indices;
		switch (meshGeneratorSettings.m_rootMeshType)
		{
		case 0:
		{
			CylindricalMeshGenerator<RootGrowthData, RootStemGrowthData, RootNodeGrowthData> meshGenerator;
			meshGenerator.Generate(m_treeModel.PeekRootSkeleton(actualIteration), vertices, indices,
				meshGeneratorSettings, glm::min(m_treeModel.PeekShootSkeleton(actualIteration).PeekNode(0).m_info.m_thickness,
					m_treeModel.PeekRootSkeleton(actualIteration).PeekNode(0).m_info.m_thickness));
		}
		break;
		case 1:
		{
			const auto treeDescriptor = m_treeDescriptor.Get<TreeDescriptor>();
			float minRadius = 0.01f;
			if (treeDescriptor)
			{
				minRadius = treeDescriptor->m_rootGrowthParameters.m_endNodeThickness;
			}
			VoxelMeshGenerator<RootGrowthData, RootStemGrowthData, RootNodeGrowthData> meshGenerator;
			meshGenerator.Generate(m_treeModel.PeekRootSkeleton(actualIteration), vertices, indices,
				meshGeneratorSettings, minRadius);
		}
		break;
		default: break;
		}
		auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
		auto material = ProjectManager::CreateTemporaryAsset<Material>();
		material->SetProgram(DefaultResources::GLPrograms::StandardProgram);
		if (meshGeneratorSettings.m_overridePresentation)
		{
			material->m_materialProperties.m_albedoColor = meshGeneratorSettings.m_presentationOverrideSettings.m_rootOverrideColor;
		}
		else {
			material->m_materialProperties.m_albedoColor = glm::vec3(80, 60, 50) / 255.0f;
		}
		material->m_materialProperties.m_roughness = 1.0f;
		material->m_materialProperties.m_metallic = 0.0f;
		mesh->SetVertices(17, vertices, indices);
		auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(rootEntity).lock();
		meshRenderer->m_mesh = mesh;
		meshRenderer->m_material = material;
	}
	if (meshGeneratorSettings.m_enableFineRoot)
	{
		Entity fineRootEntity;
		fineRootEntity = scene->CreateEntity("Fine Root Mesh");
		scene->SetParent(fineRootEntity, self);
		std::vector<glm::uint> fineRootSegments;
		std::vector<StrandPoint> fineRootPoints;
		const auto& rootSkeleton = m_treeModel.PeekRootSkeleton(actualIteration);
		const auto& rootNodeList = rootSkeleton.RefSortedNodeList();
		int fineRootIndex = 0;
		for (int i = 0; i < rootNodeList.size(); i++)
		{
			const auto& rootNode = rootSkeleton.PeekNode(rootNodeList[i]);
			const auto& rootNodeData = rootNode.m_data;
			const auto& rootNodeInfo = rootNode.m_info;
			if (rootNodeData.m_fineRootAnchors.empty()) continue;
			fineRootPoints.resize(fineRootPoints.size() + 8);
			fineRootSegments.resize(fineRootSegments.size() + 5);
			auto& p0 = fineRootPoints[fineRootIndex * 8];
			auto& p1 = fineRootPoints[fineRootIndex * 8 + 1];
			auto& p2 = fineRootPoints[fineRootIndex * 8 + 2];
			auto& p3 = fineRootPoints[fineRootIndex * 8 + 3];
			auto& p4 = fineRootPoints[fineRootIndex * 8 + 4];
			auto& p5 = fineRootPoints[fineRootIndex * 8 + 5];
			auto& p6 = fineRootPoints[fineRootIndex * 8 + 6];
			auto& p7 = fineRootPoints[fineRootIndex * 8 + 7];

			p0.m_position = rootNodeInfo.m_globalPosition * 2.0f - glm::vec3(rootNodeData.m_fineRootAnchors[0]);
			p1.m_position = rootNodeInfo.m_globalPosition;
			p2.m_position = glm::vec3(rootNodeData.m_fineRootAnchors[0]);
			p3.m_position = glm::vec3(rootNodeData.m_fineRootAnchors[1]);
			p4.m_position = glm::vec3(rootNodeData.m_fineRootAnchors[2]);
			p5.m_position = glm::vec3(rootNodeData.m_fineRootAnchors[3]);
			p6.m_position = glm::vec3(rootNodeData.m_fineRootAnchors[4]);
			p7.m_position = glm::vec3(rootNodeData.m_fineRootAnchors[4]) * 2.0f - glm::vec3(rootNodeData.m_fineRootAnchors[3]);

			p0.m_thickness = rootNodeInfo.m_thickness;
			p1.m_thickness = rootNodeData.m_fineRootAnchors[0].w;
			p2.m_thickness = rootNodeData.m_fineRootAnchors[0].w;
			p3.m_thickness = rootNodeData.m_fineRootAnchors[1].w;
			p4.m_thickness = rootNodeData.m_fineRootAnchors[2].w;
			p5.m_thickness = rootNodeData.m_fineRootAnchors[3].w;
			p6.m_thickness = rootNodeData.m_fineRootAnchors[4].w;
			p7.m_thickness = rootNodeData.m_fineRootAnchors[4].w;

			fineRootSegments[fineRootIndex * 5] = fineRootIndex * 8;
			fineRootSegments[fineRootIndex * 5 + 1] = fineRootIndex * 8 + 1;
			fineRootSegments[fineRootIndex * 5 + 2] = fineRootIndex * 8 + 2;
			fineRootSegments[fineRootIndex * 5 + 3] = fineRootIndex * 8 + 3;
			fineRootSegments[fineRootIndex * 5 + 4] = fineRootIndex * 8 + 4;

			fineRootIndex++;
		}

		auto strands = ProjectManager::CreateTemporaryAsset<Strands>();
		auto material = ProjectManager::CreateTemporaryAsset<Material>();
		material->SetProgram(DefaultResources::GLPrograms::StandardProgram);
		if (meshGeneratorSettings.m_overridePresentation)
		{
			material->m_materialProperties.m_albedoColor = meshGeneratorSettings.m_presentationOverrideSettings.m_rootOverrideColor;
		}
		else {
			material->m_materialProperties.m_albedoColor = glm::vec3(80, 60, 50) / 255.0f;
		}
		material->m_materialProperties.m_roughness = 1.0f;
		material->m_materialProperties.m_metallic = 0.0f;
		strands->SetSegments(1, fineRootSegments, fineRootPoints);
		auto strandsRenderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(fineRootEntity).lock();
		strandsRenderer->m_strands = strands;
		strandsRenderer->m_material = material;
	}
	if (meshGeneratorSettings.m_enableFoliage)
	{
		Entity foliageEntity;
		foliageEntity = scene->CreateEntity("Foliage Mesh");
		scene->SetParent(foliageEntity, self);
		if (!meshGeneratorSettings.m_detailedFoliage) {
			std::vector<Vertex> vertices;
			std::vector<unsigned int> indices;
			auto quadMesh = DefaultResources::Primitives::Quad;
			auto& quadTriangles = quadMesh->UnsafeGetTriangles();
			auto quadVerticesSize = quadMesh->GetVerticesAmount();
			size_t offset = 0;

			const auto& nodeList = m_treeModel.PeekShootSkeleton(actualIteration).RefSortedNodeList();
			for (const auto& internodeHandle : nodeList) {
				const auto& internode = m_treeModel.PeekShootSkeleton(actualIteration).PeekNode(internodeHandle);
				const auto& internodeInfo = internode.m_info;
				const auto& internodeData = internode.m_data;
				if (!meshGeneratorSettings.m_overridePresentation) {
					for (const auto& bud : internodeData.m_buds) {
						if (bud.m_status != BudStatus::Flushed) continue;
						if (bud.m_reproductiveModule.m_maturity <= 0.0f) continue;
						if (bud.m_type == BudType::Leaf)
						{
							auto& matrix = bud.m_reproductiveModule.m_transform;
							Vertex archetype;
							for (auto i = 0; i < quadMesh->GetVerticesAmount(); i++) {
								archetype.m_position =
									matrix * glm::vec4(quadMesh->UnsafeGetVertices()[i].m_position, 1.0f);
								archetype.m_normal = glm::normalize(glm::vec3(
									matrix * glm::vec4(quadMesh->UnsafeGetVertices()[i].m_normal, 0.0f)));
								archetype.m_tangent = glm::normalize(glm::vec3(
									matrix *
									glm::vec4(quadMesh->UnsafeGetVertices()[i].m_tangent, 0.0f)));
								archetype.m_texCoord =
									quadMesh->UnsafeGetVertices()[i].m_texCoord;
								vertices.push_back(archetype);
							}
							for (auto triangle : quadTriangles) {
								triangle.x += offset;
								triangle.y += offset;
								triangle.z += offset;
								indices.push_back(triangle.x);
								indices.push_back(triangle.y);
								indices.push_back(triangle.z);
							}
							offset += quadVerticesSize;
						}
					}
				}
				else
				{
					const auto& presentationSettings = meshGeneratorSettings.m_presentationOverrideSettings;
					if (internodeData.m_maxDistanceToAnyBranchEnd < presentationSettings.m_distanceToEndLimit) {
						for (int i = 0; i < presentationSettings.m_leafCountPerInternode; i++)
						{
							auto leafSize = presentationSettings.m_leafSize;
							glm::quat rotation = internodeInfo.m_globalRotation * glm::quat(glm::radians(glm::linearRand(glm::vec3(0.0f), glm::vec3(360.0f))));
							auto front = rotation * glm::vec3(0, 0, -1);
							TreeModel::ApplyTropism(internodeData.m_lightDirection, presentationSettings.m_phototropism, rotation);
							auto foliagePosition = front * (leafSize.z * 1.5f);
							auto leafTransform = glm::translate(foliagePosition) * glm::mat4_cast(rotation) * glm::scale(leafSize);

							auto& matrix = leafTransform;
							Vertex archetype;
							for (auto i = 0; i < quadMesh->GetVerticesAmount(); i++) {
								archetype.m_position =
									matrix * glm::vec4(quadMesh->UnsafeGetVertices()[i].m_position, 1.0f);
								archetype.m_normal = glm::normalize(glm::vec3(
									matrix * glm::vec4(quadMesh->UnsafeGetVertices()[i].m_normal, 0.0f)));
								archetype.m_tangent = glm::normalize(glm::vec3(
									matrix *
									glm::vec4(quadMesh->UnsafeGetVertices()[i].m_tangent, 0.0f)));
								archetype.m_texCoord =
									quadMesh->UnsafeGetVertices()[i].m_texCoord;
								vertices.push_back(archetype);
							}
							for (auto triangle : quadTriangles) {
								triangle.x += offset;
								triangle.y += offset;
								triangle.z += offset;
								indices.push_back(triangle.x);
								indices.push_back(triangle.y);
								indices.push_back(triangle.z);
							}
							offset += quadVerticesSize;
						}
					}
				}
			}

			auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
			auto material = ProjectManager::CreateTemporaryAsset<Material>();
			material->SetProgram(DefaultResources::GLPrograms::StandardProgram);
			mesh->SetVertices(17, vertices, indices);
			if (meshGeneratorSettings.m_overridePresentation)
			{
				material->m_materialProperties.m_albedoColor = meshGeneratorSettings.m_presentationOverrideSettings.m_foliageOverrideColor;
			}
			else {
				material->m_materialProperties.m_albedoColor = glm::vec3(152 / 255.0f, 203 / 255.0f, 0 / 255.0f);
			}
			material->m_materialProperties.m_roughness = 0.0f;
			auto texRef = meshGeneratorSettings.m_foliageTexture;
			if (texRef.Get<Texture2D>())
			{
				material->m_albedoTexture = texRef.Get<Texture2D>();

			}
			auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(foliageEntity).lock();
			meshRenderer->m_mesh = mesh;
			meshRenderer->m_material = material;
		}else
		{
			for(int i = 0; i < 8; i++)
			{
				float maxHealth = i / 8.0f;
				const auto& nodeList = m_treeModel.PeekShootSkeleton(actualIteration).RefSortedNodeList();
				for (const auto& internodeHandle : nodeList) {
					const auto& internode = m_treeModel.PeekShootSkeleton(actualIteration).PeekNode(internodeHandle);
					const auto& internodeInfo = internode.m_info;
					const auto& internodeData = internode.m_data;
					if (!meshGeneratorSettings.m_overridePresentation) {
						for (const auto& bud : internodeData.m_buds) {
							if (bud.m_status != BudStatus::Flushed) continue;
							if (bud.m_reproductiveModule.m_maturity <= 0.0f) continue;
							if (bud.m_type == BudType::Leaf)
							{
								auto matrix = bud.m_reproductiveModule.m_transform;
								Vertex archetype;
								
							}
						}
					}
				}
			}
		}
	}
	if (meshGeneratorSettings.m_enableFruit)
	{
		Entity fruitEntity;
		fruitEntity = scene->CreateEntity("Fruit Mesh");
		scene->SetParent(fruitEntity, self);
		std::vector<Vertex> vertices;
		std::vector<unsigned int> indices;
		auto fruitMesh = DefaultResources::Primitives::Sphere;
		auto& fruitTriangles = fruitMesh->UnsafeGetTriangles();
		auto fruitVerticesSize = fruitMesh->GetVerticesAmount();
		size_t offset = 0;

		const auto& nodeList = m_treeModel.PeekShootSkeleton(actualIteration).RefSortedNodeList();
		for (const auto& internodeHandle : nodeList) {
			const auto& internode = m_treeModel.PeekShootSkeleton(actualIteration).PeekNode(internodeHandle);
			const auto& internodeInfo = internode.m_info;
			const auto& internodeData = internode.m_data;
			if (!meshGeneratorSettings.m_overridePresentation) {
				for (const auto& bud : internodeData.m_buds) {
					if (bud.m_status != BudStatus::Flushed) continue;
					if (bud.m_reproductiveModule.m_maturity <= 0.0f) continue;
					if (bud.m_type == BudType::Fruit)
					{
						auto matrix = bud.m_reproductiveModule.m_transform;
						Vertex archetype;
						for (auto i = 0; i < fruitMesh->GetVerticesAmount(); i++) {
							archetype.m_position =
								matrix * glm::vec4(fruitMesh->UnsafeGetVertices()[i].m_position, 1.0f);
							archetype.m_normal = glm::normalize(glm::vec3(
								matrix * glm::vec4(fruitMesh->UnsafeGetVertices()[i].m_normal, 0.0f)));
							archetype.m_tangent = glm::normalize(glm::vec3(
								matrix *
								glm::vec4(fruitMesh->UnsafeGetVertices()[i].m_tangent, 0.0f)));
							archetype.m_texCoord =
								fruitMesh->UnsafeGetVertices()[i].m_texCoord;
							vertices.push_back(archetype);
						}
						for (auto triangle : fruitTriangles) {
							triangle.x += offset;
							triangle.y += offset;
							triangle.z += offset;
							indices.push_back(triangle.x);
							indices.push_back(triangle.y);
							indices.push_back(triangle.z);
						}
						offset += fruitVerticesSize;
					}
				}
			}
		}

		auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
		auto material = ProjectManager::CreateTemporaryAsset<Material>();
		material->SetProgram(DefaultResources::GLPrograms::StandardProgram);
		mesh->SetVertices(17, vertices, indices);
		if (meshGeneratorSettings.m_overridePresentation)
		{
			material->m_materialProperties.m_albedoColor = meshGeneratorSettings.m_presentationOverrideSettings.m_foliageOverrideColor;
		}
		else {
			material->m_materialProperties.m_albedoColor = glm::vec3(152 / 255.0f, 203 / 255.0f, 0 / 255.0f);
		}
		material->m_materialProperties.m_roughness = 0.0f;
		auto texRef = meshGeneratorSettings.m_foliageTexture;
		if (texRef.Get<Texture2D>())
		{
			material->m_albedoTexture = texRef.Get<Texture2D>();
		}
		auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(fruitEntity).lock();
		meshRenderer->m_mesh = mesh;
		meshRenderer->m_material = material;
	}
}

void Tree::FromLSystemString(const std::shared_ptr<LSystemString>& lSystemString)
{
}

void Tree::FromTreeGraph(const std::shared_ptr<TreeGraph>& treeGraph)
{
}

void Tree::FromTreeGraphV2(const std::shared_ptr<TreeGraphV2>& treeGraphV2)
{
}

void TreeDescriptor::OnCreate() {

}


bool OnInspectShootGrowthParameters(ShootGrowthParameters& treeGrowthParameters) {
	bool changed = false;
	if (ImGui::TreeNodeEx("Shoot Parameters", ImGuiTreeNodeFlags_DefaultOpen)) {
		changed = ImGui::DragFloat("Internode growth rate", &treeGrowthParameters.m_internodeGrowthRate, 0.01f, 0.0f, 1.0f) || changed;
		changed = ImGui::DragFloat("Leaf growth rate", &treeGrowthParameters.m_leafGrowthRate, 0.01f, 0.0f, 1.0f) || changed;
		changed = ImGui::DragFloat("Fruit growth rate", &treeGrowthParameters.m_fruitGrowthRate, 0.01f, 0.0f, 1.0f) || changed;

		if (ImGui::TreeNodeEx("Bud", ImGuiTreeNodeFlags_DefaultOpen)) {
			changed = ImGui::DragInt3("Bud count lateral/fruit/leaf", &treeGrowthParameters.m_lateralBudCount, 1, 0, 3) || changed;
			changed = ImGui::DragFloat2("Branching Angle mean/var", &treeGrowthParameters.m_branchingAngleMeanVariance.x, 0.01f, 0.0f, 100.0f) || changed;
			changed = ImGui::DragFloat2("Roll Angle mean/var", &treeGrowthParameters.m_rollAngleMeanVariance.x, 0.01f, 0.0f, 100.0f) || changed;
			changed = ImGui::DragFloat2("Apical Angle mean/var", &treeGrowthParameters.m_apicalAngleMeanVariance.x, 0.01f, 0.0f, 100.0f) || changed;
			changed = ImGui::DragFloat("Gravitropism", &treeGrowthParameters.m_gravitropism, 0.01f) || changed;
			changed = ImGui::DragFloat("Phototropism", &treeGrowthParameters.m_phototropism, 0.01f) || changed;

			changed = ImGui::DragFloat4("Lateral bud flushing prob/temp range", &treeGrowthParameters.m_lateralBudFlushingProbabilityTemperatureRange.x, 0.00001f, 0.0f, 1.0f, "%.5f") || changed;
			changed = ImGui::DragFloat4("Leaf flushing prob/temp range", &treeGrowthParameters.m_leafBudFlushingProbabilityTemperatureRange.x, 0.00001f, 0.0f, 1.0f, "%.5f") || changed;
			changed = ImGui::DragFloat4("Fruit flushing prob/temp range", &treeGrowthParameters.m_fruitBudFlushingProbabilityTemperatureRange.x, 0.00001f, 0.0f, 1.0f, "%.5f") || changed;
			changed = ImGui::DragFloat4("Bud light factor apical/lateral/leaf/fruit", &treeGrowthParameters.m_apicalBudLightingFactor, 0.01f) || changed;
			changed = ImGui::DragFloat2("Apical control base/age", &treeGrowthParameters.m_apicalControl, 0.01f) || changed;
			changed = ImGui::DragFloat3("Apical dominance base/age/dist", &treeGrowthParameters.m_apicalDominance, 0.01f) || changed;
			changed = ImGui::DragFloat4("Remove rate apical/lateral/leaf/fruit", &treeGrowthParameters.m_apicalBudExtinctionRate, 0.01f) || changed;
			changed = ImGui::DragFloat3("Vigor requirement shoot/leaf/fruit", &treeGrowthParameters.m_internodeVigorRequirement, 0.01f) || changed;
			changed = ImGui::DragFloat("Vigor requirement aggregation loss", &treeGrowthParameters.m_vigorRequirementAggregateLoss, 0.001f, 0.0f, 1.0f) || changed;

			ImGui::TreePop();
		}
		if (ImGui::TreeNodeEx("Internode", ImGuiTreeNodeFlags_DefaultOpen))
		{
			changed = ImGui::DragFloat("Internode length", &treeGrowthParameters.m_internodeLength, 0.01f) || changed;
			changed = ImGui::DragFloat3("Thickness min/factor/age", &treeGrowthParameters.m_endNodeThickness, 0.00001f, 0.0f, 1.0f, "%.6f") || changed;
			changed = ImGui::DragFloat3("Sagging thickness/reduction/max", &treeGrowthParameters.m_saggingFactorThicknessReductionMax.x, 0.01f, 0.0f, 1.0f, "%.5f") || changed;
			changed = ImGui::DragFloat("Low Branch Pruning", &treeGrowthParameters.m_lowBranchPruning, 0.01f) || changed;
			changed = ImGui::DragFloat("Light pruning factor", &treeGrowthParameters.m_endNodePruningLightFactor, 0.01f) || changed;
			ImGui::TreePop();
		}

		if (ImGui::TreeNodeEx("Leaf", ImGuiTreeNodeFlags_DefaultOpen))
		{
			changed = ImGui::DragFloat3("Size", &treeGrowthParameters.m_maxLeafSize.x, 0.01f) || changed;
			changed = ImGui::DragFloat("Position Variance", &treeGrowthParameters.m_leafPositionVariance, 0.01f) || changed;
			changed = ImGui::DragFloat("Random rotation", &treeGrowthParameters.m_leafRandomRotation, 0.01f) || changed;
			changed = ImGui::DragFloat("Chlorophyll Loss", &treeGrowthParameters.m_leafChlorophyllLoss, 0.01f) || changed;
			changed = ImGui::DragFloat("Chlorophyll temperature", &treeGrowthParameters.m_leafChlorophyllSynthesisFactorTemperature, 0.01f) || changed;
			changed = ImGui::DragFloat("Drop prob", &treeGrowthParameters.m_leafFallProbability, 0.01f) || changed;
			changed = ImGui::DragFloat("Distance To End Limit", &treeGrowthParameters.m_leafDistanceToBranchEndLimit, 0.01f) || changed;
			ImGui::TreePop();
		}
		if (ImGui::TreeNodeEx("Fruit", ImGuiTreeNodeFlags_DefaultOpen))
		{
			changed = ImGui::DragFloat3("Size", &treeGrowthParameters.m_maxFruitSize.x, 0.01f) || changed;
			changed = ImGui::DragFloat("Position Variance", &treeGrowthParameters.m_fruitPositionVariance, 0.01f) || changed;
			changed = ImGui::DragFloat("Random rotation", &treeGrowthParameters.m_fruitRandomRotation, 0.01f) || changed;

			changed = ImGui::DragFloat("Drop prob", &treeGrowthParameters.m_fruitFallProbability, 0.01f) || changed;
			ImGui::TreePop();
		}
		ImGui::TreePop();
	}

	return changed;
}

bool OnInspectRootGrowthParameters(RootGrowthParameters& rootGrowthParameters) {
	bool changed = false;
	if (ImGui::TreeNodeEx("Root Parameters", ImGuiTreeNodeFlags_DefaultOpen)) {
		if (ImGui::TreeNodeEx("Structure", ImGuiTreeNodeFlags_DefaultOpen)) {
			changed = ImGui::DragFloat("Root node length", &rootGrowthParameters.m_rootNodeLength, 0.01f) || changed;
			changed = ImGui::DragFloat("Root node elongation rate", &rootGrowthParameters.m_rootNodeGrowthRate, 0.01f) || changed;

			changed = ImGui::DragFloat3("Thickness min/factor/age", &rootGrowthParameters.m_endNodeThickness, 0.00001f, 0.0f, 1.0f, "%.6f") || changed;

			ImGui::TreePop();
		}
		if (ImGui::TreeNodeEx("Growth", ImGuiTreeNodeFlags_DefaultOpen))
		{
			changed = ImGui::DragFloat("Root node elongation vigor requirement", &rootGrowthParameters.m_rootNodeVigorRequirement, 0.01f) || changed;
			changed = ImGui::DragFloat("Vigor requirement aggregation loss", &rootGrowthParameters.m_vigorRequirementAggregateLoss, 0.001f, 0.0f, 1.0f) || changed;

			changed = ImGui::DragFloat2("Branching Angle mean/var", &rootGrowthParameters.m_branchingAngleMeanVariance.x, 0.01f) || changed;
			changed = ImGui::DragFloat2("Roll Angle mean/var", &rootGrowthParameters.m_rollAngleMeanVariance.x, 0.01f) || changed;
			changed = ImGui::DragFloat2("Apical Angle mean/var", &rootGrowthParameters.m_apicalAngleMeanVariance.x, 0.01f) || changed;
			changed = ImGui::DragFloat2("Environmental friction base/factor", &rootGrowthParameters.m_environmentalFriction, 0.01f) || changed;
			changed = ImGui::DragFloat2("Apical control base/age", &rootGrowthParameters.m_apicalControl, 0.01f) || changed;
			changed = ImGui::DragFloat3("Apical dominance base/age/dist", &rootGrowthParameters.m_apicalDominance, 0.01f) || changed;
			changed = ImGui::DragFloat("Tropism switching prob", &rootGrowthParameters.m_tropismSwitchingProbability, 0.01f) || changed;
			changed = ImGui::DragFloat("Tropism switching prob dist factor", &rootGrowthParameters.m_tropismSwitchingProbabilityDistanceFactor, 0.01f) || changed;
			changed = ImGui::DragFloat("Tropism intensity", &rootGrowthParameters.m_tropismIntensity, 0.01f) || changed;
			changed = ImGui::DragFloat("Branching probability", &rootGrowthParameters.m_branchingProbability, 0.00001f, 0.0f, 1.0f, "%.5f") || changed;
			changed = ImGui::DragFloat("Fine root segment length", &rootGrowthParameters.m_fineRootSegmentLength, 0.01f) || changed;
			changed = ImGui::DragFloat("Fine root apical angle variance", &rootGrowthParameters.m_fineRootApicalAngleVariance, 0.01f) || changed;
			changed = ImGui::DragFloat("Fine root branching angle", &rootGrowthParameters.m_fineRootBranchingAngle, 0.01f) || changed;
			changed = ImGui::DragFloat("Fine root thickness", &rootGrowthParameters.m_fineRootThickness, 0.01f) || changed;
			changed = ImGui::DragFloat("Fine root min node thickness", &rootGrowthParameters.m_fineRootMinNodeThickness) || changed;
			changed = ImGui::DragInt("Fine root node count", &rootGrowthParameters.m_fineRootNodeCount) || changed;
			ImGui::TreePop();
		}
		ImGui::TreePop();
	}
	return changed;
}

void TreeDescriptor::OnInspect() {
	bool changed = false;
	const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
	const auto soil = ecoSysLabLayer->m_soilHolder.Get<Soil>();
	const auto climate = ecoSysLabLayer->m_climateHolder.Get<Climate>();
	if (soil && climate) {
		if (ImGui::Button("Instantiate")) {
			const auto scene = Application::GetActiveScene();
			const auto treeEntity = scene->CreateEntity(GetTitle());
			const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
			float height = 0;
			const auto soilDescriptor = soil->m_soilDescriptor.Get<SoilDescriptor>();
			if (soilDescriptor)
			{
				const auto heightField = soilDescriptor->m_heightField.Get<HeightField>();
				if (heightField) height = heightField->GetValue({ 0.0f, 0.0f }) - 0.05f;
			}
			GlobalTransform globalTransform;
			globalTransform.SetPosition(glm::vec3(0, height, 0));
			scene->SetDataComponent(treeEntity, globalTransform);
			tree->m_treeDescriptor = ProjectManager::GetAsset(GetHandle());
			tree->m_treeModel.RefShootSkeleton().m_data.m_treeIlluminationEstimator.m_settings = ecoSysLabLayer->m_shadowEstimationSettings;
		}
	}
	else
	{
		ImGui::Text("Attach soil and climate entity to instantiate!");
	}
	if (OnInspectShootGrowthParameters(m_shootGrowthParameters)) { changed = true; }
	if (OnInspectRootGrowthParameters(m_rootGrowthParameters)) { changed = true; }
	if (changed) m_saved = false;
}

void TreeDescriptor::CollectAssetRef(std::vector<AssetRef>& list) {

}

void SerializeShootGrowthParameters(const std::string& name, const ShootGrowthParameters& treeGrowthParameters, YAML::Emitter& out) {
	out << YAML::Key << name << YAML::BeginMap;
	out << YAML::Key << "m_internodeGrowthRate" << YAML::Value << treeGrowthParameters.m_internodeGrowthRate;
	out << YAML::Key << "m_leafGrowthRate" << YAML::Value << treeGrowthParameters.m_leafGrowthRate;
	out << YAML::Key << "m_fruitGrowthRate" << YAML::Value << treeGrowthParameters.m_fruitGrowthRate;
	//Structure
	out << YAML::Key << "m_lateralBudCount" << YAML::Value << treeGrowthParameters.m_lateralBudCount;
	out << YAML::Key << "m_fruitBudCount" << YAML::Value << treeGrowthParameters.m_fruitBudCount;
	out << YAML::Key << "m_leafBudCount" << YAML::Value << treeGrowthParameters.m_leafBudCount;
	out << YAML::Key << "m_branchingAngleMeanVariance" << YAML::Value << treeGrowthParameters.m_branchingAngleMeanVariance;
	out << YAML::Key << "m_rollAngleMeanVariance" << YAML::Value << treeGrowthParameters.m_rollAngleMeanVariance;
	out << YAML::Key << "m_apicalAngleMeanVariance" << YAML::Value << treeGrowthParameters.m_apicalAngleMeanVariance;
	out << YAML::Key << "m_gravitropism" << YAML::Value << treeGrowthParameters.m_gravitropism;
	out << YAML::Key << "m_phototropism" << YAML::Value << treeGrowthParameters.m_phototropism;
	out << YAML::Key << "m_internodeLength" << YAML::Value << treeGrowthParameters.m_internodeLength;

	out << YAML::Key << "m_endNodeThickness" << YAML::Value << treeGrowthParameters.m_endNodeThickness;
	out << YAML::Key << "m_thicknessAccumulationFactor" << YAML::Value << treeGrowthParameters.m_thicknessAccumulationFactor;
	out << YAML::Key << "m_thicknessAccumulateAgeFactor" << YAML::Value << treeGrowthParameters.m_thicknessAccumulateAgeFactor;

	//Bud
	out << YAML::Key << "m_lateralBudFlushingProbabilityTemperatureRange" << YAML::Value << treeGrowthParameters.m_lateralBudFlushingProbabilityTemperatureRange;
	out << YAML::Key << "m_leafBudFlushingProbabilityTemperatureRange" << YAML::Value << treeGrowthParameters.m_leafBudFlushingProbabilityTemperatureRange;
	out << YAML::Key << "m_fruitBudFlushingProbabilityTemperatureRange" << YAML::Value << treeGrowthParameters.m_fruitBudFlushingProbabilityTemperatureRange;

	out << YAML::Key << "m_apicalBudLightingFactor" << YAML::Value << treeGrowthParameters.m_apicalBudLightingFactor;
	out << YAML::Key << "m_lateralBudLightingFactor" << YAML::Value << treeGrowthParameters.m_lateralBudLightingFactor;
	out << YAML::Key << "m_leafBudLightingFactor" << YAML::Value << treeGrowthParameters.m_leafBudLightingFactor;
	out << YAML::Key << "m_fruitBudLightingFactor" << YAML::Value << treeGrowthParameters.m_fruitBudLightingFactor;


	out << YAML::Key << "m_apicalControl" << YAML::Value << treeGrowthParameters.m_apicalControl;
	out << YAML::Key << "m_apicalControlAgeFactor" << YAML::Value << treeGrowthParameters.m_apicalControlAgeFactor;

	out << YAML::Key << "m_apicalDominance" << YAML::Value << treeGrowthParameters.m_apicalDominance;
	out << YAML::Key << "m_apicalDominanceAgeFactor" << YAML::Value << treeGrowthParameters.m_apicalDominanceAgeFactor;
	out << YAML::Key << "m_apicalDominanceDistanceFactor" << YAML::Value << treeGrowthParameters.m_apicalDominanceDistanceFactor;

	out << YAML::Key << "m_apicalBudExtinctionRate" << YAML::Value << treeGrowthParameters.m_apicalBudExtinctionRate;
	out << YAML::Key << "m_lateralBudExtinctionRate" << YAML::Value << treeGrowthParameters.m_lateralBudExtinctionRate;
	out << YAML::Key << "m_leafBudExtinctionRate" << YAML::Value << treeGrowthParameters.m_leafBudExtinctionRate;
	out << YAML::Key << "m_fruitBudExtinctionRate" << YAML::Value << treeGrowthParameters.m_fruitBudExtinctionRate;

	out << YAML::Key << "m_leafVigorRequirement" << YAML::Value << treeGrowthParameters.m_leafVigorRequirement;
	out << YAML::Key << "m_fruitVigorRequirement" << YAML::Value << treeGrowthParameters.m_fruitVigorRequirement;
	out << YAML::Key << "m_internodeVigorRequirement" << YAML::Value << treeGrowthParameters.m_internodeVigorRequirement;
	out << YAML::Key << "m_vigorRequirementAggregateLoss" << YAML::Value << treeGrowthParameters.m_vigorRequirementAggregateLoss;
	//Internode
	out << YAML::Key << "m_lowBranchPruning" << YAML::Value << treeGrowthParameters.m_lowBranchPruning;
	out << YAML::Key << "m_saggingFactorThicknessReductionMax" << YAML::Value << treeGrowthParameters.m_saggingFactorThicknessReductionMax;

	//Foliage
	out << YAML::Key << "m_maxLeafSize" << YAML::Value << treeGrowthParameters.m_maxLeafSize;
	out << YAML::Key << "m_leafPositionVariance" << YAML::Value << treeGrowthParameters.m_leafPositionVariance;
	out << YAML::Key << "m_leafRandomRotation" << YAML::Value << treeGrowthParameters.m_leafRandomRotation;
	out << YAML::Key << "m_leafChlorophyllLoss" << YAML::Value << treeGrowthParameters.m_leafChlorophyllLoss;
	out << YAML::Key << "m_leafChlorophyllSynthesisFactorTemperature" << YAML::Value << treeGrowthParameters.m_leafChlorophyllSynthesisFactorTemperature;
	out << YAML::Key << "m_leafFallProbability" << YAML::Value << treeGrowthParameters.m_leafFallProbability;
	out << YAML::Key << "m_leafDistanceToBranchEndLimit" << YAML::Value << treeGrowthParameters.m_leafDistanceToBranchEndLimit;

	out << YAML::Key << "m_maxFruitSize" << YAML::Value << treeGrowthParameters.m_maxFruitSize;
	out << YAML::Key << "m_fruitPositionVariance" << YAML::Value << treeGrowthParameters.m_fruitPositionVariance;
	out << YAML::Key << "m_fruitRandomRotation" << YAML::Value << treeGrowthParameters.m_fruitRandomRotation;
	out << YAML::Key << "m_fruitFallProbability" << YAML::Value << treeGrowthParameters.m_fruitFallProbability;
	out << YAML::EndMap;
}
void SerializeRootGrowthParameters(const std::string& name, const RootGrowthParameters& rootGrowthParameters, YAML::Emitter& out) {
	out << YAML::Key << name << YAML::BeginMap;

	out << YAML::Key << "m_branchingAngleMeanVariance" << YAML::Value << rootGrowthParameters.m_branchingAngleMeanVariance;
	out << YAML::Key << "m_rollAngleMeanVariance" << YAML::Value << rootGrowthParameters.m_rollAngleMeanVariance;
	out << YAML::Key << "m_apicalAngleMeanVariance" << YAML::Value << rootGrowthParameters.m_apicalAngleMeanVariance;
	out << YAML::Key << "m_rootNodeLength" << YAML::Value << rootGrowthParameters.m_rootNodeLength;
	out << YAML::Key << "m_rootNodeGrowthRate" << YAML::Value << rootGrowthParameters.m_rootNodeGrowthRate;
	out << YAML::Key << "m_endNodeThickness" << YAML::Value << rootGrowthParameters.m_endNodeThickness;
	out << YAML::Key << "m_thicknessAccumulationFactor" << YAML::Value << rootGrowthParameters.m_thicknessAccumulationFactor;
	out << YAML::Key << "m_thicknessAccumulateAgeFactor" << YAML::Value << rootGrowthParameters.m_thicknessAccumulateAgeFactor;


	out << YAML::Key << "m_rootNodeVigorRequirement" << YAML::Value << rootGrowthParameters.m_rootNodeVigorRequirement;
	out << YAML::Key << "m_vigorRequirementAggregateLoss" << YAML::Value << rootGrowthParameters.m_vigorRequirementAggregateLoss;
	out << YAML::Key << "m_environmentalFriction" << YAML::Value << rootGrowthParameters.m_environmentalFriction;
	out << YAML::Key << "m_environmentalFrictionFactor" << YAML::Value << rootGrowthParameters.m_environmentalFrictionFactor;


	out << YAML::Key << "m_apicalControl" << YAML::Value << rootGrowthParameters.m_apicalControl;
	out << YAML::Key << "m_apicalControlAgeFactor" << YAML::Value << rootGrowthParameters.m_apicalControlAgeFactor;
	out << YAML::Key << "m_apicalDominance" << YAML::Value << rootGrowthParameters.m_apicalDominance;
	out << YAML::Key << "m_apicalDominanceAgeFactor" << YAML::Value << rootGrowthParameters.m_apicalDominanceAgeFactor;
	out << YAML::Key << "m_apicalDominanceDistanceFactor" << YAML::Value << rootGrowthParameters.m_apicalDominanceDistanceFactor;
	out << YAML::Key << "m_tropismSwitchingProbability" << YAML::Value << rootGrowthParameters.m_tropismSwitchingProbability;
	out << YAML::Key << "m_tropismSwitchingProbabilityDistanceFactor" << YAML::Value << rootGrowthParameters.m_tropismSwitchingProbabilityDistanceFactor;
	out << YAML::Key << "m_tropismIntensity" << YAML::Value << rootGrowthParameters.m_tropismIntensity;
	out << YAML::Key << "m_branchingProbability" << YAML::Value << rootGrowthParameters.m_branchingProbability;

	out << YAML::EndMap;
}
void TreeDescriptor::Serialize(YAML::Emitter& out) {
	SerializeShootGrowthParameters("m_shootGrowthParameters", m_shootGrowthParameters, out);
	SerializeRootGrowthParameters("m_rootGrowthParameters", m_rootGrowthParameters, out);
}

void DeserializeShootGrowthParameters(const std::string& name, ShootGrowthParameters& treeGrowthParameters, const YAML::Node& in) {
	if (in[name]) {
		auto& param = in[name];

		if (param["m_internodeGrowthRate"]) treeGrowthParameters.m_internodeGrowthRate = param["m_internodeGrowthRate"].as<float>();
		if (param["m_leafGrowthRate"]) treeGrowthParameters.m_leafGrowthRate = param["m_leafGrowthRate"].as<float>();
		if (param["m_fruitGrowthRate"]) treeGrowthParameters.m_fruitGrowthRate = param["m_fruitGrowthRate"].as<float>();
		//Structure
		if (param["m_lateralBudCount"]) treeGrowthParameters.m_lateralBudCount = param["m_lateralBudCount"].as<int>();
		if (param["m_fruitBudCount"]) treeGrowthParameters.m_fruitBudCount = param["m_fruitBudCount"].as<int>();
		if (param["m_leafBudCount"]) treeGrowthParameters.m_leafBudCount = param["m_leafBudCount"].as<int>();

		if (param["m_branchingAngleMeanVariance"]) treeGrowthParameters.m_branchingAngleMeanVariance = param["m_branchingAngleMeanVariance"].as<glm::vec2>();
		if (param["m_rollAngleMeanVariance"]) treeGrowthParameters.m_rollAngleMeanVariance = param["m_rollAngleMeanVariance"].as<glm::vec2>();
		if (param["m_apicalAngleMeanVariance"]) treeGrowthParameters.m_apicalAngleMeanVariance = param["m_apicalAngleMeanVariance"].as<glm::vec2>();
		if (param["m_gravitropism"]) treeGrowthParameters.m_gravitropism = param["m_gravitropism"].as<float>();
		if (param["m_phototropism"]) treeGrowthParameters.m_phototropism = param["m_phototropism"].as<float>();

		if (param["m_internodeLength"]) treeGrowthParameters.m_internodeLength = param["m_internodeLength"].as<float>();

		if (param["m_endNodeThickness"]) treeGrowthParameters.m_endNodeThickness = param["m_endNodeThickness"].as<float>();
		if (param["m_thicknessAccumulationFactor"]) treeGrowthParameters.m_thicknessAccumulationFactor = param["m_thicknessAccumulationFactor"].as<float>();
		if (param["m_thicknessAccumulateAgeFactor"]) treeGrowthParameters.m_thicknessAccumulateAgeFactor = param["m_thicknessAccumulateAgeFactor"].as<float>();

		if (param["m_lowBranchPruning"]) treeGrowthParameters.m_lowBranchPruning = param["m_lowBranchPruning"].as<float>();
		if (param["m_saggingFactorThicknessReductionMax"]) treeGrowthParameters.m_saggingFactorThicknessReductionMax = param["m_saggingFactorThicknessReductionMax"].as<glm::vec3>();

		//Bud fate

		if (param["m_lateralBudFlushingProbabilityTemperatureRange"]) treeGrowthParameters.m_lateralBudFlushingProbabilityTemperatureRange = param["m_lateralBudFlushingProbabilityTemperatureRange"].as<glm::vec4>();
		if (param["m_leafBudFlushingProbabilityTemperatureRange"]) treeGrowthParameters.m_leafBudFlushingProbabilityTemperatureRange = param["m_leafBudFlushingProbabilityTemperatureRange"].as< glm::vec4>();
		if (param["m_fruitBudFlushingProbabilityTemperatureRange"]) treeGrowthParameters.m_fruitBudFlushingProbabilityTemperatureRange = param["m_fruitBudFlushingProbabilityTemperatureRange"].as<glm::vec4>();

		if (param["m_apicalBudLightingFactor"]) treeGrowthParameters.m_apicalBudLightingFactor = param["m_apicalBudLightingFactor"].as<float>();
		if (param["m_lateralBudLightingFactor"]) treeGrowthParameters.m_lateralBudLightingFactor = param["m_lateralBudLightingFactor"].as<float>();
		if (param["m_leafBudLightingFactor"]) treeGrowthParameters.m_leafBudLightingFactor = param["m_leafBudLightingFactor"].as<float>();
		if (param["m_fruitBudLightingFactor"]) treeGrowthParameters.m_fruitBudLightingFactor = param["m_fruitBudLightingFactor"].as<float>();

		if (param["m_apicalControl"]) treeGrowthParameters.m_apicalControl = param["m_apicalControl"].as<float>();
		if (param["m_apicalControlAgeFactor"]) treeGrowthParameters.m_apicalControlAgeFactor = param["m_apicalControlAgeFactor"].as<float>();
		if (param["m_apicalDominance"]) treeGrowthParameters.m_apicalDominance = param["m_apicalDominance"].as<float>();
		if (param["m_apicalDominanceAgeFactor"]) treeGrowthParameters.m_apicalDominanceAgeFactor = param["m_apicalDominanceAgeFactor"].as<float>();
		if (param["m_apicalDominanceDistanceFactor"]) treeGrowthParameters.m_apicalDominanceDistanceFactor = param["m_apicalDominanceDistanceFactor"].as<float>();

		if (param["m_apicalBudExtinctionRate"]) treeGrowthParameters.m_apicalBudExtinctionRate = param["m_apicalBudExtinctionRate"].as<float>();
		if (param["m_lateralBudExtinctionRate"]) treeGrowthParameters.m_lateralBudExtinctionRate = param["m_lateralBudExtinctionRate"].as<float>();
		if (param["m_leafBudExtinctionRate"]) treeGrowthParameters.m_leafBudExtinctionRate = param["m_leafBudExtinctionRate"].as<float>();
		if (param["m_fruitBudExtinctionRate"]) treeGrowthParameters.m_fruitBudExtinctionRate = param["m_fruitBudExtinctionRate"].as<float>();

		if (param["m_leafVigorRequirement"]) treeGrowthParameters.m_leafVigorRequirement = param["m_leafVigorRequirement"].as<float>();
		if (param["m_fruitVigorRequirement"]) treeGrowthParameters.m_fruitVigorRequirement = param["m_fruitVigorRequirement"].as<float>();
		if (param["m_internodeVigorRequirement"]) treeGrowthParameters.m_internodeVigorRequirement = param["m_internodeVigorRequirement"].as<float>();
		if (param["m_vigorRequirementAggregateLoss"]) treeGrowthParameters.m_vigorRequirementAggregateLoss = param["m_vigorRequirementAggregateLoss"].as<float>();

		//Foliage
		if (param["m_maxLeafSize"]) treeGrowthParameters.m_maxLeafSize = param["m_maxLeafSize"].as<glm::vec3>();
		if (param["m_leafPositionVariance"]) treeGrowthParameters.m_leafPositionVariance = param["m_leafPositionVariance"].as<float>();
		if (param["m_leafRandomRotation"]) treeGrowthParameters.m_leafRandomRotation = param["m_leafRandomRotation"].as<float>();
		if (param["m_leafChlorophyllLoss"]) treeGrowthParameters.m_leafChlorophyllLoss = param["m_leafChlorophyllLoss"].as<float>();
		if (param["m_leafChlorophyllSynthesisFactorTemperature"]) treeGrowthParameters.m_leafChlorophyllSynthesisFactorTemperature = param["m_leafChlorophyllSynthesisFactorTemperature"].as<float>();
		if (param["m_leafFallProbability"]) treeGrowthParameters.m_leafFallProbability = param["m_leafFallProbability"].as<float>();
		if (param["m_leafDistanceToBranchEndLimit"]) treeGrowthParameters.m_leafDistanceToBranchEndLimit = param["m_leafDistanceToBranchEndLimit"].as<float>();

		//Fruit
		if (param["m_maxFruitSize"]) treeGrowthParameters.m_maxFruitSize = param["m_maxFruitSize"].as<glm::vec3>();
		if (param["m_fruitPositionVariance"]) treeGrowthParameters.m_fruitPositionVariance = param["m_fruitPositionVariance"].as<float>();
		if (param["m_fruitRandomRotation"]) treeGrowthParameters.m_fruitRandomRotation = param["m_fruitRandomRotation"].as<float>();
		if (param["m_fruitFallProbability"]) treeGrowthParameters.m_fruitFallProbability = param["m_fruitFallProbability"].as<float>();
	}
}
void DeserializeRootGrowthParameters(const std::string& name, RootGrowthParameters& rootGrowthParameters, const YAML::Node& in) {
	if (in[name]) {
		auto& param = in[name];
		if (param["m_rootNodeLength"]) rootGrowthParameters.m_rootNodeLength = param["m_rootNodeLength"].as<float>();
		if (param["m_rootNodeGrowthRate"]) rootGrowthParameters.m_rootNodeGrowthRate = param["m_rootNodeGrowthRate"].as<float>();
		if (param["m_endNodeThickness"]) rootGrowthParameters.m_endNodeThickness = param["m_endNodeThickness"].as<float>();
		if (param["m_thicknessAccumulationFactor"]) rootGrowthParameters.m_thicknessAccumulationFactor = param["m_thicknessAccumulationFactor"].as<float>();
		if (param["m_thicknessAccumulateAgeFactor"]) rootGrowthParameters.m_thicknessAccumulateAgeFactor = param["m_thicknessAccumulateAgeFactor"].as<float>();

		if (param["m_branchingAngleMeanVariance"]) rootGrowthParameters.m_branchingAngleMeanVariance = param["m_branchingAngleMeanVariance"].as<glm::vec2>();
		if (param["m_rollAngleMeanVariance"]) rootGrowthParameters.m_rollAngleMeanVariance = param["m_rollAngleMeanVariance"].as<glm::vec2>();
		if (param["m_apicalAngleMeanVariance"]) rootGrowthParameters.m_apicalAngleMeanVariance = param["m_apicalAngleMeanVariance"].as<glm::vec2>();

		if (param["m_rootNodeVigorRequirement"]) rootGrowthParameters.m_rootNodeVigorRequirement = param["m_rootNodeVigorRequirement"].as<float>();
		if (param["m_vigorRequirementAggregateLoss"]) rootGrowthParameters.m_vigorRequirementAggregateLoss = param["m_vigorRequirementAggregateLoss"].as<float>();

		if (param["m_environmentalFriction"]) rootGrowthParameters.m_environmentalFriction = param["m_environmentalFriction"].as<float>();
		if (param["m_environmentalFrictionFactor"]) rootGrowthParameters.m_environmentalFrictionFactor = param["m_environmentalFrictionFactor"].as<float>();

		if (param["m_apicalControl"]) rootGrowthParameters.m_apicalControl = param["m_apicalControl"].as<float>();
		if (param["m_apicalControlAgeFactor"]) rootGrowthParameters.m_apicalControlAgeFactor = param["m_apicalControlAgeFactor"].as<float>();
		if (param["m_apicalDominance"]) rootGrowthParameters.m_apicalDominance = param["m_apicalDominance"].as<float>();
		if (param["m_apicalDominanceAgeFactor"]) rootGrowthParameters.m_apicalDominanceAgeFactor = param["m_apicalDominanceAgeFactor"].as<float>();
		if (param["m_apicalDominanceDistanceFactor"]) rootGrowthParameters.m_apicalDominanceDistanceFactor = param["m_apicalDominanceDistanceFactor"].as<float>();

		if (param["m_tropismSwitchingProbability"]) rootGrowthParameters.m_tropismSwitchingProbability = param["m_tropismSwitchingProbability"].as<float>();
		if (param["m_tropismSwitchingProbabilityDistanceFactor"]) rootGrowthParameters.m_tropismSwitchingProbabilityDistanceFactor = param["m_tropismSwitchingProbabilityDistanceFactor"].as<float>();
		if (param["m_tropismIntensity"]) rootGrowthParameters.m_tropismIntensity = param["m_tropismIntensity"].as<float>();

		if (param["m_branchingProbability"]) rootGrowthParameters.m_branchingProbability = param["m_branchingProbability"].as<float>();
	}
}
void TreeDescriptor::Deserialize(const YAML::Node& in) {
	DeserializeShootGrowthParameters("m_shootGrowthParameters", m_shootGrowthParameters, in);
	DeserializeRootGrowthParameters("m_rootGrowthParameters", m_rootGrowthParameters, in);
}