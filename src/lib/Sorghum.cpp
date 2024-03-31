#include "Sorghum.hpp"

#include "SorghumDescriptor.hpp"
#include "SorghumLayer.hpp"

using namespace EcoSysLab;

void Sorghum::ClearGeometryEntities()
{
	const auto scene = GetScene();
	const auto self = GetOwner();
	const auto children = scene->GetChildren(self);
	for (const auto& child : children) {
		auto name = scene->GetEntityName(child);
		if (name == "Panicle Mesh") {
			scene->DeleteEntity(child);
		}
		else if (name == "Leaf Mesh") {
			scene->DeleteEntity(child);
		}
		else if (name == "Stem Mesh") {
			scene->DeleteEntity(child);
		}
	}
}

void Sorghum::GenerateGeometryEntities(const SorghumMeshGeneratorSettings& sorghumMeshGeneratorSettings)
{
	const auto sorghumLayer = Application::GetLayer<SorghumLayer>();
	if (!sorghumLayer) return;
	const auto sorghumState = m_sorghumState.Get<SorghumState>();
	if(!sorghumState)
	{
		if(const auto sorghumDescriptor = m_sorghumDescriptor.Get<SorghumDescriptor>())
		{
			sorghumDescriptor->Apply(sorghumState);
		}
	}
	
	if(!sorghumState) return;
	if (sorghumState->m_stem.m_nodes.empty()) return;
	ClearGeometryEntities();
	const auto scene = GetScene();
	const auto owner = GetOwner();
	if(sorghumMeshGeneratorSettings.m_enablePanicle && sorghumState->m_panicle.m_seedAmount > 0)
	{
		const auto panicleEntity = scene->CreateEntity("Panicle Mesh");
		const auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(panicleEntity).lock();
		const auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
		const auto material = ProjectManager::CreateTemporaryAsset<Material>();
		meshRenderer->m_mesh = mesh;
		meshRenderer->m_material = material;
		const auto panicleMaterial = sorghumLayer->m_panicleMaterial.Get<Material>();
		//material->SetAlbedoTexture(panicleMaterial->GetAlbedoTexture());
		//material->SetNormalTexture(panicleMaterial->GetNormalTexture());
		//material->SetRoughnessTexture(panicleMaterial->GetRoughnessTexture());
		//material->SetMetallicTexture(panicleMaterial->GetMetallicTexture());
		material->m_materialProperties = panicleMaterial->m_materialProperties;
		std::vector<Vertex> vertices;
		std::vector<unsigned int> indices;
		sorghumState->m_panicle.GenerateGeometry(sorghumState->m_stem.m_nodes.back().m_position, vertices, indices);
		VertexAttributes attributes{};
		attributes.m_texCoord = true;
		mesh->SetVertices(attributes, vertices, indices);
		scene->SetParent(panicleEntity, owner);
	}
	if(sorghumMeshGeneratorSettings.m_enableStem)
	{
		const auto stemEntity = scene->CreateEntity("Stem Mesh");
		const auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(stemEntity).lock();
		const auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
		const auto material = ProjectManager::CreateTemporaryAsset<Material>();
		meshRenderer->m_mesh = mesh;
		meshRenderer->m_material = material;
		const auto stemMaterial = sorghumLayer->m_leafMaterial.Get<Material>();
		//material->SetAlbedoTexture(stemMaterial->GetAlbedoTexture());
		//material->SetNormalTexture(stemMaterial->GetNormalTexture());
		//material->SetRoughnessTexture(stemMaterial->GetRoughnessTexture());
		//material->SetMetallicTexture(stemMaterial->GetMetallicTexture());
		material->m_materialProperties = stemMaterial->m_materialProperties;
		std::vector<Vertex> vertices;
		std::vector<unsigned int> indices;
		sorghumState->m_stem.GenerateGeometry(vertices, indices);
		VertexAttributes attributes{};
		attributes.m_texCoord = true;
		mesh->SetVertices(attributes, vertices, indices);
		scene->SetParent(stemEntity, owner);
	}
	if(sorghumMeshGeneratorSettings.m_enableLeaves)
	{
		if (sorghumMeshGeneratorSettings.m_leafSeparated) {
			for (const auto& leafState : sorghumState->m_leaves)
			{
				const auto leafEntity = scene->CreateEntity("Leaf Mesh");
				const auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(leafEntity).lock();
				const auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
				const auto material = ProjectManager::CreateTemporaryAsset<Material>();
				meshRenderer->m_mesh = mesh;
				meshRenderer->m_material = material;
				const auto leafMaterial = sorghumLayer->m_leafMaterial.Get<Material>();
				//material->SetAlbedoTexture(leafMaterial->GetAlbedoTexture());
				//material->SetNormalTexture(leafMaterial->GetNormalTexture());
				//material->SetRoughnessTexture(leafMaterial->GetRoughnessTexture());
				//material->SetMetallicTexture(leafMaterial->GetMetallicTexture());
				material->m_materialProperties = leafMaterial->m_materialProperties;
				std::vector<Vertex> vertices;
				std::vector<unsigned int> indices;
				leafState.GenerateGeometry(vertices, indices, false, 0.f);
				if (sorghumMeshGeneratorSettings.m_bottomFace)
				{
					leafState.GenerateGeometry(vertices, indices, true, sorghumMeshGeneratorSettings.m_leafThickness);
				}
				VertexAttributes attributes{};
				attributes.m_texCoord = true;
				mesh->SetVertices(attributes, vertices, indices);
				scene->SetParent(leafEntity, owner);
			}
		}else
		{
			const auto leafEntity = scene->CreateEntity("Leaf Mesh");
			const auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(leafEntity).lock();
			const auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
			const auto material = ProjectManager::CreateTemporaryAsset<Material>();
			meshRenderer->m_mesh = mesh;
			meshRenderer->m_material = material;
			const auto leafMaterial = sorghumLayer->m_leafMaterial.Get<Material>();
			//material->SetAlbedoTexture(leafMaterial->GetAlbedoTexture());
			//material->SetNormalTexture(leafMaterial->GetNormalTexture());
			//material->SetRoughnessTexture(leafMaterial->GetRoughnessTexture());
			//material->SetMetallicTexture(leafMaterial->GetMetallicTexture());
			material->m_materialProperties = leafMaterial->m_materialProperties;
			std::vector<Vertex> vertices;
			std::vector<unsigned int> indices;
			for (const auto& leafState : sorghumState->m_leaves)
			{
				leafState.GenerateGeometry(vertices, indices, false, 0.f);
				if (sorghumMeshGeneratorSettings.m_bottomFace)
				{
					leafState.GenerateGeometry(vertices, indices, true, sorghumMeshGeneratorSettings.m_leafThickness);
				}
			}
			VertexAttributes attributes{};
			attributes.m_texCoord = true;
			mesh->SetVertices(attributes, vertices, indices);
			scene->SetParent(leafEntity, owner);
		}
	}
}

void Sorghum::Serialize(YAML::Emitter& out)
{
	m_sorghumState.Save("m_sorghumState", out);
	m_sorghumDescriptor.Save("m_sorghumDescriptor", out);
	m_sorghumGrowthDescriptor.Save("m_sorghumGrowthDescriptor", out);
}

void Sorghum::Deserialize(const YAML::Node& in)
{
	m_sorghumState.Load("m_sorghumState", in);
	m_sorghumGrowthDescriptor.Load("m_sorghumGrowthDescriptor", in);
	m_sorghumDescriptor.Load("m_sorghumDescriptor", in);
}

void Sorghum::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	editorLayer->DragAndDropButton<SorghumDescriptor>(m_sorghumDescriptor, "SorghumDescriptor");
	editorLayer->DragAndDropButton<SorghumGrowthDescriptor>(m_sorghumGrowthDescriptor, "SorghumGrowthDescriptor");
	editorLayer->DragAndDropButton<SorghumState>(m_sorghumState, "SorghumState");

	if(ImGui::Button("Form meshes"))
	{
		GenerateGeometryEntities(SorghumMeshGeneratorSettings{});
	}

	if(const auto sorghumDescriptor = m_sorghumDescriptor.Get<SorghumDescriptor>())
	{
		if (ImGui::TreeNode("Sorghum Descriptor settings")) {
			static int seed = 0;
			if (ImGui::DragInt("Seed", &seed)) {
				auto sorghumState = m_sorghumState.Get<SorghumState>();
				if (!sorghumState)
				{
					sorghumState = ProjectManager::CreateTemporaryAsset<SorghumState>();
					m_sorghumState = sorghumState;
				}
				sorghumDescriptor->Apply(sorghumState, seed);
				GenerateGeometryEntities(SorghumMeshGeneratorSettings{});
			}
			ImGui::TreePop();
		}
	}
	if (const auto sorghumGrowthDescriptor = m_sorghumGrowthDescriptor.Get<SorghumGrowthDescriptor>())
	{
		if (ImGui::TreeNode("Sorghum Growth Descriptor settings")) {
			static float time = 0.0f;
			if (ImGui::SliderFloat("Time", &time, 0.0f,
				sorghumGrowthDescriptor->GetCurrentEndTime())) {
				time = glm::clamp(time, 0.0f, sorghumGrowthDescriptor->GetCurrentEndTime());
				auto sorghumState = m_sorghumState.Get<SorghumState>();
				if (!sorghumState)
				{
					sorghumState = ProjectManager::CreateTemporaryAsset<SorghumState>();
					m_sorghumState = sorghumState;
				}
				sorghumGrowthDescriptor->Apply(sorghumState, time);
				GenerateGeometryEntities(SorghumMeshGeneratorSettings{});
			}
		}
	}
	static bool debugRendering = false;
	ImGui::Checkbox("Debug", &debugRendering);
	if (debugRendering) {
		static float nodeRenderSize = .5f;
		if(ImGui::TreeNode("Debug settings"))
		{
			ImGui::DragFloat("Node size", &nodeRenderSize, 0.01f, 0.0f, 1.f);
			ImGui::TreePop();
		}
		static std::shared_ptr<ParticleInfoList> nodeDebugInfoList;
		if (!nodeDebugInfoList) nodeDebugInfoList = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
		
		auto& particleInfos = nodeDebugInfoList->m_particleInfos;
		particleInfos.clear();
		if (const auto sorghumState = m_sorghumState.Get<SorghumState>())
		{
			const auto owner = GetOwner();
			const auto scene = GetScene();
			const auto plantPosition = scene->GetDataComponent<GlobalTransform>(owner).GetPosition();
			for (const auto& leafState : sorghumState->m_leaves) {
				const auto startIndex = particleInfos.size();
				particleInfos.resize(startIndex + leafState.m_nodes.size());
				for(int i = 0; i < leafState.m_nodes.size(); i++)
				{
					auto& matrix = particleInfos[startIndex + i].m_instanceMatrix;
					matrix.m_value = glm::translate(leafState.m_nodes.at(i).m_position + plantPosition) * glm::scale(glm::vec3(nodeRenderSize * leafState.m_nodes.at(i).m_leafWidth));
					particleInfos[startIndex + i].m_instanceColor = glm::vec4((leafState.m_index % 3) * 0.5f, ((leafState.m_index / 3) % 3) * 0.5f,
						((leafState.m_index / 9) % 3) * 0.5f, 1.0f);
				}
			}
			nodeDebugInfoList->SetPendingUpdate();

		}
		editorLayer->DrawGizmoCubes(nodeDebugInfoList);
	}
}

void Sorghum::CollectAssetRef(std::vector<AssetRef>& list)
{
	if (m_sorghumState.Get<SorghumState>()) list.push_back(m_sorghumState);
	if (m_sorghumGrowthDescriptor.Get<SorghumGrowthDescriptor>()) list.push_back(m_sorghumGrowthDescriptor);
	if (m_sorghumDescriptor.Get<SorghumDescriptor>()) list.push_back(m_sorghumDescriptor);
}
