#include "Sorghum.hpp"

#include "SorghumDescriptor.hpp"

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
		for(const auto& leafState : sorghumState->m_leaves)
		{
			const auto leafEntity = scene->CreateEntity("Leaf Mesh");
			const auto meshRenderer = scene->GetOrSetPrivateComponent<MeshRenderer>(leafEntity).lock();
			const auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
			const auto material = ProjectManager::CreateTemporaryAsset<Material>();
			meshRenderer->m_mesh = mesh;
			meshRenderer->m_material = material;
			std::vector<Vertex> vertices;
			std::vector<unsigned int> indices;
			leafState.GenerateGeometry(vertices, indices, sorghumMeshGeneratorSettings.m_bottomFace, sorghumMeshGeneratorSettings.m_leafThickness);
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

}

void Sorghum::Deserialize(const YAML::Node& in)
{
	m_sorghumState.Load("m_sorghumState", in);
}

void Sorghum::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	editorLayer->DragAndDropButton<SorghumState>(m_sorghumState, "SorghumState");
}

void Sorghum::CollectAssetRef(std::vector<AssetRef>& list)
{
	if (m_sorghumState.Get<SorghumState>()) list.push_back(m_sorghumState);
}
