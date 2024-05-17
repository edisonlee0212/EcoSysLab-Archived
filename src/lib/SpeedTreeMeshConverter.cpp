#include "SpeedTreeMeshConverter.hpp"

using namespace EcoSysLab;

void SpeedTreeMeshConverter::Convert()
{
	const auto originalMesh = m_originalMesh.Get<Mesh>();
	const auto foliageMesh = ProjectManager::CreateTemporaryAsset<Mesh>();
	const auto branchMesh = ProjectManager::CreateTemporaryAsset<Mesh>();


}

bool SpeedTreeMeshConverter::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	EditorLayer::DragAndDropButton<Mesh>(m_originalMesh, "Original mesh");
	if(m_originalMesh.Get<Mesh>() && ImGui::Button("Convert"))
	{
		Convert();
	}
	EditorLayer::DragAndDropButton<Mesh>(m_foliageMesh, "Foliage mesh");
	EditorLayer::DragAndDropButton<Mesh>(m_branchMesh, "Branch mesh");
}
