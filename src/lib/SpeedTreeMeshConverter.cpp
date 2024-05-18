#include "SpeedTreeMeshConverter.hpp"
#include "Prefab.hpp"
using namespace EcoSysLab;

void SpeedTreeMeshConverter::Convert()
{
	const auto originalModel = m_originalModel.Get<Prefab>();
	const auto foliageMesh = ProjectManager::CreateTemporaryAsset<Mesh>();
	const auto branchMesh = ProjectManager::CreateTemporaryAsset<Mesh>();


}

bool SpeedTreeMeshConverter::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	bool changed = false;
	if(EditorLayer::DragAndDropButton<Prefab>(m_originalModel, "SpeedTree Model")) changed = true;
	if(m_originalModel.Get<Mesh>() && ImGui::Button("Convert"))
	{
		Convert();
	}
	EditorLayer::DragAndDropButton<Mesh>(m_foliageMesh, "Foliage mesh");
	EditorLayer::DragAndDropButton<Mesh>(m_branchMesh, "Branch mesh");
	EditorLayer::DragAndDropButton<Mesh>(m_foliageMaterial, "Foliage material");
	EditorLayer::DragAndDropButton<Mesh>(m_branchMaterial, "Branch material");

	return changed;
}
