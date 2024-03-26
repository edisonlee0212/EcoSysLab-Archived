//
// Created by lllll on 10/24/2022.
//

#include "Tree.hpp"

#include <Material.hpp>
#include <Mesh.hpp>
#include "Strands.hpp"
#include "EditorLayer.hpp"
#include "Application.hpp"
#include "TreeMeshGenerator.hpp"
#include "Soil.hpp"
#include "Climate.hpp"
#include "Octree.hpp"
#include "EcoSysLabLayer.hpp"
#include "HeightField.hpp"
#include "StrandsRenderer.hpp"
#include "TreeDescriptor.hpp"

#include "BranchShape.hpp"
#include "FoliageDescriptor.hpp"
using namespace EcoSysLab;

void TreeDescriptor::OnCreate() {

}

void TreeDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
	bool changed = false;
	const auto ecoSysLabLayer = Application::GetLayer<EcoSysLabLayer>();
	std::shared_ptr<Climate> climate;
	std::shared_ptr<Soil> soil;
	const auto climateCandidate = EcoSysLabLayer::FindClimate();
	if (!climateCandidate.expired()) climate = climateCandidate.lock();
	const auto soilCandidate = EcoSysLabLayer::FindSoil();
	if (!soilCandidate.expired()) soil = soilCandidate.lock();
	if (soil && climate) {
		if (ImGui::Button("Instantiate")) {
			const auto scene = Application::GetActiveScene();
			const auto treeEntity = scene->CreateEntity(GetTitle());
			const auto tree = scene->GetOrSetPrivateComponent<Tree>(treeEntity).lock();
			float height = 0;
			if (const auto soilDescriptor = soil->m_soilDescriptor.Get<SoilDescriptor>())
			{
				if (const auto heightField = soilDescriptor->m_heightField.Get<HeightField>()) height = heightField->GetValue({ 0.0f, 0.0f }) - 0.05f;
			}
			GlobalTransform globalTransform;
			globalTransform.SetPosition(glm::vec3(0, height, 0));
			scene->SetDataComponent(treeEntity, globalTransform);
			tree->m_treeDescriptor = ProjectManager::GetAsset(GetHandle());
			editorLayer->SetSelectedEntity(treeEntity);
		}
	}
	else
	{
		ImGui::Text("Create soil and climate entity to instantiate!");
	}
	editorLayer->DragAndDropButton<ShootDescriptor>(m_shootDescriptor, "Shoot Descriptor");
	editorLayer->DragAndDropButton<FoliageDescriptor>(m_foliageDescriptor, "Foliage Descriptor");
	editorLayer->DragAndDropButton<BranchShape>(m_shootBranchShape, "Shoot Branch Shape");
	if (changed) m_saved = false;
}

void TreeDescriptor::CollectAssetRef(std::vector<AssetRef>& list) {
	if (m_shootDescriptor.Get<ShootDescriptor>()) list.push_back(m_shootDescriptor);
	if (m_foliageDescriptor.Get<FoliageDescriptor>()) list.push_back(m_foliageDescriptor);
	if (m_shootBranchShape.Get<BranchShape>()) list.push_back(m_shootBranchShape);
}


void TreeDescriptor::Serialize(YAML::Emitter& out) {
	m_shootDescriptor.Save("m_shootDescriptor", out);
	m_foliageDescriptor.Save("m_foliageDescriptor", out);
	m_shootBranchShape.Save("m_shootBranchShape", out);
}



void TreeDescriptor::Deserialize(const YAML::Node& in) {
	m_shootDescriptor.Load("m_shootDescriptor", in);
	m_foliageDescriptor.Load("m_foliageDescriptor", in);
	m_shootBranchShape.Load("m_shootBranchShape", in);
}