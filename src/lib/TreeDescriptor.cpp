//
// Created by lllll on 10/24/2022.
//

#include "Tree.hpp"

#include "Material.hpp"
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

#include "BarkDescriptor.hpp"
#include "FlowerDescriptor.hpp"
#include "FoliageDescriptor.hpp"
#include "FruitDescriptor.hpp"
#include "ShootDescriptor.hpp"
using namespace EcoSysLab;

void TreeDescriptor::OnCreate() {

}

bool TreeDescriptor::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
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
	if(editorLayer->DragAndDropButton<ShootDescriptor>(m_shootDescriptor, "Shoot Descriptor")) changed = true;
	if (editorLayer->DragAndDropButton<FoliageDescriptor>(m_foliageDescriptor, "Foliage Descriptor")) changed = true;
	if (editorLayer->DragAndDropButton<FruitDescriptor>(m_fruitDescriptor, "Fruit Descriptor")) changed = true;
	if (editorLayer->DragAndDropButton<FlowerDescriptor>(m_flowerDescriptor, "Flower Descriptor")) changed = true;

	editorLayer->DragAndDropButton<BarkDescriptor>(m_barkDescriptor, "BarkDescriptor");
	return changed;
}

void TreeDescriptor::CollectAssetRef(std::vector<AssetRef>& list) {
	if (m_shootDescriptor.Get<ShootDescriptor>()) list.push_back(m_shootDescriptor);
	if (m_foliageDescriptor.Get<FoliageDescriptor>()) list.push_back(m_foliageDescriptor);
	if (m_fruitDescriptor.Get<FruitDescriptor>()) list.push_back(m_fruitDescriptor);
	if (m_flowerDescriptor.Get<FlowerDescriptor>()) list.push_back(m_flowerDescriptor);

	if (m_barkDescriptor.Get<BarkDescriptor>()) list.push_back(m_barkDescriptor);
}


void TreeDescriptor::Serialize(YAML::Emitter& out) const {
	m_shootDescriptor.Save("m_shootDescriptor", out);
	m_foliageDescriptor.Save("m_foliageDescriptor", out);
	m_barkDescriptor.Save("m_barkDescriptor", out);

	m_fruitDescriptor.Save("m_fruitDescriptor", out);
	m_flowerDescriptor.Save("m_flowerDescriptor", out);
}



void TreeDescriptor::Deserialize(const YAML::Node& in) {
	m_shootDescriptor.Load("m_shootDescriptor", in);
	m_foliageDescriptor.Load("m_foliageDescriptor", in);
	m_barkDescriptor.Load("m_barkDescriptor", in);

	m_fruitDescriptor.Load("m_fruitDescriptor", in);
	m_flowerDescriptor.Load("m_flowerDescriptor", in);
}