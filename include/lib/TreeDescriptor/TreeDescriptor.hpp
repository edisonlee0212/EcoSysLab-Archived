#pragma once
using namespace EvoEngine;
namespace EcoSysLab {
	class TreeDescriptor : public IAsset {
	public:
		AssetRef m_shootDescriptor;
		AssetRef m_foliageDescriptor;
		
		AssetRef m_fruitDescriptor;
		AssetRef m_flowerDescriptor;

		AssetRef m_barkDescriptor;
		void OnCreate() override;

		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;

		void CollectAssetRef(std::vector<AssetRef>& list) override;

		void Serialize(YAML::Emitter& out) override;

		void Deserialize(const YAML::Node& in) override;

	};


}