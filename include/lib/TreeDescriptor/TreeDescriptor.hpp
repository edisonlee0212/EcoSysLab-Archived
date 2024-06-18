#pragma once
using namespace evo_engine;
namespace eco_sys_lab {
class TreeDescriptor : public IAsset {
 public:
  AssetRef m_shootDescriptor;
  AssetRef m_foliageDescriptor;

  AssetRef m_fruitDescriptor;
  AssetRef m_flowerDescriptor;

  AssetRef m_barkDescriptor;
  void OnCreate() override;

  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  void CollectAssetRef(std::vector<AssetRef>& list) override;

  void Serialize(YAML::Emitter& out) const override;

  void Deserialize(const YAML::Node& in) override;
};

}  // namespace eco_sys_lab