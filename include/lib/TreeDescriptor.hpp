#pragma once
#include "TreeVisualizer.hpp"
#include "TreeMeshGenerator.hpp"
#include "LSystemString.hpp"
#include "RadialBoundingVolume.hpp"
#include "TreeGraph.hpp"
#include "TreeGrowthParameters.hpp"
using namespace EvoEngine;
namespace EcoSysLab {
	class TreeDescriptor : public IAsset {
	public:
		ShootDescriptor m_shootDescriptor;

		FoliageDescriptor m_foliageDescriptor;
		TwigParameters m_twigParameters;

		AssetRef m_shootBranchShape;
		AssetRef m_rootBranchShape;

		AssetRef m_foliageAlbedoTexture;
		AssetRef m_foliageNormalTexture;
		AssetRef m_foliageRoughnessTexture;
		AssetRef m_foliageMetallicTexture;

		AssetRef m_branchAlbedoTexture;
		AssetRef m_branchNormalTexture;
		AssetRef m_branchRoughnessTexture;
		AssetRef m_branchMetallicTexture;

		

		void OnCreate() override;

		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;

		void CollectAssetRef(std::vector<AssetRef>& list) override;

		void Serialize(YAML::Emitter& out) override;

		void Deserialize(const YAML::Node& in) override;

		static bool OnInspectShootDescriptor(ShootDescriptor& shootDescriptor);
		
		static bool OnInspectFoliageDescriptor(FoliageDescriptor& foliageDescriptor);
		static void SerializeFoliageDescriptor(const std::string& name, const FoliageDescriptor& foliageDescriptor, YAML::Emitter& out);
		static void SerializeShootDescriptor(const std::string& name, const ShootDescriptor& shootDescriptor, YAML::Emitter& out);
		static void DeserializeFoliageDescriptor(const std::string& name, FoliageDescriptor& foliageDescriptor, const YAML::Node& in);
		static void DeserializeShootDescriptor(const std::string& name, ShootDescriptor& shootDescriptor, const YAML::Node& in);
	};


}