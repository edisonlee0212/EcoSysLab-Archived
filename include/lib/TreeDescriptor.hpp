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
		ShootGrowthParameters m_shootGrowthParameters;

		FoliageParameters m_foliageParameters;
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

		static bool OnInspectShootGrowthParameters(ShootGrowthParameters& treeGrowthParameters);
		static bool OnInspectFoliageParameters(FoliageParameters& foliageParameters);
		static void SerializeFoliageParameters(const std::string& name, const FoliageParameters& foliageParameters, YAML::Emitter& out);
		static void SerializeShootGrowthParameters(const std::string& name, const ShootGrowthParameters& treeGrowthParameters, YAML::Emitter& out);
		static void DeserializeFoliageParameters(const std::string& name, FoliageParameters& foliageParameters, const YAML::Node& in);
		static void DeserializeShootGrowthParameters(const std::string& name, ShootGrowthParameters& treeGrowthParameters, const YAML::Node& in);
	};
}