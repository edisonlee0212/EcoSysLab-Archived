#pragma once
#include "TreeVisualizer.hpp"
#include "TreeMeshGenerator.hpp"
#include "LSystemString.hpp"
#include "RadialBoundingVolume.hpp"
#include "TreeGraph.hpp"
#include "ShootDescriptor.hpp"
using namespace EvoEngine;
namespace EcoSysLab {
	class TreeDescriptor : public IAsset {
	public:
		AssetRef m_shootDescriptor;
		AssetRef m_foliageDescriptor;
		AssetRef m_shootBranchShape;
		void OnCreate() override;

		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;

		void CollectAssetRef(std::vector<AssetRef>& list) override;

		void Serialize(YAML::Emitter& out) override;

		void Deserialize(const YAML::Node& in) override;

	};


}