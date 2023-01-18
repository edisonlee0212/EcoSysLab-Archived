#pragma once

#include "ecosyslab_export.h"
#include "TreeModel.hpp"
#include "TreeVisualizer.hpp"
#include "TreeMeshGenerator.hpp"

using namespace UniEngine;
namespace EcoSysLab {

	class TreeDescriptor : public IAsset {
	public:
		TreeGrowthParameters m_treeGrowthParameters;
		RootGrowthParameters m_rootGrowthParameters;
		void OnCreate() override;

		void OnInspect() override;

		void CollectAssetRef(std::vector<AssetRef>& list) override;

		void Serialize(YAML::Emitter& out) override;

		void Deserialize(const YAML::Node& in) override;
	};

	class Tree : public IPrivateComponent {
		friend class EcoSysLabLayer;
		bool TryGrow();
	public:
		void Serialize(YAML::Emitter& out) override;

		void Deserialize(const YAML::Node& in) override;

		PrivateComponentRef m_soil;
		PrivateComponentRef m_climate;
		AssetRef m_treeDescriptor;
		bool m_enableHistory = false;
		TreeModel m_treeModel;
		void OnInspect() override;

		void OnDestroy() override;

		void OnCreate() override;

		void GenerateMesh(const TreeMeshGeneratorSettings& meshGeneratorSettings);
	};
}