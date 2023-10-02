#pragma once

#include "TreePipeModel.hpp"
#include "TreeVisualizer.hpp"
#include "TreeMeshGenerator.hpp"
#include "LSystemString.hpp"
#include "TreeGraph.hpp"
#include "TreeGrowthParameters.hpp"
using namespace EvoEngine;
namespace EcoSysLab {
	
	class TreeDescriptor : public IAsset {
	public:
		ShootGrowthParameters m_shootGrowthParameters;
		RootGrowthParameters m_rootGrowthParameters;

		FineRootParameters m_fineRootParameters;
		TwigParameters m_twigParameters;
		void OnCreate() override;

		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;

		void CollectAssetRef(std::vector<AssetRef>& list) override;

		void Serialize(YAML::Emitter& out) override;

		void Deserialize(const YAML::Node& in) override;
	};

	class Tree : public IPrivateComponent {
		friend class EcoSysLabLayer;
		void PrepareControllers(const std::shared_ptr<TreeDescriptor>& treeDescriptor);
		bool TryGrow(float deltaTime);
		bool TryGrowSubTree(NodeHandle internodeHandle, float deltaTime);
		ShootGrowthController m_shootGrowthController{};
		RootGrowthController m_rootGrowthController{};
		FineRootController m_fineRootController{};
		TwigController m_twigController{};

		
	public:
		PipeModelParameters m_pipeModelParameters{};
		
		void InitializeStrandRenderer();

		void Serialize(YAML::Emitter& out) override;
		bool m_splitRootTest = true;
		bool m_recordBiomassHistory = true;
		float m_leftSideBiomass;
		float m_rightSideBiomass;

		TreeMeshGeneratorSettings m_meshGeneratorSettings {};
		int m_temporalProgressionIteration = 0;
		bool m_temporalProgression = false;
		void Update() override;

		void Deserialize(const YAML::Node& in) override;

		std::vector<float> m_rootBiomassHistory;
		std::vector<float> m_shootBiomassHistory;

		PrivateComponentRef m_soil;
		PrivateComponentRef m_climate;
		AssetRef m_treeDescriptor;
		bool m_enableHistory = false;
		int m_historyIteration = 30;
		TreeModel m_treeModel{};
		TreePipeModel m_treePipeModel{};
		PipeModelPipeProfile m_baseProfile{};

		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;

		void OnDestroy() override;

		void OnCreate() override;

		void ClearMeshes() const;
		void ClearStrands() const;
		void GenerateMeshes(const TreeMeshGeneratorSettings& meshGeneratorSettings, int iteration = -1);

		void FromLSystemString(const std::shared_ptr<LSystemString>& lSystemString);
		void FromTreeGraph(const std::shared_ptr<TreeGraph>& treeGraph);
		void FromTreeGraphV2(const std::shared_ptr<TreeGraphV2>& treeGraphV2);
	};
}
