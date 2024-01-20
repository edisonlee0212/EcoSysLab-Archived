#pragma once
#include "TreeVisualizer.hpp"
#include "TreeMeshGenerator.hpp"
#include "LSystemString.hpp"
#include "RadialBoundingVolume.hpp"
#include "TreeDescriptor.hpp"
#include "TreeGraph.hpp"
#include "TreeGrowthParameters.hpp"
#include "TreePipeMeshGenerator.hpp"
using namespace EvoEngine;
namespace EcoSysLab {

	struct SkeletalGraphSettings
	{
		float m_lineThickness = 0.005f;
		float m_branchPointSize = 1.0f;
		float m_junctionPointSize = 1.75f;

		bool m_fixedPointSize = false;
		float m_fixedPointSizeFactor = 0.01f;
		glm::vec4 m_lineColor = glm::vec4(1.f, .5f, 0.5f, 1.0f);
		glm::vec4 m_branchPointColor = glm::vec4(1.f, 1.f, 0.f, 1.f);
		glm::vec4 m_junctionPointColor = glm::vec4(0.f, .7f, 1.f, 1.f);
		void OnInspect();
	};

	class Tree : public IPrivateComponent {
		friend class EcoSysLabLayer;
		void PrepareControllers(const std::shared_ptr<TreeDescriptor>& treeDescriptor);
		ShootGrowthController m_shootGrowthController{};
	public:
		PipeModelParameters m_pipeModelParameters{};

		static void SerializeTreeGrowthSettings(const TreeGrowthSettings& treeGrowthSettings, YAML::Emitter& out);
		static void DeserializeTreeGrowthSettings(TreeGrowthSettings& treeGrowthSettings, const YAML::Node& param);
		static bool OnInspectTreeGrowthSettings(TreeGrowthSettings& treeGrowthSettings);

		void PrepareProfiles();
		std::shared_ptr<Strands> GenerateStrands();

		std::shared_ptr<Mesh> GenerateBranchMesh(const TreeMeshGeneratorSettings& meshGeneratorSettings);
		std::shared_ptr<Mesh> GenerateFoliageMesh(const TreeMeshGeneratorSettings& meshGeneratorSettings);
		std::shared_ptr<Mesh> GeneratePipeModelBranchMesh(const PipeModelMeshGeneratorSettings& treePipeMeshGeneratorSettings);
		std::shared_ptr<Mesh> GeneratePipeModelFoliageMesh(const PipeModelMeshGeneratorSettings& pipeModelMeshGeneratorSettings);
		void ExportOBJ(const std::filesystem::path& path, const TreeMeshGeneratorSettings& meshGeneratorSettings);
		bool TryGrow(float deltaTime, NodeHandle baseInternodeHandle, bool pruning, float overrideGrowthRate);
		[[nodiscard]] bool ParseBinvox(const std::filesystem::path& filePath, VoxelGrid<TreeOccupancyGridBasicData>& voxelGrid, float voxelSize = 1.0f);

		void Reset();

		TreeVisualizer m_treeVisualizer {};
		
		void Serialize(YAML::Emitter& out) override;
		bool m_splitRootTest = true;
		bool m_recordBiomassHistory = true;
		float m_leftSideBiomass;
		float m_rightSideBiomass;
		SkeletalGraphSettings m_skeletalGraphSettings{};
		TreeMeshGeneratorSettings m_meshGeneratorSettings {};
		PipeModelMeshGeneratorSettings m_treePipeMeshGeneratorSettings{};
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
		void ClearSkeletalGraph() const;
		void InitializeSkeletalGraph(const std::shared_ptr<Mesh> &pointMeshSample, const std::shared_ptr<Mesh>& lineMeshSample, const SkeletalGraphSettings& skeletalGraphSettings);

		TreeModel m_treeModel{};
		
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;

		void OnDestroy() override;

		void OnCreate() override;

		void InitializeMeshRenderer(const TreeMeshGeneratorSettings& meshGeneratorSettings, int iteration = -1);
		void ClearMeshRenderer() const;
		void ClearTwigsStrandRenderer() const;


		void InitializeStrandRenderer();
		void InitializeStrandRenderer(const std::shared_ptr<Strands>& strands) const;
		void ClearStrandRenderer() const;

		void InitializePipeModelMeshRenderer(const PipeModelMeshGeneratorSettings& pipeModelMeshGeneratorSettings);

		void ClearPipeModelMeshRenderer();

		void RegisterVoxel();
		void FromLSystemString(const std::shared_ptr<LSystemString>& lSystemString);
		void FromTreeGraph(const std::shared_ptr<TreeGraph>& treeGraph);
		void FromTreeGraphV2(const std::shared_ptr<TreeGraphV2>& treeGraphV2);
		void ExportJunction(const TreeMeshGeneratorSettings& meshGeneratorSettings, const std::filesystem::path& path);
		[[maybe_unused]] bool ExportIOTree(const std::filesystem::path& path) const;
		void ExportRadialBoundingVolume(const std::shared_ptr<RadialBoundingVolume>& rbv) const;
	};
}
