#pragma once
#include "Climate.hpp"
#include "TreeVisualizer.hpp"
#include "TreeMeshGenerator.hpp"
#include "LSystemString.hpp"
#include "RadialBoundingVolume.hpp"
#include "TreeDescriptor.hpp"
#include "TreeGraph.hpp"
#include "StrandModelMeshGenerator.hpp"
#include "TreeIOTree.hpp"
#include "FoliageDescriptor.hpp"
#include "ShootDescriptor.hpp"
#include "Soil.hpp"
using namespace EvoEngine;
namespace EcoSysLab {
	struct SkeletalGraphSettings
	{
		float m_lineThickness = 0.0f;
		float m_fixedLineThickness = 0.002f;
		float m_branchPointSize = 1.0f;
		float m_junctionPointSize = 1.f;

		bool m_fixedPointSize = true;
		float m_fixedPointSizeFactor = 0.005f;
		glm::vec4 m_lineColor = glm::vec4(1.f, .5f, 0.5f, 1.0f);
		glm::vec4 m_branchPointColor = glm::vec4(1.f, 1.f, 0.f, 1.f);
		glm::vec4 m_junctionPointColor = glm::vec4(0.f, .7f, 1.f, 1.f);

		glm::vec4 m_lineFocusColor = glm::vec4(1.f, 0.f, 0.f, 1.f);
		glm::vec4 m_branchFocusColor = glm::vec4(1.f, 0.f, 0.f, 1.f);
		void OnInspect();
	};
	struct JunctionLine {
		int m_lineIndex = -1;
		glm::vec3 m_startPosition;
		glm::vec3 m_endPosition;
		float m_startRadius;
		float m_endRadius;

		glm::vec3 m_startDirection;
		glm::vec3 m_endDirection;
	};

	struct TreePartData {
		int m_treePartIndex;
		bool m_isJunction = false;
		JunctionLine m_baseLine;
		std::vector<JunctionLine> m_childrenLines;
		std::vector<SkeletonNodeHandle> m_nodeHandles;
		std::vector<bool> m_isEnd;
		std::vector<int> m_lineIndex;

		int m_numOfLeaves = 0;
	};

	class Tree : public IPrivateComponent {
		void CalculateProfiles();
		friend class EcoSysLabLayer;
		void PrepareController(const std::shared_ptr<ShootDescriptor>& shootDescriptor, const std::shared_ptr<Soil>& soil, const std::shared_ptr<Climate>& climate);
		ShootGrowthController m_shootGrowthController{};

		void GenerateTreeParts(const TreeMeshGeneratorSettings& meshGeneratorSettings, std::vector<TreePartData>& treeParts);
	public:
		StrandModelParameters m_strandModelParameters{};
		static void SerializeTreeGrowthSettings(const TreeGrowthSettings& treeGrowthSettings, YAML::Emitter& out);
		static void DeserializeTreeGrowthSettings(TreeGrowthSettings& treeGrowthSettings, const YAML::Node& param);
		static bool OnInspectTreeGrowthSettings(TreeGrowthSettings& treeGrowthSettings);

		
		void BuildStrandModel();
		
		std::shared_ptr<Strands> GenerateStrands() const;
		void GenerateTrunkMeshes(const std::shared_ptr<Mesh>& trunkMesh, const TreeMeshGeneratorSettings& meshGeneratorSettings);
		std::shared_ptr<Mesh> GenerateBranchMesh(const TreeMeshGeneratorSettings& meshGeneratorSettings);
		std::shared_ptr<Mesh> GenerateFoliageMesh(const TreeMeshGeneratorSettings& meshGeneratorSettings);
		std::shared_ptr<Strands> GenerateTwigStrands(const TreeMeshGeneratorSettings& meshGeneratorSettings);
		std::shared_ptr<ParticleInfoList> GenerateFoliageParticleInfoList(const TreeMeshGeneratorSettings& meshGeneratorSettings);
		std::shared_ptr<Mesh> GenerateStrandModelBranchMesh(const StrandModelMeshGeneratorSettings& strandModelMeshGeneratorSettings);
		std::shared_ptr<Mesh> GenerateStrandModelFoliageMesh(const StrandModelMeshGeneratorSettings& strandModelMeshGeneratorSettings);
		void ExportOBJ(const std::filesystem::path& path, const TreeMeshGeneratorSettings& meshGeneratorSettings);
		void ExportStrandModelOBJ(const std::filesystem::path& path, const StrandModelMeshGeneratorSettings& meshGeneratorSettings);

		void ExportTrunkOBJ(const std::filesystem::path& path, const TreeMeshGeneratorSettings& meshGeneratorSettings);
		bool TryGrow(float deltaTime, bool pruning, float overrideGrowthRate);

		bool TryGrowSubTree(float deltaTime, SkeletonNodeHandle baseInternodeHandle, bool pruning, float overrideGrowthRate);
		[[nodiscard]] bool ParseBinvox(const std::filesystem::path& filePath, VoxelGrid<TreeOccupancyGridBasicData>& voxelGrid, float voxelSize = 1.0f);

		void Reset();

		TreeVisualizer m_treeVisualizer {};
		
		void Serialize(YAML::Emitter& out) override;
		bool m_splitRootTest = true;
		bool m_recordBiomassHistory = true;
		float m_leftSideBiomass;
		float m_rightSideBiomass;
		TreeMeshGeneratorSettings m_meshGeneratorSettings {};
		StrandModelMeshGeneratorSettings m_strandModelMeshGeneratorSettings{};
		SkeletalGraphSettings m_skeletalGraphSettings{};

		int m_temporalProgressionIteration = 0;
		bool m_temporalProgression = false;
		void Update() override;

		

		std::vector<float> m_rootBiomassHistory;
		std::vector<float> m_shootBiomassHistory;

		PrivateComponentRef m_soil;
		PrivateComponentRef m_climate;
		AssetRef m_treeDescriptor;
		bool m_enableHistory = false;
		int m_historyIteration = 30;
		void ClearSkeletalGraph() const;
		void GenerateSkeletalGraph(const SkeletalGraphSettings& skeletalGraphSettings, SkeletonNodeHandle baseNodeHandle, const std::shared_ptr<Mesh> &pointMeshSample, const std::shared_ptr<Mesh>& lineMeshSample) const;

		TreeModel m_treeModel{};
		StrandModel m_strandModel{};
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;

		void OnDestroy() override;

		void OnCreate() override;

		void GenerateGeometryEntities(const TreeMeshGeneratorSettings& meshGeneratorSettings, int iteration = -1);
		void ClearGeometryEntities() const;
		void ClearTwigsStrandRenderer() const;


		void InitializeStrandRenderer();
		void InitializeStrandRenderer(const std::shared_ptr<Strands>& strands) const;
		void ClearStrandRenderer() const;

		void InitializeStrandModelMeshRenderer(const StrandModelMeshGeneratorSettings& strandModelMeshGeneratorSettings);

		void ClearStrandModelMeshRenderer() const;

		void RegisterVoxel();
		template<typename SrcSkeletonData, typename SrcFlowData, typename SrcNodeData>
		void FromSkeleton(const Skeleton<SrcSkeletonData, SrcFlowData, SrcNodeData>& srcSkeleton);
		void FromLSystemString(const std::shared_ptr<LSystemString>& lSystemString);
		void FromTreeGraph(const std::shared_ptr<TreeGraph>& treeGraph);
		void FromTreeGraphV2(const std::shared_ptr<TreeGraphV2>& treeGraphV2);
		void ExportTreeParts(const TreeMeshGeneratorSettings& meshGeneratorSettings, YAML::Emitter& out);
		void ExportTreeParts(const TreeMeshGeneratorSettings& meshGeneratorSettings, treeio::json& out);

		void ExportTreeParts(const TreeMeshGeneratorSettings& meshGeneratorSettings, const std::filesystem::path& path);
		[[maybe_unused]] bool ExportIOTree(const std::filesystem::path& path) const;
		void ExportRadialBoundingVolume(const std::shared_ptr<RadialBoundingVolume>& rbv) const;
		void CollectAssetRef(std::vector<AssetRef>& list) override;
		void Deserialize(const YAML::Node& in) override;
	};

	template <typename SrcSkeletonData, typename SrcFlowData, typename SrcNodeData>
	void Tree::FromSkeleton(const Skeleton<SrcSkeletonData, SrcFlowData, SrcNodeData>& srcSkeleton)
	{
		
		auto treeDescriptor = m_treeDescriptor.Get<TreeDescriptor>();
		if (!treeDescriptor) {
			EVOENGINE_WARNING("Growing tree without tree descriptor!");
			treeDescriptor = ProjectManager::CreateTemporaryAsset<TreeDescriptor>();
			m_treeDescriptor = treeDescriptor;
			const auto shootDescriptor = ProjectManager::CreateTemporaryAsset<ShootDescriptor>();
			treeDescriptor->m_shootDescriptor = shootDescriptor;
			const auto foliageDescriptor = ProjectManager::CreateTemporaryAsset<FoliageDescriptor>();
			treeDescriptor->m_foliageDescriptor = foliageDescriptor;
		}
		m_treeModel.Initialize(srcSkeleton);
		//TODO: Set up buds here.
	}
}
