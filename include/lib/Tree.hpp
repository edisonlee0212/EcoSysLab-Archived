#pragma once
#include "TreeVisualizer.hpp"
#include "TreeMeshGenerator.hpp"
#include "LSystemString.hpp"
#include "RadialBoundingVolume.hpp"
#include "TreeDescriptor.hpp"
#include "TreeGraph.hpp"
#include "TreeGrowthParameters.hpp"
using namespace EvoEngine;
namespace EcoSysLab {
	class Tree : public IPrivateComponent {
		friend class EcoSysLabLayer;
		void PrepareControllers(const std::shared_ptr<TreeDescriptor>& treeDescriptor);
		bool TryGrowSubTree(NodeHandle internodeHandle, float deltaTime);
		ShootGrowthController m_shootGrowthController{};
		RootGrowthController m_rootGrowthController{};
	public:
		static void SerializeTreeGrowthSettings(const TreeGrowthSettings& treeGrowthSettings, YAML::Emitter& out);
		static void DeserializeTreeGrowthSettings(TreeGrowthSettings& treeGrowthSettings, const YAML::Node& param);
		static bool OnInspectTreeGrowthSettings(TreeGrowthSettings& treeGrowthSettings);
		float m_xFrequency = 5.0f;
		float m_yFrequency = 5.0f;
		float m_depth = 0.25f;
		std::shared_ptr<Mesh> GenerateBranchMesh(const TreeMeshGeneratorSettings& meshGeneratorSettings);
		std::shared_ptr<Mesh> GenerateFoliageMesh(const TreeMeshGeneratorSettings& meshGeneratorSettings);
		void ExportOBJ(const std::filesystem::path& path, const TreeMeshGeneratorSettings& meshGeneratorSettings);
		bool TryGrow(float deltaTime);
		[[nodiscard]] bool ParseBinvox(const std::filesystem::path& filePath, VoxelGrid<TreeOccupancyGridBasicData>& voxelGrid, float voxelSize = 1.0f);

		void Reset();

		TreeVisualizer m_treeVisualizer {};
		bool m_enableVisualization = true;

		
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
		
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;

		void OnDestroy() override;

		void OnCreate() override;

		void ClearMeshes() const;
		void ClearStrands() const;
		void GenerateGeometry(const TreeMeshGeneratorSettings& meshGeneratorSettings, int iteration = -1);
		void RegisterVoxel();
		void FromLSystemString(const std::shared_ptr<LSystemString>& lSystemString);
		void FromTreeGraph(const std::shared_ptr<TreeGraph>& treeGraph);
		void FromTreeGraphV2(const std::shared_ptr<TreeGraphV2>& treeGraphV2);

		[[maybe_unused]] bool ExportIOTree(const std::filesystem::path& path) const;
		void ExportRadialBoundingVolume(const std::shared_ptr<RadialBoundingVolume>& rbv) const;
	};
}
