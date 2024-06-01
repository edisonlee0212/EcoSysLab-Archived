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
#include "BillboardCloud.hpp"

#ifdef BUILD_WITH_PHYSICS
#include "PhysicsLayer.hpp"
#include "RigidBody.hpp"
#endif
using namespace EvoEngine;
namespace EcoSysLab {

	struct BranchPhysicsParameters {
#pragma region Physics
		float m_density = 1.0f;
		float m_linearDamping = 8.0f;
		float m_angularDamping = 8.0f;
		int m_positionSolverIteration = 8;
		int m_velocitySolverIteration = 8;
		float m_jointDriveStiffnessFactor = 3000.0f;
		float m_jointDriveStiffnessThicknessFactor = 40.0f;
		float m_jointDriveDampingFactor = 10.0f;
		float m_jointDriveDampingThicknessFactor = 4.0f;
		bool m_enableAccelerationForDrive = true;
		float m_minimumThickness = 0.01f;
#pragma endregion
		void Serialize(YAML::Emitter& out);

		void Deserialize(const YAML::Node& in);

		template<typename SkeletonData, typename FlowData, typename NodeData>
		void Link(const std::shared_ptr<Scene>& scene,
			const Skeleton<SkeletonData, FlowData, NodeData>& skeleton,
			const std::unordered_map<unsigned, SkeletonFlowHandle>& correspondingFlowHandles,
			const Entity& entity, const Entity& child);
		void OnInspect();
	};

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
		bool m_generateMesh = true;
		float m_lowBranchPruning = 0.f;
		float m_crownShynessDistance = 0.f;
		float m_startTime = 0.f;
		void BuildStrandModel();

		std::shared_ptr<Strands> GenerateStrands() const;
		void GenerateTrunkMeshes(const std::shared_ptr<Mesh>& trunkMesh, const TreeMeshGeneratorSettings& meshGeneratorSettings);
		std::shared_ptr<Mesh> GenerateBranchMesh(const TreeMeshGeneratorSettings& meshGeneratorSettings);
		std::shared_ptr<Mesh> GenerateFoliageMesh(const TreeMeshGeneratorSettings& meshGeneratorSettings);
		std::shared_ptr<ParticleInfoList> GenerateFoliageParticleInfoList(const TreeMeshGeneratorSettings& meshGeneratorSettings);
		std::shared_ptr<Mesh> GenerateStrandModelBranchMesh(const StrandModelMeshGeneratorSettings& strandModelMeshGeneratorSettings);
		std::shared_ptr<Mesh> GenerateStrandModelFoliageMesh(const StrandModelMeshGeneratorSettings& strandModelMeshGeneratorSettings);
		void ExportOBJ(const std::filesystem::path& path, const TreeMeshGeneratorSettings& meshGeneratorSettings);
		void ExportStrandModelOBJ(const std::filesystem::path& path, const StrandModelMeshGeneratorSettings& meshGeneratorSettings);

		void ExportTrunkOBJ(const std::filesystem::path& path, const TreeMeshGeneratorSettings& meshGeneratorSettings);
		bool TryGrow(float deltaTime, bool pruning);

		bool TryGrowSubTree(float deltaTime, SkeletonNodeHandle baseInternodeHandle, bool pruning);
		[[nodiscard]] bool ParseBinvox(const std::filesystem::path& filePath, VoxelGrid<TreeOccupancyGridBasicData>& voxelGrid, float voxelSize = 1.0f);

		void Reset();

		TreeVisualizer m_treeVisualizer{};

		void Serialize(YAML::Emitter& out) const override;
		bool m_splitRootTest = true;
		bool m_recordBiomassHistory = true;
		float m_leftSideBiomass;
		float m_rightSideBiomass;
		TreeMeshGeneratorSettings m_meshGeneratorSettings{};
		StrandModelMeshGeneratorSettings m_strandModelMeshGeneratorSettings{};
		SkeletalGraphSettings m_skeletalGraphSettings{};
		BranchPhysicsParameters m_branchPhysicsParameters{};
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
		void GenerateSkeletalGraph(const SkeletalGraphSettings& skeletalGraphSettings, SkeletonNodeHandle baseNodeHandle, const std::shared_ptr<Mesh>& pointMeshSample, const std::shared_ptr<Mesh>& lineMeshSample) const;

		TreeModel m_treeModel{};
		StrandModel m_strandModel{};
		bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;

		void OnDestroy() override;

		void OnCreate() override;

		void GenerateGeometryEntities(const TreeMeshGeneratorSettings& meshGeneratorSettings, int iteration = -1);
		void ClearGeometryEntities() const;


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

		void GenerateBillboardClouds(const BillboardCloud::GenerateSettings& foliageGenerateSettings);


		void GenerateAnimatedGeometryEntities(const TreeMeshGeneratorSettings& meshGeneratorSettings, int iteration, bool enablePhysics = true);
		void ClearAnimatedGeometryEntities() const;
	};

	template <typename SkeletonData, typename FlowData, typename NodeData>
	void BranchPhysicsParameters::Link(const std::shared_ptr<Scene>& scene,
		const Skeleton<SkeletonData, FlowData, NodeData>& skeleton,
		const std::unordered_map<unsigned, SkeletonFlowHandle>& correspondingFlowHandles, const Entity& entity,
		const Entity& child)
	{
#ifdef BUILD_WITH_PHYSICS
		if (!scene->HasPrivateComponent<RigidBody>(entity))
		{
			scene->RemovePrivateComponent<RigidBody>(child);
			scene->RemovePrivateComponent<Joint>(child);
			return;
		}

		const auto& flow = skeleton.PeekFlow(correspondingFlowHandles.at(child.GetIndex()));

		const float childThickness = flow.m_info.m_startThickness;
		const float childLength = flow.m_info.m_flowLength;

		if (childThickness < m_minimumThickness) return;
		auto rigidBody = scene->GetOrSetPrivateComponent<RigidBody>(child).lock();
		rigidBody->SetEnableGravity(false);
		rigidBody->SetDensityAndMassCenter(m_density *
			childThickness *
			childThickness * childLength);
		rigidBody->SetLinearDamping(m_linearDamping);
		rigidBody->SetAngularDamping(m_angularDamping);
		rigidBody->SetSolverIterations(m_positionSolverIteration,
			m_velocitySolverIteration);
		rigidBody->SetAngularVelocity(glm::vec3(0.0f));
		rigidBody->SetLinearVelocity(glm::vec3(0.0f));

		auto joint = scene->GetOrSetPrivateComponent<Joint>(child).lock();
		joint->Link(entity);
		joint->SetType(JointType::D6);
		joint->SetMotion(MotionAxis::SwingY, MotionType::Free);
		joint->SetMotion(MotionAxis::SwingZ, MotionType::Free);
		joint->SetDrive(DriveType::Swing,
			glm::pow(childThickness,
				m_jointDriveStiffnessThicknessFactor) *
			m_jointDriveStiffnessFactor,
			glm::pow(childThickness,
				m_jointDriveDampingThicknessFactor) *
			m_jointDriveDampingFactor,
			m_enableAccelerationForDrive);
#endif
	}

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
