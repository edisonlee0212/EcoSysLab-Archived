#pragma once
#include "PipeModelData.hpp"
#include "PipeModelParameters.hpp"
#include "TreeVisualizer.hpp"
#include "TreeMeshGenerator.hpp"
#include "LSystemString.hpp"
#include "TreePipeMeshGenerator.hpp"
#include "TreePipeNode.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	class TreePipeBase : public IPrivateComponent
	{
		void GatherChildrenEntities(std::vector<Entity>& list) const;
		void ApplyProfile(
			const glm::vec3& globalPosition,
			const glm::quat& globalRotation,
			const ParticlePhysics2D<CellParticlePhysicsData>& profile, const std::unordered_map<PipeHandle, ParticleHandle>& map);
	public:
		bool m_parallelScheduling = true;
		void InstantiateExample();
		ParticlePhysicsSettings m_particlePhysicsSettings{};
		PipeModelPipeGroup m_pipeGroup;
		PipeModelParameters m_pipeModelParameters{};
		GraphAdjustmentSettings m_graphAdjustmentSettings;
		AssetRef m_nodeMaterial{};
		AssetRef m_nodeMesh{};
		float m_profileCalculationTime = 0.0f;
		int m_numOfNodes = 0;
		int m_numOfParticles = 0;
		BaseSkeleton m_skeleton;
		TreePipeMeshGeneratorSettings m_treePipeMeshGeneratorSettings{};
		void InitializeNodes();
		void ClearStrands() const;
		void InitializeStrandRenderer(float frontControlPointRatio, float backControlPointRatio, bool triplePoints, int nodeMaxCount = -1);
		void OnCreate() override;
		void InitializeProfiles();
		void CalculateProfiles();
		void CalculateProfilesV2();
		void AdjustGraph() const;
		void RestoreGraph() const;
		void ApplyProfiles();
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
		void ApplyNodeMeshMaterial();

		std::shared_ptr<Mesh> GenerateMesh(const TreePipeMeshGeneratorSettings& treePipeMeshGeneratorSettings);
		void GenerateGeometry(const TreePipeMeshGeneratorSettings& treePipeMeshGeneratorSettings);
		void ClearGeometry() const;
	};
}
