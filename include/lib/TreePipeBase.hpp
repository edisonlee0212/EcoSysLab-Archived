#pragma once
#include "PipeModelData.hpp"
#include "PipeModelParameters.hpp"
#include "TreeVisualizer.hpp"
#include "TreeMeshGenerator.hpp"
#include "LSystemString.hpp"
#include "TreePipeNode.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	struct GraphAdjustmentSettings
	{
		float m_shiftPushRatio = 1.0f;
		float m_sidePushRatio = 1.0f;
		float m_frontPushRatio = 0.0f;
		float m_rotationPushRatio = 1.0f;
	};

	class TreePipeBase : public IPrivateComponent
	{
		void GatherChildrenEntities(std::vector<Entity>& list) const;
		void ExtendPipesWithProfile(
			const glm::vec3& globalPosition,
			const glm::quat& globalRotation,
			const ParticlePhysics2D<CellParticlePhysicsData>& profile, const std::unordered_map<PipeHandle, ParticleHandle>& map);
	public:
		bool m_parallelScheduling = true;
		void InstantiateExample();
		PipeModelPipeGroup m_pipeGroup;
		PipeModelParameters m_pipeModelParameters{};
		GraphAdjustmentSettings m_graphAdjustmentSettings;
		AssetRef m_nodeMaterial{};
		AssetRef m_nodeMesh{};
		float m_profileCalculationTime = 0.0f;
		int m_numOfProfiles = 0;
		int m_numOfParticles = 0;
		BaseSkeleton m_skeleton;
		void InitializeNodes();
		void ClearStrands() const;
		void InitializeStrandRenderer(float frontControlPointRatio, float backControlPointRatio, int nodeMaxCount = -1);
		void OnCreate() override;
		void CalculateProfiles();
		void AdjustGraph() const;
		void RestoreGraph() const;
		void BuildPipes();
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
	};
}
