#pragma once

#include "TreeVisualizer.hpp"
#include "TreeMeshGenerator.hpp"
#include "LSystemString.hpp"
#include "ParticlePhysics2D.hpp"
#include "PipeModelData.hpp"
#include "PipeModelParameters.hpp"
#include "TreeGraph.hpp"
#include "TreeGrowthParameters.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	class TreePipeNode : public IPrivateComponent
	{
		void PackTask(const PipeModelParameters& pipeModelParameters, bool parallel);
		void MergeTask(const PipeModelParameters& pipeModelParameters);
		void CopyFrontToBackTask();
		void CalculateShiftTask(const PipeModelParameters& pipeModelParameters);
	public:
		GlobalTransform m_desiredGlobalTransform{};


		ParticlePhysics2D<CellParticlePhysicsData> m_frontParticlePhysics2D;
		std::unordered_map<PipeHandle, ParticleHandle> m_frontParticleMap{};

		ParticlePhysics2D<CellParticlePhysicsData> m_backParticlePhysics2D;
		std::unordered_map<PipeHandle, ParticleHandle> m_backParticleMap{};

		float m_frontControlPointDistance = 0.0f;
		float m_backControlPointDistance = 0.0f;

		float m_centerDirectionRadius = 0.0f;

		glm::vec2 m_offset = glm::vec2(0.0f);

		glm::vec2 m_shift = glm::vec2(0.0f);
		bool m_needPacking = false;
		bool m_apical = false;
		bool m_split = false;
		std::vector<std::shared_future<void>> m_tasks{};

		void InsertInterpolation(float a);
		void OnCreate() override;
		void OnDestroy() override;
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;

		void CalculateProfile(const PipeModelParameters &pipeModelParameters, bool scheduling);
		void Wait();
	};
}