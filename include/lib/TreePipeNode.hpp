#pragma once

#include "TreePipeModel.hpp"
#include "TreeVisualizer.hpp"
#include "TreeMeshGenerator.hpp"
#include "LSystemString.hpp"
#include "TreeGraph.hpp"
#include "TreeGrowthParameters.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	
	class TreePipeNode : public IPrivateComponent
	{
	public:
		ParticlePhysics2D<CellParticlePhysicsData> m_startParticlePhysics2D;
		ParticlePhysics2D<CellParticlePhysicsData> m_endParticlePhysics2D;
		PipeHandle m_pipeHandle = -1;
		std::unordered_map<PipeHandle, ParticleHandle> m_startParticleMap{};
		std::unordered_map<PipeHandle, ParticleHandle> m_endParticleMap{};

		glm::quat m_startRegulatedRotation{};
		glm::quat m_endRegulatedRotation{};

		float m_centerDirectionRadius = 0.0f;
		glm::vec2 m_offset = glm::vec2(0.0f);
		
		bool m_apical = false;

		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
	};
}