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
		ProfileHandle m_startProfileHandle = -1;
		ProfileHandle m_endProfileHandle = -1;
		PipeHandle m_pipeHandle = -1;
		std::unordered_map<PipeHandle, ParticleHandle> m_startParticleMap{};
		std::unordered_map<PipeHandle, ParticleHandle> m_endParticleMap{};

		glm::quat m_startRegulatedRotation{};
		glm::quat m_endRegulatedRotation{};

		glm::vec2 m_offset;

		bool m_apical = false;

		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
	};
}