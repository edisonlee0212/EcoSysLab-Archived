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
	struct TreePipeProfile
	{
		ParticlePhysics2D<CellParticlePhysicsData> m_particlePhysics2D;
		std::unordered_map<PipeHandle, ParticleHandle> m_particleMap{};
		Transform m_profileTransform{};
	};
	class TreePipeNode : public IPrivateComponent
	{
	public:
		std::vector<std::shared_ptr<TreePipeProfile>> m_profiles;
		PipeHandle m_pipeHandle = -1;

		float m_centerDirectionRadius = 0.0f;
		glm::vec2 m_offset = glm::vec2(0.0f);
		
		bool m_apical = false;

		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
	};
}