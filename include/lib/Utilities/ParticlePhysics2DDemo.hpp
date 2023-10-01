#pragma once
#include "ParticlePhysics2D.hpp"

using namespace EvoEngine;
namespace EcoSysLab
{
	struct ParticlePhysicsDemoData
	{
		glm::vec4 m_color = glm::vec4(1.0f);
	};
	class ParticlePhysics2DDemo : public IPrivateComponent
	{
		ParticlePhysics2D<ParticlePhysicsDemoData> m_particlePhysics2D;
	public:
		glm::vec2 m_worldCenter = glm::vec2(0.0f);
		float m_worldRadius = 1.0f;
		glm::vec2 m_gravityDirection = glm::vec2(0, 1);
		float m_gravityStrength = 9.7f;
		float m_friction = 0.01f;
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
		void FixedUpdate() override;
	};
}