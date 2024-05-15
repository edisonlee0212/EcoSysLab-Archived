#pragma once
#include "Physics2D.hpp"

using namespace EvoEngine;
namespace EcoSysLab
{
	struct Physics2DDemoData
	{
		glm::vec4 m_color = glm::vec4(1.0f);
	};
	class Physics2DDemo : public IPrivateComponent
	{
		Physics2D<Physics2DDemoData> m_physics2D;
	public:
		glm::vec2 m_worldCenter = glm::vec2(0.0f);
		float m_worldRadius = 10.0f;
		glm::vec2 m_gravityDirection = glm::vec2(0, 1);
		float m_gravityStrength = 9.7f;
		float m_friction = 1.0f;
		bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
		void FixedUpdate() override;
	};
}