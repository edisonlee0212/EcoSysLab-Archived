#include "Physics2DDemo.hpp"

#include <Times.hpp>
using namespace EcoSysLab;

void Physics2DDemo::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	static bool enableRender = true;
	if(ImGui::Button("Reset"))
	{
		m_physics2D = {};
	}
	ImGui::Checkbox("Enable render", &enableRender);
	ImGui::DragFloat2("World center", &m_worldCenter.x, 0.01f);
	ImGui::DragFloat("World radius", &m_worldRadius, 0.01f);
	ImGui::DragFloat("Gravity strength", &m_gravityStrength, 0.01f);
	ImGui::DragFloat("Friction", &m_friction, 0.1f);
	static float targetDamping = 0.1f;
	ImGui::DragFloat("Target damping", &targetDamping, 0.01f);
	if(ImGui::Button("Apply damping"))
	{
		for(auto& particle : m_physics2D.RefParticles())
		{
			particle.SetDamping(targetDamping);
		}
	}
	if(enableRender)
	{
		const std::string tag = "Physics2D Scene [" + std::to_string(GetOwner().GetIndex()) + "]";
		if (ImGui::Begin(tag.c_str()))
		{
			m_physics2D.OnInspect([&](glm::vec2 position)
				{
					const auto particleHandle = m_physics2D.AllocateParticle();
					auto& particle = m_physics2D.RefParticle(particleHandle);
					particle.SetColor(glm::vec4(glm::abs(glm::ballRand(1.0f)), 1.0f));
					particle.SetRadius(glm::linearRand(0.1f, 3.0f));
					particle.SetPosition(position);
				}, [&](const ImVec2 origin, const float zoomFactor, ImDrawList* drawList)
				{
						const auto worldCenter = m_worldCenter * zoomFactor;
						drawList->AddCircle(origin + ImVec2(worldCenter.x, worldCenter.y),
						m_worldRadius * zoomFactor,
						IM_COL32(255,
						0,
						0, 255));
				}
			);
		}
		ImGui::End();
	}
}

void Physics2DDemo::FixedUpdate()
{
	const auto gravity = m_gravityDirection * m_gravityStrength;
	m_physics2D.Simulate(Times::FixedDeltaTime(), [&](auto& particle)
		{
			//Apply gravity
			glm::vec2 acceleration = gravity;
			auto friction = -glm::normalize(particle.GetVelocity()) * m_friction;
			if(!glm::any(glm::isnan(friction)))
			{
				acceleration += friction;
			}
			{
				particle.SetAcceleration(acceleration);
			}
			//Apply constraints
			{
				const auto toCenter = particle.GetPosition() - m_worldCenter;
				const auto distance = glm::length(toCenter);
				if(distance > m_worldRadius - particle.GetRadius())
				{
					const auto n = toCenter / distance;
					particle.Move(m_worldCenter + n * (m_worldRadius - particle.GetRadius()));
				}
			}
		}
	);
}
