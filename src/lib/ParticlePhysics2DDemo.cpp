#include "ParticlePhysics2DDemo.hpp"

#include <Times.hpp>
using namespace EcoSysLab;

void ParticlePhysics2DDemo::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	static bool enableRender = true;
	static float deltaTime = 0.002f;
	ImGui::DragFloat("Simulation Delta time", &deltaTime, 0.001f, 0.001f, 1.0f);
	if (ImGui::Button("Reset"))
	{
		m_particlePhysics2D.Reset(deltaTime);
	}
	ImGui::DragFloat("Particle Softness", &m_particlePhysics2D.m_particleSoftness, 0.001f, 0.001f, 1.0f);
	ImGui::Checkbox("Enable render", &enableRender);
	ImGui::DragFloat2("World center", &m_worldCenter.x, 0.001f);
	ImGui::DragFloat("World radius", &m_worldRadius, 1.0f, 1.0f, 1000.0f);
	ImGui::DragFloat("Gravity strength", &m_gravityStrength, 0.01f);
	ImGui::DragInt("Particle Adding speed", &m_particleAddCount, 1, 1, 1000);
	static float targetDamping = 0.01f;
	ImGui::DragFloat("Target damping", &targetDamping, 0.01f, 0.0f, 1.0f);
	if (ImGui::Button("Apply damping"))
	{
		for (auto& particle : m_particlePhysics2D.RefParticles())
		{
			particle.SetDamping(targetDamping);
		}
	}
	static bool showGrid = false;
	ImGui::Checkbox("Show Grid", &showGrid);
	static float particleInitialSpeed = 1.0f;
	ImGui::DragFloat("Particle Initial speed", &particleInitialSpeed, 0.1f, 0.0f, 3.0f);
	if (enableRender)
	{
		const std::string tag = "ParticlePhysics2D Scene [" + std::to_string(GetOwner().GetIndex()) + "]";
		ImGui::SetNextWindowSize(ImVec2(300, 300), ImGuiCond_Appearing);
		if (ImGui::Begin(tag.c_str()))
		{
			static float elapsedTime = 0.0f;
			elapsedTime += Times::DeltaTime();
			m_particlePhysics2D.OnInspect([&](const glm::vec2 position)
				{
					if (elapsedTime > Times::TimeStep()) {
						elapsedTime = 0.0f;
						for(int i = 0; i < m_particleAddCount; i++)
						{
							const auto particleHandle = m_particlePhysics2D.AllocateParticle();
							auto& particle = m_particlePhysics2D.RefParticle(particleHandle);
							particle.SetColor(glm::vec4(glm::ballRand(1.0f), 1.0f));
							particle.SetPosition(position + glm::circularRand(4.0f));
							particle.SetDamping(targetDamping);
							particle.SetVelocity(glm::vec2(particleInitialSpeed, 0.0f) / static_cast<float>(Times::TimeStep()), m_particlePhysics2D.GetDeltaTime());
						}
					}
				}, [&](const ImVec2 origin, const float zoomFactor, ImDrawList* drawList)
					{
						const auto worldCenter = m_worldCenter * zoomFactor;
						drawList->AddCircle(origin + ImVec2(worldCenter.x, worldCenter.y),
							m_worldRadius * zoomFactor,
							IM_COL32(255,
								0,
								0, 255));
					},
					showGrid
					);
		}
		ImGui::End();
	}
}

void ParticlePhysics2DDemo::FixedUpdate()
{
	m_particlePhysics2D.Simulate(Times::TimeStep() / m_particlePhysics2D.GetDeltaTime(), [&](auto& particle)
		{
			//Apply gravity
			particle.SetPosition(particle.GetPosition() - m_particlePhysics2D.GetMassCenter());

			glm::vec2 acceleration = m_gravityStrength * -glm::normalize(particle.GetPosition());
			
			{
				particle.SetAcceleration(acceleration);
			}
			//Apply constraints
			{
				const auto toCenter = particle.GetPosition() - m_worldCenter;
				const auto distance = glm::length(toCenter);
				if (distance > m_worldRadius - 1.0f)
				{
					const auto n = toCenter / distance;
					particle.Move(m_worldCenter + n * (m_worldRadius - 1.0f));
				}
			}
		}
	);
}
