#include "ParticlePhysics2DDemo.hpp"

#include <Times.hpp>

#include "TreeVisualizer.hpp"
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
	ImGui::DragFloat("Particle Softness", &m_particlePhysics2D.m_settings.m_particleSoftness, 0.001f, 0.001f, 1.0f);
	ImGui::Checkbox("Enable render", &enableRender);
	ImGui::DragFloat2("World center", &m_worldCenter.x, 0.001f);
	ImGui::DragFloat("World radius", &m_worldRadius, 1.0f, 1.0f, 1000.0f);
	ImGui::DragFloat("Gravity strength", &m_gravityStrength, 0.01f);
	ImGui::DragInt("Particle Adding speed", &m_particleAddCount, 1, 1, 1000);
	ImGui::DragFloat("Target damping", &m_particlePhysics2D.m_settings.m_damping, 0.01f, 0.0f, 1.0f);
	ImGui::DragFloat("Max Velocity", &m_particlePhysics2D.m_settings.m_maxSpeed, 0.01f, 0.0f, 1.0f);
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
			glm::vec2 mousePosition{};
			static bool lastFrameClicked = false;
			bool mouseDown = false;
			if (ImGui::Button("Clear stroke"))
			{
				m_userBoundaries.clear();
			}

			static float elapsedTime = 0.0f;
			elapsedTime += Times::DeltaTime();
			m_particlePhysics2D.OnInspect([&](const glm::vec2 position)
				{
					if (editorLayer->GetKey(GLFW_KEY_LEFT_CONTROL) == KeyActionType::Press || editorLayer->GetKey(GLFW_KEY_LEFT_CONTROL) == KeyActionType::Hold)
					{
						if (elapsedTime > Times::TimeStep()) {
							elapsedTime = 0.0f;
							for (int i = 0; i < m_particleAddCount; i++)
							{
								const auto particleHandle = m_particlePhysics2D.AllocateParticle();
								auto& particle = m_particlePhysics2D.RefParticle(particleHandle);
								particle.SetColor(glm::vec4(glm::ballRand(1.0f), 1.0f));
								particle.SetPosition(position + glm::circularRand(4.0f));
								particle.SetVelocity(glm::vec2(particleInitialSpeed, 0.0f) / static_cast<float>(Times::TimeStep()), m_particlePhysics2D.GetDeltaTime());
							}
						}
					}
					else {
						mouseDown = true;
						mousePosition = position;
					}
				}, [&](const ImVec2 origin, const float zoomFactor, ImDrawList* drawList)
					{
						const auto worldCenter = m_worldCenter * zoomFactor;
						drawList->AddCircle(origin + ImVec2(worldCenter.x, worldCenter.y),
							m_worldRadius * zoomFactor,
							IM_COL32(255,
								0,
								0, 255));

						for (const auto& userBoundary : m_userBoundaries)
						{
							if (userBoundary.size() > 2) {
								for (int pointIndex = 0; pointIndex < userBoundary.size() - 1; pointIndex++)
								{
									const auto& p1 = userBoundary[pointIndex];
									const auto& p2 = userBoundary[pointIndex + 1];
									drawList->AddLine(ImVec2(origin.x + p1.x * zoomFactor,
										origin.y + p1.y * zoomFactor), ImVec2(origin.x + p2.x * zoomFactor,
											origin.y + p2.y * zoomFactor), IM_COL32(255.0f, 255.0f, 255.0f, 255.0f));
								}

								const auto& p1 = userBoundary.back();
								const auto& p2 = userBoundary[0];
								drawList->AddLine(ImVec2(origin.x + p1.x * zoomFactor,
									origin.y + p1.y * zoomFactor), ImVec2(origin.x + p2.x * zoomFactor,
										origin.y + p2.y * zoomFactor), IM_COL32(255.0f, 255.0f, 255.0f, 255.0f));
							}
						}
					}, showGrid
					);
			if (lastFrameClicked)
			{
				if (mouseDown)
				{
					//Continue recording.
					if (mousePosition != m_userBoundaries.back().back()) m_userBoundaries.back().emplace_back(mousePosition);
				}
				else
				{
					//Stop and check boundary.
					bool valid = true;
					const auto& userBoundary = m_userBoundaries.back();
					if (userBoundary.size() <= 3) valid = false;
					if (valid)
					{
						for (int lineIndex = 0; lineIndex < userBoundary.size(); lineIndex++)
						{
							const auto& p1 = userBoundary[lineIndex];
							const auto& p2 = userBoundary[(lineIndex + 1) % userBoundary.size()];
							for (int lineIndex2 = 0; lineIndex2 < userBoundary.size(); lineIndex2++)
							{
								if (lineIndex == lineIndex2) continue;
								if ((lineIndex + 1) % userBoundary.size() == lineIndex2
									|| (lineIndex2 + 1) % userBoundary.size() == lineIndex) continue;
								const auto& p3 = userBoundary[lineIndex2];
								const auto& p4 = userBoundary[(lineIndex2 + 1) % userBoundary.size()];
								if (TreeVisualizer::intersect(p1, p2, p3, p4))
								{
									valid = false;
									break;
								}
							}
							if (!valid)break;
						}
					}
					if (!valid)
					{
						m_userBoundaries.pop_back();
					}
				}
			}
			else if (mouseDown) {
				//Start recording.
				m_userBoundaries.emplace_back();
				m_userBoundaries.back().push_back(mousePosition);
			}
			lastFrameClicked = mouseDown;
		}
		ImGui::End();
	}
}

void ParticlePhysics2DDemo::FixedUpdate()
{
	m_particlePhysics2D.Simulate(Times::TimeStep() / m_particlePhysics2D.GetDeltaTime(),
		[&](auto& grid, bool gridResized)
		{
			grid.TestBoundaries(m_userBoundaries);
		},
		[&](auto& particle)
		{
			//Apply gravity
			particle.SetPosition(particle.GetPosition() - m_particlePhysics2D.GetMassCenter());
			{
				glm::vec2 acceleration = m_gravityStrength * -glm::normalize(particle.GetPosition());
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
			if (!m_particlePhysics2D.m_particleGrid2D.PeekCells().empty()) {
				const auto& cell = m_particlePhysics2D.m_particleGrid2D.RefCell(particle.GetPosition());
				if (!cell.m_inBoundary)
				{
					const auto direction = cell.m_closestPoint - particle.GetPosition();
					particle.Move(cell.m_closestPoint - glm::normalize(direction) * 0.1f);
				}
			}
		}
	);
}
