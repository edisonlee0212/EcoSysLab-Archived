#include "ParticlePhysics2DDemo.hpp"

#include <Times.hpp>

#include "TreeVisualizer.hpp"
using namespace EcoSysLab;


bool ParticlePhysics2DDemo::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	bool changed = false;
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
			static bool addAttractor = false;
			ImGui::Checkbox("Force resize grid", &m_particlePhysics2D.m_forceResetGrid);
			ImGui::Checkbox("Attractor", &addAttractor);
			ImGui::SameLine();
			if (ImGui::Button("Clear boundaries"))
			{
				m_profileBoundaries.m_boundaries.clear();
				m_boundariesUpdated = true;
			}
			ImGui::SameLine();
			if (ImGui::Button("Clear attractors"))
			{
				m_profileBoundaries.m_attractors.clear();
				m_boundariesUpdated = true;
			}
			ImGui::SameLine();
			static float edgeLengthLimit = 8;
			static bool calculateEdges = false;
			ImGui::Checkbox("Calculate edges", &calculateEdges);
			if(calculateEdges)
			{
				m_particlePhysics2D.CalculateBoundaries(edgeLengthLimit);
			}
			ImGui::DragFloat("Edge length limit", &edgeLengthLimit);
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
						m_particlePhysics2D.RenderEdges(origin, zoomFactor, drawList, IM_COL32(0.0f, 0.0f, 128.0f, 128.0f), 1.0f);
						m_particlePhysics2D.RenderBoundary(origin, zoomFactor, drawList, IM_COL32(255.f, 255.f, 255.0f, 255.0f), 4.0f);
						for (const auto& boundary : m_profileBoundaries.m_boundaries)
						{
							boundary.RenderBoundary(origin, zoomFactor, drawList, IM_COL32(255.0f, 0.0f, 0.0f, 255.0f), 2.0f);
						}
					for(const auto& attractor : m_profileBoundaries.m_attractors)
					{
						attractor.RenderAttractor(origin, zoomFactor, drawList, IM_COL32(0.0f, 255.0f, 0.0f, 255.0f), 2.0f);
					}
					}, showGrid
					);
			static glm::vec2 attractorStartMousePosition;
			if (lastFrameClicked)
			{
				if (mouseDown)
				{
					if (!addAttractor) {
						//Continue recording.
						if (glm::distance(mousePosition, m_profileBoundaries.m_boundaries.back().m_points.back()) > 1.0f) m_profileBoundaries.m_boundaries.back().m_points.emplace_back(mousePosition);
					}
					else
					{
						auto& attractorPoints = m_profileBoundaries.m_attractors.back().m_attractorPoints;
						if (attractorPoints.empty())
						{
							if (glm::distance(attractorStartMousePosition, mousePosition) > 1.0f)
							{
								attractorPoints.emplace_back(attractorStartMousePosition, mousePosition);
							}
						}
						else if (glm::distance(mousePosition, attractorPoints.back().second) > 1.0f)
						{
							attractorPoints.emplace_back(attractorPoints.back().second, mousePosition);
						}
					}
				}
				else if (!m_profileBoundaries.m_boundaries.empty())
				{
					if (!addAttractor)
					{
						//Stop and check boundary.
						if (!m_profileBoundaries.Valid(m_profileBoundaries.m_boundaries.size() - 1))
						{
							m_profileBoundaries.m_boundaries.pop_back();
						}
						else
						{
							m_profileBoundaries.m_boundaries.back().CalculateCenter();
							m_boundariesUpdated = true;
						}
					}
					else
					{
						//Stop and check attractors.
						m_boundariesUpdated = true;
					}
				}
			}
			else if (mouseDown) {
				//Start recording.
				if (!addAttractor) {
					m_profileBoundaries.m_boundaries.emplace_back();
					m_profileBoundaries.m_boundaries.back().m_points.push_back(mousePosition);
				}
				else
				{
					m_profileBoundaries.m_attractors.emplace_back();
					attractorStartMousePosition = mousePosition;
				}
			}
			lastFrameClicked = mouseDown;
		}
		ImGui::End();
	}
	return changed;
}

void ParticlePhysics2DDemo::FixedUpdate()
{
	m_particlePhysics2D.Simulate(Times::TimeStep() / m_particlePhysics2D.GetDeltaTime(),
		[&](auto& grid, bool gridResized)
		{
			if (gridResized || m_boundariesUpdated) grid.ApplyBoundaries(m_profileBoundaries);
			m_boundariesUpdated = false;
		},
		[&](auto& particle)
		{
			//Apply constraints
			auto acceleration = glm::vec2(0.f);
			if (!m_particlePhysics2D.m_particleGrid2D.PeekCells().empty()) {
				const auto& cell = m_particlePhysics2D.m_particleGrid2D.RefCell(particle.GetPosition());
				if (glm::length(cell.m_target) > glm::epsilon<float>()) {
					acceleration += m_gravityStrength * 10.0f * glm::normalize(cell.m_target);
				}
			}
			particle.SetAcceleration(acceleration);
		}
	);
}
