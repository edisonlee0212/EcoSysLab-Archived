#pragma once
#include "Particle2D.hpp"
#include "ParticleGrid2D.hpp"
#include "Times.hpp"
#include "Delaunator2D.hpp"
using namespace EvoEngine;
namespace EcoSysLab {
	struct ParticlePhysicsSettings
	{
		float m_particleSoftness = 0.1f;
		float m_damping = 0.02f;
		float m_maxSpeed = 60.0f;
	};

	template<typename ParticleData>
	class StrandModelProfile
	{
		template<typename PD>
		friend class StrandModelProfileSerializer;

		std::vector<Particle2D<ParticleData>> m_particles2D{};
		void SolveCollision(ParticleHandle p1Handle, ParticleHandle p2Handle);
		float m_deltaTime = 0.001f;
		void Update(
			const std::function<void(ParticleGrid2D& grid, bool gridResized)>& modifyGridFunc,
			const std::function<void(Particle2D<ParticleData>& particle)>& modifyParticleFunc);
		void CheckCollisions(
			const std::function<void(ParticleGrid2D& grid, bool gridResized)>& modifyGridFunc);
		glm::vec2 m_min = glm::vec2(FLT_MAX);
		glm::vec2 m_max = glm::vec2(FLT_MIN);
		float m_maxDistanceToCenter = 0.0f;
		glm::vec2 m_massCenter = glm::vec2(0.0f);
		double m_simulationTime = 0.0f;

		std::vector<std::pair<int, int>> m_edges{};
		std::vector<std::pair<int, int>> m_boundaryEdges{};
		std::vector<glm::ivec3> m_triangles{};
	public:
		[[nodiscard]] const std::vector<std::pair<int, int>>& PeekBoundaryEdges() const;
		[[nodiscard]] const std::vector<glm::ivec3>& PeekTriangles() const;
		[[nodiscard]] const std::vector<std::pair<int, int>>& PeekEdges() const;
		void RenderEdges(ImVec2 origin, float zoomFactor, ImDrawList* drawList, ImU32 color, float thickness);
		void RenderBoundary(ImVec2 origin, float zoomFactor, ImDrawList* drawList, ImU32 color, float thickness);
		void CalculateBoundaries(bool calculateBoundaryDistance, float removalLength = 8);
		ParticleGrid2D m_particleGrid2D{};
		bool m_parallel = false;
		bool m_forceResetGrid = false;
		[[nodiscard]] float GetDistanceToOrigin(const glm::vec2& direction, const glm::vec2& origin) const;
		[[nodiscard]] float GetDeltaTime() const;
		void SetEnableAllParticles(bool value);
		void Reset(float deltaTime = 0.002f);
		void CalculateMinMax();
		ParticlePhysicsSettings m_settings{};
		[[nodiscard]] ParticleHandle AllocateParticle();
		[[nodiscard]] Particle2D<ParticleData>& RefParticle(ParticleHandle handle);
		[[nodiscard]] const Particle2D<ParticleData>& PeekParticle(ParticleHandle handle) const;
		void RemoveParticle(ParticleHandle handle);
		void Shift(const glm::vec2& offset);
		[[nodiscard]] const std::vector<Particle2D<ParticleData>>& PeekParticles() const;
		[[nodiscard]] std::vector<Particle2D<ParticleData>>& RefParticles();
		void SimulateByTime(float time,
			const std::function<void(ParticleGrid2D& grid, bool gridResized)>& modifyGridFunc,
			const std::function<void(Particle2D<ParticleData>& particle)>& modifyParticleFunc);
		void Simulate(size_t iterations,
			const std::function<void(ParticleGrid2D& grid, bool gridResized)>& modifyGridFunc,
			const std::function<void(Particle2D<ParticleData>& particle)>& modifyParticleFunc);
		[[nodiscard]] glm::vec2 GetMassCenter() const;
		[[nodiscard]] float GetMaxDistanceToCenter() const;
		[[nodiscard]] glm::vec2 FindAvailablePosition(const glm::vec2& direction);
		[[nodiscard]] glm::vec2 CircularFindPosition(int index) const;
		[[nodiscard]] double GetLastSimulationTime() const;
		void OnInspect(const std::function<void(glm::vec2 position)>& func, const std::function<void(ImVec2 origin, float zoomFactor, ImDrawList*)>& drawFunc, bool showGrid = false);
	};

	template <typename T>
	void StrandModelProfile<T>::SolveCollision(ParticleHandle p1Handle, ParticleHandle p2Handle)
	{
		auto& p1 = m_particles2D.at(p1Handle);
		const auto& p2 = m_particles2D.at(p2Handle);
		if (!p1.m_enable) return;
		if (!p2.m_enable) return;

		const auto difference = p1.m_position - p2.m_position;
		const auto distance = glm::length(difference);
		if (distance < 2.0f)
		{
			glm::vec2 axis;
			if (distance < glm::epsilon<float>())
			{
				const auto dir = glm::circularRand(1.0f);
				if (p1Handle >= p2Handle)
				{
					axis = dir;
				}
				else
				{
					axis = -dir;
				}
			}
			else
			{
				axis = difference / distance;
			}
			const auto delta = 2.0f - distance;
			p1.m_deltaPosition += (1.0f - m_settings.m_particleSoftness) * 0.5f * delta * axis;
		}
	}

	template <typename T>
	void StrandModelProfile<T>::Update(
		const std::function<void(ParticleGrid2D& grid, bool gridResized)>& modifyGridFunc,
		const std::function<void(Particle2D<T>& collisionParticle)>& modifyParticleFunc)
	{
		if (m_particles2D.empty()) return;
		const auto startTime = Times::Now();
		if (m_parallel) {
			Jobs::ParallelFor(m_particles2D.size(), [&](unsigned i)
				{
					auto& particle = m_particles2D[i];
					if (particle.m_enable) modifyParticleFunc(particle);
					particle.m_deltaPosition = glm::vec2(0);
				}
			);
		}
		else
		{
			for (auto& particle : m_particles2D)
			{
				if (particle.m_enable) modifyParticleFunc(particle);
				particle.m_deltaPosition = glm::vec2(0);
			}
		}

		CheckCollisions(modifyGridFunc);

		if (m_parallel) {
			Jobs::ParallelFor(m_particles2D.size(), [&](unsigned i)
				{
					auto& particle = m_particles2D[i];
					if (particle.m_enable) particle.m_position += particle.m_deltaPosition;
					UpdateSettings updateSettings;
					updateSettings.m_dt = m_deltaTime;
					updateSettings.m_maxVelocity = m_settings.m_maxSpeed;
					updateSettings.m_damping = m_settings.m_damping;
					particle.Update(updateSettings);
				}
			);
		}
		else {
			for (auto& particle : m_particles2D)
			{
				if (particle.m_enable) particle.m_position += particle.m_deltaPosition;
				UpdateSettings updateSettings{};
				updateSettings.m_dt = m_deltaTime;
				updateSettings.m_maxVelocity = m_settings.m_maxSpeed;
				updateSettings.m_damping = m_settings.m_damping;
				particle.Update(updateSettings);
			}
		}
		m_simulationTime = Times::Now() - startTime;
	}

	template <typename T>
	void StrandModelProfile<T>::CalculateMinMax()
	{
		m_min = glm::vec2(FLT_MAX);
		m_max = glm::vec2(FLT_MIN);
		m_massCenter = glm::vec2(0.0f);
		m_maxDistanceToCenter = 0.0f;
		if (m_parallel) {
			const auto threadCount = Jobs::Workers().Size();
			std::vector<glm::vec2> mins{};
			std::vector<glm::vec2> maxs{};
			std::vector<glm::vec2> massCenters{};
			std::vector<float> maxDistances{};
			std::vector<int> enabledParticleSizes{};
			mins.resize(threadCount);
			maxs.resize(threadCount);
			massCenters.resize(threadCount);
			maxDistances.resize(threadCount);
			enabledParticleSizes.resize(threadCount);
			for (int i = 0; i < threadCount; i++)
			{
				mins[i] = glm::vec2(FLT_MAX);
				maxs[i] = glm::vec2(FLT_MIN);
				massCenters[i] = glm::vec2(0.0f);
				maxDistances[i] = 0.0f;
				enabledParticleSizes[i] = 0;
			}
			Jobs::ParallelFor(m_particles2D.size(), [&](const unsigned i, const unsigned threadIndex)
				{
					const auto& particle = m_particles2D[i];
					mins[threadIndex] = glm::min(mins[threadIndex], particle.m_position);
					maxs[threadIndex] = glm::max(maxs[threadIndex], particle.m_position);
					if (particle.m_enable) {
						enabledParticleSizes[threadIndex]++;
						massCenters[threadIndex] += particle.m_position;
						maxDistances[threadIndex] = glm::max(maxDistances[threadIndex], glm::length(particle.m_position));
					}
				}
			);
			int enabledParticleSize = 0;
			for (int i = 0; i < threadCount; i++)
			{
				m_min = glm::min(mins[i], m_min);
				m_max = glm::max(maxs[i], m_max);
				enabledParticleSize += enabledParticleSizes[i];
				m_massCenter += massCenters[i];
				m_maxDistanceToCenter = glm::max(maxDistances[i], m_maxDistanceToCenter);
			}
			m_massCenter /= enabledParticleSize;
		}
		else
		{
			int enabledParticleSize = 0;
			for (const auto& particle : m_particles2D)
			{
				m_min = glm::min(particle.m_position, m_min);
				m_max = glm::max(particle.m_position, m_max);
				if (particle.m_enable) {
					enabledParticleSize++;
					m_massCenter += particle.m_position;
					m_maxDistanceToCenter = glm::max(m_maxDistanceToCenter, glm::length(particle.m_position));
				}
			}
			m_massCenter /= enabledParticleSize;
		}
	}

	template <typename T>
	void StrandModelProfile<T>::CheckCollisions(
		const std::function<void(ParticleGrid2D& grid, bool gridResized)>& modifyGridFunc)
	{
		CalculateMinMax();

		if (m_min.x < m_particleGrid2D.m_minBound.x || m_min.y < m_particleGrid2D.m_minBound.y || m_max.x > m_particleGrid2D.m_maxBound.x || m_max.y > m_particleGrid2D.m_maxBound.y || m_forceResetGrid) {
			m_particleGrid2D.Reset(2.0f, m_min - glm::vec2(2.0f), m_max + glm::vec2(2.0f));
			modifyGridFunc(m_particleGrid2D, true);
		}
		else
		{
			m_particleGrid2D.Clear();
			modifyGridFunc(m_particleGrid2D, false);
		}
		for (ParticleHandle i = 0; i < m_particles2D.size(); i++)
		{
			const auto& particle = m_particles2D[i];
			if (particle.m_enable) m_particleGrid2D.RegisterParticle(particle.m_position, i);
		}
		if (m_parallel) {
			Jobs::ParallelFor(m_particles2D.size(), [&](unsigned i) {
				const ParticleHandle particleHandle = i;
				const auto& particle = m_particles2D[particleHandle];
				if (!particle.m_enable) return;
				const auto& coordinate = m_particleGrid2D.GetCoordinate(particle.m_position);
				for (int dx = -1; dx <= 1; dx++)
				{
					for (int dy = -1; dy <= 1; dy++)
					{
						const auto x = coordinate.x + dx;
						const auto y = coordinate.y + dy;
						if (x < 0) continue;
						if (y < 0) continue;
						if (x >= m_particleGrid2D.m_resolution.x) continue;
						if (y >= m_particleGrid2D.m_resolution.y) continue;
						const auto& cell = m_particleGrid2D.RefCell(glm::ivec2(x, y));
						for (int i = 0; i < cell.m_atomCount; i++)
						{
							if (const auto particleHandle2 = cell.m_atomHandles[i]; particleHandle != particleHandle2) {
								SolveCollision(particleHandle, particleHandle2);
							}
						}
					}
				}
				});
		}
		else
		{
			for (ParticleHandle particleHandle = 0; particleHandle < m_particles2D.size(); particleHandle++)
			{
				const auto& particle = m_particles2D[particleHandle];
				if (!particle.m_enable) continue;
				const auto& coordinate = m_particleGrid2D.GetCoordinate(particle.m_position);
				for (int dx = -1; dx <= 1; dx++)
				{
					for (int dy = -1; dy <= 1; dy++)
					{
						const auto x = coordinate.x + dx;
						const auto y = coordinate.y + dy;
						if (x < 0) continue;
						if (y < 0) continue;
						if (x >= m_particleGrid2D.m_resolution.x) continue;
						if (y >= m_particleGrid2D.m_resolution.y) continue;
						const auto& cell = m_particleGrid2D.RefCell(glm::ivec2(x, y));
						for (int i = 0; i < cell.m_atomCount; i++)
						{
							if (const auto particleHandle2 = cell.m_atomHandles[i]; particleHandle != particleHandle2) {
								SolveCollision(particleHandle, particleHandle2);
							}
						}
					}
				}
			}
		}
	}

	template <typename ParticleData>
	const std::vector<std::pair<int, int>>& StrandModelProfile<ParticleData>::PeekBoundaryEdges() const
	{
		return m_boundaryEdges;
	}

	template <typename ParticleData>
	const std::vector<glm::ivec3>& StrandModelProfile<ParticleData>::PeekTriangles() const
	{
		return m_triangles;
	}

	template <typename ParticleData>
	const std::vector<std::pair<int, int>>& StrandModelProfile<ParticleData>::PeekEdges() const
	{
		return m_edges;
	}

	template <typename ParticleData>
	void StrandModelProfile<ParticleData>::RenderEdges(ImVec2 origin, float zoomFactor, ImDrawList* drawList,
		ImU32 color, float thickness)
	{
		if (m_edges.empty()) return;
		for (const auto& edge : m_edges)
		{
			const auto& p1 = m_particles2D[edge.first].m_position;
			const auto& p2 = m_particles2D[edge.second].m_position;
			drawList->AddLine(ImVec2(origin.x + p1.x * zoomFactor,
				origin.y + p1.y * zoomFactor), ImVec2(origin.x + p2.x * zoomFactor,
					origin.y + p2.y * zoomFactor), color, thickness);
		}
	}

	template <typename ParticleData>
	void StrandModelProfile<ParticleData>::RenderBoundary(ImVec2 origin, float zoomFactor, ImDrawList* drawList,
		ImU32 color, float thickness)
	{
		if (m_boundaryEdges.empty()) return;
		for (const auto& edge : m_boundaryEdges)
		{
			const auto& p1 = m_particles2D[edge.first].m_position;
			const auto& p2 = m_particles2D[edge.second].m_position;
			drawList->AddLine(ImVec2(origin.x + p1.x * zoomFactor,
				origin.y + p1.y * zoomFactor), ImVec2(origin.x + p2.x * zoomFactor,
					origin.y + p2.y * zoomFactor), color, thickness);
		}
	}

	template <typename ParticleData>
	void StrandModelProfile<ParticleData>::CalculateBoundaries(const bool calculateBoundaryDistance, const float removalLength)
	{
		m_edges.clear();
		m_boundaryEdges.clear();
		m_triangles.clear();

		if(m_particles2D.size() < 3)
		{
			for (int i = 0; i < m_particles2D.size(); i++)
			{
				m_particles2D[i].m_boundary = true;
				m_particles2D[i].m_distanceToBoundary = 0;
			}
			return;
		}
		std::vector<float> positions(m_particles2D.size() * 2);
		for (int i = 0; i < m_particles2D.size(); i++)
		{
			positions[2 * i] = m_particles2D[i].m_position.x;
			positions[2 * i + 1] = m_particles2D[i].m_position.y;
			m_particles2D[i].m_boundary = false;
			m_particles2D[i].m_distanceToBoundary = 0;
		}
		const Delaunator::Delaunator2D d(positions);
		std::map<std::pair<int, int>, int> edges;
		for (std::size_t i = 0; i < d.triangles.size(); i += 3) {
			const auto& v0 = d.triangles[i];
			const auto& v1 = d.triangles[i + 1];
			const auto& v2 = d.triangles[i + 2];
			if (glm::distance(m_particles2D[v0].m_position, m_particles2D[v1].m_position) > removalLength
				|| glm::distance(m_particles2D[v1].m_position, m_particles2D[v2].m_position) > removalLength
				|| glm::distance(m_particles2D[v0].m_position, m_particles2D[v2].m_position) > removalLength) continue;
			++edges[std::make_pair(glm::min(v0, v1), glm::max(v0, v1))];
			++edges[std::make_pair(glm::min(v1, v2), glm::max(v1, v2))];
			++edges[std::make_pair(glm::min(v0, v2), glm::max(v0, v2))];
			m_triangles.emplace_back(glm::ivec3( v0, v1, v2 ));
		}
		std::set<int> boundaryVertices;
		for (const auto& edge : edges)
		{
			if (edge.second == 1)
			{
				m_boundaryEdges.emplace_back(edge.first);
				m_particles2D[edge.first.first].m_boundary = true;
				m_particles2D[edge.first.second].m_boundary = true;
				boundaryVertices.emplace(edge.first.first);
				boundaryVertices.emplace(edge.first.second);
			}
			m_edges.emplace_back(edge.first);
		}

		if(calculateBoundaryDistance)
		{
			for (auto& particle : m_particles2D) {
				if(particle.m_boundary || boundaryVertices.empty())
				{
					particle.m_distanceToBoundary = 0.0f;
					continue;
				}
				particle.m_distanceToBoundary = FLT_MAX;
				const auto position = particle.GetPosition();
				for (const auto& boundaryParticleHandle : boundaryVertices)
				{
					const auto& boundaryParticle = m_particles2D[boundaryParticleHandle];
					const auto currentDistance = glm::distance(position, boundaryParticle.GetPosition());
					if(particle.m_distanceToBoundary > currentDistance)
					{
						particle.m_distanceToBoundary = currentDistance;
					}
				}
			}
		}
	}

	template <typename T>
	float StrandModelProfile<T>::GetDistanceToOrigin(const glm::vec2& direction, const glm::vec2& origin) const
	{
		float maxDistance = FLT_MIN;
		if (m_parallel) {
			const auto threadCount = Jobs::Workers().Size();
			std::vector<float> maxDistances;
			maxDistances.resize(threadCount);
			for (int i = 0; i < threadCount; i++)
			{
				maxDistances[i] = FLT_MIN;
			}
			Jobs::ParallelFor(m_particles2D.size(), [&](const unsigned i, const unsigned threadIndex)
				{
					const auto& particle = m_particles2D[i];
					if (!particle.m_enable) return;
					const auto distance = glm::length(glm::closestPointOnLine(particle.m_position, glm::vec2(origin), origin + direction * 1000.0f));
					maxDistances[threadIndex] = glm::max(maxDistances[threadIndex], distance);
				});

			for (int i = 0; i < threadCount; i++)
			{
				maxDistance = glm::max(maxDistances[i], maxDistance);
			}
		}
		else
		{
			for (const auto& particle : m_particles2D)
			{
				const auto distance = glm::length(glm::closestPointOnLine(particle.m_position, glm::vec2(origin), origin + direction * 1000.0f));
				maxDistance = glm::max(maxDistance, distance);
			}
		}
		return maxDistance;
	}

	template <typename T>
	float StrandModelProfile<T>::GetDeltaTime() const
	{
		return m_deltaTime;
	}

	template <typename ParticleData>
	void StrandModelProfile<ParticleData>::SetEnableAllParticles(const bool value)
	{
		for (auto& particle : m_particles2D)
		{
			particle.m_enable = value;
		}
	}

	template <typename T>
	void StrandModelProfile<T>::Reset(const float deltaTime)
	{
		m_deltaTime = deltaTime;
		m_particles2D.clear();
		m_particleGrid2D = {};
		m_massCenter = glm::vec2(0.0f);
		m_min = glm::vec2(FLT_MAX);
		m_max = glm::vec2(FLT_MIN);
		m_maxDistanceToCenter = 0.0f;
		m_massCenter = glm::vec2(0.0f);
		m_simulationTime = 0.0f;
	}


	template <typename T>
	ParticleHandle StrandModelProfile<T>::AllocateParticle()
	{
		m_particles2D.emplace_back();
		auto& newParticle = m_particles2D.back();
		newParticle = {};
		newParticle.m_handle = m_particles2D.size() - 1;
		return newParticle.m_handle;
	}

	template <typename T>
	Particle2D<T>& StrandModelProfile<T>::RefParticle(ParticleHandle handle)
	{
		return m_particles2D[handle];
	}

	template <typename T>
	const Particle2D<T>& StrandModelProfile<T>::PeekParticle(ParticleHandle handle) const
	{
		return m_particles2D[handle];
	}

	template <typename T>
	void StrandModelProfile<T>::RemoveParticle(ParticleHandle handle)
	{
		m_particles2D[handle] = m_particles2D.back();
		m_particles2D.pop_back();
	}

	template <typename T>
	void StrandModelProfile<T>::Shift(const glm::vec2& offset)
	{
		Jobs::ParallelFor(m_particles2D.size(), [&](unsigned i)
			{
				auto& particle = m_particles2D[i];
				particle.SetPosition(particle.m_position + offset);
			}
		);
	}

	template <typename T>
	const std::vector<Particle2D<T>>& StrandModelProfile<T>::PeekParticles() const
	{
		return m_particles2D;
	}

	template <typename T>
	std::vector<Particle2D<T>>& StrandModelProfile<T>::RefParticles()
	{
		return m_particles2D;
	}

	template <typename T>
	void StrandModelProfile<T>::SimulateByTime(float time,
		const std::function<void(ParticleGrid2D& grid, bool gridResized)>& modifyGridFunc,
		const std::function<void(Particle2D<T>& collisionParticle)>& modifyParticleFunc)
	{
		const auto count = static_cast<size_t>(glm::round(time / m_deltaTime));
		for (int i = 0; i < count; i++)
		{
			Update(modifyGridFunc, modifyParticleFunc);
		}
	}

	template <typename T>
	void StrandModelProfile<T>::Simulate(const size_t iterations,
		const std::function<void(ParticleGrid2D& grid, bool gridResized)>& modifyGridFunc,
		const std::function<void(Particle2D<T>& particle)>& modifyParticleFunc)
	{
		for (int i = 0; i < iterations; i++)
		{
			Update(modifyGridFunc, modifyParticleFunc);
		}
	}

	template <typename T>
	glm::vec2 StrandModelProfile<T>::GetMassCenter() const
	{
		return m_massCenter;
	}

	template <typename ParticleData>
	float StrandModelProfile<ParticleData>::GetMaxDistanceToCenter() const
	{
		return m_maxDistanceToCenter;
	}

	template <typename ParticleData>
	glm::vec2 StrandModelProfile<ParticleData>::FindAvailablePosition(const glm::vec2& direction)
	{
		auto retVal = glm::vec2(0, 0);
		bool found = false;
		while (!found)
		{
			found = true;
			for (const auto& i : m_particles2D)
			{
				if (glm::distance(i.GetPosition(), retVal) < 2.05f)
				{
					found = false;
					break;
				}
			}
			if (!found) retVal += direction * 0.41f;
		}
		return retVal;
	}

	template <typename ParticleData>
	glm::vec2 StrandModelProfile<ParticleData>::CircularFindPosition(int index) const
	{
		if (index == 0) return glm::vec2(0.0f);
		int layer = 0;
		while(3 * layer * (layer + 1) <= index)
		{
			layer++;
		}
		const int edgeSize = layer;
		const int layerIndex = (index - 1) - 3 * (layer - 1) * layer;
		const int edgeIndex = layerIndex / edgeSize;
		const int indexInEdge = layerIndex % edgeSize;
		const glm::vec2 edgeDirection = glm::vec2(glm::cos(glm::radians(edgeIndex * 60.0f)), glm::sin(glm::radians(edgeIndex * 60.0f)));
		const glm::vec2 walkerDirection = glm::vec2(glm::cos(glm::radians((edgeIndex + 2) * 60.0f)), glm::sin(glm::radians((edgeIndex + 2) * 60.0f)));

		return edgeDirection * static_cast<float>(layer) * 2.0f + walkerDirection * static_cast<float>(indexInEdge) * 2.0f;
	}

	template <typename ParticleData>
	double StrandModelProfile<ParticleData>::GetLastSimulationTime() const
	{
		return m_simulationTime;
	}

	template <typename T>
	void StrandModelProfile<T>::OnInspect(const std::function<void(glm::vec2 position)>& func, const std::function<void(ImVec2 origin, float zoomFactor, ImDrawList*)>& drawFunc, bool showGrid)
	{
		static auto scrolling = glm::vec2(0.0f);
		static float zoomFactor = 5.f;
		ImGui::Text(("Particle count: " + std::to_string(m_particles2D.size()) +
			" | Simulation time: " + std::to_string(m_simulationTime)).c_str());
		if (ImGui::Button("Recenter")) {
			scrolling = glm::vec2(0.0f);
		}
		ImGui::SameLine();
		ImGui::DragFloat("Zoom", &zoomFactor, zoomFactor / 100.0f, 0.1f, 1000.0f);
		zoomFactor = glm::clamp(zoomFactor, 0.01f, 1000.0f);
		const ImGuiIO& io = ImGui::GetIO();
		ImDrawList* drawList = ImGui::GetWindowDrawList();

		const ImVec2 canvasP0 = ImGui::GetCursorScreenPos();      // ImDrawList API uses screen coordinates!
		ImVec2 canvasSz = ImGui::GetContentRegionAvail();   // Resize canvas to what's available
		if (canvasSz.x < 300.0f) canvasSz.x = 300.0f;
		if (canvasSz.y < 300.0f) canvasSz.y = 300.0f;
		const ImVec2 canvasP1 = ImVec2(canvasP0.x + canvasSz.x, canvasP0.y + canvasSz.y);
		const ImVec2 origin(canvasP0.x + canvasSz.x / 2.0f + scrolling.x,
			canvasP0.y + canvasSz.y / 2.0f + scrolling.y); // Lock scrolled origin
		const ImVec2 mousePosInCanvas((io.MousePos.x - origin.x) / zoomFactor,
			(io.MousePos.y - origin.y) / zoomFactor);

		// Draw border and background color
		drawList->AddRectFilled(canvasP0, canvasP1, IM_COL32(50, 50, 50, 255));
		drawList->AddRect(canvasP0, canvasP1, IM_COL32(255, 255, 255, 255));

		// This will catch our interactions
		ImGui::InvisibleButton("canvas", canvasSz,
			ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
		const bool isMouseHovered = ImGui::IsItemHovered(); // Hovered
		const bool isMouseActive = ImGui::IsItemActive();   // Held

		// Pan (we use a zero mouse threshold when there's no context menu)
		// You may decide to make that threshold dynamic based on whether the mouse is hovering something etc.
		const float mouseThresholdForPan = -1.0f;
		if (isMouseActive && ImGui::IsMouseDragging(ImGuiMouseButton_Right, mouseThresholdForPan)) {
			scrolling.x += io.MouseDelta.x;
			scrolling.y += io.MouseDelta.y;
		}
		// Context menu (under default mouse threshold)
		const ImVec2 dragDelta = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right);
		if (dragDelta.x == 0.0f && dragDelta.y == 0.0f)
			ImGui::OpenPopupOnItemClick("context", ImGuiPopupFlags_MouseButtonRight);
		if (ImGui::BeginPopup("context")) {

			ImGui::EndPopup();
		}

		// Draw profile + all lines in the canvas
		drawList->PushClipRect(canvasP0, canvasP1, true);
		if (isMouseHovered && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
			func(glm::vec2(mousePosInCanvas.x, mousePosInCanvas.y));
		}
		const size_t mod = m_particles2D.size() / 15000;
		int index = 0;
		for (const auto& particle : m_particles2D) {
			index++;
			if (mod > 1 && index % mod != 0) continue;
			const auto& pointPosition = particle.m_position;
			const auto& pointColor = particle.m_color;
			const auto canvasPosition = ImVec2(origin.x + pointPosition.x * zoomFactor,
				origin.y + pointPosition.y * zoomFactor);

			drawList->AddCircleFilled(canvasPosition,
				glm::clamp(zoomFactor, 1.0f, 100.0f),
				IM_COL32(255.0f * pointColor.x, 255.0f * pointColor.y, 255.0f * pointColor.z, particle.IsBoundary() ? 255.0f : 128.0f));
		}
		drawList->AddCircle(origin,
			glm::clamp(zoomFactor, 1.0f, 100.0f),
			IM_COL32(255,
				0,
				0, 255));
		if (showGrid) {
			for (int i = 0; i < m_particleGrid2D.m_resolution.x; i++)
			{
				for (int j = 0; j < m_particleGrid2D.m_resolution.y; j++)
				{
					const auto& cell = m_particleGrid2D.RefCell(glm::ivec2(i, j));
					const auto cellCenter = m_particleGrid2D.GetPosition(glm::ivec2(i, j));
					const auto min = ImVec2(cellCenter.x - m_particleGrid2D.m_cellSize * 0.5f, cellCenter.y - m_particleGrid2D.m_cellSize * 0.5f);

					drawList->AddQuad(
						min * zoomFactor + origin,
						ImVec2(min.x + m_particleGrid2D.m_cellSize, min.y) * zoomFactor + origin,
						ImVec2(min.x + m_particleGrid2D.m_cellSize, min.y + m_particleGrid2D.m_cellSize) * zoomFactor + origin,
						ImVec2(min.x, min.y + m_particleGrid2D.m_cellSize) * zoomFactor + origin,
						IM_COL32(0, 0, 255, 128));
					const auto cellTarget = cellCenter + cell.m_target;
					drawList->AddLine(ImVec2(cellCenter.x, cellCenter.y) * zoomFactor + origin, ImVec2(cellTarget.x, cellTarget.y) * zoomFactor + origin, IM_COL32(255, 0, 0, 128));

				}
			}
		}
		drawFunc(origin, zoomFactor, drawList);
		drawList->PopClipRect();

	}
}
