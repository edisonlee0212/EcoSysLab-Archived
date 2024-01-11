#pragma once

using namespace EvoEngine;
namespace EcoSysLab
{
	class ProfileBoundary
	{
	public:
		std::vector<glm::vec2> m_points {};
		glm::vec2 m_center = glm::vec2(0.0f);
		void CalculateCenter();
		void RenderBoundary(ImVec2 origin, float zoomFactor, ImDrawList* drawList, ImU32 color, float thickness) const;
		[[nodiscard]] bool BoundaryValid() const;
		[[nodiscard]] bool InBoundary(const glm::vec2& position) const;
		[[nodiscard]] bool InBoundary(const glm::vec2& position, glm::vec2& closestPoint) const;
		static bool Intersect(const glm::vec2& p1, const glm::vec2& q1, const glm::vec2& p2, const glm::vec2& q2);
	};
	class ProfileAttractor
	{
	public:
		std::vector<std::pair<glm::vec2, glm::vec2>> m_attractorPoints{};
		void RenderAttractor(ImVec2 origin, float zoomFactor, ImDrawList* drawList, ImU32 color, float thickness) const;
		glm::vec2 FindClosestPoint(const glm::vec2& position) const;
	};
	class ProfileConstraints
	{
	public:
		std::vector<ProfileBoundary> m_boundaries {};
		std::vector<ProfileAttractor> m_attractors{};
		[[nodiscard]] int FindBoundary(const glm::vec2& position) const;
		[[nodiscard]] bool Valid(size_t boundaryIndex) const;
		[[nodiscard]] glm::vec2 GetTarget(const glm::vec2& position) const;
	};
}