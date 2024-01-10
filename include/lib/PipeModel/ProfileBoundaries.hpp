#pragma once

using namespace EvoEngine;
namespace EcoSysLab
{
	class ProfileBoundary
	{
	public:
		std::vector<glm::vec2> m_points;
		glm::vec2 m_center;

		void CalculateCenter();

		[[nodiscard]] bool Valid() const;

		[[nodiscard]] bool InBoundary(const glm::vec2& position, glm::vec2& closestPoint) const;
		static bool Intersect(const glm::vec2& p1, const glm::vec2& q1, const glm::vec2& p2, const glm::vec2& q2);
	};

	class ProfileBoundaries
	{
	public:
		std::vector<ProfileBoundary> m_boundaries;
		[[nodiscard]] int InBoundaries(const glm::vec2& position, glm::vec2& closestPoint) const;
	};
}