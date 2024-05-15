#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <Vertex.hpp>

#include "Noises.hpp"
using namespace EvoEngine;

namespace EcoSysLab
{
	class HeightField : public IAsset
	{
	public:
		Noise2D m_noises2D;
		int m_precisionLevel = 2;
		[[nodiscard]] float GetValue(const glm::vec2& position);
		void RandomOffset(float min, float max);
		bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
		void Serialize(YAML::Emitter& out) const override;
		void Deserialize(const YAML::Node& in) override;
		void GenerateMesh(const glm::vec2& start, const glm::uvec2& resolution, float unitSize, std::vector<Vertex>& vertices, std::vector<glm::uvec3>& triangles, float xDepth = 1.0f, float zDepth = 1.0f);
	};
}