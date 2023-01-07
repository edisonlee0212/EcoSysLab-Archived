#pragma once

#include <glm/glm.hpp>
#include <vector>
#include "ecosyslab_export.h"

using namespace UniEngine;

namespace EcoSysLab
{
	class HeightField : public IAsset
	{
	public:
		AssetRef m_groundSurface;

		[[nodiscard]] float GetValue(const glm::vec2& position) const;

		void OnInspect() override;
		void Serialize(YAML::Emitter& out) override;
		void Deserialize(const YAML::Node& in) override;
		void GenerateMesh(const glm::vec2& start, const glm::uvec2& resolution, float unitSize, std::vector<Vertex>& vertices, std::vector<glm::uvec3>& triangles);
	};


	struct NoiseDescriptor
	{
		float m_noiseScale = 20.0f;
		float m_offset = 0.0f;
		float m_powerFactor = 1.0f;
		float m_noiseIntensity = 0.01f;
		float m_heightMin = -10;
		float m_heightMax = 10;
	};
	class GroundSurface : public IAsset {
	public:
		glm::vec2 m_minMax = glm::vec2(-1000, 1000);

		std::vector<NoiseDescriptor> m_noiseDescriptors;
		void OnCreate() override;
		void GenerateMesh(const glm::vec2& start, const glm::uvec2& resolution, float unitSize, const std::function<float(const glm::vec2& position)>& heightFunc, std::vector<Vertex>& vertices, std::vector<glm::uvec3>& triangles) const;
		void OnInspect() override;
		void Serialize(YAML::Emitter& out) override;
		void Deserialize(const YAML::Node& in) override;
	};
}