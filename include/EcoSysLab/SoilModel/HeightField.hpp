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
		int m_precisionLevel = 2;
		[[nodiscard]] float GetValue(const glm::vec2& position);

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
		glm::vec2 m_positionOffset;
		std::vector<NoiseDescriptor> m_noiseDescriptors;
		void OnCreate() override;
		void OnInspect() override;
		void Serialize(YAML::Emitter& out) override;
		void Deserialize(const YAML::Node& in) override;

		[[nodiscard]] float GetValue(const glm::vec2& position) const;
	};
}