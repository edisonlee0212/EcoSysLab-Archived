#pragma once

#include <glm/glm.hpp>
#include <vector>
#include "ecosyslab_export.h"

using namespace UniEngine;

namespace EcoSysLab
{
	enum class NoiseType
	{
		Simplex,
		Perlin,
		Constant
	};

	struct NoiseDescriptor
	{
		unsigned m_type;
		float m_noiseScale = 0.5f;
		float m_base = 0.0f;
		float m_powerFactor = 1.0f;
		float m_noiseIntensity = 1.0f;
		float m_heightMin = -10;
		float m_heightMax = 10;
		float m_offset;
	};
	class Noises2D {
	public:
		glm::vec2 m_minMax = glm::vec2(-1000, 1000);
		
		std::vector<NoiseDescriptor> m_noiseDescriptors;
		Noises2D();
		bool OnInspect();
		void Save(const std::string& name, YAML::Emitter& out) const;
		void Load(const std::string& name, const YAML::Node& in);

		[[nodiscard]] float GetValue(const glm::vec2& position) const;
	};

	class Noises3D {
	public:
		glm::vec2 m_minMax = glm::vec2(-1000, 1000);
		std::vector<NoiseDescriptor> m_noiseDescriptors;
		Noises3D();
		bool OnInspect();
		void Save(const std::string& name, YAML::Emitter& out) const;
		void Load(const std::string& name, const YAML::Node& in);

		[[nodiscard]] float GetValue(const glm::vec3& position) const;
	};
}