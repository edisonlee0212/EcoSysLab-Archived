#pragma once

using namespace UniEngine;
namespace EcoSysLab {
	struct NoiseDescriptor
	{
		float m_noiseScale = 20.0f;
		float m_noiseIntensity = 0.01f;
		float m_heightMin = -10;
		float m_heightMax = 10;
	};
	class FieldGround : public IAsset {
	public:
		glm::vec2 m_scale = glm::vec2(0.02f);
		glm::ivec2 m_size = glm::ivec2(150);
		float m_rowWidth = 0.0f;
		float m_alleyDepth = 0.15f;

		std::vector<NoiseDescriptor> m_noiseDescriptors;
		void OnCreate() override;
		Entity GenerateMesh(float overrideDepth = -1.0f);
		void OnInspect() override;
		void Serialize(YAML::Emitter& out) override;
		void Deserialize(const YAML::Node& in) override;
	};
}