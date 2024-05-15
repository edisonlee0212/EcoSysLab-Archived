#pragma once

using namespace EvoEngine;
namespace EcoSysLab {
	class BarkDescriptor : public IAsset
	{
	public:
		float m_barkXFrequency = 3.0f;
		float m_barkYFrequency = 5.0f;
		float m_barkDepth = 0.1f;

		float m_baseFrequency = 1.0f;
		float m_baseMaxDistance = 1.f;
		float m_baseDistanceDecreaseFactor = 2.f;
		float m_baseDepth = .1f;
		bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
		float GetValue(float xFactor, float distanceToRoot);

		void Serialize(YAML::Emitter& out) const override;
		void Deserialize(const YAML::Node& in) override;
	};
}