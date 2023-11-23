#pragma once

using namespace EvoEngine;
namespace EcoSysLab {
	class BranchShape : public IAsset
	{
	public:
		float m_barkXFrequency = 3.0f;
		float m_barkYFrequency = 10.0f;
		float m_barkDepth = 0.25f;

		float m_baseFrequency = 1.0f;
		float m_baseMaxDistance = 0.25f;
		float m_baseDistanceDecreaseFactor = 2.f;
		float m_baseDepth = 1.f;
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
		float GetValue(float xFactor, float distanceToRoot);
	};
}