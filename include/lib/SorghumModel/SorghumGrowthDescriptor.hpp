#pragma once
#include "Plot2D.hpp"
#include "SorghumState.hpp"
#include "Curve.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	struct SorghumStatePair {
		SorghumState m_left = SorghumState();
		SorghumState m_right = SorghumState();
		float m_a = 1.0f;
		int m_mode = (int)StateMode::Default;
		[[nodiscard]] int GetLeafSize() const;
		[[nodiscard]] float GetStemLength() const;
		[[nodiscard]] glm::vec3 GetStemDirection() const;
		[[nodiscard]] glm::vec3 GetStemPoint(float point) const;
	};

	class SorghumGrowthDescriptor : public IAsset {
		unsigned m_version = 0;
		friend class SorghumData;
		std::vector<std::pair<float, SorghumState>> m_sorghumStates;

	public:
		int m_mode = (int)StateMode::Default;
		[[nodiscard]] bool ImportCSV(const std::filesystem::path& filePath);
		[[nodiscard]] unsigned GetVersion() const;
		[[nodiscard]] float GetCurrentStartTime() const;
		[[nodiscard]] float GetCurrentEndTime() const;
		void Add(float time, const SorghumState& state);
		void ResetTime(float previousTime, float newTime);
		void Remove(float time);
		[[nodiscard]] SorghumStatePair Get(float time) const;

		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
		void Serialize(YAML::Emitter& out) override;
		void Deserialize(const YAML::Node& in) override;
	};
}