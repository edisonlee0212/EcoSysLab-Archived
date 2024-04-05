#pragma once
#include "Plot2D.hpp"
#include "SorghumGrowthStage.hpp"
#include "Curve.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	class SorghumState;

	class SorghumGrowthStagePair {
		void LeafStateHelper(SorghumLeafGrowthStage& left, SorghumLeafGrowthStage& right, float& a, int leafIndex) const;
	public:
		
		SorghumGrowthStage m_left = SorghumGrowthStage();
		SorghumGrowthStage m_right = SorghumGrowthStage();
		int m_mode = static_cast<int>(StateMode::Default);
		[[nodiscard]] int GetLeafSize(float a) const;
		[[nodiscard]] float GetStemLength(float a) const;
		[[nodiscard]] glm::vec3 GetStemDirection(float a) const;
		[[nodiscard]] glm::vec3 GetStemPoint(float a, float point) const;
		void ApplyPanicle(const std::shared_ptr<SorghumState>& targetState, float a) const;
		void ApplyStem(const std::shared_ptr<SorghumState>& targetState, float a) const;
		void Apply(const std::shared_ptr<SorghumState>& targetState, float a);
		void ApplyLeaves(const std::shared_ptr<SorghumState>& targetState, float a);
		void ApplyLeaf(const std::shared_ptr<SorghumState>& targetState, float a, int leafIndex);
	};

	class SorghumGrowthStages : public IAsset {
		std::vector<std::pair<float, SorghumGrowthStage>> m_sorghumGrowthStages;

	public:
		int m_mode = static_cast<int>(StateMode::Default);
		[[nodiscard]] bool ImportCSV(const std::filesystem::path& filePath);
		[[nodiscard]] float GetCurrentStartTime() const;
		[[nodiscard]] float GetCurrentEndTime() const;
		void Add(float time, const SorghumGrowthStage& state);
		void ResetTime(float previousTime, float newTime);
		void Remove(float time);
		void Apply(const std::shared_ptr<SorghumState>& targetState, float time) const;
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
		void Serialize(YAML::Emitter& out) override;
		void Deserialize(const YAML::Node& in) override;

		[[nodiscard]] Entity CreateEntity(float time = 0.0f) const;
	};
}