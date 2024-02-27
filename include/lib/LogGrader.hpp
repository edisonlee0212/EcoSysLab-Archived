#pragma once
#include <Plot2D.hpp>

#include "BranchShape.hpp"
#include "LogWood.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	struct LogWoodMeshGenerationSettings
	{
		float m_ySubdivision = 0.02f;
	};

	class ProceduralLogParameters
	{
	public:
		bool m_bottom = true;

		float m_lengthWithoutTrimInFeet = 16.0f;
		float m_lengthStepInInches = 1.f;
		float m_largeEndDiameterInInches = 25.f;
		float m_smallEndDiameterInInches = 20.f;

		unsigned m_mode = 0;
		float m_spanInInches = 0.0f;
		float m_angle = 180.0f;
		float m_crookRatio = 0.7f;
		
		bool OnInspect();
	};

	class LogGrader : public IPrivateComponent
	{
		std::shared_ptr<Mesh> m_tempCylinderMesh{};
		std::shared_ptr<Mesh> m_tempFlatMesh1{};
		std::shared_ptr<Mesh> m_tempFlatMesh2{};
		std::shared_ptr<Mesh> m_tempFlatMesh3{};
		std::shared_ptr<Mesh> m_tempFlatMesh4{};
		void RefreshMesh(const LogGrading& logGrading);
	public:
		int m_bestGradingIndex = 0;
		std::vector<LogGrading> m_availableBestGrading{};
		ProceduralLogParameters m_proceduralLogParameters;
		AssetRef m_branchShape{};
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
		void InitializeLogRandomly(const ProceduralLogParameters& proceduralLogParameters, const std::shared_ptr<BranchShape>& branchShape);
		LogWoodMeshGenerationSettings m_logWoodMeshGenerationSettings{};
		LogWood m_logWood{};
		[[nodiscard]] std::shared_ptr<Mesh> GenerateCylinderMesh(const LogWoodMeshGenerationSettings& meshGeneratorSettings) const;
		[[nodiscard]] std::shared_ptr<Mesh> GenerateFlatMesh(const LogWoodMeshGenerationSettings& meshGeneratorSettings, int startX, int endX) const;
		void OnCreate() override;
		void InitializeMeshRenderer(const LogWoodMeshGenerationSettings& meshGeneratorSettings) const;
		void ClearMeshRenderer() const;
	};
}