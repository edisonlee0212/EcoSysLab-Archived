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

		float m_lengthWithoutTrim = 4.0f;
		float m_lengthStep = 0.02f;
		float m_largeEndDiameter = 0.8f;
		float m_smallEndDiameter = 0.7f;

		unsigned m_mode = 0;
		float m_span = 0.0f;
		float m_angle = 0.0f;
		float m_crookRatio = 2.0f;
		
		bool OnInspect();
	};

	class LogGrader : public IPrivateComponent
	{
		std::shared_ptr<Mesh> m_tempCylinderMesh{};
		std::shared_ptr<Mesh> m_tempFlatMesh1{};
		std::shared_ptr<Mesh> m_tempFlatMesh2{};
		std::shared_ptr<Mesh> m_tempFlatMesh3{};
		std::shared_ptr<Mesh> m_tempFlatMesh4{};
		void RefreshMesh();
	public:
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