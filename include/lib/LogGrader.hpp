#pragma once
#include <Plot2D.hpp>

#include "BranchShape.hpp"
#include "LogWood.hpp"
#include "Noises.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	struct LogWoodMeshGenerationSettings
	{
		float m_xSubdivision = 1.0f;
		float m_ySubdivision = 0.01f;
	};
	
	struct ProceduralLogParameters
	{
		float m_height = 4.0f;
		float m_startRadius = 0.4f;
		float m_endRadius = 0.35f;
		PlottedDistribution<float> m_sweep{};
		PlottedDistribution<float> m_sweepDirectionAngle{};
	};

	class LogGrader : public IPrivateComponent
	{
	public:
		ProceduralLogParameters m_proceduralLogParameters;
		AssetRef m_branchShape{};
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
		void InitializeLogRandomly(const ProceduralLogParameters& proceduralLogParameters, const std::shared_ptr<BranchShape>& branchShape);
		LogWoodMeshGenerationSettings m_logWoodMeshGenerationSettings{};
		LogWood m_logWood {};
		[[nodiscard]] std::shared_ptr<Mesh> GenerateSurfaceMesh(const LogWoodMeshGenerationSettings& meshGeneratorSettings) const;
		void OnCreate() override;
		void InitializeMeshRenderer(const LogWoodMeshGenerationSettings& meshGeneratorSettings) const;
		void ClearMeshRenderer() const;
	};
}