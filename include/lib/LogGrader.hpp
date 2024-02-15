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
		glm::vec4 m_defectColor = glm::vec4(0, 0, 1, 1);
		glm::vec4 m_baseColor = glm::vec4(0.6, 0.3, 0.0, 1.0);
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
		[[nodiscard]] std::shared_ptr<Mesh> GenerateCylinderMesh(const LogWoodMeshGenerationSettings& meshGeneratorSettings) const;
		[[nodiscard]] std::shared_ptr<Mesh> GenerateFlatMesh(const LogWoodMeshGenerationSettings& meshGeneratorSettings, int startX, int endX) const;
		void OnCreate() override;
		void InitializeMeshRenderer(const LogWoodMeshGenerationSettings& meshGeneratorSettings) const;
		void ClearMeshRenderer() const;


	};
}