#pragma once
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
		NoiseDescriptor m_startSurfaceNoiseDescriptor{};
		NoiseDescriptor m_endSurfaceNoiseDescriptor{};
		float m_startRadius = 0.35f;
		float m_endRadius = 0.3f;
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

		void InitializeMeshRenderer(const LogWoodMeshGenerationSettings& meshGeneratorSettings) const;
		void ClearMeshRenderer() const;
	};
}