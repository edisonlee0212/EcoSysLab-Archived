#pragma once
#ifdef BUILD_WITH_RAYTRACER
#include <CUDAModule.hpp>
#endif
#include "ILayer.hpp"
#include "PointCloud.hpp"
#include "SorghumField.hpp"
#include "SorghumState.hpp"
using namespace EvoEngine;
namespace EcoSysLab {
	class SorghumLayer : public ILayer {
		static void ObjExportHelper(glm::vec3 position, std::shared_ptr<Mesh> mesh,
			std::ofstream& of, unsigned& startIndex);

	public:
#ifdef BUILD_WITH_RAYTRACER
#pragma region Illumination
		int m_seed = 0;
		float m_pushDistance = 0.001f;
		RayProperties m_rayProperties;

		bool m_enableCompressedBTF = false;
		std::vector<Entity> m_processingEntities;
		int m_processingIndex;
		bool m_processing = false;
		float m_lightProbeSize = 0.05f;
		float m_perPlantCalculationTime = 0.0f;
		void CalculateIlluminationFrameByFrame();
		void CalculateIllumination();

#pragma endregion
#endif
		SorghumMeshGeneratorSettings m_sorghumMeshGeneratorSettings;
		bool m_autoRefreshSorghums = true;
		
		AssetRef m_panicleMaterial;

		AssetRef m_leafBottomFaceMaterial;
		AssetRef m_leafMaterial;
		AssetRef m_leafCBTFGroup;

		AssetRef m_leafAlbedoTexture;
		AssetRef m_leafNormalTexture;
		AssetRef m_segmentedLeafMaterials[25];

		float m_verticalSubdivisionMaxUnitLength = 0.02f;
		int m_horizontalSubdivisionStep = 2;
		float m_skeletonWidth = 0.0025f;

		glm::vec3 m_skeletonColor = glm::vec3(0);

		void OnCreate() override;
		void GenerateMeshForAllSorghums();
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
		void Update() override;
		void LateUpdate() override;

		static void ExportSorghum(const Entity& sorghum, std::ofstream& of,
			unsigned& startIndex);
		void ExportAllSorghumsModel(const std::string& filename);


	};

} // namespace EcoSysLab
