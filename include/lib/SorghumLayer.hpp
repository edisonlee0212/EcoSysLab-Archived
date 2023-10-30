#pragma once
#ifdef BUILD_WITH_RAYTRACER
#include <CUDAModule.hpp>
#endif
#include "ILayer.hpp"
#include "PointCloud.hpp"
#include "SorghumField.hpp"
#include <Curve.hpp>
#include <LeafSegment.hpp>
#include <Spline.hpp>
using namespace EvoEngine;
namespace EcoSysLab {
	struct LeafTag : IDataComponent {};
	struct LeafGeometryTag : IDataComponent {};
	struct LeafBottomFaceGeometryTag : IDataComponent {};
	struct PanicleTag : IDataComponent {};
	struct PanicleGeometryTag : IDataComponent {};
	struct StemTag : IDataComponent {};
	struct StemGeometryTag : IDataComponent {};
	struct SorghumTag : IDataComponent {};
	class SorghumStateGenerator;



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

		bool m_bottomFace = true;
		bool m_separated = false;
		bool m_includeStem = true;
		bool m_autoRefreshSorghums = true;
		EntityArchetype m_leafArchetype;
		EntityQuery m_leafQuery;
		EntityArchetype m_sorghumArchetype;
		EntityQuery m_sorghumQuery;
		EntityArchetype m_panicleArchetype;
		EntityQuery m_panicleQuery;
		EntityArchetype m_stemArchetype;
		EntityQuery m_stemQuery;

		EntityArchetype m_leafGeometryArchetype;
		EntityQuery m_leafGeometryQuery;
		EntityArchetype m_leafBottomFaceGeometryArchetype;
		EntityQuery m_leafBottomFaceGeometryQuery;
		EntityArchetype m_panicleGeometryArchetype;
		EntityQuery m_panicleGeometryQuery;
		EntityArchetype m_stemGeometryArchetype;
		EntityQuery m_stemGeometryQuery;

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
		Entity CreateSorghum();
		Entity CreateSorghum(const std::shared_ptr<ProceduralSorghum>& descriptor);
		Entity
			CreateSorghum(const std::shared_ptr<SorghumStateGenerator>& descriptor);
		Entity CreateSorghumStem(const Entity& plantEntity);
		Entity CreateSorghumLeaf(const Entity& plantEntity, int leafIndex);
		Entity CreateSorghumPanicle(const Entity& plantEntity);
		void GenerateMeshForAllSorghums();
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
		void Update() override;
		void LateUpdate() override;

		static void ExportSorghum(const Entity& sorghum, std::ofstream& of,
			unsigned& startIndex);
		void ExportAllSorghumsModel(const std::string& filename);


	};

} // namespace EcoSysLab
