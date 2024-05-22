#pragma once

#include "BillboardCloud.hpp"
#include "Skeleton.hpp"
using namespace EvoEngine;

namespace EcoSysLab{
	class SpeedTreeMeshConverter : public IPrivateComponent {

		void Convert(const std::shared_ptr<Prefab>& currentPrefab, BillboardCloud::Cluster& cluster, const Transform& parentModelSpaceTransform);

		void Convert(const BillboardCloud::ProjectSettings& projectSettings);

	public:
		AssetRef m_originalModel;
		AssetRef m_foliageMesh;
		AssetRef m_branchMesh;
		AssetRef m_foliageMaterial;
		AssetRef m_branchMaterial;
		bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
	};
}