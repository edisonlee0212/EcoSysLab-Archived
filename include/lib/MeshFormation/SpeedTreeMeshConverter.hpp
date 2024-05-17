#pragma once

#include "Skeleton.hpp"
using namespace EvoEngine;

namespace EcoSysLab{
	class SpeedTreeMeshConverter : public IPrivateComponent {

		void Convert();

	public:
		AssetRef m_originalMesh;
		AssetRef m_foliageMesh;
		AssetRef m_branchMesh;

		bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
	};
}