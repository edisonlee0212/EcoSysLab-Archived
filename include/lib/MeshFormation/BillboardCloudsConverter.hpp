#pragma once

#include "BillboardCloud.hpp"
#include "Skeleton.hpp"
using namespace EvoEngine;

namespace EcoSysLab{
	class BillboardCloudsConverter : public IPrivateComponent {
	public:
		bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
	};
}