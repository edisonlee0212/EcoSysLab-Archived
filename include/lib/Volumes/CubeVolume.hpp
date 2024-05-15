#pragma once
#include "Mesh.hpp"
#include "IVolume.hpp"
using namespace EvoEngine;
namespace EcoSysLab {
	class CubeVolume : public IVolume {
	public:
		void ApplyMeshBounds(const std::shared_ptr<Mesh>& mesh);
		Bound m_minMaxBound;
		bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
		bool InVolume(const GlobalTransform& globalTransform, const glm::vec3& position) override;
		bool InVolume(const glm::vec3& position) override;
		glm::vec3 GetRandomPoint() override;
		void Serialize(YAML::Emitter& out) const override;
		void Deserialize(const YAML::Node& in) override;
	};
}