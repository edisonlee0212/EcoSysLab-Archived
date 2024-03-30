#pragma once
using namespace EvoEngine;
namespace EcoSysLab {
	class RectangularSorghumFieldPattern {
	public:
		glm::vec2 m_distance = glm::vec2(3.0f);
		glm::vec2 m_distanceVariance = glm::vec2(0.5f);
		glm::vec3 m_rotationVariance = glm::vec3(0.0f);
		glm::ivec2 m_size = glm::ivec2(10, 10);
		void GenerateField(std::vector<glm::mat4>& matricesList);
	};

	class SorghumField : public IAsset {
		friend class SorghumLayer;

	public:
		int m_sizeLimit = 2000;
		float m_sorghumSize = 1.0f;
		std::vector<std::pair<AssetRef, glm::mat4>> m_matrices;
		virtual void GenerateMatrices() {};
		Entity InstantiateField();

		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
		void Serialize(YAML::Emitter& out) override;
		void Deserialize(const YAML::Node& in) override;
		void CollectAssetRef(std::vector<AssetRef>& list) override;
	};
} // namespace EcoSysLab