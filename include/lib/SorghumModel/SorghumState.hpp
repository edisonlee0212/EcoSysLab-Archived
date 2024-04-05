#pragma once
#include "SorghumSpline.hpp"
using namespace EvoEngine;
namespace EcoSysLab
{
	struct SorghumMeshGeneratorSettings
	{
		bool m_enablePanicle = true;
		bool m_enableStem = true;
		bool m_enableLeaves = true;
		bool m_bottomFace = true;
		bool m_leafSeparated = false;
		float m_leafThickness = 0.001f;
		bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer);
	};

	class SorghumPanicleState
	{
	public:
		glm::vec3 m_panicleSize = glm::vec3(0, 0, 0);
		int m_seedAmount = 0;
		float m_seedRadius = 0.002f;
		bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer);
		void Serialize(YAML::Emitter& out) const;
		void Deserialize(const YAML::Node& in);

		void GenerateGeometry(const glm::vec3 &stemTip, std::vector<Vertex>& vertices,
			std::vector<unsigned int>& indices) const;
		void GenerateGeometry(const glm::vec3& stemTip, std::vector<Vertex>& vertices,
			std::vector<unsigned int>& indices, const std::shared_ptr<ParticleInfoList>& particleInfoList) const;
	};

	class SorghumStemState
	{
	public:
		SorghumSpline m_spline;
		bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer);
		void Serialize(YAML::Emitter& out) const;
		void Deserialize(const YAML::Node& in);

		void GenerateGeometry(std::vector<Vertex>& vertices,
			std::vector<unsigned int>& indices) const;
	};

	class SorghumLeafState
	{
	public:
		int m_index = 0;
		SorghumSpline m_spline;
		bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer);
		void Serialize(YAML::Emitter& out) const;
		void Deserialize(const YAML::Node& in);

		void GenerateGeometry(std::vector<Vertex>& vertices,
			std::vector<unsigned int>& indices, bool bottomFace = false, float thickness = 0.0f) const;
	};

	class SorghumState : public IAsset
	{
	public:
		SorghumPanicleState m_panicle;
		SorghumStemState m_stem;
		std::vector<SorghumLeafState> m_leaves;
		void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
		void Serialize(YAML::Emitter& out) override;
		void Deserialize(const YAML::Node& in) override;
	};

}