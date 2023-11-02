#pragma once
#include "Tree.hpp"
using namespace EvoEngine;
namespace EcoSysLab {
	class TreePointCloudScanner : public IPrivateComponent{
	public:
        struct PointCloudPointSettings {
            float m_variance = 0.0f;
            float m_ballRandRadius = 0.0f;
            bool m_typeIndex = true;
            bool m_instanceIndex = false;
            bool m_branchIndex = false;
            bool m_internodeIndex = false;
            float m_boundingBoxLimit = 1.f;

            void OnInspect();

            void Serialize(const std::string& name, YAML::Emitter& out) const;

            void Deserialize(const std::string& name, const YAML::Node& in);

        } m_pointCloudPointSettings;

        struct CameraCaptureSettings {
            int m_rayBounce = 1;
            int m_raySample = 128;
            glm::vec3 m_focusPoint = glm::vec3(0, 1, 0);
            bool m_autoAdjustFocusPoint = true;
            int m_pitchAngleStart = -30;
            int m_pitchAngleStep = 10;
            int m_pitchAngleEnd = 30;
            int m_turnAngleStart = 0.0f;
            int m_turnAngleStep = 60;
            int m_turnAngleEnd = 360.0f;
            float m_distance = 5;
            float m_fov = 60;
            glm::ivec2 m_resolution = glm::ivec2(256, 256);
            bool m_useClearColor = true;
            glm::vec3 m_backgroundColor = glm::vec3(1.0f);
            float m_cameraDepthMax = 10;

            void OnInspect();

            void Serialize(const std::string& name, YAML::Emitter& out) const;

            void Deserialize(const std::string& name, const YAML::Node& in);

            GlobalTransform GetTransform(const Bound& bound, float turnAngle, float pitchAngle) const;
        } m_pointCloudSettings;

        PrivateComponentRef m_tree;
        PrivateComponentRef m_soil;
        void GeneratePointCloud(const std::filesystem::path& savePath);
        void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;

        void Serialize(YAML::Emitter& out) override;
        void Relink(const std::unordered_map<Handle, Handle>& map, const std::shared_ptr<Scene>& scene) override;
        void Deserialize(const YAML::Node& in) override;
	};
}