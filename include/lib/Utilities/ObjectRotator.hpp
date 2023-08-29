#pragma once


using namespace EvoEngine;
namespace EcoSysLab {
    class ObjectRotator : public IPrivateComponent {
    public:
        float m_rotateSpeed;
        glm::vec3 m_rotation = glm::vec3(0, 0, 0);

        void OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;

        void FixedUpdate() override;

        void Serialize(YAML::Emitter& out) override;

        void Deserialize(const YAML::Node& in) override;
    };
} // namespace PlantFactory
