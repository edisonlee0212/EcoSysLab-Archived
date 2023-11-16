
#include "IVolume.hpp"

#include <Jobs.hpp>

using namespace EcoSysLab;

bool IVolume::InVolume(const GlobalTransform& globalTransform, const glm::vec3& position) { return false; }

bool IVolume::InVolume(const glm::vec3& position) { return false; }

void IVolume::InVolume(const GlobalTransform& globalTransform, const std::vector<glm::vec3>& positions, std::vector<bool>& results) {
    results.resize(positions.size());
    Jobs::ParallelFor(positions.size(), [&](unsigned i) {
        results[i] = InVolume(globalTransform, positions[i]);
        });
}

void IVolume::InVolume(const std::vector<glm::vec3>& positions, std::vector<bool>& results) {
    results.resize(positions.size());
    Jobs::ParallelFor(positions.size(), [&](unsigned i) {
        results[i] = InVolume(positions[i]);
        });
}

glm::vec3 SphericalVolume::GetRandomPoint() {
    return glm::ballRand(1.0f) * m_radius;
}
bool SphericalVolume::InVolume(
    const GlobalTransform& globalTransform, const glm::vec3& position) {
    return false;
}
bool SphericalVolume::InVolume(const glm::vec3& position) {
    const auto relativePosition = glm::vec3(position.x / m_radius.x, position.y / m_radius.y, position.z / m_radius.z);
    return glm::length(relativePosition) <= 1.0f;
}
