
#include "BillboardCloud.hpp"
using namespace evo_engine;

std::vector<BillboardCloud::Cluster> BillboardCloud::StochasticClusterize(
    std::vector<ClusterTriangle> operatingTriangles, const ClusterizationSettings& clusterizeSettings) {
  BoundingSphere boundingSphere;
  boundingSphere.Initialize(elements);
  const auto& settings = clusterizeSettings.foliage_clusterization_settings;
  float epsilon = boundingSphere.radius * glm::clamp(1.f - settings.density, 0.05f, 1.f);

  skipped_triangles.clear();
  auto remainingTriangles = operatingTriangles;
  std::vector<Cluster> retVal;

  int epoch = 0;
  while (!remainingTriangles.empty()) {
    Cluster newCluster;
    float maxArea = 0.f;
    std::vector<int> selectedTriangleIndices;
    std::mutex voteMutex;
    Jobs::RunParallelFor(settings.iteration, [&](unsigned iteration) {
      int seedTriangleIndex = glm::linearRand(0, static_cast<int>(remainingTriangles.size()) - 1);
      ClusterTriangle seedTriangle = remainingTriangles.at(seedTriangleIndex);
      const auto perturb0 = glm::linearRand(-epsilon, epsilon);
      const auto perturb1 = glm::linearRand(-epsilon, epsilon);
      const auto perturb2 = glm::linearRand(-epsilon, epsilon);
      const auto& seedTriangleElement = elements.at(seedTriangle.element_index);
      const auto& seedTriangleIndices = seedTriangleElement.triangles.at(seedTriangle.triangle_index);
      const auto seedTriangleNormal = CalculateNormal(seedTriangle);
      glm::vec3 seedTriangleP0 =
          seedTriangleElement.vertices.at(seedTriangleIndices.x).position + perturb0 * seedTriangleNormal;
      glm::vec3 seedTriangleP1 =
          seedTriangleElement.vertices.at(seedTriangleIndices.y).position + perturb1 * seedTriangleNormal;
      glm::vec3 seedTriangleP2 =
          seedTriangleElement.vertices.at(seedTriangleIndices.z).position + perturb2 * seedTriangleNormal;
      auto testPlaneNormal =
          glm::normalize(glm::cross(seedTriangleP0 - seedTriangleP1, seedTriangleP0 - seedTriangleP2));
      if (glm::dot(testPlaneNormal, seedTriangleNormal) < 0.f) {
        testPlaneNormal = -testPlaneNormal;
      }
      float testPlaneDistance = glm::dot(seedTriangleP0, testPlaneNormal);
      float area = 0.f;
      std::vector<int> currentPendingRemovalTriangles;
      std::vector<ClusterTriangle> trianglesForCluster;
      for (int testTriangleIndex = 0; testTriangleIndex < remainingTriangles.size(); testTriangleIndex++) {
        const auto& testTriangle = remainingTriangles.at(testTriangleIndex);
        const auto& testTriangleElement = elements.at(testTriangle.element_index);
        const auto& testTriangleIndices = testTriangleElement.triangles.at(testTriangle.triangle_index);
        const auto& testTriangleP0 = testTriangleElement.vertices.at(testTriangleIndices.x).position;
        const auto& testTriangleP1 = testTriangleElement.vertices.at(testTriangleIndices.y).position;
        const auto& testTriangleP2 = testTriangleElement.vertices.at(testTriangleIndices.z).position;

        if (!settings.fill_band &&
            glm::abs(testPlaneDistance - glm::dot(testTriangleP0, testPlaneNormal)) <=
                epsilon * settings.sample_range &&
            glm::abs(testPlaneDistance - glm::dot(testTriangleP1, testPlaneNormal)) <=
                epsilon * settings.sample_range &&
            glm::abs(testPlaneDistance - glm::dot(testTriangleP2, testPlaneNormal)) <=
                epsilon * settings.sample_range) {
          trianglesForCluster.emplace_back(remainingTriangles.at(testTriangleIndex));
        }
        if (glm::abs(testPlaneDistance - glm::dot(testTriangleP0, testPlaneNormal)) > epsilon)
          continue;
        if (glm::abs(testPlaneDistance - glm::dot(testTriangleP1, testPlaneNormal)) > epsilon)
          continue;
        if (glm::abs(testPlaneDistance - glm::dot(testTriangleP2, testPlaneNormal)) > epsilon)
          continue;
        // increment projected area (Angular area Contribution)
        // use projected area Contribution
        float angle = glm::acos(glm::abs(glm::dot(testPlaneNormal, CalculateNormal(testTriangle))));
        float angular = (glm::pi<float>() / 2.f - angle) / (glm::pi<float>() / 2.f);
        area += CalculateArea(testTriangle) * angular;

        // save reference to T with billboard plane
        currentPendingRemovalTriangles.emplace_back(testTriangleIndex);
      }

      if (settings.fill_band) {
        for (auto& operatingTriangle : operatingTriangles) {
          const auto& testTriangle = operatingTriangle;
          const auto& testTriangleElement = elements.at(testTriangle.element_index);
          const auto& testTriangleIndices = testTriangleElement.triangles.at(testTriangle.triangle_index);
          const auto& testTriangleP0 = testTriangleElement.vertices.at(testTriangleIndices.x).position;
          const auto& testTriangleP1 = testTriangleElement.vertices.at(testTriangleIndices.y).position;
          const auto& testTriangleP2 = testTriangleElement.vertices.at(testTriangleIndices.z).position;

          if (glm::abs(testPlaneDistance - glm::dot(testTriangleP0, testPlaneNormal)) <=
                  epsilon * settings.sample_range &&
              glm::abs(testPlaneDistance - glm::dot(testTriangleP1, testPlaneNormal)) <=
                  epsilon * settings.sample_range &&
              glm::abs(testPlaneDistance - glm::dot(testTriangleP2, testPlaneNormal)) <=
                  epsilon * settings.sample_range) {
            trianglesForCluster.emplace_back(operatingTriangle);
          }
        }
      }
      if (!currentPendingRemovalTriangles.empty()) {
        std::lock_guard lock(voteMutex);
        if (area > maxArea) {
          // Update cluster.
          newCluster.cluster_plane = Plane(testPlaneNormal, testPlaneDistance);
          newCluster.triangles = trianglesForCluster;
          selectedTriangleIndices = currentPendingRemovalTriangles;
        }
      }
    });

    if (selectedTriangleIndices.empty()) {
      skipped_triangles.insert(skipped_triangles.end(), remainingTriangles.begin(), remainingTriangles.end());
      break;
    }

    if (!newCluster.triangles.empty())
      retVal.emplace_back(std::move(newCluster));

    // Remove selected triangle from the remaining triangle.
    for (auto it = selectedTriangleIndices.rbegin(); it != selectedTriangleIndices.rend(); ++it) {
      remainingTriangles[*it] = remainingTriangles.back();
      remainingTriangles.pop_back();
    }
    epoch++;
    if (settings.timeout != 0 && epoch >= settings.timeout) {
      EVOENGINE_ERROR("Stochastic clustering timeout!")
      break;
    }
  }
  return retVal;
}
