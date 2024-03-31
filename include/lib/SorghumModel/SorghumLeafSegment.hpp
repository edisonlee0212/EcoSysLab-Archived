#pragma once
#include "SorghumSplineNode.hpp"

using namespace EvoEngine;
namespace EcoSysLab {
class SorghumLeafSegment {
public:
  glm::vec3 m_position;
  glm::vec3 m_front;
  glm::vec3 m_up;
  glm::quat m_rotation;
  float m_leafHalfWidth;
  float m_theta;
  float m_stemRadius;
  float m_leftHeightFactor = 1.0f;
  float m_rightHeightFactor = 1.0f;
  SorghumSplineType m_nodeType;
  SorghumLeafSegment(glm::vec3 position, glm::vec3 up, glm::vec3 front, float stemWidth,
              float leafHalfWidth, float theta, SorghumSplineType nodeType,
              float leftHeightFactor = 1.0f, float rightHeightFactor = 1.0f);

  glm::vec3 GetPoint(float angle) const;

  glm::vec3 GetNormal(float angle) const;
};
} // namespace PlantFactory