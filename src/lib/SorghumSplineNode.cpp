#include "SorghumSplineNode.hpp"
#include "SorghumLayer.hpp"

using namespace EcoSysLab;

SorghumSplineNode::SorghumSplineNode() {}
SorghumSplineNode::SorghumSplineNode(glm::vec3 position, float angle, float stemWidth, float leafWidth, float waviness, const glm::vec3& axis, const glm::vec3& left,
    SorghumSplineType nodeType, float range) {
  m_position = position;
  m_theta = angle;
  m_stemWidth = stemWidth;
  m_leafWidth = leafWidth;
  m_waviness = waviness;
  m_axis = axis;
  m_left = left;
  m_type = nodeType;
  m_range = range;
}
