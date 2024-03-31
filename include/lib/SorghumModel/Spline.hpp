#pragma once
#include <Curve.hpp>
#include "SorghumGrowthDescriptor.hpp"
using namespace EvoEngine;
namespace EcoSysLab {

    enum class SorghumSplineType
    {
	    Stem,
        LeafSheathToRoot,
    	LeafSheath,
        Leaf
    };

struct SorghumSplineNode {
  glm::vec3 m_position;
  float m_theta;
  float m_stemWidth;
  float m_leafWidth;
  float m_waviness;
  glm::vec3 m_axis;
  glm::vec3 m_left;
  SorghumSplineType m_type = SorghumSplineType::Stem;
  float m_range;

  SorghumSplineNode(glm::vec3 position, float angle, float stemWidth, float leafWidth, float waviness, const glm::vec3 &axis, const glm::vec3 &left,
      SorghumSplineType nodeType, float range);
  SorghumSplineNode();
};
} // namespace EcoSysLab