#pragma once
#include "Plot2D.hpp"

#include "Curve.hpp"
using namespace EvoEngine;
namespace EcoSysLab {
#pragma region States
enum class StateMode { Default, CubicBezier };

struct SorghumPanicleState {
  glm::vec3 m_panicleSize = glm::vec3(0, 0, 0);
  int m_seedAmount = 0;
  float m_seedRadius = 0.002f;

  bool m_saved = false;
  SorghumPanicleState();
  bool OnInspect();
  void Serialize(YAML::Emitter &out);
  void Deserialize(const YAML::Node &in);
};
struct SorghumStemState {
  BezierSpline m_spline;
  glm::vec3 m_direction = {0, 1, 0};
  Plot2D<float> m_widthAlongStem;
  float m_length = 0;

  bool m_saved = false;
  SorghumStemState();
  [[nodiscard]] glm::vec3 GetPoint(float point) const;
  void Serialize(YAML::Emitter &out);
  void Deserialize(const YAML::Node &in);
  bool OnInspect(int mode);
};
struct SorghumLeafState {
  bool m_dead = false;
  BezierSpline m_spline;
  int m_index = 0;
  float m_startingPoint = 0;
  float m_length = 0.35f;
  float m_rollAngle = 0;
  float m_branchingAngle = 0;

  Plot2D<float> m_widthAlongLeaf;
  Plot2D<float> m_curlingAlongLeaf;
  Plot2D<float> m_bendingAlongLeaf;
  Plot2D<float> m_wavinessAlongLeaf;
  glm::vec2 m_wavinessPeriodStart = glm::vec2(0.0f);
  glm::vec2 m_wavinessFrequency = glm::vec2(0.0f);

  bool m_saved = false;
  SorghumLeafState();
  void CopyShape(const SorghumLeafState &another);
  void Serialize(YAML::Emitter &out);
  void Deserialize(const YAML::Node &in);
  bool OnInspect(int mode);
};
#pragma endregion

class SorghumState {
  friend class SorghumGrowthDescriptor;
  unsigned m_version = 0;

public:
  SorghumState();
  bool m_saved = false;
  std::string m_name = "Unnamed";
  SorghumPanicleState m_panicle;
  SorghumStemState m_stem;
  std::vector<SorghumLeafState> m_leaves;
  bool OnInspect(int mode);

  void Serialize(YAML::Emitter &out);
  void Deserialize(const YAML::Node &in);
};





} // namespace EcoSysLab