#include "Noises.hpp"
#include "glm/gtc/noise.hpp"
using namespace eco_sys_lab;

void NoiseDescriptor::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "m_type" << YAML::Value << m_type;
  out << YAML::Key << "m_frequency" << YAML::Value << m_frequency;
  out << YAML::Key << "m_intensity" << YAML::Value << m_intensity;
  out << YAML::Key << "m_multiplier" << YAML::Value << m_multiplier;
  out << YAML::Key << "m_min" << YAML::Value << m_min;
  out << YAML::Key << "m_max" << YAML::Value << m_max;
  out << YAML::Key << "m_offset" << YAML::Value << m_offset;
  out << YAML::Key << "m_shift" << YAML::Value << m_shift;
  out << YAML::Key << "m_ridgid" << YAML::Value << m_ridgid;
}

void NoiseDescriptor::Deserialize(const YAML::Node& in) {
  if (in["m_type"])
    m_type = in["m_type"].as<unsigned>();
  if (in["m_frequency"])
    m_frequency = in["m_frequency"].as<float>();
  if (in["m_intensity"])
    m_intensity = in["m_intensity"].as<float>();
  if (in["m_multiplier"])
    m_multiplier = in["m_multiplier"].as<float>();
  if (in["m_min"])
    m_min = in["m_min"].as<float>();
  if (in["m_max"])
    m_max = in["m_max"].as<float>();
  if (in["m_offset"])
    m_offset = in["m_offset"].as<float>();
  if (in["m_shift"])
    m_shift = in["m_shift"].as<glm::vec3>();
  if (in["m_ridgid"])
    m_ridgid = in["m_ridgid"].as<bool>();
}

Noise2D::Noise2D() {
  m_minMax = glm::vec2(-1000, 1000);
  m_noiseDescriptors.clear();
  m_noiseDescriptors.emplace_back();
}
Noise3D::Noise3D() {
  m_minMax = glm::vec2(-1000, 1000);
  m_noiseDescriptors.clear();
  m_noiseDescriptors.emplace_back();
}

bool Noise2D::OnInspect() {
  bool changed = false;
  if (ImGui::DragFloat2("Global Min/max", &m_minMax.x, 0, -1000, 1000)) {
    changed = true;
  }
  if (ImGui::Button("New start descriptor")) {
    changed = true;
    m_noiseDescriptors.emplace_back();
  }
  for (int i = 0; i < m_noiseDescriptors.size(); i++) {
    if (ImGui::TreeNodeEx(("No." + std::to_string(i)).c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::Button("Remove")) {
        m_noiseDescriptors.erase(m_noiseDescriptors.begin() + i);
        changed = true;
        ImGui::TreePop();
        continue;
      }
      changed =
          ImGui::Combo("Type", {"Constant", "Linear", "Simplex", "Perlin"}, m_noiseDescriptors[i].m_type) || changed;
      switch (static_cast<NoiseType>(m_noiseDescriptors[i].m_type)) {
        case NoiseType::Perlin:
          changed =
              ImGui::DragFloat("Frequency", &m_noiseDescriptors[i].m_frequency, 0.00001f, 0, 0, "%.5f") || changed;
          changed =
              ImGui::DragFloat("Intensity", &m_noiseDescriptors[i].m_intensity, 0.00001f, 1, 0, "%.5f") || changed;
          changed =
              ImGui::DragFloat("Multiplier", &m_noiseDescriptors[i].m_multiplier, 0.00001f, 0, 0, "%.5f") || changed;
          if (ImGui::DragFloat("Min", &m_noiseDescriptors[i].m_min, 0.01f, -99999, m_noiseDescriptors[i].m_max)) {
            changed = true;
            m_noiseDescriptors[i].m_min = glm::min(m_noiseDescriptors[i].m_min, m_noiseDescriptors[i].m_max);
          }
          if (ImGui::DragFloat("Max", &m_noiseDescriptors[i].m_max, 0.01f, m_noiseDescriptors[i].m_min, 99999)) {
            changed = true;
            m_noiseDescriptors[i].m_max = glm::max(m_noiseDescriptors[i].m_min, m_noiseDescriptors[i].m_max);
          }
          changed = ImGui::DragFloat("Offset", &m_noiseDescriptors[i].m_offset, 0.01f) || changed;
          changed = ImGui::Checkbox("Ridgid", &m_noiseDescriptors[i].m_ridgid) || changed;
          break;
        case NoiseType::Simplex:
          changed =
              ImGui::DragFloat("Frequency", &m_noiseDescriptors[i].m_frequency, 0.00001f, 0, 0, "%.5f") || changed;
          changed =
              ImGui::DragFloat("Intensity", &m_noiseDescriptors[i].m_intensity, 0.00001f, 1, 0, "%.5f") || changed;
          changed =
              ImGui::DragFloat("Multiplier", &m_noiseDescriptors[i].m_multiplier, 0.00001f, 0, 0, "%.5f") || changed;
          if (ImGui::DragFloat("Min", &m_noiseDescriptors[i].m_min, 0.01f, -99999, m_noiseDescriptors[i].m_max)) {
            changed = true;
            m_noiseDescriptors[i].m_min = glm::min(m_noiseDescriptors[i].m_min, m_noiseDescriptors[i].m_max);
          }
          if (ImGui::DragFloat("Max", &m_noiseDescriptors[i].m_max, 0.01f, m_noiseDescriptors[i].m_min, 99999)) {
            changed = true;
            m_noiseDescriptors[i].m_max = glm::max(m_noiseDescriptors[i].m_min, m_noiseDescriptors[i].m_max);
          }
          changed = ImGui::DragFloat("Offset", &m_noiseDescriptors[i].m_offset, 0.01f) || changed;
          changed = ImGui::Checkbox("Ridgid", &m_noiseDescriptors[i].m_ridgid) || changed;
          break;
        case NoiseType::Constant:
          changed = ImGui::DragFloat("Value", &m_noiseDescriptors[i].m_offset, 0.00001f, 0, 0, "%.5f") || changed;
          break;
        case NoiseType::Linear:
          changed =
              ImGui::DragFloat("X multiplier", &m_noiseDescriptors[i].m_frequency, 0.00001f, 0, 0, "%.5f") || changed;
          changed =
              ImGui::DragFloat("Y multiplier", &m_noiseDescriptors[i].m_intensity, 0.00001f, 0, 0, "%.5f") || changed;
          changed = ImGui::DragFloat("Base", &m_noiseDescriptors[i].m_offset, 0.00001f, 0, 0, "%.5f") || changed;
          if (ImGui::DragFloat("Min", &m_noiseDescriptors[i].m_min, 0.01f, -99999, m_noiseDescriptors[i].m_max)) {
            changed = true;
            m_noiseDescriptors[i].m_min = glm::min(m_noiseDescriptors[i].m_min, m_noiseDescriptors[i].m_max);
          }
          if (ImGui::DragFloat("Max", &m_noiseDescriptors[i].m_max, 0.01f, m_noiseDescriptors[i].m_min, 99999)) {
            changed = true;
            m_noiseDescriptors[i].m_max = glm::max(m_noiseDescriptors[i].m_min, m_noiseDescriptors[i].m_max);
          }
          changed = ImGui::Checkbox("Ridgid", &m_noiseDescriptors[i].m_ridgid) || changed;
          break;
      }
      ImGui::TreePop();
    }
  }
  return changed;
}

void Noise2D::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "m_minMax" << YAML::Value << m_minMax;
  out << YAML::Key << "m_noiseDescriptors" << YAML::BeginSeq;
  for (const auto& i : m_noiseDescriptors) {
    out << YAML::BeginMap;
    i.Serialize(out);
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;
}

void Noise2D::Load(const std::string& name, const YAML::Node& in) {
  if (in[name]) {
    const auto& node = in[name];
    if (node["m_minMax"])
      m_minMax = node["m_minMax"].as<glm::vec2>();

    if (node["m_noiseDescriptors"]) {
      m_noiseDescriptors.clear();
      for (const auto& i : node["m_noiseDescriptors"]) {
        m_noiseDescriptors.emplace_back();
        auto& back = m_noiseDescriptors.back();
        back.Deserialize(i);
      }
    }
  }
}

void Noise2D::RandomOffset(const float min, const float max) {
  for (auto& i : m_noiseDescriptors)
    i.m_offset = glm::linearRand(min, max);
}

void Noise3D::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "m_minMax" << YAML::Value << m_minMax;
  out << YAML::Key << "m_noiseDescriptors" << YAML::BeginSeq;
  for (const auto& i : m_noiseDescriptors) {
    i.Serialize(out);
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;
}

void Noise3D::Load(const std::string& name, const YAML::Node& in) {
  if (in[name]) {
    const auto& node = in[name];
    if (node["m_minMax"])
      m_minMax = node["m_minMax"].as<glm::vec2>();

    if (node["m_noiseDescriptors"]) {
      m_noiseDescriptors.clear();
      for (const auto& i : node["m_noiseDescriptors"]) {
        m_noiseDescriptors.emplace_back();
        auto& back = m_noiseDescriptors.back();
        back.Deserialize(i);
      }
    }
  }
}

float Noise2D::GetValue(const glm::vec2& position) const {
  float ret_val = 0;

  for (const auto& noise_descriptor : m_noiseDescriptors) {
    const auto actual_position = position + glm::vec2(noise_descriptor.m_shift);
    float noise = 0;
    switch (static_cast<NoiseType>(noise_descriptor.m_type)) {
      case NoiseType::Perlin:
        noise = glm::perlin(noise_descriptor.m_frequency * actual_position + glm::vec2(noise_descriptor.m_offset)) *
                noise_descriptor.m_multiplier;
        noise = glm::clamp(noise, noise_descriptor.m_min, noise_descriptor.m_max);
        break;
      case NoiseType::Simplex:
        noise = glm::simplex(noise_descriptor.m_frequency * actual_position + glm::vec2(noise_descriptor.m_offset)) *
                noise_descriptor.m_multiplier;
        noise = glm::clamp(noise, noise_descriptor.m_min, noise_descriptor.m_max);
        break;
      case NoiseType::Constant:
        noise = noise_descriptor.m_offset;
        break;
      case NoiseType::Linear:
        noise = noise_descriptor.m_offset + noise_descriptor.m_frequency * actual_position.x +
                noise_descriptor.m_intensity * actual_position.y;
        noise = glm::clamp(noise, noise_descriptor.m_min, noise_descriptor.m_max);
        break;
    }
    noise = glm::pow(noise, noise_descriptor.m_intensity);
    if (noise_descriptor.m_ridgid) {
      noise = -glm::abs(noise);
    }
    ret_val += noise;
  }
  return glm::clamp(ret_val, m_minMax.x, m_minMax.y);
}

bool Noise3D::OnInspect() {
  bool changed = false;
  if (ImGui::DragFloat2("Global Min/max", &m_minMax.x, 0, -1000, 1000)) {
    changed = true;
  }
  if (ImGui::Button("New start descriptor")) {
    changed = true;
    m_noiseDescriptors.emplace_back();
  }
  for (int i = 0; i < m_noiseDescriptors.size(); i++) {
    if (ImGui::TreeNodeEx(("No." + std::to_string(i)).c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::Button("Remove")) {
        m_noiseDescriptors.erase(m_noiseDescriptors.begin() + i);
        changed = true;
        ImGui::TreePop();
        continue;
      }
      changed =
          ImGui::Combo("Type", {"Constant", "Linear", "Simplex", "Perlin"}, m_noiseDescriptors[i].m_type) || changed;
      switch (static_cast<NoiseType>(m_noiseDescriptors[i].m_type)) {
        case NoiseType::Perlin:
          changed =
              ImGui::DragFloat("Frequency", &m_noiseDescriptors[i].m_frequency, 0.00001f, 0, 0, "%.5f") || changed;
          changed =
              ImGui::DragFloat("Intensity", &m_noiseDescriptors[i].m_intensity, 0.00001f, 1, 0, "%.5f") || changed;
          changed =
              ImGui::DragFloat("Multiplier", &m_noiseDescriptors[i].m_multiplier, 0.00001f, 0, 0, "%.5f") || changed;
          if (ImGui::DragFloat("Min", &m_noiseDescriptors[i].m_min, 0.01f, -99999, m_noiseDescriptors[i].m_max)) {
            changed = true;
            m_noiseDescriptors[i].m_min = glm::min(m_noiseDescriptors[i].m_min, m_noiseDescriptors[i].m_max);
          }
          if (ImGui::DragFloat("Max", &m_noiseDescriptors[i].m_max, 0.01f, m_noiseDescriptors[i].m_min, 99999)) {
            changed = true;
            m_noiseDescriptors[i].m_max = glm::max(m_noiseDescriptors[i].m_min, m_noiseDescriptors[i].m_max);
          }
          changed = ImGui::DragFloat("Offset", &m_noiseDescriptors[i].m_offset, 0.00001f, 0, 0, "%.5f") || changed;
          changed = ImGui::DragFloat3("Shift", &m_noiseDescriptors[i].m_shift.x, 0.00001f, 0, 0, "%.5f") || changed;
          changed = ImGui::Checkbox("Ridgid", &m_noiseDescriptors[i].m_ridgid) || changed;
          break;
        case NoiseType::Simplex:
          changed =
              ImGui::DragFloat("Frequency", &m_noiseDescriptors[i].m_frequency, 0.00001f, 0, 0, "%.5f") || changed;
          changed =
              ImGui::DragFloat("Intensity", &m_noiseDescriptors[i].m_intensity, 0.00001f, 1, 0, "%.5f") || changed;
          changed =
              ImGui::DragFloat("Multiplier", &m_noiseDescriptors[i].m_multiplier, 0.00001f, 0, 0, "%.5f") || changed;
          if (ImGui::DragFloat("Min", &m_noiseDescriptors[i].m_min, 0.01f, -99999, m_noiseDescriptors[i].m_max)) {
            changed = true;
            m_noiseDescriptors[i].m_min = glm::min(m_noiseDescriptors[i].m_min, m_noiseDescriptors[i].m_max);
          }
          if (ImGui::DragFloat("Max", &m_noiseDescriptors[i].m_max, 0.01f, m_noiseDescriptors[i].m_min, 99999)) {
            changed = true;
            m_noiseDescriptors[i].m_max = glm::max(m_noiseDescriptors[i].m_min, m_noiseDescriptors[i].m_max);
          }
          changed = ImGui::DragFloat("Offset", &m_noiseDescriptors[i].m_offset, 0.00001f, 0, 0, "%.5f") || changed;
          changed = ImGui::DragFloat3("Shift", &m_noiseDescriptors[i].m_shift.x, 0.00001f, 0, 0, "%.5f") || changed;
          changed = ImGui::Checkbox("Ridgid", &m_noiseDescriptors[i].m_ridgid) || changed;
          break;
        case NoiseType::Constant:
          changed = ImGui::DragFloat("Value", &m_noiseDescriptors[i].m_offset, 0.00001f, 0, 0, "%.5f") || changed;
          break;
        case NoiseType::Linear:
          changed =
              ImGui::DragFloat("X multiplier", &m_noiseDescriptors[i].m_frequency, 0.00001f, 0, 0, "%.5f") || changed;
          changed =
              ImGui::DragFloat("Y multiplier", &m_noiseDescriptors[i].m_intensity, 0.00001f, 0, 0, "%.5f") || changed;
          changed =
              ImGui::DragFloat("Z multiplier", &m_noiseDescriptors[i].m_multiplier, 0.00001f, 0, 0, "%.5f") || changed;
          changed = ImGui::DragFloat("Base", &m_noiseDescriptors[i].m_offset, 0.00001f, 0, 0, "%.5f") || changed;
          if (ImGui::DragFloat("Min", &m_noiseDescriptors[i].m_min, 0.01f, -99999, m_noiseDescriptors[i].m_max)) {
            changed = true;
            m_noiseDescriptors[i].m_min = glm::min(m_noiseDescriptors[i].m_min, m_noiseDescriptors[i].m_max);
          }
          if (ImGui::DragFloat("Max", &m_noiseDescriptors[i].m_max, 0.01f, m_noiseDescriptors[i].m_min, 99999)) {
            changed = true;
            m_noiseDescriptors[i].m_max = glm::max(m_noiseDescriptors[i].m_min, m_noiseDescriptors[i].m_max);
          }
          changed = ImGui::DragFloat3("Shift", &m_noiseDescriptors[i].m_shift.x, 0.00001f, 0, 0, "%.5f") || changed;
          changed = ImGui::Checkbox("Ridgid", &m_noiseDescriptors[i].m_ridgid) || changed;
          break;
      }
      ImGui::TreePop();
    }
  }
  return changed;
}

float Noise3D::GetValue(const glm::vec3& position) const {
  float ret_val = 0;
  for (const auto& noise_descriptor : m_noiseDescriptors) {
    const auto actual_position = position + noise_descriptor.m_shift;
    float noise = 0;
    switch (static_cast<NoiseType>(noise_descriptor.m_type)) {
      case NoiseType::Perlin:
        noise = glm::perlin(noise_descriptor.m_frequency * actual_position + glm::vec3(noise_descriptor.m_offset)) *
                noise_descriptor.m_multiplier;
        noise = glm::clamp(noise, noise_descriptor.m_min, noise_descriptor.m_max);
        break;
      case NoiseType::Simplex:
        noise = glm::simplex(noise_descriptor.m_frequency * actual_position + glm::vec3(noise_descriptor.m_offset)) *
                noise_descriptor.m_multiplier;
        noise = glm::clamp(noise, noise_descriptor.m_min, noise_descriptor.m_max);
        break;
      case NoiseType::Constant:
        noise = noise_descriptor.m_offset;
        break;
      case NoiseType::Linear:
        noise = noise_descriptor.m_offset + noise_descriptor.m_frequency * actual_position.x +
                noise_descriptor.m_intensity * actual_position.y + noise_descriptor.m_multiplier * actual_position.z;
        noise = glm::clamp(noise, noise_descriptor.m_min, noise_descriptor.m_max);
        break;
    }
    noise = glm::pow(noise, noise_descriptor.m_intensity);
    if (noise_descriptor.m_ridgid) {
      noise = -glm::abs(noise);
    }

    ret_val += noise;
  }
  return glm::clamp(ret_val, m_minMax.x, m_minMax.y);
}