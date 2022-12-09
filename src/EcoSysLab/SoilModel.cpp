#include "SoilModel.hpp"
using namespace EcoSysLab;

float EcoSysLab::SoilModel::GetWater(const glm::vec3& position) const
{
	return 1.0f;
}

float EcoSysLab::SoilModel::GetDensity(const glm::vec3& position) const
{
	if (position.y > 0.0f) return 0.0f;
	return 1.0f / position.y;
}

float EcoSysLab::SoilModel::GetNutrient(const glm::vec3& position) const
{
	return 1.0f;
}

float EcoSysLab::SoilModel::AddWater(const glm::vec3& position, float value)
{
	return 0.0f;
}

float EcoSysLab::SoilModel::AddDensity(const glm::vec3& position, float value)
{
	return 0.0f;
}

float EcoSysLab::SoilModel::AddNutrient(const glm::vec3& position, float value)
{
	return 0.0f;
}
