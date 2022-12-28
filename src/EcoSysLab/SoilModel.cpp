#include "SoilModel.hpp"
using namespace EcoSysLab;

float SoilModel::GetWater(const glm::vec3& position) const
{
	return m_waterDensity[Index(GetCoordinate(position))];
}

float SoilModel::GetDensity(const glm::vec3& position) const
{
	if (position.y > 0.0f) return 0.0f;
	return 1.0f / position.y;
}

float SoilModel::GetNutrient(const glm::vec3& position) const
{
	return m_nutrientsDensity[Index(GetCoordinate(position))];
}

float SoilModel::AddWater(const glm::vec3& position, float value)
{
	return 0.0f;
}

float SoilModel::AddDensity(const glm::vec3& position, float value)
{
	return 0.0f;
}

float SoilModel::AddNutrient(const glm::vec3& position, float value)
{
	return 0.0f;
}


int SoilModel::Index(const int x, const int y, const int z) const
{
	return x + y * m_voxelResolution.x + z * m_voxelResolution.x * m_voxelResolution.y;
}

unsigned SoilModel::Index(const glm::uvec3& coordinate) const
{
	return coordinate.x + coordinate.y * m_voxelResolution.x + coordinate.z * m_voxelResolution.x * m_voxelResolution.y;
}

glm::uvec3 SoilModel::GetCoordinate(const unsigned index) const
{
	return { index % m_voxelResolution.x, index % (m_voxelResolution.x * m_voxelResolution.y) / m_voxelResolution.x, index / (m_voxelResolution.x * m_voxelResolution.y) };
}

glm::uvec3 SoilModel::GetCoordinate(const glm::vec3& position) const
{
	assert(position.x >= m_startPosition.x && position.y >= m_startPosition.y && position.z >= m_startPosition.z);
	assert(position.x <= m_startPosition.x + m_voxelSize * m_voxelResolution.x 
		&& position.y <= m_startPosition.y + m_voxelSize * m_voxelResolution.y 
		&& position.z <= m_startPosition.z + m_voxelSize * m_voxelResolution.z);
	return {(position.x - m_startPosition.x) / m_voxelSize, (position.y - m_startPosition.y) / m_voxelSize,(position.z - m_startPosition.z) / m_voxelSize };
}

glm::vec3 SoilModel::GetCenter(const glm::uvec3& coordinate) const
{
	return {
		m_voxelSize * (static_cast<float>(coordinate.x) + 0.5f) + m_startPosition.x,
		m_voxelSize * (static_cast<float>(coordinate.y) + 0.5f) + m_startPosition.y,
		m_voxelSize * (static_cast<float>(coordinate.z) + 0.5f) + m_startPosition.z
	};
}


void SoilModel::Convolution3(const std::vector<float>& input, std::vector<float>& output, const std::vector<int>& indices, const std::vector<float>& weights)
{
	auto entries = m_voxelResolution.x * m_voxelResolution.y * m_voxelResolution.z;
	assert(input.size() == entries);
	assert(output.size() == entries);
	assert(indices.size() == weights.size());

	// for a 3D convolution:
	assert(m_voxelResolution.x >= 3);
	assert(m_voxelResolution.y >= 3);
	assert(m_voxelResolution.z >= 3);

	// iterate over all indices that are not part of the boundary, where the whole convolution kernel can be applied
	for (auto x = 1; x < m_voxelResolution.x - 1; ++x)
	{
		for (auto y = 1; y < m_voxelResolution.y - 1; ++y)
		{
			for (auto z = 1; z < m_voxelResolution.z - 1; ++z)
			{
				auto i = Index(x, y, z);
				output[i] = 0.f;
				for (auto j = 0; j < indices.size(); ++j)
				{
					output[i] += input[i + indices[j]] * weights[j];
				}
			}
		}
	}
}

void SoilModel::Initialize(const SoilParameters& soilParameters, const glm::uvec3& voxelResolution, const float voxelDistance, const glm::vec3& minPosition) {
	m_voxelResolution = voxelResolution;
	m_voxelSize = voxelDistance;
	m_startPosition = minPosition;
	Reset();
	m_initialized = true;
}

void SoilModel::Reset()
{
	assert(m_initialized);
	m_time = 0.f;
	auto numVoxels = m_voxelResolution.x * m_voxelResolution.y * m_voxelResolution.z;
	{
		m_waterDensity.resize(numVoxels);
		std::fill(m_waterDensity.begin(), m_waterDensity.end(), 0);

		m_waterDensityBlur.resize(numVoxels);
		std::fill(m_waterDensityBlur.begin(), m_waterDensityBlur.end(), 0);

		m_gradWaterDensityX1.resize(numVoxels);
		std::fill(m_gradWaterDensityX1.begin(), m_gradWaterDensityX1.end(), 0);
		m_gradWaterDensityX2.resize(numVoxels);
		std::fill(m_gradWaterDensityX2.begin(), m_gradWaterDensityX2.end(), 0);
		m_gradWaterDensityX3.resize(numVoxels);
		std::fill(m_gradWaterDensityX3.begin(), m_gradWaterDensityX3.end(), 0);

		m_fluxX1.resize(numVoxels);
		std::fill(m_fluxX1.begin(), m_fluxX1.end(), 0);
		m_fluxX2.resize(numVoxels);
		std::fill(m_fluxX2.begin(), m_fluxX2.end(), 0);
		m_fluxX3.resize(numVoxels);
		std::fill(m_fluxX3.begin(), m_fluxX3.end(), 0);


		m_divergenceX1.resize(numVoxels);
		std::fill(m_divergenceX1.begin(), m_divergenceX1.end(), 0);
		m_divergenceX2.resize(numVoxels);
		std::fill(m_divergenceX2.begin(), m_divergenceX2.end(), 0);
		m_divergenceX3.resize(numVoxels);
		std::fill(m_divergenceX3.begin(), m_divergenceX3.end(), 0);

		m_divergence.resize(numVoxels);
		std::fill(m_divergence.begin(), m_divergence.end(), 0);

		m_grad_cw.resize(numVoxels);
		std::fill(m_grad_cw.begin(), m_grad_cw.end(), 0);

		m_nutrientsDensity.resize(numVoxels);
		std::fill(m_nutrientsDensity.begin(), m_nutrientsDensity.end(), 0);
	}
}

void SoilModel::TestSetup()
{
	assert(m_initialized);
	Reset();
	auto numVoxels = m_voxelResolution.x * m_voxelResolution.y * m_voxelResolution.z;
	const auto v = std::vector<float>(numVoxels, 0.f);

	auto blur_idx = std::vector<int>({

		Index(-1, -1, -1),
		Index(0, -1, -1),
		Index(1, -1, -1),

		Index(-1,  0, -1),
		Index(0,  0, -1),
		Index(1,  0, -1),

		Index(-1,  1, -1),
		Index(0,  1, -1),
		Index(1,  1, -1),

		Index(-1, -1, 0),
		Index(0, -1, 0),
		Index(1, -1, 0),

		Index(-1,  0, 0),
		Index(0,  0, 0),
		Index(1,  0, 0),

		Index(-1,  1, 0),
		Index(0,  1, 0),
		Index(1,  1, 0),

		Index(-1, -1, 1),
		Index(0, -1, 1),
		Index(1, -1, 1),

		Index(-1,  0, 1),
		Index(0,  0, 1),
		Index(1,  0, 1),

		Index(-1,  1, 1),
		Index(0,  1, 1),
		Index(1,  1, 1),
		});

	auto blur_weights = std::vector<float>({
		0.009188900331780544,
		0.025493013475061985,
		0.009188900331780544,

		0.025493013475061978,
		0.0707259533321939,
		0.025493013475061978,

		0.009188900331780544,
		0.025493013475061985,
		0.009188900331780544,

		0.025493013475061978,
		0.0707259533321939,
		0.025493013475061978,

		0.0707259533321939,
		0.19621691565184837,
		0.0707259533321939,

		0.025493013475061978,
		0.0707259533321939,
		0.025493013475061978,

		0.009188900331780544,
		0.025493013475061985,
		0.009188900331780544,

		0.025493013475061978,
		0.0707259533321939,
		0.025493013475061978,

		0.009188900331780544,
		0.025493013475061985,
		0.009188900331780544,
		});

	auto tmp = v;
	m_waterDensity[Index(32, 32, 16)] = 1.f;

	// blur twice
	Convolution3(m_waterDensity, tmp, blur_idx, blur_weights);
	Convolution3(tmp, m_waterDensity, blur_idx, blur_weights);


	//m_waterDensity[idx(64, 33)] = 1.f;
	//m_waterDensity[idx(65, 32)] = 1.f;
	//m_waterDensity[idx(65, 33)] = 1.f;



	// create some nutrients
	m_nutrientsDensity[Index(20, 12, 5)] = 100.f;
	m_nutrientsDensity[Index(42, 18, 15)] = 100.f;

	m_nutrientsDensity[Index(54, 8, 46)] = 50.f;
	m_nutrientsDensity[Index(32, 18, 40)] = 50.f;

	for (auto i = 0; i < 50; ++i)
	{
		Convolution3(m_nutrientsDensity, tmp, blur_idx, blur_weights);
		Convolution3(tmp, m_nutrientsDensity, blur_idx, blur_weights);
	}

	m_nutrientsDensity[Index(5, 32, 8)] = 50.f;
	m_nutrientsDensity[Index(32, 50, 30)] = 100.f;

	m_nutrientsDensity[Index(18, 4, 56)] = 50.f;
	m_nutrientsDensity[Index(27, 48, 27)] = 100.f;
	for (auto i = 0; i < 40; ++i)
	{
		Convolution3(m_nutrientsDensity, tmp, blur_idx, blur_weights);
		Convolution3(tmp, m_nutrientsDensity, blur_idx, blur_weights);
	}
}

bool SoilModel::Initialized() const
{
	return m_initialized;
}

float SoilModel::GetTime() const
{
	return m_time;
}


void SoilModel::Step(const SoilParameters& soilParameters)
{
	if (!m_initialized) Initialize(soilParameters);
	auto grad_x1_idx = std::vector<int>({
		Index(-1, 0, 0),
		Index(+1, 0, 0),
		});
	auto grad_x2_idx = std::vector<int>({
		Index(0, -1, 0),
		Index(0, +1, 0),
		});
	auto grad_x3_idx = std::vector<int>({
		Index(0, 0, -1),
		Index(0, 0, +1),
		});
	auto grad_weights = std::vector<float>({
		-m_voxelSize / 2.f,
		 m_voxelSize / 2.f
		});


	//Convolution3(m_waterDensity, m_waterDensityBlur, blur_idx, blur_weights);
	//m_waterDensityBlur = m_waterDensity;

	// compute gradient dw
	Convolution3(m_waterDensity, m_gradWaterDensityX1, grad_x1_idx, grad_weights);
	Convolution3(m_waterDensity, m_gradWaterDensityX2, grad_x2_idx, grad_weights);
	Convolution3(m_waterDensity, m_gradWaterDensityX3, grad_x3_idx, grad_weights);

	// compute the total flux:
	for (auto i = 0; i < m_waterDensity.size(); ++i)
	{
		m_fluxX1[i] = -soilParameters.m_diffusionFactor * m_gradWaterDensityX1[i] + soilParameters.m_gravityFactor * m_gravityDirection.x * m_waterDensity[i];
		m_fluxX2[i] = -soilParameters.m_diffusionFactor * m_gradWaterDensityX2[i] + soilParameters.m_gravityFactor * m_gravityDirection.y * m_waterDensity[i];
		m_fluxX3[i] = -soilParameters.m_diffusionFactor * m_gradWaterDensityX3[i] + soilParameters.m_gravityFactor * m_gravityDirection.z * m_waterDensity[i];
	}

	// compute divergence
	Convolution3(m_fluxX1, m_divergenceX1, grad_x1_idx, grad_weights);
	Convolution3(m_fluxX2, m_divergenceX2, grad_x2_idx, grad_weights);
	Convolution3(m_fluxX3, m_divergenceX3, grad_x3_idx, grad_weights);

	//Convolution3(m_gradWaterDensityX1, m_divergenceX1, grad_x1_idx, grad_weights, m_voxelResolution);

	// sum divergence
	for (auto i = 0; i < m_waterDensity.size(); ++i)
	{
		m_divergence[i] = m_divergenceX1[i] + m_divergenceX2[i] + m_divergenceX3[i];
	}

	// apply forward euler:
	for (auto i = 0; i < m_waterDensity.size(); ++i)
	{
		m_waterDensity[i] += soilParameters.m_deltaTime * (0 - m_divergence[i]);
		m_waterDensity[i] = std::max(m_waterDensity[i], 0.f);
	}


	// count total water:
	m_totalWaterDensity = 0.f;
	for (auto i = 0; i < m_waterDensity.size(); ++i)
		m_totalWaterDensity += m_waterDensity[i];

	/*
	for (auto x = 1u; x < m_voxelResolution.x-1; x++)
	{
		for (auto y = 1u; y < m_voxelResolution.y-1; y++)
		{
			dw_c[idx(x, y)] =
				  dw_o[idx(x-1, y-1)] + dw_o[idx(x, y-1)] + dw_o[idx(x+1, y-1)]
				+ dw_o[idx(x-1, y  )]                     + dw_o[idx(x+1, y  )]
				+ dw_o[idx(x-1, y+1)] + dw_o[idx(x, y+1)] + dw_o[idx(x+1, y+1)];
		}
	}
	*/
	m_time += soilParameters.m_deltaTime;
}

glm::uvec3 SoilModel::GetVoxelResolution() const
{
	return m_voxelResolution;
}

float SoilModel::GetVoxelSize() const
{
	return m_voxelSize;
}

glm::vec3 SoilModel::GetStartPosition() const
{
	return m_startPosition;
}

/*
unsigned int SoilModel::idx(glm::uvec3 p)
{
	return p.z * m_voxelResolution.x*m_voxelResolution.y + p.y * m_voxelResolution.x + p.x;
}

unsigned int SoilModel::idx(unsigned int x, unsigned int y)
{
	return y * m_voxelResolution.x + x;
}
*/
