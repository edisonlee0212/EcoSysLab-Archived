#include "SoilModel.hpp"

#include <cassert>
using namespace EcoSysLab;
using namespace std;
using namespace glm;


void SoilModel::Initialize(const SoilParameters& soilParameters, const uvec3& voxelResolution, const float voxelDistance, const vec3& minPosition)
{
	assert(!m_initialized);

	m_capacityForce = soilParameters.m_capFactor;
	m_diffusionForce = soilParameters.m_diffusionFactor;
	m_gravityForce = soilParameters.m_gravityFactor;
	m_deltaTime = soilParameters.m_deltaTime;

	m_Resolution = voxelResolution;
	m_voxelSize = voxelDistance;
	m_volumePositionMin = minPosition;

	m_blur_3x3_idx = std::vector<int>({
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

	m_blur_3x3_weights = std::vector<float>({
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
		0.009188900331780544
	});

	m_initialized = true;
	Reset();
}

void SoilModel::Reset()
{
	assert(m_initialized);

	m_time = 0.f;

	auto numVoxels = m_Resolution.x * m_Resolution.y * m_Resolution.z;
	auto empty = vector<float>(numVoxels);
	std::fill(empty.begin(), empty.end(), 0.0f);

	m_waterDensity = empty;
	m_waterDensityBlur = empty;
	m_gradWaterDensityX = empty;
	m_gradWaterDensityY = empty;
	m_gradWaterDensityZ = empty;
	m_fluxX = empty;
	m_fluxY = empty;
	m_fluxZ = empty;
	m_divergenceX = empty;
	m_divergenceY = empty;
	m_divergenceZ = empty;
	m_divergence = empty;
	m_grad_cw = empty;
	m_nutrientsDensity = empty;

	auto tmp = empty;

	// add some water
	// fill the center with water and blur it, but avoid filling the edges due to boundarie conditions.
	auto a = 10;
	for(auto x=a; x < m_Resolution.x - a; ++x)
	{
		for(auto y=a; y < m_Resolution.y - a; ++y)
		{
			for(auto z=a; z < m_Resolution.z - a; ++z)
			{
				m_waterDensity[Index(x, y, z)] = 1.0;
			}
		}
	}
	// blur twice
	Convolution3(m_waterDensity, tmp, m_blur_3x3_idx, m_blur_3x3_weights);
	Convolution3(tmp, m_waterDensity, m_blur_3x3_idx, m_blur_3x3_weights);
	//std::fill(m_waterDensity.begin(), m_waterDensity.end(), 1.0f);

	/*
	auto tmp = std::vector<float>(numVoxels, 0.f);
	//m_waterDensity[Index(32, 32, 16)] = 1.f;
	m_waterDensity[Index(32, 32, 8)] = 100.f;

	// blur twice
	Convolution3(m_waterDensity, tmp, m_blur_3x3_idx, m_blur_3x3_weights);
	Convolution3(tmp, m_waterDensity, m_blur_3x3_idx, m_blur_3x3_weights);
	*/

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
		Convolution3(m_nutrientsDensity, tmp, m_blur_3x3_idx, m_blur_3x3_weights);
		Convolution3(tmp, m_nutrientsDensity, m_blur_3x3_idx, m_blur_3x3_weights);
	}

	m_nutrientsDensity[Index(5, 32, 8)] = 50.f;
	m_nutrientsDensity[Index(32, 50, 30)] = 100.f;

	m_nutrientsDensity[Index(18, 4, 56)] = 50.f;
	m_nutrientsDensity[Index(27, 48, 27)] = 100.f;
	for (auto i = 0; i < 40; ++i)
	{
		Convolution3(m_nutrientsDensity, tmp, m_blur_3x3_idx, m_blur_3x3_weights);
		Convolution3(tmp, m_nutrientsDensity, m_blur_3x3_idx, m_blur_3x3_weights);
	}
}



void SoilModel::Convolution3(const std::vector<float>& input, std::vector<float>& output, const std::vector<int>& indices, const std::vector<float>& weights) const
{
	auto entries = m_Resolution.x * m_Resolution.y * m_Resolution.z;
	assert(input.size() == entries);
	assert(output.size() == entries);
	assert(indices.size() == weights.size());

	// for a 3D convolution:
	assert(m_Resolution.x >= 3);
	assert(m_Resolution.y >= 3);
	assert(m_Resolution.z >= 3);

	// iterate over all indices that are not part of the boundary, where the whole convolution kernel can be applied
	for (auto x = 1; x < m_Resolution.x - 1; ++x)
	{
		for (auto y = 1; y < m_Resolution.y - 1; ++y)
		{
			for (auto z = 1; z < m_Resolution.z - 1; ++z)
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

bool SoilModel::Initialized() const
{
	return m_initialized;
}

float SoilModel::GetTime() const
{
	return m_time;
}


void SoilModel::Step()
{
	assert(m_initialized);

	const auto gradXIndex = std::vector<int>({
		Index(-1, 0, 0),
		Index(+1, 0, 0),
		});
	const auto gradYIndex = std::vector<int>({
		Index(0, -1, 0),
		Index(0, +1, 0),
		});
	const auto gradZIndex = std::vector<int>({
		Index(0, 0, -1),
		Index(0, 0, +1),
		});
	const auto gradWeights = std::vector<float>({
		- 1.0f / (2.0f*m_voxelSize),
		  1.0f / (2.0f*m_voxelSize)
		});


	//Convolution3(m_waterDensity, m_waterDensityBlur, blur_idx, blur_weights);
	//m_waterDensityBlur = m_waterDensity;

	// compute gradient dw
	Convolution3(m_waterDensity, m_gradWaterDensityX, gradXIndex, gradWeights);
	Convolution3(m_waterDensity, m_gradWaterDensityY, gradYIndex, gradWeights);
	Convolution3(m_waterDensity, m_gradWaterDensityZ, gradZIndex, gradWeights);

	// boundary conditions for gradient:

	// X:
	for (auto y = 0u; y < m_Resolution.y; ++y)
	{
		for (auto z = 0u; z < m_Resolution.z; ++z)
		{
			m_gradWaterDensityX[Index(0,                y, z)] =  m_waterDensity[Index(1,                y, z)] / (2 * m_voxelSize);
			m_gradWaterDensityX[Index(m_Resolution.x-1, y, z)] = -m_waterDensity[Index(m_Resolution.x-2, y, z)] / (2 * m_voxelSize);
		}
	}

	// Y:
	for (auto x = 0u; x < m_Resolution.x; ++x)
	{
		for (auto z = 0u; z < m_Resolution.z; ++z)
		{
			m_gradWaterDensityY[Index(x, 0,                z)] =  m_waterDensity[Index(x, 1,                z)] / (2 * m_voxelSize);
			m_gradWaterDensityY[Index(x, m_Resolution.y-1, z)] = -m_waterDensity[Index(x, m_Resolution.y-2, z)] / (2 * m_voxelSize);
		}
	}

	// Z:
	for(auto x = 0u; x<m_Resolution.x; ++x)
	{
		for (auto y = 0u; y < m_Resolution.y; ++y)
		{
			m_gradWaterDensityZ[Index(x, y, 0)]                =   m_waterDensity[Index(x, y, 1               )] / (2*m_voxelSize);
			m_gradWaterDensityZ[Index(x, y, m_Resolution.z-1)] = - m_waterDensity[Index(x, y, m_Resolution.z-2)] / (2*m_voxelSize);
		}
	}

	// compute the total flux:
	for (auto i = 0; i < m_waterDensity.size(); ++i)
	{
		m_fluxX[i] = - m_diffusionForce * m_gradWaterDensityX[i] + m_gravityForce * m_gravityDirection.x * m_waterDensity[i];
		m_fluxY[i] = - m_diffusionForce * m_gradWaterDensityY[i] + m_gravityForce * m_gravityDirection.y * m_waterDensity[i];
		m_fluxZ[i] = - m_diffusionForce * m_gradWaterDensityZ[i] + m_gravityForce * m_gravityDirection.z * m_waterDensity[i];
	}

	// compute divergence
	Convolution3(m_fluxX, m_divergenceX, gradXIndex, gradWeights);
	Convolution3(m_fluxY, m_divergenceY, gradYIndex, gradWeights);
	Convolution3(m_fluxZ, m_divergenceZ, gradZIndex, gradWeights);

	// apply boundary conditions for flux:
	// ...

	//Convolution3(m_gradWaterDensityX, m_divergenceX, grad_x1_idx, grad_weights, m_Resolution);

	// sum divergence
	for (auto i = 0; i < m_waterDensity.size(); ++i)
	{
		m_divergence[i] = m_divergenceX[i] + m_divergenceY[i] + m_divergenceZ[i];
	}

	// apply forward euler:
	for (auto i = 0; i < m_waterDensity.size(); ++i)
	{
		m_waterDensity[i] += m_deltaTime * (0 - m_divergence[i]);
		m_waterDensity[i] = std::max(m_waterDensity[i], 0.f);
	}


	// count total water:
	m_totalWaterDensity = 0.f;
	for (auto i = 0; i < m_waterDensity.size(); ++i)
		m_totalWaterDensity += m_waterDensity[i];

	/*
	for (auto x = 1u; x < m_Resolution.x-1; x++)
	{
		for (auto y = 1u; y < m_Resolution.y-1; y++)
		{
			dw_c[idx(x, y)] =
				  dw_o[idx(x-1, y-1)] + dw_o[idx(x, y-1)] + dw_o[idx(x+1, y-1)]
				+ dw_o[idx(x-1, y  )]                     + dw_o[idx(x+1, y  )]
				+ dw_o[idx(x-1, y+1)] + dw_o[idx(x, y+1)] + dw_o[idx(x+1, y+1)];
		}
	}
	*/
	m_time += m_deltaTime;
}

void EcoSysLab::SoilModel::WaterLogic()
{
	ChangeWater(vec3(0, 10, 0),     10);
	ChangeWater(vec3(8, -10, 4),    20);

	if((int)(m_time / 20.0) % 2 == 0)
	{
		ChangeWater(vec3(-18, 00, -15), 10);
	}
	if ((int)((m_time+5) / 19.0) % 2 == 0)
	{
		ChangeWater(vec3(0, 20, -15),   -10);
	}
}



float SoilModel::GetWater(const vec3& position) const
{
	return m_waterDensity[Index(GetCoordinate(position))];
}

float SoilModel::GetDensity(const vec3& position) const
{
	// todo replace by actual values
	if (position.y > 0.0f) return 0.0f;
	return 1.0f / position.y;
}

float SoilModel::GetNutrient(const vec3& position) const
{
	return m_nutrientsDensity[Index(GetCoordinate(position))];
}

void SoilModel::ChangeWater(const vec3& position, float amount)
{
	// todo: actually have a subpixel shift on the kernel, depending on the position
	auto cc = GetCoordinate(position);
	assert(cc.x > 0 && cc.y > 0 && cc.z > 0); // for the 3x3 kernel below
	assert(cc.x < m_Resolution.x-1
		&& cc.y < m_Resolution.y - 1
		&& cc.z < m_Resolution.z - 1);

	auto c = Index(cc);

	for(auto i=0u; i<m_blur_3x3_idx.size(); ++i)
	{
		m_waterDensity[c+m_blur_3x3_idx[i]] += amount * m_blur_3x3_weights[i];
	}
}

void SoilModel::ChangeDensity(const vec3& position, float value)
{
	//return 0.0f;
}

void SoilModel::ChangeNutrient(const vec3& position, float value)
{
	//return 0.0f;
}


int EcoSysLab::SoilModel::Index(const uvec3& resolution, int x, int y, int z)
{
	return x + y * resolution.x + z * resolution.x * resolution.y;
}

int SoilModel::Index(const int x, const int y, const int z) const
{
	return Index(m_Resolution, x, y, z);
}

unsigned EcoSysLab::SoilModel::Index(const uvec3& resolution, const uvec3& coordinate)
{
	return coordinate.x + coordinate.y * resolution.x + coordinate.z * resolution.x * resolution.y;
}

unsigned SoilModel::Index(const uvec3& coordinate) const
{
	return Index(m_Resolution, coordinate);
}

uvec3 SoilModel::GetCoordinate(const unsigned index) const
{
	return {
		index % m_Resolution.x,
		index % (m_Resolution.x * m_Resolution.y) / m_Resolution.x,
		index / (m_Resolution.x * m_Resolution.y) };
}

uvec3 SoilModel::GetCoordinate(const vec3& position) const
{
	assert(position.x >= m_volumePositionMin.x
		&& position.y >= m_volumePositionMin.y
		&& position.z >= m_volumePositionMin.z);
	assert(position.x <= m_volumePositionMin.x + m_voxelSize * m_Resolution.x
		&& position.y <= m_volumePositionMin.y + m_voxelSize * m_Resolution.y
		&& position.z <= m_volumePositionMin.z + m_voxelSize * m_Resolution.z);
	return {
		(position.x - m_volumePositionMin.x) / m_voxelSize,
		(position.y - m_volumePositionMin.y) / m_voxelSize,
		(position.z - m_volumePositionMin.z) / m_voxelSize };
}

vec3 SoilModel::GetCenter(const uvec3& coordinate) const
{
	return {
		m_voxelSize * (static_cast<float>(coordinate.x) + 0.5f) + m_volumePositionMin.x,
		m_voxelSize * (static_cast<float>(coordinate.y) + 0.5f) + m_volumePositionMin.y,
		m_voxelSize * (static_cast<float>(coordinate.z) + 0.5f) + m_volumePositionMin.z
	};
}


uvec3 SoilModel::GetVoxelResolution() const
{
	return m_Resolution;
}

float SoilModel::GetVoxelSize() const
{
	return m_voxelSize;
}

vec3 SoilModel::GetStartPosition() const
{
	return m_volumePositionMin;
}

/*
unsigned int SoilModel::idx(uvec3 p)
{
	return p.z * m_Resolution.x*m_Resolution.y + p.y * m_Resolution.x + p.x;
}

unsigned int SoilModel::idx(unsigned int x, unsigned int y)
{
	return y * m_Resolution.x + x;
}
*/
