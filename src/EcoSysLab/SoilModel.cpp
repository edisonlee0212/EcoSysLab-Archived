#include "SoilModel.hpp"

#include <cassert>
using namespace EcoSysLab;
using namespace std;
using namespace glm;


/* Coordinate system

The voxel position is its center.
Each voxel is dx wide.

				<-dx ->
				-------------------------
				|     |     |     |     |
				|  x  |  x  |  x  |  x  |
				|     |     |     |     |
				-------------------------
				   |     |     |     |
				   |     |     |     |
X-Coordinate:   -- 0 --- 1 --- 2 --- 3 -----

The 'm_volumePositionMin' stores the lower left corner of the lower left voxel.
I.e. for m_volumePositionMin=(0, 0) and m_resolution=(2, 2), and m_dx=1,
the voxel centers are at 0.5 and 1.5.

*/

void SoilModel::Initialize(const SoilParameters& soilParameters, const glm::uvec3& voxelResolution, const vec3& minPosition)
{
	m_diffusionForce = soilParameters.m_diffusionForce;
	m_gravityForce = soilParameters.m_gravityForce;
	m_dt = soilParameters.m_deltaTime;

	m_resolution = voxelResolution;
	m_dx = soilParameters.m_deltaX;
	m_boundingBoxMin = minPosition;

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

	auto numVoxels = m_resolution.x * m_resolution.y * m_resolution.z;
	auto empty = vector<float>(numVoxels);
	std::fill(empty.begin(), empty.end(), 0.0f);

	m_w = empty;
	m_tmpValue = empty;
	m_w_grad_x = empty;
	m_w_grad_y = empty;
	m_w_grad_z = empty;

	m_div_diff_x = empty;
	m_div_diff_y = empty;
	m_div_diff_z = empty;

	m_div_grav_x = empty;
	m_div_grav_y = empty;
	m_div_grav_z = empty;

	m_grad_cw = empty;
	m_nutrientsDensity = empty;

	auto tmp = empty;

	// add some water
	/*
	// fill the center with water and blur it, but avoid filling the edges due to boundarie conditions.
	auto a = 10;
	for(auto x=a; x < m_resolution.x - a; ++x)
	{
		for(auto y=a; y < m_resolution.y - a; ++y)
		{
			for(auto z=a; z < m_resolution.z - a; ++z)
			{
				m_w[Index(x, y, z)] = 1.0;
			}
		}
	}
	// blur twice
	Convolution3(m_w, tmp, m_blur_3x3_idx, m_blur_3x3_weights);
	Convolution3(tmp, m_w, m_blur_3x3_idx, m_blur_3x3_weights);
	//std::fill(m_w.begin(), m_w.end(), 1.0f);
	*/


	/*
	auto tmp = std::vector<float>(numVoxels, 0.f);
	//m_w[Index(32, 32, 16)] = 1.f;
	m_w[Index(32, 32, 8)] = 100.f;

	// blur twice
	Convolution3(m_w, tmp, m_blur_3x3_idx, m_blur_3x3_weights);
	Convolution3(tmp, m_w, m_blur_3x3_idx, m_blur_3x3_weights);
	*/

	//m_w[idx(64, 33)] = 1.f;
	//m_w[idx(65, 32)] = 1.f;
	//m_w[idx(65, 33)] = 1.f;

	// create some nutrients
	m_nutrientsDensity[Index(20, 12, 5)] = 100.f;
	m_nutrientsDensity[Index(42, 18, 15)] = 100.f;

	m_nutrientsDensity[Index(54, 8, 46)] = 50.f;
	m_nutrientsDensity[Index(32, 18, 40)] = 50.f;

	for (auto i = 0; i < 10; ++i)
	{
		Convolution3(m_nutrientsDensity, tmp, m_blur_3x3_idx, m_blur_3x3_weights);
		Convolution3(tmp, m_nutrientsDensity, m_blur_3x3_idx, m_blur_3x3_weights);
	}

	m_nutrientsDensity[Index(5, 32, 8)] = 50.f;
	m_nutrientsDensity[Index(32, 50, 30)] = 100.f;

	m_nutrientsDensity[Index(18, 4, 56)] = 50.f;
	m_nutrientsDensity[Index(27, 48, 27)] = 100.f;
	for (auto i = 0; i < 10; ++i)
	{
		Convolution3(m_nutrientsDensity, tmp, m_blur_3x3_idx, m_blur_3x3_weights);
		Convolution3(tmp, m_nutrientsDensity, m_blur_3x3_idx, m_blur_3x3_weights);
	}

	update_w_sum();
}



void SoilModel::Convolution3(const std::vector<float>& input, std::vector<float>& output, const std::vector<int>& indices, const std::vector<float>& weights) const
{
	auto entries = m_resolution.x * m_resolution.y * m_resolution.z;
	assert(input.size() == entries);
	assert(output.size() == entries);
	assert(indices.size() == weights.size());

	// for a 3D convolution:
	assert(m_resolution.x >= 3);
	assert(m_resolution.y >= 3);
	assert(m_resolution.z >= 3);

	// iterate over all indices that are not part of the boundary, where the whole convolution kernel can be applied
	for (auto x = 1; x < m_resolution.x - 1; ++x)
	{
		for (auto y = 1; y < m_resolution.y - 1; ++y)
		{
			for (auto z = 1; z < m_resolution.z - 1; ++z)
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

void EcoSysLab::SoilModel::update_w_sum()
{
	// count total water:
	m_w_sum = 0.f;
	for (auto i = 0; i < m_w.size(); ++i)
		m_w_sum += m_w[i];
}


void SoilModel::Step()
{
	assert(m_initialized);

	const auto grad_index_x = std::vector<int>({
		Index(-1, 0, 0),
		Index(+1, 0, 0),
		});
	const auto grad_index_y = std::vector<int>({
		Index(0, -1, 0),
		Index(0, +1, 0),
		});
	const auto grad_Index_z = std::vector<int>({
		Index(0, 0, -1),
		Index(0, 0, +1),
		});

	// ----------------- diffusion -----------------
	{
		const auto wx_d = 1.0f / (2.0f * m_dx);
		const auto grad_weights = std::vector<float>({ -wx_d, wx_d });

		// compute gradient dw
		Convolution3(m_w, m_w_grad_x, grad_index_x, grad_weights);
		Convolution3(m_w, m_w_grad_y, grad_index_y, grad_weights);
		Convolution3(m_w, m_w_grad_z, grad_Index_z, grad_weights);

		// boundary conditions for gradient:

		// X:
		for (auto y = 0u; y < m_resolution.y; ++y)
		{
			for (auto z = 0u; z < m_resolution.z; ++z)
			{
				m_w_grad_x[Index(0, y, z)] = m_w[Index(1, y, z)] / (2 * m_dx);
				m_w_grad_x[Index(m_resolution.x - 1, y, z)] = -m_w[Index(m_resolution.x - 2, y, z)] / (2 * m_dx);
			}
		}

		// Y:
		for (auto x = 0u; x < m_resolution.x; ++x)
		{
			for (auto z = 0u; z < m_resolution.z; ++z)
			{
				m_w_grad_y[Index(x, 0, z)] = m_w[Index(x, 1, z)] / (2 * m_dx);
				m_w_grad_y[Index(x, m_resolution.y - 1, z)] = -m_w[Index(x, m_resolution.y - 2, z)] / (2 * m_dx);
			}
		}

		// Z:
		for (auto x = 0u; x < m_resolution.x; ++x)
		{
			for (auto y = 0u; y < m_resolution.y; ++y)
			{
				m_w_grad_z[Index(x, y, 0)] = m_w[Index(x, y, 1)] / (2 * m_dx);
				m_w_grad_z[Index(x, y, m_resolution.z - 1)] = -m_w[Index(x, y, m_resolution.z - 2)] / (2 * m_dx);
			}
		}
		// compute divergence
		Convolution3(m_w_grad_x, m_div_diff_x, grad_index_x, grad_weights);
		Convolution3(m_w_grad_y, m_div_diff_y, grad_index_y, grad_weights);
		Convolution3(m_w_grad_z, m_div_diff_z, grad_Index_z, grad_weights);

		for (auto i = 0; i < m_w.size(); ++i)
		{
			m_div_diff_x[i] *= m_diffusionForce;
			m_div_diff_y[i] *= m_diffusionForce;
			m_div_diff_z[i] *= m_diffusionForce;
		}
	}



	// ------------ gravity ------------
	{

		// Y direction:
		{
			auto a_y = m_gravityForce.y;
			auto wx = a_y * 1.f / (2.f * m_dx);
			auto theta = (a_y * m_dt / m_dx) * (a_y * m_dt / m_dx);
			auto wt = theta * 1 / (2 * m_dt);

			const auto idx = std::vector<int>({
				Index(0,  1, 0),
				Index(0, -1, 0),
				Index(0,  1, 0),
				Index(0,  0, 0),
				Index(0, -1, 0),
				});

			const auto weights = std::vector<float>({
				-wx, wx, wt, -2 * wt, wt
				});

			Convolution3(m_w, m_div_grav_y, idx, weights);
		}

		// TODO: boundary conditions here!
	}

	// apply all the fluxes:
	for (auto i = 0; i < m_w.size(); ++i)
	{
		auto divergence = (m_div_diff_x[i] + m_div_diff_y[i] + m_div_diff_z[i])
			+ (m_div_grav_x[i] + m_div_grav_y[i] + m_div_grav_z[i]);
		// ToDo: Also apply source terms here
		m_w[i] += m_dt * divergence;
	}

	/*
	for (auto x = 1u; x < m_resolution.x-1; x++)
	{
		for (auto y = 1u; y < m_resolution.y-1; y++)
		{
			dw_c[idx(x, y)] =
				  dw_o[idx(x-1, y-1)] + dw_o[idx(x, y-1)] + dw_o[idx(x+1, y-1)]
				+ dw_o[idx(x-1, y  )]                     + dw_o[idx(x+1, y  )]
				+ dw_o[idx(x-1, y+1)] + dw_o[idx(x, y+1)] + dw_o[idx(x+1, y+1)];
		}
	}
	*/
	m_time += m_dt;

	update_w_sum();
}

void EcoSysLab::SoilModel::WaterLogic()
{
	ChangeWater(vec3(0, 10, 0), 10, 12);
	ChangeWater(vec3(8, -10, 4), 20, 12);

	if ((int)(m_time / 20.0) % 2 == 0)
	{
		ChangeWater(vec3(-18, 00, -15), 10, 8);
	}
	if ((int)((m_time + 5) / 19.0) % 2 == 0)
	{
		ChangeWater(vec3(0, 20, -15), -10, 15);
	}
}



float SoilModel::GetWater(const vec3& position) const
{
	return m_w[Index(GetCoordinateFromPosition(position))];
}

float SoilModel::GetDensity(const vec3& position) const
{
	// todo replace by actual values
	if (position.y > 0.0f) return 0.0f;
	return 1.0f / position.y;
}

float SoilModel::GetNutrient(const vec3& position) const
{
	return m_nutrientsDensity[Index(GetCoordinateFromPosition(position))];
}

void SoilModel::ChangeWater(const vec3& center, float amount, float width)
{
	width /= 3.0; // seems ok :D
	auto cutoff = 3.0; // how much of the gaussian to keep

	auto voxel_min = GetCoordinateFromPosition(center - vec3(width * cutoff));
	auto voxel_max = GetCoordinateFromPosition(center + vec3(width * cutoff)) + ivec3(1);

	voxel_min = glm::max(voxel_min, ivec3(0));
	voxel_max = glm::min(voxel_max, static_cast<ivec3>(m_resolution));

	float sum = 0.f;
	for (auto z = voxel_min.z; z < voxel_max.z; ++z)
	{
		for (auto y = voxel_min.y; y < voxel_max.y; ++y)
		{
			for (auto x = voxel_min.x; x < voxel_max.x; ++x)
			{
				auto pos = GetPositionFromCoordinate({ x, y, z });
				auto l = glm::length(pos - center);
				auto v = glm::exp(-l * l / (2 * width * width));
				sum += v;
			}
		}
	}

	for (auto z = voxel_min.z; z < voxel_max.z; ++z)
	{
		for (auto y = voxel_min.y; y < voxel_max.y; ++y)
		{
			for (auto x = voxel_min.x; x < voxel_max.x; ++x)
			{
				auto pos = GetPositionFromCoordinate({ x, y, z });
				auto l = glm::length(pos - center);
				auto v = glm::exp(-l * l / (2 * width * width));
				m_w[Index(x, y, z)] += v / sum * amount;
			}
		}
	}

	/*
	// todo: actually have a subpixel shift on the kernel, depending on the position
	auto cc = GetCoordinateFromPosition(position);
	assert(cc.x > 0 && cc.y > 0 && cc.z > 0); // for the 3x3 kernel below
	assert(cc.x < m_resolution.x-1
		&& cc.y < m_resolution.y - 1
		&& cc.z < m_resolution.z - 1);

	auto c = Index(cc);

	for(auto i=0u; i<m_blur_3x3_idx.size(); ++i)
	{
		m_w[c+m_blur_3x3_idx[i]] += amount * m_blur_3x3_weights[i];
	}
	*/
	update_w_sum();
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
	return Index(m_resolution, x, y, z);
}

int EcoSysLab::SoilModel::Index(const uvec3& resolution, const uvec3& c)
{
	return Index(resolution, c.x, c.y, c.z);
}

int SoilModel::Index(const uvec3& c) const
{
	return Index(m_resolution, c.x, c.y, c.z);
}



ivec3 SoilModel::GetCoordinateFromIndex(const int index) const
{
	return {
		index % m_resolution.x,
		index % (m_resolution.x * m_resolution.y) / m_resolution.x,
		index / (m_resolution.x * m_resolution.y) };
}

ivec3 SoilModel::GetCoordinateFromPosition(const vec3& pos) const
{
	return {
		floor((pos.x - (m_boundingBoxMin.x + m_dx / 2.0)) / m_dx),
		floor((pos.y - (m_boundingBoxMin.y + m_dx / 2.0)) / m_dx),
		floor((pos.z - (m_boundingBoxMin.z + m_dx / 2.0)) / m_dx)
	};
}

vec3 SoilModel::GetPositionFromCoordinate(const glm::ivec3& coordinate) const
{
	return {
		m_boundingBoxMin.x + (m_dx / 2.0) + coordinate.x * m_dx,
		m_boundingBoxMin.y + (m_dx / 2.0) + coordinate.y * m_dx,
		m_boundingBoxMin.z + (m_dx / 2.0) + coordinate.z * m_dx };
}


uvec3 SoilModel::GetVoxelResolution() const
{
	return m_resolution;
}

float SoilModel::GetVoxelSize() const
{
	return m_dx;
}

vec3 SoilModel::GetBoundingBoxMin() const
{
	return m_boundingBoxMin;
}
