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

void SoilModel::Initialize(const SoilParameters& p)
{
	assert(!m_initialized);

	m_diffusionForce = p.m_diffusionForce;
	m_gravityForce = p.m_gravityForce;
	m_dt = p.m_deltaTime;

	m_resolution = p.m_voxelResolution;
	m_dx = p.m_deltaX;
	m_boundingBoxMin = p.m_boundingBoxMin;

	m_boundary_x = p.m_boundary_x;
	m_boundary_y = p.m_boundary_y;
	m_boundary_z = p.m_boundary_z;

	m_blur_3x3_idx = vector<int>({
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

	m_blur_3x3_weights = vector<float>({
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

	auto numVoxels = m_resolution.x * m_resolution.y * m_resolution.z;
	m_soilDensity = vector<float>(numVoxels);
	for(int i = 0; i < m_soilDensity.size(); i++)
	{
		m_soilDensity[i] = p.m_soilDensitySampleFunc(GetPositionFromCoordinate(GetCoordinateFromIndex(i)));
	}

	m_rnd = std::mt19937(std::random_device()());

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

	m_c = empty;
	m_c_grad_x = empty;
	m_c_grad_y = empty;
	m_c_grad_z = empty;

	m_nutrientsDensity = empty;
	auto tmp = empty;

	// todo: use change nutrienst here!

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


	m_version++;
}



void SoilModel::Convolution3(const vector<float>& input, vector<float>& output, const vector<int>& indices, const vector<float>& weights) const
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

	const auto grad_index_x = vector<int>({
		Index(-1, 0, 0),
		Index(+1, 0, 0),
		});
	const auto grad_index_y = vector<int>({
		Index(0, -1, 0),
		Index(0, +1, 0),
		});
	const auto grad_Index_z = vector<int>({
		Index(0, 0, -1),
		Index(0, 0, +1),
		});

	// ----------------- diffusion -----------------
	{
		const auto wx_d = 1.0f / (2.0f * m_dx);
		const auto grad_weights = vector<float>({ -wx_d, wx_d });

		// compute gradient dw
		Convolution3(m_w, m_w_grad_x, grad_index_x, grad_weights);
		Convolution3(m_w, m_w_grad_y, grad_index_y, grad_weights);
		Convolution3(m_w, m_w_grad_z, grad_Index_z, grad_weights);

		// boundary conditions for gradient:

		// X:
		if(Boundary::sink == m_boundary_x)
		{
			for (auto y = 0u; y < m_resolution.y; ++y)
			{
				for (auto z = 0u; z < m_resolution.z; ++z)
				{
					m_w_grad_x[Index(0,                y, z)] =  m_w[Index(1,                y, z)] * wx_d;
					m_w_grad_x[Index(m_resolution.x-1, y, z)] = -m_w[Index(m_resolution.x-2, y, z)] * wx_d;
				}
			}
		}
		else if(Boundary::wrap == m_boundary_x)
		{
			for (auto y = 0u; y < m_resolution.y; ++y)
			{
				for (auto z = 0u; z < m_resolution.z; ++z)
				{
					m_w_grad_x[Index(0,                y, z)] = m_w[Index(m_resolution.x-1, y, z)]*grad_weights[0] + m_w[Index(1, y, z)]*grad_weights[1];
					m_w_grad_x[Index(m_resolution.x-1, y, z)] = m_w[Index(m_resolution.x-2, y, z)]*grad_weights[0] + m_w[Index(0, y, z)]*grad_weights[1];
				}
			}
		}


		// Y:
		if(Boundary::sink == m_boundary_y)
		{
			for (auto x = 0u; x < m_resolution.x; ++x)
			{
				for (auto z = 0u; z < m_resolution.z; ++z)
				{
					m_w_grad_y[Index(x, 0,                z)] =  m_w[Index(x, 1,                z)] * wx_d;
					m_w_grad_y[Index(x, m_resolution.y-1, z)] = -m_w[Index(x, m_resolution.y-2, z)] * wx_d;
				}
			}
		}
		else if(Boundary::wrap == m_boundary_y)
		{
			for (auto x = 0u; x < m_resolution.x; ++x)
			{
				for (auto z = 0u; z < m_resolution.z; ++z)
				{
					m_w_grad_y[Index(x, 0,                z)] = m_w[Index(x, m_resolution.y-1, z)]*grad_weights[0] + m_w[Index(x, 1, z)]*grad_weights[1];
					m_w_grad_y[Index(x, m_resolution.y-1, z)] = m_w[Index(x, m_resolution.y-2, z)]*grad_weights[0] + m_w[Index(x, 0, z)]*grad_weights[1];
				}
			}
		}

		// Z:
		if(Boundary::sink == m_boundary_z)
		{
			for(auto x = 0u; x<m_resolution.x; ++x)
			{
				for (auto y = 0u; y < m_resolution.y; ++y)
				{
					m_w_grad_z[Index(x, y, 0)]                =   m_w[Index(x, y, 1               )] * wx_d;
					m_w_grad_z[Index(x, y, m_resolution.z-1)] = - m_w[Index(x, y, m_resolution.z-2)] * wx_d;
				}
			}
		}
		else if(Boundary::wrap == m_boundary_y)
		{
			for(auto x = 0u; x<m_resolution.x; ++x)
			{
				for (auto y = 0u; y < m_resolution.y; ++y)
				{
					m_w_grad_z[Index(x, y, 0               )] = m_w[Index(x, y, m_resolution.z-1)]*grad_weights[0] + m_w[Index(x, y, 1)]*grad_weights[1];
					m_w_grad_z[Index(x, y, m_resolution.z-1)] = m_w[Index(x, y, m_resolution.z-2)]*grad_weights[0] + m_w[Index(x, y, 0)]*grad_weights[1];
				}
			}
		}

		// compute divergence
		Convolution3(m_w_grad_x, m_div_diff_x, grad_index_x, grad_weights);
		Convolution3(m_w_grad_y, m_div_diff_y, grad_index_y, grad_weights);
		Convolution3(m_w_grad_z, m_div_diff_z, grad_Index_z, grad_weights);

		// X:
		if(Boundary::sink == m_boundary_x)
		{
			for (auto y = 0u; y < m_resolution.y; ++y)
			{
				for (auto z = 0u; z < m_resolution.z; ++z)
				{
					// Nothing for now...
				}
			}
		}
		else if(Boundary::wrap == m_boundary_x)
		{
			for (auto y = 0u; y < m_resolution.y; ++y)
			{
				for (auto z = 0u; z < m_resolution.z; ++z)
				{
					m_div_diff_x[Index(0,                y, z)] = m_w_grad_x[Index(m_resolution.x-1, y, z)]*grad_weights[0] + m_w_grad_x[Index(1, y, z)]*grad_weights[1];
					m_div_diff_x[Index(m_resolution.x-1, y, z)] = m_w_grad_x[Index(m_resolution.x-2, y, z)]*grad_weights[0] + m_w_grad_x[Index(0, y, z)]*grad_weights[1];
				}
			}
		}

		// Y:
		if(Boundary::sink == m_boundary_y)
		{
			// nothing?
		}
		else if(Boundary::wrap == m_boundary_y)
		{
			for (auto x = 0u; x < m_resolution.x; ++x)
			{
				for (auto z = 0u; z < m_resolution.z; ++z)
				{
					m_div_diff_y[Index(x, 0,                z)] = m_w_grad_y[Index(x, m_resolution.y-1, z)]*grad_weights[0] + m_w_grad_y[Index(x, 1, z)]*grad_weights[1];
					m_div_diff_y[Index(x, m_resolution.y-1, z)] = m_w_grad_y[Index(x, m_resolution.y-2, z)]*grad_weights[0] + m_w_grad_y[Index(x, 0, z)]*grad_weights[1];
				}
			}
		}

		// Z:
		if(Boundary::sink == m_boundary_z)
		{
			// nothing?
		}
		else if(Boundary::wrap == m_boundary_z)
		{
			for(auto x = 0u; x<m_resolution.x; ++x)
			{
				for (auto y = 0u; y < m_resolution.y; ++y)
				{
					m_div_diff_z[Index(x, y, 0               )] = m_w_grad_z[Index(x, y, m_resolution.z-1)]*grad_weights[0] + m_w_grad_z[Index(x, y, 1)]*grad_weights[1];
					m_div_diff_z[Index(x, y, m_resolution.z-1)] = m_w_grad_z[Index(x, y, m_resolution.z-2)]*grad_weights[0] + m_w_grad_z[Index(x, y, 0)]*grad_weights[1];
				}
			}
		}



		for (auto i = 0; i < m_w.size(); ++i)
		{
			m_div_diff_x[i] *= m_diffusionForce;
			m_div_diff_y[i] *= m_diffusionForce;
			m_div_diff_z[i] *= m_diffusionForce;
		}
	}



	// ------------ gravity ------------
	// X direction:
	{
		auto a_x = m_gravityForce.x;
		auto wx = a_x * 1.f/(2.f*m_dx);
		auto theta = (a_x * m_dt/m_dx) * (a_x * m_dt/m_dx);
		auto wt = theta * 1/(2*m_dt);

		const auto idx = vector<int>({
			Index( 1, 0, 0),
			Index(-1, 0, 0),
			Index( 1, 0, 0),
			Index( 0, 0, 0),
			Index(-1, 0, 0),
			});

		const auto weights = vector<float>({
			-wx, wx, wt, -2*wt, wt
			});

		Convolution3(m_w, m_div_grav_x, idx, weights);

		if( Boundary::wrap == m_boundary_x)
		{
			const auto last = m_resolution.x-1;
			for (auto y = 0u; y < m_resolution.y; ++y)
			{
				for (auto z = 0u; z < m_resolution.z; ++z)
				{
					m_div_grav_x[Index(0, y, z)] =
						  m_w[Index(1,    y, z)]*weights[0]
						+ m_w[Index(last, y, z)]*weights[1]
						+ m_w[Index(1,    y, z)]*weights[2]
						+ m_w[Index(0,    y, z)]*weights[3]
						+ m_w[Index(last, y, z)]*weights[4];

					m_div_grav_x[Index(last, y, z)] =
						m_w[Index(0,      y, z)]*weights[0]
					  + m_w[Index(last-1, y, z)]*weights[1]
					  + m_w[Index(0,      y, z)]*weights[2]
					  + m_w[Index(last,   y, z)]*weights[3]
					  + m_w[Index(last-1, y, z)]*weights[4];
				}
			}
		}
	}

	// Y direction:
	{
		auto a_y = m_gravityForce.y;
		auto wx = a_y * 1.f/(2.f*m_dx);
		auto theta = (a_y * m_dt/m_dx) * (a_y * m_dt/m_dx);
		auto wt = theta * 1/(2*m_dt);

		const auto idx = vector<int>({
			Index(0,  1, 0),
			Index(0, -1, 0),
			Index(0,  1, 0),
			Index(0,  0, 0),
			Index(0, -1, 0),
			});

		const auto weights = vector<float>({
			-wx, wx, wt, -2*wt, wt
			});

		Convolution3(m_w, m_div_grav_y, idx, weights);

		if( Boundary::wrap == m_boundary_y)
		{
			const auto last = m_resolution.y-1;
			for (auto x = 0u; x < m_resolution.x; ++x)
			{
				for (auto z = 0u; z < m_resolution.z; ++z)
				{
					m_div_grav_y[Index(x, 0, z)] =
						  m_w[Index(x, 1,    z)]*weights[0]
						+ m_w[Index(x, last, z)]*weights[1]
						+ m_w[Index(x, 1,    z)]*weights[2]
						+ m_w[Index(x, 0,    z)]*weights[3]
						+ m_w[Index(x, last, z)]*weights[4];

					m_div_grav_y[Index(x, last, z)] =
					      m_w[Index(x, 0,      z)]*weights[0]
						+ m_w[Index(x, last-1, z)]*weights[1]
						+ m_w[Index(x, 0,      z)]*weights[2]
						+ m_w[Index(x, last,   z)]*weights[3]
						+ m_w[Index(x, last-1, z)]*weights[4];
				}
			}
		}
	}

	// Z direction:
	{
		auto a_z = m_gravityForce.z;
		auto wx = a_z * 1.f/(2.f*m_dx);
		auto theta = (a_z * m_dt/m_dx) * (a_z * m_dt/m_dx);
		auto wt = theta * 1/(2*m_dt);

		const auto idx = vector<int>({
			Index(0, 0,  1),
			Index(0, 0, -1),
			Index(0, 0,  1),
			Index(0, 0,  0),
			Index(0, 0, -1),
			});

		const auto weights = vector<float>({
			-wx, wx, wt, -2*wt, wt
			});

		Convolution3(m_w, m_div_grav_z, idx, weights);

		if( Boundary::wrap == m_boundary_z)
		{
			const auto last = m_resolution.z-1;
			for(auto x = 0u; x<m_resolution.x; ++x)
			{
				for (auto y = 0u; y < m_resolution.y; ++y)
				{
					m_div_grav_z[Index(x, y, 0)] =
						  m_w[Index(x, y, 1   )]*weights[0]
						+ m_w[Index(x, y, last)]*weights[1]
						+ m_w[Index(x, y, 1   )]*weights[2]
						+ m_w[Index(x, y, 0   )]*weights[3]
						+ m_w[Index(x, y, last)]*weights[4];

					m_div_grav_z[Index(x, y, last)] =
						  m_w[Index(x, y, 0     )]*weights[0]
						+ m_w[Index(x, y, last-1)]*weights[1]
						+ m_w[Index(x, y, 0     )]*weights[2]
						+ m_w[Index(x, y, last  )]*weights[3]
						+ m_w[Index(x, y, last-1)]*weights[4];
				}
			}
		}
	}

	// apply all the fluxes:
	for (auto i = 0; i < m_w.size(); ++i)
	{
		auto divergence = (m_div_diff_x[i] + m_div_diff_y[i] + m_div_diff_z[i])
			            + (m_div_grav_x[i] + m_div_grav_y[i] + m_div_grav_z[i]);
		// ToDo: Also apply source terms here
		m_w[i] += m_dt * divergence;
	}

	m_time += m_dt;

	update_w_sum();

	m_version++;
}

void EcoSysLab::SoilModel::Irrigation()
{
	m_rnd = std::mt19937(27);

	auto bb_min = GetBoundingBoxMin();
	auto bb_max = GetBoundingBoxMax();
	std::uniform_real_distribution<> dist_x(bb_min.x, bb_max.x);
	std::uniform_real_distribution<> dist_y(bb_min.y, bb_max.y);
	std::uniform_real_distribution<> dist_z(bb_min.z, bb_max.z);

	std::uniform_real_distribution<> width(0.5, 2);

	for(auto i=0; i<30; ++i)
	{
		auto pos = vec3(dist_x(m_rnd), dist_y(m_rnd), dist_z(m_rnd));
		auto amount = m_irrigationAmount * GetDensity(pos);
		ChangeWater(pos, amount, width(m_rnd));
	}

	/*
	if ((int)(m_time / 20.0) % 2 == 0)
	{
		ChangeWater(vec3(-18, 0, 15), 10, 8);
	}
	if ((int)((m_time + 5) / 19.0) % 2 == 0)
	{
		ChangeWater(vec3(0, 20, -15), -10, 15);
	}*/
}



float SoilModel::GetWater(const vec3& position) const
{
	return m_w[Index(GetCoordinateFromPosition(position))];
}

float SoilModel::GetDensity(const vec3& position) const
{
	return m_soilDensity[Index(GetCoordinateFromPosition(position))];
}

float SoilModel::GetNutrient(const vec3& position) const
{
	return m_nutrientsDensity[Index(GetCoordinateFromPosition(position))];
}

void SoilModel::ChangeField(vector<float>& field, const vec3& center, float amount, float width)
{
	width /= 3.0; // seems ok :D
	auto cutoff = 3.0; // how much of the gaussian to keep

	auto voxel_min = GetCoordinateFromPosition(center - vec3(width * cutoff));
	auto voxel_max = GetCoordinateFromPosition(center + vec3(width * cutoff)) + ivec3(2);

	voxel_min = glm::max(voxel_min, ivec3(0));
	voxel_max = glm::min(voxel_max, static_cast<ivec3>(m_resolution)-ivec3(1));

	// the <= is important here
	float sum = 0.f;
	for (auto z=voxel_min.z; z <= voxel_max.z; ++z)
	{
		for (auto y=voxel_min.y; y <= voxel_max.y; ++y)
		{
			for (auto x=voxel_min.x; x <= voxel_max.x; ++x)
			{
				auto pos = GetPositionFromCoordinate({ x, y, z });
				auto l = glm::length(pos - center);
				auto v = glm::exp( - l*l / (2* width*width));
				sum += v;
			}
		}
	}

	for (auto z = voxel_min.z; z <= voxel_max.z; ++z)
	{
		for (auto y = voxel_min.y; y <= voxel_max.y; ++y)
		{
			for (auto x = voxel_min.x; x <= voxel_max.x; ++x)
			{
				auto pos = GetPositionFromCoordinate({ x, y, z });
				auto l = glm::length(pos - center);
				auto v = glm::exp( - l*l / (2* width*width));
				field[Index(x, y, z)] += v / sum * amount;
			}
		}
	}

}

void SoilModel::ChangeWater(const vec3& center, float amount, float width)
{
	ChangeField(m_w, center, amount, width);
	update_w_sum();
}

void SoilModel::ChangeDensity(const vec3& center, float amount, float width)
{
	ChangeField(m_soilDensity, center, amount, width);
}

void SoilModel::ChangeNutrient(const vec3& center, float amount, float width)
{
	ChangeField(m_nutrientsDensity, center, amount, width);
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
		index %  m_resolution.x,
		index % (m_resolution.x * m_resolution.y) / m_resolution.x,
		index / (m_resolution.x * m_resolution.y) };
}

ivec3 SoilModel::GetCoordinateFromPosition(const vec3& pos) const
{
	return {
		floor((pos.x - (m_boundingBoxMin.x + m_dx/2.0)) / m_dx),
		floor((pos.y - (m_boundingBoxMin.y + m_dx/2.0)) / m_dx),
		floor((pos.z - (m_boundingBoxMin.z + m_dx/2.0)) / m_dx)
	};
}

vec3 SoilModel::GetPositionFromCoordinate(const ivec3& coordinate) const
{
	return {
		m_boundingBoxMin.x + (m_dx/2.0) + coordinate.x * m_dx,
		m_boundingBoxMin.y + (m_dx/2.0) + coordinate.y * m_dx,
		m_boundingBoxMin.z + (m_dx/2.0) + coordinate.z * m_dx
	};
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

vec3 EcoSysLab::SoilModel::GetBoundingBoxMax() const
{
	return m_boundingBoxMin + vec3(m_resolution)*m_dx;
}
