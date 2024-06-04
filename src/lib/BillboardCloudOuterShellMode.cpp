#include "BillboardCloud.hpp"
using namespace EvoEngine;


class Discretization {
public:
	class Bin
	{
	public:
		float m_density;
		float m_thetaMin;
		float m_thetaMax;
		float m_phiMin;
		float m_phiMax;
		float m_roMin;
		float m_roMax;
		float m_thetaCenter;
		float m_phiCenter;
		float m_roCenter;
		glm::vec3 m_centerNormal;

		Bin()
			:m_density(0.0f),
			m_thetaMin(0.0f),
			m_thetaMax(0.0f),
			m_phiMin(0.0f),
			m_phiMax(0.0f),
			m_roMin(0.0f),
			m_roMax(0.0f),
			m_thetaCenter(0.0f),
			m_phiCenter(0.0f),
			m_roCenter(0.0f)
		{
		}

		Bin(const float thetaMin, const float thetaMax, const float phiMin, const float phiMax, const float roMin, const float roMax, const float density = 0)
			: m_density(density), m_thetaMin(thetaMin), m_thetaMax(thetaMax), m_phiMin(phiMin), m_phiMax(phiMax), m_roMin(roMin), m_roMax(roMax)
		{
			CalculateCenter();
			CalculateCenterNormal();
		}

		Bin(const Bin& bin)
		{
			m_density = bin.m_density;
			m_thetaMin = bin.m_thetaMin;
			m_thetaMax = bin.m_thetaMax;
			m_phiMin = bin.m_phiMin;
			m_phiMax = bin.m_phiMax;
			m_roMin = bin.m_roMin;
			m_roMax = bin.m_roMax;
			m_thetaCenter = bin.m_thetaCenter;
			m_phiCenter = bin.m_phiCenter;
			m_roCenter = bin.m_roCenter;
			m_centerNormal = bin.m_centerNormal;
		}

		Bin& operator=(const Bin& bin) = default;

	private:
		void CalculateCenter()
		{
			m_thetaCenter = (m_thetaMin + m_thetaMax) / 2;
			m_phiCenter = (m_phiMin + m_phiMax) / 2;
			m_roCenter = (m_roMin + m_roMax) / 2;
		}

		// calculate bin center's normal
		void CalculateCenterNormal()
		{
			const float z = glm::sin(m_phiCenter);
			const float xy = glm::cos(m_phiCenter);
			const float x = xy * glm::cos(m_thetaCenter);
			const float y = xy * glm::sin(m_thetaCenter);

			m_centerNormal = glm::vec3(x, y, z);
		}
	};
	/// m_bins
	std::vector<std::vector<std::vector<Bin>>> m_bins;
	/// fail-safe mode para
	bool m_failSafeModeTriggered;
	/// fitted plane in fail-safe mode 
	std::vector<BillboardCloud::ClusterTriangle> m_bestFittedPlaneValidTriangle;

	float m_epsilon;
	float m_weightPenalty;
	/// discretization num
	int m_discretizeThetaNum;
	int m_discretizePhiNum;
	int m_discretizeRoNum;
	/// range
	float m_thetaMax;
	float m_thetaMin;
	float m_phiMax;
	float m_phiMin;
	float m_roMax;
	float m_roMin;
	/// gap
	float m_thetaGap;
	float m_phiGap;
	float m_roGap;

	Discretization(const float roMax, const float epsilon, const int discretizeThetaNum, const int discretizePhiNum, const int discretizeRoNum)
		:m_failSafeModeTriggered(false),
		m_epsilon(epsilon),
		m_weightPenalty(10),
		m_discretizeThetaNum(discretizeThetaNum),
		m_discretizePhiNum(discretizePhiNum),
		m_discretizeRoNum(discretizeRoNum),
		m_thetaMax(2 * glm::pi<float>()),
		m_thetaMin(0),
		m_phiMax(glm::pi<float>() / 2),   // recommend "10" in paper
		m_phiMin(-glm::pi<float>() / 2),
		m_roMax(roMax),
		m_roMin(0)
	{
		m_thetaGap = (m_thetaMax - m_thetaMin) / static_cast<float>(discretizeThetaNum);
		m_phiGap = (m_phiMax - m_phiMin) / static_cast<float>(discretizePhiNum);
		m_roGap = (roMax - m_roMin) / static_cast<float>(discretizeRoNum);

		for (int i = 0; i < discretizeRoNum; i++)
		{
			std::vector<std::vector<Bin>> tmp1;
			for (int j = 0; j < discretizePhiNum; j++)
			{
				std::vector<Bin> tmp2;
				for (int k = 0; k < discretizeThetaNum; k++)
				{
					const float thetaMinTmp = m_thetaGap * k + m_thetaMin;
					const float thetaMaxTmp = thetaMinTmp + m_thetaGap;
					const float phiMinTmp = m_phiGap * j + m_phiMin;
					const float phiMaxTmp = phiMinTmp + m_phiGap;
					const float roMinTmp = m_roGap * i + m_roMin;
					const float roMaxTmp = roMinTmp + m_roGap;
					Bin bin(thetaMinTmp, thetaMaxTmp, phiMinTmp, phiMaxTmp, roMinTmp, roMaxTmp);
					tmp2.emplace_back(bin);
				}
				tmp1.emplace_back(tmp2);
			}
			m_bins.emplace_back(tmp1);
		}
	}
	/// trans the spherical coordinate of a plane into the normal vector
	static glm::vec3 SphericalCoordToNormal(const float theta, const float phi)
	{
		const float z = glm::sin(phi);
		const float xy = glm::cos(phi);
		const float x = xy * glm::cos(theta);
		const float y = xy * glm::sin(theta);
		return { x, y, z };
	}
	/// compute the min and max value of ro in the case of triangle is valid for the specific theta and phi range
	[[nodiscard]] bool ComputeRoMinMax(glm::vec2& minMax, const BillboardCloud::Element& element, const BillboardCloud::ClusterTriangle& clusterTriangle, float curThetaMin, float curThetaMax, float curPhiMin, float curPhiMax) const
	{
		const auto normal1 = SphericalCoordToNormal(curThetaMin, curPhiMin);
		const auto normal2 = SphericalCoordToNormal(curThetaMin, curPhiMax);
		const auto normal3 = SphericalCoordToNormal(curThetaMax, curPhiMin);
		const auto normal4 = SphericalCoordToNormal(curThetaMax, curPhiMax);

		const auto& triangle = element.m_triangles.at(clusterTriangle.m_triangleIndex);
		const auto p0 = element.m_vertices.at(triangle.x).m_position;
		const auto p1 = element.m_vertices.at(triangle.y).m_position;
		const auto p2 = element.m_vertices.at(triangle.z).m_position;

		const float roP0N1 = glm::dot(p0, normal1);
		const float roP0N2 = glm::dot(p0, normal2);
		const float roP0N3 = glm::dot(p0, normal3);
		const float roP0N4 = glm::dot(p0, normal4);

		const float roP1N1 = glm::dot(p1, normal1);
		const float roP1N2 = glm::dot(p1, normal2);
		const float roP1N3 = glm::dot(p1, normal3);
		const float roP1N4 = glm::dot(p1, normal4);

		const float roP2N1 = glm::dot(p2, normal1);
		const float roP2N2 = glm::dot(p2, normal2);
		const float roP2N3 = glm::dot(p2, normal3);
		const float roP2N4 = glm::dot(p2, normal4);

		const float tmp0[] = { roP0N1 - m_epsilon, roP0N2 - m_epsilon, roP0N3 - m_epsilon, roP0N4 - m_epsilon };
		const float tmp1[] = { roP1N1 - m_epsilon, roP1N2 - m_epsilon, roP1N3 - m_epsilon, roP1N4 - m_epsilon };
		const float tmp2[] = { roP2N1 - m_epsilon, roP2N2 - m_epsilon, roP2N3 - m_epsilon, roP2N4 - m_epsilon };
		const float roP0MinN = glm::min(glm::min(tmp0[0], tmp0[1]), glm::min(tmp0[2], tmp0[3]));
		const float roP1MinN = glm::min(glm::min(tmp1[0], tmp1[1]), glm::min(tmp1[2], tmp1[3]));
		const float roP2MinN = glm::min(glm::min(tmp2[0], tmp2[1]), glm::min(tmp2[2], tmp2[3]));
		const float tmp3[] = { roP0MinN, roP1MinN, roP2MinN };
		//roMin

		float roMaxPMinN = glm::max(tmp3[0], glm::max(tmp3[1], tmp3[2]));


		const float tmp4[] = { roP0N1 + m_epsilon, roP0N2 + m_epsilon, roP0N3 + m_epsilon, roP0N4 + m_epsilon };
		const float tmp5[] = { roP1N1 + m_epsilon, roP1N2 + m_epsilon, roP1N3 + m_epsilon, roP1N4 + m_epsilon };
		const float tmp6[] = { roP2N1 + m_epsilon, roP2N2 + m_epsilon, roP2N3 + m_epsilon, roP2N4 + m_epsilon };
		const float roP0MaxN = glm::max(glm::max(tmp4[0], tmp4[1]), glm::max(tmp4[2], tmp4[3]));
		const float roP1MaxN = glm::max(glm::max(tmp5[0], tmp5[1]), glm::max(tmp5[2], tmp5[3]));
		const float roP2MaxN = glm::max(glm::max(tmp6[0], tmp6[1]), glm::max(tmp6[2], tmp6[3]));
		const float tmp7[] = { roP0MaxN, roP1MaxN, roP2MaxN };
		//roMax
		float roMinPMaxN = glm::min(tmp7[0], glm::min(tmp7[1], tmp7[2]));
		if (roMinPMaxN < m_roMin) false;

		roMaxPMinN = glm::clamp(roMaxPMinN, m_roMin, m_roMax);
		roMinPMaxN = glm::min(roMinPMaxN, m_roMax);
		minMax = { roMaxPMinN, roMinPMaxN };
		return true;
	}
	void UpdateDensity(const std::vector<BillboardCloud::Element>& elements, const std::vector<BillboardCloud::ClusterTriangle>& clusterTriangles, const bool add)
	{
		if (clusterTriangles.empty())
		{
			EVOENGINE_ERROR("ERROR: The size of the input triangles is 0, that means in the last iteration, there is no fitted plane found!");
			return;
		}

		const float time = clock();
		for (auto& clusterTriangle : clusterTriangles)
		{
			const auto& element = elements.at(clusterTriangle.m_elementIndex);
			const auto triangleNormal = element.CalculateNormal(clusterTriangle.m_triangleIndex);
			const auto triangleArea = element.CalculateArea(clusterTriangle.m_triangleIndex);
			for (int i = 0; i < m_discretizePhiNum; i++)        // phiCoord
			{
				for (int j = 0; j < m_discretizeThetaNum; j++)  // thetaCoord
				{
					glm::vec2 roMaxMin;
					if (!ComputeRoMinMax(roMaxMin, element, clusterTriangle,
						m_thetaMin + j * m_thetaGap,
						m_thetaMin + (j + 1) * m_thetaGap,
						m_phiMin + i * m_phiGap,
						m_phiMin + (i + 1) * m_phiGap))
					{
						continue;
					}
					//if (roMaxMin.x < 0.f && roMaxMin.y < 0.f)
					//	continue;
					const float roMin = roMaxMin.x;
					const float roMax = roMaxMin.y;

					// add coverage, m_bins between (roMin, roMax)
					const int roCoordMin = glm::clamp(static_cast<int>((roMin - m_roMin) / m_roGap), 0, m_discretizeRoNum - 1);
					const int roCoordMax = glm::clamp(static_cast<int>((roMax - m_roMin) / m_roGap), 0, m_discretizeRoNum - 1);

					if (roCoordMax - roCoordMin > 2)
					{
						if (add)
						{
							// use the center point's normal of the bin to calculate the projected triangle area
							m_bins[roCoordMin][i][j].m_density += triangleArea *
								glm::abs(glm::dot(triangleNormal, m_bins[roCoordMin][i][j].m_centerNormal)) *
								((m_roMin + static_cast<float>(roCoordMin + 1) * m_roGap - roMin) / m_roGap);
						}
						else
						{
							m_bins[roCoordMin][i][j].m_density -= triangleArea *
								glm::abs(glm::dot(triangleNormal, m_bins[roCoordMin][i][j].m_centerNormal)) *
								((m_roMin + static_cast<float>(roCoordMin + 1) * m_roGap - roMin) / m_roGap);
						}
						for (int k = roCoordMin + 1; k < roCoordMax; k++)
						{
							if (add)
							{
								m_bins[k][i][j].m_density += triangleArea *
									glm::abs(glm::dot(triangleNormal, m_bins[k][i][j].m_centerNormal));
							}
							else
							{
								m_bins[k][i][j].m_density -= triangleArea *
									glm::abs(glm::dot(triangleNormal, m_bins[k][i][j].m_centerNormal));
							}
						}
						if (add)
						{
							m_bins[roCoordMax][i][j].m_density += triangleArea *
								glm::abs(glm::dot(triangleNormal, m_bins[roCoordMax][i][j].m_centerNormal)) *
								((roMax - (static_cast<float>(roCoordMax) * m_roGap + m_roMin)) / m_roGap);
						}
						else
						{
							m_bins[roCoordMax][i][j].m_density -= triangleArea *
								glm::abs(glm::dot(triangleNormal, m_bins[roCoordMax][i][j].m_centerNormal)) *
								((roMax - (static_cast<float>(roCoordMax) * m_roGap + m_roMin)) / m_roGap);
						}
					}
					else if (roCoordMax - roCoordMin == 1)
					{
						if (add)
						{
							m_bins[roCoordMin][i][j].m_density += triangleArea *
								glm::abs(glm::dot(triangleNormal, m_bins[roCoordMin][i][j].m_centerNormal)) *
								((m_roMin + static_cast<float>(roCoordMin + 1) * m_roGap - roMin) / m_roGap);
						}
						else
						{
							m_bins[roCoordMin][i][j].m_density -= triangleArea *
								glm::abs(glm::dot(triangleNormal, m_bins[roCoordMin][i][j].m_centerNormal)) *
								((m_roMin + static_cast<float>(roCoordMin + 1) * m_roGap - roMin) / m_roGap);
						}

						if (add)
						{
							m_bins[roCoordMax][i][j].m_density += triangleArea *
								glm::abs(glm::dot(triangleNormal, m_bins[roCoordMax][i][j].m_centerNormal)) *
								((roMax - (static_cast<float>(roCoordMax) * m_roGap + m_roMin)) / m_roGap);
						}
						else
						{
							m_bins[roCoordMax][i][j].m_density -= triangleArea *
								glm::abs(glm::dot(triangleNormal, m_bins[roCoordMax][i][j].m_centerNormal)) *
								((roMax - (static_cast<float>(roCoordMax) * m_roGap + m_roMin)) / m_roGap);
						}
					}
					else if (roCoordMax - roCoordMin == 0)
					{
						if (add)
						{
							m_bins[roCoordMin][i][j].m_density += triangleArea *
								glm::abs(glm::dot(triangleNormal, m_bins[roCoordMin][i][j].m_centerNormal));
						}
						else
						{
							m_bins[roCoordMin][i][j].m_density -= triangleArea *
								glm::abs(glm::dot(triangleNormal, m_bins[roCoordMin][i][j].m_centerNormal));
						}
					}

					// add penalty ,m_bins between (ro_min - epsilon, ro_min)
					if (roMin - m_epsilon - m_roMin > 0)
					{
						int roMinMinusEpsilonCoord = (roMin - m_epsilon - m_roMin) / m_roGap;
						for (int m = roMinMinusEpsilonCoord; m <= roCoordMin; m++)
						{
							if (add)
							{
								m_bins[m][i][j].m_density -= triangleArea *
									glm::abs(glm::dot(triangleNormal, m_bins[m][i][j].m_centerNormal)) *
									m_weightPenalty;
							}
							else
							{
								m_bins[m][i][j].m_density += triangleArea *
									glm::abs(glm::dot(triangleNormal, m_bins[m][i][j].m_centerNormal)) *
									m_weightPenalty;
							}
						}
					}
				}
			}
		}
		EVOENGINE_LOG("Updating_density... [time :" + std::to_string((clock() - time) / 1000.f) + "s]");
	}

	[[nodiscard]] float ComputeMaxDensity(glm::ivec3& binIndex) const
	{
		// pick bin with max density
		float maxDensity = -FLT_MAX;
		for (int i = 0; i < m_discretizeRoNum; i++)
		{
			for (int j = 0; j < m_discretizePhiNum; j++)
			{
				for (int k = 0; k < m_discretizeThetaNum; k++)
				{
					if (float tmp = m_bins[i][j][k].m_density; maxDensity < tmp)
					{
						maxDensity = tmp;
						binIndex.x = i;
						binIndex.y = j;
						binIndex.z = k;
					}
				}
			}
		}
		return maxDensity;
	}

	[[nodiscard]] std::vector<BillboardCloud::ClusterTriangle> ComputeBinValidSet(const std::vector<BillboardCloud::Element>& elements, const std::vector<BillboardCloud::ClusterTriangle>& clusterTriangles, const Bin& bin) const
	{
		std::vector<BillboardCloud::ClusterTriangle> binValidSet;
		for (auto& clusterTriangle : clusterTriangles)
		{
			const auto& element = elements.at(clusterTriangle.m_elementIndex);
			// we use the notion of "simple validity":
			// that is a bin is valid for a triangle as long as there exists a valid plane for the triangle in the bin 
			// if the ro min and ro max is in the range of bin's ro range, we think this triangle is valid for the bin
			glm::vec2 roMinMax;
			if (!ComputeRoMinMax(roMinMax, element, clusterTriangle, bin.m_thetaMin, bin.m_thetaMax, bin.m_phiMin, bin.m_phiMax))
			{
				continue;
			}

			if (!(roMinMax.y < bin.m_roMin) &&
				!(roMinMax.x > bin.m_roMax))
			{
				binValidSet.emplace_back(clusterTriangle);
			}
		}
		return binValidSet;
	}
	std::vector<int> ComputePlaneValidSetIndex(const std::vector<BillboardCloud::Element>& elements, const std::vector<BillboardCloud::ClusterTriangle>& clusterTriangles, const Plane& plane) const
	{
		std::vector<int> planeValidSetIndex;
		const auto planeNormal = plane.GetNormal();
		const auto planeDistance = plane.GetDistance();
		for (int i = 0; i < clusterTriangles.size(); i++)
		{
			const auto& clusterTriangle = clusterTriangles.at(i);
			const auto& element = elements.at(clusterTriangle.m_elementIndex);
			const auto& triangle = element.m_triangles.at(clusterTriangle.m_triangleIndex);
			const auto p0 = element.m_vertices.at(triangle.x).m_position;
			const auto p1 = element.m_vertices.at(triangle.y).m_position;
			const auto p2 = element.m_vertices.at(triangle.z).m_position;

			const float roP0N = glm::abs(glm::dot(p0, planeNormal));
			const float roP1N = glm::abs(glm::dot(p1, planeNormal));
			const float roP2N = glm::abs(glm::dot(p2, planeNormal));
			const float tmp0[] = { roP0N - m_epsilon ,roP1N - m_epsilon ,roP2N - m_epsilon };
			const float tmp1[] = { roP0N + m_epsilon ,roP1N + m_epsilon ,roP2N + m_epsilon };

			const float roMinTmp = glm::min(tmp0[0], glm::min(tmp0[1], tmp0[2]));
			const float roMaxTmp = glm::max(tmp1[0], glm::max(tmp1[1], tmp1[2]));

			if (planeDistance > roMinTmp && planeDistance < roMaxTmp)
				planeValidSetIndex.emplace_back(i);
		}
		return planeValidSetIndex;
	}
	void ComputeDensity(const std::vector<BillboardCloud::Element>& elements, const std::vector<BillboardCloud::ClusterTriangle>& clusterTriangles, Bin& bin) const
	{
		for (auto& clusterTriangle : clusterTriangles)
		{
			const auto& element = elements.at(clusterTriangle.m_elementIndex);
			const auto triangleNormal = element.CalculateNormal(clusterTriangle.m_triangleIndex);
			const auto triangleArea = element.CalculateArea(clusterTriangle.m_triangleIndex);
			glm::vec2 roMinMax;
			if (!ComputeRoMinMax(roMinMax, element, clusterTriangle, bin.m_thetaMin, bin.m_thetaMax, bin.m_phiMin, bin.m_phiMax))
			{
				continue;
			}
			// add coverage
			const float curRoMin = roMinMax.x;
			const float curRoMax = roMinMax.y;
			const float curRoGap = bin.m_roMax - bin.m_roMin;
			if (curRoMin < bin.m_roMin && curRoMax > bin.m_roMin && curRoMax < bin.m_roMax)
			{
				bin.m_density += triangleArea * glm::abs(glm::dot(triangleNormal, bin.m_centerNormal)) * (curRoMax - bin.m_roMin) / curRoGap;
			}
			else if (curRoMin > bin.m_roMin && curRoMin < bin.m_roMax && curRoMax > bin.m_roMax)
			{
				bin.m_density += triangleArea * glm::abs(glm::dot(triangleNormal, bin.m_centerNormal)) * (bin.m_roMax - curRoMin) / curRoGap;
			}
			else if (curRoMin >= bin.m_roMin && curRoMax <= bin.m_roMax)
			{
				bin.m_density += triangleArea * glm::abs(glm::dot(triangleNormal, bin.m_centerNormal));
			}
		}
	}

	std::vector<Bin> GetNeighbors(const Bin& bin) const
	{
		const float curBinThetaGap = bin.m_thetaMax - bin.m_thetaMin;
		const float curBinPhiGap = bin.m_phiMax - bin.m_phiMin;
		const float curBinRoGap = bin.m_roMax - bin.m_roMin;

		std::vector<Bin> binsTmp;
		for (int i = -1; i <= 1; i++)
		{
			for (int j = -1; j <= 1; j++)
			{
				for (int k = -1; k <= 1; k++)
				{
					float neighborThetaMin = bin.m_thetaMin + curBinThetaGap * k;
					float neighborThetaMax = neighborThetaMin + curBinThetaGap;
					float neighborPhiMin = bin.m_phiMin + curBinPhiGap * j;
					float neighborPhiMax = neighborPhiMin + curBinPhiGap;
					const float neighborRoMin = bin.m_roMin + curBinRoGap * i;
					const float neighborRoMax = neighborRoMin + curBinRoGap;

					if (neighborPhiMin < m_phiMin)
					{
						neighborPhiMin = -(neighborPhiMin - m_phiMin);
						neighborThetaMin = neighborThetaMin + glm::pi<float>();
					}
					if (neighborPhiMax > m_phiMax)
					{
						neighborPhiMax = glm::pi<float>() / 2 - (neighborPhiMax - m_phiMax);
						neighborThetaMin = neighborThetaMin + glm::pi<float>();
					}

					if (neighborThetaMin > 2 * glm::pi<float>())
					{
						neighborThetaMin = neighborThetaMin - 2 * glm::pi<float>();
					}
					if (neighborThetaMax > 2 * glm::pi<float>())
					{
						neighborThetaMax = neighborThetaMax - 2 * glm::pi<float>();
					}

					// if the bin's ro range is outside the specific range, discard it!
					if (neighborRoMin < m_roMin || neighborRoMax > m_roMax)
						continue;

					Bin binTmp(neighborThetaMin, neighborThetaMax, neighborPhiMin, neighborPhiMax, neighborRoMin, neighborRoMax);
					binsTmp.emplace_back(binTmp);
				}
			}
		}
		return binsTmp;
	}

	static std::vector<Bin> SubdivideBin(const Bin& bin)
	{
		std::vector<Bin> binsTmp;
		for (int i = 0; i <= 1; i++)
		{
			for (int j = 0; j <= 1; j++)
			{
				for (int k = 0; k <= 1; k++)
				{
					const float curThetaMin = bin.m_thetaMin + (bin.m_thetaMax - bin.m_thetaMin) / 2 * k;
					const float curThetaMax = curThetaMin + (bin.m_thetaMax - bin.m_thetaMin) / 2;
					const float curPhiMin = bin.m_phiMin + (bin.m_phiMax - bin.m_phiMin) / 2 * j;
					const float curPhiMax = curPhiMin + (bin.m_phiMax - bin.m_phiMin) / 2;
					const float curRoMin = bin.m_roMin + (bin.m_roMax - bin.m_roMin) / 2 * i;
					const float curRoMax = curRoMin + (bin.m_roMax - bin.m_roMin) / 2;

					Bin binTmp(curThetaMin, curThetaMax, curPhiMin, curPhiMax, curRoMin, curRoMax);
					binsTmp.emplace_back(binTmp);
				}
			}
		}
		return binsTmp;
	}
	[[nodiscard]] Plane RefineBin(const std::vector<BillboardCloud::Element>& elements, const std::vector<BillboardCloud::ClusterTriangle>& validSet, const Bin& maxDensityBin)
	{
		Plane centerPlane{ maxDensityBin.m_centerNormal, maxDensityBin.m_roCenter };
		std::vector<int> centerPlaneValidSetIndex = ComputePlaneValidSetIndex(elements, validSet, centerPlane);
		if (centerPlaneValidSetIndex.size() == validSet.size())
		{
			if (maxDensityBin.m_thetaCenter > glm::pi<float>())
			{
				centerPlane = Plane(-centerPlane.GetNormal(), centerPlane.GetDistance());
			}
			float maxDis = 0.0f;
			for (auto& clusterTriangle : validSet)
			{
				const auto& element = elements.at(clusterTriangle.m_elementIndex);
				const auto& triangle = element.m_triangles.at(clusterTriangle.m_triangleIndex);
				const auto p0 = element.m_vertices.at(triangle.x).m_position;
				const auto p1 = element.m_vertices.at(triangle.y).m_position;
				const auto p2 = element.m_vertices.at(triangle.z).m_position;
				float d0 = centerPlane.CalculatePointDistance(p0);
				float d1 = centerPlane.CalculatePointDistance(p1);
				float d2 = centerPlane.CalculatePointDistance(p2);
				maxDis = d0 > d1 ? d0 : d1;
				maxDis = maxDis > d2 ? maxDis : d2;
			}
			return centerPlane;
		}

		Bin binMax;
		binMax.m_density = FLT_MIN;

		// pick the bin and its 26 neighbors (if have)
		std::vector<Bin> neighborBins = GetNeighbors(maxDensityBin);
		for (auto& neighborBin : neighborBins)
		{
			// subdivide the bin into 8 bins
			std::vector<Bin> subdividedBins = SubdivideBin(neighborBin);
			for (auto& subdividedBin : subdividedBins)
			{
				// pick the subdivide bin with max density
				ComputeDensity(elements, validSet, subdividedBin);
				if (subdividedBin.m_density > binMax.m_density)
				{
					binMax = subdividedBin;
				}
			}
		}
		std::vector<BillboardCloud::ClusterTriangle> binMaxValidSet = ComputeBinValidSet(elements, validSet, binMax);
		if (binMaxValidSet.empty())
		{
			EVOENGINE_ERROR("ERROR: subBinMax has no valid set, we will simply return the last densest bin's center plane!");

			// if the centerPlane has no valid set in the current remain sets, the iteration will end up with infinite loop!!!
			if (!centerPlaneValidSetIndex.empty())
			{
				EVOENGINE_ERROR("INFO: but last densest bin's center plane has valid set");
				EVOENGINE_ERROR("INFO: so we can simply return the last densest bin's center plane!");

				return centerPlane;
			}
			else
			{
				EVOENGINE_ERROR("ERROR: the centerPlane has no valid set in the current remain sets too");
				EVOENGINE_ERROR("INFO: so we return the best fitted plane of the last densest bin's valid set ");

				m_failSafeModeTriggered = true;
				m_bestFittedPlaneValidTriangle = validSet;
				float totalArea = 0.f;
				glm::vec3 centroidSum = glm::vec3(0.f);
				for (auto& clusterTriangle : validSet)
				{
					const auto& element = elements.at(clusterTriangle.m_elementIndex);
					const auto& triangle = element.m_triangles.at(clusterTriangle.m_triangleIndex);
					const auto triangleArea = element.CalculateArea(triangle);
					const auto centroid = element.CalculateCentroid(triangle);
					totalArea += triangleArea;
					centroidSum += centroid * triangleArea;

				}
				auto centroid = centroidSum / totalArea;
				auto normal = glm::normalize(glm::vec3(centroid.y, centroid.z, centroid.x));
				if (glm::dot(centroid, normal) < 0)
				{
					normal = -normal;
				}
				float distance = glm::abs(glm::dot(centroid, normal));
				return Plane(normal, distance);
			}
		}
		return RefineBin(elements, binMaxValidSet, binMax);
	}
};



std::vector<BillboardCloud::Cluster> BillboardCloud::DefaultClusterize(
	std::vector<ClusterTriangle> operatingTriangles, const ClusterizationSettings& clusterizeSettings)
{
	BoundingSphere boundingSphere;
	boundingSphere.Initialize(m_elements);
	const auto& settings = clusterizeSettings.m_originalClusterizationSettings;
	const int roNum = static_cast<int>(1.5f / settings.m_epsilonPercentage);
	const float epsilon = boundingSphere.m_radius * settings.m_epsilonPercentage;

	float maxNormalDistance = 0.f;
	for (const auto& triangle : operatingTriangles)
	{
		maxNormalDistance = glm::max(maxNormalDistance, CalculateNormalDistance(triangle));
	}

	m_skippedTriangles.clear();

	std::vector<Cluster> retVal;

	int epoch = 0;

	Discretization discretization(maxNormalDistance, epsilon, settings.m_discretizationSize, settings.m_discretizationSize, roNum);
	discretization.UpdateDensity(m_elements, operatingTriangles, true);

	while (!operatingTriangles.empty())
	{
		for (int i = 0; i < operatingTriangles.size(); i++)
		{
			operatingTriangles[i].m_index = i;
		}
		std::vector<int> selectedTriangleIndices;
		Cluster newCluster;
		glm::ivec3 maxDensityBinCoordinate;
		float maxDensity = discretization.ComputeMaxDensity(maxDensityBinCoordinate);
		const auto& maxDensityBin = discretization.m_bins[maxDensityBinCoordinate.x][maxDensityBinCoordinate.y][maxDensityBinCoordinate.z];
		const auto binValidSet = discretization.ComputeBinValidSet(m_elements, operatingTriangles, maxDensityBin);
		if (!binValidSet.empty())
		{
			std::vector<ClusterTriangle> planeValidSet;
			newCluster.m_clusterPlane = discretization.RefineBin(m_elements, binValidSet, maxDensityBin);

			if (discretization.m_failSafeModeTriggered)
			{
				planeValidSet = discretization.m_bestFittedPlaneValidTriangle;
				discretization.m_bestFittedPlaneValidTriangle.clear();

				EVOENGINE_ERROR("Fitted_triangle_num: " + std::to_string(planeValidSet.size()));

				// update density by removing the fitted triangle 
				discretization.UpdateDensity(m_elements, planeValidSet, false);

				// store bbc and corresponding fitted triangles
				newCluster.m_triangles = planeValidSet;
				//bbc.emplace_back(refinedPlane);

				for (auto& triangle : planeValidSet)
				{
					selectedTriangleIndices.emplace_back(triangle.m_index);
				}
				discretization.m_failSafeModeTriggered = false;
			}
			else
			{
				// get the fitted triangles index in the whole triangles
				selectedTriangleIndices = discretization.ComputePlaneValidSetIndex(m_elements, operatingTriangles, newCluster.m_clusterPlane);

				EVOENGINE_ERROR("Fitted_triangle_num: " + std::to_string(selectedTriangleIndices.size()));

				for (int index : selectedTriangleIndices)
				{
					planeValidSet.emplace_back(operatingTriangles[index]);
				}

				// update density by removing the fitted triangles
				discretization.UpdateDensity(m_elements, planeValidSet, false);

				// store bbc and corresponding fitted triangles
				newCluster.m_triangles = planeValidSet;
			}
		}
		else
		{
			m_skippedTriangles = operatingTriangles;
			break;
		}
		retVal.emplace_back(std::move(newCluster));
		//Remove selected triangle from the remaining triangle.
		for (auto it = selectedTriangleIndices.rbegin(); it != selectedTriangleIndices.rend(); ++it)
		{
			operatingTriangles[*it] = operatingTriangles.back();
			operatingTriangles.pop_back();
		}
		epoch++;
		if (settings.m_timeout != 0 && epoch >= settings.m_timeout)
		{
			EVOENGINE_ERROR("Default clustering timeout!")
				break;
		}
	}

	return retVal;
}