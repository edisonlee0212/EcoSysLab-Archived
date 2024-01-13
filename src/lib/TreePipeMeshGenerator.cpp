#include "TreePipeMeshGenerator.hpp"
#include "Curve.hpp"
#include "Octree.hpp"
#include "Jobs.hpp"
#include "opensubdiv/bfr/faceSurface.h"
#include "regression/bfr_evaluate/bfrSurfaceEvaluator.h"
#include <glm/gtx/intersect.hpp>
#include <glm/gtx/io.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>

using namespace EcoSysLab;

typedef boost::adjacency_list<boost::vecS,
	boost::vecS,
	boost::undirectedS,
	glm::vec2
> Graph;
typedef boost::graph_traits<Graph>::adjacency_iterator AdjacencyIterator;

void TreePipeMeshGeneratorSettings::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	if (ImGui::TreeNode("Marching cubes settings"))
	{
		ImGui::Checkbox("Auto set level", &m_autoLevel);
		if (!m_autoLevel) ImGui::DragInt("Voxel subdivision level", &m_voxelSubdivisionLevel, 1, 5, 16);
		else ImGui::DragFloat("Min Cube size", &m_marchingCubeRadius, 0.0001, 0.001f, 1.0f);
		ImGui::DragInt("Smooth iteration", &m_voxelSmoothIteration, 0, 0, 10);
		if (m_voxelSmoothIteration == 0) ImGui::Checkbox("Remove duplicate", &m_removeDuplicate);
		ImGui::DragFloat("Marching Cube Radius", &m_marchingCubeRadius, 0.0001f, 0.0001f, 2.0f);
		ImGui::TreePop();
	}

	ImGui::ColorEdit3("Vertex color", &m_vertexColor.x);
	
}

int roundInDir(float val, int dir)
{
	if (dir > 0)
	{
		return int(std::ceilf(val));
	}
	else
	{
		return int(std::floorf(val));
	}
}

glm::ivec3 roundInDir(glm::vec3 val, glm::ivec3 dir)
{
	return glm::ivec3(roundInDir(val[0], dir[0]), roundInDir(val[1], dir[1]), roundInDir(val[2], dir[2]));
}

std::vector<glm::ivec3> voxelizeLineSeg(glm::vec3 start, glm::vec3 end, float voxelSideLength)
{
	// Based on Amanatides, J., & Woo, A. (1987, August). A fast voxel traversal algorithm for ray tracing. In Eurographics (Vol. 87, No. 3, pp. 3-10).
	std::vector<glm::ivec3> retVal;
	glm::vec3 dir = end - start;
	glm::vec3 dirInVoxels = dir / voxelSideLength;
	glm::ivec3 step = glm::sign(dir);

	// determine voxel of start point
	glm::vec3 posInVoxels = start / voxelSideLength;
	glm::ivec3 startVoxel = glm::floor(posInVoxels);
	retVal.push_back(startVoxel);
	glm::ivec3 endVoxel = glm::floor(end / voxelSideLength);

	// compute t deltas
	glm::vec3 tDelta = glm::vec3(voxelSideLength) / glm::abs(dir);

	// compute max value for t within voxel
	glm::ivec3 nextBorder = roundInDir(posInVoxels, step);
	glm::vec3 distToNextBorder = glm::vec3(nextBorder) - posInVoxels;
	glm::vec3 tMax = glm::abs(distToNextBorder / dirInVoxels);
	if (dirInVoxels.x <= glm::epsilon<float>())tMax.x = 0.0f;
	if (dirInVoxels.y <= glm::epsilon<float>())tMax.y = 0.0f;
	if (dirInVoxels.z <= glm::epsilon<float>())tMax.z = 0.0f;
	// min and max for debug assert
	glm::vec3 minVoxel = glm::min(startVoxel, endVoxel);
	glm::vec3 maxVoxel = glm::max(startVoxel, endVoxel);

	// now traverse voxels until we reach the end point
	glm::ivec3 voxel = startVoxel;
	while (voxel != endVoxel)
	{
		size_t minIndex = 0;

		for (size_t i = 1; i < 3; i++)
		{
			if (tMax[i] < tMax[minIndex])
			{
				minIndex = i;
			}
		}

		// update tMax and determine next voxel;
		voxel[minIndex] += step[minIndex];
		

		// check that we do not run out of range
		// This can happen due to numerical inaccuracies when adding tDelta
		for (size_t dim = 0; dim < 3; dim++)
		{
			if (!(minVoxel[dim] <= voxel[dim] && voxel[dim] <= maxVoxel[dim]))
			{
				retVal.push_back(endVoxel);
				return retVal;
			}
		}

		retVal.push_back(voxel);
		tMax[minIndex] += tDelta[minIndex];

	}

	return retVal;
}

std::vector<PipeSegmentHandle> getNextSegGroup(const PipeModelPipeGroup& pipes, const std::vector<PipeSegmentHandle>& segGroup)
{
	std::vector<PipeSegmentHandle> nextSegGroup;

	for (const PipeSegmentHandle& segHandle : segGroup)
	{
		auto& seg = pipes.PeekPipeSegment(segHandle);
		if (!seg.IsEndPipeSegment())
		{
			nextSegGroup.push_back(seg.GetNextHandle());
		}
	}

	return nextSegGroup;
}

glm::vec3 getSegDir(const PipeModelPipeGroup& pipes, PipeSegmentHandle segHandle)
{
	auto& seg = pipes.PeekPipeSegment(segHandle);
	PipeSegmentHandle prevSegHandle = seg.GetPrevHandle();

	// handle the case that this is the first segment of the pipe
	if (prevSegHandle != -1)
	{
		auto& prevSeg = pipes.PeekPipeSegment(prevSegHandle);
		std::cout << "Segment going from " << prevSeg.m_info.m_globalPosition << " to " << seg.m_info.m_globalPosition << std::endl;
		return seg.m_info.m_globalPosition - prevSeg.m_info.m_globalPosition;
	}
	else
	{
		/*glm::vec3 prevPos = pipes.PeekPipe(seg.GetPipeHandle()).m_info.m_baseInfo.m_globalPosition;
		std::cout << "start of pipe at " << prevPos << " with segment going to " << seg.m_info.m_globalPosition << std::endl;
		return seg.m_info.m_globalPosition - prevPos;*/

		// TODO: This is not accurate, but will do for now
		return getSegDir(pipes, seg.GetNextHandle());
	}
}

class Grid2D
{
private:
	float m_rasterSize = 0.0f;
	glm::ivec2 m_minIndex;
	std::vector<std::vector<std::vector<size_t> > > m_gridCells;

	void handleCells(Graph& g, float maxDistanceSqr, const std::vector<size_t>& cell0, const std::vector<size_t>& cell1) const
	{
		for (size_t u : cell0)
		{
			for (size_t v : cell1)
			{
				//std::cout << "comparing vertex " << u << " to " << v << std::endl;
				if (glm::distance2(g[u], g[v]) < maxDistanceSqr)
				{
					std::cout << "adding edge " << u << " to " << v << std::endl;
					boost::add_edge(u, v, g);
				}
			}
		}
	}

public:
	Grid2D(float rasterSize, glm::vec2 min, glm::vec2 max) : m_rasterSize(rasterSize)
	{
		m_minIndex = glm::floor(min / rasterSize);
		glm::ivec2 maxIndex = glm::floor(max / rasterSize);

		// TODO: this can probably be done more efficiently
		m_gridCells = std::vector<std::vector<std::vector<size_t> > >(maxIndex[0] - m_minIndex[0] + 1);
		for (auto& vec : m_gridCells)
		{
			vec = std::vector<std::vector<size_t> >(maxIndex[1] - m_minIndex[1] + 1);
		}
	}
	~Grid2D() {}

	void insert(const Graph& g, size_t vertexIndex)
	{
		const glm::vec2& pos = g[vertexIndex];
		glm::ivec2 index = glm::ivec2(glm::floor(pos / m_rasterSize)) - m_minIndex;

		std::cout << "inserting vertex " << vertexIndex << " into grid cell " << index << std::endl;

		m_gridCells[index[0]][index[1]].push_back(vertexIndex);
	}

	void connectNeighbors(Graph& g, float maxDistance) const
	{
		float maxDistanceSqr = maxDistance * maxDistance;

		// simply check all points against each other in neighboring cells (including diagonal)
		for (size_t i = 0; i < m_gridCells.size() - 1; i++)
		{
			for (size_t j = 0; j < m_gridCells[i].size() - 1; j++)
			{
				handleCells(g, maxDistanceSqr, m_gridCells[i][j], m_gridCells[i][j]);
				handleCells(g, maxDistanceSqr, m_gridCells[i][j], m_gridCells[i][j + 1]);
				handleCells(g, maxDistanceSqr, m_gridCells[i][j], m_gridCells[i + 1][j]);
				handleCells(g, maxDistanceSqr, m_gridCells[i][j], m_gridCells[i + 1][j + 1]);
				handleCells(g, maxDistanceSqr, m_gridCells[i + 1][j], m_gridCells[i][j + 1]);
			}
		}

		// handle last row and column
		for (size_t i = 0; i < m_gridCells.size() - 1; i++)
		{
			handleCells(g, maxDistanceSqr, m_gridCells[i].back(), m_gridCells[i].back());
			handleCells(g, maxDistanceSqr, m_gridCells[i].back(), m_gridCells[i + 1].back());
		}

		for (size_t j = 0; j < m_gridCells.back().size() - 1; j++)
		{
			handleCells(g, maxDistanceSqr, m_gridCells.back()[j], m_gridCells.back()[j]);
			handleCells(g, maxDistanceSqr, m_gridCells.back()[j], m_gridCells.back()[j + 1]);
		}

		handleCells(g, maxDistanceSqr, m_gridCells.back().back(), m_gridCells.back().back());
	}
};

void dfs(const Graph& g, size_t v, std::vector<size_t>& componentMembers, std::vector<bool>& visited)
{
	visited[v] = true;
	componentMembers.push_back(v);

	AdjacencyIterator ai, a_end;
	boost::tie(ai, a_end) = boost::adjacent_vertices(v, g);

	for (; ai != a_end; ai++)
	{
		if (!visited[*ai])
		{
			dfs(g, *ai, componentMembers, visited);
		}
	}
}

std::vector<size_t> collectComponent(const Graph& g, size_t startIndex)
{
	std::vector<size_t> componentMembers;

	// just do a dfs to collect everything
	std::vector<bool> visited(g.m_vertices.size(), false);

	dfs(g, startIndex, componentMembers, visited);
	return componentMembers;
}

std::vector<PipeSegmentHandle> getSegGroup(const PipeModelPipeGroup& pipes, size_t index)
{
	std::vector<PipeSegmentHandle> segGroup;

	for (auto& pipe : pipes.PeekPipes())
	{
		if (pipe.PeekPipeSegmentHandles().size() > index)
		{
			segGroup.push_back(pipe.PeekPipeSegmentHandles()[index]);
		}
		else
		{
			segGroup.push_back(-1);
		}
	}

	return segGroup;
}


void obtainProfiles(const PipeModelPipeGroup& pipes, std::vector<PipeSegmentHandle> segGroup, float maxDist)
{
	std::vector<bool> visited(segGroup.size(), false);

	std::vector<std::vector<PipeSegmentHandle> > profiles;

	for (auto& segHandle : segGroup)
	{
		if (segHandle == -1)
		{
			continue;
		}

		auto& seg = pipes.PeekPipeSegment(segHandle);
		PipeHandle pipeHandle = seg.GetPipeHandle();

		if (seg.m_info.m_isBoundary  && !visited[pipeHandle])
		{
			// traverse boundary
			std::vector<PipeSegmentHandle> profile;
			auto handle = segHandle;
			do
			{
				profile.push_back(handle);
				auto& seg = pipes.PeekPipeSegment(handle);
				PipeHandle pipeHandle = seg.GetPipeHandle();
				visited[pipeHandle] = true;

				// get next
				//seg.m_info.


			} while (handle != segHandle);

			profiles.push_back(profile);
		}
	}
}

size_t getNextOnBoundary(Graph& g, size_t cur, size_t prev, float& prevAngle)
{
	// first rotate prevAngel by 180 degrees because we are looking from the other side of the edge now

	if (prevAngle < 0) // TODO: should this be <=
	{
		prevAngle += glm::pi<float>();
	}
	else
	{
		prevAngle -= glm::pi<float>();
	}

	AdjacencyIterator ai, a_end;
	boost::tie(ai, a_end) = boost::adjacent_vertices(cur, g);
	size_t next = -1;
	float nextAngle = std::numeric_limits<float>::infinity();

	// The next angle must either be the smallest that is larger than the current one or if such an angle does not exist the smallest overall
	for (; ai != a_end; ai++)
	{
		if (*ai == prev)
		{
			continue;
		}

		glm::vec2 dir = g[*ai] - g[cur];
		float angle = atan2f(dir.y, dir.x);
		
		// remap such that all angles are larger than the previous
		float testAngle = angle;
		if (testAngle < prevAngle)
		{
			testAngle += 2 * glm::pi<float>();
		}

		if (testAngle < nextAngle)
		{
			nextAngle = angle;
			next = *ai;
		}
	}

	// TODO: could we have a situation where we reach a leave?
	prevAngle = nextAngle;
	return next;
}

std::vector<size_t> obtainProfile(const PipeModelPipeGroup& pipes, std::vector<PipeSegmentHandle>& segGroup, float maxDist)
{
	// sweep over tree from root to leaves to reconstruct a skeleton with bark outlines
	std::cout << "obtaining profile of segGroup with " << segGroup.size() << " segments" << std::endl;
	auto& seg = pipes.PeekPipeSegment(segGroup[0]);
	glm::vec3 planePos = seg.m_info.m_globalPosition;
	glm::vec3 planeNorm = glm::normalize(getSegDir(pipes, segGroup[0]));

	// first we need to transform into a basis in the cross section plane. This will reduce the problem to 2D
	// To do so, first find suitable orthogonal vectors to the plane's normal vector
	size_t minDim = 0;

	for (size_t i = 1; i < 3; i++)
	{
		if (planeNorm[i] < planeNorm[minDim])
		{
			minDim = i;
		}
	}

	glm::vec3 e(0, 0, 0);
	e[minDim] = 1.0f;

	glm::vec3 basis[3];

	basis[2] = planeNorm;
	basis[0] = glm::normalize(glm::cross(planeNorm, e));
	basis[1] = glm::cross(basis[0], basis[2]);

	//std::cout << "new basis vectors: " << std::endl;

	for (size_t i = 0; i < 3; i++)
	{
		//std::cout << "basis vector " << i << ": " << basis[i] << std::endl;
	}

	glm::mat3x3 basisTrans =
	{
		{basis[0][0], basis[0][1], basis[0][2]},
		{basis[1][0], basis[1][1], basis[1][2]},
		{basis[2][0], basis[2][1], basis[2][2]}
	};

	// TODO: could we do this more efficiently? E.g. the Intel Embree library provides efficent ray casts and allows for ray bundling
	// now project all of them onto plane
	std::cout << "building strand graph" << std::endl;
	Graph strandGraph;
	glm::vec3 min(std::numeric_limits<float>::infinity());
	glm::vec3 max(-std::numeric_limits<float>::infinity());

	for (auto& segHandle : segGroup)
	{
		std::cout << "processing seg with handle no. " << segHandle << std::endl;
		auto& seg = pipes.PeekPipeSegment(segHandle);
		glm::vec3 segPos = seg.m_info.m_globalPosition;
		glm::vec3 segDir = glm::normalize(getSegDir(pipes, segGroup[0]));

		float param;

		glm::intersectRayPlane(segPos, segDir, planePos, planeNorm, param);
		//std::cout << "line: " << segPos << " + t * " << segDir << " intersects plane " << planeNorm << " * (p - " << planePos << ") = 0 at t = " << param << std::endl;
		// store the intersection point in a graph
		size_t vIndex = boost::add_vertex(strandGraph);
		glm::vec3 pos = basisTrans * (segPos + segDir * param);
		//std::cout << "mapped point " << (segPos + segDir * param) << " to " << pos << std::endl;
		strandGraph[vIndex] = pos;

		min = glm::min(min, pos);
		max = glm::max(max, pos);
	}
	std::cout << "built graph of size " << strandGraph.m_vertices.size() << std::endl;

	// TODO: we should check that maxDist is at least as large as the thickest strand
	// now cluster anything below maxDist
	std::cout << "inserting points into grid" << std::endl;
	Grid2D grid(maxDist, min, max);
	for (size_t i = 0; i < strandGraph.m_vertices.size(); i++)
	{
		grid.insert(strandGraph, i);
	}

	std::cout << "connecting neighbors" << std::endl;
	grid.connectNeighbors(strandGraph, maxDist);

	std::cout << "collecting component" << std::endl;
	std::vector<size_t> cluster = collectComponent(strandGraph, 0);
	std::cout << "collected component of size " << cluster.size() << std::endl;

	// Now find an extreme point in the cluster. It must lie on the boundary. From here we can start traversing the boundary
	size_t leftmostIndex = 0;
	float leftmostCoord = std::numeric_limits<float>::infinity();

	for (size_t index : cluster)
	{
		if (strandGraph[index].x < leftmostCoord)
		{
			leftmostCoord = strandGraph[index].x;
			leftmostIndex = index;
		}
	}

	// traverse the boundary using the angles to its neighbors
	// Note: because the strands cannot overlap and there is a maximum distance between graph vertices, the graph's maximum degree is of constant size.
	std::cout << "traversing boundary" << std::endl;
	std::cout << "starting at index: " << leftmostIndex << std::endl;
	std::vector<size_t> profile{ leftmostIndex };

	// going counterclockwise from here, the next point must be the one with the smallest angle
	AdjacencyIterator ai, a_end;
	boost::tie(ai, a_end) = boost::adjacent_vertices(leftmostIndex, strandGraph);
	size_t next = -1;
	float minAngle = glm::pi<float>();

	for (; ai != a_end; ai++)
	{
		std::cout << "checking neighbor " << *ai << std::endl;
		glm::vec2 dir = strandGraph[*ai] - strandGraph[leftmostIndex];
		float angle = atan2f(dir.y, dir.x);
		if (angle < minAngle)
		{
			minAngle = angle;
			next = *ai;
		}
	}

	// from now on we will have to find the closest neighboring edge in counter-clockwise order for each step until we reach the first index again
	size_t prev = leftmostIndex;
	size_t cur = next;
	float prevAngle = minAngle;

	while (next != leftmostIndex)
	{
		if (cur == -1)
		{
			std::cerr << "Error: cur is -1" << std::endl;
			break;
		}

		std::cout << "cur: " << cur << std::endl;
		profile.push_back(cur);

		next = getNextOnBoundary(strandGraph, cur, prev, prevAngle);
		prev = cur;
		cur = next;
	}

	return profile;
}

void forEachSegment(const PipeModelPipeGroup& pipes, std::vector<PipeSegmentHandle>& segGroup, std::function<void(const PipeSegment<PipeModelPipeSegmentData>&)> func)
{
	for (auto& segHandle : segGroup)
	{
		auto& seg = pipes.PeekPipeSegment(segHandle);
		func(seg);
	}
}

void TreePipeMeshGenerator::Generate(
	const TreeModel& treeModel, std::vector<Vertex>& vertices,
                                     std::vector<unsigned>& indices, const TreePipeMeshGeneratorSettings& settings)
{
	const auto& skeleton = treeModel.PeekShootSkeleton();
	const auto& pipeGroup = skeleton.m_data.m_pipeGroup;

	std::cout << "getting first seg group" << std::endl;
	std::vector<PipeSegmentHandle> firstSegGroup = getSegGroup(pipeGroup, 0);

	std::cout << "determining max thickness" << std::endl;
	float maxThickness = 0.0f;
	forEachSegment(pipeGroup, firstSegGroup,
		[&](const PipeSegment<PipeModelPipeSegmentData>& seg)
		{
			if (seg.m_info.m_thickness < maxThickness)
			{
				maxThickness = seg.m_info.m_thickness;
			}
		}
	);

	std::cout << "obtaining profile" << std::endl;
	std::vector<size_t> profile = obtainProfile(pipeGroup, firstSegGroup, maxThickness * 1.5f);

	// let's just create a test mesh to see if this is reasonable
	std::cout << "creating test mesh" << std::endl;
	for (size_t i = 0; i < profile.size(); i++)
	{
		auto& pipe = pipeGroup.PeekPipes()[i];
		auto& segHandle = pipe.PeekPipeSegmentHandles()[0];
		auto& seg = pipeGroup.PeekPipeSegment(segHandle);

		Vertex v0;
		v0.m_position = seg.m_info.m_globalPosition;
		vertices.push_back(v0);

		// just offset by 1
		Vertex v1;
		v1.m_position = seg.m_info.m_globalPosition + glm::vec3(0.0, 1.0, 0.0);
		vertices.push_back(v1);
	}

	// create faces
	for (size_t i = 0; i < profile.size(); i++)
	{
		indices.push_back(2 * i);
		indices.push_back(2 * i + 1);
		indices.push_back((2 * i + 2) % (2 * profile.size()));

		indices.push_back(2 * i + 1);
		indices.push_back((2 * i + 2) % (2 * profile.size()));
		indices.push_back((2 * i + 3) % (2 * profile.size()));
	}

	/*
	

	for(const auto& pipe : pipeGroup.PeekPipes())
	{
		for(const auto& pipeSegmentHandle : pipe.PeekPipeSegmentHandles())
		{
			const auto& pipeSegment = pipeGroup.PeekPipeSegment(pipeSegmentHandle);
			//Get interpolated position on pipe segment. Example to get middle point here:
			const auto middlePoint = treeModel.InterpolatePipeSegmentPosition(pipeSegmentHandle, 0.5f);
			//Get interpolated direction on pipe segment. The axis is a normalized vector that points from the point close to root to end (Bottom up)
			const auto middleAxis = treeModel.InterpolatePipeSegmentAxis(pipeSegmentHandle, 0.5f);


			const auto& node = skeleton.PeekNode(pipeSegment.m_data.m_nodeHandle);
			//To access the user's defined constraints (attractors, etc.)
			const auto& profileConstraints = node.m_data.m_profileConstraints;

			//To access the position of the start of the pipe segment within a profile:
			const auto parentHandle = node.GetParentHandle();
			if(parentHandle == -1)
			{
				//If the node is the first node, the corresponding pipe segment will also be the first one in the pipe.
				assert(pipeSegmentHandle == pipe.PeekPipeSegmentHandles().front());
				//If current node is the first node of the tree, it's front profile will be used to calculate the start position of the first segment of the pipe.
				const auto& startProfile = node.m_data.m_frontProfile;

				//To get the particle that belongs to this pipe segment:
				const auto& startParticle = startProfile.PeekParticle(pipeSegment.m_data.m_frontProfileParticleHandle);

				//To access the boundary of the profile:
				const auto& startProfileBoundaryEdges = startProfile.PeekBoundaryEdges();
				for (const auto& startProfileBoundaryEdgeParticleHandles : startProfileBoundaryEdges)
				{
					const auto& particle1 = startProfile.PeekParticle(startProfileBoundaryEdgeParticleHandles.first);
					const auto& particle2 = startProfile.PeekParticle(startProfileBoundaryEdgeParticleHandles.second);
					const auto particle1Position = particle1.GetPosition();
					const auto particle2Position = particle2.GetPosition();
				}
			}else
			{
				//For other nodes, it's parent's back profile will be used to calculate the end position of the segment.
				const auto& startProfile = skeleton.PeekNode(parentHandle).m_data.m_backProfile;

				//You will also need to access the prev segment in order to retrieve the particle:
				const auto& prevPipeSegment = pipeGroup.PeekPipeSegment(pipeSegment.GetPrevHandle());
				const auto& startParticle = startProfile.PeekParticle(prevPipeSegment.m_data.m_backProfileParticleHandle);

				//To access the boundary of the profile:
				const auto& startProfileBoundaryEdges = startProfile.PeekBoundaryEdges();
				for (const auto& startProfileBoundaryEdgeParticleHandles : startProfileBoundaryEdges)
				{
					const auto& particle1 = startProfile.PeekParticle(startProfileBoundaryEdgeParticleHandles.first);
					const auto& particle2 = startProfile.PeekParticle(startProfileBoundaryEdgeParticleHandles.second);
					const auto particle1Position = particle1.GetPosition();
					const auto particle2Position = particle2.GetPosition();
				}
			}

			const auto& endProfile = node.m_data.m_backProfile;
			const auto& endProfileBoundaryEdges = endProfile.PeekBoundaryEdges();

			//The position of the back particle corresponds to the end of the pipe segment.
			const auto& endParticle = endProfile.PeekParticle(pipeSegment.m_data.m_backProfileParticleHandle);

			//To iterate through all boundary edges of the back profile:
			for (const auto& endProfileBoundaryEdgeParticleHandles : endProfileBoundaryEdges)
			{
				const auto& particle1 = endProfile.PeekParticle(endProfileBoundaryEdgeParticleHandles.first);
				const auto& particle2 = endProfile.PeekParticle(endProfileBoundaryEdgeParticleHandles.second);
				const auto particle1Position = particle1.GetPosition();
				const auto particle2Position = particle2.GetPosition();
			}
		}
	}

	//OpenSubdiv
	Bfr::Surface<float> surface{};
	*/
}
