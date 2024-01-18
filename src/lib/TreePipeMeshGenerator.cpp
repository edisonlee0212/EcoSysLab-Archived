#include "TreePipeMeshGenerator.hpp"
#include "Curve.hpp"
#include "Octree.hpp"
#include "Jobs.hpp"
#include "opensubdiv/bfr/faceSurface.h"
#include "regression/bfr_evaluate/bfrSurfaceEvaluator.h"
#include <glm/gtx/intersect.hpp>
#include <glm/gtx/io.hpp>
#include "MeshGenUtils.hpp"

using namespace EcoSysLab;

typedef std::vector<std::pair<PipeHandle, glm::vec3> > Slice;
typedef std::vector<PipeHandle> PipeCluster;

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

	ImGui::Checkbox("[DEBUG] Limit Profile Iterations", &m_limitProfileIterations);
	ImGui::DragInt("[DEBUG] Limit", &m_maxProfileIterations);

	ImGui::DragFloat("[DEBUG] MaxParam", &m_maxParam);
	
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

glm::vec3 getSegDir(const TreeModel& treeModel, const PipeSegmentHandle segHandle, float t = 1.0f)
{
	return treeModel.InterpolatePipeSegmentAxis(segHandle, t);
}

glm::vec3 getSegPos(const TreeModel& treeModel, const PipeSegmentHandle segHandle, float t = 1.0f)
{
	return treeModel.InterpolatePipeSegmentPosition(segHandle, t);
}

bool isValidPipeParam(const TreeModel& treeModel, const PipeHandle& pipeHandle, float t)
{
	const auto& pipe = treeModel.PeekShootSkeleton().m_data.m_pipeGroup.PeekPipe(pipeHandle);
	return pipe.PeekPipeSegmentHandles().size() > glm::floor(t);
}

glm::vec3 getPipeDir(const TreeModel& treeModel, const PipeHandle& pipeHandle, float t)
{
	const auto& pipe = treeModel.PeekShootSkeleton().m_data.m_pipeGroup.PeekPipe(pipeHandle);
	auto segHandle = pipe.PeekPipeSegmentHandles()[t];
	return getSegDir(treeModel, segHandle, fmod(t, 1.0));
}

glm::vec3 getPipePos(const TreeModel& treeModel, const PipeHandle& pipeHandle, float t)
{
	const auto& pipe = treeModel.PeekShootSkeleton().m_data.m_pipeGroup.PeekPipe(pipeHandle);
	auto segHandle = pipe.PeekPipeSegmentHandles()[t];
	return getSegPos(treeModel, segHandle, fmod(t, 1.0));
}

void dfs(const Graph& g, size_t v, std::vector<size_t>& componentMembers, std::vector<bool>& visited)
{
	visited[v] = true;
	componentMembers.push_back(v);

	for (size_t av : g.adjacentVertices(v))
	{
		if (!visited[av])
		{
			dfs(g, av, componentMembers, visited);
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
	//std::cout << "Flipped angle to " << prevAngle << std::endl;

	size_t next = -1;
	float nextAngle = std::numeric_limits<float>::infinity();

	// The next angle must either be the smallest that is larger than the current one or if such an angle does not exist the smallest overall
	for (size_t av : g.adjacentVertices(cur))
	{
		if (av == prev)
		{
			continue;
		}

		glm::vec2 dir = g[av] - g[cur];
		float angle = atan2f(dir.y, dir.x);
		//std::cout << " checking neighbor " << av << " with angle " << angle << std::endl;
		
		// remap such that all angles are larger than the previous
		float testAngle = angle;
		if (testAngle < prevAngle)
		{
			testAngle += 2 * glm::pi<float>();
		}

		if (testAngle < nextAngle)
		{
			nextAngle = testAngle;
			next = av;
		}
	}

	// confine to range
	if (nextAngle > glm::pi<float>())
	{
		nextAngle -= 2 * glm::pi<float>();
	}

	//std::cout << "Selected neighbor " << next << " with angle " << nextAngle << std::endl;

	// TODO: could we have a situation where we reach a leaf?
	prevAngle = nextAngle;
	return next;
}

Slice profileToSlice(const TreeModel& treeModel, std::vector<size_t>& profile, const PipeCluster& pipeCluster, float t)
{
	Slice slice;

	for (size_t i = 0; i < profile.size(); i++)
	{
		PipeHandle pipeHandle = pipeCluster[profile[i]];
		slice.push_back(std::pair<PipeHandle, glm::vec3>(pipeHandle, getPipePos(treeModel, pipeHandle, t)));
	}

	return slice;
}

std::pair<Slice, PipeCluster> computeSlice(const TreeModel& treeModel, const PipeCluster& pipesInPrevious, size_t index, std::vector<bool>& visited, float t, float maxDist)
{
	const PipeModelPipeGroup& pipes = treeModel.PeekShootSkeleton().m_data.m_pipeGroup;

	if (!isValidPipeParam(treeModel, pipesInPrevious[index], t))
	{
		return std::make_pair<Slice, PipeCluster>(Slice(), PipeCluster());
	}
	// sweep over tree from root to leaves to reconstruct a skeleton with bark outlines
	std::cout << "obtaining profile of segGroup with " << pipesInPrevious.size() << " segments" << std::endl;
	glm::vec3 planePos = getPipePos(treeModel, pipesInPrevious[index], t);
	glm::vec3 planeNorm = glm::normalize(getPipeDir(treeModel, pipesInPrevious[index], t));

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

	std::cout << "new basis vectors: " << std::endl;

	for (size_t i = 0; i < 3; i++)
	{
		std::cout << "basis vector " << i << ": " << basis[i] << std::endl;
	}

	glm::mat3x3 basisTrans =
	{
		{basis[0][0], basis[1][0], basis[2][0]},
		{basis[0][1], basis[1][1], basis[2][1]},
		{basis[0][2], basis[1][2], basis[2][2]}
	};

	// TODO: could we do this more efficiently? E.g. the Intel Embree library provides efficent ray casts and allows for ray bundling
	// now project all of them onto plane
	std::cout << "building strand graph" << std::endl;
	Graph strandGraph;
	glm::vec3 min(std::numeric_limits<float>::infinity());
	glm::vec3 max(-std::numeric_limits<float>::infinity());

	for (auto& pipeAndPos : pipesInPrevious)
	{
		//std::cout << "processing seg with handle no. " << segHandle << std::endl;
		glm::vec3 segPos = getPipePos(treeModel, pipeAndPos, t);
		glm::vec3 segDir = glm::normalize(getPipeDir(treeModel, pipeAndPos, t));

		float param;

		glm::intersectRayPlane(segPos, segDir, planePos, planeNorm, param);
		//std::cout << "line: " << segPos << " + t * " << segDir << " intersects plane " << planeNorm << " * (p - " << planePos << ") = 0 at t = " << param << std::endl;
		// store the intersection point in a graph
		size_t vIndex = strandGraph.addVertex();
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

	std::cout << "outputting graph" << std::endl;
	outputGraph(strandGraph, "strandGraph_" + std::to_string(pipesInPrevious[index]) + "_" + std::to_string(t), pipesInPrevious);

	// TODO:: maybe also use visited here
	std::cout << "collecting component" << std::endl;
	std::vector<size_t> cluster = collectComponent(strandGraph, index);
	std::cout << "collected component of size " << cluster.size() << std::endl;

	// write cluster to previous
	PipeCluster pipesInComponent;

	for (size_t indexInComponent : cluster)
	{
		visited[indexInComponent] = true;
		std::cout << "Marking strand no. " << indexInComponent << " with handle " << pipesInPrevious[indexInComponent] << " as visited." << std::endl;
		pipesInComponent.push_back(pipesInPrevious[indexInComponent]);
	}

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
	size_t next = -1;
	float minAngle = glm::pi<float>();

	for (size_t av : strandGraph.adjacentVertices(leftmostIndex))
	{
		glm::vec2 dir = strandGraph[av] - strandGraph[leftmostIndex];
		float angle = atan2f(dir.y, dir.x);
		//std::cout << "checking neighbor " << *ai << " with angle " << angle << std::endl;
		if (angle < minAngle)
		{
			minAngle = angle;
			next = av;
		}
	}

	// from now on we will have to find the closest neighboring edge in counter-clockwise order for each step until we reach the first index again
	size_t prev = leftmostIndex;
	size_t cur = next;
	float prevAngle = minAngle;

	size_t debugCounter = 0;
	while (next != leftmostIndex && debugCounter < 100)
	{
		//std::cout << " Iteration no. " << debugCounter << std::endl;
		if (cur == -1)
		{
			std::cerr << "Error: cur is -1" << std::endl;
			break;
		}

		//std::cout << "cur: " << cur << std::endl;
		profile.push_back(cur);

		next = getNextOnBoundary(strandGraph, cur, prev, prevAngle);
		prev = cur;
		cur = next;

		debugCounter++;
	}

	std::cout << "Extracted profile: ";
	for (auto& e : profile)
	{
		std::cout << e << ", ";
	}
	std::cout << std::endl;

	Slice slice = profileToSlice(treeModel, profile, pipesInPrevious, t);

	std::cout << "Extracted slice: ";
	for (auto& e : slice)
	{
		std::cout << e.first << ", ";
	}
	std::cout << std::endl;

	return std::make_pair<>(slice, pipesInComponent);
}

std::vector<std::pair<Slice, PipeCluster> > computeSlices(const TreeModel& treeModel, const PipeCluster& pipesInPrevious, float t, float maxDist)
{
	std::vector<std::pair<Slice, PipeCluster> > slices;

	std::vector<bool> visited(pipesInPrevious.size(), false);

	for (std::size_t i = 0; i < pipesInPrevious.size(); i++)
	{
		if (visited[i])
		{
			std::cout << "skipping already visited pipe no. " << i << " with handle " << pipesInPrevious[i];
			continue;
		}

		// if not visited, determine connected component around this
		std::cout << "computing slice containing pipe no. " << i << " with handle " << pipesInPrevious[i];
		auto slice = computeSlice(treeModel, pipesInPrevious, i, visited, t, maxDist);
		slices.push_back(slice);
	}

	return slices;
}

/*
std::vector<size_t> obtainProfile(const TreeModel& treeModel, const std::vector<PipeSegmentHandle>& segGroup, float maxDist, size_t maxIterations = std::numeric_limits<size_t>::max())
{
	const PipeModelPipeGroup& pipes = treeModel.PeekShootSkeleton().m_data.m_pipeGroup;
	// sweep over tree from root to leaves to reconstruct a skeleton with bark outlines
	std::cout << "obtaining profile of segGroup with " << segGroup.size() << " segments" << std::endl;
	auto& seg = pipes.PeekPipeSegment(segGroup[0]);
	glm::vec3 planePos = getSegPos(treeModel, segGroup[0], 0.0);
	glm::vec3 planeNorm = glm::normalize(getSegDir(treeModel, segGroup[0], 0.0));

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
		//std::cout << "processing seg with handle no. " << segHandle << std::endl;
		auto& seg = pipes.PeekPipeSegment(segHandle);
		glm::vec3 segPos = getSegPos(treeModel, segHandle, 0.0);
		glm::vec3 segDir = glm::normalize(getSegDir(treeModel, segHandle, 0.0));

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

	outputGraph(strandGraph, "strandGraph" + std::to_string(segGroup[0]));

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
		glm::vec2 dir = strandGraph[*ai] - strandGraph[leftmostIndex];
		float angle = atan2f(dir.y, dir.x);
		std::cout << "checking neighbor " << *ai << " with angle " << angle << std::endl;
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

	size_t debugCounter = 0;
	while (next != leftmostIndex && debugCounter < maxIterations)
	{
		std::cout << " Iteration no. " << debugCounter << std::endl;
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

		debugCounter++;
	}

	return profile;
}
*/

void forEachSegment(const PipeModelPipeGroup& pipes, std::vector<PipeSegmentHandle>& segGroup, std::function<void(const PipeSegment<PipeModelPipeSegmentData>&)> func)
{
	for (auto& segHandle : segGroup)
	{
		auto& seg = pipes.PeekPipeSegment(segHandle);
		func(seg);
	}
}

bool needToReorder(std::vector<size_t>& permutation)
{
	size_t firstIndex;
	for (size_t index : permutation)
	{
		if (index != -1)
		{
			firstIndex = index;
			break;
		}
	}

	size_t maxIndex = firstIndex;

	for (size_t index : permutation)
	{
		if (index == -1)
		{
			continue;
		}

		if (index < maxIndex && index > firstIndex)
		{
			return true;
		}
		// TODO: wrap-around
	}
}

void connect(std::vector<std::pair<PipeHandle, glm::vec3> >& slice0, size_t i0, size_t j0,
	std::vector<std::pair<PipeHandle, glm::vec3> >& slice1, size_t i1, size_t j1,
	std::vector<Vertex>& vertices, std::vector<unsigned>& indices)
{
	std::cout << "connecting " << i0 << ", " << j0 << " to " << i1 << ", " << j1 << std::endl;
	size_t vertBetween0 = (j0 + slice0.size() - i0) % slice0.size();
	size_t vertBetween1 = (j1 + slice1.size() - i1) % slice1.size();
	std::cout << vertBetween0 << " and " << vertBetween1 << " steps, respectively " << std::endl;
	size_t offset = vertices.size();

	// create vertices
	std::cout << "creating vertices" << std::endl;
	for (size_t k = 0; k <= vertBetween0; k++)
	{
		Vertex v;
		v.m_position = slice0[(i0 + k) % slice0.size()].second;
		vertices.push_back(v);
	}
	for (size_t k = 0; k <= vertBetween1; k++)
	{
		Vertex v;
		v.m_position = slice1[(i1 + k) % slice1.size()].second;
		vertices.push_back(v);
	}

	// merge the two
	std::cout << "connecting slices with triangles" << std::endl;
	size_t k0 = 0;
	size_t k1 = 0;

	while (k0 < vertBetween0 || k1 < vertBetween1)
	{
		if (k0 / double(vertBetween0) < k1 / double(vertBetween1))
		{
			// make triangle consisting of k0, k0 + 1 and k1
			indices.push_back(k0 + offset);
			indices.push_back(k0 + 1 + offset);
			indices.push_back(vertBetween0 + 1 + k1 + offset);
			k0++;
			std::cout << "increased k0 to " << k0 << std::endl;
		}
		else
		{
			// make triangle consisting of k1, k1 + 1 and k0
			indices.push_back(vertBetween0 + 1 + k1 + 1 + offset);
			indices.push_back(vertBetween0 + 1 + k1 + offset);
			indices.push_back(k0 + offset);
			k1++;
			std::cout << "increased k1 to " << k1 << std::endl;
		}
	}
}

void connectSlices(const PipeModelPipeGroup& pipes, Slice& bottomSlice, std::vector<Slice>& topSlices,
	std::vector<Vertex>& vertices, std::vector<unsigned>& indices)
{
	// TODO: support branches

	std::cout << "slice0: ";
	for (auto& e : bottomSlice)
	{
		std::cout << e.first << ", ";
	}
	std::cout << std::endl;

	// compute (incomplete) permutation that turns 0 into 1
	std::cout << "computing permutations for " << pipes.PeekPipes().size() << "pipes..." << std::endl;
	std::vector<std::pair<size_t, size_t> > pipeHandleIndexMap(pipes.PeekPipes().size(), std::make_pair<>(-1,-1));

	//std::cout << "size of pipeHandleIndexMap: " << pipeHandleIndexMap.size() << std::endl;

	for (size_t s = 0; s < topSlices.size(); s++)
	{
		for (size_t i = 0; i < topSlices[s].size(); i++)
		{
			//std::cout << "inserting pipe handle no. " << topSlices[s][i].first << std::endl;
			pipeHandleIndexMap[topSlices[s][i].first] = std::make_pair<>(s, i);
		}
	}

	std::vector<std::pair<size_t, size_t> > permutation(bottomSlice.size(), std::make_pair<>(-1, -1));

	std::cout << "mapping back to permutation vector..." << std::endl;
	for (size_t i = 0; i < permutation.size(); i++)
	{
		permutation[i] = pipeHandleIndexMap[bottomSlice[i].first];
	}

	std::cout << "permutation: ";
	for (auto& e : permutation)
	{
		std::cout << "(" << e.first << ", " << e.second << "), ";
	}
	std::cout << std::endl;

	// now we can more or less just connect them, except that we need to figure out the correct alignment and also get rid of inversions

	// TODO: for now just assume that this does not happen
	size_t prevI = -1;
	std::cout << "Finding first index..." << std::endl;
	for (size_t i = permutation.size(); i > 0; i--)
	{
		if (permutation[i - 1].second != -1)
		{
			prevI = i - 1;
			break;
		}
	}

	std::cout << "Found first index " << prevI << std::endl;
	for (size_t i = 0; i < permutation.size(); i++)
	{
		if (permutation[i].second == -1)
		{
			std::cout << "No correspondence at index " << i << std::endl;
			continue;
		}

		if (permutation[prevI].first == permutation[i].first)
		{
			std::cout << "Connecting at index " << i << std::endl;
			connect(bottomSlice, prevI, i, topSlices[permutation[i].first], permutation[prevI].second, permutation[i].second, vertices, indices);
		}
		else
		{
			std::cout << "Multiple branches not implemented yet!" << std::endl;
			// TODO
		}

		prevI = i;
	}
}

void sliceRecursively(const TreeModel& treeModel, std::pair < Slice, PipeCluster>& prevSlice, float t, float stepSize, float maxParam, float maxDist,
	std::vector<Vertex>& vertices, std::vector<unsigned>& indices)
{
	if (t > maxParam)
	{
		return;
	}

	const auto& skeleton = treeModel.PeekShootSkeleton();
	const auto& pipeGroup = skeleton.m_data.m_pipeGroup;

	auto slicesAndClusters = computeSlices(treeModel, prevSlice.second, t, maxDist);
	std::vector<Slice> topSlices;
	for (auto& s : slicesAndClusters)
	{
		topSlices.push_back(s.first);
	}
	connectSlices(pipeGroup, prevSlice.first, topSlices, vertices, indices);

	std::cout << "--- Done with slice at t = " << t << " ---" << std::endl;
	// recursive call
	t += stepSize;
	for (auto& s : slicesAndClusters)
	{
		if (s.first.size() != 0)
		{
			std::cout << "___ Slice at t = " << t << " ___" << std::endl;
			sliceRecursively(treeModel, s, t, stepSize, maxParam, maxDist, vertices, indices);
		}
	}
}

void TreePipeMeshGenerator::Generate(
	const TreeModel& treeModel, std::vector<Vertex>& vertices,
                                     std::vector<unsigned>& indices, const TreePipeMeshGeneratorSettings& settings)
{
	const auto& skeleton = treeModel.PeekShootSkeleton();
	const auto& pipeGroup = skeleton.m_data.m_pipeGroup;

	if (pipeGroup.PeekPipes().size() == 0)
	{
		return;
	}

	std::cout << "getting first seg group" << std::endl;
	std::vector<PipeSegmentHandle> segGroup0 = getSegGroup(pipeGroup, 0);

	std::cout << "determining max thickness" << std::endl;
	float maxThickness = 0.0f;
	forEachSegment(pipeGroup, segGroup0,
		[&](const PipeSegment<PipeModelPipeSegmentData>& seg)
		{
			if (seg.m_info.m_thickness > maxThickness)
			{
				maxThickness = seg.m_info.m_thickness;
			}
		}
	);

	size_t maxIterations = settings.m_limitProfileIterations ? settings.m_maxProfileIterations : std::numeric_limits<size_t>::max();
	float maxDist = 2 * maxThickness * sqrt(2) * 1.01f;
	// initial slice at root:
	std::vector<bool> visited(pipeGroup.PeekPipes().size(), false);

	// prepare initial pipe cluster:
	PipeCluster pipeCluster;

	for (size_t i = 0; i < pipeGroup.PeekPipes().size(); i++)
	{
		pipeCluster.push_back(pipeGroup.PeekPipes()[i].GetHandle());
	}

	float stepSize = 0.5f;
	float max = settings.m_maxParam;

	auto prevSlice = computeSlice(treeModel, pipeCluster, 0, visited, 0.0, maxDist);

	// for now just do a second one and that's it
	/*for (float t = stepSize; t <= max; t += stepSize)
	{
		auto slicesAndClusters = computeSlices(treeModel, prevSlice.second, t, maxDist);
		std::vector<Slice> topSlices;
		for (auto& s : slicesAndClusters)
		{
			topSlices.push_back(s.first);
		}
		connectSlices(pipeGroup, prevSlice.first, topSlices, vertices, indices);
		prevSlice = slicesAndClusters[0];
	}*/
	sliceRecursively(treeModel, prevSlice, stepSize, stepSize, max, maxDist, vertices, indices);

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
}
