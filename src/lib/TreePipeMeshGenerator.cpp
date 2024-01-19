#include "TreePipeMeshGenerator.hpp"
#include "Curve.hpp"
#include "Octree.hpp"
#include "Jobs.hpp"
#include "opensubdiv/bfr/faceSurface.h"
#include "regression/bfr_evaluate/bfrSurfaceEvaluator.h"
#include <glm/gtx/intersect.hpp>
#include <glm/gtx/io.hpp>
#include "MeshGenUtils.hpp"
#include "Delaunator2D.hpp"

#define DEBUG_OUTPUT false

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
	ImGui::DragInt("Steps per segment", &m_stepsPerSegment, 1.0f, 1, 99);

	//ImGui::Checkbox("[DEBUG] Limit Profile Iterations", &m_limitProfileIterations);
	//ImGui::DragInt("[DEBUG] Limit", &m_maxProfileIterations);

	//ImGui::DragFloat("[DEBUG] MaxParam", &m_maxParam);
	ImGui::Checkbox("Compute branch joints", &m_branchConnections);
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

void delaunay(Graph& g, float removalLength, std::vector<size_t>& candidates)
{
	std::vector<float> positions;

	for (size_t index : candidates)
	{
		//if(DEBUG_OUTPUT) std::cout << "adding vertex " << index << std::endl;
		positions.push_back(g[index].x);
		positions.push_back(g[index].y);
	}

	//if(DEBUG_OUTPUT) std::cout << "calling delaunator" << std::endl;
	const Delaunator::Delaunator2D d(positions);
	//if(DEBUG_OUTPUT) std::cout << "collecting edges" << std::endl;
	for (std::size_t i = 0; i < d.triangles.size(); i += 3) {
		const auto& v0 = candidates[d.triangles[i]];
		const auto& v1 = candidates[d.triangles[i + 1]];
		const auto& v2 = candidates[d.triangles[i + 2]];
		if (glm::distance(g[v0], g[v1]) > removalLength
			|| glm::distance(g[v1], g[v2]) > removalLength
			|| glm::distance(g[v0], g[v2]) > removalLength) continue;

		//if(DEBUG_OUTPUT) std::cout << "adding triangle " << v0 << ", " << v1 << ", " << v2 << std::endl;
		g.addEdge(v0, v1);
		g.addEdge(v1, v2);
		g.addEdge(v2, v0);
	}
}

std::vector<size_t> collectComponent(const Graph& g, size_t startIndex, std::vector<bool>& visitedUnused)
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
	//if(DEBUG_OUTPUT) std::cout << "Flipped angle to " << prevAngle << std::endl;

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
		//if(DEBUG_OUTPUT) std::cout << " checking neighbor " << av << " with angle " << angle << std::endl;
		
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

	if (next == -1)
	{
		std::cout << "Warning: reached a leaf" << std::endl;

		return prev;
	}

	// confine to range
	if (nextAngle > glm::pi<float>())
	{
		nextAngle -= 2 * glm::pi<float>();
	}

	//if(DEBUG_OUTPUT) std::cout << "Selected neighbor " << next << " with angle " << nextAngle << std::endl;

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

std::vector<size_t> computeCluster(Graph& strandGraph, glm::vec3 min, glm::vec3 max, float maxDist, const PipeCluster& pipesInPrevious, size_t index, float t, std::vector<bool>& visited)
{
	Grid2D grid(maxDist, min, max);

	//if(DEBUG_OUTPUT) std::cout << "inserting " << strandGraph.m_vertices.size() << " vertices into grid" << std::endl;
	for (size_t i = 0; i < strandGraph.m_vertices.size(); i++)
	{
		if (!visited[i])
		{
			grid.insert(strandGraph, i);
		}
	}

	//if(DEBUG_OUTPUT) std::cout << "connecting neighbors" << std::endl;
	grid.connectNeighbors(strandGraph, maxDist);

	//if(DEBUG_OUTPUT) std::cout << "outputting graph" << std::endl;
	outputGraph(strandGraph, "strandGraph_" + std::to_string(pipesInPrevious[index]) + "_" + std::to_string(t), pipesInPrevious);

	// TODO:: maybe also use visited here
	//if(DEBUG_OUTPUT) std::cout << "collecting component" << std::endl;
	return collectComponent(strandGraph, index, visited);
}

std::pair<Slice, PipeCluster> computeSlice(const TreeModel& treeModel, const PipeCluster& pipesInPrevious, size_t index, std::vector<bool>& visited, float t, float maxDist)
{
	const PipeModelPipeGroup& pipes = treeModel.PeekShootSkeleton().m_data.m_pipeGroup;

	if (!isValidPipeParam(treeModel, pipesInPrevious[index], t))
	{
		return std::make_pair<Slice, PipeCluster>(Slice(), PipeCluster());
	}
	// sweep over tree from root to leaves to reconstruct a skeleton with bark outlines
	//if(DEBUG_OUTPUT) std::cout << "obtaining profile of segGroup with " << pipesInPrevious.size() << " segments" << std::endl;
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

	//if(DEBUG_OUTPUT) std::cout << "new basis vectors: " << std::endl;

	for (size_t i = 0; i < 3; i++)
	{
		//if(DEBUG_OUTPUT) std::cout << "basis vector " << i << ": " << basis[i] << std::endl;
	}

	glm::mat3x3 basisTrans =
	{
		{basis[0][0], basis[1][0], basis[2][0]},
		{basis[0][1], basis[1][1], basis[2][1]},
		{basis[0][2], basis[1][2], basis[2][2]}
	};

	// TODO: could we do this more efficiently? E.g. the Intel Embree library provides efficent ray casts and allows for ray bundling
	// Also, this is an experimental extension to glm and might not be stable

	// now project all of them onto plane
	//if(DEBUG_OUTPUT) std::cout << "building strand graph" << std::endl;
	Graph strandGraph;
	glm::vec3 min(std::numeric_limits<float>::infinity());
	glm::vec3 max(-std::numeric_limits<float>::infinity());

	for (auto& pipeHandle : pipesInPrevious)
	{
		//if(DEBUG_OUTPUT) std::cout << "processing pipe with handle no. " << pipeHandle << std::endl;
		glm::vec3 segPos = getPipePos(treeModel, pipeHandle, t);
		glm::vec3 segDir = glm::normalize(getPipeDir(treeModel, pipeHandle, t));

		// There appears to be a bug (at least in debug compilations) where this value is not set if the result is close to 0, so we need to set it.
		float param = 0.0f;

		glm::intersectRayPlane(segPos, segDir, planePos, planeNorm, param);
		//if(DEBUG_OUTPUT) std::cout << "line: " << segPos << " + t * " << segDir << " intersects plane " << planeNorm << " * (p - " << planePos << ") = 0 at t = " << param << std::endl;
		// store the intersection point in a graph
		size_t vIndex = strandGraph.addVertex();
		glm::vec3 pos = basisTrans * (segPos + segDir * param);
		//if(DEBUG_OUTPUT) std::cout << "mapped point " << (segPos + segDir * param) << " to " << pos << std::endl;
		strandGraph[vIndex] = pos;

		min = glm::min(min, pos);
		max = glm::max(max, pos);
	}
	//if(DEBUG_OUTPUT) std::cout << "built graph of size " << strandGraph.m_vertices.size() << std::endl;

	// TODO: we should check that maxDist is at least as large as the thickest strand
	// now cluster anything below maxDist
	std::vector<size_t> cluster;

	std::vector<size_t> candidates;

	for (size_t i = 0; i < strandGraph.m_vertices.size(); i++)
	{
		if (!visited[i])
		{
			candidates.push_back(i);
		}
	}

	if (candidates.size() < 3)
	{
		std::cerr << "Error: cluster is too small" << std::endl;
	}
	else if (candidates.size() == 3)
	{
		if (DEBUG_OUTPUT) std::cout << "Defaulting to triangle" << std::endl;
		for (size_t i = 0; i < candidates.size(); i++)
		{
			cluster.push_back(candidates[i]);
			strandGraph.addEdge(candidates[i], candidates[(i + 1) % candidates.size()]);
		}
	}
	else
	{
		if(DEBUG_OUTPUT) std::cout << "computing cluster..." << std::endl;
		//cluster = computeCluster(strandGraph, min, max, maxDist * 2, pipesInPrevious, index, t, visited);

		//if(DEBUG_OUTPUT) std::cout << "executing delaunay..." << std::endl;

		delaunay(strandGraph, maxDist * 2, candidates);

		cluster = collectComponent(strandGraph, index, visited);

		//if(DEBUG_OUTPUT) std::cout << "done with delaunay..." << std::endl;
		outputGraph(strandGraph, "strandGraph_" + std::to_string(pipesInPrevious[index]) + "_" + std::to_string(t) + "_del", pipesInPrevious);

	}
	// write cluster
	//if(DEBUG_OUTPUT) std::cout << "collecting pipe handles in cluster.." << std::endl;
	PipeCluster pipesInComponent;

	for (size_t indexInComponent : cluster)
	{
		if (visited[indexInComponent])
		{
			std::cerr << "Error: index is already part of a different component" << std::endl;
		}

		visited[indexInComponent] = true;
		//if(DEBUG_OUTPUT) std::cout << "Marking strand no. " << indexInComponent << " with handle " << pipesInPrevious[indexInComponent] << " as visited." << std::endl;
		pipesInComponent.push_back(pipesInPrevious[indexInComponent]);
	}

	if(DEBUG_OUTPUT) std::cout << "finding leftmost" << std::endl;
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
	if(DEBUG_OUTPUT) std::cout << "traversing boundary" << std::endl;
	//if(DEBUG_OUTPUT) std::cout << "starting at index: " << leftmostIndex << std::endl;
	std::vector<size_t> profile{ leftmostIndex };

	// going counterclockwise from here, the next point must be the one with the smallest angle
	size_t next = -1;
	float minAngle = glm::pi<float>();

	for (size_t av : strandGraph.adjacentVertices(leftmostIndex))
	{
		glm::vec2 dir = strandGraph[av] - strandGraph[leftmostIndex];
		float angle = atan2f(dir.y, dir.x);
		//if(DEBUG_OUTPUT) std::cout << "checking neighbor " << *ai << " with angle " << angle << std::endl;
		if (angle < minAngle)
		{
			minAngle = angle;
			next = av;
		}
	}

	if (next == -1)
	{
		std::cerr << "Error: leftmost index has no neighbors" << std::endl;
	}

	// from now on we will have to find the closest neighboring edge in counter-clockwise order for each step until we reach the first index again
	size_t prev = leftmostIndex;
	size_t cur = next;
	float prevAngle = minAngle;

	size_t debugCounter = 0;
	while (next != leftmostIndex && debugCounter < 1000)
	{
		//if(DEBUG_OUTPUT) std::cout << " Iteration no. " << debugCounter << std::endl;
		if (cur == -1)
		{
			std::cerr << "Error: cur is -1" << std::endl;
			break;
		}

		//if(DEBUG_OUTPUT) std::cout << "cur: " << cur << std::endl;
		profile.push_back(cur);

		next = getNextOnBoundary(strandGraph, cur, prev, prevAngle);
		prev = cur;
		cur = next;

		debugCounter++;
	}

	Slice slice = profileToSlice(treeModel, profile, pipesInPrevious, t);

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
			//std::cout << "skipping already visited pipe no. " << i << " with handle " << pipesInPrevious[i] << std::endl;
			continue;
		}

		// if not visited, determine connected component around this
		//std::cout << "computing slice containing pipe no. " << i << " with handle " << pipesInPrevious[i] << " at t = " << t << std::endl;
		auto slice = computeSlice(treeModel, pipesInPrevious, i, visited, t, maxDist);
		slices.push_back(slice);
	}

	return slices;
}

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

void testEqual(glm::vec3 a, glm::vec3 b)
{
	if (a != b)
	{
		std::stringstream errorMessage;
		errorMessage << "Error: Vectors are not equal! a = " << a << ", b = " << b;
		throw std::runtime_error(errorMessage.str());
	}
}

void connect(std::vector<std::pair<PipeHandle, glm::vec3> >& slice0, size_t i0, size_t j0, size_t offset0,
	std::vector<std::pair<PipeHandle, glm::vec3> >& slice1, size_t i1, size_t j1, size_t offset1,
	std::vector<Vertex>& vertices, std::vector<unsigned>& indices)
{
	//if(DEBUG_OUTPUT) std::cout << "connecting " << i0 << ", " << j0 << " to " << i1 << ", " << j1 << std::endl;
	size_t vertBetween0 = (j0 + slice0.size() - i0) % slice0.size();
	size_t vertBetween1 = (j1 + slice1.size() - i1) % slice1.size();
	//if(DEBUG_OUTPUT) std::cout << vertBetween0 << " and " << vertBetween1 << " steps, respectively " << std::endl;
	size_t offset = vertices.size();

	// merge the two
	//if(DEBUG_OUTPUT) std::cout << "connecting slices with triangles" << std::endl;
	size_t k0 = 0;
	size_t k1 = 0;

	while (k0 < vertBetween0 || k1 < vertBetween1)
	{
		if (k0 / double(vertBetween0) < k1 / double(vertBetween1))
		{
			// make triangle consisting of k0, k0 + 1 and k1
			indices.push_back(offset0 + (i0 + k0 + 1) % slice0.size());
			indices.push_back(offset0 + (i0 + k0) % slice0.size());
			indices.push_back(offset1 + (i1 + k1) % slice1.size());

			testEqual(slice0[(i0 + k0 + 1) % slice0.size()].second, vertices[offset0 + (i0 + k0 + 1) % slice0.size()].m_position);
			testEqual(slice0[(i0 + k0 ) % slice0.size()].second, vertices[offset0 + (i0 + k0 ) % slice0.size()].m_position);
			testEqual(slice1[(i1 + k1) % slice1.size()].second, vertices[offset1 + (i1 + k1) % slice1.size()].m_position);

			k0++;
			//if(DEBUG_OUTPUT) std::cout << "increased k0 to " << k0 << std::endl;
		}
		else
		{
			// make triangle consisting of k1, k1 + 1 and k0
			indices.push_back(offset1 + (i1 + k1) % slice1.size());
			indices.push_back(offset1 + (i1 + k1 + 1) % slice1.size());
			indices.push_back(offset0 + (i0 + k0) % slice0.size());

			
			testEqual(slice1[(i1 + k1) % slice1.size()].second, vertices[offset1 + (i1 + k1) % slice1.size()].m_position);
			testEqual(slice1[(i1 + k1 + 1) % slice1.size()].second, vertices[offset1 + (i1 + k1 + 1) % slice1.size()].m_position);
			testEqual(slice0[(i0 + k0) % slice0.size()].second, vertices[offset0 + (i0 + k0) % slice0.size()].m_position);

			k1++;
			//if(DEBUG_OUTPUT) std::cout << "increased k1 to " << k1 << std::endl;
		}
	}
}

size_t midIndex(size_t a, size_t b, size_t size)
{
	int mid = b - a;

	if (mid < 0)
	{
		mid += size;
	}

	return (a + mid / 2) % size;
}

void connectSlices(const PipeModelPipeGroup& pipes, Slice& bottomSlice, size_t bottomOffset,
	std::vector<Slice>& topSlices, std::vector<size_t> topOffsets,
	std::vector<Vertex>& vertices, std::vector<unsigned>& indices, bool branchConnections)
{
	// TODO: support branches
	if (topSlices.size() > 1)
	{
		std::cout << "connecting " << topSlices.size() << " top slices to bottom slice" << std::endl;
	}

	//if(DEBUG_OUTPUT) std::cout << "slice0: ";
	for (auto& e : bottomSlice)
	{
		//if(DEBUG_OUTPUT) std::cout << e.first << ", ";
	}
	if(DEBUG_OUTPUT) std::cout << std::endl;

	// compute (incomplete) permutation that turns 0 into 1
	//if(DEBUG_OUTPUT) std::cout << "computing permutations for " << pipes.PeekPipes().size() << "pipes..." << std::endl;
	std::vector<std::pair<size_t, size_t> > topPipeHandleIndexMap(pipes.PeekPipes().size(), std::make_pair<>(-1,-1));
	std::vector<size_t> bottomPipeHandleIndexMap(pipes.PeekPipes().size(), -1);
	//if(DEBUG_OUTPUT) std::cout << "size of pipeHandleIndexMap: " << pipeHandleIndexMap.size() << std::endl;

	for (size_t s = 0; s < topSlices.size(); s++)
	{
		for (size_t i = 0; i < topSlices[s].size(); i++)
		{
			//if(DEBUG_OUTPUT) std::cout << "inserting pipe handle no. " << topSlices[s][i].first << std::endl;
			topPipeHandleIndexMap[topSlices[s][i].first] = std::make_pair<>(s, i);
		}
	}

	for (size_t i = 0; i < bottomSlice.size(); i++)
	{
		bottomPipeHandleIndexMap[bottomSlice[i].first] = i;
	}

	std::vector<std::pair<size_t, size_t> > bottomPermutation(bottomSlice.size(), std::make_pair<>(-1, -1));
	std::vector<std::vector<size_t> > topPermutations(topSlices.size());

	for (std::size_t s = 0; s < topSlices.size(); s++)
	{
		topPermutations[s] = std::vector<size_t>(topSlices[s].size(), -1);

		for (size_t i = 0; i < topPermutations[s].size(); i++)
		{
			topPermutations[s][i] = bottomPipeHandleIndexMap[topSlices[s][i].first];
		}
	}

	//if(DEBUG_OUTPUT) std::cout << "mapping back to permutation vector..." << std::endl;
	for (size_t i = 0; i < bottomPermutation.size(); i++)
	{
		bottomPermutation[i] = topPipeHandleIndexMap[bottomSlice[i].first];
	}

	// now we can more or less just connect them, except that we need to figure out the correct alignment and also get rid of inversions
	// TODO: for now just assume that inversions do not happen
	size_t prevI = -1;
	//if(DEBUG_OUTPUT) std::cout << "Finding first index..." << std::endl;
	for (size_t i = bottomPermutation.size(); i > 0; i--)
	{
		if (bottomPermutation[i - 1].second != -1)
		{
			prevI = i - 1;
			break;
		}
	}

	//if(DEBUG_OUTPUT) std::cout << "Found first index " << prevI << std::endl;
	for (size_t i = 0; i < bottomPermutation.size(); i++)
	{
		if (bottomPermutation[i].second == -1)
		{
			//if(DEBUG_OUTPUT) std::cout << "No correspondence at index " << i << std::endl;
			continue;
		}

		if (bottomPermutation[prevI].first == bottomPermutation[i].first)
		{
			//if(DEBUG_OUTPUT) std::cout << "Connecting at index " << i << std::endl;

			if (topSlices.size() > 1)
			{
				std::cout << "connecting top slice no. " << bottomPermutation[prevI].first << " to bottom slice" << std::endl;
				std::cout << "Bottom indices " << prevI << " to " << i << std::endl;
				std::cout << "Top indices " << bottomPermutation[prevI].second << " to " << bottomPermutation[i].second << std::endl;
			}

			connect(bottomSlice, prevI, i, bottomOffset,
				topSlices[bottomPermutation[i].first], bottomPermutation[prevI].second, bottomPermutation[i].second, topOffsets[bottomPermutation[i].first],
				vertices, indices);
		}
		else
		{
			//std::cout << "Multiple branches not implemented yet!" << std::endl;
			// idea: walk half of each top slice to the next matching strand

			// first find next point with correspondence on top profile
			size_t nextIndex = -1;

			for (size_t j = (bottomPermutation[prevI].second + 1) % topSlices[bottomPermutation[prevI].first].size();
				nextIndex == -1;
				j = (j + 1) % topSlices[bottomPermutation[prevI].first].size())
			{
				if (topPermutations[bottomPermutation[prevI].first][j] != -1)
				{
					nextIndex = j;
				}
			}

			// now do the same for the other slice 
			size_t prevIndex = -1;

			for (size_t j = (bottomPermutation[i].second == 0 ? topSlices[bottomPermutation[i].first].size() - 1 : bottomPermutation[i].second - 1);
				prevIndex == -1;
				j = (j == 0 ? topSlices[bottomPermutation[i].first].size() - 1 : j - 1))
			{
				if (topPermutations[bottomPermutation[i].first][j] != -1)
				{
					prevIndex = j;
				}
			}

			size_t bottomMid = midIndex(prevI, i, bottomSlice.size());

			size_t nextMid = midIndex(bottomPermutation[prevI].second, nextIndex, topSlices[bottomPermutation[prevI].first].size());
			size_t prevMid = midIndex(prevIndex, bottomPermutation[i].second, topSlices[bottomPermutation[i].first].size());

			// for now let us do a very simple test if we selected the correct indices
			if (branchConnections)
			{
				connect(bottomSlice, prevI, bottomMid, bottomOffset,
					topSlices[bottomPermutation[prevI].first], bottomPermutation[prevI].second, nextMid, topOffsets[bottomPermutation[prevI].first],
					vertices, indices);
				if (DEBUG_OUTPUT) std::cout << "Connected bottom indices " << prevI << " to " << bottomMid << " with " << bottomPermutation[prevI].second << " to "
					<< nextMid << " of top profile no. " << bottomPermutation[prevI].first << std::endl;

				// connect mid indices with triangle
				indices.push_back(bottomOffset + bottomMid);
				indices.push_back(topOffsets[bottomPermutation[prevI].first] + nextMid);
				indices.push_back(topOffsets[bottomPermutation[i].first] + prevMid);

				// TODO: connecting with the same top slice looks better

				//indices.push_back()

				connect(bottomSlice, bottomMid, i, bottomOffset,
					topSlices[bottomPermutation[i].first], prevMid, bottomPermutation[i].second, topOffsets[bottomPermutation[i].first],
					vertices, indices);

				if (DEBUG_OUTPUT) std::cout << "Connected bottom indices " << bottomMid << " to " << i << " with " << prevMid << " to "
					<< bottomPermutation[i].second << " of top profile no. " << bottomPermutation[i].first << std::endl;
			}
		}

		prevI = i;
	}
}

void createTwigTip(const TreeModel& treeModel, std::pair < Slice, PipeCluster>& prevSlice, size_t prevOffset, float t,
	std::vector<Vertex>& vertices, std::vector<unsigned>& indices)
{
	// compute average positions of all slice points
	glm::vec3 pos(0, 0, 0);

	for (auto& el : prevSlice.second)
	{
		pos += getPipePos(treeModel, el, t - 0.01);
	}

	Vertex v;
	v.m_position = pos / float(prevSlice.second.size());

	size_t tipIndex = vertices.size();
	vertices.push_back(v);

	for (size_t i = 0; i < prevSlice.second.size(); i++)
	{
		indices.push_back(prevOffset + i);
		indices.push_back(tipIndex);
		indices.push_back(prevOffset + (i + 1) % prevSlice.second.size());
	}
}

void sliceRecursively(const TreeModel& treeModel, std::pair < Slice, PipeCluster>& prevSlice, size_t prevOffset, float t, float stepSize, float maxDist,
	std::vector<Vertex>& vertices, std::vector<unsigned>& indices, bool branchConnections)
{
	const auto& skeleton = treeModel.PeekShootSkeleton();
	const auto& pipeGroup = skeleton.m_data.m_pipeGroup;

	// prevent problems with floating point arithmetics
	if (t + 0.01 > glm::ceil(t))
	{
		t = glm::ceil(t);
	}

	auto slicesAndClusters = computeSlices(treeModel, prevSlice.second, t, maxDist);
	std::vector<Slice> topSlices;

	bool allEmpty = true;

	for (auto& s : slicesAndClusters)
	{
		topSlices.push_back(s.first);

		if (s.first.size() != 0)
		{
			allEmpty = false;
		}
	}

	if (allEmpty)
	{
		if(DEBUG_OUTPUT) std::cout << "=== Ending recursion at t = " << t << " ===" << std::endl;

		createTwigTip(treeModel, prevSlice, prevOffset, t, vertices, indices);

		return;
	}


	std::vector<size_t> offsets;

	// create vertices
	for (Slice& s : topSlices)
	{
		offsets.push_back(vertices.size());

		for (auto& el : s)
		{
			Vertex v;
			v.m_position = el.second;
			vertices.push_back(v);
		}
	}
	
	connectSlices(pipeGroup, prevSlice.first, prevOffset, topSlices, offsets, vertices, indices, branchConnections);

	if (DEBUG_OUTPUT) std::cout << "--- Done with slice at t = " << t << " ---" << std::endl;
	// recursive call
	t += stepSize;
	for (size_t i = 0; i < slicesAndClusters.size(); i++)
	{
		if (slicesAndClusters[i].first.size() != 0)
		{
			std::cout << "___ Slice at t = " << t << " ___" << std::endl;
			sliceRecursively(treeModel, slicesAndClusters[i], offsets[i], t, stepSize, maxDist, vertices, indices, branchConnections);
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

	if(DEBUG_OUTPUT) std::cout << "getting first seg group" << std::endl;
	std::vector<PipeSegmentHandle> segGroup0 = getSegGroup(pipeGroup, 0);

	if(DEBUG_OUTPUT) std::cout << "determining max thickness" << std::endl;
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

	//size_t maxIterations = settings.m_limitProfileIterations ? settings.m_maxProfileIterations : std::numeric_limits<size_t>::max();
	float maxDist = 2 * maxThickness * sqrt(2) * 1.01f;
	// initial slice at root:
	std::vector<bool> visited(pipeGroup.PeekPipes().size(), false);

	// prepare initial pipe cluster:
	PipeCluster pipeCluster;

	for (size_t i = 0; i < pipeGroup.PeekPipes().size(); i++)
	{
		pipeCluster.push_back(pipeGroup.PeekPipes()[i].GetHandle());
	}

	float stepSize = 1.0f / settings.m_stepsPerSegment;
	//float max = settings.m_maxParam;

	auto firstSlice = computeSlice(treeModel, pipeCluster, 0, visited, 0.0, maxDist);

	// create initial vertices
	for (auto& el : firstSlice.first)
	{
		Vertex v;
		v.m_position = el.second;
		vertices.push_back(v);
	}

	sliceRecursively(treeModel, firstSlice, 0, stepSize, stepSize, maxDist, vertices, indices, settings.m_branchConnections);

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
