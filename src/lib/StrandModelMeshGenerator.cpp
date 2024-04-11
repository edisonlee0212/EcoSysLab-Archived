#include "StrandModelMeshGenerator.hpp"
#include "Curve.hpp"
#include "Octree.hpp"
#include "Jobs.hpp"
#include "TreeMeshGenerator.hpp"
#include <glm/gtx/intersect.hpp>
#include <glm/gtx/io.hpp>
#include "MeshGenUtils.hpp"
#include "Delaunator2D.hpp"
#include "TreeDescriptor.hpp"

#define DEBUG_OUTPUT false

using namespace EcoSysLab;

typedef std::vector<std::pair<StrandHandle, glm::vec3> > Slice;
typedef std::vector<StrandHandle> PipeCluster;

void StrandModelMeshGeneratorSettings::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	ImGui::Combo("Mode", { "Recursive Slicing", "Hybrid Marching Cube" }, m_generatorType);
	if (m_generatorType == 0 && ImGui::TreeNode("Recursive Slicing settings"))
	{
		ImGui::DragInt("Steps per segment", &m_stepsPerSegment, 1.0f, 1, 99);

		//ImGui::Checkbox("[DEBUG] Limit Profile Iterations", &m_limitProfileIterations);
		//ImGui::DragInt("[DEBUG] Limit", &m_maxProfileIterations);

		//ImGui::DragFloat("[DEBUG] MaxParam", &m_maxParam);
		//ImGui::Checkbox("Compute branch joints", &m_branchConnections);
		ImGui::DragInt("uCoord multiplier", &m_uMultiplier, 1, 1);
		ImGui::DragFloat("vCoord multiplier", &m_vMultiplier, 0.1f);
		ImGui::DragFloat("cluster distance factor", &m_clusterDistance, 0.1f, 1.0f, 10.0f);
		ImGui::TreePop();
	}

	if (m_generatorType == 1 && ImGui::TreeNode("Hybrid Marching Cube settings"))
	{
		ImGui::Checkbox("Auto set level", &m_autoLevel);
		if (!m_autoLevel) ImGui::DragInt("Voxel subdivision level", &m_voxelSubdivisionLevel, 1, 5, 16);
		else ImGui::DragFloat("Min Cube size", &m_marchingCubeRadius, 0.0001, 0.001f, 1.0f);
		if (m_smoothIteration == 0) ImGui::Checkbox("Remove duplicate", &m_removeDuplicate);
		ImGui::ColorEdit4("Marching cube color", &m_marchingCubeColor.x);
		ImGui::ColorEdit4("Cylindrical color", &m_cylindricalColor.x);
		ImGui::DragInt("uCoord multiplier", &m_rootDistanceMultiplier, 1, 1, 100);
		ImGui::DragFloat("vCoord multiplier", &m_circleMultiplier, 0.1f);
		ImGui::TreePop();
	}



	ImGui::DragInt("Major branch cell min", &m_minCellCountForMajorBranches, 1, 0, 1000);
	ImGui::DragInt("Minor branch cell max", &m_maxCellCountForMinorBranches, 1, 0, 1000);

	ImGui::Checkbox("Recalculate UV", &m_recalculateUV);
	ImGui::Checkbox("Fast UV", &m_fastUV);
	ImGui::DragInt("Smooth iteration", &m_smoothIteration, 0, 0, 10);
	ImGui::Checkbox("Branch", &m_enableBranch);
	ImGui::Checkbox("Foliage", &m_enableFoliage);
}

void StrandModelMeshGenerator::Generate(const StrandModel& strandModel, std::vector<Vertex>& vertices,
	std::vector<unsigned>& indices, const StrandModelMeshGeneratorSettings& settings)
{
	const float meshFormationTime = Times::Now();
	switch (settings.m_generatorType)
	{
	case StrandModelMeshGeneratorType::RecursiveSlicing:
	{
		RecursiveSlicing(strandModel, vertices, indices, settings);
	}break;
	case StrandModelMeshGeneratorType::HybridMarchingCube:
	{
		MarchingCube(strandModel, vertices, indices, settings);
	}break;
	}
	EVOENGINE_LOG("Mesh formation finished in: " + std::to_string(Times::Now() - meshFormationTime) + "s.");

		if (settings.m_recalculateUV || settings.m_generatorType == static_cast<unsigned>(StrandModelMeshGeneratorType::HybridMarchingCube)) {
			const float recalculateUVTime = Times::Now();
			CalculateUV(strandModel, vertices, settings);
			EVOENGINE_LOG("Recalculate UV time: " + std::to_string(Times::Now() - recalculateUVTime) + "s.");
		}
	
	const float meshSmoothingTime = Times::Now();
	for (int i = 0; i < settings.m_smoothIteration; i++)
	{
		MeshSmoothing(vertices, indices);
	}
	EVOENGINE_LOG("Mesh smoothing time: " + std::to_string(Times::Now() - meshSmoothingTime) + "s.");
	CylindricalMeshing(strandModel, vertices, indices, settings);
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

void verifyMesh(std::vector<Vertex>& vertices, std::vector<unsigned>& indices)
{
	std::cerr << "checking indices for " << vertices.size() << " vertices..." << std::endl;
	for (size_t index : indices)
	{
		if (index >= vertices.size())
		{
			std::cerr << "index " << index << " is out of range" << std::endl;
		}
	}
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

std::vector<StrandSegmentHandle> getNextSegGroup(const StrandModelStrandGroup& pipes, const std::vector<StrandSegmentHandle>& segGroup)
{
	std::vector<StrandSegmentHandle> nextSegGroup;

	for (const StrandSegmentHandle& segHandle : segGroup)
	{
		auto& seg = pipes.PeekStrandSegment(segHandle);
		if (!seg.IsEnd())
		{
			nextSegGroup.push_back(seg.GetNextHandle());
		}
	}

	return nextSegGroup;
}

glm::vec3 getSegDir(const StrandModel& strandModel, const StrandSegmentHandle segHandle, float t = 1.0f)
{
	return strandModel.InterpolateStrandSegmentAxis(segHandle, t);
}

glm::vec3 getSegPos(const StrandModel& strandModel, const StrandSegmentHandle segHandle, float t = 1.0f)
{
	auto retVal = strandModel.InterpolateStrandSegmentPosition(segHandle, t);
	for (size_t i = 0; i < 3; i++)
	{
		if (std::isinf(retVal[i]))
		{
			std::cerr << "Error: Interpolated segment position is infinity" << std::endl;
		}

		if (std::isnan(retVal[i]))
		{
			std::cerr << "Error: Interpolated segment position is not a number" << std::endl;
		}
	}
	return retVal;
}

NodeHandle getNodeHandle(const StrandModelStrandGroup& pipeGroup, const StrandHandle& pipeHandle, float t)
{
	size_t lookupIndex = glm::round(t) < pipeGroup.PeekStrand(pipeHandle).PeekStrandSegmentHandles().size() ? glm::round(t) : (pipeGroup.PeekStrand(pipeHandle).PeekStrandSegmentHandles().size() - 1);
	auto& pipeSegmentHandle = pipeGroup.PeekStrand(pipeHandle).PeekStrandSegmentHandles()[lookupIndex];
	auto& pipeSegment = pipeGroup.PeekStrandSegment(pipeSegmentHandle);

	return pipeSegment.m_data.m_nodeHandle;
}

bool isValidPipeParam(const StrandModel& strandModel, const StrandHandle& pipeHandle, float t)
{
	const auto& pipe = strandModel.m_strandModelSkeleton.m_data.m_strandGroup.PeekStrand(pipeHandle);
	return pipe.PeekStrandSegmentHandles().size() > glm::floor(t);
}

glm::vec3 getPipeDir(const StrandModel& strandModel, const StrandHandle& pipeHandle, float t)
{
	const auto& pipe = strandModel.m_strandModelSkeleton.m_data.m_strandGroup.PeekStrand(pipeHandle);
	auto segHandle = pipe.PeekStrandSegmentHandles()[t];
	return getSegDir(strandModel, segHandle, fmod(t, 1.0));
}

glm::vec3 getPipePos(const StrandModel& strandModel, const StrandHandle& pipeHandle, float t)
{
	const auto& pipe = strandModel.m_strandModelSkeleton.m_data.m_strandGroup.PeekStrand(pipeHandle);
	auto segHandle = pipe.PeekStrandSegmentHandles()[t];
	return getSegPos(strandModel, segHandle, fmod(t, 1.0));
}

const Particle2D<CellParticlePhysicsData>& getEndParticle(const StrandModel& strandModel, const StrandHandle& pipeHandle, size_t index)
{
	if (!isValidPipeParam(strandModel, pipeHandle, index))
	{
		std::cerr << "Error: Strand " << pipeHandle << " does not exist at " << index << std::endl;
	}

	const auto& skeleton = strandModel.m_strandModelSkeleton;
	const auto& pipe = skeleton.m_data.m_strandGroup.PeekStrand(pipeHandle);
	auto segHandle = pipe.PeekStrandSegmentHandles()[index];
	auto& pipeSegment = skeleton.m_data.m_strandGroup.PeekStrandSegment(segHandle);

	const auto& node = skeleton.PeekNode(pipeSegment.m_data.m_nodeHandle);
	const auto& startProfile = node.m_data.m_profile;
	//To access the user's defined constraints (attractors, etc.)
	const auto& profileConstraints = node.m_data.m_profileConstraints;

	//To access the position of the start of the pipe segment within a profile:
	const auto parentHandle = node.GetParentHandle();
	const auto& endParticle = startProfile.PeekParticle(pipeSegment.m_data.m_profileParticleHandle);

	return endParticle;
}

const Particle2D<CellParticlePhysicsData>& getStartParticle(const StrandModel& strandModel, const StrandHandle& pipeHandle, size_t index)
{
	if (!isValidPipeParam(strandModel, pipeHandle, index))
	{
		std::cerr << "Error: Strand " << pipeHandle << " does not exist at " << index << std::endl;
	}

	const auto& skeleton = strandModel.m_strandModelSkeleton;
	const auto& pipe = skeleton.m_data.m_strandGroup.PeekStrand(pipeHandle);
	auto segHandle = pipe.PeekStrandSegmentHandles()[index];
	auto& pipeSegment = skeleton.m_data.m_strandGroup.PeekStrandSegment(segHandle);

	const auto& node = skeleton.PeekNode(pipeSegment.m_data.m_nodeHandle);
	const auto& startProfile = node.m_data.m_profile;
	//To access the user's defined constraints (attractors, etc.)
	const auto& profileConstraints = node.m_data.m_profileConstraints;

	//To access the position of the start of the pipe segment within a profile:
	const auto parentHandle = node.GetParentHandle();
	const auto& startParticle = startProfile.PeekParticle(pipeSegment.m_data.m_profileParticleHandle);

	return startParticle;
}

float getPipePolar(const StrandModel& strandModel, const StrandHandle& pipeHandle, float t)
{
	// cheap interpolation, maybe improve this later ?
	const auto& p0 = getStartParticle(strandModel, pipeHandle, std::floor(t));
	const auto& p1 = getEndParticle(strandModel, pipeHandle, std::floor(t));
	float a1 = p1.GetPolarPosition().y;

	if (isValidPipeParam(strandModel, pipeHandle, std::ceil(t)))
	{
		const auto& p1 = getStartParticle(strandModel, pipeHandle, std::ceil(t));
		a1 = p1.GetPolarPosition().y;
	}


	float a0 = p0.GetPolarPosition().y;


	// we will just assume that the difference cannot exceed 180 degrees
	if (a1 < a0)
	{
		std::swap(a0, a1);
	}

	float interpolationParam = fmod(t, 1.0f);
	float angle;

	if (a1 - a0 > glm::pi<float>())
	{
		// rotation wraps around
		angle = fmod((a0 + 2 * glm::pi<float>()) * interpolationParam + a1 * (1 - interpolationParam), 2 * glm::pi<float>());

		if (angle > glm::pi<float>())
		{
			angle -= 2 * glm::pi<float>();
		}
	}
	else
	{
		angle = a0 * interpolationParam + a1 * (1 - interpolationParam);

		if (angle > glm::pi<float>())
		{
			angle -= 2 * glm::pi<float>();
		}
	}

	return angle;
}

void dfs(const Graph& g, size_t v, std::vector<size_t>& componentMembers, std::vector<bool>& visited)
{

	std::vector<size_t> stack;

	stack.push_back(v);

	while (!stack.empty())
	{
		size_t u = stack.back();
		stack.pop_back();

		if (!visited[u])
		{
			componentMembers.push_back(u);
			visited[u] = true;
		}

		for (size_t av : g.adjacentVertices(u))
		{
			if (!visited[av])
			{
				stack.push_back(av);
			}
		}

	}
}

void delaunay(Graph& g, float removalLength, std::vector<size_t>& candidates, const StrandModelStrandGroup& pipeGroup, const PipeCluster& prevPipes, float t)
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

		// make an exception if the three indices belong to the same skeleton part
		// does not work properly, just creates artefacts
		/*StrandHandle p0 = prevPipes[v0];
		StrandHandle p1 = prevPipes[v1];
		StrandHandle p2 = prevPipes[v2];

		NodeHandle n0 = getNodeHandle(pipeGroup, p0, t);
		NodeHandle n1 = getNodeHandle(pipeGroup, p1, t);
		NodeHandle n2 = getNodeHandle(pipeGroup, p2, t);

		if (n0 == n1 && n1 == n2)
		{
			g.addEdge(v0, v1);
			g.addEdge(v1, v2);
			g.addEdge(v2, v0);

			continue; // Don't add twice
		}*/

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

std::vector<StrandSegmentHandle> getSegGroup(const StrandModelStrandGroup& pipes, size_t index)
{
	std::vector<StrandSegmentHandle> segGroup;

	for (auto& pipe : pipes.PeekStrands())
	{
		if (pipe.PeekStrandSegmentHandles().size() > index)
		{
			segGroup.push_back(pipe.PeekStrandSegmentHandles()[index]);
		}
		else
		{
			segGroup.push_back(-1);
		}
	}

	return segGroup;
}


void obtainProfiles(const StrandModelStrandGroup& pipes, std::vector<StrandSegmentHandle> segGroup, float maxDist)
{
	std::vector<bool> visited(segGroup.size(), false);

	std::vector<std::vector<StrandSegmentHandle> > profiles;

	for (auto& segHandle : segGroup)
	{
		if (segHandle == -1)
		{
			continue;
		}

		auto& seg = pipes.PeekStrandSegment(segHandle);
		StrandHandle pipeHandle = seg.GetStrandHandle();

		if (seg.m_info.m_isBoundary && !visited[pipeHandle])
		{
			// traverse boundary
			std::vector<StrandSegmentHandle> profile;
			auto handle = segHandle;
			do
			{
				profile.push_back(handle);
				auto& seg = pipes.PeekStrandSegment(handle);
				StrandHandle pipeHandle = seg.GetStrandHandle();
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

Slice profileToSlice(const StrandModel& strandModel, std::vector<size_t>& profile, const PipeCluster& pipeCluster, float t)
{
	Slice slice;

	for (size_t i = 0; i < profile.size(); i++)
	{
		StrandHandle pipeHandle = pipeCluster[profile[i]];
		slice.push_back(std::pair<StrandHandle, glm::vec3>(pipeHandle, getPipePos(strandModel, pipeHandle, t)));
	}

	return slice;
}

std::vector<size_t> computeComponent(Graph& strandGraph, glm::vec3 min, glm::vec3 max, float maxDist, const PipeCluster& pipesInPrevious, size_t index, float t, std::vector<bool>& visited)
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
	//outputGraph(strandGraph, "strandGraph_" + std::to_string(pipesInPrevious[index]) + "_" + std::to_string(t), pipesInPrevious);

	// TODO:: maybe also use visited here
	//if(DEBUG_OUTPUT) std::cout << "collecting component" << std::endl;
	return collectComponent(strandGraph, index, visited);
}

std::pair<Graph, std::vector<size_t> > computeCluster(const StrandModel& strandModel, const PipeCluster& pipesInPrevious, size_t index, std::vector<bool>& visited, float t, float maxDist)
{
	if (!isValidPipeParam(strandModel, pipesInPrevious[index], t))
	{
		return std::make_pair<>(Graph(), std::vector<size_t>());
	}
	// sweep over tree from root to leaves to reconstruct a skeleton with bark outlines
	//if(DEBUG_OUTPUT) std::cout << "obtaining profile of segGroup with " << pipesInPrevious.size() << " segments" << std::endl;
	glm::vec3 planePos = getPipePos(strandModel, pipesInPrevious[index], t);
	glm::vec3 planeNorm = glm::normalize(getPipeDir(strandModel, pipesInPrevious[index], t));

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
		if (!isValidPipeParam(strandModel, pipeHandle, t))
		{
			// discard this pipe
			size_t vIndex = strandGraph.addVertex();
			visited[vIndex] = true;
			continue;
		}

		glm::vec3 segPos = getPipePos(strandModel, pipeHandle, t);
		glm::vec3 segDir = glm::normalize(getPipeDir(strandModel, pipeHandle, t));

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
		//std::cout << "Cluster is too small, will be discarded" << std::endl;
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
		if (DEBUG_OUTPUT) std::cout << "computing cluster..." << std::endl;
		delaunay(strandGraph, maxDist, candidates, strandModel.m_strandModelSkeleton.m_data.m_strandGroup, pipesInPrevious, t);

		cluster = collectComponent(strandGraph, index, visited);


	}
	// write cluster

	for (size_t indexInComponent : cluster)
	{
		if (visited[indexInComponent])
		{
			std::cerr << "Error: index is already part of a different component" << std::endl;
		}

		visited[indexInComponent] = true;
	}

	return std::make_pair<>(strandGraph, cluster);
}

std::pair<Slice, PipeCluster> computeSlice(const StrandModel& strandModel, const PipeCluster& pipesInPrevious, Graph& strandGraph, std::vector<size_t>& cluster, float t, float maxDist)
{
	const StrandModelStrandGroup& pipes = strandModel.m_strandModelSkeleton.m_data.m_strandGroup;

	PipeCluster pipesInComponent;

	for (size_t indexInComponent : cluster)
	{
		pipesInComponent.push_back(pipesInPrevious[indexInComponent]);
	}

	if (DEBUG_OUTPUT) std::cout << "finding leftmost" << std::endl;
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
	if (DEBUG_OUTPUT) std::cout << "traversing boundary" << std::endl;
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

	Slice slice = profileToSlice(strandModel, profile, pipesInPrevious, t);

	return std::make_pair<>(slice, pipesInComponent);
}

std::pair< std::vector<Graph>, std::vector<std::vector<size_t> > > computeClusters(const StrandModel& strandModel, const PipeCluster& pipesInPrevious, float t, float maxDist, size_t minStrandCount)
{
	const auto& skeleton = strandModel.m_strandModelSkeleton;
	const auto& pipeGroup = skeleton.m_data.m_strandGroup;

	std::vector<bool> visited(pipesInPrevious.size(), false);
	std::vector<std::vector<size_t> > clusters;
	std::vector<Graph> graphs;

	std::vector<std::vector<size_t> > tooSmallClusters;
	std::vector<Graph> tooSmallGraphs;

	for (std::size_t i = 0; i < pipesInPrevious.size(); i++)
	{
		if (visited[i])
		{
			continue;
		}

		auto graphAndCluster = computeCluster(strandModel, pipesInPrevious, i, visited, t, maxDist);

		if (graphAndCluster.second.size() >= minStrandCount)
		{
			graphs.push_back(graphAndCluster.first);
			clusters.push_back(graphAndCluster.second);
		}
		else if (graphAndCluster.second.size() != 0)
		{
			tooSmallGraphs.push_back(graphAndCluster.first);
			tooSmallClusters.push_back(graphAndCluster.second);
		}

	}

	// fix clusters that are too small

	/*if (tooSmallClusters.size() > 1)
	{
		// try to merge with each other by using skeleton nodes

		std::cerr << "Not implemented yet: More than one cluster is too small" << std::endl;
		for (size_t i = 0; i < tooSmallClusters.size(); i++)
		{
			for (size_t j = 0; j < tooSmallClusters[i].size(); j++)
			{
				for (size_t k = i; k < tooSmallClusters.size(); k++)
				{
					for (size_t l = j + 1; l < tooSmallClusters[k].size(); l++)
					{
						// TODO
					}
				}
			}
		}
	}

	// merge with the other clusters
	for (auto& smallCluster : tooSmallClusters)
	{
		std::cerr << "Trying to merge cluster of size " << smallCluster.size() << std::endl;;

		for (size_t smallIndex : smallCluster)
		{
			NodeHandle nh0 = getNodeHandle(pipeGroup, pipesInPrevious[smallIndex], t);

			std::vector<std::pair<size_t, Graph::Edge> > mergeIntoClusters;

			// search for Node handle
			for (size_t i = 0; i < clusters.size(); i++)
			{
				for (size_t index : clusters[i])
				{
					NodeHandle nh1 = getNodeHandle(pipeGroup, pipesInPrevious[index], t);

					if (nh0 == nh1)
					{
						mergeIntoClusters.push_back(std::pair<size_t, Graph::Edge>(i, Graph::Edge{index, smallIndex}));

						//graphs[i].addEdge(index, smallIndex);
					}
				}
			}

			if (mergeIntoClusters.size() == 0)
			{
				std::cerr << "could not merge small cluster!" << std::endl;
			}
			else if (mergeIntoClusters.size() == 1)
			{
				std::cerr << "could not merge small cluster!" << std::endl;
			}
			else if (mergeIntoClusters.size() == 2)
			{
				if (mergeIntoClusters[0].first == mergeIntoClusters[1].first)
				{
					clusters[mergeIntoClusters[0].first].push_back(smallIndex);
					graphs[mergeIntoClusters[0].first].addEdge(mergeIntoClusters[0].second.m_source, mergeIntoClusters[0].second.m_target);
					graphs[mergeIntoClusters[1].first].addEdge(mergeIntoClusters[1].second.m_source, mergeIntoClusters[1].second.m_target);
					std::cerr << "Merged into one cluster, this is good!" << std::endl;
				}
				else
				{
					std::cerr << "Merged into different clusters, this will not work!" << std::endl;
				}
			}
		}
	}*/

	/*if (clusters.size() > 1)
	{
		clusters.clear();
		graphs.clear();
		visited = std::vector<bool>(pipesInPrevious.size(), false);

		for (std::size_t i = 0; i < pipesInPrevious.size(); i++)
		{
			if (visited[i])
			{
				//std::cout << "skipping already visited pipe no. " << i << " with handle " << pipesInPrevious[i] << std::endl;
				continue;
			}

			auto graphAndCluster = computeCluster(strandModel, pipesInPrevious, i, visited, t, maxDist);

			graphs.push_back(graphAndCluster.first);
			clusters.push_back(graphAndCluster.second);

		}

		std::cerr << "Partitioning into " << clusters.size() << " clusters at t = " << t << std::endl;

		for (size_t i = 0; i < clusters.size(); i++)
		{
			std::cerr << "cluster " << i << std::endl;

			for (size_t j : clusters[i])
			{
				StrandHandle pipeHandle = pipesInPrevious[j];


				std::cerr << "Strand no. " << j << " with handle " << pipeHandle << " belonging to node with handle " << pipeSegment.m_data.m_nodeHandle << std::endl;
			}

			outputGraph(graphs[i], "t_" + std::to_string(t) + "_cluster_" + std::to_string(i), pipesInPrevious);
		}
	}*/

	return std::pair< std::vector<Graph>, std::vector<std::vector<size_t> > >(graphs, clusters);
}

std::vector<std::pair<Slice, PipeCluster> > computeSlices(const StrandModel& strandModel, const PipeCluster& pipesInPrevious, float t, float maxDist, size_t minStrandCount)
{

	// compute clusters
	auto graphsAndClusters = computeClusters(strandModel, pipesInPrevious, t, maxDist, minStrandCount);

	// then loop over the clusters to compute slices
	std::vector<std::pair<Slice, PipeCluster> > slices;


	for (std::size_t i = 0; i < graphsAndClusters.first.size(); i++)
	{
		// if not visited, determine connected component around this
		//std::cout << "computing slice containing pipe no. " << i << " with handle " << pipesInPrevious[i] << " at t = " << t << std::endl;
		auto slice = computeSlice(strandModel, pipesInPrevious, graphsAndClusters.first[i], graphsAndClusters.second[i], t, maxDist);
		slices.push_back(slice);
	}

	return slices;
}

void forEachSegment(const StrandModelStrandGroup& pipes, std::vector<StrandSegmentHandle>& segGroup, std::function<void(const StrandSegment<StrandModelStrandSegmentData>&)> func)
{
	for (auto& segHandle : segGroup)
	{
		auto& seg = pipes.PeekStrandSegment(segHandle);
		func(seg);
	}
}

void connect(std::vector<std::pair<StrandHandle, glm::vec3> >& slice0, size_t i0, size_t j0, size_t offset0,
	std::vector<std::pair<StrandHandle, glm::vec3> >& slice1, size_t i1, size_t j1, size_t offset1,
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

			k0++;
		}
		else
		{
			indices.push_back(offset1 + (i1 + k1) % slice1.size());
			indices.push_back(offset1 + (i1 + k1 + 1) % slice1.size());
			indices.push_back(offset0 + (i0 + k0) % slice0.size());

			k1++;
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

void connectSlices(const StrandModelStrandGroup& pipes, Slice& bottomSlice, size_t bottomOffset,
	std::vector<Slice>& topSlices, std::vector<size_t> topOffsets,
	std::vector<Vertex>& vertices, std::vector<unsigned>& indices, bool branchConnections)
{
	// compute (incomplete) permutation that turns 0 into 1
	std::vector<std::pair<size_t, size_t> > topPipeHandleIndexMap(pipes.PeekStrands().size(), std::make_pair<>(-1, -1));
	std::vector<size_t> bottomPipeHandleIndexMap(pipes.PeekStrands().size(), -1);

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

void createTwigTip(const StrandModel& strandModel, std::pair < Slice, PipeCluster>& prevSlice, size_t prevOffset, float t,
	std::vector<Vertex>& vertices, std::vector<unsigned>& indices)
{
	// compute average positions of all slice points
	glm::vec3 pos(0, 0, 0);

	for (auto& el : prevSlice.second)
	{
		if (!isValidPipeParam(strandModel, el, t))
		{
			t -= 0.01;
		}

		pos += getPipePos(strandModel, el, t);
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

void sliceRecursively(const StrandModel& strandModel, std::pair < Slice, PipeCluster>& prevSlice, size_t prevOffset, float t, float stepSize, float maxDist,
	std::vector<Vertex>& vertices, std::vector<unsigned>& indices, const StrandModelMeshGeneratorSettings& settings, float accumulatedAngle = 0.0f)
{
	const auto& skeleton = strandModel.m_strandModelSkeleton;
	const auto& pipeGroup = skeleton.m_data.m_strandGroup;

	// prevent problems with floating point arithmetics
	if (t + 0.01 > glm::ceil(t))
	{
		t = glm::ceil(t);
	}

	auto slicesAndClusters = computeSlices(strandModel, prevSlice.second, t, maxDist, settings.m_minCellCountForMajorBranches);
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
		if (DEBUG_OUTPUT) std::cout << "=== Ending recursion at t = " << t << " ===" << std::endl;

		//createTwigTip(strandModel, prevSlice, prevOffset, t - stepSize, vertices, indices);

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
			v.m_texCoord.y = t * settings.m_vMultiplier;
			v.m_texCoord.x = getPipePolar(strandModel, el.first, t) / (2 * glm::pi<float>()) * settings.m_uMultiplier + accumulatedAngle / 360.0f;

			// add twisting to uv-Coordinates
			auto nodeHandle = getNodeHandle(pipeGroup, el.first, t);
			const auto& node = skeleton.PeekNode(nodeHandle);

			float frac = fmod(t, 1.0);

			v.m_texCoord.x += frac * node.m_data.m_twistAngle / 360.0f;

			vertices.push_back(v);
		}
	}

	connectSlices(pipeGroup, prevSlice.first, prevOffset, topSlices, offsets, vertices, indices, settings.m_branchConnections);

	if (DEBUG_OUTPUT) std::cout << "--- Done with slice at t = " << t << " ---" << std::endl;
	// recursive call
	t += stepSize;
	for (size_t i = 0; i < slicesAndClusters.size(); i++)
	{
		if (slicesAndClusters[i].first.size() != 0)
		{
			if (DEBUG_OUTPUT) std::cout << "___ Slice at t = " << t << " ___" << std::endl;

			float newAccumulatedAngle = accumulatedAngle;

			if (std::floor(t - stepSize) < std::floor(t))
			{
				// need to compute new accumulated angle
				auto nodeHandle = getNodeHandle(pipeGroup, slicesAndClusters[i].second[0], t);
				const auto& node = skeleton.PeekNode(nodeHandle);
				newAccumulatedAngle += node.m_data.m_twistAngle;
			}

			sliceRecursively(strandModel, slicesAndClusters[i], offsets[i], t, stepSize, maxDist, vertices, indices, settings, newAccumulatedAngle);
		}
	}
}

void StrandModelMeshGenerator::RecursiveSlicing(
	const StrandModel& strandModel, std::vector<Vertex>& vertices,
	std::vector<unsigned>& indices, const StrandModelMeshGeneratorSettings& settings)
{
	const auto& skeleton = strandModel.m_strandModelSkeleton;
	const auto& pipeGroup = skeleton.m_data.m_strandGroup;

	if (pipeGroup.PeekStrands().size() == 0)
	{
		return;
	}

	if (DEBUG_OUTPUT) std::cout << "getting first seg group" << std::endl;
	std::vector<StrandSegmentHandle> segGroup0 = getSegGroup(pipeGroup, 0);

	if (DEBUG_OUTPUT) std::cout << "determining max thickness" << std::endl;
	float maxThickness = 0.0f;
	forEachSegment(pipeGroup, segGroup0,
		[&](const StrandSegment<StrandModelStrandSegmentData>& seg)
		{
			if (seg.m_info.m_thickness > maxThickness)
			{
				maxThickness = seg.m_info.m_thickness;
			}
		}
	);

	float maxDist = 2 * maxThickness * sqrt(2) * 2.5f * settings.m_clusterDistance;
	// initial slice at root:
	std::vector<bool> visited(pipeGroup.PeekStrands().size(), false);

	// prepare initial pipe cluster:
	PipeCluster pipeCluster;

	for (size_t i = 0; i < pipeGroup.PeekStrands().size(); i++)
	{
		pipeCluster.push_back(pipeGroup.PeekStrands()[i].GetHandle());
	}

	float stepSize = 1.0f / settings.m_stepsPerSegment;
	//float max = settings.m_maxParam;

	auto firstCluster = computeCluster(strandModel, pipeCluster, 0, visited, 0.0, maxDist);
	auto firstSlice = computeSlice(strandModel, pipeCluster, firstCluster.first, firstCluster.second, 0.0, maxDist);

	// create initial vertices
	for (auto& el : firstSlice.first)
	{
		Vertex v;
		v.m_position = el.second;
		v.m_texCoord.y = 0.0;
		v.m_texCoord.x = getPipePolar(strandModel, el.first, 0.0) / (2 * glm::pi<float>()) * settings.m_uMultiplier;
		vertices.push_back(v);
	}

	sliceRecursively(strandModel, firstSlice, 0, stepSize, stepSize, maxDist, vertices, indices, settings);
}

void StrandModelMeshGenerator::RecursiveSlicing(const StrandModel& strandModel, std::vector<Vertex>& vertices,
	std::vector<glm::vec2>& texCoords, std::vector<std::pair<unsigned, unsigned>>& indices,
	const StrandModelMeshGeneratorSettings& settings)
{
}

void StrandModelMeshGenerator::MarchingCube(const StrandModel& strandModel, std::vector<Vertex>& vertices,
                                            std::vector<unsigned>& indices, const StrandModelMeshGeneratorSettings& settings)
{
	const auto& skeleton = strandModel.m_strandModelSkeleton;
	const auto& pipeGroup = skeleton.m_data.m_strandGroup;
	// first compute extreme points
	auto min = glm::vec3(std::numeric_limits<float>::infinity());
	auto max = glm::vec3(-std::numeric_limits<float>::infinity());
	bool needTriangulation = false;
	for (const auto& pipeSegment : pipeGroup.PeekStrandSegments())
	{
		const auto& node = skeleton.PeekNode(pipeSegment.m_data.m_nodeHandle);
		const auto& profile = node.m_data.m_profile;
		if (profile.PeekParticles().size() < settings.m_minCellCountForMajorBranches) continue;
		needTriangulation = true;
		min = glm::min(pipeSegment.m_info.m_globalPosition, min);
		max = glm::max(pipeSegment.m_info.m_globalPosition, max);
	}
	if (needTriangulation) {
		min -= glm::vec3(0.1f);
		max += glm::vec3(0.1f);
		const auto boxSize = max - min;
		Octree<bool> octree;
		if (settings.m_autoLevel)
		{
			const float maxRadius = glm::max(glm::max(boxSize.x, boxSize.y), boxSize.z) * 0.5f + 2.0f * settings.m_marchingCubeRadius;
			int subdivisionLevel = -1;
			float testRadius = settings.m_marchingCubeRadius;
			while (testRadius <= maxRadius)
			{
				subdivisionLevel++;
				testRadius *= 2.f;
			}
			EVOENGINE_LOG("Mesh formation: Auto set level to " + std::to_string(subdivisionLevel))
				octree.Reset(maxRadius, subdivisionLevel, (min + max) * 0.5f);
		}
		else {
			octree.Reset(glm::max((boxSize.x, boxSize.y), glm::max(boxSize.y, boxSize.z)) * 0.5f,
				glm::clamp(settings.m_voxelSubdivisionLevel, 4, 16), (min + max) / 2.0f);
		}
		float subdivisionLength = settings.m_marchingCubeRadius * 0.5f;

		for (const auto& pipeSegment : pipeGroup.PeekStrandSegments())
		{
			const auto& node = skeleton.PeekNode(pipeSegment.m_data.m_nodeHandle);
			const auto& profile = node.m_data.m_profile;
			if (profile.PeekParticles().size() < settings.m_minCellCountForMajorBranches) continue;

			//Get interpolated position on pipe segment. Example to get middle point here:
			const auto startPosition = strandModel.InterpolateStrandSegmentPosition(pipeSegment.GetHandle(), 0.0f);
			const auto endPosition = strandModel.InterpolateStrandSegmentPosition(pipeSegment.GetHandle(), 1.0f);
			const auto distance = glm::distance(startPosition, endPosition);
			const auto stepSize = glm::max(1, static_cast<int>(distance / subdivisionLength));

			const auto polarX = profile.PeekParticle(pipeSegment.m_data.m_profileParticleHandle).GetInitialPolarPosition().y / glm::radians(360.0f);
			for (int step = 0; step < stepSize; step++)
			{
				const auto a = static_cast<float>(step) / stepSize;
				const auto position = strandModel.InterpolateStrandSegmentPosition(pipeSegment.GetHandle(), a);

				octree.Occupy(position, [&](OctreeNode& octreeNode)
					{});
			}
		}
		octree.TriangulateField(vertices, indices, settings.m_removeDuplicate);
		std::unordered_set<unsigned> indicesMap;
		for (int triangleIndex = 0; triangleIndex < indices.size() / 3; triangleIndex++)
		{
			const auto& v1i = indices[triangleIndex * 3];
			const auto& v2i = indices[triangleIndex * 3 + 1];
			const auto& v3i = indices[triangleIndex * 3 + 2];
			const auto& v1 = vertices[v1i];
			const auto& v2 = vertices[v2i];
			const auto& v3 = vertices[v3i];
			if (v1.m_position.y < 0.001f && v2.m_position.y < 0.001f && v3.m_position.y < 0.001f)
			{
				indices[triangleIndex * 3] = indices[indices.size() - 3];
				indices[triangleIndex * 3 + 1] = indices[indices.size() - 2];
				indices[triangleIndex * 3 + 2] = indices[indices.size() - 1];
				indices.pop_back();
				indices.pop_back();
				indices.pop_back();
				triangleIndex--;
			}
		}

	}
	CalculateNormal(vertices, indices);
}

void StrandModelMeshGenerator::CylindricalMeshing(const StrandModel& strandModel, std::vector<Vertex>& vertices,
	std::vector<unsigned>& indices, const StrandModelMeshGeneratorSettings& settings)
{
	const auto& skeleton = strandModel.m_strandModelSkeleton;
	const auto& pipeGroup = skeleton.m_data.m_strandGroup;
	const auto& sortedInternodeList = skeleton.PeekSortedNodeList();
	std::vector<std::vector<RingSegment>> ringsList;
	std::map<NodeHandle, int> steps{};
	ringsList.resize(sortedInternodeList.size());
	std::vector<std::shared_future<void>> results;
	std::vector<std::vector<std::pair<NodeHandle, int>>> tempSteps{};
	tempSteps.resize(Jobs::Workers().Size());
	Jobs::ParallelFor(sortedInternodeList.size(), [&](unsigned internodeIndex, unsigned threadIndex) {
		auto internodeHandle = sortedInternodeList[internodeIndex];
		const auto& internode = skeleton.PeekNode(internodeHandle);
		const auto& internodeInfo = internode.m_info;

		auto& rings = ringsList[internodeIndex];
		rings.clear();
		if (internode.GetParentHandle() == -1) return;
		int particleSize = internode.m_data.m_profile.PeekParticles().size();
		if (particleSize > settings.m_maxCellCountForMinorBranches) return;
		particleSize = glm::min(particleSize, settings.m_maxCellCountForMinorBranches);
		glm::vec3 p[4];
		glm::vec3 f[4];
		float t[4];


		p[1] = internode.m_info.m_globalPosition;
		f[1] = internode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
		t[1] = glm::sqrt(static_cast<float>(particleSize)) * internode.m_data.m_strandRadius;

		p[2] = internode.m_info.GetGlobalEndPosition();
		f[2] = internode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
		t[2] = glm::sqrt(static_cast<float>(particleSize)) * internode.m_data.m_strandRadius;
		if (internode.GetParentHandle() == -1)
		{
			p[0] = p[1] * 2.0f - p[2];
			f[0] = f[1] * 2.0f - f[2];
			t[0] = t[1] * 2.0f - t[2];
		}
		else
		{
			p[0] = skeleton.PeekNode(internode.GetParentHandle()).m_info.m_globalPosition;
			f[0] = skeleton.PeekNode(skeleton.PeekNode(internode.GetParentHandle()).GetParentHandle()).m_info.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
			int prevParticleSize = skeleton.PeekNode(internode.GetParentHandle()).m_data.m_profile.PeekParticles().size();
			t[0] = glm::sqrt(static_cast<float>(glm::min(prevParticleSize, settings.m_maxCellCountForMinorBranches))) * internode.m_data.m_strandRadius;
		}
		if (internode.IsEndNode())
		{
			p[3] = p[2] * 2.0f - p[1];
			f[3] = f[2] * 2.0f - f[1];
			t[3] = t[2] * 2.0f - t[1];
		}
		else
		{
			int maxChildSize = -1;
			NodeHandle maxChildHandle = -1;
			for (const auto& childHandle : internode.RefChildHandles()) {
				const auto& childInternode = skeleton.PeekNode(childHandle);
				const auto childSize = static_cast<float>(childInternode.m_data.m_profile.PeekParticles().size());
				if (childSize > maxChildSize)
				{
					maxChildSize = childSize;
					maxChildHandle = childHandle;
				}
			}
			for (const auto childHandle : internode.RefChildHandles())
			{
				const auto& childInternode = skeleton.PeekNode(childHandle);
				if (childHandle == maxChildHandle)
				{
					p[3] = childInternode.m_info.GetGlobalEndPosition();
					f[3] = childInternode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 0, -1);
					t[3] = glm::sqrt(static_cast<float>(childInternode.m_data.m_profile.PeekParticles().size())) * internode.m_data.m_strandRadius;
				}
			}
		}
#pragma region Subdivision internode here.
		float thicknessStart, thicknessEnd, thicknessStartT, thicknessEndT;
		Strands::CubicInterpolation(t[0], t[1], t[2], t[3], thicknessStart, thicknessStartT, 0.0f);
		Strands::CubicInterpolation(t[0], t[1], t[2], t[3], thicknessEnd, thicknessEndT, 1.0f);
		const auto diameter = glm::max(thicknessStart, thicknessEnd) * 2.0f * glm::pi<float>();
		int step = diameter / settings.m_xSubdivision;
		if (step < 4)
			step = 4;
		if (step % 2 != 0)
			++step;

		tempSteps[threadIndex].emplace_back(internodeHandle, step);
		int amount = glm::max(4.0f, internodeInfo.m_length / settings.m_ySubdivision);
		if (amount % 2 != 0)
			++amount;

		for (int ringIndex = 1; ringIndex <= amount; ringIndex++) {
			const float a = static_cast<float>(ringIndex - 1) / amount;
			const float b = static_cast<float>(ringIndex) / amount;
			glm::vec3 startPosition, endPosition, startAxis, endAxis, tempStart, tempEnd;
			Strands::CubicInterpolation(p[0], p[1], p[2], p[3], startPosition, tempStart, a);
			Strands::CubicInterpolation(p[0], p[1], p[2], p[3], endPosition, tempEnd, b);
			Strands::CubicInterpolation(f[0], f[1], f[2], f[3], startAxis, tempStart, a);
			Strands::CubicInterpolation(f[0], f[1], f[2], f[3], endAxis, tempEnd, b);
			Strands::CubicInterpolation(t[0], t[1], t[2], t[3], thicknessStart, thicknessStartT, a);
			Strands::CubicInterpolation(t[0], t[1], t[2], t[3], thicknessEnd, thicknessEndT, b);
			rings.emplace_back(
				startPosition, endPosition,
				startAxis,
				endAxis,
				thicknessStart, thicknessEnd);
		}
		}, results);
	for (auto& i : results) i.wait();

	for (const auto& list : tempSteps)
	{
		for (const auto& element : list)
		{
			steps[element.first] = element.second;
		}
	}

	std::map<NodeHandle, int> vertexLastRingStartVertexIndex{};
	for (int internodeIndex = 0; internodeIndex < sortedInternodeList.size(); internodeIndex++) {
		auto internodeHandle = sortedInternodeList[internodeIndex];
		const auto& internode = skeleton.PeekNode(internodeHandle);
		if (internode.GetParentHandle() == -1) continue;
		if (internode.m_data.m_profile.PeekParticles().size() > settings.m_maxCellCountForMinorBranches) continue;
		const auto& internodeInfo = internode.m_info;
		auto parentInternodeHandle = internode.GetParentHandle();
		bool continuous = false;
		const glm::vec3 up = internodeInfo.m_regulatedGlobalRotation * glm::vec3(0, 1, 0);
		glm::vec3 parentUp = up;
		if (parentInternodeHandle != -1)
		{
			const auto& parentInternode = skeleton.PeekNode(parentInternodeHandle);
			parentUp = parentInternode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 1, 0);
			if (parentInternode.m_data.m_profile.PeekParticles().size() <= settings.m_maxCellCountForMinorBranches) continuous = true;
		}
		auto& rings = ringsList[internodeIndex];
		if (rings.empty()) {
			continue;
		}
		// For stitching
		const int step = steps[internodeHandle];
		int pStep = step;
		if (!continuous)
		{
			pStep = steps[parentInternodeHandle];
		}
		float angleStep = 360.0f / static_cast<float>(step);
		float pAngleStep = 360.0f / static_cast<float>(pStep);
		int vertexIndex = vertices.size();
		Vertex archetype;
		archetype.m_color = settings.m_cylindricalColor;
		const auto flowHandle = internode.GetFlowHandle();
		archetype.m_vertexInfo1 = internodeHandle + 1;
		archetype.m_vertexInfo2 = flowHandle + 1;
		float textureXStep = 1.0f / pStep * 4.0f;
		if (!continuous) {
			for (int p = 0; p < pStep; p++) {
				auto& ring = rings.at(0);
				auto direction = ring.GetDirection(
					parentUp, pAngleStep * p, true);
				archetype.m_position = ring.m_startPosition + direction * ring.m_startRadius;
				const float x =
					p < pStep / 2 ? p * textureXStep : (pStep - p) * textureXStep;
				archetype.m_texCoord = glm::vec2(x, 0.0f);
				archetype.m_color = internodeInfo.m_color;
				vertices.push_back(archetype);
			}
		}
		std::vector<float> angles;
		angles.resize(step);
		std::vector<float> pAngles;
		pAngles.resize(pStep);

		for (auto p = 0; p < pStep; p++) {
			pAngles[p] = pAngleStep * p;
		}
		for (auto s = 0; s < step; s++) {
			angles[s] = angleStep * s;
		}

		std::vector<unsigned> pTarget;
		std::vector<unsigned> target;
		pTarget.resize(pStep);
		target.resize(step);
		for (int p = 0; p < pStep; p++) {
			// First we allocate nearest vertices for parent.
			auto minAngleDiff = 360.0f;
			for (auto j = 0; j < step; j++) {
				const float diff = glm::abs(pAngles[p] - angles[j]);
				if (diff < minAngleDiff) {
					minAngleDiff = diff;
					pTarget[p] = j;
				}
			}
		}
		for (int s = 0; s < step; s++) {
			// Second we allocate nearest vertices for child
			float minAngleDiff = 360.0f;
			for (int j = 0; j < pStep; j++) {
				const float diff = glm::abs(angles[s] - pAngles[j]);
				if (diff < minAngleDiff) {
					minAngleDiff = diff;
					target[s] = j;
				}
			}
		}

		textureXStep = 1.0f / step * 4.0f;
		int ringSize = rings.size();
		for (auto ringIndex = 0; ringIndex < ringSize; ringIndex++) {
			for (auto s = 0; s < step; s++) {
				auto& ring = rings.at(ringIndex);
				auto direction = ring.GetDirection(
					up, angleStep * s, false);
				archetype.m_position = ring.m_endPosition + direction * ring.m_endRadius;
				const auto x =
					s < (step / 2) ? s * textureXStep : (step - s) * textureXStep;
				const auto y = ringIndex % 2 == 0 ? 1.0f : 0.0f;
				archetype.m_texCoord = glm::vec2(x, y);
				vertices.push_back(archetype);
			}
			if (ringIndex == 0)
			{
				if (continuous) {
					int parentLastRingStartVertexIndex = vertexLastRingStartVertexIndex[parentInternodeHandle];
					for (int p = 0; p < pStep; p++) {
						if (pTarget[p] == pTarget[p == pStep - 1 ? 0 : p + 1]) {
							auto a = parentLastRingStartVertexIndex + p;
							auto b = parentLastRingStartVertexIndex + (p == pStep - 1 ? 0 : p + 1);
							auto c = vertexIndex + pTarget[p];
							if (vertices[a].m_position != vertices[b].m_position
								&& vertices[b].m_position != vertices[c].m_position
								&& vertices[a].m_position != vertices[c].m_position) {
								indices.push_back(a);
								indices.push_back(b);
								indices.push_back(c);
							}
						}
						else {
							auto a = parentLastRingStartVertexIndex + p;
							auto b = parentLastRingStartVertexIndex + (p == pStep - 1 ? 0 : p + 1);
							auto c = vertexIndex + pTarget[p];
							if (vertices[a].m_position != vertices[b].m_position
								&& vertices[b].m_position != vertices[c].m_position
								&& vertices[a].m_position != vertices[c].m_position) {
								indices.push_back(a);
								indices.push_back(b);
								indices.push_back(c);
							}
							a = vertexIndex + pTarget[p == pStep - 1 ? 0 : p + 1];
							b = vertexIndex + pTarget[p];
							c = parentLastRingStartVertexIndex + (p == pStep - 1 ? 0 : p + 1);
							if (vertices[a].m_position != vertices[b].m_position
								&& vertices[b].m_position != vertices[c].m_position
								&& vertices[a].m_position != vertices[c].m_position) {
								indices.push_back(a);
								indices.push_back(b);
								indices.push_back(c);
							}
						}
					}
				}
				else
				{
					for (int p = 0; p < pStep; p++) {
						if (pTarget[p] == pTarget[p == pStep - 1 ? 0 : p + 1]) {
							auto a = vertexIndex + p;
							auto b = vertexIndex + (p == pStep - 1 ? 0 : p + 1);
							auto c = vertexIndex + pStep + pTarget[p];
							if (vertices[a].m_position != vertices[b].m_position
								&& vertices[b].m_position != vertices[c].m_position
								&& vertices[a].m_position != vertices[c].m_position) {
								indices.push_back(a);
								indices.push_back(b);
								indices.push_back(c);
							}
						}
						else {
							auto a = vertexIndex + p;
							auto b = vertexIndex + (p == pStep - 1 ? 0 : p + 1);
							auto c = vertexIndex + pStep + pTarget[p];
							if (vertices[a].m_position != vertices[b].m_position
								&& vertices[b].m_position != vertices[c].m_position
								&& vertices[a].m_position != vertices[c].m_position) {
								indices.push_back(a);
								indices.push_back(b);
								indices.push_back(c);
							}
							a = vertexIndex + pStep + pTarget[p == pStep - 1 ? 0 : p + 1];
							b = vertexIndex + pStep + pTarget[p];
							c = vertexIndex + (p == pStep - 1 ? 0 : p + 1);

							if (vertices[a].m_position != vertices[b].m_position
								&& vertices[b].m_position != vertices[c].m_position
								&& vertices[a].m_position != vertices[c].m_position) {
								indices.push_back(a);
								indices.push_back(b);
								indices.push_back(c);
							}
						}
					}
				}
				if (!continuous) vertexIndex += pStep;
			}
			else {
				for (int s = 0; s < step - 1; s++) {
					// Down triangle
					auto a = vertexIndex + (ringIndex - 1) * step + s;
					auto b = vertexIndex + (ringIndex - 1) * step + s + 1;
					auto c = vertexIndex + ringIndex * step + s;
					if (vertices[a].m_position != vertices[b].m_position
						&& vertices[b].m_position != vertices[c].m_position
						&& vertices[a].m_position != vertices[c].m_position) {
						indices.push_back(a);
						indices.push_back(b);
						indices.push_back(c);
					}


					// Up triangle
					a = vertexIndex + ringIndex * step + s + 1;
					b = vertexIndex + ringIndex * step + s;
					c = vertexIndex + (ringIndex - 1) * step + s + 1;
					if (vertices[a].m_position != vertices[b].m_position
						&& vertices[b].m_position != vertices[c].m_position
						&& vertices[a].m_position != vertices[c].m_position) {
						indices.push_back(a);
						indices.push_back(b);
						indices.push_back(c);
					}
				}
				// Down triangle
				auto a = vertexIndex + (ringIndex - 1) * step + step - 1;
				auto b = vertexIndex + (ringIndex - 1) * step;
				auto c = vertexIndex + ringIndex * step + step - 1;
				if (vertices[a].m_position != vertices[b].m_position
					&& vertices[b].m_position != vertices[c].m_position
					&& vertices[a].m_position != vertices[c].m_position) {
					indices.push_back(a);
					indices.push_back(b);
					indices.push_back(c);
				}
				// Up triangle
				a = vertexIndex + ringIndex * step;
				b = vertexIndex + ringIndex * step + step - 1;
				c = vertexIndex + (ringIndex - 1) * step;
				if (vertices[a].m_position != vertices[b].m_position
					&& vertices[b].m_position != vertices[c].m_position
					&& vertices[a].m_position != vertices[c].m_position) {
					indices.push_back(a);
					indices.push_back(b);
					indices.push_back(c);
				}
			}
		}
		vertexLastRingStartVertexIndex[internodeHandle] = vertices.size() - step;
	}
}

void StrandModelMeshGenerator::MeshSmoothing(std::vector<Vertex>& vertices, std::vector<unsigned>& indices)
{
	std::vector<std::vector<unsigned>> connectivity;
	connectivity.resize(vertices.size());
	for (int i = 0; i < indices.size() / 3; i++)
	{
		auto a = indices[3 * i];
		auto b = indices[3 * i + 1];
		auto c = indices[3 * i + 2];
		//a
		{
			bool found1 = false;
			bool found2 = false;
			for (const auto& index : connectivity.at(a))
			{
				if (b == index) found1 = true;
				if (c == index) found2 = true;
			}
			if (!found1)
			{
				connectivity.at(a).emplace_back(b);
			}
			if (!found2)
			{
				connectivity.at(a).emplace_back(c);
			}
		}
		//b
		{
			bool found1 = false;
			bool found2 = false;
			for (const auto& index : connectivity.at(b))
			{
				if (a == index) found1 = true;
				if (c == index) found2 = true;
			}
			if (!found1)
			{
				connectivity.at(b).emplace_back(a);
			}
			if (!found2)
			{
				connectivity.at(b).emplace_back(c);
			}
		}
		//c
		{
			bool found1 = false;
			bool found2 = false;
			for (const auto& index : connectivity.at(c))
			{
				if (a == index) found1 = true;
				if (b == index) found2 = true;
			}
			if (!found1)
			{
				connectivity.at(c).emplace_back(a);
			}
			if (!found2)
			{
				connectivity.at(c).emplace_back(b);
			}
		}
	}
	std::vector<glm::vec3> newPositions;
	std::vector<glm::vec2> newUvs;
	for (int i = 0; i < vertices.size(); i++)
	{
		auto position = glm::vec3(0.0f);
		auto uv = glm::vec2(0.f);
		for (const auto& index : connectivity.at(i))
		{
			const auto& vertex = vertices.at(index);
			position += vertex.m_position;
			uv += vertex.m_texCoord;
		}
		newPositions.push_back(position / static_cast<float>(connectivity.at(i).size()));
		newUvs.push_back(uv / static_cast<float>(connectivity.at(i).size()));
	}
	for (int i = 0; i < vertices.size(); i++)
	{
		if (vertices[i].m_position.y > 0.001f) vertices[i].m_position = newPositions[i];
		else
		{
			vertices[i].m_position.x = newPositions[i].x;
			vertices[i].m_position.z = newPositions[i].z;
		}
		vertices[i].m_texCoord = newUvs[i];
	}
}

void StrandModelMeshGenerator::CalculateNormal(std::vector<Vertex>& vertices, const std::vector<unsigned>& indices)
{
	auto normalLists = std::vector<std::vector<glm::vec3>>();
	const auto size = vertices.size();
	for (auto i = 0; i < size; i++)
	{
		normalLists.emplace_back();
	}
	for (int i = 0; i < indices.size() / 3; i++)
	{
		const auto i1 = indices.at(i * 3);
		const auto i2 = indices.at(i * 3 + 1);
		const auto i3 = indices.at(i * 3 + 2);
		auto v1 = vertices[i1].m_position;
		auto v2 = vertices[i2].m_position;
		auto v3 = vertices[i3].m_position;
		auto normal = glm::normalize(glm::cross(v1 - v2, v1 - v3));
		normalLists[i1].push_back(normal);
		normalLists[i2].push_back(normal);
		normalLists[i3].push_back(normal);
	}
	for (auto i = 0; i < size; i++)
	{
		auto normal = glm::vec3(0.0f);
		for (const auto j : normalLists[i])
		{
			normal += j;
		}
		vertices[i].m_normal = glm::normalize(normal);
	}
}

glm::vec3 ProjectVec3(const glm::vec3& a, const glm::vec3& dir)
{
	return glm::normalize(dir) * glm::dot(a, dir) / glm::length(dir);
}

void StrandModelMeshGenerator::CalculateUV(const StrandModel& strandModel, std::vector<Vertex>& vertices, const StrandModelMeshGeneratorSettings& settings)
{
	if (settings.m_fastUV)
	{
		const auto& sortedNodeList = strandModel.m_strandModelSkeleton.PeekSortedNodeList();

		Jobs::ParallelFor(vertices.size(), [&](unsigned vertexIndex)
			{
				auto& vertex = vertices.at(vertexIndex);

				float minDistance = FLT_MAX;
				NodeHandle closestNodeHandle = -1;

				for (const auto& nodeHandle : sortedNodeList)
				{
					const auto& node = strandModel.m_strandModelSkeleton.PeekNode(nodeHandle);
					const auto& profile = node.m_data.m_profile;
					if (profile.PeekParticles().size() < settings.m_minCellCountForMajorBranches) continue;
					const auto nodeStart = node.m_info.m_globalPosition;
					const auto nodeEnd = node.m_info.GetGlobalEndPosition();
					const auto closestPoint = glm::closestPointOnLine(vertex.m_position, nodeStart, nodeEnd);
					if (glm::dot(nodeEnd - nodeStart, closestPoint - nodeStart) <= 0.f || glm::dot(nodeStart - nodeEnd, closestPoint - nodeEnd) <= 0.f) continue;
					const auto currentDistance = glm::distance(closestPoint, vertex.m_position) / node.m_info.m_thickness;
					if (currentDistance < minDistance)
					{
						minDistance = currentDistance;
						closestNodeHandle = nodeHandle;
					}
				}
				if (closestNodeHandle != -1)
				{
					const auto closestNode = strandModel.m_strandModelSkeleton.PeekNode(closestNodeHandle);
					const float endPointRootDistance = closestNode.m_info.m_rootDistance;
					const float startPointRootDistance = closestNode.m_info.m_rootDistance - closestNode.m_info.m_length;
					const auto closestPoint = glm::closestPointOnLine(vertex.m_position, closestNode.m_info.m_globalPosition, closestNode.m_info.GetGlobalEndPosition());
					const float distanceToStart = glm::distance(closestPoint, closestNode.m_info.m_globalPosition);
					const float a = closestNode.m_info.m_length == 0 ? 1.f : distanceToStart / closestNode.m_info.m_length;
					const float rootDistance = glm::mix(startPointRootDistance, endPointRootDistance, a);
					vertex.m_texCoord.y = rootDistance * settings.m_rootDistanceMultiplier;
					const auto v = glm::normalize(vertex.m_position - closestPoint);
					const auto up = closestNode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 1, 0);
					const auto left = closestNode.m_info.m_regulatedGlobalRotation * glm::vec3(1, 0, 0);
					const auto projUp = ProjectVec3(v, up);
					const auto projLeft = ProjectVec3(v, left);
					const glm::vec2 position = glm::vec2(glm::length(projLeft) * (glm::dot(projLeft, left) > 0.f ? 1.f : -1.f), glm::length(projUp) * (glm::dot(projUp, up) > 0.f ? 1.f : -1.f));
					const float acosVal = glm::acos(position.x / glm::length(position));
					vertex.m_texCoord.x = acosVal / glm::pi<float>();
				}
				else
				{
					vertex.m_texCoord = glm::vec2(0.0f);
				}
			}
		);
	}
	else {
		const auto& strandGroup = strandModel.m_strandModelSkeleton.m_data.m_strandGroup;
		auto min = glm::vec3(FLT_MAX);
		auto max = glm::vec3(FLT_MIN);
		for (const auto& segment : strandGroup.PeekStrandSegments())
		{
			if (segment.IsRecycled()) continue;
			const auto& node = strandModel.m_strandModelSkeleton.PeekNode(segment.m_data.m_nodeHandle);
			const auto& profile = node.m_data.m_profile;
			if (profile.PeekParticles().size() < settings.m_minCellCountForMajorBranches) continue;
			const auto segmentStart = strandGroup.GetStrandSegmentStart(segment.GetHandle());
			const auto segmentEnd = segment.m_info.m_globalPosition;
			min = glm::min(segmentStart, min);
			min = glm::min(segmentStart, min);
			max = glm::max(segmentEnd, max);
			max = glm::max(segmentEnd, max);
		}
		min -= glm::vec3(0.1f);
		max += glm::vec3(0.1f);
		VoxelGrid<std::vector<StrandSegmentHandle>> boundarySegments;
		boundarySegments.Initialize(0.01f, min, max, {});

		for (const auto& segment : strandGroup.PeekStrandSegments())
		{
			if (segment.IsRecycled()) continue;
			const auto& node = strandModel.m_strandModelSkeleton.PeekNode(segment.m_data.m_nodeHandle);
			const auto& profile = node.m_data.m_profile;
			if (profile.PeekParticles().size() < settings.m_minCellCountForMajorBranches) continue;

			const auto segmentStart = strandGroup.GetStrandSegmentStart(segment.GetHandle());
			const auto segmentEnd = segment.m_info.m_globalPosition;
			boundarySegments.Ref((segmentStart + segmentEnd) * 0.5f).emplace_back(segment.GetHandle());
		}


		Jobs::ParallelFor(vertices.size(), [&](unsigned vertexIndex)
			{
				auto& vertex = vertices.at(vertexIndex);
				float minDistance = FLT_MAX;
				StrandSegmentHandle closestSegmentHandle = -1;
				boundarySegments.ForEach(vertex.m_position, 0.05f, [&](std::vector<StrandSegmentHandle>& segmentHandles)
					{
						for (const auto& segmentHandle : segmentHandles)
						{
							const auto& segment = strandGroup.PeekStrandSegment(segmentHandle);
							const auto segmentStart = strandGroup.GetStrandSegmentStart(segmentHandle);
							const auto segmentEnd = segment.m_info.m_globalPosition;
							const auto closestPoint = glm::closestPointOnLine(vertex.m_position, segmentStart, segmentEnd);
							if (glm::dot(segmentEnd - segmentStart, closestPoint - segmentStart) <= 0.f || glm::dot(segmentStart - segmentEnd, closestPoint - segmentEnd) <= 0.f) continue;
							const auto currentDistance = glm::distance(segmentEnd, vertex.m_position);
							if (currentDistance < minDistance)
							{
								minDistance = currentDistance;
								closestSegmentHandle = segment.GetHandle();
							}
						}
					}
				);
				NodeHandle closestNodeHandle = -1;
				if (closestSegmentHandle != -1)
				{
					const auto segment = strandGroup.PeekStrandSegment(closestSegmentHandle);
					closestNodeHandle = segment.m_data.m_nodeHandle;
				}
				if (closestNodeHandle != -1)
				{
					const auto closestNode = strandModel.m_strandModelSkeleton.PeekNode(closestNodeHandle);
					const float endPointRootDistance = closestNode.m_info.m_rootDistance;
					const float startPointRootDistance = closestNode.m_info.m_rootDistance - closestNode.m_info.m_length;
					const auto closestPoint = glm::closestPointOnLine(vertex.m_position, closestNode.m_info.m_globalPosition, closestNode.m_info.GetGlobalEndPosition());
					const float distanceToStart = glm::distance(closestPoint, closestNode.m_info.m_globalPosition);
					const float a = closestNode.m_info.m_length == 0 ? 1.f : distanceToStart / closestNode.m_info.m_length;
					const float rootDistance = glm::mix(startPointRootDistance, endPointRootDistance, a);
					vertex.m_texCoord.y = rootDistance * settings.m_rootDistanceMultiplier;
					const auto v = glm::normalize(vertex.m_position - closestPoint);
					const auto up = closestNode.m_info.m_regulatedGlobalRotation * glm::vec3(0, 1, 0);
					const auto left = closestNode.m_info.m_regulatedGlobalRotation * glm::vec3(1, 0, 0);
					const auto projUp = ProjectVec3(v, up);
					const auto projLeft = ProjectVec3(v, left);
					const glm::vec2 position = glm::vec2(glm::length(projLeft) * (glm::dot(projLeft, left) > 0.f ? 1.f : -1.f), glm::length(projUp) * (glm::dot(projUp, up) > 0.f ? 1.f : -1.f));
					const float acosVal = glm::acos(position.x / glm::length(position));
					vertex.m_texCoord.x = acosVal / glm::pi<float>();
				}
				else
				{
					vertex.m_texCoord = glm::vec2(0.0f);
				}
			}
		);
	}
}
