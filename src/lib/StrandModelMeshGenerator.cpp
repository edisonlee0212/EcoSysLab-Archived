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
#include <queue>

#include "EcoSysLabLayer.hpp"

#define DEBUG_OUTPUT false

using namespace EcoSysLab;

typedef std::vector<std::pair<StrandHandle, glm::vec3> > Slice;
typedef std::vector<StrandHandle> PipeCluster;

void StrandModelMeshGeneratorSettings::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	ImGui::Combo("Mode", { "Recursive Slicing", "Marching Cube" }, m_generatorType);
	if (m_generatorType == 0 && ImGui::TreeNode("Recursive Slicing settings"))
	{
		ImGui::DragInt("Steps per segment", &m_stepsPerSegment, 1.0f, 1, 99);

		//ImGui::Checkbox("[DEBUG] Limit Profile Iterations", &m_limitProfileIterations);
		//ImGui::DragInt("[DEBUG] Limit", &m_maxProfileIterations);

		ImGui::DragFloat("[DEBUG] MaxParam", &m_maxParam);
		//ImGui::Checkbox("Compute branch joints", &m_branchConnections);
		ImGui::DragInt("uCoord multiplier", &m_uMultiplier, 1, 1);
		ImGui::DragFloat("vCoord multiplier", &m_vMultiplier, 0.1f);
		ImGui::DragFloat("cluster distance factor", &m_clusterDistance, 0.1f, 1.0f, 10.0f);
		ImGui::TreePop();
	}

	if (m_generatorType == 1 && ImGui::TreeNode("Marching Cube settings"))
	{
		ImGui::Checkbox("Auto set level", &m_autoLevel);
		if (!m_autoLevel) ImGui::DragInt("Voxel subdivision level", &m_voxelSubdivisionLevel, 1, 5, 16);
		else ImGui::DragFloat("Min Cube size", &m_marchingCubeRadius, 0.0001f, 0.001f, 1.0f);
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
	case StrandModelMeshGeneratorType::MarchingCube:
	{
		MarchingCube(strandModel, vertices, indices, settings);
	}break;
	}
	EVOENGINE_LOG("Mesh formation finished in: " + std::to_string(Times::Now() - meshFormationTime) + "s.");

	if (settings.m_recalculateUV || settings.m_generatorType == static_cast<unsigned>(StrandModelMeshGeneratorType::MarchingCube)) {
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

	CalculateNormal(vertices, indices);
}

void StrandModelMeshGenerator::Generate(const StrandModel& strandModel, std::vector<Vertex>& vertices,
	std::vector<glm::vec2>& texCoords, std::vector<std::pair<unsigned, unsigned>>& indices,
	const StrandModelMeshGeneratorSettings& settings)
{
	const float meshFormationTime = Times::Now();
	RecursiveSlicing(strandModel, vertices, texCoords, indices, settings);
	EVOENGINE_LOG("Mesh formation finished in: " + std::to_string(Times::Now() - meshFormationTime) + "s.");
	const float meshSmoothingTime = Times::Now();
	for (int i = 0; i < settings.m_smoothIteration; i++)
	{
		MeshSmoothing(vertices, indices);
	}
	EVOENGINE_LOG("Mesh smoothing time: " + std::to_string(Times::Now() - meshSmoothingTime) + "s.");
	std::vector<unsigned int> tempIndices{};
	CylindricalMeshing(strandModel, vertices, tempIndices, settings);
	for (const auto& index : tempIndices)
	{
		const auto& vertex = vertices.at(index);
		const auto texCoordsIndex = texCoords.size();
		texCoords.emplace_back(vertex.m_texCoord);
		indices.emplace_back(index, texCoordsIndex);
	}

	CalculateNormal(vertices, indices);
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
	if (DEBUG_OUTPUT) std::cout << "checking indices for " << vertices.size() << " vertices..." << std::endl;
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

	return std::pair< std::vector<Graph>, std::vector<std::vector<size_t> > >(graphs, clusters);
}

std::vector<std::pair<Slice, PipeCluster> > computeSlices(const StrandModel& strandModel, const PipeCluster& pipesInPrevious, float t, float stepSize, float maxDist, size_t minStrandCount)
{
	const auto& skeleton = strandModel.m_strandModelSkeleton;
	const auto& pipeGroup = skeleton.m_data.m_strandGroup;

	// first check if there are any pipes that might be needed for merging
	NodeHandle nh = getNodeHandle(pipeGroup, pipesInPrevious.front(), glm::floor(t - stepSize));

	const auto& node = skeleton.PeekNode(nh);
	
	PipeCluster allPipesWithSameNode;
	for (auto& kv : node.m_data.m_particleMap)
	{
		allPipesWithSameNode.push_back(kv.first);
	}

	if (allPipesWithSameNode.size() != pipesInPrevious.size())
	{
		/*std::cout << "Potential for merge at t = " << t << ". Previous slice had " << pipesInPrevious.size()
			<< " strand, but this one has potentially " << allPipesWithSameNode.size() << std::endl;*/
	}

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

void connect(std::vector<std::pair<StrandHandle, glm::vec3> >& slice0, size_t i0, size_t j0, std::pair<size_t, size_t> offset0,
	std::vector<std::pair<StrandHandle, glm::vec3> >& slice1, size_t i1, size_t j1, std::pair<size_t, size_t> offset1,
	std::vector<Vertex>& vertices, std::vector<glm::vec2>& texCoords, std::vector<std::pair<unsigned, unsigned>>& indices)
{
	if (DEBUG_OUTPUT) std::cout << "connecting " << i0 << ", " << j0 << " to " << i1 << ", " << j1 << std::endl;
	size_t vertBetween0 = (j0 + slice0.size() - i0) % slice0.size();
	size_t vertBetween1 = (j1 + slice1.size() - i1) % slice1.size();
	if (DEBUG_OUTPUT) std::cout << vertBetween0 << " and " << vertBetween1 << " steps, respectively " << std::endl;

	if (vertBetween0 > slice0.size() / 2)
	{
		if (DEBUG_OUTPUT) std::cout << "Warning: too many steps for slice 0, should probably be swapped." << std::endl;
	}

	if (vertBetween1 > slice1.size() / 2)
	{
		if (DEBUG_OUTPUT) std::cout << "Warning: too many steps for slice 1, should probably be swapped." << std::endl;
	}

	// merge the two
	if (DEBUG_OUTPUT) std::cout << "connecting slices with triangles" << std::endl;
	size_t k0 = 0;
	size_t k1 = 0;

	while (k0 < vertBetween0 || k1 < vertBetween1)
	{
		if (k0 / double(vertBetween0) < k1 / double(vertBetween1))
		{
			size_t texIndex0 = offset0.second + (i0 + k0) % slice0.size() + 1; // use end texture coordinate
			size_t texIndex1 = offset0.second + (i0 + k0) % slice0.size();
			size_t texIndex2 = offset1.second + (i1 + k1) % slice1.size();

			if ((i0 + k0) % slice0.size() + 1 == slice0.size()) // use end texture coordinate
			{
				texIndex2 = offset1.second + (i1 + k1 - 1) % slice1.size() + 1;
			}


			// make triangle consisting of k0, k0 + 1 and k1
			// assign new texture coordinates to the third corner if necessary
			glm::vec2 texCoord2 = texCoords[texIndex2];
			float avgX = 0.5 * (texCoords[texIndex0].x + texCoords[texIndex1].x);

			float diff = avgX - texCoord2.x;
			float move = glm::round(diff); // TODO: probably makes more sense to round to a multiple of the uv-coordinate factor

			if (move != 0.0)
			{
				texIndex2 = texCoords.size();
				texCoord2.x += move;
				texCoords.push_back(texCoord2);
			}

			indices.push_back(std::make_pair<>(
				offset0.first + (i0 + k0 + 1) % slice0.size(),
				texIndex0
			));
			indices.push_back(std::make_pair<>(
				offset0.first + (i0 + k0) % slice0.size(),
				texIndex1
			));
			indices.push_back(std::make_pair<>(
				offset1.first + (i1 + k1) % slice1.size(),
				texIndex2
			));

			k0++;
		}
		else
		{
			size_t texIndex0 = offset1.second + (i1 + k1) % slice1.size(); // use end texture coordinate
			size_t texIndex1 = offset1.second + (i1 + k1) % slice1.size() + 1;
			size_t texIndex2 = offset0.second + (i0 + k0) % slice0.size();

			if ((i1 + k1) % slice1.size() + 1 == slice1.size()) // use end texture coordinate
			{
				texIndex2 = offset0.second + (i0 + k0 - 1) % slice0.size() + 1;
			}

			glm::vec2 texCoord2 = texCoords[texIndex2];
			float avgX = 0.5 * (texCoords[texIndex0].x + texCoords[texIndex1].x);

			float diff = avgX - texCoord2.x;
			float move = glm::round(diff); // TODO: probably makes more sense to round to a multiple of the uv-coordinate factor

			if (move != 0.0)
			{
				texIndex2 = texCoords.size();
				texCoord2.x += move;
				texCoords.push_back(texCoord2);
			}

			indices.push_back(std::make_pair<>(
				offset1.first + (i1 + k1) % slice1.size(),
				texIndex0
			));
			indices.push_back(std::make_pair<>(
				offset1.first + (i1 + k1 + 1) % slice1.size(),
				texIndex1 // use end texture coordinate
			));

			indices.push_back(std::make_pair<>(
				offset0.first + (i0 + k0) % slice0.size(),
				texIndex2
			));

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

/* We need to deal with inversions of the strand order on the outside somehow.
 * We have two different situations:
 *
 * Case 1: A split into two branches occurs. In this case we have a well defined linear order for each bottom section that corresponds
 * to only one branch. Here, we can easily identify inversions. (TODO: Unintended interleaving could occur here, i.e. bottom
 * vertices correspond to the branches A and B in order A B A B instead of A A B B)
 *
 * Case 2: No branching, the order here is cyclic. We can resolve this by cutting the cyclic order to obtain a linear order.
 * But we should choose a good cutting point to minimize the amount of inversions in order to preserve twisting.
 * A heuristic to achieve this is to define a family of permutations sigma_i which introduce an offset i. Then identify the
 * permutation that has the most fixed points.
 */
void cyclicOrderUntangle(std::vector<size_t>& permutation)
{
	size_t n = permutation.size();

	std::vector<size_t> offsetHistogram(n, 0);

	for (size_t i = 0; i < n; i++)
	{
		size_t offset = (permutation[i] - i + n) % n;

		offsetHistogram[offset]++;
	}

	size_t indexWithMostFixedPoints = 0;

	for (size_t i = 1; i < n; i++)
	{
		if (offsetHistogram[indexWithMostFixedPoints] > offsetHistogram[i])
		{
			indexWithMostFixedPoints = i;
		}
	}


}

bool connectSlices(const StrandModelStrandGroup& pipes, Slice& bottomSlice, std::pair<unsigned, unsigned> bottomOffset,
	std::vector<Slice>& topSlices, std::vector<std::pair<unsigned, unsigned> > topOffsets,
	std::vector<Vertex>& vertices, std::vector<glm::vec2>& texCoords, std::vector<std::pair<unsigned, unsigned>>& indices, bool branchConnections)
{
	// we want to track whether we actually produced any geometry
	size_t sizeBefore = indices.size();

	// compute (incomplete) permutation that turns 0 into 1

	// map of pipe handle index to top slice and index in top slice
	std::vector<std::pair<size_t, size_t> > topPipeHandleIndexMap(pipes.PeekStrands().size(), std::make_pair<>(-1, -1));

	// map of pipe handle index to index in bottom slice
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

	// map index in bottom slice to top slice and index in top slice
	std::vector<std::pair<size_t, size_t> > bottomPermutation(bottomSlice.size(), std::make_pair<>(-1, -1));

	// map top slice and index in top slice to index in bottom slice
	std::vector<std::vector<size_t> > topPermutations(topSlices.size());

	for (std::size_t s = 0; s < topSlices.size(); s++)
	{
		topPermutations[s] = std::vector<size_t>(topSlices[s].size(), -1);

		for (size_t i = 0; i < topPermutations[s].size(); i++)
		{
			topPermutations[s][i] = bottomPipeHandleIndexMap[topSlices[s][i].first];
		}
	}

	if (DEBUG_OUTPUT) std::cout << "mapping back to permutation vector..." << std::endl;
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
	if (DEBUG_OUTPUT) std::cout << "Found first index " << prevI << std::endl;
	// need to find a start index where correspondence changes
	// TODO: only need to do this if there is a branching
	size_t startIndex = prevI; // set prevI as default because this will work if there is no branching
	for (size_t i = 0; i < bottomPermutation.size(); i++)
	{
		if (bottomPermutation[i].second == -1)
		{
			continue;
		}

		if (bottomPermutation[prevI].first != bottomPermutation[i].first)
		{
			startIndex = i;
			break;
		}
		prevI = i;
	}
	if (DEBUG_OUTPUT) std::cout << "Found start index " << startIndex << std::endl;

	std::vector<size_t> indicesWithSameBranchCorrespondence;
	size_t endIndex = prevI;
	if (DEBUG_OUTPUT) std::cout << "Found end index " << endIndex << std::endl;

	size_t sectionStart = startIndex;
	//shift this by one, otherwise the last section is not handled
	indicesWithSameBranchCorrespondence.push_back(startIndex);
	for (size_t counter = startIndex + 1; counter != startIndex + bottomPermutation.size() + 1; counter++)
	{
		size_t i = counter % bottomPermutation.size();

		if (bottomPermutation[i].second == -1)
		{
			//if(DEBUG_OUTPUT) std::cout << "No correspondence at index " << i << std::endl;
			continue;
		}

		if (bottomPermutation[prevI].first == bottomPermutation[i].first)
		{
			indicesWithSameBranchCorrespondence.push_back(i);
		}
		else
		{
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

			// TODO: we could do a very simple test if we selected the correct indices
			if (branchConnections)
			{
				connect(bottomSlice, prevI, bottomMid, bottomOffset,
					topSlices[bottomPermutation[prevI].first], bottomPermutation[prevI].second, nextMid, topOffsets[bottomPermutation[prevI].first],
					vertices, texCoords, indices);
				if (DEBUG_OUTPUT) std::cout << "Connected bottom indices " << prevI << " to " << bottomMid << " with " << bottomPermutation[prevI].second << " to "
					<< nextMid << " of top profile no. " << bottomPermutation[prevI].first << std::endl;

				// connect mid indices with triangle
				// TODO: Think about texture coordinates
				indices.push_back(std::make_pair<>(
					bottomOffset.first + bottomMid,
					bottomOffset.second + bottomMid
				));
				indices.push_back(std::make_pair<>(
					topOffsets[bottomPermutation[prevI].first].first + nextMid,
					topOffsets[bottomPermutation[prevI].first].second + nextMid
				));
				indices.push_back(std::make_pair<>(
					topOffsets[bottomPermutation[i].first].first + prevMid,
					topOffsets[bottomPermutation[i].first].second + prevMid
				));

				// TODO: connecting with the same top slice looks better

				connect(bottomSlice, bottomMid, i, bottomOffset,
					topSlices[bottomPermutation[i].first], prevMid, bottomPermutation[i].second, topOffsets[bottomPermutation[i].first],
					vertices, texCoords, indices);

				if (DEBUG_OUTPUT) std::cout << "Connected bottom indices " << bottomMid << " to " << i << " with " << prevMid << " to "
					<< bottomPermutation[i].second << " of top profile no. " << bottomPermutation[i].first << std::endl;
			}
		}

		// TODO: I'm pretty sure the topSlices.size() check is redundant
		if (((bottomPermutation[prevI].first != bottomPermutation[i].first && topSlices.size() > 1) || counter == startIndex + bottomPermutation.size()) && !indicesWithSameBranchCorrespondence.empty())
		{
			std::vector<size_t> topIndices;

			size_t branchIndex = bottomPermutation[indicesWithSameBranchCorrespondence.front()].first;

			for (size_t j = 0; j < indicesWithSameBranchCorrespondence.size(); j++)
			{
				topIndices.push_back(bottomPermutation[indicesWithSameBranchCorrespondence[j]].second);
			}

			// now check for errors and swap until there are no more errors
			// this is essentially bubble sort. We cannot use a conventional sorting algorithm here
			// because there is no global order - the comparison does not satisfy transitivity.
			// However, there is a local order and we hope that the elements are close enough to this that bubble sort works as a heuristic
			bool foundError;
			do
			{
				foundError = false;
				for (size_t j = 1; j < topIndices.size(); j++)
				{
					size_t steps = (topIndices[j] + topSlices[branchIndex].size() - topIndices[j - 1]) % topSlices[branchIndex].size();

					if (steps > (topSlices[branchIndex].size() + 1) / 2)
					{
						foundError = true;
						if (DEBUG_OUTPUT) std::cout << "found error, correcting by swapping " << topIndices[j - 1] << " and "
							<< topIndices[j] << "; steps: " << steps << "; element count: " << topSlices[branchIndex].size() << std::endl;
						size_t tmp = topIndices[j];
						topIndices[j] = topIndices[j - 1];
						topIndices[j - 1] = tmp;
					}
				}

			} while (foundError);

			for (size_t j = 1; j < indicesWithSameBranchCorrespondence.size(); j++)
			{
				size_t prevI = indicesWithSameBranchCorrespondence[j - 1];
				size_t i = indicesWithSameBranchCorrespondence[j];

				connect(bottomSlice, prevI, i, bottomOffset,
					topSlices[branchIndex], topIndices[j - 1], topIndices[j], topOffsets[branchIndex],
					vertices, texCoords, indices);
			}

			indicesWithSameBranchCorrespondence.clear();
			indicesWithSameBranchCorrespondence.push_back(i);
		}

		prevI = i;
	}

	return sizeBefore != indices.size();
}

void createTwigTip(const StrandModel& strandModel, std::pair < Slice, PipeCluster>& prevSlice, std::pair<unsigned, unsigned> prevOffset, float t,
	std::vector<Vertex>& vertices, std::vector<glm::vec2>& texCoords, std::vector<std::pair<unsigned, unsigned>>& indices)
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

	size_t tipVertIndex = vertices.size();
	vertices.push_back(v);

	size_t tipTexIndex = texCoords.size();
	texCoords.push_back(glm::vec2(t, 0.0));

	for (size_t i = 0; i < prevSlice.second.size(); i++)
	{
		indices.push_back(std::make_pair<>(
			prevOffset.first + i,
			prevOffset.second + i
		));
		indices.push_back(std::make_pair<>(
			tipVertIndex,
			tipTexIndex
		));
		indices.push_back(std::make_pair<>(
			prevOffset.first + (i + 1) % prevSlice.second.size(),
			prevOffset.second + i + 1
		));
	}
}

struct SlicingData
{
	std::pair < Slice, PipeCluster> slice;
	size_t offsetVert;
	size_t offsetTex;
	float t;
	float accumulatedAngle;
};

std::vector<SlicingData> slice(const StrandModel& strandModel, std::pair < Slice, PipeCluster>& prevSlice, std::pair<unsigned, unsigned> prevOffset, float t, float stepSize, float maxDist,
	std::vector<Vertex>& vertices, std::vector<glm::vec2>& texCoords, std::vector<std::pair<unsigned, unsigned>>& indices, const StrandModelMeshGeneratorSettings& settings, float accumulatedAngle = 0.0f)
{
	const auto& skeleton = strandModel.m_strandModelSkeleton;
	const auto& pipeGroup = skeleton.m_data.m_strandGroup;

	// prevent problems with floating point arithmetics
	if (t + 0.01 > glm::ceil(t))
	{
		t = glm::ceil(t);
	}

	if (t > settings.m_maxParam)
	{
		return {};
	}

	auto slicesAndClusters = computeSlices(strandModel, prevSlice.second, t, stepSize, maxDist, settings.m_minCellCountForMajorBranches);
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
		if (DEBUG_OUTPUT) std::cout << "=== Ending branch at t = " << t << " ===" << std::endl;

		//createTwigTip(strandModel, prevSlice, prevOffset, t - stepSize, vertices, indices);

		return {};
	}


	std::vector<std::pair<unsigned, unsigned> > offsets;

	// create vertices
	for (Slice& s : topSlices)
	{
		offsets.push_back(std::make_pair<>(vertices.size(), texCoords.size()));

		bool isFirst = true;

		for (auto& el : s)
		{
			Vertex v;
			v.m_position = el.second;

			glm::vec2 texCoord;
			texCoord.y = t * settings.m_vMultiplier;
			texCoord.x = getPipePolar(strandModel, el.first, t) / (2 * glm::pi<float>()) * settings.m_uMultiplier + accumulatedAngle / 360.0f;

			// add twisting to uv-Coordinates
			auto nodeHandle = getNodeHandle(pipeGroup, el.first, t);
			const auto& node = skeleton.PeekNode(nodeHandle);

			float frac = fmod(t, 1.0);

			texCoord.x += frac * node.m_data.m_twistAngle / 360.0f;

			// need to do proper wraparound
			if (!isFirst && texCoord.x < texCoords.back().x)
			{
				texCoord.x += settings.m_uMultiplier;
			}

			v.m_texCoord = texCoord; // legacy support
			vertices.push_back(v);
			texCoords.push_back(texCoord);

			isFirst = false;
		}

		// texCoord for final vertex
		glm::vec2 texCoord = texCoords[offsets.back().second];

		if (texCoord.x < texCoords.back().x)
		{
			texCoord.x += settings.m_uMultiplier;
		}

		texCoords.push_back(texCoord);
	}

	bool connected = connectSlices(pipeGroup, prevSlice.first, prevOffset, topSlices, offsets,
		vertices, texCoords, indices, settings.m_branchConnections);

	if (!connected)
	{
		std::cerr << "Error: did not connect the slices at t = " << t << " ---" << std::endl;
	}

	if (DEBUG_OUTPUT) std::cout << "--- Done with slice at t = " << t << " ---" << std::endl;
	// accumulate next slices
	t += stepSize;
	std::vector<SlicingData> nextSlices;

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

			nextSlices.push_back(SlicingData{ slicesAndClusters[i], offsets[i].first, offsets[i].second, t, newAccumulatedAngle });
		}
	}

	return nextSlices;
}

void sliceIteratively(const StrandModel& strandModel, std::vector<SlicingData>& startSlices, float stepSize, float maxDist,
	std::vector<Vertex>& vertices, std::vector<glm::vec2>& texCoords, std::vector<std::pair<unsigned, unsigned>>& indices, const StrandModelMeshGeneratorSettings& settings)
{
	std::queue<SlicingData> queue;

	for (SlicingData& s : startSlices)
	{
		queue.push(s);
	}

	float accumulatedAngle = 0.0f;

	while (!queue.empty())
	{
		SlicingData cur = queue.front();
		queue.pop();

		if (DEBUG_OUTPUT) std::cout << "Took next slice with t = " << cur.t << " out of the queue" << std::endl;

		std::vector<SlicingData> slices = slice(strandModel, cur.slice, std::make_pair<>(cur.offsetVert, cur.offsetTex), cur.t,
			stepSize, maxDist, vertices, texCoords, indices, settings, accumulatedAngle);

		for (SlicingData& s : slices)
		{
			queue.push(s);
		}
	}
}

void StrandModelMeshGenerator::RecursiveSlicing(
	const StrandModel& strandModel, std::vector<Vertex>& vertices,
	std::vector<unsigned>& indices, const StrandModelMeshGeneratorSettings& settings)
{
	// support mesh generation in framework
	std::vector<glm::vec2> dummyTexCoords;
	std::vector<std::pair<unsigned, unsigned>> indexPairs;

	RecursiveSlicing(strandModel, vertices, dummyTexCoords, indexPairs, settings);

	for (auto& pair : indexPairs)
	{
		indices.push_back(pair.first);
	}
}

void StrandModelMeshGenerator::RecursiveSlicing(const StrandModel& strandModel, std::vector<Vertex>& vertices,
	std::vector<glm::vec2>& texCoords, std::vector<std::pair<unsigned, unsigned>>& indices,
	const StrandModelMeshGeneratorSettings& settings)
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

	//auto firstCluster = computeCluster(strandModel, pipeCluster, 0, visited, 0.0, maxDist);
	//auto firstSlice = computeSlice(strandModel, pipeCluster, firstCluster.first, firstCluster.second, 0.0, maxDist);

	auto firstSlices = computeSlices(strandModel, pipeCluster, 0, 0, maxDist, 3.0); // TODO: magic number
	std::vector<SlicingData> startSlices;

	for (auto& slice : firstSlices)
	{
		// create initial vertices
		size_t offsetVert = vertices.size();
		size_t offsetTex = texCoords.size();

		bool isFirst = true;

		for (auto& el : slice.first)
		{
			Vertex v;
			v.m_position = el.second;

			glm::vec2 texCoord;
			texCoord.y = 0.0;
			texCoord.x = getPipePolar(strandModel, el.first, 0.0) / (2 * glm::pi<float>()) * settings.m_uMultiplier;

			v.m_texCoord = texCoord; // legacy support
			vertices.push_back(v);

			if (!isFirst && texCoord.x < texCoords.back().x)
			{
				texCoord.x += settings.m_uMultiplier;
			}

			texCoords.push_back(texCoord);

			isFirst = false;
		}

		// need two different texture coordinates for the first and final vertex
		glm::vec2 texCoord = texCoords[offsetTex];

		if (texCoord.x < texCoords.back().x)
		{
			texCoord.x += settings.m_uMultiplier;
		}

		texCoords.push_back(texCoord);

		startSlices.push_back(SlicingData{ slice, offsetVert, offsetTex, stepSize, 0.0 });
	}

	sliceIteratively(strandModel, startSlices, stepSize, maxDist, vertices, texCoords, indices, settings);
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

	}

}

void StrandModelMeshGenerator::CylindricalMeshing(const StrandModel& strandModel, std::vector<Vertex>& vertices,
	std::vector<unsigned>& indices, const StrandModelMeshGeneratorSettings& settings)
{
	const auto& skeleton = strandModel.m_strandModelSkeleton;
	const auto& sortedInternodeList = skeleton.PeekSortedNodeList();
	std::unordered_set<NodeHandle> nodeHandles;
	for(const auto& nodeHandle : sortedInternodeList)
	{
		const auto& internode = skeleton.PeekNode(nodeHandle);
		const int particleSize = internode.m_data.m_profile.PeekParticles().size();
		if (particleSize > settings.m_maxCellCountForMinorBranches) continue;
		nodeHandles.insert(nodeHandle);
	}
	const auto currentVerticesSize = vertices.size();
	const auto ecoSysLab = Application::GetLayer<EcoSysLabLayer>();
	CylindricalMeshGenerator<StrandModelSkeletonData, StrandModelFlowData, StrandModelNodeData>::GeneratePartially(
		nodeHandles, skeleton, vertices, indices, 
		ecoSysLab->m_meshGeneratorSettings,
		[&](glm::vec3& vertexPosition, const glm::vec3& direction, const float xFactor, const float yFactor)
		{},
		[&](glm::vec2& texCoords, const float xFactor, const float yFactor)
		{}
	);
	for (auto i = currentVerticesSize; i < vertices.size(); i++) {
		vertices.at(i).m_color = glm::vec4(0, 1, 0, 1);
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

void StrandModelMeshGenerator::MeshSmoothing(std::vector<Vertex>& vertices,
	std::vector<std::pair<unsigned, unsigned>>& indices)
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
			for (const auto& index : connectivity.at(a.first))
			{
				if (b.first == index) found1 = true;
				if (c.first == index) found2 = true;
			}
			if (!found1)
			{
				connectivity.at(a.first).emplace_back(b.first);
			}
			if (!found2)
			{
				connectivity.at(a.first).emplace_back(c.first);
			}
		}
		//b
		{
			bool found1 = false;
			bool found2 = false;
			for (const auto& index : connectivity.at(b.first))
			{
				if (a.first == index) found1 = true;
				if (c.first == index) found2 = true;
			}
			if (!found1)
			{
				connectivity.at(b.first).emplace_back(a.first);
			}
			if (!found2)
			{
				connectivity.at(b.first).emplace_back(c.first);
			}
		}
		//c
		{
			bool found1 = false;
			bool found2 = false;
			for (const auto& index : connectivity.at(c.first))
			{
				if (a.first == index) found1 = true;
				if (b.first == index) found2 = true;
			}
			if (!found1)
			{
				connectivity.at(c.first).emplace_back(a.first);
			}
			if (!found2)
			{
				connectivity.at(c.first).emplace_back(b.first);
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

void StrandModelMeshGenerator::CalculateNormal(std::vector<Vertex>& vertices,
	const std::vector<std::pair<unsigned, unsigned>>& indices)
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
		auto v1 = vertices[i1.first].m_position;
		auto v2 = vertices[i2.first].m_position;
		auto v3 = vertices[i3.first].m_position;
		auto normal = glm::normalize(glm::cross(v1 - v2, v1 - v3));
		normalLists[i1.first].push_back(normal);
		normalLists[i2.first].push_back(normal);
		normalLists[i3.first].push_back(normal);
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
