#include "TreePipeMeshGenerator.hpp"
#include "Curve.hpp"
#include "Octree.hpp"
#include "Jobs.hpp"
#include <gtx/io.hpp>

using namespace EcoSysLab;

void TreePipeMeshGeneratorSettings::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	//TODO: You can add ImGui settings here.
	ImGui::ColorEdit3("Vertex color", &m_vertexColor.x);
	ImGui::DragFloat("Marching Cube Radius", &m_marchingCubeRadius, 0.0001f, 0.0001f, 2.0f);
}

// use this to visualize voxel before marching cubes for debugging purposes
void insertCube(std::vector<Vertex>& vertices, std::vector<unsigned>& indices, glm::vec3 center, float sidelength)
{
	size_t indexOffset = vertices.size();

	vertices.push_back(Vertex{ center + glm::vec3(1.0, 1.0, 1.0) * sidelength / 2.0f });
	vertices.push_back(Vertex{ center + glm::vec3(-1.0, 1.0, 1.0) * sidelength / 2.0f });
	vertices.push_back(Vertex{ center + glm::vec3(-1.0, -1.0, 1.0) * sidelength / 2.0f });
	vertices.push_back(Vertex{ center + glm::vec3(1.0, -1.0, 1.0) * sidelength / 2.0f });

	vertices.push_back(Vertex{ center + glm::vec3(1.0, -1.0, -1.0) * sidelength / 2.0f });
	vertices.push_back(Vertex{ center + glm::vec3(-1.0, -1.0, -1.0) * sidelength / 2.0f });
	vertices.push_back(Vertex{ center + glm::vec3(-1.0, 1.0, -1.0) * sidelength / 2.0f });
	vertices.push_back(Vertex{ center + glm::vec3(1.0, 1.0, -1.0) * sidelength / 2.0f });

	std::vector<unsigned> cubeIndices;

	cubeIndices.insert(cubeIndices.end(), { 0, 1, 2, 0, 2, 3 }); // front
	cubeIndices.insert(cubeIndices.end(), { 2, 4, 3, 4, 2, 5 }); // bottom
	cubeIndices.insert(cubeIndices.end(), { 4, 5, 6, 4, 6, 7 }); // back
	cubeIndices.insert(cubeIndices.end(), { 0, 6, 1, 6, 0, 7 }); // top
	cubeIndices.insert(cubeIndices.end(), { 0, 3, 4, 0, 4, 7 }); // right
	cubeIndices.insert(cubeIndices.end(), { 1, 2, 5, 1, 5, 6 }); // left

	for (unsigned& index : cubeIndices)
	{
		index += indexOffset;
	}

	indices.insert(indices.end(), cubeIndices.begin(), cubeIndices.end());
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
		retVal.push_back(voxel);

		tMax[minIndex] += tDelta[minIndex];

		// check that we do not run out of range
		for (size_t dim = 0; dim < 3; dim++)
		{
			if (!(minVoxel[dim] <= voxel[dim] && voxel[dim] <= maxVoxel[dim]))
			{
				std::cerr << "voxel out of range! \n";
				std::cerr << "start voxel: " << startVoxel << "\n";
				std::cerr << "end voxel: " << endVoxel << "\n";
				std::cerr << "current voxel: " << voxel << "\n";
				std::cerr << "Aborting..." << std::endl;
				return retVal;
			}
		}
	}

	return retVal;
}

void TreePipeMeshGenerator::Generate(const PipeModelPipeGroup& pipes, std::vector<Vertex>& vertices,
                                     std::vector<unsigned>& indices, const TreePipeMeshGeneratorSettings& settings)
{
	// first compute extreme points
	glm::vec3 min = glm::vec3(std::numeric_limits<float>::infinity());
	glm::vec3 max = glm::vec3(-std::numeric_limits<float>::infinity());

	for (auto& seg : pipes.PeekPipeSegments())
	{
		min = glm::min(seg.m_info.m_globalPosition, min);
		max = glm::max(seg.m_info.m_globalPosition, max);
	}

	for (auto& pipe : pipes.PeekPipes())
	{
		min = glm::min(pipe.m_info.m_baseInfo.m_globalPosition, min);
		max = glm::max(pipe.m_info.m_baseInfo.m_globalPosition, max);
	}

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

	std::cout << "voxel size: " << settings.m_marchingCubeRadius << std::endl;

	// loop over each strand in the pipe group
	for (auto& pipe : pipes.PeekPipes())
	{
		glm::vec3 segStart = pipe.m_info.m_baseInfo.m_globalPosition;

		for (auto& segHandle : pipe.PeekPipeSegmentHandles())
		{
			auto& seg = pipes.PeekPipeSegment(segHandle);
			glm::vec3 segEnd = seg.m_info.m_globalPosition;

			std::vector<glm::ivec3> pipeVoxels =  voxelizeLineSeg(segStart, segEnd, settings.m_marchingCubeRadius);

			// insert each voxel from the segment into the octree
			for (auto& voxel : pipeVoxels)
			{
				octree.Occupy((glm::vec3(voxel) + glm::vec3(0.5f))* settings.m_marchingCubeRadius, [](OctreeNode&) {});
			}

			segStart = segEnd;
		}
		
	}

	octree.TriangulateField(vertices, indices, settings.m_removeDuplicate, settings.m_voxelSmoothIteration);
	
	// finally set vertex colors (does not work yet)
	for (Vertex& v : vertices)
	{
		v.m_color = glm::vec4(settings.m_vertexColor, 1.0f);
	}
}
