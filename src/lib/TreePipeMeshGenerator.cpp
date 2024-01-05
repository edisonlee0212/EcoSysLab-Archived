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
	glm::vec3 endPosInVoxels = end / voxelSideLength;
	glm::ivec3 startVoxel = glm::floor(posInVoxels);
	retVal.push_back(startVoxel);
	glm::ivec3 endVoxel = glm::floor(end / voxelSideLength);

	// compute t deltas
	glm::vec3 tDelta = glm::vec3(voxelSideLength) / glm::abs(dir);

	// compute max value for t within voxel
	glm::ivec3 nextBorder = roundInDir(posInVoxels, step);
	glm::vec3 distToNextBorder = glm::vec3(nextBorder) - posInVoxels;
	glm::vec3 tMax = glm::abs(distToNextBorder / dirInVoxels);

	std::cout << "startVoxel: " << startVoxel << std::endl;
	std::cout << "endVoxel: " << endVoxel << std::endl;
	std::cout << "startInVoxels: " << posInVoxels << std::endl;
	std::cout << "endPosInVoxels: " << endPosInVoxels << std::endl;
	std::cout << "dirInVoxels: " << dirInVoxels << std::endl;
	std::cout << "nextBorder: " << nextBorder << std::endl;
	std::cout << "distToNextBorder: " << distToNextBorder << std::endl;
	std::cout << "step: " << step << std::endl;
	std::cout << "tDelta: " << tDelta << std::endl;
	std::cout << "tMax: " << tMax << std::endl;

	// now traverse voxels until we reach the end point
	glm::ivec3 voxel = startVoxel;
	while (voxel != endVoxel)
	{
		size_t minIndex = 0;
		float min = tMax[0];

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
		std::cout << "next voxel: " << voxel << std::endl;
		std::cout << "new tMax: " << tMax << std::endl;

	}
	std::cout << "------------------- Done! -------------------" << std::endl;

	return retVal;
}

void TreePipeMeshGenerator::Generate(const PipeModelPipeGroup& pipes, std::vector<Vertex>& vertices,
                                     std::vector<unsigned>& indices, const TreePipeMeshGeneratorSettings& settings)
{
	//TODO: Implementation goes here.
	//See here about using internal marching cube algorithm: /include/lib/Utilities/TreeMeshGenerator.hpp#L503
	//See here about data structure for pipes (strands): /include/lib/Structures/PipeGroup.hpp#L125
	//Prepare mesh by adding vertices and indices. For each vertex, you only need to set position, other fields optional.
	//Indices start from 0.


	// first try some basic geometry
	insertCube(vertices, indices, glm::vec3(0.0f), settings.m_voxelSidelength);
	
	Octree<bool> octree;

	std::vector<glm::ivec3> voxels;
	glm::ivec3 minVoxel;
	glm::ivec3 maxVoxel;

	std::cout << "voxel size: " << settings.m_voxelSidelength << std::endl;
	// loop over each strand in the pipe group
	for (auto& pipe : pipes.PeekPipes())
	{
		glm::vec3 segStart = pipe.m_info.m_baseInfo.m_globalPosition;

		for (auto& segHandle : pipe.PeekPipeSegmentHandles())
		{
			auto& seg = pipes.PeekPipeSegment(segHandle);
			glm::vec3 segEnd = seg.m_info.m_globalPosition;

			// now process the segment
			// for testing simply put a cube here:
			//insertCube(vertices, indices, (segStart + segEnd) / 2.0f, glm::length(segEnd - segStart));

			std::vector<glm::ivec3> voxels =  voxelizeLineSeg(segStart, segEnd, settings.m_voxelSidelength);

			// for now just insert. In the future, we should put them in a voxel grid
			for (auto& voxel : voxels)
			{
				insertCube(vertices, indices, (glm::vec3(voxel) + glm::vec3(0.5f)) * settings.m_voxelSidelength, settings.m_voxelSidelength);
			}

			segStart = segEnd;
		}
		
	}
	
}
