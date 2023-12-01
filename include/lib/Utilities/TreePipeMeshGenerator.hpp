#pragma once

#include "Vertex.hpp"
#include "PipeModelData.hpp"

using namespace EvoEngine;
namespace EcoSysLab {
	struct TreePipeMeshGeneratorSettings
	{
		//TODO: Add parameters here.
	};
	class TreePipeMeshGenerator
	{
	public:
		static void Generate(const
			PipeModelPipeGroup& pipes, std::vector<Vertex>& vertices,
			std::vector<unsigned int>& indices, const TreePipeMeshGeneratorSettings& settings);
	};
}