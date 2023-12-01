#include "TreePipeMeshGenerator.hpp"
#include "Curve.hpp"
#include "Octree.hpp"
#include "Jobs.hpp"

using namespace EcoSysLab;

void TreePipeMeshGenerator::Generate(const PipeModelPipeGroup& pipes, std::vector<Vertex>& vertices,
	std::vector<unsigned>& indices, const TreePipeMeshGeneratorSettings& settings)
{
	//TODO: Implementation goes here.
	//See here about using internal marching cube algorithm: https://github.com/edisonlee0212/EcoSysLab/blob/1d52593f157eebe2f7792bb5eea8e8d953b475ac/include/lib/Utilities/TreeMeshGenerator.hpp#L503
	//Prepare mesh by adding vertices and indices. For each vertex, you only need to set position, other fields optional.


}
