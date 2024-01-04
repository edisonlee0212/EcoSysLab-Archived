#include "TreePipeMeshGenerator.hpp"
#include "Curve.hpp"
#include "Octree.hpp"
#include "Jobs.hpp"

using namespace EcoSysLab;

void TreePipeMeshGeneratorSettings::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer)
{
	//TODO: You can add ImGui settings here.
	ImGui::ColorEdit3("Vertex color", &m_vertexColor.x);
}

void TreePipeMeshGenerator::Generate(const PipeModelPipeGroup& pipes, std::vector<Vertex>& vertices,
                                     std::vector<unsigned>& indices, const TreePipeMeshGeneratorSettings& settings)
{
	//TODO: Implementation goes here.
	//See here about using internal marching cube algorithm: /include/lib/Utilities/TreeMeshGenerator.hpp#L503
	//See here about data structure for pipes (strands): /include/lib/Structures/PipeGroup.hpp#L125
	//Prepare mesh by adding vertices and indices. For each vertex, you only need to set position, other fields optional.
	//Indices start from 0.
	
}
