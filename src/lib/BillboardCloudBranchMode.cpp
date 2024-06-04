#include "BillboardCloud.hpp"
using namespace EvoEngine;


struct ConnectivityGraph
{
	struct V
	{
		std::vector<std::pair<unsigned, float>> m_connectedVertices;
		int m_index = -1;
	};

	std::vector<V> m_vs;

	void EstablishConnectivityGraph(const BillboardCloud::Element& element);
};

void ConnectivityGraph::EstablishConnectivityGraph(const BillboardCloud::Element& element)
{
	m_vs.clear();
	m_vs.resize(element.m_vertices.size());
	for (int vi = 0; vi < m_vs.size(); vi++) m_vs[vi].m_index = vi;

	for (const auto& triangle : element.m_triangles)
	{
		const auto& vertex0 = element.m_vertices[triangle.x];
		const auto& vertex1 = element.m_vertices[triangle.y];
		const auto& vertex2 = element.m_vertices[triangle.z];

		auto& v0 = m_vs[triangle.x];
		auto& v1 = m_vs[triangle.y];
		auto& v2 = m_vs[triangle.z];
		{
			bool findY = false;
			bool findZ = false;
			for (const auto& connectedIndex : v0.m_connectedVertices)
			{
				if (connectedIndex.first == triangle.y) findY = true;
				if (connectedIndex.first == triangle.z) findZ = true;
			}
			if (!findY) v0.m_connectedVertices.emplace_back(triangle.y, glm::distance(vertex0.m_position, vertex1.m_position));
			if (!findZ) v0.m_connectedVertices.emplace_back(triangle.z, glm::distance(vertex0.m_position, vertex2.m_position));
		}
		{
			bool findX = false;
			bool findZ = false;
			for (const auto& connectedIndex : v1.m_connectedVertices)
			{
				if (connectedIndex.first == triangle.x) findX = true;
				if (connectedIndex.first == triangle.z) findZ = true;
			}
			if (!findX) v1.m_connectedVertices.emplace_back(triangle.x, glm::distance(vertex1.m_position, vertex0.m_position));
			if (!findZ) v1.m_connectedVertices.emplace_back(triangle.z, glm::distance(vertex1.m_position, vertex1.m_position));
		}
		{
			bool findX = false;
			bool findY = false;
			for (const auto& connectedIndex : v2.m_connectedVertices)
			{
				if (connectedIndex.first == triangle.x) findX = true;
				if (connectedIndex.first == triangle.y) findY = true;
			}
			if (!findX) v2.m_connectedVertices.emplace_back(triangle.x, glm::distance(vertex2.m_position, vertex0.m_position));
			if (!findY) v2.m_connectedVertices.emplace_back(triangle.y, glm::distance(vertex2.m_position, vertex1.m_position));
		}
	}
}

std::vector<std::vector<unsigned>> BillboardCloud::Element::CalculateLevelSets(int seedVertexIndex)
{
	ConnectivityGraph connectivityGraph;
	connectivityGraph.EstablishConnectivityGraph(*this);

	if (seedVertexIndex == -1) {
		float minHeight = FLT_MAX;
		for (const auto& triangle : m_triangles)
		{
			const auto& vertex0 = m_vertices[triangle.x];
			const auto& vertex1 = m_vertices[triangle.y];
			const auto& vertex2 = m_vertices[triangle.z];

			if (vertex0.m_position.y < minHeight)
			{
				seedVertexIndex = triangle.x;
				minHeight = vertex0.m_position.y;
			}
			if (vertex1.m_position.y < minHeight)
			{
				seedVertexIndex = triangle.y;
				minHeight = vertex0.m_position.y;
			}
			if (vertex2.m_position.y < minHeight)
			{
				seedVertexIndex = triangle.z;
				minHeight = vertex0.m_position.y;
			}
		}
	}
	
	std::vector<std::vector<unsigned>> retVal;

	std::vector<float> dist;
	std::vector<bool> unvisited;
	unvisited.resize(m_vertices.size());
	dist.resize(m_vertices.size());
	Jobs::RunParallelFor(dist.size(), [&](unsigned i)
	{
		unvisited[i] = true;
		dist[i] = FLT_MAX;
	});
	dist[seedVertexIndex] = 0;
	std::vector<unsigned> q;
	q.emplace_back(seedVertexIndex);

	while(!q.empty())
	{
		float minDistance = FLT_MAX;
		int minDistanceIndex = -1;
		for(int testIndex = 0; testIndex < q.size(); testIndex++)
		{
			if(!unvisited[q[testIndex]]) continue;
			const auto testDist = dist[q[testIndex]];
			if(testDist < minDistance)
			{
				minDistanceIndex = testIndex;
				minDistance = testDist;
			}
		}
		unsigned u = q[minDistanceIndex];
		q[minDistanceIndex] = q.back();
		q.pop_back();

		for(const auto& neighbor : connectivityGraph.m_vs[u].m_connectedVertices)
		{
			if(dist[u] + neighbor.second < dist[neighbor.first])
			{
				dist[neighbor.first] = dist[u] + neighbor.second;
			}
			if(unvisited[neighbor.first]) q.emplace_back(neighbor.first);
		}
		unvisited[u] = false;
	}
	float maxDistance = 0.f;
	for(int index = 0; index < m_vertices.size(); index++){
		if(!unvisited[index])
		{
			maxDistance = glm::max(maxDistance, dist[index]);
		}
	}

	for(int index = 0; index < m_vertices.size(); index++)
	{
		if(unvisited[index])
		{
			dist[index] = maxDistance;
		}
	}

	for(int index = 0; index < m_vertices.size(); index++)
	{
		m_vertices[index].m_color = glm::vec4(glm::vec3(dist[index] / maxDistance), 1.f);
	}

	return retVal;
}
