/*******************************************************************************
 * CGoGN: Combinatorial and Geometric modeling with Generic N-dimensional Maps  *
 * Copyright (C), IGG Group, ICube, University of Strasbourg, France            *
 *                                                                              *
 * This library is free software; you can redistribute it and/or modify it      *
 * under the terms of the GNU Lesser General Public License as published by the *
 * Free Software Foundation; either version 2.1 of the License, or (at your     *
 * option) any later version.                                                   *
 *                                                                              *
 * This library is distributed in the hope that it will be useful, but WITHOUT  *
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License  *
 * for more details.                                                            *
 *                                                                              *
 * You should have received a copy of the GNU Lesser General Public License     *
 * along with this library; if not, write to the Free Software Foundation,      *
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.           *
 *                                                                              *
 * Web site: http://cgogn.unistra.fr/                                           *
 * Contact information: cgogn@unistra.fr                                        *
 *                                                                              *
 *******************************************************************************/

#ifndef CGOGN_CORE_FUNCTIONS_TRAVERSALS_VERTEX_H_
#define CGOGN_CORE_FUNCTIONS_TRAVERSALS_VERTEX_H_

#include <algorithm>
#include <vector>

#include <cgogn/core/utils/numerics.h>

namespace cgogn
{

template <typename MESH>
struct mesh_traits;

// some generic functions to gather local neighborhood cells

template <typename MESH, typename CELL>
std::vector<typename mesh_traits<MESH>::Vertex> incident_vertices(const MESH& m, CELL c)
{
	using Vertex = typename mesh_traits<MESH>::Vertex;
	std::vector<Vertex> vertices;
	vertices.reserve(32u);
	foreach_incident_vertex(m, c, [&](Vertex v) -> bool {
		vertices.push_back(v);
		return true;
	});
	return vertices;
}

template <typename MESH, typename CELL>
void append_incident_vertices(const MESH& m, CELL c, std::vector<typename mesh_traits<MESH>::Vertex>& vertices)
{
	using Vertex = typename mesh_traits<MESH>::Vertex;
	foreach_incident_vertex(m, c, [&vertices](Vertex v) -> bool {
		vertices.push_back(v);
		return true;
	});
}

template <size_t Count, typename MESH, typename CELL>
std::array<typename mesh_traits<MESH>::Vertex, Count> first_incident_vertices(const MESH& m, CELL c)
{
    static_assert(Count > 0);
    using Vertex = typename mesh_traits<MESH>::Vertex;
    std::array<Vertex, Count> vertices;
    for (size_t i = 0; i < Count; ++i) // compilers might be better able to optimize this
        vertices[i] = Vertex(INVALID_INDEX);
    size_t i = 0;
    foreach_incident_vertex(m, c, [&](Vertex v) -> bool {
        vertices[i] = v;
        return ++i < Count;
    });
    return vertices;
}

template <typename MESH>
std::vector<typename mesh_traits<MESH>::Vertex> adjacent_vertices_through_edge(const MESH& m,
																			   typename mesh_traits<MESH>::Vertex v)
{
	using Vertex = typename mesh_traits<MESH>::Vertex;
	std::vector<Vertex> vertices;
	vertices.reserve(32u);
	foreach_adjacent_vertex_through_edge(m, v, [&](Vertex av) -> bool {
		vertices.push_back(av);
		return true;
	});
	return vertices;
}

template <typename MESH>
void vertex_k_neighborhood(
        const MESH& m, typename mesh_traits<MESH>::Vertex v, uint32 k,
        std::vector<typename mesh_traits<MESH>::Vertex>& vertices,
        bool include_v = true, bool clear_vertices_at_start = true)
{
    using Vertex = typename mesh_traits<MESH>::Vertex;

    if (clear_vertices_at_start)
        vertices.clear();

    if (include_v)
        vertices.insert(vertices.begin(), v);

    if (k == 0)
        return;

    foreach_adjacent_vertex_through_edge(m, v, [&](Vertex av) -> bool {
        vertices.push_back(av);
        return true;
    });

    size_t k1_ring_begin = include_v ? 1 : 0;
    size_t queue_begin = k1_ring_begin;
    size_t queue_end = vertices.size();

    // Dijkstra-like traversal with maximum depth of k
    // This is faster than DFS, which would have to explore all possible paths to reach the whole k-neighborhood
    for (uint32 d = 2; d <= k && queue_begin < queue_end; ++d)
    {
        for (size_t i = queue_begin; i < queue_end; ++i)
        {
            // This needs to be as fast as possible because it's the bottleneck for volume skinning weight propagation,
            // where it's called in parallel for each vertex of the volume (same reason for reusing the same vector per thread)
            // (GCC/Clang should be able to optimize std::find to this level but MSVC is notoriously bad at inlining functions)
            foreach_adjacent_vertex_through_edge(m, vertices[i], [&](Vertex av) -> bool {
                if (av == v) // v may not have been added to the vector so check explicitly
                    return true; // vertex already in vector
                const auto s = vertices.size();
                for (size_t j = k1_ring_begin; j < s; ++j)
                    if (av == vertices[j])
                        return true; // vertex already in vector
                vertices.push_back(av);
                return true;
            });
        }

        // New queue from all vertices added this loop
        queue_begin = queue_end;
        queue_end = vertices.size();
    }
}

template <typename MESH>
std::vector<typename mesh_traits<MESH>::Vertex> vertex_k_neighborhood(
        const MESH& m, typename mesh_traits<MESH>::Vertex v, uint32 k, bool include_v = true)
{
    std::vector<typename mesh_traits<MESH>::Vertex> vertices;
    vertex_k_neighborhood(m, v, k, vertices, include_v, false);
    return vertices;
}

} // namespace cgogn

#endif // CGOGN_CORE_FUNCTIONS_TRAVERSALS_VERTEX_H_
