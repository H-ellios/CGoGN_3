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

#ifndef CGOGN_BLENDING_H_
#define CGOGN_BLENDING_H_

#include <cgogn/geometry/types/vector_traits.h>
#include <cgogn/core/types/mesh_views/cell_cache.h>
#include <memory>

namespace cgogn
{

namespace modeling
{

using geometry::Vec3;

    
template <typename MESH>
// Blending function
// Currently it's a sum of vectors of each different attributes that will be blent
void blending(MESH& m, std::vector<std::shared_ptr<typename mesh_traits<MESH>::template Attribute<Vec3>>> attributes_to_blend, std::vector<float> weight_list)
{
    using Vertex = typename mesh_traits<MESH>::Vertex;
    std::shared_ptr<typename mesh_traits<MESH>::template Attribute<Vec3>> vertex_position = cgogn::get_attribute<Vec3, Vertex>(m, "position");
    std::shared_ptr<typename mesh_traits<MESH>::template Attribute<Vec3>> repos_position = cgogn::get_attribute<Vec3, Vertex>(m, "AU00");
    typename mesh_traits<MESH>::template Attribute<Vec3>* new_vertex_pos_value = vertex_position.get();
    Vec3 diff_distance_repos = Vec3(0, 0, 0);
    float epsilon = 0.0001;

    // need to check if parallel_foreach_cell messes with the calculations 
    foreach_cell(m, [&](Vertex v) -> bool {
        value<Vec3>(m, vertex_position, v) = value<Vec3>(m, repos_position, v);
        Vec3 result = Vec3(0, 0, 0);
        float nb_au_influence = 0.;

        for (int i = 0; i < attributes_to_blend.size(); i++)
        {	
            diff_distance_repos = value<Vec3>(m, attributes_to_blend[i], v) - value<Vec3>(m, repos_position, v);
            
            if(abs(diff_distance_repos[0]) > 0. || abs(diff_distance_repos[1]) > 0. || abs(diff_distance_repos[2]) > 0.){
                nb_au_influence++;
            }
            result += diff_distance_repos * weight_list[i];
        }

        if (nb_au_influence != 0.)
            result = result / nb_au_influence;

        result[0] = (abs(result[0]) > epsilon) ? result[0] : 0. ; 
        result[1] = (abs(result[1]) > epsilon) ? result[1] : 0. ;
        result[2] = (abs(result[2]) > epsilon) ? result[2] : 0. ;
        
        value<Vec3>(m, vertex_position, v) += result ;
        return true;
    });
}

} // namespace modeling

} // namespace cgogn

#endif // CGOGN_BLENDING_H_