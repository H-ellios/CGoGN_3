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

#ifndef CGOGN_GEOMETRY_FUNCTIONS_QUATERNION_OPERATIONS_H_
#define CGOGN_GEOMETRY_FUNCTIONS_QUATERNION_OPERATIONS_H_

#include <cgogn/geometry/types/vector_traits.h>

namespace cgogn
{

namespace geometry
{

template <class T>
Eigen::Quaternion<T> operator+(Eigen::Quaternion<T> a, const Eigen::Quaternion<T>& b)
{
	a.vec() += b.vec();
	a.w() += b.w();
	return a;
}

template <class S, class T>
typename std::enable_if_t<std::is_scalar_v<S>, Eigen::Quaternion<T>&>
operator*=(Eigen::Quaternion<T>& q, const S& s)
{
	q.vec() *= s;
	q.w() *= s;
	return q;
}

template <class S, class T>
typename std::enable_if_t<std::is_scalar_v<S>, Eigen::Quaternion<T>>
operator*(Eigen::Quaternion<T> q, const S& s)
{
	return q *= s;
}

template <class S, class T>
typename std::enable_if_t<std::is_scalar_v<S>, Eigen::Quaternion<T>>
operator*(const S& s, Eigen::Quaternion<T> q)
{
	return q *= s;
}

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_FUNCTIONS_QUATERNION_OPERATIONS_H_
