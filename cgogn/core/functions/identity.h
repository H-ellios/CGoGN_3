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

#ifndef CGOGN_CORE_FUNCTIONS_IDENTITY_H_
#define CGOGN_CORE_FUNCTIONS_IDENTITY_H_

namespace cgogn
{

template <typename T>
constexpr T&& identity(T&& value) noexcept
{
    return std::forward<T>(value);
}

// It seems perfect forwarding doesn't help in certain template resolution cases,
// so we provide this const version for these purposes
template <typename T>
constexpr const T& identity_c(const T& value) noexcept
{
    return value;
}

} // namespace cgogn

#endif // CGOGN_CORE_FUNCTIONS_IDENTITY_H_
