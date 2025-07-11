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

#ifndef CGOGN_SKINNING_WEIGHT_INTERPOLATION_H_
#define CGOGN_SKINNING_WEIGHT_INTERPOLATION_H_

#include <array>
#include <vector>

#include <cgogn/geometry/types/vector_traits.h>
#include <cgogn/core/utils/assert.h>

namespace cgogn
{

namespace geometry
{

class SkinningWeightInterpolation
{
public:
	// None of these both preserve partial weights and compensate for weight lost due to the limit of four
	// Attempting to do both while supporting negative weights would require handling a number of edge cases
	enum class NormalizationType
	{
		None, // don't normalize, summed weights might exceed 1
		Sum, // divide by weight sum (result sum: 1)
		AbsSum, // divide by weight absolute sum (result sum: 1 or -1)
		CellCount, // divide by amount of contributing cells
	};

public:

	SkinningWeightInterpolation() = delete;

	static Vec4 normalize(Vec4 values,
			const NormalizationType& normalization = NormalizationType::AbsSum,
			size_t source_cell_count = 0)
	{
		switch (normalization)
		{
		case NormalizationType::None:
			return values;
		case NormalizationType::Sum:
			return values.isZero() ? values : values / values.sum();
		case NormalizationType::AbsSum:
			return values.isZero() ? values : values / std::abs(values.sum());
		case NormalizationType::CellCount:
			cgogn_message_assert(source_cell_count > 0, "Source cell count must be non-zero if interpolation uses it");
			return values / source_cell_count;
		default:
			cgogn_assert_not_reached("Missing normalization type case");
			return values;
		}
	}

	/// @brief Computes the most expressive four-bone set of indices and weights from weight sums.
	/// @param bone_weight_values a vector which's i-th element is the weight for the bone of index i
	/// @param normalization how to normalize the output weights (four bones selected among potentially more)
	/// @param source_cell_count the amount of source cells, required for `CellCount` normalization
	/// @return a pair of the most fitting bone indices (up to four) and their corresponding weights
	static std::pair<Vec4i, Vec4> compute_weights(
			const std::vector<Vec4::Scalar>& bone_weight_values,
			const NormalizationType& normalization = NormalizationType::AbsSum,
			size_t source_cell_count = 0)
	{
		Vec4i indices{ -1, -1, -1, -1 };
		Vec4 values = Vec4::Zero();

		// Find and set four highest weight values, and set corresponding indices
		for (size_t wi = 0; wi < bone_weight_values.size(); ++wi)
		{
			const auto& wv = bone_weight_values[wi];
			if (wv == 0.0) // zero weight, not worth adding
				continue;
			int target_coef = -1; // vector index to write the new weight to
			for (size_t i = 0; i < 4; ++i)
			{
				if (indices[i] == -1) // free space, put here
				{
					target_coef = i;
					break;
				}
				if (wv < values[i]) // higher value, don't put here
					continue;
				if (target_coef == -1 // no target yet, so this is the best fit
						|| values[i] < values[target_coef]) // better fit than previous target
					target_coef = i;
			}
			if (target_coef == -1) // all weights already higher than wv
				continue;
			indices[target_coef] = wi;
			values[target_coef] = wv;
		}

		values = normalize(values, normalization, source_cell_count);
		return std::make_pair(indices, values);
	}

	/// @brief Computes the most expressive four-bone set of indices and weights from those of given cells.
	/// `source_cell_weights` is optional: passing a container of a different size than `source_cells`,
	/// such as an instance of `std::array<float, 0>`, will result in uniform weights of 1.0 being used.
	/// This overload allows for the reuse of a buffer to avoid allocations.
	/// @param m the mesh containing the source cells
	/// @param weight_index an attribute on `m` describing each cell's influencing bone indices
	/// @param weight_index an attribute on `m` describing each cell's corresponding weights
	/// @param weight_value_buffer a vector to pass between calls to avoid reallocations (no pre-clearing needed)
	/// @param source_cells an indexable container of cells to interpolate the skinning weights of
	/// @param source_cell_weights an indexable container of weights to multiply their corresponding cell's weights by
	/// @param normalization how to normalize the output weights (four bones selected among potentially more)
	/// @return a pair of the most fitting bone indices (up to four) and their corresponding weights
	template <typename MESH, typename CONT_C, typename CONT_W = std::array<Vec4::Scalar, 0>>
	static std::pair<Vec4i, Vec4> compute_weights(const MESH& m,
			const typename mesh_traits<MESH>::template Attribute<Vec4i>& weight_index,
			const typename mesh_traits<MESH>::template Attribute<Vec4>& weight_value,
			std::vector<Vec4::Scalar>& weight_value_buffer,
			const CONT_C& source_cells,
			const CONT_W& source_cell_weights = {},
			const NormalizationType& normalization = NormalizationType::AbsSum)
	{
		weight_value_buffer.clear();

		if (source_cells.empty())
			return std::make_pair(Vec4i{-1, -1, -1, -1}, Vec4::Zero());

		const bool are_cells_weighted = source_cells.size() == source_cell_weights.size();

		// Sum up all weight contributions per-bone into the buffer
		for (size_t i = 0; i < source_cells.size(); ++i)
		{
			const auto& vi = index_of(m, source_cells[i]);
			const auto vw = are_cells_weighted ? source_cell_weights[i] : 1.0;
			for (size_t j = 0; j < 4; ++j)
			{
				const int wi = weight_index[vi][j];
				const double wv = vw * weight_value[vi][j];
				if (wi < 0 || wv == 0.0)
					continue;
				if (weight_value_buffer.size() <= wi)
					weight_value_buffer.resize(wi + 1, 0.0);
				weight_value_buffer[wi] += wv;
			}
		}

		return compute_weights(weight_value_buffer, normalization, source_cells.size());
	}

	/// @brief Computes the most expressive four-bone set of indices and weights from those of given cells.
	/// `source_cell_weights` is optional: passing a container of a different size than `source_cells`,
	/// such as an instance of `std::array<float, 0>`, will result in uniform weights of 1.0 being used.
	/// If calling this many times repeatedly, consider using the other overload instead
	/// in order to avoid allocations (see its `weight_value_buffer` parameter).
	/// @param m the mesh containing the source cells
	/// @param weight_index an attribute on `m` describing each cell's influencing bone indices
	/// @param weight_index an attribute on `m` describing each cell's corresponding weights
	/// @param weight_value_buffer a vector to pass between calls to avoid reallocations (no pre-clearing needed)
	/// @param source_cells an indexable container of cells to interpolate the skinning weights of
	/// @param source_cell_weights an indexable container of weights to multiply their corresponding cell's weights by
	/// @param normalization how to normalize the output weights (four bones selected among potentially more)
	/// @return a pair of the most fitting bone indices (up to four) and their corresponding weights
	template <typename MESH, typename CONT_C, typename CONT_W = std::array<Vec4::Scalar, 0>>
	static std::pair<Vec4i, Vec4> compute_weights(const MESH& m,
			const typename mesh_traits<MESH>::template Attribute<Vec4i>& weight_index,
			const typename mesh_traits<MESH>::template Attribute<Vec4>& weight_value,
			const CONT_C& source_cells,
			const CONT_W& source_cell_weights = {},
			const NormalizationType& normalization = NormalizationType::AbsSum)
	{
		std::vector<Vec4::Scalar> weight_value_buffer;
		return compute_weights(m, weight_index, weight_value, weight_value_buffer,
				source_cells, source_cell_weights, normalization);
	}

	/// @brief Interpolates between two sets of skinning indices and weights.
	/// The order of influencing bones is NOT guaranteed to be preserved, and zero-weight ones might disappear.
	/// @param indices an array of two source `Vec4i` (reminder: can be inlined as initializer list)
	/// @param values an array of two source `Vec4` (reminder: can be inlined as initializer list)
	/// @param t the interpolation factor: the first couple has full influence for 0.0, the second for 1.0
	/// @param normalization how to normalize the output weights (four bones selected among up to potentially eight)
	/// @return a pair of the most fitting bone indices (up to four) and their corresponding weights
	static std::pair<Vec4i, Vec4> lerp(
			const std::array<Vec4i, 2>& indices,
			const std::array<Vec4, 2>& values,
			const Vec4::Scalar& t,
			const NormalizationType& normalization = NormalizationType::AbsSum)
	{
		constexpr const size_t BufferSize = 8; // enough to handle weights from both inputs
		std::array<Vec4i::Scalar, BufferSize> index_buffer{ -1, -1, -1, -1, -1, -1, -1, -1 };
		std::array<Vec4::Scalar, BufferSize> value_buffer{};

		const Vec4::Scalar w[2] = { 1.0 - t, t };

		// Sum up all weight contributions per-bone into the buffer
		for (size_t i = 0; i < 2; ++i)
		{
			for (size_t j = 0; j < 4; ++j)
			{
				const int wi = indices[i][j];
				const double wv = w[i] * values[i][j];
				if (wi < 0 || wv == 0.0)
					continue;
				for (size_t k = 0; k < BufferSize; ++k)
				{
					auto& index = index_buffer[k];
					if (index != -1 && index != wi)
						continue;
					index = wi;
					value_buffer[k] += wv;
					break;
				}
			}
		}

		Vec4i interpolated_indices{ -1, -1, -1, -1 };
		Vec4 interpolated_values = Vec4::Zero();

		for (size_t i = 0; i < 4; ++i)
		{
			auto j = std::max_element(value_buffer.cbegin(), value_buffer.cend()) - value_buffer.cbegin();
			auto& index = index_buffer[j];
			auto& value = value_buffer[j];
			if (index == -1)
				break;
			interpolated_indices[i] = index;
			interpolated_values[i] = value;
			index = -1;
			value = std::numeric_limits<Vec4::Scalar>::lowest();
		}

		interpolated_values = normalize(interpolated_values, normalization, 2);
		return std::make_pair(interpolated_indices, interpolated_values);
	}
};

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_SKINNING_WEIGHT_INTERPOLATION_H_
