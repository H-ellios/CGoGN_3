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

#ifndef CGOGN_GEOMETRY_TYPES_RIGID_TRANSFORMATION_H_
#define CGOGN_GEOMETRY_TYPES_RIGID_TRANSFORMATION_H_

#include <Eigen/Dense>

namespace cgogn
{

namespace geometry
{

template <typename R = Eigen::Quaterniond, typename T = Eigen::Vector<typename R::Scalar, R::Dim>, int _Dim = R::Dim,
		typename _Scalar = std::enable_if_t<std::is_base_of_v<Eigen::RotationBase<R, _Dim>, R>,
				decltype(std::declval<typename R::Scalar>() * std::declval<typename T::Scalar>())>>
class RigidTransformation
{
public:

	static constexpr const int Dim = _Dim;
	using Scalar = _Scalar;

public:

	RigidTransformation() : r_(R::Identity()), t_(T{Eigen::Vector<typename R::Scalar, _Dim>::Zero()}) {}

	RigidTransformation(const R& rotation, const T& translation = Eigen::Vector<typename R::Scalar, _Dim>::Zero()) :
			r_(rotation), t_(translation) {}

	template <typename OtherScalar, int OtherSize>
	RigidTransformation(const Eigen::Vector<OtherScalar, OtherSize>& translation) :
			RigidTransformation(R::Identity(), translation) {}

	template <typename OtherScalar>
	RigidTransformation(const Eigen::Vector<OtherScalar, 4>& translation) :
			RigidTransformation(R::Identity(), translation.template head<_Dim>() / translation[_Dim]) {}

	template <typename OtherScalar, int OtherDim>
	RigidTransformation(Eigen::Translation<OtherScalar, OtherDim> translation) :
			RigidTransformation(R::Identity(), translation) {}

	template <typename OtherScalar, int OtherDim, int OtherMode, int OtherOptions>
	RigidTransformation(Eigen::Transform<OtherScalar, OtherDim, OtherMode, OtherOptions> transform) :
			RigidTransformation(R{transform.rotation()}, T{transform.translation()}) {}

	template <typename OtherDerived>
	RigidTransformation(const Eigen::MatrixBase<OtherDerived>& other) :
			RigidTransformation(Eigen::Transform<_Scalar, _Dim, Eigen::Isometry, OtherDerived::Options>(other)) {}

	RigidTransformation& operator*=(const RigidTransformation& other)
	{
		const auto& transform = to_transform() * other.to_transform();
		r_ = R{transform.rotation()};
		t_ = T{transform.translation()};
		return *this;
	}

	[[nodiscard]]
	friend RigidTransformation operator*(RigidTransformation a, const RigidTransformation& b)
	{
		return a *= b;
	}

	template <typename S>
	[[nodiscard]]
	static RigidTransformation interpolate(
			const RigidTransformation& a, const RigidTransformation& b, const S& t)
	{
		return {a.r_.slerp(t, b.r_), (1.0 - t) * a.t_ + t * b.t_};
	}

	[[nodiscard]]
	R rotation() const { return r_; }

	[[nodiscard]]
	T translation() const { return t_; }

	[[nodiscard]]
	auto to_transform(bool rotation_affects_translation = false) const
	{
		auto t = Eigen::Translation<_Scalar, _Dim>{t_};
		return rotation_affects_translation ? r_ * t : t * r_;
	}

	[[nodiscard]]
	auto to_transform_matrix(bool rotation_affects_translation = false) const
	{
		return to_transform(rotation_affects_translation).matrix();
	}

	void invert()
	{
		*this = inverse();
	}

	[[nodiscard]]
	RigidTransformation inverse()
	{
		return RigidTransformation{to_transform().inverse()};
	}

private:
	R r_; // rotation
	T t_; // translation
};

// https://en.cppreference.com/w/cpp/language/class_template_argument_deduction#User-defined_deduction_guides

template <typename OtherScalar>
RigidTransformation(const Eigen::Vector<OtherScalar, 2>& translation)
		-> RigidTransformation<Eigen::Rotation2D<OtherScalar>, Eigen::Vector<OtherScalar, 2>>;

template <typename OtherScalar>
RigidTransformation(const Eigen::Vector<OtherScalar, 3>& translation)
		-> RigidTransformation<Eigen::Quaternion<OtherScalar>, Eigen::Vector<OtherScalar, 3>>;

template <typename OtherScalar>
RigidTransformation(const Eigen::Vector<OtherScalar, 4>& translation)
		-> RigidTransformation<Eigen::Quaternion<OtherScalar>, Eigen::Vector<OtherScalar, 3>>;

template <typename OtherScalar>
RigidTransformation(Eigen::Translation<OtherScalar, 2> translation)
		-> RigidTransformation<Eigen::Rotation2D<OtherScalar>, Eigen::Translation<OtherScalar, 2>>;

template <typename OtherScalar>
RigidTransformation(Eigen::Translation<OtherScalar, 3> translation)
		-> RigidTransformation<Eigen::Quaternion<OtherScalar>, Eigen::Translation<OtherScalar, 3>>;

template <typename OtherScalar, int OtherMode, int OtherOptions>
RigidTransformation(Eigen::Transform<OtherScalar, 2, OtherMode, OtherOptions> transform)
		-> RigidTransformation<Eigen::Rotation2D<OtherScalar>, Eigen::Vector<OtherScalar, 2>>;

template <typename OtherScalar, int OtherMode, int OtherOptions>
RigidTransformation(Eigen::Transform<OtherScalar, 3, OtherMode, OtherOptions> transform)
		-> RigidTransformation<Eigen::Quaternion<OtherScalar>, Eigen::Vector<OtherScalar, 3>>;

template <typename OtherDerived>
RigidTransformation(const Eigen::MatrixBase<OtherDerived>& other)
		-> RigidTransformation<Eigen::Quaternion<typename OtherDerived::Scalar>,
				Eigen::Vector<typename OtherDerived::Scalar, 3>>;

using RigidTransformationQVf = RigidTransformation<Eigen::Quaternionf, Eigen::Vector3f>;
using RigidTransformationQVd = RigidTransformation<Eigen::Quaterniond, Eigen::Vector3d>;
using RigidTransformationQV = RigidTransformationQVd;

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_TYPES_RIGID_TRANSFORMATION_H_
