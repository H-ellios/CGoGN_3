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

#ifndef CGOGN_GEOMETRY_TYPES_DUAL_QUATERNION_H_
#define CGOGN_GEOMETRY_TYPES_DUAL_QUATERNION_H_

#include <cgogn/core/utils/numerics.h>
#include <cgogn/geometry/types/vector_traits.h>
#include <cgogn/geometry/functions/quaternion_operations.h>

namespace cgogn
{

namespace geometry
{

class DualQuaternion
{
public:
	DualQuaternion() : DualQuaternion({0, 0, 0}) {}

	[[nodiscard]]
	static DualQuaternion from_rotation(const Quaternion& r)
	{
		return DualQuaternion{r, {0, 0, 0}};
	}

	[[nodiscard]]
	static DualQuaternion from_translation(const Vec3& t)
	{
		return DualQuaternion{0.5 * t};
	}

	[[nodiscard]]
	static DualQuaternion from_rt(const Quaternion& r, const Vec3& t)
	{
		return DualQuaternion{r, t};
	}

	[[nodiscard]]
	static DualQuaternion from_tr(const Vec3& t, const Quaternion& r)
	{
		return DualQuaternion{t, r};
	}

	[[nodiscard]]
	static DualQuaternion from_point(const Vec3& p)
	{
		return DualQuaternion{p};
	}

	[[nodiscard]]
	static DualQuaternion zero()
	{
		return DualQuaternion{Quaternion{0, 0, 0, 0}, Quaternion{0, 0, 0, 0}};
	}

	[[nodiscard]]
	static DualQuaternion identity()
	{
		return DualQuaternion{{0, 0, 0}};
	}

	[[nodiscard]]
	static DualQuaternion lerp(DualQuaternion a, const DualQuaternion& b,
			const Scalar& s, bool shortest = false)
	{
		if (shortest && a.dot(b) < 0)
			a *= -1.0;

		return a * (1.0 - s) + b * s;
	}

	[[nodiscard]]
	static Scalar dot(const DualQuaternion& a, const DualQuaternion& b)
	{
		return a.dot(b);
	}

	[[nodiscard]]
	Quaternion real() const { return r_; }

	[[nodiscard]]
	Quaternion dual() const { return d_; }

	[[nodiscard]]
	Quaternion rotation() const { return r_; }

	[[nodiscard]]
	Vec3 translation() const { return point() * 2.0; }

	[[nodiscard]]
	Vec3 point() const { return (d_ * r_.conjugate()).vec(); }

	[[nodiscard]]
	Eigen::Isometry3d to_transform() const
	{
		return Eigen::Translation3d{translation()} * rotation();
	}

	[[nodiscard]]
	Mat4 to_transform_matrix() const
	{
		return to_transform().matrix();
	}

	[[nodiscard]]
	DualQuaternion transform(const DualQuaternion& p) const
	{
		return *this * p * conjugated();
	}

	[[nodiscard]]
	Quaternion transform(const Quaternion& r) const
	{
		return transform(from_rotation(r)).rotation();
	}

	[[nodiscard]]
	Vec3 transform(const Vec3& t) const
	{
		return transform(from_point(t)).point();
	}

	void transform_by(const DualQuaternion& q)
	{
		*this = q.transform(*this);
	}

	[[nodiscard]]
	Scalar dot(const DualQuaternion& other) const
	{
		return r_.dot(other.r_);
	}

	[[nodiscard]]
	Scalar squaredMagnitude() const
	{
		return r_.dot(r_);
	}

	[[nodiscard]]
	Scalar magnitude() const
	{
		return std::sqrt(squaredMagnitude());
	}

	void normalize()
	{
		Scalar m = magnitude();

		// Not normalizable
		if (cgogn::almost_equal_relative(m, Scalar(0)))
			return;

		r_ *= 1.0 / m;
		d_ *= 1.0 / m;
	}

	[[nodiscard]]
	DualQuaternion normalized() const
	{
		DualQuaternion res = *this;
		res.normalize();
		return res;
	}

	[[nodiscard]]
	bool isNormalized(const Scalar& prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
	{
		return std::abs(squaredMagnitude() - 1.0) <= prec;
	}

	void conjugate()
	{
		r_ = r_.conjugate();
		d_.w() *= -1.0;
	}

	[[nodiscard]]
	DualQuaternion conjugated() const
	{
		return DualQuaternion(r_.conjugate(), d_.conjugate() * -1.0);
	}

	void invert(bool assume_normalized = false)
	{
		Scalar ism = assume_normalized ? 1.0 : 1.0 / squaredMagnitude();

		r_ = r_.conjugate() * ism;
		d_ = d_.conjugate() * ism;
	}

	[[nodiscard]]
	DualQuaternion inverse(bool assume_normalized = false) const
	{
		DualQuaternion res = *this;
		res.invert(assume_normalized);
		return res;
	}

	bool isApprox(const DualQuaternion& other,
		const Scalar& prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
	{
		return qIsApprox(r_, other.r_, prec) && qIsApprox(d_, other.d_, prec);
	}

	friend DualQuaternion operator+(DualQuaternion a, const DualQuaternion& b)
	{
		return DualQuaternion(a.r_ + b.r_, a.d_ + b.d_);
	}

	friend DualQuaternion operator*(const Scalar& s, const DualQuaternion& dq)
	{
		return DualQuaternion(s * dq.r_, s * dq.d_);
	}

	friend DualQuaternion operator*(const DualQuaternion& dq, const Scalar& s)
	{
		return s * dq;
	}

	friend DualQuaternion operator*(const DualQuaternion& a, const DualQuaternion& b)
	{
		return DualQuaternion(a.r_ * b.r_, a.r_ * b.d_ + a.d_ * b.r_);
	}

	friend std::ostream& operator<<(std::ostream& os, const DualQuaternion& dq)
	{
		os << dq.r_ << " + (" << dq.d_ << ")e";
		return os;
	}

	DualQuaternion& operator+=(const DualQuaternion& other)
	{
		return (*this = *this + other);
	}

	template <class T> // Scalar or DualQuaternion
	DualQuaternion& operator*=(const T& other)
	{
		return (*this = *this * other);
	}

private:
	explicit DualQuaternion(Quaternion r, Quaternion d) : r_(r), d_(d) {}

	explicit DualQuaternion(const Quaternion& r, const Vec3& t) : r_(r.normalized())
	{
		d_ = r * Quaternion{0, 0.5 * t.x(), 0.5 * t.y(), 0.5 * t.z()};
	}

	explicit DualQuaternion(const Vec3& t, const Quaternion& r) : r_(r.normalized())
	{
		d_ = Quaternion{0, 0.5 * t.x(), 0.5 * t.y(), 0.5 * t.z()} * r;
	}

	explicit DualQuaternion(const Vec3& p) : r_({1, 0, 0, 0})
	{
		d_ = Quaternion{0, p.x(), p.y(), p.z()};
	}

	static bool qIsApprox(const Quaternion& a, const Quaternion& b, const Scalar& prec)
	{
		// isApprox fails for quaternions close to zero
		// isMuchSmallerThan somehow does not work, so we use squaredNorm instead
		return a.isApprox(b, prec)
				|| a.coeffs().squaredNorm() <= prec * prec && b.coeffs().squaredNorm() <= prec * prec;
	}

private:
	Quaternion r_; // real part (rotation)
	Quaternion d_; // dual part (translation)
};

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_TYPES_DUAL_QUATERNION_H_
