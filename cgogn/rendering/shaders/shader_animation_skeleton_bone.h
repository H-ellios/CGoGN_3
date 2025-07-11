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

#ifndef CGOGN_RENDERING_SHADERS_ANIMATION_SKELETON_BONE_H_
#define CGOGN_RENDERING_SHADERS_ANIMATION_SKELETON_BONE_H_

#include <cgogn/rendering/cgogn_rendering_export.h>
#include <cgogn/rendering/shader_program.h>

namespace cgogn
{

namespace rendering
{

DECLARE_SHADER_CLASS(AnimationSkeletonBone, false, CGOGN_STR(AnimationSkeletonBone))

class CGOGN_RENDERING_EXPORT ShaderParamAnimationSkeletonBone : public ShaderParam
{
	void set_uniforms() override;

public:
	GLColor color_;
	float32 radius_;
	float32 lighted_;

	using ShaderType = ShaderAnimationSkeletonBone;

	ShaderParamAnimationSkeletonBone(ShaderType* sh)
		: ShaderParam(sh, true), color_(1, 1, 0, 1), radius_(1.0f), lighted_(0.0f)
	{
	}

	inline ~ShaderParamAnimationSkeletonBone() override
	{
	}
};

DECLARE_SHADER_CLASS(AnimationSkeletonBoneColorNormal, true, CGOGN_STR(AnimationSkeletonBoneColorNormal))

class CGOGN_RENDERING_EXPORT ShaderParamAnimationSkeletonBoneColorNormal : public ShaderParam
{
	void set_uniforms() override;

	std::array<VBO*, 3> vbos_;
	inline void set_texture_buffer_vbo(uint32 i, VBO* vbo) override
	{
		vbos_[i] = vbo;
	}
	void bind_texture_buffers() override;
	void release_texture_buffers() override;

	enum VBOName : uint32
	{
		JOINT_POSITION = 0,
		BONE_COLOR = 1,
		BONE_NORMAL = 2,
	};

	// Follow 10 and 11, see MeshRender::draw
	static constexpr const int JOINT_POSITION_BIND_ID = 12;
	static constexpr const int BONE_COLOR_BIND_ID = 13;
	static constexpr const int BONE_NORMAL_BIND_ID = 14;

public:
	float32 radius_;
	float32 lighted_;

	using ShaderType = ShaderAnimationSkeletonBoneColorNormal;

	ShaderParamAnimationSkeletonBoneColorNormal(ShaderType* sh)
		: ShaderParam(sh, true), radius_(1.0f), lighted_(0.0f)
	{
	}

	inline ~ShaderParamAnimationSkeletonBoneColorNormal() override
	{
	}
};

} // namespace rendering

} // namespace cgogn

#endif // CGOGN_RENDERING_SHADERS_ANIMATION_SKELETON_BONE_H_
