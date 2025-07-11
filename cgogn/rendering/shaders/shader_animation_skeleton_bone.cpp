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

#include <cgogn/rendering/shaders/shader_animation_skeleton_bone.h>

namespace cgogn
{

namespace rendering
{

ShaderAnimationSkeletonBone* ShaderAnimationSkeletonBone::instance_ = nullptr;

ShaderAnimationSkeletonBone::ShaderAnimationSkeletonBone()
{
	const char* vertex_shader_source = R"(
		#version 330
		in vec3 vertex_position;
		void main()
		{
			gl_Position =  vec4(vertex_position, 1.0);
		}
	)";

	const char* geometry_shader_source = R"(
		#version 330
		layout (lines) in;
		layout (triangle_strip, max_vertices = 12) out;

		uniform mat4 projection_matrix;
		uniform mat4 model_view_matrix;
		uniform float base_radius;

		out float Nz;

		#define M_PI 3.1415926535897932384626433832795

		void emit_vertex(in vec3 P, in float n)
		{
			Nz = n;
			gl_Position = projection_matrix * model_view_matrix * vec4(P, 1.0);
			EmitVertex();
		}

		void emit_triangle(in vec3 A, in vec3 B, in vec3 C)
		{
			float n = normalize(cross(B - A, C - A)).z;
			emit_vertex(A, n);
			emit_vertex(B, n);
			emit_vertex(C, n);
			EndPrimitive();
		}

		void main()
		{
			vec4 p0 = gl_in[0].gl_Position;
			vec4 p1 = gl_in[1].gl_Position;

			vec3 base_center = p0.xyz / p0.w;
			vec3 tip = p1.xyz / p1.w;

			vec3 X = tip.xyz - base_center.xyz;

			if (dot(X, X) <= 0.0)
				return;

			X = normalize(X);

			vec3 Y = cross(X, vec3(1.0, 0.0, 0.0));

			if (dot(Y, Y) < 1.0)
				Y = cross(X, vec3(0.0, 1.0, 0.0));

			Y = normalize(Y);

			vec3 Z = cross(X, Y);

			float c = cos(2 * M_PI / 3) * base_radius;
			float s = sin(2 * M_PI / 3) * base_radius;
			vec3 base_A = base_center + Y * base_radius;
			vec3 base_B = base_center + Y * c + Z * s;
			vec3 base_C = base_center + Y * c - Z * s;

			emit_triangle(base_A, base_B, base_C);
			emit_triangle(base_A, tip, base_B);
			emit_triangle(base_B, tip, base_C);
			emit_triangle(base_C, tip, base_A);
		}
	)";

	const char* fragment_shader_source = R"(
		#version 330
		uniform vec4 color;
		uniform float lighted;

		in float Nz;

		out vec3 frag_out;

		void main()
		{
			float lambert = max(1.0 - lighted, Nz); // Nz = dot(N,0,0,1)
			frag_out = color.rgb * lambert;
		}
	)";

	load3_bind(vertex_shader_source, fragment_shader_source, geometry_shader_source, "vertex_position");
	get_uniforms("color", "base_radius", "lighted");
}

void ShaderParamAnimationSkeletonBone::set_uniforms()
{
	shader_->set_uniforms_values(color_, radius_, lighted_);
}

ShaderAnimationSkeletonBoneColorNormal* ShaderAnimationSkeletonBoneColorNormal::instance_ = nullptr;

ShaderAnimationSkeletonBoneColorNormal::ShaderAnimationSkeletonBoneColorNormal()
{
	const char* vertex_shader_source = R"(
		#version 330
		uniform usamplerBuffer joint_ind;
		uniform usamplerBuffer bone_ind;
		uniform samplerBuffer joint_position;
		uniform samplerBuffer bone_color;
		uniform samplerBuffer bone_normal;

		out vec3 color_b;
		out vec3 normal_b;

		void main()
		{
			int ind_j = int(texelFetch(joint_ind, 2 * gl_InstanceID + gl_VertexID).r);
			vec3 position_in = texelFetch(joint_position, ind_j).rgb;

			int ind_b = int(texelFetch(bone_ind, int(gl_InstanceID)).r);
			color_b = texelFetch(bone_color, ind_b).rgb;
			normal_b = texelFetch(bone_normal, ind_b).rgb;

			gl_Position = vec4(position_in, 1.0);
		}
	)";

	const char* geometry_shader_source = R"(
		#version 330
		layout (lines) in;
		layout (triangle_strip, max_vertices = 12) out;

		uniform mat4 projection_matrix;
		uniform mat4 model_view_matrix;
		uniform float base_radius;

		in vec3 color_b[];
		in vec3 normal_b[];

		out vec3 color;
		out float Nz;

		#define M_PI 3.1415926535897932384626433832795

		void emit_vertex(in vec3 P, in float n, in vec3 c)
		{
			Nz = n;
			color = c;
			gl_Position = projection_matrix * model_view_matrix * vec4(P, 1.0);
			EmitVertex();
		}

		void emit_triangle(in vec3 A, in vec3 B, in vec3 C)
		{
			float n = normalize(cross(B - A, C - A)).z;
			vec3 c = color_b[0];
			emit_vertex(A, n, c);
			emit_vertex(B, n, c);
			emit_vertex(C, n, c);
			EndPrimitive();
		}

		void main()
		{
			vec4 p0 = gl_in[0].gl_Position;
			vec4 p1 = gl_in[1].gl_Position;

			vec3 base_center = p0.xyz / p0.w;
			vec3 tip = p1.xyz / p1.w;

			vec3 X = tip.xyz - base_center.xyz;

			if (dot(X, X) <= 0.0)
				return;

			X = normalize(X);

			vec3 Y = normalize(cross(X, normal_b[0]));
			vec3 Z = cross(X, Y);

			float c = cos(2 * M_PI / 3) * base_radius;
			float s = sin(2 * M_PI / 3) * base_radius;
			vec3 base_A = base_center + Y * base_radius;
			vec3 base_B = base_center + Y * c + Z * s;
			vec3 base_C = base_center + Y * c - Z * s;

			emit_triangle(base_A, base_B, base_C);
			emit_triangle(base_A, tip, base_B);
			emit_triangle(base_B, tip, base_C);
			emit_triangle(base_C, tip, base_A);
		}
	)";

	const char* fragment_shader_source = R"(
		#version 330
		uniform float lighted;

		in vec3 color;
		in float Nz;

		out vec3 frag_out;

		void main()
		{
			float lambert = max(1.0 - lighted, Nz); // Nz = dot(N,0,0,1)
			frag_out = color.rgb * lambert;
		}
	)";

	load3_bind(vertex_shader_source, fragment_shader_source, geometry_shader_source);
	get_uniforms("joint_ind", "bone_ind", "joint_position", "bone_color", "bone_normal",
			"base_radius", "lighted");
}

void ShaderParamAnimationSkeletonBoneColorNormal::set_uniforms()
{
	// See MeshRender::draw for 10 and 11
	shader_->set_uniforms_values(10, 11, JOINT_POSITION_BIND_ID, BONE_COLOR_BIND_ID, BONE_NORMAL_BIND_ID,
			radius_, lighted_);
}

void ShaderParamAnimationSkeletonBoneColorNormal::bind_texture_buffers()
{
	vbos_[JOINT_POSITION]->bind_texture_buffer(JOINT_POSITION_BIND_ID);
	vbos_[BONE_COLOR]->bind_texture_buffer(BONE_COLOR_BIND_ID);
	vbos_[BONE_NORMAL]->bind_texture_buffer(BONE_NORMAL_BIND_ID);
}

void ShaderParamAnimationSkeletonBoneColorNormal::release_texture_buffers()
{
	vbos_[JOINT_POSITION]->release_texture_buffer(JOINT_POSITION_BIND_ID);
	vbos_[BONE_COLOR]->release_texture_buffer(BONE_COLOR_BIND_ID);
	vbos_[BONE_NORMAL]->release_texture_buffer(BONE_NORMAL_BIND_ID);
}

} // namespace rendering

} // namespace cgogn
