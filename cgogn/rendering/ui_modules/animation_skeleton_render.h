/*******************************************************************************
 * CGoGN                                                                        *
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

#ifndef CGOGN_MODULE_ANIMATION_SKELETON_RENDER_H_
#define CGOGN_MODULE_ANIMATION_SKELETON_RENDER_H_

#include <cgogn/core/types/animation/animation_skeleton.h>
#include <cgogn/core/ui_modules/mesh_provider.h>
#include <cgogn/ui/app.h>
#include <cgogn/ui/imgui_helpers.h>
#include <cgogn/ui/module.h>
#include <cgogn/ui/view.h>

#include <cgogn/geometry/types/vector_traits.h>

#include <cgogn/rendering/shaders/shader_animation_skeleton_bone.h>
#include <cgogn/rendering/shaders/shader_point_sprite.h>

#include <cgogn/geometry/algos/length.h>

#include <boost/synapse/connect.hpp>

#include <unordered_map>

namespace cgogn
{

namespace ui
{

using geometry::Mat4;
using geometry::Vec3;
using geometry::Scalar;

template <typename ...SupportedTransforms>
class AnimationSkeletonRender : public ViewModule
{
	enum AttributePerCell
	{
		GLOBAL = 0,
		PER_JOINT,
		PER_BONE,
	};
	enum ColorType
	{
		VECTOR = 0
	};

    using MESH = AnimationSkeleton;

	template <typename T>
	using Attribute = AnimationSkeleton::Attribute<T>;

	using AttributeGen = AnimationSkeleton::AttributeGen;

	using Joint = AnimationSkeleton::Joint;
	using Bone = AnimationSkeleton::Bone;

	struct Parameters
	{
		Parameters()
			: joint_position_(nullptr), joint_position_vbo_(nullptr), joint_radius_(nullptr),
			  joint_radius_vbo_(nullptr), joint_color_(nullptr), joint_color_vbo_(nullptr), bone_color_vbo_(nullptr),
			  render_joints_(true), render_bones_(true), bone_normal_vbo_(std::make_unique<rendering::VBO>()),
			  color_per_cell_(GLOBAL), bone_color_per_cell_(GLOBAL), color_type_(VECTOR), joint_scale_factor_(1.0)
		{
			param_point_sprite_ = rendering::ShaderPointSprite::generate_param();
			param_point_sprite_->color_ = {1.0f, 0.5f, 0.0f, 1.0f};

			param_point_sprite_size_ = rendering::ShaderPointSpriteSize::generate_param();
			param_point_sprite_size_->color_ = {1.0f, 0.5f, 0.0f, 1.0f};

			param_point_sprite_color_ = rendering::ShaderPointSpriteColor::generate_param();

			param_point_sprite_color_size_ = rendering::ShaderPointSpriteColorSize::generate_param();

			param_animation_skeleton_bone_ = rendering::ShaderAnimationSkeletonBone::generate_param();
			param_animation_skeleton_bone_->color_ = {1, 0.5, 1, 1};
			param_animation_skeleton_bone_->radius_ = 0.25f;
			param_animation_skeleton_bone_->lighted_ = 0.75f;

			param_animation_skeleton_bone_color_normal_ = rendering::ShaderAnimationSkeletonBoneColorNormal::generate_param();
			param_animation_skeleton_bone_color_normal_->radius_ = 0.25f;
			param_animation_skeleton_bone_color_normal_->lighted_ = 0.75f;
		}

		CGOGN_NOT_COPYABLE_NOR_MOVABLE(Parameters);

		std::shared_ptr<Attribute<Vec3>> joint_position_;
		rendering::VBO* joint_position_vbo_;
		std::shared_ptr<Attribute<Scalar>> joint_radius_;
		rendering::VBO* joint_radius_vbo_;
		std::shared_ptr<Attribute<Vec3>> joint_color_;
		rendering::VBO* joint_color_vbo_;
		std::shared_ptr<Attribute<Vec3>> bone_color_;
		rendering::VBO* bone_color_vbo_;

		// This is computed depending on the transform type on attribute selection so we handle it inside of this class
		std::shared_ptr<AttributeGen> bone_transform_;
		std::unique_ptr<rendering::VBO> bone_normal_vbo_;

		std::unique_ptr<rendering::ShaderPointSprite::Param> param_point_sprite_;
		std::unique_ptr<rendering::ShaderPointSpriteSize::Param> param_point_sprite_size_;
		std::unique_ptr<rendering::ShaderPointSpriteColor::Param> param_point_sprite_color_;
		std::unique_ptr<rendering::ShaderPointSpriteColorSize::Param> param_point_sprite_color_size_;
		std::unique_ptr<rendering::ShaderAnimationSkeletonBone::Param> param_animation_skeleton_bone_;
		std::unique_ptr<rendering::ShaderAnimationSkeletonBoneColorNormal::Param> param_animation_skeleton_bone_color_normal_;

		bool render_joints_;
		bool render_bones_;

		AttributePerCell color_per_cell_;
		AttributePerCell bone_color_per_cell_;
		ColorType color_type_;

		float32 joint_scale_factor_;
		float32 joint_base_size_;
	};

public:
	AnimationSkeletonRender(const App& app)
		: ViewModule(app, "AnimationSkeletonRender (" + std::string{mesh_traits<MESH>::name} + ")"),
		  selected_view_(app.current_view()), selected_mesh_(nullptr)
	{
	}

	~AnimationSkeletonRender()
	{
	}

private:
	template <typename T>
	static Mat4 get_transform_matrix(const T& transform)
	{
		return transform.to_transform_matrix();
	}

	template <typename T>
	static auto get_transform_matrix(const T& transform)
			-> std::enable_if_t<std::is_convertible_v<T&, Mat4&>, Mat4>
	{
		return transform;
	}

	void init_mesh(MESH* m)
	{
		for (View* v : linked_views_)
		{
			parameters_[v][m];
			std::shared_ptr<Attribute<Vec3>> joint_position = cgogn::get_attribute<Vec3, Joint>(*m, "position");
			if (joint_position)
				set_joint_position(*v, *m, joint_position);

			mesh_connections_[m].push_back(
				boost::synapse::connect<typename MeshProvider<MESH>::connectivity_changed>(m, [this, v, m]() {
					Parameters& p = parameters_[v][m];
					if (p.joint_position_)
						p.joint_base_size_ = float32(geometry::mean_edge_length(*m, p.joint_position_.get()) / 7.0);
					if (p.joint_base_size_ == 0.0)
					{
						MeshData<MESH>& md = mesh_provider_->mesh_data(*m);
						p.joint_base_size_ = float32((md.bb_max_ - md.bb_min_).norm() / 20.0);
					}
					v->request_update();
				}));
			mesh_connections_[m].push_back(
				boost::synapse::connect<typename MeshProvider<MESH>::template attribute_changed_t<Vec3>>(
					m, [this, v, m](Attribute<Vec3>* attribute) {
						Parameters& p = parameters_[v][m];
						if (p.joint_position_.get() == attribute)
						{
							p.joint_base_size_ =
								float32(geometry::mean_edge_length(*m, p.joint_position_.get()) / 7.0);
							if (p.joint_base_size_ == 0.0)
							{
								MeshData<MESH>& md = mesh_provider_->mesh_data(*m);
								p.joint_base_size_ = float32((md.bb_max_ - md.bb_min_).norm() / 20.0);
							}
						}
						v->request_update();
					}));
			mesh_connections_[m].push_back(
				boost::synapse::connect<typename MeshProvider<MESH>::attribute_changed>(
					m, [this, v, m](AttributeGen* attribute) {
						Parameters& p = parameters_[v][m];
						if (attribute && attribute == p.bone_transform_.get())
						{
							cgogn_assert(v && m);
							if constexpr (sizeof...(SupportedTransforms) > 0)
								update_bone_normal_vbo<SupportedTransforms...>(*v, *m, attribute);
							v->request_update();
						}
					}));
		}
	}

	template <typename T, typename ...Args>
	void update_bone_normal_vbo(View& v, const MESH& m, const AttributeGen* attribute)
	{
		if (auto p = dynamic_cast<const Attribute<T>*>(attribute))
			update_bone_normal_vbo(v, m, *p);
		else if constexpr (sizeof...(Args) > 0)
			update_bone_normal_vbo<Args...>(v, m, attribute);
	}

	template <typename TransformT>
	void update_bone_normal_vbo(View& v, const MESH& m, const Attribute<TransformT>& bone_transform,
			bool request_view_update = true)
	{
		Parameters& p = parameters_[&v][&m];
		MeshData<MESH>& md = mesh_provider_->mesh_data(m);
		std::vector<geometry::Vec3> normals;
		for (const auto& transform : bone_transform)
		{
			Mat4 transform_matrix = get_transform_matrix(transform);
			normals.push_back(transform_matrix.block<3, 1>(0, 1) / transform_matrix(3, 3));
		}
		rendering::update_vbo(normals, p.bone_normal_vbo_.get());
		if (request_view_update)
			v.request_update();
	}

public:
	template <typename T, typename ...Args>
	bool attribute_is_transform(const std::shared_ptr<AttributeGen>& attribute)
	{
		if (std::dynamic_pointer_cast<Attribute<T>>(attribute))
			return true;
		if constexpr (sizeof...(Args) > 0)
			return attribute_is_transform<Args...>(attribute);
		return false;
	}

	bool attribute_is_transform(const std::shared_ptr<AttributeGen>& attribute)
	{
		if constexpr (sizeof...(SupportedTransforms) > 0)
			return attribute_is_transform<SupportedTransforms...>(attribute);
		return false;
	}

	void set_joint_position(View& v, const MESH& m, const std::shared_ptr<Attribute<Vec3>>& joint_position)
	{
		Parameters& p = parameters_[&v][&m];
		if (p.joint_position_ == joint_position)
			return;

		p.joint_position_ = joint_position;
		if (p.joint_position_)
		{
			MeshData<MESH>& md = mesh_provider_->mesh_data(m);
			p.joint_position_vbo_ = md.update_vbo(p.joint_position_.get(), true);
			p.joint_base_size_ = float32(geometry::mean_edge_length(m, p.joint_position_.get()) / 7.0);
			if (p.joint_base_size_ == 0.0)
				p.joint_base_size_ = float32((md.bb_max_ - md.bb_min_).norm() / 20.0);
		}
		else
			p.joint_position_vbo_ = nullptr;

		p.param_point_sprite_->set_vbos({p.joint_position_vbo_});
		p.param_point_sprite_size_->set_vbos({p.joint_position_vbo_, p.joint_radius_vbo_});
		p.param_point_sprite_color_->set_vbos({p.joint_position_vbo_, p.joint_color_vbo_});
		p.param_point_sprite_color_size_->set_vbos({p.joint_position_vbo_, p.joint_color_vbo_, p.joint_radius_vbo_});
		p.param_animation_skeleton_bone_->set_vbos({p.joint_position_vbo_});
		p.param_animation_skeleton_bone_color_normal_->set_vbos(
				{p.joint_position_vbo_, p.bone_color_vbo_, p.bone_normal_vbo_.get()});

		v.request_update();
	}

	void set_joint_radius(View& v, const MESH& m, const std::shared_ptr<Attribute<Scalar>>& joint_radius)
	{
		Parameters& p = parameters_[&v][&m];
		if (p.joint_radius_ == joint_radius)
			return;

		p.joint_radius_ = joint_radius;
		if (p.joint_radius_)
		{
			MeshData<MESH>& md = mesh_provider_->mesh_data(m);
			p.joint_radius_vbo_ = md.update_vbo(joint_radius.get(), true);
		}
		else
			p.joint_radius_vbo_ = nullptr;

		p.param_point_sprite_size_->set_vbos({p.joint_position_vbo_, p.joint_radius_vbo_});
		p.param_point_sprite_color_size_->set_vbos({p.joint_position_vbo_, p.joint_color_vbo_, p.joint_radius_vbo_});

		v.request_update();
	}

	void set_joint_color(View& v, const MESH& m, const std::shared_ptr<Attribute<Vec3>>& joint_color)
	{
		Parameters& p = parameters_[&v][&m];
		if (p.joint_color_ == joint_color)
			return;

		p.joint_color_ = joint_color;
		if (p.joint_color_)
		{
			MeshData<MESH>& md = mesh_provider_->mesh_data(m);
			p.joint_color_vbo_ = md.update_vbo(p.joint_color_.get(), true);
		}
		else
			p.joint_color_vbo_ = nullptr;

		p.param_point_sprite_color_->set_vbos({p.joint_position_vbo_, p.joint_color_vbo_});
		p.param_point_sprite_color_size_->set_vbos({p.joint_position_vbo_, p.joint_color_vbo_, p.joint_radius_vbo_});

		v.request_update();
	}

	void reset_bone_transform(View& v, const MESH& m)
	{
		Parameters& p = parameters_[&v][&m];
		if (!p.bone_transform_)
			return;

		p.bone_transform_ = nullptr;
		// Don't reset bone_normal_vbo_ and just pass nullptr, this avoids recreating it each time

		p.param_animation_skeleton_bone_color_normal_->set_vbos(
				{p.joint_position_vbo_, p.bone_color_vbo_, nullptr});

		v.request_update();
	}

	/// @brief Sets the bone transform attribute attribute for a specific type of transform.
	/// @param v the view to update
	/// @param m the mesh that holds the attribute
	/// @param bone_transform the shared pointer to set as the current bone transform attribute
	///                       (also used to determine `bone_transform_gen` if it's null)
	/// @param bone_transform_gen the generic pointer to actually set the attribute parameter to
	///                       (inferred from `bone_transform` if null)
	template <typename TransformT>
	void set_bone_transform(View& v, const MESH& m, const std::shared_ptr<Attribute<TransformT>>& bone_transform,
			std::shared_ptr<AttributeGen> bone_transform_gen = nullptr)
	{
		if (!bone_transform)
		{
			cgogn_assert(!bone_transform_gen); // passing null bt but non-null btg makes no sense
			reset_bone_transform(v, m);
			return;
		}

		if (!bone_transform_gen)
		{
			bone_transform_gen = std::dynamic_pointer_cast<AttributeGen>(bone_transform);
			cgogn_assert(bone_transform_gen); // conversion back to AttributeGen expected to work
		}

		Parameters& p = parameters_[&v][&m];
		if (p.bone_transform_ == bone_transform_gen)
			return;

		p.bone_transform_ = bone_transform_gen;

		update_bone_normal_vbo(v, m, *bone_transform, false);

		p.param_animation_skeleton_bone_color_normal_->set_vbos(
				{p.joint_position_vbo_, p.bone_color_vbo_, p.bone_normal_vbo_.get()});

		v.request_update();
	}

	/// @brief Attempts to set the bone transform attribute with the provided one.
	/// No-op if it fails to dynamically cast to an attribute of a transform type in the template argument list.
	/// Prefer the identical overload with empty template parameters
	/// unless you specifically wish to override the list of supported transform types for this call.
	/// @param v the view to update
	/// @param m the mesh that holds the attribute
	/// @param attribute a shared pointer to the generic candidate transform attribute
	template <typename T, typename ...Args>
	void set_bone_transform(View& v, const MESH& m, const std::shared_ptr<AttributeGen>& attribute)
	{
		if (!attribute)
			reset_bone_transform(v, m);
		else if (auto p = std::dynamic_pointer_cast<Attribute<T>>(attribute))
			set_bone_transform(v, m, p, attribute);
		else if constexpr (sizeof...(Args) > 0)
			set_bone_transform<Args...>(v, m, attribute);
	}

	/// @brief Attempts to set the bone transform attribute with the provided one.
	/// No-op if it fails to dynamically cast to an attribute of a supported transform type.
	/// Prefer the non-generic attribute transform type overload if said type is known at compile time.
	/// @param v the view to update
	/// @param m the mesh that holds the attribute
	/// @param attribute a shared pointer to the generic candidate transform attribute
	void set_bone_transform(View& v, const MESH& m, const std::shared_ptr<AttributeGen>& attribute)
	{
		if (!attribute)
			reset_bone_transform(v, m);
		else if constexpr (sizeof...(SupportedTransforms) > 0)
			set_bone_transform<SupportedTransforms...>(v, m, attribute);
	}

	void set_bone_color(View& v, const MESH& m, const std::shared_ptr<Attribute<Vec3>>& bone_color)
	{
		Parameters& p = parameters_[&v][&m];
		if (p.bone_color_ == bone_color)
			return;

		p.bone_color_ = bone_color;
		if (p.bone_color_)
		{
			MeshData<MESH>& md = mesh_provider_->mesh_data(m);
			p.bone_color_vbo_ = md.update_vbo(p.bone_color_.get(), true);
		}
		else
			p.bone_color_vbo_ = nullptr;

		p.param_animation_skeleton_bone_color_normal_->set_vbos(
				{p.joint_position_vbo_, p.bone_color_vbo_, p.bone_normal_vbo_.get()});

		v.request_update();
	}

protected:
	void init() override
	{
		mesh_provider_ = static_cast<ui::MeshProvider<MESH>*>(
			app_.module("MeshProvider (" + std::string{mesh_traits<MESH>::name} + ")"));
		mesh_provider_->foreach_mesh([this](MESH& m, const std::string&) { init_mesh(&m); });
		connections_.push_back(boost::synapse::connect<typename MeshProvider<MESH>::mesh_added>(
			mesh_provider_, this, &AnimationSkeletonRender::init_mesh));
	}

	void draw(View* view) override
	{
		for (auto& [m, p] : parameters_[view])
		{
			MeshData<MESH>& md = mesh_provider_->mesh_data(*m);

			const rendering::GLMat4& proj_matrix = view->projection_matrix();
			const rendering::GLMat4& view_matrix = view->modelview_matrix();

			if (p.render_bones_)
			{
				switch (p.bone_color_per_cell_)
				{
				case GLOBAL:
					if (p.param_animation_skeleton_bone_->attributes_initialized())
					{
						p.param_animation_skeleton_bone_->bind(proj_matrix, view_matrix);
						md.draw(rendering::LINES);
						p.param_animation_skeleton_bone_->release();
					}
					break;
				case PER_BONE:
					if (p.param_animation_skeleton_bone_color_normal_->attributes_initialized())
					{
						p.param_animation_skeleton_bone_color_normal_->bind(proj_matrix, view_matrix);
						md.draw(rendering::LINES_TB);
						p.param_animation_skeleton_bone_color_normal_->release();
					}
					break;
				}
			}

			if (p.render_joints_)
			{
				if (p.joint_radius_)
				{
					switch (p.color_per_cell_)
					{
					case GLOBAL: {
						if (p.param_point_sprite_size_->attributes_initialized())
						{
							p.param_point_sprite_size_->bind(proj_matrix, view_matrix);
							md.draw(rendering::POINTS);
							p.param_point_sprite_size_->release();
						}
					}
					break;
					case PER_JOINT: {
						if (p.param_point_sprite_color_size_->attributes_initialized())
						{
							p.param_point_sprite_color_size_->bind(proj_matrix, view_matrix);
							md.draw(rendering::POINTS);
							p.param_point_sprite_color_size_->release();
						}
					}
					break;
					}
				}
				else
				{
					switch (p.color_per_cell_)
					{
					case GLOBAL: {
						if (p.param_point_sprite_->attributes_initialized())
						{
							p.param_point_sprite_->point_size_ = p.joint_base_size_ * p.joint_scale_factor_;
							p.param_point_sprite_->bind(proj_matrix, view_matrix);
							md.draw(rendering::POINTS);
							p.param_point_sprite_->release();
						}
					}
					break;
					case PER_JOINT: {
						if (p.param_point_sprite_color_->attributes_initialized())
						{
							p.param_point_sprite_color_->point_size_ = p.joint_base_size_ * p.joint_scale_factor_;
							p.param_point_sprite_color_->bind(proj_matrix, view_matrix);
							md.draw(rendering::POINTS);
							p.param_point_sprite_color_->release();
						}
					}
					break;
					}
				}
			}
		}
	}

	void left_panel() override
	{
		bool need_update = false;

		if (app_.nb_views() > 1)
			imgui_view_selector(this, selected_view_, [&](View* v) { selected_view_ = v; });

		imgui_mesh_selector(mesh_provider_, selected_mesh_, "Graph", [&](MESH& m) { selected_mesh_ = &m; });

		if (selected_view_ && selected_mesh_)
		{
			Parameters& p = parameters_[selected_view_][selected_mesh_];

			imgui_combo_attribute<Joint, Vec3>(*selected_mesh_, p.joint_position_, "Position",
												[&](const std::shared_ptr<Attribute<Vec3>>& attribute) {
													set_joint_position(*selected_view_, *selected_mesh_, attribute);
												});

			imgui_combo_attribute<Joint, Scalar>(*selected_mesh_, p.joint_radius_, "Radius",
												  [&](const std::shared_ptr<Attribute<Scalar>>& attribute) {
													  set_joint_radius(*selected_view_, *selected_mesh_, attribute);
												  });

			ImGui::Separator();
			need_update |= ImGui::Checkbox("Joints", &p.render_joints_);
			if (p.render_joints_)
			{
				if (!p.joint_radius_)
					need_update |= ImGui::SliderFloat("Size##joints", &(p.joint_scale_factor_), 0.1f, 2.0f);

				ImGui::TextUnformatted("Colors");
				ImGui::BeginGroup();
				if (ImGui::RadioButton("Global##color", p.color_per_cell_ == GLOBAL))
				{
					p.color_per_cell_ = GLOBAL;
					need_update = true;
				}
				ImGui::SameLine();
				if (ImGui::RadioButton("Per joint##color", p.color_per_cell_ == PER_JOINT))
				{
					p.color_per_cell_ = PER_JOINT;
					need_update = true;
				}
				ImGui::EndGroup();

				if (p.color_per_cell_ == GLOBAL)
				{
					if (ImGui::ColorEdit3("Color##joints", p.param_point_sprite_->color_.data(),
										  ImGuiColorEditFlags_NoInputs))
					{
						p.param_point_sprite_size_->color_ = p.param_point_sprite_->color_;
						need_update = true;
					}
				}
				else if (p.color_per_cell_ == PER_JOINT)
				{
					imgui_combo_attribute<Joint, Vec3>(
						*selected_mesh_, p.joint_color_, "Attribute##vectorjointcolor",
						[&](const std::shared_ptr<Attribute<Vec3>>& attribute) {
							set_joint_color(*selected_view_, *selected_mesh_, attribute);
						});
				}
			}

			ImGui::Separator();
			need_update |= ImGui::Checkbox("Bones", &p.render_bones_);
			if (p.render_bones_)
			{
				ImGui::TextUnformatted("Colors and roll");
				ImGui::BeginGroup();
				if (ImGui::RadioButton("Global##colorBones", p.bone_color_per_cell_ == GLOBAL))
				{
					p.bone_color_per_cell_ = GLOBAL;
					need_update = true;
				}
				ImGui::SameLine();
				if (ImGui::RadioButton("Per bone##colorBones", p.bone_color_per_cell_ == PER_BONE))
				{
					p.bone_color_per_cell_ = PER_BONE;
					need_update = true;
				}
				ImGui::EndGroup();

				switch (p.bone_color_per_cell_)
				{
				case GLOBAL:
					need_update |=
						ImGui::ColorEdit3("Color##bones", p.param_animation_skeleton_bone_->color_.data(), ImGuiColorEditFlags_NoInputs);
					break;
				case PER_BONE:
					imgui_combo_attribute<Bone, Vec3>(
						*selected_mesh_, p.bone_color_, "Color##vectorbonecolor",
						[&](const std::shared_ptr<Attribute<Vec3>>& attribute) {
							set_bone_color(*selected_view_, *selected_mesh_, attribute);
						});
					imgui_combo_any_attribute<Bone>(
						*selected_mesh_, std::dynamic_pointer_cast<AttributeGen>(p.bone_transform_), "World transform##vectorbonetransform",
						[&](const std::shared_ptr<AttributeGen>& attribute) { return attribute_is_transform(attribute); },
						[&](const std::shared_ptr<AttributeGen>& attribute) { set_bone_transform(*selected_view_, *selected_mesh_, attribute); });
					break;
				}

				if (ImGui::SliderFloat("Thickness##bones", &p.param_animation_skeleton_bone_->radius_, 0.0f, 2.0f))
				{
					p.param_animation_skeleton_bone_color_normal_->radius_= p.param_animation_skeleton_bone_->radius_;
					need_update = true;
				}

				if (ImGui::SliderFloat("Lighting##bones", &p.param_animation_skeleton_bone_->lighted_, 0.0f, 1.0f))
				{
					p.param_animation_skeleton_bone_color_normal_->lighted_ = p.param_animation_skeleton_bone_->lighted_;
					need_update = true;
				}
			}

			if (need_update)
				for (View* v : linked_views_)
					v->request_update();
		}
	}

private:
	View* selected_view_;
	const MESH* selected_mesh_;
	std::unordered_map<View*, std::unordered_map<const MESH*, Parameters>> parameters_;
	std::vector<std::shared_ptr<boost::synapse::connection>> connections_;
	std::unordered_map<const MESH*, std::vector<std::shared_ptr<boost::synapse::connection>>> mesh_connections_;
	MeshProvider<MESH>* mesh_provider_;
};

} // namespace ui

} // namespace cgogn

#endif // CGOGN_MODULE_ANIMATION_SKELETON_RENDER_H_
