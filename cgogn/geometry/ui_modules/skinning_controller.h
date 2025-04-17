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

#ifndef CGOGN_MODULE_SKINNING_CONTROLLER_H_
#define CGOGN_MODULE_SKINNING_CONTROLLER_H_

#include <array>
#include <algorithm>
#include <unordered_map>

#include <boost/core/demangle.hpp>

#include <cgogn/ui/app.h>
#include <cgogn/ui/module.h>

#include <cgogn/core/ui_modules/mesh_provider.h>
#include <cgogn/geometry/algos/skinning_helper.h>

namespace cgogn
{

namespace ui
{

using geometry::Vec3;
using geometry::Vec4;
using geometry::Vec4i;
using geometry::Scalar;

template <typename Mesh, typename TransformT>
class SkinningController : public Module
{
public:
	using Skinning = geometry::SkinningHelper<TransformT>;

	enum class UpdatePolicy
	{
		Auto,                      // depends on the corresponding auto-bind toggle
		DoNotBind,                 // only set, don't bind nor update embedding
		Bind,                      // set and bind, but don't try to update embedding
		BindAndTryUpdateEmbedding, // set and bind, then try to update embedding
	};

protected:
	using Skeleton = AnimationSkeleton;

	template <typename T>
	using AttributeSk = AnimationSkeleton::Attribute<T>;

	using Bone = AnimationSkeleton::Bone;

	template <typename T>
	using AttributeSf = typename mesh_traits<Mesh>::template Attribute<T>;

	using Vertex = typename mesh_traits<Mesh>::Vertex;

public:
	SkinningController(const App& app,
			const std::string& mesh_bind_position_attribute_unique_name = "bind_vertex_position",
			const std::string& skeleton_bind_pose_attribute_unique_name = "bind_bone_inv_world_transform_"
					+ get_demangled_transform_name()) :
		Module(app, "SkinningController (" + std::string{mesh_traits<Skeleton>::name}
				+ ", " + get_demangled_mesh_name() + ", " + get_demangled_transform_name() + ")"),
		bind_vertex_position_attribute_name_(mesh_bind_position_attribute_unique_name),
		bind_inv_world_transform_attribute_name_(skeleton_bind_pose_attribute_unique_name)
	{
	}
	~SkinningController()
	{
	}

	/// @brief Retrieves the corresponding save attribute of the provided one
	/// @param attribute the attribute to get the save of
	/// @return a pointer to the corresponding save attribute, or `nullptr` if there isn't any
	std::shared_ptr<AttributeSf<Vec3>> get_saved_vertex_position(
			const std::shared_ptr<AttributeSf<Vec3>>& attribute)
	{
		const auto& it = saved_vertex_positions_.find(attribute.get());
		return it == saved_vertex_positions_.cend() ? nullptr : it->second;
	}

	/// @brief Saves the selected vertex position attribute's values to the provided attribute.
	/// @param save the attribute to save positions to
	/// @param warn_unset whether or not to write a warning to the standard output if the operation fails
	void save_vertex_position(const std::shared_ptr<AttributeSf<Vec3>>& save, bool warn_unset = true)
	{
		if (selected_vertex_position_)
		{
			cgogn_assert(save);
			save->copy(*selected_vertex_position_);
			saved_vertex_positions_[selected_vertex_position_.get()] = save;
		}
		else if (warn_unset)
			std::cout << "[SkinningController::save_vertex_position] Vertex position isn't set" << std::endl;
	}

	/// @brief Saves the selected vertex position attribute's values to a dedicated attribute.
	/// @param warn_unset whether or not to write a warning to the standard output if the operation fails
	void save_vertex_position(bool warn_unset = true)
	{
		if (!selected_mesh_)
		{
			if (warn_unset)
				std::cout << "[SkinningController::save_vertex_position] Mesh or vertex position isn't set" << std::endl;
			return;
		}

		if (!selected_vertex_position_)
		{
			if (warn_unset)
				std::cout << "[SkinningController::save_vertex_position] Vertex position isn't set" << std::endl;
			return;
		}

		save_vertex_position(
				get_or_add_attribute<Vec3, Vertex>(*selected_mesh_, selected_vertex_position_->name() + "_" + name()),
				warn_unset);
	}

	/// @brief Restores the selected vertex position attribute's values to the provided save if it's non-null.
	/// @param save the save attribute to restore positions from
	/// @param warn_unset whether or not to write a warning to the standard output if the operation fails
	void restore_vertex_position(const std::shared_ptr<const AttributeSf<Vec3>>& save, bool warn_unset = true)
	{
		if (!save)
		{
			if (warn_unset)
				std::cout << "[SkinningController::restore_vertex_position] No save for this attribute" << std::endl;
			return;
		}

		if (!selected_vertex_position_)
		{
			if (warn_unset)
				std::cout << "[SkinningController::restore_vertex_position] Vertex position isn't set" << std::endl;
			return;
		}

		selected_vertex_position_->copy(*save);

		if (mesh_provider_)
			mesh_provider_->emit_attribute_changed(*selected_mesh_, selected_vertex_position_.get());
	}

	/// @brief Restores the selected vertex position attribute's values to its save if such a save exists.
	/// @param warn_unset whether or not to write a warning to the standard output if the operation fails
	void restore_vertex_position(bool warn_unset = true)
	{
		// Emplaces nullptr if no entry for this key, which triggers the intended behavior from the overload above
		restore_vertex_position(saved_vertex_positions_[selected_vertex_position_.get()], warn_unset);
	}

	/// @brief Changes the linked vertex position attribute, and updates the mesh if possible.
	/// @param attribute the new attribute to use as positions
	/// @param policy see `UpdatePolicy
	void set_vertex_position(const std::shared_ptr<AttributeSf<Vec3>>& attribute, UpdatePolicy policy = UpdatePolicy::Auto)
	{
		if (restore_vertex_position_on_unbind_ && attribute != selected_vertex_position_)
			if (const auto& save = get_saved_vertex_position(selected_vertex_position_))
				restore_vertex_position(save, false);

		selected_vertex_position_ = attribute;

		if (policy == UpdatePolicy::Auto)
			policy = auto_bind_mesh_ ? UpdatePolicy::BindAndTryUpdateEmbedding : UpdatePolicy::DoNotBind;

		if (policy == UpdatePolicy::DoNotBind || !selected_mesh_)
			return;

		selected_bind_vertex_position_
				= get_or_add_attribute<Vec3, Vertex>(*selected_mesh_, bind_vertex_position_attribute_name_);

		if (attribute)
			selected_bind_vertex_position_->copy(*attribute);

		if (save_vertex_position_on_first_bind_ && !get_saved_vertex_position(attribute))
			save_vertex_position(false);

		if (policy == UpdatePolicy::BindAndTryUpdateEmbedding)
			update_embedding();
	}

	/// @brief Changes the linked vertex weight index attribute, and updates the mesh if possible.
	/// @param attribute the new attribute to use as weight indices
	void set_vertex_weight_index(const std::shared_ptr<AttributeSf<Vec4i>>& attribute)
	{
		selected_vertex_weight_index_valid_ = true;
		selected_vertex_weight_index_ = attribute;
		update_embedding();
	}

	/// @brief Changes the linked vertex weight value attribute, and updates the mesh if possible.
	/// @param attribute the new attribute to use as weight values
	void set_vertex_weight_value(const std::shared_ptr<AttributeSf<Vec4>>& attribute)
	{
		selected_vertex_weight_value_ = attribute;
		update_embedding();
	}

	/// @brief Changes the linked vertex weight value attribute, and updates the mesh if possible.
	/// @param attribute the new attribute to use as transforms
	/// @param policy see `UpdatePolicy`
	void set_bone_world_transform(const std::shared_ptr<AttributeSk<TransformT>>& attribute, UpdatePolicy policy = UpdatePolicy::Auto)
	{
		selected_bone_world_transform_ = attribute;

		if (policy == UpdatePolicy::Auto)
			policy = auto_bind_skeleton_ ? UpdatePolicy::BindAndTryUpdateEmbedding : UpdatePolicy::DoNotBind;

		if (policy == UpdatePolicy::DoNotBind || !selected_skeleton_)
			return;

		selected_bind_bone_inv_world_transform_
				= get_or_add_attribute<TransformT, Bone>(*selected_skeleton_, bind_inv_world_transform_attribute_name_);

		if (attribute)
		{
			selected_bind_bone_inv_world_transform_->copy(*attribute);

			for (auto& t : *selected_bind_bone_inv_world_transform_)
				t = t.inverse();
		}

		if (policy == UpdatePolicy::BindAndTryUpdateEmbedding)
			update_embedding();
	}

	/// @brief Changes the linked mesh, and resets attribute selection for it.
	/// @param sf the new mesh to link to
	void set_mesh(Mesh* sf)
	{
		selected_mesh_ = sf;
		selected_vertex_weight_index_valid_ = true;
		selected_vertex_weight_index_.reset();
		selected_vertex_weight_value_.reset();
		set_vertex_position(get_attribute<Vec3, Vertex>(*sf, "position")); // nullptr (equiv. to reset) if not found

		mesh_connections_[0] = !sf ? nullptr
				: boost::synapse::connect<typename MeshProvider<Mesh>::template attribute_changed_t<Vec4i>>(
						sf, [&](AttributeSf<Vec4i>* attribute)
				{
						if (selected_vertex_weight_index_.get() == attribute)
							update_embedding();
				});
		mesh_connections_[1] = !sf ? nullptr
				: boost::synapse::connect<typename MeshProvider<Mesh>::template attribute_changed_t<Vec4>>(
						sf, [&](AttributeSf<Vec4>* attribute)
				{
						if (selected_vertex_weight_value_.get() == attribute)
							update_embedding();
				});
	}

	/// @brief Changes the linked skeleton, and resets attribute selection for it.
	/// @param sk the new skeleton to link to
	void set_skeleton(Skeleton* sk)
	{
		selected_skeleton_= sk;
		set_bone_world_transform(nullptr);

		skeleton_connection_ = !sk ? nullptr
				: boost::synapse::connect<typename MeshProvider<Skeleton>::template attribute_changed_t<TransformT>>(
						sk, [&](AttributeSk<TransformT>* attribute)
				{
					if (selected_bone_world_transform_.get() == attribute)
						update_embedding();
				});
	}

	/// @return the attribute name for the bind position
	[[nodiscard]]
	const std::string& mesh_bind_position_attribute_name() const
	{
		return bind_vertex_position_attribute_name_;
	}

	/// @return the attribute name for the bind pose
	[[nodiscard]]
	const std::string& skeleton_bind_pose_attribute_name() const
	{
		return bind_inv_world_transform_attribute_name_;
	}

protected:
	void init() override
	{
		mesh_provider_ = static_cast<MeshProvider<Mesh>*>(
			app_.module("MeshProvider (" + std::string{mesh_traits<Mesh>::name} + ")"));
		skeleton_provider_ = static_cast<MeshProvider<Skeleton>*>(
			app_.module("MeshProvider (" + std::string{mesh_traits<Skeleton>::name} + ")"));
	}

	void left_panel() override
	{
		imgui_mesh_selector(mesh_provider_, selected_mesh_, "Mesh", [&](Mesh& sf) { set_mesh(&sf); });

		if (selected_mesh_)
		{
			imgui_combo_attribute<Vertex, Vec3>(*selected_mesh_, selected_vertex_position_,
					"Position", [&](const std::shared_ptr<AttributeSf<Vec3>>& attribute){ set_vertex_position(attribute); });
			imgui_combo_attribute<Vertex, Vec4i>(*selected_mesh_, selected_vertex_weight_index_,
					"Weight indices", [&](const std::shared_ptr<AttributeSf<Vec4i>>& attribute){ set_vertex_weight_index(attribute); });
			imgui_combo_attribute<Vertex, Vec4>(*selected_mesh_, selected_vertex_weight_value_,
					"Weight values", [&](const std::shared_ptr<AttributeSf<Vec4>>& attribute){ set_vertex_weight_value(attribute); });

			if (!selected_vertex_weight_index_valid_)
			{
				ImGui::TextUnformatted("Invalid weight indices found");
				ImGui::TextUnformatted("Is the right skeleton selected?");
			}

			if constexpr (USE_LBS_)
				if (ImGui::Checkbox("Normalize weights", &normalize_weights_))
					update_embedding();
		}

		imgui_mesh_selector(skeleton_provider_, selected_skeleton_, "Skeleton", [&](Skeleton& sk) { set_skeleton(&sk); });

		if (selected_skeleton_)
		{
			imgui_combo_attribute<Bone, TransformT>(*selected_skeleton_, selected_bone_world_transform_,
					"World transform", [&](const std::shared_ptr<AttributeSk<TransformT>>& attribute){ set_bone_world_transform(attribute); });
		}

		if (ImGui::TreeNode("Manual update/binding"))
		{
			show_bind_controls("Bind mesh position", selected_vertex_position_ && selected_mesh_, auto_bind_mesh_, "Auto##bind_mesh",
					[&]{ set_vertex_position(selected_vertex_position_, UpdatePolicy::BindAndTryUpdateEmbedding); });

			show_bind_controls("Bind skeleton pose", selected_bone_world_transform_ && selected_skeleton_, auto_bind_skeleton_, "Auto##bind_skeleton",
					[&]{ set_bone_world_transform(selected_bone_world_transform_, UpdatePolicy::BindAndTryUpdateEmbedding); });

			// Bypass the need for the embedding to have been detected as dirty by pressing a shift key (should not be needed)
			const bool enable_button_if_possible = embedding_dirty_ || (ImGui::GetIO().KeyMods & ImGuiModFlags_Shift) != ImGuiModFlags_None;
			if (show_button("Update embedding", enable_button_if_possible && can_update_embedding()))
				update_embedding(true);
			ImGui::SameLine();
			ImGui::Checkbox("Auto##update", &auto_update_embedding_);

			ImGui::TreePop();
		}

		if (ImGui::TreeNode("Embedding saving"))
		{
			if (show_button("Save", selected_mesh_ && selected_vertex_position_))
				save_vertex_position(false);
			ImGui::SameLine();
			ImGui::Checkbox("Auto on first bind", &save_vertex_position_on_first_bind_);

			if (show_button("Restore", !!selected_vertex_position_))
				if (const auto& save = get_saved_vertex_position(selected_vertex_position_))
					restore_vertex_position(save, false);
			ImGui::SameLine();
			ImGui::Checkbox("Auto on unbind", &restore_vertex_position_on_unbind_);

			ImGui::TreePop();
		}

		if (ImGui::TreeNode("Bone influence visualization"))
		{
			show_bone_influence_visualization_controls();
			ImGui::TreePop();
		}
	}

private:
	static std::string get_demangled_mesh_name()
	{
		return boost::core::demangle(typeid(Mesh).name());
	}

	static std::string get_demangled_transform_name()
	{
		return boost::core::demangle(typeid(TransformT).name());
	}

	static bool show_button(const char* label, bool enabled)
	{
		bool res = false;

		if (!enabled)
			ImGui::BeginDisabled();

		if (ImGui::Button(label) && enabled)
			res = true;

		if (!enabled)
			ImGui::EndDisabled();

		return res;
	}

	template <bool BindOnAutoCheck = false, typename FUNC>
	static void show_bind_controls(const char* label, bool enabled, bool& auto_v, const char* auto_label, const FUNC& on_bind)
	{
		if (show_button(label, !auto_v && enabled))
			on_bind();

		ImGui::SameLine();

		if (ImGui::Checkbox(auto_label, &auto_v)) // show checkbox regardless of template parameter
			if constexpr (BindOnAutoCheck)
				if (auto_v)
					on_bind();
	}

	bool can_update_embedding()
	{
		return selected_vertex_position_ && selected_vertex_weight_index_ && selected_vertex_weight_value_
				&& selected_bone_world_transform_;
	}

	void update_embedding(bool force_update = false)
	{
		embedding_dirty_ = true;

		if (!can_update_embedding() || !auto_update_embedding_ && !force_update)
			return;

		if constexpr (USE_LBS_)
			selected_vertex_weight_index_valid_ = Skinning::compute_vertex_positions_LBS(
					*selected_mesh_, *selected_skeleton_,
					*selected_bind_bone_inv_world_transform_, *selected_bone_world_transform_,
					*selected_vertex_weight_index_, *selected_vertex_weight_value_,
					*selected_bind_vertex_position_, *selected_vertex_position_,
					normalize_weights_);
		else
			selected_vertex_weight_index_valid_ = Skinning::compute_vertex_positions_TBS(
					*selected_mesh_, *selected_skeleton_,
					*selected_bind_bone_inv_world_transform_, *selected_bone_world_transform_,
					*selected_vertex_weight_index_, *selected_vertex_weight_value_,
					*selected_bind_vertex_position_, *selected_vertex_position_);

		if (mesh_provider_)
			mesh_provider_->emit_attribute_changed(*selected_mesh_, selected_vertex_position_.get());

		embedding_dirty_ = false;
	}

	void show_bone_influence_visualization_controls()
	{
		ImGui::Checkbox("Global##influence", &global_bone_influence_computation_);

		if (global_bone_influence_computation_)
		{
			if (selected_skeleton_)
				imgui_combo_attribute<Bone, Vec3>(
						*selected_skeleton_,
						selected_bone_color_for_influence_computation_,
						"Bone color##influence",
						[&](const std::shared_ptr<AttributeSk<Vec3>>& attribute) {
								selected_bone_color_for_influence_computation_ = attribute; });
			else
				ImGui::TextUnformatted("Select skeleton to visualize influences of bones");
		}
		else if (selected_skeleton_ && selected_skeleton_->nb_bones() > 0)
		{
			int bone_index = static_cast<int>(index_of(*selected_skeleton_, selected_bone_for_influence_computation_));
			ImGui::SliderInt("Bone index##influence", &bone_index, 0, static_cast<int>(*std::max_element(
					selected_skeleton_->bone_traverser_.cbegin(), selected_skeleton_->bone_traverser_.cend(),
					[&](const Bone& a, const Bone& b) {
							return index_of(*selected_skeleton_, a) < index_of(*selected_skeleton_, b); })));
			selected_bone_for_influence_computation_ = of_index<Bone>(*selected_skeleton_, static_cast<uint32>(bone_index));

			if (selected_bone_for_influence_computation_.is_valid())
				ImGui::TextUnformatted((*selected_skeleton_->bone_name_)[static_cast<uint32>(bone_index)].c_str());
		}
		else
			ImGui::TextUnformatted("Select skeleton with bones to pick one");

		if (show_button("Update influence visualization attribute",
				selected_skeleton_ && selected_mesh_ && selected_vertex_weight_index_ && selected_vertex_weight_value_
				&& (!global_bone_influence_computation_ || selected_bone_color_for_influence_computation_)))
		{
			const auto attribute = get_or_add_attribute<Vec3, Vertex>(*selected_mesh_,
					COMPUTED_BONE_INFLUENCE_ATTRIBUTE_NAME);
			if (global_bone_influence_computation_)
				Skinning::compute_bone_influence(*selected_mesh_, *selected_skeleton_,
						*selected_vertex_weight_index_, *selected_vertex_weight_value_,
						*selected_bone_color_for_influence_computation_, *attribute);
			else
				Skinning::compute_bone_influence(*selected_mesh_, selected_bone_for_influence_computation_,
						*selected_vertex_weight_index_, *selected_vertex_weight_value_, *attribute);
			mesh_provider_->emit_attribute_changed(*selected_mesh_, attribute.get());
		}
	}

public:
	static constexpr const char* COMPUTED_BONE_INFLUENCE_ATTRIBUTE_NAME = "computed_bone_influence";

private:
	static constexpr const bool USE_LBS_ = !std::is_same_v<TransformT, geometry::DualQuaternion>;
	bool auto_bind_skeleton_ = true;
	bool auto_bind_mesh_ = true;
	bool auto_update_embedding_ = true;
	bool embedding_dirty_ = true;
	Mesh* selected_mesh_ = nullptr;
	std::shared_ptr<AttributeSf<Vec3>> selected_vertex_position_ = nullptr;
	std::shared_ptr<AttributeSf<Vec3>> selected_bind_vertex_position_ = nullptr;
	bool selected_vertex_weight_index_valid_ = true;
	std::shared_ptr<AttributeSf<Vec4i>> selected_vertex_weight_index_ = nullptr;
	std::shared_ptr<AttributeSf<Vec4>> selected_vertex_weight_value_ = nullptr;
	bool global_bone_influence_computation_ = false;
	Bone selected_bone_for_influence_computation_ = INVALID_INDEX;
	bool normalize_weights_ = true;
	std::string bind_vertex_position_attribute_name_;
	MeshProvider<Mesh>* mesh_provider_ = nullptr;
	Skeleton* selected_skeleton_ = nullptr;
	std::shared_ptr<AttributeSk<Vec3>> selected_bone_color_for_influence_computation_ = nullptr;
	std::shared_ptr<AttributeSk<TransformT>> selected_bone_world_transform_ = nullptr;
	std::shared_ptr<AttributeSk<TransformT>> selected_bind_bone_inv_world_transform_ = nullptr;
	std::string bind_inv_world_transform_attribute_name_ = nullptr;
	std::shared_ptr<boost::synapse::connection> skeleton_connection_ = nullptr;
	std::array<std::shared_ptr<boost::synapse::connection>, 2> mesh_connections_ = {nullptr, nullptr};
	MeshProvider<Skeleton>* skeleton_provider_ = nullptr;
	bool save_vertex_position_on_first_bind_ = true;
	bool restore_vertex_position_on_unbind_ = false;
	std::unordered_map<AttributeSf<Vec3>*, std::shared_ptr<AttributeSf<Vec3>>> saved_vertex_positions_;
};

} // namespace ui

} // namespace cgogn

#endif // CGOGN_MODULE_SKINNING_CONTROLLER_H_
