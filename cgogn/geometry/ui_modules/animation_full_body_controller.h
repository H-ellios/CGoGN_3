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

#ifndef CGOGN_MODULE_ANIMATION_FULL_BODY_CONTROLLER_H_
#define CGOGN_MODULE_ANIMATION_FULL_BODY_CONTROLLER_H_

#include <functional>
#include <array>
#include <set>
#include <filesystem>

#include <boost/core/demangle.hpp>
#include <boost/synapse/connect.hpp>

#include <cgogn/ui/app.h>
#include <cgogn/ui/module.h>

#include <cgogn/core/utils/numerics.h>
#include <cgogn/core/ui_modules/mesh_provider.h>
#include <cgogn/geometry/algos/animation_skeleton_embedding_helper.h>
#include <cgogn/modeling/algos/blending.h>
#include <thirdparty/rapidcsv/rapidcsv.h>

#include <cgogn/geometry/algos/skinning_helper.h>

#define DEFAULT_PATH CGOGN_STR(CGOGN_DATA_PATH)

namespace fs = std::filesystem;

namespace cgogn
{

namespace ui
{

using geometry::Vec3;
using geometry::Vec4;
using geometry::Vec4i;
using geometry::Scalar;

template <template <typename...> typename ContainerT, typename TimeT, typename TransformT , typename MESH>
class AnimationFullBodyController : public Module
{
public:

	using Skinning = geometry::SkinningHelper<TransformT>;
	enum class CheckParams
	{
		CheckAndWarnInvalid,
		CheckInvalid,
		AssumeValid,
	};

	enum class PlayMode
	{
		Pause,
		PlayOnce,
		PlayLooping,
	};

	enum class UpdatePolicy
	{
		Auto,                      // depends on the corresponding auto-bind toggle
		DoNotBind,                 // only set, don't bind nor update embedding
		Bind,                      // set and bind, but don't try to update embedding
		BindAndTryUpdateEmbedding, // set and bind, then try to update embedding
	};

	using Embedding = geometry::AnimationSkeletonEmbeddingHelper<ContainerT, TimeT, TransformT>;
	using TimePoint = typename Embedding::TimePoint;
	using RootMotionData = typename Embedding::RootMotionData;

private:
	using Skeleton = AnimationSkeleton;

	template <typename T>
	using Attribute = AnimationSkeleton::Attribute<T>;

	using Joint = AnimationSkeleton::Joint;
	using Bone = AnimationSkeleton::Bone;

    template <typename T>
	using AttributeSf = typename mesh_traits<MESH>::template Attribute<T>;

	using Vertex = typename mesh_traits<MESH>::Vertex;
	using Edge = typename mesh_traits<MESH>::Edge;
	using Face = typename mesh_traits<MESH>::Face;

	using AnimationT = geometry::KeyframedAnimation<ContainerT, TimeT, TransformT>;

public:
	AnimationFullBodyController(const App& app,
			const std::string& local_transform_attribute_unique_name = "local_transform_" + get_demangled_transform_name(),
			const std::string& world_transform_attribute_unique_name = "world_transform_" + get_demangled_transform_name()) :
		Module(app, "AnimationFullBodyController (" + std::string{mesh_traits<Skeleton>::name} + ", " + get_demangled_animation_name() + ")"),
			local_transform_attribute_name_(local_transform_attribute_unique_name),
			world_transform_attribute_name_(world_transform_attribute_unique_name)
	{
	}
	~AnimationFullBodyController()
	{
	}

	static bool ends_with(const std::string& str, const std::string& suffix)
	{
		return str.size() >= suffix.size() && str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
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
				std::cout << "[SkinningController::save_vertex_position] MESH or vertex position isn't set" << std::endl;
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
			skin_mesh_provider_->emit_attribute_changed(*selected_mesh_, selected_vertex_position_.get());
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
			skin_update_embedding();
	}

	/// @brief Changes the linked mesh, and resets attribute selection for it.
	/// @param sf the new mesh to link to
	void set_mesh(MESH* sf)
	{
		selected_mesh_ = sf;
		selected_vertex_weight_index_valid_ = true;
		selected_vertex_weight_index_.reset();
		selected_vertex_weight_value_.reset();
		set_vertex_position(get_attribute<Vec3, Vertex>(*sf, "bind_vertex_position")); // nullptr (equiv. to reset) if not found

		mesh_connections_[0] = !sf ? nullptr
				: boost::synapse::connect<typename MeshProvider<MESH>::template attribute_changed_t<Vec4i>>(
						sf, [&](AttributeSf<Vec4i>* attribute)
				{
						if (selected_vertex_weight_index_.get() == attribute)
							skin_update_embedding();
				});
		mesh_connections_[1] = !sf ? nullptr
				: boost::synapse::connect<typename MeshProvider<MESH>::template attribute_changed_t<Vec4>>(
						sf, [&](AttributeSf<Vec4>* attribute)
				{
						if (selected_vertex_weight_value_.get() == attribute)
							skin_update_embedding();
				});
	}

	/// @brief Changes the linked vertex weight index attribute, and updates the mesh if possible.
	/// @param attribute the new attribute to use as weight indices
	void set_vertex_weight_index(const std::shared_ptr<AttributeSf<Vec4i>>& attribute)
	{
		selected_vertex_weight_index_valid_ = true;
		selected_vertex_weight_index_ = attribute;
		skin_update_embedding();
	}

	/// @brief Changes the linked vertex weight value attribute, and updates the mesh if possible.
	/// @param attribute the new attribute to use as weight values
	void set_vertex_weight_value(const std::shared_ptr<AttributeSf<Vec4>>& attribute)
	{
		selected_vertex_weight_value_ = attribute;
		skin_update_embedding();
	}


	// call this function after you initialized the module
	void set_directory(std::string dirname)
	{
		directory_ = dirname;
	}

	// No signal system , call this function after you initialized the module
	void set_position_attr_name(std::string name)
	{
		pos_attr_name = name;
	}

    // Put inside a vector all the files with an extension ext
	void set_all_paths(std::string root, std::string ext, std::vector<std::string>& paths)
	{
		for (auto& p : fs::recursive_directory_iterator(root))
		{
			if (p.path().extension() == ext)
			{
				paths.push_back(p.path().string());
				std::cout << p.path().string() << std::endl;
			}
		}
		std::sort(paths.begin(), paths.end());
	}

		// Parse a csv using a filename and the separator of the csv
	void csv_parser(std::string& filename, char separator, Eigen::MatrixXd& csv_weights_detected,
					Eigen::MatrixXd& csv_weights_confirm, Eigen::VectorXd& vector_OF_rest_csv)
	{
		rapidcsv::Document doc(filename, rapidcsv::LabelParams(0, -1), rapidcsv::SeparatorParams(separator, true));
		std::ofstream outputFile("test.txt"); // Open/create a file named "test.txt" for writing
		std::vector<std::string> csv_columns_name = doc.GetColumnNames();
		std::vector<float> tempData;
		int nb_elem = 0;
		csv_.clear();
		for (int i = 0; i < csv_columns_name.size(); i++)
		{
			if (strcmp(csv_columns_name[i].c_str(), "AU28_c") != 0)
			{
				std::cout << csv_columns_name[i].c_str() << std::endl;
				tempData = doc.GetColumn<float>(csv_columns_name[i]);
				for (int j = 0; j < tempData.size(); j++)
				{
					if (outputFile.is_open())
					{
						outputFile << tempData[j];
						outputFile << ";";
					}
					else
					{
						std::cout << "Failed to create the file." << std::endl;
					}
				}
				outputFile << tempData.size();
				outputFile << "\n";
				csv_.emplace(csv_columns_name[i], tempData);
				tempData.clear();
			}
		}
		std::cout << "Text has been written to the file." << std::endl;
		outputFile.close();

		int i = 0;
		int nb_columns = 0;
		std::map<std::string, std::vector<float>>::iterator iter = csv_.begin();
		nb_elem = iter->second.size();
		for (auto& it : csv_)
		{
			if (ends_with(it.first, "_r") || ends_with(it.first, "_c"))
			{
				nb_columns++;
			}
		}

		int incr = 0;
		int incr2 = 0;

		if (nb_elem > 1)
		{
			nb_elem--;
		}

		std::cout << "NB_columns : " << nb_columns / 2 << std::endl;
		std::cout << "NB_elems : " << nb_elem << std::endl;
		csv_weights_detected.resize(nb_elem, nb_columns / 2);
		csv_weights_confirm.resize(nb_elem, nb_columns / 2);
		vector_OF_rest_csv.resize(nb_columns / 2);
		vector_OF_rest_cgogn_.resize(nb_columns / 2);

		for (auto& it : csv_)
		{
			if (ends_with(it.first, "_r"))
			{
				vector_OF_rest_csv(incr) = it.second[0];
				if (it.second.size() <= 1)
					csv_weights_detected(0, incr) = it.second[0];

				for (int j = 1; j < it.second.size(); j++)
					csv_weights_detected(j - 1, incr) = it.second[j];
				incr++;
			}
			if (ends_with(it.first, "_c"))
			{
				if (it.second.size() <= 1)
					csv_weights_confirm(0, incr) = it.second[0];
				for (int j = 1; j < it.second.size(); j++)
					csv_weights_confirm(j - 1, incr2) = it.second[j];
				incr2++;
			}
		}
	}

	// Get slopes for each AUs
	bool get_alphas_betas(std::string filename){
		std::ifstream infile(filename);
		if (!infile) {
			std::cerr << "Cannot open file\n";
			return true;
		}
		std::string line;
		if (infile.is_open())
		{
			int jacob_nb_rows = 0;
			infile >> jacob_nb_rows;
			alphas.resize(jacob_nb_rows, jacob_nb_rows);
			betas.resize(jacob_nb_rows, jacob_nb_rows);

			for (int i = 0; i < jacob_nb_rows; i++)
			{
				for (int j = 0; j < jacob_nb_rows; j++)
				{
					infile >> alphas(i, j);
					if ((i == j) && (alphas(i,j) == 0))
					{
						alphas(i,j) = 1;
					}
				}
			}

			for (int i = 0; i < jacob_nb_rows; i++)
			{
				for (int j = 0; j < jacob_nb_rows; j++)
				{
					infile >> betas(i, j);
					if (i != j)
					{
						betas(i,j) = 0;
					}
				}
			}
		}
		infile.close();
		alphas = alphas.transpose().inverse();
		return false;
	}

	// Apply slopes to each weights of each AUs for the csv
	void apply_alphas_csv(bool confirm){
		Eigen::VectorXd tmp;
		tmp.resize(csv_weights_detected_.cols());
		Eigen::MatrixXd alphas_inverse;
		alphas_inverse.resize(alphas.cols(),alphas.cols());
		alphas_inverse = alphas.transpose().inverse();
		for (int i = 0; i < csv_weights_detected_.rows(); i++)
		{
			for (int j = 0; j < csv_weights_detected_.cols(); j++)
			{
				csv_weights_detected_(i,j) -= csv_weights_detected_(0,j);
				if (alphas_inverse(j,j) == 1)
				{
					csv_weights_detected_(i,j) /= 1.5;
				}
				

				if (csv_weights_confirm_(i, j) == 0 && confirm)
					csv_weights_detected_(i, j) = 0.;
			}
			tmp = (csv_weights_detected_.row(i).transpose() - betas) * alphas;
			csv_weights_detected_.row(i) = tmp.transpose();
		}
	}

    // This function creates all the differents AUs that have been found with set_all_paths and create for each of them
	// an attribute DO NOT USE LOAD_SURFACE_FROM_FILE since it creates a new mesh and causes problems with the signal
	// system
	void setup_mesh_attributes()
	{
		for (auto path : path_aus_)
		{
			std::cout << path.substr(path.size() - 8, path.size() - (path.size() - 8) - 4) << std::endl;
			std::ifstream fp(path.c_str(), std::ios::in);
			if (!fp.good())
			{
				std::cerr << "Error opening file " << path.c_str() << std::endl;
				return;
			}
			std::shared_ptr<AttributeSf<Vec3>> au_pos = cgogn::add_attribute<Vec3, Vertex>(
				*selected_mesh_, path.substr(path.size() - 8, path.size() - (path.size() - 8) - 4));
			pos_aus_.push_back(au_pos);
			fp.seekg(0, std::ios::end);
			uint64 sz = fp.tellg();
			fp.seekg(0, std::ios::beg);
			std::vector<char> buffer(sz + 1);
			fp.read(buffer.data(), sz);
			buffer[sz] = 0;
			std::string sbuffer(buffer.data());
			std::istringstream ss(sbuffer);

			std::string tag;
			std::string line;
			std::vector<Vec3> vec_pos;
			// read vertices position
			do
			{
				ss >> tag;
				if (tag == std::string("v"))
				{
					float64 x = cgogn::io::read_double(ss, line);
					float64 y = cgogn::io::read_double(ss, line);
					float64 z = cgogn::io::read_double(ss, line);
					Vec3 temp = Vec3(x, y, z);
					vec_pos.push_back(temp);
				}
			} while (!ss.eof());

			int incr = 0;
			Vec3 point_norm;
			cgogn::foreach_cell(*selected_mesh_, [&](Vertex v) -> bool {
				point_norm = vec_pos[index_of(*selected_mesh_, v)];
				value<Vec3>(*selected_mesh_, au_pos, v) = point_norm;
				incr++;
				return true;
			});
			//geometry::rescale(*au_pos, 1);
			//skin_mesh_provider_->emit_attribute_changed(*selected_mesh_, au_pos.get());
		}
	}

	/// @brief Changes the linked bone animation attribute.
	/// Does not affect the embedding, see `set_time` and `update_embedding`.
	/// @param attribute the new attribute to use as animations
	/// @param check_params what to check about animation validity (size and sorting)
	void set_animation(const std::shared_ptr<Attribute<AnimationT>>& attribute, CheckParams check_params = CheckParams::CheckAndWarnInvalid)
	{
		selected_animation_ = attribute;

		if (!selected_animation_) // animation unset
		{
			selected_animation_time_extrema_ = {};
			return;
		}

		// Check for empty animations
		if (check_params == CheckParams::CheckAndWarnInvalid && !AnimationT::are_none_empty(
				selected_animation_->begin(), selected_animation_->end()))
			std::cout << "[AnimationFullBodyController::set_animation] Found empty animation, "
					"some bones may be animated improperly" << std::endl;

		// Check if all bones' animations are sorted
		bool all_sorted = check_params == CheckParams::AssumeValid || AnimationT::are_all_sorted(
				selected_animation_->begin(), selected_animation_->end());

		if (!all_sorted && check_params == CheckParams::CheckAndWarnInvalid)
			std::cout << "[AnimationFullBodyController::set_animation] Found unsorted animation, "
					"skeleton may be animated improperly" << std::endl;

		selected_animation_time_extrema_ = AnimationT::compute_keyframe_time_extrema(*selected_animation_, all_sorted);
	}

	/// @brief Changes the linked joint position attribute, and updates it if it's not null and a skeleton is selected.
	/// Assumes that if a skeleton is selected, its world transform attribute also is.
	/// @param attribute the new attribute to use as positions
	void set_joint_position(const std::shared_ptr<Attribute<Vec3>>& attribute)
	{
		selected_joint_position_ = attribute;

		if (selected_skeleton_ && selected_joint_position_)
			update_joint_positions_and_signal(mesh_provider_, *selected_skeleton_,
					*selected_bone_world_transform_, selected_joint_position_.get());
	}

	/// @brief Changes the current time of the animation.
	/// Updates positions accordingly if a skeleton and animation are selected.
	/// @param time the new time to set the animation to
	void set_time(TimeT time)
	{
		time_ = time;

		if (selected_skeleton_ && selected_animation_)
			skeleton_update_embedding();
	}

	/// @brief Changes the current time of the animation, and updates positions accordingly if a skeleton is selected.
	/// @param time_point the new time to set the animation to
	void set_time(TimePoint time_point)
	{
		if (!selected_animation_)
			return;

		if (!selected_animation_time_extrema_)
		{
			set_time(TimeT{});
			return;
		}

		switch (time_point)
		{
		case TimePoint::Start:
			set_time(selected_animation_time_extrema_->first);
			break;
		case TimePoint::End:
			set_time(selected_animation_time_extrema_->second);
			break;
		default:
			cgogn_assert_not_reached("Missing time point case");
		}
	}

    void set_time_start(){
        set_time(TimePoint::Start);
    }

	/// @brief Sets the animation, if any, to pause or resume.
	/// @param play_mode see `PlayMode`
	void set_play_mode(PlayMode play_mode)
	{
		play_mode_ = play_mode;
	}

	/// @brief Changes the linked skeleton, and resets attribute selection for it.
	/// @param sk the new skeleton to link to
	void set_skeleton(Skeleton* sk)
	{
		selected_skeleton_= sk;
		selected_animation_.reset();
		selected_joint_position_ = get_attribute<Vec3, Joint>(*sk, "position"); // nullptr (equiv. to reset) if not found
		selected_bone_local_transform_ = get_or_add_attribute<TransformT, Bone>(*sk, local_transform_attribute_name_);
		selected_bone_world_transform_ = get_or_add_attribute<TransformT, Bone>(*sk, world_transform_attribute_name_);
	}

	/// @return the attribute name for local transforms
	[[nodiscard]]
	const std::string& local_transform_attribute_name() const
	{
		return local_transform_attribute_name_;
	}

	/// @return the attribute name for world transforms
	[[nodiscard]]
	const std::string& world_transform_attribute_name() const
	{
		return world_transform_attribute_name_;
	}

protected:
	void init() override
	{
		mesh_provider_ = static_cast<MeshProvider<Skeleton>*>(
			app_.module("MeshProvider (" + std::string{mesh_traits<Skeleton>::name} + ")"));
		last_frame_time_ = App::frame_time_;
        set_all_paths(directory_, ".obj", path_aus_);
		set_all_paths(directory_, ".csv", path_csv_);
		std::ostringstream filename;
		filename << DEFAULT_PATH << "slopes.txt";
		std::cout << filename.str() << std::endl;
		if(get_alphas_betas(filename.str())){
			std::cout << "Run action_unit_switch exec before this one" << std::endl;
			exit(0);
		}
	}

	void left_panel() override
	{
		imgui_mesh_selector(mesh_provider_, selected_skeleton_, "Skeleton", [&](Skeleton& m) { set_skeleton(&m); });

		if (selected_skeleton_)
		{
			imgui_combo_attribute<Bone, AnimationT>(*selected_skeleton_, selected_animation_,
					"Animation", [&](const std::shared_ptr<Attribute<AnimationT>>& attribute){ set_animation(attribute); });
			imgui_combo_attribute<Joint, Vec3>(*selected_skeleton_, selected_joint_position_,
					"Position", [&](const std::shared_ptr<Attribute<Vec3>>& attribute){ set_joint_position(attribute); });

            if (ImGui::BeginCombo("Load CSV", current_item_csv))
            {
                for (int n = 0; n < path_csv_.size(); n++)
                {
                    bool is_selected = (current_item_csv == path_csv_[n].c_str());
                    if (ImGui::Selectable(path_csv_[n].substr(directory_.size(), path_csv_[n].size()).c_str(),
                                            is_selected))
                    {
                        current_item_csv = path_csv_[n].c_str();
                    }
                    if (is_selected)
                        ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }

			if (current_item_csv != NULL)
			{
				ImGui::Checkbox("Use Confirm Weights ?" , &confirm_weights);

				ImGui::Checkbox("Synchronize face and body animation ?" , &synchronize);
			}
			


			
			if (selected_animation_)
				show_time_controls();

			if (ImGui::TreeNode("Advanced"))
			{
				if (selected_animation_)
					show_advanced_time_controls();

				if (ImGui::Checkbox("Root motion", &root_motion_))
					root_motion_iteration_id_ = 0;

				ImGui::TreePop();
			}

			if (ImGui::TreeNode("Bone color generation"))
			{
				show_bone_color_generation_controls();
				ImGui::TreePop();
			}
		}
		
		advance_play(current_item_csv != NULL , current_item_csv);

        ImGui::Separator();

		last_frame_time_ = App::frame_time_;
	}

private:
	// Sets the time according to the play mode if an animation is selected.
	void advance_play(bool start , const char* current_item_csv)
	{
		if (!selected_animation_ || play_mode_ == PlayMode::Pause
				|| !selected_animation_time_extrema_) // no pose
			return;

		const auto& [start_time, end_time] = *selected_animation_time_extrema_;
		cgogn_assert(start_time <= end_time);

		if (start_time == end_time) // single pose
			return;

		if (play_mode_ == PlayMode::PlayOnce && time_ >= end_time)
		{
			set_play_mode(PlayMode::Pause); // shouldn't start playing again if the user rewinds
			return; // end already reached
		}

		TimeT new_time = time_ + time_ratio_ * static_cast<TimeT>(App::frame_time_ - last_frame_time_);
		
		// If the animation changed to one that starts later,
		// better to fast-forward to its start than to wait to catch up
		new_time = std::max(new_time, start_time);

		if (play_mode_ == PlayMode::PlayLooping)
		{
			const auto duration = end_time - start_time;
			const auto offset = new_time - start_time;
			if (root_motion_)
				root_motion_iteration_id_ += offset / duration;
			set_time(std::fmod(offset, duration) + start_time);
			if (previous_new_time > new_time)
			{
				nb_loop++;
			}
		}
		else // PlayMode::PlayOnce
		{
			set_time(std::min(new_time, end_time));
		}

		if ((play_mode_ == PlayMode::PlayLooping || play_mode_ == PlayMode::PlayOnce) && start)
		{
			if(previous_new_time == 0.){
				std::string str(current_item_csv);
				csv_.clear();
                timestamp_csv_.clear();
                csv_parser(str, ',', csv_weights_detected_, csv_weights_confirm_,
                                    vector_OF_rest_csv_);
				apply_alphas_csv(confirm_weights);
				std::map<std::string, std::vector<float>>::iterator iter = csv_.begin();
				count_timer_csv = iter->second.size();
				for (auto& it : csv_)
				{
					if (it.first == "timestamp")
						timestamp_csv_ = it.second;
				}
				incr_csv = 0;
				poids_frame = 1.;
				
			}

			TimeT csv_time = new_time;

			if (synchronize)
			{
				csv_time = fmod((new_time + (nb_loop * end_time)) , timestamp_csv_[timestamp_csv_.size() - 1]);
				incr_csv = incr_csv % (timestamp_csv_.size() - 1);
			}

			if (!synchronize && previous_new_time > new_time)
			{
				incr_csv = 0;
				poids_frame = 1;
			}

			while ((csv_time > timestamp_csv_[incr_csv]) && (incr_csv < count_timer_csv))
			{
				incr_csv++;
			}
			if (incr_csv - 1 > 0)
			{
				poids_frame =
					(csv_time - timestamp_csv_[incr_csv - 1]) / (timestamp_csv_[incr_csv] - timestamp_csv_[incr_csv - 1]);
			}

			if (incr_csv < count_timer_csv)
			{
				modeling::blending_csv(*selected_mesh_, incr_csv, poids_frame, csv_ , pos_attr_name, csv_weights_detected_);
			}
			
		}
		if (start)
		{
			previous_new_time = new_time;
		}
	}

	// Fast-forwards towards the next pose if an animation is selected.
	void advance_towards_next_pose(const TimeT& prec = Eigen::NumTraits<TimeT>::dummy_precision())
	{
		if (!selected_animation_ || !selected_animation_time_extrema_)
			return;

		const auto& [start_time, end_time] = *selected_animation_time_extrema_;
		cgogn_assert(start_time <= end_time);

		if (start_time == end_time || time_ < start_time || time_ >= end_time)
		{
			set_time(TimePoint::Start);
			return;
		}

		const std::set<TimeT> times = AnimationT::get_unique_keyframe_times(*selected_animation_);

		const auto next_pose_it = times.upper_bound(time_);
		cgogn_message_assert(next_pose_it != times.cbegin(), "Animation unique times incoherent with time bounds");

		if (!advance_pose_time_ratio_dependence_ || time_ratio_ >= 1.0)
		{
			set_time(*next_pose_it);
			return;
		}

		auto previous_pose_it = next_pose_it;
		--previous_pose_it;

		const TimeT& previous_pose_time = *previous_pose_it;
		const TimeT& next_pose_time = *next_pose_it;
		cgogn_message_assert(previous_pose_time >= start_time && next_pose_time <= end_time,
				"Animation unique times incoherent with time bounds");

		// Contrarily to advance_play in looping mode, overshooting is clamped
		const auto t = std::min(time_ + time_ratio_ * (next_pose_time - previous_pose_time), end_time);
		set_time(std::abs(next_pose_time - t) <= prec ? next_pose_time : t);
	}

	void show_time_controls()
	{
		if (!selected_animation_time_extrema_) // no pose
		{
				ImGui::TextUnformatted("Empty animation");
				return;
		}

		const auto& [start_time, end_time] = *selected_animation_time_extrema_;
		cgogn_assert(start_time <= end_time);

		if (start_time == end_time) // single pose
		{
			ImGui::LabelText("Time##L", "%.3f", static_cast<float>(time_));

			if (ImGui::Button("Set pose"))
				set_time(TimePoint::Start);

			return;
		}

		float t = static_cast<float>(time_);
		if (ImGui::SliderFloat("Time", &t, start_time, end_time))
			set_time(static_cast<TimeT>(t));

		if (ImGui::Button("<<"))
		{
			root_motion_iteration_id_ = 0;
			set_time(TimePoint::Start);
			if (current_item_csv != NULL)
			{
				incr_csv = 0;
				nb_loop = 0;
				poids_frame = 1;
				modeling::blending(*selected_mesh_,{pos_aus_[0]},{1},pos_attr_name);
			}
			previous_new_time = 0.;
			
		}
		show_tooltip_for_ui_above("Rewind");

		ImGui::SameLine();
		show_play_mode_button("><", "Play looping", PlayMode::PlayLooping);
		ImGui::SameLine();

		if (play_mode_ == PlayMode::PlayLooping // avoid flashing at end when looping
				|| time_ < end_time) // end not reached
			show_play_mode_button(">|", "Play once", PlayMode::PlayOnce);
		else if (show_button_and_tooltip("<>", "Play again"))
		{
			if (root_motion_)
				++root_motion_iteration_id_;
			set_play_mode(PlayMode::PlayOnce);
			set_time(TimePoint::Start);
		}

		ImGui::SameLine();
		show_play_mode_button("||", "Pause", PlayMode::Pause);
		ImGui::SameLine();

		if (ImGui::Button(">>"))
			set_time(TimePoint::End);
		show_tooltip_for_ui_above("Fast-forward");
	}

	void show_advanced_time_controls()
	{
		if (!selected_animation_time_extrema_ // no pose
				|| selected_animation_time_extrema_->first >= selected_animation_time_extrema_->second) // single pose
			return;

		float tr = static_cast<float>(time_ratio_);
		if (ImGui::DragFloat("Ratio", &tr, 0.0625f, 0.0f, std::numeric_limits<float>::max()))
			time_ratio_ = static_cast<TimeT>(tr);

		if (ImGui::Button(">"))
			advance_towards_next_pose();
		show_tooltip_for_ui_above("Next pose");
		ImGui::SameLine();
		ImGui::Checkbox("Ratio-dependent", &advance_pose_time_ratio_dependence_);

		ImGui::Separator();
	}

	void show_bone_color_generation_controls()
	{
		ImGui::RadioButton("Random##color_generation_mode", &color_generation_mode_, 0);
		ImGui::SameLine();
		ImGui::RadioButton("Position##color_generation_mode", &color_generation_mode_, 1);
		ImGui::SameLine();
		ImGui::RadioButton("Topo. depth##color_generation_mode", &color_generation_mode_, 2);

		if (ImGui::Button("Generate bone colors"))
		{
			const auto attribute = get_or_add_attribute<Vec3, Bone>(*selected_skeleton_,
					GENERATED_BONE_COLOR_ATTRIBUTE_NAME);
			const std::array<std::function<void()>, 3> generators
			{
				[&]{ Embedding::generate_bone_colors_random(*selected_skeleton_, *attribute); },
				[&]{
					if (selected_joint_position_)
						Embedding::generate_bone_colors_from_position(
								*selected_skeleton_, *selected_joint_position_, *attribute);
				},
				[&]{ Embedding::generate_bone_colors_from_topological_depth(*selected_skeleton_, *attribute); },
			};
			cgogn_assert(color_generation_mode_ >= 0 && color_generation_mode_ < generators.size());
			generators[color_generation_mode_]();
			mesh_provider_->emit_attribute_changed(*selected_skeleton_, attribute.get());

			// Alternative attribute (parent bone color)
			const auto attribute_p = get_or_add_attribute<Vec3, Bone>(*selected_skeleton_,
					GENERATED_PARENT_BONE_COLOR_ATTRIBUTE_NAME);
			for (const auto& bone : selected_skeleton_->bone_traverser_)
			{
				const auto& bone_index = index_of(*selected_skeleton_, bone);
				const auto& parent_bone = (*selected_skeleton_->bone_parent_)[bone_index];
				(*attribute_p)[bone_index] = (*attribute)[parent_bone.is_valid() ?
						index_of(*selected_skeleton_, parent_bone) : bone_index];
			}
			mesh_provider_->emit_attribute_changed(*selected_skeleton_, attribute_p.get());
		}
	}

	bool show_button_and_tooltip(const char* label, const char* tooltip_text, bool enabled = true)
	{
		bool res = false;

		if (!enabled)
			ImGui::BeginDisabled();

		if (ImGui::Button(label) && enabled)
			res = true;

		show_tooltip_for_ui_above(tooltip_text);

		if (!enabled)
			ImGui::EndDisabled();

		return res;
	}

	void show_play_mode_button(const char* label, const char* tooltip_text, PlayMode play_mode)
	{
		if (show_button_and_tooltip(label, tooltip_text, play_mode_ != play_mode))
			set_play_mode(play_mode);
	}

	static void show_tooltip_for_ui_above(const char* tooltip_text)
	{
		if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
			ImGui::SetTooltip("%s", tooltip_text); // format required to avoid Wformat-security
	}

	// Updates transforms and joint positions.
	// Assumes a skeleton and animation are selected as well as the corresponding transform f.
	void skeleton_update_embedding()
	{
		cgogn_assert(selected_skeleton_ && selected_animation_
				&& selected_bone_world_transform_ && selected_bone_world_transform_);

		std::optional<RootMotionData> root_motion_data{};
		if (root_motion_ && selected_animation_time_extrema_)
			root_motion_data = RootMotionData{*selected_animation_time_extrema_, root_motion_iteration_id_};

		Embedding::compute_all_transforms(time_, *selected_skeleton_, *selected_animation_,
				*selected_bone_local_transform_, *selected_bone_world_transform_, root_motion_data);

		signal_transform_attribute_changed_no_bb_update(selected_skeleton_, selected_bone_local_transform_.get());
		signal_transform_attribute_changed_no_bb_update(selected_skeleton_, selected_bone_world_transform_.get());

		// It's fine if no position attribute is selected, this should be called again as soon as one is
		if (selected_joint_position_)
			update_joint_positions_and_signal(mesh_provider_, *selected_skeleton_,
					*selected_bone_world_transform_, selected_joint_position_.get());
	}

	// Updates joint positions from the skeleton and transforms,
	// sending an update signal through the mesh provider.
	static void update_joint_positions_and_signal(
			MeshProvider<Skeleton>* mesh_provider,
			const Skeleton& as,
			const Attribute<TransformT>& world_transforms,
			Attribute<Vec3>* positions)
	{
		Embedding::compute_joint_positions(as, world_transforms, *positions);

		if (mesh_provider)
			mesh_provider->emit_attribute_changed(as, positions);
	}

	static void signal_transform_attribute_changed_no_bb_update(const Skeleton* m,
			Attribute<TransformT>* attribute)
	{
		boost::synapse::emit<MeshProvider<Skeleton>::attribute_changed>(m, attribute);
		boost::synapse::emit<MeshProvider<Skeleton>::attribute_changed_t<TransformT>>(m, attribute);
	}

	static std::string get_demangled_transform_name()
	{
		return boost::core::demangle(typeid(TransformT).name());
	}

	static std::string get_demangled_animation_name()
	{
		return boost::core::demangle(typeid(AnimationT).name());
	}

	bool can_update_embedding()
	{
		return selected_vertex_position_ && selected_vertex_weight_index_ && selected_vertex_weight_value_
				&& selected_bone_world_transform_;
	}

	void skin_update_embedding(bool force_update = false)
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
			skin_mesh_provider_->emit_attribute_changed(*selected_mesh_, selected_vertex_position_.get());

		embedding_dirty_ = false;
	}

public:
	static constexpr const char* GENERATED_BONE_COLOR_ATTRIBUTE_NAME = "generated_bone_color";
	static constexpr const char* GENERATED_PARENT_BONE_COLOR_ATTRIBUTE_NAME = "generated_parent_bone_color";

	static constexpr const bool USE_LBS_ = !std::is_same_v<TransformT, geometry::DualQuaternion>;

	PlayMode play_mode_ = PlayMode::Pause;
	decltype(App::frame_time_) last_frame_time_ = 0;
	TimeT time_ = TimeT{};
	TimeT time_ratio_ = 1.0;
	bool advance_pose_time_ratio_dependence_ = false;
	uint32 root_motion_iteration_id_ = 0;
	bool root_motion_ = false;
	int color_generation_mode_ = 0;
	Skeleton* selected_skeleton_ = nullptr;
	std::shared_ptr<Attribute<AnimationT>> selected_animation_ = nullptr;
	std::optional<std::pair<TimeT, TimeT>> selected_animation_time_extrema_ = {};
	std::shared_ptr<Attribute<Vec3>> selected_joint_position_ = nullptr;
	std::shared_ptr<Attribute<TransformT>> selected_bone_local_transform_ = nullptr;
	std::shared_ptr<Attribute<TransformT>> selected_bone_world_transform_ = nullptr;
	std::string local_transform_attribute_name_;
	std::string world_transform_attribute_name_;
	MeshProvider<Skeleton>* mesh_provider_ = nullptr;
	std::string bind_vertex_position_attribute_name_;
	bool save_vertex_position_on_first_bind_ = true;


	MeshProvider<MESH>* skin_mesh_provider_ = nullptr;
	bool auto_bind_mesh_ = true;
	bool auto_update_embedding_ = true;
	bool embedding_dirty_ = true;
	MESH* selected_mesh_ = nullptr;
	std::shared_ptr<AttributeSf<Vec3>> selected_vertex_position_ = nullptr;
	std::shared_ptr<AttributeSf<Vec3>> selected_bind_vertex_position_ = nullptr;
	bool selected_vertex_weight_index_valid_ = true;
	std::shared_ptr<AttributeSf<Vec4i>> selected_vertex_weight_index_ = nullptr;
	std::shared_ptr<AttributeSf<Vec4>> selected_vertex_weight_value_ = nullptr;
	bool normalize_weights_ = true;
	bool restore_vertex_position_on_unbind_ = false;
	std::shared_ptr<Attribute<TransformT>> selected_bind_bone_inv_world_transform_ = nullptr;
	std::unordered_map<AttributeSf<Vec3>*, std::shared_ptr<AttributeSf<Vec3>>> saved_vertex_positions_;
	std::array<std::shared_ptr<boost::synapse::connection>, 2> mesh_connections_ = {nullptr, nullptr};

	std::map<std::string, std::vector<float>> csv_;
    std::vector<std::shared_ptr<Attribute<Vec3>>> pos_aus_;

    std::vector<std::string> path_aus_;
	std::vector<std::string> path_csv_;
	std::string directory_;
	std::string pos_attr_name;

    Eigen::MatrixXd matrix_jacob;
    
    std::vector<float> weights;
	std::vector<float> timestamp_csv_;

	Eigen::VectorXd vector_OF_rest_cgogn_;
	Eigen::VectorXd vector_OF_rest_csv_;

	Eigen::VectorXd vector_confidence_lower_bound;
	Eigen::VectorXd vector_confidence_upper_bound;

    Eigen::MatrixXd csv_weights_detected_;
	Eigen::MatrixXd csv_weights_confirm_;
	int count_timer_csv = 0;
	float64 timer = 0.;
	float64 time_start = 0.;

	int incr_csv = 0;
	float poids_frame = 1.;
	TimeT previous_new_time = 0.;
	int nb_loop = 0;

	const char* current_item_csv = NULL;

    bool jacob_read = false;
	bool confirm_weights = true;
	bool synchronize = false;

	// slopes associated with each AUs
	Eigen::MatrixXd alphas;
	Eigen::MatrixXd betas;
};

} // namespace ui

} // namespace cgogn

#endif // CGOGN_MODULE_ANIMATION_FULL_BODY_CONTROLLER_H_
