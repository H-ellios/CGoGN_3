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

#ifndef CGOGN_IO_FBX_IMPORTER_H_
#define CGOGN_IO_FBX_IMPORTER_H_

#include <string>
#include <fstream>
#include <optional>
#include <unordered_map>

#include <cgogn/io/cgogn_io_export.h>

#include <cgogn/core/types/animation/animation_skeleton.h>
#include <cgogn/geometry/types/keyframed_animation.h>
#include <cgogn/geometry/types/rigid_transformation.h>
#include <cgogn/geometry/types/dual_quaternion.h>
#include <cgogn/io/surface/surface_import.h>
#include <cgogn/core/functions/attributes.h>
#include <cgogn/geometry/functions/bounding_box.h>

namespace cgogn
{

namespace io
{

class FbxImporterBase
{
public:
	enum RotationOrder
	{
		Base = 8, // using octal for a clearer representation than binary while keeping size down
		XYZ = 0012, // Z -> Y -> X
		XZY = 0021, // Y -> Z -> X
		YXZ = 0102, // Z -> X -> Y
		YZX = 0120, // X -> Z -> Y
		ZXY = 0201, // Y -> X -> Z
		ZYX = 0210, // X -> Y -> Z
	};
	static constexpr const std::array<RotationOrder, 6> ID_TO_ROTATION_ORDER{
		RotationOrder::ZYX,
		RotationOrder::ZXY,
		RotationOrder::YXZ,
		RotationOrder::YZX,
		RotationOrder::XZY,
		RotationOrder::XYZ,
	};

protected:
	using Skeleton = AnimationSkeleton;
	using ObjectId = uint64;
	using AnimTimeT = double;
	using AnimScalar = geometry::Scalar;
	using AnimationT = geometry::KeyframedAnimation<std::vector, AnimTimeT, AnimScalar>;

	class Properties
	{
	private:
		// Shared fields to save memory for non-overlapping usages
		// (usage of unions for this purpose is not ISO C++)

		std::array<std::optional<AnimScalar>, 3> aod3_0_, aod3_1_, aod3_2_, aod3_3_;
		std::optional<AnimScalar> z_0_;

	public:

		// Model

		constexpr std::array<std::optional<AnimScalar>, 3>& pre_rotation(){ return aod3_0_; }
		constexpr std::array<std::optional<AnimScalar>, 3>& post_rotation(){ return aod3_1_; }
		constexpr std::array<std::optional<AnimScalar>, 3>& lcl_translation(){ return aod3_2_; }
		constexpr std::array<std::optional<AnimScalar>, 3>& lcl_rotation(){ return aod3_3_; }
		constexpr std::optional<AnimScalar>& rotation_order(){ return z_0_; }

		constexpr const std::array<std::optional<AnimScalar>, 3>& pre_rotation() const { return aod3_0_; }
		constexpr const std::array<std::optional<AnimScalar>, 3>& post_rotation() const { return aod3_1_; }
		constexpr const std::array<std::optional<AnimScalar>, 3>& lcl_translation() const { return aod3_2_; }
		constexpr const std::array<std::optional<AnimScalar>, 3>& lcl_rotation() const { return aod3_3_; }
		constexpr const std::optional<AnimScalar>& rotation_order() const { return z_0_; }

		// AnimationCurveNode

		constexpr std::array<std::optional<AnimScalar>, 3>& d(){ return aod3_0_; }

		constexpr const std::array<std::optional<AnimScalar>, 3>& d() const { return aod3_0_; }

		// Utility

		template <size_t Size>
		static bool any_set(const std::array<std::optional<AnimScalar>, Size>& values)
		{
			for (size_t i = 0; i < Size; ++i) // compilers might be better able to optimize this
				if (values[i])
					return true;
			return false;
		}
	};

	struct Model
	{
		ObjectId id;
		std::string name;
		Properties properties;
	};

	struct MeshModel : public Model
	{
	};

	class LimbNodeModel : public Model
	{
	private:
		bool has_component(const size_t& offset) const;
		std::optional<AnimScalar> get_value(const size_t& index, const AnimTimeT& time) const;

	public:
		Skeleton::Bone bone;
		std::array<const AnimationT*, 6> animation;

		void set_animation(const std::array<const AnimationT*, 3>& anim, const std::string& transform_property_name);
		bool has_translation() const;
		bool has_rotation() const;
		Vec3 get_translation_or(const AnimTimeT& time, const Vec3& default_value) const;
		geometry::Quaternion get_rotation_or(const AnimTimeT& time,
				const geometry::Quaternion& pre_rotation, const geometry::Quaternion& post_rotation,
				const geometry::Quaternion& default_value) const;
	};

	struct Geometry
	{
		ObjectId id;
		std::string name;
		SurfaceImportData data;
	};

	struct Deformer
	{
		ObjectId id;
		std::string name;
	};

	struct ClusterDeformer : public Deformer
	{
		std::vector<std::pair<uint32, AnimScalar>> weights;
	};

	struct AnimationCurve
	{
		ObjectId id;
		AnimScalar default_value;
		AnimationT animation;
	};

	struct AnimationCurveNode
	{
		ObjectId id;
		Properties properties;
		std::array<const AnimationT*, 3> animation;

		void set_animation(const AnimationT* anim, const std::string& axis_property_name);
	};

private:
	// https://help.autodesk.com/view/FBX/2015/ENU/?guid=__cpp_ref_class_fbx_anim_curve_html
	static constexpr const AnimTimeT ANIM_TIME_RATIO = 1.0 / 46186158000l;

public:
	template <bool WarnUnsupported = true, int DefaultId = -1>
	static std::conditional_t<DefaultId >= 0, const RotationOrder&, std::optional<RotationOrder>> get_rotation_order(
			const size_t& id)
	{
		if (id < ID_TO_ROTATION_ORDER.size())
			return ID_TO_ROTATION_ORDER[id];

		if constexpr (WarnUnsupported)
		{
			if (id == ID_TO_ROTATION_ORDER.size())
				std::cout << "Warning: spheric rotation order is unsupported" << std::endl;
			else
				std::cout << "Warning: unrecognized rotation order ID " << id << std::endl;
		}

		static_assert(DefaultId < 0 || DefaultId < ID_TO_ROTATION_ORDER.size());

		if constexpr (DefaultId >= 0)
			return ID_TO_ROTATION_ORDER[DefaultId];
		else
			return std::optional<RotationOrder>{};
	}

protected:

	FbxImporterBase(bool load_surfaces, bool load_skeleton)
			: load_surfaces_(load_surfaces), load_skeleton_(load_skeleton)
	{
	}

	/// @brief Opens and parses an FBX file
	/// @param path the file path to open
	void read(const std::string& path);

	/// @brief Retrieves a pointer to the parent `LimbNodeModel` from the ID of one of its children
	/// @param child_id the ID of the child node
	/// @return a pointer to the parent `LimbNodeModel`, or `nullptr` if it wasn't found
	const LimbNodeModel* get_parent_bone(const ObjectId& child_id) const;

	/// @brief Resolves a name string potentially containing escape sequences to the desired string.
	/// Reverse engineering of how Maya reads files saved in Blender with names containing special characters,
	/// and how Blender reads those files back when escape sequences are modified,
	/// shows special characters are escaped with the format `FBXASC%03d`. For example, `"` is `FBXASC034`.
	/// This method currently leaves the string unchanged, and is present for forwards compatibility and clarity.
	/// @param name_with_escape_sequences the raw name read from the FBX file
	/// @return the processed name
	static std::string resolve_name(const std::string& name_with_escape_sequences);

	static geometry::Quaternion from_euler(const std::array<std::optional<AnimScalar>, 3>& xyz,
			const RotationOrder& rotation_order = RotationOrder::ZYX);

	const Vec3 get_default_translation(const std::string& template_key,
			const std::array<std::optional<AnimScalar>, 3>& (Properties::* f)() const) const;
	const geometry::Quaternion get_default_rotation(const std::string& template_key,
			const std::array<std::optional<AnimScalar>, 3>& (Properties::* f)() const,
			const RotationOrder& rotation_order = RotationOrder::ZYX) const;
	const RotationOrder& get_default_rotation_order(const std::string& template_key) const;

	/// @brief Sets missing values for a property in `p` from `other_values`
	/// @param p the property to fill (existing values take priority over those from `other_values`)
	/// @param other_values the values to use
	/// @param property_name the name of the property
	/// @param warn_unequal whether or not to print a warning to stdout if the values are both present and different
	template <typename OtherContainer>
	void set_missing_values(Properties& p, const OtherContainer& other_values,
			const std::string& property_name, bool warn_unequal)
	{
		auto pi = PROPERTY_INFO_.find(property_name);
		if (pi == PROPERTY_INFO_.cend())
			return;

		const auto& [ptr, size] = pi->second(p);

		cgogn_assert(other_values.size() == size);

		for (size_t i = 0; i < size; ++i)
		{
			if (!ptr[i])
				ptr[i] = other_values[i];
			else if (warn_unequal && other_values[i] && *other_values[i] != *ptr[i])
				std::cout << "Warning: inconsistent values " << *ptr[i] << " and " << *other_values[i]
						<< " for property " << property_name << std::endl;
		}
	}

private:
	void read_root(std::istream& is);
	void read_objects_node(std::istream& is);
	void read_objects_model_subnode(std::istream& is);
	void read_objects_geometry_subnode(std::istream& is);
	void read_objects_geometry_vertices_subnode(std::istream& is, SurfaceImportData& d);
	void read_objects_geometry_polygon_vertex_index_subnode(std::istream& is, SurfaceImportData& d);
	void read_objects_deformer_subnode(std::istream& is);
	void read_objects_animation_curve_subnode(std::istream& is);
	void read_objects_animation_curve_node_subnode(std::istream& is);
	void read_object_attributes(std::istream& is, ObjectId& id,
			std::string* name = nullptr, std::string* type = nullptr);
	void read_connections_node(std::istream& is);
	void read_fbx_header_extension_node(std::istream& is);
	void read_definitions_node(std::istream& is);
	void read_definitions_object_type_subnode(std::istream& is);
	void read_properties_70_subnode(std::istream& is, Properties& p);
	void read_property(std::istream& is, Properties& p);
	bool read_node(std::istream& is, std::function<void(const std::string&)> read_value,
			const std::optional<size_t>& subnode_key_reserve_size = {});
	bool read_array(std::istream& is,
			const std::function<void(uint32)>& on_size, const std::function<uint32()>& read_values);
	void read_integer_and_warn_if_not_expected(std::istream& is, const std::string& nature, int expected_version);
	void skip_node(std::istream& is);
	void skip_value(std::istream& is);
	void skip_through_character(std::istream& is, char c);
	std::optional<char> skip_through_characters(std::istream& is, const std::string& characters);

	template <typename T, size_t Size>
	static constexpr std::pair<T*, size_t> std_array_data_and_size(std::array<T, Size>& arr)
	{
		return std::make_pair(arr.data(), Size);
	}

protected:
	std::unordered_map<std::string, Properties> property_templates_;
	std::vector<MeshModel> models_mesh_;
	std::vector<LimbNodeModel> models_limb_node_;
	std::vector<Geometry> geometries_;
	std::vector<Deformer> deformers_skin_;
	std::vector<ClusterDeformer> deformers_cluster_;
	std::vector<AnimationCurve> animation_curves_;
	std::vector<AnimationCurveNode> animation_curve_nodes_;
	std::vector<std::pair<ObjectId, ObjectId>> connections_oo_;
	std::vector<std::tuple<ObjectId, ObjectId, std::string>> connections_op_;

private:
	/// @brief Mapping from FBX property name to a function returning the pointer and size (in elements, not in bytes)
	///        of the corresponding data in an instance of `Properties`
	static inline const std::unordered_map<std::string,
			std::function<std::pair<std::optional<AnimScalar>*, size_t>(Properties&)>> PROPERTY_INFO_ = {
		std::make_pair("PreRotation", [](Properties& p){ return std_array_data_and_size(p.pre_rotation()); }),
		std::make_pair("PostRotation", [](Properties& p){ return std_array_data_and_size(p.post_rotation()); }),
		std::make_pair("Lcl Translation", [](Properties& p){ return std_array_data_and_size(p.lcl_translation()); }),
		std::make_pair("Lcl Rotation", [](Properties& p){ return std_array_data_and_size(p.lcl_rotation()); }),
		std::make_pair("d|X", [](Properties& p){ return std::make_pair(p.d().data(), 1); }),
		std::make_pair("d|Y", [](Properties& p){ return std::make_pair(p.d().data() + 1, 1); }),
		std::make_pair("d|Z", [](Properties& p){ return std::make_pair(p.d().data() + 2, 1); }),
		std::make_pair("RotationOrder", [](Properties& p){ return std::make_pair(&p.rotation_order(), 1); }),
	};

	bool load_surfaces_;
	bool load_skeleton_;
};

class FbxImporter : public FbxImporterBase
{
public:
	template <typename T, typename U>
	using Map = typename std::unordered_map<T, U>;

private:
	using FbxImporterBase::FbxImporterBase; // inherit constructor

private:
	template <typename T>
	auto add_animation_attributes(Skeleton& skeleton, const std::string& candidate_name)
	{
		std::string name = candidate_name;

		// Ensure name uniqueness
		for (size_t i = 0;
				get_attribute<T, Skeleton::Bone>(skeleton, name)
						|| get_attribute<T, Skeleton::Bone>(skeleton, name + "_bind");
				name = candidate_name + std::to_string(i++));

		auto& bind_attr = *add_attribute<T, Skeleton::Bone>(skeleton, name + "_bind");
		auto& anim_attr = *add_attribute<T, Skeleton::Bone>(skeleton, name);

		return std::make_pair(std::ref(bind_attr), std::ref(anim_attr));
	}

	template <typename Surface>
	auto get_and_init_skinning_attributes(Surface& surface)
	{
		using Vertex = typename mesh_traits<Surface>::Vertex;

		auto& weight_index_attr = *get_or_add_attribute<geometry::Vec4i, Vertex>(surface, "weight_index");
		auto& weight_value_attr = *get_or_add_attribute<geometry::Vec4, Vertex>(surface, "weight_value");

		parallel_foreach_cell(surface, [&](Vertex v)
		{
			const auto i = index_of(surface, v);
			weight_index_attr[i] = {-1, -1, -1, -1};
			weight_value_attr[i] = geometry::Vec4::Zero();
			return true;
		});

		return std::make_pair(std::ref(weight_index_attr), std::ref(weight_value_attr));
	}

	template <typename Surface>
	void load_geometry(Surface& surface, Geometry& geometry)
	{
		// Import geometry to surface type
		import_surface_data(surface, geometry.data);

		// Find and compute skinning information

		auto [weight_index_attr, weight_value_attr] = get_and_init_skinning_attributes(surface);

		std::vector<std::pair<const ClusterDeformer*, Skeleton::Bone>> cluster_deformers;

		for (const auto& [skin_id, geometry_id] : connections_oo_)
		{
			if (geometry_id != geometry.id)
				continue;

			if (std::find_if(deformers_skin_.cbegin(), deformers_skin_.cend(),
					[&](const Deformer& d) { return d.id == skin_id; }) == deformers_skin_.cend())
				continue;

			for (const auto& [cluster_id, skin_id_] : connections_oo_)
			{
				if (skin_id_ != skin_id)
					continue;

				const auto deformer_it = std::find_if(deformers_cluster_.cbegin(), deformers_cluster_.cend(),
						[&](const ClusterDeformer& d) { return d.id == cluster_id; });

				if (deformer_it != deformers_cluster_.cend())
					cluster_deformers.emplace_back(&*deformer_it, INVALID_INDEX);
			}
		}

		if (cluster_deformers.empty())
			return;

		for (const auto& [bone_model_id, cluster_id] : connections_oo_)
		{
			const auto bone_model_it = std::find_if(models_limb_node_.cbegin(), models_limb_node_.cend(),
					[&](const LimbNodeModel& model) { return model.id == bone_model_id; });

			if (bone_model_it == models_limb_node_.cend())
				continue;

			const auto cluster_it = std::find_if(cluster_deformers.begin(), cluster_deformers.end(),
					[&](const std::pair<const ClusterDeformer*, Skeleton::Bone>& entry)
							{ return entry.first->id == cluster_id; });

			if (cluster_it == cluster_deformers.end())
				continue;

			cluster_it->second = bone_model_it->bone;
		}

		for (const auto& [deformer, bone] : cluster_deformers)
			for (const auto& [vertex_index, weight_value] : deformer->weights)
			{
				const auto& v = geometry.data.vertex_id_after_import_[vertex_index];

				for (size_t i = 0; i < 4; ++i)
					if (weight_index_attr[v][i] == -1)
					{
						weight_index_attr[v][i] = bone;
						weight_value_attr[v][i] = weight_value;
						break;
					}
			}
	}

	template <typename Surface>
	void load_surfaces(Map<std::string, Surface*>& surfaces, bool normalized)
	{
		using Vertex = typename mesh_traits<Surface>::Vertex;

		for (const MeshModel& m : models_mesh_)
		{
			Surface* surface = new Surface{};
			std::string name = resolve_name(m.name);
			decltype(surfaces.try_emplace(name, surface).first) it;
			bool inserted = false;

			while (std::tie(it, inserted) = surfaces.try_emplace(name, surface), !inserted)
				name += '_'; // disambiguate however we can

			for (const auto& c : connections_oo_)
			{
				ObjectId other_id;
				if (c.first == m.id)
					other_id = c.second;
				else if (c.second == m.id)
					other_id = c.first;
				else
					continue;

				const auto geometry_it = std::find_if(
						geometries_.begin(), geometries_.end(),
						[&](const Geometry& g){ return g.id == other_id; });

				if (geometry_it != geometries_.end())
					load_geometry(*surface, *geometry_it);
			}

			// @TODO connections_oo_ with parents to apply their transform, if any

			if (normalized)
				if (auto vertex_position = get_attribute<Vec3, Vertex>(*surface, "position"))
					geometry::rescale(*vertex_position, 1);
		}
	}

	void load_bones(Skeleton& skeleton);
	void associate_animations_to_bones();
	void load_animations(Skeleton& skeleton);
	Skeleton* load_skeleton();

public:
	FbxImporter() = delete;

	/// @brief Opens and parses an FBX file, filling the maps with objects inside
	/// @param path the file path to open
	/// @param surfaces a map to fill with surfaces described in the file
	/// @param skeleton an optional that would be set with the animation skeleton that may be described in the file
	/// @param load_surfaces whether or not to actually read surfaces
	/// @param load_skeleton whether or not to actually read skeletons
	/// @param normalized whether or not to normalize the positions for each surface (can offset it from bones)
	template <typename Surface>
	static void load(const std::string& path,
			Map<std::string, Surface*>& surfaces,
			std::optional<Skeleton*>& skeleton,
			bool load_surfaces = true, bool load_skeleton = true, bool normalized = false)
	{
		FbxImporter importer{load_surfaces, load_skeleton};
		importer.read(path);

		if (load_skeleton && !(skeleton = importer.load_skeleton()).value())
			skeleton = {}; // convert nullptr to empty optional

		if (load_surfaces)
			importer.load_surfaces(surfaces, normalized);
	}
};

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_FBX_IMPORTER_H_
