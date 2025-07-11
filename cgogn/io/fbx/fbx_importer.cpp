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

#include <variant>

#include <cgogn/io/utils.h>
#include <cgogn/io/fbx/fbx_importer.h>

using namespace std::literals::string_literals;

namespace cgogn
{

namespace io
{


// FbxImporterBase


void FbxImporterBase::read(const std::string& path)
{
	Scoped_C_Locale loc;

	std::ifstream is(path.c_str(), std::ios::in);
	std::istream::sentry se(is, true); // clean input stream

	read_root(is);
}

// Reads an FBX input from the beginning (node level)
void FbxImporterBase::read_root(std::istream& is)
{
	std::string node_key;
	char c;

	node_key.reserve(32);

	while (is.get(c))
	{
		if (std::isspace(c)) // ignore whitespace
			continue;

		if (c == ':') // end of key
		{
			if (node_key == "Objects"s)
				read_objects_node(is);
			else if (node_key == "Connections"s)
				read_connections_node(is);
			else if (node_key == "FBXHeaderExtension"s)
				read_fbx_header_extension_node(is);
			else if (node_key == "Definitions"s)
				read_definitions_node(is);
			else
				skip_node(is);

			node_key.clear();
		}
		else if (c == ';' && node_key.empty()) // semicolon on start of line is comment
			skip_through_character(is, '\n');
		else // read key
			node_key += c;
	}
}

// Reads an Objects node from past the declaring colon through its closing brace
void FbxImporterBase::read_objects_node(std::istream& is)
{
	if (!read_node(is,
			[&](const std::string& subnode_key) {
				if (subnode_key == "Model"s)
					read_objects_model_subnode(is);
				else if (subnode_key == "Geometry"s)
					read_objects_geometry_subnode(is);
				else if (subnode_key == "Deformer"s)
					read_objects_deformer_subnode(is);
				else if (subnode_key == "AnimationCurve"s)
					read_objects_animation_curve_subnode(is);
				else if (subnode_key == "AnimationCurveNode"s)
					read_objects_animation_curve_node_subnode(is);
				else
					skip_value(is);
			}, 32))
		std::cout << "Warning: invalid syntax for Objects node" << std::endl;
}

// Reads a Model subnode (inside an Objects node) from past the declaring colon through its closing brace
void FbxImporterBase::read_objects_model_subnode(std::istream& is)
{
	std::variant<Model, MeshModel, LimbNodeModel> model_variant{}; // default-constructed to the Model type
	auto* model = std::get_if<Model>(&model_variant); // necessarily non-null because we know that variant is assigned
	std::string type;

	read_object_attributes(is, model->id, &model->name, &type);

	if (type == "Mesh"s)
	{
		if (!load_surfaces_)
		{
			skip_node(is);
			return;
		}
		model_variant = MeshModel{*model};
		model = std::get_if<MeshModel>(&model_variant);
	}
	else if (type == "LimbNode"s)
	{
		if (!load_skeleton_)
		{
			skip_node(is);
			return;
		}
		model_variant = LimbNodeModel{*model};
		model = std::get_if<LimbNodeModel>(&model_variant);
	}

	if (!read_node(is,
			[&](const std::string& subnode_key) {
				if (subnode_key == "Version"s)
					read_integer_and_warn_if_not_expected(is, "model version", 232);
				else if (subnode_key == "Properties70"s)
					read_properties_70_subnode(is, model->properties);
				else
					skip_value(is);
			}))
		std::cout << "Warning: invalid syntax for model " << model->name << std::endl;
	else if (auto* m = std::get_if<LimbNodeModel>(&model_variant))
		models_limb_node_.push_back(std::move(*m));
	else if (auto* m = std::get_if<MeshModel>(&model_variant))
		models_mesh_.push_back(std::move(*m));

	// Other model types are ignored because we don't have a use for them yet
	// It would simply be a matter of having a vector of plain models to push to
}

// Reads a Geometry subnode (inside an Objects node) from past the declaring colon through its closing brace
void FbxImporterBase::read_objects_geometry_subnode(std::istream& is)
{
	Geometry geometry{};
	std::string type;

	read_object_attributes(is, geometry.id, &geometry.name, &type);

	if (type != "Mesh"s)
		std::cout << "Warning: expected geometry of type Mesh for geometry " << geometry.name
				<< " but got " << type << std::endl;

	if (read_node(is,
			[&](const std::string& subnode_key) {
				if (subnode_key == "Vertices"s)
					read_objects_geometry_vertices_subnode(is, geometry.data);
				else if (subnode_key == "PolygonVertexIndex"s)
					read_objects_geometry_polygon_vertex_index_subnode(is, geometry.data);
				else if (subnode_key == "GeometryVersion"s)
					read_integer_and_warn_if_not_expected(is, "geometry version", 124);
				else
					skip_value(is);
			}), 32)
		geometries_.push_back(std::move(geometry));
	else
		std::cout << "Warning: invalid syntax for geometry " << geometry.name << std::endl;
}

// Reads a Vertices subnode (inside a Geometry subnode in an Objects node)
// from past the declaring colon through its closing brace
void FbxImporterBase::read_objects_geometry_vertices_subnode(std::istream& is, SurfaceImportData& d)
{
	Vec3 v = Vec3::Zero();
	int coef = 0;

	if (read_array(is,
			[&](uint32 size){
				d.nb_vertices_ = size / 3; // number in the file is number of coefficients
				d.vertex_position_.reserve(d.nb_vertices_);
			},
			[&]{
				is >> v[coef];
				if (++coef == 3)
				{
					d.vertex_position_.push_back(v);
					v = Vec3::Zero();
					coef = 0;
				}
				return 1;
			}))
		cgogn_assert(d.nb_vertices_ == d.vertex_position_.size());
	else
		std::cout << "Warning: invalid syntax for Vertices subnode" << std::endl;
}

// Reads a PolygonVertexIndex subnode (inside a Geometry subnode in an Objects node)
// from past the declaring colon through its closing brace
void FbxImporterBase::read_objects_geometry_polygon_vertex_index_subnode(std::istream& is, SurfaceImportData& d)
{
	int nb_vertices = 0;

	if (!read_array(is,
			[&](uint32 size){ d.faces_vertex_indices_.reserve(size); },
			[&]{
				int32 id;
				is >> id;
				if (id < 0)
				{
					id = -1 - id;
					++d.nb_faces_;
					d.faces_nb_vertices_.push_back(nb_vertices + 1);
					nb_vertices = 0;
				}
				else
					++nb_vertices;
				d.faces_vertex_indices_.push_back(id);
				return 1;
			}))
		std::cout << "Warning: invalid syntax for PolygonVertexIndex subnode" << std::endl;
}

// Reads a Deformer subnode (in an Objects node) from past the declaring colon through its closing brace
void FbxImporterBase::read_objects_deformer_subnode(std::istream& is)
{
	int32 nb_vertices = -1;
	ClusterDeformer deformer;
	std::string type;

	const auto on_size = [&](uint32 size){
		cgogn_assert(nb_vertices == -1 || static_cast<uint32>(nb_vertices) == size);
		nb_vertices = size;
		deformer.weights.reserve(nb_vertices);
	};

	const auto get_weight = [&](const uint32& i) -> std::pair<uint32, AnimScalar>& {
		if (deformer.weights.size() <= i)
			deformer.weights.resize(static_cast<size_t>(i) + 1, {uint32(-1), 0.0});
		return deformer.weights[i];
	};

	const auto read_cluster_subnode = [&](const std::string& subnode_key) {
		int32 vertex_id = 0;
		if (subnode_key == "Indexes"s)
		{
			if (read_array(is, on_size,
			[&]{
				is >> get_weight(vertex_id++).first;
				return 1;
			}))
				cgogn_assert(vertex_id == nb_vertices);
			else
				std::cout << "Warning: invalid skinning weight indices syntax" << std::endl;
		}
		else if (subnode_key == "Weights"s)
		{
			if (read_array(is, on_size,
			[&]{
				is >> get_weight(vertex_id++).second;
				return 1;
			}))
				cgogn_assert(vertex_id == nb_vertices);
			else
				std::cout << "Warning: invalid skinning weight values syntax" << std::endl;
		}
		else
			skip_value(is); // we ignore transforms because we already get a bind pose to work with
	};

	read_object_attributes(is, deformer.id, &deformer.name, &type);

	if (type == "Cluster"s)
	{
		if (read_node(is, read_cluster_subnode))
			deformers_cluster_.push_back(std::move(deformer));
		else
			std::cout << "Warning: invalid deformer syntax" << std::endl;
	}
	else if (type == "Skin"s)
	{
		deformers_skin_.push_back(static_cast<Deformer>(std::move(deformer)));
		skip_value(is);
	}
	else
	{
		std::cout << "Warning: unrecognized deformer type " << type << std::endl;
		skip_value(is);
	}
}

// Reads an AnimationCurve subnode (in an Objects node) from past the declaring colon through its closing brace
void FbxImporterBase::read_objects_animation_curve_subnode(std::istream& is)
{
	int32 nb_keys = -1;
	AnimationCurve curve{};

	const auto on_size = [&](uint32 size){
		cgogn_assert(nb_keys == -1 || static_cast<uint32>(nb_keys) == size);
		nb_keys = size;
		curve.animation.reserve(nb_keys);
	};

	const auto get_keyframe = [&](const uint32& i) -> geometry::AnimationKeyframe<AnimTimeT, AnimScalar>& {
		if (curve.animation.size() <= i)
			curve.animation.resize(static_cast<size_t>(i) + 1, {0.0, 0.0});
		return curve.animation[i];
	};

	const auto read_subnode = [&](const std::string& subnode_key) {
		int32 key_id = 0;
		if (subnode_key == "KeyTime"s)
		{
			if (read_array(is, on_size,
			[&]{
				uint64 time;
				is >> time;
				get_keyframe(key_id++).time_ = time * ANIM_TIME_RATIO;
				return 1;
			}))
				cgogn_assert(key_id == nb_keys);
			else
				std::cout << "Warning: invalid key times syntax" << std::endl;
		}
		else if (subnode_key == "KeyValueFloat"s)
		{
			if (read_array(is, on_size,
			[&]{
				is >> get_keyframe(key_id++).value_;
				return 1;
			}))
				cgogn_assert(key_id == nb_keys);
			else
				std::cout << "Warning: invalid key values syntax" << std::endl;
		}
		else if (subnode_key == "Default"s)
			is >> curve.default_value;
		else
			skip_value(is);
	};

	read_object_attributes(is, curve.id);

	if (read_node(is, read_subnode))
		animation_curves_.push_back(std::move(curve));
	else
		std::cout << "Warning: invalid animation curve syntax" << std::endl;
}

// Reads an AnimationCurveNode subnode (in an Objects node) from past the declaring colon through its closing brace
void FbxImporterBase::read_objects_animation_curve_node_subnode(std::istream& is)
{
	AnimationCurveNode node{};

	read_object_attributes(is, node.id);

	if (read_node(is,
			[&](const std::string& subnode_key)
			{
				if (subnode_key == "Properties70"s)
					read_properties_70_subnode(is, node.properties);
				else
					skip_value(is);
			}))
		animation_curve_nodes_.push_back(std::move(node));
	else
		std::cout << "Warning: invalid animation curve node syntax" << std::endl;
}

void FbxImporterBase::read_object_attributes(std::istream& is, ObjectId& id,
		std::string* name, std::string* type)
{
	enum Stage { START, NAME, MIDDLE, TYPE, END };
	int stage = START;
	char c;

	is >> id;

	while (stage < END && is.get(c))
	{
		switch (stage)
		{
		case START:
		case MIDDLE:
			if (c == '"')
				++stage;
			break;
		case NAME:
			if (c == '"')
				++stage;
			else if (name)
				*name += c;
			break;
		case TYPE:
			if (c == '"')
				++stage;
			else if (type)
				*type += c;
			break;
		}
	}
}

// Reads a Connections node from past the declaring colon through its closing brace
void FbxImporterBase::read_connections_node(std::istream& is)
{
	int commas = -1; // -1 is flag for not having read a colon
	ObjectId ids[2] = {ObjectId(-1), ObjectId(-1)};
	std::string property_name;
	bool expect_property = false;
	bool in_quotes = false;
	char c, c_ = '\0';

	for (int depth = 0; is.get(c);)
	{
		switch (c)
		{
		case '{':
			cgogn_assert(depth == 0 && commas == -1); // no subnode
			++depth;
			break;
		case '}':
			--depth;
			cgogn_assert(depth == 0 && commas == -1);
			return;
		case '"':
			in_quotes = !in_quotes;
			if (!in_quotes)
			{
				if (!property_name.empty())
				{
					connections_op_.emplace_back(ids[0], ids[1], property_name);
					property_name.clear();
					expect_property = false;
					commas = -1;
				}
			}
			break;
		case ',':
			switch (commas)
			{
			case 0:
			case 1:
				is >> ids[commas];
				++commas;
				if (!expect_property && commas == 2)
				{
					connections_oo_.emplace_back(ids[0], ids[1]);
					commas = -1;
				}
				break;
			case 2:
				cgogn_assert(expect_property);
				++commas;
				break;
			default:
				cgogn_assert_not_reached("Previous commas not in [0, 2] like expected");
			}
			break;
		case ';':
			cgogn_assert(!in_quotes);
			skip_through_character(is, '\n');
			break;
		case 'P':
			if (commas == 0) // assume "OP"
			{
				cgogn_assert(in_quotes && c_ == 'O');
				expect_property = true;
				break;
			}
			[[fallthrough]];
		default:
			if (in_quotes && expect_property)
			{
				cgogn_assert(commas == 3);
				property_name += c;
			}
			else if (!in_quotes && c == ':')
			{
				cgogn_assert(commas == -1 && c_ == 'C');
				commas = 0;
			}
#ifdef CGOGN_DEBUG
			else
				c_ = c;
#endif
		}
	}

	std::cout << "Warning: invalid syntax for Connections node" << std::endl;
}

// Reads a FBXHeaderExtension node from past the declaring colon through its closing brace
void FbxImporterBase::read_fbx_header_extension_node(std::istream& is)
{
	if (!read_node(is,
			[&](const std::string& subnode_key) {
				if (subnode_key == "FBXVersion"s)
					read_integer_and_warn_if_not_expected(is, "FBX version", 7700);
				else
					skip_value(is);
			}, 32))
		std::cout << "Warning: invalid syntax for FBXHeaderExtension node" << std::endl;
}

// Reads a Definitions node from past the declaring colon through its closing brace
void FbxImporterBase::read_definitions_node(std::istream& is)
{
	if (!read_node(is,
			[&](const std::string& subnode_key) {
				if (subnode_key == "Version"s)
					read_integer_and_warn_if_not_expected(is, "definitions version", 100);
				else if (subnode_key == "ObjectType"s)
					read_definitions_object_type_subnode(is);
				else
					skip_value(is);
			}))
		std::cout << "Warning: invalid syntax for Definitions node" << std::endl;
}

void FbxImporterBase::read_definitions_object_type_subnode(std::istream& is)
{
	Properties properties{};
	std::string object_type;
	char c;

	const auto read_subsubnode = [&](const std::string& subnode_key)
	{
		if (subnode_key == "Properties70"s)
			read_properties_70_subnode(is, properties);
		else
			skip_value(is);
	};

	const auto read_subnode = [&](const std::string& subnode_key)
	{
		if (subnode_key == "PropertyTemplate"s)
		{
			// Skip quoted "FbxNode"
			skip_through_character(is, '"');
			skip_through_character(is, '"');

			if (!read_node(is, read_subsubnode))
				std::cout << "Warning: invalid syntax for PropertyTemplate subnode" << std::endl;
		}
		else
			skip_value(is);
	};

	skip_through_character(is, '"');

	while (is.get(c))
		if (c == '"')
			break;
		else
			object_type += c;

	if (is && read_node(is, read_subnode))
		property_templates_[std::move(object_type)] = std::move(properties);
	else
		std::cout << "Warning: invalid animation curve node syntax" << std::endl;
}

// Reads a Properties70 subnode from past the declaring colon through its closing brace
void FbxImporterBase::read_properties_70_subnode(std::istream& is, Properties& p)
{
	char c, c_ = '\0';

	for (int depth = 0; is.get(c);)
	{
		switch (c)
		{
		case '{':
			cgogn_assert(depth == 0); // no subnode
			++depth;
			break;
		case '}':
			--depth;
			cgogn_assert(depth == 0);
			return;
		case ':':
			cgogn_assert(c_ == 'P');
			read_property(is, p);
			c = '\0';
			break;
		case ';':
			cgogn_assert(c_ == '\0');
			skip_through_character(is, '\n');
			break;
		default:
			if (!std::isspace(c))
				c_ = c;
		}
	}

	std::cout << "Warning: invalid syntax for Properties70 subnode" << std::endl;
}

// Reads a property through the end of its line
void FbxImporterBase::read_property(std::istream& is, Properties& p)
{
	std::string name;
	char c;

	enum Stage { START, NAME, NAME_TO_TYPE, TYPE, TYPE_TO_TYPE_NAME, TYPE_NAME, TYPE_NAME_TO_LAST, LAST, END };
	int stage = START;

	while (stage < END && is.get(c))
	{
		switch (stage)
		{
		case START:
		case NAME_TO_TYPE:
		case TYPE_TO_TYPE_NAME:
		case TYPE_NAME_TO_LAST:
			if (c == '"')
				++stage;
			break;
		case NAME:
			if (c == '"')
				++stage;
			else
				name += c;
			break;
		case TYPE:
		case TYPE_NAME:
		case LAST:
			if (c == '"')
				++stage;
			break;
		}
	}

	auto pi = PROPERTY_INFO_.find(name);
	if (pi != PROPERTY_INFO_.cend())
	{
		const auto& [ptr, size] = pi->second(p);
		for (int i = 0; i < size && !is.fail() && (skip_through_character(is, ','), is); ++i)
		{
			std::remove_reference_t<decltype(*ptr[i])> value;
			is >> value;
			ptr[i].emplace(std::move(value));
		}
	}

	if (skip_through_characters(is, "\n}").value_or('\0') == '}')
		is.unget();
}

// Reads a (sub)node from past the declaring colon through its closing brace
// `subnode_key_reserve_size` is recommended to be set if and only if keys are expected to be larger than 15 (*)
// Returns whether the syntax was correct
// (*) This is because most implementations store a union of either a small local buffer or a pointer to the free store
//	GCC: see `_S_local_capacity` and `_M_local_buf` in `basic_string` (bits/basic_string.h)
//	MSVC: see `_BUF_SIZE` and `_Bxty` in `_String_val` (xstring)
bool FbxImporterBase::read_node(std::istream& is, std::function<void(const std::string&)> read_value,
		const std::optional<size_t>& subnode_key_reserve_size)
{
	std::string subnode_key;
	char c;

	if (subnode_key_reserve_size)
		subnode_key.reserve(*subnode_key_reserve_size);

	for (int depth = 0; is.get(c);)
	{
		switch (c)
		{
		case '{':
			cgogn_assert(depth == 0); // no subnode without key
			++depth;
			break;
		case '}':
			--depth;
			cgogn_assert(depth == 0);
			return true;
		case ':':
			read_value(subnode_key);
			subnode_key.clear();
			break;
		case ';':
			if (subnode_key.empty())
				skip_through_character(is, '\n');
			break;
		default:
			if (!std::isspace(c))
				subnode_key += c;
		}
	}

	return false;
}

// Reads an array subnode from past the declaring colon through its closing braces
// `on_size` is provided with the number of scalars announced in the file
// `read_values` should return the amount of scalars read (normally 1)
// Returns whether the syntax was correct
bool FbxImporterBase::read_array(std::istream& is,
		const std::function<void(uint32)>& on_size, const std::function<uint32()>& read_values)
{
	bool in_array = false;
	int32 announced_size = -1, actual_size = 0;
	char c, c_ = '\0';

	for (int depth = 0; is.get(c);)
	{
		switch (c)
		{
		case '{':
			cgogn_assert(depth == 0); // no subnode
			++depth;
			break;
		case '}':
			--depth;
			cgogn_assert(depth == 0);
			if (announced_size >= 0 && announced_size != actual_size)
				std::cout << "Warning: expected size " << announced_size
						<< " for array but it was actually of size " << actual_size << std::endl;
			return true;
		case ':':
			cgogn_assert(c_ == 'a' && depth == 1);
			in_array = true;
			c = '\0';
			[[fallthrough]];
		case ',':
			cgogn_assert(in_array);
			{
				const uint32 read_size = read_values();
				cgogn_assert(read_size <= std::numeric_limits<uint32>::max() - actual_size);
				actual_size += read_size;
			}
			break;
		case ';':
			cgogn_assert(c_ == '\0');
			skip_through_character(is, '\n');
			break;
		case '\n':
			if (depth == 1)
				c = '\0'; // allow comment on new line inside braces
			break;
		default:
			if (depth == 0)
			{
				if (c == '*')
				{
					is >> announced_size;
					on_size(announced_size);
				}
			}
			else if (!in_array && !std::isspace(c))
				c_ = c;
		}
	}

	return false;
}

// Skips whitespace, reads an integer, and warns if it is not the expected value
void FbxImporterBase::read_integer_and_warn_if_not_expected(std::istream& is,
		const std::string& nature, int expected_value)
{
	int value;
	std::string value_str = "<ERROR>";

	if ((is >> value).fail())
		value = -expected_value;
	else
		value_str = std::to_string(value);

	if (value != expected_value)
		std::cout << "Warning: expected " << nature << " " << expected_value
				<< " but got " << value_str << std::endl;
}

// Ignores everything until after the next node's yet-unopened braces close
void FbxImporterBase::skip_node(std::istream& is)
{
	char c;
	for (int depth = 0; is.get(c);)
	{
		if (c == '{')
			++depth;
		else if (c == '}')
			--depth;
		else
			continue; // ignore any other character

		if (depth == 0) // back out
			return;
	}
}

// Ignores everything until after a line break or the end of the next node
void FbxImporterBase::skip_value(std::istream& is)
{
	char c;
	while (is.get(c))
	{
		if (c == '\n') // primitive
			return;

		if (c == '{') // node
		{
			is.unget();
			skip_node(is);
		}
	}
}

// Ignores everything through a given character
void FbxImporterBase::skip_through_character(std::istream& is, char c)
{
	is.ignore(std::numeric_limits<std::streamsize>::max(), c);
}

// Ignores everything through a given set of characters
std::optional<char> FbxImporterBase::skip_through_characters(std::istream& is, const std::string& characters)
{
	char c;
	while (is.get(c))
		if (characters.find(c) != std::string::npos)
			return c;

	return {};
}

const FbxImporterBase::LimbNodeModel* FbxImporterBase::get_parent_bone(const ObjectId& child_id) const
{
	for (const auto& [child_id_, parent_id] : connections_oo_)
	{
		if (child_id != child_id_)
			continue;

		const auto it = std::find_if(models_limb_node_.cbegin(), models_limb_node_.cend(),
				[&, parent_id = parent_id](const LimbNodeModel& e) { return e.id == parent_id; });

		if (it != models_limb_node_.cend())
			return &*it;
	}

	return nullptr;
}

std::string FbxImporterBase::resolve_name(const std::string& name_with_escape_sequences)
{
	return name_with_escape_sequences; // ignore escape sequences for now
}

geometry::Quaternion FbxImporterBase::from_euler(const std::array<std::optional<AnimScalar>, 3>& xyz,
		const RotationOrder& rotation_order)
{
	static const std::array<Vec3, 3> axes = {Vec3{1, 0, 0}, Vec3{0, 1, 0}, Vec3{0, 0, 1}};

	auto res = geometry::Quaternion::Identity();
	size_t ro = rotation_order;

	for (int i = 0; i < 3; ++i)
	{
		size_t axis = ro % RotationOrder::Base;
		ro /= RotationOrder::Base;

		if (xyz[axis] && *xyz[axis] != 0.0)
			res = Eigen::AngleAxis<geometry::Quaternion::Scalar>(M_PI / 180.0 * *xyz[axis], axes[axis])
					* res;
	}

	return res;
}

const Vec3 FbxImporterBase::get_default_translation(
		const std::string& template_key, const std::array<std::optional<AnimScalar>, 3>& (Properties::* f)() const) const
{
	const auto template_it = property_templates_.find(template_key);

	if (template_it == property_templates_.cend())
		return Vec3::Zero();

	const auto& arr = (template_it->second.*f)();
	return Vec3{arr[0].value_or(0.0), arr[1].value_or(0.0), arr[2].value_or(0.0)};
}

const geometry::Quaternion FbxImporterBase::get_default_rotation(
		const std::string& template_key, const std::array<std::optional<AnimScalar>, 3>& (Properties::* f)() const,
		const RotationOrder& rotation_order) const
{
	const auto template_it = property_templates_.find(template_key);
	return template_it != property_templates_.cend() ?
			from_euler((template_it->second.*f)(), rotation_order) : geometry::Quaternion::Identity();
}

const FbxImporterBase::RotationOrder& FbxImporterBase::get_default_rotation_order(const std::string& template_key) const
{
	const auto template_it = property_templates_.find(template_key);
	return get_rotation_order<true, 0>(static_cast<size_t>(template_it != property_templates_.cend() ?
		template_it->second.rotation_order().value_or(0) : 0));
}

void FbxImporterBase::AnimationCurveNode::set_animation(const AnimationT* anim, const std::string& axis_property_name)
{
	size_t anim_id = 0;

	// Get axis offset
	if (axis_property_name == "d|Z"s)
		anim_id += 2;
	else if (axis_property_name == "d|Y"s)
		++anim_id;
	else if (axis_property_name != "d|X"s)
		return; // not d|<axis>, ignore

	animation[anim_id] = anim;
}

void FbxImporterBase::LimbNodeModel::set_animation(
		const std::array<const AnimationT*, 3>& anim, const std::string& transform_property_name)
{
	size_t offset;

	// Get transformation type offset
	if (transform_property_name == "Lcl Translation"s)
		offset = 0;
	else if (transform_property_name == "Lcl Rotation"s)
		offset = 3;
	else
		return;

	for (size_t i = 0; i < 3; ++i)
		animation[offset + i] = anim[i];
}

bool FbxImporterBase::LimbNodeModel::has_component(const size_t& offset) const
{
	return animation[offset] || animation[offset + 1] || animation[offset + 2];
}

bool FbxImporterBase::LimbNodeModel::has_translation() const { return has_component(0); }
bool FbxImporterBase::LimbNodeModel::has_rotation() const { return has_component(3); }

std::optional<FbxImporterBase::AnimScalar> FbxImporterBase::LimbNodeModel::get_value(
		const size_t& index, const AnimTimeT& time) const
{
	return animation[index] ? animation[index]->get_value(time) : std::optional<AnimScalar>();
}

Vec3 FbxImporterBase::LimbNodeModel::get_translation_or(const AnimTimeT& time, const Vec3& default_value) const
{
	if (!has_translation())
		return default_value;

	return Vec3{get_value(0, time).value_or(0.0), get_value(1, time).value_or(0.0), get_value(2, time).value_or(0.0)};
}

geometry::Quaternion FbxImporterBase::LimbNodeModel::get_rotation_or(const AnimTimeT& time,
		const geometry::Quaternion& pre_rotation, const geometry::Quaternion& post_rotation,
		const geometry::Quaternion& default_value) const
{
	if (!has_rotation())
		return default_value;

	return pre_rotation
			* FbxImporterBase::from_euler(std::array<std::optional<AnimScalar>, 3>
					{get_value(3, time), get_value(4, time), get_value(5, time)})
			* post_rotation;
}


// FbxImporter specialization non-templated methods


void FbxImporter::load_bones(Skeleton& skeleton)
{
	for (LimbNodeModel& m : models_limb_node_)
		m.bone = add_root(skeleton, m.name);

	for (const LimbNodeModel& m : models_limb_node_)
		if (auto* parent = get_parent_bone(m.id))
			attach_bone(skeleton, m.bone, parent->bone);
}

void FbxImporter::associate_animations_to_bones()
{
	for (const auto& [curve_id, curve_node_id, axis_property_name] : connections_op_)
	{
		// Get animation curve node's animation curves and targeted property

		const auto curve_node_it = std::find_if(animation_curve_nodes_.begin(), animation_curve_nodes_.end(),
				[&, curve_node_id = curve_node_id](const AnimationCurveNode& node){ return node.id == curve_node_id; });

		if (curve_node_it == animation_curve_nodes_.end())
			continue;

		const auto curve_it = std::find_if(animation_curves_.cbegin(), animation_curves_.cend(),
				[&, curve_id = curve_id](const AnimationCurve& curve){ return curve.id == curve_id; });

		if (curve_it == animation_curves_.cend())
			continue;

		// Compare node value with curve default (*)
		set_missing_values(curve_node_it->properties,
				std::array<std::optional<AnimScalar>, 1>{curve_it->default_value},
				axis_property_name, false);

		curve_node_it->set_animation(&curve_it->animation, axis_property_name);
	}

	for (const auto& [curve_node_id, model_id, transform_property_name] : connections_op_)
	{
		// Get bone's animation curve node and targeted property

		const auto model_it = std::find_if(models_limb_node_.begin(), models_limb_node_.end(),
				[&, model_id = model_id](const LimbNodeModel& model){ return model.id == model_id; });

		if (model_it == models_limb_node_.end())
			continue;

		const auto curve_node_it = std::find_if(animation_curve_nodes_.begin(), animation_curve_nodes_.end(),
				[&, curve_node_id = curve_node_id](const AnimationCurveNode& node){ return node.id == curve_node_id; });

		if (curve_node_it == animation_curve_nodes_.end())
			continue;

		model_it->set_animation(curve_node_it->animation, transform_property_name);

		// Compare model value with node's (*)
		set_missing_values(model_it->properties, curve_node_it->properties.d(), transform_property_name, true);
	}

	// (*) should be equal if both are present, if the former isn't, then try to get a value from the latter
}

void FbxImporter::load_animations(Skeleton& skeleton)
{
	using RT = geometry::RigidTransformation<geometry::Quaternion, Vec3>;
	using KA_RT = geometry::KeyframedAnimation<std::vector, AnimTimeT, RT>;
	using KA_DQ = geometry::KeyframedAnimation<std::vector, AnimTimeT, geometry::DualQuaternion>;

	// @TODO: handling multiple animations in the same file (layers?)

	associate_animations_to_bones();

	auto& attr_joint_position = *add_attribute<Vec3, Skeleton::Joint>(skeleton, "position");
	for (const auto& bone : skeleton.bone_traverser_)
	{
		const auto& [first_joint, second_joint] = (*skeleton.bone_joints_)[index_of(skeleton, bone)];
		attr_joint_position[second_joint] = attr_joint_position[first_joint] = Vec3::Zero();
	}

	auto [attr_anim_rt_b, attr_anim_rt] = add_animation_attributes<KA_RT>(skeleton, "RT");
	auto [attr_anim_dq_b, attr_anim_dq] = add_animation_attributes<KA_DQ>(skeleton, "DQ");

	const std::string template_key = "Model";
	const auto& default_rotation_order = get_default_rotation_order(template_key);
	const auto default_pre_rotation = get_default_rotation(template_key, &Properties::pre_rotation, default_rotation_order);
	const auto default_post_rotation = get_default_rotation(template_key, &Properties::post_rotation, default_rotation_order);
	const auto default_lcl_rotation = get_default_rotation(template_key, &Properties::lcl_rotation, default_rotation_order);
	const Vec3 default_lcl_translation = get_default_translation(template_key, &Properties::lcl_translation);

	for (const LimbNodeModel& m : models_limb_node_)
	{
		KA_RT anim_rt; // RigidTransformation animation
		KA_RT anim_rt_b; // RigidTransformation bind pose
		KA_DQ anim_dq; // DualQuaternion animation
		KA_DQ anim_dq_b; // DualQuaternion bind pose

		// Get keyframe times
		std::set<AnimTimeT> times;
		{
			std::vector<AnimationT> anim_copies;
			for (const auto& anim : m.animation)
				if (anim)
					anim_copies.push_back(*anim);
			times = AnimationT::get_unique_keyframe_times(anim_copies);
		}

		const auto& rotation_order = m.properties.rotation_order() ?
				get_rotation_order(static_cast<size_t>(*m.properties.rotation_order()))
						.value_or(default_rotation_order) : default_rotation_order;

		const auto rotation_d = [&](const std::array<std::optional<AnimScalar>, 3>& values,
				geometry::Quaternion default_value)
		{
			return Properties::any_set(values) ? from_euler(values, rotation_order) : default_value;
		};

		const auto lcl_translation_d = [&] {
			const auto& values = m.properties.lcl_translation();
			if (!Properties::any_set(values))
				return default_lcl_translation;
			return Vec3{values[0].value_or(0.0), values[1].value_or(0.0), values[2].value_or(0.0)};
		};

		auto pre_rotation = rotation_d(m.properties.pre_rotation(), default_pre_rotation);
		auto post_rotation = rotation_d(m.properties.post_rotation(), default_post_rotation);
		auto lcl_rotation = rotation_d(m.properties.lcl_rotation(), default_lcl_rotation);
		auto total_lcl_rotation = pre_rotation * lcl_rotation * post_rotation;
		Vec3 lcl_translation = lcl_translation_d();

		anim_rt_b.emplace_back(0.0, RT{total_lcl_rotation, lcl_translation});
		anim_dq_b.emplace_back(0.0, geometry::DualQuaternion::from_tr(lcl_translation, total_lcl_rotation));

		// We assume the interpolation to be linear, so we can create full transform keyframes by interpolating
		// each component along its own animation, instead of passing the split components beyond
		for (const auto& time : times)
		{
			auto t = m.get_translation_or(time, lcl_translation);
			auto r = m.get_rotation_or(time, pre_rotation, post_rotation, total_lcl_rotation);

			anim_rt.emplace_back(time, RT{r, t});
			anim_dq.emplace_back(time, geometry::DualQuaternion::from_tr(t, r));
		}

		if (times.empty())
		{
			anim_rt = anim_rt_b;
			anim_dq = anim_dq_b;
		}

		const auto& bone_index = index_of(skeleton, m.bone);
		attr_anim_rt[bone_index] = anim_rt;
		attr_anim_rt_b[bone_index] = anim_rt_b;
		attr_anim_dq[bone_index] = anim_dq;
		attr_anim_dq_b[bone_index] = anim_dq_b;
	}
}

FbxImporter::Skeleton* FbxImporter::load_skeleton()
{
	if (models_limb_node_.empty())
		return nullptr;

	Skeleton* skeleton = new Skeleton{};

	load_bones(*skeleton);
	load_animations(*skeleton);

	return skeleton;
}

} // namespace io

} // namespace cgogn
