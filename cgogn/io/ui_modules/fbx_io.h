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

#ifndef CGOGN_MODULE_FBX_IO_H_
#define CGOGN_MODULE_FBX_IO_H_

#include <cgogn/ui/imgui_helpers.h>
#include <cgogn/ui/module.h>
#include <cgogn/ui/portable-file-dialogs.h>
#include <cgogn/core/utils/string.h>

#include <cgogn/core/ui_modules/mesh_provider.h>
#include <cgogn/io/fbx/fbx_importer.h>

namespace cgogn
{

namespace ui
{

class App;

template <typename Surface>
class FbxIO : public Module
{
private:
	using Skeleton = AnimationSkeleton;

	template <typename T>
	using AttributeSf = typename mesh_traits<Surface>::template Attribute<T>;

	using Vertex = typename mesh_traits<Surface>::Vertex;

	template <typename T>
	using AttributeSk = typename mesh_traits<Skeleton>::template Attribute<T>;

	using Bone = Skeleton::Bone;

public:
	FbxIO(const App& app, std::shared_ptr<MeshProvider<Surface>> surface_provider, std::shared_ptr<MeshProvider<AnimationSkeleton>> skeleton_provider)
		: Module(app, "FbxIO (" + std::string{mesh_traits<Surface>::name} + ")"),
				surface_provider_(surface_provider), skeleton_provider_(skeleton_provider)
	{
	}

	~FbxIO()
	{
	}

	void load_file(const std::string& path, bool load_surfaces = true, bool load_skeletons = true, bool normalized = false)
	{
		std::string filename = filename_from_path(path);
		typename io::FbxImporter::Map<std::string, Surface*> surfaces;
		std::optional<Skeleton*> skeleton;

		io::FbxImporter::load<Surface>(path, surfaces, skeleton, load_surfaces, load_skeletons, normalized);

		for (const auto [objname, surface] : surfaces)
			register_surface(surface, objname, filename);

		if (skeleton)
			register_skeleton(skeleton.value(), filename);
	}

protected:
	void main_menu() override
	{
		static std::unique_ptr<pfd::open_file> open_file_dialog;
		if (open_file_dialog && open_file_dialog->ready())
		{
			for (const auto& file : open_file_dialog->result())
				load_file(file, mesh_traits<Surface>::dimension == 2 && std::is_default_constructible_v<Surface>);
			open_file_dialog = nullptr;
		}

		if (ImGui::BeginMenu(name_.c_str()))
		{
			ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)open_file_dialog);
			if (ImGui::MenuItem("Load files"))
				open_file_dialog = std::make_unique<pfd::open_file>("Choose file", ".",
						supported_files_, pfd::opt::multiselect);
			ImGui::PopItemFlag();
			ImGui::EndMenu();
		}
	}

private:
	template <typename T>
	inline std::string get_name(const MeshProvider<T>& mesh_provider, const std::string& objname, const std::string& filename)
	{
		std::string name = objname.empty() ? filename : filename + '>' + objname;
		if (mesh_provider.has_mesh(name))
			name += "_" + std::to_string(mesh_provider.number_of_meshes()); // may happen, effective disambiguation
		while (mesh_provider.has_mesh(name))
			name += '_'; // shouldn't happen, but disambiguate however we can
		return name;
	}

	void register_surface(Surface* surface, const std::string& objname, const std::string& filename)
	{
		if constexpr (mesh_traits<Surface>::dimension == 2 && std::is_default_constructible_v<Surface>)
			surface_provider_->register_mesh(surface, get_name(*surface_provider_, objname, filename));
	}

	void register_skeleton(Skeleton* skeleton, const std::string& filename)
	{
		skeleton_provider_->register_mesh(skeleton, get_name(*skeleton_provider_, "", filename));
	}

private:
	const std::vector<std::string> supported_files_ = {"FBX", "*.fbx"};

	std::shared_ptr<MeshProvider<Surface>> surface_provider_ = nullptr;
	std::shared_ptr<MeshProvider<AnimationSkeleton>> skeleton_provider_ = nullptr;
};

} // namespace ui

} // namespace cgogn

#endif // CGOGN_MODULE_FBX_IO_H_
