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

#include <cctype>
#include <unordered_map>

#include <cgogn/core/types/maps/map_base.h>
#include <cgogn/core/types/maps/cmap/cmap2.h>
#include <cgogn/core/types/maps/cmap/cmap3.h>
#include <cgogn/core/types/animation/animation_skeleton.h>
#include <cgogn/geometry/types/vector_traits.h>
#include <cgogn/core/utils/numerics.h>
#include <cgogn/io/utils.h>

#include <cgogn/ui/app.h>
#include <cgogn/ui/view.h>

#include <cgogn/geometry/ui_modules/animation_skeleton_controller.h>
#include <cgogn/rendering/ui_modules/animation_skeleton_render.h>
#include <cgogn/geometry/ui_modules/skinning_controller.h>
#include <cgogn/modeling/ui_modules/volume_surface_fitting.h>
#include <cgogn/modeling/ui_modules/surface_modeling.h>
#include <cgogn/rendering/ui_modules/surface_render.h>
#include <cgogn/rendering/ui_modules/volume_render.h>
#include <cgogn/core/ui_modules/mesh_provider.h>
#include <cgogn/io/ui_modules/fbx_io.h>

using Surface = cgogn::CMap2;
using Volume = cgogn::CMap3;
using Skeleton = cgogn::AnimationSkeleton;

template <typename T>
using AttributeM = typename cgogn::mesh_traits<Volume>::Attribute<T>;
using Vertex = typename cgogn::mesh_traits<Volume>::Vertex;
using Vertex2 = typename cgogn::mesh_traits<Surface>::Vertex;

template <typename T>
using AttributeS = typename cgogn::mesh_traits<Skeleton>::Attribute<T>;
using Joint = typename cgogn::mesh_traits<Skeleton>::Vertex;
using Bone = typename cgogn::mesh_traits<Skeleton>::Edge;

using Vec3 = cgogn::geometry::Vec3;
using Vec4 = cgogn::geometry::Vec4;
using Vec4i = cgogn::geometry::Vec4i;
using Quaternion = cgogn::geometry::Quaternion;
using Scalar = cgogn::geometry::Scalar;
using RigidTransformation = cgogn::geometry::RigidTransformation<Quaternion, Vec3>;
using cgogn::geometry::DualQuaternion;

using ASC_RT = cgogn::ui::AnimationSkeletonController<std::vector, double, RigidTransformation>;
using ASC_DQ = cgogn::ui::AnimationSkeletonController<std::vector, double, DualQuaternion>;

using KA_RT = cgogn::geometry::KeyframedAnimation<std::vector, double, RigidTransformation>;

using namespace cgogn::numerics;

#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_DATA_PATH) "meshes/"

// Strips everything preceding and including the last instance of a colon,
// and if `to_lower_case`, converts the result to lower case (for case-insensitive bone matching)
std::string process_bone_name(const std::string& raw_name, bool to_lower_case = true)
{
	const size_t i = raw_name.find_last_of(':');
	std::string res = i != std::string::npos ? raw_name.substr(i + 1) : raw_name;
	if (to_lower_case)
		std::transform(res.begin(), res.end(), res.begin(), [](const char& c) { return std::tolower(c); });
	return res;
}

// Reads vertex skinning weights from the following format:
// (`\\s` represents whitespace, `\\S` is its complement)
// (`{f}` is a subset of `\\S+` which can be parsed as a floating point number)
// `valid_file := {vertex_weights}*`
// `vertex_weights := ({vertex_weight}\\s+){0-4}[;]`
// `vertex_weight := \\S+\\s+{f}`
// For example, `Hips 1 ; Hips 0.03 Spine 0.37 Spine1 0.48 Spine2 0.12 ;` represents the weight of two vertices
// The reader may find files easier to understand by adding a line break after each semicolon
// When matching bone names, everything preceding and including the last instance of a colon is stripped,
// and also converts the string to lower case for insensitivity, for example, `Provider::Rig::Hips` becomes `hips`
bool load_weights(const Skeleton& sk, Volume& m, const std::vector<uint32>& vertex_id_after_import,
		const std::string& path)
{
	std::string buf;
	cgogn::io::Scoped_C_Locale loc;
	std::ifstream is(path.c_str(), std::ios::in);

	std::unordered_map<std::string, Bone> bones;
	for (const auto& bone : sk.bone_traverser_)
		bones[process_bone_name((*sk.bone_name_)[index_of(sk, bone)])] = bone;

	auto& wi = *cgogn::add_attribute<Vec4i, Vertex>(m, "weight_index");
	auto& wv = *cgogn::add_attribute<Vec4, Vertex>(m, "weight_value");

	for (const auto& i : vertex_id_after_import)
	{
		if (!is || is.eof())
			return false;

		wi[i] = {-1, -1, -1, -1};
		wv[i] = Vec4::Zero();

		for (uint32 j = 0; ; ++j)
		{
			if (!is || !(is >> buf))
				return false;

			if (buf[0] == ';')
				break;
			else if (j >= 4)
				return false;

			Vec4::Scalar w;
			is >> w;

			wi[i][j] = bones.at(process_bone_name(buf));
			wv[i][j] = w;
		}
	}

	return true;
}

int main(int argc, char** argv)
{

    using PropagationDirection = cgogn::ui::SkinnedVolumeSurfaceFitting<Surface, Volume>::PropagationDirection;
	if (argc < 4)
	{
		std::cerr << "Wrong number of arguments, requires a mesh path, a weight file path, "
				"and a FBX file path" << std::endl;
		return 1;
	}

	cgogn::thread_start();

	cgogn::ui::App app;
	app.set_window_title("Volume skinning viewer");
	app.set_window_size(1000, 800);

	cgogn::ui::MeshProvider<Volume> mp(app);
	auto sp_mp_sf = std::make_shared<cgogn::ui::MeshProvider<Surface>>(app);
	auto sp_mp_as = std::make_shared<cgogn::ui::MeshProvider<Skeleton>>(app);
	auto& mp_sf = *sp_mp_sf;
	auto& mp_as = *sp_mp_as;
	cgogn::ui::FbxIO<Surface> fbx_io(app, sp_mp_sf, sp_mp_as);
	ASC_RT asc_rt(app);
    //ASC_DQ asc_dq(app);
	cgogn::ui::SkinningController<Surface, RigidTransformation> skc_s_rt(app);
    //cgogn::ui::SkinningController<Surface, DualQuaternion> skc_s_dq(app);
	cgogn::ui::SkinningController<Volume, RigidTransformation> skc_v_rt(app);
    //cgogn::ui::SkinningController<Volume, DualQuaternion> skc_v_dq(app);
	cgogn::ui::SurfaceModeling<Surface> sm(app);
	cgogn::ui::SkinnedVolumeSurfaceFitting<Surface, Volume> vsf(app);
	cgogn::ui::AnimationSkeletonRender<RigidTransformation, DualQuaternion> asr(app);
	cgogn::ui::SurfaceRender<Surface> sr(app);
	cgogn::ui::VolumeRender<Volume> vr(app);

	app.init_modules();

	cgogn::ui::View* v1 = app.current_view();
	v1->link_module(&mp);
	v1->link_module(&mp_sf);
	v1->link_module(&mp_as);
	v1->link_module(&vsf);
	v1->link_module(&asr);
	v1->link_module(&sr);
	v1->link_module(&vr);

	Skeleton* sk{};
	fbx_io.load_file(argv[3]);
	mp_as.foreach_mesh([&](Skeleton& s, const std::string&){ sk = &s; }); // dirty but it does the job of querying the skeleton

	if (!sk)
	{
		std::cout << "Skeleton could not be loaded" << std::endl;
		return 1;
	}

	std::vector<uint32> vertex_id_after_import;
	Volume* m = mp.load_volume_from_file(argv[1], &vertex_id_after_import);
	if (!m)
	{
		std::cout << "Volume could not be loaded" << std::endl;
		return 1;
	}

	if (!load_weights(*sk, *m, vertex_id_after_import, argv[2]))
	{
		std::cout << "Volume weights could not be loaded" << std::endl;
		return 1;
	}

	mp.set_mesh_bb_vertex_position(*m, cgogn::get_attribute<Vec3, Vertex>(*m, "position"));

    auto rt_bind_attr = cgogn::get_attribute<KA_RT, Skeleton::Bone>(*sk, "RT_bind");
    asc_rt.set_skeleton(sk);
    asc_rt.set_animation(rt_bind_attr);
    asc_rt.set_time_start();

    Surface* sf = nullptr;

    mp_sf.foreach_mesh([&](Surface& s, const std::string&){
        sm.triangulate_mesh(s,cgogn::get_attribute<Vec3,Vertex2>(s,"position").get());
        if(sf == nullptr)
            sf = &s;
    });

    auto wi = cgogn::get_attribute<Vec4i, Vertex>(*m, "weight_index");
    auto wv = cgogn::get_attribute<Vec4, Vertex>(*m, "weight_value");

    auto wi_s = cgogn::get_attribute<Vec4i, Vertex2>(*sf, "weight_index");
    auto wv_s = cgogn::get_attribute<Vec4, Vertex2>(*sf, "weight_value");

    vsf.set_current_volume(m);
    vsf.set_current_volume_vertex_skinning_weight_index(wi);
    vsf.set_current_volume_vertex_skinning_weight_value(wv);

    vsf.set_current_surface(sf);
    vsf.set_current_surface_vertex_position(cgogn::get_attribute<Vec3,Vertex2>(*sf,"position"));
    vsf.set_current_surface_vertex_skinning_weight_index(wi_s);
    vsf.set_current_surface_vertex_skinning_weight_value(wv_s);

    vsf.set_current_animation_skeleton(*sk);
    vsf.set_current_animation_skeleton_joint_position(cgogn::get_attribute<Vec3, Skeleton::Joint>(*sk, "position"));

    vsf.project_on_surface();
    vsf.subdivide_volume();

    for(int i=0;i<1000;i++){
        vsf.project_on_surface();
    }
    for(int i=0;i<3;i++){
        vsf.optimize_volume_vertices(10.,10.,cgogn::geometry::ProximityPolicy(cgogn::geometry::NEAREST_POINT),false);
    }

    vsf.mark_volume_core_vertices();

    vsf.subdivide_volume();

    for(int i=0;i<10;i++){
        vsf.project_on_surface();
    }

    for(int i=0;i<5;i++){
        vsf.optimize_volume_vertices(1.,10.,cgogn::geometry::ProximityPolicy(cgogn::geometry::NEAREST_POINT),false);

    }
    for(int i=0;i<5;i++){
        vsf.optimize_volume_vertices(5.,10.,cgogn::geometry::ProximityPolicy(cgogn::geometry::NEAREST_POINT),false);

    }


    vsf.select_volume_vertices_from_core_mark();

    vsf.transfer_skinning_weights_to_skin();
    vsf.propagate_skinning_weights<PropagationDirection::BoundaryToCenter>(1, false);

    skc_s_rt.set_mesh(sf);
    skc_s_rt.set_vertex_weight_index(wi_s);
    skc_s_rt.set_vertex_weight_value(wv_s);
    skc_s_rt.set_skeleton(sk);
    skc_s_rt.set_bone_world_transform(asc_rt.selected_bone_world_transform_);


    skc_v_rt.set_mesh(m);
    skc_v_rt.set_vertex_weight_index(wi);
    skc_v_rt.set_vertex_weight_value(wv);
    skc_v_rt.set_skeleton(sk);
    skc_v_rt.set_bone_world_transform(asc_rt.selected_bone_world_transform_);

    rt_bind_attr = cgogn::get_attribute<KA_RT, Skeleton::Bone>(*sk, "RT");
    asc_rt.set_skeleton(sk);
    asc_rt.set_animation(rt_bind_attr);
    asc_rt.set_time_start();


	return app.launch();
}
