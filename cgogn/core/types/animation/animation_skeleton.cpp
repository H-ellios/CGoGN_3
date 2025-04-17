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

#include <cgogn/core/types/animation/animation_skeleton.h>

namespace cgogn
{

/*************************************************************************/
// Operators
/*************************************************************************/

bool is_root(const AnimationSkeleton& as, const AnimationSkeleton::Joint& joint)
{
	for (const auto& bone : as.bone_traverser_)
		if ((*as.bone_joints_)[bone].first == joint && is_root(as, bone))
			return true;

	return false;
}

bool is_root(const AnimationSkeleton& as, const AnimationSkeleton::Bone& bone)
{
	return !(*as.bone_parent_)[bone].is_valid();
}

AnimationSkeleton::Bone add_bone_from_existing_joints(
		AnimationSkeleton& as, const AnimationSkeleton::Bone& parent,
		std::pair<AnimationSkeleton::Joint, AnimationSkeleton::Joint> joints)
{
	AnimationSkeleton::Bone bone = add_cell<AnimationSkeleton::Bone>(as);

	(*as.bone_parent_)[bone] = parent;
	(*as.bone_joints_)[bone] = joints;
	(*as.bone_name_)[bone].clear();
	as.bone_traverser_.push_back(bone);

	return bone;
}

static AnimationSkeleton::Bone _internal_add_bone(AnimationSkeleton& as, const AnimationSkeleton::Bone& parent)
{
	using Joint = AnimationSkeleton::Joint;

	const bool& is_root = !parent.is_valid();

	cgogn_assert(is_root || std::find(as.bone_traverser_.cbegin(), as.bone_traverser_.cend(), parent)
			!= as.bone_traverser_.cend());

	Joint first_joint = is_root ? add_cell<Joint>(as) : (*as.bone_joints_)[parent].second;
	Joint second_joint = add_cell<Joint>(as);

	return add_bone_from_existing_joints(as, parent, {first_joint, second_joint});
}

AnimationSkeleton::Bone add_root(AnimationSkeleton& as)
{
	return _internal_add_bone(as, INVALID_INDEX);
}

AnimationSkeleton::Bone add_root(AnimationSkeleton& as, const std::string& name)
{
	auto bone = add_root(as);
	(*as.bone_name_)[bone] = name;
	return bone;
}

AnimationSkeleton::Bone add_root(AnimationSkeleton& as, std::string&& name)
{
	auto bone = add_root(as);
	(*as.bone_name_)[bone] = std::move(name);
	return bone;
}

AnimationSkeleton::Bone add_bone(AnimationSkeleton& as, const AnimationSkeleton::Bone& parent)
{
	return _internal_add_bone(as, parent);
}

AnimationSkeleton::Bone add_bone(AnimationSkeleton& as, const AnimationSkeleton::Bone& parent, const std::string& name)
{
	auto bone = add_bone(as, parent);
	(*as.bone_name_)[bone] = name;
	return bone;
}

AnimationSkeleton::Bone add_bone(AnimationSkeleton& as, const AnimationSkeleton::Bone& parent, std::string&& name)
{
	auto bone = add_bone(as, parent);
	(*as.bone_name_)[bone] = std::move(name);
	return bone;
}

static bool _internal_check_parent_candidate_not_in_descendants(AnimationSkeleton& as,
		const AnimationSkeleton::Bone& bone, AnimationSkeleton::Bone parent_candidate)
{
	do
	{
		if (bone == parent_candidate)
			return false;
	} while ((parent_candidate = (*as.bone_parent_)[parent_candidate]) != INVALID_INDEX);

	return true;
}

/// @brief Stable sort in `[child_i, parent_i]` using the partial order requirement that parents must preceed their children
///        with the assumption that only the bounds might violate this order in the initial state
static void _internal_ensure_bone_traverser_order(AnimationSkeleton& as,
		size_t child_i, size_t parent_i)
{
	if (child_i > parent_i) // child is already after parent in traverser
		return;

	cgogn_assert(child_i != parent_i); // redundancy for caller's checks
	cgogn_message_assert(0 <= child_i && parent_i < as.nb_bones(),
			"Iterator difference assertions broken: does bone_traverser_ still provide a random access iterator?");

	// Determine which elements from the child to the parent
	// are descendants of the parent and should be shifted after it

	auto parent = as.bone_traverser_[parent_i];
	auto search_begin = as.bone_traverser_.cbegin() + child_i;
	auto search_end = as.bone_traverser_.cbegin() + parent_i;

	auto offset = parent_i - child_i;
	std::vector<bool> is_descendant(offset + 1); // +1 so parent can be read as non-descendant in R/W loop
	std::vector<AnimationSkeleton::Bone> descendants;
	descendants.reserve(offset / 2 + 1); // guesstimating that half the bones are descendants

	is_descendant[0] = true; // child_i should be after parent
	descendants.push_back(as.bone_traverser_[child_i]);

	for (auto i = 1; i < offset; ++i)
	{
		auto p = (*as.bone_parent_)[as.bone_traverser_[child_i + i]];

		// If `p == parent`, it means bones at `i` and `child_i` are siblings, so the one at `i` also is a descendant
		// Otherwise to be a descendant its parent necessarily has to have been marked as one already
		if (p != parent)
		{
			auto p_i = std::find(search_begin, search_end, p);
			if (p_i == search_end || !is_descendant[p_i - search_begin])
				return;
		}

		if (descendants.size() == descendants.capacity()) // guess was wrong, just double the size
			descendants.reserve(offset);

		is_descendant[i] = true;
		descendants.push_back(std::move(as.bone_traverser_[child_i + i]));
	}

	// Write ancesters (including parent) then descendants (including child)

	for (auto r = 1, w = 0;; ++r, ++w)
	{
		cgogn_assert(w < r);

		while (r <= offset && is_descendant[r])
			++r;

		if (r > offset)
			break;

		as.bone_traverser_[child_i + w] = as.bone_traverser_[child_i + r];
	}

	{
		auto i = parent_i - descendants.size();
		for (auto d : descendants)
			as.bone_traverser_[++i] = d;
	}
}

void attach_bone(AnimationSkeleton& as, const AnimationSkeleton::Bone& bone, const AnimationSkeleton::Bone& new_parent)
{
	if (!new_parent.is_valid())
	{
		detach_bone(as, bone);
		return;
	}

	cgogn_assert(_internal_check_parent_candidate_not_in_descendants(as, bone, new_parent));
	cgogn_assert(std::find(as.bone_traverser_.cbegin(), as.bone_traverser_.cend(), new_parent)
			!= as.bone_traverser_.cend());

	auto& parent = (*as.bone_parent_)[bone];
	auto& joints = (*as.bone_joints_)[bone];

	if (parent == new_parent)
		return;

	parent = new_parent;
	remove_cell(as, joints.first);
	joints.first = (*as.bone_joints_)[new_parent].second;

	const auto& cb = as.bone_traverser_.cbegin();
	_internal_ensure_bone_traverser_order(as,
			std::find(cb, as.bone_traverser_.cend(), bone) - cb,
			std::find(cb, as.bone_traverser_.cend(), new_parent) - cb);
}

void detach_bone(AnimationSkeleton& as, const AnimationSkeleton::Bone& bone)
{
	auto& parent = (*as.bone_parent_)[bone];

	if (!parent.is_valid())
		return;

	parent = INVALID_INDEX;
	(*as.bone_joints_)[bone].first = add_cell<AnimationSkeleton::Joint>(as);
}

AnimationSkeleton::Bone get_parent_bone(const AnimationSkeleton& as, const AnimationSkeleton::Joint& joint)
{
	cgogn_assert(as.nb_bones() > 0);

	// Joint may be base joint of its *root* parent bone, or tip joint of its parent bone
	for (const auto& bone : as.bone_traverser_)
	{
		const auto& [first_joint, second_joint] = (*as.bone_joints_)[bone];
		if (first_joint == joint && is_root(as, bone) || second_joint == joint)
			return bone;
	}

	return INVALID_INDEX; // called with non-existent joint, or topology is broken
}

AnimationSkeleton::Joint get_base_joint(const AnimationSkeleton& as, const AnimationSkeleton::Bone& bone)
{
	cgogn_assert(bone < as.nb_bones());
	return (*as.bone_joints_)[bone].first;
}

AnimationSkeleton::Joint get_tip_joint(const AnimationSkeleton& as, const AnimationSkeleton::Bone& bone)
{
	cgogn_assert(bone < as.nb_bones());
	return (*as.bone_joints_)[bone].second;
}

void copy(AnimationSkeleton& dst, const AnimationSkeleton& src)
{
	dst = src;
}

} // namespace cgogn
