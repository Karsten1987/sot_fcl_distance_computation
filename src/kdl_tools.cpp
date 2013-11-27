#include <sot_fcl_distance_computation/kdl_tools.h>
#include <kdl/tree.hpp>

namespace KDL {

Tree get_sub_tree(KDL::Tree tree, std::vector<std::string> tips,
		std::string root) {
	Tree res(root);
	std::vector<KDL::Chain> chains(tips.size());

	for (unsigned int i = 0; i < tips.size(); ++i) {
		tree.getChain(root, tips[i], chains[i]);
	}
	for (unsigned int i = 0; i < tips.size(); ++i) {
		std::string parent_name = root;
		SegmentMap tree_segments = res.getSegments();

		for (unsigned int j = 0; j < chains[i].getNrOfSegments(); j++) {
			if (tree_segments.find(chains[i].getSegment(j).getName())
					== tree_segments.end()) { //Segment doesnt exists
				res.addSegment(chains[i].getSegment(j), parent_name);
			}
			parent_name = chains[i].getSegment(j).getName();
		}

	}
	return res;
}

//This function return the names inside a tree performing a recursive search. Getting them from the std::map
//can cause problems since it maybe reordered and the logical ordering might be lost.
//The extra parameter non_fixed_joints allows to only add segments with movable joinsts

void get_tree_segment_names(Tree tree, std::vector<std::string> &link_names,
		bool non_fixed_joints) {
	SegmentMap::const_iterator root_it = tree.getRootSegment();
	const TreeElement& currentElement = root_it->second;
	link_names.push_back(currentElement.segment.getName());
	for (std::vector<SegmentMap::const_iterator>::const_iterator child_it =
			currentElement.children.begin();
			child_it != currentElement.children.end(); ++child_it) {
		get_tree_segment_names_recursive(*child_it, link_names,
				non_fixed_joints);
	}

}

void get_tree_segment_names_recursive(const SegmentMap::const_iterator& it,
		std::vector<std::string> &link_names, bool no_fixed_joints) {

	const TreeElement& currentElement = it->second;

	if (currentElement.segment.getJoint().getType() == Joint::None
			&& no_fixed_joints) {
	} else {
		link_names.push_back(currentElement.segment.getName());
	}

	for (std::vector<SegmentMap::const_iterator>::const_iterator child_it =
			currentElement.children.begin();
			child_it != currentElement.children.end(); ++child_it) {
		get_tree_segment_names_recursive(*child_it, link_names,
				no_fixed_joints);
	}
}

void get_tree_joint_names(Tree tree, std::vector<std::string> &joint_names,
		bool non_fixed_joints) {
	SegmentMap::const_iterator root_it = tree.getRootSegment();
	const TreeElement& currentElement = root_it->second;
	//joint_names.push_back(currentElement.segment.getJoint().getName());
	for (std::vector<SegmentMap::const_iterator>::const_iterator child_it =
			currentElement.children.begin();
			child_it != currentElement.children.end(); ++child_it) {
		get_tree_joint_names_recursive(*child_it, joint_names,
				non_fixed_joints);
	}
}

void get_tree_joint_names_recursive(const SegmentMap::const_iterator& it,
		std::vector<std::string> &joint_names, bool no_fixed_joints) {

	const TreeElement& currentElement = it->second;

	if (currentElement.segment.getJoint().getType() == Joint::None
			&& no_fixed_joints) {
	} else {
		joint_names.push_back(currentElement.segment.getJoint().getName());
	}

	for (std::vector<SegmentMap::const_iterator>::const_iterator child_it =
			currentElement.children.begin();
			child_it != currentElement.children.end(); ++child_it) {
		get_tree_joint_names_recursive(*child_it, joint_names, no_fixed_joints);
	}
}

}
