#ifndef _KDL_UTILS_
#define _KDL_UTILS_

#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace KDL{

  Tree get_sub_tree(Tree tree, std::vector<std::string> tips, std::string root);

  void get_tree_segment_names(Tree tree, std::vector<std::string> &link_names, bool non_fixed_joints = false);

  /// @todo docuemnt what non fixed joint names means
  void get_tree_joint_names(Tree tree, std::vector<std::string> &joint_names, bool non_fixed_joints = false);

  /// @todo: Why does this need to be in the header, why can it just be in the cpp?
  void get_tree_segment_names_recursive(const SegmentMap::const_iterator& it, std::vector<std::string> &link_names, bool no_fixed_joints);

  void get_tree_joint_names_recursive(const SegmentMap::const_iterator& it, std::vector<std::string> &joint_names, bool no_fixed_joints);

}

#endif
