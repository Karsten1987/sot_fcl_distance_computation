// Author Karsten Knese

#include <fcl/distance.h>
#include <fcl/shape/geometric_shapes.h>
#include <sensor_msgs/JointState.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <urdf_model/model.h>
#include <tf/transform_broadcaster.h>


namespace distance {

class DistanceComputation {
public:
	DistanceComputation(const sensor_msgs::JointStateConstPtr& msg);
	~DistanceComputation();
	void updateJointStates(const sensor_msgs::JointStateConstPtr& msg);
	void updateLinkStates(const KDL::Tree& tree);
	void filter_free_flyer(const sensor_msgs::JointState::ConstPtr& msg);
	bool parseCollisionObjects(urdf::Model const &robot_model);
	fcl::DistanceResult minimum_distance(const std::string linkAname, const std::string linkBname, fcl::Vec3f& relativeP1, fcl::Vec3f& relativeP2);
	void printLinks();
	void printJoints();
	std::string getBaseFrame()const;

private:
	KDL::Tree tree_;
    boost::shared_ptr< KDL::TreeFkSolverPos_recursive > robot_fk_;
	urdf::Model model_;
//	std::map<std::string, std::vector<double> > joint_states_;
	std::vector<double> joint_states_;
	std::vector<std::string> joint_names_;
	std::map<std::string, int> link_states_;
    std::vector<std::string> link_names;

    //Vector with fcl collision geometries
    std::map<std::string,boost::shared_ptr<fcl::CollisionGeometry> > shapes_;
    std::map<std::string,boost::shared_ptr<fcl::Capsule> > capsules_;
    std::map<std::string,boost::shared_ptr<fcl::CollisionObject> > collision_objects_;

    //Vector with local transform of the collision shape in the corresponding link
    std::map<std::string,KDL::Frame> shape_frames;
    //PREALOCATION
    KDL::JntArray joints_kdl;
    KDL::Frame getTfcollisions(std::string linkName, std::string prefix="debug");
    tf::TransformBroadcaster br;
	std::string base_frame_;
};

}  // namespace DistanceComputation
