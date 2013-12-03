#include <sot_fcl_distance_computation/DistanceComputation.h>
#include <fcl/BV/OBBRSS.h>
#include <fcl/BVH/BVH_model.h>
#include <sot_fcl_distance_computation/conversions.h>
#include <sot_fcl_distance_computation/kdl_tools.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>

using namespace distance;

DistanceComputation::DistanceComputation(
		const sensor_msgs::JointStateConstPtr& msg) {
	if (!model_.initParam("robot_description")) {
		ROS_ERROR("Failed to parse urdf file");
	}
	ROS_INFO("Successfully parsed urdf file");

	if (!kdl_parser::treeFromUrdfModel(model_, tree_)) {
		ROS_ERROR("Failed to construct kdl tree");
	} else {
		ROS_DEBUG_STREAM("Successfully created kdl tree");
		std::cerr << "number of joint: " << tree_.getNrOfJoints() << std::endl;
		std::cerr << "number of links: " << tree_.getNrOfSegments()
				<< std::endl;
	}

	robot_fk_.reset(new KDL::TreeFkSolverPos_recursive(tree_));
	base_frame_ = tree_.getRootSegment()->second.segment.getName();
	std::cerr << "root/base frame: " << base_frame_ << std::endl;

	updateJointStates(msg);
	updateLinkStates(tree_);
	parseCollisionObjects(model_);

	joints_kdl.resize(joint_states_.size());
}

std::string DistanceComputation::getBaseFrame() const {
	return this->base_frame_;
}

DistanceComputation::~DistanceComputation() {
}

void DistanceComputation::printLinks() {
	ROS_INFO("Printing Link Information");
	ROS_INFO("Total Amount of Links: %d", link_states_.size());
	typedef std::map<std::string, int>::iterator it_type;
	for (it_type iterator = link_states_.begin();
			iterator != link_states_.end(); iterator++) {
		ROS_INFO("Link: %s on position: %d", iterator->first.c_str(),
				iterator->second);
	}
}

void DistanceComputation::printJoints() {
	ROS_INFO("Printing Joint Information");
	ROS_INFO("Total Amount of Joints: %d", joint_states_.size());
	for (unsigned int var = 0; var < joint_states_.size(); ++var) {
		ROS_INFO("joint order: %s, with value: %f", joint_names_[var].c_str(),
				joint_states_[var]);
	}
}

void DistanceComputation::filter_free_flyer(
		const sensor_msgs::JointState::ConstPtr& joints_msg) {
	joint_states_.clear();
	joint_names_.clear();
	for (unsigned int i = 0; i < joints_msg->name.size(); ++i) {

		std::string joint_name = std::string(joints_msg->name[i]);
		size_t baseFound = joint_name.find("base_joint_");
//		size_t baseFound = joint_name.find("WAIST_");
		if (baseFound == std::string::npos) {

			joint_states_.push_back(joints_msg->position[i]);
			joint_names_.push_back(joints_msg->name[i]);
		}
	}
}

void DistanceComputation::updateJointStates(
		const sensor_msgs::JointStateConstPtr& msg) {

	ROS_INFO("update joint states");
	filter_free_flyer(msg);
}

void DistanceComputation::updateLinkStates(const KDL::Tree& tree) {
	//Get the list of link names;
	get_tree_segment_names(tree_, link_names, false);
	for (unsigned int i = 0; i < link_names.size(); ++i) {
		link_states_[link_names[i]] = i;
	}
}

bool DistanceComputation::parseCollisionObjects(
		urdf::Model const &robot_model) {
	ROS_DEBUG_STREAM("parsing collision objects");
	ROS_DEBUG_STREAM("number of joints of tree"<<tree_.getNrOfJoints());

	typedef std::map<std::string, int>::iterator it_type;
	for (it_type iterator = link_states_.begin();
			iterator != link_states_.end(); iterator++) {

		boost::shared_ptr<urdf::Link> link;
		model_.getLink(iterator->first, link);

		if (link->collision) {

			ROS_INFO_STREAM(
					"Collision for link: "<<link->name<<" is of type "<< link->collision->geometry->type);

			if (link->collision->geometry->type == urdf::Geometry::CYLINDER) {

				boost::shared_ptr<urdf::Cylinder> collisionGeometry =
						boost::dynamic_pointer_cast<urdf::Cylinder>(
								link->collision->geometry);

				 boost::shared_ptr<fcl::Capsule>capsule = boost::shared_ptr<fcl::Capsule>(new fcl::Capsule(collisionGeometry->radius, collisionGeometry->length));

				boost::shared_ptr<fcl::CollisionObject> collision_object(
						new fcl::CollisionObject(capsule));

				collision_objects_[link->name] = collision_object;
				capsules_[link->name] = capsule;

				// Transform down the origin of the CollisionShape to the TF link
//				link->collision->origin.position.z -= collisionGeometry->length
//						/ 2;
//				// Store the transformation of the center of the CollisionShape (URDF)
				// Apply this transformation later in the update cycle to have the shape at the same origin as the link
				convert(link->collision->origin, shape_frames[link->name]);
				ROS_INFO("adding capsule for %s",link->name.c_str() );
			}
            else if(link->collision->geometry->type == urdf::Geometry::MESH){

                boost::shared_ptr< ::urdf::Mesh> collisionGeometry = boost::dynamic_pointer_cast< ::urdf::Mesh> (link->collision->geometry);

                shapes::Mesh *mesh = shapes::createMeshFromFilename(collisionGeometry->filename);

                std::vector<fcl::Vec3f> vertices;
                std::vector<fcl::Triangle> triangles;

                for(unsigned int i=0; i < mesh->vertexCount; ++i){
                    fcl::Vec3f v(mesh->vertices[3*i]*collisionGeometry->scale.x,
                                 mesh->vertices[3*i + 1]*collisionGeometry->scale.y,
                                 mesh->vertices[3*i + 2]*collisionGeometry->scale.z);

                    vertices.push_back(v);
                }

                for(unsigned int i=0; i< mesh->triangleCount; ++i){
                    fcl::Triangle t(mesh->triangles[3*i],
                                    mesh->triangles[3*i + 1],
                                    mesh->triangles[3*i + 2]);
                    triangles.push_back(t);
                }

                // add the mesh data into the BVHModel structure
                boost::shared_ptr<fcl::BVHModel<fcl::OBBRSS> > shape(new fcl::BVHModel<fcl::OBBRSS>);
                shape->beginModel();
                shape->addSubModel(vertices, triangles);
                shape->endModel();

                boost::shared_ptr<fcl::CollisionObject> collision_object(new fcl::CollisionObject(shape));
                collision_objects_[link->name] = collision_object;
                convert(link->collision->origin, shape_frames[link->name]);
                ROS_INFO("adding mesh for %s",link->name.c_str() );

            }
		} else {
			ROS_WARN_STREAM("Collision not defined for link: "<<link->name);
		}
	}
	return true;
}

fcl::DistanceResult DistanceComputation::minimum_distance(std::string linkAname,
		std::string linkBname, fcl::Vec3f& relativeP1, fcl::Vec3f& relativeP2) {

	boost::shared_ptr<urdf::Link> linkA;
	model_.getLink(linkAname, linkA);

	boost::shared_ptr<urdf::Link> linkB;
	model_.getLink(linkBname, linkB);

//	printJoints();

	KDL::Frame kdlFrameA;
	KDL::Frame kdlFrameB;

	if (joint_states_.size() != tree_.getNrOfJoints()) {
		ROS_ERROR_STREAM(
				"There is a mismatch between the configured joints and the passed joints in no self collision of safety layer");
		ROS_ERROR_STREAM("number of joint_states " << joint_states_.size());
		ROS_ERROR_STREAM("number of tree joints " << tree_.getNrOfJoints());

	}

	convert(joint_states_, joints_kdl);

	int resultbla = robot_fk_->JntToCart(joints_kdl, kdlFrameA, linkAname);
	resultbla = robot_fk_->JntToCart(joints_kdl, kdlFrameB, linkBname);
	std::cerr << "result of foward kinematic: " << resultbla << std::endl;

	// Get all the Frames inside the origin of the link
//	kdlFrameA = kdlFrameA * shape_frames[linkAname];
//	kdlFrameB = kdlFrameB * shape_frames[linkBname];

	kdlFrameB = kdlFrameB * KDL::Frame(KDL::Vector(0,0,-0.2));
	//Check for collision
	// t1, t2 will result in the new points computed by the forward kinematic
	fcl::Transform3f t1, t2;
	t1 = kdl2fcl(kdlFrameA);
	t2 = kdl2fcl(kdlFrameB);

//	if (linkA->collision->origin.position.z < 0){
//		fcl::Vec3f tmp = t1.getTranslation();
//		tmp.data[2] -= 0.2;
//		t1.setTranslation(tmp);
//	}
//	if (linkB->collision->origin.position.z < 0){
//		fcl::Vec3f tmp = t2.getTranslation();
//		tmp.data[2] -= 0.2;
//		t2.setTranslation(tmp);
//		}

	fcl::CollisionObject* obj1 = collision_objects_[linkAname].get();
	fcl::CollisionObject* obj2 = collision_objects_[linkBname].get();

	obj1->setTransform(t1);
	obj2->setTransform(t2);

	fcl::DistanceRequest request;
	request.gjk_solver_type = fcl::GST_INDEP;
	request.enable_nearest_points = true;

	// result will be returned via the collision result structure
	fcl::DistanceResult result;

	// perform distance test
	fcl::distance(obj1, obj2, request, result);

	// p1Homo, p2Homo newly computed points by FCL
	// absolutely computed w.r.t. base-frame
	fcl::Transform3f p1Homo(result.nearest_points[0]);
	fcl::Transform3f p2Homo(result.nearest_points[1]);

	fcl::Transform3f sot_compensation;
	fcl::Matrix3f sot_rot(0, 1, 0, 0, 0, 1, 1, 0, 0);
	fcl::Vec3f sot_trans(0, 0, 0);
	sot_compensation.setRotation(sot_rot);
	sot_compensation.setTranslation(sot_trans);

	fcl::Transform3f relativeP1M, relativeP2M;
	if (linkA->parent_joint->type == urdf::Joint::FIXED) {
		std::cerr << linkA->name << " is endeffector " << std::endl;
		relativeP1M = t1.inverseTimes(p1Homo);
	} else {
		std::cerr << linkA->name << " with SoT compensation " << std::endl;
		t1 = t1 * sot_compensation;
		// check that inverseTimes doesn't modify *this
		relativeP1M = t1.inverseTimes(p1Homo);
	}

	if (linkB->parent_joint->type == urdf::Joint::FIXED) {
		std::cerr << linkB->name << " is endeffector " << std::endl;
		relativeP2M = t2.inverseTimes(p2Homo);
	} else {
		std::cerr << linkB->name << " with SoT compensation " << std::endl;
		t2 = t2 * sot_compensation;
		// check that inverseTimes doesn't modify *this
		relativeP2M = t2.inverseTimes(p2Homo);
		fcl::Transform3f shift;
		shift.setTranslation(fcl::Vec3f(-0.2,0,0));
		relativeP2M = relativeP2M*shift;
	}

	fcl::Transform3f blub2 = fcl::Transform3f(result.nearest_points[1]);
	fcl::Transform3f blubb2 = fcl::Transform3f(fcl::Vec3f(0,0,-0.2));

	relativeP1 = relativeP1M.getTranslation();
	relativeP2 = relativeP2M.getTranslation();

	std::cerr << "NEAREST POINT " << linkAname << result.nearest_points[0]
			<< std::endl;
	std::cerr << "ORIGIN POINT " << linkAname << obj1->getTranslation()
			<< std::endl;
//	std::cerr << "ORIGIN ROT " << linkAname << obj1->getRotation() << std::endl;
	std::cerr << "RELATIVE POINT " << linkAname << relativeP1 << std::endl;

	std::cerr << "NEAREST POINT " << linkBname << result.nearest_points[1]
			<< std::endl;
	std::cerr << "ORIGIN POINT " << linkBname << obj2->getTranslation()
			<< std::endl;
//	std::cerr << "ORIGIN ROT " << linkBname << obj2->getRotation() << std::endl;
	std::cerr << "RELATIVE POINT " << linkBname << relativeP2 << std::endl;

	std::cerr << "RESULTING DISTANCE: " << result.min_distance << std::endl;

//	getTfcollisions(linkAname, "KDL_FORWARD_");

	tf::Transform tf;
	convert(t2, tf);
	br.sendTransform(
			tf::StampedTransform(tf, ros::Time::now(), "base_link",
					"debug_SOT_ORIGIN"));

	/* BRUTAL HACK !!*/
//	relativeP1 = fcl::Vec3f();
//	result.nearest_points[1] = fcl::Vec3f(0.5,0.0,1.0);

	return result;
}

KDL::Frame DistanceComputation::getTfcollisions(std::string linkName,
		std::string prefix /*"DEBUG"*/) {
	KDL::Frame frame;
	int res = robot_fk_->JntToCart(joints_kdl, frame, linkName);
	if (res < 0) {
		ROS_ERROR_STREAM("Problem with fk in safety layer");
	}
//	frame = frame*shape_frames[linkName];
	tf::Transform tf;
	convert(frame, tf);
	br.sendTransform(
			tf::StampedTransform(tf, ros::Time::now(), "base_link",
					prefix + linkName));

	return frame;
}
