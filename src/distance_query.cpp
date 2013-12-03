/**
 @author Karsten Knese
 @version 0.1 15/10/2013
 */

#include <sot_fcl_distance_computation/DistanceComputation.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Transform.h>
//#include <fcl/distance.h>

static const std::string JOINT_STATE_TOPIC = "/dynamic_graph/joint_states";
static const std::string RESULT_MARKER_TOPIC = "distance_query/result_marker";
static const std::string RESULT_GEOMETRY_TOPIC_1 =
		"distance_query/result_geometry_p1";
static const std::string RESULT_GEOMETRY_TOPIC_2 =
		"distance_query/result_geometry_p2";
static const std::string RESULT_GEOMETRY_TOPIC_1rel =
		"distance_query/result_geometry_p1rel";
static const std::string RESULT_GEOMETRY_TOPIC_2rel =
		"distance_query/result_geometry_p2rel";

int id_counter = 1;
std::string base_frame = "base_link";

//Draw a point
bool draw_point(const double x, const double y, const double z,
		const std::string frame, visualization_msgs::Marker& marker, float color=1.0) {
	//DRAW REFERENCE
	marker.header.frame_id = frame;
	marker.header.stamp = ros::Time().now();
	marker.ns = "goal";
	marker.id = ++id_counter;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = z;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.02;
	marker.scale.y = 0.02;
	marker.scale.z = 0.02;

	marker.color.r = color;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	marker.color.a = 1.0;
	return true;
}

void createMarkerArray(const fcl::DistanceResult& result,
		const boost::shared_ptr<visualization_msgs::MarkerArray>& markers) {
	visualization_msgs::Marker m1;
	draw_point(result.nearest_points[0].data[0],
			result.nearest_points[0].data[1], result.nearest_points[0].data[2],
			base_frame, m1, 0);

	visualization_msgs::Marker m2;
	draw_point(result.nearest_points[1].data[0],
			result.nearest_points[1].data[1], result.nearest_points[1].data[2],
			base_frame, m2, 1);

	markers->markers.push_back(m1);
	markers->markers.push_back(m2);
}

void createRelativeMarkers(fcl::Vec3f& point, const std::string frame,
		const boost::shared_ptr<visualization_msgs::MarkerArray>& markers, float color) {
	visualization_msgs::Marker m;
	draw_point(point.data[0], point.data[1], point.data[2], frame, m, color);
	markers->markers.push_back(m);
}

void createGeometryPoint(const fcl::Vec3f& point,
		const boost::shared_ptr<geometry_msgs::Transform>& msg) {
	msg->translation.x = point[0];
	msg->translation.y = point[1];
	msg->translation.z = point[2];
	msg->rotation.x = 0;
	msg->rotation.y = 0;
	msg->rotation.z = 0;
	msg->rotation.w = 1;
}

int main(int argc, char** argv) {

	std::string s1, s2;
	s1 = argv[1];
	s2 = argv[2];

	ROS_INFO("used links: %s & %s", s1.c_str(), s2.c_str());

	ros::init(argc, argv, "distance_computation");
	ros::NodeHandle nh;
	double rate = 100;
	ros::Rate loopRate(rate);

	const sensor_msgs::JointStateConstPtr initJoints =
			ros::topic::waitForMessage<sensor_msgs::JointState>(
					JOINT_STATE_TOPIC, nh);

	ROS_INFO("Joint States published");
	ROS_INFO("length of joint states %d", initJoints->name.size());

	const boost::shared_ptr<distance::DistanceComputation> distance_comp(
			new distance::DistanceComputation(initJoints));

	base_frame = distance_comp->getBaseFrame();

	ros::Subscriber joint_states_subscriber = nh.subscribe<
			sensor_msgs::JointState>(JOINT_STATE_TOPIC, 1, // Buffer size
			&distance::DistanceComputation::updateJointStates, distance_comp);

	ros::Publisher resultMarkerPub = nh.advertise<
			visualization_msgs::MarkerArray>(RESULT_MARKER_TOPIC, 10);
	boost::shared_ptr<visualization_msgs::MarkerArray> markers(
			new visualization_msgs::MarkerArray);

//	/geometry_msgs/Transform	sot::MatrixHomogeneous	matrixHomogeneous / matrixHomogeneousStamped
//	Point Publisher in Absolute Coordinates (saying in respect of the base link)
	ros::Publisher geometricPointPub1 = nh.advertise<geometry_msgs::Transform>(
			RESULT_GEOMETRY_TOPIC_1, 10);
	boost::shared_ptr<geometry_msgs::Transform> p1(
			new geometry_msgs::Transform);

	ros::Publisher geometricPointPub2 = nh.advertise<geometry_msgs::Transform>(
			RESULT_GEOMETRY_TOPIC_2, 10);
	boost::shared_ptr<geometry_msgs::Transform> p2(
			new geometry_msgs::Transform);

	//	Point Publisher in Relative Coordinates (saying in respect of each of the link Frames)
	ros::Publisher geometricPointPub1rel = nh.advertise<geometry_msgs::Transform>(
			RESULT_GEOMETRY_TOPIC_1rel, 10);
	boost::shared_ptr<geometry_msgs::Transform> p1rel(
			new geometry_msgs::Transform);

	ros::Publisher geometricPointPub2rel = nh.advertise<geometry_msgs::Transform>(
			RESULT_GEOMETRY_TOPIC_2rel, 10);
	boost::shared_ptr<geometry_msgs::Transform> p2rel(
			new geometry_msgs::Transform);

	distance_comp->printLinks();
	distance_comp->printJoints();

	fcl::Vec3f relativeP1;
	fcl::Vec3f relativeP2;

	while (ros::ok()) {
		fcl::DistanceResult result = distance_comp->minimum_distance(s1, s2, relativeP1, relativeP2);
		if (!result.nearest_points[0].isZero()
				&& !result.nearest_points[1].isZero()) {

			markers->markers.clear();
			id_counter = 0;

			createMarkerArray(result, markers);
			createRelativeMarkers(relativeP1, s1, markers, 0);
			createRelativeMarkers(relativeP2, s2, markers,1);
			resultMarkerPub.publish(markers);

			createGeometryPoint(result.nearest_points[0], p1);
			createGeometryPoint(result.nearest_points[1], p2);
			createGeometryPoint(relativeP1, p1rel);
			createGeometryPoint(relativeP2, p2rel);
			geometricPointPub1.publish(p1);
			geometricPointPub2.publish(p2);
			geometricPointPub1rel.publish(p1rel);
			geometricPointPub2rel.publish(p2rel);
		}
		ros::spinOnce();
		loopRate.sleep();
	}

	return EXIT_SUCCESS;
}
