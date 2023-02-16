#include "ugv_localization/gps_odom.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "gps_odom_node");
	ros::NodeHandle nh;

	std::string node_name = ros::this_node::getName();

	// params
	std::string coord_sys;
	std::string topic_gps, topic_odom;
	double lat, lon, yaw;
	std::string frame_id, child_frame_id;
	double cov_threshold;
	double cov_scale;

	nh.getParam(node_name+"/coordinate_system", coord_sys);
	nh.getParam(node_name+"/topic_gps", topic_gps);
	nh.getParam(node_name+"/topic_odom", topic_odom);
	nh.getParam(node_name+"/latitude", lat);
	nh.getParam(node_name+"/longitude", lon);
	nh.getParam(node_name+"/yaw", yaw);
	nh.getParam(node_name+"/frame_id", frame_id);
	nh.getParam(node_name+"/child_frame_id", child_frame_id);
	nh.getParam(node_name+"/cov_threshold", cov_threshold);
	nh.getParam(node_name+"/cov_scale", cov_scale);

	GPSOdom gps_odom(nh, coord_sys, topic_gps, topic_odom, lat, lon, yaw, frame_id, child_frame_id, cov_threshold, cov_scale);
	ros::spin();
	return 0;
}
