#include "ugv_localization/gps_navheading_subpub.hpp"


int main(int argc, char** argv)
{
	ros::init(argc, argv, "gps_navheading_node");
	ros::NodeHandle nh;

	std::string node_name = ros::this_node::getName();

	std::string topic_out;
	double cov_thresh;

    nh.getParam(node_name+"/topic_out", topic_out);
	nh.getParam(node_name+"/cov_thresh", cov_thresh);

	GPSNavHeadingSubPub gps_navheading_subpub(nh, topic_out, cov_thresh);

	ros::spin();
	return 0;
}
