#include "ros/ros.h"

// inside package
#include "ugv_localization/wheel_vel_odom.hpp"
#include "ugv_localization/gps_navheading_subpub.hpp"
#include "ugv_localization/gps_odom.hpp"


class TopicProcess
{
	private:
		ros::NodeHandle nh_;

		// respective topic handlers		
		WheelVelOdom* wheel_vel_odom_;
		GPSNavHeadingSubPub* gps_navheading_subpub_;
		GPSOdom* gps_odom_;

		// WheelVelOdom parameters
		std::string wvo_topic_in_;
		std::string wvo_topic_out_;

		// GPSNavHeadingSubPub parameters
		std::string gnsp_topic_out_;
		double gnsp_cov_thresh_;

		// GPSOdom parameters
		std::string go_coord_sys_;
		std::string go_topic_in_;
		std::string go_topic_out_;
		double go_lat_;
		double go_lon_;
		double go_yaw_;
		std::string go_frame_id_;
		std::string go_child_frame_id_;
		double go_cov_thresh_;
		double go_cov_scale_;

	public:
		TopicProcess(ros::NodeHandle& nh) : nh_(nh) {
			TopicProcess::setupParams();
			
			wheel_vel_odom_ = new WheelVelOdom(nh_, wvo_topic_in_, wvo_topic_out_);
			gps_navheading_subpub_ = new GPSNavHeadingSubPub(nh_, gnsp_topic_out_, gnsp_cov_thresh_);
			gps_odom_ = new GPSOdom(nh_, go_coord_sys_, go_topic_in_, go_topic_out_, go_lat_, go_lon_, go_yaw_, go_frame_id_, go_child_frame_id_, go_cov_thresh_, go_cov_scale_);
		}

		~TopicProcess() {}

		void setupParams() {
			std::string node_name = ros::this_node::getName();			

			nh_.getParam(node_name + "/wheel_odom_topic_in", wvo_topic_in_);
			nh_.getParam(node_name + "/wheel_odom_topic_out", wvo_topic_out_);

			nh_.getParam(node_name + "/gps_navheading_topic_out", gnsp_topic_out_);
			nh_.getParam(node_name + "/gps_navheading_cov_thresh", gnsp_cov_thresh_);

			nh_.getParam(node_name + "/gps_odom_coord_sys", go_coord_sys_);
			nh_.getParam(node_name + "/gps_odom_topic_in", go_topic_in_);
			nh_.getParam(node_name + "/gps_odom_topic_out", go_topic_out_);
			nh_.getParam(node_name + "/world_origin_latitude", go_lat_);
			nh_.getParam(node_name + "/world_origin_longitude", go_lon_);
			nh_.getParam(node_name + "/world_frame_yaw", go_yaw_);
			nh_.getParam(node_name + "/gps_odom_frame_id", go_frame_id_);
			nh_.getParam(node_name + "/gps_odom_child_frame_id", go_child_frame_id_);
			nh_.getParam(node_name + "/gps_odom_cov_thresh", go_cov_thresh_);
			nh_.getParam(node_name + "/gps_odom_cov_scale", go_cov_scale_);
		}
};


int main (int argc, char** argv)
{
	ros::init(argc, argv, "topic_process_node");
	ros::NodeHandle nh;

	TopicProcess topic_process(nh);

	ros::spin();
	return 0;
}






		
