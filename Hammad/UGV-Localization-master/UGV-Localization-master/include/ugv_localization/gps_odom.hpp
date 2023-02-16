// ROS
#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"

// message
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"

// coordinate conversions
#include "robot_localization/navsat_conversions.h"
#include "swri_transform_util/local_xy_util.h"


class GPSOdom
{
	private:
		ros::NodeHandle nh_;
		ros::Subscriber sub;
		ros::Publisher pub;

		std::string coord_sys_; // global coordinate system, either "wgs84" or "utm"
		std::string topic_gps_, topic_odom_; // topic to subscribe, and to publish
		double lat_, lon_, yaw_; // origin of map frame
		std::string frame_id_, child_frame_id_;
		double cov_threshold_; // threshold for covariance to reject bad data
		double cov_scale_; // re-scale the covariance

		tf2::Transform tf_map2utm;

		double last_time_;

	public:
		GPSOdom(ros::NodeHandle& nh, 
				std::string coord_sys,
				std::string topic_gps, std::string topic_odom, 
				double lat, double lon, double yaw, 
				std::string frame_id, std::string child_frame_id,
				double cov_threshold, double cov_scale);
		~GPSOdom();
		void callback(const sensor_msgs::NavSatFix& msg_gps);
};


