#include "ros/ros.h"
#include "tf/transform_broadcaster.h"

// message
#include "sensor_msgs/Imu.h"
#include "ublox_msgs/NavRELPOSNED9.h"

class GPSNavHeadingSubPub
{
	private:
		ros::NodeHandle nh_;
		ros::Subscriber sub_;
		ros::Publisher pub_;

		std::string topic_out_;

		double last_time_;
		double cov_thresh_;

	public:
		GPSNavHeadingSubPub(ros::NodeHandle& nh, std::string topic_out, double cov_thresh);

		~GPSNavHeadingSubPub();

		void callbackNavRelPosNed(const ublox_msgs::NavRELPOSNED9 &m);
};
