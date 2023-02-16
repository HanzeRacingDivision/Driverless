#include "ros/ros.h"

// message
#include "nav_msgs/Odometry.h"
#include "rampage_msgs/WheelOdometry.h"


class WheelVelOdom
{
	private:
		ros::NodeHandle nh_;
		ros::Subscriber sub_;
		ros::Publisher pub_;

		std::string topic_in_;
		std::string topic_out_;

		int seq_;

	public:
		WheelVelOdom(ros::NodeHandle& nh, std::string topic_in, std::string topic_out);

		~WheelVelOdom();

		void callbackWheelOdometry(const rampage_msgs::WheelOdometry &m);

		double motorRpmToVelocity(double rpm);
};
