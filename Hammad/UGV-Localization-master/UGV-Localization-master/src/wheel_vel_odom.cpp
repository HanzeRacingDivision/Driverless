#include "ugv_localization/wheel_vel_odom.hpp"


WheelVelOdom::WheelVelOdom(ros::NodeHandle& nh, std::string topic_in, std::string topic_out) : nh_(nh), topic_in_(topic_in), topic_out_(topic_out)
{
	sub_ = nh_.subscribe(topic_in_, 10, &WheelVelOdom::callbackWheelOdometry, this);
	pub_ = nh_.advertise<nav_msgs::Odometry>(topic_out_, 10);

	seq_ = 0;
}

WheelVelOdom::~WheelVelOdom() {}

void WheelVelOdom::callbackWheelOdometry(const rampage_msgs::WheelOdometry &m)
{
	nav_msgs::Odometry msg_odom;
	msg_odom.header.seq = seq_;
	msg_odom.header.stamp = m.t_epoch;
	msg_odom.header.frame_id = "odom";
	msg_odom.child_frame_id = "base_link";

	msg_odom.twist.twist.linear.x = WheelVelOdom::motorRpmToVelocity(m.delta_ticks);
	msg_odom.twist.twist.linear.y = 0;
	msg_odom.twist.twist.linear.z = 0;

	pub_.publish(msg_odom);
	seq_++;
}

double WheelVelOdom::motorRpmToVelocity(double rpm) { return 0.0006046 * rpm; }

