#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

// message
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/TransformStamped.h"

// robot_localization
#include "robot_localization/navsat_conversions.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "datum_tf_node");
	ros::NodeHandle nh;

	// params
	double lat, lon, yaw;
	std::string frame_id, child_frame_id;

	std::string node_name = ros::this_node::getName();
	nh.getParam(node_name + "/latitude", lat);
	nh.getParam(node_name + "/longitude", lon);
	nh.getParam(node_name + "/yaw", yaw);
	nh.getParam(node_name + "/frame_id", frame_id);
	nh.getParam(node_name + "/child_frame_id", child_frame_id);

	// results
	double utm_x, utm_y;
	std::string utm_zone_tmp;

	RobotLocalization::NavsatConversions::LLtoUTM(lat, lon, utm_y, utm_x, utm_zone_tmp);

	// publish static tf
	static tf2_ros::StaticTransformBroadcaster br;
	geometry_msgs::TransformStamped msg_tf;

	msg_tf.header.stamp = ros::Time::now();
	msg_tf.header.frame_id = frame_id;
	msg_tf.child_frame_id = child_frame_id;
	
	msg_tf.transform.translation.x = utm_x;
	msg_tf.transform.translation.y = utm_y;
	msg_tf.transform.translation.z = 0.0;

	tf2::Quaternion q;
	q.setRPY(0.0, 0.0, yaw);
	msg_tf.transform.rotation.x = q.x();
	msg_tf.transform.rotation.y = q.y();
	msg_tf.transform.rotation.z = q.z();
	msg_tf.transform.rotation.w = q.w();

	br.sendTransform(msg_tf);
	ros::spin();
	return 0;
}
	
