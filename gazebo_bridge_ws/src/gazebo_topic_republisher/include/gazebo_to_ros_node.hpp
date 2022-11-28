#ifndef GAZEBO_TO_ROS_
#define GAZEBO_TO_ROS_

#include <gazebo/gazebo_config.h>

#include <cstdio>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include "gazebo_bridge_interfaces_msgs/msg/gazebo_to_ros.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

gazebo::transport::SubscriberPtr subGazebo;
rclcpp::Publisher<gazebo_bridge_interfaces_msgs::msg::GazeboToRos>::SharedPtr pubDronePose;

class GazeboToRosBridge : public rclcpp::Node
{
   public:
	GazeboToRosBridge();
	~GazeboToRosBridge();

	const rclcpp::Publisher<gazebo_bridge_interfaces_msgs::msg::GazeboToRos>::SharedPtr& getPublisher();

   private:
	rclcpp::Publisher<gazebo_bridge_interfaces_msgs::msg::GazeboToRos>::SharedPtr pubDronePose;

	// gazebo_bridge_interfaces_msgs::msg::GazeboToRos msgPoseRos;
};

#endif