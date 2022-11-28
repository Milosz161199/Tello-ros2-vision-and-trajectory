#include "gazebo_to_ros_node.hpp"

GazeboToRosBridge::GazeboToRosBridge() : Node("gazebo_to_ros_bridge")
{
	RCLCPP_INFO(this->get_logger(), "Starting Gazebo to ROS2 Node.");

	pubDronePose = this->create_publisher<gazebo_bridge_interfaces_msgs::msg::GazeboToRos>("drone_pose", 10);
}

GazeboToRosBridge::~GazeboToRosBridge()
{
	std::cout << "Gazebo to ROS2 Bridge being killed" << std::endl;
	rclcpp::shutdown();
}

void poseCallback(ConstPosePtr& _msg)
{
	gazebo_bridge_interfaces_msgs::msg::GazeboToRos msgPoseRos;
	if (_msg->name() == "tello_1")
	{
		msgPoseRos.pos_x = _msg->position().x();
		msgPoseRos.pos_y = _msg->position().y();
		msgPoseRos.pos_z = _msg->position().z();

		pubDronePose->publish(msgPoseRos);
	}
	
}

int main(int argc, char** argv)
{
	gazebo::client::setup(argc, argv);
	rclcpp::init(argc, argv);

	gazebo::transport::NodePtr nodeGazebo(new gazebo::transport::Node());
	nodeGazebo->Init();

	auto node_ros = std::make_shared<rclcpp::Node>("gazebo_to_ros_bridge");

	pubDronePose = node_ros->create_publisher<gazebo_bridge_interfaces_msgs::msg::GazeboToRos>("drone_pose", 10);

	// Listen to Gazebo contacts topic
	subGazebo = nodeGazebo->Subscribe("/gazebo/default/pose/info", poseCallback);

	while(rclcpp::ok())
    {
        gazebo::common::Time::MSleep(20);
        rclcpp::spin_some(node_ros);
    }
    rclcpp::shutdown();
    gazebo::client::shutdown();
	return 0;
}
