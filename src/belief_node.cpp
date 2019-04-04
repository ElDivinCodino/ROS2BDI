#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "bdi_ros2/msg/belief_msg.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("belief_node");

  // Set the QoS. ROS 2 will provide QoS profiles based on the following use cases:
  // Default QoS settings for publishers and subscriptions (rmw_qos_profile_default).
  // Services (rmw_qos_profile_services_default).
  // Sensor data (rmw_qos_profile_sensor_data).
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;

  // set the depth to the QoS profile
  custom_qos_profile.depth = 7;

  // create a publisher
  auto chatter_pub = node->create_publisher<bdi_ros2::msg::BeliefMsg>("chatter", custom_qos_profile);

  rclcpp::WallRate loop_rate(2);

  auto msg = std::make_shared<bdi_ros2::msg::BeliefMsg>();
  auto i = 1;

  while (rclcpp::ok()) {
    msg->value = ++i;
    msg->name = "Stringa";
    std::cout << "Publishing incremental float: '" << msg->value << "'" << std::endl;
    chatter_pub->publish(msg);

    // spin: Blocking call, do work indefinitely as it comes in.
    // spin_once: Do one "cycle" of work, with optional timeout.
    // spin_some: Do all the work that is immediately available to the executor.
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  return 0;
}
