#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "bdi_ros2/msg/belief_msg.hpp"
#include "bdi_ros2/msg/goal_msg.hpp"

void chatterCallback(const bdi_ros2::msg::BeliefMsg::SharedPtr msg)
{
  std::cout << "I heard: [" << msg->value << "]" << " from " << msg->name << std::endl;
}

void chatterCallback(const bdi_ros2::msg::GoalMsg::SharedPtr msg)
{
  std::cout << "I heard: [" << msg->value << "]" << " from " << msg->name << std::endl;
}

//create a subscriber
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // create a node
  auto node = rclcpp::Node::make_shared("scheduler_node");

  auto sub1 = node->create_subscription<bdi_ros2::msg::BeliefMsg>("belief", chatterCallback, rmw_qos_profile_default);

  auto sub2 = node->create_subscription<bdi_ros2::msg::GoalMsg>("goal", chatterCallback, rmw_qos_profile_default);

  // spin: Blocking call, do work indefinitely as it comes in.
 //  spin_once: Do one "cycle" of work, with optional timeout.
 //  spin_some: Do all the work that is immediately available to the executor.
  rclcpp::spin(node);

  return 0;
}