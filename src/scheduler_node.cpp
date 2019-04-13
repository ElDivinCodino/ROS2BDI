#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "bdi_ros2/msg/goal_bool.hpp"
#include "bdi_ros2/msg/goal_string.hpp"
#include "bdi_ros2/msg/goal_int.hpp"
#include "bdi_ros2/msg/goal_float.hpp"
#include "bdi_ros2/msg/belief_bool.hpp"
#include "bdi_ros2/msg/belief_string.hpp"
#include "bdi_ros2/msg/belief_int.hpp"
#include "bdi_ros2/msg/belief_float.hpp"

void boolBeliefCallback(const bdi_ros2::msg::BeliefBool::SharedPtr msg)
{
    std::cout << "Received a Belief message! Belief: " << msg->name << "value: " << msg->value << ", priority: " << msg->priority << ", deadline: " << msg->deadline << std::endl;
}

void stringBeliefCallback(const bdi_ros2::msg::BeliefString::SharedPtr msg)
{
    std::cout << "Received a Belief message! Belief: " << msg->name << "value: " << msg->value << ", priority: " << msg->priority << ", deadline: " << msg->deadline << std::endl;
}

void intBeliefCallback(const bdi_ros2::msg::BeliefInt::SharedPtr msg)
{
    std::cout << "Received a Belief message! Belief: " << msg->name << "value: " << msg->value << ", priority: " << msg->priority << ", deadline: " << msg->deadline << std::endl;
}

void floatBeliefCallback(const bdi_ros2::msg::BeliefFloat::SharedPtr msg)
{
    std::cout << "Received a Belief message! Belief: " << msg->name << "value: " << msg->value << ", priority: " << msg->priority << ", deadline: " << msg->deadline << std::endl;
}

void boolGoalCallback(const bdi_ros2::msg::GoalBool::SharedPtr msg)
{
    std::cout << "Received a Goal message! Goal: " << msg->name << "value: " << msg->value << ", priority: " << msg->priority << ", deadline: " << msg->deadline << std::endl;
}

void stringGoalCallback(const bdi_ros2::msg::GoalString::SharedPtr msg)
{
    std::cout << "Received a Goal message! Goal: " << msg->name << "value: " << msg->value << ", priority: " << msg->priority << ", deadline: " << msg->deadline << std::endl;
}

void intGoalCallback(const bdi_ros2::msg::GoalInt::SharedPtr msg)
{
    std::cout << "Received a Goal message! Goal: " << msg->name << "value: " << msg->value << ", priority: " << msg->priority << ", deadline: " << msg->deadline << std::endl;
}

void floatGoalCallback(const bdi_ros2::msg::GoalFloat::SharedPtr msg)
{
    std::cout << "Received a Goal message! Goal: " << msg->name << "value: " << msg->value << ", priority: " << msg->priority << ", deadline: " << msg->deadline << std::endl;
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // create a node
    auto node = rclcpp::Node::make_shared("scheduler_node");

    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;

    // set the depth to the QoS profile
    custom_qos_profile.depth = 7;
    custom_qos_profile.reliability = rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE;

    // the scheduler needs to listen for every possible type of message in both topics
    auto boolBeliefSub = node->create_subscription<bdi_ros2::msg::BeliefBool>("belief", boolBeliefCallback, custom_qos_profile);
    auto stringBeliefSub = node->create_subscription<bdi_ros2::msg::BeliefString>("belief", stringBeliefCallback, custom_qos_profile);
    auto intBeliefSub = node->create_subscription<bdi_ros2::msg::BeliefInt>("belief", intBeliefCallback, custom_qos_profile);
    auto floatBeliefSub = node->create_subscription<bdi_ros2::msg::BeliefFloat>("belief", floatBeliefCallback, custom_qos_profile);

    auto boolGoalSub = node->create_subscription<bdi_ros2::msg::GoalBool>("goal", boolGoalCallback, custom_qos_profile);
    auto stringGoalSub = node->create_subscription<bdi_ros2::msg::GoalString>("goal", stringGoalCallback, custom_qos_profile);
    auto intGoalSub = node->create_subscription<bdi_ros2::msg::GoalInt>("goal", intGoalCallback, custom_qos_profile);
    auto floatGoalSub = node->create_subscription<bdi_ros2::msg::GoalFloat>("goal", floatGoalCallback, custom_qos_profile);

    // spin: Blocking call, do work indefinitely as it comes in.
    //  spin_once: Do one "cycle" of work, with optional timeout.
    //  spin_some: Do all the work that is immediately available to the executor.
    rclcpp::spin(node);

    return 0;
}