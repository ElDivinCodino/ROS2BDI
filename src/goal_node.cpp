#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "bdi_ros2/msg/goal_bool.hpp"
#include "bdi_ros2/msg/goal_string.hpp"
#include "bdi_ros2/msg/goal_int.hpp"
#include "bdi_ros2/msg/goal_float.hpp"

auto CreateGoalBoolMsg(std::string goal, bool value, int priority, float deadline)
{
    auto msg = std::make_shared<bdi_ros2::msg::GoalBool>();

    msg->name = goal;
    msg->value = value;
    msg->priority = priority;
    msg->deadline = deadline;

    return msg;
}

auto CreateGoalStringMsg(std::string goal, std::string value, int priority, float deadline)
{
    auto msg = std::make_shared<bdi_ros2::msg::GoalString>();

    msg->name = goal;
    msg->value = value;
    msg->priority = priority;
    msg->deadline = deadline;

    return msg;
}

auto CreateGoalIntMsg(std::string goal, int value, int priority, float deadline)
{
    auto msg = std::make_shared<bdi_ros2::msg::GoalInt>();

    msg->name = goal;
    msg->value = value;
    msg->priority = priority;
    msg->deadline = deadline;

    return msg;
}

auto CreateGoalFloatMsg(std::string goal, float value, int priority, float deadline)
{
    auto msg = std::make_shared<bdi_ros2::msg::GoalFloat>();

    msg->name = goal;
    msg->value = value;
    msg->priority = priority;
    msg->deadline = deadline;

    return msg;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("goal_node");

    // Set the QoS. ROS 2 will provide QoS profiles based on the following use cases:
    // Default QoS settings for publishers and subscriptions (rmw_qos_profile_default).
    // Services (rmw_qos_profile_services_default).
    // Sensor data (rmw_qos_profile_sensor_data).
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;

    // set the depth to the QoS profile
    custom_qos_profile.depth = 7;

    // create a publisher for a specific type of message
    //auto chatter_pub = node->create_publisher<bdi_ros2::msg::MessageType>("topic_name", custom_qos_profile);

    rclcpp::WallRate loop_rate(2);

    // create a message
    //auto msg = CreateGoalBoolMsg("GoalName", bool, priority, deadline);
    //auto msg = CreateGoalStringMsg("GoalName", "string", priority, deadline);
    //auto msg = CreateGoalIntMsg("GoalName", int, priority, deadline);
    //auto msg = CreateGoalFloatMsg("GoalName", float, priority, deadline);

    // publish the message
    //chatter_pub->publish(msg);
    //std::cout << "Publishing GOAL message: Goal: " << msg->name << ", value: " << msg->value << ", priority: " << msg->priority << ", deadline: " << msg->deadline << std::endl;

    rclcpp::spin_some(node);

    /*while (rclcpp::ok()) {
    
    std::cout << "Publishing incremental float: '" << msg->value << "'" << std::endl;
    chatter_pub->publish(msg);

    // spin: Blocking call, do work indefinitely as it comes in.
    // spin_once: Do one "cycle" of work, with optional timeout.
    // spin_some: Do all the work that is immediately available to the executor.
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }*/

    return 0;
}
