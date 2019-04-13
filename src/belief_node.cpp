#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "bdi_ros2/msg/belief_bool.hpp"
#include "bdi_ros2/msg/belief_string.hpp"
#include "bdi_ros2/msg/belief_int.hpp"
#include "bdi_ros2/msg/belief_float.hpp"

auto CreateBeliefBoolMsg(std::string goal, bool value, int priority, float deadline)
{
    auto msg = std::make_shared<bdi_ros2::msg::BeliefBool>();

    msg->name = goal;
    msg->value = value;
    msg->priority = priority;
    msg->deadline = deadline;

    return msg;
}

auto CreateBeliefStringMsg(std::string goal, std::string value, int priority, float deadline)
{
    auto msg = std::make_shared<bdi_ros2::msg::BeliefString>();

    msg->name = goal;
    msg->value = value;
    msg->priority = priority;
    msg->deadline = deadline;

    return msg;
}

auto CreateBeliefIntMsg(std::string goal, int value, int priority, float deadline)
{
    auto msg = std::make_shared<bdi_ros2::msg::BeliefInt>();

    msg->name = goal;
    msg->value = value;
    msg->priority = priority;
    msg->deadline = deadline;

    return msg;
}

auto CreateBeliefFloatMsg(std::string goal, float value, int priority, float deadline)
{
    auto msg = std::make_shared<bdi_ros2::msg::BeliefFloat>();

    msg->name = goal;
    msg->value = value;
    msg->priority = priority;
    msg->deadline = deadline;

    return msg;
}

int main(int argc, char *argv[])
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

    // create a publisher for a specific type of message
    //auto chatter_pub = node->create_publisher<bdi_ros2::msg::MessageType>("topic_name", custom_qos_profile);

    rclcpp::WallRate loop_rate(2);

    // create a message
    //auto msg = CreateBeliefBoolMsg("GoalName", bool, priority, deadline);
    //auto msg = CreateBeliefStringMsg("GoalName", "string", priority, deadline);
    //auto msg = CreateBeliefIntMsg("GoalName", int, priority, deadline);
    //auto msg = CreateBeliefFloatMsg("GoalName", float, priority, deadline);

    // publish the message
    //chatter_pub->publish(msg);
    //std::cout << "New BELIEF: Belief: " << msg->name << ", value: " << msg->value << ", priority: " << msg->priority << ", deadline: " << msg->deadline << std::endl;

    rclcpp::spin_some(node);

    return 0;
}
