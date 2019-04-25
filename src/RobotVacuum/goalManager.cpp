#include <iostream>
#include <memory>
#include "src/publishSubscriber.hpp"

PublishSubscriber::PublishSubscriber() : rclcpp::Node("goal_manager")
{
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;

    custom_qos_profile.depth = 7;

    // create a publisher for every type of goal message
    boolGoalPub = create_publisher<bdi_ros2::msg::GoalBool>("goal", custom_qos_profile);
    stringGoalPub = create_publisher<bdi_ros2::msg::GoalString>("goal", custom_qos_profile);
    intGoalPub = create_publisher<bdi_ros2::msg::GoalInt>("goal", custom_qos_profile);
    floatGoalPub = create_publisher<bdi_ros2::msg::GoalFloat>("goal", custom_qos_profile);

    auto boolBeliefCallback = [this](const bdi_ros2::msg::BeliefBool::SharedPtr msg) -> void {
        std::cout << "Received a Belief message! Belief: " << msg->name << "value: " << msg->value << std::endl;
    };

    auto stringBeliefCallback = [this](const bdi_ros2::msg::BeliefString::SharedPtr msg) -> void {
        std::cout << "Received a Belief message! Belief: " << msg->name << "value: " << msg->value << std::endl;
    };

    auto intBeliefCallback = [this](const bdi_ros2::msg::BeliefInt::SharedPtr msg) -> void {
        std::cout << "Received a Belief message! Belief: " << msg->name << "value: " << msg->value << std::endl;

        if (msg->name == "BatteryCharge" && msg->value >= 30 && msg->value < 50)
        {
            // create a message
            auto newMsg = std::make_shared<bdi_ros2::msg::GoalInt>();

            newMsg->name = msg->name;
            newMsg->value = 100;
            newMsg->priority = 3;
            newMsg->deadline = 100 - msg->value;

            // publish the message
            intGoalPub->publish(newMsg);
            std::cout << "Publishing GOAL message: Goal: " << newMsg->name << ", value: " << newMsg->value << ", priority: " << newMsg->priority << ", deadline: " << newMsg->deadline << std::endl;
        }
        else if (msg->name == "BatteryCharge" && msg->value < 30)
        {
            // create a message
            auto newMsg = std::make_shared<bdi_ros2::msg::GoalInt>();

            newMsg->name = msg->name;
            newMsg->value = 100;
            newMsg->priority = 1;
            newMsg->deadline = 100 - msg->value;

            // publish the message
            intGoalPub->publish(newMsg);
            std::cout << "Publishing GOAL message: Goal: " << newMsg->name << ", value: " << newMsg->value << ", priority: " << newMsg->priority << ", deadline: " << newMsg->deadline << std::endl;
        }
    };

    auto floatBeliefCallback = [this](const bdi_ros2::msg::BeliefFloat::SharedPtr msg) -> void {
        std::cout << "Received a Belief message! Belief: " << msg->name << "value: " << msg->value << std::endl;
    };

    // create a subscriber for every type of belief message
    boolBeliefSub = create_subscription<bdi_ros2::msg::BeliefBool>("belief", boolBeliefCallback, custom_qos_profile);
    stringBeliefSub = create_subscription<bdi_ros2::msg::BeliefString>("belief", stringBeliefCallback, custom_qos_profile);
    intBeliefSub = create_subscription<bdi_ros2::msg::BeliefInt>("belief", intBeliefCallback, custom_qos_profile);
    floatBeliefSub = create_subscription<bdi_ros2::msg::BeliefFloat>("belief", floatBeliefCallback, custom_qos_profile);
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PublishSubscriber>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
