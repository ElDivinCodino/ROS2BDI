#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "bdi_ros2/msg/belief_int.hpp"

int tankFilling;

auto CreateBeliefIntMsg(std::string goal, int value)
{
    auto msg = std::make_shared<bdi_ros2::msg::BeliefInt>();

    msg->name = goal;
    msg->value = value;

    return msg;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("tankSensor");

    // Set the QoS. ROS 2 will provide QoS profiles based on the following use cases:
    // Default QoS settings for publishers and subscriptions (rmw_qos_profile_default).
    // Services (rmw_qos_profile_services_default).
    // Sensor data (rmw_qos_profile_sensor_data).
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;

    // set the depth to the QoS profile
    custom_qos_profile.depth = 7;

    // create a publisher for a specific type of message
    auto chatter_pub = node->create_publisher<bdi_ros2::msg::BeliefInt>("belief", custom_qos_profile);

    // check every second the status
    rclcpp::WallRate loop_rate(1);

    // initially the tank is supposed to be empty
    tankFilling = 0;

    int oldTankFillingValue = tankFilling;

    while (rclcpp::ok())
    {
        if (oldTankFillingValue != tankFilling)
        {
            // publish the current filling level of the tank
            auto msg = CreateBeliefIntMsg("DirtTankFilling", tankFilling);
            chatter_pub->publish(msg);
            std::cout << "New BELIEF: Belief: " << msg->name << ", value: " << msg->value << std::endl;

            oldTankFillingValue = tankFilling;
        }

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    return 0;
}
