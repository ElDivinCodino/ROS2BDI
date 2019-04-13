#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "bdi_ros2/msg/belief_int.hpp"

int batteryCharge;

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

    auto node = rclcpp::Node::make_shared("batterySensor");

    // Set the QoS. ROS 2 will provide QoS profiles based on the following use cases:
    // Default QoS settings for publishers and subscriptions (rmw_qos_profile_default).
    // Services (rmw_qos_profile_services_default).
    // Sensor data (rmw_qos_profile_sensor_data).
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;

    // set the depth to the QoS profile
    custom_qos_profile.depth = 7;

    // create a publisher for a specific type of message
    auto chatter_pub = node->create_publisher<bdi_ros2::msg::BeliefInt>("belief", custom_qos_profile);

    rclcpp::WallRate loop_rate(1);

    // initially the battery is supposed to be fully charged
    batteryCharge = 100;

    while (rclcpp::ok())
    {
        // publish the current charge of the battery
        auto msg = CreateBeliefIntMsg("BatteryCharge", batteryCharge);
        chatter_pub->publish(msg);
        std::cout << "New BELIEF: Belief: " << msg->name << ", value: " << msg->value << std::endl;

        // this, together with the above rclcpp::WallRate loop_rate(1), simulates battery dropping 1% per second
        batteryCharge -= 1;

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    return 0;
}
