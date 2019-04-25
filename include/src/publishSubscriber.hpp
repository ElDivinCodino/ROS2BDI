#include "rclcpp/rclcpp.hpp"
#include "bdi_ros2/msg/goal_bool.hpp"
#include "bdi_ros2/msg/goal_string.hpp"
#include "bdi_ros2/msg/goal_int.hpp"
#include "bdi_ros2/msg/goal_float.hpp"
#include "bdi_ros2/msg/belief_bool.hpp"
#include "bdi_ros2/msg/belief_string.hpp"
#include "bdi_ros2/msg/belief_int.hpp"
#include "bdi_ros2/msg/belief_float.hpp"

class PublishSubscriber : public rclcpp::Node
{
    // publishers declaration
    rclcpp::Publisher<bdi_ros2::msg::BeliefBool>::SharedPtr boolBeliefPub;
    rclcpp::Publisher<bdi_ros2::msg::BeliefString>::SharedPtr stringBeliefPub;
    rclcpp::Publisher<bdi_ros2::msg::BeliefInt>::SharedPtr intBeliefPub;
    rclcpp::Publisher<bdi_ros2::msg::BeliefFloat>::SharedPtr floatBeliefPub;
    rclcpp::Publisher<bdi_ros2::msg::GoalBool>::SharedPtr boolGoalPub;
    rclcpp::Publisher<bdi_ros2::msg::GoalString>::SharedPtr stringGoalPub;
    rclcpp::Publisher<bdi_ros2::msg::GoalInt>::SharedPtr intGoalPub;
    rclcpp::Publisher<bdi_ros2::msg::GoalFloat>::SharedPtr floatGoalPub;

    // subscribers declaration
    rclcpp::Subscription<bdi_ros2::msg::BeliefBool>::SharedPtr boolBeliefSub;
    rclcpp::Subscription<bdi_ros2::msg::BeliefString>::SharedPtr stringBeliefSub;
    rclcpp::Subscription<bdi_ros2::msg::BeliefInt>::SharedPtr intBeliefSub;
    rclcpp::Subscription<bdi_ros2::msg::BeliefFloat>::SharedPtr floatBeliefSub;
    rclcpp::Subscription<bdi_ros2::msg::GoalBool>::SharedPtr boolGoalSub;
    rclcpp::Subscription<bdi_ros2::msg::GoalString>::SharedPtr stringGoalSub;
    rclcpp::Subscription<bdi_ros2::msg::GoalInt>::SharedPtr intGoalSub;
    rclcpp::Subscription<bdi_ros2::msg::GoalFloat>::SharedPtr floatGoalSub;

  public:
    explicit PublishSubscriber();
};