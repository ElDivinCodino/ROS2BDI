#include "rclcpp/rclcpp.hpp"
#include "bdi_ros2/msg/goal_bool.hpp"
#include "bdi_ros2/msg/goal_string.hpp"
#include "bdi_ros2/msg/goal_int.hpp"
#include "bdi_ros2/msg/goal_float.hpp"
#include "bdi_ros2/msg/belief_bool.hpp"
#include "bdi_ros2/msg/belief_string.hpp"
#include "bdi_ros2/msg/belief_int.hpp"
#include "bdi_ros2/msg/belief_float.hpp"

class GoalManager : public rclcpp::Node
{
    // publishers declaration
    rclcpp::Publisher<bdi_ros2::msg::GoalBool>::SharedPtr boolGoalPub;
    rclcpp::Publisher<bdi_ros2::msg::GoalString>::SharedPtr stringGoalPub;
    rclcpp::Publisher<bdi_ros2::msg::GoalInt>::SharedPtr intGoalPub;
    rclcpp::Publisher<bdi_ros2::msg::GoalFloat>::SharedPtr floatGoalPub;

    // subscribers declaration
    rclcpp::Subscription<bdi_ros2::msg::BeliefBool>::SharedPtr boolBeliefSub;
    rclcpp::Subscription<bdi_ros2::msg::BeliefString>::SharedPtr stringBeliefSub;
    rclcpp::Subscription<bdi_ros2::msg::BeliefInt>::SharedPtr intBeliefSub;
    rclcpp::Subscription<bdi_ros2::msg::BeliefFloat>::SharedPtr floatBeliefSub;
    

  public:
    explicit GoalManager();
};