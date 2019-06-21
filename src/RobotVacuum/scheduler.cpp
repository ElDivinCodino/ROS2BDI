#include <iostream>
#include <memory>
#include <array>
#include "src/publishSubscriber.hpp"
#include "plans/plan.hpp"
#include "src/belief.hpp"
#include "src/goal.hpp"

std::array<Plan *, 4> planSet = {new CleaningSpecific, new CleaningRoutine, new RechargeBatteryCritical, new RechargeBatterySafe};


void RescheduleIntention(Goal *goal)
{
    std::vector<Plan *> possiblePlans;

    // search for all the possible plans
    for (int i = 0; i < planSet.size(); i++)
    {
        // check if this plan reaches the desired goal
        if (planSet[i]->verifyGoal(goal))
            possiblePlans.push_back(planSet[i]);
    }
    std::cout << "Number of possible plans: " << possiblePlans.size() << std::endl;

    Plan *chosenPlan;

    // among the possible plans, choose the most suitable one (TODO: by now, it just picks the one with greatest priority <= goal's priority)
    int currentPlanPriority = -1;
    
    for (int i = 0; i < possiblePlans.size(); i++)
    {
        if (possiblePlans[i]->planPriority > currentPlanPriority && possiblePlans[i]->planPriority <= goal->getPriority())
        {
            currentPlanPriority = possiblePlans[i]->planPriority;
            chosenPlan = possiblePlans[i];
        }
    }
    std::cout << "Chosen plan: " << chosenPlan->planName << std::endl;
    chosenPlan->activatePlan();
}

PublishSubscriber::PublishSubscriber() : rclcpp::Node("scheduler")
{
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;

    custom_qos_profile.depth = 7;

    auto boolBeliefCallback = [this](const bdi_ros2::msg::BeliefBool::SharedPtr msg) -> void {
        //std::cout << "Received a Belief message! Belief: " << msg->name << "value: " << msg->value << std::endl;
    };

    auto stringBeliefCallback = [this](const bdi_ros2::msg::BeliefString::SharedPtr msg) -> void {
        //std::cout << "Received a Belief message! Belief: " << msg->name << "value: " << msg->value << std::endl;
    };

    auto intBeliefCallback = [this](const bdi_ros2::msg::BeliefInt::SharedPtr msg) -> void {
        //std::cout << "Received a Belief message! Belief: " << msg->name << "value: " << msg->value << std::endl;
    };

    auto floatBeliefCallback = [this](const bdi_ros2::msg::BeliefFloat::SharedPtr msg) -> void {
        //std::cout << "Received a Belief message! Belief: " << msg->name << "value: " << msg->value << std::endl;
    };

    auto boolGoalCallback = [this](const bdi_ros2::msg::GoalBool::SharedPtr msg) -> void {
        std::cout << "Received a Goal message! Goal: " << msg->name << "value: " << msg->value << std::endl;

        GoalBool *goal = new GoalBool(msg->name, msg->value, msg->priority, msg->deadline);
        RescheduleIntention(goal);
    };

    auto stringGoalCallback = [this](const bdi_ros2::msg::GoalString::SharedPtr msg) -> void {
        std::cout << "Received a Goal message! Goal: " << msg->name << "value: " << msg->value << std::endl;

        GoalString *goal = new GoalString(msg->name, msg->value, msg->priority, msg->deadline);
        RescheduleIntention(goal);
    };

    auto intGoalCallback = [this](const bdi_ros2::msg::GoalInt::SharedPtr msg) -> void {
        std::cout << "Received a Goal message! Goal: " << msg->name << "value: " << msg->value << std::endl;

        GoalInt *goal = new GoalInt(msg->name, msg->value, msg->priority, msg->deadline);
        RescheduleIntention(goal);
    };

    auto floatGoalCallback = [this](const bdi_ros2::msg::GoalFloat::SharedPtr msg) -> void {
        std::cout << "Received a Goal message! Goal: " << msg->name << "value: " << msg->value << std::endl;

        GoalFloat *goal = new GoalFloat(msg->name, msg->value, msg->priority, msg->deadline);
        RescheduleIntention(goal);
    };

    // create a subscriber for every type of Belief and Goal message
    boolBeliefSub = create_subscription<bdi_ros2::msg::BeliefBool>("belief", boolBeliefCallback, custom_qos_profile);
    stringBeliefSub = create_subscription<bdi_ros2::msg::BeliefString>("belief", stringBeliefCallback, custom_qos_profile);
    intBeliefSub = create_subscription<bdi_ros2::msg::BeliefInt>("belief", intBeliefCallback, custom_qos_profile);
    floatBeliefSub = create_subscription<bdi_ros2::msg::BeliefFloat>("belief", floatBeliefCallback, custom_qos_profile);
    boolGoalSub = create_subscription<bdi_ros2::msg::GoalBool>("goal", boolGoalCallback, custom_qos_profile);
    stringGoalSub = create_subscription<bdi_ros2::msg::GoalString>("goal", stringGoalCallback, custom_qos_profile);
    intGoalSub = create_subscription<bdi_ros2::msg::GoalInt>("goal", intGoalCallback, custom_qos_profile);
    floatGoalSub = create_subscription<bdi_ros2::msg::GoalFloat>("goal", floatGoalCallback, custom_qos_profile);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PublishSubscriber>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
