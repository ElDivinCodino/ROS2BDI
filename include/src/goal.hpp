//Class representing a 4-tuple (name, value, priority, deadline)
#ifndef GOAL_HPP
#define GOAL_HPP

#include "rclcpp/rclcpp.hpp"

class Goal
{
protected:
    std::string name;
    int priority;
    float deadline;

public:
    std::string getName();
    int getPriority();
    float getDeadline();
};

class GoalBool : public Goal
{
private:
    bool value;

public:
    GoalBool(std::string goalName, bool goalValue, int goalPriority, float goalDeadline);
    ~GoalBool();

    bool getValue();
};

class GoalString : public Goal
{
private:
    std::string value;

public:
    GoalString(std::string goalName, std::string goalValue, int goalPriority, float goalDeadline);
    ~GoalString();

    std::string getValue();
};

class GoalInt : public Goal
{
private:
    int value;

public:
    GoalInt(std::string goalName, int goalValue, int goalPriority, float goalDeadline);
    ~GoalInt();

    int getValue();
};

class GoalFloat : public Goal
{
private:
    float value;

public:
    GoalFloat(std::string goalName, float goalValue, int goalPriority, float goalDeadline);
    ~GoalFloat();

    float getValue();
};

#endif