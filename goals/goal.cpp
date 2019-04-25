#include "src/goal.hpp"

std::string Goal::getName()
{
    return name;
}

int Goal::getPriority()
{
    return priority;
}

float Goal::getDeadline()
{
    return deadline;
}

GoalBool::GoalBool(std::string goalName, bool goalValue, int goalPriority, float goalDeadline)
{
    name = goalName;
    value = goalValue;
    priority = goalPriority;
    deadline = goalDeadline;
}

GoalBool::~GoalBool() = default;

bool GoalBool::getValue()
{
    return value;
}

GoalString::GoalString(std::string goalName, std::string goalValue, int goalPriority, float goalDeadline)
{
    name = goalName;
    value = goalValue;
    priority = goalPriority;
    deadline = goalDeadline;
}

GoalString::~GoalString() = default;

std::string GoalString::getValue()
{
    return value;
}

GoalInt::GoalInt(std::string goalName, int goalValue, int goalPriority, float goalDeadline)
{
    name = goalName;
    value = goalValue;
    priority = goalPriority;
    deadline = goalDeadline;
}

GoalInt::~GoalInt() = default;

int GoalInt::getValue()
{
    return value;
}

GoalFloat::GoalFloat(std::string goalName, float goalValue, int goalPriority, float goalDeadline)
{
    name = goalName;
    value = goalValue;
    priority = goalPriority;
    deadline = goalDeadline;
}

GoalFloat::~GoalFloat() = default;

float GoalFloat::getValue()
{
    return value;
}