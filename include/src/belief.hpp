//Class representing a couple (name, value)
#ifndef BELIEF_HPP
#define BELIEF_HPP

#include "rclcpp/rclcpp.hpp"

class Belief
{
protected:
    std::string name;

public:
    std::string getName();
};

class BeliefBool : public Belief
{
private:
    bool value;

public:
    BeliefBool(std::string beliefName, bool beliefValue);
    ~BeliefBool();

    bool getValue();
};

class BeliefString : public Belief
{
private:
    std::string value;

public:
    BeliefString(std::string beliefName, std::string beliefValue);
    ~BeliefString();

    std::string getValue();
};

class BeliefInt : public Belief
{
private:
    int value;

public:
    BeliefInt(std::string beliefName, int beliefValue);
    ~BeliefInt();

    int getValue();
};

class BeliefFloat : public Belief
{
private:
    float value;

public:
    BeliefFloat(std::string beliefName, float beliefValue);
    ~BeliefFloat();

    float getValue();
};

#endif