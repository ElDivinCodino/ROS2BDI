#include "src/belief.hpp"

std::string Belief::getName()
{
    return name;
}

BeliefBool::BeliefBool(std::string beliefName, bool beliefValue)
{
    name = beliefName;
    value = beliefValue;
}

BeliefBool::~BeliefBool() = default;

bool BeliefBool::getValue()
{
    return value;
}

BeliefString::BeliefString(std::string beliefName, std::string beliefValue)
{
    name = beliefName;
    value = beliefValue;
}

BeliefString::~BeliefString() = default;

std::string BeliefString::getValue()
{
    return value;
}

BeliefInt::BeliefInt(std::string beliefName, int beliefValue)
{
    name = beliefName;
    value = beliefValue;
}

BeliefInt::~BeliefInt() = default;

int BeliefInt::getValue()
{
    return value;
}

BeliefFloat::BeliefFloat(std::string beliefName, float beliefValue)
{
    name = beliefName;
    value = beliefValue;
}

BeliefFloat::~BeliefFloat() = default;

float BeliefFloat::getValue()
{
    return value;
}