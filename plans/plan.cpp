#include "plans/plan.hpp"

bool Plan::verifyGoal(Goal *goal) 
{
    std::cout << "Error in Plan::verifyGoal: should never be here!" << std::endl;
    return false;
}
bool Plan::verifyPreconditions(std::vector<Belief *> beliefset)
{
    std::cout << "Error in Plan::verifyPreconditions: should never be here!" << std::endl;
    return false;
}
void Plan::activatePlan()
{
    std::cout << "Error in Plan::activatePlan: should never be here!" << std::endl;
}