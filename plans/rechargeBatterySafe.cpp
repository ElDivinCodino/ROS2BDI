#include <unistd.h>
#include "plans/plan.hpp"

RechargeBatterySafe::RechargeBatterySafe()
{
    // name of the plan
    std::string name = "rechargeBatterySafe";

    // desired belief
    BeliefInt *goal = new BeliefInt("BatteryCharge", 100);

    // planPreconditions necessary to the plan activation
    BeliefInt *precondition1 = new BeliefInt("BatteryCharge", 50);
    BeliefBool *precondition2 = new BeliefBool("Idle", true);

    // context needed to hold during the entire execution of the plan
    BeliefInt *context1 = new BeliefInt("BatteryCharge", 100); // execute the plan until BatteryCharge doesn't reach 100%

    // priority of the plan
    int priority = 3;

    // time taken by the plan (in seconds) in order to execute
    float deadline = 3200;

    // create two vectors of the superclass, which can contain every possible type of belief
    std::vector<Belief *> planPreconditions, context;

    planPreconditions.push_back(precondition1);
    planPreconditions.push_back(precondition2);

    context.push_back(context1);
    
    planName = name;
    planGoal = goal;
    planPreconditions = planPreconditions;
    planContext = context;
    planPriority = priority;
    planDeadline = deadline;
}

RechargeBatterySafe::~RechargeBatterySafe() = default;

bool RechargeBatterySafe::verifyGoal(Goal *goal)
{
    GoalInt *derivedGoal = static_cast<GoalInt *>(goal);
    BeliefInt *derivedPlanGoal = static_cast<BeliefInt *>(planGoal);

    return derivedGoal->getName() == derivedPlanGoal->getName() && derivedGoal->getValue() == derivedPlanGoal->getValue();
}

bool RechargeBatterySafe::verifyPreconditions(std::vector<Belief *> beliefset)
{
    bool activable = true;

    // check if the next precondition is verified
    bool preconditionVerified = false;

    for(int i = 0; i < beliefset.size(); i++)
    {
        if(beliefset[i]->getName() == planPreconditions[0]->getName())
        {
            // cast the two BeliefBase objects to the appropriate type to execute the GetValue() method
            BeliefInt *derivedPointer1 = static_cast<BeliefInt *>(beliefset[i]);
            BeliefInt *derivedPointer2 = static_cast<BeliefInt *>(planPreconditions[0]);

            if (derivedPointer1->getValue() < derivedPointer2->getValue())
                preconditionVerified = true;
        }
    }

    activable = activable && preconditionVerified;

    // reset the bool
    preconditionVerified = false;

    for (int i = 0; i < beliefset.size(); i++)
    {
        if (beliefset[i]->getName() == planPreconditions[1]->getName())
        {
            // cast the two BeliefBase objects to the appropriate type to execute the GetValue() method
            BeliefBool *derivedPointer1 = static_cast<BeliefBool *>(beliefset[i]);
            BeliefBool *derivedPointer2 = static_cast<BeliefBool *>(planPreconditions[0]);

            if (derivedPointer1->getValue() == derivedPointer2->getValue())
                preconditionVerified = true;
        }
    }

    activable = activable && preconditionVerified;

    return activable;
}

void RechargeBatterySafe::activatePlan()
{
    std::cout << "Going back to the charging station" << std::endl;

    // wait for part of the time needed for the plan to complete (in ms instead of s)
    usleep(planDeadline / 3);

    std::cout << "Recharging operation started" << std::endl;

    // wait for the remaining part of the time needed for the plan to complete (in ms instead of s)
    usleep(planDeadline * 2 / 3);

    std::cout << "Recharging operation completed" << std::endl;
}