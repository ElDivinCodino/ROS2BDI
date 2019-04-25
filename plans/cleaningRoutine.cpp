#include "plans/plan.hpp"

CleaningRoutine::CleaningRoutine()
{
    // name of the plan
    std::string name = "cleaningRoutine";

    // desired belief
    BeliefBool *goal = new BeliefBool("RoomClean", true);

    // preconditions necessary to the plan activation
    BeliefBool *precondition1 = new BeliefBool("RoomClean", false);
    BeliefInt *precondition2 = new BeliefInt("BatteryCharge", 40); // batterycharge should be at least at 40%, otherwise the robot cannot perform the complete cleanup

    // context needed to hold during the entire execution of the plan
    BeliefInt *context1 = new BeliefInt("BatteryCharge", 5); // batterycharge should never drop below 5%

    // priority of the plan
    int priority = 4;

    // time taken by the plan (in seconds) in order to execute
    float deadline = 5500;

    // create two vectors of the superclass, which can contain every possible type of belief
    std::vector<Belief *> preconditions, context;

    preconditions.push_back(precondition1);
    preconditions.push_back(precondition2);

    context.push_back(context1);

    planName = name;
    planGoal = goal;
    planPreconditions = preconditions;
    planContext = context;
    planPriority = priority;
    planDeadline = deadline;
}

CleaningRoutine::~CleaningRoutine() = default;

bool CleaningRoutine::verifyGoal(Goal *goal)
{
    GoalBool *derivedGoal = static_cast<GoalBool *>(goal);
    BeliefBool *derivedPlanGoal = static_cast<BeliefBool *>(planGoal);

    return derivedGoal->getName() == derivedPlanGoal->getName() && derivedGoal->getValue() == derivedPlanGoal->getValue();
}

bool CleaningRoutine::verifyPreconditions(std::vector<Belief *> beliefset)
{
    bool activable = true;

    // check if the next precondition is verified
    bool preconditionVerified = false;

    for (int i = 0; i < beliefset.size(); i++)
    {
        if (beliefset[i]->getName() == planPreconditions[0]->getName())
        {
            // cast the two BeliefBase objects to the appropriate type to execute the getValue() method
            BeliefBool *derivedPointer1 = static_cast<BeliefBool *>(beliefset[i]);
            BeliefBool *derivedPointer2 = static_cast<BeliefBool *>(planPreconditions[0]);

            if (derivedPointer1->getValue() > derivedPointer2->getValue())
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
            BeliefInt *derivedPointer1 = static_cast<BeliefInt *>(beliefset[i]);
            BeliefInt *derivedPointer2 = static_cast<BeliefInt *>(planPreconditions[0]);

            if (derivedPointer1->getValue() == derivedPointer2->getValue())
                preconditionVerified = true;
        }
    }

    activable = activable && preconditionVerified;

    return activable;
}

void CleaningRoutine::activatePlan()
{
    std::cout << "Going to the living room" << std::endl;

    // wait for part of the time needed for the plan to complete (in ms instead of s)
    usleep(planDeadline / 9);

    std::cout << "Cleaning operation started" << std::endl;

    // wait for the remaining part of the time needed for the plan to complete (in ms instead of s)
    usleep(planDeadline * 2 / 9);

    std::cout << "Cleaning operation completed" << std::endl;

    std::cout << "Going to the kitchen" << std::endl;

    // wait for part of the time needed for the plan to complete (in ms instead of s)
    usleep(planDeadline / 9);

    std::cout << "Cleaning operation started" << std::endl;

    // wait for the remaining part of the time needed for the plan to complete (in ms instead of s)
    usleep(planDeadline * 2 / 9);

    std::cout << "Cleaning operation completed" << std::endl;

    std::cout << "Going to the bathroom" << std::endl;

    // wait for part of the time needed for the plan to complete (in ms instead of s)
    usleep(planDeadline / 9);

    std::cout << "Cleaning operation started" << std::endl;

    // wait for the remaining part of the time needed for the plan to complete (in ms instead of s)
    usleep(planDeadline * 2 / 9);

    std::cout << "Cleaning operation completed" << std::endl;
}