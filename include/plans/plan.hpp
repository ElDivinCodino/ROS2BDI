#ifndef PLAN_HPP
#define PLAN_HPP
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "src/goal.hpp"
#include "src/belief.hpp"

class Plan
{

public:
    std::string planName;
    Belief *planGoal;
    std::vector<Belief *> planPreconditions;
    std::vector<Belief *> planContext;
    int planPriority;
    float planDeadline;

    virtual bool verifyGoal(Goal *goal);
    virtual bool verifyPreconditions(std::vector<Belief *> beliefset);
    virtual void activatePlan();
};

class RechargeBatteryCritical : public Plan
{
public:
    RechargeBatteryCritical();
    ~RechargeBatteryCritical();

    bool verifyGoal(Goal *goal);
    bool verifyPreconditions(std::vector<Belief *> beliefset);
    void activatePlan();
};

class RechargeBatterySafe : public Plan
{
public:
    RechargeBatterySafe();
    ~RechargeBatterySafe();

    bool verifyGoal(Goal *goal);
    bool verifyPreconditions(std::vector<Belief *> beliefset);
    void activatePlan();
};

class CleaningSpecific : public Plan
{
public:
    CleaningSpecific();
    ~CleaningSpecific();

    bool verifyGoal(Goal *goal);
    bool verifyPreconditions(std::vector<Belief *> beliefset);
    void activatePlan();
};

class CleaningRoutine : public Plan
{
public:
    CleaningRoutine();
    ~CleaningRoutine();

    bool verifyGoal(Goal *goal);
    bool verifyPreconditions(std::vector<Belief *> beliefset);
    void activatePlan();
};

#endif