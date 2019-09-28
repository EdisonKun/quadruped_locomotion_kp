#include "executor/ExecutorState.hpp"

namespace free_gait {

ExecutorState::ExecutorState()
: stepNumber_(0),
  stepPhase_(0.0)
{
}

ExecutorState::ExecutorState(size_t stepNumber, double stepPhase)
: stepNumber_(stepNumber),
  stepPhase_(stepPhase)
{
}

ExecutorState::~ExecutorState()
{
}

void ExecutorState::setState(size_t stepNumber, double stepPhase)
{
   stepNumber_ = stepNumber;
   stepPhase_ = stepPhase;
}

const size_t ExecutorState::stepNumber() const
{
  return stepNumber_;
}

const double ExecutorState::stepPhase() const
{
  return stepPhase_;
}

bool operator>(const ExecutorState& stateA, const ExecutorState& stateB)
{
  return stateA.stepNumber() >= stateB.stepNumber() && stateA.stepPhase() > stateB.stepPhase();
}

bool operator>=(const ExecutorState& stateA, const ExecutorState& stateB)
{
  return stateA.stepNumber() >= stateB.stepNumber() && stateA.stepPhase() >= stateB.stepPhase();
}

bool operator<(const ExecutorState& stateA, const ExecutorState& stateB)
{
  return stateA.stepNumber() <= stateB.stepNumber() && stateA.stepPhase() < stateB.stepPhase();
}

bool operator<=(const ExecutorState& stateA, const ExecutorState& stateB)
{
  return stateA.stepNumber() <= stateB.stepNumber() && stateA.stepPhase() <= stateB.stepPhase();
}

}//namespace
