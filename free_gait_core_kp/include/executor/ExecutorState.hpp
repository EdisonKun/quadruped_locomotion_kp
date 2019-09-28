#pragma once

#include <cstddef>

namespace free_gait {

class ExecutorState
{
 public:
  ExecutorState();
  ExecutorState(size_t stepNumber, double stepPhase);
  virtual ~ExecutorState();

  void setState(size_t stepNumber, double stepPhase);
  const size_t stepNumber() const;
  const double stepPhase() const;

  friend bool operator>(const ExecutorState& stateA, const ExecutorState& stateB);
  friend bool operator<=(const ExecutorState& stateA, const ExecutorState& stateB);
  friend bool operator<(const ExecutorState& stateA, const ExecutorState& stateB);
  friend bool operator>=(const ExecutorState& stateA, const ExecutorState& stateB);

 private:
  size_t stepNumber_;
  double stepPhase_;
};

} /* namespace free_gait */
