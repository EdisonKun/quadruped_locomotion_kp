#pragma once

#include "executor/Executor.hpp"
#include "executor/AdapterBase.hpp"
#include "executor/StateBatch.hpp"

#include <thread>//duo xian cheng
#include <memory>
#include <functional>//function object
#include <atomic>//duoxiancheng tongbu caozuo

namespace free_gait {

class BatchExecutor
{
 public:
  BatchExecutor(free_gait::Executor& executor);
  virtual ~BatchExecutor();

  void addProcessingCallback(std::function<void(bool)> callback);
  void setTimeStep(const double timeStep);
  double getTimeStep() const;
  bool process(const std::vector<free_gait::Step>& steps);
  bool isProcessing();
  void cancelProcessing();

  const StateBatch& getStateBatch() const;

 private:
  void processInThread();

  StateBatch stateBatch_;
  free_gait::Executor& executor_;

  std::function<void(bool)> callback_;
  double timeStep_;
  std::atomic<bool> isProcessing_;
  std::atomic<bool> requestForCancelling_;
};

} /* namespace free_gait_rviz_plugin */
