#pragma once

#include <executor/StateBatch.hpp>
#include <executor/AdapterBase.hpp>

namespace free_gait {

class StateBatchComputer
{
 public:
  StateBatchComputer(AdapterBase& adapter);
  virtual ~StateBatchComputer();

  void computeEndEffectorTargetsAndSurfaceNormals(StateBatch& stateBatch);
  void computeSurfaceNormals(StateBatch& stateBatch);
  void computeEndEffectorTrajectories(StateBatch& stateBatch);
  void computeStances(StateBatch& stateBatch);
  void computeBaseTrajectories(StateBatch& stateBatch);
  void computeStepIds(StateBatch& stateBatch);

 private:
  AdapterBase& adapter_;
};

} /* namespace free_gait */
