/*
 * State.hpp
 *
 *  Created on: Oct 22, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "TypeDefs.hpp"
#include "quadruped_model_kp/quadruped_state.h"
#include "quadruped_model_kp/quadruped_model.hpp"

#include <vector>
#include <iostream>
#include <unordered_map>

#include "grid_map_core/Polygon.hpp" //polygon, judge a point whether in the polygon
#include "grid_map_core/grid_map_core.hpp"

namespace free_gait {

class State: public quadruped_model_kp::QuadrupedState
{
public:
    using QD = quadruped_model_kp::QuadrupedModel_kp::QuadrupedDescription_kp;

    State();
    virtual ~State();

    virtual void initialize(const std::vector<LimbEnum>& limbs, const std::vector<BranchEnum>& branches);
    bool getRobotExecutionStatus() const;
    void setRobotExecutionStatus(bool robotExecutionStatus);

    const std::string& getStepId() const;
    void setStepId(const std::string& stepId);

    bool isSupportLeg(const LimbEnum& limb) const;
    void setSupportLeg(const LimbEnum& limb, bool isSupportLeg);
    unsigned int getNumberOfSupportLegs() const;

    bool isIgnoreContact(const LimbEnum& limb) const;
    void setIgnoreContact(const LimbEnum& limb, bool ignoreContact);

    bool hasSurfaceNormal(const LimbEnum& limb) const;
    const Vector& getSurfaceNormal(const LimbEnum& limb) const;
    void setSurfaceNormal(const LimbEnum& limb, const Vector& surfaceNormal);
    void removeSurfaceNormal(const LimbEnum& limb);

    bool isIgnoreForPoseAdaptation(const LimbEnum& limb) const;
    void setIgnoreForPoseAdaptation(const LimbEnum& limb, bool ignorePoseAdaptation);

    const JointPositionsLeg getJointPositionsForLimb(const LimbEnum& limb) const;
    void setJointPositionsForLimb(const LimbEnum& limb, const JointPositionsLeg& jointPositions);
    void setAllJointPositions(const JointPositions& jointPositions);

    const JointVelocitiesLeg getJointVelocitiesForLimb(const LimbEnum& limb) const;
    void setJointVelocitiesForLimb(const LimbEnum& limb, const JointVelocitiesLeg& jointVelocities);
    void setAllJointVelocities(const JointVelocities& jointVelocities);

    const JointAccelerationsLeg getJointAccelerationsForLimb(const LimbEnum& limb) const;
    const JointAccelations& getAllJointAccelerations() const;
    void setJointAccelerationsForLimb(const LimbEnum& limb, const JointAccelerationsLeg& jointAccelerations);
    void setAllJointAccelerations(const JointAccelations& jointAccelerations);

    const JointEffortsLeg getJointEffortsForLimb(const LimbEnum& limb) const;
    const JointEfforts& getAllJointEfforts() const;
    void setJointEffortsForLimb(const LimbEnum& limb, const JointEffortsLeg& jointEfforts);
    void setAllJointEfforts(const JointEfforts& jointEfforts);

    const ControlSetup& getControlSetup(const BranchEnum& branch) const;
    const ControlSetup& getControlSetup(const LimbEnum& limb) const;
    bool isControlSetupEmpty(const BranchEnum& branch) const;
    bool isControlSetupEmpty(const LimbEnum& limb) const;
    void setControlSetup(const BranchEnum& branch, const ControlSetup& controlSetup);
    void setControlSetup(const LimbEnum& limb, const ControlSetup& controlSetup);
    void setEmptyControlSetup(const BranchEnum& branch);
    void setEmptyControlSetup(const LimbEnum& limb);

    void getAllJointNames(std::vector<std::string>& jointNames) const;
    Position getSupportFootPosition(const LimbEnum& limb);
    void setSupportFootStance(const Stance& footInSupport);
    const Pose getFootholdsPlanePoseInWorld();
    friend std::ostream& operator << (std::ostream& out, const State& state);

private:
    LocalAngularVelocity angularVelocityBaseInWorldFrame_;//dimension 3
    JointEfforts jointEfforts_;//dimension 12
    JointAccelations jointAccelerations_;
    LinearAcceleration linearAccelerationBaseInWorldFrame_;
    AngularAcceleration angularAccelerationBaseInBaseFrame_;

    // Free gait specific.
    std::unordered_map<BranchEnum, ControlSetup, EnumClassHash> controlSetups_;
    Force netForceOnBaseInBaseFrame_;
    Torque netTorqueOnBaseInBaseFrame_;
    std::unordered_map<LimbEnum, bool, EnumClassHash> isSupportLegs_;
    std::unordered_map<LimbEnum, bool, EnumClassHash> ignoreContact_;
    std::unordered_map<LimbEnum, bool, EnumClassHash> ignoreForPoseAdaptation_;
    std::unordered_map<LimbEnum, Vector, EnumClassHash> surfaceNormals_;
    bool robotExecutionStatus_;
    std::string stepId_; // empty if undefined.

    Stance footHoldInSupport_;

    Pose footholds_plane_pose_;
};

}//namespace
