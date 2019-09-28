/*
 * State.cpp
 *
 *  Created on: Oct 22, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include "executor/State.hpp"
#include "TypePrints.hpp"

#include <stdexcept>
namespace free_gait {

State::State():quadruped_model_kp::QuadrupedState(),robotExecutionStatus_(false)
{

}

State::~State()
{

}

void State::initialize(const std::vector<LimbEnum> &limbs, const std::vector<BranchEnum> &branches)
{
    for(const auto& limb : limbs){
        isSupportLegs_[limb] = false;
        ignoreContact_[limb] = false;
        ignoreForPoseAdaptation_[limb] = false;
    }//confirm the state of each leg, support or not

    for(const auto& branch : branches){
        setEmptyControlSetup(branch);
    }//the control method of each leg

    QuadrupedState::Initialize();//initialize the joint of the quadruped model
}

bool State::getRobotExecutionStatus() const
{
    return robotExecutionStatus_;//bool
}

void State::setRobotExecutionStatus(bool robotExecutionStatus)
{
    robotExecutionStatus_ = robotExecutionStatus;
}

const std::string& State::getStepId() const
{
    return stepId_;
}

void State::setStepId(const std::string &stepId)
{
    stepId_ = stepId;
}

bool State::isSupportLeg(const LimbEnum &limb) const
{
    return isSupportLegs_.at(limb);
}

void State::setSupportLeg(const LimbEnum &limb, bool isSupportLeg)
{
    isSupportLegs_[limb] = isSupportLeg;
}

unsigned int State::getNumberOfSupportLegs() const //get the total number of the support leg
{
    unsigned int nLegs = 0;
    for (const auto& supportLeg: isSupportLegs_) {
        if (supportLeg.second) ++nLegs;
    }
    return nLegs;
}

bool State::isIgnoreContact(const LimbEnum &limb) const
{
    return ignoreContact_.at(limb);
}

void State::setIgnoreContact(const LimbEnum &limb, bool ignoreContact)
{
    ignoreContact_[limb] = ignoreContact;
}

bool State::hasSurfaceNormal(const LimbEnum &limb) const
{
    return (surfaceNormals_.count(limb) > 0u);//finds the number of elements, not only one surfacenormals?
}

const Vector& State::getSurfaceNormal(const LimbEnum &limb) const
{
    return surfaceNormals_.at(limb);
}

void State::setSurfaceNormal(const LimbEnum &limb, const Vector &surfaceNormal)
{
    surfaceNormals_[limb] = surfaceNormal;
}

void State::removeSurfaceNormal(const LimbEnum &limb)
{
    surfaceNormals_.erase(limb);
}

bool State::isIgnoreForPoseAdaptation(const LimbEnum &limb) const
{
    return ignoreForPoseAdaptation_.at(limb);
}

void State::setIgnoreForPoseAdaptation(const LimbEnum &limb, bool ignorePoseAdaptation)
{
    ignoreForPoseAdaptation_[limb] = ignorePoseAdaptation;
}

const JointPositionsLeg State::getJointPositionsForLimb(const LimbEnum &limb) const
{
    int start, n;
    start = QD::getLimbStartIndexInJ(limb);
    n = QD::getNumDofLimb();
    return JointPositionsLeg(quadruped_model_kp::QuadrupedState::getJointPositions().vector().segment(start,n));//from start to n
}

void State::setJointPositionsForLimb(const LimbEnum &limb, const JointPositionsLeg &jointPositions)
{
    quadruped_model_kp::QuadrupedState::getJointPositions().setSegment<3>(QD::getLimbStartIndexInJ(limb),jointPositions);//add elements from specific location;
}

void State::setAllJointPositions(const JointPositions &jointPositions)
{
    setCurrentLimbJoints(jointPositions);
}

const JointVelocitiesLeg State::getJointVelocitiesForLimb(const LimbEnum &limb) const
{
    int start, n;
    start = QD::getLimbStartIndexInJ(limb);
    n = QD::getNumDofLimb();
    return JointVelocitiesLeg(quadruped_model_kp::QuadrupedState::getJointVelocities().vector().segment(start,n));
}

void State::setJointVelocitiesForLimb(const LimbEnum &limb, const JointVelocitiesLeg &jointVelocities)
{
    quadruped_model_kp::QuadrupedState::getJointVelocities().setSegment<3>(QD::getLimbStartIndexInJ(limb), jointVelocities);
}

void State::setAllJointVelocities(const JointVelocities &jointVelocities)
{
    setCurrentLimbJointVelocities(jointVelocities);
}

const JointAccelerationsLeg State::getJointAccelerationsForLimb(const LimbEnum &limb) const
{
    int start, n;
    start = QD::getLimbStartIndexInJ(limb);
    n = QD::getNumDofLimb();
    return JointAccelerationsLeg(jointAccelerations_.vector().segment(start, n));
}

const JointAccelations& State::getAllJointAccelerations() const
{
  return jointAccelerations_;
}

void State::setAllJointAccelerations(const JointAccelations &jointAccelerations)
{
    jointAccelerations_ = jointAccelerations;
}

const JointEffortsLeg State::getJointEffortsForLimb(const LimbEnum &limb) const
{
    int start, n;
    start = QD::getLimbStartIndexInJ(limb);
    n = QD::getNumDofLimb();
    return JointEffortsLeg(getAllJointEfforts().vector().segment(start, n));
}

const JointEfforts& State::getAllJointEfforts() const
{
  return jointEfforts_;
}

void State::setJointEffortsForLimb(const LimbEnum& limb, const JointEffortsLeg& jointEfforts)
{
  jointEfforts_.setSegment<3>(QD::getLimbStartIndexInJ(limb), jointEfforts);
//  jointEfforts_.getSegment<QD::getNumDofLimb()>(QD::getLimbStartIndexInJ(limb)) = jointEfforts;
}

void State::setAllJointEfforts(const JointEfforts& jointEfforts)
{
  jointEfforts_ = jointEfforts;
}

const ControlSetup& State::getControlSetup(const BranchEnum& branch) const
{
  return controlSetups_.at(branch);//control level
}

const ControlSetup& State::getControlSetup(const LimbEnum& limb) const
{
  return controlSetups_.at(QD::mapEnum(limb));//branch's control setup, and the limb has a corresponding relationship with branch
}

bool State::isControlSetupEmpty(const BranchEnum &branch) const
{
    for (const auto& level : controlSetups_.at(branch)){
        if (level.second) return false;
    }
    return true;
}

bool State::isControlSetupEmpty(const LimbEnum& limb) const
{
    return isControlSetupEmpty(QD::mapEnum(limb));
}

void State::setControlSetup(const BranchEnum& branch, const ControlSetup& controlSetup)
{
  controlSetups_[branch] = controlSetup;
}

void State::setControlSetup(const LimbEnum& limb, const ControlSetup& controlSetup)
{
  controlSetups_[QD::mapEnum(limb)] =controlSetup;
}

void State::setEmptyControlSetup(const BranchEnum& branch)
{
  ControlSetup emptyControlSetup;
  emptyControlSetup[ControlLevel::Position] = false;
  emptyControlSetup[ControlLevel::Velocity] = false;
  emptyControlSetup[ControlLevel::Acceleration] = false;
  emptyControlSetup[ControlLevel::Effort] = false;
  controlSetups_[branch] = emptyControlSetup;//This branch corresponding is empty;
}

void State::setEmptyControlSetup(const LimbEnum& limb)
{
  setEmptyControlSetup(QD::mapEnum(limb));
}

void State::getAllJointNames(std::vector<std::string> &jointNames) const
{
    jointNames.clear();
    std::vector<std::string> controller_joint_names{"front_left_1_joint", "front_left_2_joint", "front_left_3_joint" ,
                                                    "front_right_1_joint", "front_right_2_joint", "front_right_3_joint",
                                                    "rear_right_1_joint", "rear_right_2_joint", "rear_right_3_joint",
                                                    "rear_left_1_joint", "rear_left_2_joint", "rear_left_3_joint"};
    jointNames.reserve(12);
    jointNames = controller_joint_names;
}

Position State::getSupportFootPosition(const LimbEnum& limb)
{
  return footHoldInSupport_.at(limb);
}

void State::setSupportFootStance(const Stance& footInSupport)
{
  footHoldInSupport_ =footInSupport;
}

const Pose State::getFootholdsPlanePoseInWorld()
{
    std::vector<Position> footholdsOrdered;
    getFootholdsCounterClockwiseOrdered(footHoldInSupport_, footholdsOrdered);//link error?
}

std::ostream& operator<<(std::ostream& out, const State& state)
{
  out << "Support legs: " << state.isSupportLegs_ << std::endl;
  out << "Ignore contact: " << state.ignoreContact_ << std::endl;
  out << "Ignore for pose adaptation: " << state.ignoreForPoseAdaptation_ << std::endl;
  out << "Control setup:" << std::endl;
  for (const auto& controlSetup : state.controlSetups_) {
    out << controlSetup.first << ": ";
    for (const auto& controlLevel : controlSetup.second) {
      if (controlLevel.second) out << controlLevel.first << ", ";
    }
    out << std::endl;
  }
  out << "Surface normals: " << state.surfaceNormals_ << std::endl;
  if (!state.stepId_.empty()) out << "Step ID: " << state.stepId_ << std::endl;
  return out;
}







}//namespace
