#pragma once

//KDL: Kinematic and dynamic library
#include "kdl_parser/kdl_parser.hpp"//construct a KDL tree from a urdf in various forms.
#include "kdl/chainfksolver.hpp"//forward calculation
#include "kdl/chain.hpp"//form a chain
#include "kdl/chainfksolverpos_recursive.hpp"//in the recursive way

#include "kdl/chainjnttojacsolver.hpp"//calculate the jacobian
#include "kdl/frames.hpp"//some basic calculation and definition of frames
#include "kdl/frames_io.hpp"//overloading the frames input and output
#include "kdl/tree.hpp"//get a kinematic tree
#include "kdl/chaindynparam.hpp"//dynamic calculation

#include "kindr/Core"
#include "quadruped_model_kp/quadruped_model.hpp"
#include "quadruped_model_kp/common/typedefs.hpp"

#include "iostream"
#include "string.h"
#include "ros/ros.h"
#include "memory.h"
#include "unordered_map"
#include "utility"

using namespace romo;
namespace quadruped_model_kp {

using QD = quadruped_model_kp::QuadrupedModel_kp::QuadrupedDescription_kp;

using LimbEnum = QD::LimbEnum;
using BranchEnum = QD::BranchEnum;
using JointNodeEnum = QD::JointNodeEnum;
typedef std::unordered_map<LimbEnum, Pose, EnumClassHash> HipPoseInBase;//limb number and this lim pose

class QuadrupedKinematics_kp
{
public:
  QuadrupedKinematics_kp();
  ~QuadrupedKinematics_kp();
  QuadrupedKinematics_kp(const QuadrupedKinematics_kp& other);
  bool LoadRobotDescriptionFromFile(const std::string filename);
  bool ForwardKinematicsSolve(const JointPositionLimb& joint_position, const LimbEnum& limb, Pose& Cartisian_pose);
  bool InverseKinematicsSolve(const Position& foot_position, const LimbEnum& limb,
                              const JointPositionLimb& joint_position_last,
                              JointPositionLimb& joint_positions,
                              const std::string LimbType = "IN_LEFT");
  bool AnalysticJacobian(const JointPositionLimb& joint_position, const LimbEnum& limb, Eigen::MatrixXd& jacobian);
  bool AnalysticJacobianForLink(const JointPositionLimb& joint_positions, const LimbEnum& limb, const int& link_index, Eigen::MatrixXd& jacobian);
  double MapToPI(double q);

  bool setHipPoseInBase(const KDL::Chain& kdl_chain, const LimbEnum& limb);
  Position getPositionFootToHipInHipFrame(const LimbEnum& limb, const Position& foot_position_in_base) const;
  Position getPositionBaseToHipInBaseFrame(const LimbEnum& limb) const;

  JointTorquesLimb getGravityCompensationForLimb(const LimbEnum& limb, const JointPositionLimb& joint_positions, const Force& gravety_in_baseframe);

  KDL::Chain LF_Chain, RF_Chain,RH_Chain,LH_Chain;
  HipPoseInBase hip_pose_in_base_;

private:
  KDL::Tree tree_;
  KDL::JntArray joint_position_last_;

};

}//namespace
