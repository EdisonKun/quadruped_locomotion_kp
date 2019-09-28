/*
 * TypeDefs.hpp
 *
 *  Created on: Jun 1, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// quadruped model
#include <quadruped_model_kp/common/typedefs.hpp>
#include <quadruped_model_kp/quadruped_model.hpp>

// STL
#include <unordered_map>
#include <map>
#include <iostream>
#include <vector>
#include <algorithm>

namespace free_gait {

//struct EnumClassHash
//{
//  template<typename T>
//  std::size_t operator()(T t) const
//  {
//    return static_cast<std::size_t>(t);
//  }
//};

// Import enum aliases.
using QD = quadruped_model_kp::QuadrupedModel_kp::QuadrupedDescription_kp;

using LimbEnum = QD::LimbEnum;
using BranchEnum = QD::BranchEnum;
using JointNodeEnum = QD::JointNodeEnum;
//using ContactEnum = QD::ContactEnum;
//using FrameTransformEnum = QD::ConcreteTopology::FrameTransformEnum;

// Import kindr aliases.
using Transform = romo::Pose;
using romo::Pose;
using romo::Twist;
using romo::RotationQuaternion;
using romo::AngleAxis;
using romo::RotationMatrix;
using romo::EulerAnglesZyx;
using romo::RotationVector;
using romo::EulerAnglesXyz;
using romo::EulerAnglesXyzDiff;
using romo::Position;
using Position2 = kindr::Position<double, 2>;
/**
 *template <typename PrimType_, int Dimension_>
 *using Position = Vector<PhysicalType::Position, PrimType_, Dimension_>;
 */
using romo::LinearVelocity;
using romo::LocalAngularVelocity;
using romo::EulerAnglesZyxDiff;
using romo::LinearAcceleration;
using romo::AngularAcceleration;
using romo::Force;
using romo::Torque;
using romo::Vector;

// Import robot-specific kindr quantities.
using quadruped_model_kp::GeneralizedCoordinates;
using quadruped_model_kp::GeneralizedVelocities;
using quadruped_model_kp::GeneralizedAccelerations;
using quadruped_model_kp::JointPositions;
using JointPositionsLeg = quadruped_model_kp::JointPositionLimb;
using quadruped_model_kp::JointVelocities;
using JointVelocitiesLeg = quadruped_model_kp::JointVelocitiesLimb;
using quadruped_model_kp::JointAccelations;
using JointAccelerationsLeg = quadruped_model_kp::JointAccelationsLimb;
using JointEfforts = quadruped_model_kp::JointTorques;
using JointEffortsLeg = quadruped_model_kp::JointTorquesLimb;

enum class ControlLevel
{
  Position,
  Velocity,
  Acceleration,
  Effort
};

const std::vector<LimbEnum> limbEnumCounterClockWiseOrder = { LimbEnum::LF_LEG,
                                                              LimbEnum::LH_LEG,
                                                              LimbEnum::RH_LEG,
                                                              LimbEnum::RF_LEG };

typedef std::unordered_map<ControlLevel, bool, EnumClassHash> ControlSetup;//whether use the controllevel
typedef std::unordered_map<LimbEnum, Position, EnumClassHash> Stance;
typedef std::unordered_map<LimbEnum, Position2, EnumClassHash> PlanarStance;//stands for what?

struct CompareByCounterClockwiseOrder;
void getFootholdsCounterClockwiseOrdered(const Stance& stance, std::vector<Position>& footholds);

} // namespace
