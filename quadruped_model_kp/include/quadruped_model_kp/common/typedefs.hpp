#pragma once

#include "kindr/Core"

namespace romo {

/**
 * @brief D means double;
 */
typedef kindr::HomTransformQuatD Pose;//transform with a rotation quaternion
typedef kindr::TwistLinearVelocityLocalAngularVelocityD Twist;//combine the linear and angular to integrate a twist?
typedef kindr::RotationQuaternionD RotationQuaternion;//unit quaternion respresentation of a rotation
typedef kindr::AngleAxisD AngleAxis;//rotation around a axis and a angle
typedef kindr::RotationMatrixD RotationMatrix;//rotation matrix
typedef kindr::EulerAnglesZyxD EulerAnglesZyx;//rotation zyx double
typedef kindr::EulerAnglesZyxDiffD EulerAnglesZyxDiff;//derivatives of ZYX
typedef kindr::RotationVectorD RotationVector;//rotation vector
typedef kindr::EulerAnglesXyzD EulerAnglesXyz;//rotation xyz double
typedef kindr::EulerAnglesXyzDiffD EulerAnglesXyzDiff;//derivatives of xyz
typedef kindr::Position3D Position;//
typedef kindr::LocalAngularVelocity<double> LocalAngularVelocity;//local angular velocity in 3D space
typedef kindr::Velocity3D LinearVelocity;//
typedef kindr::Acceleration3D LinearAcceleration;//
typedef kindr::AngularAcceleration3D AngularAcceleration;//
typedef kindr::Force3D Force;//
typedef kindr::Torque3D Torque;//
typedef kindr::VectorTypeless3D Vector;//vector without type
};
