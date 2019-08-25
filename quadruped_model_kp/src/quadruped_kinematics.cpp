#include "quadruped_model_kp/quadruped_kinematics.h"
#include "ros/package.h"

namespace quadruped_model_kp {
using namespace std;

QuadrupedKinematics_kp::QuadrupedKinematics_kp()
{
  string urdf_dir = ros::package::getPath("quadruped_model_kp")+"/urdf/quadruped_model.urdf";
  LoadRobotDescriptionFromFile(urdf_dir);
  std::cout <<urdf_dir << std::endl;
}

QuadrupedKinematics_kp::~QuadrupedKinematics_kp()
{}

QuadrupedKinematics_kp::QuadrupedKinematics_kp(const QuadrupedKinematics_kp& other)
  : tree_(other.tree_),
    hip_pose_in_base_(other.hip_pose_in_base_),
    LF_Chain(other.LF_Chain),
    RF_Chain(other.RF_Chain),
    RH_Chain(other.RH_Chain),
    LH_Chain(other.LH_Chain)
{}

bool QuadrupedKinematics_kp::LoadRobotDescriptionFromFile(const string filename)
{
 if(!kdl_parser::treeFromFile(filename, tree_))
  {
     ROS_ERROR("Failed to load robot description to KDL tree");
     return false;
  }

 tree_.getChain("base_link","lf_foot_Link",LF_Chain);//root chain to tip chain
 tree_.getChain("base_link","rf_foot_Link",RF_Chain);
 tree_.getChain("base_link","rh_foot_Link",RH_Chain);
 tree_.getChain("base_link","lh_foot_Link",LH_Chain);

 setHipPoseInBase(LF_Chain,LimbEnum::LF_LEG);
 setHipPoseInBase(RF_Chain,LimbEnum::RF_LEG);
 setHipPoseInBase(RH_Chain,LimbEnum::RH_LEG);
 setHipPoseInBase(LH_Chain,LimbEnum::LH_LEG);

 return true;
}

bool QuadrupedKinematics_kp::setHipPoseInBase(const KDL::Chain &kdl_chain, const LimbEnum &limb)
{
  KDL::Frame cartisian_frame;
  cartisian_frame = kdl_chain.getSegment(0).getFrameToTip();
  Position translation(cartisian_frame(0,3), cartisian_frame(1,3), cartisian_frame(2,3));
  RotationMatrix rotation_matrix(cartisian_frame(0,0), cartisian_frame(0,1), cartisian_frame(0,2),
                                 cartisian_frame(1,0), cartisian_frame(1,1), cartisian_frame(1,2),
                                 cartisian_frame(2,0), cartisian_frame(2,1),cartisian_frame(2,2));

  hip_pose_in_base_[limb] = Pose(translation, RotationQuaternion(rotation_matrix));
  return true;
}

Position QuadrupedKinematics_kp::getPositionFootToHipInHipFrame(const LimbEnum &limb, const Position &foot_position_in_base) const
{
  return hip_pose_in_base_.at(limb).inverseTransform(foot_position_in_base);
}

bool QuadrupedKinematics_kp::ForwardKinematicsSolve(const JointPositionLimb &joint_position, const LimbEnum &limb, Pose &Cartisian_pose)
{
  int number_of_joints = joint_position.vector().size();
  KDL::JntArray joints = KDL::JntArray(number_of_joints);
  KDL::Frame Cartisian_frame;

  for(int i = 0; i < number_of_joints; i++){
    joints(i) = joint_position(i);
  }

  switch (limb) {
  case LimbEnum::LF_LEG:
  {
    KDL::ChainFkSolverPos_recursive lf_fk_solver(LF_Chain);
    /**
      *Implementation of a recursive forward position kinematics
     * algorithm to calculate the position transformation from joint
     * space to Cartesian space of a general kinematic chain (KDL::Chain).
     * @param q_in input joint coordinates
     * @param p_out reference to output cartesian pose
     * The return calue is int.
      */
    if (lf_fk_solver.JntToCart(joints,Cartisian_frame) < 0)
    {
      cout << "failed to solve forward kinematics problem" << endl;
      return false;
    }
    break;
  }

  case LimbEnum::RF_LEG:
  {
    KDL::ChainFkSolverPos_recursive rf_fk_solver(RF_Chain);
    if(rf_fk_solver.JntToCart(joints, Cartisian_frame)<0)
    {
      cout<<"Failed to solve Forward kinematics problem"<<endl;
      return false;
    }
    break;
    }
  case LimbEnum::LH_LEG:
  {
    KDL::ChainFkSolverPos_recursive lh_fk_solver(LH_Chain);
    if(lh_fk_solver.JntToCart(joints, Cartisian_frame)<0)
    {
      cout<<"Failed to solve Forward kinematics problem"<<endl;
      return false;
    }
    break;
  }
  case LimbEnum::RH_LEG:
   {
    KDL::ChainFkSolverPos_recursive rh_fk_solver(RH_Chain);
    if(rh_fk_solver.JntToCart(joints, Cartisian_frame)<0)
    {
      cout<<"Failed to solve Forward kinematics problem"<<endl;
      return false;
    }
    break;
   }
  }

  Eigen::Vector3d translation(Cartisian_frame(0,3), Cartisian_frame(1,3), Cartisian_frame(2,3));
  RotationMatrix rotation_matrix(Cartisian_frame(0,0), Cartisian_frame(0,1), Cartisian_frame(0,2),
                                 Cartisian_frame(1,0), Cartisian_frame(1,1), Cartisian_frame(1,2),
                                 Cartisian_frame(2,0), Cartisian_frame(2,1), Cartisian_frame(2,2));

  Cartisian_pose = Pose(Position(translation), RotationQuaternion(rotation_matrix));
  return true;
}

bool QuadrupedKinematics_kp::AnalysticJacobian(const JointPositionLimb &joint_position, const LimbEnum &limb, Eigen::MatrixXd &jacobian)
{
  int number_of_joints = joint_position.vector().size();
  KDL::JntArray joints = KDL::JntArray(number_of_joints);
  KDL::Jacobian J;
  J.resize(number_of_joints);
  for(int i = 0; i < number_of_joints; i++)
  {
    joints(i) = joint_position(i);
  }

  int error_code = 0;
  switch (limb)
  {
  case LimbEnum::LF_LEG:
  {
    KDL::ChainJntToJacSolver jacobian_solver(LF_Chain);
    error_code = jacobian_solver.JntToJac(joints,J);//output jacobian, return 0
    if(error_code != 0)
    {
      cout << "Failed to solve Jacobian problem" << "error code : " << error_code << endl;
      return false;
    }
    break;
  }
  case LimbEnum::RF_LEG:
  {
    KDL::ChainJntToJacSolver jacobian_solver(RF_Chain);
    error_code = jacobian_solver.JntToJac(joints,J);
    if (error_code != 0)
    {
      cout << "failed to solve jacobian problem" << endl;
      return false;
    }
    break;
  }
  case LimbEnum::RH_LEG:
  {
    KDL::ChainJntToJacSolver jacobian_solver(RH_Chain);
    error_code = jacobian_solver.JntToJac(joints,J);
    if (error_code != 0)
    {
      cout << "failed to solve jacobian problem" << endl;
      return false;
    }
    break;
  }
  case LimbEnum::LH_LEG:
  {
    KDL::ChainJntToJacSolver jacobian_solver(LH_Chain);
    error_code = jacobian_solver.JntToJac(joints,J);
    if (error_code != 0)
    {
      cout << "failed to solve jacobian problem" << endl;
      return false;
    }
    break;
  }
  }
  jacobian = J.data;
  return true;
}

bool QuadrupedKinematics_kp::AnalysticJacobianForLink(const JointPositionLimb &joint_positions, const LimbEnum &limb, const int &link_index, Eigen::MatrixXd &jacobian)
{
  int number_of_joints = link_index;
  KDL::JntArray joints = KDL::JntArray(number_of_joints);
  KDL::Jacobian J;
  J.resize(number_of_joints);
  for (int i = 0; i < number_of_joints; ++i) {
    joints(i) = joint_positions(i);
  }
  int error_code = 0;
  switch (limb)
  {
  case LimbEnum::LF_LEG:
  {
    KDL::Chain LF_Chain_Link;
    LF_Chain_Link.addSegment(LF_Chain.getSegment(0));
    cout << LF_Chain.getSegment(0).getName() << endl;
    for(int i = 1; i < link_index; i++)
    {
      KDL::Vector com = LF_Chain.getSegment(i).getInertia().getCOG();
      LF_Chain_Link.addSegment(KDL::Segment(LF_Chain.getSegment(i).getName(),
                                            LF_Chain.getSegment(i).getJoint(),
                                            KDL::Frame(com)));
    }
    KDL::ChainJntToJacSolver jacobian_solver(LF_Chain_Link);
    error_code = jacobian_solver.JntToJac(joints,J);
    if(error_code != 0)
    {
      cout << "failed to solve Jacobian problem " << "error_code is " << error_code << endl;
      return false;
    }
  }
  case LimbEnum::RF_LEG:
    {
      KDL::ChainJntToJacSolver jacobian_solver(RF_Chain);
      if(jacobian_solver.JntToJac(joints, J) != 0)
      {
        cout<<"Failed to solve Jacobian problem"<<endl;
        return false;
      }
      break;
    }
  case LimbEnum::LH_LEG:
  {
    KDL::ChainJntToJacSolver jacobian_solver(LH_Chain);
    if(jacobian_solver.JntToJac(joints, J) != 0)
    {
      cout<<"Failed to solve Jacobian problem"<<endl;
      return false;
    }
    break;
  }
  case LimbEnum::RH_LEG:
   {
    KDL::ChainJntToJacSolver jacobian_solver(RH_Chain);
    if(jacobian_solver.JntToJac(joints, J) != 0)
    {
      cout<<"Failed to solve Jacobian problem"<<endl;
      return false;
    }
    break;
   }
  }

  jacobian = J.data;
  return true;
}

bool QuadrupedKinematics_kp::InverseKinematicsSolve(const Position &foot_position, const LimbEnum &limb, const JointPositionLimb &joint_position_last, JointPositionLimb &joint_positions, const string LimbType)
{
  double d,l1,l2,px,py,pz,alpha,beta1,beta2;
  d = 0.1;
  l1 = 0.25;
  l2 = 0.25;
  Position foot_position_in_hip = getPositionFootToHipInHipFrame(limb, foot_position);
  px = foot_position_in_hip(0);
  py = foot_position_in_hip(1);
  pz = foot_position_in_hip(2);

  double cos_theta3 = (l2 * l2 + l1 * l1 -((px * px + py * py + pz * pz) - d * d)) /2/l1/l2;
  if(cos_theta3 < -1)
    cos_theta3 = -1;
  if(cos_theta3 > 1)
    cos_theta3 = 1;
  Eigen::VectorXd theta3(4);
  Eigen::MatrixXd results(3,4);

  theta3(0) = M_PI - acos(cos_theta3);
  theta3(1) = M_PI - acos(cos_theta3);
  theta3(2) = -M_PI + acos(cos_theta3);
  theta3(3) = -M_PI + acos(cos_theta3);

  alpha = atan2(py,px);
  beta1 = atan2(d,sqrt(fabs(px*px + py*py - d*d)));
  beta2 = atan2(-d,-sqrt(fabs(px*px + py*py - d*d)));

  int i = 0;
  while (i<4) {
    double a,b,q1,q2,q3;
    q3=MapToPI(theta3(i));
    // Left arm configure
    q1=MapToPI(alpha - beta1);
    a = atan2(pz,-sqrt(fabs(px*px + py*py - d*d)));
    b = atan2(l2*sin(q3), l1 + l2*cos(q3));
    if(a>0)
    {
      q2 = MapToPI(a - b - M_PI);
      results.row(i) << q1,q2,q3;
    }
    if(a<0)
    {
      q2 = MapToPI(a - b + M_PI);
      results.row(i) << q1,q2,q3;
    }
    //right arm config
    i = i +1;
    q1 = MapToPI(alpha + beta2);
    a = atan2(pz,sqrt(fabs(px*px + py*py - d*d)));
    q2 = MapToPI(a - b + M_PI);
    results.row(i) << q1,q2,q3;

    i=i+1;
  }

  double min_difference = 100;
  int min_index;

  if(LimbType == "IN_LEFT")
    min_index = 2;
  if(LimbType == "IN_RIGHT")
    min_index = 1;
  if(LimbType == "OUT_LEFT")
    min_index = 0;
  if(LimbType == "OUT_RIGHT")
    min_index = 3;

  joint_positions << results(min_index,0),results(min_index,1),results(min_index,2);

  if(!isnan(joint_positions(0))&&!isnan(joint_positions(1))&&!isnan(joint_positions(2))){//NaN
      return true;
    }else{
      ROS_WARN("Failed to Sovle Inverse Kinematics!");
      return false;
    }
}

JointTorquesLimb QuadrupedKinematics_kp::getGravityCompensationForLimb(const LimbEnum &limb, const JointPositionLimb &joint_positions, const Force &gravety_in_baseframe)
{
  KDL::Vector gravity_vector = KDL::Vector(gravety_in_baseframe(0),
                                           gravety_in_baseframe(1),
                                           gravety_in_baseframe(2));
  int number_of_joints = 3;
  KDL::JntArray joints = KDL::JntArray(number_of_joints);
  KDL::JntArray gravity_matrix = KDL::JntArray(number_of_joints);
  for (int i = 0; i < number_of_joints; ++i) {
    joints(i) = joint_positions(i);
  }
  int error_code = 0;
  switch (limb) {
    case LimbEnum::LF_LEG:
      {
        KDL::ChainDynParam dynamic_param = KDL::ChainDynParam(LF_Chain, gravity_vector);
        error_code = dynamic_param.JntToGravity(joints, gravity_matrix);
        if(error_code != 0)
        {
          cout<<"Failed to solve Jacobian problem"<<" error code :"<<error_code<<endl;
        }
        break;
      }
    case LimbEnum::RF_LEG:
      {
        KDL::ChainDynParam dynamic_param = KDL::ChainDynParam(RF_Chain, gravity_vector);
        error_code = dynamic_param.JntToGravity(joints, gravity_matrix);
        if(error_code != 0)
        {
          cout<<"Failed to solve Jacobian problem"<<endl;
        }
        break;
      }
    case LimbEnum::LH_LEG:
    {
        KDL::ChainDynParam dynamic_param = KDL::ChainDynParam(LH_Chain, gravity_vector);
        error_code = dynamic_param.JntToGravity(joints, gravity_matrix);
        if(error_code != 0)
      {
        cout<<"Failed to solve Jacobian problem"<<endl;
      }
      break;
    }
    case LimbEnum::RH_LEG:
     {
        KDL::ChainDynParam dynamic_param = KDL::ChainDynParam(RH_Chain, gravity_vector);
        error_code = dynamic_param.JntToGravity(joints, gravity_matrix);
        if(error_code != 0)
      {
        cout<<"Failed to solve Jacobian problem"<<endl;
      }
      break;
     }
  }
  JointTorquesLimb gravity_compensation_torque;
  for (int i = 0; i < number_of_joints; ++i) {
    gravity_compensation_torque(i) = gravity_matrix(i);
  }
  return gravity_compensation_torque;
}

double QuadrupedKinematics_kp::MapToPI(double q)
{
  double out;
  out = q;
  if(q>M_PI)
    out = 2*M_PI - q;
  if(q<-M_PI)
    out = 2*M_PI + q;
  return out;
}

Position QuadrupedKinematics_kp::getPositionBaseToHipInBaseFrame(const LimbEnum &limb) const
{
  Position hip_pose_in_base = hip_pose_in_base_.at(limb).getPosition();
  hip_pose_in_base.z() = 0.0;
  return hip_pose_in_base;
}
}//Namespace
