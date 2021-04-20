/*
 *  quadrupedkinematics.cpp
 *  Descriotion:
 *
 *  Created on: Mar 18, 2019
 *  Author: Shunyao Wang
 *  Institute: Harbin Institute of Technology, Shenzhen
 */
#include "quadruped_model/quadrupedkinematics.h"
#include <ros/package.h>
namespace quadruped_model {
using namespace std;
//get the robot model from urdf using a function from KDL
QuadrupedKinematics::QuadrupedKinematics()
{
  string urdf_dir = ros::package::getPath("quadruped_model") + "/urdf/laikago.urdf";
  LoadRobotDescriptionFromFile(urdf_dir);
  std::cout<<urdf_dir<<std::endl;
}

QuadrupedKinematics::~QuadrupedKinematics()
{
}

//get the robot model from other robot
QuadrupedKinematics::QuadrupedKinematics(const QuadrupedKinematics& other)
  : tree_(other.tree_),
    hip_pose_in_base_(other.hip_pose_in_base_),
    LF_Chain(other.LF_Chain),
    RF_Chain(other.RF_Chain),
    RH_Chain(other.RH_Chain),
    LH_Chain(other.LH_Chain)
{
//  cout<<"QuadrupedKinematics Class has been Copied"<<endl;
}

//using urdf to build the robot.
bool QuadrupedKinematics::LoadRobotDescriptionFromFile(const std::string filename)
{
  if(!kdl_parser::treeFromFile(filename, tree_))
  {
    ROS_ERROR("Failed to load robot description to KDL tree");
    return false;
  }  
  tree_.getChain("base", "LF_FOOT", LF_Chain);
  tree_.getChain("base", "RF_FOOT", RF_Chain);
  tree_.getChain("base", "RH_FOOT", RH_Chain);
  tree_.getChain("base", "LH_FOOT", LH_Chain);
  setHipPoseInBase(LF_Chain,LimbEnum::LF_LEG);
  setHipPoseInBase(RF_Chain,LimbEnum::RF_LEG);
  setHipPoseInBase(RH_Chain,LimbEnum::RH_LEG);
  setHipPoseInBase(LH_Chain,LimbEnum::LH_LEG);
  return true;
}

//set hip pose in baseCoor using the KDL
bool QuadrupedKinematics::setHipPoseInBase(const KDL::Chain& kdl_chain, const LimbEnum& limb)
{
  KDL::Frame cartisian_frame;
  //设置Segment的惯量,getSegment(0)
  //Frame is a 4*4 homoT
  cartisian_frame = kdl_chain.getSegment(0).getFrameToTip();
  Position translation(cartisian_frame(0,3), cartisian_frame(1,3), cartisian_frame(2,3));
  RotationMatrix rotation_matrix(cartisian_frame(0,0), cartisian_frame(0,1), cartisian_frame(0,2),
                                 cartisian_frame(1,0), cartisian_frame(1,1), cartisian_frame(1,2),
                                 cartisian_frame(2,0), cartisian_frame(2,1), cartisian_frame(2,2));
  hip_pose_in_base_[limb] = Pose(translation, RotationQuaternion(rotation_matrix));
  return true;
}

//TODO: seems like not using
Position QuadrupedKinematics:: getPositionBaseToLegMassCenterInBaseFrame(const LimbEnum& limb,
                                                                         const JointPositionsLimb& joint_positions) const
{
  KDL::Frame T01,T12,T23, cartisian_frame;
  Position leg_mass_center;
  KDL::Chain limb_chain;
  KDL::JntArray joints(2);
  joints(0) = joint_positions(0);
  joints(1) = joint_positions(1);
  switch (limb) {
    case LimbEnum::LF_LEG:
      {
        limb_chain.addSegment(LF_Chain.getSegment(0));
        limb_chain.addSegment(LF_Chain.getSegment(1));
        limb_chain.addSegment(KDL::Segment("lf_mass_center_link",KDL::Joint("mass_center_joint"), KDL::Frame(KDL::Vector(0.175,0.0,0.1))));
        break;
      }
    case LimbEnum::RF_LEG:
      {
        limb_chain.addSegment(RF_Chain.getSegment(0));
        limb_chain.addSegment(RF_Chain.getSegment(1));
        limb_chain.addSegment(KDL::Segment("rf_mass_center_link",KDL::Joint("mass_center_joint"), KDL::Frame(KDL::Vector(0.175,0.0,0.1))));
        break;
      }
    case LimbEnum::RH_LEG:
      {
        limb_chain.addSegment(RH_Chain.getSegment(0));
        limb_chain.addSegment(RH_Chain.getSegment(1));
        limb_chain.addSegment(KDL::Segment("rh_mass_center_link",KDL::Joint("mass_center_joint"), KDL::Frame(KDL::Vector(0.175,0.0,0.1))));
        break;
      }
    case LimbEnum::LH_LEG:
      {
        limb_chain.addSegment(LH_Chain.getSegment(0));
        limb_chain.addSegment(LH_Chain.getSegment(1));
        limb_chain.addSegment(KDL::Segment("lh_mass_center_link",KDL::Joint("mass_center_joint"), KDL::Frame(KDL::Vector(0.175,0.0,0.1))));
        break;
      }
    }

  KDL::ChainFkSolverPos_recursive fk_solver(limb_chain);
  fk_solver.JntToCart(joints, cartisian_frame);

  Position translation(cartisian_frame(0,3), cartisian_frame(1,3), cartisian_frame(2,3));
  RotationMatrix rotation_matrix(cartisian_frame(0,0), cartisian_frame(0,1), cartisian_frame(0,2),
                                 cartisian_frame(1,0), cartisian_frame(1,1), cartisian_frame(1,2),
                                 cartisian_frame(2,0), cartisian_frame(2,1), cartisian_frame(2,2));
  RotationQuaternion rotation(rotation_matrix);
  Position leg_mass_center_in_base = translation + rotation.rotate(leg_mass_center);
  return leg_mass_center_in_base;
}

//get foot position in hip_frame from foot position in base_postion
Position QuadrupedKinematics::getPositionFootToHipInHipFrame(const LimbEnum& limb, const Position& foot_position_in_base) const
{
  return hip_pose_in_base_.at(limb).inverseTransform(foot_position_in_base);
}

//using joint angles to compute cartesian_pose
bool QuadrupedKinematics::FowardKinematicsSolve(const JointPositionsLimb& joint_position,
                                                const LimbEnum& limb, Pose& cartisian_pose)
{
  int number_of_joints = joint_position.vector().size();
  KDL::JntArray joints = KDL::JntArray(number_of_joints);
  KDL::Frame cartisian_frame;
  for(int i = 0; i<number_of_joints; i++)
  {
    joints(i) = joint_position(i);
  }
  switch (limb) {
    case LimbEnum::LF_LEG:
      {
        KDL::ChainFkSolverPos_recursive lf_fk_solver(LF_Chain);
        if(lf_fk_solver.JntToCart(joints, cartisian_frame)<0)
        {
          cout<<"Failed to solve Forward kinematics problem"<<endl;
          return false;
        }
        break;
      }
    case LimbEnum::RF_LEG:
      {
        KDL::ChainFkSolverPos_recursive rf_fk_solver(RF_Chain);
        if(rf_fk_solver.JntToCart(joints, cartisian_frame)<0)
        {
          cout<<"Failed to solve Forward kinematics problem"<<endl;
          return false;
        }
        break;
      }
    case LimbEnum::LH_LEG:
    {
      KDL::ChainFkSolverPos_recursive lh_fk_solver(LH_Chain);
      if(lh_fk_solver.JntToCart(joints, cartisian_frame)<0)
      {
        cout<<"Failed to solve Forward kinematics problem"<<endl;
        return false;
      }
      break;
    }
    case LimbEnum::RH_LEG:
     {
      KDL::ChainFkSolverPos_recursive rh_fk_solver(RH_Chain);
      if(rh_fk_solver.JntToCart(joints, cartisian_frame)<0)
      {
        cout<<"Failed to solve Forward kinematics problem"<<endl;
        return false;
      }
      break;
     }

  }

  Eigen::Vector3d translation(cartisian_frame(0,3), cartisian_frame(1,3), cartisian_frame(2,3));
  RotationMatrix rotation_matrix(cartisian_frame(0,0), cartisian_frame(0,1), cartisian_frame(0,2),
                                 cartisian_frame(1,0), cartisian_frame(1,1), cartisian_frame(1,2),
                                 cartisian_frame(2,0), cartisian_frame(2,1), cartisian_frame(2,2));

//  Eigen::Vector3d leg(0.25,0,0);
//  // something wrong with rotation matrix?
//  cout<<leg<<endl;
  cartisian_pose = Pose(Position(translation), RotationQuaternion(rotation_matrix));
  return true;
}

//compute Jaccobian
bool QuadrupedKinematics::AnalysticJacobian(const JointPositionsLimb& joint_positions, const LimbEnum& limb, Eigen::MatrixXd& jacobian)
{
  int number_of_joints = joint_positions.vector().size();
  KDL::JntArray joints = KDL::JntArray(number_of_joints);
  KDL::Jacobian J;
  J.resize(number_of_joints);
  for(int i = 0; i<number_of_joints; i++)
  {
    joints(i) = joint_positions(i);
  }
  int error_code = 0;
  switch (limb) {
    case LimbEnum::LF_LEG:
      {
        //std::cout<<"LF_LEG...................."<<std::endl;
        KDL::ChainJntToJacSolver jacobian_solver(LF_Chain);
        error_code = jacobian_solver.JntToJac(joints, J);
        if(error_code != 0)
        {
          cout<<"Failed to solve Jacobian problem"<<" error code :"<<error_code<<endl;
          return false;
        }
        break;
      }
    case LimbEnum::RF_LEG:
      {
        //std::cout<<"RF_LEG...................."<<std::endl;
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
      //std::cout<<"LH_LEG...................."<<std::endl;
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
      //std::cout<<"RH_LEG...................."<<std::endl;
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
  //std::cout<<"jacobian computed by kdl is "<<jacobian<<std::endl;
  return true;
}

//compute Jaccobian_Dot using CalculateJacobianDotFromJointVel
bool QuadrupedKinematics::AnalysticJacobianDot(const JointPositionsLimb& joint_positions,
                                               const JointVelocitiesLimb& joint_velocity,
                                               const LimbEnum& limb, Eigen::MatrixXd& jacobian_dot)
{
  int number_of_joints = joint_velocity.vector().size();
  KDL::JntArray q(number_of_joints);
  KDL::JntArray qdot(number_of_joints);
  for(int i = 0; i<number_of_joints; i++)
  {
    qdot(i) = joint_velocity(i);
    q(i) = joint_positions(i);
  }

  KDL::JntArrayVel joint_vel(q, qdot);
  KDL::Jacobian JacDot;
  JacDot.resize(number_of_joints);
  int error_code = 0;

  if(error_code != 0)
  {
    cout<<"Failed to solve Jacobian problem"<<" error code :"<<error_code<<endl;
  }

  switch (limb) {
    case LimbEnum::LF_LEG:
      {
        if(!CalculateJacobianDotFromJointVel(LF_Chain, joint_vel, JacDot))
        {
          cout<<"Failed to solve Jacobian Dot problem"<<endl;
          return false;
        }
        break;
      }
    case LimbEnum::RF_LEG:
      {
        if(!CalculateJacobianDotFromJointVel(RF_Chain, joint_vel, JacDot))
        {
          cout<<"Failed to solve Jacobian Dot problem"<<endl;
          return false;
        }
        break;
      }
    case LimbEnum::LH_LEG:
    {
      if(!CalculateJacobianDotFromJointVel(LH_Chain, joint_vel, JacDot))
      {
        cout<<"Failed to solve Jacobian Dot problem"<<endl;
        return false;
      }
      break;
    }
    case LimbEnum::RH_LEG:
     {
      if(!CalculateJacobianDotFromJointVel(RH_Chain, joint_vel, JacDot))
      {
        cout<<"Failed to solve Jacobian Dot problem"<<endl;
        return false;
      }
      break;
     }

  }
  jacobian_dot = JacDot.data;
  return true;
}

//compute Jaccobian_Dot
bool QuadrupedKinematics::CalculateJacobianDotFromJointVel(const KDL::Chain& chain,
                                                           const KDL::JntArrayVel& q_in,
                                                           KDL::Jacobian& jdot)
{
  // Let's compute Jdot in the corresponding representation
  unsigned int segmentNr = chain.getNrOfSegments();
  int error_code = 0;
  int number_of_joints = chain.getNrOfJoints();
  KDL::Jacobian jac_;
  jac_.resize(number_of_joints);

  KDL::ChainJntToJacSolver jacobian_solver(chain);

  error_code = jacobian_solver.JntToJac(q_in.q, jac_);
  if(error_code != 0)
  {
    cout<<"Failed to solve Jacobian problem in 'CalculateJacobianDotFromJointVel()' "<<" error code :"<<error_code<<endl;
  }
  int k=0;
  KDL::Twist t_djdq_, jac_j_, jac_i_, jac_dot_k_;

  for(unsigned int i=0;i<segmentNr;++i)
  {
      //Only increase joint nr if the segment has a joint
      if(chain.getSegment(i).getJoint().getType()!=KDL::Joint::None){

          for(unsigned int j=0;j<chain.getNrOfJoints();++j)
          {
              // Column J is the sum of all partial derivatives  ref (41)
//              if(!locked_joints_[j])
              int joint_idx = j;
              int column_idx = k;

              jac_j_ = jac_.getColumn(joint_idx);
              jac_i_ = jac_.getColumn(column_idx);

              SetToZero(t_djdq_);

              if(joint_idx < column_idx)
              {
                  // P_{\Delta}({}_{bs}J^{j})  ref (20)
                  t_djdq_.vel = jac_j_.rot * jac_i_.vel;
                  t_djdq_.rot = jac_j_.rot * jac_i_.rot;
              }else if(joint_idx > column_idx)
              {
                  // M_{\Delta}({}_{bs}J^{j})  ref (23)
                  SetToZero(t_djdq_.rot);
                  t_djdq_.vel = -jac_j_.vel * jac_i_.rot;
              }else if(joint_idx == column_idx)
              {
                   // ref (40)
                   SetToZero(t_djdq_.rot);
                   t_djdq_.vel = jac_i_.rot * jac_i_.vel;
              }

            jac_dot_k_ += t_djdq_ * q_in.qdot(j);
          }
          jdot.setColumn(k++, jac_dot_k_);
          SetToZero(jac_dot_k_);
      }
  }
  return true;
}

//not using
bool QuadrupedKinematics::AnalysticJacobianForLink(const JointPositionsLimb& joint_positions, const LimbEnum& limb, const int& link_index, Eigen::MatrixXd& jacobian)
{
  int number_of_joints = link_index;
  KDL::JntArray joints = KDL::JntArray(number_of_joints);
  KDL::Jacobian J;
  J.resize(number_of_joints);
  for(int i = 0; i<number_of_joints; i++)
  {
    joints(i) = joint_positions(i);
  }
  int error_code = 0;
  switch (limb) {
    case LimbEnum::LF_LEG:
      {
        //自己设置一个链条?
        KDL::Chain LF_Chain_Link;
        LF_Chain_Link.addSegment(LF_Chain.getSegment(0));
        cout<<LF_Chain.getSegment(0).getName()<<endl;
        for(int i = 1; i<link_index; i++)
          {
            KDL::Vector com = LF_Chain.getSegment(i).getInertia().getCOG();
            LF_Chain_Link.addSegment(KDL::Segment(LF_Chain.getSegment(i).getName(),
                                                  LF_Chain.getSegment(i).getJoint(),
                                                  KDL::Frame(com)));
          }
        //
        KDL::ChainJntToJacSolver jacobian_solver(LF_Chain_Link);
        error_code = jacobian_solver.JntToJac(joints, J);
        if(error_code != 0)
        {
          cout<<"Failed to solve Jacobian problem"<<" error code :"<<error_code<<endl;
          return false;
        }
        break;
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

//get joint position from the foot position
bool QuadrupedKinematics::InverseKinematicsSolve(const Position& foot_position, const LimbEnum& limb,
                                                 const JointPositionsLimb& joint_positions_last,
                                                 JointPositionsLimb& joint_positions,
                                                 const std::string LimbType)
{
  double d,l1,l2,px,py,pz,alpha,beta1,beta2;
  d=0.037;
  l1=0.25;
  l2=0.25;
//  cout<<"px in base = "<<foot_position(0)<<endl
//      <<"py in base = "<<foot_position(1)<<endl
//      <<"pz in base = "<<foot_position(2)<<endl;
  Position foot_position_in_hip = getPositionFootToHipInHipFrame(limb, foot_position);
  px=foot_position_in_hip(0);
  py=foot_position_in_hip(1);
  pz=foot_position_in_hip(2);
//  cout<<"px in hip = "<<px<<endl
//      <<"py in hip = "<<py<<endl
//      <<"pz in hip = "<<pz<<endl;
  double cos_theta3 = (l2*l2 + l1*l1 - ((px*px + py*py + pz*pz) - d*d))/2/l1/l2;
  if(cos_theta3<-1)
    cos_theta3 = -1;
  if(cos_theta3>1)
    cos_theta3 = 1;
  Eigen::VectorXd theta3(4);
  Eigen::MatrixXd results(4,3);
  //right
  theta3(0) = M_PI - acos(cos_theta3);
  theta3(1) = M_PI - acos(cos_theta3);
  //left
  theta3(2) = -M_PI + acos(cos_theta3);
  theta3(3) = -M_PI + acos(cos_theta3);

  alpha = atan2(py,px);
  beta1 = atan2(d,sqrt(fabs(px*px + py*py - d*d)));
  beta2 = atan2(-d,-sqrt(fabs(px*px + py*py - d*d)));
  int i = 0;
//  while (i<4) {
//    double a,b,q1,q2,q3;
//    q3=MapToPI(theta3(i));

//    // Left arm configure
//    q1=MapToPI(alpha - beta1);
//    a = atan2(pz,-sqrt(fabs(px*px + py*py - d*d)));
//    b = atan2(l2*sin(q3), l1 + l2*cos(q3));
//    if(a>0)
//    {
//      q2 = MapToPI(a - b - M_PI);
//      results.row(i) << q1,q2,q3;
//    }
//    if(a<0)
//    {
//      q2 = MapToPI(a - b + M_PI);
//      results.row(i) << q1,q2,q3;
//    }


//    //right arm config
//    i = i +1;
//    q1 = MapToPI(alpha + beta2);
//    a = atan2(pz,sqrt(fabs(px*px + py*py - d*d)));
//    q2 = MapToPI(a - b + M_PI);
//    results.row(i) << q1,q2,q3;
//    i=i+1;
//  }
  //MXR::NOTE:  choose the right inverse_solution for limbType(which is defined in quadruped_state.cpp)
  if(LimbType == "IN_LEFT"){
      double a,b,q1,q2,q3;
      q3=MapToPI(theta3(2));
      cout<<alpha<<"--IL--" << beta1<<endl;

      // Left arm configure
      q1=MapToPI(alpha - beta1);
      a = atan2(pz,-sqrt(fabs(px*px + py*py - d*d)));
      b = atan2(l2*sin(q3), l1 + l2*cos(q3));
      if(a>0)
      {
        q2 = MapToPI(a - b - M_PI);
        results.row(2) << q1,q2,q3;
      }
      if(a<0)
      {
        q2 = MapToPI(a - b + M_PI);
        results.row(2) << q1,q2,q3;
      }
  }

  if(LimbType == "IN_RIGHT"){
      double a,b,q1,q2,q3;
      q3=MapToPI(theta3(1));
      //right arm config
      cout<<alpha<<"--IR--" << beta2<<endl;
      q1 = MapToPI(alpha + beta2);
      a = atan2(pz,sqrt(fabs(px*px + py*py - d*d)));
      b = atan2(l2*sin(q3), l1 + l2*cos(q3));
      q2 = MapToPI(a - b + M_PI);
      results.row(1) << q1,q2,q3;
  }

  if(LimbType == "OUT_LEFT"){
      double a,b,q1,q2,q3;
      q3=MapToPI(theta3(2));
      cout<<alpha<<"--OL--" << beta1<<endl;

      // Left arm configure
      q1 = MapToPI(alpha + beta1);

      q1=MapToPI(alpha - beta1);
      a = atan2(pz,-sqrt(fabs(px*px + py*py - d*d)));
      b = atan2(l2*sin(q3), l1 + l2*cos(q3));
      if(a>0)
      {
        q2 = MapToPI(a - b - M_PI);
        results.row(0) << q1,q2,q3;
      }
      if(a<0)
      {
        q2 = MapToPI(a - b + M_PI);
        results.row(0) << q1,q2,q3;
      }
  }

  if(LimbType == "OUT_RIGHT"){
      double a,b,q1,q2,q3;
      q3=MapToPI(theta3(1));
      //right arm config
      cout<<alpha<<"--OR--" << beta2<<endl;
      q1 = MapToPI(alpha + beta2);
      a = atan2(pz,sqrt(fabs(px*px + py*py - d*d)));
      b = atan2(l2*sin(q3), l1 + l2*cos(q3));
      q2 = MapToPI(a - b + M_PI);
      results.row(3) << q1,q2,q3;
  }


  double min_difference = 100;
  int min_index;
//  Eigen::MatrixXd v(1,3);
//  Eigen::MatrixXd w(1,3);
//  v<< 1,2,3;
//  w<< 1,1,1;
//  cout<<v.norm()<<endl;

  /*Eigen::VectorXd joint_last_vector;
  joint_last_vector.resize(3);
  joint_last_vector << joint_positions_last(0),joint_positions_last(1),joint_positions_last(2);


  for(int j = 0;j<4;j++)
  {
    Eigen::VectorXd da = joint_last_vector - results.row(j);
    if(da.norm()< min_difference)
    {
      min_difference = da.norm();
      min_index = j;
    }
//    cout<<da.normalized()<<endl;
    //    if(da.normalized())
  }*/

  if(LimbType == "IN_LEFT")
    min_index = 2;
  if(LimbType == "IN_RIGHT")
    min_index = 1;
  if(LimbType == "OUT_LEFT")
    min_index = 0;
  if(LimbType == "OUT_RIGHT")
    min_index = 3;

  joint_positions << results(min_index,0),results(min_index,1),results(min_index,2);
  std::cout<<"min_index= "<<min_index<<" "<<joint_positions<<std::endl;
//  cout<<results<<endl;

  if(!isnan(joint_positions(0))&&!isnan(joint_positions(1))&&!isnan(joint_positions(2))){
      return true;
    }else{
      ROS_WARN("Failed to Sovle Inverse Kinematics!");
      return false;
    }

}

//get gravity compensation for each leg
JointTorquesLimb QuadrupedKinematics::getGravityCompensationForLimb(const LimbEnum& limb,
                                               const JointPositionsLimb& joint_positions,
                                               const Force& gravity_in_baseframe)
{
  KDL::Vector gravity_vector = KDL::Vector(gravity_in_baseframe(0),
                                           gravity_in_baseframe(1),
                                           gravity_in_baseframe(2));
  int number_of_joints = 3;
  KDL::JntArray joints = KDL::JntArray(number_of_joints);
  KDL::JntArray gravity_matrix = KDL::JntArray(number_of_joints);
  for(int i = 0; i<number_of_joints; i++)
  {
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
  for(unsigned int i =0;i<number_of_joints;i++)
    gravity_compensation_torque(i) = gravity_matrix(i);

  return gravity_compensation_torque;
}

//mapping the angle to 0-PI
double QuadrupedKinematics::MapToPI(double q)
{
  double out;
  out = q;
  if(q>M_PI)
    out = 2*M_PI - q;
  if(q<-M_PI)
    out = 2*M_PI + q;
  return out;
}



//get Hip postion In Base_Frame
Position QuadrupedKinematics::getPositionBaseToHipInBaseFrame(const LimbEnum& limb) const
{
  Position hip_position_in_base = hip_pose_in_base_.at(limb).getPosition();
  hip_position_in_base.z() = 0.;
  return hip_position_in_base;
}
}//namespace
