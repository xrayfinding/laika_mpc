/*****************************************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, Péter Fankhauser, Christian Gehring, C. Dario Bellicoso, Stelian Coros
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Autonomous Systems Lab nor ETH Zurich
*     nor the names of its contributors may be used to endorse or
*     promote products derived from this software without specific
*     prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*/
/*!
* @file     VirtualModelController.hpp
* @author   Péter Fankhauser, Christian Gehring
* @date     Aug 6, 2013
* @brief
*/
#include "balance_controller/motion_control/VirtualModelController.hpp"
//#include "robotUtils/loggers/logger.hpp"
//#include "kindr/rotations/eigen/EulerAnglesZyx.hpp"

//#include "loco/temp_helpers/math.hpp"

using namespace std;
using namespace Eigen;

namespace balance_controller {

VirtualModelController::VirtualModelController(const ros::NodeHandle& node_handle,
                                               std::shared_ptr<free_gait::State> robot_state,
                                               std::shared_ptr<ContactForceDistributionBase> contactForceDistribution,
                                               std::shared_ptr<MPC::ConvexMpc> convexMpc)
    : MotionControllerBase(node_handle, robot_state),
      contactForceDistribution_(contactForceDistribution),
      convexMpc_(convexMpc),
      gravityCompensationForcePercentage_(1.0)
{

  limbs_.push_back(free_gait::LimbEnum::LF_LEG);
  limbs_.push_back(free_gait::LimbEnum::RF_LEG);
  limbs_.push_back(free_gait::LimbEnum::LH_LEG);
  limbs_.push_back(free_gait::LimbEnum::RH_LEG);

  dynamicReconfigureServer_.reset(new DynamicConfigServer(node_handle));
  reconfigureCallbackType_ = boost::bind(&VirtualModelController::dynamicReconfigureCallback, this, _1, _2);
  dynamicReconfigureServer_->setCallback(reconfigureCallbackType_);

}

VirtualModelController::~VirtualModelController()
{

}

void VirtualModelController::dynamicReconfigureCallback(balance_controller::balance_controllerConfig& config, uint32_t level)
{
  ROS_INFO("dynamic reconfigure update once");
  proportionalGainTranslation_.x() = config.heading_kp;
  derivativeGainTranslation_.x() = config.heading_kd;
  feedforwardGainTranslation_.x() = config.heading_kff;

  proportionalGainTranslation_.y() = config.lateral_kp;
  derivativeGainTranslation_.y() = config.lateral_kd;
  feedforwardGainTranslation_.y() = config.lateral_kff;

  proportionalGainTranslation_.z() = config.vertical_kp;
  derivativeGainTranslation_.z() = config.vertical_kd;
  feedforwardGainTranslation_.z() = config.vertical_kff;

  proportionalGainRotation_.x() = config.roll_kp;
  derivativeGainRotation_.x() = config.roll_kd;
  feedforwardGainRotation_.x() = config.roll_kff;

  proportionalGainRotation_.y() = config.pitch_kp;
  derivativeGainRotation_.y() = config.pitch_kd;
  feedforwardGainRotation_.y() = config.pitch_kff;

  proportionalGainRotation_.z() = config.yaw_kp;
  derivativeGainRotation_.z() = config.yaw_kd;
  feedforwardGainRotation_.z() = config.yaw_kff;

  gravityCompensationForcePercentage_ = config.gravity_compensation_percentage;

}

bool VirtualModelController::addToLogger()
{
//  robotUtils::addToLog(virtualForceInBaseFrame_.toImplementation(), "VMC_desired_force", "N", true);
//  robotUtils::addToLog(virtualTorqueInBaseFrame_.toImplementation(), "VMC_desired_torque", "Nm", true);

//  robotUtils::logger->addToLog(virtualForceInBaseFrame_.toImplementation(), "VMC_desired_force", "VMC", "N", true);
//  robotUtils::logger->addToLog(virtualTorqueInBaseFrame_.toImplementation(), "VMC_desired_torque", "VMC", "Nm", true);
  //! WSHY: the usage?
//  robotUtils::logger->addDoubleKindrForceToLog(virtualForceInBaseFrame_, "des_force", "VMC", "N", true);
//  robotUtils::logger->addDoubleKindrTorqueToLog(virtualTorqueInBaseFrame_, "des_torque", "VMC", "Nm", true);

//  robotUtils::logger->updateLogger(true);
  return true;
}
/**
 * @brief VirtualModelController::compute
 * @return
 */
bool VirtualModelController::compute()
{
//  if (!isParametersLoaded()) return false;

//  computeError();
//  computeGravityCompensation();
//  computeVirtualForce();
//  computeVirtualTorque();
//  if (!contactForceDistribution_->computeForceDistribution(virtualForceInBaseFrame_, virtualTorqueInBaseFrame_)) {
//    return false;
//  }

  /*
   *GoLaoxu:if follow codes are executed, the force that we used is caculated by MPC(sigle rigid body).
   *
   */
//  Position footpos = robot_state_->getPositionBaseToFootInBaseFrame(free_gait::LimbEnum::LF_LEG);
  collections_4_mpc(true);
  vector<double> mpc_ans =  convexMpc_->ComputeContactForces(com_position, com_velocity, com_roll_pitch_yaw, com_angular_velocity,
                                   foot_contact_states,foot_positions_body_frame ,foot_friction_coeffs,
                                   desired_com_position, desired_com_velocity, desired_com_roll_pitch_yaw,desired_com_angular_velocity);
  vector<double> mpc_use(mpc_ans.begin(), mpc_ans.begin()+12);
    if (!contactForceDistribution_->computeForceDistribution(virtualForceInBaseFrame_, virtualTorqueInBaseFrame_, mpc_use)) {
      return false;
    }
  return true;
}

bool VirtualModelController::QuaternionToEuler_desired(){
    vector<double> euler;
    euler.resize(3);
    const double Epsilon = 0.0009765625f;
        const double Threshold = 0.5f - Epsilon;

        double TEST = desired_quaternion[3] * desired_quaternion[1] - desired_quaternion[0] * desired_quaternion[2];

        if (TEST < -Threshold || TEST > Threshold) // 奇异姿态,俯仰角为±90°
        {
            int sign = 1;
            if(TEST <= -1e-7){
                sign = -1;
            }
            euler[2] = -2 * sign * (double)atan2(desired_quaternion[0], desired_quaternion[3]); // yaw

            euler[1] = sign * (EIGEN_PI / 2.0); // pitch

            euler[0] = 0; // roll

        }
        else
        {
            euler[0] = atan2(2 * (desired_quaternion[1]*desired_quaternion[2] + desired_quaternion[3]*desired_quaternion[0]), desired_quaternion[3]*desired_quaternion[3] - desired_quaternion[0]*desired_quaternion[0] - desired_quaternion[1]*desired_quaternion[1] + desired_quaternion[2]*desired_quaternion[2]);
            euler[1] = asin(-2 * (desired_quaternion[0]*desired_quaternion[2] - desired_quaternion[3]*desired_quaternion[1]));
            euler[2] = atan2(2 * (desired_quaternion[0]*desired_quaternion[1] + desired_quaternion[3]*desired_quaternion[2]), desired_quaternion[3]*desired_quaternion[3] + desired_quaternion[0]*desired_quaternion[0] - desired_quaternion[1]*desired_quaternion[1] - desired_quaternion[2]*desired_quaternion[2]);
        }
        desired_com_roll_pitch_yaw.assign(euler.begin(), euler.end());
        return true;
}

bool VirtualModelController::QuaternionToEuler(){
    vector<double> euler;
    euler.resize(3);
    const double Epsilon = 0.0009765625f;
        const double Threshold = 0.5f - Epsilon;

        double TEST = quaternion[3] * quaternion[1] - quaternion[0] * quaternion[2];

        if (TEST < -Threshold || TEST > Threshold) // 奇异姿态,俯仰角为±90°
        {
            int sign = 1;
            if(TEST <= -1e-7){
                sign = -1;
            }
            euler[2] = -2 * sign * (double)atan2(desired_quaternion[0], desired_quaternion[3]); // yaw

            euler[1] = sign * (EIGEN_PI / 2.0); // pitch

            euler[0] = 0; // roll

        }
        else
        {
            euler[0] = atan2(2 * (quaternion[1]*quaternion[2] + quaternion[3]*quaternion[0]), quaternion[3]*quaternion[3] - quaternion[0]*quaternion[0] - quaternion[1]*quaternion[1] + quaternion[2]*quaternion[2]);
            euler[1] = asin(-2 * (quaternion[0]*quaternion[2] - quaternion[3]*quaternion[1]));
            euler[2] = atan2(2 * (quaternion[0]*quaternion[1] + quaternion[3]*quaternion[2]), quaternion[3]*quaternion[3] + quaternion[0]*quaternion[0] - quaternion[1]*quaternion[1] - quaternion[2]*quaternion[2]);
        }
        com_roll_pitch_yaw.assign(euler.begin(), euler.end());
        return true;
}


bool VirtualModelController::computeError()
{
  //! WSHY: torso_ can be replaced with adapter or state
  // Errors are defined as (q_desired - q_actual).

  /***************************************************
   *  Method I
   ***************************************************/
  //! WSHY: control frame set as local Map frame, which yaw angel align with base
//  EulerAnglesZyx orinetationWorldToBase(robot_state_->getTargetOrientationBaseToWorld().inverted());

//  const RotationQuaternion& orientationControlToBase = RotationQuaternion(EulerAnglesZyx(0.0,
//                                                                                         orinetationWorldToBase.setUnique().vector()(1),
//                                                                                         orinetationWorldToBase.setUnique().vector()(2)));
//  const RotationQuaternion& orientationWorldToControl = RotationQuaternion(orinetationWorldToBase) * orientationControlToBase.inverted();

   const RotationQuaternion& orientationControlToBase = robot_state_->getTargetOrientationBaseToWorld().inverted();//torso_->getMeasuredState().getOrientationControlToBase();
   positionErrorInControlFrame_ = robot_state_->getTargetPositionWorldToBaseInWorldFrame() //torso_->getDesiredState().getPositionControlToBaseInControlFrame()
       - robot_state_->getPositionWorldToBaseInWorldFrame();//torso_->getMeasuredState().getPositionControlToBaseInControlFrame();
   //! WSHY: in base !!!
    kindr::RotationQuaternionD test;
    test.w() = robot_state_->getOrientationBaseToWorld().inverted().w();
    test.x() = 0.0001;
    test.y() = 0.0005;
    test.z() = robot_state_->getOrientationBaseToWorld().inverted().z();
  orientationError_ = -orientationControlToBase.boxMinus(robot_state_->getOrientationBaseToWorld().inverted());//torso_->getDesiredState().getOrientationControlToBase().boxMinus(
      //torso_->getMeasuredState().getOrientationControlToBase());
//  if(robot_state_->getOrientationBaseToWorld().x()<0.001&&robot_state_->getOrientationBaseToWorld().y()<0.001){
//      ROS_WARN("IN THIS BRANCH");
//      orientationError_ = -orientationControlToBase.boxMinus(test);
//  }
  /***************************************************
   *  Method II
   ***************************************************/
//  const RotationQuaternion& orientationWorldToControl = torso_->getMeasuredState().getOrientationWorldToControl();
//  const Position positionErrorInWorldFrame_ = torso_->getDesiredState().getPositionControlToBaseInControlFrame()
//          - torso_->getMeasuredState().getPositionControlToBaseInControlFrame();
//  positionErrorInBaseFrame_ = torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame().rotate(positionErrorInWorldFrame_);
//
//  orientationError_ = torso_->getDesiredState().getWorldToBaseOrientationInWorldFrame().boxMinus(
//      torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame());


  linearVelocityErrorInControlFrame_ = robot_state_->getTargetLinearVelocityBaseInWorldFrame()
      - robot_state_->getLinearVelocityBaseInWorldFrame();//orientationControlToBase.inverseRotate(robot_state_->getLinearVelocityBaseInWorldFrame());//torso_->getDesiredState().getLinearVelocityBaseInControlFrame() - orientationControlToBase.inverseRotate(torso_->getMeasuredState().getLinearVelocityBaseInBaseFrame());
  angularVelocityErrorInControlFrame_ = robot_state_->getTargetAngularVelocityBaseInBaseFrame()
      -robot_state_->getAngularVelocityBaseInBaseFrame();//torso_->getDesiredState().getAngularVelocityBaseInControlFrame() - orientationControlToBase.inverseRotate(torso_->getMeasuredState().getAngularVelocityBaseInBaseFrame());
 return true;
}

bool VirtualModelController::computeGravityCompensation()
{
  //! WSHY: torso_ can be replaced with adapter or state
  const LinearAcceleration gravitationalAccelerationInWorldFrame = LinearAcceleration(0.0,0.0,-9.8);//torso_->getProperties().getGravity();
  LinearAcceleration gravitationalAccelerationInBaseFrame = robot_state_->getOrientationBaseToWorld().inverseRotate(gravitationalAccelerationInWorldFrame);//torso_->getMeasuredState().getOrientationWorldToBase().rotate(gravitationalAccelerationInWorldFrame);

//  gravitationalAccelerationInBaseFrame /= 1.1;

  const Force forceTorso = Force(-gravityCompensationForcePercentage_*robot_state_->getRobotMass() * gravitationalAccelerationInBaseFrame);
  gravityCompensationForceInBaseFrame_ = forceTorso;
  //! WSHY: F = mg, T = r_com X F
  gravityCompensationTorqueInBaseFrame_ = Torque(robot_state_->getCenterOfMassInBase().cross(forceTorso));//Torque(torso_->getProperties().getBaseToCenterOfMassPositionInBaseFrame().cross(forceTorso));

  for (const auto& leg : limbs_)
  {

        const Force forceLeg = Force(-gravityCompensationForcePercentage_*robot_state_->getLegMass(leg) * gravitationalAccelerationInBaseFrame);
        gravityCompensationForceInBaseFrame_ += forceLeg;
//        if(!robot_state_->isSupportLeg(leg))
//          {
            gravityCompensationTorqueInBaseFrame_ += Torque(robot_state_->getPositionLegBaseToCoMInBaseFrame(leg).cross(forceLeg));//Torque(leg->getProperties().getBaseToCenterOfMassPositionInBaseFrame().cross(forceLeg));
//          }

  }
//  Force gravityCompensationForceWorldFrame_ = torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame().inverseRotate(gravityCompensationForce_);
//  gravityCompensationForceWorldFrame_.x() = 0.0;
//  gravityCompensationForceWorldFrame_.y() = 0.0;
//  gravityCompensationForce_ = torso_->getMeasuredState().getWorldToBaseOrientationInWorldFrame().rotate(gravityCompensationForceWorldFrame_);

  return true;
}


bool VirtualModelController::computeVirtualForce()
{

  //! WSHY: torso_ can be replaced with adapter or state
  //!
//  EulerAnglesZyx EulerZYXWorldToBase(robot_state_->getTargetOrientationBaseToWorld().inverted());

//  const RotationQuaternion& orientationControlToBase = RotationQuaternion(EulerAnglesZyx(0.0,
//                                                                                         EulerZYXWorldToBase.setUnique().vector()(1),
//                                                                                         EulerZYXWorldToBase.setUnique().vector()(2)));
//  const RotationQuaternion& orientationWorldToBase = RotationQuaternion(EulerZYXWorldToBase);
//  const RotationQuaternion& orientationWorldToControl = orientationWorldToBase * orientationControlToBase.inverted();

  const RotationQuaternion& orientationControlToBase = robot_state_->getOrientationBaseToWorld().inverted();//torso_->getMeasuredState().getOrientationControlToBase();
  const RotationQuaternion& orientationWorldToBase = robot_state_->getOrientationBaseToWorld().inverted();//torso_->getMeasuredState().getOrientationWorldToBase();
  const RotationQuaternion& orientationWorldToControl = orientationWorldToBase*orientationControlToBase.inverted();//robot_state_->getTargetOrientationBaseToWorld();//torso_->getMeasuredState().getOrientationWorldToControl();

  Vector3d feedforwardTermInControlFrame = Vector3d::Zero();
  feedforwardTermInControlFrame.x() += robot_state_->getTargetLinearVelocityBaseInWorldFrame().x();//torso_->getDesiredState().getLinearVelocityBaseInControlFrame().x();
  feedforwardTermInControlFrame.y() += robot_state_->getTargetLinearVelocityBaseInWorldFrame().y();//torso_->getDesiredState().getLinearVelocityBaseInControlFrame().y();
  //! WSHY: the follow 6 lines are not using yet
  Position positionErrorInWorldFrame = orientationWorldToControl.inverseRotate(positionErrorInControlFrame_);
  Force gravityCompensationFeedbackInWorldFrame = Force(proportionalGainTranslation_.cwiseProduct(Position(0.0,0.0,positionErrorInWorldFrame.z()).toImplementation()));

  LinearVelocity velocityErrorInWorldFrame = orientationWorldToControl.inverseRotate(linearVelocityErrorInControlFrame_);
  Force gravityDampingCompensationFeedbackInWorldFrame = Force(derivativeGainTranslation_.cwiseProduct(Position(0.0,0.0,velocityErrorInWorldFrame.z()).toImplementation()));

  Force gravityCompensationFeedbackInBaseFrame = orientationWorldToBase.rotate(gravityCompensationFeedbackInWorldFrame);
  Force gravityDampingCompensationFeedbackInBaseFrame = orientationWorldToBase.rotate(gravityDampingCompensationFeedbackInWorldFrame);


//  if(std::abs(linearVelocityErrorInControlFrame_.toImplementation()[0])<0.01){
//      linearVelocityErrorInControlFrame_.toImplementation()[0] = 0.0;
//  }
//  if(std::abs(linearVelocityErrorInControlFrame_.toImplementation()[1])<0.01){
//      linearVelocityErrorInControlFrame_.toImplementation()[1] = 0.0;
//  }
//  if(std::abs(linearVelocityErrorInControlFrame_.toImplementation()[2])<0.01){
//      linearVelocityErrorInControlFrame_.toImplementation()[2] = 0.0;
//  }
//  virtualForceInBaseFrame_ = orientationControlToBase.rotate(Force(proportionalGainTranslation_.cwiseProduct(positionErrorInControlFrame_.toImplementation())))
//                       + orientationControlToBase.rotate(Force(derivativeGainTranslation_.cwiseProduct(linearVelocityErrorInControlFrame_.toImplementation())))
//                       + orientationControlToBase.rotate(Force(feedforwardGainTranslation_.cwiseProduct(feedforwardTermInControlFrame)))
//                       + gravityCompensationForceInBaseFrame_;
  //! WSHY:
  virtualForceInBaseFrame_ = Force(proportionalGainTranslation_.cwiseProduct(orientationControlToBase.rotate(positionErrorInControlFrame_).toImplementation()))
                       + Force(derivativeGainTranslation_.cwiseProduct(orientationControlToBase.rotate(linearVelocityErrorInControlFrame_).toImplementation()))
                       + Force(feedforwardGainTranslation_.cwiseProduct(orientationControlToBase.rotate(feedforwardTermInControlFrame)))
                       + gravityCompensationForceInBaseFrame_
                       + gravityCompensationFeedbackInBaseFrame
                       + gravityDampingCompensationFeedbackInBaseFrame;

  return true;
}


bool VirtualModelController::computeVirtualTorque()
{
  const RotationQuaternion& orientationControlToBase = robot_state_->getOrientationBaseToWorld().inverted();//torso_->getMeasuredState().getOrientationControlToBase();

  Vector3d feedforwardTermInControlFrame = Vector3d::Zero();
  feedforwardTermInControlFrame.z() += robot_state_->getTargetAngularVelocityBaseInBaseFrame().z();//torso_->getDesiredState().getAngularVelocityBaseInControlFrame().z();


//  virtualTorqueInBaseFrame_ = Torque(proportionalGainRotation_.cwiseProduct(orientationError_))
//                       + orientationControlToBase.rotate(Torque(derivativeGainRotation_.cwiseProduct(angularVelocityErrorInControlFrame_.toImplementation())))
//                       + orientationControlToBase.rotate(Torque(feedforwardGainRotation_.cwiseProduct(feedforwardTermInControlFrame)))
//                       + gravityCompensationTorqueInBaseFrame_;

//  if(std::abs(angularVelocityErrorInControlFrame_.toImplementation()[0])<0.05){
//      angularVelocityErrorInControlFrame_.toImplementation()[0] = 0.0;
//  }
//  if(std::abs(angularVelocityErrorInControlFrame_.toImplementation()[1])<0.05){
//      angularVelocityErrorInControlFrame_.toImplementation()[1] = 0.0;
//  }
//  if(std::abs(angularVelocityErrorInControlFrame_.toImplementation()[2])<0.05){
//      angularVelocityErrorInControlFrame_.toImplementation()[2] = 0.0;
//  }
  virtualTorqueInBaseFrame_ = Torque(proportionalGainRotation_.cwiseProduct(orientationError_))
      //+Torque(derivativeGainRotation_.cwiseProduct(angularVelocityErrorInControlFrame_.toImplementation()))
     // +Torque(feedforwardGainRotation_.cwiseProduct(feedforwardTermInControlFrame))
                       +orientationControlToBase.rotate(Torque(derivativeGainRotation_.cwiseProduct(angularVelocityErrorInControlFrame_.toImplementation())))
                       +orientationControlToBase.rotate(Torque(feedforwardGainRotation_.cwiseProduct(feedforwardTermInControlFrame)))
                       + gravityCompensationTorqueInBaseFrame_;

  return true;
}


bool VirtualModelController::collections_4_mpc(){
    //1.com_position
    Position _com_position = robot_state_->getPositionWorldToBaseInWorldFrame();
    com_position.resize(3);
    com_position[0] = _com_position.x();
    com_position[1] = _com_position.y();
    com_position[2] = _com_position.z();
    //2.com_velocity
    LinearVelocity _com_velocity = robot_state_->getLinearVelocityBaseInWorldFrame();
    com_velocity.resize(3);
    com_velocity[0] = _com_velocity.x();
    com_velocity[1] = _com_velocity.y();
    com_velocity[2] = _com_velocity.z();
    //3.rpy
    com_roll_pitch_yaw.resize(3);
    RotationQuaternion _rotation_orien= robot_state_->getOrientationBaseToWorld();
    quaternion.resize(4);
    quaternion[0] = _rotation_orien.x();
    quaternion[1] = _rotation_orien.y();
    quaternion[2] = _rotation_orien.z();
    quaternion[3] = _rotation_orien.w();
    QuaternionToEuler();
    //don't consider the yaw angel
//    com_roll_pitch_yaw[2] = 0;
    //4.ang_vel
    LocalAngularVelocity _com_angular_velocity = robot_state_->getAngularVelocityBaseInBaseFrame();
    com_angular_velocity.resize(3);
    com_angular_velocity[0] = _com_angular_velocity.x();
    com_angular_velocity[1] = _com_angular_velocity.y();
    com_angular_velocity[2] = _com_angular_velocity.z();
    //5.contact_states
    foot_contact_states.resize(4);
    for (const auto& leg : limbs_){
        int _n;
        switch (leg) {
            case free_gait::LimbEnum::LF_LEG:
                _n = 0;
                break;
            case free_gait::LimbEnum::RF_LEG:
                _n = 1;
                break;
            case free_gait::LimbEnum::RH_LEG:
                _n = 2;
                break;
            case free_gait::LimbEnum::LH_LEG:
                _n = 3;
                break;
            default:
                ROS_ERROR("no leg be chosen");
        }
        if(robot_state_->isSupportLeg(leg)){
            foot_contact_states[_n] = 1;
        }
        else{
            foot_contact_states[_n] = 0;
        }
    }
    //6.foot_postion
    foot_positions_body_frame.resize(12);
    for(const auto& leg: limbs_){
        int _n;
        switch (leg) {
            case free_gait::LimbEnum::LF_LEG:
                _n = 0;
                break;
            case free_gait::LimbEnum::RF_LEG:
                _n = 1;
                break;
            case free_gait::LimbEnum::RH_LEG:
                _n = 2;
                break;
            case free_gait::LimbEnum::LH_LEG:
                _n = 3;
                break;
            default:
                ROS_ERROR("no leg be chosen");
        }
        Position footpos = robot_state_->getPositionBaseToFootInBaseFrame(leg);
        foot_positions_body_frame[_n * 3]  = footpos.x();
        foot_positions_body_frame[_n * 3 + 1]  = footpos.y();
        foot_positions_body_frame[_n * 3 + 2]  = footpos.z();
    }
    //7.frition_coeffs
    foot_friction_coeffs = {0.45, 0.45, 0.45, 0.45};
    //8.desire_pos
    Position _desired_com_position = robot_state_->getTargetPositionWorldToBaseInWorldFrame();
    desired_com_position.resize(3);
    desired_com_position[0] = _desired_com_position.x();
    desired_com_position[1] = _desired_com_position.y();
    desired_com_position[2] = _desired_com_position.z();
    //9.desire_vel
    LinearVelocity _desired_com_velocity = robot_state_->getTargetLinearVelocityBaseInWorldFrame();
    desired_com_velocity.resize(3);
    desired_com_velocity[0] = _desired_com_velocity.x();
    desired_com_velocity[1] = _desired_com_velocity.y();
    desired_com_velocity[2] = _desired_com_velocity.z();
    //10.desire_rpy
    _rotation_orien = robot_state_->getTargetOrientationBaseToWorld();
    desired_quaternion.resize(4);
    desired_quaternion[0] = _rotation_orien.x();
    desired_quaternion[1] = _rotation_orien.y();
    desired_quaternion[2] = _rotation_orien.z();
    desired_quaternion[3] = _rotation_orien.w();
    desired_com_roll_pitch_yaw.resize(3);
    QuaternionToEuler_desired();
    //11.desire_ang_vel
    LocalAngularVelocity _desired_com_angular_velocity = robot_state_->getTargetAngularVelocityBaseInBaseFrame();
    desired_com_angular_velocity.resize(3);
    desired_com_angular_velocity[0] = _desired_com_angular_velocity.x();
    desired_com_angular_velocity[1] = _desired_com_angular_velocity.y();
    desired_com_angular_velocity[2] = _desired_com_angular_velocity.z();
}

bool VirtualModelController::collections_4_mpc(bool ways_mit){
    if(!ways_mit){
        return false;
    }
    const RotationQuaternion& orientationControlToBase = robot_state_->getOrientationBaseToWorld().inverted();//torso_->getMeasuredState().getOrientationControlToBase();
    const RotationQuaternion& orientationWorldToBase = robot_state_->getOrientationBaseToWorld().inverted();//torso_->getMeasuredState().getOrientationWorldToBase();
    const RotationQuaternion& orientationWorldToControl = orientationWorldToBase*orientationControlToBase.inverted();//robot_state_->getTargetOrientationBaseToWorld();//torso_->getMeasuredState().getOrientationWorldToControl();

    //1.com_position
    Position _com_position = robot_state_->getPositionWorldToBaseInWorldFrame();
    com_position.resize(3);
    com_position[0] = _com_position.x();
    com_position[1] = _com_position.y();
    com_position[2] = _com_position.z();

    com_position[0] = 0.0;
    com_position[1] = 0.0;

    //2.com_velocity
    LinearVelocity _com_velocity = robot_state_->getLinearVelocityBaseInWorldFrame();
    com_velocity.resize(3);
    com_velocity[0] = _com_velocity.x()/0.36;
    com_velocity[1] = _com_velocity.y()/0.36;
    com_velocity[2] = _com_velocity.z()/0.36;
    //3.rpy
    com_roll_pitch_yaw.resize(3);
    RotationQuaternion _rotation_orien= robot_state_->getOrientationBaseToWorld();
    quaternion.resize(4);
    quaternion[0] = _rotation_orien.x();
    quaternion[1] = _rotation_orien.y();
    quaternion[2] = _rotation_orien.z();
    quaternion[3] = _rotation_orien.w();
    QuaternionToEuler();

    //don't consider the yaw angel
    com_roll_pitch_yaw[2] = 0;
    //4.ang_vel


   // LocalAngularVelocity _com_angular_velocity =  robot_state_->getAngularVelocityBaseInBaseFrame();

    LocalAngularVelocity _com_angular_velocity =  orientationControlToBase.rotate(robot_state_->getAngularVelocityBaseInBaseFrame());
    com_angular_velocity.resize(3);
    com_angular_velocity[0] = _com_angular_velocity.x();
    com_angular_velocity[1] = _com_angular_velocity.y();
    com_angular_velocity[2] = _com_angular_velocity.z();
    //5.contact_states
    foot_contact_states.resize(4);
    for (const auto& leg : limbs_){
        int _n;
        switch (leg) {
            case free_gait::LimbEnum::LF_LEG:
                _n = 0;
                break;
            case free_gait::LimbEnum::RF_LEG:
                _n = 1;
                break;
            case free_gait::LimbEnum::RH_LEG:
                _n = 2;
                break;
            case free_gait::LimbEnum::LH_LEG:
                _n = 3;
                break;
            default:
                ROS_ERROR("no leg be chosen");
        }
        if(robot_state_->isSupportLeg(leg)){
            foot_contact_states[_n] = 1;
        }
        else{
            foot_contact_states[_n] = 0;
        }
    }
    //6.foot_postion
    foot_positions_body_frame.resize(12);
    for(const auto& leg: limbs_){
        int _n;
        switch (leg) {
            case free_gait::LimbEnum::LF_LEG:
                _n = 0;
                break;
            case free_gait::LimbEnum::RF_LEG:
                _n = 1;
                break;
            case free_gait::LimbEnum::RH_LEG:
                _n = 2;
                break;
            case free_gait::LimbEnum::LH_LEG:
                _n = 3;
                break;
            default:
                ROS_ERROR("no leg be chosen");
        }
        Position footpos = robot_state_->getPositionBaseToFootInBaseFrame(leg);
        foot_positions_body_frame[_n * 3]  = footpos.x();
        foot_positions_body_frame[_n * 3 + 1]  = footpos.y();
        foot_positions_body_frame[_n * 3 + 2]  = footpos.z();
    }
//    std::cout << "Real foot position: " << foot_positions_body_frame[0] <<" " <<
//                 foot_positions_body_frame[1]<<" "<<
//                 foot_positions_body_frame[2]<< endl;
    //7.frition_coeffs
    foot_friction_coeffs = {0.45, 0.45, 0.45, 0.45};
    //8.desire_pos
    Position _desired_com_position = robot_state_->getTargetPositionWorldToBaseInWorldFrame();
    desired_com_position.resize(3);
    desired_com_position[0] = _desired_com_position.x();
    desired_com_position[1] = _desired_com_position.y();
    desired_com_position[2] = _desired_com_position.z();

    desired_com_position[0] = 0.0;
    desired_com_position[1] = 0.0;
    //0319
    //9.desire_vel
    LinearVelocity _desired_com_velocity = robot_state_->getTargetLinearVelocityBaseInWorldFrame();
    desired_com_velocity.resize(3);
    desired_com_velocity[0] = _desired_com_velocity.x();
    desired_com_velocity[1] = _desired_com_velocity.y();
    desired_com_velocity[2] = _desired_com_velocity.z();
    desired_com_velocity[2] = 0.0;

    //0319
    desired_com_velocity[0] = 0;
    desired_com_velocity[1] = 0;
    desired_com_velocity[2] = 0;

    //10.desire_rpy
    _rotation_orien = robot_state_->getTargetOrientationBaseToWorld();
    desired_quaternion.resize(4);
    desired_quaternion[0] = _rotation_orien.x();
    desired_quaternion[1] = _rotation_orien.y();
    desired_quaternion[2] = _rotation_orien.z();
    desired_quaternion[3] = _rotation_orien.w();
    desired_com_roll_pitch_yaw.resize(3);
    QuaternionToEuler_desired();
//    std::cout << "desired rpy " <<
//                 desired_com_roll_pitch_yaw[2] << " "<< std::endl;
    desired_com_roll_pitch_yaw[0] = 0.0;
    desired_com_roll_pitch_yaw[1] = 0.0;
    desired_com_roll_pitch_yaw[2] = 0.0;

//    std::cout << " rpy " <<
//                 com_roll_pitch_yaw[2] << " "<< std::endl;

    //11.desire_ang_vel
    LocalAngularVelocity _desired_com_angular_velocity = robot_state_->getTargetAngularVelocityBaseInBaseFrame();
    desired_com_angular_velocity.resize(3);
    desired_com_angular_velocity[0] = _desired_com_angular_velocity.x();
    desired_com_angular_velocity[1] = _desired_com_angular_velocity.y();
    desired_com_angular_velocity[2] = _desired_com_angular_velocity.z();
    desired_com_angular_velocity[0] = 0.0;
    desired_com_angular_velocity[1] = 0.0;
    //0319
    //Golaoxu : this has great influence of mpc controller
    desired_com_angular_velocity[2] = 0.0;

    return  true;
}


bool VirtualModelController::isParametersLoaded() const
{
  if (isParametersLoaded_) return true;

  cout << "Virtual model control parameters are not loaded." << endl; // TODO use warning output
  return false;
}


std::ostream& operator << (std::ostream& out, const VirtualModelController& motionController)
{
  IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");
  std::cout.precision(3);
//  kindr::EulerAnglesYprPD a;
  kindr::EulerAnglesYprPD errorYawRollPitch(motionController.orientationError_);
  Force netForce, netForceError;
  Torque netTorque, netTorqueError;
  motionController.contactForceDistribution_->getNetForceAndTorqueOnBase(netForce, netTorque);
  netForceError = motionController.virtualForceInBaseFrame_ - netForce;
  netTorqueError = motionController.virtualTorqueInBaseFrame_ - netTorque;

  motionController.isParametersLoaded();
  out << "Position error" << motionController.positionErrorInControlFrame_.toImplementation().format(CommaInitFmt) << endl;
  out << "Orientation error" << motionController.orientationError_.format(CommaInitFmt) << endl;
  out << "Linear velocity error" << motionController.linearVelocityErrorInControlFrame_.toImplementation().format(CommaInitFmt) << endl;
  out << "Angular velocity error" << motionController.angularVelocityErrorInControlFrame_.toImplementation().format(CommaInitFmt) << endl;
  out << "Gravity compensation force" << motionController.gravityCompensationForceInBaseFrame_.toImplementation().format(CommaInitFmt) << endl;
  out << "Gravity compensation torque" << motionController.gravityCompensationTorqueInBaseFrame_.toImplementation().format(CommaInitFmt) << endl;
  out << "Desired virtual force" << motionController.virtualForceInBaseFrame_.toImplementation().format(CommaInitFmt) << endl;
  out << "Desired virtual torque" << motionController.virtualTorqueInBaseFrame_.toImplementation().format(CommaInitFmt) << endl;
  out << "Net force error" << netForceError.toImplementation().format(CommaInitFmt) << endl;
  out << "Net torque error" << netTorqueError.toImplementation().format(CommaInitFmt) << endl;
  out << "gravity comp k: " << motionController.gravityCompensationForcePercentage_ << endl;
  return out;
}

Force VirtualModelController::getDesiredVirtualForceInBaseFrame() const {
  return virtualForceInBaseFrame_;
}

Torque VirtualModelController::getDesiredVirtualTorqueInBaseFrame() const {
  return virtualTorqueInBaseFrame_;
}

void VirtualModelController::getDistributedVirtualForceAndTorqueInBaseFrame(Force& netForce, Torque& netTorque) const {
  contactForceDistribution_->getNetForceAndTorqueOnBase(netForce, netTorque);
}
/**
 * @brief VirtualModelController::loadParameters, replace it with rosparam
 * @param handle
 * @return
 */
bool VirtualModelController::loadParameters(const TiXmlHandle& handle)
{
  isParametersLoaded_ = false;
  TiXmlElement* element;

  TiXmlHandle gainsHandle(handle.FirstChild("VirtualModelController").FirstChild("Gains"));
  element = gainsHandle.Element();
  if (!element) {
    printf("Could not find VirtualModelController:Gains\n");
    return false;
  }

  // Heading
  element = gainsHandle.FirstChild("Heading").Element();
  if (element->QueryDoubleAttribute("kp", &proportionalGainTranslation_.x())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Heading:kp\n");
    return false;
  }
  if (element->QueryDoubleAttribute("kd", &derivativeGainTranslation_.x())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Heading:kd\n");
    return false;
  }
  if (element->QueryDoubleAttribute("kff", &feedforwardGainTranslation_.x())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Heading:kff\n");
    return false;
  }

  // Lateral
  element = gainsHandle.FirstChild("Lateral").Element();
  if (element->QueryDoubleAttribute("kp", &proportionalGainTranslation_.y())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Lateral:kp\n");
    return false;
  }
  if (element->QueryDoubleAttribute("kd", &derivativeGainTranslation_.y())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Lateral:kd\n");
    return false;
  }
  if (element->QueryDoubleAttribute("kff", &feedforwardGainTranslation_.y())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Lateral:kff\n");
    return false;
  }

  // Vertical
  element = gainsHandle.FirstChild("Vertical").Element();
  if (element->QueryDoubleAttribute("kp", &proportionalGainTranslation_.z())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Vertical:kp\n");
    return false;
  }
  if (element->QueryDoubleAttribute("kd", &derivativeGainTranslation_.z())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Vertical:kd\n");
    return false;
  }
  if (element->QueryDoubleAttribute("kff", &feedforwardGainTranslation_.z())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Vertical:kff\n");
    return false;
  }

  // Roll
  element = gainsHandle.FirstChild("Roll").Element();
  if (element->QueryDoubleAttribute("kp", &proportionalGainRotation_.x())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Roll:kp\n");
    return false;
  }
  if (element->QueryDoubleAttribute("kd", &derivativeGainRotation_.x())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Roll:kd\n");
    return false;
  }
  if (element->QueryDoubleAttribute("kff", &feedforwardGainRotation_.x())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Roll:kff\n");
    return false;
  }

  // Pitch
  element = gainsHandle.FirstChild("Pitch").Element();
  if (element->QueryDoubleAttribute("kp", &proportionalGainRotation_.y())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Pitch:kp\n");
    return false;
  }
  if (element->QueryDoubleAttribute("kd", &derivativeGainRotation_.y())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Pitch:kd\n");
    return false;
  }
  if (element->QueryDoubleAttribute("kff", &feedforwardGainRotation_.y())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Pitch:kff\n");
    return false;
  }

  // Yaw
  element = gainsHandle.FirstChild("Yaw").Element();
  if (element->QueryDoubleAttribute("kp", &proportionalGainRotation_.z())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Yaw:kp\n");
    return false;
  }
  if (element->QueryDoubleAttribute("kd", &derivativeGainRotation_.z())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Yaw:kd\n");
    return false;
  }
  if (element->QueryDoubleAttribute("kff", &feedforwardGainRotation_.z())!=TIXML_SUCCESS) {
    printf("Could not find VirtualModelController:Gains:Yaw:kff\n");
    return false;
  }

  isParametersLoaded_ = true;
  return true;
}

bool VirtualModelController::loadParameters()
{
  isParametersLoaded_ = false;
  // Headings
  if (node_handle_.hasParam("/balance_controller/virtual_model_controller/heading/kp")) {
    node_handle_.getParam("/balance_controller/virtual_model_controller/heading/kp", proportionalGainTranslation_.x());
  } else {
    ROS_ERROR("Did not find ROS parameter for robot state topic '/balance_controller/virtual_model_controller/heading/kp'.");
    return false;
  }
  if (node_handle_.hasParam("/balance_controller/virtual_model_controller/heading/kd")) {
    node_handle_.getParam("/balance_controller/virtual_model_controller/heading/kd", derivativeGainTranslation_.x());
  } else {
    ROS_ERROR("Did not find ROS parameter for robot state topic '/balance_controller/virtual_model_controller/heading/kd'.");
    return false;
  }
  if (node_handle_.hasParam("/balance_controller/virtual_model_controller/heading/kff")) {
    node_handle_.getParam("/balance_controller/virtual_model_controller/heading/kff", feedforwardGainTranslation_.x());
  } else {
    ROS_ERROR("Did not find ROS parameter for robot state topic '/balance_controller/virtual_model_controller/heading/kff'.");
    return false;
  }
  // Laterals
  if (node_handle_.hasParam("/balance_controller/virtual_model_controller/lateral/kp")) {
    node_handle_.getParam("/balance_controller/virtual_model_controller/lateral/kp", proportionalGainTranslation_.y());
  } else {
    ROS_ERROR("Did not find ROS parameter for robot state topic '/balance_controller/virtual_model_controller/lateral/kp'.");
    return false;
  }
  if (node_handle_.hasParam("/balance_controller/virtual_model_controller/lateral/kd")) {
    node_handle_.getParam("/balance_controller/virtual_model_controller/lateral/kd", derivativeGainTranslation_.y());
  } else {
    ROS_ERROR("Did not find ROS parameter for robot state topic '/balance_controller/virtual_model_controller/lateral/kd'.");
    return false;
  }
  if (node_handle_.hasParam("/balance_controller/virtual_model_controller/lateral/kff")) {
    node_handle_.getParam("/balance_controller/virtual_model_controller/lateral/kff", feedforwardGainTranslation_.y());
  } else {
    ROS_ERROR("Did not find ROS parameter for robot state topic '/balance_controller/virtual_model_controller/lateral/kff'.");
    return false;
  }
  // Verticals
  if (node_handle_.hasParam("/balance_controller/virtual_model_controller/vertical/kp")) {
    node_handle_.getParam("/balance_controller/virtual_model_controller/vertical/kp", proportionalGainTranslation_.z());
  } else {
    ROS_ERROR("Did not find ROS parameter for robot state topic '/balance_controller/virtual_model_controller/vertical/kp'.");
    return false;
  }
  if (node_handle_.hasParam("/balance_controller/virtual_model_controller/vertical/kd")) {
    node_handle_.getParam("/balance_controller/virtual_model_controller/vertical/kd", derivativeGainTranslation_.z());
  } else {
    ROS_ERROR("Did not find ROS parameter for robot state topic '/balance_controller/virtual_model_controller/vertical/kd'.");
    return false;
  }
  if (node_handle_.hasParam("/balance_controller/virtual_model_controller/vertical/kff")) {
    node_handle_.getParam("/balance_controller/virtual_model_controller/vertical/kff", feedforwardGainTranslation_.z());
  } else {
    ROS_ERROR("Did not find ROS parameter for robot state topic '/balance_controller/virtual_model_controller/vertical/kff'.");
    return false;
  }
  // Roll
  if (node_handle_.hasParam("/balance_controller/virtual_model_controller/roll/kp")) {
    node_handle_.getParam("/balance_controller/virtual_model_controller/roll/kp", proportionalGainRotation_.x());
  } else {
    ROS_ERROR("Did not find ROS parameter for robot state topic '/balance_controller/virtual_model_controller/roll/kp'.");
    return false;
  }
  if (node_handle_.hasParam("/balance_controller/virtual_model_controller/roll/kd")) {
    node_handle_.getParam("/balance_controller/virtual_model_controller/roll/kd", derivativeGainRotation_.x());
  } else {
    ROS_ERROR("Did not find ROS parameter for robot state topic '/balance_controller/virtual_model_controller/roll/kd'.");
    return false;
  }
  if (node_handle_.hasParam("/balance_controller/virtual_model_controller/roll/kff")) {
    node_handle_.getParam("/balance_controller/virtual_model_controller/roll/kff", feedforwardGainRotation_.x());
  } else {
    ROS_ERROR("Did not find ROS parameter for robot state topic '/balance_controller/virtual_model_controller/roll/kff'.");
    return false;
  }
  // Pitch
  if (node_handle_.hasParam("/balance_controller/virtual_model_controller/pitch/kp")) {
    node_handle_.getParam("/balance_controller/virtual_model_controller/pitch/kp", proportionalGainRotation_.y());
  } else {
    ROS_ERROR("Did not find ROS parameter for robot state topic '/balance_controller/virtual_model_controller/pitch/kp'.");
    return false;
  }
  if (node_handle_.hasParam("/balance_controller/virtual_model_controller/pitch/kd")) {
    node_handle_.getParam("/balance_controller/virtual_model_controller/pitch/kd", derivativeGainRotation_.y());
  } else {
    ROS_ERROR("Did not find ROS parameter for robot state topic '/balance_controller/virtual_model_controller/pitch/kd'.");
    return false;
  }
  if (node_handle_.hasParam("/balance_controller/virtual_model_controller/pitch/kff")) {
    node_handle_.getParam("/balance_controller/virtual_model_controller/pitch/kff", feedforwardGainRotation_.y());
  } else {
    ROS_ERROR("Did not find ROS parameter for robot state topic '/balance_controller/virtual_model_controller/pitch/kff'.");
    return false;
  }
  // Yaw
  if (node_handle_.hasParam("/balance_controller/virtual_model_controller/yaw/kp")) {
    node_handle_.getParam("/balance_controller/virtual_model_controller/yaw/kp", proportionalGainRotation_.z());
  } else {
    ROS_ERROR("Did not find ROS parameter for robot state topic '/balance_controller/virtual_model_controller/yaw/kp'.");
    return false;
  }
  if (node_handle_.hasParam("/balance_controller/virtual_model_controller/yaw/kd")) {
    node_handle_.getParam("/balance_controller/virtual_model_controller/yaw/kd", derivativeGainRotation_.z());
  } else {
    ROS_ERROR("Did not find ROS parameter for robot state topic '/balance_controller/virtual_model_controller/yaw/kd'.");
    return false;
  }
  if (node_handle_.hasParam("/balance_controller/virtual_model_controller/yaw/kff")) {
    node_handle_.getParam("/balance_controller/virtual_model_controller/yaw/kff", feedforwardGainRotation_.z());
  } else {
    ROS_ERROR("Did not find ROS parameter for robot state topic '/balance_controller/virtual_model_controller/yaw/kff'.");
    return false;
  }
  if (node_handle_.hasParam("/balance_controller/virtual_model_controller/gravity_compensation_percentage")) {
    node_handle_.getParam("/balance_controller/virtual_model_controller/gravity_compensation_percentage", gravityCompensationForcePercentage_);
  } else {
    ROS_ERROR("Did not find ROS parameter for robot state topic '/balance_controller/virtual_model_controller/gravity_compensation_percentage'.");
    return false;
  }
  isParametersLoaded_ = true;
  ROS_INFO("Parameters Loaded from Server");
  return true;
}


const Eigen::Vector3d& VirtualModelController::getProportionalGainTranslation() const {
  return proportionalGainTranslation_;
}
const Eigen::Vector3d& VirtualModelController::getDerivativeGainTranslation() const {
  return derivativeGainTranslation_;
}
const Eigen::Vector3d& VirtualModelController::getFeedforwardGainTranslation() const {
  return feedforwardGainTranslation_;
}

const Eigen::Vector3d& VirtualModelController::getProportionalGainRotation() const {
  return proportionalGainRotation_;
}
const Eigen::Vector3d& VirtualModelController::getDerivativeGainRotation() const {
  return derivativeGainRotation_;
}
const Eigen::Vector3d& VirtualModelController::getFeedforwardGainRotation() const {
  return feedforwardGainRotation_;
}

void VirtualModelController::setProportionalGainRotation(const Eigen::Vector3d& gains) {
  proportionalGainRotation_ = gains;
}
void VirtualModelController::setDerivativeGainRotation(const Eigen::Vector3d& gains) {
  derivativeGainRotation_ = gains;
}
void VirtualModelController::setFeedforwardGainRotation(const Eigen::Vector3d& gains) {
  feedforwardGainRotation_ = gains;
}

void VirtualModelController::setProportionalGainTranslation(const Eigen::Vector3d& gains) {
  proportionalGainTranslation_ = gains;
}
void VirtualModelController::setDerivativeGainTranslation(const Eigen::Vector3d& gains) {
  derivativeGainTranslation_ = gains;
}
void VirtualModelController::setFeedforwardGainTranslation(const Eigen::Vector3d& gains) {
  feedforwardGainTranslation_ = gains;
}

void VirtualModelController::setGainsHeading(double kp, double kd, double kff) {
  proportionalGainTranslation_.x() = kp;
  derivativeGainTranslation_.x() = kd;
  feedforwardGainTranslation_.x() = kff;
}
void VirtualModelController::setGainsLateral(double kp, double kd, double kff) {
  proportionalGainTranslation_.y() = kp;
  derivativeGainTranslation_.y() = kd;
  feedforwardGainTranslation_.y() = kff;
}
void VirtualModelController::setGainsVertical(double kp, double kd, double kff) {
  proportionalGainTranslation_.z() = kp;
  derivativeGainTranslation_.z() = kd;
  feedforwardGainTranslation_.z() = kff;
}
void VirtualModelController::setGainsRoll(double kp, double kd, double kff) {
  proportionalGainRotation_.x() = kp;
  derivativeGainRotation_.x() = kd;
  feedforwardGainRotation_.x() = kff;
}

void VirtualModelController::setGainsPitch(double kp, double kd, double kff) {
  proportionalGainRotation_.y() = kp;
  derivativeGainRotation_.y() = kd;
  feedforwardGainRotation_.y() = kff;
}
void VirtualModelController::setGainsYaw(double kp, double kd, double kff) {
  proportionalGainRotation_.z() = kp;
  derivativeGainRotation_.z() = kd;
  feedforwardGainRotation_.z() = kff;
}

void VirtualModelController::getGainsHeading(double& kp, double& kd, double& kff) {
  kp = proportionalGainTranslation_.x();
  kd = derivativeGainTranslation_.x();
  kff = feedforwardGainTranslation_.x();
}
void VirtualModelController::getGainsLateral(double& kp, double& kd, double& kff) {
  kp = proportionalGainTranslation_.y();
  kd = derivativeGainTranslation_.y();
  kff = feedforwardGainTranslation_.y();
}
void VirtualModelController::getGainsVertical(double& kp, double& kd, double& kff) {
  kp = proportionalGainTranslation_.z();
  kd = derivativeGainTranslation_.z();
  kff = feedforwardGainTranslation_.z();
}
void VirtualModelController::getGainsRoll(double& kp, double& kd, double& kff) {
  kp = proportionalGainRotation_.x();
  kd = derivativeGainRotation_.x();
  kff = feedforwardGainRotation_.x();
}
void VirtualModelController::getGainsPitch(double& kp, double& kd, double& kff) {
  kp = proportionalGainRotation_.y();
  kd = derivativeGainRotation_.y();
  kff = feedforwardGainRotation_.y();
}
void VirtualModelController::getGainsYaw(double& kp, double& kd, double& kff) {
  kp = proportionalGainRotation_.z();
  kd = derivativeGainRotation_.z();
  kff = feedforwardGainRotation_.z();
}

void VirtualModelController::setGravityCompensationForcePercentage(double percentage) {
  gravityCompensationForcePercentage_ = percentage;
}


double VirtualModelController::getGravityCompensationForcePercentage() const {
  return gravityCompensationForcePercentage_;
}

const ContactForceDistributionBase& VirtualModelController::getContactForceDistribution() const {
  return *contactForceDistribution_.get();
}

bool VirtualModelController::setToInterpolated(const MotionControllerBase& motionController1, const MotionControllerBase& motionController2, double t) {
  const VirtualModelController& controller1 = static_cast<const VirtualModelController& >(motionController1);
  const VirtualModelController& controller2 = static_cast<const VirtualModelController& >(motionController2);


  if (!controller1.checkIfParametersLoaded()) {
    return false;
  }

  if (!controller2.checkIfParametersLoaded()) {
    return false;
  }

  this->proportionalGainTranslation_ = linearlyInterpolate(controller1.getProportionalGainTranslation(),  controller2.getProportionalGainTranslation(), 0.0, 1.0, t);
  this->derivativeGainTranslation_ = linearlyInterpolate(controller1.getDerivativeGainTranslation(),  controller2.getDerivativeGainTranslation(), 0.0, 1.0, t);
  this->feedforwardGainTranslation_ = linearlyInterpolate(controller1.getFeedforwardGainTranslation(),  controller2.getFeedforwardGainTranslation(), 0.0, 1.0, t);

  this->proportionalGainRotation_ = linearlyInterpolate(controller1.getProportionalGainRotation(),  controller2.getProportionalGainRotation(), 0.0, 1.0, t);
  this->derivativeGainRotation_ = linearlyInterpolate(controller1.getDerivativeGainRotation(),  controller2.getDerivativeGainRotation(), 0.0, 1.0, t);
  this->feedforwardGainRotation_ = linearlyInterpolate(controller1.getFeedforwardGainRotation(),  controller2.getFeedforwardGainRotation(), 0.0, 1.0, t);


  if (!contactForceDistribution_->setToInterpolated(controller1.getContactForceDistribution(), controller2.getContactForceDistribution(),t)) {
    return false;
  }
  return true;
}

} /* namespace loco */
