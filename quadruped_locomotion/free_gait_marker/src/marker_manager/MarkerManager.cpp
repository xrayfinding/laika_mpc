/*
 * MarkerManager.cpp
 *
 *  Created on: Feb 28, 2015
 *      Author: Péter Fankhauser, Dario Bellicoso
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_marker/marker_manager/MarkerManager.hpp"

// ROS
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

// STD
#include <algorithm>
#include <stdexcept>

using namespace interactive_markers;
using namespace visualization_msgs;

static const int NUMBER_OF_LEGS = 4;

static const std::string LEFT_FORE_NAME = "LF_LEG";
static const int LEFT_FORE_ID = 0;
static const std::string RIGHT_FORE_NAME = "RF_LEG";
static const int RIGHT_FORE_ID = 3;
static const std::string LEFT_HIND_NAME = "LH_LEG";
static const int LEFT_HIND_ID = 2;
static const std::string RIGHT_HIND_NAME = "RH_LEG";
static const int RIGHT_HIND_ID = 1;

namespace free_gait_marker {

MarkerManager::MarkerManager(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      server_(ros::this_node::getName()),
      knotIds_(NUMBER_OF_LEGS, 0),
      trajectoryIds_(NUMBER_OF_LEGS, 0),
      splines_(NUMBER_OF_LEGS),
      trajectories_(NUMBER_OF_LEGS),
      showTrajectory_(NUMBER_OF_LEGS, false)
{
  loadManagerParameters();

  stepActionClient_ = std::unique_ptr<
      actionlib::SimpleActionClient<free_gait_msgs::ExecuteStepsAction>>(
      new actionlib::SimpleActionClient<free_gait_msgs::ExecuteStepsAction>(
          actionServerTopic_, true));

  setupMenus();

//  trajectoriesPublisher_ =
//      nodeHandle.advertise<visualization_msgs::MarkerArray>("trajectory_points",
//                                                            100);

  lf_trajectoriesPublisher_ =
      nodeHandle.advertise<trajectory_msgs::MultiDOFJointTrajectory>("lf_trajectory_points",
                                                            100);
  rf_trajectoriesPublisher_ =
      nodeHandle.advertise<trajectory_msgs::MultiDOFJointTrajectory>("rf_trajectory_points",
                                                            100);
  rh_trajectoriesPublisher_ =
      nodeHandle.advertise<trajectory_msgs::MultiDOFJointTrajectory>("rh_trajectory_points",
                                                            100);
  lh_trajectoriesPublisher_ =
      nodeHandle.advertise<trajectory_msgs::MultiDOFJointTrajectory>("lh_trajectory_points",
                                                            100);

  // Add an interactive marker for each foot
  addFootholdMarker(LEFT_FORE_ID, LEFT_FORE_NAME);
  addFootholdMarker(RIGHT_FORE_ID, RIGHT_FORE_NAME);
  addFootholdMarker(LEFT_HIND_ID, LEFT_HIND_NAME);
  addFootholdMarker(RIGHT_HIND_ID, RIGHT_HIND_NAME);
  applyChanges();

  // By default, the foot markers are initially deactivated
  for (auto& foothold : footholdList_)
    deactivateFoothold(foothold);
  applyChanges();
}

MarkerManager::~MarkerManager()
{
}

void MarkerManager::setupMenus()
{
  /**********************************
   * Setup foot marker context menu *
   **********************************/
  footMenuHandler_.insert(
      "Activate", boost::bind(&MarkerManager::activateCallback, this, _1));
  footMenuHandler_.insert("Reset",
                          boost::bind(&MarkerManager::resetCallback, this, _1));

  interactive_markers::MenuHandler::EntryHandle stepHandle = footMenuHandler_
        .insert("Step");
    interactive_markers::MenuHandler::EntryHandle trajHandle = footMenuHandler_
        .insert("Trajectory");

  footMenuHandler_.insert(
      stepHandle, "Send step goal",
      boost::bind(&MarkerManager::sendStepCallback, this, _1));

  footMenuHandler_.insert(
      trajHandle, "Add knot",
      boost::bind(&MarkerManager::setKnotCallback, this, _1));
  footMenuHandler_.insert(
      trajHandle, "Clear",
      boost::bind(&MarkerManager::clearTrajectoryCallback, this, _1));
  footMenuHandler_.insert(
      trajHandle, "Show",
      boost::bind(&MarkerManager::showTrajectoryCallback, this, _1));
  footMenuHandler_.insert(
      trajHandle, "Send",
      boost::bind(&MarkerManager::sendTrajectoryCallback, this, _1));
  footMenuHandler_.insert(
      trajHandle, "Clear knots",
      boost::bind(&MarkerManager::deleteKnotsCallback, this, _1));
  /**********************************/

  /**********************************
   * Setup knot marker context menu *
   **********************************/
//  knotMenuHandler_.insert(
//      "Update", boost::bind(&MarkerManager::knotMenuUpdateCallback, this, _1));
//  knotMenuHandler_.insert(
//      "Reset", boost::bind(&MarkerManager::knotMenuResetCallback, this, _1));
  knotMenuHandler_.insert(
      "Delete", boost::bind(&MarkerManager::knotMenuDeleteCallback, this, _1));
  /**********************************/
}

void MarkerManager::activateCallback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (!activateFoothold(getFootholdByMarkerName(feedback->marker_name.c_str())))
    ROS_WARN_STREAM(
        "Foothold marker with name '" << feedback->marker_name.c_str() << "' could not be activated.");
  applyChanges();
}

void MarkerManager::sendStepCallback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  sendStepGoal();
  applyChanges();
}

void MarkerManager::resetCallback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (!deactivateFoothold(
      getFootholdByMarkerName(feedback->marker_name.c_str())))
    ROS_WARN_STREAM(
        "Foothold marker with name '" << feedback->marker_name.c_str() << "' could not be deactivated.");
  applyChanges();
}

void MarkerManager::setKnotCallback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  const int legId = getFootholdByMarkerName(feedback->marker_name.c_str())
      .stepNumber;
  std::string knotMarkerName { feedback->marker_name + "_knot_"
      + std::to_string(knotIds_[legId]) };
  addKnotMarker(knotIds_[legId]++, knotMarkerName, feedback->pose);
  applyChanges();
}

void MarkerManager::sendTrajectoryCallback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{

}

void MarkerManager::clearTrajectoryCallback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  const int legId = getFootholdByMarkerName(feedback->marker_name.c_str())
      .stepNumber;
  showTrajectory_[legId] = false;
  clearTrajectory(legId);
}

void MarkerManager::showTrajectoryCallback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  const int legId = getFootholdByMarkerName(feedback->marker_name.c_str())
      .stepNumber;
  showTrajectory_[legId] = true;
  updateTrajectory(legId, feedback->marker_name);
}

void MarkerManager::knotMenuUpdateCallback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  int legId = -1;
  std::string name = "";
  getIdAndNameFromMarker(feedback, legId, name);
  updateTrajectory(legId, name);
}

void MarkerManager::deleteKnotsCallback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  int legId = -1;
  std::string name = "";
  getIdAndNameFromMarker(feedback, legId, name);

  for (size_t k = 0; k < knotIds_.size(); k++) {
    server_.erase(feedback->marker_name + "_" + "knot_" + std::to_string(k));
    applyChanges();
  }

  clearTrajectory(legId);
}

void MarkerManager::getIdAndNameFromMarker(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback,
    int& legId, std::string& name)
{
  if (feedback->marker_name.find(LEFT_FORE_NAME) != std::string::npos) {
    legId = LEFT_FORE_ID;
    name = LEFT_FORE_NAME;
  } else if (feedback->marker_name.find(RIGHT_FORE_NAME) != std::string::npos) {
    legId = RIGHT_FORE_ID;
    name = RIGHT_FORE_NAME;
  } else if (feedback->marker_name.find(LEFT_HIND_NAME) != std::string::npos) {
    legId = LEFT_HIND_ID;
    name = LEFT_HIND_NAME;
  } else if (feedback->marker_name.find(RIGHT_HIND_NAME) != std::string::npos) {
    legId = RIGHT_HIND_ID;
    name = RIGHT_HIND_NAME;
  } else {
    ROS_INFO_STREAM(
        "[MarkerManager::getIdAndNameFromMarker] Could not find id and name.");
    return;
  }
}

void MarkerManager::knotMenuResetCallback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{

}

void MarkerManager::knotMenuDeleteCallback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  server_.erase(feedback->marker_name);
  applyChanges();
  int legId = -1;
  std::string name = "";
  getIdAndNameFromMarker(feedback, legId, name);
  updateTrajectory(legId, name);
}

void MarkerManager::loadManagerParameters()
{
  nodeHandle_.param("foothold/frame_id", footholdFrameId_, std::string("map"));
  nodeHandle_.param("action_server_topic", actionServerTopic_,
                    std::string("/locomotion_controller/step"));
  double duration;
  nodeHandle_.param("wait_for_action_timeout", duration, 5.0);
  waitForActionTimeout_.fromSec(duration);
  nodeHandle_.param("foothold/frame_id", footholdFrameId_, std::string("map"));
  nodeHandle_.param("ignore_base_motion", ignoreBaseMotion_, true);
}

void MarkerManager::addFootholdMarker(const unsigned int stepNumber,
                                      const std::string& legName)
{
  markers::MarkerFoot marker;
  marker.loadParameters(nodeHandle_);
  ROS_INFO("Foot hold frame in Manager %s", footholdFrameId_.c_str());
  marker.setupFootholdMarker(stepNumber, legName);
  tf::Vector3 position(1,1,1);
  tf::pointTFToMsg(position, marker.pose.position);
  ROS_INFO("mark info : ");
  std::cout<<"name : "<<marker.name<<std::endl;
  std::string footFrameId;
  if (legName == LEFT_FORE_NAME)
    footFrameId = "LF_FOOT";//"LF_FOOT";
  else if (legName == RIGHT_FORE_NAME)
    footFrameId = "RF_FOOT";//"RF_FOOT";
  else if (legName == LEFT_HIND_NAME)
    footFrameId = "LH_FOOT";//"LH_FOOT";
  else if (legName == RIGHT_HIND_NAME)
    footFrameId = "RH_FOOT";//"RH_FOOT";
  else {
    ROS_WARN_STREAM(
        ("No corresponding frame id for the foot found for leg name `" + legName
            + "'.").c_str());
    return;
  }

  footholdList_.emplace_back(Foothold { stepNumber, legName, marker.name,
      footFrameId, true });
  server_.insert(marker,
                 boost::bind(&MarkerManager::footholdCallback, this, _1));
  footMenuHandler_.apply(server_, marker.name);

//  activateFoothold(getFootholdByMarkerName(marker.name));
}

void MarkerManager::addKnotMarker(const unsigned int markerNumber,
                                  const std::string& markerName,
                                  const geometry_msgs::Pose& pose)
{
  markers::MarkerKnot marker;
  marker.loadParameters(nodeHandle_);
  marker.setupMarker(markerNumber, markerName);

  server_.insert(marker, boost::bind(&MarkerManager::knotCallback, this, _1));
  applyChanges();

  knotMenuHandler_.apply(server_, marker.name);
  applyChanges();

  // set knot pose
  std_msgs::Header header;
  header.frame_id = marker.header.frame_id;//"map";
  header.stamp = ros::Time(0.0);
  if (!server_.setPose(markerName, pose, header)) {
    ROS_WARN_STREAM("Marker with name '" << markerName << "' not found.");
  }
  ROS_INFO("set %s ,position is (%f, %f, %f)",markerName.c_str(), pose.position.x,
           pose.position.y, pose.position.z);
  applyChanges();

}

void MarkerManager::clearTrajectory(int legId)
{
  trajectories_[legId].points.clear();
  trajectories_[legId].joint_names.clear();
  trajectories_[legId].header.stamp = ros::Time::now();
  trajectories_[legId].header.frame_id = footholdFrameId_;//"map";

  splines_[legId].clear();
  trajectoryIds_[legId] = 0;
}


void MarkerManager::updateTrajectory(int legId, const std::string& markerName)
{
  if (!showTrajectory_[legId])
    return;

  std::vector<double> tData;
  std::vector<Eigen::Matrix<double, 3, 1>> vData;
  double tf = 0.0;

  clearTrajectory(legId);

  tData.clear();
  vData.clear();

  for (size_t k = 0; k < knotIds_[legId]; k++) {
    std::string knotName = markerName + "_knot_" + std::to_string(k);
    markers::MarkerKnot marker;
    if (server_.get(knotName, marker)) {
      tData.push_back(tf);
      Eigen::Matrix<double, 3, 1> vpoint;
      vpoint << marker.pose.position.x, marker.pose.position.y, marker.pose
          .position.z;
      vData.push_back(vpoint);
      // todo: set tf appropriately
      tf += 1.0;
    }
  }

  // at least two knots are needed to fit a trajectory
  if (tData.size() > 1) {

    splines_.at(legId).fitCurve(tData, vData);
    double dt = 0.1;

    for (int k = 0; k * dt < splines_.at(legId).getMaxTime(); k++) {
      trajectory_msgs::MultiDOFJointTrajectoryPoint point;
      geometry_msgs::Transform transform;
      geometry_msgs::Twist twist_vel;
      geometry_msgs::Twist twist_acc;

      Eigen::Vector3d position;
      splines_.at(legId).evaluate(position, k*dt);
      transform.translation.x = position.x();
      transform.translation.y = position.y();
      transform.translation.z = position.z();

      Eigen::Vector3d linearVelocity;
      splines_.at(legId).evaluateDerivative(linearVelocity, k*dt,1);
      twist_vel.linear.x = linearVelocity.x();
      twist_vel.linear.y = linearVelocity.y();
      twist_vel.linear.z = linearVelocity.z();

      ROS_INFO_STREAM("vel: " << twist_vel.linear.x);

      Eigen::Vector3d linearAcc;
      splines_.at(legId).evaluateDerivative(linearAcc, k*dt,2);
      twist_acc.linear.x = linearAcc.x();
      twist_acc.linear.y = linearAcc.y();
      twist_acc.linear.z = linearAcc.z();


      point.transforms.push_back(transform);
      point.velocities.push_back(twist_vel);
      point.accelerations.push_back(twist_acc);

      point.time_from_start = ros::Duration(k*dt);

      trajectories_.at(legId).points.push_back(point);

      trajectories_.at(legId).header.stamp = ros::Time::now();
      trajectories_.at(legId).header.frame_id = footholdFrameId_;//"map";
      trajectories_.at(legId).joint_names.push_back(std::to_string(k));

    }
  }

}


void MarkerManager::footholdCallback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
//  Foothold& foothold(getFootholdByMarkerName(feedback->marker_name.c_str()));
//  if (foothold.isActive) return;
//  if (!activateFoothold(foothold))
////     ROS_WARN_STREAM("Foothold marker with name `%s` could not be activated.", feedback->marker_name.c_str());
//  applyChanges();
}

void MarkerManager::knotCallback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  int legId = -1;
  std::string name = "";
  getIdAndNameFromMarker(feedback, legId, name);
  updateTrajectory(legId, name);
  applyChanges();
}

bool MarkerManager::activateFoothold(Foothold& foothold)
{
  if (foothold.isActive)
    return true;
  // Get pose.
  markers::MarkerFoot marker;
  if (!getMarkerFromFoothold(foothold, marker))
    return false;
  tf::Stamped<tf::Point> positionInPassiveFrame;  // Foot frame.
  tf::Stamped<tf::Point> positionInActiveFrame;  // Map frame.
  tf::pointMsgToTF(marker.pose.position, positionInPassiveFrame);
  positionInPassiveFrame.frame_id_ = foothold.footFrameId;
  positionInPassiveFrame.stamp_ = ros::Time(0.0);
  transformListener_.transformPoint(footholdFrameId_, positionInPassiveFrame,
                                    positionInActiveFrame);
  geometry_msgs::Pose pose;
  tf::pointTFToMsg(positionInActiveFrame, pose.position);
  std::cout<<"positionInPassiveFrame : "<<positionInPassiveFrame<<"positionInActiveFrame"<<positionInActiveFrame<<std::endl;
  // Get header.
  std_msgs::Header header;
  header.frame_id = positionInActiveFrame.frame_id_;
  header.stamp = positionInActiveFrame.stamp_;

  // Update marker pose.
  if (!server_.setPose(foothold.markerName, pose, header)) {
    ROS_WARN_STREAM(
        "Marker with name '" << foothold.markerName.c_str() << "' not found.");
    return false;
  }

  foothold.isActive = true;
  return true;
}

bool MarkerManager::deactivateFoothold(Foothold& foothold)
{
  std_msgs::Header header;
  header.frame_id = foothold.footFrameId;
  header.stamp = ros::Time(0.0);
  if (!server_.setPose(foothold.markerName, geometry_msgs::Pose(), header)) {
    ROS_WARN_STREAM(
        "Marker with name '" << foothold.markerName.c_str() << "' not found.");
    return false;
  }

  foothold.isActive = false;
  return true;
}

bool MarkerManager::deactivateAllFootholds()
{
  for (auto& foothold : footholdList_) {
    if (!deactivateFoothold(foothold))
      return false;
  }
  return true;
}

void MarkerManager::applyChanges()
{
  server_.applyChanges();
}

MarkerManager::Foothold& MarkerManager::getFootholdByMarkerName(
    const std::string& markerName)
{
  for (auto& foothold : footholdList_) {
    if (foothold.markerName == markerName) {
      return foothold;
    }
  }
  throw std::out_of_range(
      "Foothold with marker name " + markerName + " not found.");
}

bool MarkerManager::getMarkerFromFoothold(
    const MarkerManager::Foothold& foothold, markers::MarkerFoot& marker)
{
  if (!server_.get(foothold.markerName, marker)) {
    ROS_WARN_STREAM(
        "Marker with name '" << foothold.markerName.c_str() << "' not found.");
    return false;
  }
  return true;
}

bool MarkerManager::sendStepGoal()
{
  if (!stepActionClient_->isServerConnected()) {
    ROS_INFO("Waiting for step action server to start.");
    if (!stepActionClient_->waitForServer(waitForActionTimeout_)) {
      ROS_WARN_STREAM("No step action server found, ignoring action.");
      return false;
    }
  }

  std::sort(footholdList_.begin(), footholdList_.end());
  free_gait_msgs::ExecuteStepsGoal goal;

  for (const auto& foothold : footholdList_) {
    markers::MarkerFoot marker;
    if (!getMarkerFromFoothold(foothold, marker)) {
      ROS_WARN_STREAM("Goal not sent.");
      return false;
    }

    if (!foothold.isActive)
      continue;
    free_gait_msgs::Step preStep;
    free_gait_msgs::BaseAuto baseMotion;
    baseMotion.height = 0.45;
    baseMotion.support_margin = 0.07;
    baseMotion.average_linear_velocity = 0.1;
    baseMotion.average_angular_velocity = 0.2;
    preStep.base_auto.push_back(baseMotion);
    if(!ignoreBaseMotion_)
      goal.steps.push_back(preStep);

    free_gait_msgs::Step step;
    free_gait_msgs::Footstep footstep;
    footstep.name = foothold.legName;
    footstep.target.header.frame_id = footholdFrameId_;
    footstep.target.point = marker.pose.position;
    footstep.profile_type = "straight";
//    footstep.profile_height = 0.15;
    footstep.average_velocity = 0.3;
//    footstep.ignore_for_pose_adaptation = true;
//    footstep.ignore_contact = true;
//    swingData.profile.type = "square";
    step.footstep.push_back(footstep);
    goal.steps.push_back(step);
    ROS_INFO("send %s footstep target of (%f, %f, %f)",foothold.legName.c_str(), marker.pose.position.x,
             marker.pose.position.y, marker.pose.position.z);
  }

  stepActionClient_->sendGoal(goal);
  stepActionClient_->waitForResult();
  deactivateAllFootholds();
//  actionlib::SimpleClientGoalState state = ac.getState();

  return true;
}

void MarkerManager::publishKnots()
{
//  for (auto& msg : trajectories_) {
//    visualization_msgs::MarkerArrayPtr msgPtr(
//        new visualization_msgs::MarkerArray(msg));
//    trajectoriesPublisher_.publish(
//        visualization_msgs::MarkerArrayConstPtr(msgPtr));
//  }

//  for (auto& msg : trajectories_) {
//    trajectory_msgs::MultiDOFJointTrajectoryPtr msgPtr(
//        new trajectory_msgs::MultiDOFJointTrajectory(msg));
//    trajectoriesPublisher_.publish(
//        trajectory_msgs::MultiDOFJointTrajectoryConstPtr(msgPtr));
//  }

    trajectory_msgs::MultiDOFJointTrajectoryPtr lf_msgPtr(
        new trajectory_msgs::MultiDOFJointTrajectory(trajectories_.at(LEFT_FORE_ID)));
    lf_trajectoriesPublisher_.publish(
        trajectory_msgs::MultiDOFJointTrajectoryConstPtr(lf_msgPtr));

    trajectory_msgs::MultiDOFJointTrajectoryPtr rf_msgPtr(
        new trajectory_msgs::MultiDOFJointTrajectory(trajectories_.at(RIGHT_FORE_ID)));
    rf_trajectoriesPublisher_.publish(
        trajectory_msgs::MultiDOFJointTrajectoryConstPtr(lf_msgPtr));

    trajectory_msgs::MultiDOFJointTrajectoryPtr rh_msgPtr(
        new trajectory_msgs::MultiDOFJointTrajectory(trajectories_.at(RIGHT_HIND_ID)));
    rh_trajectoriesPublisher_.publish(
        trajectory_msgs::MultiDOFJointTrajectoryConstPtr(rh_msgPtr));

    trajectory_msgs::MultiDOFJointTrajectoryPtr lh_msgPtr(
        new trajectory_msgs::MultiDOFJointTrajectory(trajectories_.at(LEFT_HIND_ID)));
    lh_trajectoriesPublisher_.publish(
        trajectory_msgs::MultiDOFJointTrajectoryConstPtr(lh_msgPtr));
}

void MarkerManager::updateKnots() {
  if (showTrajectory_[LEFT_FORE_ID]) {
    updateTrajectory(LEFT_FORE_ID, LEFT_FORE_NAME);
  }
  if (showTrajectory_[RIGHT_FORE_ID]) {
    updateTrajectory(RIGHT_FORE_ID, RIGHT_FORE_NAME);
  }
  if (showTrajectory_[LEFT_HIND_ID]) {
    updateTrajectory(LEFT_HIND_ID, LEFT_HIND_NAME);
  }
  if (showTrajectory_[RIGHT_HIND_ID]) {
    updateTrajectory(RIGHT_HIND_ID, RIGHT_HIND_NAME);
  }
}

void MarkerManager::print()
{
  ROS_INFO_STREAM("Foothold List:" << std::endl);
  for (const auto& foothold : footholdList_) {
    ROS_INFO_STREAM("Step Nr.: " << foothold.stepNumber);
    ROS_INFO_STREAM("Leg Name: " << foothold.legName);
    ROS_INFO_STREAM("Marker Name: " << foothold.markerName);
    ROS_INFO_STREAM("Foot Frame ID: " << foothold.footFrameId);
    ROS_INFO_STREAM(
        "Is Active: " << (foothold.isActive ? "Yes" : "No") << std::endl);
  }
}

} /* namespace free_gait_marker */
