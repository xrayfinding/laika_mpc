/*
 * PoseOptimizationSQP.cpp
 *
 *  Created on: Jun 10, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich
 */

#include "qp_solver/pose_optimization/PoseOptimizationSQP.hpp"
#include "qp_solver/pose_optimization/poseparameterization.h"
#include "qp_solver/pose_optimization/PoseOptimizationProblem.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <kindr/Core>
//#include <message_logger/message_logger.hpp>

//#include <numopt_quadprog/ActiveSetFunctionMinimizer.hpp>
//#include <numopt_sqp/SQPFunctionMinimizer.hpp>

#include <functional>

namespace free_gait {

PoseOptimizationSQP::PoseOptimizationSQP(const AdapterBase& adapter)
    : PoseOptimizationBase(adapter),
      timer_("PoseOptimizationSQP"),
      durationInCallback_(0.0),
      nIterations_(0)
{
  objective_.reset(new PoseOptimizationObjectiveFunction());

  constraints_.reset(new PoseOptimizationFunctionConstraints());
  PoseOptimizationFunctionConstraints::LegPositions positionsBaseToHipInBaseFrame;
  for (const auto& limb : adapter_.getLimbs()) {
    positionsBaseToHipInBaseFrame[limb] = adapter_.getPositionBaseToHipInBaseFrame(limb);
  }

  constraints_->setPositionsBaseToHip(positionsBaseToHipInBaseFrame);

  timer_.setAlpha(1.0);
}

PoseOptimizationSQP::~PoseOptimizationSQP()
{
}

void PoseOptimizationSQP::setCurrentState(const State& state)
{
  originalState_ = state;
}

void PoseOptimizationSQP::registerOptimizationStepCallback(OptimizationStepCallbackFunction callback)
{
  optimizationStepCallback_ = callback;
}

bool PoseOptimizationSQP::optimize(Pose& pose)
{
  timer_.pinTime("total");
  durationInCallback_ = 0.0;
  state_ = originalState_;
  checkSupportRegion();

  objective_->setInitialPose(pose);
  objective_->setStance(stance_);
  objective_->setNominalStance(nominalStanceInBaseFrame_);
  objective_->setSupportRegion(supportRegion_);

  constraints_->setStance(stance_);
  constraints_->setSupportRegion(supportRegion_);
  constraints_->setLimbLengthConstraints(minLimbLenghts_, maxLimbLenghts_);

  state_.setPoseBaseToWorld(pose);
  adapter_.setInternalDataFromState(state_);
//  adapter_.setInternalDataFromState(state_, false, true, false, false); // To guide IK.
//  updateJointPositionsInState(state_); // For CoM calculation.
//  adapter_.setInternalDataFromState(state_);
//  adapter_.setInternalDataFromState(state_, false, true, false, false);
  const Position centerOfMassInBaseFrame(adapter_.transformPosition(adapter_.getWorldFrameId(), adapter_.getBaseFrameId(),
                                 adapter_.getCenterOfMassInWorldFrame()));
  objective_->setCenterOfMass(centerOfMassInBaseFrame);
  constraints_->setCenterOfMass(centerOfMassInBaseFrame);
  callExternalOptimizationStepCallback(0);

  // Optimize.

  PoseOptimizationProblem problem(objective_, constraints_);
//  std::shared_ptr<numopt_common::QuadraticProblemSolver> qpSolver(
//      new numopt_quadprog::ActiveSetFunctionMinimizer);
//  numopt_sqp::SQPFunctionMinimizer solver(qpSolver, 30, 0.01, 3, -DBL_MAX);
//  solver.registerOptimizationStepCallback(
//      std::bind(&PoseOptimizationSQP::optimizationStepCallback, this, std::placeholders::_1, std::placeholders::_2,
//                std::placeholders::_3, std::placeholders::_4));
//  solver.setCheckConstraints(false);
//  solver.setPrintOutput(false);
  auto quadratic_solver = std::shared_ptr<qp_solver::QuadraticProblemSolver>(new QuadraticProblemSolver());
  sqp_solver::SequenceQuadraticProblemSolver solver(quadratic_solver, 0.05, 30);
  PoseParameterization params;
  params.setPose(pose);
//  double functionValue;
//  if (!solver.minimize(&problem, params, functionValue)) return false;
  if(!solver.minimize(problem, params)) return false;
  pose = params.getPose();

  // TODO Fix unit quaternion?

  timer_.splitTime("total");
  return true;
}

void PoseOptimizationSQP::optimizationStepCallback(const size_t iterationStep,
                                                   const PoseParameterization& parameters,
                                                   const double functionValue,
                                                   const bool finalIteration)
{
  nIterations_ = iterationStep;
  auto& poseParameterization = dynamic_cast<const PoseParameterization&>(parameters);

  // Update center of mass. // TODO Make optional.
//  state_.setPoseBaseToWorld(poseParameterization.getPose());
//  state_.setAllJointPositions(originalState_.getJointPositions());
//  adapter_.setInternalDataFromState(state_, false, true, false, false);
//  updateJointPositionsInState(state_);
//  adapter_.setInternalDataFromState(state_, false, true, false, false); // TODO Improve efficiency.
//  const Position centerOfMassInBaseFrame(adapter_.transformPosition(adapter_.getWorldFrameId(), adapter_.getBaseFrameId(),
//                                 adapter_.getCenterOfMassInWorldFrame()));
//  objective_->setCenterOfMass(centerOfMassInBaseFrame);
//  constraints_->setCenterOfMass(centerOfMassInBaseFrame);

  if (optimizationStepCallback_) {
    timer_.pinTime("callback");
    auto& poseParameterization = dynamic_cast<const PoseParameterization&>(parameters);
    state_.setPoseBaseToWorld(poseParameterization.getPose());
    timer_.splitTime("callback");
    durationInCallback_ += timer_.getAverageElapsedTimeUSec("callback");
  }

  callExternalOptimizationStepCallback(iterationStep + 1, functionValue, finalIteration);
}

void PoseOptimizationSQP::callExternalOptimizationStepCallbackWithPose(const Pose& pose, const size_t iterationStep,
                                                                       const double functionValue,
                                                                       const bool finalIteration)
{
  state_ = originalState_;
  state_.setPoseBaseToWorld(pose);
  callExternalOptimizationStepCallback(iterationStep, functionValue, finalIteration);
}

double PoseOptimizationSQP::getOptimizationDuration() const
{
  return timer_.getAverageElapsedTimeUSec("total") - durationInCallback_;
}

size_t PoseOptimizationSQP::getNumberOfIterations() const
{
  return nIterations_;
}

void PoseOptimizationSQP::callExternalOptimizationStepCallback(const size_t iterationStep, const double functionValue,
                                                               const bool finalIteration)
{
  if (optimizationStepCallback_) {
    timer_.pinTime("callback");
//    adapter_.setInternalDataFromState(state_, false, true, false, false);
    adapter_.setInternalDataFromState(state_);
    State previewState(state_);
    updateJointPositionsInState(previewState);
    optimizationStepCallback_(iterationStep, previewState, functionValue, finalIteration);
    timer_.splitTime("callback");
    durationInCallback_ += timer_.getAverageElapsedTimeUSec("callback");
  }
}

} /* namespace */
