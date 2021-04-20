/*
 * BatchExecutor.cpp
 *
 *  Created on: Dec 20, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_core/executor/BatchExecutor.hpp"
#include "free_gait_core/executor/State.hpp"

namespace free_gait {

BatchExecutor::BatchExecutor(free_gait::Executor& executor)
    : executor_(executor),
      timeStep_(0.01),
      isProcessing_(false),
      requestForCancelling_(false)
{
}

BatchExecutor::~BatchExecutor()
{
}

void BatchExecutor::addProcessingCallback(std::function<void(bool)> callback)
{
  callback_ = callback;
}

void BatchExecutor::setTimeStep(const double timeStep)
{
  if (isProcessing_) throw std::runtime_error("Batch executor error: Cannot change time step during processing.");
  timeStep_ = timeStep;
}

double BatchExecutor::getTimeStep() const
{
  return timeStep_;
}

bool BatchExecutor::process(const std::vector<free_gait::Step>& steps)
{
  if (isProcessing_) return false;
  isProcessing_ = true;
  executor_.reset();
  executor_.getQueue().add(steps);
  std::thread thread(std::bind(&BatchExecutor::processInThread, this));
  thread.detach();
  return true;
}

bool BatchExecutor::isProcessing()
{
  return isProcessing_;
}

void BatchExecutor::cancelProcessing()
{
  requestForCancelling_ = true;
}

const StateBatch& BatchExecutor::getStateBatch() const
{
  if (isProcessing_) throw std::runtime_error("Batch executor error: Cannot access state during processing.");
  return stateBatch_;
}

void BatchExecutor::processInThread()
{
  stateBatch_.clear();
  double time = 0.0;
  while (!executor_.getQueue().empty() && !requestForCancelling_) {
    executor_.advance(timeStep_);
    time += timeStep_;
//    std::cout<<"Updated Joint Positions: "<<std::endl<<executor_.getState().getJointPositions()<<std::endl;
    stateBatch_.addState(time, executor_.getState());
//    std::cout<<"Updated Joint Positions: "<<std::endl<<stateBatch_.getState(time).getJointPositions()<<std::endl;
  }
  requestForCancelling_ = false;
  isProcessing_ = false;
  callback_(true);
}

} /* namespace */
