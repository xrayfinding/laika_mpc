/******************************************************************************
 * Copyright 2017 Samuel Bachmann                                             *
 *                                                                            *
 * Redistribution and use in source and binary forms, with or without         *
 * modification, are permitted provided that the following conditions are met:*
 *                                                                            *
 * 1. Redistributions of source code must retain the above copyright notice,  *
 * this list of conditions and the following disclaimer.                      *
 *                                                                            *
 * 2. Redistributions in binary form must reproduce the above copyright       *
 * notice, this list of conditions and the following disclaimer in the        *
 * documentation and/or other materials provided with the distribution.       *
 *                                                                            *
 * 3. Neither the name of the copyright holder nor the names of its           *
 * contributors may be used to endorse or promote products derived from this  *
 * software without specific prior written permission.                        *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"*
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE  *
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE *
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE  *
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR        *
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF       *
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS   *
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN    *
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)    *
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF     *
 * THE POSSIBILITY OF SUCH DAMAGE.                                            *
 *                                                                            *
 * Author: Samuel Bachmann <samuel.bachmann@gmail.com>                        *
 ******************************************************************************/

#include "rqt_free_gait_monitor/FreeGaitMonitorPlugin.h"

#include <pluginlib/class_list_macros.h>

namespace rqt_free_gait_monitor {

/*****************************************************************************/
/** Constructor/Destructor                                                  **/
/*****************************************************************************/

FreeGaitMonitorPlugin::FreeGaitMonitorPlugin()
    : rqt_gui_cpp::Plugin(), widget_(0), descriptions_(1000) {

  setObjectName("FreeGaitMonitorPlugin");

  qRegisterMetaType<free_gait_msgs::ExecuteStepsActionGoal>
      ("free_gait_msgs::ExecuteStepsActionGoal");
  qRegisterMetaType<free_gait_msgs::ExecuteStepsActionFeedback>
      ("free_gait_msgs::ExecuteStepsActionFeedback");
  qRegisterMetaType<free_gait_msgs::ExecuteStepsActionResult>
      ("free_gait_msgs::ExecuteStepsActionResult");
  qRegisterMetaType<std_srvs::SetBoolResponse>
      ("std_srvs::SetBoolResponse");
  qRegisterMetaType<std_srvs::TriggerResponse>
      ("std_srvs::TriggerResponse");
}

/*****************************************************************************/
/** Initialization/Shutdown                                                 **/
/*****************************************************************************/

void FreeGaitMonitorPlugin::initPlugin(qt_gui_cpp::PluginContext &context) {
  widget_ = new QWidget();
  ui_.setupUi(widget_);
  if (context.serialNumber() > 1) {
    widget_->setWindowTitle(widget_->windowTitle() +
        " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  // Initialize subscribers.
  goalSubscriber_ = getNodeHandle().subscribe<
      free_gait_msgs::ExecuteStepsActionGoal>(
      getNodeHandle().param<std::string>("/free_gait/action_server", "") +
          "/goal", 10, &FreeGaitMonitorPlugin::goalCallback, this);
  feedbackSubscriber_ = getNodeHandle().subscribe<
      free_gait_msgs::ExecuteStepsActionFeedback>(
      getNodeHandle().param<std::string>("/free_gait/action_server", "") +
          "/feedback", 10, &FreeGaitMonitorPlugin::feedbackCallback, this);
  resultSubscriber_ = getNodeHandle().subscribe<
      free_gait_msgs::ExecuteStepsActionResult>(
      getNodeHandle().param<std::string>("/free_gait/action_server", "") +
          "/result", 10, &FreeGaitMonitorPlugin::resultCallback, this);

  // Initialize service clients.
  pauseClient_ = getNodeHandle().serviceClient<std_srvs::SetBool>(
      getNodeHandle().param<std::string>(
          "/free_gait/pause_execution_service", ""), false);

  stopClient_ = getNodeHandle().serviceClient<std_srvs::Trigger>(
      getNodeHandle().param<std::string>(
          "/free_gait/stop_execution_service", ""), false);

  // Initialize progress bar.
  ui_.progressBarAll->setMinimum(0);
  ui_.progressBarAll->setMaximum(1);
  ui_.progressBarAll->setValue(1);
  ui_.progressBarAll->setFormat("");

  ui_.progressBarStep->setMinimum(0);
  ui_.progressBarStep->setMaximum(1);
  ui_.progressBarStep->setValue(1);
  ui_.progressBarStep->setFormat("");

  // Set icon.
  ui_.labelStatus->setPixmap(QPixmap(":/icons/16x16/done.svg"));

  // Initialize buttons.
  ui_.pushButtonPlay->setEnabled(false);
  ui_.pushButtonPause->setEnabled(false);
  ui_.pushButtonStop->setEnabled(false);

  // Initialize navigation buttons.
  updateNavigationButtonStates();
  ui_.widgetScrollArea->installEventFilter(this);
  ui_.pushButtonGoTop->installEventFilter(this);
  ui_.pushButtonGoUp->installEventFilter(this);
  ui_.pushButtonGoDown->installEventFilter(this);
  ui_.pushButtonGoBottom->installEventFilter(this);

  // Initialize debug info.
  collapseDebug();

  // Connect signals and slots.
  connect(this,
          SIGNAL(updateGoalSignal(free_gait_msgs::ExecuteStepsActionGoal)),
          this,
          SLOT(updateGoal(free_gait_msgs::ExecuteStepsActionGoal)));
  connect(this,
          SIGNAL(updateFeedbackSignal(
              free_gait_msgs::ExecuteStepsActionFeedback)),
          this,
          SLOT(updateFeedback(free_gait_msgs::ExecuteStepsActionFeedback)));
  connect(this,
          SIGNAL(updateResultSignal(free_gait_msgs::ExecuteStepsActionResult)),
          this,
          SLOT(updateResult(free_gait_msgs::ExecuteStepsActionResult)));

  connect(ui_.pushButtonGoTop, SIGNAL(clicked()),
          this, SLOT(onPushButtonGoTop()));
  connect(ui_.pushButtonGoUp, SIGNAL(clicked()),
          this, SLOT(onPushButtonGoUp()));
  connect(ui_.pushButtonGoDown, SIGNAL(clicked()),
          this, SLOT(onPushButtonGoDown()));
  connect(ui_.pushButtonGoBottom, SIGNAL(clicked()),
          this, SLOT(onPushButtonGoBottom()));

  connect(ui_.pushButtonPlay, SIGNAL(clicked()),
          this, SLOT(onPushButtonPlay()));
  connect(ui_.pushButtonPause, SIGNAL(clicked()),
          this, SLOT(onPushButtonPause()));
  connect(ui_.pushButtonStop, SIGNAL(clicked()),
          this, SLOT(onPushButtonStop()));

  connect(ui_.clickableLabelExpandCollapse, SIGNAL(clicked()),
          this, SLOT(onClickableLabelExpandCollapse()));
  connect(ui_.pushButtonDeleteHistory, SIGNAL(clicked()),
          this, SLOT(onPushButtonDeleteHistory()));
}

void FreeGaitMonitorPlugin::shutdownPlugin() {
  goalSubscriber_.shutdown();
  feedbackSubscriber_.shutdown();
  resultSubscriber_.shutdown();
}

/*****************************************************************************/
/** Settings                                                                **/
/*****************************************************************************/

void FreeGaitMonitorPlugin::saveSettings(
    qt_gui_cpp::Settings &plugin_settings,
    qt_gui_cpp::Settings &instance_settings) const {
}

void FreeGaitMonitorPlugin::restoreSettings(
    const qt_gui_cpp::Settings &plugin_settings,
    const qt_gui_cpp::Settings &instance_settings) {
}

/*****************************************************************************/
/** Callbacks                                                               **/
/*****************************************************************************/

void FreeGaitMonitorPlugin::goalCallback(
    const free_gait_msgs::ExecuteStepsActionGoalConstPtr &goal) {
  emit updateGoalSignal(*goal);
}

void FreeGaitMonitorPlugin::feedbackCallback(
    const free_gait_msgs::ExecuteStepsActionFeedbackConstPtr &feedback) {
  emit updateFeedbackSignal(*feedback);
}

void FreeGaitMonitorPlugin::resultCallback(
    const free_gait_msgs::ExecuteStepsActionResultConstPtr &result) {
  emit updateResultSignal(*result);
}

/*****************************************************************************/
/** Methods                                                                 **/
/*****************************************************************************/

void FreeGaitMonitorPlugin::updateNavigationButtonStates() {
  if (descriptions_.size() > 0) {
    ui_.labelStepNumber->setText(QString::number(descriptions_.index() + 1));
    ui_.labelStepMax->setText(QString::number(descriptions_.size()));

    ui_.pushButtonDeleteHistory->setEnabled(true);
  } else {
    ui_.labelStepNumber->setText("...");
    ui_.labelStepMax->setText("...");

    ui_.pushButtonDeleteHistory->setEnabled(false);
  }

  if (isOnBottom_) {
    ui_.pushButtonGoBottom->setEnabled(false);
    ui_.pushButtonGoDown->setEnabled(false);
    ui_.pushButtonGoUp->setEnabled(descriptions_.size() > 1);
    ui_.pushButtonGoTop->setEnabled(descriptions_.size() > 1);
    return;
  }

  ui_.pushButtonGoBottom->setEnabled(true);
  ui_.pushButtonGoDown->setEnabled(
      descriptions_.size() > 1 && descriptions_.index() !=
                                      descriptions_.size()-1);
  ui_.pushButtonGoUp->setEnabled(
      descriptions_.size() > 1 && descriptions_.index() != 0);
  ui_.pushButtonGoTop->setEnabled(
      descriptions_.size() > 1 && descriptions_.index() != 0);
}

void FreeGaitMonitorPlugin::expandDebug() {
  ui_.plainTextEditDescription->show();
  ui_.widgetScrollArea->show();
}

void FreeGaitMonitorPlugin::collapseDebug() {
  ui_.plainTextEditDescription->hide();
  ui_.widgetScrollArea->hide();
}

/*****************************************************************************/
/** Events                                                                  **/
/*****************************************************************************/

bool FreeGaitMonitorPlugin::eventFilter(QObject *object, QEvent *event) {
  if (event->type() == QEvent::Wheel) {
    QWheelEvent *wheelEvent = static_cast<QWheelEvent *>(event);
    int numDegrees = wheelEvent->delta() / 8;
    int numSteps = numDegrees / 15;

    descriptions_.moveIndex(-numSteps);
    ui_.plainTextEditDescription->setText(descriptions_.currentQString());
    isOnBottom_ = false;
    updateNavigationButtonStates();

    wheelEvent->accept();
  }
  return QObject::eventFilter(object, event);
}

/*****************************************************************************/
/** Slots                                                                   **/
/*****************************************************************************/

void FreeGaitMonitorPlugin::updateGoal(free_gait_msgs::ExecuteStepsActionGoal goal) {
  // get goal time stamp
  currentGoalStamp_ = goal.goal_id.stamp;

  // init progress bar
  int totalSteps = (int)goal.goal.steps.size();
  ui_.progressBarAll->setMinimum(0);
  ui_.progressBarAll->setMaximum(
      (int)(progressBarMultiplicator_ * totalSteps));
  ui_.progressBarAll->setValue(0);
  std::stringstream progressBarTextFreeGait;
  progressBarTextFreeGait << 0 << "/" << totalSteps << " steps";
  ui_.progressBarAll->setFormat(QString::fromStdString(
      progressBarTextFreeGait.str()));

  ui_.progressBarStep->setMinimum(0);
  ui_.progressBarStep->setMaximum(1);
  ui_.progressBarStep->setValue(0);
  ui_.progressBarStep->setFormat("");

  // reset text
  updateNavigationButtonStates();

  ui_.plainTextEditDescription->setText(" ");

  // pause/play button
  ui_.pushButtonStop->setEnabled(true);
  ui_.pushButtonPause->setEnabled(true);
  ui_.pushButtonPlay->setEnabled(false);

  // update status
  ui_.labelStatus->setPixmap(QPixmap(":/icons/16x16/play.svg"));
}

void FreeGaitMonitorPlugin::updateFeedback(
    free_gait_msgs::ExecuteStepsActionFeedback feedback) {
  // update progress bar
  double totalSteps = (double)feedback.feedback.number_of_steps_in_goal;
  double stepNumber = (double)feedback.feedback.step_number;
  if (stepNumber < 0.0) {
    stepNumber = 0.0;
  }
  double freeGaitProgress = stepNumber + feedback.feedback.phase;
  ui_.progressBarAll->setMaximum((int)(progressBarMultiplicator_ * totalSteps));
  ui_.progressBarAll->setValue((int)(
      progressBarMultiplicator_ * freeGaitProgress));
  std::stringstream progressBarTextFreeGait;
  progressBarTextFreeGait << (int)stepNumber << "/"
                          << (int)totalSteps << " steps";
  ui_.progressBarAll->setFormat(QString::fromStdString(
      progressBarTextFreeGait.str()));

  double stepProgress = feedback.feedback.phase *
      feedback.feedback.duration.toSec();
  double stepMaximum = feedback.feedback.duration.toSec();
  ui_.progressBarStep->setMinimum(0);
  ui_.progressBarStep->setMaximum((int)(
      progressBarMultiplicator_ * stepMaximum));
  ui_.progressBarStep->setValue((int)(
      progressBarMultiplicator_ * stepProgress));
  std::stringstream progressBarText;
  progressBarText << std::fixed << std::setprecision(2);
  progressBarText << stepProgress << "/" << stepMaximum << " s";
  ui_.progressBarStep->setFormat(QString::fromStdString(progressBarText.str()));

  // update text
  if (!feedback.feedback.description.empty()) {
    description_t description;
    description.message = QString::fromStdString(
        feedback.feedback.description);
    boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
    std::string timestamp = boost::posix_time::to_simple_string(now);
    description.timestamp = QString::fromStdString(timestamp);
    descriptions_.push_back(description);

    if (isOnBottom_) {
      descriptions_.moveIndexBack();
      ui_.plainTextEditDescription->setText(descriptions_.backQString());
    } else {
      ui_.plainTextEditDescription->setText(descriptions_.currentQString());
    }
  }
  updateNavigationButtonStates();

  // update legs
  QString activeBranches = "";
  for (auto active_branch : feedback.feedback.active_branches) {
    activeBranches += QString::fromStdString(active_branch) + "  ";
  }
  ui_.labelActiveBranches->setText(activeBranches);

  // update status
  switch (feedback.feedback.status) {
    case free_gait_msgs::ExecuteStepsFeedback::PROGRESS_PAUSED:
      ui_.labelStatus->setPixmap(QPixmap(":/icons/16x16/pause.svg"));
      break;
    case free_gait_msgs::ExecuteStepsFeedback::PROGRESS_EXECUTING:
      ui_.labelStatus->setPixmap(QPixmap(":/icons/16x16/play.svg"));
      break;
    case free_gait_msgs::ExecuteStepsFeedback::PROGRESS_UNKNOWN:
      ui_.labelStatus->setPixmap(QPixmap(":/icons/16x16/unknown.svg"));
      break;
    default:
      ui_.labelStatus->setPixmap(QPixmap(":/icons/16x16/warning.svg"));
      break;
  }
}

void FreeGaitMonitorPlugin::updateResult(
    free_gait_msgs::ExecuteStepsActionResult result) {
  // reset progress bar
  ui_.progressBarAll->setMinimum(0);
  ui_.progressBarAll->setMaximum(1);
  ui_.progressBarAll->setValue(1);
  ui_.progressBarAll->setFormat("");

  ui_.progressBarStep->setMinimum(0);
  ui_.progressBarStep->setMaximum(1);
  ui_.progressBarStep->setValue(1);
  ui_.progressBarStep->setFormat("");

  // reset legs
  ui_.labelActiveBranches->setText("...");

  // reset status
  bool setButtons = true;
  switch (result.status.status) {
    case actionlib_msgs::GoalStatus::SUCCEEDED:
    case actionlib_msgs::GoalStatus::PREEMPTED:
    case actionlib_msgs::GoalStatus::RECALLED:
      ui_.labelStatus->setPixmap(QPixmap(":/icons/16x16/done.svg"));
      break;
    case actionlib_msgs::GoalStatus::ABORTED:
    case actionlib_msgs::GoalStatus::REJECTED:
      ui_.labelStatus->setPixmap(QPixmap(":/icons/16x16/failed.svg"));
      break;
    case actionlib_msgs::GoalStatus::LOST:
      setButtons = false;
      ui_.labelStatus->setPixmap(QPixmap(":/icons/16x16/unknown.svg"));
      break;
    default:
      ui_.labelStatus->setPixmap(QPixmap(":/icons/16x16/warning.svg"));
      break;
  }

  // if result from the current goal
  if (setButtons && result.status.goal_id.stamp == currentGoalStamp_) {
    // pause/play button
    ui_.pushButtonStop->setEnabled(false);
    ui_.pushButtonPause->setEnabled(false);
    ui_.pushButtonPlay->setEnabled(false);
  }
}

void FreeGaitMonitorPlugin::onPushButtonGoTop() {
  descriptions_.moveIndexFront();
  ui_.plainTextEditDescription->setText(descriptions_.currentQString());
  isOnBottom_ = false;

  updateNavigationButtonStates();
}

void FreeGaitMonitorPlugin::onPushButtonGoUp() {
  descriptions_.moveIndex(-1);
  ui_.plainTextEditDescription->setText(descriptions_.currentQString());
  isOnBottom_ = false;

  updateNavigationButtonStates();
}

void FreeGaitMonitorPlugin::onPushButtonGoDown() {
  descriptions_.moveIndex(1);
  ui_.plainTextEditDescription->setText(descriptions_.currentQString());
  isOnBottom_ = false;

  updateNavigationButtonStates();
}

void FreeGaitMonitorPlugin::onPushButtonGoBottom() {
  descriptions_.moveIndexBack();
  ui_.plainTextEditDescription->setText(descriptions_.backQString());
  isOnBottom_ = true;

  updateNavigationButtonStates();
}

void FreeGaitMonitorPlugin::onPushButtonPlay() {
  ui_.pushButtonPause->setEnabled(false);
  ui_.pushButtonPlay->setEnabled(false);

  std_srvs::SetBoolRequest request;
  request.data = false;

  WorkerThreadPausePlay *workerThreadPausePlay = new WorkerThreadPausePlay;
  connect(workerThreadPausePlay,
          SIGNAL(result(bool, std_srvs::SetBoolResponse)),
          this,
          SLOT(onPushButtonPlayResult(bool, std_srvs::SetBoolResponse)));
  connect(workerThreadPausePlay, SIGNAL(finished()),
          workerThreadPausePlay, SLOT(deleteLater()));
  workerThreadPausePlay->setClient(pauseClient_);
  workerThreadPausePlay->setRequest(request);
  workerThreadPausePlay->start();
}

void FreeGaitMonitorPlugin::onPushButtonPause() {
  ui_.pushButtonPause->setEnabled(false);
  ui_.pushButtonPlay->setEnabled(false);

  std_srvs::SetBoolRequest request;
  request.data = true;

  WorkerThreadPausePlay *workerThreadPausePlay = new WorkerThreadPausePlay;
  connect(workerThreadPausePlay,
          SIGNAL(result(bool, std_srvs::SetBoolResponse)),
          this,
          SLOT(onPushButtonPauseResult(bool, std_srvs::SetBoolResponse)));
  connect(workerThreadPausePlay, SIGNAL(finished()),
          workerThreadPausePlay, SLOT(deleteLater()));
  workerThreadPausePlay->setClient(pauseClient_);
  workerThreadPausePlay->setRequest(request);
  workerThreadPausePlay->start();
}

void FreeGaitMonitorPlugin::onPushButtonStop() {
  ui_.pushButtonStop->setEnabled(false);
  ui_.pushButtonPause->setEnabled(false);
  ui_.pushButtonPlay->setEnabled(false);

  WorkerThreadStop *workerThreadStop = new WorkerThreadStop;
  connect(workerThreadStop,
          SIGNAL(result(bool, std_srvs::TriggerResponse)),
          this,
          SLOT(onPushButtonStopResult(bool, std_srvs::TriggerResponse)));
  connect(workerThreadStop, SIGNAL(finished()),
          workerThreadStop, SLOT(deleteLater()));
  workerThreadStop->setClient(stopClient_);
  workerThreadStop->start();
}

void FreeGaitMonitorPlugin::onPushButtonPlayResult(
    bool isOk, std_srvs::SetBoolResponse response) {
  if (isOk && response.success) {
    ui_.pushButtonPause->setEnabled(true);
    ui_.pushButtonPlay->setEnabled(false);
  } else {
    ui_.pushButtonPause->setEnabled(false);
    ui_.pushButtonPlay->setEnabled(true);
  }
}

void FreeGaitMonitorPlugin::onPushButtonPauseResult(
    bool isOk, std_srvs::SetBoolResponse response) {
  if (isOk && response.success) {
    ui_.pushButtonPause->setEnabled(false);
    ui_.pushButtonPlay->setEnabled(true);
  } else {
    ui_.pushButtonPause->setEnabled(true);
    ui_.pushButtonPlay->setEnabled(false);
  }
}

void FreeGaitMonitorPlugin::onPushButtonStopResult(
    bool isOk, std_srvs::TriggerResponse response) {
  if (isOk && response.success) {
    ui_.pushButtonStop->setEnabled(false);
    ui_.pushButtonPause->setEnabled(false);
    ui_.pushButtonPlay->setEnabled(false);
  } else {
    ui_.pushButtonStop->setEnabled(true);
    ui_.pushButtonPause->setEnabled(true);
    ui_.pushButtonPlay->setEnabled(false);
  }
}

void FreeGaitMonitorPlugin::onClickableLabelExpandCollapse() {
  if (ui_.plainTextEditDescription->isVisible()) {
    collapseDebug();
    ui_.clickableLabelExpandCollapse->setPixmap(
        QPixmap(":/icons/16x16/expand.svg"));
  } else {
    expandDebug();
    ui_.clickableLabelExpandCollapse->setPixmap(
        QPixmap(":/icons/16x16/collapse.svg"));
  }
}

void FreeGaitMonitorPlugin::onPushButtonDeleteHistory() {
  descriptions_.clear();
  ui_.plainTextEditDescription->setText("");

  updateNavigationButtonStates();
}

} // namespace

PLUGINLIB_EXPORT_CLASS(rqt_free_gait_monitor::FreeGaitMonitorPlugin, rqt_gui_cpp::Plugin)
