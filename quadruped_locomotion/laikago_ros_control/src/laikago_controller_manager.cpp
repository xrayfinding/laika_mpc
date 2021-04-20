#include <iostream>
#include <ros/ros.h>
#include "controller_manager/controller_manager.h"
#include "pluginlib/class_loader.h"
#include "ros/callback_queue.h"
#include <robot_state_lcm_hardware_interface.hpp>
#include <controller_manager/controller_manager.h>
//#include <robot_state_interface.hpp>
#include <balance_controller/ros_controler/robot_state_interface.hpp>

#include <boost/chrono.hpp>
#include <atomic>
#include <signal.h>

#include "std_msgs/Bool.h"
#include "free_gait_msgs/SetLimbConfigure.h"


std::atomic<bool> quit(false);//signal flag

void got_signal(int)
{
 quit = true;
}

class LaikagoControllerManager
{
public:

    LaikagoControllerManager(const ros::NodeHandle& nh)
        : nh_(nh)
    {
        ROS_WARN("Init LaikagoControllerManager!!!!!!!!!!");
        LCM_HW_.reset(new laikago_ros_control::RobotStateLcmHardwareInterface);
        LCM_HW_->init(nh_, nh_);
        controller_manager_.reset(new controller_manager::ControllerManager(LCM_HW_.get()));
        double loop_hz = 500;
        ros::Duration update_freq = ros::Duration(1.0 / loop_hz);

        ros::TimerOptions control_timer_options(ros::Duration(0.0025),
                                                boost::bind(&LaikagoControllerManager::update, this, _1),
                                                &update_queue_, false, false);
        //control_loop_ = nh_.createTimer(update_freq, &LaikagoControllerManager::update, this);
        control_loop_ = nh_.createTimer(control_timer_options);
        timer_thread_ = boost::thread(boost::bind(&LaikagoControllerManager::timerThread, this));
        control_loop_.start();
    }

    ~LaikagoControllerManager()
    {

      LCM_HW_->~RobotStateLcmHardwareInterface();
      ROS_INFO("Deconstruct BalanceControllerManager");
    }

    void update(const ros::TimerEvent&)
    {
        ros::Time time = ros::Time::now();
        ros::Duration period(0.0020);

        LCM_HW_->read(time, period);
        controller_manager_->update(time, period);
        LCM_HW_->write(time, period);
    }

    void update()
    {
      ros::Time time = ros::Time::now();
      ros::Duration peroid(0.01);
      ros::Rate rate(100);
      while (ros::ok()) {
          ROS_INFO("Loop once");
          time = ros::Time::now();


          LCM_HW_->read(time, peroid);
          controller_manager_->update(time, peroid);
          LCM_HW_->write(time, peroid);
          rate.sleep();
        }

    }
    void timerThread()
    {
      static const double timeout = 0.0025;
      while (nh_.ok()) {
          update_queue_.callAvailable(ros::WallDuration(timeout));
        }
    }

private:

    ros::NodeHandle nh_;
    std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
    std::shared_ptr<laikago_ros_control::RobotStateLcmHardwareInterface> LCM_HW_;

    ros::Timer control_loop_;
    boost::thread control_loop_thread_, timer_thread_;
    ros::CallbackQueue update_queue_;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "laikago_controller_manager");
    ros::NodeHandle nh;

    ros::MultiThreadedSpinner spinner(2);
    LaikagoControllerManager controller_manager(nh);

    struct sigaction sa;
    memset(&sa, 0, sizeof (sa));
    sa.sa_handler = got_signal;
    sigaction(SIGINT, &sa, NULL);
    spinner.spin();

    while (ros::ok()) {
        if(quit) break;
      }
    controller_manager.~LaikagoControllerManager();
    ROS_INFO("LaikagoControllerManager is shutdown");
    return 0;
}
