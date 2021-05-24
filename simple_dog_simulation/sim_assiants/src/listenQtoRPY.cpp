#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "ros/ros.h"
#include <iostream>
#include "free_gait_msgs/RobotState.h"
#include "geometry_msgs/Vector3.h"
using namespace std;
Eigen::Quaterniond orient;
Eigen::Vector3d rpy_ang;
geometry_msgs::Vector3 pub_msg;
void basePoseCallback(const free_gait_msgs::RobotStateConstPtr& robot_state){
    orient.w() = robot_state->base_pose.pose.pose.orientation.w;
    orient.x() = robot_state->base_pose.pose.pose.orientation.x;
    orient.y() = robot_state->base_pose.pose.pose.orientation.y;
    orient.z() = robot_state->base_pose.pose.pose.orientation.z;
    Eigen::Matrix3d rot_mat = orient.toRotationMatrix();
    rpy_ang = rot_mat.matrix().eulerAngles(2,1,0);
    pub_msg.x = rpy_ang.x();
    pub_msg.y = rpy_ang.y();
    pub_msg.z = rpy_ang.z();
}
int main(int argc, char ** argv){
    ros::init(argc,argv,"pubRPY");
    ros::NodeHandle nh;
    orient.setIdentity();
    rpy_ang.setZero();
    ros::Subscriber quatListen = nh.subscribe("/gazebo/robot_states",1,basePoseCallback);
    ros::Publisher pub_rpy = nh.advertise<geometry_msgs::Vector3>("/rpy_ang",1);
    ros::Rate rate(100);
    while(ros::ok()){
        pub_rpy.publish(pub_msg);
        ros::spinOnce();
        rate.sleep();
    }
}
