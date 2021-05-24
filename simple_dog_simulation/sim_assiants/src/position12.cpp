#include <ros/ros.h>
//#include "servocontrol.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"
#include "string.h"
#include <cmath>
#include <fstream>
using namespace std;
const int MOTOR_MULTI = 1;
const int MOTOR_MULTI2 = 8;
int flag_init = 0;
std_msgs::Float64MultiArray joint_group_positions;
std_msgs::Float64MultiArray joint_group_init_positions;
std_msgs::Float64MultiArray joint_group_init0_positions;
std_msgs::Float64MultiArray init_position;
ifstream _csvInput;
string _Oneline;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "position12motor");
    ros::NodeHandle nh;
    _csvInput.open("/home/glx/laikago_catkin/src/simple_dog_simulation/sim_assiants/src/dma_3.csv");
    init_position.data.resize(12);
    joint_group_positions.data.resize(12);
    joint_group_positions.data[0] = 0;
    joint_group_positions.data[1] = 0.7;
    joint_group_positions.data[2] = -1.55;
    joint_group_positions.data[3] = 0;
    joint_group_positions.data[4] = 0.7;
    joint_group_positions.data[5] = -1.55;
    joint_group_positions.data[6] = 0;
    joint_group_positions.data[7] = 0.7;
    joint_group_positions.data[8] = -1.55;
    joint_group_positions.data[9] = 0;
    joint_group_positions.data[10] = 0.7;
    joint_group_positions.data[11] = -1.55;

    joint_group_init0_positions.data.resize(12);
    //lf
    joint_group_init0_positions.data[0] = 0;
    joint_group_init0_positions.data[1] = 1.8;
    joint_group_init0_positions.data[2] = -2.7;
    //rf
    joint_group_init0_positions.data[3] = 0;
    joint_group_init0_positions.data[4] = 1.8;
    joint_group_init0_positions.data[5] = -2.7;
    //br
    joint_group_init0_positions.data[6] = 0;
    joint_group_init0_positions.data[7] = 1.8;
    joint_group_init0_positions.data[8] = -2.7;
    //bl
    joint_group_init0_positions.data[9] = 0;
    joint_group_init0_positions.data[10] = 1.8;
    joint_group_init0_positions.data[11] = -2.7;
    //the position of joint for controller init
    joint_group_init_positions.data.resize(12);
    //lf
    joint_group_init_positions.data[0] = 8.647888888566158212e-04;
    joint_group_init_positions.data[1] = 7.771166839953650429e-01;
    joint_group_init_positions.data[2] = -1.553032606298970109e+00;
    //rf
    joint_group_init_positions.data[3] = 8.647888888566158212e-04;
    joint_group_init_positions.data[4] = 7.771166839953650429e-01;
    joint_group_init_positions.data[5] = -1.553032606298970109e+00;
    //br
    joint_group_init_positions.data[6] = 8.647888888566158212e-04;
    joint_group_init_positions.data[7] = 7.771166839953650429e-01;
    joint_group_init_positions.data[8] = -1.553032606298970109e+00;
    //bl
    joint_group_init_positions.data[9] = 8.647888888566158212e-04;
    joint_group_init_positions.data[10] = 7.771166839953650429e-01;
    joint_group_init_positions.data[11] = -1.553032606298970109e+00;

    ros::Publisher joint_group_position_pub = nh.advertise<std_msgs::Float64MultiArray>("/all_joints_position_group_controller/command",1);
    ros::Rate rate(20);
    ROS_INFO("get Ready");
    double time1 = 0.0;
    int count = 0;
    while(ros::ok())
    {
        if(flag_init == 0){
            cout << "init position" << endl;
            for (int i = 0; i < 12; i++) {
                init_position.data[i] = joint_group_init0_positions.data[i] + (joint_group_init_positions.data[i] - joint_group_init0_positions.data[i]) * count / 100.0;
            }
            cout << init_position.data[11]<< endl;
            joint_group_position_pub.publish(init_position);
            count++;
            if(count >= 100){
                flag_init = 1;
            }
        }
        else {
            getline(_csvInput, _Oneline);
            //cout << _Oneline << endl;
            istringstream _Readstr(_Oneline);
            string _partOfstr;
            for(int i = 0; i < 12; i++)
            {
                getline(_Readstr, _partOfstr,' ');
                joint_group_positions.data[i] = atof(_partOfstr.c_str());
                cout << joint_group_positions.data[i]<<" ";
            }
            cout << endl;
//            //lf
//            joint_group_positions.data[0] = 0.0;
//            joint_group_positions.data[1] = joint_group_init_positions.data[1] + (8*2*3.14/360*sin(2*3.14/0.4 * time1-3.14/2)) * MOTOR_MULTI;
//            joint_group_positions.data[2] = joint_group_init_positions.data[2] + (-5*2*3.14/360*sin(2*3.14/0.4*time1+3.14)/2-fabs(5*2*3.14/360*sin(2*3.14/0.4*time1+3.14)/2))*MOTOR_MULTI2;
//            //rf
//            joint_group_positions.data[3] = 0.0;
//            joint_group_positions.data[4] = joint_group_init_positions.data[4] - (8*2*3.14/360*sin( 2*3.14/0.4*time1+3.14/2))*MOTOR_MULTI;
//            joint_group_positions.data[5] = joint_group_init_positions.data[5] - (-5*2*3.14/360*sin( 2*3.14/0.4*time1)/2-fabs( 5*2*3.14/360*sin( 2*3.14/0.4*time1)/2))*MOTOR_MULTI2;

//            //br
//            joint_group_positions.data[6] = 0.0;
//            joint_group_positions.data[7] = joint_group_init_positions.data[7] - (8*2*3.14/360*sin(2*3.14/0.4 * time1-3.14/2)) * MOTOR_MULTI;
//            joint_group_positions.data[8] = joint_group_init_positions.data[8] - (-5*2*3.14/360*sin(2*3.14/0.4*time1+3.14)/2-fabs(5*2*3.14/360*sin(2*3.14/0.4*time1+3.14)/2))*MOTOR_MULTI2;
//            //bl
//            joint_group_positions.data[9] = 0.0;
//            joint_group_positions.data[10] = joint_group_init_positions.data[10] + (8*2*3.14/360*sin( 2*3.14/0.4*time1+3.14/2))*MOTOR_MULTI;
//            joint_group_positions.data[11] = joint_group_init_positions.data[11] + (-5*2*3.14/360*sin( 2*3.14/0.4*time1)/2-fabs( 5*2*3.14/360*sin( 2*3.14/0.4*time1)/2))*MOTOR_MULTI2;

            //cout << joint_group_positions.data[10]<<"  "<<joint_group_positions.data[11] << endl;
            if(joint_group_positions.data[0]==0 and joint_group_positions.data[1]==0) break;
            joint_group_position_pub.publish(joint_group_positions);
            //time1 = time1 + 0.001;
        }

        ros::spinOnce();
        rate.sleep();
    }
}
