/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "laikago_sdk/laikago_sdk.hpp"
#include <math.h>
#include <iostream>

using namespace laikago;

LowCmd cmd = {0};
LowState state = {0};
Control control(LOWLEVEL);
UDP udp(LOW_CMD_LENGTH, LOW_STATE_LENGTH);
LCM mylcm;

void UDPRecv()
{
    udp.Recv();
}

void LCMRecv()
{
    mylcm.Recv();
}

void RobotControl()
{
    udp.GetState(state);
    mylcm.Send(state);

     std::cout<<"RF_HAA"<<std::endl;
     printf("%f\n", state.motorState[FR_0].position);
     std::cout<<"RF_HFE"<<std::endl;
     printf("%f\n", state.motorState[FR_1].position);
     std::cout<<"RF_KFE"<<std::endl;
     printf("%f\n", state.motorState[FR_2].position);

    mylcm.Get(cmd);
    udp.Send(cmd);
}

int main(void)
{
    control.loop.SetLCM(true);
    mylcm.SubscribeCmd();
    control.InitCmdData(cmd);
    control.loop.RegistFunc("UDP/Send", RobotControl);
    control.loop.RegistFunc("UDP/Recv", UDPRecv);
    control.loop.RegistFunc("LCM/Recv", LCMRecv);
    control.loop.Start();

    return 0;
}
