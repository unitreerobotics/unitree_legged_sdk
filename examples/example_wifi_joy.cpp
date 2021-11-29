/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    // Custom(): safe(LeggedType::A1, HIGHLEVEL), udp(){
    Custom(uint8_t level): safe(LeggedType::A1), udp(8090,"192.168.123.10", 8007, HIGH_CMD_LENGTH, HIGH_STATE_LENGTH){
        udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    int motiontime = 0;
    float dt = 0.02;     // 0.001~0.01
};


void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

void Custom::RobotControl() 
{
    motiontime += 2;
    udp.GetRecv(state);
    // printf("%d   %f\n", motiontime, state.forwardSpeed);

    cmd.forwardSpeed = 0.0f;
    cmd.sideSpeed = 0.0f;
    cmd.rotateSpeed = 0.0f;
    cmd.bodyHeight = 0.0f;

    cmd.mode = 0;
    cmd.roll  = 0;
    cmd.pitch = 0;
    cmd.yaw = 0;


    udp.SetSend(cmd);
}

int main(void) 
{
    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom(HIGHLEVEL);
    InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while(1){
        sleep(10);
    };

    return 0; 
}
