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
    Custom(uint8_t level) : safe(LeggedType::B1),
                            udp(level, 8090, "192.168.123.220", 8082)
    {
        udp.InitCmdData(cmd);
        // udp.print = true;
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    int motiontime = 0;
    float dt = 0.002; // 0.001~0.01
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
    printf("%d   %f\n", motiontime, state.imu.rpy[2]);

    cmd.mode = 0; // 0:idle, default stand      1:forced stand     2:walk continuously
    cmd.gaitType = 0;
    cmd.speedLevel = 0;
    cmd.footRaiseHeight = 0;
    cmd.bodyHeight = 0;
    cmd.euler[0] = 0;
    cmd.euler[1] = 0;
    cmd.euler[2] = 0;
    cmd.velocity[0] = 0.0f;
    cmd.velocity[1] = 0.0f;
    cmd.yawSpeed = 0.0f;
    cmd.reserve = 0;

    if (motiontime > 0 && motiontime < 2000)
    {
        cmd.mode = 6;
    }
    else if(motiontime >= 2000 && motiontime < 3000)
    {
        cmd.mode = 1;
    }
    else if(motiontime >= 3000 && motiontime < 4000)
    {
        cmd.mode = 1;
        cmd.euler[0] = 0.3;
    }
    else if(motiontime >= 4000 && motiontime < 6000)
    {
        cmd.mode = 1;
        cmd.euler[0] = -0.3;
    }
    else if(motiontime >= 6000 && motiontime < 8000)
    {
        cmd.mode = 1;
        cmd.euler[1] = 0.3;
    }
    else if(motiontime >= 8000 && motiontime < 10000)
    {
        cmd.mode = 1;
        cmd.euler[1] = -0.3;
    }
    else if(motiontime >= 10000 && motiontime < 12000)
    {
        cmd.mode = 1;
        cmd.euler[2] = 0.3;
    }
    else if(motiontime >= 12000 && motiontime < 14000)
    {
        cmd.mode = 1;
        cmd.euler[2] = -0.3;
    }
    else if(motiontime >= 14000 && motiontime < 15000)
    {
        cmd.mode = 1;
    }
    else if(motiontime >= 15000 && motiontime < 18000)
    {
        cmd.mode = 2;
        cmd.velocity[0] = 0.3;
        cmd.yawSpeed = 0.2;
    }
    else if(motiontime >= 18000 && motiontime < 21000)
    {
        cmd.mode = 2;
        cmd.velocity[0] = -0.3;
        cmd.yawSpeed = -0.2;
    }
    else if(motiontime >= 21000 && motiontime < 22000)
    {
        cmd.mode = 2;
    }
    else if(motiontime >= 22000 && motiontime < 23000)
    {
        cmd.mode = 1;
    }
    else if(motiontime >= 23000 && motiontime < 26000)
    {
        cmd.mode = 2;
        cmd.gaitType = 3;
        cmd.velocity[0] = 0.3;
    }
    else if(motiontime >= 26000 && motiontime < 29000)
    {
        cmd.mode = 2;
        cmd.gaitType = 3;
        cmd.velocity[0] = -0.3;
    }
   else if(motiontime >= 29000 && motiontime < 30000)
    {
        cmd.mode = 2;
    }
    else if(motiontime >= 30000 && motiontime < 31000)
    {
        cmd.mode = 1;
    }
    else if(motiontime >= 31000 && motiontime < 34000)
    {
        cmd.mode = 2;
        cmd.velocity[0] = 0.3;
        cmd.footRaiseHeight = 0.1;
    }
    else if(motiontime >= 34000 && motiontime < 37000)
    {
        cmd.mode = 2;
        cmd.velocity[0] = -0.3;
        cmd.footRaiseHeight = -0.1;
    }
    else if(motiontime >= 37000 && motiontime < 38000)
    {
        cmd.mode = 2;
    }
    else if(motiontime >= 39000 && motiontime < 40000)
    {
        cmd.mode = 1;
    }
    else if(motiontime >= 40000 && motiontime < 43000)
    {
        cmd.mode = 1;
        cmd.bodyHeight = 0.1;
    }
    else if(motiontime >= 43000 && motiontime < 46000)
    {
        cmd.mode = 1;
        cmd.bodyHeight = -0.1;
    }
    else if(motiontime >= 46000 && motiontime < 49000)
    {
        cmd.mode = 1;
    }
    else if(motiontime >= 49000 && motiontime < 52000)
    {
        cmd.mode = 6;
    }
    else if(motiontime >= 52000 && motiontime < 55000)
    {
        cmd.mode = 5;
    }
    else if(motiontime >= 55000 && motiontime < 58000)
    {
        cmd.mode = 7;
    }
    else 
    {
        cmd.mode = 0;
    }
    
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
    LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send", custom.dt, 3, boost::bind(&Custom::UDPSend, &custom));
    LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv, &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while (1)
    {
        sleep(10);
    };

    return 0;
}
