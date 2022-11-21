/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>

using namespace UNITREE_LEGGED_SDK;

// low cmd
constexpr uint16_t TARGET_PORT = 8007;
constexpr uint16_t LOCAL_PORT = 8082;
constexpr char TARGET_IP[] = "192.168.123.10";   // target IP address

const int LOW_CMD_LENGTH = 610;
const int LOW_STATE_LENGTH = 771;

class Custom
{
public:
    Custom(uint8_t level): 
        safe(LeggedType::Aliengo), 
        udp(LOCAL_PORT, TARGET_IP,TARGET_PORT, LOW_CMD_LENGTH, LOW_STATE_LENGTH){
        udp.InitCmdData(cmd);
        cmd.levelFlag = LOWLEVEL;
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    Safety safe;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    int Tpi = 0;
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
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
    motiontime++;
    udp.GetRecv(state);
    // if(motiontime%50 == 0){
        // printf("%ld    %ld     %ld\n", udp.udpState.RecvLoseError, udp.udpState.RecvCount, udp.udpState.SendCount);
    // }
    // printf("%10.5f  %10.5f  %10.5f  %10.5f  %10.5f  %10.5f  %10.5f  %10.5f  %10.5f  %10.5f  %10.5f  %10.5f\n", 
    //         state.motorState[0].q, state.motorState[1].q, state.motorState[2].q, 
    //         state.motorState[3].q, state.motorState[4].q, state.motorState[5].q, 
    //         state.motorState[6].q, state.motorState[7].q, state.motorState[8].q, 
    //         state.motorState[9].q, state.motorState[10].q, state.motorState[11].q );
    // gravity compensation
    cmd.motorCmd[FR_0].tau = -1.6f;
    // cmd.motorCmd[FL_0].tau = +0.0f;
    // cmd.motorCmd[RR_0].tau = -1.0f;
    // cmd.motorCmd[RL_0].tau = +1.0f;

    if( motiontime >= 500){
        float speed = 2 * sin(3*M_PI*Tpi/2000.0);

        cmd.motorCmd[FR_1].q = PosStopF;
        cmd.motorCmd[FR_1].dq = speed;
        cmd.motorCmd[FR_1].Kp = 0;
        cmd.motorCmd[FR_1].Kd = 4;
        // cmd.motorCmd[FR_1].Kd = 6;
        cmd.motorCmd[FR_1].tau = 0.0f;


        // cmd.motorCmd[FL_1].q = PosStopF;
        // cmd.motorCmd[FL_1].dq = speed;
        // cmd.motorCmd[FL_1].Kp = 0;
        // // cmd.motorCmd[FL_1].Kd = 4;
        // cmd.motorCmd[FL_1].Kd = 6;
        // cmd.motorCmd[FL_1].tau = 0.0f;

        Tpi++;
    }

    // if(motiontime > 10){
    //     safe.PowerProtect(cmd, state, 1);
    //     // safe.PositionProtect(cmd, state, 0.087);
    // }

    udp.SetSend(cmd);
}

int main(void) 
{
    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom(LOWLEVEL);
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
