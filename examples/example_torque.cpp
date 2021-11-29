// /*****************************************************************
//  Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
// ******************************************************************/

// #include "unitree_legged_sdk/unitree_legged_sdk.h"
// #include <math.h>
// #include <iostream>
// #include <unistd.h>

// using namespace UNITREE_LEGGED_SDK;

// class Custom
// {
// public:
//     // Custom(): safe(LeggedType::A1, LOWLEVEL), udp(){
//     Custom(uint8_t level): safe(LeggedType::Aliengo), udp(level){
//         udp.InitCmdData(cmd);
//     }
//     void UDPSend();
//     void UDPRecv();
//     void RobotControl();

//     Safety safe;
//     UDP udp;
//     LowCmd cmd = {0};
//     LowState state = {0};
//     int motiontime = 0;
//     float dt = 0.002;     // 0.001~0.01
// };

// void Custom::UDPRecv()
// { 
//     udp.Recv();
// }

// void Custom::UDPSend()
// {  
//     udp.Send();
// }

// void Custom::RobotControl() 
// {
//     motiontime++;
//     udp.GetRecv(state);
//     // printf("%d\n", motiontime);
//     // gravity compensation
//     cmd.motorCmd[FR_0].tau = -0.65f;
//     cmd.motorCmd[FL_0].tau = +0.65f;
//     cmd.motorCmd[RR_0].tau = -0.65f;
//     cmd.motorCmd[RL_0].tau = +0.65f;

//     if( motiontime >= 500){
//         float torque = (0 - state.motorState[FR_1].q)*10.0f + (0 - state.motorState[FR_1].dq)*1.0f;
//         if(torque > 5.0f) torque = 5.0f;
//         if(torque < -5.0f) torque = -5.0f;

//         cmd.motorCmd[FR_1].q = PosStopF;
//         cmd.motorCmd[FR_1].dq = VelStopF;
//         cmd.motorCmd[FR_1].Kp = 0;
//         cmd.motorCmd[FR_1].Kd = 0;
//         cmd.motorCmd[FR_1].tau = torque;
//     }
//     safe.PowerProtect(cmd, state, 1);

//     udp.SetSend(cmd);
// }

// int main(void)
// {
//     std::cout << "Communication level is set to LOW-level." << std::endl
//               << "WARNING: Make sure the robot is hung up." << std::endl
//               << "Press Enter to continue..." << std::endl;
//     std::cin.ignore();

//     Custom custom(LOWLEVEL);
//     InitEnvironment();
//     LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
//     LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
//     LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

//     loop_udpSend.start();
//     loop_udpRecv.start();
//     loop_control.start();

//     while(1){
//         sleep(10);
//     };

//     return 0; 
// }















/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    // Custom(): safe(LeggedType::A1, LOWLEVEL), udp(){
    Custom(uint8_t level): safe(LeggedType::Aliengo), udp(level){
        udp.InitCmdData(cmd);
    }
    void UDPSend();
    void UDPRecv();
    void RobotControl();

    Safety safe;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
    int sin_count = 0;
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
    // printf("%d\n", motiontime);
    // gravity compensation
    cmd.motorCmd[FR_0].tau = -0.65f;
    cmd.motorCmd[FL_0].tau = +0.65f;
    cmd.motorCmd[RR_0].tau = -0.65f;
    cmd.motorCmd[RL_0].tau = +0.65f;

    // float freq_Hz = 1;
    float freq_Hz = 2;
    // float freq_Hz = 5;
    float freq_rad = freq_Hz * 2* M_PI;
    float t = dt*sin_count;

    if( motiontime >= 500){
        sin_count++;
        // float torque = (0 - state.motorState[FR_1].q)*10.0f + (0 - state.motorState[FR_1].dq)*1.0f;
        float torque = 2 * sin(t*freq_rad);
        if(torque > 5.0f) torque = 5.0f;
        if(torque < -5.0f) torque = -5.0f;

        cmd.motorCmd[FR_2].q = PosStopF;
        cmd.motorCmd[FR_2].dq = VelStopF;
        cmd.motorCmd[FR_2].Kp = 0;
        cmd.motorCmd[FR_2].Kd = 0;
        cmd.motorCmd[FR_2].tau = torque;
    }
    safe.PowerProtect(cmd, state, 1);

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




















// #include "unitree_legged_sdk/unitree_legged_sdk.h"
// #include <math.h>
// #include <iostream>
// #include <unistd.h>

// using namespace UNITREE_LEGGED_SDK;

// class Custom
// {
// public:
//     // Custom(): safe(LeggedType::A1, LOWLEVEL), udp(){
//     Custom(uint8_t level): safe(LeggedType::Aliengo), udp(level){
//         udp.InitCmdData(cmd);
//     }
//     void UDPSend();
//     void UDPRecv();
//     void RobotControl();

//     Safety safe;
//     UDP udp;
//     LowCmd cmd = {0};
//     LowState state = {0};
//     int motiontime = 0;
//     float dt = 0.002;     // 0.001~0.01
// };

// void Custom::UDPRecv()
// { 
//     udp.Recv();
// }

// void Custom::UDPSend()
// {  
//     udp.Send();
// }

// void Custom::RobotControl() 
// {
//     motiontime++;
//     udp.GetRecv(state);
//     // printf("%d\n", motiontime);
//     // gravity compensation
//     // cmd.motorCmd[FR_0].tau = -0.65f;
//     // cmd.motorCmd[FL_0].tau = +0.65f;
//     // cmd.motorCmd[RR_0].tau = -0.65f;
//     // cmd.motorCmd[RL_0].tau = +0.65f;

//     // if( motiontime >= 500){
//     //     float torque = (0 - state.motorState[FR_1].q)*10.0f + (0 - state.motorState[FR_1].dq)*1.0f;
//     //     if(torque > 5.0f) torque = 5.0f;
//     //     if(torque < -5.0f) torque = -5.0f;

//     //     cmd.motorCmd[FR_1].q = PosStopF;
//     //     cmd.motorCmd[FR_1].dq = VelStopF;
//     //     cmd.motorCmd[FR_1].Kp = 0;
//     //     cmd.motorCmd[FR_1].Kd = 0;
//     //     cmd.motorCmd[FR_1].tau = torque;
//     // }
//     // safe.PowerProtect(cmd, state, 1);

//     cmd.levelFlag = BACKFLIPLEVEL;
//     udp.SetSend(cmd);
// }

// int main(void)
// {
//     std::cout << "Communication level is set to LOW-level." << std::endl
//               << "WARNING: Make sure the robot is hung up." << std::endl
//               << "Press Enter to continue..." << std::endl;
//     std::cin.ignore();

//     Custom custom(LOWLEVEL);
//     // Custom custom(HIGHLEVEL);
//     InitEnvironment();
//     LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
//     LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
//     LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

//     loop_udpSend.start();
//     loop_udpRecv.start();
//     loop_control.start();

//     while(1){
//         sleep(10);
//     };

//     return 0; 
// }

























// #include "unitree_legged_sdk/unitree_legged_sdk.h"
// #include <math.h>
// #include <iostream>
// #include <unistd.h>

// using namespace UNITREE_LEGGED_SDK;

// class Custom
// {
// public:
//     Custom(uint8_t level): safe(LeggedType::A1), udp(level){
//         udp.InitCmdData(cmd);
//     }
//     void UDPSend();
//     void UDPRecv();
//     void RobotControl();

//     Safety safe;
//     UDP udp;
//     LowCmd cmd = {0};
//     LowState state = {0};
//     int motiontime = 0;
//     float dt = 0.002;     // 0.001~0.01
// };

// void Custom::UDPRecv()
// { 
//     udp.Recv();
// }

// void Custom::UDPSend()
// {  
//     udp.Send();
// }

// void Custom::RobotControl() 
// {
//     motiontime++;
//     udp.GetRecv(state);
//     // printf("%d   %f %f %f %f %f %f\n", motiontime, 
//     //                         state.motorState[0].q, state.motorState[0].q_raw, 
//     //                         state.motorState[0].dq, state.motorState[0].dq_raw,
//     //                         state.motorState[0].ddq, state.motorState[0].ddq_raw);
//     // printf("%d    %f   %f\n", motiontime, state.motorState[11].ddq, state.motorState[11].ddq*0.022);
//     float temp = state.motorState[11].ddq*0.022;
//     if(fabs(temp)>2) printf("%d    %f\n", motiontime, temp);


//     // if( motiontime >= 500){
//     //     // float torque = (0 - state.motorState[FR_2].q)*10.0f + (0 - state.motorState[FR_2].dq)*1.0f;
//     //     // if(torque > 5.0f) torque = 5.0f;
//     //     // if(torque < -5.0f) torque = -5.0f;

//     //     cmd.motorCmd[FR_2].q = PosStopF;
//     //     cmd.motorCmd[FR_2].dq = 0;
//     //     cmd.motorCmd[FR_2].Kp = 0;
//     //     cmd.motorCmd[FR_2].Kd = 20;
//     //     cmd.motorCmd[FR_2].tau = 0;
//     // }
//     safe.PowerProtect(cmd, state, 1);

//     udp.SetSend(cmd);
// }

// int main(void)
// {
//     std::cout << "Communication level is set to LOW-level." << std::endl
//               << "WARNING: Make sure the robot is hung up." << std::endl
//               << "Press Enter to continue..." << std::endl;
//     std::cin.ignore();

//     Custom custom(LOWLEVEL);
//     InitEnvironment();
//     LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
//     LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
//     LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

//     loop_udpSend.start();
//     loop_udpRecv.start();
//     loop_control.start();

//     while(1){
//         sleep(10);
//     };

//     return 0; 
// }