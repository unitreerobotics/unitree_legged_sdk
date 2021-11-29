// /*****************************************************************
//  Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
// ******************************************************************/

// #include "unitree_legged_sdk/unitree_legged_sdk.h"
// #include <math.h>
// #include <iostream>

// using namespace UNITREE_LEGGED_SDK;

// class Custom
// {
// public:
//     // Custom(): safe(LeggedType::A1, LOWLEVEL), udp(){
//     Custom(uint8_t level): safe(LeggedType::Aliengo), udp(level){
//         udp.InitCmdData(cmd);
//     }
//     void UDPRecv();
//     void UDPSend();
//     void RobotControl();

//     Safety safe;
//     UDP udp;
//     LowCmd cmd = {0};
//     LowState state = {0};
//     int Tpi = 0;
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
//     printf("%f %f  %f\n", state.imu.rpy[2] , state.motorState[FL_1].q, state.motorState[FL_1].dq);
//     // gravity compensation
//     cmd.motorCmd[FR_0].tau = -0.65f;
//     cmd.motorCmd[FL_0].tau = +0.65f;
//     cmd.motorCmd[RR_0].tau = -0.65f;
//     cmd.motorCmd[RL_0].tau = +0.65f;

//     if( motiontime >= 500){
//         float speed = 1 * sin(3*M_PI*Tpi/1000.0);

//         cmd.motorCmd[FL_2].q = PosStopF;
//         cmd.motorCmd[FL_2].dq = speed;
//         cmd.motorCmd[FL_2].Kp = 0;
//         cmd.motorCmd[FL_2].Kd = 4;
//         cmd.motorCmd[FL_2].tau = 0.0f;
//         Tpi++;
//     }

//     if(motiontime > 10){
//         safe.PowerProtect(cmd, state, 1);
//         // safe.PositionProtect(cmd, state, 0.087);
//     }

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

















#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    // Custom(): safe(LeggedType::A1, LOWLEVEL), udp(){
    Custom(uint8_t level): safe(LeggedType::Aliengo), 
    udp(level)
    // udp(8080, "192.168.123.10", 8007, HIGH_CMD_LENGTH, HIGH_STATE_LENGTH, 2)
    {
        udp.InitCmdData(cmd);
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
        // printf("%f \n", state.imu.rpy[2]);
    // printf("temp: ");
    // for(int i=0; i<12; i++){
    //     printf("%5d ", state.motorState[i].temperature);
    // }
    // printf("\n");
    // printf("%3.6f       %3.6f\n",  state.imu.rpy[2], state.imu.gyroscope[2]);
    printf("%d       %d\n",  state.motorState[1].mode, state.motorState[2].mode);
    // }
    // printf("%10.5f  %10.5f  %10.5f  %10.5f  %10.5f  %10.5f  %10.5f  %10.5f  %10.5f  %10.5f  %10.5f  %10.5f\n", 
    //         state.motorState[0].q, state.motorState[1].q, state.motorState[2].q, 
    //         state.motorState[3].q, state.motorState[4].q, state.motorState[5].q, 
    //         state.motorState[6].q, state.motorState[7].q, state.motorState[8].q, 
    //         state.motorState[9].q, state.motorState[10].q, state.motorState[11].q );
    // gravity compensation
    // cmd.motorCmd[FR_0].tau = -0.65f;
    // cmd.motorCmd[FL_0].tau = +0.65f;
    // cmd.motorCmd[RR_0].tau = -0.65f;
    // cmd.motorCmd[RL_0].tau = +0.65f;

    // if( motiontime >= 500){
    //     float speed = 1 * sin(3*M_PI*Tpi/1000.0);

    //     cmd.motorCmd[FL_2].q = PosStopF;
    //     cmd.motorCmd[FL_2].dq = speed;
    //     cmd.motorCmd[FL_2].Kp = 0;
    //     cmd.motorCmd[FL_2].Kd = 4;
    //     cmd.motorCmd[FL_2].tau = 0.0f;
    //     Tpi++;
    // }

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

    // Custom custom(HIGHLEVEL);
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

// using namespace UNITREE_LEGGED_SDK;

// class Custom
// {
// public:
//     // Custom(): safe(LeggedType::A1, LOWLEVEL), udp(){
//     Custom(uint8_t level): safe(LeggedType::Aliengo), udp(level){
//         udp.InitCmdData(cmd);
//     }
//     void UDPRecv();
//     void UDPSend();
//     void RobotControl();

//     Safety safe;
//     UDP udp;
//     LowCmd cmd = {0};
//     LowState state = {0};
//     int Tpi = 0;
//     int motiontime = 0;
//     float dt = 0.002;     // 0.001~0.01
//     float qInit_2[3]={0};
//     float qInit_3[3]={0};
//     float qDes_2[3]={0};
//     float qDes_3[3]={0};
//     int rate_count = 0;
//     float Kp[3] = {0};  
//     float Kd[3] = {0};
// };

// void Custom::UDPRecv()
// {
//     udp.Recv();
// }

// void Custom::UDPSend()
// {  
//     udp.Send();
// }

// double jointLinearInterpolation(double initPos, double targetPos, double rate)
// {
//     double p;
//     rate = std::min(std::max(rate, 0.0), 1.0);
//     p = initPos*(1-rate) + targetPos*rate;
//     return p;
// }

// void Custom::RobotControl() 
// {
//     motiontime++;
//     udp.GetRecv(state);
//     printf("%10.5f  %10.5f  %10.5f  %10.5f  %10.5f  %10.5f  %10.5f  %10.5f  %10.5f  %10.5f  %10.5f  %10.5f\n", 
//             state.motorState[0].q, state.motorState[1].q, state.motorState[2].q, 
//             state.motorState[3].q, state.motorState[4].q, state.motorState[5].q, 
//             state.motorState[6].q, state.motorState[7].q, state.motorState[8].q, 
//             state.motorState[9].q, state.motorState[10].q, state.motorState[11].q );

//     float rearStand_J_pos[3] = { 0.0f, 2.21f, -2.1f};

//     if( motiontime >= 0 && motiontime < 10){
//         qInit_2[0] = state.motorState[6].q;
//         qInit_2[1] = state.motorState[7].q;
//         qInit_2[2] = state.motorState[8].q;
//         qInit_3[0] = state.motorState[9].q;
//         qInit_3[1] = state.motorState[10].q;
//         qInit_3[2] = state.motorState[11].q;
//     }
//     // second, move to the origin point of a sine movement with Kp Kd
//     // if( motiontime >= 500 && motiontime < 1500){
//     if( motiontime >= 10 && motiontime < 400){
//         rate_count++;
//         double rate = rate_count/300.0;                       // needs count to 300
//         Kp[0] = 5.0; Kp[1] = 5.0; Kp[2] = 5.0; 
//         Kd[0] = 1.0; Kd[1] = 1.0; Kd[2] = 1.0;
//         // Kp[0] = 45.0; Kp[1] = 45.0; Kp[2] = 45.0; 
//         // Kd[0] = 3.0; Kd[1] = 3.0; Kd[2] = 3.0;
//         // Kp[0] = 4.5; Kp[1] = 4.5; Kp[2] = 4.5; 
//         // Kd[0] = 0.3; Kd[1] = 0.3; Kd[2] = 0.3;
//         // Kp[0] = 10; Kp[1] = 10; Kp[2] = 10; 
//         // Kd[0] = 0.4; Kd[1] = 0.4; Kd[2] = 0.4;
        
//         qDes_2[0] = jointLinearInterpolation(qInit_2[0], rearStand_J_pos[0], rate);
//         qDes_2[1] = jointLinearInterpolation(qInit_2[1], rearStand_J_pos[1], rate);
//         qDes_2[2] = jointLinearInterpolation(qInit_2[2], rearStand_J_pos[2], rate);
//         qDes_3[0] = jointLinearInterpolation(qInit_3[0], rearStand_J_pos[0], rate);
//         qDes_3[1] = jointLinearInterpolation(qInit_3[1], rearStand_J_pos[1], rate);
//         qDes_3[2] = jointLinearInterpolation(qInit_3[2], rearStand_J_pos[2], rate);
//     }

//     if( motiontime >= 500){
//         Kp[0] = 50.0; Kp[1] = 50.0; Kp[2] = 50.0; 
//         Kd[0] = 3.0; Kd[1] = 3.0; Kd[2] = 3.0;

//         qDes_2[0] = rearStand_J_pos[0];
//         qDes_2[1] = rearStand_J_pos[1];
//         qDes_2[2] = rearStand_J_pos[2];
//         qDes_3[0] = rearStand_J_pos[0];
//         qDes_3[1] = rearStand_J_pos[1];
//         qDes_3[2] = rearStand_J_pos[2];
//     }

//     cmd.motorCmd[6].q = qDes_2[0];
//     cmd.motorCmd[6].dq = 0;
//     cmd.motorCmd[6].Kp = Kp[0];
//     cmd.motorCmd[6].Kd = Kd[0];
//     cmd.motorCmd[6].tau = 0.0f;
//     cmd.motorCmd[7].q = qDes_2[1];
//     cmd.motorCmd[7].dq = 0;
//     cmd.motorCmd[7].Kp = Kp[1];
//     cmd.motorCmd[7].Kd = Kd[1];
//     cmd.motorCmd[7].tau = 0.0f;
//     cmd.motorCmd[8].q = qDes_2[2];
//     cmd.motorCmd[8].dq = 0;
//     cmd.motorCmd[8].Kp = Kp[2];
//     cmd.motorCmd[8].Kd = Kd[2];
//     cmd.motorCmd[8].tau = 0.0f;
//     cmd.motorCmd[9].q = qDes_3[0];
//     cmd.motorCmd[9].dq = 0;
//     cmd.motorCmd[9].Kp = Kp[0];
//     cmd.motorCmd[9].Kd = Kd[0];
//     cmd.motorCmd[9].tau = 0.0f;
//     cmd.motorCmd[10].q = qDes_3[1];
//     cmd.motorCmd[10].dq = 0;
//     cmd.motorCmd[10].Kp = Kp[1];
//     cmd.motorCmd[10].Kd = Kd[1];
//     cmd.motorCmd[10].tau = 0.0f;
//     cmd.motorCmd[11].q = qDes_3[2];
//     cmd.motorCmd[11].dq = 0;
//     cmd.motorCmd[11].Kp = Kp[2];
//     cmd.motorCmd[11].Kd = Kd[2];
//     cmd.motorCmd[11].tau = 0.0f;

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
















// #include "unitree_legged_sdk/unitree_legged_sdk.h"
// #include <math.h>
// #include <iostream>

// using namespace UNITREE_LEGGED_SDK;

// class Custom
// {
// public:
//     // Custom(): safe(LeggedType::A1, LOWLEVEL), udp(){
//     Custom(uint8_t level): safe(LeggedType::Aliengo), udp(level){
//         udp.InitCmdData(cmd);
//     }
//     void UDPRecv();
//     void UDPSend();
//     void RobotControl();

//     Safety safe;
//     UDP udp;
//     LowCmd cmd = {0};
//     LowState state = {0};
//     int Tpi = 0;
//     int motiontime = 0;
//     float dt = 0.002;     // 0.001~0.01
//     float qInit_2[3]={0};
//     float qInit_3[3]={0};
//     float qDes_2[3]={0};
//     float qDes_3[3]={0};
//     int rate_count = 0;
//     float Kp[3] = {0};  
//     float Kd[3] = {0};
// };

// void Custom::UDPRecv()
// {
//     udp.Recv();
// }

// void Custom::UDPSend()
// {  
//     udp.Send();
// }

// double jointLinearInterpolation(double initPos, double targetPos, double rate)
// {
//     double p;
//     rate = std::min(std::max(rate, 0.0), 1.0);
//     p = initPos*(1-rate) + targetPos*rate;
//     return p;
// }

// void Custom::RobotControl() 
// {
//     motiontime++;
//     // memset(&state, 0, sizeof(LowState));
//     udp.GetRecv(state);
//     printf("q:%10.5f  dq:%10.5f  q:%10.5f  dq:%10.5f  q:%10.5f  dq:%10.5f  q:%10.5f  dq:%10.5f  q:%10.5f  dq:%10.5f  q:%10.5f  dq:%10.5f \n", 
//             state.motorState[0].q, state.motorState[0].dq, 
//             state.motorState[1].q, state.motorState[1].dq, 
//             state.motorState[2].q, state.motorState[2].dq, 
//             state.motorState[3].q, state.motorState[3].dq, 
//             state.motorState[4].q, state.motorState[4].dq, 
//             state.motorState[5].q, state.motorState[5].dq
            
//             );


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
