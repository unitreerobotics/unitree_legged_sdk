/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom(uint8_t level) : safe(LeggedType::A1), udp(level), mylcm(level){}
    void UDPRecv(){
        udp.Recv();
    }
    void UDPSend(){
        udp.Send();
    }
    void LCMRecv();
    void RobotControl() ;

    Safety safe;
    UDP udp;
    LCM mylcm;
    LowCmd cmd = {0};
    LowState state = {0};
};

void Custom::LCMRecv()
{
    if(mylcm.lowCmdLCMHandler.isrunning){
        pthread_mutex_lock(&mylcm.lowCmdLCMHandler.countMut);
        mylcm.lowCmdLCMHandler.counter++;
        if(mylcm.lowCmdLCMHandler.counter > 10){
            printf("Error! LCM Time out.\n");
            exit(-1);              // can be commented out
        }
        pthread_mutex_unlock(&mylcm.lowCmdLCMHandler.countMut);
    }
    mylcm.Recv();
}

void Custom::RobotControl() 
{
    udp.GetRecv(state);
    mylcm.Send(state);
    // printf("State size: %ld, tick: %d, robotid: %d\n", sizeof(state), state.tick, state.robotID);
    // printf("%d   %f %f %f\n", state.tick, state.imu.rpy[0], state.imu.rpy[1], state.imu.rpy[2]);
    // printf("State size: %ld %ld\n", sizeof(state), sizeof(state.motorState[0]));
    // printf("%f %f %f %d\n", state.motorState[11].q, state.motorState[11].dq, state.motorState[11].tauEst, state.motorState[11].temperature);
    // printf("%d %d %d %d     %d\n", state.footForce[0], state.footForce[1],state.footForce[2],state.footForce[3], state.tick);
    mylcm.Get(cmd);
    udp.SetSend(cmd);
}

int main(void) 
{
    
    Custom custom(LOWLEVEL);
    InitEnvironment();
    custom.mylcm.SubscribeCmd();

    LoopFunc loop_control("control_loop", 0.002, 3, boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("UDP_Send", 0.002, 3, boost::bind(&Custom::UDPSend, &custom));
    LoopFunc loop_udpRecv("UDP_Recv", 0.002, 3, boost::bind(&Custom::UDPRecv, &custom));
    LoopFunc loop_lcm("LCM_Recv", 0.002, 3, boost::bind(&Custom::LCMRecv, &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_lcm.start();
    loop_control.start();

    while(1){
        sleep(10);
    }

    return 0; 
}
