#include "unitree_legged_sdk/lcm_server.h"
#include <iostream>

using namespace UNITREE_LEGGED_SDK;

//argv[1]:control level: LOWLEVEL or HIGHLEVEL, not case sensitive
int main(int argc, char *argv[]) 
{
    // LeggedType rname;
    // if(strcasecmp(argv[1], "A1") == 0)
    //     rname = LeggedType::A1;
    // else if(strcasecmp(argv[1], "Aliengo") == 0)
    //     rname = LeggedType::Aliengo;
    // else
    // {
    //     std::cout << "Robot name error! Can only be A1 or Aliengo(not case sensitive)" << std::endl;
    //     exit(-1);
    // }

    // InitEnvironment();
    
    if(strcasecmp(argv[1], "LOWLEVEL") == 0)
    {
        Lcm_Server_Low server;
        server.mylcm.SubscribeCmd();
        LoopFunc loop_control("control_loop", 0.002, boost::bind(&Lcm_Server_Low::RobotControl, &server));
        LoopFunc loop_udpSend("UDP_Send", 0.002, 3, boost::bind(&Lcm_Server_Low::UDPSend, &server));
        LoopFunc loop_udpRecv("UDP_Recv", 0.002, 3, boost::bind(&Lcm_Server_Low::UDPRecv, &server));
        LoopFunc loop_lcm("LCM_Recv", 0.002, boost::bind(&Lcm_Server_Low::LCMRecv, &server));
        loop_udpSend.start();
        loop_udpRecv.start();
        loop_lcm.start();
        loop_control.start();
        while(1){
            sleep(10);
        }
    }
    else if(strcasecmp(argv[1], "HIGHLEVEL") == 0)
    {
        Lcm_Server_High server;
        server.mylcm.SubscribeCmd();
        LoopFunc loop_control("control_loop", 0.002, boost::bind(&Lcm_Server_High::RobotControl, &server));
        LoopFunc loop_udpSend("UDP_Send", 0.002, 3, boost::bind(&Lcm_Server_High::UDPSend, &server));
        LoopFunc loop_udpRecv("UDP_Recv", 0.002, 3, boost::bind(&Lcm_Server_High::UDPRecv, &server));
        LoopFunc loop_lcm("LCM_Recv", 0.002, boost::bind(&Lcm_Server_High::LCMRecv, &server));
        loop_udpSend.start();
        loop_udpRecv.start();
        loop_lcm.start();
        loop_control.start();
        while(1){
            sleep(10);
        }
    }
    else
    {
        std::cout << "Control level name error! Can only be highlevel or lowlevel(not case sensitive)" << std::endl;
        exit(-1);
    }

    return 0; 
}