/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

// Notice: This exemple should running on another PC, and make sure the Ethernet is stable.

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include "multi_pc_type.h"

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom(): udp(8017, "192.168.2.100", 8018, sizeof(AAA), sizeof(BBB)){}
    void UDPRecv();
    void UDPSend();
    void Calc();

    UDP udp;
    AAA a;
    BBB b;
    float dt = 0.01;
};

void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

void Custom::Calc() 
{
    udp.GetRecv((char*)&b);
    // printf("%f\n", b.yaw);
    a.direction += 1000;

    udp.SetSend((char*)&a);
}

int main(void) 
{
    Custom custom;
    // InitEnvironment();
    LoopFunc loop_calc("calc_loop",   custom.dt,    boost::bind(&Custom::Calc,    &custom));
    LoopFunc loop_udpSend("udp_send", custom.dt, 3, boost::bind(&Custom::UDPSend, &custom));
    LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv, &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_calc.start();

    while(1){
        sleep(10);
    };

    return 0; 
}
