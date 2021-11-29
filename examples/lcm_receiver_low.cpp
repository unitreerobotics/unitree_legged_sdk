/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom(uint8_t level) : mylcm(level){
        cmd.levelFlag = level;
        for(int i = 0; i<12; i++){
            cmd.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
        }
    }
    void LCMRecv();
    void RobotControl();

    LCM mylcm;
    LowCmd cmd = {0};
    LowState state = {0};
};

void Custom::LCMRecv()
{
    mylcm.Recv();
}

void Custom::RobotControl()
{
    // mylcm.Recv();
    mylcm.Get(state);
    printf("State size: %ld, tick: %d, robotid: %d\n", sizeof(state), state.tick, state.robotID);
    printf("%d   %f %f %f\n", state.tick, state.imu.rpy[0], state.imu.rpy[1], state.imu.rpy[2]);
    printf("%d %d %d %d\n", state.footForce[0], state.footForce[1],state.footForce[2],state.footForce[3]);
    mylcm.Send(cmd);
}

int main(void)
{

    Custom custom(LOWLEVEL);
    InitEnvironment();
    // custom.mylcm.SubscribeCmd();
    custom.mylcm.SubscribeState();
    LoopFunc loop_lcm("LCM_Recv", 0.002, 3, boost::bind(&Custom::LCMRecv, &custom));
    LoopFunc loop_control("control_loop", 0.002, 3, boost::bind(&Custom::RobotControl, &custom));
    loop_lcm.start();
    loop_control.start();

    while(1){
        sleep(10);
    }

    return 0;
}
