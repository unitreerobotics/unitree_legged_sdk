#!/usr/bin/python

import sys
import time
import math

sys.path.append('../lib/python/amd64')
import robot_interface as sdk


if __name__ == '__main__':

    d = {'FR_0':0, 'FR_1':1, 'FR_2':2,
         'FL_0':3, 'FL_1':4, 'FL_2':5, 
         'RR_0':6, 'RR_1':7, 'RR_2':8, 
         'RL_0':9, 'RL_1':10, 'RL_2':11 }
    PosStopF  = math.pow(10,9)
    VelStopF  = 16000.0
    HIGHLEVEL = 0xee
    LOWLEVEL  = 0xff

    # udp = sdk.UDP(8080, "192.168.123.10", 8007, 614, 807, False, sdk.RecvEnum.nonBlock)
    udp = sdk.UDP(LOWLEVEL, 8080, "192.168.123.10", 8007)
    safe = sdk.Safety(sdk.LeggedType.Go1)
    
    cmd = sdk.LowCmd()
    state = sdk.LowState()
    udp.InitCmdData(cmd)

    Tpi = 0
    motiontime = 0
    while True:
        time.sleep(0.002)
        motiontime += 1

        # print(motiontime)
        # print(state.imu.rpy[2])
        cmd.motorCmd[d['FR_0']].q = PosStopF
        cmd.motorCmd[d['FR_0']].dq = VelStopF
        cmd.motorCmd[d['FR_0']].Kp = 0
        cmd.motorCmd[d['FR_0']].Kd = 0
        cmd.motorCmd[d['FR_0']].tau = -5.0
        
        
        udp.Recv()
        udp.GetRecv(state)
        
        if( motiontime >= 500):
            speed = 2 * math.sin(3*math.pi*Tpi/2000.0)

            cmd.motorCmd[d['FL_1']].q = PosStopF
            #cmd.motorCmd[d['FR_2']].q = 0
            cmd.motorCmd[d['FR_1']].dq = speed
            cmd.motorCmd[d['FR_1']].Kp = 0
            cmd.motorCmd[d['FR_1']].Kd = 4
            cmd.motorCmd[d['FR_1']].tau = 0.0

            Tpi += 1
        
        # if(motiontime > 10):
        #     safe.PowerProtect(cmd, state, 1)
        

        udp.SetSend(cmd)
        udp.Send()
