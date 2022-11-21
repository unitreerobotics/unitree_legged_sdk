#!/usr/bin/python

import sys
import time
import math

sys.path.append('../lib/python/amd64')
import robot_interface as sdk

# high cmd
TARGET_PORT = 8082
LOCAL_PORT = 8081
TARGET_IP = "192.168.123.220"   # target IP address

HIGH_CMD_LENGTH = 113
HIGH_STATE_LENGTH = 244


if __name__ == '__main__':

    HIGHLEVEL = 0x00
    LOWLEVEL  = 0xff

    # udp = sdk.UDP(8080, "192.168.123.161", 8082, 129, 1087, False, sdk.RecvEnum.nonBlock)
    # udp = sdk.UDP(HIGHLEVEL, 8080, "192.168.123.161", 8082)

    cmd = sdk.HighCmd()
    state = sdk.HighState()

    

    
    udp = sdk.UDP(LOCAL_PORT, TARGET_IP, TARGET_PORT, HIGH_CMD_LENGTH, HIGH_STATE_LENGTH, -1)

    udp.InitCmdData(cmd)

    motiontime = 0
    while True:
        time.sleep(0.002)
        motiontime = motiontime + 1

        # print(motiontime)
        # print(state.imu.rpy[0])
        
        
        udp.Recv()
        udp.GetRecv(state)
        

        # print(motiontime, state.motorState[0].q, state.motorState[1].q, state.motorState[2].q)
        print(state.imu.rpy[2])

        cmd.mode = 0      # 0:idle, default stand      1:forced stand     2:walk continuously
        cmd.gaitType = 0
        cmd.speedLevel = 0
        cmd.dFootRaiseHeight = 0
        cmd.dBodyHeight = 0
        cmd.rpy = [0, 0, 0]
        cmd.velocity = [0, 0]
        cmd.yawSpeed = 0.0
        cmd.reserve = 0

        # cmd.mode = 2
        # cmd.gaitType = 1
        # # cmd.position = [1, 0]
        # # cmd.position[0] = 2
        # cmd.velocity = [-0.2, 0] # -1  ~ +1
        # cmd.yawSpeed = 0
        # cmd.bodyHeight = 0.1

        if(motiontime > 0 and motiontime < 1000):
            cmd.mode = 1
            cmd.rpy = [-0.3, 0, 0]
        
        if(motiontime > 1000 and motiontime < 2000):
            cmd.mode = 1
            cmd.rpy = [0.3, 0, 0]
        
        if(motiontime > 2000 and motiontime < 3000):
            cmd.mode = 1
            cmd.rpy = [0, -0.2, 0]
        
        if(motiontime > 3000 and motiontime < 4000):
            cmd.mode = 1
            cmd.rpy = [0, 0.2, 0]
        
        if(motiontime > 4000 and motiontime < 5000):
            cmd.mode = 1
            cmd.rpy = [0, 0, -0.2]
        
        if(motiontime > 5000 and motiontime < 6000):
            cmd.mode = 1
            cmd.rpy = [0.2, 0, 0]
        
        if(motiontime > 6000 and motiontime < 7000):
            cmd.mode = 1
            cmd.dBodyHeight = -0.2
        
        if(motiontime > 7000 and motiontime < 8000):
            cmd.mode = 1
            cmd.dBodyHeight = 0.1
        
        if(motiontime > 8000 and motiontime < 9000):
            cmd.mode = 1
            cmd.dBodyHeight = 0.0
        
        if(motiontime > 9000 and motiontime < 11000):
            cmd.mode = 5
        
        if(motiontime > 11000 and motiontime < 13000):
            cmd.mode = 6
        
        if(motiontime > 13000 and motiontime < 14000):
            cmd.mode = 0
        
        if(motiontime > 14000 and motiontime < 18000):
            cmd.mode = 2
            cmd.gaitType = 2
            cmd.velocity = [0.4, 0] # -1  ~ +1
            cmd.yawSpeed = 2
            cmd.dFootRaiseHeight = 0.1
            # printf("walk\n")
        
        if(motiontime > 18000 and motiontime < 20000):
            cmd.mode = 0
            cmd.velocity = [0, 0]
        
        if(motiontime > 20000 and motiontime < 24000):
            cmd.mode = 2
            cmd.gaitType = 1
            cmd.velocity = [0.2, 0] # -1  ~ +1
            cmd.dBodyHeight = 0.1
            # printf("walk\n")
            

        udp.SetSend(cmd)
        udp.Send()
