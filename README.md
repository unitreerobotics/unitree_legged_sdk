# v3.5.1
The unitree_legged_sdk is mainly used for communication between PC and Controller board.
It also can be used in other PCs with UDP.

### Notice
support robot: Go1

not support robot: Laikago, Aliengo, A1. (Check release [v3.3.1](https://github.com/unitreerobotics/unitree_legged_sdk/releases/tag/v3.3.1) for support)

### Sport Mode
```bash
Legged_sport    >= v1.36.0
firmware H0.1.7 >= v0.1.35
         H0.1.9 >= v0.1.35
```

### Dependencies
* [Boost](http://www.boost.org) (version 1.5.4 or higher)
* [CMake](http://www.cmake.org) (version 2.8.3 or higher)
* [LCM](https://lcm-proj.github.io) (version 1.4.0 or higher)
* [g++](https://gcc.gnu.org/) (version 8.3.0 or higher)
```bash
cd lcm-x.x.x
mkdir build
cd build
cmake ../
make
sudo make install
```

### Build
```bash
mkdir build
cd build
cmake ../
make
```

### Usage
Run examples with 'sudo' for memory locking.

### Interface
Here is the low-level position control example using python interface:

```python 
from unitree_legged_sdk.build.robot_interface import LowInterface, HighInterface
import time
import numpy as np


def jointLinearInterpolation(initPos, targetPos, rate):
    rate = min(max(rate, 0.0), 1.0)
    p = initPos*(1-rate) + targetPos*rate
    return p


if __name__ == "__main__":
    robot = LowInterface()
    motion_time = 0
    rate_count = 0
    qInit = np.zeros(3)
    qDes = np.zeros(3)
    sin_mid_q = [0.0, 1.2, -2.0]
    sin_count = 0

    while True:
        motion_time += 1
        robot.reset_command()
        position = np.zeros(12)
        velocity = np.zeros(12)
        torque = np.zeros(12)
        Kp = np.zeros(12)
        Kd = np.zeros(12)

        state = robot.recv_observation()
        print("motion time: ", motion_time, " ", state.footForce)

        if motion_time >= 0 and motion_time < 10:
            qInit[0] = state.motorState[FR_0].q
            qInit[1] = state.motorState[FR_1].q
            qInit[2] = state.motorState[FR_2].q

        if motion_time >= 10 and motion_time < 400:
            rate_count += 1
            rate = rate_count/200.0
            qDes[0] = jointLinearInterpolation(qInit[0], sin_mid_q[0], rate)
            qDes[1] = jointLinearInterpolation(qInit[1], sin_mid_q[1], rate)
            qDes[2] = jointLinearInterpolation(qInit[2], sin_mid_q[2], rate)

        if motion_time >= 400:
            sin_count += 1
            sin_joint1 = 0.6 * np.sin(3*np.pi*sin_count/1000.0)
            sin_joint2 = -0.6 * np.sin(1.8*np.pi*sin_count/1000.0)
            qDes[0] = sin_mid_q[0]
            qDes[1] = sin_mid_q[1]
            qDes[2] = sin_mid_q[2] + sin_joint2

        torque[FR_0] = -0.65
        torque[FL_0] = 0.65
        torque[RR_0] = -0.65
        torque[RL_0] = 0.65
        Kp[FR_0] = 5.0
        Kp[FR_1] = 5.0
        Kp[FR_2] = 5.0      
        Kd[FR_0] = 1.0
        Kd[FR_1] = 1.0
        Kd[FR_2] = 1.0
        position[FR_0] = qDes[0]
        position[FR_1] = qDes[1]
        position[FR_2] = qDes[2]
        command = np.concatenate([
            position, velocity, torque, Kp, Kd
        ])
        robot.set_command(command)
        time.sleep(0.001)
```