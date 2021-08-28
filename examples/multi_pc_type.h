/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/
#ifndef _MULTI_PC_TYPE_H_
#define _MULTI_PC_TYPE_H_

#include <stdint.h>

struct AAA{
    int direction;
    float deepth;
    uint32_t crc;
};

struct BBB{
    float yaw;
    float pitch;
    float roll;
    uint32_t crc;
};

#endif
