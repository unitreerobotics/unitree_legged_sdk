/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#ifndef _UNITREE_LEGGED_COMM_H_
#define _UNITREE_LEGGED_COMM_H_

#include <stdint.h>
#include <array>

namespace UNITREE_LEGGED_SDK 
{

	constexpr int HIGHLEVEL = 0x00;
	constexpr int LOWLEVEL  = 0xff;
	constexpr int BACKFLIPLEVEL  = 0xf0;
	constexpr double PosStopF = (2.146E+9f);
	constexpr double VelStopF = (16000.0f);

#pragma pack(1)

	typedef struct
	{
		float x;
		float y;
		float z;
	} Cartesian;

	typedef struct
	{
		// float quaternion[4];               // quaternion, normalized, (w,x,y,z)
		std::array<float, 4> quaternion;
		// float gyroscope[3];                // angular velocity （unit: rad/s)
		std::array<float, 3> gyroscope;
		// float accelerometer[3];            // m/(s2)
		std::array<float, 3> accelerometer;
		// float rpy[3];                      // euler angle（unit: rad)
		std::array<float, 3> rpy;
		int8_t temperature;
	} IMU;                                 // when under accelerated motion, the attitude of the robot calculated by IMU will drift.

	typedef struct
	{
		uint8_t r;
		uint8_t g;
		uint8_t b;
	} LED;                                 // foot led brightness: 0~255

	typedef struct
	{
		uint8_t mode;                      // motor working mode 
		float q;                           // current angle (unit: radian)
		float dq;                          // current velocity (unit: radian/second)
		float ddq;                       // current acc (unit: radian/second*second)
		float tauEst;                    // current estimated output torque (unit: N.m)
		float q_raw;                       // current angle (unit: radian)
		float dq_raw;                      // current velocity (unit: radian/second)
		float ddq_raw;
		int8_t temperature;                // current temperature (temperature conduction is slow that leads to lag)
		// uint32_t reserve[2];
		std::array<uint32_t, 2> reserve;
	} MotorState;                          // motor feedback

	typedef struct
	{
		uint8_t mode;                      // desired working mode
		float q;                           // desired angle (unit: radian) 
		float dq;                          // desired velocity (unit: radian/second)
		float tau;                       // desired output torque (unit: N.m)
		float Kp;                        // desired position stiffness (unit: N.m/rad )
		float Kd;                        // desired velocity stiffness (unit: N.m/(rad/s) )
		// uint32_t reserve[3];
		std::array<uint32_t, 3> reserve;
	} MotorCmd;                            // motor control

	typedef struct
	{
		uint8_t levelFlag;                 // flag to distinguish high level or low level
		uint16_t commVersion;
		uint16_t robotID;
		uint32_t SN; 
		uint8_t bandWidth;
		IMU imu;
		// MotorState motorState[20];
		std::array<MotorState, 20> motorState;
		// int16_t footForce[4];              // force sensors
		std::array<int16_t, 4> footForce;
		// int16_t footForceEst[4];           // force sensors
		std::array<int16_t, 4> footForceEst;
		uint32_t tick;                     // reference real-time from motion controller (unit: us)
		// uint8_t wirelessRemote[40];        // wireless commands
		std::array<uint8_t, 40> wirelessRemote;
		uint32_t reserve;
		uint32_t crc;
	} LowState;                            // low level feedback

	typedef struct 
	{
		uint8_t levelFlag;
		uint16_t commVersion;
		uint16_t robotID;
		uint32_t SN;
		uint8_t bandWidth;
		// MotorCmd motorCmd[20];
		std::array<MotorCmd, 20> motorCmd;
		// LED led[4];
		std::array<LED, 4> led;
		// uint8_t wirelessRemote[40];
		std::array<uint8_t, 40> wirelessRemote;
		uint32_t reserve;
		uint32_t crc;
	} LowCmd;                              // low level control

	typedef struct
	{
        uint8_t levelFlag;
        uint16_t commVersion;
        uint16_t robotID;
        uint32_t SN;
        uint8_t bandWidth;
        uint8_t mode;
        IMU imu;
        // float position[3];                 // (unit: m), from robot own odometry in inertial frame, usually drift
		std::array<float, 3> position;
        // float velocity[3];                 // (unit: m/s), forwardSpeed, sideSpeed, updownSpeed in body frame
		std::array<float, 3> velocity;
        float yawSpeed;                    // (unit: rad/s), rotateSpeed in body frame.
        // Cartesian footPosition2Body[4];    // foot position relative to body
		std::array<Cartesian, 4> footPosition2Body;
        // Cartesian footSpeed2Body[4];       // foot speed relative to body
		std::array<Cartesian, 4> footSpeed2Body;
        // int16_t footForce[4];
		std::array<int16_t, 4> footForce;
        // uint8_t wirelessRemote[40];
		std::array<uint8_t, 40> wirelessRemote;
        uint32_t reserve;
        uint32_t crc;
	} HighState;                           // high level feedback

	typedef struct
	{
        uint8_t levelFlag;
        uint16_t commVersion;
        uint16_t robotID;
        uint32_t SN;
        uint8_t bandWidth;
        uint8_t mode;                      // 0.idle, default stand | 1.force stand (controlled by dBodyHeight + rpy)
                                           // 2.target velocity walking (controlled by velocity + yawSpeed)
                                           // 3.target position walking (controlled by position + rpy[2])
                                           // 4. path mode walking (reserve for future release)
                                           // 5. position stand down. |6. position stand up |7. damping mode | 8. recovery mode
        uint8_t gaitType;                  // 0.trot | 1. trot running  | 2.climb stair
        uint8_t speedLevel;                // 0. default low speed. 1. medium speed 2. high speed. during walking
        float dFootRaiseHeight;            // (unit: m), swing foot height adjustment from default swing height.
        float dBodyHeight;                 // (unit: m), body height adjustment from default body height.
        // float position[2];                 // (unit: m), desired x and y position in inertial frame.
	    std::array<float, 2> position;
		// float rpy[3];                      // (unit: rad), desired yaw-pitch-roll euler angle, expressed in roll(rpy[0]) pitch(rpy[1]) yaw(rpy[2])
        std::array<float, 3> rpy;
		// float velocity[2];                 // (unit: m/s), forwardSpeed, sideSpeed in body frame.
        std::array<float, 2> velocity;
		float yawSpeed;                    // (unit: rad/s), rotateSpeed in body frame.
        // LED led[4];
        std::array<LED, 4> led;
		// uint8_t wirelessRemote[40];
		std::array<uint8_t, 40> wirelessRemote;
        uint32_t reserve;
        uint32_t crc;
    } HighCmd;                             // high level control         uint8_t mode

#pragma pack()

	typedef struct     
	{
		unsigned long long TotalCount;     // total loop count
		unsigned long long SendCount;      // total send count
		unsigned long long RecvCount;      // total receive count
		unsigned long long SendError;      // total send error 
		unsigned long long FlagError;      // total flag error 
		unsigned long long RecvCRCError;   // total reveive CRC error	
		unsigned long long RecvLoseError;  // total lose package count	
	} UDPState;                            // UDP communication state

	constexpr int HIGH_CMD_LENGTH   = (sizeof(HighCmd));
	constexpr int HIGH_STATE_LENGTH = (sizeof(HighState));
	
}

#endif
