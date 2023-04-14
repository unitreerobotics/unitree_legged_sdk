/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#ifndef _UNITREE_LEGGED_COMM_H_
#define _UNITREE_LEGGED_COMM_H_

#include <stdint.h>
#include <array>

namespace UNITREE_LEGGED_SDK
{

	constexpr int HIGHLEVEL = 0xee;
	constexpr int LOWLEVEL = 0xff;
	constexpr int TRIGERLEVEL = 0xf0;
	constexpr double PosStopF = (2.146E+9f);
	constexpr double VelStopF = (16000.0f);
	extern const int HIGH_CMD_LENGTH;	// sizeof(HighCmd)
	extern const int HIGH_STATE_LENGTH; // sizeof(HighState)
	extern const int LOW_CMD_LENGTH;	// shorter than sizeof(LowCmd),   bytes compressed LowCmd length
	extern const int LOW_STATE_LENGTH;	// shorter than sizeof(LowState), bytes compressed LowState length

#pragma pack(1)

	typedef struct
	{
		uint8_t off; // off 0xA5, WARNING: it will turn off the battery
		std::array<uint8_t, 3> reserve;
	} BmsCmd;

	typedef struct
	{
		uint8_t version_h;
		uint8_t version_l;
		uint8_t bms_status;// 1 represents normal, others represents abnormal
		uint8_t SOC;	 // SOC 0-100%, a int range from 0 ~ 100
		int32_t current; // mA, negtive means discharge
		uint16_t cycle; //it represents how many times battery was fully charged and fully discharged 
		std::array<int8_t, 8> BQ_NTC;	   // x1 degrees centigrade
		std::array<int8_t, 8> MCU_NTC;	   // x1 degrees centigrade
		std::array<uint16_t, 30> cell_vol; // cell voltage mV
										   // std::array<int8_t, 2> BQ_NTC;                  // x1 degrees centigrade
										   // std::array<int8_t, 2> MCU_NTC;                 // x1 degrees centigrade
										   // std::array<uint16_t, 10>  cell_vol;            // cell voltage mV
	} BmsState;

	typedef struct
	{
		float x;
		float y;
		float z;
	} Cartesian;

	typedef struct
	{
		std::array<float, 4> quaternion;	// quaternion, normalized, (w,x,y,z)
		std::array<float, 3> gyroscope;		// angular velocity （unit: rad/s)    (raw data)
		std::array<float, 3> accelerometer; // m/(s2)                             (raw data)
		std::array<float, 3> rpy;			// euler angle（unit: rad)
		int8_t temperature; // celsius, the IMU's temperature
	} IMU; // when under accelerated motion, the attitude of the robot calculated by IMU will drift.

	typedef struct
	{
		uint8_t r;
		uint8_t g;
		uint8_t b;
	} LED; // foot led brightness: 0~255, it's useless in B1

	typedef struct
	{
		uint8_t mode; // motor working mode
		float q;	  // current angle (unit: radian)
		float dq;	  // current velocity (unit: radian/second)
		float ddq;	  // current acc (unit: radian/second*second)
		float tauEst; // current estimated output torque (unit: N.m)
		float q_raw;  // current angle (unit: radian)
		float dq_raw; // current velocity (unit: radian/second)
		float ddq_raw; //current accleration (unit: radian/second*second)
		int8_t temperature; // current temperature (temperature conduction is slow that leads to lag)
		std::array<uint32_t, 2> reserve;
	} MotorState; // motor feedback

	typedef struct
	{
		uint8_t mode; // desired working mode
		float q;	  // desired angle (unit: radian)
		float dq;	  // desired velocity (unit: radian/second)
		float tau;	  // desired output torque (unit: N.m)
		float Kp;	  // desired position stiffness (unit: N.m/rad )
		float Kd;	  // desired velocity stiffness (unit: N.m/(rad/s) )
		std::array<uint32_t, 3> reserve;
	} MotorCmd; // motor control

	typedef struct
	{
		std::array<uint8_t, 2> head;
		uint8_t levelFlag;
		uint8_t frameReserve;

		std::array<uint32_t, 2> SN;
		std::array<uint32_t, 2> version;
		uint16_t bandWidth;
		IMU imu;
		std::array<MotorState, 20> motorState;
		BmsState bms;
		std::array<int16_t, 4> footForce;		// reserve
		std::array<int16_t, 4> footForceEst;	// reserve
		uint32_t tick;							// reference real-time from motion controller (unit: ms)
		std::array<uint8_t, 40> wirelessRemote; // wireless commands
		uint32_t reserve;

		uint32_t crc;
	} LowState; // low level feedback

	typedef struct
	{
		std::array<uint8_t, 2> head;
		uint8_t levelFlag;
		uint8_t frameReserve;

		std::array<uint32_t, 2> SN;
		std::array<uint32_t, 2> version;
		uint16_t bandWidth;
		std::array<MotorCmd, 20> motorCmd;
		BmsCmd bms;
		std::array<uint8_t, 40> wirelessRemote;
		uint32_t reserve;

		uint32_t crc;
	} LowCmd; // low level control

	typedef struct
	{
		std::array<uint8_t, 2> head;
		uint8_t levelFlag;
		uint8_t frameReserve;

		std::array<uint32_t, 2> SN;
		std::array<uint32_t, 2> version;
		uint16_t bandWidth;
		IMU imu;
		std::array<MotorState, 20> motorState;
		BmsState bms;
		std::array<int16_t, 4> footForce; // foot force in z axis, which is parallel to gravity (unit: N)
		std::array<int16_t, 4> footForceEst; // foot force in z axis, which is parallel to gravity (unit: N)
		uint8_t mode; //current mode, more detail in HighCmd comment
		float progress; // reserve
		uint8_t gaitType;			   // 0.idle  1.trot  2.trot running  3.climb stair  4.trot obstacle
		float footRaiseHeight;		   // (unit: m, default: 0.15m), foot up height while walking
		std::array<float, 3> position; // (unit: m), from own odometry in inertial frame, usually drift
		float bodyHeight;			   // (unit: m, default: 0.58m),
		std::array<float, 3> velocity; // (unit: m/s), forwardSpeed, sideSpeed, rotateSpeed in body frame
		float yawSpeed;				   // (unit: rad/s), rotateSpeed in body frame
		std::array<float, 4> rangeObstacle; //reserve
		std::array<Cartesian, 4> footPosition2Body; // foot position relative to body
		std::array<Cartesian, 4> footSpeed2Body;	// foot speed relative to body
		std::array<uint8_t, 40> wirelessRemote;
		uint32_t reserve;

		uint32_t crc;
	} HighState; // high level feedback, update frequency: 400Hz

	typedef struct
	{
		std::array<uint8_t, 2> head;
		uint8_t levelFlag;
		uint8_t frameReserve;

		std::array<uint32_t, 2> SN;
		std::array<uint32_t, 2> version;
		uint16_t bandWidth;
		uint8_t mode; // 0. idle, default stand  1. force stand (controlled by dBodyHeight + ypr)
					  // 2. target velocity walking (controlled by velocity + yawSpeed)
					  // 4. path mode walking (reserve for future release)
					  // 5. position stand down.
					  // 6. position stand up
					  // 7. damping mode
					  // 9. recovery stand


		uint8_t gaitType;			   // 0.idle  1.trot  2.trot running  3.climb stair  4.trot obstacle
		uint8_t speedLevel;			   // 0. default low speed. 1. medium speed 2. high speed. during walking, only respond MODE 3
		float footRaiseHeight;		   // (unit: m, default: 0.15m), foot up height while walking, delta value, range:[-0.1, 0.15]
		float bodyHeight;			   // (unit: m, default: 0.58m), delta value, range:[-0.16, 0.16]
		std::array<float, 2> position; // reserve
		std::array<float, 3> euler;	   // (unit: rad), roll pitch yaw in stand mode, roll range:[-0.3, 0.3], pitch range:[-0.3, 0.3], yaw range:[-0.6, 0.6]
		std::array<float, 2> velocity; // (unit: m/s), forwardSpeed, sideSpeed in body frame, forwardSpeed range:[-0.8, 1.2], sideSpeed range: [-0.25, 0.25]
		float yawSpeed;				   // (unit: rad/s), rotateSpeed in body frame, range:[-0.75, 0.75]
		std::array<float, 2> dComXy;   //reserve
		std::array<float, 2> dstandFootXy; //reserve
		BmsCmd bms;
		std::array<LED, 4> led;        //reserve
		std::array<uint8_t, 40> wirelessRemote;
		uint32_t reserve;

		uint32_t crc;
	} HighCmd; // high level control

#pragma pack()

	typedef struct
	{
		unsigned long long TotalCount;	  // total loop count
		unsigned long long SendCount;	  // total send count
		unsigned long long RecvCount;	  // total receive count
		unsigned long long SendError;	  // total send error
		unsigned long long FlagError;	  // total flag error
		unsigned long long RecvCRCError;  // total reveive CRC error
		unsigned long long RecvLoseError; // total lose package count
	} UDPState;							  // UDP communication state

}

#endif
