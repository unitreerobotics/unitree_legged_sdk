/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#ifndef _UNITREE_LEGGED_UDP_H_
#define _UNITREE_LEGGED_UDP_H_

#include "comm.h"
#include "quadruped.h"
#include <pthread.h>

namespace UNITREE_LEGGED_SDK
{

    constexpr int UDP_CLIENT_PORT = 8080;                       // local port
    constexpr int UDP_SERVER_PORT = 8007;                       // target port
    constexpr char UDP_SERVER_IP_BASIC[] = "192.168.123.10";    // target IP address
    constexpr char UDP_SERVER_IP_SPORT[] = "192.168.123.161";   // target IP address

    // Notice: User defined data(like struct) should add crc(4Byte) at the end.
    class UDP {
	public:
        UDP(uint8_t level, HighLevelType highControl = HighLevelType::Basic);  // unitree dafault IP and Port
        UDP(uint16_t localPort, const char* targetIP, uint16_t targetPort, int sendLength, int recvLength, int useTimeOut = -1);
        UDP(uint16_t localPort, int sendLength, int recvLength, bool isServer = false); // as server, client IP and port can change
        ~UDP();
        void InitCmdData(HighCmd& cmd);
        void InitCmdData(LowCmd& cmd);
        void switchLevel(int level);

		int SetSend(HighCmd&);
        int SetSend(LowCmd&);
        int SetSend(char* cmd);
        void GetRecv(HighState&);
        void GetRecv(LowState&);
        void GetRecv(char*);
        int Send();
        int Recv(); // directly save in buffer
        
        UDPState udpState;
        char*    targetIP;
        uint16_t targetPort;
        char*    localIP;
        uint16_t localPort;
        bool accessible;       // can access or not
        int useTimeOut;        // use time out method or not, (unit: ms)
        bool isServer;         // server mode need not connected, use for circumstances that data from internet with changeable IP/port

    private:
        void init(uint16_t localPort, const char* targetIP, uint16_t targetPort);
        
        uint8_t levelFlag = HIGHLEVEL;   // default: high level
        int sockFd;
        bool connected; // udp works with connected, rather than server mode
        int sendLength;
        int recvLength;
        char* recvBuf;
        char* recvSource;
        char* sendBuf;
        int lose_recv;
        pthread_mutex_t sendMut;
        pthread_mutex_t recvMut;
	};

}

#endif
