/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/joystick.h"
#include <math.h>
#include <iostream>
#include <unistd.h>

using namespace UNITREE_LEGGED_SDK;

#define LOCAL 8090
#define TARGET 8007

class Custom
{
public:
  Custom(uint8_t level) : safe(LeggedType::Go1),
                          udp(level, LOCAL, "192.168.123.10", TARGET) //IP and port may need to change? not sure.. example_joystick uses this one but example_walk uses another
  {
    udp.InitCmdData(cmd);
  }
  void UDPSend();
  void UDPRecv();
  void RobotControl();
  void UserInput();

  Safety safe;
  UDP udp;
  HighCmd cmd = {0};
  HighState state = {0};
  xRockerBtnDataStruct _keyData;
  xRockerBtnDataStruct _joystickOut;
  int motiontime = 0;
  string input = "";
  float dt = 0.002; // 0.001~0.01
};

void Custom::UDPRecv()
{
  udp.Recv();
}

void Custom::UDPSend()
{
  udp.Send();
}

void Custom::UserInput()
{
  std::cin >> input;
  if(input == "1")
  {
      string message = (_joystickOut.btn.components.select == 1) ? "Shifting into bendy mode!" : "Releasing select!";
      std::cout << message << std::endl;
      _joystickOut.btn.components.select = !(_joystickOut.btn.components.select);
  }
  else if(input == "2")
  {
      string message = (_joystickOut.btn.components.start == 1) ? "Shifting into walk mode!" : "Releasing start!";
      std::cout << message << std::endl;
      _joystickOut.btn.components.start = !(_joystickOut.btn.components.start);
  }
  else if(input == "w")
  {
      _joystickOut.ry += 0.05;
      std::cout << "Pushing the stick forward. ry = " << _joystickOut.ry << std::endl;
  }
  else if(input == "a")
  {
      _joystickOut.rx -= 0.05;
      std::cout << "Pushing the stick left. rx = " << _joystickOut.rx << std::endl;
  }
  else if(input == "d")
  {
      _joystickOut.rx += 0.05;
      std::cout << "Pushing the stick right. rx = " << _joystickOut.rx << std::endl;
  }
  else if(input == "s")
  {
      _joystickOut.ry = 0;
      _joystickOut.rx = 0;
      std::cout << "Resetting ry and rx to 0!" << std::endl;
  }
}

void Custom::RobotControl()
{
  motiontime++;
  udp.GetRecv(state);

  memcpy(&_keyData, &state.wirelessRemote[0], 40);
  memcpy(&cmd.wirelessRemote[0], &_joystickOut, 40);

  // if((int)_joystickOut.btn.components.select == 1)
  // {
  //   _joystickOut.btn.components.select = 0;
  //   std::cout << "Turning the bit off after it has been sent" << std::endl;
  // }

  // if((int)_joystickOut.btn.components.start == 1)
  // {
  //   _joystickOut.btn.components.start = 0;
  //   std::cout << "Turning the bit off after it has been sent" << std::endl;
  // }

  if ((int)_keyData.btn.components.A == 1)
  {
    std::cout << "The key A is pressed, and the value of lx is " << _keyData.lx << std::endl;
  }

  udp.SetSend(cmd);
}

int main(void)
{
  std::cout << "Communication level is set to HIGH-level." << std::endl
            << "WARNING: Make sure the robot is standing on the ground." << std::endl
            << "Press Enter to continue..." << std::endl;
  std::cin.ignore();

  Custom custom(HIGHLEVEL);
  LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom));
  LoopFunc loop_input("input_loop", custom.dt, boost::bind(&Custom::UserInput, &custom));
  LoopFunc loop_udpSend("udp_send", custom.dt, 3, boost::bind(&Custom::UDPSend, &custom));
  LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv, &custom));

  loop_udpSend.start();
  loop_udpRecv.start();
  loop_input.start();
  loop_control.start();

  while (1)
  {
    sleep(10);
  };

  return 0;
}
