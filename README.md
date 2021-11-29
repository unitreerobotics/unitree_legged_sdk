The laikago_sdk is mainly used for communication between PC and laikago control board.
It also can be used in other PCs with UDP.

## Dependencies
* [Boost](http://www.boost.org) (version 1.5.4 or higher)
* [CMake](http://www.cmake.org) (version 2.8.3 or higher)
* [LCM](https://lcm-proj.github.io) (version 1.4.0 or higher)
```bash
cd lcm-x.x.x
mkdir build
cd build
cmake ../
make
sudo make install
```

## Build
```bash
mkdir build
cd build
cmake ../
make
```

## Usage
Run examples with 'sudo' for memory locking.

### Only Mini-PC
* Run examples below examples/one_pc.

##### With ROS
* Run lcm server examples/one_pc/multi_process/xxx.
```bash
sudo ./sdk_lcm_server_low
```
* Build laikago_real in catkin workspace.
```bash
roslaunch laikago_real low_client.launch
rosrun laikago_real position_lcm_publisher
```

### Mini-PC + TX2 (or Others, like Xavier)
* Use USB3.0->Ethernet for Mini-PC to extend Ethernet port. 
* Configure IP address for each.
* Run examples/multi_pc/host on TX2.
* Run examples/multi_pc/slave on Mini-PC.
