# v3.4.2
The unitree_legged_sdk is mainly used for communication between PC and Controller board.
It also can be used in other PCs with UDP.

### Notice
support robot: Go1

not support robot: Laikago, Aliengo, A1. (Check release [v3.3.1](https://github.com/unitreerobotics/unitree_legged_sdk/releases/tag/v3.3.1) for support)

### Sport Mode
Legged_sport >= v1.32

### Dependencies
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

### Build
```bash
mkdir build
cd build
cmake ../
make
```

### Usage
Run examples with 'sudo' for memory locking.
