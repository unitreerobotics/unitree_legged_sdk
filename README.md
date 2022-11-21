# v3.8.1
The unitree_legged_sdk is mainly used for communication between PC and Controller board.
It also can be used in other PCs with UDP.

### Notice
support robot: B1

not support robot: Laikago, Aliengo, A1. (Check release [v3.3.1](https://github.com/unitreerobotics/unitree_legged_sdk/releases/tag/v3.3.1) for support)

### Sport Mode
```bash
Legged_sport    >= v3.24
firmware H0.1.7 >= v0.1.35
         H0.1.9 >= v0.1.35
```

### Dependencies
* [Boost](http://www.boost.org) (version 1.71.0 or higher)
* [CMake](http://www.cmake.org) (version 3.16.3 or higher)
* [g++](https://gcc.gnu.org/) (version 9.4.0 or higher)


### Build
```bash
mkdir build
cd build
cmake ../
make
```

### Run

#### Cpp
Run examples with 'sudo' for memory locking.

#### Python
##### arm
change `sys.path.append('../lib/python/amd64')` to `sys.path.append('../lib/python/arm64')`
