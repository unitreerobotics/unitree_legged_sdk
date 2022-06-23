#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/joystick.h"
#include <array>
#include <math.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h> 

using namespace UNITREE_LEGGED_SDK;
namespace py = pybind11;

class LowInterface {
public:
    LowInterface(): safe(LeggedType::Go1), udp(LOWLEVEL){
        udp.InitCmdData(cmd);
    }
    void UDPRecv(){
        udp.Recv();
    }
    void UDPSend(){
        udp.Send();
    }
    LowState RecvObservation();
    py::array_t<double, 12> GetMotorPosition(LowState _state); // current angle (unit: radian)
    py::array_t<double, 12> GetMotorVelocity(LowState _state); // current velocity (unit: radian/second)
    py::array_t<double, 12> GetMotorAcceleration(LowState _state); // current acc (unit: radian/second^2)
    py::array_t<double, 12> GetMotorTorque(LowState _state);   // current estimated output torque (unit: N.m)
    py::array_t<double, 36> GetJointStates(LowState _state); 
    py::array_t<double, 4> GetQuaternion(LowState _state);     // quaternion, normalized, (w,x,y,z)
    py::array_t<double, 3> GetGyroscope(LowState _state);      // angular velocity （unit: rad/s)   (raw data)
    py::array_t<double, 3> GetAccelerometer(LowState _state);  // m/(s^2)                           (raw data)
    py::array_t<double, 3> GetAngle(LowState _state);          // euler angle（unit: rad)
    py::array_t<double, 3> CalcLinearVel(LowState _state);
    xRockerBtnDataStruct GetKeyInput();
    void ResetCommand();
    void SetCommand(std::array<float, 60> motorcmd);
    void SendCommand();

    Safety safe;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    xRockerBtnDataStruct _keyData;
    float dt = 0.002;     // 0.001~0.01
};

LowState LowInterface::RecvObservation() {
    udp.Recv(); 
    udp.GetRecv(state);
    memcpy(&_keyData, state.wirelessRemote.data(), 40);
    return state;
}

py::array_t<double, 12> LowInterface::GetMotorPosition(LowState _state){
    std::array<float, 12> angle;
    for (int motor_id = 0; motor_id < 12; motor_id++) {
        angle[motor_id] = _state.motorState[motor_id].q;
    }
    py::array_t<double, 12> ret =  py::cast(angle);
    return ret;
}

py::array_t<double, 12> LowInterface::GetMotorVelocity(LowState _state){
    std::array<float, 12> velocity;
    for (int motor_id = 0; motor_id < 12; motor_id++) {
        velocity[motor_id] = _state.motorState[motor_id].dq;
    }
    py::array_t<double, 12> ret =  py::cast(velocity);
    return ret;
}

py::array_t<double, 12> LowInterface::GetMotorAcceleration(LowState _state){
    std::array<float, 12> acc;
    for (int motor_id = 0; motor_id < 12; motor_id++) {
        acc[motor_id] = _state.motorState[motor_id].ddq;
    }
    py::array_t<double, 12> ret =  py::cast(acc);
    return ret;
}

py::array_t<double, 12> LowInterface::GetMotorTorque(LowState _state){
    std::array<float, 12> torque;
    for (int motor_id = 0; motor_id < 12; motor_id++) {
        torque[motor_id] = _state.motorState[motor_id].tauEst;
    }
    py::array_t<double, 12> ret =  py::cast(torque);
    return ret;
}

py::array_t<double, 36> LowInterface::GetJointStates(LowState _state){
    std::array<float, 36> joint_states;
    for (int motor_id = 0; motor_id < 12; motor_id++) {
        joint_states[motor_id] = _state.motorState[motor_id].q;
        joint_states[motor_id+12] = _state.motorState[motor_id].dq;
        joint_states[motor_id+24] = _state.motorState[motor_id].tauEst;
    }
    py::array_t<double, 36> ret =  py::cast(joint_states);
    return ret;
}

py::array_t<double, 4> LowInterface::GetQuaternion(LowState _state){
    py::array_t<double, 4> ret =  py::cast(_state.imu.quaternion);
    return ret;
}

py::array_t<double, 3> LowInterface::GetGyroscope(LowState _state){
    py::array_t<double, 3> ret =  py::cast(_state.imu.gyroscope);
    return ret;
}

py::array_t<double, 3> LowInterface::GetAccelerometer(LowState _state){
    py::array_t<double, 3> ret =  py::cast(_state.imu.accelerometer);
    return ret;
}   

py::array_t<double, 3> LowInterface::GetAngle(LowState _state){
    py::array_t<double, 3> ret =  py::cast(_state.imu.rpy);
    return ret;
}

xRockerBtnDataStruct LowInterface::GetKeyInput(){
    return _keyData;
}

void LowInterface::ResetCommand() {
    for (int motor_id = 0; motor_id < 12; motor_id++) {
        cmd.motorCmd[motor_id].mode = 0x0A;
        cmd.motorCmd[motor_id].q = 0.0;
        cmd.motorCmd[motor_id].dq = 0.0;
        cmd.motorCmd[motor_id].tau = 0.0;
        cmd.motorCmd[motor_id].Kp = 0.0;
        cmd.motorCmd[motor_id].Kd = 0.0;
    }
}

void LowInterface::SetCommand(std::array<float, 60> motorcmd){
    for (int motor_id = 0; motor_id < 12; motor_id++) {
        cmd.motorCmd[motor_id].mode = 0x0A;
        // set position
        cmd.motorCmd[motor_id].q = motorcmd[motor_id];
        // set velocity
        cmd.motorCmd[motor_id].dq = motorcmd[motor_id+12];
        // set torque
        cmd.motorCmd[motor_id].tau = motorcmd[motor_id+24];
        // set position stiffness
        cmd.motorCmd[motor_id].Kp = motorcmd[motor_id+36];
        // set velocity stiffness
        cmd.motorCmd[motor_id].Kd = motorcmd[motor_id+48];
    }
}

void LowInterface::SendCommand() {
    cmd.levelFlag = LOWLEVEL;
    safe.PositionLimit(cmd);
    int res1 = safe.PowerProtect(cmd, state, 1);
    if(res1 < 0) exit(-1);
    udp.SetSend(cmd);
    udp.Send();
}


class HighInterface {
public:
    HighInterface(): safe(LeggedType::Go1),
    udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState)){
        udp.InitCmdData(cmd);
    }
    void UDPRecv(){
        udp.Recv();
    }
    void UDPSend(){
        udp.Send();
    }
    HighState RecvObservation();
    py::array_t<double, 12> GetMotorPosition(HighState _state);
    py::array_t<double, 12> GetMotorVelocity(HighState _state);
    py::array_t<double, 12> GetMotorAcceleration(HighState _state);
    py::array_t<double, 12> GetMotorTorque(HighState _state);
    py::array_t<double, 36> GetJointStates(HighState _state); 
    py::array_t<double, 4> GetQuaternion(HighState _state);     // quaternion, normalized, (w,x,y,z)
    py::array_t<double, 3> GetGyroscope(HighState _state);      // angular velocity （unit: rad/s)   (raw data)
    py::array_t<double, 3> GetAccelerometer(HighState _state);  // m/(s^2)                           (raw data)
    py::array_t<double, 3> GetAngle(HighState _state);   // euler angle（unit: rad)
    xRockerBtnDataStruct GetKeyInput();
   
    void ResetCommand();
    void SetCommand(std::array<float, 11> highcmd);
    //     uint8_t mode=0, uint8_t gaitType=0, uint8_t speedLevel=0, float footRaiseHeight=0.0f, float bodyHeight=0.0f, 
    //     std::array<float, 3> euler={0}, std::array<float, 2> v={0}, float yawSpeed=0.0f, bool send=true);
    void SendCommand();

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    xRockerBtnDataStruct _keyData;
    float dt = 0.002;     // 0.001~0.01
};

HighState HighInterface::RecvObservation() {
    udp.Recv(); 
    udp.GetRecv(state);
    memcpy(&_keyData, state.wirelessRemote.data(), 40);
    return state;
}

py::array_t<double, 12> HighInterface::GetMotorPosition(HighState _state){
    std::array<float, 12> angle;
    for (int motor_id = 0; motor_id < 12; motor_id++) {
        angle[motor_id] = _state.motorState[motor_id].q;
    }
    py::array_t<double, 12> ret =  py::cast(angle);
    return ret;
}

py::array_t<double, 12> HighInterface::GetMotorVelocity(HighState _state){
    std::array<float, 12> velocity;
    for (int motor_id = 0; motor_id < 12; motor_id++) {
        velocity[motor_id] = _state.motorState[motor_id].dq;
    }
    py::array_t<double, 12> ret =  py::cast(velocity);
    return ret;
}

py::array_t<double, 12> HighInterface::GetMotorAcceleration(HighState _state){
    std::array<float, 12> acc;
    for (int motor_id = 0; motor_id < 12; motor_id++) {
        acc[motor_id] = _state.motorState[motor_id].ddq;
    }
    py::array_t<double, 12> ret =  py::cast(acc);
    return ret;
}

py::array_t<double, 12> HighInterface::GetMotorTorque(HighState _state){
    std::array<float, 12> torque;
    for (int motor_id = 0; motor_id < 12; motor_id++) {
        torque[motor_id] = _state.motorState[motor_id].tauEst;
    }
    py::array_t<double, 12> ret =  py::cast(torque);
    return ret;
}

py::array_t<double, 36> HighInterface::GetJointStates(HighState _state){
    std::array<float, 36> joint_states;
    for (int motor_id = 0; motor_id < 12; motor_id++) {
        joint_states[motor_id] = _state.motorState[motor_id].q;
        joint_states[motor_id+12] = _state.motorState[motor_id].dq;
        joint_states[motor_id+24] = _state.motorState[motor_id].tauEst;
    }
    py::array_t<double, 36> ret =  py::cast(joint_states);
    return ret;
}

py::array_t<double, 4> HighInterface::GetQuaternion(HighState _state){
    py::array_t<double, 4> ret =  py::cast(_state.imu.quaternion);
    return ret;
}

py::array_t<double, 3> HighInterface::GetGyroscope(HighState _state){
    py::array_t<double, 3> ret =  py::cast(_state.imu.gyroscope);
    return ret;
}

py::array_t<double, 3> HighInterface::GetAccelerometer(HighState _state){
    py::array_t<double, 3> ret =  py::cast(_state.imu.accelerometer);
    return ret;
}   

py::array_t<double, 3> HighInterface::GetAngle(HighState _state){
    py::array_t<double, 3> ret =  py::cast(_state.imu.rpy);
    return ret;
}

xRockerBtnDataStruct HighInterface::GetKeyInput(){
    return _keyData;
}

void HighInterface::ResetCommand() {
    cmd.mode = 0;      // 0:idle, default stand      1:forced stand     2:walk continuously
    cmd.gaitType = 0;
    cmd.speedLevel = 0;
    cmd.footRaiseHeight = 0;
    cmd.bodyHeight = 0;
    cmd.euler[0]  = 0;
    cmd.euler[1] = 0;
    cmd.euler[2] = 0;
    cmd.velocity[0] = 0.0f;
    cmd.velocity[1] = 0.0f;
    cmd.yawSpeed = 0.0f;
    cmd.reserve = 0;
}

void HighInterface::SetCommand(std::array<float, 11> highcmd){
    // 0. idle, default stand  1. force stand   (controlled by dBodyHeight + ypr)
    // 2. target velocity walking               (controlled by velocity + yawSpeed)
    // 3. target position walking               (controlled by position + ypr[0])
    // 4. path mode walking                     (reserve for future release)
    // 5. position stand down. 
    // 6. position stand up 
    // 7. damping mode 
    // 8. recovery stand
    // 9. backflip
    // 10. jumpYaw
    // 11. straightHand
    // 12. dance1
    // 13. dance2
    if(0 <= highcmd[0] && highcmd[0] <= 13) {
        cmd.mode = highcmd[0];
    }
    // 0.idle  1.trot  2.trot running  3.climb stair
    if(0 <= highcmd[1] && highcmd[1] <= 3) {
        cmd.gaitType = highcmd[1];
    }     
    // 0. default low speed. 1. medium speed 2. high speed. during walking, only respond MODE 3
    if(0 <= highcmd[2] && highcmd[2] <= 2) {
        cmd.speedLevel = highcmd[2];
    }    
    cmd.footRaiseHeight = highcmd[3];
    cmd.bodyHeight = highcmd[4];
    cmd.euler[0] = highcmd[5];
    cmd.euler[1] = highcmd[6];
    cmd.euler[2] = highcmd[7];
    cmd.velocity[0] = highcmd[8];
    cmd.velocity[1] = highcmd[9];
    cmd.yawSpeed = highcmd[10];
    cmd.reserve = 0;
}

void HighInterface::SendCommand() {
    cmd.levelFlag = HIGHLEVEL;
    udp.SetSend(cmd);
    udp.Send();
}


PYBIND11_MODULE(robot_interface, m) {
    m.doc() = R"pbdoc(
          Go1 Robot Interface Python Bindings
          -----------------------
          .. currentmodule:: go1_robot_interface
          .. autosummary::
             :toctree: _generate
      )pbdoc";

    m.attr("HIGHLEVEL") = py::int_(HIGHLEVEL);
    m.attr("LOWLEVEL") = py::int_(LOWLEVEL);
    m.attr("TRIGERLEVEL") = py::int_(TRIGERLEVEL);
    m.attr("PosStopF") = py::float_(PosStopF);
    m.attr("VelStopF") = py::float_(VelStopF);
    m.attr("HIGH_CMD_LENGTH") = py::int_(HIGH_CMD_LENGTH);
    m.attr("HIGH_STATE_LENGTH") = py::int_(HIGH_STATE_LENGTH);
    m.attr("LOW_CMD_LENGTH") = py::int_(LOW_CMD_LENGTH);
    m.attr("LOW_STATE_LENGTH") = py::int_(LOW_STATE_LENGTH);
    m.attr("FR_0") = py::int_(FR_0);
    m.attr("FR_1") = py::int_(FR_1);
    m.attr("FR_2") = py::int_(FR_2);
    m.attr("FL_0") = py::int_(FL_0);
    m.attr("FL_1") = py::int_(FL_1);
    m.attr("FL_2") = py::int_(FL_2);
    m.attr("RR_0") = py::int_(RR_0);
    m.attr("RR_1") = py::int_(RR_1);
    m.attr("RR_2") = py::int_(RR_2);
    m.attr("RL_0") = py::int_(RL_0);
    m.attr("RL_1") = py::int_(RL_1);
    m.attr("RL_2") = py::int_(RL_2);

    py::class_<BmsCmd>(m, "BmsCmd")
        .def(py::init<>())
        .def_readwrite("off", &BmsCmd::off)
        .def_readwrite("reserve", &BmsCmd::reserve);

    py::class_<BmsState>(m, "BmsState")
        .def(py::init<>())
        .def_readwrite("version_h", &BmsState::version_h)
        .def_readwrite("version_l", &BmsState::version_l)
        .def_readwrite("bms_status", &BmsState::bms_status)
        .def_readwrite("SOC", &BmsState::SOC)
        .def_readwrite("current", &BmsState::current)
        .def_readwrite("cycle", &BmsState::cycle)
        .def_readwrite("BQ_NTC", &BmsState::BQ_NTC)
        .def_readwrite("MCU_NTC", &BmsState::MCU_NTC)
        .def_readwrite("cell_vol", &BmsState::cell_vol);

    py::class_<Cartesian>(m, "Cartesian")
        .def(py::init<>())
        .def_readwrite("x", &Cartesian::x)
        .def_readwrite("y", &Cartesian::y)
        .def_readwrite("z", &Cartesian::z)
        .def("__repr__", [](const Cartesian& v){
                return cartesian_repr(v);
            });

    py::class_<IMU>(m, "IMU")
        .def(py::init<>())
        .def_readwrite("quaternion", &IMU::quaternion)
        .def_readwrite("gyroscope", &IMU::gyroscope)
        .def_readwrite("accelerometer", &IMU::accelerometer)
        .def_readwrite("rpy", &IMU::rpy)
        .def_readwrite("temperature", &IMU::temperature)
        .def("__repr__", [](const IMU& v){
                return imu_repr(v);
            });

    py::class_<LED>(m, "LED")
        .def(py::init<>())
        .def_readwrite("r", &LED::r)
        .def_readwrite("g", &LED::g)
        .def_readwrite("b", &LED::b);

    py::class_<MotorState>(m, "MotorState")
        .def(py::init<>())
        .def_readwrite("mode", &MotorState::mode)
        .def_readwrite("q", &MotorState::q)
        .def_readwrite("dq", &MotorState::dq)
        .def_readwrite("ddq", &MotorState::ddq)
        .def_readwrite("tauEst", &MotorState::tauEst)
        .def_readwrite("q_raw", &MotorState::q_raw)
        .def_readwrite("dq_raw", &MotorState::dq_raw)
        .def_readwrite("ddq_raw", &MotorState::ddq_raw)
        .def_readwrite("temperature", &MotorState::temperature)
        .def_readwrite("reserve", &MotorState::reserve);

    py::class_<MotorCmd>(m, "MotorCmd")
        .def(py::init<>())
        .def_readwrite("mode", &MotorCmd::mode)
        .def_readwrite("q", &MotorCmd::q)
        .def_readwrite("dq", &MotorCmd::dq)
        .def_readwrite("tau", &MotorCmd::tau)
        .def_readwrite("Kp", &MotorCmd::Kp)
        .def_readwrite("Kd", &MotorCmd::Kd)
        .def_readwrite("reserve", &MotorCmd::reserve);

    py::class_<LowState>(m, "LowState")
        .def(py::init<>())
        .def_readwrite("head", &LowState::head)
        .def_readwrite("levelFlag", &LowState::levelFlag)
        .def_readwrite("frameReserve", &LowState::frameReserve)
        .def_readwrite("SN", &LowState::SN)
        .def_readwrite("version", &LowState::version)
        .def_readwrite("bandWidth", &LowState::bandWidth)
        .def_readwrite("imu", &LowState::imu)
        .def_readwrite("motorState", &LowState::motorState)
        .def_readwrite("bms", &LowState::bms)
        .def_readwrite("footForce", &LowState::footForce)
        .def_readwrite("footForceEst", &LowState::footForceEst)
        .def_readwrite("tick", &LowState::tick)
        .def_readwrite("wirelessRemote", &LowState::wirelessRemote)
        .def_readwrite("reserve", &LowState::reserve)
        .def_readwrite("crc", &LowState::crc);

    py::class_<LowCmd>(m, "LowCmd")
        .def(py::init<>())
        .def_readwrite("head", &LowCmd::head)
        .def_readwrite("levelFlag", &LowCmd::levelFlag)
        .def_readwrite("frameReserve", &LowCmd::frameReserve)
        .def_readwrite("SN", &LowCmd::SN)
        .def_readwrite("version", &LowCmd::version)
        .def_readwrite("bandWidth", &LowCmd::bandWidth)
        .def_readwrite("motorCmd", &LowCmd::motorCmd)
        .def_readwrite("bms", &LowCmd::bms)
        .def_readwrite("wirelessRemote", &LowCmd::wirelessRemote)
        .def_readwrite("reserve", &LowCmd::reserve)
        .def_readwrite("crc", &LowCmd::crc);

    py::class_<HighState>(m, "HighState")
        .def(py::init<>())
        .def_readwrite("head", &HighState::head)
        .def_readwrite("levelFlag", &HighState::levelFlag)
        .def_readwrite("frameReserve", &HighState::frameReserve)
        .def_readwrite("SN", &HighState::SN)
        .def_readwrite("version", &HighState::version)
        .def_readwrite("bandWidth", &HighState::bandWidth)
        .def_readwrite("imu", &HighState::imu)
        .def_readwrite("motorState", &HighState::motorState)
        .def_readwrite("bms", &HighState::bms)
        .def_readwrite("footForce", &HighState::footForce)
        .def_readwrite("footForceEst", &HighState::footForceEst)
        .def_readwrite("mode", &HighState::mode)
        .def_readwrite("progress", &HighState::progress)
        .def_readwrite("gaitType", &HighState::gaitType)
        .def_readwrite("footRaiseHeight", &HighState::footRaiseHeight)
        .def_readwrite("position", &HighState::position)
        .def_readwrite("bodyHeight", &HighState::bodyHeight)
        .def_readwrite("velocity", &HighState::velocity)
        .def_readwrite("yawSpeed", &HighState::yawSpeed)
        .def_readwrite("rangeObstacle", &HighState::rangeObstacle)
        .def_readwrite("footPosition2Body", &HighState::footPosition2Body)
        .def_readwrite("footSpeed2Body", &HighState::footSpeed2Body)
        .def_readwrite("wirelessRemote", &HighState::wirelessRemote)
        .def_readwrite("reserve", &HighState::reserve)
        .def_readwrite("crc", &HighState::crc);

    py::class_<HighCmd>(m, "HighCmd")
        .def(py::init<>())
        .def_readwrite("head", &HighCmd::head)
        .def_readwrite("levelFlag", &HighCmd::levelFlag)
        .def_readwrite("frameReserve", &HighCmd::frameReserve)
        .def_readwrite("SN", &HighCmd::SN)
        .def_readwrite("version", &HighCmd::version)
        .def_readwrite("bandWidth", &HighCmd::bandWidth)
        .def_readwrite("mode", &HighCmd::mode)
        .def_readwrite("gaitType", &HighCmd::gaitType)
        .def_readwrite("speedLevel", &HighCmd::speedLevel)
        .def_readwrite("footRaiseHeight", &HighCmd::footRaiseHeight)
        .def_readwrite("bodyHeight", &HighCmd::bodyHeight)
        .def_readwrite("postion", &HighCmd::position)
        .def_readwrite("euler", &HighCmd::euler)
        .def_readwrite("velocity", &HighCmd::velocity)
        .def_readwrite("yawSpeed", &HighCmd::yawSpeed)
        .def_readwrite("bms", &HighCmd::bms)
        .def_readwrite("led", &HighCmd::led)
        .def_readwrite("wirelessRemote", &HighCmd::wirelessRemote)
        .def_readwrite("reserve", &HighCmd::reserve)
        .def_readwrite("crc", &HighCmd::crc);

    py::class_<UDPState>(m, "UDPState")
        .def(py::init<>())
        .def_readwrite("TotalCount", &UDPState::TotalCount)
        .def_readwrite("SendCount", &UDPState::SendCount)
        .def_readwrite("RecvCount", &UDPState::RecvCount)
        .def_readwrite("SendError", &UDPState::SendError)
        .def_readwrite("FlagError", &UDPState::FlagError)
        .def_readwrite("RecvCRCError", &UDPState::RecvCRCError)
        .def_readwrite("RecvLoseError", &UDPState::RecvLoseError);

    py::class_<LowInterface>(m, "LowInterface")
        .def(py::init<>())
        .def("udp_recv", &LowInterface::UDPRecv)
        .def("udp_send", &LowInterface::UDPSend)
        .def("recv_observation", &LowInterface::RecvObservation)
        .def("get_joint_pos", &LowInterface::GetMotorPosition)
        .def("get_joint_vel", &LowInterface::GetMotorVelocity)
        .def("get_joint_acc", &LowInterface::GetMotorAcceleration)
        .def("get_joint_tau", &LowInterface::GetMotorTorque)
        .def("get_joint_states", &LowInterface::GetJointStates)
        .def("get_quaternion", &LowInterface::GetQuaternion)
        .def("get_gyroscope", &LowInterface::GetGyroscope)
        .def("get_accelerometer", &LowInterface::GetAccelerometer)
        .def("get_angle", &LowInterface::GetAngle)
        .def("get_key_input", &LowInterface::GetKeyInput)
        .def("set_command", &LowInterface::SetCommand)
        .def("reset_command", &LowInterface::ResetCommand)
        .def("send_command", &LowInterface::SendCommand);

    py::class_<HighInterface>(m, "HighInterface")
        .def(py::init<>())
        .def("udp_recv", &HighInterface::UDPRecv)
        .def("udp_send", &HighInterface::UDPSend)
        .def("recv_observation", &HighInterface::RecvObservation)
        .def("get_joint_pos", &HighInterface::GetMotorPosition)
        .def("get_joint_vel", &HighInterface::GetMotorVelocity)
        .def("get_joint_acc", &HighInterface::GetMotorAcceleration)
        .def("get_joint_tau", &HighInterface::GetMotorTorque)
        .def("get_joint_states", &HighInterface::GetJointStates)
        .def("get_quaternion", &HighInterface::GetQuaternion)
        .def("get_gyroscope", &HighInterface::GetGyroscope)
        .def("get_accelerometer", &HighInterface::GetAccelerometer)
        .def("get_angle", &HighInterface::GetAngle)
        .def("get_key_input", &HighInterface::GetKeyInput)
        .def("reset_command", &HighInterface::ResetCommand)
        .def("set_command", &HighInterface::SetCommand)
        .def("send_command", &HighInterface::SendCommand);

    #ifdef VERSION_INFO
      m.attr("__version__") = VERSION_INFO;
    #else
      m.attr("__version__") = "dev";
    #endif
      m.attr("TEST") = py::int_(int(42));

}
