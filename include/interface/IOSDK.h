#ifndef IOSDK_H
#define IOSDK_H

#include "interface/IOInterface.h"
#include <string>
#include "common/gamepad.hpp"
// 引入宇树的通信方式，以及消息类型
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
// 初始化话题
static const std::string HG_CMD_TOPIC = "rt/lowcmd";
static const std::string HG_IMU_TORSO = "rt/secondary_imu";
static const std::string HG_STATE_TOPIC = "rt/lowstate";

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

const int G1_NUM_MOTOR = 29;
enum class Mode {
  PR = 0,  // Series Control for Ptich/Roll Joints
  AB = 1   // Parallel Control for A/B Joints
};
// 继承类IOIterface
class IOSDK : public IOInterface
{
private:
    ChannelPublisherPtr<LowCmd_> lowcmd_publisher_;
    ChannelSubscriberPtr<LowState_> lowstate_subscriber_;
    LowlevelCmd _lowCmd;
    LowlevelState _lowState;
    // 变量rx_是一个结构体，其包含40个字节缓冲区去获得机器人的状态的数据，以及RF_RX是解析遥控器数据后的存放的空间
    REMOTE_DATA_RX rx_;
    // 遥控器状态解析器，内部存储所有按键/摇杆的当前状态。比如
    //按键是否按下：gamepad_.start.pressed、gamepad_.A.pressed、gamepad_.L2.pressed…… 
    //摇杆数值：gamepad_.lx、gamepad_.ly、gamepad_.rx、gamepad_.ry
    Gamepad gamepad_;
    uint8_t mode_machine_;
    int counter_;
    UserCommand userCmd_;
    UserValue userValue_;

    void LowStateHandler(const void *message);

public:
    IOSDK(/* args */);
    ~IOSDK(){}
    void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);
};






#endif //IOSDK_H