#ifndef IOROS2_H
#define IOROS2_H

#include "interface/IOInterface.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>
#include <mutex>
#include <thread>
#include <set>
#include <linux/joystick.h>

#define CASBOT_NUM_MOTOR 25

class IOROS2 : public IOInterface
{
public:
    IOROS2();
    ~IOROS2();
    void sendRecv(const LowlevelCmd *cmd, LowlevelState *state) override;

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    rclcpp::Node::SharedPtr _node;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _jointStateSub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imuSub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _jointCmdPub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _pdGainsPub;

    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> _executor;
    std::thread _spinThread;
    std::thread _directJoyThread;
    void directJoystickLoop();

    LowlevelState _lowState;
    std::mutex _stateMutex;

    UserCommand _userCmd;
    UserValue _userValue;
    std::mutex _gamepadMutex;

    int _counter;
    bool _running;
};

#endif  // IOROS2_H
