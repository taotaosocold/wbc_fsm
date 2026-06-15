#include "interface/IOROS2.h"
#include <iostream>
#include <cstring>
#include <chrono>
#include <algorithm>

IOROS2::IOROS2()
{
    _node = std::make_shared<rclcpp::Node>("casbot_fsm_io");

    _jointCmdPub = _node->create_publisher<sensor_msgs::msg::JointState>("/motion/joint_cmd", 10);

    _jointStateSub = _node->create_subscription<sensor_msgs::msg::JointState>(
        "/motion/joint_state", 10,
        std::bind(&IOROS2::jointStateCallback, this, std::placeholders::_1));

    _imuSub = _node->create_subscription<sensor_msgs::msg::Imu>(
        "/motion/imu", 10,
        std::bind(&IOROS2::imuCallback, this, std::placeholders::_1));

    _joySub = _node->create_subscription<crb_ros_msg::msg::JoystickCmdReport>(
        "/joystick_events", 10,
        std::bind(&IOROS2::joyCallback, this, std::placeholders::_1));

    _counter = 0;
    _userCmd = UserCommand::NONE;
    _userValue.setZero();
    _running = true;

    _executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    _executor->add_node(_node);

    _spinThread = std::thread([this]() {
        while (_running && rclcpp::ok()) {
            _executor->spin_some();
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    });

    std::cout << "[IOROS2] Initialized. Topics:" << std::endl;
    std::cout << "  Sub: /motion/joint_state" << std::endl;
    std::cout << "  Sub: /motion/imu" << std::endl;
    std::cout << "  Sub: /joystick_events" << std::endl;
    std::cout << "  Pub: /motion/joint_cmd" << std::endl;
}

IOROS2::~IOROS2()
{
    _running = false;
    if (_spinThread.joinable()) {
        _spinThread.join();
    }
}

void IOROS2::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(_stateMutex);

    for (size_t i = 0; i < CASBOT_NUM_MOTOR && i < msg->position.size(); i++) {
        _lowState.motorState[i].q = static_cast<float>(msg->position[i]);
    }
    for (size_t i = 0; i < CASBOT_NUM_MOTOR && i < msg->velocity.size(); i++) {
        _lowState.motorState[i].dq = static_cast<float>(msg->velocity[i]);
    }
    for (size_t i = 0; i < CASBOT_NUM_MOTOR && i < msg->effort.size(); i++) {
        _lowState.motorState[i].tauEst = static_cast<float>(msg->effort[i]);
    }
}

void IOROS2::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(_stateMutex);

    _lowState.imu.quaternion[0] = static_cast<float>(msg->orientation.w);
    _lowState.imu.quaternion[1] = static_cast<float>(msg->orientation.x);
    _lowState.imu.quaternion[2] = static_cast<float>(msg->orientation.y);
    _lowState.imu.quaternion[3] = static_cast<float>(msg->orientation.z);

    _lowState.imu.gyroscope[0] = static_cast<float>(msg->angular_velocity.x);
    _lowState.imu.gyroscope[1] = static_cast<float>(msg->angular_velocity.y);
    _lowState.imu.gyroscope[2] = static_cast<float>(msg->angular_velocity.z);

    _lowState.imu.accelerometer[0] = static_cast<float>(msg->linear_acceleration.x);
    _lowState.imu.accelerometer[1] = static_cast<float>(msg->linear_acceleration.y);
    _lowState.imu.accelerometer[2] = static_cast<float>(msg->linear_acceleration.z);
}

void IOROS2::joyCallback(const crb_ros_msg::msg::JoystickCmdReport::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(_gamepadMutex);

    // Helper: check if a button ID is in pressed_buttons
    auto has = [&](uint32_t id) {
        auto &pb = msg->pressed_buttons;
        return std::find(pb.begin(), pb.end(), id) != pb.end();
    };

    // Axes from joystick (left_x/y, right_x/y)
    _userValue.lx = msg->left_x;
    _userValue.ly = msg->left_y;
    _userValue.rx = msg->right_x;
    _userValue.ry = msg->right_y;

    // D-pad
    bool dpad_up    = (msg->axis_y < 0);
    bool dpad_down  = (msg->axis_y > 0);
    bool dpad_left  = (msg->axis_x < 0);
    bool dpad_right = (msg->axis_x > 0);

    // Buttons: A=0, B=1, LB=4, RB=5, BACK=6, START=7, LT=11, RT=12
    bool btn_a      = has(0);
    bool btn_b      = has(1);
    bool btn_r1     = has(5);
    bool btn_start  = has(7);
    bool btn_select = has(6);
    bool axis_r2    = has(12);  // RT
    bool axis_l2    = has(11);  // LT

    _userCmd = UserCommand::NONE;

    if (btn_start) {
        _userCmd = UserCommand::START;
    }
    else if (btn_select) {
        _userCmd = UserCommand::SELECT;
    }
    else if (btn_r1 && dpad_up) {
        _userCmd = UserCommand::R1_UP;
    }
    else if (btn_r1 && dpad_left) {
        _userCmd = UserCommand::R1_LEFT;
    }
    else if (btn_r1 && dpad_right) {
        _userCmd = UserCommand::R1_RIGHT;
    }
    else if (btn_r1) {
        _userCmd = UserCommand::R1;
    }
    else if (axis_r2 && dpad_up) {
        _userCmd = UserCommand::R2_UP;
    }
    else if (axis_r2 && dpad_down) {
        _userCmd = UserCommand::R2_DOWN;
    }
    else if (axis_r2 && btn_b) {
        _userCmd = UserCommand::R2_B;
    }
    else if (axis_r2 && btn_a) {
        _userCmd = UserCommand::R2_A;
    }
    else if (axis_r2) {
        _userCmd = UserCommand::R2;
    }
    else if (axis_l2 && btn_b) {
        _userCmd = UserCommand::L2_B;
    }
    else if (axis_l2) {
        _userCmd = UserCommand::L2;
    }
}

void IOROS2::sendRecv(const LowlevelCmd *cmd, LowlevelState *state)
{
    // publish joint command
    auto jointCmdMsg = sensor_msgs::msg::JointState();
    jointCmdMsg.header.stamp = _node->now();

    for (int i = 0; i < CASBOT_NUM_MOTOR; i++) {
        jointCmdMsg.name.push_back("joint_" + std::to_string(i));
        jointCmdMsg.position.push_back(cmd->motorCmd[i].q);
        jointCmdMsg.velocity.push_back(cmd->motorCmd[i].dq);
        jointCmdMsg.effort.push_back(cmd->motorCmd[i].tau);
    }

    _jointCmdPub->publish(jointCmdMsg);

    // copy state
    {
        std::lock_guard<std::mutex> lock(_stateMutex);

        for (int i = 0; i < CASBOT_NUM_MOTOR; i++) {
            state->motorState[i].q = _lowState.motorState[i].q;
            state->motorState[i].dq = _lowState.motorState[i].dq;
            state->motorState[i].tauEst = _lowState.motorState[i].tauEst;
        }

        for (int i = 0; i < 3; i++) {
            state->imu.quaternion[i] = _lowState.imu.quaternion[i];
            state->imu.accelerometer[i] = _lowState.imu.accelerometer[i];
            state->imu.gyroscope[i] = _lowState.imu.gyroscope[i];
        }
        state->imu.quaternion[3] = _lowState.imu.quaternion[3];
    }

    {
        std::lock_guard<std::mutex> lock(_gamepadMutex);
        state->userCmd = _userCmd;
        state->userValue = _userValue;
    }

    _counter++;
}
