#include "interface/IOROS2.h"
#include <iostream>
#include <cstring>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>

IOROS2::IOROS2()
{
    _node = std::make_shared<rclcpp::Node>("casbot_fsm_io");

    _jointCmdPub = _node->create_publisher<sensor_msgs::msg::JointState>("/motion/joint_cmd", 10);

    _pdGainsPub = _node->create_publisher<std_msgs::msg::Float64MultiArray>("/motion/pd_gains", 10);

    _jointStateSub = _node->create_subscription<sensor_msgs::msg::JointState>(
        "/motion/joint_state", 10,
        std::bind(&IOROS2::jointStateCallback, this, std::placeholders::_1));

    _imuSub = _node->create_subscription<sensor_msgs::msg::Imu>(
        "/motion/imu", 10,
        std::bind(&IOROS2::imuCallback, this, std::placeholders::_1));

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

    _directJoyThread = std::thread(&IOROS2::directJoystickLoop, this);

    std::cout << "[IOROS2] Initialized. Topics:" << std::endl;
    std::cout << "  Sub: /motion/joint_state" << std::endl;
    std::cout << "  Sub: /motion/imu" << std::endl;
    std::cout << "  Pub: /motion/joint_cmd" << std::endl;
    std::cout << "  Joystick: /dev/input/js0 (direct)" << std::endl;
}

IOROS2::~IOROS2()
{
    _running = false;
    if (_spinThread.joinable()) {
        _spinThread.join();
    }
    if (_directJoyThread.joinable()) {
        _directJoyThread.join();
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

// ============================================================================
// Direct USB joystick reader — reads /dev/input/js0 via Linux joystick API.
// Same code path for sim2sim (x86) and sim2real (ARM).
// ============================================================================
void IOROS2::directJoystickLoop()
{
    int fd = open("/dev/input/js0", O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        std::cerr << "[IOROS2] ERROR: Cannot open /dev/input/js0" << std::endl;
        return;
    }
    std::cout << "[IOROS2] Joystick reader started on /dev/input/js0" << std::endl;

    UserCommand lastDetected = UserCommand::NONE;
    std::set<uint32_t> btns;
    struct { int16_t x, y, z, rz, hat0x, hat0y; } axes = {};

    while (_running && rclcpp::ok()) {
        struct js_event e;
        bool updated = false;
        while (read(fd, &e, sizeof(e)) == sizeof(e)) {
            updated = true;
            if ((e.type & ~0x80) == JS_EVENT_BUTTON) {
                int crb = -1;
                switch (e.number) {
                    case 0:  crb = 0;  break;  // A
                    case 1:  crb = 1;  break;  // B
                    case 2:  crb = 2;  break;  // X
                    case 3:  crb = 3;  break;  // Y
                    case 6:  crb = 4;  break;  // TL → LB
                    case 7:  crb = 5;  break;  // TR → RB
                    case 10: crb = 6;  break;  // Select → BACK
                    case 11: crb = 7;  break;  // Start → START
                    case 8:  crb = 11; break;  // TL2 → LT
                    case 9:  crb = 12; break;  // TR2 → RT
                }
                std::cout << "[Joy] raw btn=" << (int)e.number
                          << " val=" << e.value
                          << " crb=" << crb << std::endl;
                if (crb >= 0) {
                    if (e.value) btns.insert(crb); else btns.erase(crb);
                }
            } else if ((e.type & ~0x80) == JS_EVENT_AXIS) {
                switch (e.number) {
                    case 0: axes.x     = e.value; break;
                    case 1: axes.y     = e.value; break;
                    case 2: axes.z     = e.value; break;
                    case 3: axes.rz    = e.value; break;
                    case 4: axes.hat0x = e.value; break;
                    case 5: axes.hat0y = e.value; break;
                }
            }
        }

        if (updated) {
            auto has = [&](uint32_t id) { return btns.find(id) != btns.end(); };

            bool btn_a      = has(0);
            bool btn_b      = has(1);
            bool btn_x      = has(2);
            bool btn_y      = has(3);
            bool btn_lb     = has(4);
            bool btn_rb     = has(5);
            bool btn_select = has(6);
            bool btn_start  = has(7);
            bool axis_lt    = has(11);
            bool axis_rt    = has(12);

            // Single-button mapping for mode switching
            UserCommand detected = UserCommand::NONE;
            if (btn_start)        detected = UserCommand::START;
            else if (btn_select)  detected = UserCommand::SELECT;
            else if (btn_a)       detected = UserCommand::R2_A;
            else if (btn_b)       detected = UserCommand::L2_B;
            else if (btn_x)       detected = UserCommand::R2;
            else if (btn_y)       detected = UserCommand::R1;
            else if (btn_rb)      detected = UserCommand::R1_UP;
            else if (btn_lb)      detected = UserCommand::R2_B;
            else if (axis_lt)     detected = UserCommand::L2;
            else if (axis_rt)     detected = UserCommand::R2;

            std::lock_guard<std::mutex> lock(_gamepadMutex);
            _userValue.lx = axes.x / 32767.0f;
            _userValue.ly = axes.y / 32767.0f;
            _userValue.rx = axes.z / 32767.0f;
            _userValue.ry = axes.rz / 32767.0f;

            if (detected != lastDetected) {
                _userCmd = detected;
                lastDetected = detected;
                if (detected != UserCommand::NONE)
                    std::cout << "[Joy] " << static_cast<int>(detected) << std::endl;
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    close(fd);
}

void IOROS2::sendRecv(const LowlevelCmd *cmd, LowlevelState *state)
{
    auto jointCmdMsg = sensor_msgs::msg::JointState();
    jointCmdMsg.header.stamp = _node->now();

    static const char* joint_names[CASBOT_NUM_MOTOR] = {
        "leg_l1_joint", "leg_l2_joint", "leg_l3_joint",
        "leg_l4_joint", "leg_l5_joint", "leg_l6_joint",
        "leg_r1_joint", "leg_r2_joint", "leg_r3_joint",
        "leg_r4_joint", "leg_r5_joint", "leg_r6_joint",
        "head_yaw_joint", "head_pitch_joint",
        "waist_yaw_joint",
        "l_shoulder_pitch_joint", "l_shoulder_roll_joint", "l_shoulder_yaw_joint",
        "l_elbow_pitch_joint", "l_wrist_yaw_joint",
        "r_shoulder_pitch_joint", "r_shoulder_roll_joint", "r_shoulder_yaw_joint",
        "r_elbow_pitch_joint", "r_wrist_yaw_joint"
    };

    for (int i = 0; i < CASBOT_NUM_MOTOR; i++) {
        jointCmdMsg.name.push_back(joint_names[i]);
        jointCmdMsg.position.push_back(cmd->motorCmd[i].q);
        jointCmdMsg.velocity.push_back(cmd->motorCmd[i].dq);
        jointCmdMsg.effort.push_back(cmd->motorCmd[i].tau);
    }

    _jointCmdPub->publish(jointCmdMsg);

    auto pdGainsMsg = std_msgs::msg::Float64MultiArray();
    pdGainsMsg.data.resize(CASBOT_NUM_MOTOR * 2);
    for (int i = 0; i < CASBOT_NUM_MOTOR; i++) {
        pdGainsMsg.data[i] = cmd->motorCmd[i].Kp;
        pdGainsMsg.data[i + CASBOT_NUM_MOTOR] = cmd->motorCmd[i].Kd;
    }
    _pdGainsPub->publish(pdGainsMsg);

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
        _userCmd = UserCommand::NONE;
    }

    _counter++;
}
