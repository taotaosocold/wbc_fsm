#include "interface/IOSDK.h"
#include <stdio.h>
#include <iostream>

uint32_t crc32_core(uint32_t *ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

IOSDK::IOSDK()
{
    // ChannelFactory::Instance()->Init(0, "eth0"); // eth0 for real robot
    ChannelFactory::Instance()->Init(1, "lo"); // lo for simulation

    lowcmd_publisher_.reset(new ChannelPublisher<LowCmd_>(HG_CMD_TOPIC));
    lowcmd_publisher_->InitChannel();

    lowstate_subscriber_.reset(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
    lowstate_subscriber_->InitChannel(std::bind(&IOSDK::LowStateHandler, this, std::placeholders::_1), 1);

    counter_ = 0;
    userCmd_ = UserCommand::NONE;
    userValue_.setZero();
    mode_machine_ = 0;
}

void IOSDK::sendRecv(const LowlevelCmd *cmd, LowlevelState *state)
{
    // send control cmd
    LowCmd_ dds_low_command;
    dds_low_command.mode_pr() = static_cast<uint8_t>(Mode::PR);
    dds_low_command.mode_machine() = mode_machine_;
    for (size_t i = 0; i < G1_NUM_MOTOR; i++)
    {
        
        dds_low_command.motor_cmd().at(i).mode() = 1; // 1:Enable, 0:Disable
        dds_low_command.motor_cmd().at(i).tau() = cmd->motorCmd[i].tau;
        dds_low_command.motor_cmd().at(i).q() = cmd->motorCmd[i].q;
        dds_low_command.motor_cmd().at(i).dq() = cmd->motorCmd[i].dq;
        dds_low_command.motor_cmd().at(i).kp() = cmd->motorCmd[i].Kp;
        dds_low_command.motor_cmd().at(i).kd() = cmd->motorCmd[i].Kd;
        // std::cout<<"des_q: "<<dds_low_command.motor_cmd().at(i).q()<<std::endl;
    }

    dds_low_command.crc() = crc32_core((uint32_t *)&dds_low_command, (sizeof(dds_low_command) >> 2) - 1);
    bool wrt = lowcmd_publisher_->Write(dds_low_command);

    for (int i = 0; i < G1_NUM_MOTOR; i++)
    {
        state->motorState[i].q = _lowState.motorState[i].q;
        state->motorState[i].dq = _lowState.motorState[i].dq;
    }
    for (int i = 0; i < 3; i++)
    {
        state->imu.quaternion[i] = _lowState.imu.quaternion[i];
        state->imu.accelerometer[i] = _lowState.imu.accelerometer[i];
        state->imu.gyroscope[i] = _lowState.imu.gyroscope[i];
    }
    state->imu.quaternion[3] = _lowState.imu.quaternion[3];

    state->userCmd = userCmd_;
    state->userValue = userValue_;
}

// IOSDK 没有通过 CmdPanel 来读取手柄，而是直接在类内部自己做完了
// 在 LowStateHandler 回调中，它从机器人状态报文中提取遥控器数据
void IOSDK::LowStateHandler(const void *message)
{
    //将 message 强制转换为 const LowState_*（宇树 DDS 状态消息类型）,后续直接用low_state访问数据
    LowState_ low_state = *(const LowState_ *)message;
    // crc完整性校验，校验失败则直接返回丢弃本次报文
    if (low_state.crc() != crc32_core((uint32_t *)&low_state, (sizeof(LowState_) >> 2) - 1))
    {
        std::cout << "[ERROR] CRC Error" << std::endl;
        return;
    }

    // get motor state
    // 获取电机值，_lowState 是项目统一的底层状态结构，用于后续传递给 ControlFrame
    for (int i = 0; i < G1_NUM_MOTOR; ++i)
    {
        _lowState.motorState[i].q = low_state.motor_state()[i].q();
        _lowState.motorState[i].dq = low_state.motor_state()[i].dq();
    }
    
    // get imu state
    // 获得IMU的数据
    _lowState.imu.gyroscope[0] = low_state.imu_state().gyroscope()[0];
    _lowState.imu.gyroscope[1] = low_state.imu_state().gyroscope()[1];
    _lowState.imu.gyroscope[2] = low_state.imu_state().gyroscope()[2];

    _lowState.imu.quaternion[0] = low_state.imu_state().quaternion()[0];
    _lowState.imu.quaternion[1] = low_state.imu_state().quaternion()[1];
    _lowState.imu.quaternion[2] = low_state.imu_state().quaternion()[2];
    _lowState.imu.quaternion[3] = low_state.imu_state().quaternion()[3];

    _lowState.imu.accelerometer[0] = low_state.imu_state().accelerometer()[0];
    _lowState.imu.accelerometer[1] = low_state.imu_state().accelerometer()[1];
    _lowState.imu.accelerometer[2] = low_state.imu_state().accelerometer()[2];

    // update gamepad
    // 更新遥控器数据，low_state.wireless_remote() 返回机器人接收到的无线遥控器原始数据（40 字节），复制给rx_
    memcpy(rx_.buff, &low_state.wireless_remote()[0], 40);
    // 用rx_值去更新gamepad_值
    gamepad_.update(rx_.RF_RX);

    // update mode machine
    // 检测状态机，检测当前机器人状态机模式是否改变
    if (mode_machine_ != low_state.mode_machine())
    {
        if (mode_machine_ == 0)
            std::cout << "G1 type: " << unsigned(low_state.mode_machine()) << std::endl;
        mode_machine_ = low_state.mode_machine();
    }

    if(gamepad_.start.pressed)
    {
        userCmd_ = UserCommand::START;          
    }
    if(gamepad_.select.pressed)
    {
        userCmd_ = UserCommand::SELECT; 
    }

    if(gamepad_.R2.pressed)
    {
        userCmd_ = UserCommand::R2;
    }
    if (gamepad_.L2.pressed)
    {
        userCmd_ = UserCommand::L2;
    }
    if(gamepad_.R1.pressed)
    {
        userCmd_ = UserCommand::R1;
    }
    if (gamepad_.R2.pressed && gamepad_.A.pressed)
    {
        userCmd_ = UserCommand::R2_A;
    }
    if (gamepad_.L2.pressed && gamepad_.B.pressed)
    {
        userCmd_ = UserCommand::L2_B;
    }
    if (gamepad_.R1.pressed && gamepad_.up.pressed)
    {
        userCmd_ = UserCommand::R1_UP;
    }
    if (gamepad_.R1.pressed && gamepad_.left.pressed)
    {
        userCmd_ = UserCommand::R1_LEFT;
    }
    if (gamepad_.R1.pressed && gamepad_.right.pressed)
    {
        userCmd_ = UserCommand::R1_RIGHT;
    }
    if (gamepad_.R2.pressed && gamepad_.up.pressed)
    {
        userCmd_ = UserCommand::R2_UP;
    }
    if (gamepad_.R2.pressed && gamepad_.down.pressed)
    {
        userCmd_ = UserCommand::R2_DOWN;
    }
    if (gamepad_.R2.pressed && gamepad_.B.pressed)
    {
        userCmd_ = UserCommand::R2_B;
    }

    userValue_.lx = -gamepad_.lx;
    userValue_.ly = gamepad_.ly;
    userValue_.rx = -gamepad_.rx;
    userValue_.ry = gamepad_.ry;
}