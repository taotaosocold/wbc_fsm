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
    // 初始化lowcmd发布者
    lowcmd_publisher_.reset(new ChannelPublisher<LowCmd_>(HG_CMD_TOPIC));
    lowcmd_publisher_->InitChannel();
    // 初始化lowstate订阅者，设置的消息类型为LowState_，监听的话题是HG_STATE_TOPIC也就是/rt/lowstate这个话题
    lowstate_subscriber_.reset(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
    // 设置回调函数LowStateHandler，一旦订阅者监听到话题收到信息，就会订阅消息并执行回调函数
    lowstate_subscriber_->InitChannel(std::bind(&IOSDK::LowStateHandler, this, std::placeholders::_1), 1);

    counter_ = 0;
    userCmd_ = UserCommand::NONE;
    userValue_.setZero();
    mode_machine_ = 0;
}
// sendRecv 是 IOSDK 对基类 IOInterface 纯虚函数的实现。参数 cmd 是上层控制算法填充好的指令，state 是用来返回给上层的机器人最新状态
void IOSDK::sendRecv(const LowlevelCmd *cmd, LowlevelState *state)
{
    // send control cmd
    // 在栈上创建一个宇树 DDS 协议定义的底层指令对象 LowCmd_，后续会将其序列化并发送给机器人
    LowCmd_ dds_low_command;
    // 设置控制模式为 PR（串联控制），值从枚举 Mode::PR（0）强转为 uint8_t。这表示 Pitch/Roll 关节采用串联控制方式
    dds_low_command.mode_pr() = static_cast<uint8_t>(Mode::PR);
    // 将内部记录的模式机状态 mode_machine_（由 LowStateHandler 从机器人报文同步）填入指令消息，用于维持与机器人的状态机同步
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
    // crc校验
    dds_low_command.crc() = crc32_core((uint32_t *)&dds_low_command, (sizeof(dds_low_command) >> 2) - 1);
    // 通过 DDS 通道发布器 lowcmd_publisher_ 将 dds_low_command 发送给机器人
    bool wrt = lowcmd_publisher_->Write(dds_low_command);
    // 内部状态缓存 _lowState 中所有电机的位置和速度复制到上层传入的 state->motorState
    for (int i = 0; i < G1_NUM_MOTOR; i++)
    {
        // 获得电机数据
        state->motorState[i].q = _lowState.motorState[i].q;
        state->motorState[i].dq = _lowState.motorState[i].dq;
    }
    for (int i = 0; i < 3; i++)
    {
        // 获得四元数和加速度和陀螺仪
        state->imu.quaternion[i] = _lowState.imu.quaternion[i];
        state->imu.accelerometer[i] = _lowState.imu.accelerometer[i];
        state->imu.gyroscope[i] = _lowState.imu.gyroscope[i];
    }
    state->imu.quaternion[3] = _lowState.imu.quaternion[3];
    //将 IOSDK 内部解析出的当前用户命令（如 START、L2_B 等）和摇杆数值（lx, ly, rx, ry）写入 state，让上层 FSM 状态机能够根据这些输入决定行为切换
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
    // 获得IMU的数据，存放到变量_lowState中
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