# wbc_fsm — CasBot BeyondMimic 全身控制部署

基于 ONNX Runtime 的 BeyondMimic 策略部署系统，通过 ROS2 + 共享内存与 MuJoCo 仿真器通信，支持 sim2sim（x86）和 sim2real（ARM）部署。

## 架构

```
┌─────────────┐   共享内存    ┌──────────────┐   ROS2 Topics   ┌──────────────┐
│ casbot_mujoco │ ◄──────────► │ casbot_bridge │ ◄─────────────► │   wbc_fsm    │
│  (MuJoCo 仿真) │  q,dq,tau,  │  (ROS2 桥接)  │  joint_state,  │ (ONNX 策略)   │
│              │  q_cmd,kp,kd │              │  imu, joy,     │              │
│  1kHz 物理   │  imu,joy     │  1kHz timer  │  joint_cmd,    │  50Hz 推理   │
│              │              │              │  pd_gains      │              │
└─────────────┘              └──────────────┘                └──────────────┘
```

- **casbot_mujoco**：MuJoCo 物理引擎，1kHz 步进，PD 伺服控制，传感器数据写入共享内存
- **casbot_bridge**：读取共享内存发布 ROS2 话题，订阅 `/motion/joint_cmd` 和 `/motion/pd_gains` 写回共享内存
- **wbc_fsm**：FSM 状态机 + ONNX BeyondMimic 策略推理，50Hz 发布关节指令和 PD 增益

### 数据流

```
传感器 (sensordata)
  → 共享内存 (q, dq, tau, imu_quat, imu_gyro, imu_acc, base_vel)
    → ROS2 /motion/joint_state, /motion/imu
      → IOROS2 → LowlevelState → DataConverter → ONNX 推理
        → processAction → computePdTarget → LowlevelCmd
          → ROS2 /motion/joint_cmd, /motion/pd_gains
            → casbot_bridge → 共享内存 (q_cmd, dq_cmd, tau_ff, kp, kd)
              → applyPdControl → d->ctrl → MuJoCo
```

## 依赖

| 依赖 | sim2sim (x86_64) | sim2real (aarch64) |
|---|---|---|
| CMake >= 3.14 | ✓ | ✓ |
| C++17 | ✓ | ✓ |
| ROS2 Humble | ✓ | ✓ |
| ONNX Runtime 1.22.0 | linux-x64 | linux-aarch64 |
| Eigen3 | ✓ | ✓ |
| nlohmann_json | ✓ | ✓ |
| MuJoCo 3.4.0 | ✓ (仿真器) | — |

## 编译

### 环境准备

```bash
sudo apt install patchelf
source /opt/ros/humble/setup.bash
```

### 安装 ONNX Runtime

**x86_64（sim2sim）：**

```bash
cd wbc_fsm
mkdir -p third_party && cd third_party
wget https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-x64-1.22.0.tgz
tar -xzf onnxruntime-linux-x64-1.22.0.tgz
```

**aarch64（sim2real，在 ARM 设备上执行）：**

```bash
cd wbc_fsm
mkdir -p third_party && cd third_party
wget https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-aarch64-1.22.0.tgz
tar -xzf onnxruntime-linux-aarch64-1.22.0.tgz
```

### 编译 wbc_fsm

编译前先确认 `CMakeLists.txt` 中 ONNX Runtime 路径指向当前平台：

```cmake
# x86_64 (sim2sim)
set(ONNXRUNTIME_ROOT ${PROJECT_SOURCE_DIR}/third_party/onnxruntime-linux-x64-1.22.0)
# aarch64 (sim2real)
# set(ONNXRUNTIME_ROOT ${PROJECT_SOURCE_DIR}/third_party/onnxruntime-linux-aarch64-1.22.0)
```

x86_64 用第一行，aarch64 则注释第一行、启用第二行。

```bash
cd wbc_fsm
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

> **注意**：首次 `cmake ..` 后若切换平台，需 `rm -rf build && mkdir build && cd build && cmake ..` 重新配置。已编译过的 x86 二进制在 ARM 上不能使用，需 clean rebuild。

## sim2sim 部署流程（x86_64）

### 1. 编译所有组件

```bash
# 编译 casbot_mujoco + casbot_bridge
cd casbot_mujoco/build
make casbot_mujoco casbot_bridge -j$(nproc)

# 编译 wbc_fsm
cd wbc_fsm/build
make -j$(nproc)
```

### 2. 启动（三个终端，严格按顺序）

```bash
# 终端 1：启动 ROS2 桥接
cd casbot_mujoco/build
./casbot_bridge

# 终端 2：启动 MuJoCo 仿真（等待 bridge ready 后）
cd casbot_mujoco/build
./casbot_mujoco

# 终端 3：启动 ONNX 策略控制器
cd wbc_fsm/build
./wbc_fsm
```

### 3. 操作流程

| 步骤 | 操作 | 说明 |
|---|---|---|
| 1 | wbc_fsm 启动 → PASSIVE | 阻尼模式，Kp=Kd=10，hold 当前位姿 |
| 2 | 仿真窗口按 **Reset** | 机器人可能因初始姿态不稳而跌倒 |
| 3 | 手柄 **START** | 进入 FIXSTAND，Kp/Kd 使用标准值，2 秒插值到位 |
| 4 | 手柄 **R2+A** 或 **R1+UP** | 进入 WBC，ONNX 策略接管 |

### 手柄按键映射

| 按键组合 | 功能 |
|---|---|
| START | PASSIVE → FIXSTAND |
| R2+A / R1+UP | → WBC（策略执行） |
| R2 | 暂停 WBC |
| R1 | 恢复 WBC |
| L2+B | → PASSIVE |
| SELECT | 退出程序 |

## sim2real 部署流程（aarch64，ARM 机器人端）

### 架构差异

sim2real 不需要 MuJoCo 和共享内存。wbc_fsm 通过 ROS2 话题直接与机器人底层驱动通信：

```
┌─────────────┐   ROS2 Topics   ┌──────────────┐
│ CasBot 机器人 │ ◄─────────────► │   wbc_fsm    │
│ (实时驱动)    │  joint_state,  │ (ONNX 策略)   │
│             │  imu, joy,     │              │
│             │  joint_cmd,    │              │
│             │  pd_gains      │              │
└─────────────┘                └──────────────┘
```

机器人端需要提供以下 ROS2 话题：
- **发布** `/motion/joint_state`（sensor_msgs/JointState）
- **发布** `/motion/imu`（sensor_msgs/Imu）
- **发布** `/joystick_events`（crb_ros_msg/JoystickCmdReport）
- **订阅** `/motion/joint_cmd`（sensor_msgs/JointState）
- **订阅** `/motion/pd_gains`（std_msgs/Float64MultiArray）

### 部署步骤

#### 1. ARM 交叉编译（在 x86 开发机上）

```bash
# 安装 ARM 交叉编译工具链
sudo apt install g++-aarch64-linux-gnu

# 下载 ARM ONNX Runtime
cd wbc_fsm/third_party
wget https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-aarch64-1.22.0.tgz
tar -xzf onnxruntime-linux-aarch64-1.22.0.tgz

# 交叉编译
cd wbc_fsm/build_arm
cmake .. \
  -DCMAKE_TOOLCHAIN_FILE=../cmake/aarch64-toolchain.cmake \
  -DONNXRUNTIME_ROOT=../third_party/onnxruntime-linux-aarch64-1.22.0
make -j$(nproc)
```

交叉编译工具链 `cmake/aarch64-toolchain.cmake`：

```cmake
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
```

#### 2. 或者 ARM 原生编译（在机器人上直接编译）

将 wbc_fsm 代码拷贝到机器人 PC 后，**切换 `CMakeLists.txt` 中的 ONNX Runtime 路径**：

```cmake
# x86_64 (sim2sim)
# set(ONNXRUNTIME_ROOT ${PROJECT_SOURCE_DIR}/third_party/onnxruntime-linux-x64-1.22.0)
# aarch64 (sim2real)
set(ONNXRUNTIME_ROOT ${PROJECT_SOURCE_DIR}/third_party/onnxruntime-linux-aarch64-1.22.0)
```

然后下载依赖并编译：

```bash
# 下载 ARM ONNX Runtime
cd wbc_fsm
mkdir -p third_party && cd third_party
wget https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-aarch64-1.22.0.tgz
tar -xzf onnxruntime-linux-aarch64-1.22.0.tgz

# 编译
cd ..
source /opt/ros/humble/setup.bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

# 准备sdk
准备casbot-motion_2.2.16_arm64.deb在机器人PC上
sudo dpkg -i casbot-motion_2.2.16_arm64.deb 安装，得到hl_motion
source /opt/ros/humble/setup.bash 激活ros环境
cd hl_motion
source setup.bash 激活sdk
cd bin
./hlorin 启动sdk
进入全身调试模式，等待/motion/joint_cmd 指令
ros2 service call /motion/whole_body_debug std_srvs/srv/SetBool "{data: true}"


#### 3. 运行

```bash
# 确保机器人底层驱动已启动（/motion/joint_state 等话题有数据）
source /opt/ros/humble/setup.bash
cd /workspace/wbc_fsm/build
./wbc_fsm
```

操作流程与 sim2sim 相同：PASSIVE → FIXSTAND → WBC。

**安全注意事项：**
- 实机运行时务必先进入 FIXSTAND 站稳，再切入 WBC
- 紧急情况按 SELECT 退出，或按 L2+B 切回 PASSIVE
- 建议用悬吊装置保护机器人

## ONNX 模型

模型文件放在 `model/` 目录。当前支持的模型通过 ONNX metadata 自动配置关节排序、Kp/Kd、action_scales 等参数。

模型 metadata 必须包含：
- `joint_names`：关节名称列表
- `joint_stiffness`：Kp 值
- `joint_damping`：Kd 值
- `default_joint_pos`：默认关节位置
- `action_scale`：动作缩放系数
- `body_names`：身体部件名称
- `anchor_body_name`：锚定身体名称

## FSM 状态

| 状态 | 说明 | Kp/Kd 来源 |
|---|---|---|
| PASSIVE | 阻尼保持，Kp=Kd=10 hold 当前位置 | 硬编码 |
| FIXSTAND | 位控站立，2s 插值到目标位姿 | 硬编码（标准值） |
| WBC | ONNX 策略全身控制 | ONNX metadata → ROS2 传输 |

## 目录结构

```
wbc_fsm/
├── build/               # 编译输出
├── cmake/               # CMake 工具链（交叉编译）
├── config/              # JSON 配置文件
├── include/
│   ├── common/          # DataConverter, math 工具
│   ├── FSM/             # 状态机 (Passive, FixedStand, WBC)
│   ├── interface/       # IOROS2 (ROS2 通信)
│   └── message/         # LowlevelCmd, LowlevelState
├── model/               # ONNX 模型文件
├── src/
│   ├── common/          # data_converter.cpp
│   ├── FSM/             # 各状态实现
│   ├── interface/       # IOROS2.cpp
│   └── main.cpp
├── third_party/         # ONNX Runtime
└── CMakeLists.txt
```
