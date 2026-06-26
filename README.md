# wbc_fsm — CasBot BeyondMimic Whole-Body Control Deployment

ONNX Runtime-based BeyondMimic policy deployment system. Communicates with MuJoCo simulator via ROS2 + shared memory. Supports both sim2sim (x86) and sim2real (ARM) deployment.

[中文](README_zh.md) | English

## Architecture

```
┌──────────────┐   Shared Memory   ┌──────────────┐   ROS2 Topics    ┌──────────────┐
│ casbot_mujoco │ ◄────────────────► │ casbot_bridge │ ◄──────────────► │   wbc_fsm    │
│  (MuJoCo sim) │  q,dq,tau,       │  (ROS2 bridge) │  joint_state,   │ (ONNX policy) │
│              │  q_cmd,kp,kd,     │               │  imu, joy,      │              │
│  1kHz physics │  imu,joy          │  1kHz timer   │  joint_cmd,     │  50Hz infer  │
│              │                   │               │  pd_gains       │              │
└──────────────┘                   └───────────────┘                 └──────────────┘
```

- **casbot_mujoco**: MuJoCo physics engine, 1kHz step, PD servo, sensor data → shared memory
- **casbot_bridge**: Reads shared memory → ROS2 topics, subscribes `/motion/joint_cmd` + `/motion/pd_gains` → shared memory
- **wbc_fsm**: FSM state machine + ONNX BeyondMimic policy inference, 50Hz joint command + PD gain publishing

### Data Flow

```
Sensors (sensordata)
  → shared memory (q, dq, tau, imu_quat, imu_gyro, imu_acc, base_vel)
    → ROS2 /motion/joint_state, /motion/imu
      → IOROS2 → LowlevelState → DataConverter → ONNX inference
        → processAction → computePdTarget → LowlevelCmd
          → ROS2 /motion/joint_cmd, /motion/pd_gains
            → casbot_bridge → shared memory (q_cmd, dq_cmd, tau_ff, kp, kd)
              → applyPdControl → d->ctrl → MuJoCo
```

## Dependencies

| Dependency | sim2sim (x86_64) | sim2real (aarch64) |
|---|---|---|
| CMake >= 3.14 | ✓ | ✓ |
| C++17 | ✓ | ✓ |
| ROS2 Humble | ✓ | ✓ |
| ONNX Runtime 1.22.0 | linux-x64 | linux-aarch64 |
| Eigen3 | ✓ | ✓ |
| nlohmann_json | ✓ | ✓ |
| MuJoCo 3.4.0 | ✓ (simulator) | — |

## Building

### Environment

```bash
source /opt/ros/humble/setup.bash
```

### Install ONNX Runtime

**x86_64 (sim2sim):**

```bash
cd /home/casbot/Desktop/wbc_fsm
mkdir -p third_party && cd third_party
wget https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-x64-1.22.0.tgz
tar -xzf onnxruntime-linux-x64-1.22.0.tgz
```

**aarch64 (sim2real, on ARM device):**

```bash
cd /home/casbot/Desktop/wbc_fsm
mkdir -p third_party && cd third_party
wget https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-aarch64-1.22.0.tgz
tar -xzf onnxruntime-linux-aarch64-1.22.0.tgz
```

### Build wbc_fsm

```bash
cd /home/casbot/Desktop/wbc_fsm
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

## sim2sim Deployment (x86_64)

### 1. Build All Components

```bash
# Build casbot_mujoco + casbot_bridge
cd /home/casbot/Desktop/casbot_mujoco/build
make casbot_mujoco casbot_bridge -j$(nproc)

# Build wbc_fsm
cd /home/casbot/Desktop/wbc_fsm/build
make -j$(nproc)
```

### 2. Launch (3 terminals, strict order)

```bash
# Terminal 1: ROS2 bridge
cd /home/casbot/Desktop/casbot_mujoco/build
./casbot_bridge

# Terminal 2: MuJoCo simulation (wait for bridge ready)
cd /home/casbot/Desktop/casbot_mujoco/build
./casbot_mujoco

# Terminal 3: ONNX policy controller
cd /home/casbot/Desktop/wbc_fsm/build
./wbc_fsm
```

### 3. Operation

| Step | Action | Description |
|---|---|---|
| 1 | wbc_fsm starts → PASSIVE | Damping mode, Kp=Kd=10, hold current pose |
| 2 | Press **Reset** in sim window | Reset if robot falls from initial pose |
| 3 | Gamepad **START** | Enter FIXSTAND, standard Kp/Kd, 2s interpolation |
| 4 | Gamepad **R2+A** or **R1+UP** | Enter WBC, ONNX policy takes over |

### Gamepad Mapping

| Button | Function |
|---|---|
| START | PASSIVE → FIXSTAND |
| R2+A / R1+UP | → WBC (policy execution) |
| R2 | Pause WBC |
| R1 | Resume WBC |
| L2+B | → PASSIVE |
| SELECT | Exit |

## sim2real Deployment (aarch64, on ARM robot)

### Architecture Difference

sim2real does NOT use MuJoCo or shared memory. wbc_fsm communicates directly with robot drivers via ROS2 topics:

```
┌──────────────┐   ROS2 Topics    ┌──────────────┐
│ CasBot Robot  │ ◄──────────────► │   wbc_fsm    │
│ (realtime drv)│  joint_state,   │ (ONNX policy) │
│              │  imu, joy,      │              │
│              │  joint_cmd,     │              │
│              │  pd_gains       │              │
└──────────────┘                 └──────────────┘
```

The robot must provide these ROS2 topics:
- **Publish** `/motion/joint_state` (sensor_msgs/JointState)
- **Publish** `/motion/imu` (sensor_msgs/Imu)
- **Publish** `/joystick_events` (crb_ros_msg/JoystickCmdReport)
- **Subscribe** `/motion/joint_cmd` (sensor_msgs/JointState)
- **Subscribe** `/motion/pd_gains` (std_msgs/Float64MultiArray)

### Deployment Steps

#### 1. Cross-compilation (on x86 dev machine)

```bash
# Install ARM cross-compilation toolchain
sudo apt install g++-aarch64-linux-gnu

# Download ARM ONNX Runtime
cd /home/casbot/Desktop/wbc_fsm/third_party
wget https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-aarch64-1.22.0.tgz
tar -xzf onnxruntime-linux-aarch64-1.22.0.tgz

# Cross-compile
cd /home/casbot/Desktop/wbc_fsm/build_arm
cmake .. \
  -DCMAKE_TOOLCHAIN_FILE=../cmake/aarch64-toolchain.cmake \
  -DONNXRUNTIME_ROOT=../third_party/onnxruntime-linux-aarch64-1.22.0
make -j$(nproc)
```

Cross-compilation toolchain `cmake/aarch64-toolchain.cmake`:

```cmake
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
```

#### 2. Or native build (compile directly on robot)

After copying wbc_fsm code to the robot PC:

```bash
# Download ARM ONNX Runtime
cd /home/casbot/Desktop/wbc_fsm/third_party
wget https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-aarch64-1.22.0.tgz
tar -xzf onnxruntime-linux-aarch64-1.22.0.tgz

# Build
cd /home/casbot/Desktop/wbc_fsm/build
cmake ..
make -j$(nproc)
```

#### 3. Run

```bash
# Ensure robot drivers are running (topics like /motion/joint_state have data)
source /opt/ros/humble/setup.bash
cd /home/casbot/Desktop/wbc_fsm/build
./wbc_fsm
```

Operation flow matches sim2sim: PASSIVE → FIXSTAND → WBC.

**Safety Notes:**
- Always enter FIXSTAND to stabilize before switching to WBC
- Emergency: press SELECT to exit, or L2+B to return to PASSIVE
- Use a safety harness/suspension for the robot

## ONNX Model

Model files go in `model/`. The ONNX model metadata auto-configures joint ordering, Kp/Kd, action_scales, etc.

Required metadata:
- `joint_names`: joint name list
- `joint_stiffness`: Kp values
- `joint_damping`: Kd values
- `default_joint_pos`: default joint positions
- `action_scale`: action scaling factors
- `body_names`: body part names
- `anchor_body_name`: anchor body name

## FSM States

| State | Description | Kp/Kd Source |
|---|---|---|
| PASSIVE | Damping hold, Kp=Kd=10, hold current pose | Hardcoded |
| FIXSTAND | Position-hold stand, 2s interpolation to target | Hardcoded (standard) |
| WBC | ONNX policy whole-body control | ONNX metadata → ROS2 |

## Directory Structure

```
wbc_fsm/
├── build/               # Build output
├── cmake/               # CMake toolchains (cross-compile)
├── config/              # JSON config files
├── include/
│   ├── common/          # DataConverter, math utils
│   ├── FSM/             # State machine (Passive, FixedStand, WBC)
│   ├── interface/       # IOROS2 (ROS2 communication)
│   └── message/         # LowlevelCmd, LowlevelState
├── model/               # ONNX model files
├── src/
│   ├── common/          # data_converter.cpp
│   ├── FSM/             # State implementations
│   ├── interface/       # IOROS2.cpp
│   └── main.cpp
├── third_party/         # ONNX Runtime
└── CMakeLists.txt
```
