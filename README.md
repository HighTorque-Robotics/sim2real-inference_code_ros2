# HighTorque RL Custom Inference Demo
# 高擎机电强化学习推理演示

[English](#english) | [中文](#中文)

---

## English

### Overview

This is an open-source ROS2 Foxy-based reinforcement learning inference demonstration package for HighTorque humanoid robots. It provides a complete example of how to deploy and run RL policies on real hardware using RKNN inference engine (Rockchip Neural Network).

**Developed by 高擎机电 (HighTorque Robotics)**

**ROS2 Version:** Foxy Fitzroy

**Key Features:**
- Real-time RL policy inference on ARM-based controllers
- Easy-to-configure YAML parameter system
- Joystick control for state transitions
- Comprehensive observation and action processing
- 100Hz control loop for smooth robot motion
- Multi-threaded architecture for improved real-time performance and reduced latency

### System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    User Control Layer                        │
│  ┌──────────┐         ┌──────────┐         ┌──────────┐    │
│  │ /cmd_vel │         │   /joy   │         │ /imu/data│    │
│  └────┬─────┘         └────┬─────┘         └────┬─────┘    │
└───────┼────────────────────┼────────────────────┼──────────┘
        │                    │                    │
        ▼                    ▼                    ▼
┌─────────────────────────────────────────────────────────────┐
│          hightorque_rl_inference_node (This Package)         │
│  ┌────────────────────────────────────────────────────────┐ │
│  │  Observation Processing (36-dim)                        │ │
│  │  • Gait phase (sin/cos)                                 │ │
│  │  • Command velocities (x, y, yaw)                       │ │
│  │  • Joint positions & velocities (12 DOF)                │ │
│  │  • Base angular velocity & orientation                  │ │
│  └────────────────────────────────────────────────────────┘ │
│                           │                                  │
│                           ▼                                  │
│  ┌────────────────────────────────────────────────────────┐ │
│  │         RKNN Inference Engine (.rknn model)             │ │
│  └────────────────────────────────────────────────────────┘ │
│                           │                                  │
│                           ▼                                  │
│  ┌────────────────────────────────────────────────────────┐ │
│  │  Action Processing (12 DOF)                             │ │
│  │  • Action clipping & scaling                            │ │
│  │  • Motor direction mapping                              │ │
│  │  • State-based scaling (STANDBY/RUNNING)                │ │
│  └────────────────────────────────────────────────────────┘ │
└───────────────────────────┬─────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                    Robot Control Layer                       │
│  ┌──────────────────┐         ┌──────────────────┐         │
│  │ /pi_plus_all     │         │ /pi_plus_preset  │         │
│  │ (Joint Commands) │         │ (Reset Commands) │         │
│  └──────────────────┘         └──────────────────┘         │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
                   ┌──────────────────┐
                   │  Robot Hardware  │
                   └──────────────────┘
```

### Prerequisites

**Hardware Requirements:**
- HighTorque humanoid robot (Pi Plus or compatible)
- ARM-based controller with RKNN support (e.g., RK3588)
- Joystick controller (for mode switching)

**Software Requirements:**
- Ubuntu 20.04 (or compatible)
- ROS2 Foxy Fitzroy
- Eigen3
- yaml-cpp
- RKNN runtime library (included in package)

### Installation

1. **Create a ROS2 workspace** (if you don't have one):
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. **Clone this repository**:
```bash
git clone <repository-url> sim2real-inference_code_ros2
cd sim2real-inference_code_ros2
```

3. **Install dependencies**:
```bash
sudo apt-get update
sudo apt-get install ros-foxy-sensor-msgs ros-foxy-geometry-msgs \
                     libeigen3-dev libyaml-cpp-dev
```

4. **Build the package**:
```bash
cd ~/ros2_ws
colcon build --packages-select hightorque_rl_inference
```

5. **Source the workspace**:
```bash
source install/setup.bash
```

### Quick Start

#### Step 1: Start the Robot in Developer Mode

First, ensure your robot is running and in developer mode. This should start the following ROS2 topics:
- `/sim2real_master_node/rbt_state` - Robot joint states
- `/sim2real_master_node/mtr_state` - Motor states
- `/yesense_imu/imu` - IMU data

#### Step 2: Configure Parameters

Edit the configuration file to match your robot and policy:
```bash
cd ~/ros2_ws/src/sim2real-inference_code_ros2/src/hightorque_rl_inference
nano config_example.yaml
```

Key parameters to configure:
- `policy_name`: Your RKNN model filename
- `num_actions`: Number of actuated joints (default: 12)
- `clip_actions_lower/upper`: Joint angle limits for your robot
- `motor_direction`: Motor rotation directions
- `map_index`: **Critical** - Joint order mapping from your policy output to low-level controller expected order
  - The low-level controller expects: `[L_hip_pitch, L_hip_roll, L_hip_yaw, L_knee, L_ankle_pitch, L_ankle_roll, R_hip_pitch, R_hip_roll, R_hip_yaw, R_knee, R_ankle_pitch, R_ankle_roll]`
  - If your policy outputs a different order, configure `map_index` to remap: `map_index[i]` = policy output index for controller position `i`
  - Example: `map_index: [5, 4, 3, 2, 1, 0, 11, 10, 9, 8, 7, 6]` reverses the order

#### Step 3: Launch the Inference Node

```bash
ros2 launch hightorque_rl_inference hightorque_rl_inference.launch.py
```

You should see output indicating:
```
[INFO] Loading config from: /path/to/config_example.yaml
[INFO] YAML config loaded successfully
[INFO] Initialization successful
[INFO] === 启动多线程控制循环 / Starting Multi-threaded Control Loop ===
```

#### Step 4: Control the Robot

The system uses a **state machine** with three states:

1. **NOT_READY** (Initial State)
   - Robot is waiting for initialization
   - **Transition to STANDBY**: Press `LT + RT + START` on joystick

2. **STANDBY** (Ready State)
   - Robot is balanced but uses minimal action scale (0.05)
   - Safe mode for testing
   - **Transition to RUNNING**: Press `LT + RT + LB` on joystick

3. **RUNNING** (Active State)
   - Full RL policy execution with configured `action_scale`
   - Robot responds to `/cmd_vel` commands
   - **Transition to STANDBY**: Press `LT + RT + LB` again

**Sending velocity commands**:
```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Turn left
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Configuration Guide

See [docs/configuration.md](docs/configuration.md) for detailed parameter descriptions.

### ROS Topics

#### Subscribed Topics (Input Data)

**1. `/sim2real_master_node/rbt_state`** (sensor_msgs/JointState)
- **Queue Size:** 100
- **Publisher:** `LowlevelControllerNode` (low-level controller)
- **Frequency:** Determined by low-level controller
- **Data Content:**
  ```cpp
  msg->position[0-11]   // 12 joint angles in robot coordinate frame
  msg->velocity[0-11]   // 12 joint velocities in robot coordinate frame (optional)
  ```

**2. `/sim2real_master_node/mtr_state`** (sensor_msgs/JointState)
- **Queue Size:** 100
- **Publisher:** `LowlevelControllerNode` (low-level controller)
- **Frequency:** Determined by low-level controller
- **Data Content:**
  ```cpp
  msg->position[0-11]   // 12 joint angles in motor coordinate frame
  msg->velocity[0-11]   // 12 joint velocities in motor coordinate frame (optional)
  ```

**3. `/yesense_imu/imu`** (sensor_msgs/Imu)
- **Queue Size:** 100
- **Publisher:** IMU driver node
- **Frequency:** IMU publish rate (typically 100-200Hz)
- **Data Content:**
  ```cpp
  msg->orientation (quaternion: x, y, z, w)  // Base orientation
  msg->angular_velocity (x, y, z)            // Base angular velocity
  ```

**4. `/cmd_vel`** (geometry_msgs/Twist)
- **Queue Size:** 50
- **Publisher:** External control node (keyboard, navigation stack, etc.)
- **Frequency:** Determined by publisher
- **Data Content:**
  ```cpp
  msg->linear.x   // Linear velocity x (forward/backward), limited to [-0.55, 0.55]
  msg->linear.y   // Linear velocity y (left/right), limited to [-0.3, 0.3]
  msg->angular.z  // Angular velocity z (rotation), limited to [-2.0, 2.0]
  ```

**5. `/joy`** (sensor_msgs/Joy)
- **Queue Size:** 10
- **Publisher:** Joystick driver node
- **Data Content:**
  ```cpp
  msg->axes[]     // Joystick axes values
  msg->buttons[]  // Joystick button states
  ```

#### Published Topics (Output Data)

**1. `/pi_plus_all`** (sensor_msgs/JointState)
- **Queue Size:** 1000
- **Subscriber:** `LowlevelControllerNode` (low-level controller)
- **Publish Frequency:** 100Hz (`rlCtrlFreq_`)
- **Data Content:**
  ```cpp
  msg->position[0-11]   // 12 joint target positions (RL policy output, scaled)
  msg->position[12-21]  // Other joint positions (set to 0.0)
  msg->header.stamp     // ros::Time(0) - indicates immediate execution
  ```
- **Joint Order (Expected by Low-level Controller):**
  ```
  Index:  0    1    2    3    4    5    6    7    8    9    10   11
  Joint:  L_hip_pitch, L_hip_roll, L_hip_yaw, L_knee, L_ankle_pitch, L_ankle_roll,
          R_hip_pitch, R_hip_roll, R_hip_yaw, R_knee, R_ankle_pitch, R_ankle_roll
  ```
- **Important: Joint Mapping Configuration**
  - The low-level controller expects joints in the above order
  - If your RL policy outputs joints in a different order, you **must** configure `map_index` in `config_example.yaml` to map your policy's output order to this expected order
  - `map_index[i]` specifies which policy output index should be sent to position `i` in `/pi_plus_all`
  - Example: If your policy outputs `[ankle_roll, ankle_pitch, knee, ...]` but the controller expects `[hip_pitch, hip_roll, ...]`, set `map_index: [5, 4, 3, 2, 1, 0, ...]` to reorder

**2. `/pi_plus_preset`** (sensor_msgs/JointState)
- **Queue Size:** 10
- **Subscriber:** `LowlevelControllerNode` (low-level controller)
- **Publish Frequency:** Only during state transitions (LT+RT+START)
- **Data Content:**
  ```cpp
  msg->header.frame_id = "zero"  // Preset pose name
  msg->header.stamp.fromSec(2.0) // Duration (seconds)
  ```

#### Data Flow Diagram

```
┌─────────────────────────────────────────────────────────┐
│   External Inference Node (hightorque_rl_inference)     │
│                                                          │
│   Subscribed Topics:                                    │
│   ├─ /sim2real_master_node/rbt_state  → Joint states    │
│   ├─ /sim2real_master_node/mtr_state  → Motor states   │
│   ├─ /yesense_imu/imu                → IMU data        │
│   ├─ /cmd_vel                          → Velocity cmd    │
│   └─ /joy                              → Joystick input │
│                                                          │
│   Processing:                                           │
│   1. Build 36-dim observation vector                   │
│   2. RKNN inference → 12-dim action                    │
│                                                          │
│   Published Topics:                                     │
│   ├─ /pi_plus_all      → Joint commands (100Hz)        │
│   └─ /pi_plus_preset   → Preset commands (on demand)   │
└─────────────────────────────────────────────────────────┘
         ↓                                    ↑
         │                                    │
    /pi_plus_all                        /rbt_state, /mtr_state
         │                                    │
         ↓                                    ↑
┌─────────────────────────────────────────────────────────┐
│   Low-level Controller (LowlevelControllerNode)          │
│                                                          │
│   Subscribes: /pi_plus_all → Executes PD control        │
│   Publishes: /rbt_state, /mtr_state                     │
└─────────────────────────────────────────────────────────┘
```

#### Observation Vector Structure (36 dimensions)

The observation vector is constructed as follows:

- **`observations_[0-1]`**: Gait phase (sin/cos of step counter)
  - STANDBY: `[1.0, -1.0]`
  - RUNNING: `[sin(2π*step), cos(2π*step)]`

- **`observations_[2-4]`**: Command velocities (scaled)
  - `[2]`: Linear velocity x (forward/backward)
  - `[3]`: Linear velocity y (left/right)
  - `[4]`: Angular velocity z (rotation)

- **`observations_[5-16]`**: Joint positions (12 DOF, scaled by `rbtLinPosScale_`)

- **`observations_[17-28]`**: Joint velocities (12 DOF, scaled by `rbtLinVelScale_`)

- **`observations_[29-31]`**: Base angular velocity (scaled by `rbtAngVelScale_`)

- **`observations_[32-34]`**: Base orientation (Euler angles: roll, pitch, yaw)

All observations are clipped to `[-clipObs_, clipObs_]` (default: ±18.0).

### Multi-threading Architecture

This inference system uses a **multi-threaded architecture** that distributes different types of callbacks to independent threads for parallel processing, significantly improving system real-time performance and inference efficiency.

#### Why Multi-threading?

**Single-threaded Problems:**
- All topic processing and inference run **serially** in the same thread
- Data callbacks **block inference computation**
- Inference computation **blocks new data reception**
- Causes data delays and poor inference performance

**Multi-threading Benefits:**
- Sensor data, inference computation, and command input are **processed in parallel**
- Data callbacks don't block inference
- Inference computation doesn't block data reception
- Significantly reduces data latency and improves inference performance

#### Threading Architecture

The system uses **3 independent threads** to handle different tasks:

```
┌─────────────────────────────────────────────────────────────┐
│                   MultiThreadedExecutor                      │
│                      (3 threads)                             │
└─────────────────────────────────────────────────────────────┘
           │                    │                    │
           ▼                    ▼                    ▼
    ┌──────────┐        ┌──────────┐        ┌──────────┐
    │ Thread 1 │        │ Thread 2 │        │ Thread 3 │
    │  Sensor  │        │  Control │        │  Command │
    └──────────┘        └──────────┘        └──────────┘
         │                    │                    │
         ▼                    ▼                    ▼
  ┌──────────┐         ┌──────────┐        ┌──────────┐
  │ IMU Data │         │Obs Update│        │cmd_vel   │
  │Joint Pos │         │RKNN Inf. │        │Joy input │
  │Joint Vel │         │Cmd Pub.  │        │          │
  │          │         │100Hz Loop│        │          │
  └──────────┘         └──────────┘        └──────────┘
```

**Thread 1: Sensor Callback Group** (High Priority)
- Handles high-frequency sensor data:
  - `/yesense_imu/imu` - IMU data (orientation, angular velocity)
  - `/sim2real_master_node/rbt_state` - Robot joint states
  - `/sim2real_master_node/mtr_state` - Motor states
- Independent thread ensures **real-time sensor data reception**
- Not blocked by inference computation
- Uses `std::mutex` for thread-safe data access

**Thread 2: Control Loop Callback Group** (High Priority)
- Runs main control loop (100Hz timer):
  1. Update observations (`updateObservation()`)
  2. Run RKNN inference (`updateAction()`)
  3. Publish joint commands
- Independent thread ensures **uninterrupted inference execution**
- Timer provides precise frequency control
- Not interrupted by sensor callbacks

**Thread 3: Command Input Callback Group** (Medium Priority)
- Handles user commands:
  - `/cmd_vel` - Velocity commands (x, y, yaw)
  - `/joy` - Joystick input (mode switching)
- Independent thread for user input processing
- Doesn't affect sensor and inference threads
- Uses `std::mutex` for command data protection

#### Thread Safety Mechanisms

To ensure data consistency in a multi-threaded environment, the system uses:

**1. Mutexes**
Each shared data structure has a dedicated mutex:

| Data | Mutex | Accessing Threads |
|------|-------|-------------------|
| Robot joint states | `robotStateMutex_` | Sensor ↔ Control |
| Motor states | `motorStateMutex_` | Sensor ↔ Control |
| IMU data | `imuMutex_` | Sensor ↔ Control |
| Velocity commands | `commandMutex_` | Command ↔ Control |
| Gait phase | `stepMutex_` | Control (exclusive) |
| Trigger time | `triggerMutex_` | Command (joystick) |

**2. Atomic Variables**
For simple flags, no locking needed:
```cpp
std::atomic<bool> quit_;           // Exit flag
std::atomic<bool> stateReceived_;  // Data reception flag
std::atomic<bool> imuReceived_;    // IMU reception flag
```

**3. Callback Groups**
Uses ROS2's callback group mechanism to isolate different callback types:
```cpp
// Create independent callback groups
sensorCallbackGroup_ = create_callback_group(MutuallyExclusive);
controlCallbackGroup_ = create_callback_group(MutuallyExclusive);
commandCallbackGroup_ = create_callback_group(MutuallyExclusive);

// Assign callback groups to subscribers
auto options = rclcpp::SubscriptionOptions();
options.callback_group = sensorCallbackGroup_;
imuSub_ = create_subscription<Imu>("/imu", 100, callback, options);
```

#### Performance Comparison

| Metric | Single-threaded | Multi-threaded |
|--------|----------------|----------------|
| Sensor data latency | 10-20ms | < 2ms |
| Inference frequency stability | High variance | Stable 100Hz |
| CPU utilization | Single core 100% | Multi-core load balanced |
| Inference performance | Poor due to delays | Excellent real-time performance |

#### Verifying Multi-threading

After launching, you should see:
```
[INFO] === 初始化多线程回调组 / Initializing Multi-threaded Callback Groups ===
[INFO] === 启动多线程控制循环 / Starting Multi-threaded Control Loop ===
[INFO] 控制频率: 100.0 Hz
[INFO] 多线程控制循环已启动！
[INFO] - 传感器数据回调：独立线程
[INFO] - 控制循环：独立线程 (100.0 Hz)
[INFO] - 指令输入回调：独立线程
[INFO] === 开始多线程执行 / Starting Multi-threaded Execution ===
```

### Troubleshooting

**Q: "Timeout waiting for robot data"**
- Ensure the robot is running and topics are being published
- Check topic names with `ros2 topic list`
- Verify topic data with `ros2 topic echo /sim2real_master_node/rbt_state`

**Q: "Model loading failed"**
- Check that the `.rknn` model file exists in `policy/` directory
- Verify `policy_name` in `config_example.yaml` matches your file
- Ensure RKNN runtime library is correctly installed

**Q: Robot behaves erratically**
- Check `motor_direction` configuration
- Verify `map_index` matches your robot's joint ordering
- Adjust `action_scale` to a lower value
- Review `clip_actions_lower/upper` limits

For more issues, see [docs/troubleshooting.md](docs/troubleshooting.md)

### Development Guide

#### Adding Your Own RL Policy

1. Convert your trained policy to RKNN format (`.rknn` file)
2. Place it in the `policy/` directory
3. Update `config_example.yaml` with:
   - New `policy_name`
   - Correct `num_single_obs` and `num_actions`
   - Appropriate scaling parameters
4. Test in STANDBY mode first before switching to RUNNING

#### Modifying Observation Space

Edit `src/hightorque_rl_inference.cpp`, function `updateObservation()`:
```cpp
void InferenceDemo::updateObservation()
{
    // Resize observations if needed
    observations_.resize(numSingleObs_);
    
    // Add your custom observations
    observations_[0] = /* your observation 1 */;
    observations_[1] = /* your observation 2 */;
    // ...
}
```

See [docs/development.md](docs/development.md) for more details.

### Project Structure

```
sim2real-inference_code_ros2/
├── src/
│   ├── hightorque_rl_inference/
│   │   ├── CMakeLists.txt          # Build configuration
│   │   ├── package.xml             # Package metadata
│   │   ├── config_example.yaml     # Default configuration
│   │   ├── include/
│   │   │   └── hightorque_rl_inference/
│   │   │       └── hightorque_rl_inference.h    # Main class header
│   │   ├── launch/
│   │   │   └── hightorque_rl_inference.launch.py   # Launch file
│   │   ├── policy/
│   │   │   ├── policy_0322_12dof_4000.rknn  # Example model
│   │   │   └── combined_model_dwaq_v1226.rknn
│   │   └── src/
│   │       ├── hightorque_rl_inference.cpp      # Main implementation
│   │       └── main.cpp                # Entry point
│   └── sim2real_msg_ros2/          # Message package
├── docs/                           # Documentation
├── README.md                       # This file
└── .gitignore                      # Git ignore rules
```

---

## 中文

### 项目简介

这是一个基于 ROS2 Foxy 的开源强化学习推理演示包，专为 HighTorque 人形机器人设计。它提供了一个完整的示例，展示如何使用 RKNN 推理引擎（Rockchip Neural Network）在真实硬件上部署和运行强化学习策略。

**开发商：高擎机电（HighTorque Robotics）**

**ROS2 版本：** Foxy Fitzroy

**核心特性：**
- 在 ARM 架构控制器上实时运行强化学习策略推理
- 简单易用的 YAML 参数配置系统
- 手柄控制状态切换
- 完整的观测值和动作处理流程
- 100Hz 控制频率，实现流畅的机器人运动
- 多线程架构，显著提升实时性能并降低延迟

### 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                      用户控制层                              │
│  ┌──────────┐         ┌──────────┐         ┌──────────┐    │
│  │ /cmd_vel │         │   /joy   │         │ /imu/data│    │
│  └────┬─────┘         └────┬─────┘         └────┬─────┘    │
└───────┼────────────────────┼────────────────────┼──────────┘
        │                    │                    │
        ▼                    ▼                    ▼
┌─────────────────────────────────────────────────────────────┐
│        hightorque_rl_inference_node (本功能包)               │
│  ┌────────────────────────────────────────────────────────┐ │
│  │  观测值处理 (36维)                                       │ │
│  │  • 步态相位 (sin/cos)                                    │ │
│  │  • 速度指令 (x, y, yaw)                                  │ │
│  │  • 关节位置和速度 (12自由度)                            │ │
│  │  • 基座角速度和姿态                                      │ │
│  └────────────────────────────────────────────────────────┘ │
│                           │                                  │
│                           ▼                                  │
│  ┌────────────────────────────────────────────────────────┐ │
│  │         RKNN 推理引擎 (.rknn 模型)                       │ │
│  └────────────────────────────────────────────────────────┘ │
│                           │                                  │
│                           ▼                                  │
│  ┌────────────────────────────────────────────────────────┐ │
│  │  动作处理 (12自由度)                                     │ │
│  │  • 动作裁剪和缩放                                        │ │
│  │  • 电机方向映射                                          │ │
│  │  • 基于状态的缩放 (STANDBY/RUNNING)                     │ │
│  └────────────────────────────────────────────────────────┘ │
└───────────────────────────┬─────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                      机器人控制层                            │
│  ┌──────────────────┐         ┌──────────────────┐         │
│  │ /pi_plus_all     │         │ /pi_plus_preset  │         │
│  │ (关节指令)       │         │ (复位指令)       │         │
│  └──────────────────┘         └──────────────────┘         │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
                   ┌──────────────────┐
                   │    机器人硬件    │
                   └──────────────────┘
```

### 环境要求

**硬件要求：**
- HighTorque 人形机器人（Pi Plus 或兼容机型）
- 支持 RKNN 的 ARM 控制器（如 RK3588）
- 游戏手柄控制器（用于模式切换）

**软件要求：**
- Ubuntu 20.04（或兼容版本）
- ROS2 Foxy Fitzroy
- Eigen3
- yaml-cpp
- RKNN 运行时库（已包含在功能包中）

### 安装步骤

1. **创建 ROS2 工作空间**（如果还没有）：
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. **克隆本仓库**：
```bash
git clone <repository-url> sim2real-inference_code_ros2
cd sim2real-inference_code_ros2
```

3. **安装依赖**：
```bash
sudo apt-get update
sudo apt-get install ros-foxy-sensor-msgs ros-foxy-geometry-msgs \
                     libeigen3-dev libyaml-cpp-dev
```

4. **编译功能包**：
```bash
cd ~/ros2_ws
colcon build --packages-select hightorque_rl_inference
```

5. **加载工作空间环境**：
```bash
source install/setup.bash
```

### 快速开始

#### 步骤 1：启动机器人开发者模式

首先，确保你的机器人正在运行并处于开发者模式。这将启动以下 ROS2 话题：
- `/sim2real_master_node/rbt_state` - 机器人关节状态
- `/sim2real_master_node/mtr_state` - 电机状态
- `/yesense_imu/imu` - IMU 数据

#### 步骤 2：配置参数

编辑配置文件以匹配你的机器人和策略：
```bash
cd ~/ros2_ws/src/sim2real-inference_code_ros2/src/hightorque_rl_inference
nano config_example.yaml
```

需要配置的关键参数：
- `policy_name`: 你的 RKNN 模型文件名
- `num_actions`: 驱动关节数量（默认：12）
- `clip_actions_lower/upper`: 机器人的关节角度限制
- `motor_direction`: 电机旋转方向
- `map_index`: **重要** - 从策略输出到低层控制器期望顺序的关节映射
  - 低层控制器期望的顺序：`[左髋pitch, 左髋roll, 左髋yaw, 左膝, 左踝pitch, 左踝roll, 右髋pitch, 右髋roll, 右髋yaw, 右膝, 右踝pitch, 右踝roll]`
  - 如果您的策略输出顺序不同，需要配置 `map_index` 进行重映射：`map_index[i]` = 控制器位置 `i` 对应的策略输出索引
  - 示例：`map_index: [5, 4, 3, 2, 1, 0, 11, 10, 9, 8, 7, 6]` 表示反转顺序

#### 步骤 3：启动推理节点

```bash
ros2 launch hightorque_rl_inference hightorque_rl_inference.launch.py
```

你应该看到以下输出：
```
[INFO] Loading config from: /path/to/config_example.yaml
[INFO] YAML config loaded successfully
[INFO] Initialization successful
[INFO] === 启动多线程控制循环 / Starting Multi-threaded Control Loop ===
```

#### 步骤 4：控制机器人

系统使用**状态机**，包含三个状态：

1. **NOT_READY（未就绪）**（初始状态）
   - 机器人等待初始化
   - **切换到 STANDBY**：按下手柄上的 `LT + RT + START`

2. **STANDBY（待机）**（就绪状态）
   - 机器人保持平衡但使用最小动作缩放（0.05）
   - 测试安全模式
   - **切换到 RUNNING**：按下手柄上的 `LT + RT + LB`

3. **RUNNING（运行）**（活动状态）
   - 使用配置的 `action_scale` 完整执行强化学习策略
   - 机器人响应 `/cmd_vel` 指令
   - **切换回 STANDBY**：再次按下 `LT + RT + LB`

**发送速度指令**：
```bash
# 前进
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 左转
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# 停止
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 配置指南

详细的参数说明请参见 [docs/configuration.md](docs/configuration.md)

### ROS 话题

#### 订阅的话题（输入数据）

**1. `/sim2real_master_node/rbt_state`** (sensor_msgs/JointState)
- **队列大小：** 100
- **发布者：** `LowlevelControllerNode`（低层控制器）
- **频率：** 由低层控制器决定
- **数据内容：**
  ```cpp
  msg->position[0-11]   // 12个关节在机器人坐标系下的角度
  msg->velocity[0-11]   // 12个关节在机器人坐标系下的速度（可选）
  ```

**2. `/sim2real_master_node/mtr_state`** (sensor_msgs/JointState)
- **队列大小：** 100
- **发布者：** `LowlevelControllerNode`（低层控制器）
- **频率：** 由低层控制器决定
- **数据内容：**
  ```cpp
  msg->position[0-11]   // 12个关节在电机坐标系下的角度
  msg->velocity[0-11]   // 12个关节在电机坐标系下的速度（可选）
  ```

**3. `/yesense_imu/imu`** (sensor_msgs/Imu)
- **队列大小：** 100
- **发布者：** IMU 驱动节点
- **频率：** IMU 发布频率（通常 100-200Hz）
- **数据内容：**
  ```cpp
  msg->orientation (四元数: x, y, z, w)  // 基座姿态
  msg->angular_velocity (x, y, z)        // 基座角速度
  ```

**4. `/cmd_vel`** (geometry_msgs/Twist)
- **队列大小：** 50
- **发布者：** 外部控制节点（键盘控制、导航栈等）
- **频率：** 由发布者决定
- **数据内容：**
  ```cpp
  msg->linear.x   // 线速度 x（前后），限制范围 [-0.55, 0.55]
  msg->linear.y   // 线速度 y（左右），限制范围 [-0.3, 0.3]
  msg->angular.z  // 角速度 z（旋转），限制范围 [-2.0, 2.0]
  ```

**5. `/joy`** (sensor_msgs/Joy)
- **队列大小：** 10
- **发布者：** 手柄驱动节点
- **数据内容：**
  ```cpp
  msg->axes[]     // 手柄摇杆轴值
  msg->buttons[]  // 手柄按钮状态
  ```

#### 发布的话题（输出数据）

**1. `/pi_plus_all`** (sensor_msgs/JointState)
- **队列大小：** 1000
- **订阅者：** `LowlevelControllerNode`（低层控制器）
- **发布频率：** 100Hz (`rlCtrlFreq_`)
- **数据内容：**
  ```cpp
  msg->position[0-11]   // 12个关节的目标位置（RL策略输出，经过缩放）
  msg->position[12-21]  // 其他关节位置（设为0.0）
  msg->header.stamp     // ros::Time(0) - 表示立即执行
  ```
- **关节顺序（低层控制器期望的顺序）：**
  ```
  索引:  0    1    2    3    4    5    6    7    8    9    10   11
  关节:  左髋pitch, 左髋roll, 左髋yaw, 左膝, 左踝pitch, 左踝roll,
         右髋pitch, 右髋roll, 右髋yaw, 右膝, 右踝pitch, 右踝roll
  ```
- **重要：关节映射配置**
  - 低层控制器期望以上述顺序接收关节数据
  - 如果您的 RL 策略输出的关节顺序不同，**必须**在 `config_example.yaml` 中配置 `map_index` 参数，将策略输出的顺序映射到上述期望顺序
  - `map_index[i]` 指定策略输出的哪个索引应发送到 `/pi_plus_all` 的位置 `i`
  - 示例：如果您的策略输出顺序是 `[ankle_roll, ankle_pitch, knee, ...]`，但控制器期望 `[hip_pitch, hip_roll, ...]`，则需要设置 `map_index: [5, 4, 3, 2, 1, 0, ...]` 来重新排序

**2. `/pi_plus_preset`** (sensor_msgs/JointState)
- **队列大小：** 10
- **订阅者：** `LowlevelControllerNode`（低层控制器）
- **发布频率：** 仅在状态切换时（LT+RT+START）
- **数据内容：**
  ```cpp
  msg->header.frame_id = "zero"  // 预设姿态名称
  msg->header.stamp.fromSec(2.0) // 持续时间（秒）
  ```

#### 数据流图

```
┌─────────────────────────────────────────────────────────┐
│   外部推理节点 (hightorque_rl_inference)                │
│                                                          │
│   订阅的话题：                                            │
│   ├─ /sim2real_master_node/rbt_state  → 关节状态        │
│   ├─ /sim2real_master_node/mtr_state  → 电机状态        │
│   ├─ /yesense_imu/imu                 → IMU数据          │
│   ├─ /cmd_vel                          → 速度指令        │
│   └─ /joy                              → 手柄输入        │
│                                                          │
│   处理：                                                  │
│   1. 构建36维观测向量                                    │
│   2. RKNN推理 → 12维动作                                │
│                                                          │
│   发布的话题：                                            │
│   ├─ /pi_plus_all      → 关节控制命令 (100Hz)           │
│   └─ /pi_plus_preset   → 预设姿态命令 (按需)            │
└─────────────────────────────────────────────────────────┘
         ↓                                    ↑
         │                                    │
    /pi_plus_all                        /rbt_state, /mtr_state
         │                                    │
         ↓                                    ↑
┌─────────────────────────────────────────────────────────┐
│   低层控制器 (LowlevelControllerNode)                     │
│                                                          │
│   订阅：/pi_plus_all → 执行PD控制 → 发布状态             │
└─────────────────────────────────────────────────────────┘
```

#### 观测向量结构（36维）

观测向量按以下方式构建：

- **`observations_[0-1]`**：步态相位（步进计数器的 sin/cos）
  - STANDBY：`[1.0, -1.0]`
  - RUNNING：`[sin(2π*step), cos(2π*step)]`

- **`observations_[2-4]`**：速度指令（缩放后）
  - `[2]`：线速度 x（前后）
  - `[3]`：线速度 y（左右）
  - `[4]`：角速度 z（旋转）

- **`observations_[5-16]`**：关节位置（12自由度，缩放因子 `rbtLinPosScale_`）

- **`observations_[17-28]`**：关节速度（12自由度，缩放因子 `rbtLinVelScale_`）

- **`observations_[29-31]`**：基座角速度（缩放因子 `rbtAngVelScale_`）

- **`observations_[32-34]`**：基座姿态（欧拉角：roll, pitch, yaw）

所有观测值都会被裁剪到 `[-clipObs_, clipObs_]`（默认：±18.0）。

### 多线程架构

本推理系统采用**多线程架构**，将不同类型的回调函数分配到独立的线程中并行处理，显著提高了系统的实时性和推理效率。

#### 为什么需要多线程？

**单线程问题：**
- 所有话题处理和推理在同一个线程中**串行执行**
- 数据回调会**阻塞推理计算**
- 推理计算会**阻塞新数据接收**
- 导致数据延迟和推理效果差

**多线程优势：**
- 传感器数据、推理计算、指令输入**并行处理**
- 数据回调不会阻塞推理
- 推理计算不会阻塞数据接收
- 显著降低数据延迟，提高推理效果

#### 线程架构

系统使用 **3 个独立线程**分别处理不同任务：

```
┌─────────────────────────────────────────────────────────────┐
│                   MultiThreadedExecutor                      │
│                      (3 threads)                             │
└─────────────────────────────────────────────────────────────┘
           │                    │                    │
           ▼                    ▼                    ▼
    ┌──────────┐        ┌──────────┐        ┌──────────┐
    │ Thread 1 │        │ Thread 2 │        │ Thread 3 │
    │  传感器  │        │  控制循环 │        │  指令输入 │
    │  Sensor  │        │  Control │        │  Command │
    └──────────┘        └──────────┘        └──────────┘
         │                    │                    │
         ▼                    ▼                    ▼
  ┌──────────┐         ┌──────────┐        ┌──────────┐
  │ IMU数据  │         │观测更新  │        │cmd_vel   │
  │关节状态  │         │RKNN推理  │        │手柄输入  │
  │Joint Pos │         │指令发布  │        │Joy input │
  │Joint Vel │         │100Hz循环 │        │          │
  └──────────┘         └──────────┘        └──────────┘
```

**线程 1: 传感器数据回调组**（高优先级）
- 处理高频传感器数据：
  - `/yesense_imu/imu` - IMU数据（姿态、角速度）
  - `/sim2real_master_node/rbt_state` - 机器人关节状态
  - `/sim2real_master_node/mtr_state` - 电机状态
- 独立线程确保传感器数据**实时接收**
- 不会被推理计算阻塞
- 使用 `std::mutex` 保护数据线程安全

**线程 2: 控制循环回调组**（高优先级）
- 运行主控制循环（100Hz定时器）：
  1. 更新观测值 (`updateObservation()`)
  2. 运行RKNN推理 (`updateAction()`)
  3. 发布关节指令
- 独立线程确保推理**不间断运行**
- 定时器精确控制频率
- 不会被传感器回调打断

**线程 3: 指令输入回调组**（中优先级）
- 处理用户指令：
  - `/cmd_vel` - 速度指令（x, y, yaw）
  - `/joy` - 手柄输入（模式切换）
- 独立线程处理用户输入
- 不影响传感器和推理线程
- 使用 `std::mutex` 保护指令数据

#### 线程安全机制

为了保证多线程环境下的数据一致性，系统使用以下线程安全措施：

**1. 互斥锁**
每个共享数据都有专用的互斥锁：

| 数据 | 互斥锁 | 访问线程 |
|------|--------|----------|
| 机器人关节状态 | `robotStateMutex_` | 传感器线程 ← → 控制线程 |
| 电机状态 | `motorStateMutex_` | 传感器线程 ← → 控制线程 |
| IMU数据 | `imuMutex_` | 传感器线程 ← → 控制线程 |
| 速度指令 | `commandMutex_` | 指令线程 ← → 控制线程 |
| 步态相位 | `stepMutex_` | 控制线程（独占） |
| 触发时间 | `triggerMutex_` | 指令线程（手柄） |

**2. 原子变量**
用于简单的标志位，无需加锁：
```cpp
std::atomic<bool> quit_;           // 退出标志
std::atomic<bool> stateReceived_;  // 数据接收标志
std::atomic<bool> imuReceived_;    // IMU接收标志
```

**3. 回调组**
使用ROS2的回调组机制隔离不同类型的回调：
```cpp
// 创建独立的回调组
sensorCallbackGroup_ = create_callback_group(MutuallyExclusive);
controlCallbackGroup_ = create_callback_group(MutuallyExclusive);
commandCallbackGroup_ = create_callback_group(MutuallyExclusive);

// 为订阅者指定回调组
auto options = rclcpp::SubscriptionOptions();
options.callback_group = sensorCallbackGroup_;
imuSub_ = create_subscription<Imu>("/imu", 100, callback, options);
```

#### 性能对比

| 指标 | 单线程 | 多线程 |
|------|--------|--------|
| 传感器数据延迟 | 10-20ms | < 2ms |
| 推理频率稳定性 | 波动大 | 稳定100Hz |
| CPU利用率 | 单核100% | 多核负载均衡 |
| 推理效果 | 数据延迟导致效果差 | 实时性好，效果优 |

#### 验证多线程运行

启动后查看日志输出：
```
[INFO] === 初始化多线程回调组 / Initializing Multi-threaded Callback Groups ===
[INFO] === 启动多线程控制循环 / Starting Multi-threaded Control Loop ===
[INFO] 控制频率: 100.0 Hz
[INFO] 多线程控制循环已启动！
[INFO] - 传感器数据回调：独立线程
[INFO] - 控制循环：独立线程 (100.0 Hz)
[INFO] - 指令输入回调：独立线程
[INFO] === 开始多线程执行 / Starting Multi-threaded Execution ===
```

### 常见问题

**问："Timeout waiting for robot data"**
- 确保机器人正在运行且话题正在发布
- 使用 `ros2 topic list` 检查话题名称
- 使用 `ros2 topic echo /sim2real_master_node/rbt_state` 验证话题数据

**问："Model loading failed"**
- 检查 `.rknn` 模型文件是否存在于 `policy/` 目录
- 验证 `config_example.yaml` 中的 `policy_name` 与文件名匹配
- 确保 RKNN 运行时库已正确安装

**问：机器人行为异常**
- 检查 `motor_direction` 配置
- 验证 `map_index` 与机器人的关节顺序匹配
- 将 `action_scale` 调整为较小的值
- 检查 `clip_actions_lower/upper` 限制

更多问题请参见 [docs/troubleshooting.md](docs/troubleshooting.md)

### 开发指南

#### 添加自己的强化学习策略

1. 将训练好的策略转换为 RKNN 格式（`.rknn` 文件）
2. 将其放置在 `policy/` 目录
3. 更新 `config_example.yaml`：
   - 新的 `policy_name`
   - 正确的 `num_single_obs` 和 `num_actions`
   - 适当的缩放参数
4. 先在 STANDBY 模式下测试，然后再切换到 RUNNING

#### 修改观测空间

编辑 `src/hightorque_rl_inference.cpp`，修改 `updateObservation()` 函数：
```cpp
void InferenceDemo::updateObservation()
{
    // 如需要，调整观测维度
    observations_.resize(numSingleObs_);
    
    // 添加自定义观测
    observations_[0] = /* 你的观测值 1 */;
    observations_[1] = /* 你的观测值 2 */;
    // ...
}
```

更多详情请参见 [docs/development.md](docs/development.md)

### 项目结构

```
sim2real-inference_code_ros2/
├── src/
│   ├── hightorque_rl_inference/
│   │   ├── CMakeLists.txt          # 编译配置
│   │   ├── package.xml             # 功能包元数据
│   │   ├── config_example.yaml     # 默认配置
│   │   ├── include/
│   │   │   └── hightorque_rl_inference/
│   │   │       └── hightorque_rl_inference.h    # 主类头文件
│   │   ├── launch/
│   │   │   └── hightorque_rl_inference.launch.py   # 启动文件
│   │   ├── policy/
│   │   │   ├── policy_0322_12dof_4000.rknn  # 示例模型
│   │   │   └── combined_model_dwaq_v1226.rknn
│   │   └── src/
│   │       ├── hightorque_rl_inference.cpp      # 主实现
│   │       └── main.cpp                # 程序入口
│   └── sim2real_msg_ros2/          # 消息功能包
├── docs/                           # 文档目录
├── README.md                       # 本文件
└── .gitignore                      # Git 忽略规则
```