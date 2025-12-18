# Development Guide / 开发指南

**Developed by 高擎机电 (HighTorque Robotics)**

**ROS2 Version:** Foxy Fitzroy

[English](#english) | [中文](#中文)

---

## English

This guide explains how to extend and modify the HighTorque RL Inference Demo for your own robotics projects.

### Project Structure

```
src/hightorque_rl_inference/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package metadata
├── config_example.yaml     # Runtime configuration
├── include/
│   └── hightorque_rl_inference/
│       └── hightorque_rl_inference.h    # Main class declaration
├── launch/
│   └── hightorque_rl_inference.launch.py   # Launch file
├── policy/
│   └── *.rknn                  # RKNN model files
└── src/
    ├── hightorque_rl_inference.cpp      # Main implementation
    └── main.cpp                # Entry point
```

### Key Classes and Functions

#### HighTorqueRLInference Class

The main class handling RL inference:

**Key Members:**
- `numActions_`: Number of robot joints (DOF)
- `numSingleObs_`: Observation vector dimension
- `observations_`: Current observation vector
- `action_`: Current action vector
- `ctx_`: RKNN inference context

**Key Methods:**
- `init()`: Initialize ROS2 topics and load policy
- `loadPolicy()`: Load RKNN model from file
- `updateObservation()`: Build observation vector
- `updateAction()`: Run inference to generate actions
- `run()`: Start control loop timer

---

## Customizing Observation Space

### Current Observation Structure (36-dim)

```cpp
observations_[0]     = sin(2*pi*step)      // Gait phase
observations_[1]     = cos(2*pi*step)      // Gait phase
observations_[2:4]   = cmd velocities      // x, y, yaw
observations_[5:16]  = joint positions     // 12 DOF
observations_[17:28] = joint velocities    // 12 DOF
observations_[29:31] = base angular vel    // 3-axis
observations_[32:34] = base orientation    // Euler angles
```

### Adding New Observations

**Example: Add base linear velocity**

1. **Increase `num_single_obs` in config:**
   ```yaml
   num_single_obs: 39  # Was 36, now 36+3
   ```

2. **Add member variable in header file** (`hightorque_rl_inference.h`):
   ```cpp
   Eigen::Vector3d baseLinVel_;
   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr baseVelSub_;
   ```

3. **Subscribe to additional topic** in `init()` function:
   ```cpp
   baseVelSub_ = this->create_subscription<geometry_msgs::msg::Twist>(
       "/base_velocity", 10,
       std::bind(&HighTorqueRLInference::baseVelCallback, this, std::placeholders::_1));
   ```

4. **Add callback to store data:**
   ```cpp
   void HighTorqueRLInference::baseVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
   {
       baseLinVel_(0) = msg->linear.x;
       baseLinVel_(1) = msg->linear.y;
       baseLinVel_(2) = msg->linear.z;
   }
   ```

5. **Update `updateObservation()` function:**
   ```cpp
   void HighTorqueRLInference::updateObservation()
   {
       // ... existing observations ...
       
       // Add new observations at the end
       observations_[35] = baseLinVel_(0);
       observations_[36] = baseLinVel_(1);
       observations_[37] = baseLinVel_(2);
       
       // ... rest of function ...
   }
   ```

6. **Retrain your policy** with the new observation space

---

## Customizing Action Space

### Current Action Structure (12-dim)

12 joint position targets, one per actuated joint.

### Adding New Actions

**Example: Include gripper control**

1. **Increase `num_actions` in config:**
   ```yaml
   num_actions: 13  # Was 12, now 12+1 for gripper
   ```

2. **Update action limits:**
   ```yaml
   clip_actions_lower: [-1.00, ..., 0.0]   # Add gripper lower
   clip_actions_upper: [1.00, ..., 1.0]    # Add gripper upper
   ```

3. **Add publisher in header file:**
   ```cpp
   rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripperPub_;
   ```

4. **Initialize publisher in `init()` function:**
   ```cpp
   gripperPub_ = this->create_publisher<std_msgs::msg::Float64>("/gripper_cmd", 10);
   ```

5. **Modify `controlLoopCallback()` function** to handle gripper action:
   ```cpp
   void HighTorqueRLInference::controlLoopCallback()
   {
       // ... existing code ...
       
       // Separate gripper action
       double gripperCmd = action_[12];
       
       // Publish gripper command
       std_msgs::msg::Float64 gripperMsg;
       gripperMsg.data = gripperCmd;
       gripperPub_->publish(gripperMsg);
   }
   ```

---

## Model Conversion

### PyTorch → ONNX → RKNN

#### Step 1: Export to ONNX

```python
import torch
import torch.onnx

# Load your trained policy
policy = torch.jit.load("policy.pt")
policy.eval()

# Create dummy input (batch_size=1, obs_dim=36)
dummy_input = torch.randn(1, 36)

# Export to ONNX
torch.onnx.export(
    policy,
    dummy_input,
    "policy.onnx",
    input_names=['observation'],
    output_names=['action'],
    dynamic_axes={
        'observation': {0: 'batch_size'},
        'action': {0: 'batch_size'}
    },
    opset_version=11
)
```

#### Step 2: Convert ONNX to RKNN

```python
from rknn.api import RKNN

# Create RKNN object
rknn = RKNN(verbose=True)

# Config
print('--> Config model')
rknn.config(
    mean_values=[[0, 0, 0]],
    std_values=[[1, 1, 1]],
    target_platform='rk3588'
)

# Load ONNX model
print('--> Loading model')
ret = rknn.load_onnx(model='policy.onnx')

# Build RKNN model
print('--> Building model')
ret = rknn.build(do_quantization=False)

# Export RKNN model
print('--> Export model')
ret = rknn.export_rknn('policy.rknn')

rknn.release()
```

#### Step 3: Quantization (Optional, for better performance)

```python
# Create dataset for quantization
def dataset_generator():
    # Generate or load representative observation samples
    for i in range(100):
        obs = np.random.randn(1, 36).astype(np.float32)
        yield [obs]

# Build with quantization
rknn.config(
    mean_values=[[0, 0, 0]],
    std_values=[[1, 1, 1]],
    target_platform='rk3588',
    quantized_dtype='asymmetric_quantized-8'
)

rknn.build(do_quantization=True, dataset=dataset_generator)
rknn.export_rknn('policy_quantized.rknn')
```

---

## Debugging Techniques

### 1. Print Observation Values

Add to `updateObservation()`:
```cpp
void HighTorqueRLInference::updateObservation()
{
    // ... build observations ...
    
    // Debug print
    RCLCPP_DEBUG(this->get_logger(), "Observations:");
    for (int i = 0; i < numSingleObs_; ++i) {
        RCLCPP_DEBUG(this->get_logger(), "  [%d] = %.4f", i, observations_[i]);
    }
}
```

### 2. Log Action Values

Add to `updateAction()`:
```cpp
void HighTorqueRLInference::updateAction()
{
    // ... run inference ...
    
    // Debug print
    std::stringstream ss;
    ss << "Actions: ";
    for (int i = 0; i < numActions_; ++i) {
        ss << action_[i] << " ";
    }
    RCLCPP_DEBUG(this->get_logger(), "%s", ss.str().c_str());
}
```

### 3. Visualize Observations

Observations are already published to `/rl_observation` topic. You can monitor them:
```bash
ros2 topic echo /rl_observation
```

### 4. Record Data for Analysis

```bash
# Record all relevant topics
ros2 bag record /sim2real_master_node/rbt_state \
                /sim2real_master_node/mtr_state \
                /yesense_imu/imu \
                /cmd_vel \
                /pi_plus_all \
                /rl_observation \
                /rl_action

# Play back later
ros2 bag play <bag_name>
```

### 5. Performance Profiling

Add timing to measure inference performance:
```cpp
#include <chrono>

void HighTorqueRLInference::updateAction()
{
    auto start = std::chrono::high_resolution_clock::now();
    
    // ... inference code ...
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    RCLCPP_DEBUG(this->get_logger(), "Inference time: %ld μs", duration.count());
}
```

---

## Testing Best Practices

### 1. Test in STANDBY Mode First

Always start testing in STANDBY mode (uses minimal action scale 0.05):
1. Launch the node
2. Press `LT + RT + START` to enter STANDBY
3. Observe robot behavior
4. Check for unexpected movements
5. Verify all joints receive commands

### 2. Gradual Testing

1. Start with low `action_scale` (0.3-0.5)
2. Test basic standing/balancing
3. Gradually increase `action_scale`
4. Test velocity commands at slow speeds first
5. Only switch to RUNNING mode after STANDBY tests pass

### 3. Monitor Topics

Use these commands to monitor system:
```bash
# Check topic rates
ros2 topic hz /sim2real_master_node/rbt_state
ros2 topic hz /pi_plus_all

# Monitor joint commands
ros2 topic echo /pi_plus_all

# Check observations
ros2 topic echo /rl_observation
```

---

## 中文

本指南解释如何扩展和修改 HighTorque RL 推理演示以用于您自己的机器人项目。

### 项目结构

```
src/hightorque_rl_inference/
├── CMakeLists.txt          # 构建配置
├── package.xml             # 包元数据
├── config_example.yaml     # 运行时配置
├── include/
│   └── hightorque_rl_inference/
│       └── hightorque_rl_inference.h    # 主类声明
├── launch/
│   └── hightorque_rl_inference.launch.py   # 启动文件
├── policy/
│   └── *.rknn                  # RKNN 模型文件
└── src/
    ├── hightorque_rl_inference.cpp      # 主实现
    └── main.cpp                # 入口点
```

### 关键类和函数

#### HighTorqueRLInference 类

处理 RL 推理的主类：

**关键成员：**
- `numActions_`：机器人关节数（自由度）
- `numSingleObs_`：观测向量维度
- `observations_`：当前观测向量
- `action_`：当前动作向量
- `ctx_`：RKNN 推理上下文

**关键方法：**
- `init()`：初始化 ROS2 话题并加载策略
- `loadPolicy()`：从文件加载 RKNN 模型
- `updateObservation()`：构建观测向量
- `updateAction()`：运行推理以生成动作
- `run()`：启动控制循环定时器

---

## 自定义观测空间

### 当前观测结构（36维）

```cpp
observations_[0]     = sin(2*pi*step)      // 步态相位
observations_[1]     = cos(2*pi*step)      // 步态相位
observations_[2:4]   = cmd velocities      // x, y, yaw
observations_[5:16]  = joint positions     // 12 自由度
observations_[17:28] = joint velocities    // 12 自由度
observations_[29:31] = base angular vel    // 3 轴
observations_[32:34] = base orientation    // 欧拉角
```

### 添加新观测

**示例：添加基座线速度**

1. **在配置中增加 `num_single_obs`：**
   ```yaml
   num_single_obs: 39  # 原来是 36，现在 36+3
   ```

2. **在头文件中添加成员变量** (`hightorque_rl_inference.h`):
   ```cpp
   Eigen::Vector3d baseLinVel_;
   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr baseVelSub_;
   ```

3. **在 `init()` 函数中订阅额外话题：**
   ```cpp
   baseVelSub_ = this->create_subscription<geometry_msgs::msg::Twist>(
       "/base_velocity", 10,
       std::bind(&HighTorqueRLInference::baseVelCallback, this, std::placeholders::_1));
   ```

4. **添加回调以存储数据：**
   ```cpp
   void HighTorqueRLInference::baseVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
   {
       baseLinVel_(0) = msg->linear.x;
       baseLinVel_(1) = msg->linear.y;
       baseLinVel_(2) = msg->linear.z;
   }
   ```

5. **更新 `updateObservation()` 函数：**
   ```cpp
   void HighTorqueRLInference::updateObservation()
   {
       // ... 现有观测 ...
       
       // 在末尾添加新观测
       observations_[35] = baseLinVel_(0);
       observations_[36] = baseLinVel_(1);
       observations_[37] = baseLinVel_(2);
       
       // ... 函数的其余部分 ...
   }
   ```

6. **使用新观测空间重新训练策略**

---

## 自定义动作空间

### 当前动作结构（12维）

12 个关节位置目标，每个驱动关节一个。

### 添加新动作

**示例：包含夹爪控制**

1. **在配置中增加 `num_actions`：**
   ```yaml
   num_actions: 13  # 原来是 12，现在 12+1 用于夹爪
   ```

2. **更新动作限制：**
   ```yaml
   clip_actions_lower: [-1.00, ..., 0.0]   # 添加夹爪下限
   clip_actions_upper: [1.00, ..., 1.0]    # 添加夹爪上限
   ```

3. **在头文件中添加发布者：**
   ```cpp
   rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripperPub_;
   ```

4. **在 `init()` 函数中初始化发布者：**
   ```cpp
   gripperPub_ = this->create_publisher<std_msgs::msg::Float64>("/gripper_cmd", 10);
   ```

5. **修改 `controlLoopCallback()` 函数**以处理夹爪动作：
   ```cpp
   void HighTorqueRLInference::controlLoopCallback()
   {
       // ... 现有代码 ...
       
       // 分离夹爪动作
       double gripperCmd = action_[12];
       
       // 发布夹爪命令
       std_msgs::msg::Float64 gripperMsg;
       gripperMsg.data = gripperCmd;
       gripperPub_->publish(gripperMsg);
   }
   ```

---

## 模型转换

### PyTorch → ONNX → RKNN

#### 步骤 1：导出到 ONNX

```python
import torch
import torch.onnx

# 加载训练好的策略
policy = torch.jit.load("policy.pt")
policy.eval()

# 创建虚拟输入 (batch_size=1, obs_dim=36)
dummy_input = torch.randn(1, 36)

# 导出到 ONNX
torch.onnx.export(
    policy,
    dummy_input,
    "policy.onnx",
    input_names=['observation'],
    output_names=['action'],
    dynamic_axes={
        'observation': {0: 'batch_size'},
        'action': {0: 'batch_size'}
    },
    opset_version=11
)
```

#### 步骤 2：将 ONNX 转换为 RKNN

```python
from rknn.api import RKNN

# 创建 RKNN 对象
rknn = RKNN(verbose=True)

# 配置
print('--> Config model')
rknn.config(
    mean_values=[[0, 0, 0]],
    std_values=[[1, 1, 1]],
    target_platform='rk3588'
)

# 加载 ONNX 模型
print('--> Loading model')
ret = rknn.load_onnx(model='policy.onnx')

# 构建 RKNN 模型
print('--> Building model')
ret = rknn.build(do_quantization=False)

# 导出 RKNN 模型
print('--> Export model')
ret = rknn.export_rknn('policy.rknn')

rknn.release()
```

#### 步骤 3：量化（可选，以获得更好的性能）

```python
# 为量化创建数据集
def dataset_generator():
    # 生成或加载代表性观测样本
    for i in range(100):
        obs = np.random.randn(1, 36).astype(np.float32)
        yield [obs]

# 使用量化构建
rknn.config(
    mean_values=[[0, 0, 0]],
    std_values=[[1, 1, 1]],
    target_platform='rk3588',
    quantized_dtype='asymmetric_quantized-8'
)

rknn.build(do_quantization=True, dataset=dataset_generator)
rknn.export_rknn('policy_quantized.rknn')
```

---

## 调试技巧

### 1. 打印观测值

添加到 `updateObservation()`：
```cpp
void HighTorqueRLInference::updateObservation()
{
    // ... 构建观测 ...
    
    // 调试打印
    RCLCPP_DEBUG(this->get_logger(), "Observations:");
    for (int i = 0; i < numSingleObs_; ++i) {
        RCLCPP_DEBUG(this->get_logger(), "  [%d] = %.4f", i, observations_[i]);
    }
}
```

### 2. 记录动作值

添加到 `updateAction()`：
```cpp
void HighTorqueRLInference::updateAction()
{
    // ... 运行推理 ...
    
    // 调试打印
    std::stringstream ss;
    ss << "Actions: ";
    for (int i = 0; i < numActions_; ++i) {
        ss << action_[i] << " ";
    }
    RCLCPP_DEBUG(this->get_logger(), "%s", ss.str().c_str());
}
```

### 3. 可视化观测

观测值已发布到 `/rl_observation` 话题。您可以监控它们：
```bash
ros2 topic echo /rl_observation
```

### 4. 记录数据以供分析

```bash
# 记录所有相关话题
ros2 bag record /sim2real_master_node/rbt_state \
                /sim2real_master_node/mtr_state \
                /yesense_imu/imu \
                /cmd_vel \
                /pi_plus_all \
                /rl_observation \
                /rl_action

# 稍后回放
ros2 bag play <bag_name>
```

### 5. 性能分析

添加计时以测量推理性能：
```cpp
#include <chrono>

void HighTorqueRLInference::updateAction()
{
    auto start = std::chrono::high_resolution_clock::now();
    
    // ... 推理代码 ...
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    RCLCPP_DEBUG(this->get_logger(), "Inference time: %ld μs", duration.count());
}
```

---

## 测试最佳实践

### 1. 先在 STANDBY 模式下测试

始终在 STANDBY 模式下开始测试（使用最小动作缩放 0.05）：
1. 启动节点
2. 按 `LT + RT + START` 进入 STANDBY
3. 观察机器人行为
4. 检查意外运动
5. 验证所有关节是否接收指令

### 2. 渐进式测试

1. 从低 `action_scale` 开始（0.3-0.5）
2. 测试基本站立/平衡
3. 逐渐增加 `action_scale`
4. 首先以慢速测试速度指令
5. 只有在 STANDBY 测试通过后才切换到 RUNNING 模式

### 3. 监控话题

使用这些命令监控系统：
```bash
# 检查话题频率
ros2 topic hz /sim2real_master_node/rbt_state
ros2 topic hz /pi_plus_all

# 监控关节指令
ros2 topic echo /pi_plus_all

# 检查观测值
ros2 topic echo /rl_observation
```

---

**高擎机电（HighTorque Robotics）**  
**最后更新**：2025年12月
