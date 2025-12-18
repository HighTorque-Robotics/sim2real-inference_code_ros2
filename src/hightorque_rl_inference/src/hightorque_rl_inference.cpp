/**
 * @file hightorque_rl_inference.cpp
 * @brief HighTorque RL Inference Package - Main implementation
 *        高擎机电强化学习推理功能包 - 主实现文件
 * 
 * This file implements the core reinforcement learning inference system for
 * humanoid robot control. It handles:
 * - RKNN model loading and inference
 * - Observation space processing (36-dim with gait phase, velocities, joint states, IMU)
 * - Action space generation (12 DOF joint commands)
 * - State machine for safe mode transitions (NOT_READY -> STANDBY -> RUNNING)
 * - ROS topic subscriptions and publications
 * 
 * 本文件实现了人形机器人控制的核心强化学习推理系统。它处理：
 * - RKNN 模型加载和推理
 * - 观测空间处理（36维，包含步态相位、速度、关节状态、IMU）
 * - 动作空间生成（12自由度关节指令）
 * - 安全模式转换的状态机（未就绪 -> 待机 -> 运行）
 * - ROS 话题订阅和发布
 * 
 * @author 高擎机电 (HighTorque Robotics)
 * @date 2025
 * @copyright Copyright (c) 2025 高擎机电 (HighTorque Robotics)
 */

#include "hightorque_rl_inference/hightorque_rl_inference.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <atomic>
#include <sstream>
#include <limits>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace hightorque_rl_inference
{

    /**
     * @brief Load data from file at specific offset
     *        从文件的特定偏移量加载数据
     * 
     * @param fp File pointer / 文件指针
     * @param ofst Offset in bytes / 字节偏移量
     * @param sz Size to read / 读取大小
     * @return unsigned char* Loaded data / 加载的数据
     */
    static unsigned char* loadData(FILE* fp, size_t ofst, size_t sz)
    {
        unsigned char* data;
        int ret;

        data = NULL;

        if (NULL == fp)
        {
            return NULL;
        }

        ret = fseek(fp, ofst, SEEK_SET);
        if (ret != 0)
        {
            printf("blob seek failure.\n");
            return NULL;
        }

        data = (unsigned char*)malloc(sz);
        if (data == NULL)
        {
            printf("buffer malloc failure.\n");
            return NULL;
        }
        ret = fread(data, 1, sz, fp);
        return data;
    }

    /**
     * @brief Read entire file into memory
     *        将整个文件读入内存
     * 
     * @param filename Path to file / 文件路径
     * @param modelSize Output parameter for file size / 文件大小输出参数
     * @return unsigned char* File data / 文件数据
     */
    static unsigned char* readFileData(const char* filename, int* modelSize)
    {
        FILE* fp;
        unsigned char* data;

        fp = fopen(filename, "rb");
        if (NULL == fp)
        {
            printf("Open file %s failed.\n", filename);
            return NULL;
        }

        fseek(fp, 0, SEEK_END);
        int size = ftell(fp);

        data = loadData(fp, 0, size);
        fclose(fp);

        *modelSize = size;
        return data;
    }

    HighTorqueRLInference::HighTorqueRLInference(const rclcpp::NodeOptions& options)
        : rclcpp::Node("hightorque_rl_inference_node", options),
          quit_(false),
          stateReceived_(false),
          imuReceived_(false)
    {
        RCLCPP_INFO(this->get_logger(), "=== Loading configuration from YAML ===");

        std::string pkgPath = ament_index_cpp::get_package_share_directory("hightorque_rl_inference");
        std::string configFile = pkgPath + "/config_example.yaml";
        this->declare_parameter<std::string>("config_file", configFile);
        this->get_parameter("config_file", configFile);

        RCLCPP_INFO(this->get_logger(), "Loading config from: %s", configFile.c_str());

        try
        {
            YAML::Node config = YAML::LoadFile(configFile);

            // 读取基本参数（使用正确的默认值）
            numActions_ = config["num_actions"].as<int>(12);
            numSingleObs_ = config["num_single_obs"].as<int>(36);
            frameStack_ = config["frame_stack"].as<int>(1);
            clipObs_ = config["clip_obs"].as<double>(18.0);

            // 读取策略名称并构建完整路径
            std::string policyName = config["policy_name"].as<std::string>("policy_0322_12dof_4000.rknn");
            policyPath_ = pkgPath + "/policy/" + policyName;

            // 读取控制频率
            double dt = config["dt"].as<double>(0.001);
            int decimation = config["decimation"].as<int>(10);
            rlCtrlFreq_ = 1.0 / (dt * decimation);

            // 读取缩放参数（使用正确的默认值）
            cmdLinVelScale_ = config["cmd_lin_vel_scale"].as<double>(1.0);
            cmdAngVelScale_ = config["cmd_ang_vel_scale"].as<double>(1.25);
            rbtLinPosScale_ = config["rbt_lin_pos_scale"].as<double>(1.0);
            rbtLinVelScale_ = config["rbt_lin_vel_scale"].as<double>(1.0);
            rbtAngVelScale_ = config["rbt_ang_vel_scale"].as<double>(1.0);
            actionScale_ = config["action_scale"].as<double>(1.0);

            // 读取速度限制参数
            cmdVelXMin_ = config["cmd_vel_x_min"].as<double>(-0.55);
            cmdVelXMax_ = config["cmd_vel_x_max"].as<double>(0.55);
            cmdVelYMin_ = config["cmd_vel_y_min"].as<double>(-0.30);
            cmdVelYMax_ = config["cmd_vel_y_max"].as<double>(0.30);
            cmdVelYawMin_ = config["cmd_vel_yaw_min"].as<double>(-2.00);
            cmdVelYawMax_ = config["cmd_vel_yaw_max"].as<double>(2.00);

            // 读取动作限制
            std::vector<double> clipLower = config["clip_actions_lower"].as<std::vector<double>>();
            std::vector<double> clipUpper = config["clip_actions_upper"].as<std::vector<double>>();

            // 读取电机配置
            if (config["motor_direction"])
            {
                motorDirection_ = config["motor_direction"].as<std::vector<int>>();
            }
            if (config["urdf_dof_pos_offset"])
            {
                urdfOffset_ = config["urdf_dof_pos_offset"].as<std::vector<double>>();
            }
            if (config["map_index"])
            {
                actualToPolicyMap_ = config["map_index"].as<std::vector<int>>();
            }

            this->declare_parameter<std::string>("model_type", "pi_plus");
            this->get_parameter("model_type", modelType_);

            RCLCPP_INFO(this->get_logger(), "YAML config loaded successfully:");
            RCLCPP_INFO(this->get_logger(), "  num_actions: %d", numActions_);
            RCLCPP_INFO(this->get_logger(), "  num_single_obs: %d", numSingleObs_);
            RCLCPP_INFO(this->get_logger(), "  frame_stack: %d", frameStack_);
            RCLCPP_INFO(this->get_logger(), "  rl_ctrl_freq: %.1f Hz", rlCtrlFreq_);
            RCLCPP_INFO(this->get_logger(), "  policy_path: %s", policyPath_.c_str());
            RCLCPP_INFO(this->get_logger(), "  action_scale: %.2f", actionScale_);

            clipActionsLower_.resize(numActions_);
            clipActionsUpper_.resize(numActions_);
            for (int i = 0; i < numActions_ && i < (int)clipLower.size(); ++i)
            {
                clipActionsLower_[i] = static_cast<float>(clipLower[i]);
                clipActionsUpper_[i] = static_cast<float>(clipUpper[i]);
            }
        }
        catch (const YAML::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "YAML parsing error: %s", e.what());
            RCLCPP_ERROR(this->get_logger(), "Using default parameters from original launch file");
        }

        robotJointPositions_ = Eigen::VectorXd::Zero(numActions_);
        robotJointVelocities_ = Eigen::VectorXd::Zero(numActions_);
        motorJointPositions_ = Eigen::VectorXd::Zero(numActions_);
        motorJointVelocities_ = Eigen::VectorXd::Zero(numActions_);
        eulerAngles_ = Eigen::Vector3d::Zero();
        baseAngVel_ = Eigen::Vector3d::Zero();
        command_ = Eigen::Vector3d::Zero();
        action_ = Eigen::VectorXd::Zero(numActions_);

        observations_ = Eigen::VectorXd::Zero(numSingleObs_);
        for (int i = 0; i < frameStack_; ++i)
        {
            histObs_.push_back(Eigen::VectorXd::Zero(numSingleObs_));
        }
        obsInput_ = Eigen::MatrixXd::Zero(1, numSingleObs_ * frameStack_);

        quat_ = Eigen::Quaterniond::Identity();

        if (urdfOffset_.size() != static_cast<size_t>(numActions_))
        {
            urdfOffset_.assign(numActions_, 0.0);
        }
        if (motorDirection_.size() != static_cast<size_t>(numActions_))
        {
            motorDirection_.assign(numActions_, 1);
        }
        std::vector<int> defaultMap(numActions_);
        for (int i = 0; i < numActions_; ++i)
            defaultMap[i] = i;
        if (actualToPolicyMap_.size() != static_cast<size_t>(numActions_))
        {
            actualToPolicyMap_ = defaultMap;
        }

        this->declare_parameter<double>("steps_period", 60.0);
        this->get_parameter("steps_period", stepsPeriod_);
        step_ = 0.0;
        lastTrigger_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }

    HighTorqueRLInference::~HighTorqueRLInference()
    {
        quit_ = true;
        if (policyLoaded_)
            rknn_destroy(ctx_);
    }

    bool HighTorqueRLInference::init()
    {
        std::string presetTopic = "/" + modelType_ + "_preset";
        // auto presetQos = rclcpp::QoS(10).reliable().durability_volatile();
        auto presetQos = rclcpp::QoS(10);
        presetPub_ = this->create_publisher<std_msgs::msg::String>(presetTopic, presetQos);

        std::string topicName = "/" + modelType_ + "_all";
        auto cmdQos = rclcpp::QoS(1000).best_effort().durability_volatile();
        
        jointCmdPub_ = this->create_publisher<sensor_msgs::msg::JointState>(topicName, cmdQos);
        
        std::string obsTopicName = "/rl_observation";
        auto obsQos = rclcpp::QoS(100);
        obsPub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(obsTopicName, obsQos);
        
        std::string actionTopicName = "/rl_action";
        actionPub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(actionTopicName, obsQos);

        robotStateSub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/sim2real_master_node/rbt_state", 100,
            std::bind(&HighTorqueRLInference::robotStateCallback, this, std::placeholders::_1));
        motorStateSub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/sim2real_master_node/mtr_state", 100,
            std::bind(&HighTorqueRLInference::motorStateCallback, this, std::placeholders::_1));
        imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/yesense_imu/imu", 100,
            std::bind(&HighTorqueRLInference::imuCallback, this, std::placeholders::_1));
        cmdVelSub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 50,
            std::bind(&HighTorqueRLInference::cmdVelCallback, this, std::placeholders::_1));

        std::string joy_topic = "/joy";
        this->declare_parameter<std::string>("joy_topic", joy_topic);
        this->get_parameter("joy_topic", joy_topic);
        joySub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            joy_topic, 10,
            std::bind(&HighTorqueRLInference::joyCallback, this, std::placeholders::_1));

        rlPathClient_ = this->create_client<sim2real_msg_ros2::srv::Common>("/develop/rl_path");

        if (!loadPolicy())
        {
            return false;
        }
        return true;
    }

    bool HighTorqueRLInference::loadPolicy()
    {
        int modelSize = 0;
        unsigned char* modelData = readFileData(policyPath_.c_str(), &modelSize);
        if (!modelData)
        {
            return false;
        }
        int ret = rknn_init(&ctx_, modelData, modelSize, 0, nullptr);
        free(modelData);
        if (ret < 0)
        {
            return false;
        }
        ret = rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &ioNum_, sizeof(ioNum_));
        if (ret < 0)
        {
            return false;
        }

        memset(rknnInputs_, 0, sizeof(rknnInputs_));
        rknnInputs_[0].index = 0;
        rknnInputs_[0].size = obsInput_.size() * sizeof(float);
        rknnInputs_[0].pass_through = false;
        rknnInputs_[0].type = RKNN_TENSOR_FLOAT32;
        rknnInputs_[0].fmt = RKNN_TENSOR_NHWC;

        memset(rknnOutputs_, 0, sizeof(rknnOutputs_));
        rknnOutputs_[0].want_float = true;
        policyLoaded_ = true;
        return true;
    }

    /**
     * @brief Update observation vector for RL policy
     *        更新强化学习策略的观测向量
     * 
     * Constructs a 36-dimensional observation vector containing:
     * - [0-1]: Gait phase (sin/cos of step counter)
     * - [2-4]: Command velocities (x, y, yaw) scaled
     * - [5-16]: Joint positions (12 DOF) scaled
     * - [17-28]: Joint velocities (12 DOF) scaled
     * - [29-31]: Base angular velocity scaled
     * - [32-34]: Base orientation (Euler angles)
     * 
     * 构造一个 36 维观测向量，包含：
     * - [0-1]: 步态相位（步进计数器的 sin/cos）
     * - [2-4]: 速度指令（x, y, yaw）缩放后
     * - [5-16]: 关节位置（12 自由度）缩放后
     * - [17-28]: 关节速度（12 自由度）缩放后
     * - [29-31]: 基座角速度缩放后
     * - [32-34]: 基座姿态（欧拉角）
     */
    void HighTorqueRLInference::updateObservation()
    {
        if (observations_.size() != numSingleObs_)
        {
            observations_.resize(numSingleObs_);
        }

        step_ += 1.0 / stepsPeriod_;

        observations_[0] = currentState_ == STANDBY ? 1.0 : std::sin(2 * M_PI * step_);
        observations_[1] = currentState_ == STANDBY ? -1.0 : std::cos(2 * M_PI * step_);

        double cmdX = currentState_ == STANDBY ? 0.0 : std::clamp(command_[0], cmdVelXMin_, cmdVelXMax_);
        double cmdY = currentState_ == STANDBY ? 0.0 : std::clamp(command_[1], cmdVelYMin_, cmdVelYMax_);
        double cmdYaw = currentState_ == STANDBY ? 0.0 : std::clamp(command_[2], cmdVelYawMin_, cmdVelYawMax_);

        observations_[2] = cmdX * cmdLinVelScale_ * (cmdX < 0 ? 0.5 : 1.0);
        observations_[3] = cmdY * cmdLinVelScale_;
        observations_[4] = cmdYaw * cmdAngVelScale_;
        std::unique_lock<std::shared_timed_mutex> lk(mutex_);

        observations_.segment(5, numActions_) = robotJointPositions_ * rbtLinPosScale_;

        observations_.segment(17, numActions_) = robotJointVelocities_ * rbtLinVelScale_;
        lk.unlock();

        observations_.segment(29, 3) = baseAngVel_ * rbtAngVelScale_;

        observations_.segment(32, 3) = eulerAngles_;

        for (int i = 0; i < numSingleObs_; ++i)
        {
            observations_[i] = std::clamp(observations_[i], -clipObs_, clipObs_);
        }

        histObs_.push_back(observations_);
        histObs_.pop_front();
    }

    /**
     * @brief Run RKNN inference to generate actions
     *        运行 RKNN 推理以生成动作
     * 
     * This function:
     * 1. Prepares input tensor from observation history (frame stacking)
     * 2. Converts Eigen matrix to float vector for RKNN
     * 3. Sets RKNN inputs and runs inference
     * 4. Retrieves and clamps output actions to safe limits
     * 5. Releases RKNN output buffers
     * 
     * 此函数：
     * 1. 从观测历史准备输入张量（帧堆叠）
     * 2. 将 Eigen 矩阵转换为 RKNN 的 float 向量
     * 3. 设置 RKNN 输入并运行推理
     * 4. 检索输出动作并裁剪到安全范围
     * 5. 释放 RKNN 输出缓冲区
     */
    void HighTorqueRLInference::updateAction()
    {
        for (int i = 0; i < frameStack_; ++i)
        {
            obsInput_.block(0, i * numSingleObs_, 1, numSingleObs_) = histObs_[i].transpose();
        }

        std::vector<float> inputData(obsInput_.size());
        for (size_t i = 0; i < obsInput_.size(); ++i)
        {
            inputData[i] = obsInput_(i);
        }

        rknnInputs_[0].buf = inputData.data();
        rknn_inputs_set(ctx_, ioNum_.n_input, rknnInputs_);
        rknn_run(ctx_, nullptr);
        rknn_outputs_get(ctx_, ioNum_.n_output, rknnOutputs_, nullptr);

        float* outputData = static_cast<float*>(rknnOutputs_[0].buf);
        for (int i = 0; i < numActions_; ++i)
        {
            action_[i] = std::clamp(outputData[i], clipActionsLower_[i], clipActionsUpper_[i]);
        }

        rknn_outputs_release(ctx_, ioNum_.n_output, rknnOutputs_);

        if (actionPub_)
        {
            std_msgs::msg::Float64MultiArray actionMsg;
            actionMsg.layout.dim.resize(1);
            actionMsg.layout.dim[0].label = "action";
            actionMsg.layout.dim[0].size = action_.size();
            actionMsg.layout.dim[0].stride = action_.size();
            actionMsg.data.resize(action_.size());
            for (int i = 0; i < action_.size(); ++i)
            {
                actionMsg.data[i] = action_[i];
            }
            actionPub_->publish(actionMsg);
        }
    }

    void HighTorqueRLInference::quat2euler()
    {
        double x = quat_.x();
        double y = quat_.y();
        double z = quat_.z();
        double w = quat_.w();

        double t0 = 2.0 * (w * x + y * z);
        double t1 = 1.0 - 2.0 * (x * x + y * y);
        double roll = std::atan2(t0, t1);

        double t2 = std::clamp(2.0 * (w * y - z * x), -1.0, 1.0);
        double pitch = std::asin(t2);

        double t3 = 2.0 * (w * z + x * y);
        double t4 = 1.0 - 2.0 * (y * y + z * z);
        double yaw = std::atan2(t3, t4);

        eulerAngles_ << roll, pitch, yaw;
    }

    void HighTorqueRLInference::robotStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() < static_cast<size_t>(numActions_))
        {
            return;
        }

        std::unique_lock<std::shared_timed_mutex> lk(mutex_);
        for (int i = 0; i < numActions_; ++i)
        {
            robotJointPositions_[i] = msg->position[i];
            if (msg->velocity.size() >= static_cast<size_t>(numActions_))
            {
                robotJointVelocities_[i] = msg->velocity[i];
            }
        }

        if (!stateReceived_)
        {
            stateReceived_ = true;
        }
    }

    void HighTorqueRLInference::motorStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() < static_cast<size_t>(numActions_))
        {
            return;
        }

        for (int i = 0; i < numActions_; ++i)
        {
            int policyIdx = actualToPolicyMap_[i];
            if (policyIdx >= 0 && policyIdx < numActions_)
            {
                motorJointPositions_[policyIdx] = msg->position[i] * motorDirection_[i];
                if (msg->velocity.size() >= static_cast<size_t>(numActions_))
                {
                    motorJointVelocities_[policyIdx] = msg->velocity[i] * motorDirection_[i];
                }
            }
        }

        if (!stateReceived_)
        {
            stateReceived_ = true;
        }
    }

    void HighTorqueRLInference::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        quat_.x() = msg->orientation.x;
        quat_.y() = msg->orientation.y;
        quat_.z() = msg->orientation.z;
        quat_.w() = msg->orientation.w;
        quat2euler();
        baseAngVel_[0] = msg->angular_velocity.x;
        baseAngVel_[1] = msg->angular_velocity.y;
        baseAngVel_[2] = msg->angular_velocity.z;
        if (!imuReceived_)
        {
            imuReceived_ = true;
        }
    }

    void HighTorqueRLInference::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        command_[0] = std::clamp(msg->linear.x, cmdVelXMin_, cmdVelXMax_);
        command_[1] = std::clamp(msg->linear.y, cmdVelYMin_, cmdVelYMax_);
        command_[2] = std::clamp(msg->angular.z, cmdVelYawMin_, cmdVelYawMax_);
    }

    void HighTorqueRLInference::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        int axis2 = 2, axis5 = 5, btn_start = 7, btn_lb = 4;
        this->get_parameter_or<int>("joy_axis2", axis2, axis2);
        this->get_parameter_or<int>("joy_axis5", axis5, axis5);
        this->get_parameter_or<int>("joy_button_start", btn_start, btn_start);
        this->get_parameter_or<int>("joy_button_lb", btn_lb, btn_lb);

        bool ltPressed = (axis2 >= 0 && axis2 < (int)msg->axes.size()) && (std::abs(msg->axes[axis2]) > 0.8);
        bool rtPressed = (axis5 >= 0 && axis5 < (int)msg->axes.size()) && (std::abs(msg->axes[axis5]) > 0.8);
        bool startPressed = (btn_start >= 0 && btn_start < (int)msg->buttons.size()) && (msg->buttons[btn_start] == 1);
        bool lbPressed = (btn_lb >= 0 && btn_lb < (int)msg->buttons.size()) && (msg->buttons[btn_lb] == 1);

        bool triggerReset = ltPressed && rtPressed && startPressed;
        bool triggerToggle = ltPressed && rtPressed && lbPressed;

        if (triggerReset && (this->now() - lastTrigger_).seconds() > 1.0)
        {
            if (currentState_ == NOT_READY)
            {
                lastTrigger_ = this->now();

                double reset_duration = 2.0;
                this->get_parameter_or<double>("reset_duration", reset_duration, reset_duration);

                std_msgs::msg::String preset;
                preset.data = "zero:" + std::to_string(reset_duration);

                presetPub_->publish(preset);
                auto sleep_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(reset_duration));
                rclcpp::sleep_for(sleep_duration);
                currentState_ = STANDBY;
            }
        }

        if (triggerToggle && (this->now() - lastTrigger_).seconds() > 1.0)
        {
            if (currentState_ == STANDBY || currentState_ == RUNNING)
            {
                lastTrigger_ = this->now();

                if (currentState_ == STANDBY)
                {
                    currentState_ = RUNNING;
                }
                else if (currentState_ == RUNNING)
                {
                    currentState_ = STANDBY;
                }
            }
        }
    }

    void HighTorqueRLInference::run()
    {
        RCLCPP_INFO(this->get_logger(), "等待外部service /develop/rl_path 可用...");
        if (!rlPathClient_->wait_for_service(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "Service /develop/rl_path 超时未响应");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "调用 /develop/rl_path service...");
        auto request = std::make_shared<sim2real_msg_ros2::srv::Common::Request>();
        request->enable = true;
        request->str = "/home/hightorque/zy_workspace/sim2real_master_ros2/src/sim2real_master_ros2/src/sim2real_master/config/walk/devel_control.yaml";
        auto result = rlPathClient_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == 
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = result.get();
            if (response->result)
            {
                RCLCPP_INFO(this->get_logger(), "Service调用成功: %s", response->message.c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Service调用失败: %s", response->error_message.c_str());
                return;
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "无法调用 /develop/rl_path service");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "开始主循环...");
        rclcpp::Rate rate(rlCtrlFreq_);

        while (rclcpp::ok() && !quit_)
        {
            rclcpp::spin_some(this->get_node_base_interface());

            if (currentState_ == NOT_READY)
            {
                rate.sleep();
                continue;
            }

            rclcpp::spin_some(this->get_node_base_interface());
            
            updateObservation();
            
            std_msgs::msg::Float64MultiArray obsMsg;
            obsMsg.layout.dim.resize(1);
            obsMsg.layout.dim[0].label = "observation";
            obsMsg.layout.dim[0].size = observations_.size();
            obsMsg.layout.dim[0].stride = observations_.size();
            obsMsg.data.resize(observations_.size());
            for (size_t i = 0; i < observations_.size(); ++i)
            {
                obsMsg.data[i] = observations_[i];
            }
            obsPub_->publish(obsMsg);
            
            rclcpp::spin_some(this->get_node_base_interface());
            
            updateAction();

            sensor_msgs::msg::JointState msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = "";

            msg.name.resize(22);
            msg.position.resize(22);
            msg.velocity.resize(22);
            msg.effort.resize(22);

            double scale = (currentState_ == RUNNING) ? actionScale_ : 0.05;
            // double scale = actionScale_;
            
            for (int i = 0; i < 12; ++i)
            {
                msg.name[i] = "test" + std::to_string(i + 1);
                msg.position[i] = action_[i] * scale;
                msg.velocity[i] = 0.0;
                msg.effort[i] = 0.0;
            }
            for (int i = 12; i < 22; ++i)
            {
                msg.name[i] = "test" + std::to_string(i + 1);
                msg.position[i] = 0.0;
                msg.velocity[i] = 0.0;
                msg.effort[i] = 0.0;
            }
           

            jointCmdPub_->publish(msg);
            rate.sleep();
        }
    }

} // namespace hightorque_rl_inference

