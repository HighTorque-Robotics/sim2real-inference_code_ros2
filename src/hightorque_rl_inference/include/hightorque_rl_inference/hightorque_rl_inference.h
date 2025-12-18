#ifndef HIGHTORQUE_RL_INFERENCE_H
#define HIGHTORQUE_RL_INFERENCE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "sim2real_msg_ros2/srv/common.hpp"
#include <Eigen/Dense>
#include <memory>
#include <deque>
#include <shared_mutex>
#include <mutex>
#include <atomic>
#include <vector>

#ifdef PLATFORM_ARM
#include "rknn_api.h"
#endif

namespace hightorque_rl_inference
{

    class HighTorqueRLInference : public rclcpp::Node
    {
        public:
            HighTorqueRLInference(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
            ~HighTorqueRLInference();

            bool init();
            void run();
            void stop() { quit_ = true; }

        private:
            // ===== 核心功能函数 / Core Functions =====
            bool loadPolicy();
            void updateObservation();
            void updateAction();
            void quat2euler();
            
            // ===== 回调函数 / Callback Functions =====
            void robotStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
            void motorStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
            void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
            void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
            void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
            
            // ===== 控制循环定时器回调 / Control Loop Timer Callback =====
            void controlLoopCallback();

            // ===== ROS2 发布者和订阅者 / Publishers and Subscribers =====
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointCmdPub_;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr presetPub_;
            rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr obsPub_;
            rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr actionPub_;
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr robotStateSub_;
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motorStateSub_;
            rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub_;
            rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySub_;
            rclcpp::Client<sim2real_msg_ros2::srv::Common>::SharedPtr rlPathClient_;
            
            // ===== 定时器 / Timer =====
            rclcpp::TimerBase::SharedPtr controlTimer_;
            
            // ===== 回调组（用于多线程） / Callback Groups (for multi-threading) =====
            rclcpp::CallbackGroup::SharedPtr sensorCallbackGroup_;    // 传感器数据（IMU、关节状态）
            rclcpp::CallbackGroup::SharedPtr controlCallbackGroup_;   // 控制循环
            rclcpp::CallbackGroup::SharedPtr commandCallbackGroup_;   // 指令输入（cmd_vel、joy）

            std::string modelType_;
            std::string policyPath_;
            int numActions_;
            int numSingleObs_;
            int frameStack_;
            double rlCtrlFreq_;
            double clipObs_;

            double cmdLinVelScale_;
            double cmdAngVelScale_;
            double rbtLinPosScale_;
            double rbtLinVelScale_;
            double rbtAngVelScale_;
            double actionScale_;

            double cmdVelXMin_;
            double cmdVelXMax_;
            double cmdVelYMin_;
            double cmdVelYMax_;
            double cmdVelYawMin_;
            double cmdVelYawMax_;

            std::vector<float> clipActionsLower_;
            std::vector<float> clipActionsUpper_;

            // ===== 机器人状态数据（需要线程保护） / Robot State Data (thread-protected) =====
            // Robot关节数据（相对位置，来自rbt_state话题）
            Eigen::VectorXd robotJointPositions_;
            Eigen::VectorXd robotJointVelocities_;
            std::mutex robotStateMutex_;  // 保护robotJointPositions_和robotJointVelocities_

            // Motor关节数据（绝对角度，来自mtr_state话题，用于策略推理）
            Eigen::VectorXd motorJointPositions_;
            Eigen::VectorXd motorJointVelocities_;
            std::mutex motorStateMutex_;  // 保护motorJointPositions_和motorJointVelocities_

            // IMU数据
            Eigen::Quaterniond quat_;
            Eigen::Vector3d eulerAngles_;
            Eigen::Vector3d baseAngVel_;
            std::mutex imuMutex_;  // 保护quat_, eulerAngles_, baseAngVel_

            // 速度指令
            Eigen::Vector3d command_;
            std::mutex commandMutex_;  // 保护command_

            Eigen::VectorXd observations_;
            std::deque<Eigen::VectorXd> histObs_;
            Eigen::MatrixXd obsInput_;

            Eigen::VectorXd action_;

            std::vector<double> urdfOffset_;
            std::vector<int> motorDirection_;
            std::vector<int> actualToPolicyMap_; // 输入关节顺序映射：从mtr_state顺序到policy训练时的顺序

            double stepsPeriod_;
            double step_;
            std::mutex stepMutex_;  // 保护step_

            std::atomic<bool> quit_;
            std::atomic<bool> stateReceived_;
            std::atomic<bool> imuReceived_;
            rclcpp::Time lastTrigger_;
            std::mutex triggerMutex_;  // 保护lastTrigger_

#ifdef PLATFORM_ARM
            rknn_context ctx_{};
            rknn_input_output_num ioNum_{};
            rknn_input rknnInputs_[1]{};
            rknn_output rknnOutputs_[1]{};
            bool policyLoaded_{false};
#endif

            // 状态机（类内成员）
            enum State
            {
                NOT_READY,
                STANDBY,
                RUNNING
            };
            State currentState_ = NOT_READY;
    };

} // namespace hightorque_rl_inference

#endif

