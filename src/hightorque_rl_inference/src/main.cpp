/**
 * @file main.cpp
 * @brief Entry point for HighTorque RL inference node (Multi-threaded)
 *        高擎机电强化学习推理节点入口（多线程版本）
 * 
 * This file contains the main function that initializes the ROS node and
 * creates the HighTorqueRLInference object with multi-threaded executor.
 * It handles signal interrupts (Ctrl+C) for graceful shutdown.
 * 
 * 多线程架构：
 * - 传感器数据回调（IMU、关节状态）在独立线程中处理
 * - 控制循环（推理和指令发布）在独立线程中运行
 * - 指令输入（cmd_vel、joy）在独立线程中处理
 * 
 * Multi-threaded Architecture:
 * - Sensor data callbacks (IMU, joint states) processed in dedicated thread
 * - Control loop (inference and command publishing) runs in dedicated thread
 * - Command inputs (cmd_vel, joy) processed in dedicated thread
 * 
 * @author 高擎机电 (HighTorque Robotics)
 * @date 2025
 * @copyright Copyright (c) 2025 高擎机电 (HighTorque Robotics)
 */

#include "hightorque_rl_inference/hightorque_rl_inference.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <signal.h>

std::shared_ptr<hightorque_rl_inference::HighTorqueRLInference> g_demo;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> g_executor;

/**
 * @brief Signal handler for graceful shutdown
 *        优雅关闭的信号处理器
 * 
 * @param sig Signal number / 信号编号
 */
void signalHandler(int sig)
{
    (void)sig;
    RCLCPP_INFO(rclcpp::get_logger("main"), "收到中断信号，正在优雅关闭 / Interrupt signal received, shutting down gracefully");
    if (g_demo)
    {
        g_demo->stop();
    }
    if (g_executor)
    {
        g_executor->cancel();
    }
    rclcpp::shutdown();
}

/**
 * @brief Main entry point (Multi-threaded)
 *        主入口点（多线程版本）
 * 
 * Initializes ROS node with multi-threaded executor, creates HighTorqueRLInference 
 * instance, and starts the inference loop with parallel callback processing.
 * 
 * 初始化带有多线程执行器的 ROS 节点，创建 HighTorqueRLInference 实例，
 * 并启动具有并行回调处理的推理循环。
 * 
 * @param argc Argument count / 参数数量
 * @param argv Argument values / 参数值
 * @return int Exit code / 退出代码
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    signal(SIGINT, signalHandler);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "=== 创建多线程执行器 / Creating Multi-threaded Executor ===");
    
    // 创建多线程执行器，使用3个线程分别处理：
    // 1. 传感器数据回调（IMU、关节状态）
    // 2. 控制循环（推理和指令发布）
    // 3. 指令输入回调（cmd_vel、joy）
    // Create multi-threaded executor with 3 threads for:
    // 1. Sensor data callbacks (IMU, joint states)
    // 2. Control loop (inference and command publishing)
    // 3. Command input callbacks (cmd_vel, joy)
    g_executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
        rclcpp::ExecutorOptions(), 
        3  // 使用3个线程 / Use 3 threads
    );
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "=== 创建RL推理节点 / Creating RL Inference Node ===");
    g_demo = std::make_shared<hightorque_rl_inference::HighTorqueRLInference>();
    
    if (!g_demo->init())
    {
        RCLCPP_ERROR(g_demo->get_logger(), "初始化失败 / Initialization failed!");
        return -1;
    }
    
    RCLCPP_INFO(g_demo->get_logger(), "初始化成功 / Initialization successful");
    
    // ⚠️ 注意：必须先调用 run() 再添加到 executor
    // ⚠️ Important: Must call run() before adding to executor
    // 因为 run() 中使用了 spin_until_future_complete 等待 service 响应
    // Because run() uses spin_until_future_complete to wait for service response
    
    // 启动控制循环（等待service，创建定时器）
    // Start control loop (wait for service, create timer)
    g_demo->run();
    
    // 将节点添加到多线程执行器
    // Add node to multi-threaded executor
    g_executor->add_node(g_demo);
    
    RCLCPP_INFO(g_demo->get_logger(), "=== 开始多线程执行 / Starting Multi-threaded Execution ===");
    
    // 启动多线程执行器（阻塞直到收到shutdown信号）
    // Start multi-threaded executor (blocks until shutdown signal)
    g_executor->spin();
    
    RCLCPP_INFO(g_demo->get_logger(), "=== 主函数完成 / Main function completed ===");
    g_executor.reset();
    g_demo.reset();
    
    return 0;
}

