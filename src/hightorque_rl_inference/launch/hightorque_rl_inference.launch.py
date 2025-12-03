from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="hightorque_rl_inference",
            executable="hightorque_rl_inference_node",
            name="hightorque_rl_inference_node",
            output="screen",
            parameters=[
                {
                    # 可选：覆盖模型类型（与原 ROS1 launch 中保持一致）
                    "model_type": "pi_plus",
                    # 可选：配置文件路径，不设置时在节点内部使用默认 config_example.yaml
                    # "config_file": "<absolute_or_share_path>/config_example.yaml",
                }
            ],
        )
    ])


