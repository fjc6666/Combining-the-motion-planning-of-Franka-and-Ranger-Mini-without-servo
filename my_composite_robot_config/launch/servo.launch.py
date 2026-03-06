import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

# 辅助函数：加载 yaml
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # 1. 获取 MoveIt Servo 配置
    servo_yaml = load_yaml("my_composite_robot_config", "config/moveit_servo.yaml")
    
    # 2. 获取机器人描述 (URDF & SRDF) - 这一步必须和 bringup_gazebo 保持一致
    # 注意：为了简化，这里假设你已经能正确获取 robot_description_content
    # 实际使用时，你可能需要复制 bringup_gazebo.launch.py 里生成 robot_description 的那段 Command 代码
    # 这里我只写核心逻辑：
    
    # 获取 description 包路径
    description_pkg_path = get_package_share_directory('composite_robot_description')
    xacro_file = os.path.join(description_pkg_path, 'urdf', 'mobile_manipulator.urdf.xacro')
    robot_description_content = Command(['xacro ', xacro_file])
    
    robot_description = {"robot_description": robot_description_content}
    
    srdf_file = os.path.join(get_package_share_directory("my_composite_robot_config"), "config", "franka_ranger_combined.srdf")
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    # 3. 配置 Servo 节点
    servo_node = ComposableNode(
        package="moveit_servo",
        plugin="moveit_servo::ServoNode",
        name="servo_node",
        parameters=[
            servo_yaml,
            robot_description,
            robot_description_semantic,
        ],
    )

    # 4. 启动容器
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[servo_node],
        output="screen",
    )

    return LaunchDescription([container])