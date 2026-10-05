import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command

# ==========================================
# 辅助函数：安全地加载 YAML 配置文件
# ==========================================
def load_yaml(package_name, file_path):
    """
    根据功能包名和文件相对路径，读取 yaml 文件并转换为 Python 字典。
    增加了路径检查，如果文件不存在会抛出带有绝对路径的致命错误，方便排错。
    """
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    
    if not os.path.exists(absolute_file_path):
        raise FileNotFoundError(f"【致命错误】找不到配置文件: {absolute_file_path}")
        
    with open(absolute_file_path, "r") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    # ==========================================
    # 1. 获取 MoveIt Servo 的算法配置参数
    # ==========================================
    # 读取我们之前写好的 servo_config.yaml
    servo_yaml = load_yaml("my_composite_robot_config", "config/servo_config.yaml")
    
    # ==========================================
    # 2. 获取机器人的物理与语义描述 (URDF & SRDF)
    # ==========================================
    # MoveIt Servo 是底层运动学算法，它必须知道机器人长什么样（URDF）以及哪些关节不能碰（SRDF）。
    # 这里需要和你在 bringup_gazebo 中加载 URDF 的方式绝对一致。
    
    # 2.1 动态解析 URDF (Xacro)
    description_pkg_path = get_package_share_directory('composite_robot_description')
    xacro_file = os.path.join(description_pkg_path, 'urdf', 'mobile_manipulator.urdf.xacro')
    # 使用 Command 动作在后台执行 `xacro` 命令将 .xacro 转换为标准的 urdf 字符串
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {"robot_description": robot_description_content}
    
    # 2.2 读取 SRDF (语义描述)
    srdf_file = os.path.join(get_package_share_directory("my_composite_robot_config"), "config", "franka_ranger_combined.srdf")
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    # ==========================================
    # 3. 定义组件节点 (Composable Node)
    # ==========================================
    # 我们并不直接运行 Servo，而是把它定义为一个待加载的“组件”。
    servo_node = ComposableNode(
        package="moveit_servo",
        plugin="moveit_servo::ServoNode", # 这是 C++ 代码中注册的 Component 类名
        name="servo_node",                # 在 ROS 网络中显示的节点名
        parameters=[                      # 把前面获取的所有参数塞给它
            servo_yaml,
            robot_description,
            robot_description_semantic,
            {"use_sim_time": True},
        ],
    )

    # ==========================================
    # 4. 创建并启动容器 (Container)
    # ==========================================
    # 这是真正被操作系统执行的单进程程序。
    container = ComposableNodeContainer(
        name="servo_container",           # 容器自己的名字
        namespace="/",
        package="rclcpp_components",      # 容器的启动引擎包
        executable="component_container_mt", # _mt 代表 Multi-Threaded (多线程)，允许容器内的组件并行执行
        composable_node_descriptions=[servo_node], # 把上面定义好的 servo_node 塞进容器里启动
        output="screen",
    )

    # 返回给 ROS 2 Launch 引擎执行
    return LaunchDescription([container])
