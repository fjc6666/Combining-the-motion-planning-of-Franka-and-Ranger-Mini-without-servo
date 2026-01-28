import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. 定义包名和文件路径
    pkg_name = 'composite_robot_description'
    pkg_share = get_package_share_directory(pkg_name)
    
    # URDF 文件路径
    xacro_file = os.path.join(pkg_share, 'urdf', 'mobile_manipulator.urdf.xacro')
    # Rviz 配置文件路径 (如果没有，第一次启动后手动保存一个)
    rviz_config_file = os.path.join(pkg_share, 'config', 'config.rviz')

    # 2. 解析 Xacro 得到 URDF 内容
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}

    # 3. 节点配置
    
    # Robot State Publisher: 发布静态 TF 变换
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # Gazebo: 启动仿真环境
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # Spawn Entity: 在 Gazebo 中生成机器人模型
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'franka_ranger_bot'],
        output='screen'
    )

    # Rviz2: 可视化界面
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file], # 加载配置文件
        output='screen'
    )

    # 4. 控制器加载器 (Spawners)
    # 必须等 Gazebo 加载完机器人后才能加载控制器，否则会报错
    
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    load_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["franka_arm_controller"],
        output="screen",
    )

    # 5. 组装启动描述
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        # 这里的逻辑是：Spawn 结束后 -> 启动 Joint State -> 启动 Arm Controller -> 启动 Rviz
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_arm_controller, rviz_node],
            )
        ),
    ])