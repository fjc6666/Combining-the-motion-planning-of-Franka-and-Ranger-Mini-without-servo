import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import Command

def generate_launch_description():
    # 1. 准备路径
    # 1. 获取源 URDF 路径并生成内容
    # 注意：这里直接指向 composite_robot_description 里的源文件，不再用 MoveIt 配置包里的副本
    xacro_file = os.path.join(
        get_package_share_directory('composite_robot_description'),
        'urdf',
        'mobile_manipulator.urdf.xacro'
    )
    
    robot_description_content = Command(['xacro ', xacro_file])

    # 2. 配置 MoveIt
    moveit_config = (
        MoveItConfigsBuilder("franka_ranger_combined", package_name="my_composite_robot_config")
        # .robot_description(file_path="...")  <--- 删除这一行！
        .robot_description_semantic(file_path="config/franka_ranger_combined.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # 强制覆盖 robot_description，使用最新的源文件内容
    moveit_config.robot_description = {'robot_description': robot_description_content}

    
    # 获取 Gazebo 的标准启动文件
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # 2. 关键：把 robot_description (URDF) 读出来，准备传给 Gazebo
    # 注意：这里我们直接用 MoveIt 加载好的 URDF 内容
    robot_description = moveit_config.robot_description

    # 3. 在 Gazebo 中生成机器人 (Spawn Entity)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_robot'],
        output='screen'
    )

    # 4. 加载控制器 (这里是关键！)
    # 我们不仅要加载关节状态广播器，还要加载 MoveIt 需要的那个 franka_arm_controller
    
    # 4.1 加载 joint_state_broadcaster
    load_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # 4.2 加载机械臂控制器 (名字要和你 ros2_controllers.yaml 里的一致，通常是 franka_arm_controller)
    load_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["franka_arm_controller", "--controller-manager", "/controller_manager"],
    )


    load_base_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ranger_base_controller", "--controller-manager", "/controller_manager"],
    )

    # 5. 启动 MoveIt (Move Group)
    # 让 MoveIt 知道我们现在是在仿真（use_sim_time=True）
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    # 6. 启动 RViz (可选，为了看规划效果)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(get_package_share_directory("my_composite_robot_config"), "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True}
        ],
    )
    
    # 7. 发布 Robot State (TF 树)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": True} 
        ],
    )
    # 8. 强行连接 world 和 base_footprint (解决 TF 断裂问题)
    # 这一步对于仿真至关重要，让 MoveIt 知道机器人在世界原点
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_footprint'],
        parameters=[{'use_sim_time': True}]  # <--- 加上这一行！让它和 Gazebo 同步
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        static_tf,
        spawn_entity,
        # 1. 机器人生成后 -> 启动 joint_state_broadcaster
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_jsb],
            )
        ),
        # 2. joint_state_broadcaster 启动后 -> 启动 机械臂控制器
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_jsb,
                on_exit=[load_arm_controller],
            )
        ),
        # 3. 【新增】机械臂控制器启动后 -> 启动 底盘控制器 (解决溜车问题)
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_arm_controller,
                on_exit=[load_base_controller],
            )
        ),
        # 4. 底盘控制器启动后 -> 启动 MoveIt 和 RViz
        # (这样保证 MoveIt 启动时，所有底层控制器都已经就绪)
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_arm_controller,
                on_exit=[run_move_group_node, rviz_node],
            )
        ),
    ])