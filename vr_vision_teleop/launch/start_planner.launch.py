import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # ---------------------------------------------------------
    # 核心修改：直接加载生成的配置包，省去手动拼凑路径的麻烦
    # ---------------------------------------------------------
    moveit_config = (
        MoveItConfigsBuilder(
            # 机器人名字（如果不确定，去 my_composite_robot_config/.setup_assistant 里的 name 看看，通常是 franka_ranger_combined）
            "franka_ranger_combined", 
            package_name="my_composite_robot_config"
        )
        
        .planning_pipelines(pipelines=["ompl"]) # 只用 OMPL，避开 Pilz 报错
        .to_moveit_configs()
    )

    # ---------------------------------------------------------
    # 启动节点
    # ---------------------------------------------------------
    planner_node = Node(
        package="vr_vision_teleop",
        executable="robot_planner_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True}, # 仿真模式必开
        ],
    )

    return LaunchDescription([
        planner_node
    ])