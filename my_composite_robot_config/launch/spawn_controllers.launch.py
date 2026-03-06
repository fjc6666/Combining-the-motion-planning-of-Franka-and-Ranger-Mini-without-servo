from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("franka_ranger_combined", package_name="my_composite_robot_config").to_moveit_configs()
    return generate_spawn_controllers_launch(moveit_config)
