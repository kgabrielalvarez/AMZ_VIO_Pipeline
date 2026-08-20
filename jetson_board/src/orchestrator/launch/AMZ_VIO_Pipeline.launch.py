from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    ld = LaunchDescription()

    can_driver_node = Node(
        package = "can_driver",
        executable = "can_driver",
        parameters = [PathJoinSubstitution([
                      FindPackageShare('orchestrator'), 'config', 'amz_vio_pipeline.yaml'])]
    )

    ld.add_action(can_driver_node)

    return ld