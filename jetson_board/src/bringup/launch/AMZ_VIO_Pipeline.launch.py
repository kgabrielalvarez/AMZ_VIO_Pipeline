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

    camera_driver_node = Node(
        package = "camera_driver",
        executable = "camera_driver",
        parameters = [PathJoinSubstitution([
                      FindPackageShare('orchestrator'), 'config', 'amz_vio_pipeline.yaml'])]
    )
    ld.add_action(camera_driver_node)

    orchestrator_node = Node(
        package = "orchestrator",
        executable = "orchestrator"
    )
    ld.add_action(orchestrator_node)

    return ld