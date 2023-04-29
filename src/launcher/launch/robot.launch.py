from launch import LaunchDescription, actions
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    tirette_publisher = Node(
        package="hardware_package",
        executable="tirette_publisher",
        name="tirette_publisher"
    )

    motion_service = Node(
        package="control_package",
        executable="motion_service",
        name="motion_service"
    )

    arm_service = Node(
        package="actuator_package",
        executable="arm_service",
        name="arm_service"
    )

    ia = Node(
        package="ia_package",
        executable="ia_node",
        name="ia_node",
        on_exit=actions.Shutdown()
    )

    ld.add_action(tirette_publisher)
    ld.add_action(motion_service)
    ld.add_action(arm_service)
    ld.add_action(ia)

    return ld
