from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    node1 = ExecuteProcess(cmd=["ros2", "run", "robot_bridge", "release_node"], output="screen")

    node2 = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "robot_bridge",
            "bridge",
            "--ros-args",
            "--params-file",
            "/home/xuanhaosong/sairol_ws/src/sairol_ws/robot_bridge/params/config.yaml",
        ],
        output="screen",
    )

    node2_launch = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=node1, on_exit=[node2])
    )

    return LaunchDescription([node1, node2_launch])
