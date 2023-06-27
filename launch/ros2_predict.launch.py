from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ros2_prescyent",
                namespace="ros2_predict1",
                executable="ros2_predict",
                name="predictor_node",
                parameters=[
                    {
                        "predictor_path": "",
                        "history_size": 10,
                        "future_size": 10,
                        "time_step": 10,
                    }
                ],
            ),
        ]
    )
