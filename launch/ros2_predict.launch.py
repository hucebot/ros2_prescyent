from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ros2_prescyent",
                namespace="prescyent_predict",
                executable="prescyent_predict",
                name="predictor_node",
                parameters=[
                    {
                        "predictor_path": "",
                        "history_size": 10,
                        "future_size": 10,
                        "predictor_frequency": 10,
                    }
                ],
            ),
        ]
    )
