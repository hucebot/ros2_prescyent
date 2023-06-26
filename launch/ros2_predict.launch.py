from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    predictor_launch_arg = DeclareLaunchArgument(
        'predictor', default_value=TextSubstitution(text='0')
    )
    history_size_launch_arg = DeclareLaunchArgument(
        'history_size', default_value=TextSubstitution(text='10')
    )
    future_size_launch_arg = DeclareLaunchArgument(
        'future_size', default_value=TextSubstitution(text='10')
    )
    time_step_launch_arg = DeclareLaunchArgument(
        'time_step', default_value=TextSubstitution(text='10')
    )
    return LaunchDescription([
        predictor_launch_arg,
        history_size_launch_arg,
        future_size_launch_arg,
        time_step_launch_arg,
        Node(
            package='ros2_prescyent', namespace= "ros2_predict1",
            executable='ros2_predict', name='predictor_node',
            parameters=[{
            'predictor': LaunchConfiguration('predictor'),
            'history_size': LaunchConfiguration('history_size'),
            'future_size': LaunchConfiguration('future_size'),
            'time_step': LaunchConfiguration('time_step'),
         }])
   ])
