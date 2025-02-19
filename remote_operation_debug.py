from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
        # Get the directory of this launch file
    launch_file_dir = os.path.dirname(os.path.realpath(__file__))
    map_image_folder = os.path.abspath(os.path.join(launch_file_dir, "../assets/maps/"))
    map_folder = os.path.abspath(os.path.join(launch_file_dir, "../assets/tracks/"))
    vehicle_param = os.path.abspath(os.path.join(launch_file_dir, "../assets/vehicle_params/"))

    
    return LaunchDescription([
        Node(
            package='foxglove_bridge',
            namespace='global',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[
                {'port': 8765},
            ],
        ),
        Node(
            package='visualizer',
            namespace='ego_vehicle',
            executable='visualizer',
            name='visualizer',
            parameters=[
                {"asset folder": map_image_folder}
            ]
        ),
        Node(
            package='simulated_remote_operator',
            namespace='ego_vehicle',
            executable='simulated_remote_operator',
            name='simulated_remote_operator',
        ),
        Node(
            package='simulated_vehicle',
            namespace='ego_vehicle',
            executable='simulated_vehicle',
            name='simulated_vehicle',
            parameters=[
                {"set_start_position_x": 604862.718},
                {"set_start_position_y": 5797111.860},
                {"set_start_psi": 0.0},
                {"controllable": True},
                {"vehicle_model_file" : vehicle_param + "/NGC.json"}
            ]
        ),
        Node(
            package='mission_control',
            namespace='ego_vehicle',
            executable='mission_control',
            name='mission_control',
            parameters=[
                {"R2S map file": map_folder + "/de_bs_borders_wfs.r2sr"},
                {"goal_position_x" : 604988.297},
                {"goal_position_y": 5797111.0}
            ]
        ),
    ]
)
