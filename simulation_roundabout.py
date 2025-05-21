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
                {'send_buffer_limit' : 500000000}
            ],
        ),
        Node(
            package='visualizer',
            namespace='ego_vehicle',
            executable='vehicle_visualizer_node',
            name='visualizer',
            parameters=[
                {"asset folder": map_image_folder},
                {"visualize_vehicle": True},
                {"visualize_trajectory": True},
                {"visualize_state": True},
                {"visualize_route": True},
                {"visualize_local_map": True},
                {"visualize_goal_point": True},
                {"visualize_map_image": False},
                {"visualize_traffic_participants": True},
                {"visualize_traffic_participants_predicted_trajectories": True},
             ],
        ),
        Node(
            package='simulated_vehicle',
            namespace='ego_vehicle',
            executable='simulated_vehicle',
            name='simulated_vehicle',
            parameters=[
                {"set_start_position_x": 604821.82},
                {"set_start_position_y": 5797112.35},
                {"set_start_psi": 3.06},
                {"controllable": True},
                {"vehicle_model_file" : vehicle_param + "/NGC.json"}
            ]
        ),
        Node(
            package='decision_maker',
            namespace='ego_vehicle',
            executable='decision_maker',
            name='decision_maker',
            parameters=[
                {"debug_mode_active": False},
                {"optinlc_route_following": True}, # 0 for Lane following, 1 for OptiNLC route following
                {"vehicle_model_file" : vehicle_param + "/NGC.json"},
                {"planner_settings_keys": [ "wheel_base",
                                           "lateral_weight",
                                           "heading_weight",
                                           "maximum_velocity",
                                           "min_distance_to_vehicle_ahead"]},
                {"planner_settings_values": [ 2.7,
                                               0.1,
                                               0.25,
                                               4.0,
                                               7.0]},
            ],
            #output={'both': 'log'},
        ),
        Node(
            package='mission_control',
            namespace='ego_vehicle',
            executable='mission_control',
            name='mission_control',
            parameters=[
                {"map file": map_folder + "/de_bs_borders_wfs.r2sr"},
                {"goal_position_x": 604714.26},
                {"goal_position_y": 5797115.51}
            ]
        ),
       Node(
           package='trajectory_tracker',
           namespace='ego_vehicle',
           executable='trajectory_tracker_node',
           name='trajectory_tracker',
           parameters=[
                {"set_controller": 1}, # 0 for MPC, 1 for PID
                {"controller_settings_keys": [ "kp_x",
                                           "ki_x",
                                           "velocity_weight",
                                           "kp_y",
                                           "ki_y",
                                           "heading_weight",
                                           "kp_omega",
                                           "dt",
                                           "steering_comfort"]},

                {"controller_settings_values": [ 0.3,
                                               0.02,
                                               0.3,
                                               0.25,
                                               0.0,
                                               0.3,
                                               0.1,
                                               0.05,
                                               2.5]},
                {"vehicle_model_file" : vehicle_param + "/NGC.json"},

           ],
           #output={'both': 'log'},
       ),
    ]
)
