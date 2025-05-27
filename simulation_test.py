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
                {"visualize_map_image": True},
                {"visualize_traffic_participants": True},
                {"visualize_ignored_traffic_participants": True},
                {"visualize_infrastructure_validity_area": True},
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
                {"visualize_planned_trajectory": True},
                {"visualize_route": True},
                {"visualize_local_map": True},
                {"visualize_goal_point": True},
                {"visualize_map_image": True},
             ],
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
            package='decision_maker',
            namespace='ego_vehicle',
            executable='decision_maker',
            name='decision_maker',
            parameters=[
                {"debug_mode_active": False},
                {"optinlc_route_following": False}, # 0 for Lane following, 1 for OptiNLC route following
                {"vehicle_model_file" : vehicle_param + "/NGC.json"}

            ],
            output={'both': 'log'},
        ),
        Node(
            package='mission_control',
            namespace='ego_vehicle',
            executable='mission_control',
            name='mission_control',
            parameters=[
                {"map file": map_folder + "/de_bs_borders_wfs.r2sr"},
                {"goal_position_x" : 604988.297},
                {"goal_position_y": 5797111.0}
            ]
        ),
       Node(
           package='trajectory_tracker',
           namespace='ego_vehicle',
           executable='trajectory_tracker_node',
           name='trajectory_tracker',
           parameters=[
               {"set_controller": 2}, # 0 for MPC, 1 for PID
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

        ########################################### second vehicle #########################################
        
        # Node(
        #     package='simulated_vehicle',
        #     namespace='traffic_participant_2',
        #     executable='simulated_vehicle',
        #     name='simulated_vehicle',
        #     parameters=[
        #         {"set_start_position_x": 604786.672},
        #         {"set_start_position_y": 5797162.799},
        #         {"set_start_psi": 1.22},
        #         {"set_shape": [4.5, 2.0, 2.0]}, # length, width, height
        #         {"controllable": True},
        #         {"vehicle_id": 2},
        #         {"vehicle_model_file" : vehicle_param + "/NGC.json"},
        #     ]
        # ),
        # Node(
        #     package='decision_maker',
        #     namespace='traffic_participant_2',
        #     executable='decision_maker',
        #     name='decision_maker',
        #     parameters=[
        #         {"debug_mode_active": False},
        #         {"optinlc_route_following": True}, # 0 for Lane following, 1 for OptiNLC route following
        #         {"planner_settings_keys": [ "wheel_base",
        #                                    "lateral_weight",
        #                                    "heading_weight",
        #                                    "maximum_velocity",
        #                                    "min_distance_to_vehicle_ahead",
        #                                    "look_ahead_for_curvature",
        #                                    "look_behind_for_curvature"]},

        #        {"planner_settings_values": [ 2.7,
        #                                        0.2,
        #                                        0.02,
        #                                        5.0,
        #                                        10.0,
        #                                        40.0,
        #                                        20.0]},
        #       {"vehicle_model_file" : vehicle_param + "/NGC.json"},
        #     ],
        # ),
        # Node(
        #     package='mission_control',
        #     namespace='traffic_participant_2',
        #     executable='mission_control',
        #     name='mission_control',
        #     parameters=[
        #         {"map file": os.path.abspath("assets/tracks/de_bs_borders_wfs.r2sr")},
        #         {"goal_position_x" : 604791.697},
        #         {"goal_position_y": 5797180.0}
        #     ]
        # ),
    #    Node(
    #        package='trajectory_tracker',
    #        namespace='traffic_participant_2',
    #        executable='trajectory_tracker_node',
    #        name='trajectory_tracker',
    #        parameters=[
    #            {"set_controller": 1}, # 0 for MPC, 1 for PID
    #            {"controller_settings_keys": [ "kp_x",
    #                                        "ki_x",
    #                                        "velocity_weight",
    #                                        "kp_y",
    #                                        "ki_y",
    #                                        "heading_weight",
    #                                        "kp_omega",
    #                                        "dt",
    #                                        "steering_comfort"]},

    #            {"controller_settings_values": [ 0.3,
    #                                            0.02,
    #                                            0.3,
    #                                            0.25,
    #                                            0.0,
    #                                            0.3,
    #                                            0.1,
    #                                            0.05,
    #                                            2.5]},
    #            {"vehicle_model_file" : vehicle_param + "/NGC.json"},
    #        ],
    #        #output={'both': 'log'},
    #    ),

       
        # Node(
        #     package='trajectory_tracker',
        #     namespace='ego_vehicle',
        #     executable='trajectory_tracker_node',
        #     name='trajectory_tracker',
        #     parameters=[
        #         {"vehicle_type": "simulation"}, # decide how to do turning
        #         {"set_controller": 2}, # 0 for MPC, 1 for PID
        #         {"controller_settings_keys": [ "horizon_steps",
        #                                     "max_iterations",
        #                                     "dt",
        #                                     "wheelbase",
        #                                     "lateral_weight",
        #                                     "longitudinal_weight",
        #                                     "heading_weight",
        #                                     "vel_weight",
        #                                     "acc_weight",
        #                                     "jerk_weight",
        #                                     "steer_rate_weight",
        #                                     "steer_weight",
        #                                     "convergence_threshold",
        #                                     "debug active"]},

        #         {"controller_settings_values": [ 60,
        #                                         500,
        #                                         0.05,
        #                                         2.7,
        #                                         10.0,
        #                                         1.0,
        #                                         3.0,
        #                                         10.0,
        #                                         5.0,
        #                                         20.0,
        #                                         10.0,
        #                                         1.0,
        #                                         0.00000001,
        #                                         0]},
        #        {"vehicle_model_file" : vehicle_param + "/NGC.json"},
        #     ],
        #     #output={'both': 'log'},
        # ),
    ]
)
