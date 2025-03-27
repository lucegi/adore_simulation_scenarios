from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
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
            namespace='global',
            executable='visualizer',
            name='visualizer',
            parameters=[
                {"asset folder": map_image_folder},
                {"whitelist": ["ego_vehicle", "sim_vehicle_2", "infrastructure"]},
                {"visualization_offset_x": 604702.898},
                {"visualization_offset_y": 5797111.036},
            ]
        ),

        ############################################ Infrastructure ################################################
        
       Node(
            package='decision_maker_infrastructure',
            namespace='infrastructure',
            executable='decision_maker_infrastructure',
            name='decision_maker_infrastructure',
            parameters=[
                {"map file":  map_folder + "/de_bs_borders_wfs.r2sr"},
                {"infrastructure_position_x": 604790.672},
                {"infrastructure_position_y": 5797129.799},
                {"debug_mode_active": True},
                {"validity_polygon": [604750.672, 5797109.799,
                                       604750.672, 5797139.799,
                                       604799.672, 5797139.799,
                                       604799.672, 5797109.799,] }
            ]
        ),
        ########################################### first vehicle #########################################
        Node(
            package='simulated_vehicle',
            namespace='ego_vehicle',
            executable='simulated_vehicle',
            name='simulated_vehicle',
            parameters=[
                {"set_start_position_x": 604843.301},
                {"set_start_position_y": 5797114.574},
                {"set_start_psi": 3.1},
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
                {"v2x_id": 5},
                {"debug_mode_active": True},
                {"optinlc_route_following": False}, # 0 for Lane following, 1 for OptiNLC route following
                {"only_follow_reference_trajectories": False},
                {"planner_settings_keys": [ "wheel_base",
                                           "lateral_weight",
                                           "heading_weight",
                                           "maximum_velocity",
                                           "min_distance_to_vehicle_ahead",
                                           "look_ahead_for_curvature",
                                           "look_behind_for_curvature"]},

               {"planner_settings_values": [ 2.7,
                                               0.2,
                                               0.02,
                                               5.0,
                                               10.0,
                                               40.0,
                                               20.0]},
                {"vehicle_model_file" : vehicle_param + "/NGC.json"}
            ],
            # output={'both': 'log'},
        ),
        Node(
            package='mission_control',
            namespace='ego_vehicle',
            executable='mission_control',
            name='mission_control',
            parameters=[
                {"map file": map_folder + "/de_bs_borders_wfs.r2sr"},
                {"goal_position_x" : 604791.697},
                {"goal_position_y": 5797180.0}
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
                {"vehicle_model_file" : vehicle_param + "/NGC.json"}
           ],
        #    output={'both': 'log'},
       ),

        ########################################### second vehicle #########################################
        
        Node(
            package='simulated_vehicle',
            namespace='sim_vehicle_2',
            executable='simulated_vehicle',
            name='simulated_vehicle2',
            parameters=[
                {"set_start_position_x": 604702.898},
                {"set_start_position_y": 5797111.036},
                {"set_start_psi": 0.0},
                {"set_shape": [4.5, 2.0, 2.0]}, # length, width, height
                {"controllable": True},
                {"vehicle_model_file" : vehicle_param + "/NGC.json"}
            ]
        ),
         Node(
             package='decision_maker',
             namespace='sim_vehicle_2',
             executable='decision_maker',
             name='decision_maker2',
             parameters=[
                 {"v2x_id": 2},
                 {"debug_mode_active": True},
                 {"optinlc_route_following": False}, # 0 for Lane following, 1 for OptiNLC route following
                 {"only_follow_reference_trajectories": False},
                 {"planner_settings_keys": [ "wheel_base",
                                            "lateral_weight",
                                            "heading_weight",
                                            "maximum_velocity",
                                            "min_distance_to_vehicle_ahead",
                                            "look_ahead_for_curvature",
                                            "look_behind_for_curvature"]},

                {"planner_settings_values": [ 2.7,
                                                0.2,
                                                0.02,
                                                5.0,
                                                10.0,
                                                40.0,
                                                20.0]},
                {"vehicle_model_file" : vehicle_param + "/NGC.json"}
             ],
            #  output={'both': 'log'},
         ),
         Node(
             package='mission_control',
             namespace='sim_vehicle_2',
             executable='mission_control',
             name='mission_control2',
             parameters=[
                {"map file": map_folder + "/de_bs_borders_wfs.r2sr"},
                {"goal_position_x" : 604791.697},
                {"goal_position_y": 5797180.0}
             ]
         ),
        Node(
            package='trajectory_tracker',
            namespace='sim_vehicle_2',
            executable='trajectory_tracker_node',
            name='trajectory_tracker2',
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
            {"vehicle_model_file" : vehicle_param + "/NGC.json"}
            ],
            # output={'both': 'log'},
        ),

     
     ]
)
