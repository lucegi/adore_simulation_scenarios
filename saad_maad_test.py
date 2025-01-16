from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
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
                {"asset folder": os.path.abspath("assets/maps/")}
            ]
        ),
        Node(
            package='simulated_vehicle',
            namespace='ego_vehicle',
            executable='simulated_vehicle',
            name='simulated_vehicle',
            parameters=[
                {"set_start_position_x": 604784.818},
                {"set_start_position_y": 5797117.860},
                {"set_start_psi": 2.63},
                {"controllable": True},
                {"other_vehicle_namespaces": ["traffic_participant_2"]}
            ]
        ),
        Node(
            package='decision_maker',
            namespace='ego_vehicle',
            executable='decision_maker',
            name='decision_maker',
            parameters=[
                {"v2x_id": 1},
                {"debug_mode_active": True},
                {"optinlc_route_following": False}, # 0 for Lane following, 1 for OptiNLC route following
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
                                               20.0]}
            ],
        ),
        Node(
            package='mission_control',
            namespace='ego_vehicle',
            executable='mission_control',
            name='mission_control',
            parameters=[
                {"R2S map file": os.path.abspath("assets/tracks/de_bs_borders_wfs.r2sr")},
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
                                               2.5]}
           ],
           #output={'both': 'log'},
       ),

        ########################################### second vehicle #########################################
        
        Node(
            package='simulated_vehicle',
            namespace='traffic_participant_2',
            executable='simulated_vehicle',
            name='simulated_vehicle2',
            parameters=[
                {"set_start_position_x": 604780.672},
                {"set_start_position_y": 5797119.799},
                {"set_start_psi": 2.22},
                {"set_shape": [4.5, 2.0, 2.0]}, # length, width, height
                {"controllable": True},
            ]
        ),
         Node(
             package='decision_maker',
             namespace='traffic_participant_2',
             executable='decision_maker',
             name='decision_maker2',
             parameters=[
                 {"v2x_id": 2},
                 {"debug_mode_active": False},
                 {"optinlc_route_following": False}, # 0 for Lane following, 1 for OptiNLC route following
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
                                                20.0]}
             ],
         ),
         Node(
             package='mission_control',
             namespace='traffic_participant_2',
             executable='mission_control',
             name='mission_control2',
             parameters=[
                 {"R2S map file": os.path.abspath("assets/tracks/de_bs_borders_wfs.r2sr")},
                 {"goal_position_x" : 604791.697},
                 {"goal_position_y": 5797180.0}
             ]
         ),
        Node(
            package='trajectory_tracker',
            namespace='traffic_participant_2',
            executable='trajectory_tracker_node',
            name='trajectory_tracker2',
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
                                                2.5]}
            ],
            #output={'both': 'log'},
        ),

       Node(
            package='decision_maker_infrastructure',
            namespace='infrastructure',
            executable='decision_maker_infrastructure',
            name='decision_maker_infrastructure',
            parameters=[

            {"R2S map file": os.path.abspath("assets/tracks/de_bs_borders_wfs.r2sr")},

            ]
        ),
     
     ]
)
