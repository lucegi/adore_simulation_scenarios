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
        # ================ infrastructure  ==================
        Node(
            package='simulated_infrastructure',
            namespace='infrastructure',
            executable='simulated_infrastructure',
            name='simulated_infrastructure',
            parameters=[
                {"infrastructure_position_x": 604790.672},
                {"infrastructure_position_y": 5797129.799},
                {"validity_polygon": [604750.672, 5797109.799,
                                       604750.672, 5797160.799,
                                       604799.672, 5797160.799,
                                       604799.672, 5797109.799,] }
            ]
        ),
        Node(
            package='visualizer',
            namespace='infrastructure',
            executable='infrastructure_visualizer_node',
            name='visualizer',
            parameters=[
                {"asset folder": map_image_folder},
                {"visualize_infrastructure": True},
                {"visualize_local_map": True},
                {"visualize_validity_area": True},
                {"visualize_map_image": True}, # Only one visualizer can show images
                {"visualize_traffic_participants": True},
            ]
        ),
       Node(
            package='decision_maker_infrastructure',
            namespace='infrastructure',
            executable='decision_maker_infrastructure',
            name='decision_maker_infrastructure',
            parameters=[
                {"map file":  map_folder + "/de_bs_borders_wfs.r2sr"},
                {"infrastructure_position_x": 604790.672},
                {"infrastructure_position_y": 5797129.799},
                {"debug_mode_active": False},
                {"validity_polygon": [604750.672, 5797109.799,
                                       604750.672, 5797160.799,
                                       604799.672, 5797160.799,
                                       604799.672, 5797109.799,] }
            ]
        ),

        # ================ VEHICLE 1 ========================
        Node(
            package='visualizer',
            namespace='ego_vehicle',
            executable='vehicle_visualizer_node',
            name='visualizer',
            parameters=[
                {"asset folder": map_image_folder},
                {"visualize_vehicle": False},
                {"visualize_planned_trajectory": False},
                {"visualize_route": False},
                {"visualize_local_map": False},
                {"visualize_goal_point": True},
                {"visualize_map_image": False},
                {"visualize_traffic_participants": False},
             ],
        ),
        Node(
            package='simulated_vehicle',
            namespace='ego_vehicle',
            executable='simulated_vehicle',
            name='simulated_vehicle',
            parameters=[
                {"set_start_position_x": 604835.481},
                {"set_start_position_y": 5797113.518},
                {"set_start_psi": 3.14},
                {"controllable": True},
                {"id": 111}, # Make sure this is unique for each vehicle
                {"v2x_id": 111},
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

        Node(
            package='visualizer',
            namespace='sim_vehicle_1',
            executable='vehicle_visualizer_node',
            name='visualizer',
            parameters=[
                {"asset folder": map_image_folder},
                {"visualize_vehicle": False},
                {"visualize_planned_trajectory": False},
                {"visualize_route": False},
                {"visualize_local_map": False},
                {"visualize_goal_point": True},
                {"visualize_map_image": False},
             ],
        ),
        Node(
            package='simulated_vehicle',
            namespace='sim_vehicle_1',
            executable='simulated_vehicle',
            name='simulated_vehicle',
            parameters=[
                {"set_start_position_x": 604731.230},
                {"set_start_position_y": 5797112.750},
                {"set_start_psi": 0.0},
                {"set_shape": [4.5, 2.0, 2.0]}, # length, width, height
                {"controllable": True},
                {"vehicle_id": 222},
                {"v2x_id": 0},
                {"vehicle_model_file" : vehicle_param + "/NGC.json"}
            ]
        ),
        Node(
            package='decision_maker',
            namespace='sim_vehicle_1',
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
            namespace='sim_vehicle_1',
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
           namespace='sim_vehicle_1',
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
     ########################################### third vehicle #########################################


        Node(
            package='visualizer',
            namespace='sim_vehicle_2',
            executable='vehicle_visualizer_node',
            name='visualizer',
            parameters=[
                {"asset folder": map_image_folder},
                {"visualize_vehicle": False},
                {"visualize_planned_trajectory": False},
                {"visualize_route": False},
                {"visualize_local_map": False},
                {"visualize_goal_point": True},
                {"visualize_map_image": False},
             ],
        ),
        Node(
            package='simulated_vehicle',
            namespace='sim_vehicle_2',
            executable='simulated_vehicle',
            name='simulated_vehicle',
            parameters=[
                {"set_start_position_x": 604788.611},
                {"set_start_position_y": 5797185.181},
                {"set_start_psi": 5.0 },
                {"set_shape": [4.5, 2.0, 2.0]}, # length, width, height
                {"controllable": True},
                {"vehicle_id": 333}, # Make sure this is unique for each vehicle
                {"v2x_id": 333},
                {"vehicle_model_file" : vehicle_param + "/NGC.json"}
            ]
        ),
        Node(
            package='decision_maker',
            namespace='sim_vehicle_2',
            executable='decision_maker',
            name='decision_maker',
            parameters=[
                {"debug_mode_active": False},
                {"optinlc_route_following": False}, # 0 for Lane following, 1 for OptiNLC route following
                {"vehicle_model_file" : vehicle_param + "/NGC.json"}

            ],
            # output={'both': 'log'},
        ),
        Node(
            package='mission_control',
            namespace='sim_vehicle_2',
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
           namespace='sim_vehicle_2',
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
    ]
)
