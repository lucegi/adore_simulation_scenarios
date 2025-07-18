from launch import LaunchDescription
from launch_ros.actions import Node
import os
import sys
base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if base_dir not in sys.path:
    sys.path.insert(0, base_dir)
from scenario_helpers.simulated_vehicle import create_simulated_vehicle_nodes
from scenario_helpers.simulated_infrastructure import create_infrastructure_nodes
from scenario_helpers.visualizer import create_visualization_nodes


def generate_launch_description():
    launch_file_dir = os.path.dirname(os.path.realpath(__file__))
    map_image_folder = os.path.abspath(os.path.join(launch_file_dir, "../assets/maps/"))
    map_folder = os.path.abspath(os.path.join(launch_file_dir, "../assets/tracks/"))
    vehicle_param = os.path.abspath(os.path.join(launch_file_dir, "../assets/vehicle_params/"))
    map_file = map_folder + "/de_bs_borders_wfs.r2sr"
    vehicle_model_file = vehicle_param + "/NGC.json"

    return LaunchDescription([
        # ================ visualization ==================
        *create_visualization_nodes(
            whitelist=["ego_vehicle", "sim_vehicle_1", "sim_vehicle_2", "infrastructure"],
            asset_folder=map_image_folder,
            use_center_ego=False,
            ns="infrastructure"
        ),
        # ================ Infrastructure ===================
        *create_infrastructure_nodes(
            position=(606453.910, 5797315.369),
            polygon=[
                606438.850, 5797325.544,
                606477.292, 5797323.258,
                606478.079, 5797312.037,
                606455.904, 5797299.986,
                606443.210, 5797300.491,
            ],
            map_file=map_file
        ),
        # ================ Vehicles ========================
        *create_simulated_vehicle_nodes(
            namespace="ego_vehicle",
            start_pose=(606425.120, 5797326.700, 0.0),
            goal_position=(606532.605, 5797313.325),
            vehicle_id=111,
            v2x_id=111,
            model_file=vehicle_model_file,
            map_file=map_file,
            shape=(4.5, 2.0, 2.0)
        ),

        *create_simulated_vehicle_nodes(
            namespace="sim_vehicle_1",
            start_pose=(606451.140, 5797285.841, 3.14 / 2.0),
            goal_position=(606532.605, 5797313.325),
            vehicle_id=222,
            v2x_id=222,
            model_file=vehicle_model_file,
            map_file=map_file,
            shape=(4.5, 2.0, 2.0)
        ),

        *create_simulated_vehicle_nodes(
            namespace="sim_vehicle_2",
            start_pose=(606497.528, 5797318.186, -3.14),
            goal_position=(606449.144, 5797290.938),
            vehicle_id=333,
            v2x_id=333,
            model_file=vehicle_model_file,
            map_file=map_file,
            shape=(4.5, 2.0, 2.0)
        ),
    ])
