from launch import LaunchDescription
import os
from launch_ros.actions import Node
import sys
base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if base_dir not in sys.path:
    sys.path.insert(0, base_dir)
from scenario_helpers.simulated_vehicle import create_simulated_vehicle_nodes
from scenario_helpers.visualizer import create_visualization_nodes

def generate_launch_description():
    # Get the directory of this launch file
    launch_file_dir = os.path.dirname(os.path.realpath(__file__))
    map_image_folder = os.path.abspath(os.path.join(launch_file_dir, "../assets/maps/"))
    map_folder = os.path.abspath(os.path.join(launch_file_dir, "../assets/tracks/"))
    vehicle_param = os.path.abspath(os.path.join(launch_file_dir, "../assets/vehicle_params/"))
    map_file = map_folder + "/de_bs_borders_wfs.r2sr"
    vehicle_model_file = vehicle_param + "/NGC.json"

    return LaunchDescription([
        *create_visualization_nodes(
            whitelist=["ego_vehicle"],
            asset_folder=map_image_folder,
            use_center_ego=True
        ),
        *create_simulated_vehicle_nodes(
            namespace="ego_vehicle",
            start_pose=(605105.0, 5795182.83, -1.5),
            goal_position=(605051.41, 5795026.90),
            map_file=map_file,
            model_file=vehicle_model_file,
            controllable=True,
            v2x_id=0,
            vehicle_id=0,
            controller=1,
            debug=False,
            local_map_size=150.0
        ),
        Node(
            package='simulated_traffic_signal',
            namespace='ego_vehicle',
            executable='simulated_traffic_signal',
            name='traffic_lights',
            parameters=[
                {"permanent_red": True},
                {"traffic_lights": ["t1", "t2", "t3", "t4", "t5", "t6", "t7", "t8"]},
                {"t1.x": 605072.0},
                {"t1.y": 5795118.348},
                {"t1.red_duration": 10.0},
                {"t1.yellow_duration": 5.0},
                {"t1.green_duration": 10.0},

                {"t2.x": 605075.0},
                {"t2.y": 5795117.348},
                {"t2.red_duration": 30.0},
                {"t2.yellow_duration": 5.0},
                {"t2.green_duration": 30.0},

                {"t3.x": 605063.8},
                {"t3.y": 5795052.486},
                {"t3.red_duration": 10.0},
                {"t3.yellow_duration": 5.0},
                {"t3.green_duration": 10.0},

                {"t4.x": 605066.794},
                {"t4.y": 5795052.2},
                {"t4.red_duration": 10.0},
                {"t4.yellow_duration": 5.0},
                {"t4.green_duration": 10.0},

                {"t5.x": 605070.0},
                {"t5.y": 5795051.986},
                {"t5.red_duration": 10.0},
                {"t5.yellow_duration": 5.0},
                {"t5.green_duration": 10.0},

                {"t6.x": 605087.505},
                {"t6.y": 5795064.942},
                {"t6.red_duration": 10.0},
                {"t6.yellow_duration": 5.0},
                {"t6.green_duration": 10.0},

                {"t7.x": 605045.124},
                {"t7.y": 5795104.993},
                {"t7.red_duration": 10.0},
                {"t7.yellow_duration": 5.0},
                {"t7.green_duration": 10.0},

                {"t8.x": 605045.124},
                {"t8.y": 5795107.993},
                {"t8.red_duration": 10.0},
                {"t8.yellow_duration": 5.0},
                {"t8.green_duration": 10.0},
            ]
        ),
    ])

