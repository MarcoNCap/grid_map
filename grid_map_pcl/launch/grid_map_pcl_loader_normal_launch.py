from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('rover_config'), 'config', 'layer.yaml')

    rover_config = get_package_share_directory('rover_config') # Get the path to the package
    path_to_pcd = os.path.join(rover_config,'models','mars_yard','meshes') # Get the path to the pcd file
    saving_bag = os.path.join('','root','src','rover_config','models','mars_yard','meshes') # Get the path to the rosbag file
    bag_name = 'elevation_map_test_2_1.bag' # Name of the rosbag file

    full_bag_path = os.path.join(path_to_pcd, bag_name) # Get the full path to the rosbag file
    map_layer_name = 'elevation' # Name of the layer in the grid map

    node_params = [
            {'folder_path': path_to_pcd}, # Path to the folder containing the pcd file
            {'pcd_filename': 'mars_yard.pcd'}, # Name of the pcd file without the extension
            {'map_rosbag_topic': '/grid_map'}, # The topic where the grid map will be published
            {'output_grid_map': bag_name}, # THe name of the rosbag file where the grid map will be saved
            {'map_frame': 'map'}, # The frame where the grid map will be published
            {'map_layer_name': map_layer_name}, #The name of the layer in the grid map
            {'prefix': ''},
            {'set_verbosity_to_debug': True}
        ] #Creates a list of dictionaries with parameters for the node

    # Create a container for the composable nodes
    container = ComposableNodeContainer(
        name='grid_map_pipeline',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[

            # -------------------------
            # 1) Node loading the PCD file into a grid map
            # -------------------------
            ComposableNode(
                package='grid_map_pcl',
                plugin='grid_map::GridMapPclLoaderNode',
                name='pcl_loader',
                remappings=[('grid_map', 'grid_map')],
                parameters=node_params,
            ),

            # ----------------------------------
            # 2) Node applying Filters to the grid map
            # ----------------------------------
            ComposableNode(
                package='grid_map_filters',
                plugin='grid_map::GridMapFiltersNode',
                name='grid_map_filters',
                remappings=[
                    ('grid_map_in',  'grid_map'),
                    ('grid_map_out', 'grid_map'),
                ],
                parameters=[cfg],
            ),

        ],
        output='screen',
    )

    return LaunchDescription([container])