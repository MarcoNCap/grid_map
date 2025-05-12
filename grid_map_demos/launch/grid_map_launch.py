# Importing libraries

#Librarires from grid_map_pcl_loader_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


# Configuring paths from grid_map_loader_demo_launch.py
rover_config = get_package_share_directory('rover_config') # Get the path to the package
path_to_pcd = os.path.join(rover_config,'models','mars_yard','meshes') # Get the path to the pcd file
bag_name = 'elevation_map4.bag' # Name of the rosbag file
full_bag_path = os.path.join(path_to_pcd, bag_name) # Get the full path to the rosbag file



# def _conditionally_create_bag(context, *args, **kwargs):

#     bag = context.launch_configurations['full_bag_path']
#     if not os.path.exists(bag):
#         # If the bag file does not exist, create it
#         pcl_loader_node = Node(
#             package='grid_map_pcl',
#             executable='grid_map_pcl_loader_node',
#             name='grid_map_pcl_loader_node',
#             output='screen',
#             parameters=[
#                 {'folder_path': context.launch_configurations['path_to_pcd']},
#                 {'pcd_filename': 'mars_yard.pcd'},
#                 {'map_rosbag_topic': '/grid_map'},
#                 {'output_grid_map': os.path.basename(bag)},
#                 {'map_frame': 'map'},
#                 {'map_layer_name': context.launch_configurations['map_layer_name']},
#                 {'prefix': ''},
#                 {'set_verbosity_to_debug': True},
#             ]
#         )
#         # e un log esplicativo
#         log = LogInfo(msg=[f"Bag does not exist at: {bag} yet.  Creating it."])
#         return [log, pcl_loader_node]
#     else:
#         return [ LogInfo(msg=[f"Bag already exists at: {bag}. No need to create it."]) ]

    

def generate_launch_description():

    filters_config_file = LaunchConfiguration('filters_config')
    visualization_config_file = LaunchConfiguration('visualization_config')
    rviz_config_file = LaunchConfiguration('rviz_config')
    grid_map_demos_dir = get_package_share_directory('grid_map_demos')


    #Declare launch arguments:
    rover_config_arg = DeclareLaunchArgument(
        'rover_config',
        default_value = get_package_share_directory('rover_config'),
        description ='Path to the rover_config package'
    )

    path_to_pcd_arg = DeclareLaunchArgument(
        'path_to_pcd',
        default_value = os.path.join(rover_config,'models','mars_yard','meshes'),
        description='Path to the pcd file'
    )
    bag_name_arg = DeclareLaunchArgument(
        'bag_name',
        default_value='grid_map.bag',
        description='Name of the rosbag file containing the grid map'
    )

    # bag_flag = DeclareLaunchArgument(
    #     'bag_flag',
    #     default_value = True,
    #     description='Flag to indicate if the bag file exists'
    # )

    full_bag_path_arg = DeclareLaunchArgument(
        'full_bag_path',
        default_value=full_bag_path,
        description='Full path to the rosbag file'
    )
    declare_filters_config_file_cmd = DeclareLaunchArgument(
        'filters_config',
        default_value=os.path.join(
            grid_map_demos_dir, 'config', 'filters_demo_filter_chain_copy.yaml'),
        description='Full path to the filter chain config file to use')

    declare_visualization_config_file_cmd = DeclareLaunchArgument(
        'visualization_config',
        default_value=os.path.join(
            grid_map_demos_dir, 'config', 'filters_demo.yaml'),
        description='Full path to the Gridmap visualization config file to use')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            grid_map_demos_dir, 'rviz', 'filters_demo_custom.rviz'),
        description='Full path to the RVIZ config file to use')
    

    # Declare node actions

    play_grid_map_bag_node = ExecuteProcess(
        cmd = ['ros2','bag','play','--loop','--log-level','fatal',LaunchConfiguration('full_bag_path')]
    )

    filter_map_node = Node(
        package='grid_map_demos',
        executable='filters_demo',
        name='grid_map_filters',
        output='screen',
        parameters=[filters_config_file],
        arguments=['--ros-args', '--log-level', 'ERROR']
    )

    grid_map_visualization_node = Node(
        package='grid_map_visualization',
        executable='grid_map_visualization',
        name='grid_map_visualization',
        output='screen',
        parameters=[visualization_config_file]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    ld = LaunchDescription()
    # Add the launch arguments to the launch description
    ld.add_action(declare_filters_config_file_cmd)
    ld.add_action(declare_visualization_config_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(rover_config_arg)
    ld.add_action(path_to_pcd_arg)
    ld.add_action(bag_name_arg)
    # ld.add_action(bag_flag)
    ld.add_action(full_bag_path_arg)

    # Add node actions to the launch description
    ld.add_action(play_grid_map_bag_node)
    ld.add_action(filter_map_node)
    ld.add_action(grid_map_visualization_node)
    ld.add_action(rviz2_node)


    return ld









    







