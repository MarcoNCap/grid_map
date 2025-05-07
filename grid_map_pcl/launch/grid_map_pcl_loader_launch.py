from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

rover_config = get_package_share_directory('rover_config') # Get the path to the package
path_to_pcd = os.path.join(rover_config,'models','mars_yard','meshes') # Get the path to the pcd file

def generate_launch_description():

    node_params = [
        {'folder_path': path_to_pcd}, # Path to the folder containing the pcd file
        {'pcd_filename': 'mars_yard.pcd'}, # Name of the pcd file without the extension
        {'map_rosbag_topic': '/grid_map'}, # The topic where the grid map will be published
        {'output_grid_map': 'elevation_map_custom.bag'}, # THe name of the rosbag file where the grid map will be saved
        {'map_frame': 'map'}, # The frame where the grid map will be published
        {'map_layer_name': 'elevation'}, #The name of the layer in the grid map
        {'prefix': ''},
        {'set_verbosity_to_debug': True}
    ] #Creates a list of dictionaries with parameters for the node

# Next, we create a node for the pcl loader
    # The node is defined using the Node class from launch_ros.actions
    pcl_loader_node = Node(
        package='grid_map_pcl', # The package name where the node is located
        executable='grid_map_pcl_loader_node_custom', # The name of the executable
        name='grid_map_pcl_loader_node_custom', # The name of the node
        output='screen', # The output of the node will be displayed on the screen
        parameters=node_params # The parameters for the node are passed as a list of dictionaries
    ) # 

    ld = LaunchDescription() # Creates a LaunchDescription object

    ld.add_action(pcl_loader_node) # Adds the pcl_loader_node to the launch description

    return ld


# This launch file does the following things: 
# - Creates a grid map mulilayer from a pcd file
# - Saves the grid map to a rosbag file elevation_map.bag that can be used to rebuild the map in memory
# - The map is published on the /grid_map topic