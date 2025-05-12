from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

## C

rover_config = get_package_share_directory('rover_config') # Get the path to the package
path_to_pcd = os.path.join(rover_config,'models','mars_yard','meshes') # Get the path to the pcd file
saving_bag = os.path.join('','root','src','rover_config','models','mars_yard','meshes') # Get the path to the rosbag file
bag_name = 'elevation_map_test_5.bag' # Name of the rosbag file

full_bag_path = os.path.join(path_to_pcd, bag_name) # Get the full path to the rosbag file
map_layer_name = 'elevation' # Name of the layer in the grid map

if not os.path.exists(full_bag_path): # Check if the path to the rosbag file exists
    print("Creating bag at: ", full_bag_path) # Print the path to the rosbag file
    def generate_launch_description():

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

    # Next, we create a node for the pcl loader
        # The node is defined using the Node class from launch_ros.actions
        pcl_loader_node = Node(
            package='grid_map_pcl', # The package name where the node is located
            executable='grid_map_pcl_loader_node', # The name of the executable
            name='grid_map_pcl_loader_node', # The name of the node
            output='screen', # The output of the node will be displayed on the screen
            parameters=node_params # The parameters for the node are passed as a list of dictionaries
        ) # 

        ld = LaunchDescription() # Creates a LaunchDescription object

        ld.add_action(pcl_loader_node) # Adds the pcl_loader_node to the launch description

        print("Bag created at: ", full_bag_path) # Prints the path to the rosbag file
        print("Grid map published on: ", '/grid_map') # Prints the topic where the grid map will be published
        print("Grid map frame: ", 'map') # Prints the frame where the grid map will be published
        print("Grid map layer name: ", map_layer_name) # Prints the name of the layer in the grid map

        return ld
else:
    print("Bag already exists at: ", full_bag_path)


# This launch file does the following things: 
# - Creates a grid map mulilayer from a pcd file
# - Saves the grid map to a rosbag file elevation_map.bag that can be used to rebuild the map in memory
# - The map is published on the /grid_map topic