/*
 * grid_map_pcl_loader_node.cpp
 *
 *  Created on: Aug 26, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <memory>
#include <string>
#include <utility>

#include "grid_map_core/GridMap.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "grid_map_pcl/GridMapPclLoader.hpp"
#include "grid_map_pcl/helpers.hpp"

namespace gm = ::grid_map::grid_map_pcl;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("grid_map_pcl_loader_node");
  gm::setVerbosityLevelToDebugIfFlagSet(node);

  rclcpp::QoS custom_qos = rclcpp::QoS(1).transient_local();
  auto gridMapPub = node->create_publisher<grid_map_msgs::msg::GridMap>(
    "grid_map_from_raw_pointcloud", custom_qos);

  grid_map::GridMapPclLoader gridMapPclLoader(node->get_logger());
  const std::string pathToCloud = gm::getPcdFilePath(node);
  gridMapPclLoader.loadParameters(gm::getParameterPath());
  gridMapPclLoader.loadCloudFromPcdFile(pathToCloud);

  gm::processPointcloud(&gridMapPclLoader, node); // Process the point cloud to create the grid_map

  grid_map::GridMap gridMap = gridMapPclLoader.getGridMap(); // Load the created grid_map
  // gridMap.add("noise", 0.015 * grid_map::Matrix::Random(gridMap.getSize()(0), gridMap.getSize()(1)));
  // // gridMap.add("normal_z");
  gridMap.setFrameId(gm::getMapFrame(node)); // Set the frame ID for the grid_map

  // gridMap.add("surface_normals", Matrix::Random(gridMap.getSize()(0), gridMap.getSize()(1))); // Create a layer for the surface normals
  
  // gridMap.add("normal_x",std::numeric_limits<float>::quiet_NaN()); 
  // gridMap.add("normal_y",std::numeric_limits<float>::quiet_NaN());
  // gridMap.add("normal_z",std::numeric_limits<float>::quiet_NaN());

  // const double res = gridMap.getResolution();
  // const auto size = gridMap.getSize();

  // for (grid_map::GridMapIterator it(gridMap); !it.isPastEnd(); ++it) {
  //   const auto idx = *it;

  //   if (!gridMap.isValid(idx, "elevation")) { continue; } // Skip invalid cells

  //   // Get the position of the current cell and select the 4 neighbors
  //   grid_map::Index idx_left  = idx + grid_map::Index(-1,  0);
  //   grid_map::Index idx_right = idx + grid_map::Index(+1,  0);
  //   grid_map::Index idx_down  = idx + grid_map::Index( 0, -1);
  //   grid_map::Index idx_up    = idx + grid_map::Index( 0, +1);

  //   double dzdx = 0.0, dzdy = 0.0; // Initialize derivatives
  // // Differentiate in X (central difference)
  // if (idx(0) > 0 && idx(0) < size(0) - 1 &&
  //     gridMap.isValid(idx_left, "elevation") &&
  //     gridMap.isValid(idx_right, "elevation"))
  // {
  //   double zL = gridMap.at("elevation", idx_left);
  //   double zR = gridMap.at("elevation", idx_right);
  //   dzdx = (zR - zL) / (2.0 * res);
  // }

  // // Diff. in Y (central difference)
  // if (idx(1) > 0 && idx(1) < size(1) - 1 &&
  //     gridMap.isValid(idx_down, "elevation") &&
  //     gridMap.isValid(idx_up, "elevation"))
  // {
  //   double zD = gridMap.at("elevation", idx_down); 
  //   double zU = gridMap.at("elevation", idx_up);
  //   dzdy = (zU - zD) / (2.0 * res);
  // }

  // // Build the normal vector 
  // Eigen::Vector3d normal(-dzdx, -dzdy, 1.0);
  // normal.normalize(); // Normalize the normal vector

  // // Write the normal vector to the grid map
  // gridMap.at("normal_x", idx) = static_cast<float>(normal.x());
  // gridMap.at("normal_y", idx) = static_cast<float>(normal.y());
  // gridMap.at("normal_z", idx) = static_cast<float>(normal.z());

  // }
  


  gm::saveGridMap(gridMap, node, gm::getMapRosbagTopic(node)); // Save the grid_map to a rosbag topic

  // publish grid map
  auto msg = grid_map::GridMapRosConverter::toMessage(gridMap);
  gridMapPub->publish(std::move(msg));

  // run
  rclcpp::spin(node->get_node_base_interface());
  return EXIT_SUCCESS;
}