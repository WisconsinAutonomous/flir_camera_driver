#include "flir_camera_synchronizer/flir_camera_synchronizer.h"

#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<flir_camera_synchronizer::FlirCameraSynchronizer>(options);

  RCLCPP_INFO(node->get_logger(), "flir_camera_synchronizer started up!");
  // actually run the node
  rclcpp::spin(node);  // should not return
  rclcpp::shutdown();
  return 0;
}

