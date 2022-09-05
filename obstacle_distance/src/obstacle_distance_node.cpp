
#include "obstacle_distance/obstacle_distance.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<obsdist::ObstacleDistance>());

  rclcpp::shutdown();
  return 0;
}