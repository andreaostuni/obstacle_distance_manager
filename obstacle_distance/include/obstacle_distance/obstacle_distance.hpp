#define DEBUG_TOPICS true

#ifdef DEBUG_TOPICS
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#endif

// ROS2
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"

#include "obstacle_distance_msgs/msg/obstacle_distance.hpp"
#include "obstacle_distance_msgs/srv/get_obstacle_distance.hpp"
#include "obstacle_distance_msgs/srv/get_static_obstacle_distance.hpp"
#include "rclcpp/rclcpp.hpp"

#include <mutex>
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace obsdist {

class ObstacleDistance : public rclcpp::Node {
public:
  /**
   * @brief Construct a new ObstacleDistance object
   *
   */
  ObstacleDistance();
  /**
   * @brief Destroy the ObstacleDistance object
   *
   */
  ~ObstacleDistance();

  void obstacleDistanceService(
      const std::shared_ptr<
          obstacle_distance_msgs::srv::GetObstacleDistance::Request>
          request,
      std::shared_ptr<
          obstacle_distance_msgs::srv::GetObstacleDistance::Response>
          response);

  bool staticObstacleDistanceService(
      const std::shared_ptr<
          obstacle_distance_msgs::srv::GetStaticObstacleDistance::Request>
          request,
      std::shared_ptr<
          obstacle_distance_msgs::srv::GetStaticObstacleDistance::Response>
          response);

  void gridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

#ifdef DEBUG_TOPICS
  void
  clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr point_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr closestobs_pub_;
#endif

private:
  obstacle_distance_msgs::msg::ObstacleDistance
  compute_obstacle_distance(const nav_msgs::msg::OccupancyGrid &og,
                            nav_msgs::msg::OccupancyGrid &vis);
  void getStaticMap();

  void look_label_in_distance(const cv::Mat &labels, int label, int celldist,
                              int row, int col, int width, int height,
                              unsigned int &index, float &dist);

  // cv::Mat binaryMap_;
  cv::Mat distMap_;
  bool distMap_initialized_;
  int obs_thresh_;
  obstacle_distance_msgs::msg::ObstacleDistance obsDist_;
  nav_msgs::msg::OccupancyGrid vis_;
  std::mutex mutex_;

  // service servers
  rclcpp::Service<obstacle_distance_msgs::srv::GetObstacleDistance>::SharedPtr
      obs_srv_;
  rclcpp::Service<obstacle_distance_msgs::srv::GetStaticObstacleDistance>::
      SharedPtr static_obs_srv_;

  // service clients
  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr static_map_client_;

  // topic subscriptors
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;

  // topic publishers
  rclcpp::Publisher<obstacle_distance_msgs::msg::ObstacleDistance>::SharedPtr
      obs_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr og_pub_;
  static char * cost_translation_table_;
};
} // namespace obsdist