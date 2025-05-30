#include "obstacle_distance/obstacle_distance.hpp"

namespace obsdist {

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

char * ObstacleDistance::cost_translation_table_ = NULL;
/**
 * @brief Construct a new Obstacle Distance:: Obstacle Distance object
 *
 */
ObstacleDistance::ObstacleDistance() : Node("obstacle_distance") {

  // 3 working modes:
  //
  //  1 - the node works through a service (/get_obstacle_distance) which
  //      receives an OccupancyGrid as input and returns an ObstacleDistance
  //      msg.
  //
  //  2 - The node uses the map_server service /map to get the static
  //      map only once, and send the created ObstacleDistance when
  //      receives a call to the service /get_static_obstacle_distance.
  //
  //  3 - The node subcribes to the OccupancyGrid topic published by
  //      a local or a global costmap (the user must indicate the topic name)
  //      and publishes the created ObstacleDistance in the topic
  //      /obstacle_distance
  int mode = this->declare_parameter<int>("mode", 1);
  obs_thresh_ = this->declare_parameter<int>("obstacle_threshold", 100);
  distMap_initialized_ = false;

  og_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "obstacle_distance_occGrid", 1);

  if (mode == 1) {
    obs_srv_ =
        this->create_service<obstacle_distance_msgs::srv::GetObstacleDistance>(
            std::string("get_obstacle_distance"),
            std::bind(&ObstacleDistance::obstacleDistanceService, this, _1,
                      _2));
    RCLCPP_INFO(this->get_logger(),
                "Mode %i. Service /get_obstacle_distance enabled", mode);
  } else if (mode == 2) {
    // map_server service client
    static_map_client_ =
        this->create_client<nav_msgs::srv::GetMap>("map_server/map");
    // service server
    static_obs_srv_ = this->create_service<
        obstacle_distance_msgs::srv::GetStaticObstacleDistance>(
        std::string("get_static_obstacle_distance"),
        std::bind(&ObstacleDistance::staticObstacleDistanceService, this, _1,
                  _2));
    RCLCPP_INFO(this->get_logger(),
                "Mode %i. Getting static map from /map_server/map service...",
                mode);
    getStaticMap();

  } else if (mode == 3) {
    std::string costmap_topic = this->declare_parameter<std::string>(
        "costmap_topic", std::string("local_costmap/costmap"));

    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        costmap_topic, 1, std::bind(&ObstacleDistance::gridCallback, this, _1));

    obs_pub_ =
        this->create_publisher<obstacle_distance_msgs::msg::ObstacleDistance>(
            "obstacle_distance", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    RCLCPP_INFO(this->get_logger(),
                "Mode %i. subscribed to topic /%s\nand publishing in topics "
                "/obstacle_distance and /obstacle_distance_occGrid",
                mode, costmap_topic.c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "Mode not recognized! Shutting down node!!");
    rclcpp::shutdown();
  }

#ifdef DEBUG_TOPICS
  point_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "my_clicked_point", 1);
  closestobs_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "my_closest_obstacle", 1);
  point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "clicked_point", 1,
      std::bind(&ObstacleDistance::clickedPointCallback, this, _1));
#endif
}

/**
 * @brief Destroy the Obstacle Distance:: Obstacle Distance object
 *
 */
ObstacleDistance::~ObstacleDistance() {}

/**
 * @brief obstacleDistancService
 *
 * @param request contains an input OccupancyGrid msg
 * @param response contains an output ObstacleDistance msg
 */
void ObstacleDistance::obstacleDistanceService(
    const std::shared_ptr<
        obstacle_distance_msgs::srv::GetObstacleDistance::Request>
        request,
    std::shared_ptr<obstacle_distance_msgs::srv::GetObstacleDistance::Response>
        response) {
  nav_msgs::msg::OccupancyGrid og = request->map;
  // obstacle_distance_msgs::msg::ObstacleDistance od =
  // compute_obstacle_distance(og);
  nav_msgs::msg::OccupancyGrid vis;
  response->obstacledistance = compute_obstacle_distance(og, vis);
}

/**
 * @brief staticObstacleDistanceService
 *
 * @param request empty string
 * @param response contains an the ObstacleDistance msg obtained from the static
 * map of the map server
 */
bool ObstacleDistance::staticObstacleDistanceService(
    const std::shared_ptr<
        obstacle_distance_msgs::srv::GetStaticObstacleDistance::Request>,
    std::shared_ptr<
        obstacle_distance_msgs::srv::GetStaticObstacleDistance::Response>
        response) {

  if (distMap_initialized_) {
    mutex_.lock();
    response->obstacledistance = obsDist_;
    // Check whether these publications provoke any problem
    og_pub_->publish(vis_);
    obs_pub_->publish(obsDist_);
    mutex_.unlock();
    return true;
  }
  return false;
}

void ObstacleDistance::gridCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Costmap received!");
  nav_msgs::msg::OccupancyGrid vi;
  nav_msgs::msg::OccupancyGrid og;
  og.header = msg->header;
  og.info = msg->info;
  og.data = msg->data;

  std::chrono::time_point<std::chrono::system_clock> t1 =
      std::chrono::system_clock::now();
  obstacle_distance_msgs::msg::ObstacleDistance od =
      compute_obstacle_distance(og, vi);

  std::chrono::time_point<std::chrono::system_clock> t2 =
      std::chrono::system_clock::now();
  auto millisecs =
      std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  RCLCPP_DEBUG(this->get_logger(),
              "Time to compute obstacle dist: %.4f millisecs",
              (float)millisecs);

  mutex_.lock();
  distMap_initialized_ = true;
  obsDist_ = od;
  vis_ = vi;
  mutex_.unlock();
  obs_pub_->publish(od);
  og_pub_->publish(vi);
}

void ObstacleDistance::getStaticMap() {
  while (!static_map_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  RCLCPP_DEBUG(this->get_logger(), "Geting static map...");
  // Call the service
  auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
  auto result = static_map_client_->async_send_request(request);
  
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                         result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_DEBUG(this->get_logger(), "Map received!");
    nav_msgs::msg::OccupancyGrid vis;
    obstacle_distance_msgs::msg::ObstacleDistance od =
        compute_obstacle_distance(result.get()->map, vis);
    mutex_.lock();
    obsDist_ = od;
    vis_ = vis;
    distMap_initialized_ = true;
    mutex_.unlock();
    og_pub_->publish(vis);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service /map for getting "
                                     "the static map from the map server");
  }
}

void ObstacleDistance::look_label_in_distance(const cv::Mat &labels, int label,
                                              int celldist, int row, int col,
                                              int width, int height,
                                              unsigned int &index,
                                              float &dist) {
                                                
  static const std::vector<std::pair<int, int>> directions = {
      {-celldist, 0}, {celldist, 0}, {0, -celldist}, {0, celldist}};

  int closest_index = -1;
  float min_dist = std::numeric_limits<float>::max();

  for (const auto &[dr, dc] : directions) {
    int r = row + dr;
    int c = col + dc;

    if (r >= 0 && r < height && c >= 0 && c < width) {
      for (int d = -celldist; d <= celldist; ++d) {
        int nr = (dr == 0) ? r : r + d;
        int nc = (dc == 0) ? c : c + d;

        if (nr >= 0 && nr < height && nc >= 0 && nc < width) {
          if (labels.at<int>(nc, nr) == label && distMap_.at<float>(nc, nr) < 
              min_dist) {
              min_dist = distMap_.at<float>(nc, nr);
              closest_index = nr * width + nc;
            
          }
        }
      }
    }
  }
  index = closest_index;
  dist = min_dist;
}

obstacle_distance_msgs::msg::ObstacleDistance
ObstacleDistance::compute_obstacle_distance(
    const nav_msgs::msg::OccupancyGrid &og, nav_msgs::msg::OccupancyGrid &vis) {

  // define the obstacle distance message
  obstacle_distance_msgs::msg::ObstacleDistance od;
  od.header = og.header;
  od.header.stamp = this->get_clock()->now();
  od.info = og.info;

  // fill the visualization message (for rviz)
  vis.header = og.header;
  vis.header.stamp = od.header.stamp;
  vis.info = og.info;

  // allocate map structs so that x/y in the world correspond to x/y in the
  // image
  // (=> cv::Mat is rotated by 90 deg, because it's row-major!)
  cv::Mat binaryMap = cv::Mat(og.info.width, og.info.height, CV_8UC1); // 8-bit single channel (grayscale) image
  distMap_.release();
  distMap_ = cv::Mat(binaryMap.size(), CV_32FC1); // 32-bit float single channel image for distance map
  cv::Mat visualizationMap = cv::Mat(binaryMap.size(), CV_8UC1); // 8-bit single channel (grayscale) image for visualization
  cv::Mat labels = cv::Mat(binaryMap.size(), CV_32SC1); // 32-bit signed int single channel image for labels

  std::vector<signed char>::const_iterator mapDataIter = og.data.begin(); // iterator for the occupancy grid data
  unsigned char map_occ_thresh = (unsigned char)obs_thresh_; // threshold for occupied cells

  // iterate over map, store in image
  // (0,0) is lower left corner of OccupancyGrid
  for (unsigned int j = 0; j < og.info.height; ++j) {
    for (unsigned int i = 0; i < og.info.width; ++i) {
      if (*mapDataIter > map_occ_thresh) {
        binaryMap.at<uchar>(i, j) = 0; // 0 -> black (occupied)
      } else {
        binaryMap.at<uchar>(i, j) = 255; // 255 -> white (free)
      }
      mapDataIter++;
    }
  }

  cv::distanceTransform(binaryMap, distMap_, labels, cv::DIST_L2,
                        cv::DIST_MASK_PRECISE, cv::DIST_LABEL_PIXEL);

  // Get a costmap for visualization
  cv::Mat cellDistMap = distMap_.clone();
  cv::Mat dst;
  if (cost_translation_table_ == NULL) {
    cost_translation_table_ = new char[256];

    // special values:
    cost_translation_table_[0] = 0;  // NO obstacle
    cost_translation_table_[253] = 99;  // INSCRIBED obstacle
    cost_translation_table_[254] = 100;  // LETHAL obstacle
    cost_translation_table_[255] = -1;  // UNKNOWN

    // regular cost values scale the range 1 to 252 (inclusive) to fit
    // into 1 to 98 (inclusive).
    for (int i = 1; i < 253; i++) {
      cost_translation_table_[i] = static_cast<char>(1 + (97 * (i - 1)) / 251);
    }
  }
  cv::normalize(distMap_, dst, 0, 1, cv::NORM_MINMAX);
  for (unsigned int j = 0; j < og.info.height; ++j) {
    for (unsigned int i = 0; i < og.info.width; ++i) {
      auto cost_value = 255 - static_cast<unsigned char>(
          (dst.at<float>(i, j) * 255 + 0.5));
      visualizationMap.at<unsigned char>(i, j) = 
          cost_translation_table_[cost_value];
      
      distMap_.at<float>(i, j) = distMap_.at<float>(i, j) * od.info.resolution;
    }
  }

  std::chrono::time_point<std::chrono::system_clock> t4 =
      std::chrono::system_clock::now();

  // compute labels
  int index = 0;
  // j-> row, i -> col
  for (unsigned int j = 0; j < od.info.height; ++j) {
    for (unsigned int i = 0; i < od.info.width; ++i) {
      int celldist = static_cast<int>(cellDistMap.at<float>(i, j) + 0.5);
      if (celldist == 0)
        od.indexes.push_back(index);
      else {
        unsigned int final_idx;
        float min_dist = std::numeric_limits<float>::max();
        unsigned int idx;
        for (int cdist = celldist - 1; cdist <= celldist + 1; cdist++) {
          if (cdist > 0) {
            float dist = std::numeric_limits<float>::max();
            look_label_in_distance(labels, labels.at<int>(i, j), cdist, j, i,
                                   od.info.width, od.info.height, idx, dist);
            if (dist < min_dist) {
              min_dist = dist;
              final_idx = idx;
            }
          }
        }
        od.indexes.push_back((int)final_idx);
      }
      index++;
    }
  }

  std::chrono::time_point<std::chrono::system_clock> t5 =
      std::chrono::system_clock::now();

  auto millisecs4 =
      std::chrono::duration_cast<std::chrono::milliseconds>(t5 - t4).count();

  RCLCPP_DEBUG(this->get_logger(), "Time to compute labels: %li millisecs",
              millisecs4);

  // distance map is in cells,
  // we transform to distance in meters

  // Transform back to array data
  vis.data.clear();
  for (unsigned int j = 0; j < od.info.height; ++j) {
    for (unsigned int i = 0; i < od.info.width; ++i) {
      od.distances.push_back(distMap_.at<float>(i, j));
      vis.data.push_back((int)visualizationMap.at<unsigned char>(i, j));
    }
  }
  return od;
}

#ifdef DEBUG_TOPICS
void ObstacleDistance::clickedPointCallback(
    const geometry_msgs::msg::PointStamped::SharedPtr msg) {

  mutex_.lock();
  obstacle_distance_msgs::msg::ObstacleDistance od = obsDist_;
  mutex_.unlock();

  // map point (person) to cell in the distance grid
  unsigned int xcell = (unsigned int)floor(
      (msg->point.x - od.info.origin.position.x) / od.info.resolution);
  unsigned int ycell = (unsigned int)floor(
      (msg->point.y - od.info.origin.position.y) / od.info.resolution);
  // cell to index of the array
  if (xcell >= (unsigned int)od.info.width) {
    xcell = (unsigned int)(od.info.width - 1);
  }
  if (ycell >= (unsigned int)od.info.height) {
    ycell = (unsigned int)(od.info.height - 1);
  }
  unsigned int index = xcell + ycell * od.info.width;

  float dist = od.distances[index]; // not used
  unsigned int ob_idx = od.indexes[index];

  // index to cell
  if (ob_idx >= (od.info.width * od.info.height)) {
    RCLCPP_WARN_STREAM(
        this->get_logger(),
        "Index " << ob_idx << " greater than map size!!! Setting to last cell");
    ob_idx = (od.info.width * od.info.height) - 1;
  }
  unsigned int o_ycell = floor(ob_idx / od.info.width);
  unsigned int o_xcell = ob_idx % od.info.width;

  // cell to world point (obstacle)
  float x = o_xcell * od.info.resolution + od.info.origin.position.x;
  float y = o_ycell * od.info.resolution + od.info.origin.position.y;

  RCLCPP_DEBUG_STREAM(
      this->get_logger(),
      "Clicked point: x: " << msg->point.x << " y: " << msg->point.y
                           << " Cell x: " << xcell << " cell y: " << ycell
                           << " Index: " << index
                           << " obstacle distance: " << dist
                           << " Closest obstacle index: " << ob_idx
                           << " Cell xcell: " << o_xcell
                           << " ycell: " << o_ycell
                           << " World point x: " << x
                           << " y: " << y);
  // Publish the clicked point and the closest obstacle

  visualization_msgs::msg::Marker m1;
  m1.header.frame_id = "map";
  m1.header.stamp = this->get_clock()->now();
  m1.type = m1.SPHERE;
  m1.id = 1;
  m1.action = m1.ADD;
  m1.scale.x = 0.4;
  m1.scale.y = 0.4;
  m1.scale.z = 0.4;
  m1.color.a = 1.0;
  m1.color.r = 0.0;
  m1.color.g = 0.0;
  m1.color.b = 1.0;
  m1.pose.position.x = msg->point.x;
  m1.pose.position.y = msg->point.y;
  m1.pose.position.z = 0.02;
  visualization_msgs::msg::Marker m2;
  m2 = m1;
  m2.color.r = 1.0;
  m2.color.g = 0.0;
  m2.color.b = 0.0;
  m2.pose.position.x = x;
  m2.pose.position.y = y;
  m2.pose.position.z = 0.02;
  point_pub_->publish(m1);
  closestobs_pub_->publish(m2);
}
#endif

} // namespace obsdist