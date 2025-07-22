#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // 使用新的头文件
#include <vector>
#include <queue>
#include <utility>  // for std::pair

#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>  // 引入四元数库
#include <geometry_msgs/msg/pose.hpp>   // 引入几何消息库
#include <unordered_map>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
using GridIndex = std::pair<int, int>;
struct GridIndexHash {
  size_t operator()(const GridIndex& idx) const {
    return std::hash<int>()(idx.first) ^ std::hash<int>()(idx.second);
  }
};

std::unordered_map<GridIndex, int8_t, GridIndexHash> global_occupancy_map_;

using PointT = pcl::PointXYZ;
class PointCloudToOccupancyGridNode : public rclcpp::Node {
public:
  PointCloudToOccupancyGridNode() : Node("pointcloud_to_occupancy_grid_node") {

    // 订阅里程计话题
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", 50,
      std::bind(&PointCloudToOccupancyGridNode::odom_callback, this, std::placeholders::_1));

    // 订阅障碍物点云话题
    sub_obstacle_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/non_ground_points_colored", 50,
      std::bind(&PointCloudToOccupancyGridNode::obstacle_callback, this, std::placeholders::_1));

    // 订阅地面点云话题
    sub_ground_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/ground_points_colored", 50,
      std::bind(&PointCloudToOccupancyGridNode::ground_callback, this, std::placeholders::_1));

    // 发布栅格图
    pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/pointcloud_occupancy_grid", 10);



    timer_ = this->create_wall_timer(
      // std::chrono::seconds(1),
      std::chrono::duration<double>(0.01),
      std::bind(&PointCloudToOccupancyGridNode::timer_callback, this)
    );


    declare_parameter("grid_resolution", 0.2); // 每格0.1米
    declare_parameter("grid_width", 100);       // 20m x 20m
    declare_parameter("grid_height", 100);
    declare_parameter("min_z", odom_z-6);
    declare_parameter("max_z", odom_z+1);

    RCLCPP_INFO(this->get_logger(), "odom_z-3=%f,odom_z+0.1=%f",odom_z-3,odom_z+0.1);
  }
  std::vector<int8_t> occupancy_grid_;
  std::vector<int8_t> new_grid;
private:
  double current_yaw_ ;
  double odom_z;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_ground_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_obstacle_;
  nav_msgs::msg::Odometry::SharedPtr msg2;
  void update_global_map(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
                       int8_t occupancy_value) {

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  pcl::fromROSMsg(*msg, *cloud);
  // RCLCPP_INFO(this->get_logger(), "odom_z-6=%f,odom_z+1=%f",odom_z-6,odom_z+1);
  double resolution = get_parameter("grid_resolution").as_double();
  float min_z = get_parameter("min_z").as_double();
  float max_z = get_parameter("max_z").as_double();
  int width_ = get_parameter("grid_width").as_int();
  int height_ = get_parameter("grid_height").as_int();
  for (const auto& pt : cloud->points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
    if (pt.z-odom_z < -1.5|| pt.z - odom_z > -0.1) continue;

    // 将点云投影到全局地图
    double world_x = (pt.x);
    double world_y = (pt.y);

    int gx = static_cast<int>(std::floor(world_x / resolution));
    int gy = static_cast<int>(std::floor(world_y / resolution));

    global_occupancy_map_[{gx, gy}] = occupancy_value;
  }
}
#include <queue>

void remove_isolated_obstacles(std::vector<int8_t>& grid, int width, int height, int min_obstacle_size = 5) {
  std::vector<bool> visited(width * height, false);
  const int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
  const int dy[8] = {0, -1, -1, -1, 0, 1, 1, 1};

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int idx = y * width + x;
      if (grid[idx] == 100 && !visited[idx]) {
        // 找到新的障碍块，进行 flood fill
        std::queue<std::pair<int, int>> q;
        std::vector<int> obstacle_indices;
        q.push({x, y});
        visited[idx] = true;
        obstacle_indices.push_back(idx);

        while (!q.empty()) {
          auto [cx, cy] = q.front(); q.pop();

          for (int i = 0; i < 8; ++i) {
            int nx = cx + dx[i];
            int ny = cy + dy[i];

            if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
            int nidx = ny * width + nx;
            if (!visited[nidx] && grid[nidx] == 100) {
              visited[nidx] = true;
              q.push({nx, ny});
              obstacle_indices.push_back(nidx);
            }
          }
        }

        // 如果障碍块太小，就去除它
        if (obstacle_indices.size() < min_obstacle_size) {
          for (int i : obstacle_indices) {
            grid[i] = 50; // 标记为 free
          }
        }
      }
    }
  }
}

void timer_callback() {
  double resolution = get_parameter("grid_resolution").as_double();
  int width = get_parameter("grid_width").as_int();
  int height = get_parameter("grid_height").as_int();

  std::vector<int8_t> local_grid(width * height, 50);

  int center_x = static_cast<int>(std::floor(current_position_.x / resolution));
  int center_y = static_cast<int>(std::floor(current_position_.y / resolution));

  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      int gx = center_x + j - width / 2;
      int gy = center_y + i - height / 2;
      GridIndex key = {gx, gy};

      auto it = global_occupancy_map_.find(key);
      if (it != global_occupancy_map_.end()) {
        local_grid[i * width + j] = it->second;
      }
    }
  }
  for (int x = 0; x < height; ++x) {
      for (int y = 0; y < width; ++y) {
          int idx = y * width + x;
          int8_t cell = local_grid[idx];  // 当前格子的值
          // RCLCPP_INFO(this->get_logger(), "Occupied cell at (%d, %d) has  nearby free cells", i, j);
        //   if (cell == 100) {
        //           int not_free_count = 0;
        //           // 检查距离1~3范围内8个方向的格子
        //         for(int i = 1;i<=3;i++){
        //             if (local_grid[(y - i) * width + x] != 50 && local_grid[(y - i) * width + x] != 100) ++not_free_count;
        //             // 下
        //             if (local_grid[(y + i) * width + x] != 50 && local_grid[(y + i) * width + x] != 100) ++not_free_count;
        //             // 左
        //             if (local_grid[y * width + (x - i)] != 50 && local_grid[y * width + (x - i)] != 100) ++not_free_count;
        //             // 右
        //             if (local_grid[y * width + (x + i)] != 50 && local_grid[y * width + (x + i)] != 100) ++not_free_count;
        //             // 左上
        //             if (local_grid[(y - i) * width + (x - i)] != 50 && local_grid[(y - i) * width + (x - i)] != 100) ++not_free_count;
        //             // 右上
        //             if (local_grid[(y - i) * width + (x + i)] != 50 && local_grid[(y - i) * width + (x + i)] != 100) ++not_free_count;
        //             // 左下
        //             if (local_grid[(y + i) * width + (x - i)] != 50 && local_grid[(y + i) * width + (x - i)] != 100) ++not_free_count;
        //             // 右下
        //             if (local_grid[(y + i) * width + (x + i)] != 50 && local_grid[(y + i) * width + (x + i)] != 100) ++not_free_count;
        //         }
        //           if(not_free_count>=18){
        //             // RCLCPP_INFO(this->get_logger(), "Occupied cell at (%d, %d) has  nearby free cells", not_free_count, not_free_count);
        //             local_grid[idx] = 0;  // 转为空闲
        //           }
        // }
    }
  }
  std::vector<int8_t> dilated_grid = local_grid;  // 拷贝一个原始副本

  int radius = 1; // 膨胀的半径（格子）

  for (int y = radius; y < height - radius; ++y) {
      for (int x = radius; x < width - radius; ++x) {
          int idx = y * width + x;

          // 如果当前格子不是白色，检查邻域内是否有白色
          if (local_grid[idx] != 0) {
              bool has_white_neighbor = false;

              for (int dy = -radius; dy <= radius; ++dy) {
                  for (int dx = -radius; dx <= radius; ++dx) {
                      int n_idx = (y + dy) * width + (x + dx);
                      if (local_grid[n_idx] == 0) {
                          has_white_neighbor = true;
                          break;
                      }
                  }
                  if (has_white_neighbor) break;
              }

              // 如果邻域内有白色，当前也变成白色
              if (has_white_neighbor) {
                  dilated_grid[idx] = 0;
              }
          }
      }
  }
  int radius2 = 1;
  local_grid = dilated_grid; // 更新为膨胀后的栅格图
  std::vector<int8_t> eroded_grid = local_grid;

  for (int y = radius2; y < height - radius2; ++y) {
      for (int x = radius2; x < width - radius2; ++x) {
          int idx = y * width + x;

          if (local_grid[idx] == 0) {
              bool all_white = true;

              for (int dy = -radius2; dy <= radius2; ++dy) {
                  for (int dx = -radius2; dx <= radius2; ++dx) {
                      int n_idx = (y + dy) * width + (x + dx);
                      if (local_grid[n_idx] != 0 ) {
                          all_white = false;
                          break;
                      }
                  }
                  if (!all_white) break;
              }

              if (!all_white) {
                  eroded_grid[idx] = 100;
              }
          }
      }
  }

  local_grid = eroded_grid;
  // remove_isolated_obstacles(local_grid, width, height, 5);

  // 发布局部地图
  nav_msgs::msg::OccupancyGrid grid_msg;
  grid_msg.header.stamp = this->get_clock()->now();
  grid_msg.header.frame_id = "camera_init";
  grid_msg.info.resolution = resolution;
  grid_msg.info.width = width;
  grid_msg.info.height = height;
  grid_msg.info.origin.position.x = (center_x - width / 2) * resolution;
  grid_msg.info.origin.position.y = (center_y - height / 2) * resolution;
  grid_msg.info.origin.position.z = odom_z;
  grid_msg.info.origin.orientation.w = 1.0;
  grid_msg.info.origin.orientation.x = 0.0;
  grid_msg.info.origin.orientation.y = 0.0;
  grid_msg.info.origin.orientation.z = 0.0;
  grid_msg.data = local_grid;

  pub_->publish(grid_msg);
}

  // void timer_callback() {
  //   if (latest_ground_) {
  //     process_pointcloud(latest_ground_,current_position_ ,0);   // 白色标记地面
  //   }
  //   if (latest_obstacle_) {
  //     process_pointcloud(latest_obstacle_, current_position_ ,100);  // 黑色标记障碍物
  //   }
  // }
  // 里程计回调，获取当前位置和姿态
  // void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  //   current_position_ = msg->pose.pose.position;
  //   current_orientation_ = msg->pose.pose.orientation;
  //   // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Point: (%f, %f, %f)", current_position_.x, current_position_.y, current_position_.z);
  // }
void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_position_ = msg->pose.pose.position;
    current_orientation_ = msg->pose.pose.orientation;
    odom_z = current_position_.z;

    // 将四元数转换为欧拉角（roll, pitch, yaw）
    tf2::Quaternion q(
        current_orientation_.x,
        current_orientation_.y,
        current_orientation_.z,
        current_orientation_.w
    );

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    current_yaw_ = yaw;  

    // 可选打印
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Yaw angle (rad): %.3f", yaw);
}
void obstacle_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  latest_obstacle_ = msg;
  update_global_map(msg, 100);
}

void ground_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  latest_ground_ = msg;
  update_global_map(msg, 0);
}

// void publish_occupancy_grid(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int width,int height,double resolution,std::vector<int8_t> occupancy_grid_) {
//   nav_msgs::msg::OccupancyGrid occ;
//   occ.header.stamp = this->get_clock()->now();
//   occ.header.frame_id = msg->header.frame_id;  

//   occ.info.resolution = resolution;
//   occ.info.width = width;
//   occ.info.height = height;
//   int origin_position_x = -(resolution*width)/2;
//   int origin_position_y = -(resolution*height)/2;
//   occ.info.origin.position.x = origin_position_x;
//   occ.info.origin.position.y = origin_position_y;
//   occ.info.origin.orientation = geometry_msgs::msg::Quaternion();
//   occ.data = occupancy_grid_;

//   pub_->publish(occ);
// }
  // 地面点云回调

  rclcpp::Time last_cloud_time_;
  // 点云处理函数
  void process_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg,  geometry_msgs::msg::Point& current_orientation,int occupancy_value) {
    // RCLCPP_INFO(this->get_logger(), 
    //         "Position: x = %.3f, y = %.3f, z = %.3f", 
    //         current_position_.x, current_position_.y, current_position_.z);

    rclcpp::Time last_publish_time = this->now();
    rclcpp::Time current_time = msg->header.stamp;
//为了循环播放bag不从半路更新地图
    if (last_cloud_time_.nanoseconds() > 0) {
        rclcpp::Duration delta = current_time - last_cloud_time_;

        if (delta.seconds() < -5.0) {  
            // RCLCPP_WARN(this->get_logger(), "Detected large time jump: %.2f sec. Clearing occupancy grid.",
            //             delta.seconds());
            std::fill(occupancy_grid_.begin(), occupancy_grid_.end(), 50);
        }
    }

    last_cloud_time_ = current_time;
    // 获取参数
    double resolution = get_parameter("grid_resolution").as_double();
    int width = get_parameter("grid_width").as_int();
    int height = get_parameter("grid_height").as_int();
    float min_z = get_parameter("min_z").as_double();
    float max_z = get_parameter("max_z").as_double();

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty point cloud.");
      return;
    }
    ///===============================
    ///============================

    // 初始化地图
    occupancy_grid_.resize(width * height, 50);
    new_grid.resize(width * height, 50);
    for (const auto& pt : cloud->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
        if (pt.z < min_z || pt.z > max_z) continue;

        double world_x = pt.x;
        double world_y = pt.y;

        int x_idx = static_cast<int>(((world_x-current_position_.x)+ (width * resolution) / 2.0) / resolution);
        int y_idx = static_cast<int>(((world_y-current_position_.y) + (height * resolution) / 2.0) / resolution);

        if (x_idx >= 0 && x_idx < width && y_idx >= 0 && y_idx < height) {
            int idx = y_idx * width + x_idx;
            int idx_1 = (y_idx-100) * width + (x_idx-100);
              occupancy_grid_[idx] = occupancy_value;  // 不清空，仅覆盖/更新
              occupancy_grid_[idx+10] = occupancy_grid_[idx];
            
            ////////ysh===============///////////////////

        }

    }
    int cur_idx_x = static_cast<int>((current_position_.x+(width*resolution)/2.0)/resolution);
    int cur_idx_y = static_cast<int>((current_position_.y+(height*resolution)/2.0)/resolution);
    // int cur_grid_idx = cur_idx_y * width + cur_idx_x;
    // RCLCPP_INFO(this->get_logger(), 
    //     "Current grid x: %d, Current grid y: %d", cur_idx_x, cur_idx_y);

    ////================
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int idx = y * width + x;
            int8_t cell = occupancy_grid_[idx];  // 当前格子的值
            if(cell==100){
                int not_free_count = 0;
                // int not_free_count1 = 0;
                // int not_free_count2 = 0;
                for(int i = 1;i<=3;i++){
                    if (occupancy_grid_[(y - i) * width + x] != 50 && occupancy_grid_[(y - i) * width + x] != 100) ++not_free_count;
                    // 下
                    if (occupancy_grid_[(y + i) * width + x] != 50 && occupancy_grid_[(y + i) * width + x] != 100) ++not_free_count;
                    // 左
                    if (occupancy_grid_[y * width + (x - i)] != 50 && occupancy_grid_[y * width + (x - i)] != 100) ++not_free_count;
                    // 右
                    if (occupancy_grid_[y * width + (x + i)] != 50 && occupancy_grid_[y * width + (x + i)] != 100) ++not_free_count;
                    // 左上
                    if (occupancy_grid_[(y - i) * width + (x - i)] != 50 && occupancy_grid_[(y - i) * width + (x - i)] != 100) ++not_free_count;
                    // 右上
                    if (occupancy_grid_[(y - i) * width + (x + i)] != 50 && occupancy_grid_[(y - i) * width + (x + i)] != 100) ++not_free_count;
                    // 左下
                    if (occupancy_grid_[(y + i) * width + (x - i)] != 50 && occupancy_grid_[(y + i) * width + (x - i)] != 100) ++not_free_count;
                    // 右下
                    if (occupancy_grid_[(y + i) * width + (x + i)] != 50 && occupancy_grid_[(y + i) * width + (x + i)] != 100) ++not_free_count;
                }
                // if (occupancy_grid_[(y - 1) * width + x] != 50 && occupancy_grid_[(y - 1) * width + x] != 100) ++not_free_count;
                // // 下
                // if (occupancy_grid_[(y + 1) * width + x] != 50 && occupancy_grid_[(y + 1) * width + x] != 100) ++not_free_count;
                // // 左
                // if (occupancy_grid_[y * width + (x - 1)] != 50 && occupancy_grid_[y * width + (x - 1)] != 100) ++not_free_count;
                // // 右
                // if (occupancy_grid_[y * width + (x + 1)] != 50 && occupancy_grid_[y * width + (x + 1)] != 100) ++not_free_count;
                // // 左上
                // if (occupancy_grid_[(y - 1) * width + (x - 1)] != 50 && occupancy_grid_[(y - 1) * width + (x - 1)] != 100) ++not_free_count;
                // // 右上
                // if (occupancy_grid_[(y - 1) * width + (x + 1)] != 50 && occupancy_grid_[(y - 1) * width + (x + 1)] != 100) ++not_free_count;
                // // 左下
                // if (occupancy_grid_[(y + 1) * width + (x - 1)] != 50 && occupancy_grid_[(y + 1) * width + (x - 1)] != 100) ++not_free_count;
                // // 右下
                // if (occupancy_grid_[(y + 1) * width + (x + 1)] != 50 && occupancy_grid_[(y + 1) * width + (x + 1)] != 100) ++not_free_count;
                // // 左
                // if (occupancy_grid_[y * width + (x - 1)] != 50 && occupancy_grid_[y * width + (x - 1)] != 100) ++not_free_count1;
                // if (occupancy_grid_[y * width + (x - 2)] != 50 && occupancy_grid_[y * width + (x - 2)] != 100) ++not_free_count1;
                // if (occupancy_grid_[y * width + (x - 3)] != 50 && occupancy_grid_[y * width + (x - 3)] != 100) ++not_free_count1;
                // if (occupancy_grid_[y * width + (x - 4)] != 50 && occupancy_grid_[y * width + (x - 4)] != 100) ++not_free_count1;
                // if (occupancy_grid_[y * width + (x - 5)] != 50 && occupancy_grid_[y * width + (x - 5)] != 100) ++not_free_count1;
                // if (occupancy_grid_[y * width + (x - 6)] != 50 && occupancy_grid_[y * width + (x - 6)] != 100) ++not_free_count1;
                // // 右
                // if (occupancy_grid_[y * width + (x + 1)] != 50 && occupancy_grid_[y * width + (x + 1)] != 100) ++not_free_count2;
                // if (occupancy_grid_[y * width + (x + 2)] != 50 && occupancy_grid_[y * width + (x + 2)] != 100) ++not_free_count2;
                // if (occupancy_grid_[y * width + (x + 3)] != 50 && occupancy_grid_[y * width + (x + 3)] != 100) ++not_free_count2;
                // if (occupancy_grid_[y * width + (x + 4)] != 50 && occupancy_grid_[y * width + (x + 4)] != 100) ++not_free_count2;
                // if (occupancy_grid_[y * width + (x + 5)] != 50 && occupancy_grid_[y * width + (x + 5)] != 100) ++not_free_count2;
                // if (occupancy_grid_[y * width + (x + 6)] != 50 && occupancy_grid_[y * width + (x + 6)] != 100) ++not_free_count2;
                // if (not_free_count1 + not_free_count2 >= 10) {
                //     occupancy_grid_[idx] = 0;  // 转为空闲
                // }
                  if (not_free_count>= 20) {  
                      occupancy_grid_[idx] = 0;  // 转为空闲
                  }
            }

        }
    }

    // publish_occupancy_grid(msg,width, height, resolution, occupancy_grid_);

    // 发布栅格地图
    nav_msgs::msg::OccupancyGrid occ;
    occ.header.stamp = this->get_clock()->now();
    occ.header.frame_id = msg->header.frame_id;
    // std::cout <<"occ.header.frame_id =="<<occ.header.frame_id <<std::endl;

    occ.info.resolution = resolution;
    occ.info.width = width;
    occ.info.height = height;
    occ.info.origin.orientation = current_orientation_;
    // occ.info.origin.position.x = -50.0;  // 例如 0 或其他全局参考点
    // occ.info.origin.position.y = -50.0;
    occ.info.origin.position.x = current_position_.x-10;  // 例如 0 或其他全局参考点
    occ.info.origin.position.y = current_position_.y-10;
    occ.info.origin.orientation = geometry_msgs::msg::Quaternion();
    occ.data = occupancy_grid_;
    // current_orientation = current_position_;
      RCLCPP_INFO(this->get_logger(), 
            "Position: x = %.3f, y = %.3f, z = %.3f", 
            current_orientation.x-current_position_.x, current_orientation.y, current_orientation.z);
    pub_->publish(occ);
  }

  // 订阅的里程计位置
  geometry_msgs::msg::Point current_position_;
  geometry_msgs::msg::Quaternion current_orientation_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_obstacle_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_ground_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;

};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudToOccupancyGridNode>());
  rclcpp::shutdown();
  return 0;
}
