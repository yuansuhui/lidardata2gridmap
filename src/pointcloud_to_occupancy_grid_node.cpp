#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // 使用新的头文件

#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>  // 引入四元数库
#include <geometry_msgs/msg/pose.hpp>   // 引入几何消息库
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
      std::chrono::seconds(1),
      // std::chrono::duration<double>(0.2),
      std::bind(&PointCloudToOccupancyGridNode::timer_callback, this)
    );


    declare_parameter("grid_resolution", 0.2); // 每格0.1米
    declare_parameter("grid_width", 500);       // 20m x 20m
    declare_parameter("grid_height", 500);
    declare_parameter("min_z", -2.5);
    declare_parameter("max_z", -0.6);

    RCLCPP_INFO(this->get_logger(), "PointCloud to OccupancyGrid node started.");
  }
  std::vector<int8_t> occupancy_grid_;
private:
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_ground_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_obstacle_;
  void timer_callback() {
    if (latest_ground_) {
      process_pointcloud(latest_ground_, 0);   // 白色标记地面
    }
    if (latest_obstacle_) {
      process_pointcloud(latest_obstacle_, 100);  // 黑色标记障碍物
    }
  }
  // 里程计回调，获取当前位置和姿态
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_position_ = msg->pose.pose.position;
    current_orientation_ = msg->pose.pose.orientation;
  }
  void ground_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // process_pointcloud(msg, 0);    // 地面用白色表示
    latest_ground_ = msg;  // 仅缓存，不立即处理
  }
  // 障碍物点云回调
  void obstacle_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // process_pointcloud(msg, 100);  // 障碍物用黑色表示
    latest_obstacle_ = msg; 
  }
void publish_occupancy_grid(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int width,int height,double resolution,std::vector<int8_t> occupancy_grid_) {
  nav_msgs::msg::OccupancyGrid occ;
  occ.header.stamp = this->get_clock()->now();
  occ.header.frame_id = msg->header.frame_id;  

  occ.info.resolution = resolution;
  occ.info.width = width;
  occ.info.height = height;
  occ.info.origin.position.x = -50.0;
  occ.info.origin.position.y = -50.0;
  occ.info.origin.orientation = geometry_msgs::msg::Quaternion();
  occ.data = occupancy_grid_;

  pub_->publish(occ);
}
  // 地面点云回调

  rclcpp::Time last_cloud_time_;
  // 点云处理函数
  void process_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int occupancy_value) {
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
    for (const auto& pt : cloud->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
        if (pt.z < min_z || pt.z > max_z) continue;

        double world_x = pt.x;
        double world_y = pt.y;

        int x_idx = static_cast<int>((world_x + (width * resolution) / 2.0) / resolution);
        int y_idx = static_cast<int>((world_y + (height * resolution) / 2.0) / resolution);

        if (x_idx >= 0 && x_idx < width && y_idx >= 0 && y_idx < height) {
            int idx = y_idx * width + x_idx;
            occupancy_grid_[idx] = occupancy_value;  // 不清空，仅覆盖/更新
            ////////ysh===============///////////////////

        }

    }
    ////================
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int idx = y * width + x;
            int8_t cell = occupancy_grid_[idx];  // 当前格子的值
            if(cell==100){
                // if((occupancy_grid_[(y+1) * width + x] != 50) &&     // 下
                //     (occupancy_grid_[(y-1) * width + x] != 50) &&     // 上
                //     (occupancy_grid_[y * width + (x-1)] != 50) &&     // 左
                //     (occupancy_grid_[y * width + (x+1)] != 50) &&     // 右
                //     (occupancy_grid_[(y-1) * width + (x-1)] != 50) && // 左上
                //     (occupancy_grid_[(y-1) * width + (x+1)] != 50) && // 右上
                //     (occupancy_grid_[(y+1) * width + (x-1)] != 50) && // 左下
                //     (occupancy_grid_[(y+1) * width + (x+1)] != 50)){    // 右下
                //       occupancy_grid_[idx]=0;
                // }
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

    publish_occupancy_grid(msg,width, height, resolution, occupancy_grid_);

    // // 发布栅格地图
    // nav_msgs::msg::OccupancyGrid occ;
    // occ.header.stamp = this->get_clock()->now();
    // occ.header.frame_id = msg->header.frame_id;
    // // std::cout <<"occ.header.frame_id =="<<occ.header.frame_id <<std::endl;

    // occ.info.resolution = resolution;
    // occ.info.width = width;
    // occ.info.height = height;
    // occ.info.origin.orientation = current_orientation_;
    // occ.info.origin.position.x = -50.0;  // 例如 0 或其他全局参考点
    // occ.info.origin.position.y = -50.0;
    // occ.info.origin.orientation = geometry_msgs::msg::Quaternion();
    // occ.data = occupancy_grid_;

    // pub_->publish(occ);
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
