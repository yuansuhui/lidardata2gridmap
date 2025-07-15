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
  rclcpp::Time last_process_time_;
  double min_interval_sec_ = 0.1;  // 最小处理间隔 0.1s（即10Hz）  
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
    pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("pointcloud_occupancy_grid", 10);

    declare_parameter("grid_resolution", 0.06);  // 每格0.1米
    declare_parameter("grid_width", 1000);       // 20m x 20m
    declare_parameter("grid_height", 1000);
    declare_parameter("min_z", -2.5);
    declare_parameter("max_z", 0.5);

    RCLCPP_INFO(this->get_logger(), "PointCloud to OccupancyGrid node started.");
  }
  std::vector<int8_t> occupancy_grid_;
private:
  // 里程计回调，获取当前位置和姿态
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_position_ = msg->pose.pose.position;
    current_orientation_ = msg->pose.pose.orientation;
  }

  // 障碍物点云回调
  void obstacle_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    process_pointcloud(msg, 100);  // 障碍物用黑色表示
  }

  // 地面点云回调
  void ground_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    process_pointcloud(msg, 0);    // 地面用白色表示
  }
  rclcpp::Time last_cloud_time_;
  // 点云处理函数
  void process_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int occupancy_value) {
    rclcpp::Time current_time = msg->header.stamp;

    if (last_cloud_time_.nanoseconds() > 0) {
        rclcpp::Duration delta = current_time - last_cloud_time_;

        if (delta.seconds() < -5.0) {  
            RCLCPP_WARN(this->get_logger(), "Detected large time jump: %.2f sec. Clearing occupancy grid.",
                        delta.seconds());
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

    // 初始化地图
    occupancy_grid_.resize(width * height, 50);
    // std::vector<int8_t> grid(width * height, 50);  // 初始值设置为50（灰色，未知）
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
        }
    }
    // 将点云数据转换到世界坐标系
    // for (const auto& pt : cloud->points) {
    //   // std::cout << "Point cloud size: " << cloud->points.size() << std::endl;
    //   if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
    //   if (pt.z < min_z || pt.z > max_z) continue;

    //   // 先旋转，再平移
    //   // 将四元数转换为旋转矩阵
    //   tf2::Quaternion quat(current_orientation_.x, current_orientation_.y,
    //                       current_orientation_.z, current_orientation_.w);
      
    //   // RCLCPP_INFO(this->get_logger(), "Quaternion: x=%.4f, y=%.4f, z=%.4f, w=%.4f",quat.x(), quat.y(), quat.z(), quat.w());
    //   tf2::Matrix3x3 rot_matrix(quat);
    //   // 创建 tf2::Vector3 来表示点
    //   tf2::Vector3 point(pt.x, pt.y, pt.z);

    //   // 应用旋转
    //   // tf2::Vector3 rotated_point = rot_matrix * point;
    //   tf2::Vector3 rotated_point =  point;

    //   // 然后应用平移
    //   // double world_x = rotated_point.x() + current_position_.x;
    //   // double world_y = rotated_point.y() + current_position_.y;
    //   // double world_z = rotated_point.z() + current_position_.z;
    //   double world_x = rotated_point.x() ;
    //   double world_y = rotated_point.y() ;
    //   double world_z = rotated_point.z() ;
    //   // 将转换后的点云坐标映射到栅格中
    //   int x_idx = static_cast<int>((world_x + (width * resolution) / 2.0) / resolution);
    //   int y_idx = static_cast<int>((world_y + (height * resolution) / 2.0) / resolution);

    //   if (x_idx >= 0 && x_idx < width && y_idx >= 0 && y_idx < height) {
    //     int idx = y_idx * width + x_idx;
    //     grid[idx] = occupancy_value;  // 设置为障碍物或地面
    //   }
    // }

    // 发布栅格地图
    nav_msgs::msg::OccupancyGrid occ;
    occ.header.stamp = this->get_clock()->now();
    occ.header.frame_id = msg->header.frame_id;
    // std::cout <<"occ.header.frame_id =="<<occ.header.frame_id <<std::endl;

    occ.info.resolution = resolution;
    occ.info.width = width;
    occ.info.height = height;
    // occ.info.origin.position.x = -width * resolution / 2.0 + current_position_.x;
    // occ.info.origin.position.y = -height * resolution / 2.0 + current_position_.y;
    // occ.info.origin.position.x = 0.0;
    // occ.info.origin.position.y = 0.0;
    // occ.info.origin.position.z = 0.0;
    // occ.info.origin.orientation = current_orientation_;
    // occ.info.origin.position.x = current_position_.x - width * resolution / 2.0;
    // occ.info.origin.position.y = current_position_.y - height * resolution / 2.0;
    // occ.info.origin.position.z = 0.0;
    occ.info.origin.orientation = current_orientation_;
    occ.info.origin.position.x = -5.0;  // 例如 0 或其他全局参考点
    occ.info.origin.position.y = -5.0;
    occ.info.origin.orientation = geometry_msgs::msg::Quaternion();
    occ.data = occupancy_grid_;

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
