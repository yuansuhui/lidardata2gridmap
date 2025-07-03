#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using PointT = pcl::PointXYZ;

class PointCloudToOccupancyGridNode : public rclcpp::Node {
public:
  PointCloudToOccupancyGridNode() : Node("pointcloud_to_occupancy_grid_node") {
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/ground_points_colored", 10,
      std::bind(&PointCloudToOccupancyGridNode::pointcloud_callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("pointcloud_occupancy_grid", 10);

    declare_parameter("grid_resolution", 0.1);  // 每格0.1米
    declare_parameter("grid_width", 500);       // 20m x 20m
    declare_parameter("grid_height", 500);
    declare_parameter("min_z", -100.0);
    declare_parameter("max_z", 100.0);

    RCLCPP_INFO(this->get_logger(), "PointCloud to OccupancyGrid node started.");
  }

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // 参数
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

    // 初始化地图 [-10m, +10m] 假设为中心
    std::vector<int8_t> grid(width * height, 0);  // 未占据默认 0

    for (const auto& pt : cloud->points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
      if (pt.z < min_z || pt.z > max_z) continue;

      int x_idx = static_cast<int>((pt.x + (width * resolution) / 2.0) / resolution);
      int y_idx = static_cast<int>((pt.y + (height * resolution) / 2.0) / resolution);

      if (x_idx >= 0 && x_idx < width && y_idx >= 0 && y_idx < height) {
        int idx = y_idx * width + x_idx;
        grid[idx] = 100;  // 占据
      }
    }

    nav_msgs::msg::OccupancyGrid occ;
    occ.header.stamp = this->get_clock()->now();
    occ.header.frame_id = msg->header.frame_id;

    occ.info.resolution = resolution;
    occ.info.width = width;
    occ.info.height = height;
    occ.info.origin.position.x = -width * resolution / 2.0;
    occ.info.origin.position.y = -height * resolution / 2.0;
    occ.info.origin.position.z = 0.0;
    occ.info.origin.orientation.w = 1.0;

    occ.data = grid;

    pub_->publish(occ);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudToOccupancyGridNode>());
  rclcpp::shutdown();
  return 0;
}
