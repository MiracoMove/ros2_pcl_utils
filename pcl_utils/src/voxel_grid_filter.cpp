#include "pcl_utils/voxel_grid_filter.hpp"

#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"

namespace pcl_utils {

VoxelGridFilter::VoxelGridFilter(/* args */) : rclcpp::Node("pcl_voxel_grid_node") {
  pcl_sub_g_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions pcl_sub_opt;
  pcl_sub_opt.callback_group = pcl_sub_g_;
  pcl_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "camera/points", rclcpp::QoS{5}, std::bind(&VoxelGridFilter::PclCallback, this, std::placeholders::_1),
      pcl_sub_opt);

  pcl_pub_g_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::PublisherOptions pcl_pub_opt;
  pcl_pub_opt.callback_group = pcl_pub_g_;
  pcl_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("camera/points_filtered", rclcpp::SensorDataQoS(), pcl_pub_opt);
}

VoxelGridFilter::~VoxelGridFilter() {}

void VoxelGridFilter::PclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(cloud);
  filter.setLeafSize(0.05f, 0.05f, 0.05f);
  filter.filter(*cloud);

  pcl::PassThrough<pcl::PointXYZ> pass_z;
  pass_z.setInputCloud(cloud);
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(0.2, 3.9);
  pass_z.filter(*cloud);

  pcl::PassThrough<pcl::PointXYZ> pass_y;
  pass_y.setInputCloud(cloud);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(0, 0.7);
  pass_y.filter(*cloud);

  sensor_msgs::msg::PointCloud2 plc_msg;
  pcl::toROSMsg(*cloud, plc_msg);
  plc_msg.header = msg->header;
  pcl_pub_->publish(std::move(plc_msg));
}

}  // namespace pcl_utils

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<pcl_utils::VoxelGridFilter>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}