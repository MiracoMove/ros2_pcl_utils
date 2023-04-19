#pragma once
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace pcl_utils {

class VoxelGridFilter : public rclcpp::Node {
 private:
  /* data */
  rclcpp::CallbackGroup::SharedPtr pcl_sub_g_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
  rclcpp::CallbackGroup::SharedPtr pcl_pub_g_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;

  /* methods */
  void PclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

 public:
  VoxelGridFilter(/* args */);
  ~VoxelGridFilter();
};

}  // namespace pcl_utils