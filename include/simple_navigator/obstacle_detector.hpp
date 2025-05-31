#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class ObstacleDetector : public rclcpp::Node
{
public:
    ObstacleDetector();

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr obstacle_pub_;
};