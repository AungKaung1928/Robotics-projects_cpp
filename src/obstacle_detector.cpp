#include "simple_navigator/obstacle_detector.hpp"
#include <algorithm>

ObstacleDetector::ObstacleDetector() : Node("obstacle_detector")
{
    // Subscribe to LiDAR data
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, 
        std::bind(&ObstacleDetector::scan_callback, this, std::placeholders::_1));
    
    // Publish obstacle info [distance, angle]
    obstacle_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/obstacle_info", 10);
    
    RCLCPP_INFO(this->get_logger(), "Obstacle Detector Started");
}

void ObstacleDetector::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    float min_distance = 10.0;  // Start with large number
    int min_index = 0;
    
    // Find closest obstacle
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        if (msg->ranges[i] < min_distance && msg->ranges[i] > 0.1) {
            min_distance = msg->ranges[i];
            min_index = i;
        }
    }
    
    // Calculate angle of closest obstacle
    float angle = msg->angle_min + min_index * msg->angle_increment;
    
    // Publish [distance, angle]
    auto obstacle_msg = std_msgs::msg::Float32MultiArray();
    obstacle_msg.data = {min_distance, angle};
    obstacle_pub_->publish(obstacle_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleDetector>());
    rclcpp::shutdown();
    return 0;
}