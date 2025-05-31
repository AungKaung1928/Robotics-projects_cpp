#include "simple_navigator/simple_navigator.hpp"

SimpleNavigator::SimpleNavigator() : Node("simple_navigator")
{
    // Subscribe to obstacle info
    obstacle_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/obstacle_info", 10,
        std::bind(&SimpleNavigator::obstacle_callback, this, std::placeholders::_1));
    
    // Publisher for robot movement
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // Timer to control robot regularly
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&SimpleNavigator::move_robot, this));
    
    obstacle_distance_ = 10.0;
    obstacle_detected_ = false;
    
    RCLCPP_INFO(this->get_logger(), "Simple Navigator Started");
}

void SimpleNavigator::obstacle_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() >= 2) {
        obstacle_distance_ = msg->data[0];
        obstacle_angle_ = msg->data[1];
        obstacle_detected_ = (obstacle_distance_ < 1.0);  // 1 meter threshold
    }
}

void SimpleNavigator::move_robot()
{
    auto cmd = geometry_msgs::msg::Twist();
    
    if (obstacle_detected_) {
        // Turn away from obstacle
        cmd.angular.z = (obstacle_angle_ > 0) ? -0.5 : 0.5;  // Turn opposite direction
        cmd.linear.x = 0.0;  // Stop forward movement
        RCLCPP_INFO(this->get_logger(), "Avoiding obstacle at %.2fm", obstacle_distance_);
    } else {
        // Move forward
        cmd.linear.x = 0.3;  // 0.3 m/s forward
        cmd.angular.z = 0.0;  // No turning
    }
    
    cmd_pub_->publish(cmd);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleNavigator>());
    rclcpp::shutdown();
    return 0;
}