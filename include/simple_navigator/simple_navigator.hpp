
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <chrono>

class SimpleNavigator : public rclcpp::Node
{
public:
    SimpleNavigator();

private:
    void obstacle_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void move_robot();
    
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr obstacle_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    float obstacle_distance_;
    float obstacle_angle_;
    bool obstacle_detected_;
};