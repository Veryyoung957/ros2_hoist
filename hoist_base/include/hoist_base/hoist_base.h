#ifndef MPU6050NODE_H
#define MPU6050NODE_H

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <string>
#include <serial/serial.h>

class hoist_base : public rclcpp::Node {
public:
    hoist_base();

private:
    bool set_zero_orientation(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>);
    void callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    bool zero_orientation_set;
};

#endif // MPU6050NODE_H