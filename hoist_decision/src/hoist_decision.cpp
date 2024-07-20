#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_int.hpp"  // Assuming service type for setting integer

using namespace std;

class PointCloudProcessor : public rclcpp::Node {
public:
    PointCloudProcessor() : Node("point_cloud_processor") {
        point_cloud_topic = this->declare_parameter<string>("point_cloud_topic", "/cloud");
        odom_topic = this->declare_parameter<string>("odom_topic", "/odom");
        r = this->declare_parameter<float>("distance_to_target", 1.0); // Distance r from target

        point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            point_cloud_topic, 10, std::bind(&PointCloudProcessor::pointCloudCallback, this, std::placeholders::_1));

        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10, std::bind(&PointCloudProcessor::odomCallback, this, std::placeholders::_1));

        service_ = this->create_service<std_srvs::srv::Trigger>(
            "cycle_process_points", std::bind(&PointCloudProcessor::cycleProcessPoints, this, std::placeholders::_1, std::placeholders::_2));

        target_points_ = {{1.0, 2.0}, {1.5, 2.5}, {2.0, 1.5}, {0.5, 0.5}};
        special_point_ = {3.0, 3.0, 0.0};  // Special target point
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::fromROSMsg(*msg, *last_received_cloud_);
        RCLCPP_INFO(this->get_logger(), "Received and stored new point cloud data.");
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_position_ = msg->pose.pose.position;
        RCLCPP_INFO(this->get_logger(), "Updated robot position to x: %f, y: %f, z: %f", robot_position_.x, robot_position_.y, robot_position_.z);
    }

    bool cycleProcessPoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        sorted_target_points_ = findAndSortPointsInRegion(target_points_);
        std::sort(target_points_set.begin(), target_points_set.end(), 
                  [this](const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
                      return distanceToPoint(a) < distanceToPoint(b);
                  });
        
        switch (cycle_counter_ % 4) {  // Now using modulo 4 for the new cycle
            case 0:
                performFirstAction();
                response->message = "First action performed.";
                break;
            case 1:
                performSecondAction();
                response->message = "Second action performed.";
                break;
            case 2:
                performThirdAction();
                response->message = "Third action performed.";
                break;
            case 3:
                performFourthAction();  // New case for performing the fourth action
                response->message = "Fourth action performed.";
                break;
        }
        cycle_counter_++;  // Increase the counter
        response->success = true;
        return true;
    }

    void performFirstAction() {
        geometry_msgs::msg::Pose target_pose = calculateTargetPose(special_point_, true);
        bool result = GoToPose(target_pose);  // Assuming GoToPose is defined elsewhere
        RCLCPP_INFO(this->get_logger(), "Navigated to pose: success = %s", result ? "true" : "false");
        target_points_.erase(target_points_.begin());  // Removing the first point for now
    }

    void performSecondAction() {
        geometry_msgs::msg::Pose target_pose = calculateTargetPose(special_point_, false);
        bool result = GoToPose(target_pose);
        RCLCPP_INFO(this->get_logger(), "Navigated to pose: success = %s", result ? "true" : "false");
        target_points_.erase(target_points_.begin());
    }

    void performThirdAction() {
        // Calculate closest target, special handling for special_point_

        double min_distance = std::numeric_limits<double>::max();
        geometry_msgs::msg::Point closest_point;

        // Check special point first
        double distance_to_special = distanceToPoint(special_point_);
        if (!special_is_remove) {
            min_distance = distance_to_special;
            closest_point = special_point_;
            
        }

        // Check other targets
        for (auto& point : target_points_set) {
            double distance = distanceToPoint(point);
            if (distance < min_distance) {
                min_distance = distance;
                closest_point.x = point.x;
                closest_point.y = point.y;
                closest_point.z = 0.0;
                if(closest_point == special_point_)
                    special_is_remove = true;
                else target_points_set.erase(target_points_set.begin());
            }
        }

        geometry_msgs::msg::Pose target_pose = calculateTargetPose(closest_point, true);
        GoToPose(target_pose);

        // Call service based on what the closest target was
        auto client = this->create_client<std_srvs::srv::SetInt>("height_degree");
        if (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Service not available.");
            return;
        }

        auto request = std::make_shared<std_srvs::srv::SetInt::Request>();
        request->data = closest_is_special ? 2 : 3;
        auto result = client->async_send_request(request);

        // Assuming the removal of the closest point is handled elsewhere or different in each call
    }

    void performFourthAction() {
        // Similar to performThirdAction but for back-facing poses
        double min_distance = std::numeric_limits<double>::max();
        geometry_msgs::msg::Point closest_point;

        // Check special point first
        double distance_to_special = distanceToPoint(special_point_);
        if (!special_is_remove) {
            min_distance = distance_to_special;
            closest_point = special_point_;
            
        }

        // Check other targets
        for (auto& point : target_points_set) {
            double distance = distanceToPoint(point);
            if (distance < min_distance) {
                min_distance = distance;
                closest_point.x = point.x;
                closest_point.y = point.y;
                closest_point.z = 0.0;
                if(closest_point == special_point_)
                    special_is_remove = true;
                else target_points_set.erase(target_points_set.begin());
            }
        }

        geometry_msgs::msg::Pose target_pose = calculateTargetPose(closest_point, true);
        GoToPose(target_pose);

        // Call service based on what the closest target was
        auto client = this->create_client<std_srvs::srv::SetInt>("height_degree");
        if (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Service not available.");
            return;
        }

        auto request = std::make_shared<std_srvs::srv::SetInt::Request>();
        request->data = closest_is_special ? 2 : 3;
        auto result = client->async_send_request(request);

    }

    geometry_msgs::msg::Pose calculateTargetPose(const geometry_msgs::msg::Point& target, bool face_towards) {
        geometry_msgs::msg::Pose pose;
        double dx = target.x - robot_position_.x;
        double dy = target.y - robot_position_.y;
        double angle = atan2(dy, dx);

        if (!face_towards) {
            angle += M_PI;  // Add π to face away
        }

        // Calculate the pose at distance r from the target
        pose.position.x = target.x - r * cos(angle);
        pose.position.y = target.y - r * sin(angle);
        pose.position.z = 0;  // Assuming flat terrain

        // Orientation as a quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, angle);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        return pose;
    }
    vector<geometry_msgs::msg::Point> findAndSortPointsInRegion(vector<geometry_msgs::msg::Point> points_) {
        vector<geometry_msgs::msg::Point> points_in_region;
        for (auto& point : last_received_cloud_->points) {
            for (auto& target : points_) {
                if (isPointInRegion(point.x, point.y, target.first, target.second, 0.5)) {
                    geometry_msgs::msg::Point pt{point.x, point.y, point.z};
                    points_in_region.push_back(pt);
                }
            }
        }
        std::sort(points_in_region.begin(), points_in_region.end(), 
                  [this](const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
                      return distanceToPoint(a) < distanceToPoint(b);
                  });
        return points_in_region;
    }
    double distanceToPoint(const geometry_msgs::msg::Point& point) {
        return sqrt(pow(point.x - robot_position_.x, 2) + pow(point.y - robot_position_.y, 2));
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
    vector<geometry_msgs::msg::Point> target_points_,target_points_set,sorted_target_points_,sorted_target_points_set;
    geometry_msgs::msg::Point robot_position_, special_point_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr last_received_cloud_{new pcl::PointCloud<pcl::PointXYZ>};
    string point_cloud_topic, odom_topic;
    float r;  // Distance to maintain from the target point
    int cycle_counter_ = 0;  // Counter for service calls
    bool special_is_remove = false;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudProcessor>());
    rclcpp::shutdown();
    return 0;
}
