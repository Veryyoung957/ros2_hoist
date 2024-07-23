#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "hoist_msgs/msg/gimble.hpp"
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include <sensor_msgs/point_cloud2_iterator.hpp>

#define NONE 0
#define SPIN 1
#define GO_TO_POSE 2
#define BACKUP 3
#define COMPUTE_PATH 4
#define FOLLOW_PATH 5
#define FOLLOW_WAYPOINTS 6


class hoist_decision : public rclcpp::Node {
public:
    hoist_decision() : Node("point_cloud_processor") {
        point_cloud_topic = this->declare_parameter<std::string>("point_cloud_topic", "/cloud");
        odom_topic = this->declare_parameter<std::string>("odom_topic", "/odometry/filtered");
        r = this->declare_parameter<float>("distance_to_target", 0.4); // Distance r from target

        point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            point_cloud_topic, 10, std::bind(&hoist_decision::pointCloudCallback, this, std::placeholders::_1));

        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10, std::bind(&hoist_decision::odomCallback, this, std::placeholders::_1));

        gimble_publisher_ = this->create_publisher<hoist_msgs::msg::Gimble>("gimble_mode",4);

        service_ = this->create_service<std_srvs::srv::Trigger>(
            "cycle_process_points", std::bind(&hoist_decision::cycleProcessPoints, this, std::placeholders::_1, std::placeholders::_2));

        target_points_.push_back(create_point(0.05, 0.0));
        target_points_.push_back(create_point(0.425, 0.));
        target_points_.push_back(create_point(0.425, 1.29903));
        target_points_.push_back(create_point(0.6125, 0.64951));
        target_points_.push_back(create_point(1.175, 1.29903));
        target_points_.push_back(create_point(0.9875, 0.64951));
        target_points_.push_back(create_point(1.550, 0.0));
        target_points_.push_back(create_point(1.175, 0.0));
        target_points_.push_back(create_point(1.175, -1.29903));
        target_points_.push_back(create_point(0.9875, -0.64951));
        target_points_.push_back(create_point(0.425, -1.29903));
        target_points_.push_back(create_point(0.6125, -0.64951));
        special_point_ = create_point(1.2,0.0);  // Special target point
        target_points_set.push_back(create_point(-1.305, -0.755));
        target_points_set.push_back(create_point(-1.305, 0.755));
        target_points_set.push_back(create_point(1.755, 0.755));
        target_points_set.push_back(create_point(1.755, -0.755));
    }
    bool GoToPose(geometry_msgs::msg::Pose::SharedPtr pose,int mode_1,int mode_2)
    {
        using namespace std::placeholders;

        nav_to_pose_client->wait_for_action_server();
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose.pose.position = pose->position;
        goal_msg.pose.pose.orientation = pose->orientation;
        goal_msg.behavior_tree = "";

        current_executing = GO_TO_POSE;
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&hoist_decision::goal_response_callback, this, _1 );
        // send_goal_options.goal_response_callback =
        //     [this](std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> future) {
        //         this->goal_response_callback(future);
        //     };
        send_goal_options.feedback_callback =
            std::bind(&hoist_decision::feedback_callback<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>>, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&hoist_decision::result_callback, this, _1, mode_1, mode_2);

        auto send_goal_future = nav_to_pose_client->async_send_goal(goal_msg,send_goal_options);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(),send_goal_future);
        auto goal_handle = send_goal_future.get();
        if (goal_handle != NULL ) {
        if ( goal_handle->get_status() != action_msgs::msg::GoalStatus::STATUS_ACCEPTED ) {
            if (debug)
            RCLCPP_INFO(this->get_logger(),"GoToPose request was rejected");
            current_executing = NONE;
            return false;
        }
        }
        else {
            if (debug)
                RCLCPP_INFO(this->get_logger(),"GoToPose request was rejected");
            current_executing = NONE;    
            return false;
        }
        if (debug)
        RCLCPP_INFO(this->get_logger(),"GoToPose request was accepted.");
        //future_go_to_pose = nav_to_pose_client->async_get_result(goal_handle);
        
        return true;
    }
private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        last_received_cloud_ = msg;
        RCLCPP_INFO(this->get_logger(), "Received and stored new point cloud data.");
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_position_ = msg->pose.pose.position;
        //RCLCPP_INFO(this->get_logger(), "Updated robot position to x: %f, y: %f, z: %f", robot_position_.x, robot_position_.y, robot_position_.z);
    }

    bool cycleProcessPoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        sorted_target_points_ = findAndSortPointsInRegion(target_points_);
        std::sort(target_points_set.begin(), target_points_set.end(), 
                  [this](const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
                      return distanceToPoint(a) < distanceToPoint(b);
                  });
        
        switch (cycle_counter_ % 4) { 
            case 0:
                performFirstAction();
                break;
            case 1:
                performSecondAction();
                break;
            case 2:
                performThirdAction();
                break;
            case 3:
                performFourthAction(); 
                break;
        }
        cycle_counter_++;  // Increase the counter
        response->success = true;
        return true;
    }

    void performFirstAction() {
        geometry_msgs::msg::Point::Ptr is_target_point = checkPointsForCloud(std::make_shared<geometry_msgs::msg::Point>(target_points_[0]),std::make_shared<geometry_msgs::msg::Point>(target_points_[1]));
        if (is_target_point){
            geometry_msgs::msg::Pose::Ptr target_pose = calculateTargetPose(is_target_point, true);
            bool result = GoToPose(target_pose,1,1);  // Assuming GoToPose is defined elsewhere
            RCLCPP_INFO(this->get_logger(), "Navigated to pose: success = %s", result ? "true" : "false");
            // target_points_.erase(target_points_.begin());  // Removing the first point for now
        }
    }

    void performSecondAction() {
        geometry_msgs::msg::Point::Ptr is_target_point = checkPointsForCloud(std::make_shared<geometry_msgs::msg::Point>(target_points_[2]),std::make_shared<geometry_msgs::msg::Point>(target_points_[3]));
        if (is_target_point){
            geometry_msgs::msg::Pose::Ptr target_pose = calculateTargetPose(is_target_point, false);
            bool result = GoToPose(target_pose,1,2);  // Assuming GoToPose is defined elsewhere
            RCLCPP_INFO(this->get_logger(), "Navigated to pose: success = %s", result ? "true" : "false");
            // target_points_.erase(target_points_.begin());  // Removing the first point for now
        }
    }

    void performThirdAction() {
        geometry_msgs::msg::Pose::Ptr target_pose = calculateTargetPose(std::make_shared<geometry_msgs::msg::Point>(target_points_set[0]), true);
        GoToPose(target_pose,3,1);
    }

    void performFourthAction() {
        // Similar to performThirdAction but for back-facing poses
        geometry_msgs::msg::Pose::Ptr target_pose = calculateTargetPose(std::make_shared<geometry_msgs::msg::Point>(target_points_set[1]), true);
        GoToPose(target_pose,3,2);

    }
    void performFifthAction() {
        geometry_msgs::msg::Point::Ptr is_target_point = checkPointsForCloud(std::make_shared<geometry_msgs::msg::Point>(target_points_[4]),std::make_shared<geometry_msgs::msg::Point>(target_points_[5]));
        if (is_target_point){
            geometry_msgs::msg::Pose::Ptr target_pose = calculateTargetPose(is_target_point, true);
            bool result = GoToPose(target_pose,1,1);  // Assuming GoToPose is defined elsewhere
            RCLCPP_INFO(this->get_logger(), "Navigated to pose: success = %s", result ? "true" : "false");
        }
    }

    void performSixthAction() {
        geometry_msgs::msg::Point::Ptr is_target_point = checkPointsForCloud(std::make_shared<geometry_msgs::msg::Point>(target_points_[6]),std::make_shared<geometry_msgs::msg::Point>(target_points_[7]));
        if (is_target_point){
            geometry_msgs::msg::Pose::Ptr target_pose = calculateTargetPose(is_target_point, false);
            bool result = GoToPose(target_pose,1,2);  // Assuming GoToPose is defined elsewhere
            RCLCPP_INFO(this->get_logger(), "Navigated to pose: success = %s", result ? "true" : "false");
        }
    }

    void performSenevthAction() {
        geometry_msgs::msg::Pose::Ptr target_pose = calculateTargetPose(std::make_shared<geometry_msgs::msg::Point>(special_point_), true);
        GoToPose(target_pose,2,1);
    }

    void performEighthAction() {
        geometry_msgs::msg::Pose::Ptr target_pose = calculateTargetPose(std::make_shared<geometry_msgs::msg::Point>(target_points_set[2]), false);
        GoToPose(target_pose,3,2);

    }
    void performNinthAction() {
        geometry_msgs::msg::Point::Ptr is_target_point = checkPointsForCloud(std::make_shared<geometry_msgs::msg::Point>(target_points_[8]),std::make_shared<geometry_msgs::msg::Point>(target_points_[9]));
        if (is_target_point){
            geometry_msgs::msg::Pose::Ptr target_pose = calculateTargetPose(is_target_point, true);
            bool result = GoToPose(target_pose,1,2);  // Assuming GoToPose is defined elsewhere
            RCLCPP_INFO(this->get_logger(), "Navigated to pose: success = %s", result ? "true" : "false");
        }
    }
    void performTenthAction() {
        geometry_msgs::msg::Pose::Ptr target_pose = calculateTargetPose(std::make_shared<geometry_msgs::msg::Point>(target_points_set[3]), true);
        GoToPose(target_pose,3,2);

    }
    geometry_msgs::msg::Pose::Ptr calculateTargetPose(geometry_msgs::msg::Point::Ptr target, bool face_towards) {
        geometry_msgs::msg::Pose::Ptr pose;
        double dx = target->x - robot_position_.x;
        double dy = target->y - robot_position_.y;
        double angle = atan2(dy, dx);

        if (!face_towards) {
            angle += M_PI;  // Add π to face away
        }

        // Calculate the pose at distance r from the target
        pose->position.x = target->x - r * cos(angle);
        pose->position.y = target->y - r * sin(angle);
        pose->position.z = 0;  // Assuming flat terrain

        // Orientation as a quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, angle);
        pose->orientation.x = q.x();
        pose->orientation.y = q.y();
        pose->orientation.z = q.z();
        pose->orientation.w = q.w();

        return pose;
    }
    std::vector<geometry_msgs::msg::Point> findAndSortPointsInRegion(std::vector<geometry_msgs::msg::Point> points_) {
        std::vector<geometry_msgs::msg::Point> points_in_region;
        sensor_msgs::PointCloud2Iterator<float> iter_x(*last_received_cloud_, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*last_received_cloud_, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*last_received_cloud_, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;
            for (auto& target : points_) {
                if (isPointInRegion(x, y, target.x, target.y, 0.5)) {
                    // geometry_msgs::msg::Point pt{point.x, point.y, point.z};
                    points_in_region.push_back(target);
                }
            }
        }
        
        // std::sort(points_in_region.begin(), points_in_region.end(), 
        //           [this](const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
        //               return distanceToPoint(a) < distanceToPoint(b);
        //           });
        return points_in_region;
    }
    bool isPointInRegion(double x, double y, double center_x, double center_y, double threshold) {
        double dist = sqrt(pow(x - center_x, 2) + pow(y - center_y, 2));
        return dist <= threshold;
    }
    bool findPointsInRegion(geometry_msgs::msg::Point::Ptr points_){
        std::vector<geometry_msgs::msg::Point> points_in_region;
        sensor_msgs::PointCloud2Iterator<float> iter_x(*last_received_cloud_, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*last_received_cloud_, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*last_received_cloud_, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;
            
            if (isPointInRegion(x, y, points_->x, points_->y, 0.08)) {
                // geometry_msgs::msg::Point pt{point.x, point.y, point.z};
                return true;
            }
        }
    }
    double distanceToPoint(const geometry_msgs::msg::Point& point) {
        return sqrt(pow(point.x - robot_position_.x, 2) + pow(point.y - robot_position_.y, 2));
    }
    void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result,int & mode_1,int & mode_2)
    {
        auto gimble_message = hoist_msgs::msg::Gimble();
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                gimble_message.mod_1 = mode_1;
                gimble_message.mod_2 = mode_2;
                gimble_publisher_->publish(gimble_message);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
    }
    geometry_msgs::msg::Point::Ptr checkPointsForCloud(geometry_msgs::msg::Point::Ptr point1,geometry_msgs::msg::Point::Ptr point2) {
        bool found_in_point1 = findPointsInRegion(point1);
        bool found_in_point2 = findPointsInRegion(point2);

        if (found_in_point1 && found_in_point2) {
            RCLCPP_ERROR(this->get_logger(), "Both points have point cloud data.");
            return 0;
        } else if (found_in_point1) {
            RCLCPP_INFO(this->get_logger(), "Point 1 has point cloud data.");
            return point1;
        } else if (found_in_point2) {
            RCLCPP_INFO(this->get_logger(), "Point 2 has point cloud data.");
            return point2;
        } else {
            RCLCPP_INFO(this->get_logger(), "Neither point has point cloud data.");
            return 0;
        }
    }
    void goal_response_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal)
    {
        auto goal_handle = goal.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the action server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by the action server, waiting for result");
        }
    }
    template<typename T,typename Q>
    void feedback_callback(T,Q feedback) {
    }
    geometry_msgs::msg::Point create_point(double x, double y, double z = 0.0) {
        geometry_msgs::msg::Point point;
        point.x = x;
        point.y = y;
        point.z = z;
        return point;
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
    rclcpp::Publisher<hoist_msgs::msg::Gimble>::SharedPtr gimble_publisher_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client;
    std::vector<geometry_msgs::msg::Point> target_points_,target_points_set,sorted_target_points_,sorted_target_points_set;
    geometry_msgs::msg::Point robot_position_, special_point_;
    sensor_msgs::msg::PointCloud2::Ptr last_received_cloud_;
    std::string point_cloud_topic, odom_topic;
    float r;  // Distance to maintain from the target point
    int cycle_counter_ = 0;  // Counter for service calls
    bool special_is_remove = false;
    bool debug = false;
    int current_executing;
    rclcpp_action::ResultCode status;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<hoist_decision>());
    rclcpp::shutdown();
    return 0;
}
