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

enum class ActionStatus
{
  UNKNOWN = 0,
  PROCESSING = 1,
  FAILED = 2,
  SUCCEEDED = 3
};

using namespace std::chrono_literals;

class hoist_decision : public rclcpp::Node {
public:
    // using ActionT = nav2_msgs::action::FollowWaypoints;
    using ClientT = nav2_msgs::action::NavigateToPose;
    // using ActionServer = nav2_util::SimpleActionServer<ActionT>;
    using ActionClient = rclcpp_action::Client<ClientT>;
    hoist_decision() : Node("point_cloud_processor") {
        point_cloud_topic = this->declare_parameter<std::string>("point_cloud_topic", "/cloud");
        odom_topic = this->declare_parameter<std::string>("odom_topic", "/odometry/filtered");
        r = this->declare_parameter<float>("distance_to_target", 0.0); // Distance r from target

        point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            point_cloud_topic, rclcpp::SensorDataQoS(), std::bind(&hoist_decision::pointCloudCallback, this, std::placeholders::_1));

        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10, std::bind(&hoist_decision::odomCallback, this, std::placeholders::_1));

        gimble_publisher_ = this->create_publisher<hoist_msgs::msg::Gimble>("gimble_mode",4);
        // callback_group_ = create_callback_group(
        //     rclcpp::CallbackGroupType::MutuallyExclusive,
        //     false);
        // callback_group_executor_.add_callback_group(callback_group_, get_node_base_interface());
        service_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);;
        service_ = this->create_service<std_srvs::srv::Trigger>(
            "cycle_process_points", std::bind(&hoist_decision::cycleProcessPoints, this, std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default, service_cb_group_);
        callback_group_ = create_callback_group(
            rclcpp::CallbackGroupType::Reentrant,
            false);
        callback_group_executor_.add_callback_group(callback_group_, get_node_base_interface());
        nav_to_pose_client = rclcpp_action::create_client<ClientT>(
            get_node_base_interface(),
            get_node_graph_interface(),
            get_node_logging_interface(),
            get_node_waitables_interface(),
            "navigate_to_pose", callback_group_);
        target_points_.push_back(create_point(0.0, 0.450));
        target_points_.push_back(create_point(0.0, 0.825));
        target_points_.push_back(create_point(-1.29903, 0.825));
        target_points_.push_back(create_point(-0.64951, 1.0125));
        target_points_.push_back(create_point(-1.29903, 1.575));
        target_points_.push_back(create_point(-0.64951, 1.4875));
        target_points_.push_back(create_point(0.0, 1.950));
        target_points_.push_back(create_point(0.0, 1.575));
        target_points_.push_back(create_point(1.29903, 1.575));
        target_points_.push_back(create_point(0.64951, 1.4875));
        target_points_.push_back(create_point(1.29903, 0.925));
        target_points_.push_back(create_point(0.64951, 1.0125));
        special_point_ = create_point(0.0,1.2);  // Special target point
        target_points_set.push_back(create_point(0.755, -1.305));
        target_points_set.push_back(create_point(-0.755, -1.305));
        target_points_set.push_back(create_point(-0.755, 1.755));
        target_points_set.push_back(create_point(0.755, 1.755));
    }
    /* bool GoToPose(geometry_msgs::msg::Pose::SharedPtr pose,int mode_1,int mode_2)
    {
        using namespace std::placeholders;
        RCLCPP_INFO(this->get_logger(),"In GoToPose");
        nav_to_pose_client->wait_for_action_server();
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose.pose = *pose;
        goal_msg.pose.header.stamp = this->now();
        goal_msg.behavior_tree = "";
        rclcpp::WallRate r(loop_rate_);
        current_executing = GO_TO_POSE;
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        RCLCPP_INFO(this->get_logger(),"After Created send_goal_options");
        send_goal_options.goal_response_callback =
            std::bind(&hoist_decision::goal_response_callback, this, _1 );
        send_goal_options.result_callback =
            std::bind(&hoist_decision::resultCallback, this, _1,);
        RCLCPP_INFO(this->get_logger(),"Before async_send_goal");
        auto send_goal_future = nav_to_pose_client->async_send_goal(goal_msg,send_goal_options);
        RCLCPP_INFO(this->get_logger(),"After async_send_goal");
        // send_goal_future.wait_for(10s);
        if (callback_group_executor_.spin_until_future_complete(send_goal_future) ==
                rclcpp::FutureReturnCode::SUCCESS) {
                auto goal_handle = send_goal_future.get();
                // 处理结果
                if (goal_handle != NULL ) {
                    if ( goal_handle->get_status() != action_msgs::msg::GoalStatus::STATUS_ACCEPTED ) {
                    if (debug)
                    RCLCPP_INFO(this->get_logger(),"FollowWaypoints request was rejected");
                    current_executing = NONE;
                    return false;
                    }
                }
                else {
                    if (debug)
                    RCLCPP_INFO(this->get_logger(),"FollowWaypoints request was rejected");
                    current_executing = NONE;    
                    return false;
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to get result");
            }
        
        // callback_group_executor_.spin_until_future_complete(this->get_node_base_interface(),send_goal_future);
        // rclcpp::spin_until_future_complete(this->get_node_base_interface(),send_goal_future);
        // auto async_res_future = nav_to_pose_client->async_get_result(send_goal_future.get()); //async_result();
        RCLCPP_INFO(this->get_logger(),"After async_get_result");
        // async_res_future.wait_for(10s);
        // RCLCPP_INFO(this->get_logger(),"After spin_until_future_complete");
        // auto goal_handle = async_res_future.get();
        // if (goal_handle.code != rclcpp_action::ResultCode::SUCCEEDED)
        //     {
        //         RCLCPP_WARN(get_logger(), "Action did not succeed");
        //         return false;
        //     }
        if (debug)
        RCLCPP_INFO(this->get_logger(),"GoToPose request was accepted.");
        //future_go_to_pose = nav_to_pose_client->async_get_result(goal_handle);
        // callback_group_executor_.spin_some();
        // r.sleep();
        return true;
    } */
private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        last_received_cloud_ = msg;
        RCLCPP_INFO_ONCE(this->get_logger(), "Received and stored new point cloud data.");
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_position_ = msg->pose.pose.position;
        //RCLCPP_INFO(this->get_logger(), "Updated robot position to x: %f, y: %f, z: %f", robot_position_.x, robot_position_.y, robot_position_.z);
    }

    void cycleProcessPoints(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        //sorted_target_points_ = findAndSortPointsInRegion(target_points_);
        RCLCPP_INFO(this->get_logger(), "seriver is called");
        // std::sort(target_points_set.begin(), target_points_set.end(), 
        //           [this](const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
        //               return distanceToPoint(a) < distanceToPoint(b);
        //           });
        
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
        response->success = (current_goal_status_ != ActionStatus::SUCCEEDED);
        // return true;
    }
    void performFirstAction() {
        // geometry_msgs::msg::Point::Ptr is_target_point = checkPointsForCloud(std::make_shared<geometry_msgs::msg::Point>(target_points_[0]),std::make_shared<geometry_msgs::msg::Point>(target_points_[1]));
        //if (is_target_point){
        RCLCPP_INFO_STREAM(this->get_logger(),"performFirstAction is called"<<target_points_.size());
        geometry_msgs::msg::Point::Ptr is_target_point = std::make_shared<geometry_msgs::msg::Point>(target_points_[1]);
        RCLCPP_INFO_STREAM(this->get_logger(),"target_pose Succeed"<<is_target_point);
        geometry_msgs::msg::Pose::Ptr target_pose = calculateTargetPose(is_target_point, true);
        RCLCPP_INFO_STREAM(this->get_logger(),"target_pose Succeed"<<target_pose);
        followWaypoints(target_pose,1,1);  // Assuming GoToPose is defined elsewhere
        // RCLCPP_INFO(this->get_logger(), "Navigated to pose: success = %s", result ? "true" : "false");
        // target_points_.erase(target_points_.begin());  // Removing the first point for now
        //}
    }

    void performSecondAction() {
        // geometry_msgs::msg::Point::Ptr is_target_point = checkPointsForCloud(std::make_shared<geometry_msgs::msg::Point>(target_points_[2]),std::make_shared<geometry_msgs::msg::Point>(target_points_[3]));
        RCLCPP_INFO(this->get_logger(), "performSecondAction is called");
        geometry_msgs::msg::Point::Ptr is_target_point = std::make_shared<geometry_msgs::msg::Point>(target_points_[2]);
        if (is_target_point){
            geometry_msgs::msg::Pose::Ptr target_pose = calculateTargetPose(is_target_point, false);
            followWaypoints(target_pose,1,2);  // Assuming GoToPose is defined elsewhere
            // RCLCPP_INFO(this->get_logger(), "Navigated to pose: success = %s", result ? "true" : "false");
            // target_points_.erase(target_points_.begin());  // Removing the first point for now
        }
    }

    void performThirdAction() {
        geometry_msgs::msg::Pose::Ptr target_pose = calculateTargetPose(std::make_shared<geometry_msgs::msg::Point>(target_points_set[0]), true);
        followWaypoints(target_pose,3,1);
    }

    void performFourthAction() {
        // Similar to performThirdAction but for back-facing poses
        geometry_msgs::msg::Pose::Ptr target_pose = calculateTargetPose(std::make_shared<geometry_msgs::msg::Point>(target_points_set[1]), true);
        followWaypoints(target_pose,3,2);

    }
    void performFifthAction() {
        geometry_msgs::msg::Point::Ptr is_target_point = checkPointsForCloud(std::make_shared<geometry_msgs::msg::Point>(target_points_[4]),std::make_shared<geometry_msgs::msg::Point>(target_points_[5]));
        if (is_target_point){
            geometry_msgs::msg::Pose::Ptr target_pose = calculateTargetPose(is_target_point, true);
            followWaypoints(target_pose,1,1);  // Assuming GoToPose is defined elsewhere
            // RCLCPP_INFO(this->get_logger(), "Navigated to pose: success = %s", result ? "true" : "false");
        }
    }

    void performSixthAction() {
        geometry_msgs::msg::Point::Ptr is_target_point = checkPointsForCloud(std::make_shared<geometry_msgs::msg::Point>(target_points_[6]),std::make_shared<geometry_msgs::msg::Point>(target_points_[7]));
        if (is_target_point){
            geometry_msgs::msg::Pose::Ptr target_pose = calculateTargetPose(is_target_point, false);
            followWaypoints(target_pose,1,2);  // Assuming GoToPose is defined elsewhere
            // RCLCPP_INFO(this->get_logger(), "Navigated to pose: success = %s", result ? "true" : "false");
        }
    }

    void performSenevthAction() {
        geometry_msgs::msg::Pose::Ptr target_pose = calculateTargetPose(std::make_shared<geometry_msgs::msg::Point>(special_point_), true);
        followWaypoints(target_pose,2,1);
    }

    void performEighthAction() {
        geometry_msgs::msg::Pose::Ptr target_pose = calculateTargetPose(std::make_shared<geometry_msgs::msg::Point>(target_points_set[2]), false);
        followWaypoints(target_pose,3,2);

    }
    void performNinthAction() {
        geometry_msgs::msg::Point::Ptr is_target_point = checkPointsForCloud(std::make_shared<geometry_msgs::msg::Point>(target_points_[8]),std::make_shared<geometry_msgs::msg::Point>(target_points_[9]));
        if (is_target_point){
            geometry_msgs::msg::Pose::Ptr target_pose = calculateTargetPose(is_target_point, true);
            followWaypoints(target_pose,1,2);  // Assuming GoToPose is defined elsewhere
            // RCLCPP_INFO(this->get_logger(), "Navigated to pose: success = %s", result ? "true" : "false");
        }
    }
    void performTenthAction() {
        geometry_msgs::msg::Pose::Ptr target_pose = calculateTargetPose(std::make_shared<geometry_msgs::msg::Point>(target_points_set[3]), true);
        followWaypoints(target_pose,3,2);

    }
    // geometry_msgs::msg::Pose::Ptr calculateTargetPose(geometry_msgs::msg::Point::Ptr target, bool face_towards) {
    //     auto pose = std::make_shared<geometry_msgs::msg::Pose>();
    //     double dx = target->x - robot_position_.x;
    //     double dy = target->y - robot_position_.y;
    //     double angle = atan2(dy, dx);

    //     if (!face_towards) {
    //         angle += M_PI;  // Add π to face away
    //     }

    //     // Calculate the pose at distance r from the target
    //     pose->position.x = target->x - r * cos(angle);
    //     pose->position.y = target->y - r * sin(angle);
    //     pose->position.z = 0;  // Assuming flat terrain

    //     // Orientation as a quaternion
    //     tf2::Quaternion q;
    //     q.setRPY(0, 0, angle);
    //     pose->orientation.x = q.x();
    //     pose->orientation.y = q.y();
    //     pose->orientation.z = q.z();
    //     pose->orientation.w = q.w();

    //     return pose;
    // }
    geometry_msgs::msg::Pose::Ptr calculateTargetPose(geometry_msgs::msg::Point::Ptr target, bool face_towards) {
        auto pose = std::make_shared<geometry_msgs::msg::Pose>();
        double angle = 3.1415926;

        if (!face_towards) {
            angle += M_PI;  // Add π to face away
        }

        // Calculate the pose at distance r from the target
        pose->position.x = target->x;
        pose->position.y = target->y - r;
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
    void resultCallback(
    const rclcpp_action::ClientGoalHandle<ClientT>::WrappedResult & result)
    {
    if (result.goal_id != future_goal_handle_.get()->get_goal_id()) {
        RCLCPP_DEBUG(
        get_logger(),
        "Goal IDs do not match for the current goal handle and received result."
        "Ignoring likely due to receiving result for an old goal.");
        return;
    }

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
        current_goal_status_ = ActionStatus::SUCCEEDED;
        return;
        case rclcpp_action::ResultCode::ABORTED:
        current_goal_status_ = ActionStatus::FAILED;
        return;
        case rclcpp_action::ResultCode::CANCELED:
        current_goal_status_ = ActionStatus::FAILED;
        return;
        default:
        current_goal_status_ = ActionStatus::UNKNOWN;
        return;
    }
    }

    void followWaypoints(geometry_msgs::msg::Pose::SharedPtr pose,int mode_1,int mode_2)
    {
        // auto goal = action_server_->get_current_goal();
        // auto feedback = std::make_shared<ActionT::Feedback>();
        // auto result = std::make_shared<ActionT::Result>();

        rclcpp::WallRate r(loop_rate_);
        bool new_goal = true;

        while (rclcpp::ok()) {
            if (new_goal) {
                new_goal = false;
                ClientT::Goal client_goal;
                client_goal.pose.pose = *pose;

                auto send_goal_options = rclcpp_action::Client<ClientT>::SendGoalOptions();
                send_goal_options.result_callback =
                    std::bind(&hoist_decision::resultCallback, this, std::placeholders::_1);
                send_goal_options.goal_response_callback =
                    std::bind(&hoist_decision::goal_response_callback, this, std::placeholders::_1);
                future_goal_handle_ =
                    nav_to_pose_client->async_send_goal(client_goal, send_goal_options);
                current_goal_status_ = ActionStatus::PROCESSING;
            }

            if (current_goal_status_ == ActionStatus::FAILED) {
                RCLCPP_WARN(
                get_logger(), "Failed to process waypoint in waypoint "
                "list and stop on failure is enabled."
                " Terminating action.");
                return;
            } else if (current_goal_status_ == ActionStatus::SUCCEEDED) {
            RCLCPP_INFO(
                get_logger(), "Succeeded processing waypoint, processing waypoint task execution");
            bool is_task_executed =processAtWaypoint(mode_1,mode_2);
            RCLCPP_INFO(
                get_logger(), "Task execution at waypoint%s",
                is_task_executed ? "succeeded" : "failed!");
            // if task execution was failed and stop_on_failure_ is on , terminate action
            }
            if (current_goal_status_ != ActionStatus::PROCESSING &&
            current_goal_status_ != ActionStatus::UNKNOWN)
            {
            // Update server state
            new_goal = true;
            RCLCPP_INFO_EXPRESSION(
                get_logger(),
                (static_cast<int>(now().seconds()) % 30 == 0),
                "Processing waypoint ...");
            }

            callback_group_executor_.spin_some();
            r.sleep();
        }
    }
    void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result,int & mode_1,int & mode_2)
    {
        auto gimble_message = hoist_msgs::msg::Gimble();
        RCLCPP_INFO_STREAM(this->get_logger(),"Result callbacked");
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                // gimble_message.mod_1 = mode_1;
                // gimble_message.mod_2 = mode_2;
                // gimble_publisher_->publish(gimble_message);
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
    bool processAtWaypoint(int mode_1,int mode_2){
        auto msg = hoist_msgs::msg::Gimble();
        msg.mod_1 = mode_1;
        msg.mod_2 = mode_2;
        gimble_publisher_->publish(msg);
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
    rclcpp::Publisher<hoist_msgs::msg::Gimble>::SharedPtr gimble_publisher_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client;
    ActionStatus current_goal_status_;
    std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult> future_go_to_pose;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::MultiThreadedExecutor callback_group_executor_;
    rclcpp::CallbackGroup::SharedPtr service_cb_group_;
    std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr> future_goal_handle_;
    std::vector<geometry_msgs::msg::Point> target_points_,target_points_set,sorted_target_points_,sorted_target_points_set;
    geometry_msgs::msg::Point robot_position_, special_point_;
    sensor_msgs::msg::PointCloud2::Ptr last_received_cloud_;
    std::string point_cloud_topic, odom_topic;
    int loop_rate_ = 20;
    float r;  // Distance to maintain from the target point
    int cycle_counter_ = 0;  // Counter for service calls
    bool special_is_remove = false;
    bool debug = true;
    bool stop_on_failure_ = false;
    int current_executing;
    rclcpp_action::ResultCode status;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<hoist_decision>();
    // rclcpp::executors::MultiThreadedExecutor executor;
    // executor.add_node(node);
    // executor.spin();
    // executor.remove_node(node);
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
