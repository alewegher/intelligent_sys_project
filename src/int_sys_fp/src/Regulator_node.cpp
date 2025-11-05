#include <rclcpp/rclcpp.hpp>
#include<iostream>
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "UWB_utils_emulator.hpp"
#include <deque>
#include "int_sys_fp/msg/anchor_dist.hpp"
#include "int_sys_fp/msg/robot_dist.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

# define PI 3.14159265358979323846

using namespace Eigen;


class ControllerClass : public rclcpp::Node{

    public:

        ControllerClass() : Node("controller_node")
        {
            // Create publishers for TurtleBot3 cmd_vel topics
            cmd_vel_robot1_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            cmd_vel_robot2_pub = this->create_publisher<geometry_msgs::msg::Twist>("/tb3_2/cmd_vel", 10);
            cmd_vel_robot3_pub = this->create_publisher<geometry_msgs::msg::Twist>("/tb3_3/cmd_vel", 10);

            // Create publishers for debug/monitoring
            kf_filtered_robot_pose_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("kf_filtered_robot_pose", 10);
            tracking_error_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("tracking_error", 10);
            position_comparison_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("position_comparison", 10);
            desired_trajectory_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("desired_trajectory_array", 10);

            // Create subscribers for filtered UWB data
            uwb_anchor_sub = this->create_subscription<int_sys_fp::msg::AnchorDist>(
                "/uwb/filtered_anchor_distances", 10,
                std::bind(&ControllerClass::UWB_read_anchor_callback, this, std::placeholders::_1));
            
            uwb_robot_sub = this->create_subscription<int_sys_fp::msg::RobotDist>(
                "/uwb/filtered_robot_distances", 10,
                std::bind(&ControllerClass::UWB_robot_callback, this, std::placeholders::_1));

            // Subscriber for desired trajectory from planner
            trajectory_sub = this->create_subscription<geometry_msgs::msg::Pose>(
                "/desired_trajectory", 10,
                std::bind(&ControllerClass::trajectory_callback, this, std::placeholders::_1));
            
            // Subscribers for ground truth odometry from Gazebo (for debugging)
            odom_robot1_sub = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odom", 10,
                std::bind(&ControllerClass::odom_robot1_callback, this, std::placeholders::_1));
            
            odom_robot2_sub = this->create_subscription<nav_msgs::msg::Odometry>(
                "/tb3_2/odom", 10,
                std::bind(&ControllerClass::odom_robot2_callback, this, std::placeholders::_1));
            
            odom_robot3_sub = this->create_subscription<nav_msgs::msg::Odometry>(
                "/tb3_3/odom", 10,
                std::bind(&ControllerClass::odom_robot3_callback, this, std::placeholders::_1));

            // Initialize robot states
            robot_positions.resize(3, Eigen::Vector2d::Zero());
            robot_velocities.resize(3, Eigen::Vector2d::Zero());
            inter_robot_distances.resize(3, Eigen::Vector2d::Zero());
            gazebo_positions.resize(3, Eigen::Vector2d::Zero());
            
            // Initialize anchor positions from sensor config
            load_anchor_positions();
            
            // Load controller parameters
            load_controller_params();
            
            // Initialize desired trajectory (default at origin)
            desired_position = Eigen::Vector2d::Zero();
            desired_velocity = Eigen::Vector2d::Zero();
            
            // Desired inter-robot distance for equilateral triangle
            desired_distance = 1.0; // meters
            
            // Create control timer (50 Hz)
            control_timer = this->create_wall_timer(
                std::chrono::milliseconds(20),
                std::bind(&ControllerClass::control_loop, this));
            
            RCLCPP_INFO(this->get_logger(), "Controller node initialized");
            RCLCPP_INFO(this->get_logger(), "Anchor positions loaded: 3 anchors");
            RCLCPP_INFO(this->get_logger(), "Controller gains loaded from config");
        }

        void load_anchor_positions() {
            // Load anchor positions from sensor_params.yaml
            try {
                std::string package_share = ament_index_cpp::get_package_share_directory("int_sys_fp");
                std::string yaml_path = package_share + "/sensor_params.yaml";
                YAML::Node config = YAML::LoadFile(yaml_path);
                
                auto uwb_config = config["UWB_sensor"];
                anchor_positions.resize(3);
                
                for(int i = 0; i < 3; i++) {
                    auto anchor_node = uwb_config["anchor " + std::to_string(i+1)]["position"];
                    anchor_positions[i] = Eigen::Vector3d(
                        anchor_node[0].as<double>(),
                        anchor_node[1].as<double>(),
                        anchor_node[2].as<double>()
                    );
                }
                
                RCLCPP_INFO(this->get_logger(), "Anchors: [%.1f,%.1f] [%.1f,%.1f] [%.1f,%.1f]",
                    anchor_positions[0].x(), anchor_positions[0].y(),
                    anchor_positions[1].x(), anchor_positions[1].y(),
                    anchor_positions[2].x(), anchor_positions[2].y());
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load anchors: %s", e.what());
                // Default positions
                anchor_positions.resize(3);
                anchor_positions[0] = Eigen::Vector3d(0.0, 0.0, 0.0);
                anchor_positions[1] = Eigen::Vector3d(10.0, 0.0, 0.0);
                anchor_positions[2] = Eigen::Vector3d(0.0, 10.0, 0.0);
            }
        }

        void load_controller_params() {
            try {
                std::string package_share = ament_index_cpp::get_package_share_directory("int_sys_fp");
                std::string yaml_path = package_share + "/controller.yaml";
                YAML::Node config = YAML::LoadFile(yaml_path);
                
                auto ctrl_params = config["controller_params"];
                
                // Load position gains (Kp)
                auto kp_r1 = ctrl_params["position_gains"]["Kp_r1"];
                Kp_centroid = Eigen::Vector2d(kp_r1[0].as<double>(), kp_r1[1].as<double>());
                
                // Load velocity gains (Kv)
                auto kv_r1 = ctrl_params["velocity_gains"]["Kv_r1"];
                Kv_centroid = Eigen::Vector2d(kv_r1[0].as<double>(), kv_r1[1].as<double>());
                
                // Load max velocities
                auto max_vel = ctrl_params["max_velocities"]["max_vel_r"];
                max_linear_vel = max_vel[0].as<double>();
                
                auto max_omega = ctrl_params["max_velocities"]["max_omega_r"];
                max_angular_vel = max_omega[0].as<double>();
                
                // Formation control gain
                Kf = 0.5; // Formation gain
                
                RCLCPP_INFO(this->get_logger(), "Controller gains: Kp=[%.2f,%.2f] Kv=[%.2f,%.2f] Kf=%.2f",
                    Kp_centroid.x(), Kp_centroid.y(), Kv_centroid.x(), Kv_centroid.y(), Kf);
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to load controller params: %s, using defaults", e.what());
                Kp_centroid = Eigen::Vector2d(0.6, 0.6);
                Kv_centroid = Eigen::Vector2d(0.3, 0.3);
                Kf = 0.5;
                max_linear_vel = 0.22;
                max_angular_vel = 2.84;
            }
        }

        void trajectory_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
            desired_position = Eigen::Vector2d(msg->position.x, msg->position.y);
            
            // Republish as Float64MultiArray for easy plotting
            auto traj_msg = std_msgs::msg::Float64MultiArray();
            traj_msg.data = {desired_position.x(), desired_position.y()};
            desired_trajectory_pub->publish(traj_msg);
            
            RCLCPP_DEBUG(this->get_logger(), "New desired position: [%.2f, %.2f]", 
                desired_position.x(), desired_position.y());
        }

        Eigen::Vector3d solve_triangle(double l1,double l2,double l3){
            
            // initialize the result vector
            Eigen::Vector3d angles = Eigen::Vector3d::Zero();
            
            // check validity of triangle

            if (l1 + l2 <= l3 || l1 + l3 <= l2 || l2 + l3 <= l1) {
                RCLCPP_ERROR(this->get_logger(), "Invalid triangle sides: %f, %f, %f", l1, l2, l3);
                return angles; // return zero angles for invalid triangle
            }

            
            // compute the angles using the law of cosines

            angles[0] = std::acos((std::pow(l3, 2) + std::pow(l1, 2) - std::pow(l2, 2)) / (2 * l3 * l1));
            angles[1] = std::acos((std::pow(l1, 2) + std::pow(l2, 2) - std::pow(l3, 2)) / (2 * l1 * l2));
            angles[2] = PI - angles[0] - angles[1]; // third angle is 180 degrees minus the sum of the other two

            return angles;
        }

        void trilaterate_robot_position(int robot_idx, double d1, double d2, double d3) {
            // Simple 2D trilateration using three anchor distances
            // Anchors: A1=(x1,y1), A2=(x2,y2), A3=(x3,y3)
            // Robot at (x,y) with distances d1, d2, d3
            
            Eigen::Vector2d p1(anchor_positions[0].x(), anchor_positions[0].y());
            Eigen::Vector2d p2(anchor_positions[1].x(), anchor_positions[1].y());
            Eigen::Vector2d p3(anchor_positions[2].x(), anchor_positions[2].y());
            
            // Using standard trilateration algorithm
            double A = 2 * (p2.x() - p1.x());
            double B = 2 * (p2.y() - p1.y());
            double C = d1*d1 - d2*d2 - p1.x()*p1.x() + p2.x()*p2.x() - p1.y()*p1.y() + p2.y()*p2.y();
            
            double D = 2 * (p3.x() - p2.x());
            double E = 2 * (p3.y() - p2.y());
            double F = d2*d2 - d3*d3 - p2.x()*p2.x() + p3.x()*p3.x() - p2.y()*p2.y() + p3.y()*p3.y();
            
            double denom = A*E - B*D;
            if (std::abs(denom) < 1e-6) {
                RCLCPP_WARN(this->get_logger(), "Trilateration singular matrix for robot %d", robot_idx);
                return;
            }
            
            double x = (C*E - F*B) / denom;
            double y = (A*F - D*C) / denom;
            
            robot_positions[robot_idx] = Eigen::Vector2d(x, y);
            
            RCLCPP_DEBUG(this->get_logger(), "Robot %d estimated at [%.2f, %.2f] from distances [%.2f,%.2f,%.2f]",
                robot_idx, x, y, d1, d2, d3);
        }

        Eigen::Vector2d compute_centroid() {
            Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
            for(const auto& pos : robot_positions) {
                centroid += pos;
            }
            centroid /= 3.0;
            return centroid;
        }

        Eigen::Vector2d compute_centroid_tracking_control(const Eigen::Vector2d& centroid) {
            // PD controller for centroid tracking
            Eigen::Vector2d position_error = desired_position - centroid;
            Eigen::Vector2d velocity_error = desired_velocity - Eigen::Vector2d::Zero(); // assuming stationary target
            
            Eigen::Vector2d control = Kp_centroid.cwiseProduct(position_error) + 
                                     Kv_centroid.cwiseProduct(velocity_error);
            
            return control;
        }

        Eigen::Vector2d compute_formation_control(int robot_idx) {
            // Formation control to maintain equilateral triangle
            // Force to maintain equal distances between all robots
            Eigen::Vector2d formation_force = Eigen::Vector2d::Zero();
            
            for(int j = 0; j < 3; j++) {
                if(j == robot_idx) continue;
                
                Eigen::Vector2d diff = robot_positions[robot_idx] - robot_positions[j];
                double current_dist = diff.norm();
                
                if(current_dist < 1e-3) continue; // Avoid division by zero
                
                // Spring-like force: pushes apart if too close, pulls together if too far
                double error = current_dist - desired_distance;
                Eigen::Vector2d direction = diff / current_dist;
                
                formation_force += -Kf * error * direction; // negative to correct the error
            }
            
            return formation_force;
        }

        void control_loop() {
            // Main control loop called at 50 Hz
            
            // Check if we have valid position estimates
            bool valid_data = true;
            for(const auto& pos : robot_positions) {
                if(pos.norm() < 1e-6) {
                    valid_data = false;
                    break;
                }
            }
            
            if(!valid_data) {
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                    "Waiting for valid robot position estimates...");
                return;
            }
            
            // Compute centroid
            Eigen::Vector2d centroid = compute_centroid();
            
            // Compute centroid tracking control
            Eigen::Vector2d centroid_control = compute_centroid_tracking_control(centroid);
            
            // Publish tracking error (desired vs estimated centroid)
            publish_tracking_error();
            
            // Publish position comparison (Gazebo ground truth vs trilateration)
            publish_position_comparison();
            
            // Compute and publish velocity commands for each robot
            for(int i = 0; i < 3; i++) {
                // Formation control
                Eigen::Vector2d formation_control = compute_formation_control(i);
                
                // Total control = centroid tracking + formation maintenance
                Eigen::Vector2d total_control = centroid_control + formation_control;
                
                // Saturate velocity
                double vel_norm = total_control.norm();
                if(vel_norm > max_linear_vel) {
                    total_control = total_control / vel_norm * max_linear_vel;
                }
                
                // Create Twist message
                auto cmd_msg = geometry_msgs::msg::Twist();
                cmd_msg.linear.x = total_control.x();
                cmd_msg.linear.y = total_control.y();
                cmd_msg.angular.z = 0.0; // Assuming holonomic control or simplified model
                
                // Publish to appropriate robot
                if(i == 0) cmd_vel_robot1_pub->publish(cmd_msg);
                else if(i == 1) cmd_vel_robot2_pub->publish(cmd_msg);
                else if(i == 2) cmd_vel_robot3_pub->publish(cmd_msg);
                
                RCLCPP_DEBUG(this->get_logger(), "Robot %d cmd: [%.3f, %.3f]", 
                    i, total_control.x(), total_control.y());
            }
        }

        double mid(double a, double b){
            return (a+b)/2.0;
        }

        Eigen::Vector2d apply_rotation(const Eigen::Vector2d & vec ,Eigen::Rotation2Dd & rotation){
            return rotation * vec;
        }

        void UWB_read_anchor_callback(const int_sys_fp::msg::AnchorDist::SharedPtr msg){
            // Callback per dati UWB anchor distances filtrati dal Kalman Filter
            // msg contiene: distances_a1, distances_a2, distances_a3
            // ogni array contiene le distanze di tutti i robot a quell'anchor
            
            if(msg->distances_a1.size() != 3 || msg->distances_a2.size() != 3 || msg->distances_a3.size() != 3) {
                RCLCPP_WARN(this->get_logger(), "Expected 3 distances per anchor, got a1:%zu a2:%zu a3:%zu", 
                           msg->distances_a1.size(), msg->distances_a2.size(), msg->distances_a3.size());
                return;
            }
            
            // Trilateration per ogni robot usando le distanze filtrate
            for(size_t robot_idx = 0; robot_idx < 3; robot_idx++) {
                double d1 = msg->distances_a1[robot_idx]; // Distance to anchor 1
                double d2 = msg->distances_a2[robot_idx]; // Distance to anchor 2  
                double d3 = msg->distances_a3[robot_idx]; // Distance to anchor 3
                
                // Skip if out of range (marked as -1.0)
                if(d1 < 0 || d2 < 0 || d3 < 0) {
                    RCLCPP_DEBUG(this->get_logger(), "Robot %d has out-of-range distances, skipping", robot_idx);
                    continue;
                }
                
                // Perform trilateration
                trilaterate_robot_position(robot_idx, d1, d2, d3);
            }
            
            // Publish estimated robot positions for monitoring
            auto robot_pose_msg = std_msgs::msg::Float64MultiArray();
            robot_pose_msg.data.clear();
            for(const auto& pos : robot_positions) {
                robot_pose_msg.data.push_back(pos.x());
                robot_pose_msg.data.push_back(pos.y());
            }
            kf_filtered_robot_pose_pub->publish(robot_pose_msg);
        }

        void UWB_robot_callback(const int_sys_fp::msg::RobotDist::SharedPtr msg){
            // Callback per dati UWB robot-to-robot distances filtrati
            // msg contiene: distances_r1, distances_r2, distances_r3
            // ogni array contiene le distanze di quel robot verso gli altri
            
            if(msg->distances_r1.size() != 2 || msg->distances_r2.size() != 2 || msg->distances_r3.size() != 2) {
                RCLCPP_WARN(this->get_logger(), "Expected 2 distances per robot, got r1:%zu r2:%zu r3:%zu", 
                           msg->distances_r1.size(), msg->distances_r2.size(), msg->distances_r3.size());
                return;
            }
            
            // Store inter-robot distances for formation control
            // Robot 1 sees: [dist to R2, dist to R3]
            // Robot 2 sees: [dist to R1, dist to R3]
            // Robot 3 sees: [dist to R1, dist to R2]
            inter_robot_distances[0] = Eigen::Vector2d(msg->distances_r1[0], msg->distances_r1[1]);
            inter_robot_distances[1] = Eigen::Vector2d(msg->distances_r2[0], msg->distances_r2[1]);
            inter_robot_distances[2] = Eigen::Vector2d(msg->distances_r3[0], msg->distances_r3[1]);
            
            // Log average distance for monitoring equilateral formation
            double avg_dist = (msg->distances_r1[0] + msg->distances_r1[1] + 
                              msg->distances_r2[0] + msg->distances_r2[1] +
                              msg->distances_r3[0] + msg->distances_r3[1]) / 6.0;
            
            RCLCPP_DEBUG(this->get_logger(), "Inter-robot distances avg: %.3f m (desired: %.3f m)", 
                avg_dist, desired_distance);
        }

        // Odometry callbacks for ground truth from Gazebo (for debugging)
        void odom_robot1_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            gazebo_positions[0] = Eigen::Vector2d(msg->pose.pose.position.x, msg->pose.pose.position.y);
        }
        
        void odom_robot2_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            gazebo_positions[1] = Eigen::Vector2d(msg->pose.pose.position.x, msg->pose.pose.position.y);
        }
        
        void odom_robot3_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            gazebo_positions[2] = Eigen::Vector2d(msg->pose.pose.position.x, msg->pose.pose.position.y);
        }

        // Publish tracking error (desired vs estimated centroid)
        void publish_tracking_error() {
            // Calculate estimated centroid from trilateration
            Eigen::Vector2d estimated_centroid = (robot_positions[0] + robot_positions[1] + robot_positions[2]) / 3.0;
            
            // Calculate error (desired - estimated)
            Eigen::Vector2d position_error = desired_position - estimated_centroid;
            
            // Publish only error vector (2 values: x, y)
            auto msg = std_msgs::msg::Float64MultiArray();
            msg.data = {position_error.x(), position_error.y()};
            tracking_error_pub->publish(msg);
            
            RCLCPP_DEBUG(this->get_logger(), "Tracking error: [%.3f, %.3f]m (norm: %.3f m)",
                position_error.x(), position_error.y(), position_error.norm());
        }
        
        // Publish position comparison (Gazebo ground truth vs trilateration estimate)
        void publish_position_comparison() {
            auto msg = std_msgs::msg::Float64MultiArray();
            
            // Format: [gazebo_r1_x, gazebo_r1_y, trilat_r1_x, trilat_r1_y, error_r1,
            //          gazebo_r2_x, gazebo_r2_y, trilat_r2_x, trilat_r2_y, error_r2,
            //          gazebo_r3_x, gazebo_r3_y, trilat_r3_x, trilat_r3_y, error_r3]
            msg.data.resize(15);
            
            for(int i = 0; i < 3; i++) {
                msg.data[i*5 + 0] = gazebo_positions[i].x();
                msg.data[i*5 + 1] = gazebo_positions[i].y();
                msg.data[i*5 + 2] = robot_positions[i].x();
                msg.data[i*5 + 3] = robot_positions[i].y();
                
                // Calculate error
                Eigen::Vector2d error = gazebo_positions[i] - robot_positions[i];
                msg.data[i*5 + 4] = error.norm();
            }
            
            position_comparison_pub->publish(msg);
            
            // Log
            double avg_error = (msg.data[4] + msg.data[9] + msg.data[14]) / 3.0;
            RCLCPP_INFO(this->get_logger(), "📍 Position errors (Gazebo vs Trilat): R1:%.3fm R2:%.3fm R3:%.3fm (avg:%.3fm)",
                msg.data[4], msg.data[9], msg.data[14], avg_error);
        }




    private:

        // Publishers for robot cmd_vel
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_robot1_pub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_robot2_pub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_robot3_pub;

        // Publishers for monitoring
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr kf_filtered_robot_pose_pub;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tracking_error_pub;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_comparison_pub;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr desired_trajectory_pub;

        // Subscribers for UWB filtered data
        rclcpp::Subscription<int_sys_fp::msg::AnchorDist>::SharedPtr uwb_anchor_sub;
        rclcpp::Subscription<int_sys_fp::msg::RobotDist>::SharedPtr uwb_robot_sub;
        
        // Subscriber for desired trajectory from planner
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr trajectory_sub;
        
        // Subscribers for ground truth from Gazebo (for debugging)
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_robot1_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_robot2_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_robot3_sub;

        // Control timer (50 Hz)
        rclcpp::TimerBase::SharedPtr control_timer;

        // Robot states (2D for simplicity)
        std::vector<Eigen::Vector2d> robot_positions;        // Estimated from trilateration
        std::vector<Eigen::Vector2d> robot_velocities;       // Could be estimated or measured
        std::vector<Eigen::Vector2d> inter_robot_distances;  // Distances between robots
        
        // Ground truth positions from Gazebo odometry (for debugging)
        std::vector<Eigen::Vector2d> gazebo_positions;       // True positions from Gazebo
        
        // Anchor positions (loaded from config)
        std::vector<Eigen::Vector3d> anchor_positions;
        
        // Desired trajectory from planner
        Eigen::Vector2d desired_position;
        Eigen::Vector2d desired_velocity;
        
        // Formation control parameters
        double desired_distance;  // Desired inter-robot distance for equilateral triangle
        
        // Controller gains
        Eigen::Vector2d Kp_centroid;  // Position gain for centroid tracking
        Eigen::Vector2d Kv_centroid;  // Velocity gain for centroid tracking
        double Kf;                     // Formation control gain
        
        // Velocity limits
        double max_linear_vel;
        double max_angular_vel;


};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerClass>());
    rclcpp::shutdown();
    return 0;
}