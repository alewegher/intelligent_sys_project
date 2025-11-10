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
            desired_trajectory_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("desired_trajectory_array", 10);
            centroid_position_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("centroid_position", 10);

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
            
            // Subscribers for robot odometry (to get orientation)
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
            robot_orientations.resize(3, 0.0);  // Yaw angles in radians
            previous_heading_errors.resize(3, 0.0);  // Previous heading errors for derivative
            
            // Initialize PID integral state
            integral_error = Eigen::Vector2d::Zero();
            
            // Initialize anchor positions from sensor config
            load_anchor_positions();
            
            // Load controller parameters (returns control frequency from YAML)
            double control_frequency = load_controller_params();
            
            // Calculate control period from frequency
            control_dt = 1.0 / control_frequency;
            
            // Initialize desired trajectory (default at origin)
            desired_position = Eigen::Vector2d::Zero();
            desired_velocity = Eigen::Vector2d::Zero();
            
            // Desired inter-robot distance for equilateral triangle
            desired_distance = 1.5; // meters
            
            // Formation control activation threshold
            tracking_error_threshold = 0.2; // meters - activate formation only when tracking error < threshold
            
            // Formation control gain
            Kf = 0.8; // Formation maintenance gain during tracking
            
            // Initial formation phase: stiff PD controller (critically damped)
            Kp_formation = 2.0;  // Stiff proportional gain
            Kd_formation = 2.0 * 0.98 * std::sqrt(Kp_formation);  // Critically damped: Kd = 2*ζ*sqrt(Kp), ζ=0.98
            
            // Formation convergence threshold (norm of distance errors)
            formation_convergence_threshold = 0.15; // meters - switch to tracking when formation error < 15cm
            
            // Start in FORMATION phase
            current_phase = ControlPhase::FORMATION;
            
            RCLCPP_INFO(this->get_logger(), "=== CONTROL STATE MACHINE ===");
            RCLCPP_INFO(this->get_logger(), "Initial phase: FORMATION (stiff PD control)");
            RCLCPP_INFO(this->get_logger(), "Formation gains: Kp=%.2f Kd=%.2f (critically damped)", Kp_formation, Kd_formation);
            RCLCPP_INFO(this->get_logger(), "Convergence threshold: %.3f m", formation_convergence_threshold);
            RCLCPP_INFO(this->get_logger(), "Will switch to TRACKING phase when formation achieved");
            
            // Create control timer with frequency from YAML
            control_timer = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency)),
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

        double load_controller_params() {
            double control_frequency = 50.0; // Default frequency
            
            try {
                std::string package_share = ament_index_cpp::get_package_share_directory("int_sys_fp");
                std::string yaml_path = package_share + "/controller.yaml";
                YAML::Node config = YAML::LoadFile(yaml_path);
                
                auto ctrl_params = config["controller_params"];
                
                // Load control frequency
                if(ctrl_params["control_frequency"]) {
                    control_frequency = ctrl_params["control_frequency"].as<double>();
                }
                
                // Load position gains (Kp)
                auto kp_r1 = ctrl_params["position_gains"]["Kp_r1"];
                Kp_centroid = Eigen::Vector2d(kp_r1[0].as<double>(), kp_r1[1].as<double>());
                
                // Load velocity gains (Kv)
                auto kv_r1 = ctrl_params["velocity_gains"]["Kv_r1"];
                Kv_centroid = Eigen::Vector2d(kv_r1[0].as<double>(), kv_r1[1].as<double>());
                
                // Load integral gains (Ki)
                auto ki_r1 = ctrl_params["integral_gains"]["Ki_r1"];
                Ki_centroid = Eigen::Vector2d(ki_r1[0].as<double>(), ki_r1[1].as<double>());
                
                // Load integral limits for anti-windup
                auto max_int = ctrl_params["integral_limits"]["max_integral"];
                max_integral_error = Eigen::Vector2d(max_int[0].as<double>(), max_int[1].as<double>());
                
                // Load max velocities
                auto max_vel = ctrl_params["max_velocities"]["max_vel_r"];
                max_linear_vel = max_vel[0].as<double>();
                
                auto max_omega = ctrl_params["max_velocities"]["max_omega_r"];
                max_angular_vel = max_omega[0].as<double>();
                
                // Formation control gain
                Kf = 0.5; // Formation gain
                
                RCLCPP_INFO(this->get_logger(), "Controller frequency: %.1f Hz (dt=%.4f s)", 
                    control_frequency, 1.0/control_frequency);
                RCLCPP_INFO(this->get_logger(), "Controller gains: Kp=[%.2f,%.2f] Kv=[%.2f,%.2f] Ki=[%.2f,%.2f] Kf=%.2f",
                    Kp_centroid.x(), Kp_centroid.y(), Kv_centroid.x(), Kv_centroid.y(), 
                    Ki_centroid.x(), Ki_centroid.y(), Kf);
                RCLCPP_INFO(this->get_logger(), "Integral limits: [%.2f, %.2f]",
                    max_integral_error.x(), max_integral_error.y());
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to load controller params: %s, using defaults", e.what());
                Kp_centroid = Eigen::Vector2d(1.5, 1.5);
                Kv_centroid = Eigen::Vector2d(0.8, 0.8);
                Ki_centroid = Eigen::Vector2d(0.3, 0.3);
                max_integral_error = Eigen::Vector2d(2.0, 2.0);
                Kf = 0.5;
                max_linear_vel = 0.22;
                max_angular_vel = 2.84;
            }
            
            return control_frequency;
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
        
        double compute_formation_error() {
            // Compute formation error as norm of distance error vector
            // Using symmetric distances: (d_ij + d_ji)/2
            
            // Get current distances from inter_robot_distances
            // Robot 0: [d_01, d_02], Robot 1: [d_10, d_12], Robot 2: [d_20, d_21]
            double d_01 = inter_robot_distances[0].x();
            double d_02 = inter_robot_distances[0].y();
            double d_10 = inter_robot_distances[1].x();
            double d_12 = inter_robot_distances[1].y();
            double d_20 = inter_robot_distances[2].x();
            double d_21 = inter_robot_distances[2].y();
            
            // Compute symmetric average distances
            double d_avg_01 = (d_01 + d_10) / 2.0;
            double d_avg_02 = (d_02 + d_20) / 2.0;
            double d_avg_12 = (d_12 + d_21) / 2.0;
            
            // Distance errors (current - desired)
            double e_01 = d_avg_01 - desired_distance;
            double e_02 = d_avg_02 - desired_distance;
            double e_12 = d_avg_12 - desired_distance;
            
            // Norm of error vector [e_01, e_02, e_12]
            double formation_error = std::sqrt(e_01*e_01 + e_02*e_02 + e_12*e_12);
            
            return formation_error;
        }

        Eigen::Vector2d compute_centroid_tracking_control(const Eigen::Vector2d& centroid, double dt) {
            // PID controller for centroid tracking with anti-windup
            Eigen::Vector2d position_error = desired_position - centroid;
            Eigen::Vector2d velocity_error = desired_velocity - Eigen::Vector2d::Zero(); // assuming stationary target
            
            // Proportional term
            Eigen::Vector2d P_term = Kp_centroid.cwiseProduct(position_error);
            
            // Derivative term
            Eigen::Vector2d D_term = Kv_centroid.cwiseProduct(velocity_error);
            
            // Integral term - accumulate error over time
            integral_error += position_error * dt;
            
            // Anti-windup: clamp integral error to prevent excessive accumulation
            integral_error.x() = std::max(-max_integral_error.x(), std::min(max_integral_error.x(), integral_error.x()));
            integral_error.y() = std::max(-max_integral_error.y(), std::min(max_integral_error.y(), integral_error.y()));
            
            Eigen::Vector2d I_term = Ki_centroid.cwiseProduct(integral_error);
            
            // Total control
            Eigen::Vector2d control = P_term + I_term + D_term;
            
            RCLCPP_DEBUG(this->get_logger(), "PID: P=[%.3f,%.3f] I=[%.3f,%.3f] D=[%.3f,%.3f] total=[%.3f,%.3f]",
                P_term.x(), P_term.y(), I_term.x(), I_term.y(), D_term.x(), D_term.y(),
                control.x(), control.y());
            
            return control;
        }

        Eigen::Vector2d compute_formation_control(int robot_idx, bool use_stiff_pd = false) {
            // Formation control to maintain equilateral triangle
            
            Eigen::Vector2d formation_force = Eigen::Vector2d::Zero();
            
            // Get the other two robot indices
            int other1 = (robot_idx + 1) % 3;
            int other2 = (robot_idx + 2) % 3;
            
            // Compute vectors to other robots
            Eigen::Vector2d diff1 = robot_positions[other1] - robot_positions[robot_idx];
            Eigen::Vector2d diff2 = robot_positions[other2] - robot_positions[robot_idx];
            
            double dist1 = diff1.norm();
            double dist2 = diff2.norm();
            
            // Avoid division by zero
            if(dist1 < 1e-3 || dist2 < 1e-3) {
                RCLCPP_DEBUG(this->get_logger(), "Robot %d: distances too small, skipping formation control", robot_idx);
                return formation_force;
            }
            
            // Normalized direction vectors
            Eigen::Vector2d dir1 = diff1 / dist1;
            Eigen::Vector2d dir2 = diff2 / dist2;
            
            // Individual distance errors
            double error1 = dist1 - desired_distance;
            double error2 = dist2 - desired_distance;
            
            if(use_stiff_pd) {
                // FORMATION PHASE: Stiff PD control (critically damped)
                Eigen::Vector2d force1 = Kp_formation * error1 * dir1;
                Eigen::Vector2d force2 = Kp_formation * error2 * dir2;
                
                formation_force = force1 + force2;
                
                RCLCPP_DEBUG(this->get_logger(), "Robot %d FORMATION: e1=%.3f e2=%.3f F=[%.3f,%.3f]",
                    robot_idx, error1, error2, formation_force.x(), formation_force.y());
                
            } else {
                // TRACKING PHASE: Soft spring-like forces on EACH link independently
                // This preserves distances better during motion
                
                // Spring force proportional to distance error on each link
                // Force direction: attract if too far, repel if too close
                Eigen::Vector2d force1 = Kf * error1 * dir1;  // Force toward/from robot 1
                Eigen::Vector2d force2 = Kf * error2 * dir2;  // Force toward/from robot 2
                
                formation_force = force1 + force2;
                
                RCLCPP_DEBUG(this->get_logger(), "Robot %d TRACKING: e1=%.3f e2=%.3f F=[%.3f,%.3f]",
                    robot_idx, error1, error2, formation_force.x(), formation_force.y());
            }
            
            return formation_force;
        }

        void control_loop() {
            // Main control loop
            
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
            
            // Publish centroid position
            auto centroid_msg = std_msgs::msg::Float64MultiArray();
            centroid_msg.data = {centroid.x(), centroid.y()};
            centroid_position_pub->publish(centroid_msg);
            
            // Compute formation error for state machine
            double formation_error = compute_formation_error();
            
            // STATE MACHINE: Check for phase transitions
            if(current_phase == ControlPhase::FORMATION) {
                if(formation_error < formation_convergence_threshold) {
                    current_phase = ControlPhase::TRACKING;
                    RCLCPP_INFO(this->get_logger(), "=== PHASE TRANSITION: FORMATION → TRACKING ===");
                    RCLCPP_INFO(this->get_logger(), "Formation achieved! Error: %.3f m < %.3f m", 
                        formation_error, formation_convergence_threshold);
                    RCLCPP_INFO(this->get_logger(), "Now tracking trajectory with PID + formation maintenance");
                } else {
                    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "FORMATION phase: error=%.3f m (target: < %.3f m)", 
                        formation_error, formation_convergence_threshold);
                }
            }
            
            // Compute control based on current phase
            Eigen::Vector2d centroid_control = Eigen::Vector2d::Zero();
            double tracking_error_magnitude = 0.0;
            
            if(current_phase == ControlPhase::FORMATION) {
                // FORMATION PHASE: No trajectory tracking
                centroid_control = Eigen::Vector2d::Zero();
                
            } else {
                // TRACKING PHASE: PID centroid tracking
                centroid_control = compute_centroid_tracking_control(centroid, control_dt);
                
                Eigen::Vector2d tracking_error_vec = desired_position - centroid;
                tracking_error_magnitude = tracking_error_vec.norm();
                
                RCLCPP_DEBUG(this->get_logger(), "TRACKING: error=%.3f m, formation=%.3f m", 
                    tracking_error_magnitude, formation_error);
            }
            
            // Publish tracking error
            publish_tracking_error(centroid);
            
            // Compute velocity commands for each robot
            bool saturation_occurred = false;
            
            for(int i = 0; i < 3; i++) {
                Eigen::Vector2d total_control = centroid_control;
                
                // Add formation control based on current phase
                if(current_phase == ControlPhase::FORMATION) {
                    // FORMATION phase: stiff PD control
                    Eigen::Vector2d formation_control = compute_formation_control(i, true);
                    total_control += formation_control;
                    
                } else {
                    // TRACKING phase: ALWAYS apply formation control (not conditional)
                    // Reduce Kf to make it softer and avoid interference
                    Eigen::Vector2d formation_control = compute_formation_control(i, false);
                    
                    // Scale formation control based on tracking error (less weight when far from target)
                    double formation_weight = 1.0;
                    if(tracking_error_magnitude > tracking_error_threshold) {
                        // Reduce formation control when tracking error is large
                        formation_weight = 0.3; // 30% weight when far from trajectory
                    }
                    
                    total_control += formation_weight * formation_control;
                    
                    RCLCPP_DEBUG(this->get_logger(), "Robot %d: formation_weight=%.2f", i, formation_weight);
                }
                
                // Convert 2D velocity to differential drive
                double desired_speed = total_control.norm();
                
                // Saturate linear velocity
                if(desired_speed > max_linear_vel) {
                    desired_speed = max_linear_vel;
                    saturation_occurred = true;
                }
                
                double desired_heading = std::atan2(total_control.y(), total_control.x());
                double current_yaw = robot_orientations[i];
                double heading_error = desired_heading - current_yaw;
                
                // Normalize heading error to [-pi, pi]
                while(heading_error > PI) heading_error -= 2.0 * PI;
                while(heading_error < -PI) heading_error += 2.0 * PI;
                
                // Derivative term: (current_error - previous_error) / dt
                double heading_error_derivative = (heading_error - previous_heading_errors[i]) / control_dt;
                
                // Update previous error for next iteration
                previous_heading_errors[i] = heading_error;
                
                auto cmd_msg = geometry_msgs::msg::Twist();
                
                double speed_factor = std::cos(heading_error);
                double min_speed_factor = 0.5;
                cmd_msg.linear.x = desired_speed * std::max(min_speed_factor, speed_factor);
                cmd_msg.linear.y = 0.0;
                
                // PD control for angular velocity: Kp * error + Kd * derivative
                double Kp_yaw = 2.5;  // Proportional gain for heading
                double Kd_yaw = 0.8;  // Derivative gain for heading (damping)
                cmd_msg.angular.z = Kp_yaw * heading_error + Kd_yaw * heading_error_derivative;
                
                if(std::abs(cmd_msg.angular.z) > max_angular_vel) {
                    cmd_msg.angular.z = (cmd_msg.angular.z > 0) ? max_angular_vel : -max_angular_vel;
                    saturation_occurred = true;
                }
                
                // Publish commands
                if(i == 0) cmd_vel_robot1_pub->publish(cmd_msg);
                else if(i == 1) cmd_vel_robot2_pub->publish(cmd_msg);
                else if(i == 2) cmd_vel_robot3_pub->publish(cmd_msg);
                
                RCLCPP_DEBUG(this->get_logger(), "Robot %d: yaw=%.1f° desired=%.1f° error=%.1f° v=%.3f ω=%.3f", 
                    i, current_yaw * 180.0 / PI, desired_heading * 180.0 / PI, 
                    heading_error * 180.0 / PI, cmd_msg.linear.x, cmd_msg.angular.z);
            }
            
            // Anti-windup
            if(saturation_occurred) {
                double back_calc_gain = 0.3;
                integral_error *= back_calc_gain;
                
                RCLCPP_DEBUG(this->get_logger(), "Saturation detected - reducing integral error");
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
                    RCLCPP_DEBUG(this->get_logger(), "Robot %d has out-of-range distances, skipping", (int)robot_idx);
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

        // Odometry callbacks to get robot orientations
        void odom_robot1_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            robot_orientations[0] = extract_yaw_from_quaternion(msg->pose.pose.orientation);
        }
        
        void odom_robot2_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            robot_orientations[1] = extract_yaw_from_quaternion(msg->pose.pose.orientation);
        }
        
        void odom_robot3_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            robot_orientations[2] = extract_yaw_from_quaternion(msg->pose.pose.orientation);
        }
        
        // Extract yaw angle from quaternion
        double extract_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q) {
            // Convert quaternion to yaw using standard formula
            // yaw = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y^2 + q.z^2))
            double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
            double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
            return std::atan2(siny_cosp, cosy_cosp);
        }

        // Publish tracking error (desired vs estimated centroid)
        void publish_tracking_error(const Eigen::Vector2d& centroid) {
            // Calculate error (desired - estimated)
            // Use the SAME centroid that was used for control computation!
            Eigen::Vector2d position_error = desired_position - centroid;
            
            // Publish only error vector (2 values: x, y)
            auto msg = std_msgs::msg::Float64MultiArray();
            msg.data = {position_error.x(), position_error.y()};
            tracking_error_pub->publish(msg);
            
            RCLCPP_DEBUG(this->get_logger(), "Tracking error: [%.3f, %.3f]m (norm: %.3f m)",
                position_error.x(), position_error.y(), position_error.norm());
        }




    private:

        // Publishers for robot cmd_vel
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_robot1_pub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_robot2_pub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_robot3_pub;

        // Publishers for monitoring
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr kf_filtered_robot_pose_pub;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tracking_error_pub;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr desired_trajectory_pub;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr centroid_position_pub;

        // Subscribers for UWB filtered data
        rclcpp::Subscription<int_sys_fp::msg::AnchorDist>::SharedPtr uwb_anchor_sub;
        rclcpp::Subscription<int_sys_fp::msg::RobotDist>::SharedPtr uwb_robot_sub;
        
        // Subscriber for desired trajectory from planner
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr trajectory_sub;
        
        // Subscribers for robot odometry (to get orientation)
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_robot1_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_robot2_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_robot3_sub;

        // Control timer (50 Hz)
        rclcpp::TimerBase::SharedPtr control_timer;

        // Robot states (2D for simplicity)
        std::vector<Eigen::Vector2d> robot_positions;        // Estimated from trilateration
        std::vector<Eigen::Vector2d> robot_velocities;       // Could be estimated or measured
        std::vector<Eigen::Vector2d> inter_robot_distances;  // Distances between robots
        std::vector<double> robot_orientations;              // Robot yaw angles (from odometry)
        std::vector<double> previous_heading_errors;         // Previous heading errors for derivative term
        
        // Anchor positions (loaded from config)
        std::vector<Eigen::Vector3d> anchor_positions;
        
        // Desired trajectory from planner
        Eigen::Vector2d desired_position;
        Eigen::Vector2d desired_velocity;
        
        // Formation control parameters
        double desired_distance;           // Desired inter-robot distance for equilateral triangle
        double tracking_error_threshold;   // Threshold below which formation control activates
        
        // State machine for control phases
        enum class ControlPhase {
            FORMATION,   // Phase 1: Achieve equilateral formation (stiff PD, no tracking)
            TRACKING     // Phase 2: Track trajectory (PID + formation maintenance)
        };
        ControlPhase current_phase;
        double formation_convergence_threshold;  // Threshold for switching to tracking phase
        
        // Controller gains
        Eigen::Vector2d Kp_centroid;  // Position gain for centroid tracking
        Eigen::Vector2d Kv_centroid;  // Velocity gain for centroid tracking
        Eigen::Vector2d Ki_centroid;  // Integral gain for centroid tracking
        double Kf;                     // Formation control gain
        double Kp_formation;           // Stiff proportional gain for initial formation
        double Kd_formation;           // Derivative gain for initial formation (critically damped)
        
        // Integral state for PID controller
        Eigen::Vector2d integral_error;      // Accumulated integral error
        Eigen::Vector2d max_integral_error;  // Anti-windup limit
        
        // Velocity limits
        double max_linear_vel;
        double max_angular_vel;
        
        // Control loop period (calculated from timer frequency)
        double control_dt;


};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerClass>());
    rclcpp::shutdown();
    return 0;
}