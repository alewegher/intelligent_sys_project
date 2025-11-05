/**
 * @file UKF.cpp
 * @brief C++ implementation of Kalman Filter for UWB distance measurements
 * 
 * Converted from KF.py for better performance
 */

#include <rclcpp/rclcpp.hpp>
#include "int_sys_fp/msg/robot_dist.hpp"
#include "int_sys_fp/msg/anchor_dist.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <vector>
#include <deque>
#include <algorithm>
#include <cmath>

using namespace Eigen;

class DistanceKFNode : public rclcpp::Node {
public:
    DistanceKFNode() : Node("distance_kf_node") {
        RCLCPP_INFO(this->get_logger(), "Distance Kalman Filter Node Started (C++)");
        
        // Configuration
        node_freq_ = 50.0;  // Hz
        dt_ = 1.0 / node_freq_;
        
        // Initialize measurements storage
        anchor_distances_.resize(3);  // 3 anchors
        robot_distances_.resize(3);   // 3 robots
        
        // Parse parameters
        parse_kf_parameters();
        
        RCLCPP_INFO(this->get_logger(), "Node frequency set to: %.1f Hz", node_freq_);
        
        // Initialize KF state
        initialize_kf_state();
        
        // Create subscribers
        uwb_anchor_sub_ = this->create_subscription<int_sys_fp::msg::AnchorDist>(
            "uwb/anchor_distances", 10,
            std::bind(&DistanceKFNode::read_anchor_callback, this, std::placeholders::_1));
        
        uwb_robot_sub_ = this->create_subscription<int_sys_fp::msg::RobotDist>(
            "uwb/robot_distances", 10,
            std::bind(&DistanceKFNode::read_robot_callback, this, std::placeholders::_1));
        
        // Create publishers
        filtered_anchor_pub_ = this->create_publisher<int_sys_fp::msg::AnchorDist>(
            "uwb/filtered_anchor_distances", 10);
        
        filtered_robot_pub_ = this->create_publisher<int_sys_fp::msg::RobotDist>(
            "uwb/filtered_robot_distances", 10);
        
        covariance_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "ukf_covariance_matrix", 10);
        
        // Create timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(dt_ * 1000)),
            std::bind(&DistanceKFNode::update, this));
    }

private:
    // Configuration
    double node_freq_;
    double dt_;
    
    // Measurement storage
    std::vector<std::vector<double>> anchor_distances_;  // [anchor_idx][robot_idx]
    std::vector<std::vector<double>> robot_distances_;   // [robot_idx][other_robot]
    
    // KF state
    static constexpr int N_ANCHOR_MEASUREMENTS = 9;  // 3 robots × 3 anchors
    static constexpr int N_ROBOT_MEASUREMENTS = 6;   // 3 robots × 2 others
    static constexpr int TOTAL_MEASUREMENTS = N_ANCHOR_MEASUREMENTS + N_ROBOT_MEASUREMENTS;
    
    VectorXd distance_estimates_;
    VectorXd distance_covariances_;
    VectorXd measurement_variances_;
    
    // Process noise
    double distance_process_noise_;
    double rate_process_noise_;
    
    // Measurement history for outlier detection
    std::vector<std::deque<double>> measurement_history_;
    static constexpr int MAX_HISTORY_LENGTH = 10;
    
    // Sensor parameters
    std::vector<double> sensor_anchor_std_;
    std::vector<double> sensor_robot_std_;
    
    // KF parameters
    bool outlier_detection_enabled_;
    
    // ROS interfaces
    rclcpp::Subscription<int_sys_fp::msg::AnchorDist>::SharedPtr uwb_anchor_sub_;
    rclcpp::Subscription<int_sys_fp::msg::RobotDist>::SharedPtr uwb_robot_sub_;
    rclcpp::Publisher<int_sys_fp::msg::AnchorDist>::SharedPtr filtered_anchor_pub_;
    rclcpp::Publisher<int_sys_fp::msg::RobotDist>::SharedPtr filtered_robot_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr covariance_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    void parse_kf_parameters() {
        try {
            std::string package_share = ament_index_cpp::get_package_share_directory("int_sys_fp");
            std::string yaml_path = package_share + "/sensor_params.yaml";
            
            YAML::Node config = YAML::LoadFile(yaml_path);
            auto uwb_sensor = config["UWB_sensor"];
            
            // Extract anchor standard deviations
            sensor_anchor_std_.clear();
            for(int i = 1; i <= 3; i++) {
                auto anchor = uwb_sensor["anchor " + std::to_string(i)];
                double stddev = anchor["noise model"]["stddev"].as<double>(0.05);
                sensor_anchor_std_.push_back(stddev);
            }
            
            // Extract robot standard deviations
            sensor_robot_std_.clear();
            for(int i = 1; i <= 3; i++) {
                auto robot = uwb_sensor["robot " + std::to_string(i)];
                double stddev = robot["noise model"]["stddev"].as<double>(0.05);
                sensor_robot_std_.push_back(stddev);
            }
            
            RCLCPP_INFO(this->get_logger(), "Loaded anchor std devs: [%.3f, %.3f, %.3f]",
                sensor_anchor_std_[0], sensor_anchor_std_[1], sensor_anchor_std_[2]);
            RCLCPP_INFO(this->get_logger(), "Loaded robot std devs: [%.3f, %.3f, %.3f]",
                sensor_robot_std_[0], sensor_robot_std_[1], sensor_robot_std_[2]);
            
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Error loading sensor params: %s, using defaults", e.what());
            sensor_anchor_std_ = {0.05, 0.05, 0.05};
            sensor_robot_std_ = {0.05, 0.05, 0.05};
        }
        
        // Default KF parameters
        distance_process_noise_ = 0.01;
        rate_process_noise_ = 0.1;
        outlier_detection_enabled_ = true;
        
        RCLCPP_INFO(this->get_logger(), "Using default KF parameters: process_noise, outlier_detection");
    }
    
    void initialize_kf_state() {
        // Initialize estimates
        distance_estimates_ = VectorXd::Zero(TOTAL_MEASUREMENTS);
        distance_covariances_ = VectorXd::Constant(TOTAL_MEASUREMENTS, 0.1);
        
        // Initialize measurement variances
        measurement_variances_.resize(TOTAL_MEASUREMENTS);
        
        // Anchor measurement variances (3 robots × 3 anchors = 9)
        int idx = 0;
        for(int robot = 0; robot < 3; robot++) {
            for(int anchor = 0; anchor < 3; anchor++) {
                measurement_variances_(idx++) = sensor_anchor_std_[anchor] * sensor_anchor_std_[anchor];
            }
        }
        
        // Robot measurement variances (3 robots × 2 others = 6)
        for(int robot = 0; robot < 3; robot++) {
            measurement_variances_(idx++) = sensor_robot_std_[robot] * sensor_robot_std_[robot];
            measurement_variances_(idx++) = sensor_robot_std_[robot] * sensor_robot_std_[robot];
        }
        
        // Initialize history
        measurement_history_.resize(TOTAL_MEASUREMENTS);
        
        RCLCPP_INFO(this->get_logger(), "Distance KF initialized: %d distance measurements", TOTAL_MEASUREMENTS);
        RCLCPP_INFO(this->get_logger(), "Anchor measurement vars: [%.4f, %.4f, %.4f]",
            measurement_variances_(0), measurement_variances_(3), measurement_variances_(6));
        RCLCPP_INFO(this->get_logger(), "Robot measurement vars: [%.4f, %.4f]",
            measurement_variances_(9), measurement_variances_(11));
    }
    
    void read_anchor_callback(const int_sys_fp::msg::AnchorDist::SharedPtr msg) {
        anchor_distances_[0] = msg->distances_a1;
        anchor_distances_[1] = msg->distances_a2;
        anchor_distances_[2] = msg->distances_a3;
    }
    
    void read_robot_callback(const int_sys_fp::msg::RobotDist::SharedPtr msg) {
        robot_distances_[0] = msg->distances_r1;
        robot_distances_[1] = msg->distances_r2;
        robot_distances_[2] = msg->distances_r3;
    }
    
    void update() {
        try {
            // Run Gaussian filtering
            gaussian_filtering();
            
            // Apply outlier detection
            if(outlier_detection_enabled_) {
                for(int i = 0; i < TOTAL_MEASUREMENTS; i++) {
                    apply_advanced_filtering(i);
                }
            }
            
            // Log statistics
            int valid_count = (distance_estimates_.array() > 0.01).count();
            if(valid_count > 0) {
                double avg_distance = 0.0;
                int count = 0;
                for(int i = 0; i < TOTAL_MEASUREMENTS; i++) {
                    if(distance_estimates_(i) > 0.01) {
                        avg_distance += distance_estimates_(i);
                        count++;
                    }
                }
                avg_distance /= count;
                double avg_uncertainty = distance_covariances_.mean();
                
                RCLCPP_DEBUG(this->get_logger(), "Distance stats - Avg: %.2fm, Uncertainty: %.4f",
                    avg_distance, avg_uncertainty);
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in distance filtering: %s", e.what());
        }
    }
    
    bool has_measurements() const {
        // Check if we have any measurements
        for(const auto& distances : anchor_distances_) {
            if(!distances.empty()) return true;
        }
        for(const auto& distances : robot_distances_) {
            if(!distances.empty()) return true;
        }
        return false;
    }
    
    VectorXd get_measurement_vector() {
        VectorXd z = VectorXd::Constant(TOTAL_MEASUREMENTS, 0.01);
        int idx = 0;
        
        // Anchor measurements (9 total)
        for(int robot_idx = 0; robot_idx < 3; robot_idx++) {
            for(int anchor_idx = 0; anchor_idx < 3; anchor_idx++) {
                if(anchor_idx < static_cast<int>(anchor_distances_.size()) &&
                   robot_idx < static_cast<int>(anchor_distances_[anchor_idx].size())) {
                    double dist = anchor_distances_[anchor_idx][robot_idx];
                    z(idx) = (dist > 0) ? dist : 0.01;
                }
                idx++;
            }
        }
        
        // Robot measurements (6 total)
        for(int robot_idx = 0; robot_idx < 3; robot_idx++) {
            if(robot_idx < static_cast<int>(robot_distances_.size())) {
                const auto& dists = robot_distances_[robot_idx];
                z(idx++) = (dists.size() > 0 && dists[0] > 0) ? dists[0] : 0.01;
                z(idx++) = (dists.size() > 1 && dists[1] > 0) ? dists[1] : 0.01;
            } else {
                idx += 2;
            }
        }
        
        return z;
    }
    
    void gaussian_filtering() {
        if(!has_measurements()) {
            RCLCPP_DEBUG(this->get_logger(), "No measurements available for filtering");
            return;
        }
        
        VectorXd raw_measurements = get_measurement_vector();
        
        // Apply scalar Kalman filter to each distance
        for(int i = 0; i < TOTAL_MEASUREMENTS; i++) {
            if(raw_measurements(i) > 0.01) {
                filter_single_distance(i, raw_measurements(i));
            }
        }
        
        // Publish results
        publish_filtered_results();
        publish_covariance_matrix();
    }
    
    void filter_single_distance(int measurement_idx, double raw_distance) {
        // Current estimate and covariance
        double x = distance_estimates_(measurement_idx);
        double P = distance_covariances_(measurement_idx);
        
        // PREDICTION STEP
        double x_pred = x;  // Assume slow evolution
        double P_pred = P + distance_process_noise_;
        
        // UPDATE STEP
        double R = measurement_variances_(measurement_idx);
        
        // Kalman gain
        double K = P_pred / (P_pred + R);
        
        // Innovation
        double innovation = raw_distance - x_pred;
        
        // Update
        double x_new = x_pred + K * innovation;
        double P_new = (1.0 - K) * P_pred;
        
        // Store
        distance_estimates_(measurement_idx) = x_new;
        distance_covariances_(measurement_idx) = P_new;
        
        // Update history
        auto& history = measurement_history_[measurement_idx];
        if(history.size() >= MAX_HISTORY_LENGTH) {
            history.pop_front();
        }
        history.push_back(raw_distance);
        
        // Log large innovations
        if(std::abs(innovation) > 3.0 * std::sqrt(R)) {
            RCLCPP_WARN(this->get_logger(), "Large innovation in measurement %d: %.3fm", 
                measurement_idx, innovation);
        }
    }
    
    void apply_advanced_filtering(int measurement_idx) {
        const auto& history = measurement_history_[measurement_idx];
        if(history.size() < 3) return;
        
        // Get recent measurements
        std::vector<double> recent(history.end() - std::min(5, static_cast<int>(history.size())), history.end());
        
        // Calculate median
        std::sort(recent.begin(), recent.end());
        double median_val = recent[recent.size() / 2];
        
        // Calculate MAD (Median Absolute Deviation)
        std::vector<double> deviations;
        for(double val : recent) {
            deviations.push_back(std::abs(val - median_val));
        }
        std::sort(deviations.begin(), deviations.end());
        double mad = deviations[deviations.size() / 2];
        
        // Check for outlier
        double current_estimate = distance_estimates_(measurement_idx);
        if(std::abs(current_estimate - median_val) > 3.0 * mad && mad > 0.01) {
            // Outlier detected
            distance_estimates_(measurement_idx) = median_val;
            distance_covariances_(measurement_idx) *= 1.5;
            
            RCLCPP_DEBUG(this->get_logger(), "Outlier correction applied to measurement %d", measurement_idx);
        }
    }
    
    void publish_filtered_results() {
        // Publish anchor distances
        auto anchor_msg = int_sys_fp::msg::AnchorDist();
        anchor_msg.header.stamp = this->get_clock()->now();
        anchor_msg.header.frame_id = "kf_filtered";
        
        anchor_msg.distances_a1 = {
            distance_estimates_(0),   // robot 0 to anchor 1
            distance_estimates_(3),   // robot 1 to anchor 1
            distance_estimates_(6)    // robot 2 to anchor 1
        };
        anchor_msg.distances_a2 = {
            distance_estimates_(1),
            distance_estimates_(4),
            distance_estimates_(7)
        };
        anchor_msg.distances_a3 = {
            distance_estimates_(2),
            distance_estimates_(5),
            distance_estimates_(8)
        };
        
        filtered_anchor_pub_->publish(anchor_msg);
        
        // Publish robot distances
        auto robot_msg = int_sys_fp::msg::RobotDist();
        robot_msg.header.stamp = this->get_clock()->now();
        robot_msg.header.frame_id = "kf_filtered";
        
        robot_msg.distances_r1 = {distance_estimates_(9), distance_estimates_(10)};
        robot_msg.distances_r2 = {distance_estimates_(11), distance_estimates_(12)};
        robot_msg.distances_r3 = {distance_estimates_(13), distance_estimates_(14)};
        
        filtered_robot_pub_->publish(robot_msg);
        
        // Log
        int num_valid = (distance_estimates_.array() > 0.01).count();
        double avg_uncertainty = distance_covariances_.mean();
        RCLCPP_INFO(this->get_logger(), "📡 Published filtered distances: %d/%d valid, avg uncertainty: %.4fm",
            num_valid, TOTAL_MEASUREMENTS, avg_uncertainty);
    }
    
    void publish_covariance_matrix() {
        auto cov_msg = std_msgs::msg::Float64MultiArray();
        
        // Create diagonal covariance matrix
        MatrixXd cov_matrix = distance_covariances_.asDiagonal();
        
        // Flatten
        cov_msg.data.resize(TOTAL_MEASUREMENTS * TOTAL_MEASUREMENTS);
        for(int i = 0; i < TOTAL_MEASUREMENTS; i++) {
            for(int j = 0; j < TOTAL_MEASUREMENTS; j++) {
                cov_msg.data[i * TOTAL_MEASUREMENTS + j] = cov_matrix(i, j);
            }
        }
        
        // Set layout
        std_msgs::msg::MultiArrayDimension dim1, dim2;
        dim1.label = "rows";
        dim1.size = TOTAL_MEASUREMENTS;
        dim1.stride = TOTAL_MEASUREMENTS * TOTAL_MEASUREMENTS;
        
        dim2.label = "cols";
        dim2.size = TOTAL_MEASUREMENTS;
        dim2.stride = TOTAL_MEASUREMENTS;
        
        cov_msg.layout.dim = {dim1, dim2};
        cov_msg.layout.data_offset = 0;
        
        covariance_pub_->publish(cov_msg);
        
        double total_uncertainty = distance_covariances_.sum();
        double max_uncertainty = distance_covariances_.maxCoeff();
        double min_uncertainty = distance_covariances_.minCoeff();
        
        RCLCPP_DEBUG(this->get_logger(), "Distance uncertainties - Total: %.4f, Max: %.4f, Min: %.4f",
            total_uncertainty, max_uncertainty, min_uncertainty);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<DistanceKFNode>();
    
    try {
        rclcpp::spin(node);
    } catch(const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}