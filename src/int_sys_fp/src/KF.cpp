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
            "/uwb/anchor_distances", 10,
            std::bind(&DistanceKFNode::read_anchor_callback, this, std::placeholders::_1));
        
        uwb_robot_sub_ = this->create_subscription<int_sys_fp::msg::RobotDist>(
            "/uwb/robot_distances", 10,
            std::bind(&DistanceKFNode::read_robot_callback, this, std::placeholders::_1));
        
        // Create publishers
        filtered_anchor_pub_ = this->create_publisher<int_sys_fp::msg::AnchorDist>(
            "/uwb/filtered_anchor_distances", 10);
        
        filtered_robot_pub_ = this->create_publisher<int_sys_fp::msg::RobotDist>(
            "/uwb/filtered_robot_distances", 10);
        
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

    // Sensor parameters
    std::vector<double> sensor_anchor_std_;
    std::vector<double> sensor_robot_std_;
    
    // KF parameters
    bool outlier_detection_enabled_;

    // MAD outlier detector, all ROS parameters (see parse_kf_parameters)
    int    mad_window_;
    int    mad_min_history_;
    int    mad_history_length_;
    double mad_kappa_;
    double mad_floor_;
    double mad_cov_inflation_;
    double mad_scale_;
    long   outlier_events_ = 0;   // cumulative count, logged periodically

    // ROS interfaces
    rclcpp::Subscription<int_sys_fp::msg::AnchorDist>::SharedPtr uwb_anchor_sub_;
    rclcpp::Subscription<int_sys_fp::msg::RobotDist>::SharedPtr uwb_robot_sub_;
    rclcpp::Publisher<int_sys_fp::msg::AnchorDist>::SharedPtr filtered_anchor_pub_;
    rclcpp::Publisher<int_sys_fp::msg::RobotDist>::SharedPtr filtered_robot_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    void parse_kf_parameters() {
        try {
            std::string package_share = ament_index_cpp::get_package_share_directory("int_sys_fp");
            std::string yaml_path = package_share + "/config/sensor_params.yaml";
            
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
        
        // KF and MAD settings as ROS parameters. Every default below reproduces exactly
        // the value that used to be hardcoded here or inside apply_advanced_filtering(),
        // so an unconfigured run behaves as before.
        //
        // Deliberately ROS parameters rather than YAML: the package's YAML files are read
        // from the installed share/ directory, so changing them needs a rebuild - and an
        // experimental axis you have to rebuild to sweep is not an axis. (Note UKF_params.yaml
        // documents an outlier_detection block, but no node has ever read that file.)
        distance_process_noise_ = declare_parameter<double>("distance_process_noise", 0.01);
        rate_process_noise_     = declare_parameter<double>("rate_process_noise", 0.1);
        outlier_detection_enabled_ = declare_parameter<bool>("mad_enabled", true);
        mad_window_             = declare_parameter<int>("mad_window", 5);
        mad_min_history_        = declare_parameter<int>("mad_min_history", 3);
        mad_history_length_     = declare_parameter<int>("mad_history_length", 10);
        mad_kappa_              = declare_parameter<double>("mad_kappa", 3.0);
        mad_floor_              = declare_parameter<double>("mad_floor", 0.01);
        mad_cov_inflation_      = declare_parameter<double>("mad_cov_inflation", 1.5);
        // Gaussian-consistency factor. The threshold is kappa * mad_scale * MAD. With
        // mad_scale = 1.0 (the historical behaviour) "3*MAD" is really ~2.02 sigma, NOT
        // 3 sigma - the consistent estimator of sigma is 1.4826*MAD. Set mad_scale:=1.4826
        // for a true 3-sigma rule. Default kept at 1.0 so behaviour is unchanged.
        mad_scale_              = declare_parameter<double>("mad_scale", 1.0);

        mad_history_length_ = std::max(1, mad_history_length_);
        mad_window_         = std::max(1, mad_window_);
        mad_min_history_    = std::max(1, std::min(mad_min_history_, mad_history_length_));

        RCLCPP_INFO(this->get_logger(),
            "KF: q=%.4f | MAD: %s window=%d min_hist=%d hist=%d kappa=%.2f scale=%.4f "
            "floor=%.4f inflation=%.2f (effective threshold = %.3f*MAD)",
            distance_process_noise_, outlier_detection_enabled_ ? "on" : "off",
            mad_window_, mad_min_history_, mad_history_length_, mad_kappa_, mad_scale_,
            mad_floor_, mad_cov_inflation_, mad_kappa_ * mad_scale_);
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
        if(msg->distances_a1.size() != 3 || msg->distances_a2.size() != 3 || msg->distances_a3.size() != 3) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Invalid anchor distances array size");
            return;
        }
        anchor_distances_[0] = msg->distances_a1;
        anchor_distances_[1] = msg->distances_a2;
        anchor_distances_[2] = msg->distances_a3;
    }
    
    void read_robot_callback(const int_sys_fp::msg::RobotDist::SharedPtr msg) {
        if(msg->distances_r1.size() != 2 || msg->distances_r2.size() != 2 || msg->distances_r3.size() != 2) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Invalid robot distances array size");
            return;
        }
        robot_distances_[0] = msg->distances_r1;
        robot_distances_[1] = msg->distances_r2;
        robot_distances_[2] = msg->distances_r3;
    }
    
    void update() {
        try {
            // Run Gaussian filtering
            const bool filtered = gaussian_filtering();

            // Apply outlier detection
            if(outlier_detection_enabled_) {
                for(int i = 0; i < TOTAL_MEASUREMENTS; i++) {
                    apply_advanced_filtering(i);
                }
            }

            // Publish only after the MAD pass, so the correction is visible in the same cycle
            if(filtered) {
                publish_filtered_results();
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

            // How often MAD actually fires is the first thing you need when interpreting
            // a "MAD on vs off" comparison - if this stays at 0, the two configurations
            // are measuring the same thing.
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                "MAD outlier corrections so far: %ld", outlier_events_);

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
                   anchor_distances_[anchor_idx].size() > 0 &&
                   robot_idx < static_cast<int>(anchor_distances_[anchor_idx].size())) {
                    double dist = anchor_distances_[anchor_idx][robot_idx];
                    z(idx) = (dist > 0.001) ? dist : 0.01;
                } else {
                    z(idx) = 0.01;
                }
                idx++;
            }
        }
        
        // Robot measurements (6 total)
        for(int robot_idx = 0; robot_idx < 3; robot_idx++) {
            if(robot_idx < static_cast<int>(robot_distances_.size()) && robot_distances_[robot_idx].size() >= 2) {
                const auto& dists = robot_distances_[robot_idx];
                z(idx++) = (dists[0] > 0.001) ? dists[0] : 0.01;
                z(idx++) = (dists[1] > 0.001) ? dists[1] : 0.01;
            } else {
                z(idx++) = 0.01;
                z(idx++) = 0.01;
            }
        }
        
        return z;
    }
    
    /// @return true if measurements were available and the estimates were updated.
    bool gaussian_filtering() {
        if(!has_measurements()) {
            RCLCPP_DEBUG(this->get_logger(), "No measurements available for filtering");
            return false;
        }

        VectorXd raw_measurements = get_measurement_vector();

        // Apply scalar Kalman filter to each distance
        for(int i = 0; i < TOTAL_MEASUREMENTS; i++) {
            if(raw_measurements(i) > 0.01) {
                filter_single_distance(i, raw_measurements(i));
            }
        }

        // NOTE: publishing deliberately happens in update(), AFTER the MAD pass. It used
        // to happen here, which meant an outlier correction only reached subscribers on
        // the following cycle - so any "MAD on vs off" comparison was measuring a
        // one-cycle-delayed effect rather than the correction itself.
        return true;
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
        while(static_cast<int>(history.size()) >= mad_history_length_) {
            history.pop_front();
        }
        history.push_back(raw_distance);
        
        // Log large innovations
        // if(std::abs(innovation) > 3.0 * std::sqrt(R)) {
        //     RCLCPP_WARN(this->get_logger(), "Large innovation in measurement %d: %.3fm", 
        //         measurement_idx, innovation);
        // }
    }
    
    /**
     * @brief MAD-based outlier correction on one distance channel.
     *
     * Note what this actually compares: the history holds the RAW measurements, but the
     * value tested against their median is the KF ESTIMATE. So this detects "the estimate
     * has drifted away from recent raw data", not "this measurement is an outlier" - the
     * reverse of the reference paper, which screens the raw measurement before filtering.
     * Kept as-is (mad_stage would be the place to add the paper's variant) but worth
     * knowing before interpreting any MAD result.
     *
     * With an even-sized window, recent[n/2] takes the UPPER middle element rather than
     * averaging the two middles. Irrelevant at the default window of 5, visible at 4.
     */
    void apply_advanced_filtering(int measurement_idx) {
        const auto& history = measurement_history_[measurement_idx];
        if(static_cast<int>(history.size()) < mad_min_history_) return;

        // Get recent measurements
        std::vector<double> recent(
            history.end() - std::min(mad_window_, static_cast<int>(history.size())),
            history.end());

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
        if(std::abs(current_estimate - median_val) > mad_kappa_ * mad_scale_ * mad
           && mad > mad_floor_) {
            // Outlier detected
            distance_estimates_(measurement_idx) = median_val;
            distance_covariances_(measurement_idx) *= mad_cov_inflation_;
            outlier_events_++;

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
        // RCLCPP_INFO(this->get_logger(), "📡 Published filtered distances: %d/%d valid, avg uncertainty: %.4fm",
        //     num_valid, TOTAL_MEASUREMENTS, avg_uncertainty);
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