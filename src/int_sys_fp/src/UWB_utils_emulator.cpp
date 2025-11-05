/**
 * @file UWB_utils_emulator.cpp
 * @brief Emulator for UWB utilities
 *
 * This file contains the implementation of the UWB utilities emulator.
 */

#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <random>
#include <algorithm>        
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "int_sys_fp/msg/robot_dist.hpp"
#include "int_sys_fp/msg/anchor_dist.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

class UWBNodeClass : public rclcpp::Node{
    public:
        UWBNodeClass() : Node("uwb_sensor_emulator") {
            // Initialize all vectors FIRST before parsing parameters
            robot_positions_.resize(3); // expressed in ground frame in Gazebo or another simulation environment like Rviz 
            robot_velocities_.resize(3);
            
            // Initialize distance vectors
            anchor_distances_.resize(9); // 3 robots x 3 anchors
            robot_distances_.resize(3);  // 3 robots, each will have distances to other robots
            for(int i = 0; i < 3; i++) {
                robot_distances_[i].resize(2); // Each robot has distances to 2 other robots
            }
            
            // Initialize sensor range vectors
            anchor_sensor_ranges_.resize(3);
            robot_sensor_ranges_.resize(3);
            for(int i = 0; i < 3; i++) {
                anchor_sensor_ranges_[i].resize(2); // [min_range, max_range]
                robot_sensor_ranges_[i].resize(2);  // [min_range, max_range]
            }
            
            // Note: parameter_parsing() is now called from init() after we have the noise_type argument

            // Check simulation time parameter (already declared by ROS2)
            bool use_sim_time = false;
            try {
                use_sim_time = this->get_parameter("use_sim_time").as_bool();
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Could not get use_sim_time parameter, defaulting to false");
            }
            
            if (use_sim_time) {
                RCLCPP_INFO(this->get_logger(), "Using simulation time from Gazebo");
            } else {
                RCLCPP_INFO(this->get_logger(), "Using wall clock time");
            }
            // Create publishers for UWB distance data usando i messaggi custom
            anchor_distances_pub_ = this->create_publisher<int_sys_fp::msg::AnchorDist>(
                "uwb/anchor_distances", 10);
            robot_distances_pub_ = this->create_publisher<int_sys_fp::msg::RobotDist>(
                "uwb/robot_distances", 10);
            
            // ROS parameter for sensor frequency will be declared after parsing YAML
        }

        void init(int noise_type_arg){
            // Parse parameters with the noise type from command line
            parameter_parsing(noise_type_arg);
            
            // Initialize subscribers to TurtleBot3 odometry topics
            // TurtleBot3 publishes on /odom, /tb3_2/odom, /tb3_3/odom
            
            robot1_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odom", 10,
                std::bind(&UWBNodeClass::robot1_callback, this, std::placeholders::_1));
                
            robot2_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/tb3_2/odom", 10,
                std::bind(&UWBNodeClass::robot2_callback, this, std::placeholders::_1));
                
            robot3_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/tb3_3/odom", 10,
                std::bind(&UWBNodeClass::robot3_callback, this, std::placeholders::_1));

            // Create timer for publishing UWB data using the configured sensor frequency

            int timer_period_ms = static_cast<int>(1000.0 / sensor_frequency_);

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(timer_period_ms),
                std::bind(&UWBNodeClass::publish_data, this));

            RCLCPP_INFO(this->get_logger(), "UWB Sensor Emulator initialized with TurtleBot3 topics");
            RCLCPP_INFO(this->get_logger(), "Publishing frequency: %.1f Hz (period: %d ms)", sensor_frequency_, timer_period_ms);
        }

        // Getter for sensor frequency (public access)
        double get_sensor_frequency() const {
            return sensor_frequency_;
        }

        void parameter_parsing(int noise_type_arg){
            
            // Use noise type from command-line argument (passed from main)
            int NOISE_TYPE = noise_type_arg;

            // Check the type of noise if it is valid or set to fallback case 
            
            if(NOISE_TYPE != 1 && NOISE_TYPE != 2){
                RCLCPP_WARN(this->get_logger(),"Invalid noise type selected, defaulting to gaussian noise");
                NOISE_TYPE = 1;
            }
            
            RCLCPP_INFO(this->get_logger(), "Noise type argument: %d", NOISE_TYPE);

            // Set noise type and YAML file based on user choice
            
            YAML::Node config;

            if(NOISE_TYPE == 1) {
                noise_type_ = "Gaussian";
                RCLCPP_INFO(this->get_logger(), "Using Gaussian noise");
                config = YAML::LoadFile(yaml_gauss);
            } else {
                noise_type_ = "Uniform";
                RCLCPP_INFO(this->get_logger(), "Using Uniform noise");
                config = YAML::LoadFile(yaml_uniform);
            }

            try {
                auto uwb_config = config["UWB_sensor"];
                
                // Anchor 1
                auto anchor1 = uwb_config["anchor 1"]["position"];
                anchor_positions_[0] = Eigen::Vector3d(
                    anchor1[0].as<double>(),
                    anchor1[1].as<double>(),
                    anchor1[2].as<double>()
                );
                
                // Anchor 2  
                auto anchor2 = uwb_config["anchor 2"]["position"];
                anchor_positions_[1] = Eigen::Vector3d(
                    anchor2[0].as<double>(),
                    anchor2[1].as<double>(),
                    anchor2[2].as<double>()
                );
                
                // Anchor 3
                auto anchor3 = uwb_config["anchor 3"]["position"];
                anchor_positions_[2] = Eigen::Vector3d(
                    anchor3[0].as<double>(),
                    anchor3[1].as<double>(),
                    anchor3[2].as<double>()
                );
                
                // Load noise parameters from first anchor (assuming all anchors use same noise model)
                auto noise_model = uwb_config["anchor 1"]["noise model"];
                
                // Verify all anchors have the same noise type
                std::string anchor1_noise = uwb_config["anchor 1"]["noise model"]["type"].as<std::string>();
                std::string anchor2_noise = uwb_config["anchor 2"]["noise model"]["type"].as<std::string>();
                std::string anchor3_noise = uwb_config["anchor 3"]["noise model"]["type"].as<std::string>();
                
                if(anchor1_noise != anchor2_noise || anchor1_noise != anchor3_noise){
                    RCLCPP_WARN(this->get_logger(),"Different noise types detected among anchors, using anchor 1 type: %s", anchor1_noise.c_str());
                }

                // Load and verify sensor frequencies are consistent
                double sensor_frequency_1 = uwb_config["anchor 1"]["frequency"].as<double>();
                double sensor_frequency_2 = uwb_config["anchor 2"]["frequency"].as<double>();
                double sensor_frequency_3 = uwb_config["anchor 3"]["frequency"].as<double>();
                
                // Check for consistency among anchor frequencies
                if(sensor_frequency_1 != sensor_frequency_2 || sensor_frequency_1 != sensor_frequency_3){
                    RCLCPP_WARN(this->get_logger(),"Different sensor frequencies detected among anchors, using anchor 1 frequency: %.1f Hz", sensor_frequency_1);
                }
                
                // Validate sensor frequency (must be positive)
                if(sensor_frequency_1 <= 0.0){
                    RCLCPP_WARN(this->get_logger(),"Invalid sensor frequency detected (%.1f Hz), defaulting to 50Hz", sensor_frequency_1);
                    sensor_frequency_1 = 50.0;
                }
                
                // Store the sensor frequency as member variable
                sensor_frequency_ = sensor_frequency_1;
                
                // Declare and set ROS parameter for other nodes to access
                this->declare_parameter("uwb_sensor_frequency", 50.0);
                this->set_parameter(rclcpp::Parameter("uwb_sensor_frequency", sensor_frequency_));
                
                RCLCPP_INFO(this->get_logger(), "Sensor frequency set to: %.1f Hz", sensor_frequency_);
                RCLCPP_INFO(this->get_logger(), "ROS parameter 'uwb_sensor_frequency' published for synchronization");
                
                // Parse noise parameters based on the selected noise type
                if (noise_type_ == "Gaussian") {
                    noise_mean_ = noise_model["mean"].as<double>();
                    noise_stddev_ = noise_model["stddev"].as<double>();
                    RCLCPP_INFO(this->get_logger(), "Noise model: Gaussian (mean=%.3f, stddev=%.3f)", 
                               noise_mean_, noise_stddev_);
                } else if (noise_type_ == "Uniform") {
                    // For uniform noise: min and max range
                    noise_min_ = noise_model["min"].as<double>();
                    noise_max_ = noise_model["max"].as<double>();
                    RCLCPP_INFO(this->get_logger(), "Noise model: Uniform (min=%.3f, max=%.3f)", 
                               noise_min_, noise_max_);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Unknown noise type '%s', defaulting to Gaussian", 
                               noise_type_.c_str());
                    noise_type_ = "Gaussian";
                    noise_mean_ = 0.0;
                    noise_stddev_ = 0.05;
                }
                
                RCLCPP_INFO(this->get_logger(), "Loaded anchor positions: [%.1f,%.1f,%.1f], [%.1f,%.1f,%.1f], [%.1f,%.1f,%.1f]",
                    anchor_positions_[0].x(), anchor_positions_[0].y(), anchor_positions_[0].z(),
                    anchor_positions_[1].x(), anchor_positions_[1].y(), anchor_positions_[1].z(),
                    anchor_positions_[2].x(), anchor_positions_[2].y(), anchor_positions_[2].z());
                    
                // Load sensor ranges for anchors and robots
                for(int i=0; i<3;i++){
                    anchor_sensor_ranges_[i][0] = uwb_config["anchor "+std::to_string(i+1)]["min range"].as<double>();
                    anchor_sensor_ranges_[i][1] = uwb_config["anchor "+std::to_string(i+1)]["max range"].as<double>();
                    robot_sensor_ranges_[i][0]  = uwb_config["robot "+std::to_string(i+1)]["min range"].as<double>();
                    robot_sensor_ranges_[i][1]  = uwb_config["robot "+std::to_string(i+1)]["max range"].as<double>();    
                }
                
                RCLCPP_INFO(this->get_logger(), "Loaded sensor ranges successfully");
                    
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error loading parameters: %s", e.what());
            }
        }

        // Callback functions for robot positions from Gazebo
        void robot1_callback(const nav_msgs::msg::Odometry& msg) {
            robot_positions_[0] = Eigen::Vector3d(
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            );
            robot_velocities_[0] = Eigen::Vector3d(
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z
            );
            calculate_distances();
        }
        
        void robot2_callback(const nav_msgs::msg::Odometry& msg) {
            robot_positions_[1] = Eigen::Vector3d(
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            );
            robot_velocities_[1] = Eigen::Vector3d(
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z
            );
            calculate_distances();
        }
        
        void robot3_callback(const nav_msgs::msg::Odometry& msg) {
            robot_positions_[2] = Eigen::Vector3d(
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            );
            robot_velocities_[2] = Eigen::Vector3d(
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z
            );
            calculate_distances();
        }

        double compute_distances(Eigen::Vector3d p1,Eigen::Vector3d p2){
            return (p1 - p2).norm();
        }

        void calculate_distances() {
            // Calculate and store distances from each robot to each anchor
            for(int robot_idx = 0; robot_idx < 3; robot_idx++) {
                for(int anchor_idx = 0; anchor_idx < 3; anchor_idx++) {
                    double distance = compute_distances(robot_positions_[robot_idx], anchor_positions_[anchor_idx]);
                    // Store all distances, not just robot 0
                    anchor_distances_[robot_idx * 3 + anchor_idx] = distance;
                }
            }
            
            // Calculate distances between robots
            for(int i = 0; i < 3; i++) {
                int other_idx = 0;
                for(int j = 0; j < 3; j++) {
                    if(i != j) {
                        double distance = compute_distances(robot_positions_[i], robot_positions_[j]);
                        robot_distances_[i][other_idx] = distance;
                        other_idx++;
                    }
                }
            }
        }

        void publish_data(){
            // Publish anchor distances using new message structure
            // One message containing distances from all robots to all anchors
            auto anchor_msg = int_sys_fp::msg::AnchorDist();
            anchor_msg.header.stamp = this->get_clock()->now();
            anchor_msg.header.frame_id = "uwb_frame";
            
            // Initialize arrays for 3 robots each
            anchor_msg.distances_a1.resize(3);  // distances from all robots to anchor 1
            anchor_msg.distances_a2.resize(3);  // distances from all robots to anchor 2  
            anchor_msg.distances_a3.resize(3);  // distances from all robots to anchor 3
            
            // Fill distances for each anchor
            for(int robot_id = 0; robot_id < 3; robot_id++) {
                // Distance from robot_id to anchor 0 (a1)
                anchor_msg.distances_a1[robot_id] = anchor_distances_[robot_id * 3 + 0];
                // Distance from robot_id to anchor 1 (a2)
                anchor_msg.distances_a2[robot_id] = anchor_distances_[robot_id * 3 + 1];
                // Distance from robot_id to anchor 2 (a3)
                anchor_msg.distances_a3[robot_id] = anchor_distances_[robot_id * 3 + 2];
            }
            
            // Apply noise and saturation for anchor distances
            add_noise(anchor_msg.distances_a1);
            add_noise(anchor_msg.distances_a2);
            add_noise(anchor_msg.distances_a3);
            
            apply_anchor_saturation(anchor_msg.distances_a1, 0);  // anchor 1
            apply_anchor_saturation(anchor_msg.distances_a2, 1);  // anchor 2
            apply_anchor_saturation(anchor_msg.distances_a3, 2);  // anchor 3
            
            anchor_distances_pub_->publish(anchor_msg);
            
            // Publish robot-to-robot distances using new message structure
            // One message containing distances from all robots to other robots
            auto robot_msg = int_sys_fp::msg::RobotDist();
            robot_msg.header.stamp = this->get_clock()->now();
            robot_msg.header.frame_id = "uwb_frame";
            
            // Initialize arrays for robot distances (each robot has 2 distances to other robots)
            robot_msg.distances_r1.resize(2);  // distances from robot 1 to other robots
            robot_msg.distances_r2.resize(2);  // distances from robot 2 to other robots
            robot_msg.distances_r3.resize(2);  // distances from robot 3 to other robots
            
            // Fill robot-to-robot distances
            robot_msg.distances_r1 = robot_distances_[0];  // robot 1 distances
            robot_msg.distances_r2 = robot_distances_[1];  // robot 2 distances
            robot_msg.distances_r3 = robot_distances_[2];  // robot 3 distances
            
            // Apply noise and saturation for robot distances
            add_noise(robot_msg.distances_r1);
            add_noise(robot_msg.distances_r2);
            add_noise(robot_msg.distances_r3);
            
            apply_robot_saturation(robot_msg.distances_r1, 0);  // robot 1
            apply_robot_saturation(robot_msg.distances_r2, 1);  // robot 2
            apply_robot_saturation(robot_msg.distances_r3, 2);  // robot 3
            
            robot_distances_pub_->publish(robot_msg);
        }
 
        void add_noise(std::vector<double>& distances) {
            if (noise_type_ == "Gaussian") {
                    // Gaussian noise distribution
                    std::normal_distribution<double> noise_dist(noise_mean_, noise_stddev_);
                    
                    for (auto& distance : distances) {
                        double noise = noise_dist(gen_);
                        distance += noise;
                        // Ensure distance remains positive
                        if (distance < 0.0) distance = 0.01; // minimum 1cm
                    }
                    
                } else if (noise_type_ == "Uniform") {
                    // Uniform noise distribution
                    std::uniform_real_distribution<double> noise_dist(noise_min_, noise_max_);
                    
                    for (auto& distance : distances) {
                        double noise = noise_dist(gen_);
                        distance += noise;
                        // Ensure distance remains positive
                        if (distance < 0.0) distance = 0.01; // minimum 1cm
                    }
                }
            }

        // Helper function to apply saturation for anchor distances
        void apply_anchor_saturation(std::vector<double>& distances, int anchor_id) {
            for(int robot_id = 0; robot_id < distances.size(); robot_id++) {
                if(distances[robot_id] < anchor_sensor_ranges_[anchor_id][0] ||
                   distances[robot_id] > anchor_sensor_ranges_[anchor_id][1]) 
                {
                    double original_distance = distances[robot_id];
                    distances[robot_id] = -1.0; // Indicate out of range with -1.0
                    RCLCPP_WARN(this->get_logger(), "Robot %d to Anchor %d: distance %.2f m out of range [%.1f, %.1f] m - saturated to -1.0", 
                               robot_id, anchor_id, original_distance, 
                               anchor_sensor_ranges_[anchor_id][0], anchor_sensor_ranges_[anchor_id][1]);
                }
            }
        }

        // Helper function to apply saturation for robot-to-robot distances
        void apply_robot_saturation(std::vector<double>& distances, int robot_id) {
            for(int dist_idx = 0; dist_idx < distances.size(); dist_idx++) {
                if(distances[dist_idx] < robot_sensor_ranges_[robot_id][0] ||
                   distances[dist_idx] > robot_sensor_ranges_[robot_id][1]) 
                {
                    double original_distance = distances[dist_idx];
                    distances[dist_idx] = -1.0; // Indicate out of range with -1.0
                    RCLCPP_WARN(this->get_logger(), "Robot %d inter-robot distance %.2f m out of range [%.1f, %.1f] m - saturated to -1.0", 
                               robot_id, original_distance, 
                               robot_sensor_ranges_[robot_id][0], robot_sensor_ranges_[robot_id][1]);
                }
            }
        }

    private:
        // Publishers - aggiornati per usare i messaggi custom
        rclcpp::Publisher<int_sys_fp::msg::AnchorDist>::SharedPtr anchor_distances_pub_;
        rclcpp::Publisher<int_sys_fp::msg::RobotDist>::SharedPtr robot_distances_pub_;
        
        // Subscribers for robot positions from Gazebo
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot1_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot2_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot3_sub_;
        
        // Timer for publishing
        rclcpp::TimerBase::SharedPtr timer_;
        
        // Robot states
        std::vector<Eigen::Vector3d> robot_positions_;
        std::vector<Eigen::Vector3d> robot_velocities_;
        
        // Anchor positions (loaded from YAML)
        Eigen::Vector3d anchor_positions_[3];
        
        // Distance storage
        std::vector<double> anchor_distances_;  // distances from robots to anchors
        std::vector<std::vector<double>> robot_distances_;  // distances between robots
        
        // Noise parameters
        std::string noise_type_;              // "Gaussian" or "Uniform"
        double noise_mean_;                   // For Gaussian noise
        double noise_stddev_;                 // For Gaussian noise
        double noise_min_, noise_max_;        // For Uniform noise
        
        // Sensor configuration
        double sensor_frequency_;                                   // UWB sensor frequency in Hz
        std::vector<std::vector<double>> anchor_sensor_ranges_;     // Sensor ranges for each anchor
        std::vector<std::vector<double>> robot_sensor_ranges_;      // Sensor ranges for each robot
        
        std::random_device rd_;               // generate the seed for the random number generator
        std::mt19937 gen_{rd_()};             // generated sequence based on Marsenne-Twister 

        // YAML configuration choice - usando ament_index per trovare i file
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("int_sys_fp");
        std::string yaml_gauss = package_share_directory + "/sensor_params.yaml";
        std::string yaml_uniform = package_share_directory + "/sensor_params_uniform.yaml";
        int NOISE_TYPE = 1; // Default to Gaussian noise
};

// Main function to create and run the node
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // Parse noise type from command-line argument
    // Usage: uwb_emulator <noise_type>
    //   noise_type: 1 = Gaussian, 2 = Uniform
    int noise_type = 1; // Default to Gaussian
    
    if (argc > 1) {
        try {
            noise_type = std::stoi(argv[1]);
            if (noise_type != 1 && noise_type != 2) {
                std::cerr << "Warning: Invalid noise type " << noise_type << ", defaulting to Gaussian (1)" << std::endl;
                noise_type = 1;
            }
        } catch (const std::exception& e) {
            std::cerr << "Error parsing noise type argument: " << e.what() << ", defaulting to Gaussian (1)" << std::endl;
            noise_type = 1;
        }
    } else {
        std::cout << "No noise type specified, defaulting to Gaussian (1)" << std::endl;
        std::cout << "Usage: uwb_emulator <noise_type> where noise_type is 1 (Gaussian) or 2 (Uniform)" << std::endl;
    }
    
    auto node = std::make_shared<UWBNodeClass>();
    
    node->init(noise_type);
    
    RCLCPP_INFO(node->get_logger(), "UWB Emulator starting...");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}





