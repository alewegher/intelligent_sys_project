#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from int_sys_fp.msg import AnchorDist, RobotDist
import scipy.linalg as LA
import scipy.stats as stat
from rclpy.parameter import Parameter
import time
import yaml
from ament_index_python.packages import get_package_share_directory
import os
from std_msgs.msg import Float64MultiArray, MultiArrayDimension


class DistanceKFNode(Node):

    def __init__(self):
        # Initialize the ROS node FIRST
        super().__init__('distance_kf_node')
        self.get_logger().info("Distance Kalman Filter Node Started")
        
        # set the default parameters to get the frequency of the UWB sensor from the parameter server
        self.UWB_NODE_NAME = 'uwb_sensor_emulator'
        self.FREQ_PARAM = 'uwb_sensor_frequency'
        
        # Get correct file paths using ament_index
        try:
            package_share_directory = get_package_share_directory('int_sys_fp')
            sensor_conf_file_path = os.path.join(package_share_directory, 'sensor_params.yaml')
        except Exception as e:
            self.get_logger().error(f"Could not find package directory: {e}")
            # Fallback to relative path
            sensor_conf_file_path = 'sensor_params.yaml'
            
        # Initialize data storage
        self.anchor_distances = [[] for _ in range(3)]  # 3 anchors
        self.robot_distances = [[] for _ in range(3)]   # 3 robots
        
        # Parse configuration parameters
        self.parse_KF_parameters(sensor_conf_file_path)
        # Use fixed frequency (50Hz to match UWB emulator)
        self.node_freq = 50.0
        
        self.get_logger().info(f"Node frequency set to: {self.node_freq} Hz")
        
        self.dt = 1.0/self.node_freq # in seconds 
        
        # Create subscription to UWB anchor distances 
        self.uwb_anchor_sub = self.create_subscription(
            AnchorDist,
            'uwb/anchor_distances',
            self.read_anchor_callback,
            10)
        self.uwb_robot_sub = self.create_subscription(
            RobotDist,
            'uwb/robot_distances',
            self.read_robot_callback,
            10)
        
        # Create publishers for filtered results
        self.filtered_anchor_pub = self.create_publisher(
            AnchorDist, 'uwb/filtered_anchor_distances', 10)
        self.filtered_robot_pub = self.create_publisher(
            RobotDist, 'uwb/filtered_robot_distances', 10)
        
        # Initialize UKF state and covariance
        self.initialize_ukf_state()
        
        # Create timer for periodic updates
        self.timer = self.create_timer(self.dt, self.update)
    
    def initialize_ukf_state(self):
        """Initialize Kalman Filter for distance measurements only (model-free)"""
        
        # We're filtering individual distance measurements, not robot states
        # Each distance measurement gets its own scalar Kalman filter
        
        # Number of distance measurements to filter
        self.n_anchor_measurements = 9  # 3 robots × 3 anchors  
        self.n_robot_measurements = 6   # 3 robots × 2 others each
        self.total_measurements = self.n_anchor_measurements + self.n_robot_measurements
        
        # Initialize scalar Kalman filters for each distance measurement
        # Each filter estimates: [distance, distance_rate]
        self.n_states_per_filter = 2
        
        # Storage for filtered distance estimates and covariances
        self.distance_estimates = np.zeros(self.total_measurements)
        self.distance_covariances = np.ones(self.total_measurements) * 0.1  # Initial uncertainty
        self.distance_rates = np.zeros(self.total_measurements)  # Rate of change
        
        # Process noise for distance evolution (how much distances can change)
        self.distance_process_noise = self.kf_params.get('process_noise', {}).get('distance_var', 0.01)
        self.rate_process_noise = self.kf_params.get('process_noise', {}).get('rate_var', 0.1)
        
        # Measurement noise from sensor standard deviations
        self.anchor_measurement_vars = np.square(self.sensor_anchor_std)
        self.robot_measurement_vars = np.square(self.sensor_robot_std)
        
        # Combine all measurement variances
        anchor_vars_expanded = np.tile(self.anchor_measurement_vars, 3)  # 3 robots per anchor
        # CORRECTED: Use np.repeat for correct variance mapping
        # The structure is [var_r1, var_r1, var_r2, var_r2, var_r3, var_r3]
        robot_vars_expanded = np.repeat(self.robot_measurement_vars, 2)
        self.measurement_variances = np.concatenate([anchor_vars_expanded, robot_vars_expanded])
        
        # Initialize history buffer for measurements
        self.measurement_history = [[] for _ in range(self.total_measurements)]
        self.max_history_length = 10
        
        self.get_logger().info(f"Distance KF initialized: {self.total_measurements} distance measurements")
        self.get_logger().info(f"Anchor measurement vars: {self.anchor_measurement_vars}")
        self.get_logger().info(f"Robot measurement vars: {self.robot_measurement_vars}")
    
    def update(self):
        """Timer callback for periodic updates"""
        try:
            # Run distance filtering
            self.gaussian_filtering()
            
            # Apply advanced filtering (outlier detection, etc.)
            if self.kf_params.get('outlier_detection', {}).get('enabled', True):
                for i in range(self.total_measurements):
                    self.apply_advanced_filtering(i)
            
            # Log filtering statistics
            if hasattr(self, 'distance_estimates'):
                valid_distances = self.distance_estimates[self.distance_estimates > 0.01]
                if len(valid_distances) > 0:
                    avg_distance = np.mean(valid_distances)
                    std_distance = np.std(valid_distances)
                    avg_uncertainty = np.mean(self.distance_covariances)
                    
                    self.get_logger().debug(f"Distance stats - Avg: {avg_distance:.2f}m, "
                                          f"Std: {std_distance:.3f}m, Uncertainty: {avg_uncertainty:.4f}")
                    
        except Exception as e:
            self.get_logger().error(f"Error in distance filtering: {e}")
    
    def read_anchor_callback(self, msg):
        for i in range(1, 4):
            attribute = f"distances_a{i}"
            try:
                distances = getattr(msg, attribute)
                if len(distances) > 0:  # Only update if we have data
                    self.anchor_distances[i-1] = distances
            except AttributeError:
                self.get_logger().warn(f"Missing attribute {attribute} in anchor distance message")

    def read_robot_callback(self, msg):
        for i in range(1,4):
            attribute = f"distances_r{i}"
            try:
                distances = getattr(msg, attribute)
                if len(distances) > 0:  # Only update if we have data
                    self.robot_distances[i-1] = distances
            except AttributeError:
                self.get_logger().warn(f"Missing attribute {attribute} in robot distance message")
    
    def gaussian_filtering(self):
        """Perform scalar Kalman filtering on distance measurements (model-free)"""
        
        if not self.has_measurements():
            self.get_logger().debug("No measurements available for filtering")
            return
        
        # Get current raw measurements
        raw_measurements = self.get_measurement_vector()
        
        # Apply scalar Kalman filter to each distance measurement
        for i in range(self.total_measurements):
            if raw_measurements[i] > 0.01:  # Valid measurement (not out-of-range)
                self.filter_single_distance(i, raw_measurements[i])
        
        # Publish filtered results
        self.publish_filtered_results()
    
    def filter_single_distance(self, measurement_idx, raw_distance):
        """Apply scalar Kalman filter to a single distance measurement"""
        
        # Current estimate and covariance
        x = self.distance_estimates[measurement_idx]  # Current distance estimate
        P = self.distance_covariances[measurement_idx]  # Current uncertainty
        
        # PREDICTION STEP (simple model: distance evolves slowly)
        # Predicted distance (assume small change)
        x_pred = x
        
        # Predicted covariance (add process noise)
        P_pred = P + self.distance_process_noise
        
        # UPDATE STEP
        # Measurement noise for this specific measurement
        R = self.measurement_variances[measurement_idx]
        
        # Kalman gain
        K = P_pred / (P_pred + R)
        
        # Innovation (difference between measurement and prediction)
        innovation = raw_distance - x_pred
        
        # Update estimate
        x_new = x_pred + K * innovation
        
        # Update covariance
        P_new = (1 - K) * P_pred
        
        # Store updated values
        self.distance_estimates[measurement_idx] = x_new
        self.distance_covariances[measurement_idx] = P_new
        
        # Update measurement history for adaptive filtering
        if len(self.measurement_history[measurement_idx]) >= self.max_history_length:
            self.measurement_history[measurement_idx].pop(0)
        self.measurement_history[measurement_idx].append(raw_distance)
        
        # Log significant innovations
        if abs(innovation) > 3 * np.sqrt(R):
            self.get_logger().warn(f"Large innovation in measurement {measurement_idx}: {innovation:.3f}m")
    
    def apply_advanced_filtering(self, measurement_idx):
        """Apply more sophisticated filtering based on measurement history"""
        
        history = self.measurement_history[measurement_idx]
        if len(history) < 3:
            return  # Need more history
        
        # Detect and handle outliers using median filter
        recent_measurements = np.array(history[-5:])  # Last 5 measurements
        median_val = np.median(recent_measurements)
        mad = np.median(np.abs(recent_measurements - median_val))  # Median Absolute Deviation
        
        # If current estimate is far from median, pull it back
        current_estimate = self.distance_estimates[measurement_idx]
        if abs(current_estimate - median_val) > 3 * mad and mad > 0.01:
            # Outlier detected, use median filter result
            self.distance_estimates[measurement_idx] = median_val
            # Increase uncertainty due to outlier
            self.distance_covariances[measurement_idx] *= 1.5
            
            self.get_logger().debug(f"Outlier correction applied to measurement {measurement_idx}")
    
    def has_measurements(self):
        """Check if we have valid measurements"""
        try:
            # Check if we have at least some anchor and robot measurements
            anchor_available = any(len(distances) > 0 for distances in self.anchor_distances)
            robot_available = any(len(distances) > 0 for distances in self.robot_distances)
            return anchor_available or robot_available  # OR instead of AND for gradual startup
        except Exception as e:
            self.get_logger().debug(f"Error checking measurements: {e}")
            return False
    
    def get_measurement_vector(self):
        """Convert stored measurements to measurement vector"""
        z = np.zeros(self.total_measurements)
        idx = 0
        
        # Anchor measurements (9 total: 3 robots × 3 anchors)
        # Organization: [r0_a0, r0_a1, r0_a2, r1_a0, r1_a1, r1_a2, r2_a0, r2_a1, r2_a2]
        for robot_idx in range(3):
            for anchor_idx in range(3):
                if len(self.anchor_distances[anchor_idx]) > robot_idx:
                    distance = self.anchor_distances[anchor_idx][robot_idx]
                    # Handle out-of-range measurements (-1.0)
                    z[idx] = max(distance, 0.01) if distance > 0 else 0.01
                else:
                    z[idx] = 0.01  # Default small distance
                idx += 1
        
        # Robot-to-robot measurements (6 total: 3 robots × 2 others each)
        for robot_idx in range(3):
            if len(self.robot_distances[robot_idx]) >= 2:
                z[idx] = max(self.robot_distances[robot_idx][0], 0.01)
                z[idx + 1] = max(self.robot_distances[robot_idx][1], 0.01)
            elif len(self.robot_distances[robot_idx]) == 1:
                z[idx] = max(self.robot_distances[robot_idx][0], 0.01)
                z[idx + 1] = 0.01
            else:
                z[idx] = 0.01
                z[idx + 1] = 0.01
            idx += 2
        
        return z
    
    def publish_filtered_results(self):
        """Publish filtered distance measurements back as UWB messages"""
        
        # Create filtered anchor distances message
        anchor_msg = AnchorDist()
        anchor_msg.header.stamp = self.get_clock().now().to_msg()
        anchor_msg.header.frame_id = "kf_filtered"
        
        # Extract filtered anchor distances (first 9 measurements)
        # Organized as: [r0_a0, r0_a1, r0_a2, r1_a0, r1_a1, r1_a2, r2_a0, r2_a1, r2_a2]
        anchor_msg.distances_a1 = [
            self.distance_estimates[0],  # robot 0 to anchor 1
            self.distance_estimates[3],  # robot 1 to anchor 1
            self.distance_estimates[6]   # robot 2 to anchor 1
        ]
        anchor_msg.distances_a2 = [
            self.distance_estimates[1],  # robot 0 to anchor 2
            self.distance_estimates[4],  # robot 1 to anchor 2
            self.distance_estimates[7]   # robot 2 to anchor 2
        ]
        anchor_msg.distances_a3 = [
            self.distance_estimates[2],  # robot 0 to anchor 3
            self.distance_estimates[5],  # robot 1 to anchor 3
            self.distance_estimates[8]   # robot 2 to anchor 3
        ]
        
        self.filtered_anchor_pub.publish(anchor_msg)
        self.get_logger().debug("Published filtered anchor distances")
        
        # Create filtered robot distances message
        robot_msg = RobotDist()
        robot_msg.header.stamp = self.get_clock().now().to_msg()
        robot_msg.header.frame_id = "kf_filtered"
        
        # Extract filtered robot-to-robot distances (last 6 measurements)
        # Robot distances start at index 9 (after 9 anchor measurements)
        # Organized as: [r1_to_others, r2_to_others, r3_to_others]
        robot_msg.distances_r1 = [
            self.distance_estimates[9],   # robot 1 to robot 2 
            self.distance_estimates[10]   # robot 1 to robot 3
        ]
        robot_msg.distances_r2 = [
            self.distance_estimates[11],  # robot 2 to robot 1
            self.distance_estimates[12]   # robot 2 to robot 3
        ]
        robot_msg.distances_r3 = [
            self.distance_estimates[13],  # robot 3 to robot 1
            self.distance_estimates[14]   # robot 3 to robot 2
        ]
        
        self.filtered_robot_pub.publish(robot_msg)
        self.get_logger().debug("Published filtered robot distances")
        
        # Log filtering quality
        avg_uncertainty = np.mean(self.distance_covariances)
        num_valid_measurements = np.sum(self.distance_estimates > 0.01)
        self.get_logger().info(f"📡 Published filtered distances: {num_valid_measurements}/{self.total_measurements} valid, "
                             f"avg uncertainty: {avg_uncertainty:.4f}m")
    
    def parse_KF_parameters(self, sensor_conf_file_path):
        """Parse Kalman Filter parameters from single sensor_params.yaml file"""
        
        # Initialize parameter storage
        self.kf_params = {} # Dictionary to hold KF parameters
        self.sensor_anchor_std = [] # List to hold anchor's sensors's standard deviations
        self.sensor_robot_std = [] # List to hold robot's sensors's standard deviations

        # Set default KF parameters (no longer from separate file)
        self.kf_params = {
            'process_noise': {'distance_var': 0.01, 'rate_var': 0.1},
            'outlier_detection': {'enabled': True}
        }
        self.get_logger().info(f"Using default UKF parameters: {list(self.kf_params.keys())}")
            
        try:
            # Load sensor parameters from single YAML file
            with open(sensor_conf_file_path, 'r') as sensor_file:
                sensor_config = yaml.safe_load(sensor_file)
                uwb_sensor = sensor_config.get('UWB_sensor', {})
                
                # Extract anchor standard deviations
                for j in range(1, 4):
                    anchor_key = f"anchor {j}"
                    if anchor_key in uwb_sensor:
                        noise_model = uwb_sensor[anchor_key].get('noise model', {})
                        stddev = noise_model.get('stddev', 0.05)
                        self.sensor_anchor_std.append(stddev)
                    else:
                        self.get_logger().warn(f"Missing {anchor_key} in sensor config, using default 0.05")
                        self.sensor_anchor_std.append(0.05)
                
                # Extract robot standard deviations
                for j in range(1, 4):
                    robot_key = f"robot {j}"
                    if robot_key in uwb_sensor:
                        noise_model = uwb_sensor[robot_key].get('noise model', {})
                        stddev = noise_model.get('stddev', 0.05)
                        self.sensor_robot_std.append(stddev)
                    else:
                        self.get_logger().warn(f"Missing {robot_key} in sensor config, using default 0.05")
                        self.sensor_robot_std.append(0.05)
                        
                self.get_logger().info(f"Loaded anchor std devs from {sensor_conf_file_path}: {self.sensor_anchor_std}")
                self.get_logger().info(f"Loaded robot std devs from {sensor_conf_file_path}: {self.sensor_robot_std}")
                
        except FileNotFoundError:
            self.get_logger().error(f"Sensor config file not found: {sensor_conf_file_path}. Using default standard deviations.")
            # Use defaults if file is missing
            self.sensor_anchor_std = [0.05, 0.05, 0.05]
            self.sensor_robot_std = [0.05, 0.05, 0.05]
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error parsing sensor YAML file: {e}. Using default standard deviations.")
            # Use defaults on parsing error
            self.sensor_anchor_std = [0.05, 0.05, 0.05]
            self.sensor_robot_std = [0.05, 0.05, 0.05]
    
    def std_to_cov(self, std_list):
        """Convert list of standard deviations to covariance matrix"""
        return np.diag(np.square(std_list))
    
    
def main(args=None):
    """Main function to run the Kalman Filter node"""
    rclpy.init(args=args)
    
    kf_node = DistanceKFNode()
    
    try:
        rclpy.spin(kf_node)
    except KeyboardInterrupt:
        pass
    finally:
        kf_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

