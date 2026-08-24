#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Dense>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "int_sys_fp/msg/fsm_state.hpp"
#include "pose_dynamics.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

# define PI 3.14159265358979323846

using namespace Eigen;

// Per-robot distributed controller: one instance per robot (selected via the
// mandatory 'robot_id' parameter), each with its own local FSM. Robots
// exchange only pose estimates (from pose_ekf_node/pose_ukf_node) and FSM
// state broadcasts - there is no central node computing commands for all
// three robots anymore.
class ControllerClass : public rclcpp::Node{

    public:

        ControllerClass() : Node("controller_node")
        {
            declare_parameter<int>("robot_id", -1);
            robot_id_ = get_parameter("robot_id").as_int();
            if(robot_id_ < 0 || robot_id_ > 2) {
                RCLCPP_FATAL(this->get_logger(),
                    "robot_id parameter not set (or out of range) - must be 0, 1 or 2. "
                    "This node must be launched with an explicit robot_id.");
                throw std::runtime_error("controller_node: missing/invalid robot_id parameter");
            }
            topic_prefix_ = pose_dynamics::topicPrefix(robot_id_);
            neighbor_ids_ = pose_dynamics::neighborIds(robot_id_);

            // Publisher for this robot's own cmd_vel only
            cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(topic_prefix_ + "/cmd_vel", 10);

            // Debug/monitoring publishers (topic-prefixed so the 3 instances don't collide)
            tracking_error_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(topic_prefix_ + "/tracking_error", 10);
            desired_trajectory_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(topic_prefix_ + "/desired_trajectory_array", 10);
            centroid_position_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(topic_prefix_ + "/centroid_position", 10);

            // FSM state broadcast (distributed consensus, see control_loop)
            fsm_state_pub = this->create_publisher<int_sys_fp::msg::FsmState>(topic_prefix_ + "/fsm_state", 10);
            for(int k = 0; k < 2; k++) {
                int idx = k;
                std::string prefix = pose_dynamics::topicPrefix(neighbor_ids_[k]);
                fsm_neighbor_subs_[k] = this->create_subscription<int_sys_fp::msg::FsmState>(
                    prefix + "/fsm_state", 10,
                    [this, idx](const int_sys_fp::msg::FsmState::SharedPtr msg) {
                        neighbor_phase_[idx] = msg->phase;
                        neighbor_fsm_error_[idx] = msg->formation_error;
                        have_neighbor_fsm_[idx] = true;
                    });
            }

            // Subscriber for desired trajectory from planner (shared reference, broadcast)
            trajectory_sub = this->create_subscription<geometry_msgs::msg::Pose>(
                "/desired_trajectory", 10,
                std::bind(&ControllerClass::trajectory_callback, this, std::placeholders::_1));

            // Pose estimate subscriptions: own + the two neighbors (from pose_ekf_node/pose_ukf_node)
            own_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                topic_prefix_ + "/pose_estimate", 10,
                [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                    robot_positions[robot_id_] = Eigen::Vector2d(msg->pose.pose.position.x, msg->pose.pose.position.y);
                    own_theta_ = extract_yaw_from_quaternion(msg->pose.pose.orientation);
                    have_position_[robot_id_] = true;
                });
            for(int k = 0; k < 2; k++) {
                int gid = neighbor_ids_[k];
                std::string prefix = pose_dynamics::topicPrefix(gid);
                neighbor_pose_subs_[k] = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                    prefix + "/pose_estimate", 10,
                    [this, gid](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                        robot_positions[gid] = Eigen::Vector2d(msg->pose.pose.position.x, msg->pose.pose.position.y);
                        have_position_[gid] = true;
                    });
            }

            robot_positions.resize(3, Eigen::Vector2d::Zero());
            have_position_ = {false, false, false};

            // Initialize PID integral state
            integral_error = Eigen::Vector2d::Zero();

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
            tracking_error_threshold = 0.2; // meters - reduce formation weight when tracking error > threshold

            // Formation control gain
            Kf = 0.8; // Formation maintenance gain during tracking

            // Initial formation phase: stiff PD controller (critically damped)
            Kp_formation = 2.0;  // Stiff proportional gain
            Kd_formation = 2.0 * 0.98 * std::sqrt(Kp_formation);  // Critically damped: Kd = 2*ζ*sqrt(Kp), ζ=0.98

            // Formation convergence threshold (norm of distance errors)
            formation_convergence_threshold = 0.15; // meters - start transition to tracking when formation error < 15cm

            // Start in FORMATION phase (phase_blend_ = 0: pure formation control)
            current_phase = ControlPhase::FORMATION;

            RCLCPP_INFO(this->get_logger(), "=== DISTRIBUTED CONTROLLER: robot_id=%d ===", robot_id_);
            RCLCPP_INFO(this->get_logger(), "Neighbors: %d, %d", neighbor_ids_[0], neighbor_ids_[1]);
            RCLCPP_INFO(this->get_logger(), "Initial phase: FORMATION (stiff PD control)");
            RCLCPP_INFO(this->get_logger(), "Formation gains: Kp=%.2f Kd=%.2f (critically damped)", Kp_formation, Kd_formation);
            RCLCPP_INFO(this->get_logger(), "Convergence threshold: %.3f m, transition ramp: %.2f s", formation_convergence_threshold, phase_transition_ramp_s_);

            // Create control timer with frequency from YAML
            control_timer = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency)),
                std::bind(&ControllerClass::control_loop, this));

            RCLCPP_INFO(this->get_logger(), "Controller node initialized");
        }

        double load_controller_params() {
            double control_frequency = 50.0; // Default frequency

            try {
                std::string package_share = ament_index_cpp::get_package_share_directory("int_sys_fp");
                std::string yaml_path = package_share + "/config/controller.yaml";
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

                // Phase transition gain-blending window (mitigates error peaks at FSM switching)
                if(ctrl_params["phase_transition_ramp_s"]) {
                    phase_transition_ramp_s_ = ctrl_params["phase_transition_ramp_s"].as<double>();
                }

                // Formation control gain
                Kf = 0.5; // Formation gain

                RCLCPP_INFO(this->get_logger(), "Controller frequency: %.1f Hz (dt=%.4f s)",
                    control_frequency, 1.0/control_frequency);
                RCLCPP_INFO(this->get_logger(), "Controller gains: Kp=[%.2f,%.2f] Kv=[%.2f,%.2f] Ki=[%.2f,%.2f] Kf=%.2f",
                    Kp_centroid.x(), Kp_centroid.y(), Kv_centroid.x(), Kv_centroid.y(),
                    Ki_centroid.x(), Ki_centroid.y(), Kf);
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

            auto traj_msg = std_msgs::msg::Float64MultiArray();
            traj_msg.data = {desired_position.x(), desired_position.y()};
            desired_trajectory_pub->publish(traj_msg);
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
            // Symmetric: distances computed once from estimated positions (no need to
            // average two independently-measured readings like the old raw-UWB version).
            double d01 = (robot_positions[0] - robot_positions[1]).norm();
            double d02 = (robot_positions[0] - robot_positions[2]).norm();
            double d12 = (robot_positions[1] - robot_positions[2]).norm();

            double e01 = d01 - desired_distance;
            double e02 = d02 - desired_distance;
            double e12 = d12 - desired_distance;

            return std::sqrt(e01*e01 + e02*e02 + e12*e12);
        }

        Eigen::Vector2d compute_centroid_tracking_control(const Eigen::Vector2d& centroid, double dt) {
            Eigen::Vector2d position_error = desired_position - centroid;
            Eigen::Vector2d velocity_error = desired_velocity - Eigen::Vector2d::Zero(); // assuming stationary target

            Eigen::Vector2d P_term = Kp_centroid.cwiseProduct(position_error);
            Eigen::Vector2d D_term = Kv_centroid.cwiseProduct(velocity_error);

            integral_error += position_error * dt;
            integral_error.x() = std::max(-max_integral_error.x(), std::min(max_integral_error.x(), integral_error.x()));
            integral_error.y() = std::max(-max_integral_error.y(), std::min(max_integral_error.y(), integral_error.y()));

            Eigen::Vector2d I_term = Ki_centroid.cwiseProduct(integral_error);

            return P_term + I_term + D_term;
        }

        Eigen::Vector2d compute_formation_control(int robot_idx, bool use_stiff_pd) {
            // Formation control to maintain equilateral triangle
            Eigen::Vector2d formation_force = Eigen::Vector2d::Zero();

            int other1 = neighbor_ids_[0];
            int other2 = neighbor_ids_[1];

            Eigen::Vector2d diff1 = robot_positions[other1] - robot_positions[robot_idx];
            Eigen::Vector2d diff2 = robot_positions[other2] - robot_positions[robot_idx];

            double dist1 = diff1.norm();
            double dist2 = diff2.norm();

            if(dist1 < 1e-3 || dist2 < 1e-3) {
                return formation_force;
            }

            Eigen::Vector2d dir1 = diff1 / dist1;
            Eigen::Vector2d dir2 = diff2 / dist2;

            double error1 = dist1 - desired_distance;
            double error2 = dist2 - desired_distance;

            if(use_stiff_pd) {
                // FORMATION PHASE: Stiff PD control (critically damped)
                formation_force = Kp_formation * error1 * dir1 + Kp_formation * error2 * dir2;
            } else {
                // TRACKING PHASE: soft spring-like forces on each link independently
                formation_force = Kf * error1 * dir1 + Kf * error2 * dir2;
            }

            return formation_force;
        }

        void control_loop() {
            if(!have_position_[0] || !have_position_[1] || !have_position_[2]) {
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Waiting for pose estimates (own+neighbors)...");
                return;
            }

            Eigen::Vector2d centroid = compute_centroid();

            auto centroid_msg = std_msgs::msg::Float64MultiArray();
            centroid_msg.data = {centroid.x(), centroid.y()};
            centroid_position_pub->publish(centroid_msg);

            double formation_error = compute_formation_error();

            // STATE MACHINE: local debounce + neighbor AND-consensus before starting the ramp
            if(!transitioning_) {
                if(formation_error < formation_convergence_threshold) {
                    formation_stable_counter_++;
                } else {
                    formation_stable_counter_ = 0;
                }

                bool local_ready = formation_stable_counter_ >= formation_stable_required_;
                bool neighbors_ready = true;
                for(int k = 0; k < 2; k++) {
                    bool neighbor_ok = have_neighbor_fsm_[k] &&
                        (neighbor_phase_[k] == 1 || neighbor_fsm_error_[k] < formation_convergence_threshold);
                    neighbors_ready = neighbors_ready && neighbor_ok;
                }

                if(local_ready && neighbors_ready) {
                    transitioning_ = true;
                    RCLCPP_INFO(this->get_logger(), "=== PHASE TRANSITION START: FORMATION -> TRACKING (ramp %.2fs) ===",
                        phase_transition_ramp_s_);
                }
            }

            if(transitioning_) {
                phase_blend_ = std::min(1.0, phase_blend_ + control_dt / phase_transition_ramp_s_);
                if(phase_blend_ >= 1.0 && current_phase == ControlPhase::FORMATION) {
                    current_phase = ControlPhase::TRACKING;
                    RCLCPP_INFO(this->get_logger(), "=== PHASE TRANSITION COMPLETE: now in TRACKING ===");
                }
            }

            // Publish own FSM state for neighbor consensus
            auto fsm_msg = int_sys_fp::msg::FsmState();
            fsm_msg.header.stamp = this->get_clock()->now();
            fsm_msg.phase = (current_phase == ControlPhase::TRACKING) ? 1 : 0;
            fsm_msg.formation_error = formation_error;
            fsm_state_pub->publish(fsm_msg);

            // Compute control: blend formation-only and tracking control over the ramp
            Eigen::Vector2d centroid_control = Eigen::Vector2d::Zero();
            double tracking_error_magnitude = 0.0;
            if(phase_blend_ > 0.0) {
                centroid_control = phase_blend_ * compute_centroid_tracking_control(centroid, control_dt);
                tracking_error_magnitude = (desired_position - centroid).norm();
            }

            Eigen::Vector2d formation_stiff = compute_formation_control(robot_id_, true);
            Eigen::Vector2d formation_soft = compute_formation_control(robot_id_, false);

            double formation_weight = 1.0;
            if(phase_blend_ > 0.0 && tracking_error_magnitude > tracking_error_threshold) {
                formation_weight = 0.3; // reduce formation control weight when far from trajectory
            }

            Eigen::Vector2d total_control = centroid_control
                + (1.0 - phase_blend_) * formation_stiff
                + phase_blend_ * formation_weight * formation_soft;

            publish_tracking_error(centroid);

            // Convert 2D velocity to differential drive
            double desired_speed = total_control.norm();
            bool saturation_occurred = false;
            if(desired_speed > max_linear_vel) {
                desired_speed = max_linear_vel;
                saturation_occurred = true;
            }

            double desired_heading = std::atan2(total_control.y(), total_control.x());
            double heading_error = desired_heading - own_theta_;
            while(heading_error > PI) heading_error -= 2.0 * PI;
            while(heading_error < -PI) heading_error += 2.0 * PI;

            double heading_error_derivative = (heading_error - previous_heading_error_) / control_dt;
            previous_heading_error_ = heading_error;

            auto cmd_msg = geometry_msgs::msg::Twist();
            double speed_factor = std::cos(heading_error);
            double min_speed_factor = 0.5;
            cmd_msg.linear.x = desired_speed * std::max(min_speed_factor, speed_factor);
            cmd_msg.linear.y = 0.0;

            double Kp_yaw = 2.5;
            double Kd_yaw = 0.8;
            cmd_msg.angular.z = Kp_yaw * heading_error + Kd_yaw * heading_error_derivative;

            if(std::abs(cmd_msg.angular.z) > max_angular_vel) {
                cmd_msg.angular.z = (cmd_msg.angular.z > 0) ? max_angular_vel : -max_angular_vel;
                saturation_occurred = true;
            }

            cmd_vel_pub->publish(cmd_msg);

            if(saturation_occurred) {
                double back_calc_gain = 0.3;
                integral_error *= back_calc_gain;
            }
        }

        // Extract yaw angle from quaternion
        double extract_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q) {
            double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
            double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
            return std::atan2(siny_cosp, cosy_cosp);
        }

        void publish_tracking_error(const Eigen::Vector2d& centroid) {
            Eigen::Vector2d position_error = desired_position - centroid;
            auto msg = std_msgs::msg::Float64MultiArray();
            msg.data = {position_error.x(), position_error.y()};
            tracking_error_pub->publish(msg);
        }

    private:

        int robot_id_;
        std::string topic_prefix_;
        std::array<int, 2> neighbor_ids_;

        // Publishers
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tracking_error_pub;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr desired_trajectory_pub;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr centroid_position_pub;
        rclcpp::Publisher<int_sys_fp::msg::FsmState>::SharedPtr fsm_state_pub;

        // Subscribers
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr trajectory_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr own_pose_sub_;
        std::array<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr, 2> neighbor_pose_subs_;
        std::array<rclcpp::Subscription<int_sys_fp::msg::FsmState>::SharedPtr, 2> fsm_neighbor_subs_;

        rclcpp::TimerBase::SharedPtr control_timer;

        // Robot state (global-indexed: own + 2 neighbors, from pose_estimate topics)
        std::vector<Eigen::Vector2d> robot_positions;
        std::array<bool, 3> have_position_;
        double own_theta_ = 0.0;
        double previous_heading_error_ = 0.0;

        // Desired trajectory from planner
        Eigen::Vector2d desired_position;
        Eigen::Vector2d desired_velocity;

        // Formation control parameters
        double desired_distance;
        double tracking_error_threshold;

        // State machine for control phases
        enum class ControlPhase {
            FORMATION,
            TRACKING
        };
        ControlPhase current_phase;
        double formation_convergence_threshold;
        double phase_blend_ = 0.0;       // 0 = pure formation, 1 = pure tracking
        bool transitioning_ = false;
        double phase_transition_ramp_s_ = 1.5;

        // Neighbor FSM state (for AND-consensus before starting the transition ramp)
        std::array<uint8_t, 2> neighbor_phase_ = {0, 0};
        std::array<double, 2> neighbor_fsm_error_ = {1e6, 1e6};
        std::array<bool, 2> have_neighbor_fsm_ = {false, false};

        // Controller gains
        Eigen::Vector2d Kp_centroid;
        Eigen::Vector2d Kv_centroid;
        Eigen::Vector2d Ki_centroid;
        double Kf;
        double Kp_formation;
        double Kd_formation;

        Eigen::Vector2d integral_error;
        Eigen::Vector2d max_integral_error;

        double max_linear_vel;
        double max_angular_vel;
        double control_dt;

        int formation_stable_counter_ = 0;
        int formation_stable_required_ = 10; // consecutive cycles below threshold before starting the ramp

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<ControllerClass>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("controller_node"), "Fatal error: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
