/**
 * @file PoseEKF_node.cpp
 * @brief Per-robot Extended Kalman Filter estimating pose [x,y,theta].
 *
 * Predict: unicycle model f(x,u), u=[v_cmd, omega_imu] (see pose_dynamics.hpp).
 * Update: h(x) = [3 anchor distances, 2 neighbor distances, theta_imu] (one-step
 * Riccati recursion, gain recomputed every cycle since F_k/H_k depend on theta_k -
 * no DARE/idare solve, see plan's "Nota tecnica importante").
 *
 * One instance per robot, selected via the mandatory 'robot_id' parameter
 * (0/1/2) - this is what makes the architecture distributed: N independent
 * node instances instead of one node looping over N robots.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "int_sys_fp/msg/anchor_dist.hpp"
#include "int_sys_fp/msg/robot_dist.hpp"
#include "int_sys_fp/msg/pose_estimate_debug.hpp"
#include "pose_dynamics.hpp"

#include <array>
#include <random>
#include <cmath>
#include <tuple>

using pose_dynamics::State;
using pose_dynamics::Input;

class PoseEKF {
public:
    std::pair<State, Eigen::Matrix3d> predict(const State& x, const Eigen::Matrix3d& P,
                                               const Input& u, const Eigen::Matrix3d& Q, double dt) {
        State x_pred = pose_dynamics::f(x, u, dt);
        Eigen::Matrix3d Fm = pose_dynamics::F(x, u, dt);
        Eigen::Matrix3d P_pred = Fm * P * Fm.transpose() + Q;
        return {x_pred, P_pred};
    }

    std::tuple<State, Eigen::Matrix3d, Eigen::VectorXd, Eigen::VectorXd> update(
            const State& x_pred, const Eigen::Matrix3d& P_pred,
            const Eigen::VectorXd& z, const Eigen::MatrixXd& R,
            const std::array<Eigen::Vector2d, 3>& anchors,
            const std::array<Eigen::Vector2d, 2>& neighbors) {
        Eigen::VectorXd z_pred = pose_dynamics::h(x_pred, anchors, neighbors);
        Eigen::MatrixXd Hm = pose_dynamics::H(x_pred, anchors, neighbors);

        Eigen::VectorXd innovation = z - z_pred;
        innovation(5) = pose_dynamics::normalizeAngle(innovation(5));  // theta row

        Eigen::MatrixXd S = Hm * P_pred * Hm.transpose() + R;
        Eigen::MatrixXd K = P_pred * Hm.transpose() * S.inverse();

        State x_upd = x_pred + K * innovation;
        x_upd(2) = pose_dynamics::normalizeAngle(x_upd(2));

        Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d P_upd = (I3 - K * Hm) * P_pred;
        return {x_upd, P_upd, z_pred, innovation};
    }
};

class PoseEKFNode : public rclcpp::Node {
public:
    PoseEKFNode() : Node("pose_ekf_node") {
        declare_parameter<int>("robot_id", -1);
        robot_id_ = get_parameter("robot_id").as_int();
        if (robot_id_ < 0 || robot_id_ > 2) {
            RCLCPP_FATAL(get_logger(),
                "robot_id parameter not set (or out of range) - must be 0, 1 or 2. "
                "This node must be launched with an explicit robot_id.");
            throw std::runtime_error("pose_ekf_node: missing/invalid robot_id parameter");
        }

        topic_prefix_ = pose_dynamics::topicPrefix(robot_id_);
        neighbor_ids_ = pose_dynamics::neighborIds(robot_id_);

        node_freq_ = 50.0;
        dt_ = 1.0 / node_freq_;

        load_params();

        x_ = State::Zero();
        P_ = p0_scale_ * Eigen::Matrix3d::Identity();
        neighbor_pos_[0] = Eigen::Vector2d::Zero();
        neighbor_pos_[1] = Eigen::Vector2d::Zero();

        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            topic_prefix_ + "/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) { last_v_ = msg->linear.x; });

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            topic_prefix_ + "/imu", 10,
            std::bind(&PoseEKFNode::imu_callback, this, std::placeholders::_1));

        anchor_sub_ = create_subscription<int_sys_fp::msg::AnchorDist>(
            "/uwb/anchor_distances", 10,
            std::bind(&PoseEKFNode::anchor_callback, this, std::placeholders::_1));

        robot_dist_sub_ = create_subscription<int_sys_fp::msg::RobotDist>(
            "/uwb/robot_distances", 10,
            std::bind(&PoseEKFNode::robot_dist_callback, this, std::placeholders::_1));

        for (int k = 0; k < 2; ++k) {
            std::string prefix = pose_dynamics::topicPrefix(neighbor_ids_[k]);
            int idx = k;
            neighbor_subs_[k] = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                prefix + "/pose_estimate", 10,
                [this, idx](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                    neighbor_pos_[idx] = Eigen::Vector2d(msg->pose.pose.position.x, msg->pose.pose.position.y);
                });
        }

        pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            topic_prefix_ + "/pose_estimate", 10);

        debug_pub_ = create_publisher<int_sys_fp::msg::PoseEstimateDebug>(
            topic_prefix_ + "/pose_debug", 10);

        timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(dt_ * 1000)),
                                    std::bind(&PoseEKFNode::step, this));

        RCLCPP_INFO(get_logger(), "PoseEKF for robot_id=%d started (topic prefix '%s')",
                    robot_id_, topic_prefix_.c_str());
    }

private:
    void load_params() {
        Q_ = Eigen::Matrix3d::Identity() * 0.001;
        p0_scale_ = 10.0;
        imu_orientation_noise_std_ = 0.02;
        anchor_positions_ = {Eigen::Vector2d(0, 0), Eigen::Vector2d(10, 0), Eigen::Vector2d(0, 10)};
        sensor_anchor_std_ = {0.05, 0.05, 0.05};
        sensor_robot_std_ = {0.05, 0.05, 0.05};

        try {
            std::string share = ament_index_cpp::get_package_share_directory("int_sys_fp");

            YAML::Node pf = YAML::LoadFile(share + "/pose_filter_params.yaml")["pose_filter"];
            Q_(0, 0) = pf["Q"]["x_var"].as<double>(0.001);
            Q_(1, 1) = pf["Q"]["y_var"].as<double>(0.001);
            Q_(2, 2) = pf["Q"]["theta_var"].as<double>(0.001);
            p0_scale_ = pf["initial_covariance"]["p0_scale"].as<double>(10.0);
            imu_orientation_noise_std_ = pf["imu_orientation_noise_std"].as<double>(0.02);

            YAML::Node sp = YAML::LoadFile(share + "/sensor_params.yaml")["UWB_sensor"];
            for (int i = 0; i < 3; ++i) {
                auto anchor_node = sp["anchor " + std::to_string(i + 1)];
                auto pos = anchor_node["position"];
                anchor_positions_[i] = Eigen::Vector2d(pos[0].as<double>(), pos[1].as<double>());
                sensor_anchor_std_[i] = anchor_node["noise model"]["stddev"].as<double>(0.05);
                sensor_robot_std_[i] =
                    sp["robot " + std::to_string(i + 1)]["noise model"]["stddev"].as<double>(0.05);
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Failed to load pose filter params: %s, using defaults", e.what());
        }

        RCLCPP_INFO(get_logger(), "Q=diag(%.5f,%.5f,%.5f) P0_scale=%.2f imu_theta_std=%.4f",
                    Q_(0, 0), Q_(1, 1), Q_(2, 2), p0_scale_, imu_orientation_noise_std_);
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        last_omega_ = msg->angular_velocity.z;
        last_R_omega_ = msg->angular_velocity_covariance[8];  // real, plugin-populated (see plan notes)

        // orientation_covariance is never populated by gazebo_ros_imu_sensor (TODO in
        // its source) - the field is near ground-truth, so inject synthetic noise
        // before using it as a correction, keeping theta estimation non-trivial.
        double siny_cosp = 2.0 * (msg->orientation.w * msg->orientation.z +
                                   msg->orientation.x * msg->orientation.y);
        double cosy_cosp = 1.0 - 2.0 * (msg->orientation.y * msg->orientation.y +
                                         msg->orientation.z * msg->orientation.z);
        double yaw_true = std::atan2(siny_cosp, cosy_cosp);

        std::normal_distribution<double> noise(0.0, imu_orientation_noise_std_);
        last_theta_imu_ = pose_dynamics::normalizeAngle(yaw_true + noise(rng_));
        have_imu_ = true;
    }

    void anchor_callback(const int_sys_fp::msg::AnchorDist::SharedPtr msg) {
        const std::array<const std::vector<double>*, 3> rows = {
            &msg->distances_a1, &msg->distances_a2, &msg->distances_a3};
        for (int k = 0; k < 3; ++k) {
            if (robot_id_ < static_cast<int>(rows[k]->size())) {
                double d = (*rows[k])[robot_id_];
                if (d > 0.0) z_anchor_[k] = d;
            }
        }
        have_anchor_ = true;
    }

    void robot_dist_callback(const int_sys_fp::msg::RobotDist::SharedPtr msg) {
        const std::vector<double>* row = nullptr;
        switch (robot_id_) {
            case 0: row = &msg->distances_r1; break;
            case 1: row = &msg->distances_r2; break;
            case 2: row = &msg->distances_r3; break;
        }
        if (row && row->size() == 2) {
            if ((*row)[0] > 0.0) z_neighbor_[0] = (*row)[0];
            if ((*row)[1] > 0.0) z_neighbor_[1] = (*row)[1];
            have_robot_dist_ = true;
        }
    }

    void step() {
        if (!have_anchor_ || !have_robot_dist_) return;

        Input u{last_v_, last_omega_};

        // Input noise (measured omega) propagates into theta process noise: an error
        // of variance R_omega in the rate integrates to variance R_omega*dt^2 in theta.
        Eigen::Matrix3d Q_eff = Q_;
        Q_eff(2, 2) += last_R_omega_ * dt_ * dt_;

        auto [x_pred, P_pred] = ekf_.predict(x_, P_, u, Q_eff, dt_);

        Eigen::VectorXd z(6);
        z << z_anchor_[0], z_anchor_[1], z_anchor_[2], z_neighbor_[0], z_neighbor_[1],
             have_imu_ ? last_theta_imu_ : x_pred(2);

        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(6, 6);
        for (int k = 0; k < 3; ++k) R(k, k) = sensor_anchor_std_[k] * sensor_anchor_std_[k];
        for (int k = 0; k < 2; ++k) {
            double s = sensor_robot_std_[neighbor_ids_[k]];
            R(3 + k, 3 + k) = s * s;
        }
        R(5, 5) = imu_orientation_noise_std_ * imu_orientation_noise_std_;

        auto [x_upd, P_upd, z_pred, innovation] = ekf_.update(x_pred, P_pred, z, R, anchor_positions_, neighbor_pos_);
        x_ = x_upd;
        P_ = P_upd;

        publish();
        publish_debug(z, z_pred, innovation, Q_eff, R);
    }

    void publish() {
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        msg.header.stamp = get_clock()->now();
        msg.header.frame_id = "world";
        msg.pose.pose.position.x = x_(0);
        msg.pose.pose.position.y = x_(1);
        msg.pose.pose.orientation.z = std::sin(x_(2) / 2.0);
        msg.pose.pose.orientation.w = std::cos(x_(2) / 2.0);

        for (auto& c : msg.pose.covariance) c = 0.0;
        msg.pose.covariance[0] = P_(0, 0);
        msg.pose.covariance[1] = P_(0, 1);
        msg.pose.covariance[6] = P_(1, 0);
        msg.pose.covariance[7] = P_(1, 1);
        msg.pose.covariance[35] = P_(2, 2);

        pose_pub_->publish(msg);
    }

    void publish_debug(const Eigen::VectorXd& z, const Eigen::VectorXd& z_pred,
                        const Eigen::VectorXd& innovation, const Eigen::Matrix3d& Q_eff,
                        const Eigen::MatrixXd& R) {
        auto msg = int_sys_fp::msg::PoseEstimateDebug();
        msg.header.stamp = get_clock()->now();
        msg.header.frame_id = "world";
        msg.robot_id = static_cast<uint8_t>(robot_id_);
        msg.x = x_(0);
        msg.y = x_(1);
        msg.theta = x_(2);

        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                msg.p[r * 3 + c] = P_(r, c);

        for (int k = 0; k < 6; ++k) {
            msg.z[k] = z(k);
            msg.z_pred[k] = z_pred(k);
            msg.innovation[k] = innovation(k);
            msg.r_diag[k] = R(k, k);
        }
        for (int k = 0; k < 3; ++k) msg.q_diag[k] = Q_eff(k, k);

        msg.have_imu = have_imu_;
        msg.v_cmd = last_v_;
        msg.omega_imu = last_omega_;
        msg.dt = dt_;

        debug_pub_->publish(msg);
    }

    int robot_id_;
    std::string topic_prefix_;
    std::array<int, 2> neighbor_ids_;

    double node_freq_, dt_;
    Eigen::Matrix3d Q_;
    double p0_scale_;
    double imu_orientation_noise_std_;
    std::array<Eigen::Vector2d, 3> anchor_positions_;
    std::array<double, 3> sensor_anchor_std_;
    std::array<double, 3> sensor_robot_std_;

    State x_;
    Eigen::Matrix3d P_;
    PoseEKF ekf_;

    double last_v_ = 0.0;
    double last_omega_ = 0.0;
    double last_R_omega_ = 1e-6;
    double last_theta_imu_ = 0.0;
    bool have_imu_ = false;

    std::array<double, 3> z_anchor_ = {1.0, 1.0, 1.0};
    std::array<double, 2> z_neighbor_ = {1.0, 1.0};
    std::array<Eigen::Vector2d, 2> neighbor_pos_;
    bool have_anchor_ = false;
    bool have_robot_dist_ = false;

    std::mt19937 rng_{std::random_device{}()};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<int_sys_fp::msg::AnchorDist>::SharedPtr anchor_sub_;
    rclcpp::Subscription<int_sys_fp::msg::RobotDist>::SharedPtr robot_dist_sub_;
    std::array<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr, 2> neighbor_subs_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<int_sys_fp::msg::PoseEstimateDebug>::SharedPtr debug_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<PoseEKFNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("pose_ekf_node"), "Fatal error: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
