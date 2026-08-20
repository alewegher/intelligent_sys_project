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
#include <std_msgs/msg/float64_multi_array.hpp>
#include "int_sys_fp/msg/pose_estimate_debug.hpp"
#include "pose_dynamics.hpp"
#include "sensor_config.hpp"
#include "cov_fusion.hpp"

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
        P_pred = 0.5 * (P_pred + P_pred.transpose());   // kill rounding asymmetry
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

        // Joseph form, not the short (I - K H) P. P is a covariance: it must stay
        // symmetric positive semi-definite. The short form is only symmetric in exact
        // arithmetic, and any asymmetry it introduces is fed straight back into the next
        // predict (and, in the UKF, into a Cholesky). (I-KH)P(I-KH)' + K R K' is a sum of
        // two congruences of PSD matrices, so it is PSD by construction in floating point
        // too. The explicit symmetrisation then removes the last rounding asymmetry.
        Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d IKH = I3 - K * Hm;
        Eigen::Matrix3d P_upd = IKH * P_pred * IKH.transpose() + K * R * K.transpose();
        P_upd = 0.5 * (P_upd + P_upd.transpose());
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

        // Must match the noise_type given to the UWB emulator - it selects which sensor
        // YAML the measurement model and R are built from. Read before load_params().
        declare_parameter<int>("noise_type", 1);
        noise_type_ = get_parameter("noise_type").as_int();

        // 'raw' = UWB straight from the emulator; 'filtered' = routed through the legacy
        // distance KF + MAD outlier detector, which is what makes MAD causally relevant
        // to state estimation instead of a parallel baseline nothing consumes.
        uwb_source_ = declare_parameter<std::string>("uwb_source", "raw");
        if (uwb_source_ != "raw" && uwb_source_ != "filtered") {
            RCLCPP_FATAL(get_logger(), "uwb_source must be 'raw' or 'filtered', got '%s'",
                         uwb_source_.c_str());
            throw std::runtime_error("pose_ekf_node: invalid uwb_source parameter");
        }
        const bool use_filtered = (uwb_source_ == "filtered");
        anchor_topic_     = use_filtered ? "/uwb/filtered_anchor_distances" : "/uwb/anchor_distances";
        robot_dist_topic_ = use_filtered ? "/uwb/filtered_robot_distances"  : "/uwb/robot_distances";
        RCLCPP_INFO(get_logger(), "UWB source: %s (%s, %s)", uwb_source_.c_str(),
                    anchor_topic_.c_str(), robot_dist_topic_.c_str());

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
            anchor_topic_, 10,
            std::bind(&PoseEKFNode::anchor_callback, this, std::placeholders::_1));

        robot_dist_sub_ = create_subscription<int_sys_fp::msg::RobotDist>(
            robot_dist_topic_, 10,
            std::bind(&PoseEKFNode::robot_dist_callback, this, std::placeholders::_1));

        // With uwb_source:=filtered the data comes from distance_kf_node, which is only
        // launched when enable_legacy_kf:=true. If it is not running nothing ever arrives
        // and step() sits silently in its have_anchor_/have_robot_dist_ guard forever, so
        // warn explicitly instead of letting the whole system look "started but frozen".
        if (uwb_source_ == "filtered") {
            startup_watchdog_ = create_wall_timer(std::chrono::seconds(20), [this]() {
                startup_watchdog_->cancel();
                if (!have_anchor_ || !have_robot_dist_) {
                    RCLCPP_FATAL(get_logger(),
                        "uwb_source=filtered but no data on %s / %s after 20 s "
                        "(publishers seen: %zu / %zu). Is distance_kf_node running? "
                        "It requires enable_legacy_kf:=true.",
                        anchor_topic_.c_str(), robot_dist_topic_.c_str(),
                        count_publishers(anchor_topic_), count_publishers(robot_dist_topic_));
                }
            });
        }

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

        // /pose_debug always carries WHATEVER IS ON THE CONTROL PATH, so existing analysis
        // scripts keep working unchanged on sdre_ci_experimental bags. The extra topics
        // below expose the individual branches. Deliberately extra topics rather than new
        // fields in PoseEstimateDebug.msg: changing the .msg changes its type hash, and
        // bags recorded with the old definition would stop deserialising against the
        // regenerated MATLAB message package.
        debug_pub_ = create_publisher<int_sys_fp::msg::PoseEstimateDebug>(
            topic_prefix_ + "/pose_debug", 10);

        if (gain_mode_ == "sdre_ci_experimental") {
            debug_riccati_pub_ = create_publisher<int_sys_fp::msg::PoseEstimateDebug>(
                topic_prefix_ + "/pose_debug_riccati", 10);
            debug_sdre_pub_ = create_publisher<int_sys_fp::msg::PoseEstimateDebug>(
                topic_prefix_ + "/pose_debug_sdre", 10);
            debug_ci_pub_ = create_publisher<int_sys_fp::msg::PoseEstimateDebug>(
                topic_prefix_ + "/pose_debug_ci", 10);
            gain_mode_debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
                topic_prefix_ + "/gain_mode_debug", 10);
            dare_warm_ = Q_;   // first solve starts from Q; later cycles warm-start from P_prior
        }

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

            // Read the SAME sensor file the UWB emulator was given, so anchor positions
            // and R match the noise actually being generated. Reading sensor_params.yaml
            // unconditionally (as this used to) made noise_type:=2 runs inconsistent:
            // wrong anchor geometry and an R ~33x too optimistic.
            YAML::Node sp =
                YAML::LoadFile(share + sensor_config::sensorYamlName(noise_type_))["UWB_sensor"];
            for (int i = 0; i < 3; ++i) {
                auto anchor_node = sp["anchor " + std::to_string(i + 1)];
                auto pos = anchor_node["position"];
                anchor_positions_[i] = Eigen::Vector2d(pos[0].as<double>(), pos[1].as<double>());
                sensor_anchor_std_[i] = sensor_config::equivalentStd(anchor_node["noise model"]);
                sensor_robot_std_[i] =
                    sensor_config::equivalentStd(sp["robot " + std::to_string(i + 1)]["noise model"]);
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Failed to load pose filter params: %s, using defaults", e.what());
        }

        RCLCPP_INFO(get_logger(), "Q=diag(%.5f,%.5f,%.5f) P0_scale=%.2f imu_theta_std=%.4f",
                    Q_(0, 0), Q_(1, 1), Q_(2, 2), p0_scale_, imu_orientation_noise_std_);
        RCLCPP_INFO(get_logger(), "Sensor config: %s -> anchor_std=%.4f robot_std=%.4f",
                    sensor_config::sensorYamlName(noise_type_).c_str() + 1,
                    sensor_anchor_std_[0], sensor_robot_std_[0]);

        load_gain_mode_params();
    }

    /// gain_mode defaults come from the YAML but can be overridden per-launch by the ROS
    /// parameter, which is what finally makes the (previously dead) YAML key live.
    void load_gain_mode_params() {
        std::string gm_yaml = "one_step_riccati";
        try {
            std::string share = ament_index_cpp::get_package_share_directory("int_sys_fp");
            YAML::Node pf = YAML::LoadFile(share + "/pose_filter_params.yaml")["pose_filter"];
            if (pf["gain_mode"]) gm_yaml = pf["gain_mode"].as<std::string>("one_step_riccati");
        } catch (const std::exception&) { /* keep the default */ }

        gain_mode_ = declare_parameter<std::string>("gain_mode", gm_yaml);
        if (gain_mode_ != "one_step_riccati" && gain_mode_ != "sdre_ci_experimental") {
            RCLCPP_FATAL(get_logger(), "gain_mode must be 'one_step_riccati' or "
                         "'sdre_ci_experimental', got '%s'", gain_mode_.c_str());
            throw std::runtime_error("pose_ekf_node: invalid gain_mode parameter");
        }
        ci_weight_      = declare_parameter<double>("ci_weight", 0.5);
        ci_weight_      = std::min(1.0, std::max(0.0, ci_weight_));
        ci_weight_mode_ = declare_parameter<std::string>("ci_weight_mode", "fixed");
        ci_feedback_    = declare_parameter<bool>("ci_feedback", true);
        sdre_cov_mode_  = declare_parameter<std::string>("sdre_cov_mode", "propagated");
        dare_opts_.max_iters = declare_parameter<int>("dare_max_iters", 500);
        dare_opts_.tol       = declare_parameter<double>("dare_tol", 1e-9);

        if (ci_weight_mode_ != "fixed" && ci_weight_mode_ != "min_trace") {
            RCLCPP_FATAL(get_logger(), "ci_weight_mode must be 'fixed' or 'min_trace', got '%s'",
                         ci_weight_mode_.c_str());
            throw std::runtime_error("pose_ekf_node: invalid ci_weight_mode parameter");
        }
        if (sdre_cov_mode_ != "propagated" && sdre_cov_mode_ != "steady_state") {
            RCLCPP_FATAL(get_logger(), "sdre_cov_mode must be 'propagated' or 'steady_state', "
                         "got '%s'", sdre_cov_mode_.c_str());
            throw std::runtime_error("pose_ekf_node: invalid sdre_cov_mode parameter");
        }

        if (gain_mode_ == "sdre_ci_experimental") {
            RCLCPP_INFO(get_logger(), "gain_mode=sdre_ci_experimental ci_weight=%.3f mode=%s "
                        "sdre_cov=%s feedback=%s", ci_weight_, ci_weight_mode_.c_str(),
                        sdre_cov_mode_.c_str(), ci_feedback_ ? "closed-loop" : "shadow");
            if (ci_weight_mode_ == "min_trace" && sdre_cov_mode_ == "propagated") {
                // K_A minimises the posterior covariance for this prior by definition, so
                // P_A <= P_B in the Loewner order, so trace(P_ci(w)) decreases monotonically
                // in w and the minimum always sits at w = 1 - i.e. the plain EKF. Correct,
                // but a degenerate knob: pair min_trace with sdre_cov_mode:=steady_state.
                RCLCPP_WARN(get_logger(),
                    "ci_weight_mode=min_trace with sdre_cov_mode=propagated degenerates to "
                    "w=1 (the plain one-step Riccati estimate). Use "
                    "sdre_cov_mode:=steady_state for a non-trivial optimal weight.");
            }
        }
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

        // One timestamp for every message this cycle emits, so the pose_debug* topics can
        // be joined on header.stamp in MATLAB. Calling now() per publish would give each
        // branch a slightly different stamp and make them unjoinable.
        const rclcpp::Time cycle_stamp = get_clock()->now();

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

        if (gain_mode_ == "one_step_riccati") {
            x_ = x_upd;
            P_ = P_upd;
            publish(cycle_stamp);
            publish_debug(debug_pub_, cycle_stamp, x_, P_, z, z_pred, innovation, Q_eff, R);
            return;
        }

        // ---- sdre_ci_experimental --------------------------------------------------
        // Estimate B uses the STEADY-STATE gain from the DARE, frozen at exactly the same
        // operating point the classic branch just used, and the SAME innovation - only
        // the gain differs, so the comparison is apples-to-apples.
        Eigen::Matrix<double, 3, 3> A = pose_dynamics::F(x_, u, dt_);
        Eigen::Matrix<double, 6, 3> C = pose_dynamics::H(x_pred, anchor_positions_, neighbor_pos_);
        Eigen::Matrix<double, 6, 6> R6 = R;

        auto dare = cov_fusion::solveFilterDare<3, 6>(A, C, Q_eff, R6, dare_warm_, dare_opts_);

        bool fused_ok = false;
        cov_fusion::CiResult ci;
        State x_sdre = x_upd;
        Eigen::Matrix3d P_sdre = P_upd;

        if (dare.converged) {
            dare_warm_ = dare.P_prior;   // warm start: keeps the next solve to a few iterations

            x_sdre = x_pred + dare.K * innovation;
            if (x_sdre.allFinite() && std::abs(x_sdre(2)) < 1e6) {
                x_sdre(2) = pose_dynamics::normalizeAngle(x_sdre(2));

                if (sdre_cov_mode_ == "steady_state") {
                    P_sdre = dare.P_post;
                } else {
                    // 'propagated': actual covariance of estimate B given the ACTUAL current
                    // prior, in Joseph form so it stays PSD. This is the honest choice and it
                    // makes CI meaningful (both covariances refer to the same prior) - but see
                    // the note on ci_weight_mode below.
                    const Eigen::Matrix3d IKC = Eigen::Matrix3d::Identity() - dare.K * C;
                    P_sdre = IKC * P_pred * IKC.transpose()
                             + dare.K * R6 * dare.K.transpose();
                }

                ci = cov_fusion::fuseCI(x_upd, P_upd, x_sdre, P_sdre,
                                        ci_weight_, ci_weight_mode_ == "min_trace",
                                        &pose_dynamics::normalizeAngle);
                fused_ok = ci.ok;
            }
        }

        if (!fused_ok) {
            // Any numerical failure degrades to the classic estimate, so SDRE-CI can never
            // do worse than the plain filter because of a solver problem. dare_ok on the
            // diagnostics topic makes how often this happens measurable.
            dare_warm_ = P_pred;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "SDRE/CI unavailable this cycle (dare converged=%d iters=%d) - "
                "falling back to the one-step Riccati estimate",
                static_cast<int>(dare.converged), dare.iterations);
        }

        // ci_feedback:=false is "shadow mode": everything is computed and published, but
        // the control path keeps the classic estimate. That makes all three estimators
        // comparable on an IDENTICAL trajectory and measurement stream, which closed loop
        // cannot do (there each estimator steers the robots differently).
        if (fused_ok && ci_feedback_) {
            x_ = ci.x;
            P_ = ci.P;
        } else {
            x_ = x_upd;
            P_ = P_upd;
        }

        publish(cycle_stamp);
        publish_debug(debug_pub_, cycle_stamp, x_, P_, z, z_pred, innovation, Q_eff, R);
        publish_debug(debug_riccati_pub_, cycle_stamp, x_upd, P_upd, z, z_pred, innovation, Q_eff, R);
        publish_debug(debug_sdre_pub_, cycle_stamp, x_sdre, P_sdre, z, z_pred, innovation, Q_eff, R);
        if (fused_ok) {
            publish_debug(debug_ci_pub_, cycle_stamp, ci.x, ci.P, z, z_pred, innovation, Q_eff, R);
        }

        std_msgs::msg::Float64MultiArray diag;
        diag.data = {fused_ok ? ci.w : 1.0,
                     static_cast<double>(dare.iterations),
                     dare.converged ? 1.0 : 0.0,
                     dare.residual};
        gain_mode_debug_pub_->publish(diag);
    }

    void publish(const rclcpp::Time& stamp) {
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        msg.header.stamp = stamp;
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

    /// Takes the publisher, state and covariance explicitly so the same routine can emit
    /// the classic, SDRE and fused estimates on their own topics.
    void publish_debug(const rclcpp::Publisher<int_sys_fp::msg::PoseEstimateDebug>::SharedPtr& pub,
                        const rclcpp::Time& stamp,
                        const State& x, const Eigen::Matrix3d& P,
                        const Eigen::VectorXd& z, const Eigen::VectorXd& z_pred,
                        const Eigen::VectorXd& innovation, const Eigen::Matrix3d& Q_eff,
                        const Eigen::MatrixXd& R) {
        auto msg = int_sys_fp::msg::PoseEstimateDebug();
        msg.header.stamp = stamp;
        msg.header.frame_id = "world";
        msg.robot_id = static_cast<uint8_t>(robot_id_);
        msg.x = x(0);
        msg.y = x(1);
        msg.theta = x(2);

        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                msg.p[r * 3 + c] = P(r, c);

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

        pub->publish(msg);
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
    int noise_type_ = 1;  // 1 = Gaussian, 2 = Uniform (must match the UWB emulator)
    std::string uwb_source_ = "raw";
    std::string anchor_topic_;
    std::string robot_dist_topic_;
    rclcpp::TimerBase::SharedPtr startup_watchdog_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<int_sys_fp::msg::AnchorDist>::SharedPtr anchor_sub_;
    rclcpp::Subscription<int_sys_fp::msg::RobotDist>::SharedPtr robot_dist_sub_;
    std::array<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr, 2> neighbor_subs_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<int_sys_fp::msg::PoseEstimateDebug>::SharedPtr debug_pub_;
    rclcpp::Publisher<int_sys_fp::msg::PoseEstimateDebug>::SharedPtr debug_riccati_pub_;
    rclcpp::Publisher<int_sys_fp::msg::PoseEstimateDebug>::SharedPtr debug_sdre_pub_;
    rclcpp::Publisher<int_sys_fp::msg::PoseEstimateDebug>::SharedPtr debug_ci_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gain_mode_debug_pub_;

    // gain_mode / SDRE-CI configuration and state
    std::string gain_mode_ = "one_step_riccati";
    std::string ci_weight_mode_ = "fixed";
    std::string sdre_cov_mode_ = "propagated";
    double ci_weight_ = 0.5;
    bool ci_feedback_ = true;
    cov_fusion::DareOptions dare_opts_;
    Eigen::Matrix3d dare_warm_ = Eigen::Matrix3d::Identity();
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
