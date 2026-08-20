/**
 * @file pose_dynamics.hpp
 * @brief Shared unicycle process model and UWB+IMU measurement model for the
 *        pose-based EKF/UKF (state x=[x,y,theta] per robot).
 *
 * Used identically by PoseEKF_node.cpp and UKF.cpp so that both filters share
 * the exact same f(), F(), h(), H() and therefore stay directly comparable.
 */

#ifndef INT_SYS_FP_POSE_DYNAMICS_HPP
#define INT_SYS_FP_POSE_DYNAMICS_HPP

#include <eigen3/Eigen/Dense>
#include <cmath>
#include <string>

namespace pose_dynamics {

// State: x = [x, y, theta]
using State = Eigen::Vector3d;

struct Input {
    double v;      // linear velocity (from cmd_vel)
    double omega;  // angular velocity (from IMU gyro z, see Regulator_node/pose filter nodes)
};

inline double normalizeAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// f(x,u): discrete unicycle model, Euler step
inline State f(const State& x, const Input& u, double dt) {
    State x_next;
    x_next(0) = x(0) + u.v * std::cos(x(2)) * dt;
    x_next(1) = x(1) + u.v * std::sin(x(2)) * dt;
    x_next(2) = normalizeAngle(x(2) + u.omega * dt);
    return x_next;
}

// F = df/dx, evaluated at (x,u) — used by EKF only
inline Eigen::Matrix3d F(const State& x, const Input& u, double dt) {
    Eigen::Matrix3d Fm = Eigen::Matrix3d::Identity();
    Fm(0, 2) = -u.v * std::sin(x(2)) * dt;
    Fm(1, 2) = u.v * std::cos(x(2)) * dt;
    return Fm;
}

// Measurement model: h(x) = [d_anchor0, d_anchor1, d_anchor2, d_neighbor0, d_neighbor1, theta]
// anchors: 3 anchor xy positions. neighbors: xy positions of the two other robots
// (their last received pose estimate, held fixed for this correction — decentralized
// "naive" approximation, cross-covariance with neighbors is not fused).
inline Eigen::VectorXd h(const State& x,
                          const std::array<Eigen::Vector2d, 3>& anchors,
                          const std::array<Eigen::Vector2d, 2>& neighbors) {
    Eigen::VectorXd z(6);
    Eigen::Vector2d p(x(0), x(1));
    for (int k = 0; k < 3; ++k) {
        z(k) = (p - anchors[k]).norm();
    }
    for (int k = 0; k < 2; ++k) {
        z(3 + k) = (p - neighbors[k]).norm();
    }
    z(5) = x(2);
    return z;
}

// H = dh/dx, evaluated at x (linear in theta row, analytic for the 5 range rows) — used by EKF,
// and reused by the UKF's linear-only theta handling nowhere (UKF differentiates via sigma points).
inline Eigen::MatrixXd H(const State& x,
                          const std::array<Eigen::Vector2d, 3>& anchors,
                          const std::array<Eigen::Vector2d, 2>& neighbors) {
    Eigen::MatrixXd Hm = Eigen::MatrixXd::Zero(6, 3);
    Eigen::Vector2d p(x(0), x(1));
    for (int k = 0; k < 3; ++k) {
        Eigen::Vector2d diff = p - anchors[k];
        double d = diff.norm();
        if (d < 1e-6) d = 1e-6;
        Hm(k, 0) = diff.x() / d;
        Hm(k, 1) = diff.y() / d;
    }
    for (int k = 0; k < 2; ++k) {
        Eigen::Vector2d diff = p - neighbors[k];
        double d = diff.norm();
        if (d < 1e-6) d = 1e-6;
        Hm(3 + k, 0) = diff.x() / d;
        Hm(3 + k, 1) = diff.y() / d;
    }
    Hm(5, 2) = 1.0;
    return Hm;
}

// For robot_id in {0,1,2}, returns the two neighbor ids in the same ascending,
// self-skipping order used by UWB_utils_emulator.cpp's RobotDist rows
// (robot i's row = [d(i, j) for j in 0..2, j != i, ascending]).
inline std::array<int, 2> neighborIds(int robot_id) {
    std::array<int, 2> n;
    int idx = 0;
    for (int j = 0; j < 3; ++j) {
        if (j != robot_id) n[idx++] = j;
    }
    return n;
}

// Topic namespace prefix for a given robot_id, matching the asymmetric mapping
// produced by synchronized_system.launch.py's spawn_entity calls: robot 0 is
// unnamespaced (/cmd_vel, /odom, /imu), robots 1/2 are under /tb3_2, /tb3_3.
// Shared by pose_ekf_node, pose_ukf_node and regulator_node so all three agree.
inline std::string topicPrefix(int robot_id) {
    switch (robot_id) {
        case 0: return "";
        case 1: return "/tb3_2";
        case 2: return "/tb3_3";
        default: return "";
    }
}

}  // namespace pose_dynamics

#endif  // INT_SYS_FP_POSE_DYNAMICS_HPP
