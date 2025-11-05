#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/imu.hpp>   
#include<geometry_msgs/msg/twist.hpp>
#include "int_sys_fp/msg/robot_dist.hpp"
#include "int_sys_fp/msg/anchor_dist.hpp"
#include <Eigen/Dense>
#include<Eigen/Cholesky>
#include <yaml-cpp/yaml.h>
#include <iostream>

using namespace Eigen;


namespace UKF{

    MatrixXd computeSigmaPoints(const VectorXd& x, const MatrixXd& P, double lambda) {
        int n = x.size();
        MatrixXd sigmaPoints(n, 2 * n + 1);
        MatrixXd A = ((lambda + n) * P).llt().matrixL();

        sigmaPoints.col(0) = x;
        for (int i = 0; i < n; i++) {
            sigmaPoints.col(i + 1) = x + A.col(i);
            sigmaPoints.col(i + 1 + n) = x - A.col(i);
        }
        return sigmaPoints;
    }

    VectorXd computeWeights(int n, double lambda) {
        VectorXd weights(2 * n + 1);
        weights(0) = lambda / (lambda + n);
        for (int i = 1; i < 2 * n + 1; i++) {
            weights(i) = 0.5 / (n + lambda);
        }
        return weights;
    }

    MatrixXd compute_cov_matrix(VectorXd &x, const VectorXd &x_mean){
        VectorXd delta = x-x_mean;
        return delta * delta.transpose();
    }

    VectorXd 
}