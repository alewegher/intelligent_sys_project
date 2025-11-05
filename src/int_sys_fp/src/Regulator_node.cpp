#include <rclcpp/rclcpp.hpp>
#include<iostream>
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
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

            // Create publishers 

            kf_filtered_robot_pose_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("kf_filtered_robot_pose", 15);
            kf_filtered_centroid_pose_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("kf_filtered_centroid_pose", 15);
            centroid_pose_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("centroid_pose", 15);
            effort_robot1_pub = this->create_publisher<geometry_msgs::msg::Twist>("effort_robot1", 15);
            effort_robot2_pub = this->create_publisher<geometry_msgs::msg::Twist>("effort_robot2", 15);
            effort_robot3_pub = this->create_publisher<geometry_msgs::msg::Twist>("effort_robot3", 15);

            // Create subscribers for UWB data
            // std::bind allows us to access the class methods without explicitly calling the constructor of the class --> no need to create an istance of the class to use the methods 
            uwb_anchor_sub = this->create_subscription<int_sys_fp::msg::AnchorDist>(
                "/uwb/anchor_distances", 10,
                std::bind(&ControllerClass::UWB_read_anchor_callback, this, std::placeholders::_1));
            
            uwb_robot_sub = this->create_subscription<int_sys_fp::msg::RobotDist>(
                "/uwb/robot_distances", 10,
                std::bind(&ControllerClass::UWB_robot_callback, this, std::placeholders::_1));

            // initialize robot positions 

            robot_positions.resize(3);
            robot_velocities.resize(3);
            robot_efforts.resize(3);

            // continue next ...



        }

        void init(){
            
        }

        void update(){}

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

        void compute_positions(std::vector<Eigen::Vector3d> & anchor_distances, std::vector<Eigen::Vector3d> & anchor_positions, std::vector<Eigen::Vector2d> & robot_distances){

            if(anchor_distances.size() != 3 || anchor_positions.size() != 3){

                RCLCPP_WARN(this->get_logger(),"Invalid anchor distances or positions format: expected 3 distances vectors and 3 positions vectors");

                return;

            } 

            // initialize return vector

            std::vector<Eigen::Vector3d> robot_postions(3,Eigen::Vector3d::Zero());

            // compute the angles for the trilateration

            Eigen::Vector3d GF_POSE = Eigen::Vector3d::Zero();

            // compute the USEFUL distances of the anchors 

            double a1a3 = (anchor_positions[2]-anchor_positions[0]).norm();

            double a1a2 = (anchor_positions[1]-anchor_positions[0]).norm();

            Eigen::Vector3d angle_1 = solve_triangle( // first triangle to solve 
                anchor_distances[0][0],
                anchor_distances[2][0],
                a1a2
            ); 

            Eigen::Vector3d angle_2 = solve_triangle( // second triangle to solve
                a1a3,
                anchor_distances[2][0],
                anchor_distances[0][0]
            ); 

            Eigen::Vector3d angle_3 = solve_triangle( // third triangle to solve
                anchor_distances[2][1],
                mid(robot_distances[0][0],robot_distances[0][1]), // make the mean of the two noisy distances
                anchor_distances[2][0]
            ); 

            Eigen::Vector3d angle_4 = solve_triangle( // fourth triangle to solve
                anchor_distances[1][0],
                mid(robot_distances[0][1],robot_distances[2][0]), // make the mean of the two noisy distances
                anchor_distances[1][2]
            ); 

            // extract useful angles to compute frame transformations 
            
            // sarà da bestemmiare capire se sono giusti, troppi triangoli non capisco na sega !!!
            double theta_r1 = angle_1[0];
            double theta_r2 = angle_2[0]+ angle_1[2] - PI/2.0 ;
            double theta_r3 = PI - angle_4[0]-angle_1[1]; 

            // compute the rotation matrices for each robot 

            Eigen::Rotation2Dd Rzr1, Rzr2, Rzr3;

            Rzr1 = Eigen::Rotation2Dd(theta_r1);
            Rzr2 = Eigen::Rotation2Dd(theta_r2);
            Rzr3 = Eigen::Rotation2Dd(theta_r3);

            // define the robot positions in the implicit rotated frame and apply the rotations to go in global frame

            Eigen::Vector2d r1_pos = apply_rotation(Eigen::Vector2d(anchor_distances[0][0], 0.0), Rzr1);
            Eigen::Vector2d r2_pos = apply_rotation(Eigen::Vector2d(anchor_distances[2][1], 0.0), Rzr2);
            Eigen::Vector2d r3_pos = apply_rotation(Eigen::Vector2d(anchor_distances[1][2], 0.0), Rzr3);

        }

        double mid(double a, double b){
            return (a+b)/2.0;
        }

        Eigen::Vector2d apply_rotation(const Eigen::Vector2d & vec ,Eigen::Rotation2Dd & rotation){
            // Apply the rotation to the vector
            return rotation * vec;
        }

        void UWB_read_anchor_callback(const int_sys_fp::msg::AnchorDist::SharedPtr msg){
            // Callback per dati UWB anchor distances
            // msg contiene: distances_a1, distances_a2, distances_a3
            // ogni array contiene le distanze di tutti i robot a quell'anchor
            
            if(msg->distances_a1.size() != 3 || msg->distances_a2.size() != 3 || msg->distances_a3.size() != 3) {
                RCLCPP_WARN(this->get_logger(), "Expected 3 distances per anchor, got a1:%zu a2:%zu a3:%zu", 
                           msg->distances_a1.size(), msg->distances_a2.size(), msg->distances_a3.size());
                return;
            }
            
            // Process trilaterazione per ogni robot
            for(int robot_idx = 0; robot_idx < 3; robot_idx++) {
                double d1 = msg->distances_a1[robot_idx]; // Distance to anchor 1
                double d2 = msg->distances_a2[robot_idx]; // Distance to anchor 2  
                double d3 = msg->distances_a3[robot_idx]; // Distance to anchor 3
                
                // TODO: Implementa trilaterazione vera qui
                // Eigen::Vector3d pos = solve_trilateration(d1, d2, d3, anchor_positions);
                // robot_positions[robot_idx] = pos;
                
                RCLCPP_DEBUG(this->get_logger(), "Robot %d distances: [%.3f, %.3f, %.3f]", 
                           robot_idx, d1, d2, d3);
            }
        }

        void UWB_robot_callback(const int_sys_fp::msg::RobotDist::SharedPtr msg){
            // Callback per dati UWB robot-to-robot distances
            // msg contiene: distances_r1, distances_r2, distances_r3
            // ogni array contiene le distanze di quel robot verso gli altri
            
            if(msg->distances_r1.size() != 2 || msg->distances_r2.size() != 2 || msg->distances_r3.size() != 2) {
                RCLCPP_WARN(this->get_logger(), "Expected 2 distances per robot, got r1:%zu r2:%zu r3:%zu", 
                           msg->distances_r1.size(), msg->distances_r2.size(), msg->distances_r3.size());
                return;
            }
            
            // Process distanze inter-robot per controllo formazione
            RCLCPP_DEBUG(this->get_logger(), "Robot distances - R1:[%.3f,%.3f] R2:[%.3f,%.3f] R3:[%.3f,%.3f]",
                       msg->distances_r1[0], msg->distances_r1[1], 
                       msg->distances_r2[0], msg->distances_r2[1],
                       msg->distances_r3[0], msg->distances_r3[1]);
        }

        void write_effort_command_callback(){}
        
        Eigen::Vector3d compute_velocities(){
            // Placeholder for velocity computation
            // In a real implementation, this would compute the robot's velocities based on the current state
            return Eigen::Vector3d::Zero();
        }

        Eigen::Vector3d compute_accelerations(){
            // Placeholder for velocity computation
            // In a real implementation, this would compute the robot's velocities based on the current state
            return Eigen::Vector3d::Zero();
        }



    private:

        // Publishers 

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr kf_filtered_robot_pose_pub;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr kf_filtered_centroid_pose_pub;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr centroid_pose_pub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr effort_robot1_pub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr effort_robot2_pub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr effort_robot3_pub;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr error_centroid_pub;

        // Subscribers for UWB data

        rclcpp::Subscription<int_sys_fp::msg::AnchorDist>::SharedPtr uwb_anchor_sub;
        rclcpp::Subscription<int_sys_fp::msg::RobotDist>::SharedPtr uwb_robot_sub;

        // Timer for Publishing 

        rclcpp::TimerBase::SharedPtr timer_;

        // Robot states 

        std::vector<Eigen::Vector3d> robot_positions;
        std::vector<Eigen::Vector3d> robot_velocities;
        std::vector<Eigen::Vector3d> robot_efforts;
        
        // Anchor positions (should match UWB emulator config)
        std::vector<Eigen::Vector3d> anchor_positions;


};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerClass>());
    rclcpp::shutdown();
    return 0;
}