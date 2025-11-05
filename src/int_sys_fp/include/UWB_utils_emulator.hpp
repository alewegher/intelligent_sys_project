/**
 * @file UWB_utils_emulator.hpp
 * @brief Dichiarazione della classe UWBNodeClass per l'emulatore UWB.
 *
 * Questo file contiene la dichiarazione della classe UWBNodeClass
 * e le inclusion guards per evitare inclusioni multiple.
 */

#ifndef UWB_UTILS_EMULATOR_HPP
#define UWB_UTILS_EMULATOR_HPP

// Include necessari per la dichiarazione della classe
// La maggior parte degli include necessari è già nel file .cpp

#include "rclcpp/rclcpp.hpp"
#include <eigen3/Eigen/Dense> // Necessario per la definizione della classe se usa Eigen::Vector3d
#include <nav_msgs/msg/odometry.hpp>

/**
 * @class UWBNodeClass
 * @brief Nodo ROS 2 che emula un sensore UWB.
 *
 * Questa classe gestisce la lettura della posizione dei robot (Odometry),
 * calcola le distanze verso ancore e altri robot, applica rumore e saturazione,
 * e pubblica i dati UWB.
 */
class UWBNodeClass : public rclcpp::Node
{
public:
    // Costruttore principale
    UWBNodeClass();
    
    // Metodo di inizializzazione che prende il tipo di rumore in input
    void init(int noise_type_arg);

    // Getter per la frequenza del sensore (utilizzato anche nel main)
    double get_sensor_frequency() const;

private:
    // Dichiarazione di tutte le funzioni membro che il compilatore deve conoscere
    // (Non è necessario ridichiarare tutti i membri privati)
    
    void parameter_parsing(int noise_type_arg);

    // Callbacks per l'Odometria
    void robot1_callback(const nav_msgs::msg::Odometry& msg);
    void robot2_callback(const nav_msgs::msg::Odometry& msg);
    void robot3_callback(const nav_msgs::msg::Odometry& msg);

    // Funzioni di calcolo e pubblicazione
    double compute_distances(Eigen::Vector3d p1, Eigen::Vector3d p2);
    void calculate_distances();
    void publish_data();
    void add_noise(std::vector<double>& distances);
    void apply_anchor_saturation(std::vector<double>& distances, int anchor_id);
    void apply_robot_saturation(std::vector<double>& distances, int robot_id);

    // I membri privati NON vanno ridichiarati qui, in quanto sono dettagli di implementazione
    // che il compilatore gestisce direttamente nel file .cpp.
};

#endif // UWB_UTILS_EMULATOR_HPP