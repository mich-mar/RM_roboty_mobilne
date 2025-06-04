#ifndef FLEET_CONTROLLER_CONTROLLER_HPP_
#define FLEET_CONTROLLER_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <queue>
#include "fleet_controller/points.hpp"

/**
 * @brief Kontroler zarządzający flotą robotów i ich zadaniami
 */
class FleetController : public rclcpp::Node {
public:
    /**
     * @brief Konstruktor inicjalizujący kontroler floty
     */
    FleetController();

private:
    // Stałe konfiguracyjne
    static const int NUM_ROBOTS = 16;
    const double LINEAR_SPEED = 0.2;
    const double ANGULAR_SPEED = 0.5;
    const double POSITION_TOLERANCE = 0.05;
    const double ANGLE_TOLERANCE = 0.05;

    /**
     * @brief Struktura reprezentująca zadanie transportowe
     */
    struct Task {
        int pickup_point;    ///< Numer punktu odbioru (0-9)
        int delivery_point;  ///< Numer punktu dostawy (0-9)
        Point2D pickup;      ///< Współrzędne punktu odbioru
        Point2D delivery;    ///< Współrzędne punktu dostawy
    };

    /**
     * @brief Struktura przechowująca stan robota
     */
    struct RobotState {
        geometry_msgs::msg::Pose current_pose;  ///< Aktualna pozycja i orientacja
        Point2D current_position;               ///< Aktualna pozycja 2D
        bool is_busy;                          ///< Czy robot wykonuje zadanie
        Task current_task;                     ///< Aktualnie wykonywane zadanie
        int movement_phase;                    ///< Faza ruchu
        double last_heading_error;             ///< Poprzedni błąd kierunku (dla PID)
        double heading_error_integral;         ///< Całka błędu kierunku (dla PID)
        
        enum class State {
            WAITING,
            MOVING_TO_PICKUP,
            MOVING_TO_DELIVERY,
            RETURNING_TO_QUEUE
        } state;
    };

    // Struktury danych
    std::queue<Task> task_queue;              ///< Kolejka zadań
    std::vector<int> queue_state;             ///< Stan kolejki (-1 = puste miejsce)
    std::vector<RobotState> robots;           ///< Stan wszystkich robotów

    // ROS 2 komunikacja
    std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> robot_publishers;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subscribers;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr task_subscriber;
    rclcpp::TimerBase::SharedPtr control_timer;

    // Callbacks
    void task_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg, int robot_id);
    void control_loop();

    // Funkcje sterowania ruchem
    bool rotate_robot(int robot_id, double target_angle);
    bool move_straight(int robot_id, const Point2D& target);
    bool move_to_pickup(int robot_id);
    bool move_to_delivery(int robot_id);
    bool return_to_queue(int robot_id);

    // Funkcje pomocnicze
    void reorganize_queue();
    bool is_point_reached(const Point2D& current, const Point2D& target);
    double get_robot_yaw(const geometry_msgs::msg::Pose& pose);
    int find_first_empty_queue_position();
};

#endif  // FLEET_CONTROLLER_CONTROLLER_HPP_
