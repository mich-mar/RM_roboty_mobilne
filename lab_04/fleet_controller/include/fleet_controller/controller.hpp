#ifndef FLEET_CONTROLLER_CONTROLLER_HPP_
#define FLEET_CONTROLLER_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <queue>
#include "fleet_controller/points.hpp"

class FleetController : public rclcpp::Node {
public:
    FleetController();

private:
    // Stałe
    int NUM_ROBOTS = 16;
    double LINEAR_SPEED = 0.2;
    double ANGULAR_SPEED = 0.5;
    double POSITION_TOLERANCE = 0.01;
    double ANGLE_TOLERANCE = 0.05;
    double COURSE_CHECK_DISTANCE = 0.5;

    // Struktury danych
    struct Task {
        int pickup_point;
        int delivery_point;
        Point2D pickup;
        Point2D delivery;
    };

    // Kolejka zadań
    std::queue<Task> task_queue;

    // Kolejka robotów (-1 oznacza puste miejsce)
    std::vector<int> queue_state;

    // Publishers i Subscribers
    std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> robot_publishers;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subscribers;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr task_subscriber;
    
    // Timer do głównej pętli kontrolnej
    rclcpp::TimerBase::SharedPtr control_timer;

    // Stan robotów
    struct RobotState {
    geometry_msgs::msg::Pose current_pose;
    Point2D current_position;
    bool is_busy;
    Task current_task;
    int movement_phase;
    double desired_heading = 0.0;  // Pożądany kąt kursu
    double last_heading_error = 0.0;
    double heading_error_integral = 0.0;
    enum class State {
        WAITING,
        MOVING_TO_PICKUP,
        MOVING_TO_DELIVERY,
        RETURNING_TO_QUEUE
       } state;
    };
    std::vector<RobotState> robots;

    // Callbacki
    void task_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg, int robot_id);
    void control_loop();

    // Funkcje ruchu
    bool rotate_robot(int robot_id, double target_angle);
    bool move_straight(int robot_id, const Point2D& target);
    bool move_to_pickup(int robot_id);
    bool move_to_delivery(int robot_id);
    bool return_to_queue(int robot_id);

    // Funkcje pomocnicze
    void reorganize_queue();
    bool is_point_reached(const Point2D& current, const Point2D& target);
    double get_robot_yaw(const geometry_msgs::msg::Pose& pose);
    Point2D get_queue_position(int queue_position);
    int find_first_empty_queue_position();

    double calculate_heading_to_target(const Point2D& current, const Point2D& target) {
        double dx = target.x - current.x;
        double dy = target.y - current.y;
        return std::atan2(dy, dx);
    }
};

#endif  // FLEET_CONTROLLER_CONTROLLER_HPP_
