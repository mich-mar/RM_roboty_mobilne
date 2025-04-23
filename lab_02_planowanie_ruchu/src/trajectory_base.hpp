#ifndef TRAJECTORY_BASE_HPP
#define TRAJECTORY_BASE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

// Klasa bazowa dla trajektorii, która obsługuje unikanie przeszkód
class TrajectoryBase : public rclcpp::Node {
public:
    // Enum opisujący możliwe stany unikania przeszkód
    enum class AvoidanceState {
        NONE,            // Brak przeszkody
        BACKING_FRONT,   // Cofanie do tyłu w przypadku przeszkody z przodu
        BACKING_BACK,    // Cofanie do przodu w przypadku przeszkody z tyłu
        ROTATING         // Obrót, aby zmienić kierunek
    };

    // Konstruktor klasy
    TrajectoryBase(const std::string& name) : Node(name) {
        // Parametry domyślne dla trajektorii i unikania przeszkód
        linear_ = 0.1;             // Prędkość liniowa
        angular_ = 0.3;            // Prędkość kątowa (obrót)
        obs_dist_ = 0.35;          // Odległość, poniżej której robot uznaje przeszkodę
        rotation_angle_ = M_PI / 4.0; // Kąt obrotu (45 stopni)
        debug_obstacles_ = false;   // Włączenie debugowania

        // Utworzenie subskrybenta do odbioru danych z lasera
        pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10); // Publikacja trajektorii
        sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
                "scan", 10, [this](sensor_msgs::msg::LaserScan::SharedPtr msg) { scan_cb(msg); }
        ); // Subskrypcja danych z LIDARa

        start_time_ = now();         // Czas rozpoczęcia
        last_scan_time_ = now();    // Czas ostatniego odczytu skanu

        // Tworzenie timerów
        timer_ = create_wall_timer(100ms, [this]() { tick(); }); // Timer do aktualizacji trajektorii
        debug_timer_ = create_wall_timer(2000ms, [this]() { debug_info(); }); // Timer do debugowania

        RCLCPP_INFO(get_logger(), "%s uruchomiony", name.c_str());
    }

protected:
    // Funkcja wirtualna do nadpisania w klasach pochodnych, odpowiadająca za obliczanie trajektorii
    virtual void calculateTrajectory(geometry_msgs::msg::Twist& twist) = 0;

    // Zmienna do publikacji trajektorii (ruchu robota)
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    // Zmienna do subskrypcji danych LIDAR
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    // Zmienna do timera aktualizacji trajektorii
    rclcpp::TimerBase::SharedPtr timer_;
    // Zmienna do timera debugowania
    rclcpp::TimerBase::SharedPtr debug_timer_;

    // Parametry ruchu robota
    double linear_;              // Prędkość liniowa
    double angular_;             // Prędkość kątowa
    double obs_dist_;            // Minimalna odległość, przy której uznaje się przeszkodę
    double rotation_angle_;      // Kąt obrotu (w radianach)
    bool debug_obstacles_;       // Flaga do debugowania przeszkód

    // Stany przeszkód (czy są przeszkody z przodu i z tyłu robota)
    bool obstacle_front_ = false;
    bool obstacle_back_ = false;
    AvoidanceState avoidance_state_ = AvoidanceState::NONE; // Stan unikania przeszkód

    // Zmienne do zarządzania czasem
    rclcpp::Time start_time_;        // Czas rozpoczęcia
    rclcpp::Time rotation_start_time_; // Czas rozpoczęcia obrotu
    rclcpp::Time last_scan_time_;     // Czas ostatniego skanu

    // Funkcja wywoływana cyklicznie do debugowania informacji o stanie przeszkód
    void debug_info() {
        if (debug_obstacles_) {
            RCLCPP_INFO(get_logger(), "Stan przeszkód: przód=%s, tył=%s, stan=%d",
                        obstacle_front_ ? "TAK" : "NIE",
                        obstacle_back_ ? "TAK" : "NIE",
                        static_cast<int>(avoidance_state_));
        }
    }

    // Funkcja callback, która przetwarza dane z LIDAR
    void scan_cb(sensor_msgs::msg::LaserScan::SharedPtr msg) {
        last_scan_time_ = now();  // Aktualizacja czasu ostatniego skanu
        if (msg->ranges.empty()) return;  // Sprawdzanie, czy dane z LIDAR są puste

        obstacle_front_ = false;  // Resetowanie stanu przeszkód
        obstacle_back_ = false;

        size_t n = msg->ranges.size();  // Rozmiar danych z LIDAR
        size_t center = n / 2;          // Środek skanu
        size_t delta = n / 8;           // Ustalanie zakresu skanowania

        // Sprawdzanie przeszkód z przodu robota
        for (size_t i = 0; i < delta * 2; i++) {
            size_t idx = (center - delta + i) % n;
            if (std::isfinite(msg->ranges[idx]) && msg->ranges[idx] < obs_dist_) {
                obstacle_front_ = true;
                break;
            }
        }

        // Sprawdzanie przeszkód z tyłu robota
        for (size_t i = 0; i < delta * 2; i++) {
            size_t idx = (center + n/2 - delta + i) % n;
            if (std::isfinite(msg->ranges[idx]) && msg->ranges[idx] < obs_dist_) {
                obstacle_back_ = true;
                break;
            }
        }

        updateAvoidanceState();  // Aktualizacja stanu unikania przeszkód
    }

    // Funkcja, która aktualizuje stan unikania przeszkód
    void updateAvoidanceState() {
        if (avoidance_state_ == AvoidanceState::NONE) {
            // Jeśli nie ma przeszkód, sprawdź, gdzie trzeba unikać
            if (obstacle_front_) {
                avoidance_state_ = AvoidanceState::BACKING_FRONT;
                RCLCPP_INFO(get_logger(), "Przeszkoda z przodu - cofanie");
            } else if (obstacle_back_) {
                avoidance_state_ = AvoidanceState::BACKING_BACK;
                RCLCPP_INFO(get_logger(), "Przeszkoda z tyłu - jazda do przodu");
            }
        } else if (avoidance_state_ == AvoidanceState::BACKING_FRONT && !obstacle_front_) {
            avoidance_state_ = AvoidanceState::ROTATING;
            rotation_start_time_ = now();
            RCLCPP_INFO(get_logger(), "Zakończono BACKING_FRONT - obrót 45 stopni");
        } else if (avoidance_state_ == AvoidanceState::BACKING_BACK && !obstacle_back_) {
            avoidance_state_ = AvoidanceState::ROTATING;
            rotation_start_time_ = now();
            RCLCPP_INFO(get_logger(), "Zakończono BACKING_BACK - obrót 45 stopni");
        }
    }

    // Funkcja wykonywana cyklicznie, odpowiedzialna za aktualizację trajektorii robota
    void tick() {
        auto twist = geometry_msgs::msg::Twist();
        double time_since_scan = (now() - last_scan_time_).seconds();

        // Jeśli minęło za dużo czasu od ostatniego skanu, zatrzymaj robota
        if (time_since_scan > 2.0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Brak danych z lidaru. Stop.");
            pub_->publish(twist);
            return;
        }

        // Logika w zależności od stanu unikania przeszkód
        switch (avoidance_state_) {
            case AvoidanceState::BACKING_FRONT:
                twist.linear.x = linear_;
                twist.angular.z = 0;
                break;
            case AvoidanceState::BACKING_BACK:
                twist.linear.x = -linear_;
                twist.angular.z = 0;
                break;
            case AvoidanceState::ROTATING: {
                twist.angular.z = -angular_; // Obrót w lewo
                double dt = (now() - rotation_start_time_).seconds();
                if (dt >= rotation_angle_ / angular_) {
                    avoidance_state_ = AvoidanceState::NONE; // Zakończono obrót
                    RCLCPP_INFO(get_logger(), "Zakończono obrót 45 stopni - kontynuuj trajektorię");
                }
                break;
            }
            default:
                calculateTrajectory(twist); // Obliczanie trajektorii (do nadpisania w klasach pochodnych)
        }

        pub_->publish(twist);  // Publikowanie wynikowej trajektorii
    }
};

#endif // TRAJECTORY_BASE_HPP
