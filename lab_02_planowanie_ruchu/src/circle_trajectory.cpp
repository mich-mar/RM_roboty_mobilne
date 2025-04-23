#include "trajectory_base.hpp"  // Dołączenie pliku nagłówkowego z definicją klasy TrajectoryBase

// Klasa CircleTrajectory dziedziczy po klasie TrajectoryBase i implementuje trajektorię ruchu robota w kształcie okręgu
class CircleTrajectory : public TrajectoryBase {
public:
    // Konstruktor klasy CircleTrajectory — ustawia nazwę węzła oraz inicjalizuje parametry specyficzne dla okręgu
    CircleTrajectory() : TrajectoryBase("circle_trajectory") {
        // Parametr określający promień okręgu
        radius_ = 0.3; 
        
        // Wypisanie informacji o parametrach trajektorii na logu
        RCLCPP_INFO(get_logger(), "Parametry: prędkość=%.2f, promień=%.2f, próg przeszkód=%.2f",
                    linear_, radius_, obs_dist_);
    }

private:
    // Nadpisana metoda obliczająca trajektorię robota w kształcie okręgu
    void calculateTrajectory(geometry_msgs::msg::Twist& twist) override {
        // Ustawienie prędkości liniowej na stałą prędkość
        twist.linear.x = linear_;
        
        // Ustawienie prędkości kątowej (wzór na prędkość kątową w ruchu po okręgu)
        twist.angular.z = linear_ / radius_;
    }

    double radius_;  // Promień okręgu (zmienna prywatna, używana w metodzie calculateTrajectory)
};

// Główna funkcja uruchamiająca program
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);  // Inicjalizacja systemu ROS 2
    rclcpp::spin(std::make_shared<CircleTrajectory>());  // Uruchomienie węzła CircleTrajectory
    rclcpp::shutdown();  // Zakończenie działania systemu ROS 2
    return 0;
}
