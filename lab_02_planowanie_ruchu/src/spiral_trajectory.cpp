#include "trajectory_base.hpp"  // Dołączenie pliku nagłówkowego z definicją klasy TrajectoryBase

// Klasa SpiralTrajectory dziedziczy po klasie TrajectoryBase i implementuje trajektorię ruchu robota w kształcie spirali
class SpiralTrajectory : public TrajectoryBase {
public:
    // Konstruktor klasy SpiralTrajectory — ustawia nazwę węzła oraz inicjalizuje parametry specyficzne dla spirali
    SpiralTrajectory() : TrajectoryBase("spiral_trajectory") {
        // Parametr określający szybkość "rozkręcania się" spirali
        spiral_factor_ = 0.1;  
        
        // Czas rozpoczęcia generowania trajektorii spirali
        spiral_time_ = now();  
        
        // Wypisanie informacji o parametrach trajektorii na logu
        RCLCPP_INFO(get_logger(), "Parametry: prędkość=%.2f, współczynnik spirali=%.2f, próg przeszkód=%.2f",
                    linear_, spiral_factor_, obs_dist_);
    }

private:
    // Nadpisana metoda obliczająca trajektorię robota
    void calculateTrajectory(geometry_msgs::msg::Twist& twist) override {
        // Obliczenie czasu, który upłynął od rozpoczęcia generowania spirali
        double t = (now() - spiral_time_).seconds();
        
        // Obliczenie promienia spirali (r) w zależności od upływającego czasu i współczynnika spirali
        double r = 0.1 * (1.0 + spiral_factor_ * t);

        // Ustawienie prędkości liniowej na stałą prędkość
        twist.linear.x = linear_;
        
        // Ustawienie prędkości kątowej w zależności od promienia spirali
        twist.angular.z = linear_ / r;

        // Jeśli promień spirali przekroczy 1.5, zresetuj czas spirali
        if (r > 1.5) {
            spiral_time_ = now();  // Reset czasu, aby rozpocząć nową spiralę
            RCLCPP_INFO(get_logger(), "Reset spirali");  // Informacja na logu
        }
    }

    double spiral_factor_;  // Współczynnik odpowiedzialny za kształt spirali
    rclcpp::Time spiral_time_;  // Czas rozpoczęcia generowania spirali
};

// Główna funkcja uruchamiająca program
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);  // Inicjalizacja systemu ROS 2
    rclcpp::spin(std::make_shared<SpiralTrajectory>());  // Uruchomienie węzła SpiralTrajectory
    rclcpp::shutdown();  // Zakończenie działania systemu ROS 2
    return 0;
}
