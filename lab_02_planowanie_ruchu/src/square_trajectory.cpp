#include "trajectory_base.hpp"  // Dołączenie pliku nagłówkowego z definicją klasy TrajectoryBase

// Klasa SquareTrajectory dziedziczy po klasie TrajectoryBase i implementuje trajektorię ruchu robota w kształcie kwadratu
class SquareTrajectory : public TrajectoryBase {
public:
    // Konstruktor klasy SquareTrajectory — ustawia nazwę węzła oraz długość boku kwadratu
    SquareTrajectory() : TrajectoryBase("square_trajectory") {
        side_ = 0.3;  // Długość boku kwadratu (w metrach)

        // Wypisanie informacji o parametrach trajektorii na logu
        RCLCPP_INFO(get_logger(), "Parametry: prędkość=%.2f, bok=%.2f, próg przeszkód=%.2f",
                    linear_, side_, obs_dist_);
    }

private:
    // Główna funkcja odpowiedzialna za obliczanie trajektorii ruchu robota w kształcie kwadratu
    void calculateTrajectory(geometry_msgs::msg::Twist& twist) override {
        // Obliczenie czasu, który upłynął od rozpoczęcia działania węzła
        double t = (now() - start_time_).seconds();
        
        // Obliczenie czasu potrzebnego na przejechanie jednego boku kwadratu (prędkość / długość boku)
        double move_time = side_ / linear_;
        
        // Obliczenie czasu obrotu o 90 stopni (korekta na podstawie prędkości kątowej)
        double turn_time = (M_PI_2 / angular_) * 1.23; 
        
        // Czas trwania pełnego cyklu: jeden bok + obrót
        double total_step_time = move_time + turn_time;
        
        // Czas na pełny cykl: 4 boki kwadratu + 4 obroty
        double full_cycle = 4 * total_step_time;
        
        // Obliczenie, gdzie jesteśmy w cyklu (przejechany czas modulo pełny cykl)
        double current_time = fmod(t, full_cycle);
        
        // Obliczenie, w jakiej fazie cyklu jesteśmy: jazda prosto czy skręt
        double phase_time = fmod(current_time, total_step_time);

        // Sprawdzamy, czy w danej fazie cyklu robot jedzie prosto, czy skręca
        if (phase_time < move_time) {
            // Jeśli czas fazy jest mniejszy niż czas na przejechanie boku, jedź prosto
            twist.linear.x = linear_;
            twist.angular.z = 0.0;  // Brak obrotu
        } else {
            // Jeśli czas fazy to czas obrotu, skręć w prawo
            twist.linear.x = 0.0;   // Brak ruchu do przodu
            twist.angular.z = angular_;  // Obrót o zadaną prędkość kątową
        }
    }

    double side_;  // Długość boku kwadratu (zmienna prywatna, używana w metodzie calculateTrajectory)
};

// Główna funkcja uruchamiająca program
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);  // Inicjalizacja systemu ROS 2
    rclcpp::spin(std::make_shared<SquareTrajectory>());  // Uruchomienie węzła SquareTrajectory
    rclcpp::shutdown();  // Zakończenie działania systemu ROS 2
    return 0;
}
