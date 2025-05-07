#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>
#include <vector>
#include <memory>

/**
 * @class SimplePolygonPlanner
 * @brief Planer generujący punkty docelowe wzdłuż wielokąta foremnego z punktami pośrednimi.
 *
 * Klasa publikuje kolejne cele (jako PoseStamped) wzdłuż boków wielokąta foremnego, 
 * z określoną liczbą punktów pośrednich na każdym boku. Czas między publikacjami jest konfigurowalny.
 */
class SimplePolygonPlanner : public rclcpp::Node {
public:
    /**
     * @brief Konstruktor klasy SimplePolygonPlanner.
     *
     * Inicjalizuje węzeł ROS2, odczytuje parametry, tworzy wydawcę wiadomości
     * oraz uruchamia timer do cyklicznego publikowania celów.
     */
    SimplePolygonPlanner() : Node("simple_polygon_planner") {
        // Parametry domyślne
        declare_parameter("sides", 4);         // Domyślnie 4 boki (kwadrat)
        declare_parameter("length", 1.0);      // Domyślnie długość boku 1.0 metr
        declare_parameter("interval", 5.0);    // Czas w sekundach między publikacjami
        declare_parameter("intermediate_points", 5);  // Liczba punktów pośrednich na każdym boku

        // Odczyt parametrów
        sides_ = get_parameter("sides").as_int();
        length_ = get_parameter("length").as_double();
        interval_ = get_parameter("interval").as_double();
        intermediate_points_ = get_parameter("intermediate_points").as_int();

        // Tworzenie wydawcy
        goal_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

        // Generacja punktów trasy
        generate_polygon_with_intermediate_points();

        // Timer do wysyłania punktów
        timer_ = create_wall_timer(
            std::chrono::duration<double>(interval_),
            std::bind(&SimplePolygonPlanner::publish_next_goal, this)
        );

        // Logowanie
        RCLCPP_INFO(get_logger(), "Planer wielokąta uruchomiony z parametrami:");
        RCLCPP_INFO(get_logger(), "- Liczba boków: %d", sides_);
        RCLCPP_INFO(get_logger(), "- Długość boku: %.2f m", length_);
        RCLCPP_INFO(get_logger(), "- Punkty pośrednie na boku: %d", intermediate_points_);
        RCLCPP_INFO(get_logger(), "- Interwał wysyłania: %.2f s", interval_);
        RCLCPP_INFO(get_logger(), "- Łączna liczba punktów: %zu", goals_.size());
    }

private:
    /**
     * @brief Generuje wierzchołki i punkty pośrednie wielokąta foremnego.
     *
     * Oblicza współrzędne wszystkich punktów (wierzchołków i pośrednich)
     * na podstawie liczby boków, długości boku i liczby punktów pośrednich.
     */
    void generate_polygon_with_intermediate_points() {
        RCLCPP_INFO(get_logger(), "Generowanie wielokąta z punktami pośrednimi...");

        // Kąt środkowy i promień okręgu opisanego
        double central_angle = 2.0 * M_PI / sides_;
        double radius = length_ / (2.0 * sin(central_angle / 2.0));

        RCLCPP_INFO(get_logger(), "Promień: %.2f m", radius);

        // Generacja wierzchołków
        std::vector<std::array<double, 2>> vertices;
        for (int i = 0; i < sides_; ++i) {
            double angle = i * central_angle;
            double x = radius * cos(angle);
            double y = radius * sin(angle);
            vertices.push_back({x, y});
            RCLCPP_INFO(get_logger(), "Wierzchołek %d: (%.2f, %.2f)", i, x, y);
        }

        // Zamknięcie pętli wielokąta
        vertices.push_back(vertices[0]);

        // Czyść poprzednie dane
        goals_.clear();

        // Generowanie punktów trasy (wierzchołki i punkty pośrednie)
        for (size_t i = 0; i < vertices.size() - 1; ++i) {
            double x1 = vertices[i][0];
            double y1 = vertices[i][1];
            double x2 = vertices[i + 1][0];
            double y2 = vertices[i + 1][1];

            // Kąt kierunku ruchu
            double angle = std::atan2(y2 - y1, x2 - x1);

            // Dodanie wierzchołka
            goals_.push_back({x1, y1, angle});

            // Dodanie punktów pośrednich
            for (int j = 1; j <= intermediate_points_; ++j) {
                double t = static_cast<double>(j) / (intermediate_points_ + 1);
                double x = x1 + t * (x2 - x1);
                double y = y1 + t * (y2 - y1);
                goals_.push_back({x, y, angle});
            }
        }

        // Dodanie końcowego punktu (kopii pierwszego)
        double x_end = vertices.back()[0];
        double y_end = vertices.back()[1];
        double angle_end = std::atan2(vertices[1][1] - y_end, vertices[1][0] - x_end);
        goals_.push_back({x_end, y_end, angle_end});

        RCLCPP_INFO(get_logger(), "Wygenerowano %zu punktów trasy", goals_.size());
    }

    /**
     * @brief Publikuje następny punkt trasy jako wiadomość PoseStamped.
     *
     * Jeśli wszystkie punkty zostały już wysłane, wyłącza węzeł.
     */
    void publish_next_goal() {
        if (current_goal_index_ >= static_cast<int>(goals_.size())) {
            RCLCPP_INFO(get_logger(), "Wszystkie cele wysłane. Kończenie pracy...");
            rclcpp::shutdown();
            return;
        }

        // Dane celu
        double x = goals_[current_goal_index_][0];
        double y = goals_[current_goal_index_][1];
        double angle = goals_[current_goal_index_][2];

        // Konwersja kąta na kwaternion
        double qz = sin(angle / 2.0);
        double qw = cos(angle / 2.0);

        // Przygotowanie wiadomości
        auto goal_msg = geometry_msgs::msg::PoseStamped();
        goal_msg.header.stamp = now();
        goal_msg.header.frame_id = "map";
        goal_msg.pose.position.x = x;
        goal_msg.pose.position.y = y;
        goal_msg.pose.position.z = 0.0;
        goal_msg.pose.orientation.x = 0.0;
        goal_msg.pose.orientation.y = 0.0;
        goal_msg.pose.orientation.z = qz;
        goal_msg.pose.orientation.w = qw;

        // Publikacja wiadomości
        goal_publisher_->publish(goal_msg);

        RCLCPP_INFO(get_logger(), "Wysłano cel %d/%zu: (%.2f, %.2f)",
                    current_goal_index_ + 1, goals_.size(), x, y);

        current_goal_index_++;
    }

    // Publisher do wysyłania pozycji celu
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
    
    // Timer do publikacji
    rclcpp::TimerBase::SharedPtr timer_;

    // Parametry planera
    int sides_;
    double length_;
    double interval_;
    int intermediate_points_;

    // Lista punktów trasy (x, y, orientacja)
    std::vector<std::array<double, 3>> goals_;
    int current_goal_index_ = 0;
};

/**
 * @brief Funkcja główna.
 *
 * Inicjalizuje system ROS2 i uruchamia pętlę główną węzła.
 */
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimplePolygonPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
