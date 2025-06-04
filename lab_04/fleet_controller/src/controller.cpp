#include "fleet_controller/controller.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <random>

/**
 * @brief Kontroler zarządzający flotą robotów i ich zadaniami
 * 
 * Klasa odpowiedzialna za:
 * - zarządzanie kolejką robotów oczekujących na zadania
 * - przydzielanie zadań robotom
 * - kontrolę ruchu robotów (przemieszczanie się do punktów odbioru/dostawy)
 * - monitorowanie pozycji robotów poprzez odometry
 */
FleetController::FleetController() : Node("fleet_controller") {
    // Inicjalizacja kolejki robotów
    queue_state.resize(NUM_ROBOTS, -1);
    robots.resize(NUM_ROBOTS);

    // Na początku wszystkie roboty są w kolejce
    for (int i = 0; i < NUM_ROBOTS; ++i) {
        queue_state[i] = i;
        robots[i].is_busy = false;
        robots[i].state = RobotState::State::WAITING;
        robots[i].movement_phase = 0;
    }

    // Publishers i subscribers dla każdego robota
    for (int i = 0; i < NUM_ROBOTS; ++i) {
        robot_publishers.push_back(
            this->create_publisher<geometry_msgs::msg::Twist>(
                "tb_" + std::to_string(i) + "/cmd_vel", 10));

        auto odom_callback = [this, i](nav_msgs::msg::Odometry::SharedPtr msg) {
            this->odom_callback(msg, i);
        };

        odom_subscribers.push_back(
            this->create_subscription<nav_msgs::msg::Odometry>(
                "tb_" + std::to_string(i) + "/odom", 10, odom_callback));
    }

    // Subskrypcja zadań
    task_subscriber = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "new_task", 10,
        std::bind(&FleetController::task_callback, this, std::placeholders::_1));

    // Timer kontrolny (10Hz)
    control_timer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&FleetController::control_loop, this));
}

/**
 * @brief Callback obsługujący nowe zadania otrzymane z topicu
 * 
 * Interpretacja wiadomości:
 * - Jeśli msg->data[0] == 0: losuj punkt odbioru
 * - Jeśli msg->data[1] == 0: losuj punkt dostawy
 * - W przeciwnym razie użyj wartości z wiadomości (konwertując na indeks 0-9)
 * 
 * @param msg Wiadomość zawierająca tablicę dwóch liczb całkowitych:
 *           [punkt_odbioru (1-10 lub 0 dla losowego), punkt_dostawy (1-10 lub 0 dla losowego)]
 */
void FleetController::task_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 2) {
        Task new_task;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> point_dist(0, 9); // Rozkład dla punktów (0-9)
        
        // Obsługa punktu odbioru
        if (msg->data[0] == 0) {
            // Losuj punkt odbioru
            new_task.pickup_point = point_dist(gen);
        } else {
            // Użyj wartości z wiadomości
            new_task.pickup_point = msg->data[0] - 1;  // Konwersja na indeks (0-9)
        }
        
        // Obsługa punktu dostawy
        if (msg->data[1] == 0) {
            // Losuj punkt dostawy (różny od punktu odbioru)
            do {
                new_task.delivery_point = point_dist(gen);
            } while (new_task.delivery_point == new_task.pickup_point);
        } else {
            // Użyj wartości z wiadomości
            new_task.delivery_point = msg->data[1] - 1;
        }
        
        // Sprawdź czy punkty są w prawidłowym zakresie
        if (new_task.pickup_point >= 0 && new_task.pickup_point < 10 &&
            new_task.delivery_point >= 0 && new_task.delivery_point < 10 &&
            new_task.pickup_point != new_task.delivery_point) {
            
            new_task.pickup = pickup_stations[new_task.pickup_point];
            new_task.delivery = delivery_stations[new_task.delivery_point];
            task_queue.push(new_task);
            
            RCLCPP_INFO(this->get_logger(), 
                "New task added: pickup at point %d (%.1f,%.1f), delivery at point %d (%.1f,%.1f)",
                new_task.pickup_point + 1,  // Konwersja z powrotem na numerację 1-10 dla wyświetlania
                pickup_stations[new_task.pickup_point].x,
                pickup_stations[new_task.pickup_point].y,
                new_task.delivery_point + 1,  // Konwersja z powrotem na numerację 1-10 dla wyświetlania
                delivery_stations[new_task.delivery_point].x,
                delivery_stations[new_task.delivery_point].y);
        }
    }
}

/**
 * @brief Callback obsługujący odometrię robotów
 * 
 * @param msg Wiadomość zawierająca dane odometrii
 * @param robot_id Identyfikator robota
 */
void FleetController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg, int robot_id) {
    robots[robot_id].current_pose = msg->pose.pose;
    robots[robot_id].current_position.x = msg->pose.pose.position.x;
    robots[robot_id].current_position.y = msg->pose.pose.position.y;
}

/**
 * @brief Oblicza kąt obrotu robota (yaw) z kwaternionów
 * 
 * @param pose Pozycja robota zawierająca orientację w postaci kwaternionów
 * @return double Kąt w radianach, znormalizowany do zakresu [-π, π]
 */
double FleetController::get_robot_yaw(const geometry_msgs::msg::Pose& pose) {
    tf2::Quaternion q;
    q.setX(pose.orientation.x);
    q.setY(pose.orientation.y);
    q.setZ(pose.orientation.z);
    q.setW(pose.orientation.w);
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Normalizacja do zakresu [-π, π]
    while (yaw > M_PI) yaw -= 2 * M_PI;
    while (yaw < -M_PI) yaw += 2 * M_PI;
    
    return yaw;
}

/**
 * @brief Sprawdza czy robot dotarł do zadanego punktu
 * 
 * @param current Aktualna pozycja robota
 * @param target Pozycja docelowa
 * @return true Jeśli robot jest wystarczająco blisko celu (w granicach POSITION_TOLERANCE)
 * @return false W przeciwnym razie
 */
bool FleetController::is_point_reached(const Point2D& current, const Point2D& target) {
    return (std::abs(current.x - target.x) < POSITION_TOLERANCE &&
            std::abs(current.y - target.y) < POSITION_TOLERANCE);
}

/**
 * @brief Znajduje pierwsze wolne miejsce w kolejce
 * 
 * @return int Indeks pierwszego wolnego miejsca lub -1 jeśli nie znaleziono
 */
int FleetController::find_first_empty_queue_position() {
    for (int i = 0; i < NUM_ROBOTS; ++i) {
        if (queue_state[i] == -1) {
            return i;
        }
    }
    return -1;  // Nie znaleziono pustego miejsca
}

/**
 * @brief Obraca robota do zadanego kąta
 * 
 * @param robot_id Identyfikator robota
 * @param target_angle Docelowy kąt w radianach
 * @return true Jeśli robot osiągnął zadaną orientację
 * @return false Jeśli robot jest w trakcie obrotu
 */
bool FleetController::rotate_robot(int robot_id, double target_angle) {
    auto& robot = robots[robot_id];
    geometry_msgs::msg::Twist cmd_vel;

    // ZAWSZE zerujemy wszystkie prędkości na początku
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;

    // Pobierz aktualny kąt
    double current_angle = get_robot_yaw(robot.current_pose);
    
    // Oblicz różnicę kątów
    double angle_diff = target_angle - current_angle;
    
    // Normalizacja różnicy do zakresu [-π, π]
    while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

    // Sprawdź czy cel osiągnięty
    if (std::abs(angle_diff) < ANGLE_TOLERANCE) {
        // Upewnij się, że robot jest zatrzymany
        robot_publishers[robot_id]->publish(cmd_vel);
        return true;
    }

    // Ustaw TYLKO prędkość kątową, wszystkie inne zostają na 0
    cmd_vel.angular.z = ANGULAR_SPEED * ((angle_diff > 0) ? 1.0 : -1.0);
    
    // Publikuj komendę
    robot_publishers[robot_id]->publish(cmd_vel);
    return false;
}

/**
 * @brief Kontroluje ruch prostoliniowy robota do zadanego punktu
 * 
 * Implementuje kontroler PID dla sterowania kątem.
 * Jeśli odchylenie od kursu jest zbyt duże, robot najpierw się obraca.
 * 
 * @param robot_id Identyfikator robota
 * @param target Punkt docelowy
 * @return true Jeśli robot dotarł do celu
 * @return false Jeśli robot jest w trakcie ruchu
 */
bool FleetController::move_straight(int robot_id, const Point2D& target) {
    auto& robot = robots[robot_id];
    geometry_msgs::msg::Twist cmd_vel;

    // Zero all velocities
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;

    // Calculate distance to target
    double dx = target.x - robot.current_position.x;
    double dy = target.y - robot.current_position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    // If we reached the target, stop and return true
    if (distance < POSITION_TOLERANCE) {
        robot_publishers[robot_id]->publish(cmd_vel);
        // Reset PID errors when reaching target
        robot.last_heading_error = 0.0;
        robot.heading_error_integral = 0.0;
        return true;
    }

    // Calculate desired heading to target
    double desired_heading = atan2(dy, dx);
    double current_heading = get_robot_yaw(robot.current_pose);
    double heading_error = desired_heading - current_heading;
    
    // Normalize heading error to [-π, π]
    while (heading_error > M_PI) heading_error -= 2 * M_PI;
    while (heading_error < -M_PI) heading_error += 2 * M_PI;

    // PID dla kontroli kąta
    double Kp = 0.5;  // Proportional gain (możesz dostosować)
    double Ki = 0.01; // Integral gain (możesz dostosować)
    double Kd = 0.1;  // Derivative gain (możesz dostosować)
    double dt = 0.1;  // Zakładając, że control_loop działa co 100ms
    double WIDER_ANGLE_TOLERANCE = 0.15; // Około 8.6 stopni

    // Obliczenie członu całkującego (integral)
    robot.heading_error_integral += heading_error * dt;
    // Ograniczenie członu całkującego aby zapobiec wind-up
    double MAX_INTEGRAL = 1.0;
    robot.heading_error_integral = std::max(-MAX_INTEGRAL, 
                                          std::min(MAX_INTEGRAL, robot.heading_error_integral));

    // Obliczenie członu różniczkującego (derivative)
    double heading_error_derivative = (heading_error - robot.last_heading_error) / dt;
    robot.last_heading_error = heading_error;

    // Obliczenie sterowania PID
    double angular_control = Kp * heading_error + 
                           Ki * robot.heading_error_integral +
                           Kd * heading_error_derivative;

    // Jeśli robot nie jest skierowany w stronę celu, obróć go
    if (std::abs(heading_error) > WIDER_ANGLE_TOLERANCE) {
        // Tylko obrót z PID
        cmd_vel.angular.z = std::max(-ANGULAR_SPEED, 
                                   std::min(ANGULAR_SPEED, angular_control));
    } else {
        // Jeśli robot jest w miarę skierowany w dobrą stronę, 
        // jedź do przodu z delikatną korektą kąta
        cmd_vel.linear.x = LINEAR_SPEED;
        cmd_vel.angular.z = angular_control * 0.5; // Zmniejszone sterowanie kątem podczas jazdy
    }

    robot_publishers[robot_id]->publish(cmd_vel);
    return false;
}

/**
 * @brief Reorganizuje kolejkę robotów
 * 
 * Przesuwa roboty do przodu gdy pojawią się luki w kolejce.
 * Aktualizuje queue_state i wysyła komendy ruchu do robotów.
 */
void FleetController::reorganize_queue() {
    bool changes_made = false;
    // Przesuwamy wszystkie roboty do przodu, wypełniając luki
    for (int i = 0; i < NUM_ROBOTS - 1; i++) {
        // Jeśli znaleźliśmy pusty slot
        if (queue_state[i] == -1) {
            // Szukaj następnego robota w kolejce
            for (int j = i + 1; j < NUM_ROBOTS; j++) {
                if (queue_state[j] != -1) {
                    int robot_id = queue_state[j];
                    // Sprawdź czy robot może się przemieścić
                    if (!robots[robot_id].is_busy && robots[robot_id].state == RobotState::State::WAITING) {
                        // Przesuń robota na wolne miejsce
                        queue_state[i] = robot_id;
                        queue_state[j] = -1;
                        
                        // Wyślij robota na nową pozycję
                        move_straight(robot_id, robot_bases[i]);
                        
                        RCLCPP_INFO(this->get_logger(), 
                            "Moving robot %d to queue position %d at (%.1f,%.1f)", 
                            robot_id, i, robot_bases[i].x, robot_bases[i].y);
                        
                        changes_made = true;
                        break;
                    }
                }
            }
        }
    }

    if (changes_made) {
        // Debug: wyświetl stan kolejki
        std::string queue_debug = "Queue state: ";
        for (int i = 0; i < NUM_ROBOTS; i++) {
            queue_debug += std::to_string(queue_state[i]) + " ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", queue_debug.c_str());
    }
}

/**
 * @brief Główna pętla kontrolna systemu
 * 
 * Wykonuje:
 * 1. Obsługę pierwszego robota w kolejce (przydzielanie nowych zadań)
 * 2. Obsługę wszystkich robotów (aktualizacja stanu i sterowanie)
 * 3. Zarządzanie kolejką (reorganizacja gdy potrzebna)
 */
void FleetController::control_loop() {
    // Obsługa pierwszego robota w kolejce
    if (!task_queue.empty() && queue_state[0] != -1) {
        int first_robot = queue_state[0];
        auto& robot = robots[first_robot];

        if (!robot.is_busy) {
            // Przydziel nowe zadanie
            robot.current_task = task_queue.front();
            task_queue.pop();
            robot.is_busy = true;
            robot.state = RobotState::State::MOVING_TO_PICKUP;
            
            // Najpierw wyświetl debug info
            RCLCPP_INFO(this->get_logger(), "Before queue update - Queue state[0]: %d", queue_state[0]);
            
            // Zwolnij miejsce w kolejce i reorganizuj
            queue_state[0] = -1;
            reorganize_queue();
            
            // Debug info po aktualizacji
            RCLCPP_INFO(this->get_logger(), "After queue update - Queue state[0]: %d", queue_state[0]);
            
            RCLCPP_INFO(this->get_logger(), 
                "Assigned task to robot %d: pickup at (%.1f,%.1f), delivery at (%.1f,%.1f)",
                first_robot,
                robot.current_task.pickup.x,
                robot.current_task.pickup.y,
                robot.current_task.delivery.x,
                robot.current_task.delivery.y);
        }
    }

    // Obsługa wszystkich robotów
    for (int i = 0; i < NUM_ROBOTS; i++) {
        auto& robot = robots[i];
        
        if (robot.is_busy) {
            switch (robot.state) {
                case RobotState::State::MOVING_TO_PICKUP:
                    if (move_to_pickup(i)) {
                        robot.state = RobotState::State::MOVING_TO_DELIVERY;
                        RCLCPP_INFO(this->get_logger(), "Robot %d reached pickup point", i);
                    }
                    break;

                case RobotState::State::MOVING_TO_DELIVERY:
                    if (move_to_delivery(i)) {
                        robot.state = RobotState::State::RETURNING_TO_QUEUE;
                        RCLCPP_INFO(this->get_logger(), "Robot %d reached delivery point", i);
                    }
                    break;

                case RobotState::State::RETURNING_TO_QUEUE:
                    if (return_to_queue(i)) {
                        robot.state = RobotState::State::WAITING;
                        robot.is_busy = false;
                        RCLCPP_INFO(this->get_logger(), "Robot %d returned to queue", i);
                    }
                    break;

                case RobotState::State::WAITING:
                    // Robot czeka w kolejce, nic nie rób
                    break;
            }
        } else if (robot.state == RobotState::State::WAITING) {
            // Sprawdź pozycję w kolejce i kontynuuj ruch jeśli potrzeba
            for (int pos = 0; pos < NUM_ROBOTS; pos++) {
                if (queue_state[pos] == i) {
                    if (!is_point_reached(robot.current_position, robot_bases[pos])) {
                        move_straight(i, robot_bases[pos]);
                    }
                    break;
                }
            }
        }
    }
}

/**
 * @brief Przemieszcza robota do punktu odbioru zadania
 * 
 * Wykonuje ruch w czterech fazach:
 * 1. Obrót w lewo (90 stopni)
 * 2. Jazda do punktu pośredniego (wyrównanie X)
 * 3. Obrót w prawo (-90 stopni)
 * 4. Jazda do punktu końcowego
 * 
 * @param robot_id Identyfikator robota
 * @return true Jeśli robot dotarł do punktu odbioru
 * @return false Jeśli robot jest w trakcie ruchu
 */
bool FleetController::move_to_pickup(int robot_id) {
    auto& robot = robots[robot_id];
    static Point2D intermediate_point;

    switch (robot.movement_phase) {
        case 0:  // Obrót w lewo (90 stopni)
            if (rotate_robot(robot_id, 0.0)) {
                robot.movement_phase = 1;
                intermediate_point = {robot.current_task.pickup.x, robot.current_position.y};
            }
            break;

        case 1:  // Jazda do punktu X
            if (move_straight(robot_id, intermediate_point)) {
                robot.movement_phase = 2;
            }
            break;

        case 2:  // Obrót w prawo (-90 stopni)
            if (rotate_robot(robot_id, -M_PI/2)) {
                robot.movement_phase = 3;
            }
            break;

        case 3:  // Jazda do punktu Y
            if (move_straight(robot_id, robot.current_task.pickup)) {
                robot.movement_phase = 0;  // Reset dla następnego użycia
                return true;
            }
            break;
    }

    return false;
}

/**
 * @brief Przemieszcza robota do punktu dostawy
 * 
 * Wykonuje ruch w sześciu fazach:
 * 1. Obrót o 180 stopni
 * 2. Jazda do Y=0
 * 3. Obrót w kierunku X punktu docelowego
 * 4. Jazda do odpowiedniego X
 * 5. Obrót w kierunku punktu docelowego
 * 6. Jazda do punktu końcowego
 * 
 * @param robot_id Identyfikator robota
 * @return true Jeśli robot dotarł do punktu dostawy
 * @return false Jeśli robot jest w trakcie ruchu
 */
bool FleetController::move_to_delivery(int robot_id) {
    auto& robot = robots[robot_id];
    static Point2D intermediate_point;

    switch (robot.movement_phase) {
        case 0:  // Pierwszy obrót o 180 stopni
            if (rotate_robot(robot_id, M_PI/2)) {
                robot.movement_phase = 1;
                intermediate_point = {robot.current_task.pickup.x, 0};
            }
            break;

        case 1:  // Jazda do Y=0
            if (move_straight(robot_id, intermediate_point)) {
                robot.movement_phase = 2;
            }
            break;

        case 2:  // Obrót w kierunku X punktu docelowego
            {
                double target_angle = (robot.current_task.delivery.x > robot.current_position.x) ? 0.0 : M_PI;
                if (rotate_robot(robot_id, target_angle)) {
                    robot.movement_phase = 3;
                    intermediate_point = {robot.current_task.delivery.x, robot.current_position.y};
                }
            }
            break;

        case 3:  // Jazda do odpowiedniego X
            if (move_straight(robot_id, intermediate_point)) {
                robot.movement_phase = 4;
            }
            break;

        case 4:  // Obrót w kierunku punktu docelowego
            {
                if (rotate_robot(robot_id, M_PI/2)) {
                    robot.movement_phase = 5;
                }
            }
            break;

        case 5:  // Jazda do punktu docelowego
            if (move_straight(robot_id, robot.current_task.delivery)) {
                robot.movement_phase = 0;  // Reset dla następnego użycia
                return true;
            }
            break;
    }

    return false;
}

/**
 * @brief Przemieszcza robota z powrotem do kolejki
 * 
 * Wykonuje ruch w sześciu fazach:
 * 1. Obrót o 90 stopni
 * 2. Jazda do Y ostatniego miejsca w kolejce
 * 3. Obrót w prawo
 * 4. Jazda do X kolejki
 * 5. Obrót w lewo (ustawienie jak w kolejce)
 * 6. Jazda do wolnego miejsca w kolejce
 * 
 * @param robot_id Identyfikator robota
 * @return true Jeśli robot dotarł do kolejki i zajął miejsce
 * @return false Jeśli robot jest w trakcie ruchu
 */
bool FleetController::return_to_queue(int robot_id) {
    auto& robot = robots[robot_id];
    static Point2D intermediate_point;
    static int target_queue_position = -1;

    switch (robot.movement_phase) {
        case 0:  // Pierwszy obrót o 90 stopni
            if (rotate_robot(robot_id, -M_PI/2)) {
                robot.movement_phase = 1;
		intermediate_point = {robot.current_task.delivery.x, robot_bases[15].y};
            }
            break;

        case 1:  // Jazda do Y ostatniego miejsca w kolejce
            if (move_straight(robot_id, intermediate_point)) {
                robot.movement_phase = 2;
            }
            break;

        case 2:  // Obrót w prawo
            if (rotate_robot(robot_id, M_PI)) {
                robot.movement_phase = 3;
            }
            break;

        case 3:  // Jazda do X kolejki
            if (move_straight(robot_id, robot_bases[15])) {
                robot.movement_phase = 4;
            }
            break;

        case 4:  // Obrót w lewo (ustawienie jak w kolejce)
            if (rotate_robot(robot_id, -M_PI/2)) {
                robot.movement_phase = 5;
                target_queue_position = find_first_empty_queue_position();
                if (target_queue_position != -1) {
                    intermediate_point = robot_bases[target_queue_position];
                }
            }
            break;

        case 5:  // Jazda do wolnego miejsca w kolejce
            if (target_queue_position != -1 && move_straight(robot_id, intermediate_point)) {
                queue_state[target_queue_position] = robot_id;
                robot.movement_phase = 0;  // Reset dla następnego użycia
                return true;
            }
            break;
    }

    return false;
}
