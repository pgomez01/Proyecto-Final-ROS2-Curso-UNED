#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "drone_control/action/navigate.hpp"


using namespace std::chrono_literals;

struct Waypoints {
  double x;
  double y;
  double z;
};

class ControllerNode : public rclcpp::Node {
public:
  using Navigate = drone_control::action::Navigate;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<Navigate>;

  ControllerNode() : Node("controller_node"), current_waypoint_index_(0) {
    // 1. Crear el cliente de la acción
    client_ptr_ = rclcpp_action::create_client<Navigate>(this, "navigate");


    // 2. Definir la lista de misiones (X, Y)
    waypoints_ = {
      {0.0, 0.0, 5.0},   // Waypoint 1: Esquina superior 
      {0.0, 0.0, 0.0},
      {5.0, 0.0, 0.0},
      {0.0, 0.0, 0.0},
      {0.0, 5.0, 0.0},
      {0.0, 0.0, 0.0}
    };

    // 3. Usamos un temporizador de 2 segundos para dar tiempo a que el dron 
    // termine de arrancar antes de lanzarle la primera orden.
    timer_ = this->create_wall_timer(
      2000ms, std::bind(&ControllerNode::send_next_goal, this));
  }

private:
  rclcpp_action::Client<Navigate>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<Waypoints> waypoints_;
  size_t current_waypoint_index_;

  void send_next_goal() {
    // Apagar el temporizador para que no dispare repetidas veces
    this->timer_->cancel();

    // Comprobar si hemos terminado todos los puntos
    if (current_waypoint_index_ >= waypoints_.size()) {
      RCLCPP_INFO(this->get_logger(), "¡MISION COMPLETADA! Todos los objetivos alcanzados.");
      rclcpp::shutdown();
      return;
    }

    // Esperar a que el servidor (el dron) esté en línea
    if (!client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "El dron no responde. ¿Está encendido el base_node?");
      rclcpp::shutdown();
      return;
    }

    // Preparar las coordenadas
    auto goal_msg = Navigate::Goal();
    goal_msg.target_x = waypoints_[current_waypoint_index_].x;
    goal_msg.target_y = waypoints_[current_waypoint_index_].y;
    goal_msg.target_z = waypoints_[current_waypoint_index_].z;

    RCLCPP_INFO(this->get_logger(), "=> Enviando al Waypoint %zu: X=%.2f, Y=%.2f, Z=%.2f", 
      current_waypoint_index_ + 1, goal_msg.target_x, goal_msg.target_y, goal_msg.target_z);

    // Configurar qué funciones procesarán la respuesta del dron
    auto send_goal_options = rclcpp_action::Client<Navigate>::SendGoalOptions();
    
    send_goal_options.goal_response_callback =
      std::bind(&ControllerNode::goal_response_callback, this, std::placeholders::_1);
      
    send_goal_options.feedback_callback =
      std::bind(&ControllerNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
      
    send_goal_options.result_callback =
      std::bind(&ControllerNode::result_callback, this, std::placeholders::_1);

    // ¡Enviar la orden!
    client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  // CALLBACK 1: El dron dice si acepta o rechaza el viaje
  void goal_response_callback(const GoalHandleNavigate::SharedPtr & goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "El dron rechazó la ruta.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Ruta aceptada por el dron. Volando...");
    }
  }

  // CALLBACK 2: El dron nos cuenta a qué distancia está (Feedback)
  void feedback_callback(
    GoalHandleNavigate::SharedPtr,
    const std::shared_ptr<const Navigate::Feedback> feedback)
  {
    RCLCPP_INFO_THROTTLE
    (this->get_logger(),
    *this->get_clock(),
    1000,
    "Distancia a objetivo: %.2f m", feedback->distance_remaining);
  }

  // CALLBACK 3: El dron ha llegado al destino (Result)
  void result_callback(const GoalHandleNavigate::WrappedResult & result) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Confirmación del dron: %s", result.result->message.c_str());
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Vuelo abortado de emergencia.");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Vuelo cancelado.");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Error desconocido en el vuelo.");
        return;
    }
    
    // Pasar al siguiente waypoint
    current_waypoint_index_++;
    
    // Pausa de 3 segundos en el punto antes de enviar la siguiente orden
    timer_ = this->create_wall_timer(
      3000ms, std::bind(&ControllerNode::send_next_goal, this));
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
