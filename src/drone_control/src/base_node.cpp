#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "rclcpp_action/rclcpp_action.hpp" 
#include "drone_control/action/navigate.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class DroneBaseNode : public rclcpp::Node {
public:
  //Definir un alias corto para nuestra acción
  using Navigate = drone_control::action::Navigate;
  using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<Navigate>;

  DroneBaseNode() : Node("base_node"), current_x_(0.0), current_y_(0.0), current_z_(0.0) {
    // Inicializar el publicador de transformaciones
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    //Inicializar el publicador de telemetría a 1Hz
    telemetry_pub_ = this->create_publisher<std_msgs::msg::String>("telemetry", 10);
    telemetry_timer_= this->create_wall_timer(1000ms, std::bind(&DroneBaseNode::publish_telemetry,this));
    
    //Inicializar el subscriptor
    stop_sub_= this->create_subscription<std_msgs::msg::Bool>(
      "emergency_stop", 10, std::bind(&DroneBaseNode::stop_callback, this, std::placeholders::_1));
    
    // Timer para mover el dron a 10Hz
    timer_ = this->create_wall_timer(100ms, std::bind(&DroneBaseNode::broadcast_tf, this));
    
    //2. Inicializar el Action Server
    action_server_ = rclcpp_action::create_server<Navigate>(
      this,
      "navigate", //Nombre del topic de la acción
      std::bind(&DroneBaseNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DroneBaseNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&DroneBaseNode::handle_accepted, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(),"Dron encendidio. Esperando coordenadas destino...");
  }

private:
  //---Variables de estado---
  double current_x_, current_y_, current_z_;
  double target_x_, target_y_, target_z_;
  bool is_navigating_ = false;
  bool emergency_stop_active_ = false;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr telemetry_pub_;
  rclcpp::TimerBase::SharedPtr telemetry_timer_;
  rclcpp_action::Server<Navigate>::SharedPtr action_server_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Navigate::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Recibida petición de viaje hacia X: %.2f, Y: %2.f", goal->target_x, goal->target_y);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavigate> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Viaje cancelado por el usuario");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void publish_telemetry(){
    auto msg = std_msgs::msg::String();
    std::string estado = "EN ESPERA (IDLE)";

    //Determinamos el estado actual del dron
    if (emergency_stop_active_){
      estado= "PARADA DE EMERGENCIA ACTIVA";
    } else if (is_navigating_){
      estado = "VOLANDO EN MISIÓN";
    }

    //Formateamos el texto con la altitud real (eje Z)
    char buffer[100];
    snprintf(buffer, sizeof(buffer), "Altitud: %.2f m | Modo de vuelo: %s", current_z_, estado.c_str());
    msg.data = buffer;

    telemetry_pub_->publish(msg);
  }
  void handle_accepted(const std::shared_ptr<GoalHandleNavigate> goal_handle) {
    // Usamos una función lambda o bind, pero asegurándonos de que la firma sea compatible
    std::thread{std::bind(&DroneBaseNode::execute_navigation, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute_navigation(const std::shared_ptr<GoalHandleNavigate> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Iniciando motores...");

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Navigate::Feedback>();
    auto result = std::make_shared<Navigate::Result>();

    target_x_ = goal->target_x;
    target_y_ = goal->target_y;
    target_z_ = goal->target_z;
    is_navigating_ = true;

    rclcpp::Rate loop_rate(10); //10Hz

    while(rclcpp::ok()){
      //Si cancela en medio del vuelo
      if (goal_handle->is_canceling()){
        result->success = false;
        result->message = "Cancelado el vuelo";
        goal_handle->canceled(result);
        is_navigating_ = false;
        return; 
      }
      //Comprobar parada de emergencia
      if( emergency_stop_active_){
        result->success = false;
        result -> message = "Vuelo abortado por Obstáculo Crítico";
        goal_handle->abort(result);
        is_navigating_ = false;
        return;
      }

      //Matematicas básicas de trayectoria hacia el objetivo
      double dx = target_x_ - current_x_;
      double dy = target_y_ - current_y_;
      double dz = target_z_ - current_z_;
      double distance = std::sqrt(dx*dx + dy*dy+ dz*dz);

      //Comprobación para saber si hemos llegado
      if(distance < 0.1){
        break;
      }

      //Mover el dron poco a poco hacia el destino (0.1m por ciclo)
      current_x_ += (dx / distance)*0.1;
      current_y_ += (dy / distance)*0.1;
      current_z_ += (dz/ distance)*0.1;

      //Publicar feedback
      feedback->distance_remaining = distance;
      feedback->current_state = "Volando hacia el destino";
      goal_handle->publish_feedback(feedback);

      loop_rate.sleep();
    }
    
    //Llegada exitosa
    if(rclcpp::ok()){
      result->success = true;
      result->message = "Destino alcanzado";
      goal_handle->succeed(result);
      is_navigating_ = false;
      RCLCPP_INFO(this->get_logger(), "¡Aterrizaje completado!");
    }
  }

  //---Publicador de TFs (Visualización)---
  void broadcast_tf()
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";

    t.transform.translation.x = current_x_;
    t.transform.translation.y = current_y_;
    t.transform.translation.z = current_z_;

    tf2::Quaternion q;
    q.setRPY(0,0,0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
  }
  //Callback que se activa al recibir un mensaje
  void stop_callback(const std_msgs::msg::Bool::SharedPtr msg){
    if (msg->data){
      emergency_stop_active_= true;
      RCLCPP_WARN(this->get_logger(), "¡Parada de emergencia activada!");
    } else{
      emergency_stop_active_ = false;
      RCLCPP_INFO(this->get_logger(), "Sistema de seguridad restablecido");
    }
  }

};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneBaseNode>());
  rclcpp::shutdown();
  return 0;
}