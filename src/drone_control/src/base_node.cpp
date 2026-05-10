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
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"

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
    
    cmd_vel_pub_=this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&DroneBaseNode::odom_callback, this, std::placeholders::_1));
    //Inicializar el subscriptor
    stop_sub_= this->create_subscription<std_msgs::msg::Bool>(
      "emergency_stop", 10, std::bind(&DroneBaseNode::stop_callback, this, std::placeholders::_1));

    pause_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/pause_mission", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg){
        if(msg->data && !is_paused_){
          RCLCPP_ERROR(this->get_logger(), "VUELO PAUSADO");
        }else if(!msg->data && is_paused_){
          RCLCPP_INFO(this->get_logger(), "VUELO REANUDADO, PAUSA DESACTIVADA");
        }
        is_paused_=msg->data;
      }
    );
    laser_sub_ = this ->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",10,std::bind(&DroneBaseNode::laser_callback,this, std::placeholders::_1)
    );
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
  bool emergency_= false;
  bool is_paused_=false;
  float dist_frente_ = 10.0;
  float dist_izq_ = 10.0;
  float dist_der_ = 10.0;
  float dist_atras_= 10.0;
  float dist_diag_der_ = 10.0;
  float dist_diag_izq_ = 10.0;
  double current_yaw_ = 0.0;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr telemetry_pub_;
  rclcpp::TimerBase::SharedPtr telemetry_timer_;
  rclcpp_action::Server<Navigate>::SharedPtr action_server_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pause_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

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

    bool notified_obstacle = false; //flag de obstaculo detectado
    bool pause=false; //flag de pausa de vuelo

    enum EstadoVuelo { VOLANDO, ROTANDO_ESQUIVA, RODEANDO_OBSTACULO};
    EstadoVuelo estado_actual = VOLANDO;
    int direccion_esquiva = 1; //1= IZQUIERDA, -1 = DERECHA
    int ciclos_rodeo = 0;

    while(rclcpp::ok()){
      //Si cancela en medio del vuelo
      if (goal_handle->is_canceling()){

        geometry_msgs::msg::Twist stop_msg;
        cmd_vel_pub_->publish(stop_msg);

        result->success = false;
        result->message = "Cancelado el vuelo";
        goal_handle->canceled(result);
        is_navigating_ = false;
        return; 
      }
      //Matematicas básicas de trayectoria hacia el objetivo
      double dx = target_x_ - current_x_;
      double dy = target_y_ - current_y_;
      double dz = target_z_ - current_z_;
      double distance = std::sqrt(dx*dx + dy*dy+ dz*dz);

      //Comprobar parada de emergencia
      if( emergency_stop_active_){

        geometry_msgs::msg::Twist stop_msg;
        cmd_vel_pub_->publish(stop_msg);

        if(notified_obstacle == false){
          feedback->distance_remaining = distance;
          feedback->current_state = "Esperando: Obstáculo bloqueando el camino";
          goal_handle->publish_feedback(feedback);
          notified_obstacle = true;
        }
        loop_rate.sleep();
        continue;
      }
      //Pausar vuelo
      if(is_paused_){
        geometry_msgs::msg::Twist stop_msg;
        cmd_vel_pub_->publish(stop_msg);
        if(pause == false){
          feedback->distance_remaining = distance;
          feedback->current_state ="PAUSADO - Esperando orden de reanudar";
          goal_handle->publish_feedback(feedback);
          pause = true;
        } 
        loop_rate.sleep();

        continue;
      }
      notified_obstacle = false;
      //Comprobación para saber si hemos llegado
      if(distance < 0.1){
        break;
      }
      
      // Transiciones de estado
      if (estado_actual == VOLANDO && (dist_frente_ < 1.5 || dist_diag_izq_ <1.3 || dist_diag_der_ < 1.3)){
        estado_actual = ROTANDO_ESQUIVA;
        // Decisión táctica con chivato en consola
        if ((dist_izq_ + dist_diag_izq_) > (dist_der_ + dist_diag_der_)) {
          direccion_esquiva = 1;
          RCLCPP_WARN(this->get_logger(), "OBSTÁCULO: Frente bloqueado. Hueco a la IZQ (%.2fm > %.2fm). Girando a la izquierda...", dist_izq_, dist_der_);
        } else {
          direccion_esquiva = -1;
          RCLCPP_WARN(this->get_logger(), "OBSTÁCULO: Frente bloqueado. Hueco a la DER (%.2fm > %.2fm). Girando a la derecha...", dist_der_, dist_izq_);
        }
      }
      //Via libre
      else if (estado_actual == ROTANDO_ESQUIVA && dist_frente_ > 4.0){
        estado_actual = RODEANDO_OBSTACULO;
        ciclos_rodeo = 0;
        RCLCPP_INFO(this->get_logger(), "¡Via libre frontal!");
      }

      else if(estado_actual == RODEANDO_OBSTACULO){
        ciclos_rodeo++;
        bool costado_libre = false;
        if (direccion_esquiva == 1) {
          costado_libre = (dist_der_ > 3.5 && dist_diag_der_ > 3.5); // ¿La derecha está libre?
        } else {
          costado_libre = (dist_izq_ > 5.0 && dist_diag_izq_ > 3.5); // ¿La izquierda está libre?
        }
        // Seguridad extra: Si aparece un nuevo obstáculo imprevisto al frente
        if (dist_frente_ < 1.5 || dist_diag_der_ < 1.0 || dist_diag_izq_ < 1.0) {
          estado_actual = ROTANDO_ESQUIVA;
        }
        // Si el costado ya está despejado, significa que dejamos la caja atrás
        else if (costado_libre && dist_atras_ > 2.0 && ciclos_rodeo >35) {
            estado_actual = VOLANDO;
            RCLCPP_INFO(this->get_logger(), "¡Obstáculo rebasado con éxito! Buscando rumbo a la meta...");
        }
      }
      

      //Acciones de estado
      geometry_msgs::msg::Twist cmd_msg;
      
      if (estado_actual == ROTANDO_ESQUIVA){
        cmd_msg.linear.x=0.0;
        cmd_msg.linear.y=0.0;
        cmd_msg.linear.z=0.0;
        cmd_msg.angular.z=0.6 * direccion_esquiva;

        feedback -> current_state = "ROTANDO (Buscando salida)";
      }
      else if(estado_actual == RODEANDO_OBSTACULO){
        cmd_msg.linear.x = 0.5;
        cmd_msg.linear.y = 0.0;
        cmd_msg.linear.z = (dz/distance)*0.5;
        cmd_msg.angular.z = 0.0;

        feedback->current_state = "Rodeando Obstaculo";

      }
      else{
        //Estado VOLANDO
        double dist_horizontal = std::sqrt(dx*dx +dy*dy);

        //Escudo vertical
        if (dist_horizontal < 0.1){
          cmd_msg.linear.x = 0.0;
          cmd_msg.linear.y = 0.0;
          cmd_msg.angular.z = 0.0;
        }
        else
        {
          double angle_to_goal = std::atan2(dy,dx);
          double angle_error = angle_to_goal - current_yaw_;

          while(angle_error > M_PI) angle_error -= 2* M_PI;
          while(angle_error < -M_PI) angle_error += 2* M_PI;
            
          if (std::abs(angle_error) > 0.26){
            cmd_msg.linear.x = 0.0;
            cmd_msg.linear.y = 0.0;
            cmd_msg.angular.z = 0.4*(angle_error > 0 ? 1 : -1);
          }else{
            cmd_msg.linear.x = 0.5;
            cmd_msg.linear.y = 0.0;
            cmd_msg.angular.z = 0.2*angle_error;
          }
      }
        cmd_msg.linear.z = (dz/distance)*0.5;
        feedback->current_state = "Volando hacia destino";
      }
      cmd_vel_pub_->publish(cmd_msg);

      feedback->distance_remaining = distance;
      goal_handle->publish_feedback(feedback);
      loop_rate.sleep();
    }
    
    //Llegada exitosa
    if(rclcpp::ok()){
      //Apagar Motores al llegar
      geometry_msgs::msg::Twist stop_msg;
      cmd_vel_pub_->publish(stop_msg);

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
    q.setRPY(0,0,current_yaw_);
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
      if (emergency_==false){
        RCLCPP_WARN(this->get_logger(), "¡Parada de emergencia activada!");
        emergency_= true;
      }
    } else{
      emergency_stop_active_ = false;
      if (emergency_==true){
        RCLCPP_INFO(this->get_logger(), "Sistema de seguridad restablecido");
        emergency_ = false;
      }
      
    }
  }
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_z_ = msg->pose.pose.position.z;

    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    current_yaw_ = yaw;
  }
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    if(msg->ranges.empty())return;
    auto get_min_distance = [&](int start_idx, int end_idx){
      float min_val = 10.0;
      for(int i = start_idx; i < end_idx; i++){
        float r = msg->ranges[i];
        //Filtro de errores de Gazebo (inf o NaN)
        if(!std::isinf(r) && !std::isnan(r) && r > msg->range_min && r < msg->range_max){
          if (r < min_val){
            min_val = r;
          }
        }
      }
      return min_val;
    };
    dist_der_= get_min_distance(45, 110);
    dist_diag_der_= get_min_distance(110,160);
    dist_frente_=get_min_distance(160,200);
    dist_diag_izq_= get_min_distance(200,250);
    dist_izq_=get_min_distance(250, 315);
    float atras_der= get_min_distance(0,45);
    float atras_izq= get_min_distance(315,360);
    dist_atras_ = std::min(atras_der,atras_izq);

    RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(), 1000,
  "LASER -> Izq: %.2fm | Frente: %2fm | Der: %2fm | DiagIzq: %2fm | DiagDer: %2fm | Atras: %2fm",
   dist_izq_, dist_frente_, dist_der_, dist_diag_izq_, dist_diag_der_, dist_atras_);

  }

};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneBaseNode>());
  rclcpp::shutdown();
  return 0;
}