#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp" // Mensaje estándar para LIDAR
#include "std_msgs/msg/bool.hpp"

class SeguridadNode : public rclcpp::Node {
public:
  SeguridadNode() : Node("seguridad_node"), obstacle_detected_(false) {
    // 1. Publicador: Envía la señal de parada
    pub_stop_ = this->create_publisher<std_msgs::msg::Bool>("emergency_stop", 10);
    
    // 2. Suscriptor: Escucha al sensor LIDAR
    sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(), std::bind(&SeguridadNode::scan_callback, this, std::placeholders::_1));
      
    RCLCPP_INFO(this->get_logger(), "Sistema de seguridad LIDAR activado. Rango seguro: 1.5m");
  }

private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_stop_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  bool obstacle_detected_;

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    bool danger = false;

    // Analizamos cada rayo láser del array
    for (float range : msg->ranges) {
      // Ignoramos valores infinitos o nulos típicos de los sensores
      if (std::isnan(range) || std::isinf(range)) {
        continue;
      }
      
      // Si algún rayo choca a menos de 1.5 metros...
      if (range < 1.5 && range > msg->range_min) {
        danger = true;
        break; // No hace falta mirar más, ¡ya hay peligro!
      }
    }

    // Publicamos el estado constantemente
    auto stop_msg = std_msgs::msg::Bool();
    stop_msg.data = danger;
    pub_stop_->publish(stop_msg);

    // Logs limpios (solo avisa cuando cambia la situación)
    if (danger && !obstacle_detected_) {
      RCLCPP_WARN(this->get_logger(), "¡OBSTÁCULO DETECTADO! Activando parada de emergencia.");
      obstacle_detected_ = true;
    } else if (!danger && obstacle_detected_) {
      RCLCPP_INFO(this->get_logger(), "Camino despejado. Desactivando bloqueo.");
      obstacle_detected_ = false;
    }
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SeguridadNode>());
  rclcpp::shutdown();
  return 0;
}