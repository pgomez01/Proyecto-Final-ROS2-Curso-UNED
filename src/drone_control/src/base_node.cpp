#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

class DroneBaseNode : public rclcpp::Node {
public:
  DroneBaseNode() : Node("base_node"), x_pos_(0.0) {
    // Inicializar el publicador de transformaciones
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // Timer para mover el dron a 10Hz
    timer_ = this->create_wall_timer(100ms, std::bind(&DroneBaseNode::broadcast_timer_callback, this));
  }

private:
  void broadcast_timer_callback() {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";       // Marco fijo del mundo [cite: 26]
    t.child_frame_id = "base_link";   // Marco del dron

    // Simular movimiento lineal simple
    x_pos_ += 0.05; 
    t.transform.translation.x = x_pos_;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 1.0; // Altura de vuelo [cite: 22]

    // Sin rotación por ahora (cuaternión identidad)
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(t);
  }

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  double x_pos_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneBaseNode>());
  rclcpp::shutdown();
  return 0;
}