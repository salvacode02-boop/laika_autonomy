#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <unitree_go/msg/low_state.hpp>

// /lowstate (IMU del cuerpo) -> sensor_msgs/Imu. Solo suscribe /lowstate; no publica nada hacia el robot.
class LowstateImuRelay : public rclcpp::Node {
public:
  LowstateImuRelay() : Node("lowstate_imu_relay") {
    in_topic_ = declare_parameter<std::string>("lowstate_topic", "/lowstate");
    out_topic_ = declare_parameter<std::string>("out_topic", "/laika/body_imu");
    frame_ = declare_parameter<std::string>("frame_id", "base_link");
    // +-10 ms por movimiento lento; re-medir con un bag de movimiento rapido
    latency_ = declare_parameter<double>("latency", 0.0);
    pub_ = create_publisher<sensor_msgs::msg::Imu>(out_topic_, rclcpp::SensorDataQoS().keep_last(10));
    sub_ = create_subscription<unitree_go::msg::LowState>(
        in_topic_, rclcpp::SensorDataQoS().keep_last(10),
        [this](unitree_go::msg::LowState::ConstSharedPtr m) {
          sensor_msgs::msg::Imu o;
          o.header.stamp = now() - rclcpp::Duration::from_seconds(latency_);
          o.header.frame_id = frame_;
          const auto &s = m->imu_state;
          o.orientation.x = s.quaternion[1]; o.orientation.y = s.quaternion[2];
          o.orientation.z = s.quaternion[3]; o.orientation.w = s.quaternion[0];  // Unitree: wxyz
          o.angular_velocity.x = s.gyroscope[0]; o.angular_velocity.y = s.gyroscope[1];
          o.angular_velocity.z = s.gyroscope[2];
          o.linear_acceleration.x = s.accelerometer[0]; o.linear_acceleration.y = s.accelerometer[1];
          o.linear_acceleration.z = s.accelerometer[2];
          pub_->publish(o);
        });
  }

private:
  std::string in_topic_, out_topic_, frame_;
  double latency_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LowstateImuRelay>());
  rclcpp::shutdown();
  return 0;
}
