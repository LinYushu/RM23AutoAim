#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>

#include "std_msgs/msg/char.hpp"

// C++ system
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "auto_aim_interfaces/msg/target.hpp"

namespace rm_serial_driver
{
#define M_PI 3.14159265358979323846

class RMSerialDriver : public rclcpp::Node
{
public:
  explicit RMSerialDriver(const rclcpp::NodeOptions & options);

  ~RMSerialDriver() override;

private:
  inline double deg2rad(float deg) { return deg / 180.0 * M_PI; }

  inline double rad2deg(float rad) { return rad / M_PI * 180.0; }

  void getParams();

  void receiveData();

  void sendData(auto_aim_interfaces::msg::Target::SharedPtr msg);

  void reopenPort();

  //- io互斥锁
  std::unique_ptr<IoContext> owned_ctx_;

  //- 串口名称
  std::string device_name_;

  //- serial driver配置参数和serialDriver
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

  //time offset
  double timestamp_offset_ = 0;
  //- 发布状态
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr enemy_color_pub_;

  //- 订阅
  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;

  //发送弹速
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bullet_speed_pub_;

  //Publisher latency
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;

  //- 测试1 通信timer发布消息
  rclcpp::TimerBase::SharedPtr mTimer;
  void pubMsgCallback();

  std::thread receive_thread_;
};
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
