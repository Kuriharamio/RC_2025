#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include "serial/serial.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp" // 引入正确的头文件
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"

using std::placeholders::_1;

serial::Serial ros_serial;

float test_send_buffer[2] = {66, 77};

class SerialToSTM32 : public rclcpp::Node
{
public:
  SerialToSTM32()
      : Node("serial_to_stm32")
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 假设位置数据（x, y）通过 "location" 话题传输，使用 Point 类型消息
    sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "location", 10, std::bind(&SerialToSTM32::topic_callback, this, _1)); // 创建订阅location话题的订阅者

    // 创建定时器，每0.5秒更新test_send_buffer并发送
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&SerialToSTM32::timer_callback, this));
  }

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_; // 定时器

  void topic_callback(const geometry_msgs::msg::Point::SharedPtr msg) const
{
    float x = msg->x;
    float y = msg->y;
    
    // 将两个 float 拆解成字节数组
    uint8_t buffer[10] = {0xAA, 0x55, 0};  // 头帧
    memcpy(buffer + 2, &x, 4);
    memcpy(buffer + 6, &y, 4);

    // 打印调试信息
    std::cout << "Sending float data: x = " << x << ", y = " << y << std::endl;
    for (int i = 0; i < 8; i++) {
        std::cout << std::hex << (buffer[i] & 0xff) << " ";
    }
    std::cout << std::endl;

    if (ros_serial.isOpen()) {
        ros_serial.write(buffer, sizeof(buffer));
    } else {
        RCLCPP_ERROR(this->get_logger(), "Serial port not open. Skipping write.");
    }
}


void timer_callback()
{
  float x(0), y(0);
  try {
      auto right_laser_geom_ = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
      
      x = right_laser_geom_.transform.translation.x;
      y = right_laser_geom_.transform.translation.y;
      

  } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
      return;
  }


    // 测试用 float 数据（模拟坐标）
    // float x = test_send_buffer[0];
    // float y = test_send_buffer[1];

    // 打印浮点值
    std::cout << "Sending float: x = " << x << ", y = " << y << std::endl;

    // 将 float 拆成字节发送
    uint8_t buffer[10] = {0xAA, 0x55, 0};  // 头帧
    memcpy(buffer + 2, &x, 4);
    memcpy(buffer + 6, &y, 4);

    // 打印字节内容用于调试
    std::cout << "Byte buffer: ";
    for (int i = 0; i < 10; i++)
    {
        std::cout << std::hex << (buffer[i] & 0xff) << " ";
    }
    std::cout << std::endl;

    // 发送数据
    if (ros_serial.isOpen()) {
      ros_serial.write(buffer, sizeof(buffer));
    }
    // 测试值递增
    test_send_buffer[0] += 0.01;
    test_send_buffer[1] += 0.01;

    if (test_send_buffer[0] > 1000.0f)
        test_send_buffer[0] = 66.0f;
    if (test_send_buffer[1] > 1000.0f)
        test_send_buffer[1] = 77.0f;
}

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // 初始化串口配置
  ros_serial.setPort("/dev/ttyUSB0");
  ros_serial.setBaudrate(115200);
  serial::Timeout to = serial::Timeout::simpleTimeout(1000);
  ros_serial.setTimeout(to);

  try
  {
    ros_serial.open();
  }
  catch (serial::IOException &e)
  {
    std::cout << "Unable to open stm32 serial port" << std::endl;
    return -1;
  }

  if (ros_serial.isOpen())
  {
    std::cout << "Serial port opened successfully" << std::endl;
  }
  else
  {
    return -1;
  }

  // 创建并启动 ROS 2 节点
  auto node = std::make_shared<SerialToSTM32>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  ros_serial.close();

  return 0;
}