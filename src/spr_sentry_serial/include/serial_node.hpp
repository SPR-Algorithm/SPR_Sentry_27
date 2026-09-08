#ifndef SERIAL_NODE_HPP
#define SERIAL_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rm_interfaces/msg/game_state.hpp>
#include "protocol.hpp"
#include "serial_node/data_buffer.hpp"

#include <cstdint>
#include <string>
#include <thread>
#include <atomic>
#include <memory>

namespace serial_node {

class GimbalYawPublisher {
public:
    GimbalYawPublisher(rclcpp::Node* node, std::shared_ptr<DataBuffer> buffer);
    void update();

private:
    rclcpp::Node* node_;
    std::shared_ptr<DataBuffer> buffer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
};

class GameStatePublisher {
public:
    GameStatePublisher(rclcpp::Node* node, std::shared_ptr<DataBuffer> buffer);
    void update();

private:
    rclcpp::Node* node_;
    std::shared_ptr<DataBuffer> buffer_;
    rclcpp::Publisher<rm_interfaces::msg::GameState>::SharedPtr pub_;
};

class SerialNode : public rclcpp::Node {
public:
    SerialNode();
    ~SerialNode();

private:
    std::string port_name_;
    int baud_rate_;
    int serial_fd_;

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr spin_or_not_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr posture_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    //缓冲区用来存储接收到的数据
    std::shared_ptr<DataBuffer> data_buffer_;

    float last_linear_x_{0.F};
    float last_linear_y_{0.F};
    float last_angular_z_{0.F};
    uint8_t spin_or_not_{0};
    uint8_t posture_{3};
    uint8_t status_{0};
    std::shared_ptr<GimbalYawPublisher> yaw_pub_;
    std::shared_ptr<GameStatePublisher> game_state_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::thread receive_thread_;
    std::atomic<bool> is_running_;

    void initSerial();
    void receiveThreadFunc();
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void spinOrNotCallback(const std_msgs::msg::UInt8::SharedPtr msg);
    void postureCallback(const std_msgs::msg::UInt8::SharedPtr msg);
    void timerCallback();
    bool sendData(const serial_protocol::TxData& data);
};

} //namespace serial_node
#endif