#include "serial_node.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <algorithm>

namespace serial_node {

SerialNode::SerialNode() : Node("serial_node"), serial_fd_(-1), is_running_(true) {
    
    this->declare_parameter<std::string>("port_name", "/dev/ttyUSB1");
    this->declare_parameter<int>("baud_rate", 115200);
    this->get_parameter("port_name", port_name_);
    this->get_parameter("baud_rate", baud_rate_);
    
    timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&SerialNode::timerCallback, this));

    data_buffer_ = std::make_shared<DataBuffer>();
    yaw_pub_ = std::make_shared<GimbalYawPublisher>(this, data_buffer_);
    game_state_pub_ = std::make_shared<GameStatePublisher>(this, data_buffer_);

    //create subscription for vel & spin_or_not & posture state
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&SerialNode::cmdVelCallback, this, std::placeholders::_1));
    spin_or_not_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "spin_or_not", 10, std::bind(&SerialNode::spinOrNotCallback, this, std::placeholders::_1));
    posture_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "posture", 10,std::bind(&SerialNode::postureCallback,this,std::placeholders::_1));

    initSerial();
    receive_thread_ = std::thread(&SerialNode::receiveThreadFunc, this);

    RCLCPP_INFO(this->get_logger(), "串口初始化成功 Sentry serial started on %s", port_name_.c_str());
    RCLCPP_INFO(
        this->get_logger(),
        "发送/接收协议帧: Tx=%zuB(负载%zuB) ---- Rx=%zuB ---- SOF=0x%02X EOF=0x%02X",
        sizeof(serial_protocol::SerialFrame<serial_protocol::TxData>),
        sizeof(serial_protocol::TxData),
        sizeof(serial_protocol::SerialFrame<serial_protocol::RxData>),
        serial_protocol::kFrameStart,
        serial_protocol::kFrameEnd);
}

SerialNode::~SerialNode() {
    is_running_ = false;
    if (receive_thread_.joinable()) receive_thread_.join();
    if (serial_fd_ >= 0) close(serial_fd_);
}

void SerialNode::initSerial() {
    serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ == -1) {
        RCLCPP_ERROR(this->get_logger(), "Error: Cannot open %s", port_name_.c_str());
        return;
    }

    struct termios options;
    if (tcgetattr(serial_fd_, &options) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error: tcgetattr failed for %s", port_name_.c_str());
        close(serial_fd_);
        serial_fd_ = -1;
        return;
    }

    speed_t speed = B115200;
    switch (baud_rate_) {
        case 115200: speed = B115200; break;
        case 921600: speed = B921600; break;
        case 460800: speed = B460800; break;
        case 230400: speed = B230400; break;
        default:
            RCLCPP_WARN(this->get_logger(), "Unsupported baud_rate=%d, fallback to 115200", baud_rate_);
            speed = B115200;
            break;
    }

    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    // raw 模式，避免行缓冲/回显影响
    cfmakeraw(&options);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cc[VMIN] = 1;
    options.c_cc[VTIME] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &options) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error: tcsetattr failed for %s", port_name_.c_str());
        close(serial_fd_);
        serial_fd_ = -1;
        return;
    }
}

void SerialNode::receiveThreadFunc() {
    constexpr uint8_t sof = serial_protocol::kFrameStart;
    constexpr uint8_t eof = serial_protocol::kFrameEnd;
    constexpr size_t frame_size = sizeof(serial_protocol::SerialFrame<serial_protocol::RxData>);

    std::vector<uint8_t> buf;
    buf.reserve(4096);

    while (is_running_ && rclcpp::ok()) {
        if (serial_fd_ < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        uint8_t tmp[256];
        const int n = static_cast<int>(read(serial_fd_, tmp, sizeof(tmp)));
        if (n > 0) {
            buf.insert(buf.end(), tmp, tmp + n);

            while (true) {
                // 同步：找 SOF(0xFF)
                auto it = std::find(buf.begin(), buf.end(), sof);
                if (it == buf.end()) {
                    // 保留最后 1 字节，避免 SOF 被切分在两次 read 之间
                    if (buf.size() > 1) {
                        buf.erase(buf.begin(), buf.end() - 1);
                    }
                    break;
                }
                if (it != buf.begin()) {
                    buf.erase(buf.begin(), it);
                }

                if (buf.size() < frame_size) break;

                const uint8_t* raw = buf.data();
                serial_protocol::SerialFrame<serial_protocol::RxData> rx_frame;
                std::memcpy(&rx_frame, raw, frame_size);

                if (rx_frame.start != sof || rx_frame.end != eof) {
                    // 当前对齐点不是合法帧：丢掉 1 字节继续同步
                    buf.erase(buf.begin());
                    continue;
                }

                data_buffer_->set(rx_frame.payload);
                yaw_pub_->update();
                game_state_pub_->update();
                
                buf.erase(buf.begin(), buf.begin() + frame_size);
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}


void SerialNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    last_linear_x_ = static_cast<float>(msg->linear.x);
    last_linear_y_ = static_cast<float>(-msg->linear.y);
    last_angular_z_ = static_cast<float>(msg->angular.z);
}

void SerialNode::spinOrNotCallback(const std_msgs::msg::UInt8::SharedPtr msg) {
    spin_or_not_ = msg->data;
}

void SerialNode::postureCallback(const std_msgs::msg::UInt8::SharedPtr msg){
    posture_ = msg->data;
}

void SerialNode::timerCallback(){
    serial_protocol::TxData tx_data;
    tx_data.linear_x = last_linear_x_;
    tx_data.linear_y = last_linear_y_;
    tx_data.angular_z = last_angular_z_;
    tx_data.spin_or_not = spin_or_not_;
    tx_data.posture = posture_;
    sendData(tx_data);
//     RCLCPP_INFO_THROTTLE(
//     this->get_logger(),
//     *this->get_clock(),
//     500,
//     "\n"
//     "-------------------- Tx_Data --------------------\n"
//     "linear_x                  : %.4f\n"
//     "linear_y                  : %.4f\n"
//     "angular_z                  : %.4f\n"
//     "spin_or_not                  : %u\n"
//     "posture                  : %u\n"
//     "----------------------------------------------------",
//     tx_data.linear_x,
//     tx_data.linear_y,
//     tx_data.angular_z,
//     tx_data.spin_or_not,
//     tx_data.posture
//   );
}

bool SerialNode::sendData(const serial_protocol::TxData& data) {
    if (serial_fd_ < 0) return false;
    serial_protocol::SerialFrame<serial_protocol::TxData> frame;
    frame.start = serial_protocol::kFrameStart;
    frame.payload = data;
    frame.end = serial_protocol::kFrameEnd;

    const int n = write(serial_fd_, &frame, sizeof(frame));
    const bool ok = static_cast<size_t>(n) == sizeof(frame);
    return ok;
}

} // namespace serial_node

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<serial_node::SerialNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}