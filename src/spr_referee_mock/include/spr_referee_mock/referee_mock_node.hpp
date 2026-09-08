#ifndef SPR_REFEREE_MOCK__REFEREE_MOCK_NODE_HPP_
#define SPR_REFEREE_MOCK__REFEREE_MOCK_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/game_state.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace spr_referee_mock {

struct DecisionOutputSnapshot {
  struct Channel {
    bool ever_received{false};
    bool stale{true};
    double age_sec{0.0};
    rclcpp::Time last_stamp{};
  };

  Channel spin;
  uint8_t spin_or_not{0};

  Channel posture_ch;
  uint8_t posture{0};

  Channel cmd_vel;
  double cmd_linear_x{0.0};
  double cmd_linear_y{0.0};
  double cmd_angular_z{0.0};

  Channel cmd_vel_nav2;
  double nav2_linear_x{0.0};
  double nav2_linear_y{0.0};
  double nav2_angular_z{0.0};
};

struct GameStateFields {
  uint16_t current_hp{600};
  uint8_t game_progress{4};
  uint16_t stage_remain_time{420};
  uint8_t armor_id{0};
  uint8_t hurt_type{0};
  uint16_t my_outpost_hp{1500};
  uint16_t enemy_outpost_hp{1500};
  uint16_t my_base_hp{5000};
  uint16_t enemy_base_hp{5000};
  uint16_t projectile_allowance_17mm{500};
  uint8_t rfid{0};
};

class RefereeMockNode : public rclcpp::Node {
 public:
  RefereeMockNode();

  void set_fields(const GameStateFields& fields);
  GameStateFields get_fields() const;
  void publish_once();

  double publish_rate_hz() const;
  void set_publish_rate_hz(double hz);

  void tick_decision_output();
  DecisionOutputSnapshot get_decision_output() const;
  std::string spin_or_not_topic() const;
  std::string posture_topic() const;
  std::string cmd_vel_topic() const;
  std::string cmd_vel_nav2_topic() const;

 private:
  rm_interfaces::msg::GameState build_msg(const GameStateFields& fields) const;
  void load_params_to_fields();
  void setup_decision_subscribers();

  rclcpp::Publisher<rm_interfaces::msg::GameState>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr spin_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr posture_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_nav2_sub_;

  mutable std::mutex fields_mutex_;
  mutable std::mutex decision_mutex_;
  GameStateFields fields_;
  DecisionOutputSnapshot decision_output_;
  double publish_rate_hz_{10.0};
  double display_stale_timeout_sec_{1.0};

  std::string spin_or_not_topic_;
  std::string posture_topic_;
  std::string cmd_vel_topic_;
  std::string cmd_vel_nav2_topic_;
};

}  // namespace spr_referee_mock

#endif  // SPR_REFEREE_MOCK__REFEREE_MOCK_NODE_HPP_
