#include "serial_node.hpp"

namespace serial_node {

GameStatePublisher::GameStatePublisher(rclcpp::Node* node, std::shared_ptr<DataBuffer> buffer)
: node_(node), buffer_(std::move(buffer)) {
  pub_ = node_->create_publisher<rm_interfaces::msg::GameState>("game_state", 10);
}

void GameStatePublisher::update() {
  if (!buffer_ || !buffer_->hasData()) return;
  const auto rx = buffer_->get();


  RCLCPP_INFO_THROTTLE(
    node_->get_logger(),
    *node_->get_clock(),
    500,
    "\n"
    "-------------------- game_state --------------------\n"
    "current_hp                  : %u\n"
    "game_progress               : %u\n"
    "stage_remain_time           : %u\n"
    "my_outpost_hp               : %u\n"
    "enemy_outpost_hp            : %u\n"
    "my_base_hp                  : %u\n"
    "enemy_base_hp               : %u\n"
    "projectile_allowance_17mm   : %u\n"
    "rfid                        : %u\n"
    "----------------------------------------------------",
    rx.current_hp,
    rx.game_progress,
    rx.stage_remain_time,
    rx.my_outpost_hp,
    rx.enemy_outpost_hp,
    rx.my_base_hp,
    rx.enemy_base_hp,
    rx.projectile_allowance_17mm,
    rx.rfid
  );

  rm_interfaces::msg::GameState msg;
  msg.current_hp = rx.current_hp;
  msg.game_progress = rx.game_progress;
  msg.stage_remain_time = rx.stage_remain_time;
  msg.armor_id = 0;
  msg.hurt_type = 0;

  msg.my_outpost_hp = rx.my_outpost_hp;
  msg.enemy_outpost_hp = rx.enemy_outpost_hp;
  msg.my_base_hp = rx.my_base_hp;
  msg.enemy_base_hp = rx.enemy_base_hp;

  msg.projectile_allowance_17mm = rx.projectile_allowance_17mm;
  msg.rfid = rx.rfid;

  pub_->publish(msg);
}

}  // namespace serial_node

