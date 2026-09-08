#include "spr_referee_mock/referee_mock_node.hpp"

namespace spr_referee_mock {

RefereeMockNode::RefereeMockNode() : Node("referee_mock") {
  declare_parameter<std::string>("game_state_topic_name", "game_state");
  declare_parameter<std::string>("spin_or_not_topic_name", "spin_or_not");
  declare_parameter<std::string>("posture_topic_name", "posture");
  declare_parameter<std::string>("cmd_vel_topic_name", "cmd_vel");
  declare_parameter<std::string>("cmd_vel_nav2_topic_name", "cmd_vel_nav2_result");
  declare_parameter<double>("display_stale_timeout_sec", 1.0);
  declare_parameter<double>("publish_rate_hz", 10.0);
  declare_parameter<int>("current_hp", 600);
  declare_parameter<int>("game_progress", 4);
  declare_parameter<int>("stage_remain_time", 420);
  declare_parameter<int>("armor_id", 0);
  declare_parameter<int>("hurt_type", 0);
  declare_parameter<int>("my_outpost_hp", 1500);
  declare_parameter<int>("enemy_outpost_hp", 1500);
  declare_parameter<int>("my_base_hp", 5000);
  declare_parameter<int>("enemy_base_hp", 5000);
  declare_parameter<int>("projectile_allowance_17mm", 500);
  declare_parameter<int>("rfid", 0);

  load_params_to_fields();
  publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
  display_stale_timeout_sec_ = get_parameter("display_stale_timeout_sec").as_double();

  spin_or_not_topic_ = get_parameter("spin_or_not_topic_name").as_string();
  posture_topic_ = get_parameter("posture_topic_name").as_string();
  cmd_vel_topic_ = get_parameter("cmd_vel_topic_name").as_string();
  cmd_vel_nav2_topic_ = get_parameter("cmd_vel_nav2_topic_name").as_string();

  const auto topic = get_parameter("game_state_topic_name").as_string();
  pub_ = create_publisher<rm_interfaces::msg::GameState>(topic, 10);
  setup_decision_subscribers();

  RCLCPP_INFO(
    get_logger(),
    "Referee mock publishing to '%s' at %.1f Hz (bypasses spr_sentry_serial)",
    topic.c_str(), publish_rate_hz_);
  RCLCPP_INFO(
    get_logger(),
    "Monitoring decision outputs: spin='%s', posture='%s', cmd_vel='%s', nav2='%s'",
    spin_or_not_topic_.c_str(), posture_topic_.c_str(),
    cmd_vel_topic_.c_str(), cmd_vel_nav2_topic_.c_str());
}

void RefereeMockNode::setup_decision_subscribers() {
  spin_sub_ = create_subscription<std_msgs::msg::UInt8>(
    spin_or_not_topic_, 10,
    [this](const std_msgs::msg::UInt8::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(decision_mutex_);
      decision_output_.spin.ever_received = true;
      decision_output_.spin.last_stamp = now();
      decision_output_.spin_or_not = msg->data;
    });

  posture_sub_ = create_subscription<std_msgs::msg::UInt8>(
    posture_topic_, 10,
    [this](const std_msgs::msg::UInt8::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(decision_mutex_);
      decision_output_.posture_ch.ever_received = true;
      decision_output_.posture_ch.last_stamp = now();
      decision_output_.posture = msg->data;
    });

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, 10,
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(decision_mutex_);
      decision_output_.cmd_vel.ever_received = true;
      decision_output_.cmd_vel.last_stamp = now();
      decision_output_.cmd_linear_x = msg->linear.x;
      decision_output_.cmd_linear_y = msg->linear.y;
      decision_output_.cmd_angular_z = msg->angular.z;
    });

  cmd_vel_nav2_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_nav2_topic_, 10,
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(decision_mutex_);
      decision_output_.cmd_vel_nav2.ever_received = true;
      decision_output_.cmd_vel_nav2.last_stamp = now();
      decision_output_.nav2_linear_x = msg->linear.x;
      decision_output_.nav2_linear_y = msg->linear.y;
      decision_output_.nav2_angular_z = msg->angular.z;
    });
}

void RefereeMockNode::tick_decision_output() {
  std::lock_guard<std::mutex> lock(decision_mutex_);
  const auto t = now();
  const double timeout = display_stale_timeout_sec_;
  auto update = [&](DecisionOutputSnapshot::Channel& ch) {
    if (!ch.ever_received) {
      ch.stale = true;
      ch.age_sec = 0.0;
      return;
    }
    ch.age_sec = (t - ch.last_stamp).seconds();
    ch.stale = ch.age_sec > timeout;
  };
  update(decision_output_.spin);
  update(decision_output_.posture_ch);
  update(decision_output_.cmd_vel);
  update(decision_output_.cmd_vel_nav2);
}

void RefereeMockNode::load_params_to_fields() {
  GameStateFields f;
  f.current_hp = static_cast<uint16_t>(get_parameter("current_hp").as_int());
  f.game_progress = static_cast<uint8_t>(get_parameter("game_progress").as_int());
  f.stage_remain_time =
    static_cast<uint16_t>(get_parameter("stage_remain_time").as_int());
  f.armor_id = static_cast<uint8_t>(get_parameter("armor_id").as_int());
  f.hurt_type = static_cast<uint8_t>(get_parameter("hurt_type").as_int());
  f.my_outpost_hp = static_cast<uint16_t>(get_parameter("my_outpost_hp").as_int());
  f.enemy_outpost_hp =
    static_cast<uint16_t>(get_parameter("enemy_outpost_hp").as_int());
  f.my_base_hp = static_cast<uint16_t>(get_parameter("my_base_hp").as_int());
  f.enemy_base_hp = static_cast<uint16_t>(get_parameter("enemy_base_hp").as_int());
  f.projectile_allowance_17mm =
    static_cast<uint16_t>(get_parameter("projectile_allowance_17mm").as_int());
  f.rfid = static_cast<uint8_t>(get_parameter("rfid").as_int());
  fields_ = f;
}

rm_interfaces::msg::GameState RefereeMockNode::build_msg(
  const GameStateFields& fields) const {
  rm_interfaces::msg::GameState msg;
  msg.current_hp = fields.current_hp;
  msg.game_progress = fields.game_progress;
  msg.stage_remain_time = fields.stage_remain_time;
  msg.armor_id = fields.armor_id;
  msg.hurt_type = fields.hurt_type;
  msg.my_outpost_hp = fields.my_outpost_hp;
  msg.enemy_outpost_hp = fields.enemy_outpost_hp;
  msg.my_base_hp = fields.my_base_hp;
  msg.enemy_base_hp = fields.enemy_base_hp;
  msg.projectile_allowance_17mm = fields.projectile_allowance_17mm;
  msg.rfid = fields.rfid;
  return msg;
}

void RefereeMockNode::set_fields(const GameStateFields& fields) {
  std::lock_guard<std::mutex> lock(fields_mutex_);
  fields_ = fields;
}

GameStateFields RefereeMockNode::get_fields() const {
  std::lock_guard<std::mutex> lock(fields_mutex_);
  return fields_;
}

void RefereeMockNode::publish_once() {
  GameStateFields snapshot;
  {
    std::lock_guard<std::mutex> lock(fields_mutex_);
    snapshot = fields_;
  }
  pub_->publish(build_msg(snapshot));
}

double RefereeMockNode::publish_rate_hz() const { return publish_rate_hz_; }

void RefereeMockNode::set_publish_rate_hz(double hz) {
  publish_rate_hz_ = hz > 0.1 ? hz : 0.1;
}

DecisionOutputSnapshot RefereeMockNode::get_decision_output() const {
  std::lock_guard<std::mutex> lock(decision_mutex_);
  return decision_output_;
}

std::string RefereeMockNode::spin_or_not_topic() const { return spin_or_not_topic_; }

std::string RefereeMockNode::posture_topic() const { return posture_topic_; }

std::string RefereeMockNode::cmd_vel_topic() const { return cmd_vel_topic_; }

std::string RefereeMockNode::cmd_vel_nav2_topic() const { return cmd_vel_nav2_topic_; }

}  // namespace spr_referee_mock
