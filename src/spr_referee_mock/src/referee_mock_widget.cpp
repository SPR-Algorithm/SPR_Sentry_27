#include "spr_referee_mock/referee_mock_widget.hpp"

#include <QFormLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>

namespace spr_referee_mock {

namespace {
constexpr int kSpinMaxHp = 10000;

QString spin_text(uint8_t v) {
  switch (v) {
    case 0: return QObject::tr("关 (0)");
    case 1: return QObject::tr("开 (1)");
    default: return QObject::tr("未知 (%1)").arg(v);
  }
}

QString posture_text(uint8_t v) {
  switch (v) {
    case 1: return QObject::tr("进攻 (1)");
    case 2: return QObject::tr("防御 (2)");
    case 3: return QObject::tr("移动 (3)");
    default: return QObject::tr("未知 (%1)").arg(v);
  }
}

QSpinBox* make_u16_spin(int max_val, QWidget* parent) {
  auto* spin = new QSpinBox(parent);
  spin->setRange(0, max_val);
  return spin;
}
}  // namespace

RefereeMockWidget::RefereeMockWidget(const std::shared_ptr<RefereeMockNode>& node,
                                     QWidget* parent)
: QMainWindow(parent), node_(node) {
  build_ui();
  sync_ui_from_node();

  connect(&spin_timer_, &QTimer::timeout, this, [this]() {
    if (node_) {
      rclcpp::spin_some(node_);
      node_->tick_decision_output();
      refresh_decision_display();
    }
  });
  spin_timer_.start(20);

  connect(&publish_timer_, &QTimer::timeout, this, &RefereeMockWidget::on_spin_timer);
}

void RefereeMockWidget::build_ui() {
  setWindowTitle(tr("SPR 裁判模拟 & 决策输出监控"));
  resize(520, 780);

  auto* central = new QWidget(this);
  auto* root = new QVBoxLayout(central);

  auto* hint = new QLabel(
    tr("直接发布 game_state 至决策树 Topics2Blackboard，无需 spr_sentry_serial。\n"
       "game_progress: 0未开始 1准备 2自检 3倒计时 4对战 5结算"),
    central);
  hint->setWordWrap(true);
  root->addWidget(hint);

  auto* form_group = new QGroupBox(tr("GameState 字段"), central);
  auto* form = new QFormLayout(form_group);

  game_progress_combo_ = new QComboBox(form_group);
  game_progress_combo_->addItem(tr("0 - 未开始"), 0);
  game_progress_combo_->addItem(tr("1 - 准备"), 1);
  game_progress_combo_->addItem(tr("2 - 自检"), 2);
  game_progress_combo_->addItem(tr("3 - 倒计时"), 3);
  game_progress_combo_->addItem(tr("4 - 对战"), 4);
  game_progress_combo_->addItem(tr("5 - 结算"), 5);
  form->addRow(tr("比赛阶段"), game_progress_combo_);

  current_hp_spin_ = make_u16_spin(kSpinMaxHp, form_group);
  form->addRow(tr("本机血量"), current_hp_spin_);

  stage_remain_time_spin_ = make_u16_spin(65535, form_group);
  form->addRow(tr("阶段剩余时间(s)"), stage_remain_time_spin_);

  armor_id_spin_ = make_u16_spin(255, form_group);
  form->addRow(tr("装甲板 ID"), armor_id_spin_);

  hurt_type_spin_ = make_u16_spin(255, form_group);
  form->addRow(tr("受击类型"), hurt_type_spin_);

  my_outpost_hp_spin_ = make_u16_spin(kSpinMaxHp, form_group);
  form->addRow(tr("己方前哨 HP"), my_outpost_hp_spin_);

  enemy_outpost_hp_spin_ = make_u16_spin(kSpinMaxHp, form_group);
  form->addRow(tr("敌方前哨 HP"), enemy_outpost_hp_spin_);

  my_base_hp_spin_ = make_u16_spin(kSpinMaxHp, form_group);
  form->addRow(tr("己方基地 HP"), my_base_hp_spin_);

  enemy_base_hp_spin_ = make_u16_spin(kSpinMaxHp, form_group);
  form->addRow(tr("敌方基地 HP"), enemy_base_hp_spin_);

  projectile_spin_ = make_u16_spin(kSpinMaxHp, form_group);
  form->addRow(tr("17mm 剩余弹量"), projectile_spin_);

  rfid_spin_ = make_u16_spin(255, form_group);
  form->addRow(tr("RFID"), rfid_spin_);

  rate_spin_ = new QDoubleSpinBox(form_group);
  rate_spin_->setRange(0.5, 50.0);
  rate_spin_->setSingleStep(0.5);
  rate_spin_->setSuffix(tr(" Hz"));
  form->addRow(tr("发布频率"), rate_spin_);

  root->addWidget(form_group);

  auto* decision_group = new QGroupBox(tr("决策树输出监控（只读，RViz 不可见）"), central);
  auto* decision_form = new QFormLayout(decision_group);

  spin_value_label_ = new QLabel(tr("等待 spin_or_not…"), decision_group);
  spin_value_label_->setWordWrap(true);
  posture_value_label_ = new QLabel(tr("等待 posture…"), decision_group);
  posture_value_label_->setWordWrap(true);
  cmd_vel_value_label_ = new QLabel(tr("等待 cmd_vel…"), decision_group);
  cmd_vel_value_label_->setWordWrap(true);
  cmd_vel_nav2_value_label_ = new QLabel(tr("等待 cmd_vel_nav2_result…"), decision_group);
  cmd_vel_nav2_value_label_->setWordWrap(true);

  decision_form->addRow(tr("小陀螺 spin_or_not"), spin_value_label_);
  decision_form->addRow(tr("姿态 posture"), posture_value_label_);
  decision_form->addRow(tr("底盘速度 cmd_vel"), cmd_vel_value_label_);
  decision_form->addRow(tr("Nav2/对齐速度"), cmd_vel_nav2_value_label_);

  decision_topics_label_ = new QLabel(decision_group);
  decision_topics_label_->setWordWrap(true);
  decision_topics_label_->setStyleSheet("color: gray; font-size: 11px;");
  decision_form->addRow(tr("订阅话题"), decision_topics_label_);

  root->addWidget(decision_group);

  auto* preset_group = new QGroupBox(tr("快捷场景 (RMUC_adaptation_2026)"), central);
  auto* preset_layout = new QVBoxLayout(preset_group);
  auto* preset_row1 = new QHBoxLayout();
  auto* preset_row2 = new QHBoxLayout();
  auto* btn_battle = new QPushButton(tr("对战默认"), preset_group);
  auto* btn_countdown = new QPushButton(tr("倒计时"), preset_group);
  auto* btn_outpost = new QPushButton(tr("敌方前哨击破"), preset_group);
  auto* btn_supply = new QPushButton(tr("需提前补给"), preset_group);
  auto* btn_healed = new QPushButton(tr("补给完成"), preset_group);
  preset_row1->addWidget(btn_battle);
  preset_row1->addWidget(btn_countdown);
  preset_row1->addWidget(btn_outpost);
  preset_row2->addWidget(btn_supply);
  preset_row2->addWidget(btn_healed);
  preset_layout->addLayout(preset_row1);
  preset_layout->addLayout(preset_row2);
  root->addWidget(preset_group);

  auto* ctrl_row = new QHBoxLayout();
  apply_btn_ = new QPushButton(tr("立即发布一次"), central);
  auto_publish_btn_ = new QPushButton(tr("自动周期发布"), central);
  auto_publish_btn_->setCheckable(true);
  ctrl_row->addWidget(apply_btn_);
  ctrl_row->addWidget(auto_publish_btn_);
  root->addLayout(ctrl_row);

  status_label_ = new QLabel(tr("就绪"), central);
  root->addWidget(status_label_);

  setCentralWidget(central);

  connect(apply_btn_, &QPushButton::clicked, this, &RefereeMockWidget::on_apply_clicked);
  connect(auto_publish_btn_, &QPushButton::toggled, this,
    &RefereeMockWidget::on_auto_publish_toggled);
  connect(btn_battle, &QPushButton::clicked, this, &RefereeMockWidget::on_preset_battle);
  connect(btn_countdown, &QPushButton::clicked, this, &RefereeMockWidget::on_preset_countdown);
  connect(btn_outpost, &QPushButton::clicked, this,
    &RefereeMockWidget::on_preset_enemy_outpost_down);
  connect(btn_supply, &QPushButton::clicked, this, &RefereeMockWidget::on_preset_early_supply);
  connect(btn_healed, &QPushButton::clicked, this, &RefereeMockWidget::on_preset_healed_at_supply);
}

void RefereeMockWidget::sync_ui_from_node() {
  if (!node_) {
    return;
  }
  write_fields_to_ui(node_->get_fields());
  rate_spin_->setValue(node_->publish_rate_hz());
  decision_topics_label_->setText(
    tr("spin: %1 | posture: %2 | cmd_vel: %3 | nav2: %4")
      .arg(QString::fromStdString(node_->spin_or_not_topic()))
      .arg(QString::fromStdString(node_->posture_topic()))
      .arg(QString::fromStdString(node_->cmd_vel_topic()))
      .arg(QString::fromStdString(node_->cmd_vel_nav2_topic())));
  refresh_decision_display();
}

QString RefereeMockWidget::format_channel_value(
  const DecisionOutputSnapshot::Channel& ch, const QString& text) const {
  if (!ch.ever_received) {
    return tr("%1 — 等待数据…").arg(text);
  }
  if (ch.stale) {
    return tr("%1 — 已超时 (%2s 未更新)").arg(text).arg(ch.age_sec, 0, 'f', 2);
  }
  return tr("%1 — 实时 (%2s 前)").arg(text).arg(ch.age_sec, 0, 'f', 2);
}

void RefereeMockWidget::set_channel_label(
  QLabel* label, const DecisionOutputSnapshot::Channel& ch, const QString& text) {
  label->setText(format_channel_value(ch, text));
  if (!ch.ever_received) {
    label->setStyleSheet("color: #888; font-weight: normal;");
  } else if (ch.stale) {
    label->setStyleSheet("color: #c60; font-weight: normal;");
  } else {
    label->setStyleSheet("color: #080; font-weight: bold;");
  }
}

void RefereeMockWidget::refresh_decision_display() {
  if (!node_) {
    return;
  }
  const auto out = node_->get_decision_output();

  set_channel_label(
    spin_value_label_, out.spin,
    spin_text(out.spin_or_not));

  set_channel_label(
    posture_value_label_, out.posture_ch,
    posture_text(out.posture));

  const QString cmd_vel_text = tr("vx=%1, vy=%2, ωz=%3")
    .arg(out.cmd_linear_x, 0, 'f', 3)
    .arg(out.cmd_linear_y, 0, 'f', 3)
    .arg(out.cmd_angular_z, 0, 'f', 3);
  set_channel_label(cmd_vel_value_label_, out.cmd_vel, cmd_vel_text);

  const QString nav2_text = tr("vx=%1, vy=%2, ωz=%3")
    .arg(out.nav2_linear_x, 0, 'f', 3)
    .arg(out.nav2_linear_y, 0, 'f', 3)
    .arg(out.nav2_angular_z, 0, 'f', 3);
  set_channel_label(cmd_vel_nav2_value_label_, out.cmd_vel_nav2, nav2_text);
}

void RefereeMockWidget::sync_node_from_ui() {
  if (!node_) {
    return;
  }
  node_->set_fields(read_fields_from_ui());
  node_->set_publish_rate_hz(rate_spin_->value());
}

GameStateFields RefereeMockWidget::read_fields_from_ui() const {
  GameStateFields f;
  f.game_progress =
    static_cast<uint8_t>(game_progress_combo_->currentData().toInt());
  f.current_hp = static_cast<uint16_t>(current_hp_spin_->value());
  f.stage_remain_time = static_cast<uint16_t>(stage_remain_time_spin_->value());
  f.armor_id = static_cast<uint8_t>(armor_id_spin_->value());
  f.hurt_type = static_cast<uint8_t>(hurt_type_spin_->value());
  f.my_outpost_hp = static_cast<uint16_t>(my_outpost_hp_spin_->value());
  f.enemy_outpost_hp = static_cast<uint16_t>(enemy_outpost_hp_spin_->value());
  f.my_base_hp = static_cast<uint16_t>(my_base_hp_spin_->value());
  f.enemy_base_hp = static_cast<uint16_t>(enemy_base_hp_spin_->value());
  f.projectile_allowance_17mm = static_cast<uint16_t>(projectile_spin_->value());
  f.rfid = static_cast<uint8_t>(rfid_spin_->value());
  return f;
}

void RefereeMockWidget::write_fields_to_ui(const GameStateFields& f) {
  const int idx = game_progress_combo_->findData(f.game_progress);
  if (idx >= 0) {
    game_progress_combo_->setCurrentIndex(idx);
  }
  current_hp_spin_->setValue(f.current_hp);
  stage_remain_time_spin_->setValue(f.stage_remain_time);
  armor_id_spin_->setValue(f.armor_id);
  hurt_type_spin_->setValue(f.hurt_type);
  my_outpost_hp_spin_->setValue(f.my_outpost_hp);
  enemy_outpost_hp_spin_->setValue(f.enemy_outpost_hp);
  my_base_hp_spin_->setValue(f.my_base_hp);
  enemy_base_hp_spin_->setValue(f.enemy_base_hp);
  projectile_spin_->setValue(f.projectile_allowance_17mm);
  rfid_spin_->setValue(f.rfid);
}

void RefereeMockWidget::on_apply_clicked() {
  sync_node_from_ui();
  node_->publish_once();
  status_label_->setText(tr("已手动发布 game_state"));
}

void RefereeMockWidget::on_auto_publish_toggled(bool enabled) {
  sync_node_from_ui();
  if (enabled) {
    const int ms = static_cast<int>(1000.0 / node_->publish_rate_hz());
    publish_timer_.start(ms > 0 ? ms : 100);
    status_label_->setText(tr("自动发布中…"));
  } else {
    publish_timer_.stop();
    status_label_->setText(tr("自动发布已停止"));
  }
}

void RefereeMockWidget::on_spin_timer() {
  sync_node_from_ui();
  node_->publish_once();
}

void RefereeMockWidget::on_preset_battle() {
  GameStateFields f;
  f.game_progress = 4;
  f.current_hp = 600;
  f.stage_remain_time = 420;
  f.my_outpost_hp = 1500;
  f.enemy_outpost_hp = 1500;
  f.my_base_hp = 5000;
  f.enemy_base_hp = 5000;
  f.projectile_allowance_17mm = 500;
  write_fields_to_ui(f);
  sync_node_from_ui();
  node_->publish_once();
  status_label_->setText(tr("预设: 对战默认"));
}

void RefereeMockWidget::on_preset_countdown() {
  GameStateFields f = read_fields_from_ui();
  f.game_progress = 3;
  f.stage_remain_time = 10;
  write_fields_to_ui(f);
  sync_node_from_ui();
  node_->publish_once();
  status_label_->setText(tr("预设: 倒计时"));
}

void RefereeMockWidget::on_preset_enemy_outpost_down() {
  GameStateFields f = read_fields_from_ui();
  f.game_progress = 4;
  f.enemy_outpost_hp = 0;
  write_fields_to_ui(f);
  sync_node_from_ui();
  node_->publish_once();
  status_label_->setText(tr("预设: 敌方前哨击破"));
}

void RefereeMockWidget::on_preset_early_supply() {
  GameStateFields f = read_fields_from_ui();
  f.game_progress = 4;
  f.enemy_outpost_hp = 1500;
  f.current_hp = 100;
  f.projectile_allowance_17mm = 0;
  write_fields_to_ui(f);
  sync_node_from_ui();
  node_->publish_once();
  status_label_->setText(tr("预设: 低血量/无弹量需补给"));
}

void RefereeMockWidget::on_preset_healed_at_supply() {
  GameStateFields f = read_fields_from_ui();
  f.game_progress = 4;
  f.enemy_outpost_hp = 0;
  f.current_hp = 400;
  f.projectile_allowance_17mm = 500;
  write_fields_to_ui(f);
  sync_node_from_ui();
  node_->publish_once();
  status_label_->setText(tr("预设: 补给区回血完成 (HP>=350)"));
}

}  // namespace spr_referee_mock
