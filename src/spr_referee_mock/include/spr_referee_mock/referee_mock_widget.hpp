#ifndef SPR_REFEREE_MOCK__REFEREE_MOCK_WIDGET_HPP_
#define SPR_REFEREE_MOCK__REFEREE_MOCK_WIDGET_HPP_

#include <memory>

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QSpinBox>
#include <QTimer>

#include "spr_referee_mock/referee_mock_node.hpp"

namespace spr_referee_mock {

class RefereeMockWidget : public QMainWindow {
  Q_OBJECT

 public:
  explicit RefereeMockWidget(const std::shared_ptr<RefereeMockNode>& node,
                             QWidget* parent = nullptr);

 private slots:
  void on_apply_clicked();
  void on_auto_publish_toggled(bool enabled);
  void on_preset_battle();
  void on_preset_countdown();
  void on_preset_enemy_outpost_down();
  void on_preset_early_supply();
  void on_preset_healed_at_supply();
  void on_spin_timer();
  void refresh_decision_display();

 private:
  void build_ui();
  void sync_ui_from_node();
  QString format_channel_value(const DecisionOutputSnapshot::Channel& ch,
    const QString& text) const;
  void set_channel_label(QLabel* label, const DecisionOutputSnapshot::Channel& ch,
    const QString& text);
  void sync_node_from_ui();
  GameStateFields read_fields_from_ui() const;
  void write_fields_to_ui(const GameStateFields& fields);

  std::shared_ptr<RefereeMockNode> node_;
  QTimer spin_timer_;
  QTimer publish_timer_;

  QComboBox* game_progress_combo_{nullptr};
  QSpinBox* current_hp_spin_{nullptr};
  QSpinBox* stage_remain_time_spin_{nullptr};
  QSpinBox* armor_id_spin_{nullptr};
  QSpinBox* hurt_type_spin_{nullptr};
  QSpinBox* my_outpost_hp_spin_{nullptr};
  QSpinBox* enemy_outpost_hp_spin_{nullptr};
  QSpinBox* my_base_hp_spin_{nullptr};
  QSpinBox* enemy_base_hp_spin_{nullptr};
  QSpinBox* projectile_spin_{nullptr};
  QSpinBox* rfid_spin_{nullptr};
  QDoubleSpinBox* rate_spin_{nullptr};
  QPushButton* apply_btn_{nullptr};
  QPushButton* auto_publish_btn_{nullptr};
  QLabel* status_label_{nullptr};

  QLabel* spin_value_label_{nullptr};
  QLabel* posture_value_label_{nullptr};
  QLabel* cmd_vel_value_label_{nullptr};
  QLabel* cmd_vel_nav2_value_label_{nullptr};
  QLabel* decision_topics_label_{nullptr};
};

}  // namespace spr_referee_mock

#endif  // SPR_REFEREE_MOCK__REFEREE_MOCK_WIDGET_HPP_
