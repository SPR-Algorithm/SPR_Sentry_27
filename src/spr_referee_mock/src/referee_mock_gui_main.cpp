#include <QApplication>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "spr_referee_mock/referee_mock_node.hpp"
#include "spr_referee_mock/referee_mock_widget.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  auto node = std::make_shared<spr_referee_mock::RefereeMockNode>();
  spr_referee_mock::RefereeMockWidget window(node);
  window.show();

  const int ret = app.exec();
  rclcpp::shutdown();
  return ret;
}
