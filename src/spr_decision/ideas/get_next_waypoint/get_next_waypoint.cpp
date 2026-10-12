#include "get_next_waypoint.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <utility>

namespace spr_decision
{
namespace
{

std::string trim(const std::string & text)
{
  const auto first = std::find_if_not(
    text.begin(), text.end(), [](unsigned char c) {return std::isspace(c);});
  if (first == text.end()) {
    return {};
  }
  const auto last = std::find_if_not(
    text.rbegin(), text.rend(), [](unsigned char c) {return std::isspace(c);}).base();
  return std::string(first, last);
}

std::vector<std::string> splitCsvRow(const std::string & line)
{
  std::vector<std::string> columns;
  std::stringstream stream(line);
  std::string column;
  while (std::getline(stream, column, ',')) {
    columns.emplace_back(std::move(column));
  }
  return columns;
}

bool parseDouble(const std::string & text, double & value)
{
  try {
    const auto cleaned = trim(text);
    std::size_t processed = 0U;
    value = std::stod(cleaned, &processed);
    return !cleaned.empty() && processed == cleaned.size();
  } catch (const std::exception &) {
    return false;
  }
}

bool isWaypointEditorHeader(const std::vector<std::string> & columns)
{
  if (columns.size() < 9U) {
    return false;
  }

  auto first_column = trim(columns[0]);
  if (first_column.size() >= 3U &&
    static_cast<unsigned char>(first_column[0]) == 0xEFU &&
    static_cast<unsigned char>(first_column[1]) == 0xBBU &&
    static_cast<unsigned char>(first_column[2]) == 0xBFU)
  {
    first_column.erase(0U, 3U);
  }

  static const std::vector<std::string> expected = {
    "id", "pose_x", "pose_y", "pose_z", "rot_x", "rot_y", "rot_z", "rot_w", "command"};
  if (first_column != expected[0]) {
    return false;
  }
  for (std::size_t i = 1U; i < expected.size(); ++i) {
    if (trim(columns[i]) != expected[i]) {
      return false;
    }
  }
  return true;
}

std::string parseCommand(const std::vector<std::string> & columns)
{
  std::vector<std::string> command_parts(columns.begin() + 8, columns.end());
  while (!command_parts.empty() && trim(command_parts.back()).empty()) {
    command_parts.pop_back();
  }

  std::ostringstream command;
  for (std::size_t i = 0U; i < command_parts.size(); ++i) {
    if (i > 0U) {
      command << ',';
    }
    command << command_parts[i];
  }

  auto result = trim(command.str());
  if (result.size() >= 2U && result.front() == '"' && result.back() == '"') {
    result = result.substr(1U, result.size() - 2U);
  }
  return result;
}

}  // namespace

GetNextWaypoint::GetNextWaypoint(
  const std::string & name, const BT::NodeConfig & config,
  std::shared_ptr<rclcpp::Node> node)
: BT::SyncActionNode(name, config), node_(std::move(node))
{
}

BT::PortsList GetNextWaypoint::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "csv_file", "waypoint_editor 导出的 CSV 文件路径"),
    BT::InputPort<bool>(
      "loop", true, "到达文件末尾后是否从第一个点重新开始"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>(
      "target_pose", "当前选中的导航目标"),
    BT::OutputPort<std::string>(
      "waypoint_id", "CSV 第一列中的点位 ID"),
    BT::OutputPort<std::string>(
      "waypoint_command", "CSV command 列；当前节点只透传，不执行"),
    BT::OutputPort<int>(
      "waypoint_index", "当前点位在文件中的零基索引"),
    BT::OutputPort<bool>(
      "is_last_waypoint", "当前点是否为文件中的最后一个点")};
}

BT::NodeStatus GetNextWaypoint::tick()
{
  const auto csv_file = getInput<std::string>("csv_file");
  if (!csv_file || trim(csv_file.value()).empty()) {
    RCLCPP_ERROR(node_->get_logger(), "GetNextWaypoint requires a non-empty [csv_file]");
    return BT::NodeStatus::FAILURE;
  }

  if (!loadCsvIfNeeded(trim(csv_file.value()))) {
    return BT::NodeStatus::FAILURE;
  }

  const bool loop = getInput<bool>("loop").value_or(true);
  if (cursor_ >= waypoints_.size()) {
    if (!loop) {
      RCLCPP_INFO(node_->get_logger(), "GetNextWaypoint reached the end of '%s'", loaded_csv_file_.c_str());
      return BT::NodeStatus::FAILURE;
    }
    cursor_ = 0U;
  }

  auto selected_pose = waypoints_[cursor_].pose;
  selected_pose.header.stamp = node_->now();
  const bool is_last = cursor_ + 1U == waypoints_.size();

  setOutput("target_pose", selected_pose);
  setOutput("waypoint_id", waypoints_[cursor_].id);
  setOutput("waypoint_command", waypoints_[cursor_].command);
  setOutput("waypoint_index", static_cast<int>(cursor_));
  setOutput("is_last_waypoint", is_last);

  ++cursor_;
  if (loop && cursor_ >= waypoints_.size()) {
    cursor_ = 0U;
  }

  return BT::NodeStatus::SUCCESS;
}

bool GetNextWaypoint::loadCsvIfNeeded(const std::string & csv_file)
{
  if (csv_file == loaded_csv_file_ && !waypoints_.empty()) {
    return true;
  }

  std::ifstream input(csv_file);
  if (!input.is_open()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to open waypoint CSV: %s", csv_file.c_str());
    return false;
  }

  std::string line;
  if (!std::getline(input, line) || !isWaypointEditorHeader(splitCsvRow(line))) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Waypoint CSV header is incompatible with waypoint_editor: %s", csv_file.c_str());
    return false;
  }

  std::vector<WaypointRecord> parsed;
  std::size_t line_number = 1U;
  while (std::getline(input, line)) {
    ++line_number;
    if (trim(line).empty()) {
      continue;
    }

    const auto columns = splitCsvRow(line);
    // waypoint_editor writes a trailing comma for an empty command. std::getline does not
    // preserve that final empty field, so a valid row may contain only the 8 pose columns.
    if (columns.size() < 8U) {
      RCLCPP_WARN(
        node_->get_logger(), "Skip malformed waypoint CSV row %zu in %s",
        line_number, csv_file.c_str());
      continue;
    }

    WaypointRecord waypoint;
    waypoint.id = trim(columns[0]);
    waypoint.pose.header.frame_id = "map";
    if (waypoint.id.empty() ||
      !parseDouble(columns[1], waypoint.pose.pose.position.x) ||
      !parseDouble(columns[2], waypoint.pose.pose.position.y) ||
      !parseDouble(columns[3], waypoint.pose.pose.position.z) ||
      !parseDouble(columns[4], waypoint.pose.pose.orientation.x) ||
      !parseDouble(columns[5], waypoint.pose.pose.orientation.y) ||
      !parseDouble(columns[6], waypoint.pose.pose.orientation.z) ||
      !parseDouble(columns[7], waypoint.pose.pose.orientation.w))
    {
      RCLCPP_WARN(
        node_->get_logger(), "Skip invalid waypoint CSV row %zu in %s",
        line_number, csv_file.c_str());
      continue;
    }

    waypoint.command = parseCommand(columns);
    parsed.emplace_back(std::move(waypoint));
  }

  if (parsed.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Waypoint CSV contains no valid points: %s", csv_file.c_str());
    return false;
  }

  waypoints_ = std::move(parsed);
  loaded_csv_file_ = csv_file;
  cursor_ = 0U;
  RCLCPP_INFO(
    node_->get_logger(), "Loaded %zu waypoint(s) from %s",
    waypoints_.size(), loaded_csv_file_.c_str());
  return true;
}

}  // namespace spr_decision
