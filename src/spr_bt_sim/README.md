# SPR 行为树决策仿真

这个包现在只验证 `spr_decision` 的决策结果，不再启动 Gazebo、机器人模型、雷达、Point-LIO、建图、重定位或 Nav2 导航栈。

启动后包含：

- `spr_referee_mock`：GUI 修改并发布血量、弹量、比赛时间等 `game_state`。
- `spr_decision`：运行指定行为树。
- 从实车 `spr_nav_bringup/map/reality/rmuc2026.*` 复制的地图快照和 RViz。
- 假 `NavigateToPose` action server：接收行为树导航目标，但不控制机器人。
- RViz 决策标记：蓝色圆柱是手动机器人位姿，红色箭头是行为树目标，橙线是目标历史，绿色文字显示决策状态。

包名为 `spr_bt_sim`；运行链路不依赖 Gazebo。

## 构建与启动

```bash
cd /home/sentry/workspace27/new_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to spr_referee_mock spr_decision spr_bt_sim
source install/setup.bash
ros2 launch spr_bt_sim decision_sim.launch.py
```

原命令仍可用：

```bash
ros2 launch spr_bt_sim simulation.launch.py
```

## 默认测试方法：RViz 手动模拟到达

默认 `result_mode: manual_pose`。

1. 在 `spr_referee_mock` 中设置比赛阶段、血量、弹量、剩余时间等，然后保持“自动周期发布”开启，或点击“立即发布一次”。
2. 行为树产生目标后，RViz 地图上会出现红色目标箭头。
3. 点击 RViz 工具栏的 **2D Pose Estimate**，在目标附近点下并拖动朝向。
4. 蓝色机器人标记移动到该位置；进入 `xy_tolerance` 后，假 action server 向行为树返回成功。
5. 行为树继续执行，新的姿态、小陀螺状态或下一导航目标会显示在 RViz 和 referee mock 窗口中。

这里必须用 **2D Pose Estimate**。`2D Goal Pose` 表示发布一个新的导航目标，不能代表机器人已经到达行为树目标。

当前行为树的多数目标四元数为全零，所以默认 `check_goal_yaw: false`，只按 XY 距离判断。如果行为树目标补齐了有效朝向，可将它改为 `true`。

## 统一参数

所有仿真开关和参数都在：

```text
src/spr_bt_sim/config/simulation.yaml
```

安装后修改源码 YAML 时，使用 `--symlink-install` 通常可直接生效；也可复制一份配置并显式指定：

```bash
ros2 launch spr_bt_sim decision_sim.launch.py \
  config_file:=/absolute/path/to/my_decision_sim.yaml
```

主要参数：

| 参数 | 作用 |
|---|---|
| `namespace` | 全部话题、action 和节点使用的机器人命名空间 |
| `use_rviz` / `rviz_config` | 是否启动 RViz，以及 RViz 配置文件路径 |
| `map_yaml` | RViz 二维地图，默认使用本包内的实车 `rmuc2026` 快照 |
| `referee_mock.enabled` | 是否启动裁判数据 GUI |
| `referee_mock.auto_publish` | 启动后是否按 `publish_rate_hz` 自动发布 GUI 中的裁判数据 |
| `referee_mock.current_hp` 等 | GUI 和自动发布使用的裁判数据初值 |
| `decision.enabled` | 是否启动 `spr_decision` |
| `decision.tree_xml_file` | 要验证的行为树 XML，默认使用本包内的当前实车行为树快照 |
| `decision.start_delay_sec` | referee mock 和假 action server 启动后，延迟启动行为树的时间 |
| `decision.parameters.enable_groot` | 是否启动 Groot2 发布器；离线仿真默认关闭，避免占用端口 |
| `decision.parameters.subscribe_target` | 是否订阅视觉跟踪目标；纯裁判数据测试默认关闭 |
| `decision.parameters.*` | 其余 `tree_exec_node` 参数，例如 tick 周期、话题名和攻击距离 |
| `result_mock.result_mode` | 导航结果模拟方式，见下表 |
| `result_mock.result_delay_sec` | `succeed`/`abort` 模式返回结果前的延迟 |
| `result_mock.robot_pose` | RViz 首次手动设位姿前的 `[x, y, yaw(rad)]` |
| `result_mock.xy_tolerance` | `manual_pose` 模式的到达距离容差 |
| `result_mock.check_goal_yaw` | 是否同时检查手动位姿朝向 |
| `result_mock.yaw_tolerance_deg` | 朝向到达容差 |
| `result_mock.map_frame` / `robot_frame` | 可视化和模拟 TF 使用的坐标系 |
| `result_mock.publish_robot_tf` | 是否发布模拟的 `map -> chassis` TF |
| `result_mock.*_topic` / `action_name` | 手动位姿、目标、标记的话题名及导航 action 名 |
| `result_mock.max_goal_history` | RViz 保存并连线显示的最近目标数量 |

`result_mode` 可选值：

| 值 | 行为 |
|---|---|
| `manual_pose` | 默认；用 RViz `2D Pose Estimate` 移到目标附近才返回成功 |
| `hold` | 一直保持 action 执行中，只查看当前决策目标 |
| `succeed` | `result_delay_sec` 秒后自动成功，适合连续测试完整序列 |
| `abort` | `result_delay_sec` 秒后自动失败，测试行为树失败分支 |

修改行为树示例：

```yaml
decision:
  tree_xml_file: behavior_tree/RMUC_adaptation_2026_A.xml
```

自动跑后续序列示例：

```yaml
result_mock:
  result_mode: succeed
  result_delay_sec: 1.0
```

## 数据接口

默认命名空间是 `/red_standard_robot1`：

| 接口 | 类型 | 方向/用途 |
|---|---|---|
| `game_state` | `rm_interfaces/msg/GameState` | referee mock → 行为树 |
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 行为树 → 假 action server |
| `initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | RViz 手动设置模拟机器人位姿 |
| `decision_goal` | `geometry_msgs/msg/PoseStamped` | 最新行为树目标，Transient Local |
| `decision_markers` | `visualization_msgs/msg/MarkerArray` | RViz 决策可视化 |
| `posture` | `std_msgs/msg/UInt8` | 行为树姿态：1 进攻、2 防御、3 移动 |
| `spin_or_not` | `std_msgs/msg/UInt8` | 行为树小陀螺：0 关、1 开 |
| `tf` | TF | 假 `map -> chassis`，由手动位姿更新 |

可以不用 RViz，直接发布模拟机器人位姿：

```bash
ros2 topic pub --once /red_standard_robot1/initialpose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 7.26, y: 3.77}, orientation: {w: 1.0}}}}"
```

查看行为树最近给出的目标：

```bash
ros2 topic echo /red_standard_robot1/decision_goal --qos-durability transient_local
```

## 与实车文件的关系

- `map/rmuc2026.yaml` 和 PGM 是当前实车地图的快照；修改它不会影响实车包。
- 实车地图更新后，如需同步，重新复制 `spr_nav_bringup/map/reality/rmuc2026.*` 即可。
- `behavior_tree/` 是当前 `spr_decision/behavior_tree/` 的测试快照；修改它不会影响实车包。
- 本包不会修改实车地图、行为树或导航配置。
- 如需试验性修改，复制文件到自定义路径，再在统一 YAML 中改路径即可。
