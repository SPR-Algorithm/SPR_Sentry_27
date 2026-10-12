# spr_decision

## 设计想法：GetNextWaypoint

`ideas/get_next_waypoint/` 保存了一个 `GetNextWaypoint` 参考实现，用于按文件顺序读取
`waypoint_editor` 导出的 CSV 点位，并逐次输出 `geometry_msgs/msg/PoseStamped`。

它当前只是一个设计想法：**没有加入 CMake、不会被编译、没有注册为行为树节点，也没有加入任何行为树、launch 或参数文件**。现有决策流程和实车行为不受影响。

支持的 CSV 表头与 `waypoint_editor` 保持一致：

```csv
id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,command,
```

一个 CSV 文件当前代表一条有序路线。节点按文件中的行顺序取点；`command` 列可以为空，非空内容会通过输出端口透传，但不会被执行。

### 端口

| 方向 | 端口 | 类型 | 说明 |
|---|---|---|---|
| 输入 | `csv_file` | `std::string` | CSV 文件路径，必填 |
| 输入 | `loop` | `bool` | 末尾是否回到第一个点，默认 `true` |
| 输出 | `target_pose` | `geometry_msgs/msg/PoseStamped` | 当前点位，坐标系为 `map` |
| 输出 | `waypoint_id` | `std::string` | CSV 中的点位 ID |
| 输出 | `waypoint_command` | `std::string` | CSV 中的 command 内容，仅透传 |
| 输出 | `waypoint_index` | `int` | 当前点位的零基索引 |
| 输出 | `is_last_waypoint` | `bool` | 当前点是否为最后一点 |

未来若正式实现并接入，预期用法如下：

```xml
<Sequence>
  <GetNextWaypoint csv_file="{waypoint_csv}"
                   loop="true"
                   target_pose="{target_pose}"
                   waypoint_id="{waypoint_id}"/>
  <Nav2Pose goal="{target_pose}"/>
</Sequence>
```

### 未来价值

该节点未来可以把巡逻点、补给点等坐标从行为树 XML 中移到点位文件，使 RViz 中的点位编辑结果可以复用于仿真和实车，并减少 A/B 方地图适配时对行为树逻辑的修改。

正式接入前仍需确定导航失败、行为树抢占和路线重置时的游标策略。当前节点在每次成功取点后立即推进游标；CSV 内容在进程运行期间原地修改时也不会自动重新加载。
