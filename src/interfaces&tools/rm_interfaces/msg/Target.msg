std_msgs/Header header
bool tracking
string id  # 当前追踪机器人编号
int32 armors_num  # 该目标装甲板数量
geometry_msgs/Point position  # 目标旋转中心位置(m)
geometry_msgs/Vector3 velocity  # 旋转中心速度(m/s)
float64 yaw  # 装甲板朝向(rad)
float64 v_yaw  # 目标yaw角速度(rad/s)
float64 radius_1  # 旋转中心到装甲板/一对装甲板水平半径(m)
float64 radius_2  # 四装甲另一对水平半径(m)
float64 d_za  # 四装甲高度差(m)
float64 d_zc  # 装甲板相对目标中心高度偏移(m)
float64 yaw_diff
float64 position_diff
