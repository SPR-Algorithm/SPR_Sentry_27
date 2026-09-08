#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_ros import Buffer, TransformListener, TransformException
import tf_transformations
import math
import json
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class StatusWzPublisher(Node):
    def __init__(self):
        super().__init__('status_wz_publisher', namespace='red_standard_robot1')

        # 加载多边形 JSON 参数
        default_polygons = json.dumps([
            [[5.63, -1.25], [6.13, -2.1], [7.88, -1.7], [7.33, -0.7]]
        ])
        self.declare_parameter('polygons_json', default_polygons)
        polygons_str = self.get_parameter('polygons_json').get_parameter_value().string_value
        print("default_polygons",polygons_str)
        self.get_logger().info(f'polygons_json 参数内容: {polygons_str}')
        try:
            self.polygons = json.loads(polygons_str)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'解析 polygons_json 失败: {e}')
            self.polygons = []

        # TF 监听
        qos_tf = QoSProfile(depth=10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, qos=qos_tf)

        # 参数：角速度限幅
        self.declare_parameter('wz_limit', 3.14)
        self.wz_limit = self.get_parameter('wz_limit').get_parameter_value().double_value

        # 状态记录
        self.robot_position = None
        self.inside_polygon_id = 0
        self.last_plan_time = self.get_clock().now()
        self.plan_timeout_sec = 0.5

        # 订阅 local_plan，发布 status_wz   local_plan
        self.sub_plan = self.create_subscription(Path, 'plan', self.plan_callback, 10)
        self.pub_status = self.create_publisher(Float32MultiArray, 'status_wz', 0)

        # 定时器：检查是否超时清零
        self.create_timer(0.2, self.check_plan_timeout)

        self.get_logger().info(f'节点启动，已加载 {len(self.polygons)} 个多边形')

    def get_robot_position(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'chassis', rclpy.time.Time())
            x, y = trans.transform.translation.x, trans.transform.translation.y
            self.robot_position = [x, y]
            self.inside_polygon_id = self.is_inside_any_polygon(self.robot_position)
        except TransformException as e:
            self.get_logger().warn(f'TF 获取失败: {e}')
            self.robot_position = None
            self.inside_polygon_id = 0

    def plan_callback(self, msg: Path):
        # 更新收到路径时间
        self.last_plan_time = self.get_clock().now()
        self.get_robot_position()

        # 获取底盘 yaw
        try:
            trans = self.tf_buffer.lookup_transform('odom', 'chassis', rclpy.time.Time())
            q = trans.transform.rotation
            _, _, chassis_yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        except TransformException:
            chassis_yaw = 0.0

        # 提取路径最后两点，计算原始路径方向
        poses = msg.poses
        if len(poses) < 2 or self.robot_position is None:
            return
        x1, y1 = poses[-2].pose.position.x, poses[-2].pose.position.y
        x2, y2 = poses[-1].pose.position.x, poses[-1].pose.position.y
        raw_yaw = math.atan2(y2 - y1, x2 - x1)

        # 对齐到最近边方向，并选最小旋转候选
        edge_raw = round(raw_yaw / (math.pi/2)) * (math.pi/2)
        cand1 = edge_raw
        cand2 = (edge_raw + math.pi) % (2*math.pi)
        cand2 = (cand2 + math.pi) % (2*math.pi) - math.pi
        err1 = abs(self._angle_diff(cand1, chassis_yaw))
        err2 = abs(self._angle_diff(cand2, chassis_yaw))
        path_yaw = cand1 if err1 < err2 else cand2

        # # 对齐到前后两条边（原始方向和反方向），并选最小旋转候选
        # cand1 = raw_yaw
        # cand2 = (raw_yaw + math.pi) % (2 * math.pi)
        # cand2 = (cand2 + math.pi) % (2 * math.pi) - math.pi  # 归一化到[-pi, pi]
        # err1 = abs(self._angle_diff(cand1, chassis_yaw))
        # err2 = abs(self._angle_diff(cand2, chassis_yaw))
        # path_yaw = cand1 if err1 < err2 else cand2


        # # 计算误差并限幅（限制旋转到 ±π/2）
        # yaw_err = self._angle_diff(path_yaw, chassis_yaw)
        # max_rot = math.pi/2
        # yaw_err = max(-max_rot, min(max_rot, yaw_err))
        
        
        # 计算误差并限幅（限制旋转到 ±π/4）
        yaw_err = self._angle_diff(path_yaw, chassis_yaw)
        max_rot = math.pi/4
        yaw_err = max(-max_rot, min(max_rot, yaw_err))
        
       

        # 接近目标清零
        dist = math.hypot(x2 - x1, y2 - y1)
        if dist < 0.1:
            wz = 0.0
            self.get_logger().info('接近目标点，清零 wz')



        # # 修改wz发布逻辑：如果夹角大于5度则发布1.0，否则为0
        # angle_diff_deg = abs(math.degrees(yaw_err))
        # wz = 1.5 if angle_diff_deg > 5 else 0.0


        # 如果夹角为负值则wz=-1.5，正值则wz=1.5，0则为0
        angle_diff_deg = abs(math.degrees(yaw_err))
        if yaw_err < 0 and angle_diff_deg > 5:
            if angle_diff_deg > 15:
                wz = 2.0
            else:
                wz = 0.5
        elif yaw_err > 0 and angle_diff_deg > 5:
            if angle_diff_deg > 15:
                wz = -2.0
            else:
                wz = -0.5
        else:
            wz = 0.0




        # 发布 Float32MultiArray: [flag, wz]
        out = Float32MultiArray()
        out.data = [float(bool(self.inside_polygon_id)), wz]
        self.pub_status.publish(out)
        # self.get_logger().info(
        #     f'raw_yaw={raw_yaw:.2f} edge_raw={edge_raw:.2f} chassis_yaw={chassis_yaw:.2f} angle_diff={angle_diff_deg:.2f}° wz={wz}'
        # )
        self.get_logger().info(
            f'raw_yaw={raw_yaw:.2f} chassis_yaw={chassis_yaw:.2f} angle_diff={angle_diff_deg:.2f}° wz={wz}'
        )

    def check_plan_timeout(self):
        # 超时未收到路径，则清零 wz
        dt = (self.get_clock().now() - self.last_plan_time).nanoseconds * 1e-9
        if dt > self.plan_timeout_sec:
            out = Float32MultiArray()
            out.data = [float(bool(self.inside_polygon_id)), 0.0]
            self.pub_status.publish(out)
            #self.get_logger().info(f'plan超时 {dt:.2f}s 清零 wz')

    def is_inside_any_polygon(self, point):
        for idx, verts in enumerate(self.polygons, start=1):
            if self._point_in_quad(verts[0], verts[1], verts[2], verts[3], point):
                return idx
        return 0

    @staticmethod
    def _point_in_quad(p1, p2, p3, p4, p):
        def cross(a, b, c):
            return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        return (cross(p1, p2, p) * cross(p3, p4, p) >= 0 and
                cross(p2, p3, p) * cross(p4, p1, p) >= 0)

    @staticmethod
    def _angle_diff(target, current):
        a = target - current
        return (a + math.pi) % (2 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    node = StatusWzPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()









