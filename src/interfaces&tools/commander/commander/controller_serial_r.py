
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import String, Int16, Int32, Float32MultiArray
from sensor_msgs.msg import PointCloud2, JointState
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation as R
import serial
import serial.tools.list_ports
import time
import math
import tf_transformations
from tf2_ros import TransformException, Buffer, TransformListener, TransformBroadcaster
import struct
from rm_interfaces.msg import GameState
from rm_interfaces.msg import Target
from sensor_msgs.msg import JointState
from math import cos, sin
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy



class ControllerSerial(Node):
    def __init__(self):
        super().__init__(
            node_name='cmd_vel_subscriber',
            namespace='red_standard_robot1'
        )

        # 创建一个订阅者来接收 cmd_vel 消息    cmd_vel  cmd_vel_decomposed   cmd_vel_x_multiplier
        
        # self.subscription = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)
        # self.events_publisher = self.create_publisher(String, 'events', 10)

        self.cloud_point = None
      
        self.i = 0
        self.start_pose = None
        self.position = None

        self.open_port()
        self.race_status = 0
        self.blood = 400
        self.bullet_allowance = 200

        self.time = 0
        self.icp_start = False
        self.timer_serial_read = self.create_timer(0.05, self.msg_read)

        
        # 创建 TF 缓存和监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 创建 TF 广播器
        self.tf_broadcaster = TransformBroadcaster(self)

        self.status = 0
        self.wz = 0.0
        
        self.current_cloud = o3d.geometry.PointCloud()

        self.trans_init = np.eye(4)

        self.init_done = False
        self.overlap_threshold = 80

        self.yaw = 0.0
        self.aim_yaw = 0.0      
        self.reach_polygon = 0
        self.send_goal = 0

        self.gimbal_yaw_rad = 0


        # 所有话题名不带 leading slash，都会自动加上 namespace 前缀
        # 订阅 /red_standard_robot1/cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10
        )
        # 发布 /red_standard_robot1/events
        self.rfid_publisher = self.create_publisher(
            String,
            'rfid_status',
            10
        )

        # 发布 /red_standard_robot1/serial/gimbal_joint_state
        self.joint_state_publisher = self.create_publisher(
            JointState,
            'serial/gimbal_joint_state',
            10
        )


        # 发布 /red_standard_robot1/referee/robot_status
        self.pub_game_state = self.create_publisher(
            GameState,
            'game_state',
            10
        )

        #   # 定时器频率约为30Hz
        # self.timer = self.create_timer(0.033, self.publish_gimbal_tf)
        self.gimbal_relative_angle_aim = 0 
        # 订阅 status_wz 话题
        self.create_subscription(
            Float32MultiArray,
            'status_wz',  # 订阅 status_wz 话题
            self.status_wz_callback,
            10
        )
        





    def reach_polygon_callback(self, msg):
        # 在这里处理接收到的消息
        self.get_logger().info(f'Received message on topic /reach_polygon: {msg.data}')
        self.reach_polygon = msg.data

    def status_callback(self, msg):
        self.status = msg.data

    def send_goal_callback(self, msg):
        self.send_goal = msg.data

    def msg_read(self):
        try:
            # 读取串口数据
            self.ser.flushInput()
            data = self.ser.read(26)
            if data[0] == 0xff and data[25] == 0xfe:
  
                yaw_data = struct.unpack('>i', data[1:5])[0]
                self.yaw = float(yaw_data) / 10000000.0

                self.gimbal_relative_angle = -self.yaw   #取反 电机角度与之前代码角度相反
        # 发布关节状态数据
               # print('gimbal: ', -self.yaw)
                # 发布 JointState 消息
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                # 此处 joint 名称需要与你机器人模型中定义的关节名称一致
                joint_state_msg.name = ['gimbal_yaw_joint']
                joint_state_msg.position = [self.gimbal_relative_angle]
                self.joint_state_publisher.publish(joint_state_msg)

 


# data 0 ff 1234yaw 5 比赛状态 6 7 剩余时间 8 9自己剩余血量 10 11剩余发弹量 12 rfid
#13 14 蓝方基地血量 15 16 蓝方前哨站血量 1718 红方基地   19 20 红方前哨站     21 22 23 24  aim_gimbal_yaw  25fe

          # 发布 GameStatus
                gs = GameState()
               
            #    my==red

                game_progress = data[5]  #0 未开始 1 准备 2 自检 3 倒计时 4 对战 5 结算
                stage_remain_time = int.from_bytes(data[6:8], byteorder='big', signed=True)
                current_hp = struct.unpack('>H', data[8:10])[0]
                projectile_allowance_17mm = int.from_bytes(data[10:12], byteorder='big', signed=True)
                rfid = data[12]  
                my_outpost_hp = int.from_bytes(data[19:21], byteorder='big', signed=True)
                enemy_outpost_hp = int.from_bytes(data[15:17], byteorder='big', signed=True)
                my_base_hp = int.from_bytes(data[17:19], byteorder='big', signed=True)
                enemy_base_hp = int.from_bytes(data[13:15], byteorder='big', signed=True)
                
                gs.my_outpost_hp       = my_outpost_hp
                gs.enemy_outpost_hp    = enemy_outpost_hp
                gs.game_progress       = game_progress
                gs.stage_remain_time   = stage_remain_time
                gs.current_hp = current_hp
                gs.projectile_allowance_17mm = projectile_allowance_17mm
                gs.my_base_hp       = my_base_hp
                gs.enemy_base_hp       = enemy_base_hp
                gs.rfid       = rfid


      ##  groot2 bt tree

                # gs.my_outpost_hp       = my_outpost_hp
                # gs.enemy_outpost_hp    = enemy_outpost_hp
                # gs.game_progress       = 4
                # gs.stage_remain_time   = 400
                # gs.current_hp = 300
                # gs.projectile_allowance_17mm = 300
                # gs.my_base_hp       = my_base_hp
                # gs.enemy_base_hp       = enemy_base_hp
                # gs.rfid       = rfid


  # 新增：打印解析后的内容
                print('-' * 80)
          #      print("已发送数据:")
                print(f"yaw={self.yaw}, yaw={self.aim_yaw},game_progress={game_progress}, stage_remain_time={stage_remain_time},my_outpost_hp={my_outpost_hp}")
              #  
              # print(f"解析后: yaw={self.yaw}, game_progress={game_progress}, stage_remain_time={stage_remain_time}, current_hp={current_hp}, projectile_allowance_17mm={projectile_allowance_17mm}, my_outpost_hp={my_outpost_hp}, enemy_outpost_hp={enemy_outpost_hp}, my_base_hp={my_base_hp}, enemy_base_hp={enemy_base_hp}")
                #print('-' * 80)              

                self.pub_game_state.publish(gs)
             #   self.get_logger().debug(
                   #f"yaw={self.yaw}, progress={game_progress}, remain_time={stage_remain_time}")
                # # print(f'game_progress: {data[5]}')



    #  #发布运动模式  事件
                # msg = String()
    #           msg.data = str(data[1])
    #           self.events_publisher.publish(msg)
                # print(f'send goal: {data[5]}')
                

     #发布rfid status 0 false 1 true
                msg = String()
                msg.data = str(data[12])
                self.rfid_publisher.publish(msg)
                print(f'rfid status: {data[12]}')

                
        except Exception as exc:
            print("receive error", exc)


    def status_wz_callback(self, msg):

        # 获取状态的线速度 x 和角速度 z
        self.status = msg.data[0]  # 第一个元素是 flag
        angular_z = 1.0 * msg.data[1]  # 第二个元素是 wz

        if angular_z == 0.0:
            angular_z = 40
        elif angular_z < 0.1 and angular_z > 0.0:
            angular_z = 41
        elif angular_z > -0.1 and angular_z < 0.0:
            angular_z = 39
        else:
            angular_z = int((angular_z + 4) * 10)
        
        self.wz=angular_z
        




        # # 更新发送数据的部分
        # data = [0xFF, 0, 0, 0, 0, 2, 0, 0xFE]

        # # 将线速度和角速度映射到数据
        # data[3] = int(self.wz)  # 将角速度 z 转换为整数
        # data[4] = int(self.status)     # 将线速度 x 转换为整数

        # # 发送数据给电控
        # self.sendTwist(data)




    # def sendTwist(self, data):
    #     try:
    #         self.ser.write(data)
    #         print('-' * 80)
    #         print("已发送数据:")
    #         print(data)
    #         print('-' * 80)
    #     except Exception as exc:
    #         print("发送异常", exc)


    def twist_callback(self, msg):
        linear_x = msg.linear.x
        linear_y = -msg.linear.y
        # angular_z = msg.angular.z

        
     #   print(linear_x, linear_y)
        if linear_x == 0.0:
            linear_x = 40
        elif linear_x < 0.1 and linear_x > 0.0:
            linear_x = 41
        elif linear_x > -0.1 and linear_x < 0.0:
            linear_x = 39
        else:
            linear_x = int((linear_x + 4) * 10)
        
        if linear_y == 0.0:
            linear_y = 40
        elif linear_y < 0.1 and linear_y > 0.0:
            linear_y = 41
        elif linear_y > -0.1 and linear_y < 0.0:
            linear_y = 39
        else:
            linear_y = int((linear_y + 4) * 10)


        # if angular_z == 0.0:
        #     angular_z = 40
        # elif angular_z < 0.1 and angular_z > 0.0:
        #     angular_z = 41
        # elif angular_z > -0.1 and angular_z < 0.0:
        #     angular_z = 39
        # else:
        #     angular_z = int((angular_z + 4) * 10)
        
        





        self.sendTwist(linear_x, linear_y)
        # self.sendTwist(linear_x, linear_y,angular_z)
        # 将数据发送给电控
        # self.sendTwist([0xFF, linear_x, linear_y, 0, 0, 2, 0, 0xFE])
        print(linear_x, linear_y)

    def open_port(self):
        port = '/dev/ttyUSB0'  # 串口号
        baudrate = 115200  # 波特率
        try:
            self.ser = serial.Serial(port, baudrate, timeout=2)
            if self.ser.isOpen() == True:
                print("串口打开成功")
                self.port_clean()
            else:
                self.ser.open()
        except Exception as exc:
            print("串口打开异常", exc)

    def port_clean(self):
        self.ser.flushInput()#清理缓冲区数据
        
    def close_port(self):
        try:
            self.ser.close()
            if self.ser.isOpen():
                print("串口未关闭")
            else:
                print("串口已关闭")
        except Exception as exc:
            print("串口关闭异常", exc)

    def sendTwist(self, linear_x, linear_y):
        linear_x = int(linear_x)
        linear_y = int(linear_y)
        # angular_z = int(angular_z)
      
        data =[0XFF,0,0,0,0,2,0,0XFE]

        
        data[1]=linear_x      #x
        data[2]=linear_y      #y



      # 将线速度和角速度映射到数据
        # data[3] = angular_z  # 将角速度 z 转换为整数
        data[3]=int(self.wz)
        data[4] = int(self.status)     # 将线速度 x 转换为整数

  
        # data[3]=0
        # if self.status == 2 and data[1] != 40 and data[2] != 40:
        #     data[3] = 1
        # else:
        #     data[3]=self.status
        # data[4] = self.send_goal
        # data[5]=2

     


        try:
            send_datas = data#需要发送的数据，此处直接发送列表数据，不需要设置编码格式，如果发送字符串需要制定编码格式
            # ser.write(str(send_datas+'\r\n').encode("utf-8"))
            self.ser.write(send_datas)
            print('-' * 80)
            print("已发送数据:")
            print(send_datas)
            print('-' * 80)

        except Exception as exc:
            print("发送异常", exc)




def main(args=None):
    rclpy.init(args=args)
    node = ControllerSerial()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



