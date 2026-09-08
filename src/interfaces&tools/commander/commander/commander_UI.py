import sys
import threading
import yaml
import rclpy
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from std_msgs.msg import Float32
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import *
import cv2
from PyQt5.QtCore import *
import sys
import yaml
from PyQt5.QtGui import *
from UI_classes import UI_class
from UI_classes import Config_Dialog
from UI_classes import Map_Edit 

class ROSQtNode(QMainWindow, QWidget, UI_class.Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.init_ui()

        # Create a thread for running the ROS node
        self.ros_thread = threading.Thread(target=self.run_ros_node)
        self.ros_thread.start()

        self.map_path_config = ''
        self.map_path_img = ''

        '''
        image: 指定地图的图像文件,mode: 地图的编码模式,resolution: 地图的分辨率，指每个像素表示的实际距离
        origin: 地图坐标系的原点，以及地图的旋转,negate: 用于将地图的值取反,occupied_thresh: 占据阈值,free_thresh: 自由阈值
        y
        ⬆
          ➡x
        '''
        self.flag = 0
        self.resize_flag = 0
        self.map_yaml = {}
        self.configs = {}
        self.img_map = None
        self.map_width = 0
        self.map_height = 0
        self.map_origin_x = 0
        self.map_origin_y = 0
        # 画面标签位置
        self.tx = 390
        self.ty = 60
        # 画面标签大小
        self.maxWidth = 630
        self.maxHeight = 630
        self.real_x = 0.0
        self.real_y = 0.0
        self.ratio = 1

        self.config_dialog_is_open = False

        
    def init_ui(self):
        self.map_label.setText(" ")
        self.actionOpen_map.triggered.connect(self.load_map)
        self.actionSave_config.triggered.connect(self.save_yaml)

        # 设置列数
        self.config_show_treeWidget.setColumnCount(2)
        # 设置头的标题
        self.config_show_treeWidget.setHeaderLabels(['Strategy Type','Traget Pose'])
        self.config_show_treeWidget.clicked.connect(self.onTreeClicked)
        self.Duration_root = QTreeWidgetItem(self.config_show_treeWidget)
        self.Duration_root.setText(0, 'Duration')
        self.Specific_Duration_root = QTreeWidgetItem(self.config_show_treeWidget)
        self.Specific_Duration_root.setText(0, 'Specific Duration')
        self.Events_root = QTreeWidgetItem(self.config_show_treeWidget)
        self.Events_root.setText(0, 'Events')

        self.delete_one_item_pushButton.clicked.connect(self.deleteOneConfigItem)
        self.map_edit_pushButton.clicked.connect(self.map_edit)


    def run_ros_node(self):
        rclpy.init()

        self.node = rclpy.create_node('ros_qt_node')
        self.subscriber = self.node.create_subscription(
            Float32,
            'duration',
            self.callback,
            QoSProfile(depth=10)
        )

        while rclpy.ok():
            rclpy.spin_once(self.node)

        self.node.destroy_node()
        rclpy.shutdown()

    def callback(self, msg):
        # Update the Qt label from the ROS callback
        pass

    def load_map(self):
        # 弹出文件选择对话框
        self.map_path_config, _ = QFileDialog.getOpenFileName(self, '选择图片', '', 'Config (*.yaml)')
        if self.map_path_config:
            # 显示选择的图片
            self.load_config(self.map_path_config)
            self.map_path_img = self.map_path_config.split(".")[0] + '.pgm'
            self.show_cv_img(self.map_path_img)

    def load_config(self, path):
        with open(path, 'r', encoding='utf-8') as f:
            data = f.read()
            self.map_yaml = yaml.load(data, Loader=yaml.FullLoader)
            # print(self.map_yaml)

    def draw_arrow(self):
        self.map_origin_x = -self.map_yaml['origin'][0]
        self.map_origin_y = -self.map_yaml['origin'][1]
        origin_point = (int(self.map_origin_x / self.map_yaml['resolution']), self.map_height - int(self.map_origin_y / self.map_yaml['resolution']))
        end_point_x = (int(self.map_origin_x / self.map_yaml['resolution']) + 10, self.map_height - int(self.map_origin_y / self.map_yaml['resolution']))
        end_point_y = (int(self.map_origin_x / self.map_yaml['resolution']), self.map_height - int(self.map_origin_y / self.map_yaml['resolution']) - 10)
        # cv2.circle(self.img_map, origin_point, 1, (0, 0, 255), 2)
        self.img_map = cv2.arrowedLine(self.img_map, origin_point, end_point_x, (255, 0, 0), 1)
        self.img_map = cv2.arrowedLine(self.img_map, origin_point, end_point_y, (0, 255, 0), 1)

    def show_cv_img(self, path):
        self.img_map = cv2.imread(path)
        self.map_width = self.img_map.shape[1]
        self.map_height = self.img_map.shape[0]
        self.draw_arrow()
        self.resize_flag = 0
        self.flag = 1
        self.Resize()
        QtImg = QtGui.QImage(self.img_map.data,
                             self.img_map.shape[1],
                             self.img_map.shape[0],
                             self.img_map.shape[1] * 3,
                             QtGui.QImage.Format_RGB888)
        jpg_out = QtGui.QPixmap(QtImg).scaled(
            self.map_label.width(), self.map_label.height())
        
        self.map_label.setPixmap(jpg_out)
    
    def map_edit(self):
        if len(self.map_path_img) != 0:
            # img = cv2.imread(self.map_path_img)  # 读取图像

            # def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
            #     if event == cv2.EVENT_LBUTTONDOWN:
            #         xy = "%d,%d" % (x, y)
            #         b, g, r = img[y, x]
            #         print("x,y:",x, y,"BGR:", b, g, r)
            #         img[y, x] = (0, 0, 0)
            #         # img[y, x] = (205, 205, 205)
            #         # img[y, x] = (254, 254, 254)
            #         cv2.imshow("image", img)
                    
            # cv2.namedWindow("image")
            # cv2.setMouseCallback("image", on_EVENT_LBUTTONDOWN)
            # while(1):
            #     cv2.imshow("image", img)
            #     key = cv2.waitKey(5) & 0xFF
            #     if key == ord(' '):
            #         break
            # cv2.destroyAllWindows()
            self.map_editor = Map_Edit.MapEdit(self.map_path_img)
            self.map_editor.show()
            def Exit():
                self.map_editor.close()
            self.map_editor.exit_pushButton.clicked.connect(Exit)
            self.map_editor.exec_()

    def mouseDoubleClickEvent(self, event):
        if event.button() == Qt.LeftButton:
            # QMessageBox.information(self, "Mouse Double Click", "Left button double clicked!")
            self.config_dialog = Config_Dialog.ConfigDialog()
            self.config_dialog.show()
            self.config_dialog_is_open = True
            self.config_dialog.get_real_pose(self.real_x, self.real_y)
            # config_dialog.exec_()
            if self.config_dialog.exec_() == QDialog.Accepted:
                print("Dialog is accepted")
                result = self.config_dialog.getConfig()
                print(result)
                if result != None:
                    self.addOneConfigItem(result)
                self.config_dialog_is_open = False
            else:
                print("Dialog is rejected")
                self.config_dialog_is_open = False
            
        elif event.button() == Qt.RightButton:
            QMessageBox.information(self, "Mouse Double Click", "Right button double clicked!")

        # 调用父类的事件处理函数
        super().mouseDoubleClickEvent(event)

    def mousePressEvent(self, cursor_event):
        x= cursor_event.pos().x()
        y = cursor_event.pos().y()
        x -= self.tx
        y -= (self.ty + 22)
        x = int(x / self.ratio)
        y = int(y / self.ratio)
        
        if x >= 0 and x <= self.map_width and y >= 0 and y <= self.map_height:
            self.real_x = x * self.map_yaml['resolution'] + self.map_yaml['origin'][0]
            y = self.map_height - y
            self.real_y = y * self.map_yaml['resolution'] + self.map_yaml['origin'][1]
            self.real_x = round(self.real_x, 2)
            self.real_y = round(self.real_y, 2)
            self.statusBar.showMessage(f'Current pose:x={self.real_x},y={self.real_y}',5000)
            if self.config_dialog_is_open:
                print('send success')
                self.config_dialog.get_real_pose(self.real_x, self.real_y)
                
            self.update()

    def save_yaml(self):
        if len(self.configs) != 0:
            with open('/home/spr/Sentry_ws/src/SPR-RM-Sentry/src/commander/commander/config1.yaml', 'w') as file:
                yaml.dump(self.configs, file, default_flow_style=False)

    def handle_dialog_exit(self, exit_status):
        print(f'Dialog exit status: {exit_status}')

    def Resize(self):
        if self.flag != 0:
            if self.resize_flag == 0:
                # 获取长宽最大缩放比例
                ratio1 = self.maxHeight / self.map_width
                ratio2 = self.maxWidth / self.map_height

                if ratio1 >= ratio2:
                    self.ratio = ratio2
                else:
                    self.ratio = ratio1
                
                labelWidth = int(self.map_width * self.ratio)
                labelHeight = int(self.map_height * self.ratio)

                self.map_label.resize(labelWidth, labelHeight)

                if ratio1 > ratio2:
                    self.ty += int((self.maxHeight - labelHeight) / 2)
                else:
                    self.tx += int((self.maxWidth - labelWidth) / 2)
                self.map_label.move(self.tx, self.ty)

                self.resize_flag = 1

    def onTreeClicked(self, qmodelindex):
        item = self.config_show_treeWidget.currentItem()
        print("key=%s ,value=%s" % (item.text(0), item.text(1)))

    def addOneConfigItem(self, data):
        # root_dict = {"Duration": 0, "SpecificDuration": 1, "Events": 2}
        # if data != None:
        #     for key1 in data.keys():
        #         item = self.config_show_treeWidget.topLevelItem(root_dict[key1])
        #         node = QTreeWidgetItem(item)
        #         for key2 in data[key1].keys():
        #             node.setText(0, str(key2))
        #             node.setText(1, str(data[key1][key2][0]))
        if data != None:
            for strategy_type in data.keys():
                if strategy_type not in self.configs:
                    self.configs[strategy_type] = data[strategy_type]
                else:
                    for strategy_name in data[strategy_type].keys():
                        self.configs[strategy_type][strategy_name] = data[strategy_type][strategy_name]
            print(self.configs)
 
 
    def updateTreeNodeBtn(self):
        print('--- updateTreeNodeBtn ---')
        item = self.config_show_treeWidget.currentItem()
        item.setText(0,'updateNode')
        item.setText(1,'20')		
 
 
    def deleteOneConfigItem(self):
        item = self.config_show_treeWidget.currentItem()
        if item != self.config_show_treeWidget.topLevelItem():
            root = self.config_show_treeWidget.invisibleRootItem()
            for item in self.config_show_treeWidget.selectedItems():
                (item.parent() or root).removeChild(item)

def main():
    app = QApplication(sys.argv)
    ros_qt_node = ROSQtNode()
    ros_qt_node.show()

    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
