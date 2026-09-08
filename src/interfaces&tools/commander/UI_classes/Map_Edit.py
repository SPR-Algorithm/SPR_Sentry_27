from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui, QtWidgets
import UI_classes.Map_Edit_class
import cv2
import yaml
 
class MapEdit(QDialog, QWidget, UI_classes.Map_Edit_class.Ui_Dialog):
    def __init__(self, path):
        super(MapEdit, self).__init__(None)

        self.setupUi(self)

        self.resize_flag = 0
        # 画面标签位置
        self.tx = 210
        self.ty = 60
        # 画面标签大小
        self.maxWidth = 550
        self.maxHeight = 550
        self.ratio = 1

        self.map_yaml = {}
        self.map_path = path
        self.map_path_config = self.map_path.split(".")[0] + '.yaml'
        self.img_map = cv2.imread(self.map_path)
        self.show_cv_img()
        self.load_config(self.map_path_config)

        self.edit_mode = 'view'
        self.obstacle_pushButton.clicked.connect(self.obstacle_mode)
        self.free_pushButton.clicked.connect(self.free_mode)
        self.unexplored_pushButton.clicked.connect(self.unexplored_mode)
        self.view_pushButton.clicked.connect(self.view_mode)
        self.save_pushButton.clicked.connect(self.save_map)

    def load_config(self, path):
        with open(path, 'r', encoding='utf-8') as f:
            data = f.read()
            self.map_yaml = yaml.load(data, Loader=yaml.FullLoader)
            # print(self.map_yaml)

    def mouseMoveEvent(self, event):
        x = event.x()
        y = event.y()
        x -= self.tx
        y -= self.ty
        x = int(x / self.ratio)
        y = int(y / self.ratio)
        if x >= 0 and x <= self.map_width and y >= 0 and y <= self.map_height:
            if self.edit_mode == 'obstacle':
                self.img_map[y, x] = (0, 0, 0)
            if self.edit_mode == 'free':
                self.img_map[y, x] = (254, 254, 254)
            if self.edit_mode == 'unexplored':
                self.img_map[y, x] = (205, 205, 205)
            self.show_cv_img()
            # print(x, y)
                
        self.update()

    def show_cv_img(self):
        
        self.map_width = self.img_map.shape[1]
        self.map_height = self.img_map.shape[0]
        self.Resize()
        QtImg = QtGui.QImage(self.img_map.data,
                             self.img_map.shape[1],
                             self.img_map.shape[0],
                             self.img_map.shape[1] * 3,
                             QtGui.QImage.Format_RGB888)
        jpg_out = QtGui.QPixmap(QtImg).scaled(
            self.label.width(), self.label.height())
        
        self.label.setPixmap(jpg_out)

    def Resize(self):
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

            self.label.resize(labelWidth, labelHeight)

            if ratio1 > ratio2:
                self.ty += int((self.maxHeight - labelHeight) / 2)
            else:
                self.tx += int((self.maxWidth - labelWidth) / 2)
            self.label.move(self.tx, self.ty)

            self.resize_flag = 1

    def mouseDoubleClickEvent(self, event):
        if event.button() == Qt.LeftButton:
            # QMessageBox.information(self, "Mouse Double Click", "Left button double clicked!")
            pass
            
        elif event.button() == Qt.RightButton:
            QMessageBox.information(self, "Mouse Double Click", "Right button double clicked!")

        # 调用父类的事件处理函数
        super().mouseDoubleClickEvent(event)

    def obstacle_mode(self):
        self.edit_mode = 'obstacle'

    def free_mode(self):
        self.edit_mode = 'free'

    def unexplored_mode(self):
        self.edit_mode = 'unexplored'

    def view_mode(self):
        self.edit_mode = 'view'

    def save_map(self):
        gray_image = cv2.cvtColor(self.img_map, cv2.COLOR_BGR2GRAY)
        cv2.imwrite("/home/spr/map_e.pgm", gray_image)
        self.map_yaml['image'] = 'map_e.pgm'
        with open('/home/spr/map_e.yaml', 'w') as file:
            yaml.dump(self.map_yaml, file, default_flow_style=False)