from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import UI_classes.Config_Dialog_class
 
class ConfigDialog(QDialog, QWidget, UI_classes.Config_Dialog_class.Ui_Dialog):
    def __init__(self, parent=None):
        super(ConfigDialog, self).__init__(parent)

        self.setupUi(self)

        self.x_map = 0.0
        self.y_map = 0.0
        self.orientation = 0.0
        self.strategy_name = None
        self.cruise_frequency = 1

        self.one_config_strategy = {}
        self.one_target_dict = {}
        self.one_config_pose = []
        self.config_mode = []

        self.strategy_type_list = ["Duration", "SpecificDuration", "Events"]
        self.strategy_type = self.strategy_type_list[0]
        self.select_strategy_comboBox.addItems(self.strategy_type_list)
        self.select_strategy_comboBox.currentIndexChanged.connect(self.select_strategy_change)

        self.move_mode_list = ["singel-pose", "circle_cruise", "return_cruise"]
        self.move_mode = self.move_mode_list[0]
        self.move_mode_comboBox.addItems(self.move_mode_list)
        self.move_mode_comboBox.currentIndexChanged.connect(self.move_mode_change)

        self.pose_x_lineEdit.setValidator(QDoubleValidator())
        self.pose_y_lineEdit.setValidator(QDoubleValidator())
        self.orientation_lineEdit.setValidator(QDoubleValidator())
        self.cruise_frequency_lineEdit.setValidator(QIntValidator())

        self.pose_x_lineEdit.textChanged.connect(self.pose_x_changed)
        self.pose_y_lineEdit.textChanged.connect(self.pose_y_changed)
        self.orientation_lineEdit.textChanged.connect(self.orientation_changed)
        self.strategy_name_lineEdit.textChanged.connect(self.strategy_name_changed)
        self.cruise_frequency_lineEdit.textChanged.connect(self.cruise_frequency_changed)

        self.model=QStandardItemModel(0,3)
        self.model.setHorizontalHeaderLabels(['x','y','orientation'])

        self.poses_tableView.setModel(self.model)

        self.orientation_lineEdit.setText('0.0')
        self.cruise_frequency_lineEdit.setText('1')

        self.buttonBox.accepted.connect(self.accept) # type: ignore
        self.buttonBox.rejected.connect(self.reject) # type: ignore
        self.create_pushButton.clicked.connect(self.add_row)
        self.delete_pushButton.clicked.connect(self.remove_row)


    def get_real_pose(self, x, y):
        self.x_map = x
        self.y_map = y
        self.pose_x_lineEdit.setText(str(self.x_map))
        self.pose_y_lineEdit.setText(str(self.y_map))

    def select_strategy_change(self,i):
        self.strategy_type = self.select_strategy_comboBox.currentText()
        print(f'strategy type:{self.strategy_type}')

    def move_mode_change(self,i):
        self.move_mode = self.move_mode_comboBox.currentText()
        print(f'move mode:{self.move_mode}')

    def pose_x_changed(self, text):
        self.x_map = float(text)
        print(f'x:{self.x_map}')

    def pose_y_changed(self, text):
        self.y_map = float(text)
        print(f'y:{self.y_map}')

    def orientation_changed(self, text):
        self.orientation = float(text)
        print(f'orientation:{self.orientation}')

    def strategy_name_changed(self, text):
        self.strategy_name = text
        if self.strategy_type == 'Duration':
            self.strategy_name = int(self.strategy_name)
        print(f'strategy name:{self.strategy_name}')

    def cruise_frequency_changed(self, text):
        self.cruise_frequency = int(text)
        print(f'cruise frequency:{self.cruise_frequency}')

    def add_row(self):
        # 在模型中动态添加一行数据
        new_row_data = [self.x_map, self.y_map, self.orientation]
        new_row_items = [QStandardItem(str(value)) for value in new_row_data]
        self.model.appendRow(new_row_items)

    def remove_row(self):
        # 获取选中行的索引
        selected_index = self.poses_tableView.selectionModel().currentIndex()
        # print(selected_index.row())

        # 如果有选中的行，则删除选中行
        if selected_index.isValid():
            self.model.removeRow(selected_index.row())

    def closeEvent(self, event):
        # 在对话框关闭时，发送对话框的退出状态给主窗口
        if self.result() == self.Accepted:
            self.parent().handle_dialog_exit('Accepted')
        else:
            self.parent().handle_dialog_exit('Rejected')

    # def getConfig(self):
    #     if self.strategy_name != None:
    #         if self.model.rowCount() <= 1:
    #             self.one_config_pose = [self.x_map, self.y_map, self.orientation]
    #         elif self.model.rowCount() > 1:
    #             for row in range(self.model.rowCount()):
    #                 temp_pose = []
    #                 for col in range(self.model.columnCount()):
    #                     index = self.model.index(row, col)
    #                     data = self.model.data(index)
    #                     temp_pose.append(float(data))
    #                 self.one_config_pose.append(temp_pose)
    #         self.config_mode.append(self.move_mode)
    #         self.config_mode.append(int(self.cruise_frequency))
    #         move_list = [self.one_config_pose, self.config_mode]
    #         self.one_target_dict[self.strategy_name] = move_list
    #         self.one_config_strategy[self.strategy_type] = self.one_target_dict
    #         return self.one_config_strategy
    #     else:
    #         return None
        
    def getConfig(self):
        if self.strategy_name != None:
            if self.model.rowCount() <= 1:
                self.one_config_pose = [self.x_map, self.y_map, self.orientation]
            elif self.model.rowCount() > 1:
                for row in range(self.model.rowCount()):
                    temp_pose = []
                    for col in range(self.model.columnCount()):
                        index = self.model.index(row, col)
                        data = self.model.data(index)
                        temp_pose.append(float(data))
                    self.one_config_pose.append(temp_pose)

            move_dict = {}

            move_dict['pose'] = self.one_config_pose
            self.config_mode.append(self.move_mode)
            self.config_mode.append(int(self.cruise_frequency))
            move_dict['mode'] = self.config_mode
            move_dict['level'] = [1, 'r']
            self.one_target_dict[self.strategy_name] = move_dict
            self.one_config_strategy[self.strategy_type] = self.one_target_dict
            return self.one_config_strategy
        else:
            return None